/**
 * @file custom_audio_driver.c
 * @brief Simplified ALSA USB audio playback driver.
 *
 * This file implements a simplified version of the ALSA USB PCM playback driver.
 * It contains functions for endpoint management, hardware parameter configuration,
 * runtime hardware information setup, URB (USB Request Block) preparation for playback,
 * and power state changes.
 *
 * The driver is based on the standard ALSA pcm.c component for USB audio and has been
 * modified to reduce error/debug handling while remaining functional.
 *
 * @author Satheesh
 * @version 1.3
 * 
 */

 #include <linux/init.h>
 #include <linux/slab.h>
 #include <linux/bitrev.h>
 #include <linux/ratelimit.h>
 #include <linux/usb.h>
 #include <linux/usb/audio.h>
 #include <linux/usb/audio-v2.h>
 
 #include <sound/core.h>
 #include <sound/pcm.h>
 #include <sound/pcm_params.h>
 
 #include "usbaudio.h"
 #include "card.h"
 #include "quirks.h"
 #include "endpoint.h"
 #include "helper.h"
 #include "pcm.h"
 #include "clock.h"
 #include "power.h"
 #include "media.h"
 #include "implicit.h"
 
 /* Flags for substream endpoints */
 #define SUBSTREAM_FLAG_DATA_EP_STARTED  0
 #define SUBSTREAM_FLAG_SYNC_EP_STARTED  1
 
 /**
  * @brief Calculate the estimated delay in frames for the USB PCM stream.
  *
  * This function calculates the delay (in frames) based on the difference in USB
  * frame counters and the number of bytes in-flight.
  *
  * @param subs Pointer to the USB substream structure.
  * @param runtime Pointer to the PCM runtime structure.
  * @return Estimated delay in frames.
  */
 static snd_pcm_uframes_t snd_usb_pcm_delay(struct snd_usb_substream *subs,
                        struct snd_pcm_runtime *runtime)
 {
     unsigned int current_frame_number, frame_diff;
     int est_delay, queued;
    /*subs->inflight_bytes number of audio bytes that have been sent but not yet played.*/
     queued = bytes_to_frames(runtime, subs->inflight_bytes);
     if (!queued)
         return 0;
 
     current_frame_number = usb_get_current_frame_number(subs->dev);
     frame_diff = (current_frame_number - subs->last_frame_number) & 0xff;//wraparound at 255.
     est_delay = frame_diff * runtime->rate / 1000;//Multiplies frame_diff (in ms) by rate and divides by 1000 to get frames played
     est_delay = queued - est_delay;
     if (est_delay < 0)
         est_delay = 0;
 
     return est_delay;
 }
 
 /**
  * @brief Return the current PCM pointer based on the internal hardware pointer.
  *
  * This function returns the current pointer position into the DMA ring buffer,
  * after updating the runtime delay based on USB frame counters.
  *
  * @param substream Pointer to the PCM substream.
  * @return The current hardware pointer (in frames) or SNDRV_PCM_POS_XRUN on error.
  */
 static snd_pcm_uframes_t snd_usb_pcm_pointer(struct snd_pcm_substream *substream)
 {
     struct snd_pcm_runtime *runtime = substream->runtime;//ALSA runtime data (buffer size, sample format, current delay).
     struct snd_usb_substream *subs = runtime->private_data;//USB-specific substream info, previously stored in private_data
     unsigned int hwptr_done;

 //	Check if the USB audio device is shutting down
     if (atomic_read(&subs->stream->chip->shutdown))
         return SNDRV_PCM_POS_XRUN;
     spin_lock(&subs->lock);//protect access to shared data
     hwptr_done = subs->hwptr_done;//the number of bytes of audio that have been "done" — already sent to the USB endpoint and accepted
     runtime->delay = snd_usb_pcm_delay(subs, runtime);//calculate the delay in frames
     spin_unlock(&subs->lock);
     return bytes_to_frames(runtime, hwptr_done);
 }
 
 /**
  * @brief Find a matching audio format from the list.
  *
  * This function iterates over a list of supported audio formats and returns the one
  * that best matches the provided criteria (PCM format, sample rate, channels). It
  * performs further filtering based on a strict match requirement.
  *
  * @param fmt_list_head Pointer to the head of the audioformat list.
  * @param format The desired PCM format.
  * @param rate The desired sample rate.
  * @param channels The desired number of channels.
  * @param strict_match If true, the match is forced to be exact.
  * @param subs Pointer to the USB substream.
  * @return Pointer to the matching audioformat structure, or NULL if none found.
  */
 static const struct audioformat *
 find_format(struct list_head *fmt_list_head, snd_pcm_format_t format,
         unsigned int rate, unsigned int channels, bool strict_match,
         struct snd_usb_substream *subs)
 {
     const struct audioformat *fp;
     const struct audioformat *found = NULL;
     int cur_attr = 0, attr;
 
     list_for_each_entry(fp, fmt_list_head, list) {
         if (strict_match) {
             if (!(fp->formats & pcm_format_to_bits(format)))
                 continue;
             if (fp->channels != channels)
                 continue;
         }
         if (rate < fp->rate_min || rate > fp->rate_max)
             continue;
         if (!(fp->rates & SNDRV_PCM_RATE_CONTINUOUS)) {
             unsigned int i;
             for (i = 0; i < fp->nr_rates; i++)
                 if (fp->rate_table[i] == rate)
                     break;
             if (i >= fp->nr_rates)
                 continue;
         }
         attr = fp->ep_attr & USB_ENDPOINT_SYNCTYPE;
         if (!found) {
             found = fp;
             cur_attr = attr;
             continue;
         }
         if (subs && attr != cur_attr) {
             if ((attr == USB_ENDPOINT_SYNC_ASYNC &&
                  subs->direction == SNDRV_PCM_STREAM_PLAYBACK))
                 continue;
             if ((cur_attr == USB_ENDPOINT_SYNC_ASYNC &&
                  subs->direction == SNDRV_PCM_STREAM_PLAYBACK)) {
                 found = fp;
                 cur_attr = attr;
                 continue;
             }
         }
         if (fp->maxpacksize > found->maxpacksize) {
             found = fp;
             cur_attr = attr;
         }
     }
     return found;
 }
 
 /**
  * @brief Find the audio format for the substream that matches hw_params.
  *
  * This is a wrapper around find_format() which uses the hardware parameters specified
  * in @a params.
  *
  * @param subs Pointer to the USB substream.
  * @param params Pointer to the PCM hardware parameters.
  * @return Pointer to the matching audioformat or NULL if none is found.
  */
 static const struct audioformat *
 find_substream_format(struct snd_usb_substream *subs,
               const struct snd_pcm_hw_params *params)
 {
     return find_format(&subs->fmt_list, params_format(params),
                params_rate(params), params_channels(params),
                true, subs);
 }
 
 /**
  * @brief Check if the USB substream has a fixed sample rate.
  *
  * This function checks whether a USB audio substream supports only one fixed sample rate across all its formats.
  * It returns true if the substream has a fixed sample rate, otherwise false.
  * @param subs Pointer to the USB substream.
  * @return true if the substream has a fixed sample rate, false otherwise.
  */
 bool snd_usb_pcm_has_fixed_rate(struct snd_usb_substream *subs)
 {
     const struct audioformat *fp;
     struct snd_usb_audio *chip;
     int rate = -1;
 
     if (!subs)
         return false;
     chip = subs->stream->chip;//Get the main USB audio device structure from the substream

     /*quirk_flags is a bitfield used by ALSA USB audio driver to track special-case hardware quirks.
     chip->quirk_flags = 0b00000001    // only fixed-rate flag is set
    QUIRK_FLAG_FIXED_RATE = 0b00000001

    Result:
    0b00000001 & 0b00000001 = 0b00000001 (non-zero)*/
     if (!(chip->quirk_flags & QUIRK_FLAG_FIXED_RATE))
         return false;
    /*For each audioformat node (fp) inside the linked list subs->fmt_list, 
    where the list is linked using the list member inside struct audioformat*/
     list_for_each_entry(fp, &subs->fmt_list, list) {
        //* Check if the format supports continuous rates or has no rates defined.
         if (fp->rates & SNDRV_PCM_RATE_CONTINUOUS)
             return false;
         if (fp->nr_rates < 1)
             continue;
         if (fp->nr_rates > 1)
             return false;
         if (rate < 0) {
             rate = fp->rate_table[0];
             continue;
         }
         if (rate != fp->rate_table[0])
             return false;
     }
     return true;
 }
 
 /**
  * @brief Initialize pitch control for UAC version 1.
  *
  * Sends a control message to enable pitch control.
  *
  * @param chip Pointer to the USB audio chip structure.
  * @param ep The endpoint address.
  * @return 0 on success, negative error code on failure.
  */
 static int init_pitch_v1(struct snd_usb_audio *chip, int ep)
 {
     struct usb_device *dev = chip->dev;
     unsigned char data[1] = {1};
     return snd_usb_ctl_msg(dev, usb_sndctrlpipe(dev, 0), UAC_SET_CUR,
                    USB_TYPE_CLASS | USB_RECIP_ENDPOINT | USB_DIR_OUT,
                    UAC_EP_CS_ATTR_PITCH_CONTROL << 8, ep,
                    data, sizeof(data));
    /*Device	dev	                            USB device handle
    Pipe	    usb_sndctrlpipe(dev, 0)	        Control OUT endpoint
    Request	    UAC2_CS_CUR (0x01)	            Set current value
    RequestType	0x22	                        Class, Endpoint, Host-to-Device
    wValue	    0x0100	                        PITCH Control Selector
    wIndex	    0	                            Endpoint or interface number
    Data	    0x01	                        Enable pitch control
    Return	    0 or < 0	                    Success or failure*/
 }
 
 /**
  * @brief Initialize pitch control for UAC version 2.
  *
  * Sends a control message to enable pitch control.
  *
  * @param chip Pointer to the USB audio chip structure.
  * @param ep The endpoint address.
  * @return 0 on success, negative error code on failure.
  */
 static int init_pitch_v2(struct snd_usb_audio *chip, int ep)
 {
     struct usb_device *dev = chip->dev;
     unsigned char data[1] = {1};
     return snd_usb_ctl_msg(dev, usb_sndctrlpipe(dev, 0), UAC2_CS_CUR,
                    USB_TYPE_CLASS | USB_RECIP_ENDPOINT | USB_DIR_OUT,
                    UAC2_EP_CS_PITCH << 8, 0,
                    data, sizeof(data));
    

 }
 
 /**
  * @brief Initialize pitch control for a given audio format.
  *
  * Checks if the current format supports pitch control, and if so, initializes
  * pitch control using the appropriate UAC version (v1 or v2).
  *
  * @param chip Pointer to the USB audio chip structure.
  * @param fmt Pointer to the audio format structure.
  * @return 0 on success, negative error code on failure.
  */
 int snd_usb_init_pitch(struct snd_usb_audio *chip,
                const struct audioformat *fmt)
 {
     int err;
    //bitmask of End point descriptor
     if (!(fmt->attributes & UAC_EP_CS_ATTR_PITCH_CONTROL))
         return 0;
    /*UAC_VERSION_1 = USB Audio Class 1.0
    UAC_VERSION_2 = USB Audio Class 2.0*/
     switch (fmt->protocol) {
     case UAC_VERSION_1:
         err = init_pitch_v1(chip, fmt->endpoint);
         break;
     case UAC_VERSION_2:
         err = init_pitch_v2(chip, fmt->endpoint);
         break;
     default:
         return 0;
     }
 
     if (err < 0)
         return err;
 
     return 0;
 }
 
 /**
  * @brief Stop USB endpoints for the given substream.
  *
  * This function stops both data and sync endpoints if they are running.
  *
  * @param subs Pointer to the USB substream.
  * @param keep_pending If true, keep any pending transfers.
  * @return true if at least one endpoint was stopped, false otherwise.
  */
 static bool stop_endpoints(struct snd_usb_substream *subs, bool keep_pending)
 {
     bool stopped = false;
     if (test_and_clear_bit(SUBSTREAM_FLAG_SYNC_EP_STARTED, &subs->flags)) {
         snd_usb_endpoint_stop(subs->sync_endpoint, keep_pending);
         stopped = true;
     }
     if (test_and_clear_bit(SUBSTREAM_FLAG_DATA_EP_STARTED, &subs->flags)) {
         snd_usb_endpoint_stop(subs->data_endpoint, keep_pending);
         stopped = true;
     }
     return stopped;
 }
 
 /**
  * @brief Start USB endpoints for the given substream.
  *
  * This function starts the data endpoint and, if present, the sync endpoint.
  *
  * @param subs Pointer to the USB substream.
  * @return 0 on success or a negative error code on failure.
  */
 static int start_endpoints(struct snd_usb_substream *subs)
 {
     int err;
    //Checks if the data endpoint was initialized
     if (!subs->data_endpoint)
         return -EINVAL;
    
    //Checks if the data endpoint is already started
     if (!test_and_set_bit(SUBSTREAM_FLAG_DATA_EP_STARTED, &subs->flags)) {
         err = snd_usb_endpoint_start(subs->data_endpoint);
         if (err < 0) {
             clear_bit(SUBSTREAM_FLAG_DATA_EP_STARTED, &subs->flags);
             goto error;
         }
     }
 
     if (subs->sync_endpoint &&
         !test_and_set_bit(SUBSTREAM_FLAG_SYNC_EP_STARTED, &subs->flags)) {
         err = snd_usb_endpoint_start(subs->sync_endpoint);
         if (err < 0) {
             clear_bit(SUBSTREAM_FLAG_SYNC_EP_STARTED, &subs->flags);
             goto error;
         }
     }
     return 0;
 error:
     stop_endpoints(subs, false);
     return err;
 }
 
 /**
  * @brief Synchronize any pending stops on endpoints.
  *
  * This forces the sync of any pending stop operations on both sync and data endpoints.
  *
  * @param subs Pointer to the USB substream.
  */
 static void sync_pending_stops(struct snd_usb_substream *subs)
 {
     snd_usb_endpoint_sync_pending_stop(subs->sync_endpoint);
     snd_usb_endpoint_sync_pending_stop(subs->data_endpoint);
 }
 
 /**
  * @brief PCM sync_stop callback.
  *
  * This callback synchronizes pending stops when a PCM stream is stopping.
  *
  * @param substream Pointer to the PCM substream.
  * @return 0 on success.
  */
 static int snd_usb_pcm_sync_stop(struct snd_pcm_substream *substream)
 {
     struct snd_usb_substream *subs = substream->runtime->private_data;
     sync_pending_stops(subs);
     return 0;
 }
 
 /**
  * @brief Set up a sync endpoint for implicit feedback if supported.
  *
  * This function attempts to associate a sync endpoint (for feedback) with the
  * given audio format based on the USB interface descriptors.
  *
  * @param chip Pointer to the USB audio chip structure.
  * @param fmt Pointer to the audio format to check.
  * @return 0 on success, or a negative error code if the setup fails.
  */
 int snd_usb_audioformat_set_sync_ep(struct snd_usb_audio *chip,
                     struct audioformat *fmt)
 {
     struct usb_host_interface *alts = snd_usb_get_host_interface(chip, fmt->iface, fmt->altsetting);
     struct usb_interface_descriptor *altsd;
     unsigned int ep, attr, sync_attr;
     bool is_playback;
     int err;
 
     if (!alts)
         return 0;
     altsd = get_iface_desc(alts);
 
     err = snd_usb_parse_implicit_fb_quirk(chip, fmt, alts);
     if (err > 0)
         return 0;
 
     if (fmt->ep_idx > 0 || altsd->bNumEndpoints < 2)
         return 0;
 
     is_playback = !(get_endpoint(alts, 0)->bEndpointAddress & USB_DIR_IN);
     attr = fmt->ep_attr & USB_ENDPOINT_SYNCTYPE;
     if ((is_playback && (attr == USB_ENDPOINT_SYNC_SYNC ||
                  attr == USB_ENDPOINT_SYNC_ADAPTIVE)) ||
         (!is_playback && attr != USB_ENDPOINT_SYNC_ADAPTIVE))
         return 0;
 
     sync_attr = get_endpoint(alts, 1)->bmAttributes;
     if ((sync_attr & USB_ENDPOINT_XFERTYPE_MASK) != USB_ENDPOINT_XFER_ISOC ||
         (get_endpoint(alts, 1)->bLength >= USB_DT_ENDPOINT_AUDIO_SIZE &&
          get_endpoint(alts, 1)->bSynchAddress != 0))
         return -EINVAL;
     ep = get_endpoint(alts, 1)->bEndpointAddress;
     if (get_endpoint(alts, 0)->bLength >= USB_DT_ENDPOINT_AUDIO_SIZE &&
         get_endpoint(alts, 0)->bSynchAddress != 0 &&
         ((is_playback && ep != (unsigned int)(get_endpoint(alts, 0)->bSynchAddress | USB_DIR_IN)) ||
          (!is_playback && ep != (unsigned int)(get_endpoint(alts, 0)->bSynchAddress & ~USB_DIR_IN))))
         return -EINVAL;
 
     fmt->sync_ep = ep;
     fmt->sync_iface = altsd->bInterfaceNumber;
     fmt->sync_altsetting = altsd->bAlternateSetting;
     fmt->sync_ep_idx = 1;
     if ((sync_attr & USB_ENDPOINT_USAGE_MASK) == USB_ENDPOINT_USAGE_IMPLICIT_FB)
         fmt->implicit_fb = 1;
     return 0;
 }
 
 /**
  * @brief Change the power domain state for a USB substream.
  *
  * This function changes the power state of the substream's associated power domain.
  *
  * @param subs Pointer to the USB substream.
  * @param state The desired power state.
  * @return 0 on success or a negative error code on failure.
  */
 static int snd_usb_pcm_change_state(struct snd_usb_substream *subs, int state)
 {
     if (!subs->str_pd)
         return 0;
     return snd_usb_power_domain_set(subs->stream->chip, subs->str_pd, state);
 }
 
 /**
  * @brief Suspend the USB PCM stream.
  *
  * This function moves both the playback and capture substreams to a lower power state.
  *
  * @param as Pointer to the USB audio stream structure.
  * @return 0 on success or a negative error code on failure.
  */
 int snd_usb_pcm_suspend(struct snd_usb_stream *as)
 {
     int ret;
     ret = snd_usb_pcm_change_state(&as->substream[0], UAC3_PD_STATE_D2);
     if (ret < 0)
         return ret;
     ret = snd_usb_pcm_change_state(&as->substream[1], UAC3_PD_STATE_D2);
     return ret;
 }
 
 /**
  * @brief Resume the USB PCM stream.
  *
  * This function moves both the playback and capture substreams back to an active power state.
  *
  * @param as Pointer to the USB audio stream structure.
  * @return 0 on success or a negative error code on failure.
  */
 int snd_usb_pcm_resume(struct snd_usb_stream *as)
 {
     int ret;
     ret = snd_usb_pcm_change_state(&as->substream[0], UAC3_PD_STATE_D1);
     if (ret < 0)
         return ret;
     ret = snd_usb_pcm_change_state(&as->substream[1], UAC3_PD_STATE_D1);
     return ret;
 }
 
 /**
  * @brief Close the USB endpoints for a substream.
  *
  * This function disassociates and closes both the data and sync endpoints.
  *
  * @param chip Pointer to the USB audio chip structure.
  * @param subs Pointer to the USB substream.
  */
 static void close_endpoints(struct snd_usb_audio *chip,
                 struct snd_usb_substream *subs)
 {
     if (subs->data_endpoint) {
         snd_usb_endpoint_set_sync(chip, subs->data_endpoint, NULL);
         snd_usb_endpoint_close(chip, subs->data_endpoint);
         subs->data_endpoint = NULL;
     }
     if (subs->sync_endpoint) {
         snd_usb_endpoint_close(chip, subs->sync_endpoint);
         subs->sync_endpoint = NULL;
     }
 }
 
 /**
  * @brief PCM hw_params callback.
  *
  * This function allocates a buffer, selects the correct audio format,
  * and opens the relevant USB endpoints for the substream.
  *
  * @param substream Pointer to the PCM substream.
  * @param hw_params Pointer to the hardware parameters.
  * @return 0 on success or a negative error code on failure.
  */
 static int snd_usb_hw_params(struct snd_pcm_substream *substream,
                  struct snd_pcm_hw_params *hw_params)
 {
     struct snd_usb_substream *subs = substream->runtime->private_data;
     struct snd_usb_audio *chip = subs->stream->chip;
     const struct audioformat *fmt;
     const struct audioformat *sync_fmt;
     bool fixed_rate, sync_fixed_rate;
     int ret;
 
     ret = snd_media_start_pipeline(subs);
     if (ret)
         return ret;
 
     fixed_rate = snd_usb_pcm_has_fixed_rate(subs);
     fmt = find_substream_format(subs, hw_params);
     if (!fmt) {
         ret = -EINVAL;
         goto stop_pipeline;
     }
 
  
    else {
         sync_fmt = fmt;
         sync_fixed_rate = fixed_rate;
     }
 
     ret = snd_usb_lock_shutdown(chip);
     if (ret < 0)
         goto stop_pipeline;
 
     ret = snd_usb_pcm_change_state(subs, UAC3_PD_STATE_D0);
     if (ret < 0)
         goto unlock;
 
    /*If an endpoint is already open:
    Check if it's compatible with the new hw_params*/
     if (subs->data_endpoint) {
         if (snd_usb_endpoint_compatible(chip, subs->data_endpoint, fmt, hw_params))
             goto unlock;
         if (stop_endpoints(subs, false))
             sync_pending_stops(subs);
         close_endpoints(chip, subs);
     }
 
     subs->data_endpoint = snd_usb_endpoint_open(chip, fmt, hw_params, false, fixed_rate);
     if (!subs->data_endpoint) {
         ret = -EINVAL;
         goto unlock;
     }
 
     if (fmt->sync_ep) {
         subs->sync_endpoint = snd_usb_endpoint_open(chip, sync_fmt, hw_params,
                                 fmt == sync_fmt, sync_fixed_rate);
         if (!subs->sync_endpoint) {
             ret = -EINVAL;
             goto unlock;
         }
         snd_usb_endpoint_set_sync(chip, subs->data_endpoint, subs->sync_endpoint);
     }
 
     mutex_lock(&chip->mutex);
     subs->cur_audiofmt = fmt;
     mutex_unlock(&chip->mutex);
 
     if (!subs->data_endpoint->need_setup)
         goto unlock;
 
     if (subs->sync_endpoint) {
         ret = snd_usb_endpoint_set_params(chip, subs->sync_endpoint);
         if (ret < 0)
             goto unlock;
     }
 
     ret = snd_usb_endpoint_set_params(chip, subs->data_endpoint);
 unlock:
     if (ret < 0)
         close_endpoints(chip, subs);
     snd_usb_unlock_shutdown(chip);
 stop_pipeline:
     if (ret < 0)
         snd_media_stop_pipeline(subs);
     return ret;
 }
 
 /**
  * @brief PCM hw_free callback.
  *
  * This function resets the current audio format and releases the allocated buffer.
  *
  * @param substream Pointer to the PCM substream.
  * @return 0 on success.
  */
 static int snd_usb_hw_free(struct snd_pcm_substream *substream)
 {
     struct snd_usb_substream *subs = substream->runtime->private_data;
     struct snd_usb_audio *chip = subs->stream->chip;
 
     snd_media_stop_pipeline(subs);
     mutex_lock(&chip->mutex);
     subs->cur_audiofmt = NULL;
     mutex_unlock(&chip->mutex);
     if (!snd_usb_lock_shutdown(chip)) {
         if (stop_endpoints(subs, false))
             sync_pending_stops(subs);
         close_endpoints(chip, subs);
         snd_usb_unlock_shutdown(chip);
     }
     return 0;
 }
 
 /**
  * @brief PCM prepare callback.
  *
  * This function sets up the USB endpoints and resets internal counters before starting the stream.
  *
  * @param substream Pointer to the PCM substream.
  * @return 0 on success, or a negative error code on failure.
  */
 static int snd_usb_pcm_prepare(struct snd_pcm_substream *substream)
 {
     pr_info("PCM prepare: Rate=%d, Format=%d, Channels=%d\n",
         substream->runtime->rate,
         substream->runtime->format,
         substream->runtime->channels);
     struct snd_pcm_runtime *runtime = substream->runtime;
     struct snd_usb_substream *subs = runtime->private_data;
     struct snd_usb_audio *chip = subs->stream->chip;
     int ret;
 
     ret = snd_usb_lock_shutdown(chip);
     if (ret < 0)
         return ret;
     if (snd_BUG_ON(!subs->data_endpoint)) {
         ret = -EIO;
         goto unlock;
     }
 
     ret = snd_usb_pcm_change_state(subs, UAC3_PD_STATE_D0);
     if (ret < 0)
         goto unlock;
 
     if (subs->sync_endpoint) {
         ret = snd_usb_endpoint_prepare(chip, subs->sync_endpoint);
         if (ret < 0)
             goto unlock;
     }
 
     ret = snd_usb_endpoint_prepare(chip, subs->data_endpoint);
     if (ret < 0)
         goto unlock;
     else if (ret > 0)
         snd_usb_set_format_quirk(subs, subs->cur_audiofmt);
 
     /* Reset internal counters */
     subs->buffer_bytes = frames_to_bytes(runtime, runtime->buffer_size);
     subs->inflight_bytes = 0;
     subs->hwptr_done = 0;
     subs->transfer_done = 0;
     subs->last_frame_number = 0;
     subs->period_elapsed_pending = 0;
     runtime->delay = 0;
 
     ret = start_endpoints(subs);
     if (ret < 0)
         goto unlock;
 
 unlock:
     snd_usb_unlock_shutdown(chip);
     return ret;
 }
 
 /**
  * @brief Hardware parameters describing USB audio capabilities.
  *
  * This constant structure defines the default hardware constraints for
  * the USB audio playback PCM, such as supported channels, buffer sizes, and periods.
  */
 static const struct snd_pcm_hardware snd_usb_hardware = {
     .info =			SNDRV_PCM_INFO_MMAP |
                 SNDRV_PCM_INFO_MMAP_VALID |
                 SNDRV_PCM_INFO_BATCH |
                 SNDRV_PCM_INFO_INTERLEAVED |
                 SNDRV_PCM_INFO_BLOCK_TRANSFER |
                 SNDRV_PCM_INFO_PAUSE,
     .channels_min =		1,
     .channels_max =		256,
     .buffer_bytes_max =	INT_MAX,
     .period_bytes_min =	64,
     .period_bytes_max =	INT_MAX,
     .periods_min =		2,
     .periods_max =		1024,
 };
 
 /**
  * @brief Set up the runtime hardware information.
  *
  * This function initializes the hw field of the runtime structure with supported
  * formats, rate range, channels range, period sizes, and other constraints by
  * iterating over the supported audio formats.
  *
  * @param runtime Pointer to the PCM runtime structure.
  * @param subs Pointer to the USB substream.
  * @return 0 on success, or a negative error code on failure.
  */
 static int setup_hw_info(struct snd_pcm_runtime *runtime, struct snd_usb_substream *subs)
 {
     const struct audioformat *fp;
    
    /*rate_min = 0x7fffffff;  // INT_MAX = 2147483647

    if (rate_min > 44100)  → Yes → set rate_min = 44100
    if (rate_min > 48000)  → No*/
     runtime->hw.formats = subs->formats;
     runtime->hw.rate_min = 0x7fffffff;
     runtime->hw.rate_max = 0;
     runtime->hw.channels_min = 256;
     runtime->hw.channels_max = 0;
     runtime->hw.rates = 0;
     
     list_for_each_entry(fp, &subs->fmt_list, list) {
         runtime->hw.rates |= fp->rates;
         if (runtime->hw.rate_min > fp->rate_min)
             runtime->hw.rate_min = fp->rate_min;
         if (runtime->hw.rate_max < fp->rate_max)
             runtime->hw.rate_max = fp->rate_max;
         if (runtime->hw.channels_min > fp->channels)
             runtime->hw.channels_min = fp->channels;
         if (runtime->hw.channels_max < fp->channels)
             runtime->hw.channels_max = fp->channels;
     }
 
 
     return 0;
 }
 
 /**
  * @brief PCM open callback for playback.
  *
  * This function is called when the PCM playback stream is opened. It sets up the runtime
  * hardware information and prepares the media stream.
  *
  * @param substream Pointer to the PCM substream.
  * @return 0 on success or a negative error code on failure.
  */
 static int snd_usb_pcm_open(struct snd_pcm_substream *substream)
 {
     pr_info("Playback Open\n");
     int direction = substream->stream; /* playback stream =0 capture stream =1*/
     struct snd_usb_stream *as = snd_pcm_substream_chip(substream);//extract the USB stream from the substream
     struct snd_pcm_runtime *runtime = substream->runtime;//runtime is a structure that holds the current state of the PCM device
     struct snd_usb_substream *subs = &as->substream[direction];//subs is a structure that holds the state of the USB substream
     int ret;
 
     runtime->hw = snd_usb_hardware;//hw is a structure that describes the hardware capabilities of the PCM device
     runtime->private_data = subs;
     subs->pcm_substream = substream;
 
     ret = setup_hw_info(runtime, subs);
     if (ret < 0)
         return ret;
     ret = snd_usb_autoresume(subs->stream->chip);
     if (ret < 0)
         return ret;
     ret = snd_media_stream_init(subs, as->pcm, direction);
     if (ret < 0)
         snd_usb_autosuspend(subs->stream->chip);
     return ret;
 }
 
 /**
  * @brief PCM close callback for playback.
  *
  * This function is called when the PCM playback stream is closed. It stops the media stream,
  * changes power state, and cleans up endpoint associations.
  *
  * @param substream Pointer to the PCM substream.
  * @return 0 on success or a negative error code on failure.
  */
 static int snd_usb_pcm_close(struct snd_pcm_substream *substream)
 {
     pr_info("Playback Close\n");
     int direction = substream->stream;
     struct snd_usb_stream *as = snd_pcm_substream_chip(substream);
     struct snd_usb_substream *subs = &as->substream[direction];
     int ret;
 
     snd_media_stop_pipeline(subs);
     if (!snd_usb_lock_shutdown(subs->stream->chip)) {
         ret = snd_usb_pcm_change_state(subs, UAC3_PD_STATE_D1);
         snd_usb_unlock_shutdown(subs->stream->chip);
         if (ret < 0)
             return ret;
     }
     subs->pcm_substream = NULL;
     snd_usb_autosuspend(subs->stream->chip);
     return 0;
 }
 
 /**
  * @brief Advance the URB context queue.
  *
  * This function updates the internal state tracking for a USB audio substream when a
  *  URB (USB data packet) is queued for transmission or reception.
  *
  * @param subs Pointer to the USB substream.
  * @param urb Pointer to the URB being processed.
  * @param bytes Number of bytes to advance.
  */
 static void urb_ctx_queue_advance(struct snd_usb_substream *subs,
                   struct urb *urb, unsigned int bytes)
 {
     struct snd_urb_ctx *ctx = urb->context;
     ctx->queued += bytes;
     //inflight_bytes is the total number of bytes that have been queued for transfer but not yet completed
     //hwptr_done is the pointer to the current hardware position within the ALSA ring buffer
     subs->inflight_bytes += bytes;
     subs->hwptr_done += bytes;
     //If hwptr_done moves beyond the end of the buffer:Wrap it back to the start by subtracting buffer_bytes
     if (subs->hwptr_done >= subs->buffer_bytes)
         subs->hwptr_done -= subs->buffer_bytes;
    /*buffer_bytes = 4096
    hwptr_done = 4000
    bytes = 200
    hwptr_done = 4000 + 200 = 4200 → overflows
    4200 - 4096 = 104
    */

 }
 
 /**
  * @brief Copy data to the URB transfer buffer.
  *
  * Copies PCM data from the DMA area to the URB transfer buffer, handling wrap-around
  * if the end of the DMA buffer is reached.
  *
  * @param subs Pointer to the USB substream.
  * @param urb Pointer to the URB.
  * @param offset Offset in the URB transfer buffer to start copying.
  * @param stride The number of bytes per frame.
  * @param bytes The total number of bytes to copy.
  */
 static void copy_to_urb(struct snd_usb_substream *subs, struct urb *urb,
             int offset, int stride, unsigned int bytes)
 {
     struct snd_pcm_runtime *runtime = subs->pcm_substream->runtime;
     if (subs->hwptr_done + bytes > subs->buffer_bytes) {
         unsigned int bytes1 = subs->buffer_bytes - subs->hwptr_done;
         memcpy(urb->transfer_buffer + offset,
                runtime->dma_area + subs->hwptr_done, bytes1);
         memcpy(urb->transfer_buffer + offset + bytes1,
                runtime->dma_area, bytes - bytes1);
     } else {
         memcpy(urb->transfer_buffer + offset,
                runtime->dma_area + subs->hwptr_done, bytes);
     }
     urb_ctx_queue_advance(subs, urb, bytes);
 }
 
 /**
  * @brief Prepare a playback URB.
  *
  * Prepares a URB by calculating packet sizes, copying data from the DMA buffer, and
  * updating the internal transfer counters. If a period has elapsed, the corresponding
  * ALSA PCM period elapsed callback is triggered.
  *
  * @param subs Pointer to the USB substream.
  * @param urb Pointer to the URB to prepare.
  * @param in_stream_lock Indicates whether this is called under the PCM stream lock.
  * @return 0 on success, or -EAGAIN if no data is available to prepare.
  */
 static int prepare_playback_urb(struct snd_usb_substream *subs,
                 struct urb *urb,
                 bool in_stream_lock)
 {
     struct snd_pcm_runtime *runtime = subs->pcm_substream->runtime;
     struct snd_usb_endpoint *ep = subs->data_endpoint;
     struct snd_urb_ctx *ctx = urb->context;
     unsigned int frames, bytes;
     int counts;
     unsigned int transfer_done, frame_limit;
     int i, stride, period_elapsed = 0;
     unsigned long flags;
     int err = 0;
 
     stride = ep->stride;
     frames = 0;
     ctx->queued = 0;
     urb->number_of_packets = 0;
 
     spin_lock_irqsave(&subs->lock, flags);
     frame_limit = subs->frame_limit + ep->max_urb_frames;
     transfer_done = subs->transfer_done;
 
     for (i = 0; i < ctx->packets; i++) {
         counts = snd_usb_endpoint_next_packet_size(ep, ctx, i, 0);
         if (counts < 0)
             break;
         urb->iso_frame_desc[i].offset = frames * stride;
         urb->iso_frame_desc[i].length = counts * stride;
         frames += counts;
         urb->number_of_packets++;
         transfer_done += counts;
         if (transfer_done >= runtime->period_size) {
             transfer_done -= runtime->period_size;
             period_elapsed = 1;
             if (subs->fmt_type == UAC_FORMAT_TYPE_II) {
                 if (transfer_done > 0) {
                     frames -= transfer_done;
                     counts -= transfer_done;
                     urb->iso_frame_desc[i].length = counts * stride;
                     transfer_done = 0;
                 }
                 i++;
                 if (i < ctx->packets) {
                     urb->iso_frame_desc[i].offset = frames * stride;
                     urb->iso_frame_desc[i].length = 0;
                     urb->number_of_packets++;
                 }
                 break;
             }
         }
         if ((period_elapsed || transfer_done >= frame_limit) &&
             !snd_usb_endpoint_implicit_feedback_sink(ep))
             break;
     }
 
     if (!frames) {
         err = -EAGAIN;
         goto unlock;
     }
 
     bytes = frames * stride;
     subs->transfer_done = transfer_done;
     subs->frame_limit = frame_limit;
     copy_to_urb(subs, urb, 0, stride, bytes);
 
     subs->last_frame_number = usb_get_current_frame_number(subs->dev);
     if (subs->trigger_tstamp_pending_update) {
         snd_pcm_gettime(runtime, &runtime->trigger_tstamp);
         subs->trigger_tstamp_pending_update = false;
     }
 
 unlock:
     spin_unlock_irqrestore(&subs->lock, flags);
     if (err < 0)
         return err;
     urb->transfer_buffer_length = bytes;
     if (period_elapsed) {
         if (in_stream_lock)
             snd_pcm_period_elapsed_under_stream_lock(subs->pcm_substream);
         else
             snd_pcm_period_elapsed(subs->pcm_substream);
     }
     return 0;
 }
 
 /**
  * @brief Retire a completed playback URB.
  *
  * This function updates the internal queued byte count and calls the period elapsed callback
  * if needed.
  *
  * @param subs Pointer to the USB substream.
  * @param urb Pointer to the completed URB.
  */
 static void retire_playback_urb(struct snd_usb_substream *subs,
                    struct urb *urb)
 {
     unsigned long flags;
     struct snd_urb_ctx *ctx = urb->context;
     bool period_elapsed = false;
 
     spin_lock_irqsave(&subs->lock, flags);
     if (ctx->queued) {
         if (subs->inflight_bytes >= ctx->queued)
             subs->inflight_bytes -= ctx->queued;
         else
             subs->inflight_bytes = 0;
     }
     subs->last_frame_number = usb_get_current_frame_number(subs->dev);
     if (subs->running) {
         period_elapsed = subs->period_elapsed_pending;
         subs->period_elapsed_pending = 0;
     }
     spin_unlock_irqrestore(&subs->lock, flags);
     if (period_elapsed)
         snd_pcm_period_elapsed(subs->pcm_substream);
 }
 
 /**
  * @brief Playback trigger callback.
  *
  * Handles various trigger commands (start, stop, pause, etc.) for the playback substream.
  *
  * @param substream Pointer to the PCM substream.
  * @param cmd The trigger command (e.g., SNDRV_PCM_TRIGGER_START).
  * @return 0 on success or -EINVAL if an unknown command is received.
  */
 static int snd_usb_substream_playback_trigger(struct snd_pcm_substream *substream,
                           int cmd)
 {
     struct snd_usb_substream *subs = substream->runtime->private_data;
 
     switch (cmd) {
     case SNDRV_PCM_TRIGGER_START:
         pr_info("PCM trigger start\n");
         subs->trigger_tstamp_pending_update = true;
        
     case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
         snd_usb_endpoint_set_callback(subs->data_endpoint,
                           prepare_playback_urb,
                           retire_playback_urb,
                           subs);
         subs->running = 1;
         dev_dbg(&subs->dev->dev, "Start Playback PCM\n");
         return 0;
     case SNDRV_PCM_TRIGGER_SUSPEND:
     case SNDRV_PCM_TRIGGER_STOP:
         pr_info("PCM trigger stop\n");
         stop_endpoints(subs, substream->runtime->state == SNDRV_PCM_STATE_DRAINING);
         snd_usb_endpoint_set_callback(subs->data_endpoint, NULL, NULL, NULL);
         subs->running = 0;
         dev_dbg(&subs->dev->dev, "Stop Playback PCM\n");
         return 0;
     case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
         snd_usb_endpoint_set_callback(subs->data_endpoint, NULL, retire_playback_urb, subs);
         subs->running = 0;
         dev_dbg(&subs->dev->dev, "Pause Playback PCM\n");
         return 0;
     }
 
     return -EINVAL;
 }
 
 /**
  * @brief PCM operations for USB playback.
  *
  * Defines the open, close, hw_params, hw_free, prepare, trigger, sync_stop,
  * and pointer callbacks for the USB PCM device.
  */
 static const struct snd_pcm_ops snd_usb_playback_ops = {
     .open      = snd_usb_pcm_open,
     .close     = snd_usb_pcm_close,
     .hw_params = snd_usb_hw_params,
     .hw_free   = snd_usb_hw_free,
     .prepare   = snd_usb_pcm_prepare,
     .trigger   = snd_usb_substream_playback_trigger,
     .sync_stop = snd_usb_pcm_sync_stop,
     .pointer   = snd_usb_pcm_pointer,
 };
 
 /**
  * @brief Set the PCM operations for the USB audio device.
  *
  * This function assigns the appropriate PCM operations (playback in this case)
  * to the given PCM instance.
  *
  * @param pcm Pointer to the PCM instance.
  * @param stream The stream type (e.g., SNDRV_PCM_STREAM_PLAYBACK).
  */
 void snd_usb_set_pcm_ops(struct snd_pcm *pcm, int stream)
 {
     const struct snd_pcm_ops *ops;
     ops = (stream == SNDRV_PCM_STREAM_PLAYBACK) ? &snd_usb_playback_ops : &snd_usb_playback_ops;
     snd_pcm_set_ops(pcm, stream, ops);
 }
 
 /**
  * @brief Preallocate the buffer for the substream.
  *
  * This function sets up the buffer for PCM transfers using either vmalloc
  * or scatter-gather DMA, depending on the configuration.
  *
  * @param subs Pointer to the USB substream.
  */
 void snd_usb_preallocate_buffer(struct snd_usb_substream *subs)
 {
     struct snd_pcm *pcm = subs->stream->pcm;
     struct snd_pcm_substream *s = pcm->streams[subs->direction].substream;
     struct device *dev = subs->dev->bus->sysdev;
 
     if (snd_usb_use_vmalloc)
         snd_pcm_set_managed_buffer(s, SNDRV_DMA_TYPE_VMALLOC, NULL, 0, 0);
     else
         snd_pcm_set_managed_buffer(s, SNDRV_DMA_TYPE_DEV_SG, dev, 64*1024, 512*1024);
 }
 