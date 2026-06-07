/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include "a2d_int.h"
#include "common/bt_defs.h"
#include "common/bt_target.h"
#include "stack/a2d_api.h"
#include "stack/a2d_sbc.h"
#include "stack/a2dp_vendor_opus.h"
#include "stack/a2dp_vendor_opus_decoder.h"

#if (defined(OPUS_DEC_INCLUDED) && OPUS_DEC_INCLUDED == TRUE)

static const tA2DP_OPUS_CIE a2dp_opus_android_sink_caps = {
    A2DP_OPUS_ANDROID_VENDOR_ID,
    A2DP_OPUS_ANDROID_CODEC_ID,
    A2DP_OPUS_VARIANT_ANDROID,
    A2DP_OPUS_CHANNEL_MODE_STEREO,
    A2DP_OPUS_20MS_FRAMESIZE,
    A2DP_OPUS_SAMPLING_FREQ_48000,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
};

static const tA2DP_OPUS_CIE a2dp_opus_pipewire_sink_caps = {
    A2DP_OPUS_PIPEWIRE_VENDOR_ID,
    A2DP_OPUS_PIPEWIRE_CODEC_ID,
    A2DP_OPUS_VARIANT_PIPEWIRE,
    0,
    0,
    0,
    2,
    1,
    A2DP_OPUS_AUDIO_LOCATION_FL | A2DP_OPUS_AUDIO_LOCATION_FR,
    A2DP_OPUS_FRAME_DURATION_10 | A2DP_OPUS_FRAME_DURATION_20 | A2DP_OPUS_FRAME_DURATION_40,
    A2DP_OPUS_FRAME_BITRATE_ALL,
    0,
    0,
    0,
    0,
    A2DP_OPUS_FRAME_BITRATE_ALL,
};

static const tA2DP_OPUS_CIE a2dp_opus_android_default_config = {
    A2DP_OPUS_ANDROID_VENDOR_ID,
    A2DP_OPUS_ANDROID_CODEC_ID,
    A2DP_OPUS_VARIANT_ANDROID,
    A2DP_OPUS_CHANNEL_MODE_STEREO,
    A2DP_OPUS_20MS_FRAMESIZE,
    A2DP_OPUS_SAMPLING_FREQ_48000,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
};

static const tA2DP_OPUS_CIE a2dp_opus_pipewire_default_config = {
    A2DP_OPUS_PIPEWIRE_VENDOR_ID,
    A2DP_OPUS_PIPEWIRE_CODEC_ID,
    A2DP_OPUS_VARIANT_PIPEWIRE,
    0,
    0,
    0,
    2,
    1,
    A2DP_OPUS_AUDIO_LOCATION_FL | A2DP_OPUS_AUDIO_LOCATION_FR,
    A2DP_OPUS_FRAME_DURATION_20,
    A2DP_OPUS_FRAME_BITRATE_ALL,
    0,
    0,
    0,
    0,
    A2DP_OPUS_FRAME_BITRATE_ALL,
};

static const tA2DP_DECODER_INTERFACE a2dp_decoder_interface_opus = {
    a2dp_opus_decoder_init,
    a2dp_opus_decoder_cleanup,
    NULL,
    a2dp_opus_decoder_decode_packet_header,
    a2dp_opus_decoder_decode_packet,
    NULL,
    NULL,
    a2dp_opus_decoder_configure,
};

static bool a2dp_opus_is_android(uint32_t vendor_id, uint16_t codec_id) {
  return vendor_id == A2DP_OPUS_ANDROID_VENDOR_ID &&
         codec_id == A2DP_OPUS_ANDROID_CODEC_ID;
}

static bool a2dp_opus_is_pipewire(uint32_t vendor_id, uint16_t codec_id) {
  return vendor_id == A2DP_OPUS_PIPEWIRE_VENDOR_ID &&
         codec_id == A2DP_OPUS_PIPEWIRE_CODEC_ID;
}

bool A2DP_VendorCodecIsOpus(uint32_t vendor_id, uint16_t codec_id) {
  return a2dp_opus_is_android(vendor_id, codec_id) ||
         a2dp_opus_is_pipewire(vendor_id, codec_id);
}

tA2D_STATUS A2DP_BuildInfoOpus(uint8_t media_type, const tA2DP_OPUS_CIE* p_ie,
                               uint8_t* p_result) {
  if (p_ie == NULL || p_result == NULL) {
    return A2D_INVALID_PARAMS;
  }

  const bool is_android = a2dp_opus_is_android(p_ie->vendorId, p_ie->codecId);
  const bool is_pipewire = a2dp_opus_is_pipewire(p_ie->vendorId, p_ie->codecId);
  if (!is_android && !is_pipewire) {
    return A2D_WRONG_CODEC;
  }

  *p_result++ = is_android ? A2DP_OPUS_ANDROID_CODEC_LEN : A2DP_OPUS_PIPEWIRE_CODEC_LEN;
  *p_result++ = (media_type << 4);
  *p_result++ = A2D_MEDIA_CT_NON_A2DP;

  *p_result++ = (uint8_t)(p_ie->vendorId & 0x000000FF);
  *p_result++ = (uint8_t)((p_ie->vendorId & 0x0000FF00) >> 8);
  *p_result++ = (uint8_t)((p_ie->vendorId & 0x00FF0000) >> 16);
  *p_result++ = (uint8_t)((p_ie->vendorId & 0xFF000000) >> 24);
  *p_result++ = (uint8_t)(p_ie->codecId & 0x00FF);
  *p_result++ = (uint8_t)((p_ie->codecId & 0xFF00) >> 8);

  if (is_android) {
    *p_result++ = (uint8_t)(p_ie->channelMode |
                            p_ie->frameSize |
                            p_ie->samplingFreq);
    return A2D_SUCCESS;
  }

  *p_result++ = p_ie->channels;
  *p_result++ = p_ie->coupled_streams;
  *p_result++ = (uint8_t)(p_ie->audio_location & 0x000000FF);
  *p_result++ = (uint8_t)((p_ie->audio_location & 0x0000FF00) >> 8);
  *p_result++ = (uint8_t)((p_ie->audio_location & 0x00FF0000) >> 16);
  *p_result++ = (uint8_t)((p_ie->audio_location & 0xFF000000) >> 24);
  *p_result++ = p_ie->frame_duration;
  *p_result++ = (uint8_t)(p_ie->maximum_bitrate & 0x00FF);
  *p_result++ = (uint8_t)((p_ie->maximum_bitrate & 0xFF00) >> 8);
  *p_result++ = p_ie->bidi_channels;
  *p_result++ = p_ie->bidi_coupled_streams;
  *p_result++ = (uint8_t)(p_ie->bidi_audio_location & 0x000000FF);
  *p_result++ = (uint8_t)((p_ie->bidi_audio_location & 0x0000FF00) >> 8);
  *p_result++ = (uint8_t)((p_ie->bidi_audio_location & 0x00FF0000) >> 16);
  *p_result++ = (uint8_t)((p_ie->bidi_audio_location & 0xFF000000) >> 24);
  *p_result++ = p_ie->bidi_frame_duration;
  *p_result++ = (uint8_t)(p_ie->bidi_maximum_bitrate & 0x00FF);
  *p_result++ = (uint8_t)((p_ie->bidi_maximum_bitrate & 0xFF00) >> 8);

  return A2D_SUCCESS;
}

tA2D_STATUS A2DP_ParseInfoOpus(tA2DP_OPUS_CIE* p_ie,
                               const uint8_t* p_codec_info,
                               bool is_capability) {
  uint8_t losc;
  uint8_t media_type;
  tA2D_CODEC_TYPE codec_type;

  if (p_ie == NULL || p_codec_info == NULL) return A2D_INVALID_PARAMS;
  memset(p_ie, 0, sizeof(*p_ie));

  losc = *p_codec_info++;
  media_type = (*p_codec_info++) >> 4;
  codec_type = *p_codec_info++;
  if (media_type != A2D_MEDIA_TYPE_AUDIO ||
      codec_type != A2D_MEDIA_CT_NON_A2DP) {
    return A2D_WRONG_CODEC;
  }

  p_ie->vendorId = (*p_codec_info & 0x000000FF) |
                   (*(p_codec_info + 1) << 8 & 0x0000FF00) |
                   (*(p_codec_info + 2) << 16 & 0x00FF0000) |
                   (*(p_codec_info + 3) << 24 & 0xFF000000);
  p_codec_info += 4;
  p_ie->codecId = (*p_codec_info & 0x00FF) |
                  (*(p_codec_info + 1) << 8 & 0xFF00);
  p_codec_info += 2;

  if (a2dp_opus_is_android(p_ie->vendorId, p_ie->codecId)) {
    if (losc != A2DP_OPUS_ANDROID_CODEC_LEN) {
      return A2D_WRONG_CODEC;
    }
    p_ie->variant = A2DP_OPUS_VARIANT_ANDROID;
    uint8_t config = *p_codec_info++;
    p_ie->channelMode = config & A2DP_OPUS_CHANNEL_MODE_MASK;
    p_ie->frameSize = config & A2DP_OPUS_FRAMESIZE_MASK;
    p_ie->samplingFreq = config & A2DP_OPUS_SAMPLING_FREQ_MASK;

    if (is_capability) {
      if (A2D_BitsSet(p_ie->channelMode) == A2D_SET_ZERO_BIT)
        return A2D_BAD_CH_MODE;
      if (A2D_BitsSet(p_ie->frameSize) == A2D_SET_ZERO_BIT)
        return A2D_FAIL;
      if (A2D_BitsSet(p_ie->samplingFreq) == A2D_SET_ZERO_BIT)
        return A2D_BAD_SAMP_FREQ;
      return A2D_SUCCESS;
    }

    if (A2D_BitsSet(p_ie->channelMode) != A2D_SET_ONE_BIT)
      return A2D_BAD_CH_MODE;
    if (A2D_BitsSet(p_ie->frameSize) != A2D_SET_ONE_BIT)
      return A2D_FAIL;
    if (A2D_BitsSet(p_ie->samplingFreq) != A2D_SET_ONE_BIT)
      return A2D_BAD_SAMP_FREQ;
    return A2D_SUCCESS;
  }

  if (!a2dp_opus_is_pipewire(p_ie->vendorId, p_ie->codecId) ||
      losc != A2DP_OPUS_PIPEWIRE_CODEC_LEN) {
    return A2D_WRONG_CODEC;
  }

  p_ie->variant = A2DP_OPUS_VARIANT_PIPEWIRE;
  p_ie->channels = *p_codec_info++;
  p_ie->coupled_streams = *p_codec_info++;
  p_ie->audio_location = (*p_codec_info & 0x000000FF) |
                         (*(p_codec_info + 1) << 8 & 0x0000FF00) |
                         (*(p_codec_info + 2) << 16 & 0x00FF0000) |
                         (*(p_codec_info + 3) << 24 & 0xFF000000);
  p_codec_info += 4;
  p_ie->frame_duration = *p_codec_info++;
  p_ie->maximum_bitrate = (*p_codec_info & 0x00FF) |
                          (*(p_codec_info + 1) << 8 & 0xFF00);
  p_codec_info += 2;
  p_ie->bidi_channels = *p_codec_info++;
  p_ie->bidi_coupled_streams = *p_codec_info++;
  p_ie->bidi_audio_location = (*p_codec_info & 0x000000FF) |
                              (*(p_codec_info + 1) << 8 & 0x0000FF00) |
                              (*(p_codec_info + 2) << 16 & 0x00FF0000) |
                              (*(p_codec_info + 3) << 24 & 0xFF000000);
  p_codec_info += 4;
  p_ie->bidi_frame_duration = *p_codec_info++;
  p_ie->bidi_maximum_bitrate = (*p_codec_info & 0x00FF) |
                               (*(p_codec_info + 1) << 8 & 0xFF00);

  if (is_capability) {
    if (A2D_BitsSet(p_ie->channels) == A2D_SET_ZERO_BIT)
      return A2D_BAD_CH_MODE;
    if (A2D_BitsSet(p_ie->coupled_streams) == A2D_SET_ZERO_BIT)
      return A2D_BAD_CH_MODE;
    if (A2D_BitsSet(p_ie->audio_location) == A2D_SET_ZERO_BIT)
      return A2D_BAD_CH_MODE;
  }

  return A2D_SUCCESS;
}

tA2D_STATUS A2DP_IsVendorPeerSourceCodecValidOpus(const uint8_t* p_codec_info) {
  tA2DP_OPUS_CIE cfg_cie;
  tA2D_STATUS status = A2DP_ParseInfoOpus(&cfg_cie, p_codec_info, true);
  if (status == A2D_SUCCESS) return status;
  return A2DP_ParseInfoOpus(&cfg_cie, p_codec_info, false);
}

bool A2DP_IsVendorPeerSinkCodecValidOpus(const uint8_t* p_codec_info) {
  tA2DP_OPUS_CIE cfg_cie;
  return (A2DP_ParseInfoOpus(&cfg_cie, p_codec_info, false) == A2D_SUCCESS) ||
         (A2DP_ParseInfoOpus(&cfg_cie, p_codec_info, true) == A2D_SUCCESS);
}

btav_a2dp_codec_index_t A2DP_VendorSinkCodecIndexOpus(const uint8_t* p_codec_info) {
  tA2DP_OPUS_CIE cfg_cie;
  if (A2DP_ParseInfoOpus(&cfg_cie, p_codec_info, true) != A2D_SUCCESS &&
      A2DP_ParseInfoOpus(&cfg_cie, p_codec_info, false) != A2D_SUCCESS) {
    return BTAV_A2DP_CODEC_INDEX_MAX;
  }
  return (cfg_cie.variant == A2DP_OPUS_VARIANT_ANDROID)
             ? BTAV_A2DP_CODEC_INDEX_SINK_OPUS_ANDROID
             : BTAV_A2DP_CODEC_INDEX_SINK_OPUS;
}

btav_a2dp_codec_index_t A2DP_VendorSourceCodecIndexOpus(const uint8_t* p_codec_info) {
  UNUSED(p_codec_info);
  return BTAV_A2DP_CODEC_INDEX_SOURCE_OPUS;
}

const char* A2DP_VendorCodecNameOpus(const uint8_t* p_codec_info) {
  tA2DP_OPUS_CIE cfg_cie;
  if (A2DP_ParseInfoOpus(&cfg_cie, p_codec_info, true) != A2D_SUCCESS &&
      A2DP_ParseInfoOpus(&cfg_cie, p_codec_info, false) != A2D_SUCCESS) {
    return "Opus";
  }
  return (cfg_cie.variant == A2DP_OPUS_VARIANT_ANDROID) ? "Opus" : "Opus 05";
}

bool A2DP_VendorCodecTypeEqualsOpus(const uint8_t* p_codec_info_a,
                                    const uint8_t* p_codec_info_b) {
  tA2DP_OPUS_CIE cie_a;
  tA2DP_OPUS_CIE cie_b;
  tA2D_STATUS a2dp_status = A2DP_ParseInfoOpus(&cie_a, p_codec_info_a, true);
  if (a2dp_status != A2D_SUCCESS) return false;
  a2dp_status = A2DP_ParseInfoOpus(&cie_b, p_codec_info_b, true);
  if (a2dp_status != A2D_SUCCESS) return false;
  return cie_a.vendorId == cie_b.vendorId && cie_a.codecId == cie_b.codecId;
}

bool A2DP_VendorInitCodecConfigOpus(btav_a2dp_codec_index_t codec_index, UINT8 *p_result) {
  switch (codec_index) {
    case BTAV_A2DP_CODEC_INDEX_SINK_OPUS:
      return A2DP_BuildInfoOpus(A2D_MEDIA_TYPE_AUDIO, &a2dp_opus_pipewire_sink_caps,
                                p_result) == A2D_SUCCESS;
    case BTAV_A2DP_CODEC_INDEX_SINK_OPUS_ANDROID:
      return A2DP_BuildInfoOpus(A2D_MEDIA_TYPE_AUDIO, &a2dp_opus_android_sink_caps,
                                p_result) == A2D_SUCCESS;
    default:
      return false;
  }
}

bool A2DP_VendorInitCodecConfigOpusSink(uint8_t* p_codec_info) {
  return A2DP_BuildInfoOpus(A2D_MEDIA_TYPE_AUDIO, &a2dp_opus_pipewire_sink_caps,
                            p_codec_info) == A2D_SUCCESS;
}

bool A2DP_VendorBuildCodecConfigOpus(UINT8 *p_src_cap, UINT8 *p_result) {
  tA2DP_OPUS_CIE src_cap;
  tA2DP_OPUS_CIE pref_cap;
  tA2D_STATUS status;

  if ((status = A2DP_ParseInfoOpus(&src_cap, p_src_cap, true)) != A2D_SUCCESS) {
    APPL_TRACE_ERROR("%s: Cant parse src cap ret = %d", __func__, status);
    return false;
  }

  if (src_cap.variant == A2DP_OPUS_VARIANT_ANDROID) {
    pref_cap = a2dp_opus_android_default_config;
    pref_cap.channelMode &= src_cap.channelMode;
    pref_cap.frameSize &= src_cap.frameSize;
    pref_cap.samplingFreq &= src_cap.samplingFreq;

    if (A2D_BitsSet(pref_cap.channelMode) == A2D_SET_ZERO_BIT ||
        A2D_BitsSet(pref_cap.frameSize) == A2D_SET_ZERO_BIT ||
        A2D_BitsSet(pref_cap.samplingFreq) == A2D_SET_ZERO_BIT) {
      APPL_TRACE_ERROR("%s: Unsupported Android Opus capabilities", __func__);
      return false;
    }
  } else {
    pref_cap = a2dp_opus_pipewire_default_config;

    if (src_cap.channels < (2 * src_cap.coupled_streams)) {
      APPL_TRACE_ERROR("%s: Invalid PipeWire Opus channel config", __func__);
      return false;
    }
    pref_cap.channels = min(src_cap.channels, pref_cap.channels);
    pref_cap.coupled_streams = min(src_cap.coupled_streams, pref_cap.coupled_streams);
    pref_cap.audio_location = src_cap.audio_location &
                              (A2DP_OPUS_AUDIO_LOCATION_FL | A2DP_OPUS_AUDIO_LOCATION_FR);
    pref_cap.frame_duration = src_cap.frame_duration &
                              a2dp_opus_pipewire_default_config.frame_duration;
    if (A2D_BitsSet(pref_cap.audio_location) == A2D_SET_ZERO_BIT ||
        A2D_BitsSet(pref_cap.frame_duration) == A2D_SET_ZERO_BIT) {
      APPL_TRACE_ERROR("%s: Unsupported PipeWire Opus capabilities", __func__);
      return false;
    }
    if (src_cap.maximum_bitrate > 0) {
      pref_cap.maximum_bitrate = src_cap.maximum_bitrate;
    }
  }

  return A2DP_BuildInfoOpus(A2D_MEDIA_TYPE_AUDIO, &pref_cap, p_result) == A2D_SUCCESS;
}

const tA2DP_DECODER_INTERFACE* A2DP_GetVendorDecoderInterfaceOpus(
    const uint8_t* p_codec_info) {
  if (!A2DP_IsVendorPeerSinkCodecValidOpus(p_codec_info)) return NULL;
  return &a2dp_decoder_interface_opus;
}

#endif /* defined(OPUS_DEC_INCLUDED) && OPUS_DEC_INCLUDED == TRUE) */
