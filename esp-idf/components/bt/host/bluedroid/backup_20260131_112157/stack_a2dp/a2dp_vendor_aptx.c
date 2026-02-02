/*
 * SPDX-FileCopyrightText: 2016 The Android Open Source Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include "a2d_int.h"
#include "common/bt_defs.h"
#include "common/bt_target.h"
#include "stack/a2d_api.h"
#include "stack/a2d_sbc.h"
#include "stack/a2dp_vendor_aptx.h"
#include "stack/a2dp_vendor_aptx_decoder.h"

#if (defined(APTX_DEC_INCLUDED) && APTX_DEC_INCLUDED == TRUE)

/* aptX Source codec capabilities */
static const tA2DP_APTX_CIE a2dp_aptx_source_caps = {
    A2DP_APTX_VENDOR_ID,                                       /* vendorId */
    A2DP_APTX_CODEC_ID_BLUETOOTH,                              /* codecId */
    (A2DP_APTX_SAMPLERATE_44100 | A2DP_APTX_SAMPLERATE_48000), /* sampleRate */
    A2DP_APTX_CHANNELS_STEREO,                                 /* channelMode */
    A2DP_APTX_FUTURE_1,                                        /* future1 */
    A2DP_APTX_FUTURE_2,                                        /* future2 */
    BTAV_A2DP_CODEC_BITS_PER_SAMPLE_16 /* bits_per_sample */
};

/* aptX Sink codec capabilities */
static const tA2DP_APTX_CIE a2dp_aptx_sink_caps = {
    A2DP_APTX_VENDOR_ID,                                       /* vendorId */
    A2DP_APTX_CODEC_ID_BLUETOOTH,                              /* codecId */
    (A2DP_APTX_SAMPLERATE_44100 | A2DP_APTX_SAMPLERATE_48000), /* sampleRate */
    A2DP_APTX_CHANNELS_STEREO,                                 /* channelMode */
    A2DP_APTX_FUTURE_1,                                        /* future1 */
    A2DP_APTX_FUTURE_2,                                        /* future2 */
    BTAV_A2DP_CODEC_BITS_PER_SAMPLE_16 /* bits_per_sample */
};

/* Default aptX codec configuration */
static const tA2DP_APTX_CIE a2dp_aptx_default_config = {
    A2DP_APTX_VENDOR_ID,               /* vendorId */
    A2DP_APTX_CODEC_ID_BLUETOOTH,      /* codecId */
    A2DP_APTX_SAMPLERATE_48000,        /* sampleRate */
    A2DP_APTX_CHANNELS_STEREO,         /* channelMode */
    A2DP_APTX_FUTURE_1,                /* future1 */
    A2DP_APTX_FUTURE_2,                /* future2 */
    BTAV_A2DP_CODEC_BITS_PER_SAMPLE_16 /* bits_per_sample */
};

static const tA2DP_DECODER_INTERFACE a2dp_decoder_interface_aptx = {
    a2dp_aptx_decoder_init,
    a2dp_aptx_decoder_cleanup,
    a2dp_aptx_decoder_reset,
    a2dp_aptx_decoder_decode_packet_header,
    a2dp_aptx_decoder_decode_packet,
    NULL,  // decoder_start
    NULL,  // decoder_suspend
    a2dp_aptx_decoder_configure,
};

tA2D_STATUS A2DP_BuildInfoAptx(uint8_t media_type,
                                       const tA2DP_APTX_CIE* p_ie,
                                       uint8_t* p_result) {
  if (p_ie == NULL || p_result == NULL) {
    return A2D_INVALID_PARAMS;
  }

  *p_result++ = A2DP_APTX_CODEC_LEN;
  *p_result++ = (media_type << 4);
  *p_result++ = A2D_MEDIA_CT_NON_A2DP;
  *p_result++ = (uint8_t)(p_ie->vendorId & 0x000000FF);
  *p_result++ = (uint8_t)((p_ie->vendorId & 0x0000FF00) >> 8);
  *p_result++ = (uint8_t)((p_ie->vendorId & 0x00FF0000) >> 16);
  *p_result++ = (uint8_t)((p_ie->vendorId & 0xFF000000) >> 24);
  *p_result++ = (uint8_t)(p_ie->codecId & 0x00FF);
  *p_result++ = (uint8_t)((p_ie->codecId & 0xFF00) >> 8);
  *p_result++ = p_ie->sampleRate | p_ie->channelMode;

  return A2D_SUCCESS;
}

tA2D_STATUS A2DP_ParseInfoAptx(tA2DP_APTX_CIE* p_ie,
                                const uint8_t* p_codec_info,
                                bool is_capability) {
  uint8_t losc;
  uint8_t media_type;
  tA2D_CODEC_TYPE codec_type;

  if (p_ie == NULL || p_codec_info == NULL) return A2D_INVALID_PARAMS;

  // Check the codec capability length
  losc = *p_codec_info++;
  if (losc != A2DP_APTX_CODEC_LEN) return A2D_WRONG_CODEC;

  media_type = (*p_codec_info++) >> 4;
  codec_type = *p_codec_info++;
  /* Check the Media Type and Media Codec Type */
  if (media_type != AVDT_MEDIA_AUDIO ||
      codec_type != A2D_MEDIA_CT_NON_A2DP) {
    return A2D_WRONG_CODEC;
  }

  // Check the Vendor ID and Codec ID */
  p_ie->vendorId = (*p_codec_info & 0x000000FF) |
                   (*(p_codec_info + 1) << 8 & 0x0000FF00) |
                   (*(p_codec_info + 2) << 16 & 0x00FF0000) |
                   (*(p_codec_info + 3) << 24 & 0xFF000000);
  p_codec_info += 4;
  p_ie->codecId =
      (*p_codec_info & 0x00FF) | (*(p_codec_info + 1) << 8 & 0xFF00);
  p_codec_info += 2;
  if (p_ie->vendorId != A2DP_APTX_VENDOR_ID ||
      p_ie->codecId != A2DP_APTX_CODEC_ID_BLUETOOTH) {
    return A2D_WRONG_CODEC;
  }

  p_ie->channelMode = *p_codec_info & 0x0F;
  p_ie->sampleRate = *p_codec_info & 0xF0;
  p_codec_info++;

  if (is_capability) {
    // NOTE: The checks here are very liberal. We should be using more
    // pedantic checks specific to the SRC or SNK as specified in the spec.
    if (A2D_BitsSet(p_ie->sampleRate) == A2D_SET_ZERO_BIT)
      return A2D_BAD_SAMP_FREQ;
    if (A2D_BitsSet(p_ie->channelMode) == A2D_SET_ZERO_BIT)
      return A2D_BAD_CH_MODE;

    return A2D_SUCCESS;
  }

  if (A2D_BitsSet(p_ie->sampleRate) != A2D_SET_ONE_BIT)
    return A2D_BAD_SAMP_FREQ;
  if (A2D_BitsSet(p_ie->channelMode) != A2D_SET_ONE_BIT)
    return A2D_BAD_CH_MODE;
  return A2D_SUCCESS;
}

tA2D_STATUS A2DP_IsVendorPeerSourceCodecValidAptx(const uint8_t* p_codec_info) {
  tA2DP_APTX_CIE cfg_cie;

  /* Use a liberal check when parsing the codec info */
  tA2D_STATUS status;
  status = A2DP_ParseInfoAptx(&cfg_cie, p_codec_info, true);
  if (status == A2D_SUCCESS) {
    return status;
  }

  return A2DP_ParseInfoAptx(&cfg_cie, p_codec_info, false);
}

bool A2DP_IsVendorPeerSinkCodecValidAptx(const uint8_t* p_codec_info) {
  tA2DP_APTX_CIE cfg_cie;

  /* Use a liberal check when parsing the codec info */
  return (A2DP_ParseInfoAptx(&cfg_cie, p_codec_info, false) == A2D_SUCCESS) ||
         (A2DP_ParseInfoAptx(&cfg_cie, p_codec_info, true) == A2D_SUCCESS);
}

tA2D_STATUS A2DP_CodecInfoMatchesCapabilityAptx(
    const tA2DP_APTX_CIE* p_cap, const uint8_t* p_codec_info,
    bool is_capability) {
  tA2D_STATUS status;
  tA2DP_APTX_CIE cfg_cie;

  /* parse configuration */
  status = A2DP_ParseInfoAptx(&cfg_cie, p_codec_info, is_capability);
  if (status != A2D_SUCCESS) {
    APPL_TRACE_ERROR("%s:%d %s status = 0x%X", __FILE__, __LINE__, __FUNCTION__, status);
    return status;
  }

  /* verify that each parameter is in range */

  LOG_VERBOSE("%s: FREQ peer: 0x%x, capability 0x%x", __func__,
              cfg_cie.sampleRate, p_cap->sampleRate);
  LOG_VERBOSE("%s: CH_MODE peer: 0x%x, capability 0x%x", __func__,
              cfg_cie.channelMode, p_cap->channelMode);

  /* sampling frequency */
  if ((cfg_cie.sampleRate & p_cap->sampleRate) == 0) return A2D_NS_SAMP_FREQ;

  /* channel mode */
  if ((cfg_cie.channelMode & p_cap->channelMode) == 0) return A2D_NS_CH_MODE;

  return A2D_SUCCESS;
}

btav_a2dp_codec_index_t A2DP_VendorSinkCodecIndexAptx(
    const uint8_t* p_codec_info) {
  UNUSED(p_codec_info);
  return BTAV_A2DP_CODEC_INDEX_SINK_APTX;
}

btav_a2dp_codec_index_t A2DP_VendorSourceCodecIndexAptx(
    const uint8_t* p_codec_info) {
  UNUSED(p_codec_info);
  return BTAV_A2DP_CODEC_INDEX_SOURCE_APTX;
}

const char* A2DP_VendorCodecNameAptx(const uint8_t* p_codec_info) {
  UNUSED(p_codec_info);
  return "aptX";
}

bool A2DP_VendorCodecTypeEqualsAptx(const uint8_t* p_codec_info_a,
                                    const uint8_t* p_codec_info_b) {
  tA2DP_APTX_CIE aptx_cie_a;
  tA2DP_APTX_CIE aptx_cie_b;

  // Check whether the codec info contains valid data
  tA2D_STATUS a2dp_status =
      A2DP_ParseInfoAptx(&aptx_cie_a, p_codec_info_a, true);
  if (a2dp_status != A2D_SUCCESS) {
    if (a2dp_status != A2D_WRONG_CODEC)
      LOG_ERROR("%s: cannot decode codec information: %d", __func__, a2dp_status);
    return false;
  }
  a2dp_status = A2DP_ParseInfoAptx(&aptx_cie_b, p_codec_info_b, true);
  if (a2dp_status != A2D_SUCCESS) {
    if (a2dp_status != A2D_WRONG_CODEC)
      LOG_ERROR("%s: cannot decode codec information: %d", __func__, a2dp_status);
    return false;
  }

  return true;
}

bool A2DP_VendorInitCodecConfigAptx(btav_a2dp_codec_index_t codec_index, UINT8 *p_result) {
  switch(codec_index) {
    case BTAV_A2DP_CODEC_INDEX_SINK_APTX:
      return A2DP_VendorInitCodecConfigAptxSink(p_result);
    default:
      break;
  }
  return false;
}

bool A2DP_VendorInitCodecConfigAptxSink(uint8_t* p_codec_info) {
  tA2D_STATUS sts = A2D_FAIL;
  sts = A2DP_BuildInfoAptx(A2D_MEDIA_TYPE_AUDIO, &a2dp_aptx_sink_caps, p_codec_info);
  return sts == A2D_SUCCESS;
}

bool A2DP_VendorBuildCodecConfigAptx(UINT8 *p_src_cap, UINT8 *p_result) {
  tA2DP_APTX_CIE src_cap;
  tA2DP_APTX_CIE pref_cap = a2dp_aptx_default_config;
  tA2D_STATUS status;

  if ((status = A2DP_ParseInfoAptx(&src_cap, p_src_cap, TRUE)) != 0) {
    APPL_TRACE_ERROR("%s: Cant parse src cap ret = %d", __func__, status);
    return false;
  }

  if (src_cap.sampleRate & A2DP_APTX_SAMPLERATE_48000) {
    pref_cap.sampleRate = A2DP_APTX_SAMPLERATE_48000;
  } else if (src_cap.sampleRate & A2DP_APTX_SAMPLERATE_44100) {
    pref_cap.sampleRate = A2DP_APTX_SAMPLERATE_44100;
  } else {
    APPL_TRACE_ERROR("%s: Unsupported sample rate 0x%x", __func__,
                     src_cap.sampleRate);
    return false;
  }

  if (src_cap.channelMode & A2DP_APTX_CHANNELS_STEREO) {
    pref_cap.channelMode = A2DP_APTX_CHANNELS_STEREO;
  } else if (src_cap.channelMode & A2DP_APTX_CHANNELS_STEREO) {
    pref_cap.channelMode = A2DP_APTX_CHANNELS_MONO;
  } else {
    APPL_TRACE_ERROR("%s: Unsupported channel mode 0x%x", __func__,
                     src_cap.channelMode);
    return false;
  }

  A2DP_BuildInfoAptx(A2D_MEDIA_TYPE_AUDIO, (tA2DP_APTX_CIE *) &pref_cap, p_result);
  return true;
}

const tA2DP_DECODER_INTERFACE* A2DP_GetVendorDecoderInterfaceAptx(
    const uint8_t* p_codec_info) {
  if (!A2DP_IsVendorPeerSinkCodecValidAptx(p_codec_info)) return NULL;

  return &a2dp_decoder_interface_aptx;
}

#endif /* defined(APTX_DEC_INCLUDED) && APTX_DEC_INCLUDED == TRUE) */
