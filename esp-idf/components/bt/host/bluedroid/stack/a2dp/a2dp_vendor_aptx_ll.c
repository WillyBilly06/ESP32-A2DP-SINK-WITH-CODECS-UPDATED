/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include "a2d_int.h"
#include "common/bt_defs.h"
#include "common/bt_target.h"
#include "stack/a2d_api.h"
#include "stack/a2d_sbc.h"
#include "stack/a2dp_vendor_aptx_ll.h"
#include "stack/a2dp_vendor_aptx_ll_constants.h"
#include "stack/a2dp_vendor_aptx_decoder.h"

#if (defined(A2D_INCLUDED) && A2D_INCLUDED == TRUE)

/* aptX-LL Source codec capabilities */
static const tA2DP_APTX_LL_CIE a2dp_aptx_ll_source_caps = {
    A2DP_APTX_LL_VENDOR_ID,          /* vendorId */
    A2DP_APTX_LL_CODEC_ID_BLUETOOTH, /* codecId */
    (A2DP_APTX_LL_SAMPLERATE_44100 |
     A2DP_APTX_LL_SAMPLERATE_48000),   /* sampleRate */
    A2DP_APTX_LL_CHANNELS_STEREO,      /* channelMode */
    0,    /* bidirect_link */
    0,    /* has_new_caps */
    BTAV_A2DP_CODEC_BITS_PER_SAMPLE_16 /* bits_per_sample */
};

/* aptX-LL Sink codec capabilities */
static const tA2DP_APTX_LL_CIE a2dp_aptx_ll_sink_caps = {
    A2DP_APTX_LL_VENDOR_ID,          /* vendorId */
    A2DP_APTX_LL_CODEC_ID_BLUETOOTH, /* codecId */
    (A2DP_APTX_LL_SAMPLERATE_44100 |
     A2DP_APTX_LL_SAMPLERATE_48000),   /* sampleRate */
    A2DP_APTX_LL_CHANNELS_STEREO,      /* channelMode */
    0,    /* bidirect_link */
    0,    /* has_new_caps */
    BTAV_A2DP_CODEC_BITS_PER_SAMPLE_16 /* bits_per_sample */
};

/* Default aptX-LL codec configuration */
static const tA2DP_APTX_LL_CIE a2dp_aptx_ll_default_config = {
    A2DP_APTX_LL_VENDOR_ID,            /* vendorId */
    A2DP_APTX_LL_CODEC_ID_BLUETOOTH,   /* codecId */
    A2DP_APTX_LL_SAMPLERATE_48000,     /* sampleRate */
    A2DP_APTX_LL_CHANNELS_STEREO,      /* channelMode */
    0,    /* bidirect_link */
    0,    /* has_new_caps */
    BTAV_A2DP_CODEC_BITS_PER_SAMPLE_16 /* bits_per_sample */
};

static const tA2DP_DECODER_INTERFACE a2dp_decoder_interface_aptx_ll = {
    a2dp_aptx_decoder_init,
    a2dp_aptx_decoder_cleanup,
    a2dp_aptx_decoder_reset,
    a2dp_aptx_decoder_decode_packet_header,
    a2dp_aptx_decoder_decode_packet,
    NULL,  // decoder_start
    NULL,  // decoder_suspend
    a2dp_aptx_decoder_configure,
};

// Builds the aptX-LL Media Codec Capabilities byte sequence beginning from the
// LOSC octet. |media_type| is the media type |AVDT_MEDIA_TYPE_*|.
// |p_ie| is a pointer to the aptX-LL Codec Information Element information.
// The result is stored in |p_result|. Returns A2DP_SUCCESS on success,
// otherwise the corresponding A2DP error status code.
tA2DP_STATUS A2DP_BuildInfoAptxLl(uint8_t media_type,
                                         const tA2DP_APTX_LL_CIE* p_ie,
                                         uint8_t* p_result) {
  if (p_ie == NULL || p_result == NULL) {
    return A2DP_INVALID_PARAMS;
  }

  *p_result++ = A2DP_APTX_LL_CODEC_LEN;
  *p_result++ = (media_type << 4);
  *p_result++ = A2DP_MEDIA_CT_NON_A2DP;
  *p_result++ = (uint8_t)(p_ie->vendorId & 0x000000FF);
  *p_result++ = (uint8_t)((p_ie->vendorId & 0x0000FF00) >> 8);
  *p_result++ = (uint8_t)((p_ie->vendorId & 0x00FF0000) >> 16);
  *p_result++ = (uint8_t)((p_ie->vendorId & 0xFF000000) >> 24);
  *p_result++ = (uint8_t)(p_ie->codecId & 0x00FF);
  *p_result++ = (uint8_t)((p_ie->codecId & 0xFF00) >> 8);
  *p_result++ = p_ie->sampleRate | p_ie->channelMode;
  *p_result++ = ((p_ie->has_new_caps & 0x1) << 1) |
                ((p_ie->bidirect_link & 0x1) << 0);

  return A2DP_SUCCESS;
}

// Parses the aptX-LL Media Codec Capabilities byte sequence beginning from the
// LOSC octet. The result is stored in |p_ie|. The byte sequence to parse is
// |p_codec_info|. If |is_capability| is true, the byte sequence is
// codec capabilities, otherwise is codec configuration.
// Returns A2DP_SUCCESS on success, otherwise the corresponding A2DP error
// status code.
tA2DP_STATUS A2DP_ParseInfoAptxLl(tA2DP_APTX_LL_CIE* p_ie,
                                         const uint8_t* p_codec_info,
                                         bool is_capability) {
  uint8_t losc;
  uint8_t media_type;
  tA2DP_CODEC_TYPE codec_type;

  if (p_ie == NULL || p_codec_info == NULL) return A2DP_INVALID_PARAMS;

  // Check the codec capability length
  losc = *p_codec_info++;
  if (losc != A2DP_APTX_LL_CODEC_LEN) return A2DP_WRONG_CODEC;

  media_type = (*p_codec_info++) >> 4;
  codec_type = *p_codec_info++;
  /* Check the Media Type and Media Codec Type */
  if (media_type != AVDT_MEDIA_TYPE_AUDIO ||
      codec_type != A2DP_MEDIA_CT_NON_A2DP) {
    return A2DP_WRONG_CODEC;
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
  if (p_ie->vendorId != A2DP_APTX_LL_VENDOR_ID ||
      p_ie->codecId != A2DP_APTX_LL_CODEC_ID_BLUETOOTH) {
    return A2DP_WRONG_CODEC;
  }

  p_ie->channelMode = *p_codec_info & 0x0F;
  p_ie->sampleRate = *p_codec_info & 0xF0;
  p_codec_info++;

  p_ie->has_new_caps = (*p_codec_info >> 1) & 0x01;
  p_ie->bidirect_link = (*p_codec_info >> 0) & 0x01;
  p_codec_info++;

  if (is_capability) {
    // NOTE: The checks here are very liberal. We should be using more
    // pedantic checks specific to the SRC or SNK as specified in the spec.
    if (A2DP_BitsSet(p_ie->sampleRate) == A2DP_SET_ZERO_BIT)
      return A2DP_BAD_SAMP_FREQ;
    if (A2DP_BitsSet(p_ie->channelMode) == A2DP_SET_ZERO_BIT)
      return A2DP_BAD_CH_MODE;

    return A2DP_SUCCESS;
  }

  if (A2DP_BitsSet(p_ie->sampleRate) != A2DP_SET_ONE_BIT)
    return A2DP_BAD_SAMP_FREQ;
  if (A2DP_BitsSet(p_ie->channelMode) != A2DP_SET_ONE_BIT)
    return A2DP_BAD_CH_MODE;

  return A2DP_SUCCESS;
}

tA2D_STATUS A2DP_IsVendorPeerSourceCodecValidAptxLl(const uint8_t* p_codec_info) {
  tA2DP_APTX_LL_CIE cfg_cie;

  /* Use a liberal check when parsing the codec info */
  tA2D_STATUS status;
  status = A2DP_ParseInfoAptxLl(&cfg_cie, p_codec_info, true);
  if (status == A2D_SUCCESS) {
    return status;
  }

  return A2DP_ParseInfoAptxLl(&cfg_cie, p_codec_info, false);
}

bool A2DP_IsVendorPeerSinkCodecValidAptxLl(const uint8_t* p_codec_info) {
  tA2DP_APTX_LL_CIE cfg_cie;

  /* Use a liberal check when parsing the codec info */
  return (A2DP_ParseInfoAptxLl(&cfg_cie, p_codec_info, false) ==
          A2DP_SUCCESS) ||
         (A2DP_ParseInfoAptxLl(&cfg_cie, p_codec_info, true) == A2DP_SUCCESS);
}

// Checks whether A2DP aptX-LL codec configuration matches with a device's
// codec capabilities. |p_cap| is the aptX-LL codec configuration.
// |p_codec_info| is the device's codec capabilities.
// If |is_capability| is true, the byte sequence is codec capabilities,
// otherwise is codec configuration.
// |p_codec_info| contains the codec capabilities for a peer device that
// is acting as an A2DP source.
// Returns A2DP_SUCCESS if the codec configuration matches with capabilities,
// otherwise the corresponding A2DP error status code.
tA2DP_STATUS A2DP_CodecInfoMatchesCapabilityAptxLl(
    const tA2DP_APTX_LL_CIE* p_cap, const uint8_t* p_codec_info,
    bool is_capability) {
  tA2DP_STATUS status;
  tA2DP_APTX_LL_CIE cfg_cie;

  /* parse configuration */
  status = A2DP_ParseInfoAptxLl(&cfg_cie, p_codec_info, is_capability);
  if (status != A2DP_SUCCESS) {
    LOG_ERROR("%s: parsing failed %d", __func__, status);
    return status;
  }

  /* verify that each parameter is in range */

  LOG_VERBOSE("%s: FREQ peer: 0x%x, capability 0x%x", __func__,
              cfg_cie.sampleRate, p_cap->sampleRate);
  LOG_VERBOSE("%s: CH_MODE peer: 0x%x, capability 0x%x", __func__,
              cfg_cie.channelMode, p_cap->channelMode);

  /* sampling frequency */
  if ((cfg_cie.sampleRate & p_cap->sampleRate) == 0) return A2DP_NS_SAMP_FREQ;

  /* channel mode */
  if ((cfg_cie.channelMode & p_cap->channelMode) == 0) return A2DP_NS_CH_MODE;

  return A2DP_SUCCESS;
}

btav_a2dp_codec_index_t A2DP_VendorSinkCodecIndexAptxLl(
    const uint8_t* p_codec_info) {
  return BTAV_A2DP_CODEC_INDEX_SINK_APTX_LL;
}

btav_a2dp_codec_index_t A2DP_VendorSourceCodecIndexAptxLl(
    const uint8_t* p_codec_info) {
  return BTAV_A2DP_CODEC_INDEX_SOURCE_APTX_LL;
}

const char* A2DP_VendorCodecNameAptxLl(
    UNUSED_ATTR const uint8_t* p_codec_info) {
  return "aptX-LL";
}

bool A2DP_VendorCodecTypeEqualsAptxLl(const uint8_t* p_codec_info_a,
                                      const uint8_t* p_codec_info_b) {
  tA2DP_APTX_LL_CIE aptx_ll_cie_a;
  tA2DP_APTX_LL_CIE aptx_ll_cie_b;

  // Check whether the codec info contains valid data
  tA2DP_STATUS a2dp_status =
      A2DP_ParseInfoAptxLl(&aptx_ll_cie_a, p_codec_info_a, true);
  if (a2dp_status != A2DP_SUCCESS) {
    if (a2dp_status != A2D_WRONG_CODEC)
      LOG_ERROR("%s: cannot decode codec information: %d", __func__, a2dp_status);
    return false;
  }
  a2dp_status = A2DP_ParseInfoAptxLl(&aptx_ll_cie_b, p_codec_info_b, true);
  if (a2dp_status != A2DP_SUCCESS) {
    if (a2dp_status != A2D_WRONG_CODEC)
      LOG_ERROR("%s: cannot decode codec information: %d", __func__, a2dp_status);
    return false;
  }

  return true;
}

bool A2DP_VendorInitCodecConfigAptxLl(btav_a2dp_codec_index_t codec_index, UINT8 *p_result) {
  switch(codec_index) {
    case BTAV_A2DP_CODEC_INDEX_SINK_APTX_LL:
      return A2DP_VendorInitCodecConfigAptxLlSink(p_result);
    default:
      break;
  }
  return false;
}

bool A2DP_VendorInitCodecConfigAptxLlSink(uint8_t* p_codec_info) {
  tA2D_STATUS sts = A2D_FAIL;
  sts = A2DP_BuildInfoAptxLl(A2D_MEDIA_TYPE_AUDIO, &a2dp_aptx_ll_sink_caps, p_codec_info);
  return sts == A2D_SUCCESS;
}

bool A2DP_VendorBuildCodecConfigAptxLl(UINT8 *p_src_cap, UINT8 *p_result) {
  tA2DP_APTX_LL_CIE src_cap;
  tA2DP_APTX_LL_CIE pref_cap = a2dp_aptx_ll_default_config;
  tA2D_STATUS status;

  if ((status = A2DP_ParseInfoAptxLl(&src_cap, p_src_cap, TRUE)) != 0) {
    APPL_TRACE_ERROR("%s: Cant parse src cap ret = %d", __func__, status);
    return false;
  }

  if (src_cap.sampleRate & A2DP_APTX_LL_SAMPLERATE_48000) {
    pref_cap.sampleRate = A2DP_APTX_LL_SAMPLERATE_48000;
  } else if (src_cap.sampleRate & A2DP_APTX_LL_SAMPLERATE_44100) {
    pref_cap.sampleRate = A2DP_APTX_LL_SAMPLERATE_44100;
  } else {
    APPL_TRACE_ERROR("%s: Unsupported sample rate 0x%x", __func__,
                     src_cap.sampleRate);
    return false;
  }

  if (src_cap.channelMode & A2DP_APTX_LL_CHANNELS_STEREO) {
    pref_cap.channelMode = A2DP_APTX_LL_CHANNELS_STEREO;
  } else if (src_cap.channelMode & A2DP_APTX_LL_CHANNELS_MONO) {
    pref_cap.channelMode = A2DP_APTX_LL_CHANNELS_MONO;
  } else {
    APPL_TRACE_ERROR("%s: Unsupported channel mode 0x%x", __func__,
                     src_cap.channelMode);
    return false;
  }

  A2DP_BuildInfoAptxLl(A2D_MEDIA_TYPE_AUDIO, (tA2DP_APTX_LL_CIE *) &pref_cap, p_result);
  return true;
}

const tA2DP_DECODER_INTERFACE* A2DP_GetVendorDecoderInterfaceAptxLl(
    const uint8_t* p_codec_info) {
  if (!A2DP_IsVendorPeerSinkCodecValidAptxLl(p_codec_info)) return NULL;

  return &a2dp_decoder_interface_aptx_ll;
}

#endif /* #if (defined(A2D_INCLUDED) && A2D_INCLUDED == TRUE) */
