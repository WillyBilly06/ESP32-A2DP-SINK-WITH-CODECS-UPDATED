/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include "a2d_int.h"
#include "common/bt_defs.h"
#include "common/bt_target.h"
#include "stack/a2d_api.h"
#include "stack/a2d_sbc.h"
#include "stack/a2dp_vendor_lc3plus.h"
#include "stack/a2dp_vendor_lc3plus_decoder.h"

#if (defined(LC3PLUS_DEC_INCLUDED) && LC3PLUS_DEC_INCLUDED == TRUE)

/* Lc3Plus Source codec capabilities */
static const tA2DP_LC3PLUS_CIE a2dp_lc3plus_source_caps = {
    A2DP_LC3PLUS_VENDOR_ID,  // vendorId
    A2DP_LC3PLUS_CODEC_ID,   // codecId
    A2DP_LC3PLUS_FRAME_DURATION_2_5MS |
      A2DP_LC3PLUS_FRAME_DURATION_5MS |
      A2DP_LC3PLUS_FRAME_DURATION_10MS,
    A2DP_LC3PLUS_CHANNELS_1 | A2DP_LC3PLUS_CHANNELS_2,
    A2DP_LC3PLUS_SAMPLING_FREQ_48000 | A2DP_LC3PLUS_SAMPLING_FREQ_96000
};

/* Lc3Plus Sink codec capabilities */
static const tA2DP_LC3PLUS_CIE a2dp_lc3plus_sink_caps = {
    A2DP_LC3PLUS_VENDOR_ID,  // vendorId
    A2DP_LC3PLUS_CODEC_ID,   // codecId
    A2DP_LC3PLUS_FRAME_DURATION_2_5MS |
      A2DP_LC3PLUS_FRAME_DURATION_5MS |
      A2DP_LC3PLUS_FRAME_DURATION_10MS,
    A2DP_LC3PLUS_CHANNELS_1 | A2DP_LC3PLUS_CHANNELS_2,
    A2DP_LC3PLUS_SAMPLING_FREQ_48000 | A2DP_LC3PLUS_SAMPLING_FREQ_96000
};

/* Default Lc3Plus codec configuration */
static const tA2DP_LC3PLUS_CIE a2dp_lc3plus_default_config = {
    A2DP_LC3PLUS_VENDOR_ID,  // vendorId
    A2DP_LC3PLUS_CODEC_ID,   // codecId
    A2DP_LC3PLUS_FRAME_DURATION_2_5MS |
      A2DP_LC3PLUS_FRAME_DURATION_5MS |
      A2DP_LC3PLUS_FRAME_DURATION_10MS,
    A2DP_LC3PLUS_CHANNELS_1 | A2DP_LC3PLUS_CHANNELS_2,
    A2DP_LC3PLUS_SAMPLING_FREQ_48000 | A2DP_LC3PLUS_SAMPLING_FREQ_96000
};

static const tA2DP_DECODER_INTERFACE a2dp_decoder_interface_lc3plus = {
    a2dp_lc3plus_decoder_init,
    a2dp_lc3plus_decoder_cleanup,
    NULL,  // decoder_reset,
    a2dp_lc3plus_decoder_decode_packet_header,
    a2dp_lc3plus_decoder_decode_packet,
    NULL,  // decoder_start
    NULL,  // decoder_suspend
    a2dp_lc3plus_decoder_configure,
};

tA2D_STATUS A2DP_BuildInfoLc3Plus(uint8_t media_type,
                                       const tA2DP_LC3PLUS_CIE* p_ie,
                                       uint8_t* p_result) {
  if (p_ie == NULL || p_result == NULL) {
    return A2D_INVALID_PARAMS;
  }

  *p_result++ = A2DP_LC3PLUS_CODEC_LEN;
  *p_result++ = (media_type << 4);
  *p_result++ = A2D_MEDIA_CT_NON_A2DP;

  // Vendor ID and Codec ID
  *p_result++ = (uint8_t)(p_ie->vendorId & 0x000000FF);
  *p_result++ = (uint8_t)((p_ie->vendorId & 0x0000FF00) >> 8);
  *p_result++ = (uint8_t)((p_ie->vendorId & 0x00FF0000) >> 16);
  *p_result++ = (uint8_t)((p_ie->vendorId & 0xFF000000) >> 24);
  *p_result++ = (uint8_t)(p_ie->codecId & 0x00FF);
  *p_result++ = (uint8_t)((p_ie->codecId & 0xFF00) >> 8);

  *p_result++ = (uint8_t)(p_ie->frame_duration);
  *p_result++ = (uint8_t)(p_ie->channels);
  *p_result++ = (uint8_t)((p_ie->frequency & 0xFF00) >> 8);
  *p_result++ = (uint8_t)(p_ie->frequency & 0x00FF);

  return A2D_SUCCESS;
}

tA2D_STATUS A2DP_ParseInfoLc3Plus(tA2DP_LC3PLUS_CIE* p_ie,
                                       const uint8_t* p_codec_info,
                                       bool is_capability) {
  uint8_t losc;
  uint8_t media_type;
  tA2D_CODEC_TYPE codec_type;

  if (p_ie == NULL || p_codec_info == NULL) return A2D_INVALID_PARAMS;

  // Check the codec capability length
  losc = *p_codec_info++;
  if (losc != A2DP_LC3PLUS_CODEC_LEN) {
    return A2D_WRONG_CODEC;
  }

  media_type = (*p_codec_info++) >> 4;
  codec_type = *p_codec_info++;
  /* Check the Media Type and Media Codec Type */
  if (media_type != A2D_MEDIA_TYPE_AUDIO ||
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

  if (p_ie->vendorId != A2DP_LC3PLUS_VENDOR_ID ||
      p_ie->codecId != A2DP_LC3PLUS_CODEC_ID) {
    return A2D_WRONG_CODEC;
  }

  p_ie->frame_duration = *p_codec_info++;
  p_ie->channels = *p_codec_info++;
  p_ie->frequency =
      (*(p_codec_info + 1) >> 8 & 0x00FF) | (*p_codec_info << 8 & 0xFF00);
  p_codec_info += 2;
  
  if (is_capability) {
    // NOTE: The checks here are very liberal. We should be using more
    // pedantic checks specific to the SRC or SNK as specified in the spec.
      if (A2D_BitsSet(p_ie->frame_duration) == A2D_SET_ZERO_BIT)
      return A2D_FAIL;
    if (A2D_BitsSet(p_ie->channels) == A2D_SET_ZERO_BIT)
      return A2D_NS_CH_MODE;
    if (A2D_BitsSet(p_ie->frequency) == A2D_SET_ZERO_BIT)
      return A2D_NS_SAMP_FREQ;

    return A2D_SUCCESS;
  }

  return A2D_SUCCESS;
}

tA2D_STATUS A2DP_IsVendorPeerSourceCodecValidLc3Plus(const uint8_t* p_codec_info) {
  tA2DP_LC3PLUS_CIE cfg_cie;

  /* Use a liberal check when parsing the codec info */
  tA2D_STATUS status;
  status = A2DP_ParseInfoLc3Plus(&cfg_cie, p_codec_info, true);
  if (status == A2D_SUCCESS) {
    return status;
  }

  return A2DP_ParseInfoLc3Plus(&cfg_cie, p_codec_info, false);
}

bool A2DP_IsVendorPeerSinkCodecValidLc3Plus(const uint8_t* p_codec_info) {
  tA2DP_LC3PLUS_CIE cfg_cie;

  /* Use a liberal check when parsing the codec info */
  return (A2DP_ParseInfoLc3Plus(&cfg_cie, p_codec_info, false) == A2D_SUCCESS) ||
         (A2DP_ParseInfoLc3Plus(&cfg_cie, p_codec_info, true) == A2D_SUCCESS);
}

btav_a2dp_codec_index_t A2DP_VendorSinkCodecIndexLc3Plus(
    const uint8_t* p_codec_info) {
  UNUSED(p_codec_info);
  return BTAV_A2DP_CODEC_INDEX_SINK_LC3PLUS;
}

btav_a2dp_codec_index_t A2DP_VendorSourceCodecIndexLc3Plus(
    const uint8_t* p_codec_info) {
  UNUSED(p_codec_info);
  return BTAV_A2DP_CODEC_INDEX_SOURCE_LC3PLUS;
}

const char* A2DP_VendorCodecNameLc3Plus(const uint8_t* p_codec_info) {
  UNUSED(p_codec_info);
  return "Lc3Plus";
}

bool A2DP_VendorCodecTypeEqualsLc3Plus(const uint8_t* p_codec_info_a,
                                    const uint8_t* p_codec_info_b) {
  tA2DP_LC3PLUS_CIE cie_a;
  tA2DP_LC3PLUS_CIE cie_b;

  // Check whether the codec info contains valid data
  tA2D_STATUS a2dp_status = A2DP_ParseInfoLc3Plus(&cie_a, p_codec_info_a, true);
  if (a2dp_status != A2D_SUCCESS) {
    if (a2dp_status != A2D_WRONG_CODEC)
      APPL_TRACE_ERROR("%s: cannot decode codec information: %d", __func__, a2dp_status);
    return false;
  }
  a2dp_status = A2DP_ParseInfoLc3Plus(&cie_b, p_codec_info_b, true);
  if (a2dp_status != A2D_SUCCESS) {
    if (a2dp_status != A2D_WRONG_CODEC)
      APPL_TRACE_ERROR("%s: cannot decode codec information: %d", __func__, a2dp_status);
    return false;
  }

  return true;
}

bool A2DP_VendorInitCodecConfigLc3Plus(btav_a2dp_codec_index_t codec_index, UINT8 *p_result) {
  switch(codec_index) {
    case BTAV_A2DP_CODEC_INDEX_SINK_LC3PLUS:
      return A2DP_VendorInitCodecConfigLc3PlusSink(p_result);
    default:
      break;
  }
  return false;
}

bool A2DP_VendorInitCodecConfigLc3PlusSink(uint8_t* p_codec_info) {
  tA2D_STATUS sts = A2D_FAIL;
  sts = A2DP_BuildInfoLc3Plus(A2D_MEDIA_TYPE_AUDIO, &a2dp_lc3plus_sink_caps,
                           p_codec_info);
  return sts == A2D_SUCCESS;
}

bool A2DP_VendorBuildCodecConfigLc3Plus(UINT8 *p_src_cap, UINT8 *p_result) {
  tA2DP_LC3PLUS_CIE src_cap;
  tA2DP_LC3PLUS_CIE pref_cap = a2dp_lc3plus_default_config;
  tA2D_STATUS status;

  if ((status = A2DP_ParseInfoLc3Plus(&src_cap, p_src_cap, TRUE)) != 0) {
    APPL_TRACE_ERROR("%s: Cant parse src cap ret = %d", __func__, status);
    return false;
  }

  pref_cap.frame_duration = src_cap.frame_duration;
  pref_cap.channels = src_cap.channels;
  pref_cap.frequency = src_cap.frequency;
  
  status = A2DP_BuildInfoLc3Plus(A2D_MEDIA_TYPE_AUDIO, (tA2DP_LC3PLUS_CIE *) &pref_cap, p_result);
  return status == A2D_SUCCESS;
}

const tA2DP_DECODER_INTERFACE* A2DP_GetVendorDecoderInterfaceLc3Plus(
    const uint8_t* p_codec_info) {
  if (!A2DP_IsVendorPeerSinkCodecValidLc3Plus(p_codec_info)) return NULL;

  return &a2dp_decoder_interface_lc3plus;
}

#endif /* defined(LC3PLUS_DEC_INCLUDED) && LC3PLUS_DEC_INCLUDED == TRUE) */
