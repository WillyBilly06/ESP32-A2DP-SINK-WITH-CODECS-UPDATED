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

/* Opus Source codec capabilities */
static const tA2DP_OPUS_CIE a2dp_opus_source_caps = {
    A2DP_OPUS_VENDOR_ID,  // vendorId
    A2DP_OPUS_CODEC_ID,   // codecId
    2,  // Channel Count
    1,  // Coupled Stream Count
    // Audio Location Configuration
    A2DP_OPUS_AUDIO_LOCATION_FL | A2DP_OPUS_AUDIO_LOCATION_FR,
    (A2DP_OPUS_FRAME_DURATION_10 |A2DP_OPUS_FRAME_DURATION_20 |
        A2DP_OPUS_FRAME_DURATION_40),
    A2DP_OPUS_FRAME_BITRATE_ALL,
    0,  // Channel Count
    0,  // Coupled Stream Count
    // Audio Location Configuration
    0x0,
    (0),
    A2DP_OPUS_FRAME_BITRATE_ALL,
};

/* Opus Sink codec capabilities */
static const tA2DP_OPUS_CIE a2dp_opus_sink_caps = {
    A2DP_OPUS_VENDOR_ID,  // vendorId
    A2DP_OPUS_CODEC_ID,   // codecId
    2,  // Channel Count
    1,  // Coupled Stream Count
    // Audio Location Configuration
    A2DP_OPUS_AUDIO_LOCATION_FL | A2DP_OPUS_AUDIO_LOCATION_FR,
    (A2DP_OPUS_FRAME_DURATION_10 |A2DP_OPUS_FRAME_DURATION_20 |
        A2DP_OPUS_FRAME_DURATION_40),
    A2DP_OPUS_FRAME_BITRATE_ALL,
    0,  // Channel Count
    0,  // Coupled Stream Count
    // Audio Location Configuration
    0x0,
    (0),
    A2DP_OPUS_FRAME_BITRATE_ALL,
};

/* Default Opus codec configuration */
static const tA2DP_OPUS_CIE a2dp_opus_default_config = {
    A2DP_OPUS_VENDOR_ID,  // vendorId
    A2DP_OPUS_CODEC_ID,   // codecId
    2,  // Channel Count
    1,  // Coupled Stream Count
    // Audio Location Configuration
    A2DP_OPUS_AUDIO_LOCATION_FL | A2DP_OPUS_AUDIO_LOCATION_FR,
    A2DP_OPUS_FRAME_DURATION_10,
    A2DP_OPUS_FRAME_BITRATE_ALL,
    0,  // Channel Count
    0,  // Coupled Stream Count
    // Audio Location Configuration
    0x0,
    (0),
    A2DP_OPUS_FRAME_BITRATE_ALL,
};

static const tA2DP_DECODER_INTERFACE a2dp_decoder_interface_opus = {
    a2dp_opus_decoder_init,
    a2dp_opus_decoder_cleanup,
    NULL,  // decoder_reset,
    a2dp_opus_decoder_decode_packet_header,
    a2dp_opus_decoder_decode_packet,
    NULL,  // decoder_start
    NULL,  // decoder_suspend
    a2dp_opus_decoder_configure,
};

tA2D_STATUS A2DP_BuildInfoOpus(uint8_t media_type,
                                       const tA2DP_OPUS_CIE* p_ie,
                                       uint8_t* p_result) {
  if (p_ie == NULL || p_result == NULL) {
    return A2D_INVALID_PARAMS;
  }

  *p_result++ = A2DP_OPUS_CODEC_LEN;
  *p_result++ = (media_type << 4);
  *p_result++ = A2D_MEDIA_CT_NON_A2DP;

  // Vendor ID and Codec ID
  *p_result++ = (uint8_t)(p_ie->vendorId & 0x000000FF);
  *p_result++ = (uint8_t)((p_ie->vendorId & 0x0000FF00) >> 8);
  *p_result++ = (uint8_t)((p_ie->vendorId & 0x00FF0000) >> 16);
  *p_result++ = (uint8_t)((p_ie->vendorId & 0xFF000000) >> 24);
  *p_result++ = (uint8_t)(p_ie->codecId & 0x00FF);
  *p_result++ = (uint8_t)((p_ie->codecId & 0xFF00) >> 8);

  *p_result++ = (uint8_t)(p_ie->channels);
  *p_result++ = (uint8_t)(p_ie->coupled_streams);
  *p_result++ = (uint8_t)(p_ie->audio_location & 0x000000FF);
  *p_result++ = (uint8_t)((p_ie->audio_location & 0x0000FF00) >> 8);
  *p_result++ = (uint8_t)((p_ie->audio_location & 0x00FF0000) >> 16);
  *p_result++ = (uint8_t)((p_ie->audio_location & 0xFF000000) >> 24);
  *p_result++ = (uint8_t)(p_ie->frame_duration);
  *p_result++ = (uint8_t)(p_ie->maximum_bitrate & 0x00FF);
  *p_result++ = (uint8_t)((p_ie->maximum_bitrate & 0xFF00) >> 8);

  *p_result++ = (uint8_t)(p_ie->bidi_channels);
  *p_result++ = (uint8_t)(p_ie->bidi_coupled_streams);
  *p_result++ = (uint8_t)(p_ie->bidi_audio_location & 0x000000FF);
  *p_result++ = (uint8_t)((p_ie->bidi_audio_location & 0x0000FF00) >> 8);
  *p_result++ = (uint8_t)((p_ie->bidi_audio_location & 0x00FF0000) >> 16);
  *p_result++ = (uint8_t)((p_ie->bidi_audio_location & 0xFF000000) >> 24);
  *p_result++ = (uint8_t)(p_ie->bidi_frame_duration);
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

  // Check the codec capability length
  losc = *p_codec_info++;
  if (losc != A2DP_OPUS_CODEC_LEN) {
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
  if (p_ie->vendorId != A2DP_OPUS_VENDOR_ID ||
      p_ie->codecId != A2DP_OPUS_CODEC_ID) {
    return A2D_WRONG_CODEC;
  }

  p_ie->channels = *p_codec_info++;
  p_ie->coupled_streams = *p_codec_info++;

  p_ie->audio_location = (*p_codec_info & 0x000000FF) |
                         (*(p_codec_info + 1) << 8 & 0x0000FF00) |
                         (*(p_codec_info + 2) << 16 & 0x00FF0000) |
                         (*(p_codec_info + 3) << 24 & 0xFF000000);
  p_codec_info += 4;

  p_ie->frame_duration = *p_codec_info++;
  p_ie->maximum_bitrate =
      (*p_codec_info & 0x00FF) | (*(p_codec_info + 1) << 8 & 0xFF00);
  p_codec_info += 2;
  
  if (is_capability) {
    // NOTE: The checks here are very liberal. We should be using more
    // pedantic checks specific to the SRC or SNK as specified in the spec.
    if (A2D_BitsSet(p_ie->channels) == A2D_SET_ZERO_BIT)
      return A2D_BAD_CH_MODE;
    if (A2D_BitsSet(p_ie->coupled_streams) == A2D_SET_ZERO_BIT)
      return A2D_BAD_CH_MODE;
    if (A2D_BitsSet(p_ie->audio_location) == A2D_SET_ZERO_BIT)
      return A2D_BAD_CH_MODE;

    return A2D_SUCCESS;
  }

  return A2D_SUCCESS;
}

tA2D_STATUS A2DP_IsVendorPeerSourceCodecValidOpus(const uint8_t* p_codec_info) {
  tA2DP_OPUS_CIE cfg_cie;

  /* Use a liberal check when parsing the codec info */
  tA2D_STATUS status;
  status = A2DP_ParseInfoOpus(&cfg_cie, p_codec_info, true);
  if (status == A2D_SUCCESS) {
    return status;
  }

  return A2DP_ParseInfoOpus(&cfg_cie, p_codec_info, false);
}

bool A2DP_IsVendorPeerSinkCodecValidOpus(const uint8_t* p_codec_info) {
  tA2DP_OPUS_CIE cfg_cie;

  /* Use a liberal check when parsing the codec info */
  return (A2DP_ParseInfoOpus(&cfg_cie, p_codec_info, false) == A2D_SUCCESS) ||
         (A2DP_ParseInfoOpus(&cfg_cie, p_codec_info, true) == A2D_SUCCESS);
}

btav_a2dp_codec_index_t A2DP_VendorSinkCodecIndexOpus(
    const uint8_t* p_codec_info) {
  UNUSED(p_codec_info);
  return BTAV_A2DP_CODEC_INDEX_SINK_OPUS;
}

btav_a2dp_codec_index_t A2DP_VendorSourceCodecIndexOpus(
    const uint8_t* p_codec_info) {
  UNUSED(p_codec_info);
  return BTAV_A2DP_CODEC_INDEX_SOURCE_OPUS;
}

const char* A2DP_VendorCodecNameOpus(const uint8_t* p_codec_info) {
  UNUSED(p_codec_info);
  return "Opus";
}

bool A2DP_VendorCodecTypeEqualsOpus(const uint8_t* p_codec_info_a,
                                    const uint8_t* p_codec_info_b) {
  tA2DP_OPUS_CIE cie_a;
  tA2DP_OPUS_CIE cie_b;

  // Check whether the codec info contains valid data
  tA2D_STATUS a2dp_status = A2DP_ParseInfoOpus(&cie_a, p_codec_info_a, true);
  if (a2dp_status != A2D_SUCCESS) {
    if (a2dp_status != A2D_WRONG_CODEC)
      APPL_TRACE_ERROR("%s: cannot decode codec information: %d", __func__, a2dp_status);
    return false;
  }
  a2dp_status = A2DP_ParseInfoOpus(&cie_b, p_codec_info_b, true);
  if (a2dp_status != A2D_SUCCESS) {
    if (a2dp_status != A2D_WRONG_CODEC)
      APPL_TRACE_ERROR("%s: cannot decode codec information: %d", __func__, a2dp_status);
    return false;
  }

  return true;
}

bool A2DP_VendorInitCodecConfigOpus(btav_a2dp_codec_index_t codec_index, UINT8 *p_result) {
  switch(codec_index) {
    case BTAV_A2DP_CODEC_INDEX_SINK_OPUS:
      return A2DP_VendorInitCodecConfigOpusSink(p_result);
    default:
      break;
  }
  return false;
}

bool A2DP_VendorInitCodecConfigOpusSink(uint8_t* p_codec_info) {
  tA2D_STATUS sts = A2D_FAIL;
  sts = A2DP_BuildInfoOpus(A2D_MEDIA_TYPE_AUDIO, &a2dp_opus_sink_caps,
                           p_codec_info);
  return sts == A2D_SUCCESS;
}

bool A2DP_VendorBuildCodecConfigOpus(UINT8 *p_src_cap, UINT8 *p_result) {
  tA2DP_OPUS_CIE src_cap;
  tA2DP_OPUS_CIE pref_cap = a2dp_opus_default_config;
  tA2D_STATUS status;

  if ((status = A2DP_ParseInfoOpus(&src_cap, p_src_cap, TRUE)) != 0) {
    APPL_TRACE_ERROR("%s: Cant parse src cap ret = %d", __func__, status);
    return false;
  }

  if (src_cap.channels < (2 * src_cap.coupled_streams)) {
    APPL_TRACE_ERROR("%s: Invalid channel configuration - channels = %d, streams = %d", __func__,
                     src_cap.channels, src_cap.coupled_streams);
    return false;
  }

  pref_cap.channels = min(src_cap.channels, pref_cap.channels);
  pref_cap.coupled_streams = min(src_cap.coupled_streams, pref_cap.coupled_streams);

  pref_cap.audio_location = 0;
  if (src_cap.audio_location & A2DP_OPUS_AUDIO_LOCATION_FL) {
    pref_cap.audio_location |= A2DP_OPUS_AUDIO_LOCATION_FL;
  }
  if (src_cap.audio_location & A2DP_OPUS_AUDIO_LOCATION_FR) {
    pref_cap.audio_location |= A2DP_OPUS_AUDIO_LOCATION_FR;
  }
  if (A2D_BitsSet(pref_cap.audio_location) == A2D_SET_ZERO_BIT) {
    APPL_TRACE_ERROR("%s: Unsupported audio locations 0x%x", __func__,
                     src_cap.audio_location);
    return false;
  }

  if (src_cap.frame_duration & pref_cap.frame_duration) {
    pref_cap.frame_duration = src_cap.frame_duration;
  } else {
    APPL_TRACE_ERROR("%s: Unsupported frame duration 0x%x", __func__,
                     src_cap.frame_duration);
    return false;
  }

  if (src_cap.maximum_bitrate > 0) {
    pref_cap.maximum_bitrate = src_cap.maximum_bitrate;
  }
  
  status = A2DP_BuildInfoOpus(A2D_MEDIA_TYPE_AUDIO, (tA2DP_OPUS_CIE *) &pref_cap, p_result);
  return status == A2D_SUCCESS;
}

const tA2DP_DECODER_INTERFACE* A2DP_GetVendorDecoderInterfaceOpus(
    const uint8_t* p_codec_info) {
  if (!A2DP_IsVendorPeerSinkCodecValidOpus(p_codec_info)) return NULL;

  return &a2dp_decoder_interface_opus;
}

#endif /* defined(OPUS_DEC_INCLUDED) && OPUS_DEC_INCLUDED == TRUE) */
