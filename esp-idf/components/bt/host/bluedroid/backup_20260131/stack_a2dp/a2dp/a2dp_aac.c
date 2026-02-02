/*
 * SPDX-FileCopyrightText: 2016 The Android Open Source Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/******************************************************************************
 *
 *  Utility functions to help build and parse the AAC Codec Information
 *  Element and Media Payload.
 *
 ******************************************************************************/

#include "common/bt_target.h"

#include <string.h>
#include "stack/a2d_api.h"
#include "a2d_int.h"
#include "stack/a2dp_aac.h"
#include "stack/a2dp_aac_decoder.h"
#include "stack/a2dp_codec_api.h"
#include "common/bt_defs.h"

#define A2DP_AAC_DEFAULT_BITRATE 320000  // 320 kbps
#define A2DP_AAC_MIN_BITRATE 64000       // 64 kbps

/* AAC Source codec capabilities */
static const tA2DP_AAC_CIE a2dp_aac_cbr_source_caps = {
    // objectType
    A2DP_AAC_OBJECT_TYPE_MPEG2_LC | A2DP_AAC_OBJECT_TYPE_MPEG2_LC,
    // sampleRate
    // TODO: AAC 48.0kHz sampling rate should be added back - see b/62301376
    A2DP_AAC_SAMPLING_FREQ_44100,
    // channelMode
    A2DP_AAC_CHANNEL_MODE_STEREO,
    // variableBitRateSupport
    A2DP_AAC_VARIABLE_BIT_RATE_DISABLED,
    // bitRate
    A2DP_AAC_DEFAULT_BITRATE,
    // bits_per_sample
    BTAV_A2DP_CODEC_BITS_PER_SAMPLE_16};

/* AAC Sink codec capabilities */
static const tA2DP_AAC_CIE a2dp_aac_sink_caps = {
    // objectType
    A2DP_AAC_OBJECT_TYPE_MPEG2_LC | A2DP_AAC_OBJECT_TYPE_MPEG2_LC,
    // sampleRate
    A2DP_AAC_SAMPLING_FREQ_44100 | A2DP_AAC_SAMPLING_FREQ_48000,
    // channelMode
    A2DP_AAC_CHANNEL_MODE_MONO | A2DP_AAC_CHANNEL_MODE_STEREO,
    // variableBitRateSupport
    A2DP_AAC_VARIABLE_BIT_RATE_ENABLED,
    // bitRate
    A2DP_AAC_DEFAULT_BITRATE,
    // bits_per_sample
    BTAV_A2DP_CODEC_BITS_PER_SAMPLE_16};

static const tA2DP_DECODER_INTERFACE a2dp_decoder_interface_aac = {
    a2dp_aac_decoder_init,
    a2dp_aac_decoder_cleanup,
    NULL,  // decoder_reset
    a2dp_aac_decoder_decode_packet_header,
    a2dp_aac_decoder_decode_packet,
    NULL,  // decoder_start
    NULL,  // decoder_suspend
    a2dp_aac_decoder_configure,
};

static tA2D_STATUS A2DP_CodecInfoMatchesCapabilityAac(
    const tA2DP_AAC_CIE* p_cap, const uint8_t* p_codec_info,
    bool is_capability);

// Builds the AAC Media Codec Capabilities byte sequence beginning from the
// LOSC octet. |media_type| is the media type |AVDT_MEDIA_TYPE_*|.
// |p_ie| is a pointer to the AAC Codec Information Element information.
// The result is stored in |p_result|. Returns A2D_SUCCESS on success,
// otherwise the corresponding A2DP error status code.
tA2D_STATUS A2DP_BuildInfoAac(uint8_t media_type,
                                      const tA2DP_AAC_CIE* p_ie,
                                      uint8_t* p_result) {
  if (p_ie == NULL || p_result == NULL) {
    return A2D_INVALID_PARAMS;
  }

  *p_result++ = A2DP_AAC_CODEC_LEN;
  *p_result++ = (media_type << 4);
  *p_result++ = A2D_MEDIA_CT_M24;

  // Object Type
  if (p_ie->objectType == 0) return A2D_INVALID_PARAMS;
  *p_result++ = p_ie->objectType;

  // Sampling Frequency
  if (p_ie->sampleRate == 0) return A2D_INVALID_PARAMS;
  *p_result++ = (uint8_t)(p_ie->sampleRate & A2DP_AAC_SAMPLING_FREQ_MASK0);
  *p_result = (uint8_t)((p_ie->sampleRate & A2DP_AAC_SAMPLING_FREQ_MASK1) >> 8);

  // Channel Mode
  if (p_ie->channelMode == 0) return A2D_INVALID_PARAMS;
  *p_result++ |= (p_ie->channelMode & A2DP_AAC_CHANNEL_MODE_MASK);

  // Variable Bit Rate Support
  *p_result = (p_ie->variableBitRateSupport & A2DP_AAC_VARIABLE_BIT_RATE_MASK);

  // Bit Rate
  *p_result++ |= (uint8_t)((p_ie->bitRate & A2DP_AAC_BIT_RATE_MASK0) >> 16);
  *p_result++ = (uint8_t)((p_ie->bitRate & A2DP_AAC_BIT_RATE_MASK1) >> 8);
  *p_result++ = (uint8_t)(p_ie->bitRate & A2DP_AAC_BIT_RATE_MASK2);

  return A2D_SUCCESS;
}

// Parses the AAC Media Codec Capabilities byte sequence beginning from the
// LOSC octet. The result is stored in |p_ie|. The byte sequence to parse is
// |p_codec_info|. If |is_capability| is true, the byte sequence is
// codec capabilities, otherwise is codec configuration.
// Returns A2D_SUCCESS on success, otherwise the corresponding A2DP error
// status code.
tA2D_STATUS A2DP_ParseInfoAac(tA2DP_AAC_CIE* p_ie,
                                      const uint8_t* p_codec_info,
                                      bool is_capability) {
  uint8_t losc;
  uint8_t media_type;
  tA2D_CODEC_TYPE codec_type;

  if (p_ie == NULL || p_codec_info == NULL) return A2D_INVALID_PARAMS;

  // Check the codec capability length
  losc = *p_codec_info++;
  if (losc != A2DP_AAC_CODEC_LEN) return A2D_WRONG_CODEC;

  media_type = (*p_codec_info++) >> 4;
  codec_type = *p_codec_info++;
  /* Check the Media Type and Media Codec Type */
  if (media_type != A2D_MEDIA_TYPE_AUDIO || codec_type != A2D_MEDIA_CT_M24) {
    return A2D_WRONG_CODEC;
  }

  p_ie->objectType = *p_codec_info++;

  p_ie->sampleRate = ((uint16_t)*p_codec_info & A2DP_AAC_SAMPLING_FREQ_MASK0) |
                      ((uint16_t)*(p_codec_info + 1) << 8 & A2DP_AAC_SAMPLING_FREQ_MASK1);

  p_codec_info++;
  p_ie->channelMode = *p_codec_info & A2DP_AAC_CHANNEL_MODE_MASK;


  p_codec_info++;
  p_ie->variableBitRateSupport =
      *p_codec_info & A2DP_AAC_VARIABLE_BIT_RATE_MASK;

  p_ie->bitRate = ((*p_codec_info) << 16 & A2DP_AAC_BIT_RATE_MASK0) |
                  (*(p_codec_info + 1) << 8 & A2DP_AAC_BIT_RATE_MASK1) |
                  (*(p_codec_info + 2) & A2DP_AAC_BIT_RATE_MASK2);
  p_codec_info += 3;

  if (is_capability) {
    // NOTE: The checks here are very liberal. We should be using more
    // pedantic checks specific to the SRC or SNK as specified in the spec.
    if (A2D_BitsSet(p_ie->objectType) == A2D_SET_ZERO_BIT)
      return A2D_BAD_OBJ_TYPE;
    if (A2D_BitsSet(p_ie->sampleRate) == A2D_SET_ZERO_BIT)
      return A2D_BAD_SAMP_FREQ;
    if (A2D_BitsSet(p_ie->channelMode) == A2D_SET_ZERO_BIT)
      return A2D_BAD_CH_MODE;

    return A2D_SUCCESS;
  }

  if (A2D_BitsSet(p_ie->objectType) != A2D_SET_ONE_BIT)
    return A2D_BAD_OBJ_TYPE;
  if (A2D_BitsSet(p_ie->sampleRate) != A2D_SET_ONE_BIT)
    return A2D_BAD_SAMP_FREQ;
  if (A2D_BitsSet(p_ie->channelMode) != A2D_SET_ONE_BIT)
    return A2D_BAD_CH_MODE;

  return A2D_SUCCESS;
}

// Checks whether A2DP AAC codec configuration matches with a device's codec
// capabilities. |p_cap| is the AAC codec configuration. |p_codec_info| is
// the device's codec capabilities. |is_capability| is true if
// |p_codec_info| contains A2DP codec capability.
// Returns A2DP_SUCCESS if the codec configuration matches with capabilities,
// otherwise the corresponding A2DP error status code.
static tA2D_STATUS A2DP_CodecInfoMatchesCapabilityAac(
    const tA2DP_AAC_CIE* p_cap, const uint8_t* p_codec_info,
    bool is_capability) {
  tA2D_STATUS status;
  tA2DP_AAC_CIE cfg_cie;

  /* parse configuration */
  status = A2DP_ParseInfoAac(&cfg_cie, p_codec_info, is_capability);
  if (status != A2D_SUCCESS) {
    LOG_ERROR("%s: parsing failed %d", __func__, status);
    return status;
  }

  /* verify that each parameter is in range */

  LOG_VERBOSE("%s: Object Type peer: 0x%x, capability 0x%x", __func__,
              cfg_cie.objectType, p_cap->objectType);
  LOG_VERBOSE("%s: Sample Rate peer: %u, capability %u", __func__,
              cfg_cie.sampleRate, p_cap->sampleRate);
  LOG_VERBOSE("%s: Channel Mode peer: 0x%x, capability 0x%x", __func__,
              cfg_cie.channelMode, p_cap->channelMode);
  LOG_VERBOSE("%s: Variable Bit Rate Support peer: 0x%x, capability 0x%x",
              __func__, cfg_cie.variableBitRateSupport,
              p_cap->variableBitRateSupport);
  LOG_VERBOSE("%s: Bit Rate peer: %u, capability %u", __func__, cfg_cie.bitRate,
              p_cap->bitRate);

  /* Object Type */
  if ((cfg_cie.objectType & p_cap->objectType) == 0) return A2D_BAD_OBJ_TYPE;

  /* Sample Rate */
  if ((cfg_cie.sampleRate & p_cap->sampleRate) == 0) return A2D_BAD_SAMP_FREQ;

  /* Channel Mode */
  if ((cfg_cie.channelMode & p_cap->channelMode) == 0) return A2D_NS_CH_MODE;

  return A2D_SUCCESS;
}

const char* A2DP_CodecNameAac(const uint8_t* p_codec_info) {
  UNUSED(p_codec_info);
  return "AAC";
}

bool A2DP_IsPeerSinkCodecValidAac(const uint8_t* p_codec_info) {
  tA2DP_AAC_CIE cfg_cie;

  /* Use a liberal check when parsing the codec info */
  return (A2DP_ParseInfoAac(&cfg_cie, (UINT8 *)p_codec_info, false) == A2D_SUCCESS) ||
         (A2DP_ParseInfoAac(&cfg_cie, (UINT8 *)p_codec_info, true) == A2D_SUCCESS);
}

bool A2DP_CodecTypeEqualsAac(const uint8_t* p_codec_info_a,
                             const uint8_t* p_codec_info_b) {
  tA2DP_AAC_CIE aac_cie_a;
  tA2DP_AAC_CIE aac_cie_b;

  // Check whether the codec info contains valid data
  tA2D_STATUS a2dp_status =
      A2DP_ParseInfoAac(&aac_cie_a, p_codec_info_a, true);
  if (a2dp_status != A2D_SUCCESS) {
    LOG_ERROR("%s: cannot decode codec information: %d", __func__, a2dp_status);
    return false;
  }
  a2dp_status = A2DP_ParseInfoAac(&aac_cie_b, p_codec_info_b, true);
  if (a2dp_status != A2D_SUCCESS) {
    LOG_ERROR("%s: cannot decode codec information: %d", __func__, a2dp_status);
    return false;
  }

  return true;
}

bool A2DP_IsSinkCodecValidAac(const uint8_t* p_codec_info) {
  UNUSED(p_codec_info);
  tA2DP_AAC_CIE cfg_cie;

  /* Use a liberal check when parsing the codec info */
  return (A2DP_ParseInfoAac(&cfg_cie, p_codec_info, false) == A2D_SUCCESS) ||
         (A2DP_ParseInfoAac(&cfg_cie, p_codec_info, true) == A2D_SUCCESS);
}

bool A2DP_IsPeerSourceCodecValidAac(const uint8_t* p_codec_info) {
  UNUSED(p_codec_info);
  tA2DP_AAC_CIE cfg_cie;

  /* Use a liberal check when parsing the codec info */
  return (A2DP_ParseInfoAac(&cfg_cie, p_codec_info, false) == A2D_SUCCESS) ||
         (A2DP_ParseInfoAac(&cfg_cie, p_codec_info, true) == A2D_SUCCESS);
}

tA2D_STATUS A2DP_IsSinkCodecSupportedAac(const uint8_t* p_codec_info) {
  return A2DP_CodecInfoMatchesCapabilityAac(&a2dp_aac_sink_caps, p_codec_info,
                                            false);
}

tA2D_STATUS A2DP_IsPeerSourceCodecSupportedAac(const uint8_t* p_codec_info) {
  return A2DP_CodecInfoMatchesCapabilityAac(&a2dp_aac_sink_caps, p_codec_info,
                                            true);
}

btav_a2dp_codec_index_t A2DP_SinkCodecIndexAac(const uint8_t* p_codec_info) {
  UNUSED(p_codec_info);
  return BTAV_A2DP_CODEC_INDEX_SINK_AAC;
}

btav_a2dp_codec_index_t A2DP_SourceCodecIndexAac(const uint8_t* p_codec_info) {
  UNUSED(p_codec_info);
  return BTAV_A2DP_CODEC_INDEX_SOURCE_AAC;
}

const tA2DP_DECODER_INTERFACE* A2DP_GetDecoderInterfaceAac(
    const uint8_t* p_codec_info) {
  if (!A2DP_IsSinkCodecValidAac(p_codec_info)) return NULL;

  return &a2dp_decoder_interface_aac;
}

bool A2DP_InitCodecConfigAac(btav_a2dp_codec_index_t codec_index, UINT8 *p_result) {
  switch(codec_index) {
    case BTAV_A2DP_CODEC_INDEX_SINK_AAC:
      return A2DP_InitCodecConfigAacSink(p_result);
    default:
      break;
  }

// #if (BTA_AV_CO_CP_SCMS_T == TRUE)
//   /* Content protection info - support SCMS-T */
//   uint8_t* p = p_cfg->protect_info;
//   *p++ = AVDT_CP_LOSC;
//   UINT16_TO_STREAM(p, AVDT_CP_SCMS_T_ID);
//   p_cfg->num_protect = 1;
// #endif

  return false;
}

bool A2DP_InitCodecConfigAacSink(uint8_t* p_codec_info) {
  return A2DP_BuildInfoAac(A2D_MEDIA_TYPE_AUDIO, &a2dp_aac_sink_caps,
                           p_codec_info) == A2D_SUCCESS;
}

bool A2DP_BuildCodecConfigAac(UINT8 *p_src_cap, UINT8 *p_result) {
  UNUSED(p_src_cap);
  tA2D_STATUS sts = A2D_FAIL;
  sts = A2DP_BuildInfoAac(A2D_MEDIA_TYPE_AUDIO, &a2dp_aac_sink_caps, p_result);
  return sts == A2D_SUCCESS;
}
