/*
 * SPDX-FileCopyrightText: 2016 The Android Open Source Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "string.h"
#include "stack/a2d_api.h"
#include "stack/a2d_sbc.h"
#include "stack/a2dp_codec_api.h"
#include "stack/a2dp_aac.h"
#include "stack/a2dp_vendor.h"
#include "stack/a2dp_vendor_aptx_hd.h"
#include "bta/bta_av_api.h"
#include "bta/bta_av_ci.h"
#include "bta/bta_av_co.h"
#include "bta/bta_av_sbc.h"
#include "bta/bta_sys.h"
#include "btc/btc_util.h"
#include "btc_a2dp.h"
#include "btc_a2dp_source.h"
#include "btc_av_co.h"
#include "common/bt_target.h"
#include "osi/mutex.h"

#if (A2D_INCLUDED == TRUE)

tA2D_CODEC_TYPE A2DP_GetCodecType(const uint8_t* p_codec_info) {
  return (tA2D_CODEC_TYPE)(p_codec_info[AVDT_CODEC_TYPE_INDEX]);
}

tA2D_STATUS A2DP_ParseInfo(uint8_t* p_ie, const uint8_t* p_codec_info,
                           bool is_capability) {
  tA2D_CODEC_TYPE codec_type = A2DP_GetCodecType(p_codec_info);

  LOG_VERBOSE("%s: codec_type = 0x%x", __func__, codec_type);

  switch (codec_type) {
    case A2D_MEDIA_CT_SBC:
      return A2D_ParsSbcInfo((tA2D_SBC_CIE*)p_ie, (UINT8*)p_codec_info, is_capability);
#if (defined(AAC_DEC_INCLUDED) && AAC_DEC_INCLUDED == TRUE)
    case A2D_MEDIA_CT_M24:
      return A2DP_ParseInfoAac((tA2DP_AAC_CIE*)p_ie, p_codec_info, is_capability);
#endif
    case A2D_MEDIA_CT_NON_A2DP:
      return A2DP_VendorParseInfo(p_ie, p_codec_info, is_capability);
    default:
      break;
  }

  LOG_ERROR("%s: unsupported codec type 0x%x", __func__, codec_type);
  return A2D_FAIL;
}

bool A2DP_IsPeerSinkCodecValid(const uint8_t* p_codec_info) {
  tA2D_CODEC_TYPE codec_type = A2DP_GetCodecType(p_codec_info);

  LOG_VERBOSE("%s: codec_type = 0x%x", __func__, codec_type);

  switch (codec_type) {
    case A2D_MEDIA_CT_SBC:
      return A2DP_IsPeerSinkCodecValidSbc(p_codec_info);
#if (defined(AAC_DEC_INCLUDED) && AAC_DEC_INCLUDED == TRUE)
    case A2D_MEDIA_CT_M24:
      return A2DP_IsPeerSinkCodecValidAac(p_codec_info);
#endif
    case A2D_MEDIA_CT_NON_A2DP:
      return A2DP_IsVendorPeerSinkCodecValid(p_codec_info);
    default:
      break;
  }

  LOG_ERROR("%s: unsupported codec type 0x%x", __func__, codec_type);
  return false;
}

tA2D_STATUS A2DP_IsSinkCodecSupported(const uint8_t* p_codec_info) {
  tA2D_CODEC_TYPE codec_type = A2DP_GetCodecType(p_codec_info);

  LOG_VERBOSE("%s: codec_type = 0x%x", __func__, codec_type);

  switch (codec_type) {
    case A2D_MEDIA_CT_SBC:
      return A2DP_IsSinkCodecSupportedSbc(p_codec_info);
#if (defined(AAC_DEC_INCLUDED) && AAC_DEC_INCLUDED == TRUE)
    case A2D_MEDIA_CT_M24:
      return A2DP_IsSinkCodecSupportedAac(p_codec_info);
#endif
    case A2D_MEDIA_CT_NON_A2DP:
      return A2DP_IsVendorSinkCodecSupported(p_codec_info);
    default:
      break;
  }

  LOG_ERROR("%s: unsupported codec type 0x%x", __func__, codec_type);
  return A2D_FAIL;
}

tA2D_STATUS A2DP_IsPeerSourceCodecSupported(const uint8_t* p_codec_info) {
  tA2D_CODEC_TYPE codec_type = A2DP_GetCodecType(p_codec_info);

  LOG_VERBOSE("%s: codec_type = 0x%x", __func__, codec_type);

  switch (codec_type) {
    case A2D_MEDIA_CT_SBC:
      return A2DP_IsPeerSourceCodecSupportedSbc(p_codec_info);
#if (defined(AAC_DEC_INCLUDED) && AAC_DEC_INCLUDED == TRUE)
    case A2D_MEDIA_CT_M24:
      return A2DP_IsPeerSourceCodecSupportedAac(p_codec_info);
#endif
    case A2D_MEDIA_CT_NON_A2DP:
      return A2DP_IsVendorPeerSourceCodecSupported(p_codec_info);
    default:
      break;
  }

  LOG_ERROR("%s: unsupported codec type 0x%x", __func__, codec_type);
  return A2D_FAIL;
}

const tA2DP_DECODER_INTERFACE* A2DP_GetDecoderInterface(
    const uint8_t* p_codec_info) {
  tA2D_CODEC_TYPE codec_type = A2DP_GetCodecType(p_codec_info);

  switch (codec_type) {
#if (defined(SBC_DEC_INCLUDED) && SBC_DEC_INCLUDED == TRUE)
    case A2D_MEDIA_CT_SBC:
      return A2DP_GetDecoderInterfaceSbc(p_codec_info);
#endif /* #if (defined(SBC_DEC_INCLUDED) && SBC_DEC_INCLUDED == TRUE) */
#if (defined(AAC_DEC_INCLUDED) && AAC_DEC_INCLUDED == TRUE)
    case A2D_MEDIA_CT_M24:
      return A2DP_GetDecoderInterfaceAac(p_codec_info);
#endif
    case A2D_MEDIA_CT_NON_A2DP:
      return A2DP_GetVendorDecoderInterface(p_codec_info);
    default:
      break;
  }

  LOG_ERROR("%s: unsupported codec type 0x%x", __func__, codec_type);
  return NULL;
}

btav_a2dp_codec_index_t A2DP_SinkCodecIndex(const uint8_t* p_codec_info) {
  tA2D_CODEC_TYPE codec_type = A2DP_GetCodecType(p_codec_info);

  LOG_VERBOSE("%s: codec_type = 0x%x", __func__, codec_type);

  switch (codec_type) {
    case A2D_MEDIA_CT_SBC:
      return A2DP_SinkCodecIndexSbc(p_codec_info);
#if (defined(AAC_DEC_INCLUDED) && AAC_DEC_INCLUDED == TRUE)
    case A2D_MEDIA_CT_M24:
      return A2DP_SinkCodecIndexAac(p_codec_info);
#endif
    case A2D_MEDIA_CT_NON_A2DP:
      return A2DP_VendorSinkCodecIndex(p_codec_info);
    default:
      break;
  }

  LOG_ERROR("%s: unsupported codec type 0x%x", __func__, codec_type);
  return BTAV_A2DP_CODEC_INDEX_MAX;
}

btav_a2dp_codec_index_t A2DP_SourceCodecIndex(const uint8_t* p_codec_info) {
  tA2D_CODEC_TYPE codec_type = A2DP_GetCodecType(p_codec_info);

  LOG_VERBOSE("%s: codec_type = 0x%x", __func__, codec_type);

  switch (codec_type) {
    case A2D_MEDIA_CT_SBC:
      return A2DP_SourceCodecIndexSbc(p_codec_info);
#if (defined(AAC_DEC_INCLUDED) && AAC_DEC_INCLUDED == TRUE)
    case A2D_MEDIA_CT_M24:
      return A2DP_SourceCodecIndexAac(p_codec_info);
#endif
    case A2D_MEDIA_CT_NON_A2DP:
      return A2DP_VendorSourceCodecIndex(p_codec_info);
    default:
      break;
  }

  LOG_ERROR("%s: unsupported codec type 0x%x", __func__, codec_type);
  return BTAV_A2DP_CODEC_INDEX_MAX;
}

bool A2DP_InitCodecConfig(btav_a2dp_codec_index_t codec_index, UINT8 *p_result) {

  LOG_VERBOSE("%s: codec_index = %d", __func__, codec_index);

  if (A2DP_InitCodecConfigSbc(codec_index, p_result)) {
    return true;
  }

#if (defined(AAC_DEC_INCLUDED) && AAC_DEC_INCLUDED == TRUE)
  if (A2DP_InitCodecConfigAac(codec_index, p_result)) {
    return true;
  }
#endif

  if (A2DP_VendorInitCodecConfig(codec_index, p_result)) {
    return true;
  }

  LOG_ERROR("%s: unsupported codec index %u", __func__, codec_index);
  return false;
}

bool A2DP_BuildCodecConfig(UINT8 *p_src_cap, UINT8 *p_result) {
  tA2D_CODEC_TYPE codec_type = A2DP_GetCodecType(p_src_cap);

  LOG_VERBOSE("%s: codec_type = 0x%x", __func__, codec_type);

  switch (codec_type) {
    case A2D_MEDIA_CT_SBC:
      return A2DP_BuildCodecConfigSbc(p_src_cap, p_result);
#if (defined(AAC_DEC_INCLUDED) && AAC_DEC_INCLUDED == TRUE)
    case A2D_MEDIA_CT_M24:
      return A2DP_BuildCodecConfigAac(p_src_cap, p_result);
#endif
    case A2D_MEDIA_CT_NON_A2DP:
      return A2DP_VendorBuildCodecConfig(p_src_cap, p_result);
    default:
      LOG_ERROR("%s: unsupported codec type 0x%x", __func__, codec_type);
      return false;
  }
}

const char* A2DP_CodecName(const uint8_t* p_codec_info) {
  tA2D_CODEC_TYPE codec_type = A2DP_GetCodecType(p_codec_info);

  LOG_VERBOSE("%s: codec_type = 0x%x", __func__, codec_type);

  switch (codec_type) {
    case A2D_MEDIA_CT_SBC:
      return A2DP_CodecNameSbc(p_codec_info);
#if (defined(AAC_DEC_INCLUDED) && AAC_DEC_INCLUDED == TRUE)
    case A2D_MEDIA_CT_M24:
      return A2DP_CodecNameAac(p_codec_info);
#endif
    case A2D_MEDIA_CT_NON_A2DP:
      return A2DP_VendorCodecName(p_codec_info);
    default:
      break;
  }

  LOG_ERROR("%s: unsupported codec type 0x%x", __func__, codec_type);
  return "UNKNOWN CODEC";
}

bool A2DP_CodecTypeEquals(const uint8_t* p_codec_info_a,
                          const uint8_t* p_codec_info_b) {
  tA2D_CODEC_TYPE codec_type_a = A2DP_GetCodecType(p_codec_info_a);
  tA2D_CODEC_TYPE codec_type_b = A2DP_GetCodecType(p_codec_info_b);

  if (codec_type_a != codec_type_b) return false;

  switch (codec_type_a) {
    case A2D_MEDIA_CT_SBC:
      return A2DP_CodecTypeEqualsSbc(p_codec_info_a, p_codec_info_b);
#if (defined(AAC_DEC_INCLUDED) && AAC_DEC_INCLUDED == TRUE)
    case A2D_MEDIA_CT_M24:
      return A2DP_CodecTypeEqualsAac(p_codec_info_a, p_codec_info_b);
#endif
    case A2D_MEDIA_CT_NON_A2DP:
      return A2DP_VendorCodecTypeEquals(p_codec_info_a, p_codec_info_b);
    default:
      break;
  }

  LOG_ERROR("%s: unsupported codec type 0x%x", __func__, codec_type_a);
  return false;
}

#endif  ///A2D_INCLUDED
