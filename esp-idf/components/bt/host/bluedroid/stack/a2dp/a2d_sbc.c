/******************************************************************************
 *
 *  Copyright (C) 2002-2012 Broadcom Corporation
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/

/******************************************************************************
 *
 *  Utility functions to help build and parse SBC Codec Information Element
 *  and Media Payload.
 *
 ******************************************************************************/

#include "common/bt_target.h"

#include <string.h>
#include "stack/a2d_api.h"
#include "a2d_int.h"
#include "stack/a2d_sbc.h"
#include "stack/a2d_sbc_decoder.h"
#include "stack/a2dp_codec_api.h"
#include "common/bt_defs.h"


#if (defined(A2D_INCLUDED) && A2D_INCLUDED == TRUE)

#define BTA_AV_CO_SBC_MAX_BITPOOL  53

#if !defined(BTC_AV_SBC_DEFAULT_SAMP_FREQ)
#define BTC_AV_SBC_DEFAULT_SAMP_FREQ A2D_SBC_IE_SAMP_FREQ_44
#endif

/* SBC SRC codec capabilities */
const tA2D_SBC_CIE a2dp_sbc_src_caps = {
    (A2D_SBC_IE_SAMP_FREQ_44), /* samp_freq */
    (A2D_SBC_IE_CH_MD_MONO | A2D_SBC_IE_CH_MD_STEREO | A2D_SBC_IE_CH_MD_JOINT | A2D_SBC_IE_CH_MD_DUAL), /* ch_mode */
    (A2D_SBC_IE_BLOCKS_16 | A2D_SBC_IE_BLOCKS_12 | A2D_SBC_IE_BLOCKS_8 | A2D_SBC_IE_BLOCKS_4), /* block_len */
    (A2D_SBC_IE_SUBBAND_4 | A2D_SBC_IE_SUBBAND_8), /* num_subbands */
    (A2D_SBC_IE_ALLOC_MD_L | A2D_SBC_IE_ALLOC_MD_S), /* alloc_mthd */
    BTA_AV_CO_SBC_MAX_BITPOOL, /* max_bitpool */
    A2D_SBC_IE_MIN_BITPOOL /* min_bitpool */
};

/* SBC SINK codec capabilities */
static const tA2D_SBC_CIE a2dp_sbc_sink_caps = {
    (A2D_SBC_IE_SAMP_FREQ_48 | A2D_SBC_IE_SAMP_FREQ_44), /* samp_freq */
    (A2D_SBC_IE_CH_MD_MONO | A2D_SBC_IE_CH_MD_STEREO | A2D_SBC_IE_CH_MD_JOINT | A2D_SBC_IE_CH_MD_DUAL), /* ch_mode */
    (A2D_SBC_IE_BLOCKS_16 | A2D_SBC_IE_BLOCKS_12 | A2D_SBC_IE_BLOCKS_8 | A2D_SBC_IE_BLOCKS_4), /* block_len */
    (A2D_SBC_IE_SUBBAND_4 | A2D_SBC_IE_SUBBAND_8), /* num_subbands */
    (A2D_SBC_IE_ALLOC_MD_L | A2D_SBC_IE_ALLOC_MD_S), /* alloc_mthd */
    A2D_SBC_IE_MAX_BITPOOL, /* max_bitpool */
    A2D_SBC_IE_MIN_BITPOOL /* min_bitpool */
};

/* Default SBC codec configuration */
const tA2D_SBC_CIE a2dp_sbc_default_config = {
    BTC_AV_SBC_DEFAULT_SAMP_FREQ,   /* samp_freq */
    A2D_SBC_IE_CH_MD_JOINT,         /* ch_mode */
    A2D_SBC_IE_BLOCKS_16,           /* block_len */
    A2D_SBC_IE_SUBBAND_8,           /* num_subbands */
    A2D_SBC_IE_ALLOC_MD_L,          /* alloc_mthd */
    BTA_AV_CO_SBC_MAX_BITPOOL,      /* max_bitpool */
    A2D_SBC_IE_MIN_BITPOOL          /* min_bitpool */
};

static const tA2DP_DECODER_INTERFACE a2dp_decoder_interface_sbc = {
    a2dp_sbc_decoder_init,
    a2dp_sbc_decoder_cleanup,
    a2dp_sbc_decoder_reset,
    a2dp_sbc_decoder_decode_packet_header,
    a2dp_sbc_decoder_decode_packet,
    NULL,  // decoder_start
    NULL,  // decoder_suspend
    a2dp_sbc_decoder_configure,
};

static tA2D_STATUS A2DP_CodecInfoMatchesCapabilitySbc(
    const tA2D_SBC_CIE* p_cap, const uint8_t* p_codec_info,
    bool is_capability);

/******************************************************************************
**
** Function         A2D_BldSbcInfo
**
** Description      This function is called by an application to build
**                  the SBC Media Codec Capabilities byte sequence
**                  beginning from the LOSC octet.
**                  Input Parameters:
**                      media_type:  Indicates Audio, or Multimedia.
**
**                      p_ie:  The SBC Codec Information Element information.
**
**                  Output Parameters:
**                      p_result:  the resulting codec info byte sequence.
**
** Returns          A2D_SUCCESS if function execution succeeded.
**                  Error status code, otherwise.
******************************************************************************/
tA2D_STATUS A2D_BldSbcInfo(UINT8 media_type, tA2D_SBC_CIE *p_ie, UINT8 *p_result)
{
    tA2D_STATUS status;

    if ( p_ie == NULL || p_result == NULL ||
            (p_ie->samp_freq & ~A2D_SBC_IE_SAMP_FREQ_MSK) ||
            (p_ie->ch_mode & ~A2D_SBC_IE_CH_MD_MSK) ||
            (p_ie->block_len & ~A2D_SBC_IE_BLOCKS_MSK) ||
            (p_ie->num_subbands & ~A2D_SBC_IE_SUBBAND_MSK) ||
            (p_ie->alloc_mthd & ~A2D_SBC_IE_ALLOC_MD_MSK) ||
            (p_ie->max_bitpool < p_ie->min_bitpool) ||
            (p_ie->max_bitpool < A2D_SBC_IE_MIN_BITPOOL) ||
            (p_ie->max_bitpool > A2D_SBC_IE_MAX_BITPOOL) ||
            (p_ie->min_bitpool < A2D_SBC_IE_MIN_BITPOOL) ||
            (p_ie->min_bitpool > A2D_SBC_IE_MAX_BITPOOL) ) {
        /* if any unused bit is set */
        status = A2D_INVALID_PARAMS;
    } else {
        status = A2D_SUCCESS;
        *p_result++ = A2D_SBC_INFO_LEN;
        *p_result++ = media_type;
        *p_result++ = A2D_MEDIA_CT_SBC;

        /* Media Codec Specific Information Element */
        *p_result++ = p_ie->samp_freq | p_ie->ch_mode;

        *p_result++ = p_ie->block_len | p_ie->num_subbands | p_ie->alloc_mthd;

        *p_result++ = p_ie->min_bitpool;
        *p_result   = p_ie->max_bitpool;
    }
    return status;
}

/******************************************************************************
**
** Function         A2D_ParsSbcInfo
**
** Description      This function is called by an application to parse
**                  the SBC Media Codec Capabilities byte sequence
**                  beginning from the LOSC octet.
**                  Input Parameters:
**                      p_info:  the byte sequence to parse.
**
**                      for_caps:  TRUE, if the byte sequence is for get capabilities response.
**
**                  Output Parameters:
**                      p_ie:  The SBC Codec Information Element information.
**
** Returns          A2D_SUCCESS if function execution succeeded.
**                  Error status code, otherwise.
******************************************************************************/
tA2D_STATUS A2D_ParsSbcInfo(tA2D_SBC_CIE *p_ie, UINT8 *p_info, BOOLEAN for_caps)
{
    tA2D_STATUS status = A2D_SUCCESS;
    UINT8   losc;

    if ( p_ie == NULL || p_info == NULL) {
        status = A2D_INVALID_PARAMS;
    } else {
        losc    = *p_info++;
        p_info++;
        /* If the function is called for the wrong Media Type or Media Codec Type */
        if (losc != A2D_SBC_INFO_LEN || *p_info != A2D_MEDIA_CT_SBC) {
            status = A2D_WRONG_CODEC;
        } else {
            p_info++;
            p_ie->samp_freq = *p_info & A2D_SBC_IE_SAMP_FREQ_MSK;
            p_ie->ch_mode   = *p_info & A2D_SBC_IE_CH_MD_MSK;
            p_info++;
            p_ie->block_len     = *p_info & A2D_SBC_IE_BLOCKS_MSK;
            p_ie->num_subbands  = *p_info & A2D_SBC_IE_SUBBAND_MSK;
            p_ie->alloc_mthd    = *p_info & A2D_SBC_IE_ALLOC_MD_MSK;
            p_info++;
            p_ie->min_bitpool = *p_info++;
            p_ie->max_bitpool = *p_info;
            if (p_ie->min_bitpool < A2D_SBC_IE_MIN_BITPOOL || p_ie->min_bitpool > A2D_SBC_IE_MAX_BITPOOL ) {
                status = A2D_BAD_MIN_BITPOOL;
            }

            if (p_ie->max_bitpool < A2D_SBC_IE_MIN_BITPOOL || p_ie->max_bitpool > A2D_SBC_IE_MAX_BITPOOL ||
                    p_ie->max_bitpool < p_ie->min_bitpool) {
                status = A2D_BAD_MAX_BITPOOL;
            }

            if (for_caps == FALSE) {
                if (A2D_BitsSet(p_ie->samp_freq) != A2D_SET_ONE_BIT) {
                    status = A2D_BAD_SAMP_FREQ;
                } else if (A2D_BitsSet(p_ie->ch_mode) != A2D_SET_ONE_BIT) {
                    status = A2D_BAD_CH_MODE;
                } else if (A2D_BitsSet(p_ie->block_len) != A2D_SET_ONE_BIT) {
                    status = A2D_BAD_BLOCK_LEN;
                } else if (A2D_BitsSet(p_ie->num_subbands) != A2D_SET_ONE_BIT) {
                    status = A2D_BAD_SUBBANDS;
                } else if (A2D_BitsSet(p_ie->alloc_mthd) != A2D_SET_ONE_BIT) {
                    status = A2D_BAD_ALLOC_MTHD;
                }
            }
        }
    }
    return status;
}

/******************************************************************************
**
** Function         A2D_BldSbcMplHdr
**
** Description      This function is called by an application to parse
**                  the SBC Media Payload header.
**                  Input Parameters:
**                      frag:  1, if fragmented. 0, otherwise.
**
**                      start:  1, if the starting packet of a fragmented frame.
**
**                      last:  1, if the last packet of a fragmented frame.
**
**                      num:  If frag is 1, this is the number of remaining fragments
**                            (including this fragment) of this frame.
**                            If frag is 0, this is the number of frames in this packet.
**
**                  Output Parameters:
**                      p_dst:  the resulting media payload header byte sequence.
**
** Returns          void.
******************************************************************************/
void A2D_BldSbcMplHdr(UINT8 *p_dst, BOOLEAN frag, BOOLEAN start, BOOLEAN last, UINT8 num)
{
    if (p_dst) {
        *p_dst  = 0;
        if (frag) {
            *p_dst  |= A2D_SBC_HDR_F_MSK;
        }
        if (start) {
            *p_dst  |= A2D_SBC_HDR_S_MSK;
        }
        if (last) {
            *p_dst  |= A2D_SBC_HDR_L_MSK;
        }
        *p_dst  |= (A2D_SBC_HDR_NUM_MSK & num);
    }
}

/******************************************************************************
**
** Function         A2D_ParsSbcMplHdr
**
** Description      This function is called by an application to parse
**                  the SBC Media Payload header.
**                  Input Parameters:
**                      p_src:  the byte sequence to parse..
**
**                  Output Parameters:
**                      frag:  1, if fragmented. 0, otherwise.
**
**                      start:  1, if the starting packet of a fragmented frame.
**
**                      last:  1, if the last packet of a fragmented frame.
**
**                      num:  If frag is 1, this is the number of remaining fragments
**                            (including this fragment) of this frame.
**                            If frag is 0, this is the number of frames in this packet.
**
** Returns          void.
******************************************************************************/
void A2D_ParsSbcMplHdr(UINT8 *p_src, BOOLEAN *p_frag, BOOLEAN *p_start, BOOLEAN *p_last, UINT8 *p_num)
{
    if (p_src && p_frag && p_start && p_last && p_num) {
        *p_frag = (*p_src & A2D_SBC_HDR_F_MSK) ? TRUE : FALSE;
        *p_start = (*p_src & A2D_SBC_HDR_S_MSK) ? TRUE : FALSE;
        *p_last = (*p_src & A2D_SBC_HDR_L_MSK) ? TRUE : FALSE;
        *p_num  = (*p_src & A2D_SBC_HDR_NUM_MSK);
    }
}

const char* A2DP_CodecNameSbc(const uint8_t* p_codec_info) {
  UNUSED(p_codec_info);
  return "SBC";
}

bool A2DP_IsSinkCodecValidSbc(const uint8_t* p_codec_info) {
  tA2D_SBC_CIE cfg_cie;

  /* Use a liberal check when parsing the codec info */
  return (A2D_ParsSbcInfo(&cfg_cie, (UINT8 *)p_codec_info, false) == A2D_SUCCESS) ||
         (A2D_ParsSbcInfo(&cfg_cie, (UINT8 *)p_codec_info, true) == A2D_SUCCESS);
}

bool A2DP_IsPeerSinkCodecValidSbc(const uint8_t* p_codec_info) {
  tA2D_SBC_CIE cfg_cie;

  /* Use a liberal check when parsing the codec info */
  return (A2D_ParsSbcInfo(&cfg_cie, (UINT8 *)p_codec_info, false) == A2D_SUCCESS) ||
         (A2D_ParsSbcInfo(&cfg_cie, (UINT8 *)p_codec_info, true) == A2D_SUCCESS);
}

tA2D_STATUS A2DP_IsSinkCodecSupportedSbc(const uint8_t* p_codec_info) {
  return A2DP_CodecInfoMatchesCapabilitySbc(&a2dp_sbc_sink_caps, p_codec_info,
                                            false);
}

tA2D_STATUS A2DP_IsPeerSourceCodecSupportedSbc(const uint8_t* p_codec_info) {
  return A2DP_CodecInfoMatchesCapabilitySbc(&a2dp_sbc_sink_caps, p_codec_info,
                                            true);
}

btav_a2dp_codec_index_t A2DP_SinkCodecIndexSbc(const uint8_t* p_codec_info) {
  UNUSED(p_codec_info);
  return BTAV_A2DP_CODEC_INDEX_SINK_SBC;
}

btav_a2dp_codec_index_t A2DP_SourceCodecIndexSbc(const uint8_t* p_codec_info) {
  UNUSED(p_codec_info);
  return BTAV_A2DP_CODEC_INDEX_SOURCE_SBC;
}

// Checks whether A2DP SBC codec configuration matches with a device's codec
// capabilities. |p_cap| is the SBC codec configuration. |p_codec_info| is
// the device's codec capabilities. |is_capability| is true if
// |p_codec_info| contains A2DP codec capability.
// Returns A2D_SUCCESS if the codec configuration matches with capabilities,
// otherwise the corresponding A2DP error status code.
static tA2D_STATUS A2DP_CodecInfoMatchesCapabilitySbc(
    const tA2D_SBC_CIE* p_cap, const uint8_t* p_codec_info,
    bool is_capability) {
  tA2D_STATUS status;
  tA2D_SBC_CIE cfg_cie;

  /* parse configuration */
  status = A2D_ParsSbcInfo(&cfg_cie, (uint8_t*)p_codec_info, is_capability);
  if (status != A2D_SUCCESS) {
    LOG_ERROR("%s: parsing failed %d", __func__, status);
    return status;
  }

  /* verify that each parameter is in range */

  LOG_VERBOSE("%s: FREQ peer: 0x%x, capability 0x%x", __func__,
              cfg_cie.samp_freq, p_cap->samp_freq);
  LOG_VERBOSE("%s: CH_MODE peer: 0x%x, capability 0x%x", __func__,
              cfg_cie.ch_mode, p_cap->ch_mode);
  LOG_VERBOSE("%s: BLOCK_LEN peer: 0x%x, capability 0x%x", __func__,
              cfg_cie.block_len, p_cap->block_len);
  LOG_VERBOSE("%s: SUB_BAND peer: 0x%x, capability 0x%x", __func__,
              cfg_cie.num_subbands, p_cap->num_subbands);
  LOG_VERBOSE("%s: ALLOC_METHOD peer: 0x%x, capability 0x%x", __func__,
              cfg_cie.alloc_mthd, p_cap->alloc_mthd);
  LOG_VERBOSE("%s: MIN_BitPool peer: 0x%x, capability 0x%x", __func__,
              cfg_cie.min_bitpool, p_cap->min_bitpool);
  LOG_VERBOSE("%s: MAX_BitPool peer: 0x%x, capability 0x%x", __func__,
              cfg_cie.max_bitpool, p_cap->max_bitpool);

  /* sampling frequency */
  if ((cfg_cie.samp_freq & p_cap->samp_freq) == 0) return A2D_NS_SAMP_FREQ;

  /* channel mode */
  if ((cfg_cie.ch_mode & p_cap->ch_mode) == 0) return A2D_NS_CH_MODE;

  /* block length */
  if ((cfg_cie.block_len & p_cap->block_len) == 0) return A2D_BAD_BLOCK_LEN;

  /* subbands */
  if ((cfg_cie.num_subbands & p_cap->num_subbands) == 0)
    return A2D_NS_SUBBANDS;

  /* allocation method */
  if ((cfg_cie.alloc_mthd & p_cap->alloc_mthd) == 0)
    return A2D_NS_ALLOC_MTHD;

  /* min bitpool */
  if (cfg_cie.min_bitpool > p_cap->max_bitpool) return A2D_NS_MIN_BITPOOL;

  /* max bitpool */
  if (cfg_cie.max_bitpool < p_cap->min_bitpool) return A2D_NS_MAX_BITPOOL;

  return A2D_SUCCESS;
}

bool A2DP_CodecTypeEqualsSbc(const uint8_t* p_codec_info_a,
                             const uint8_t* p_codec_info_b) {
  tA2D_SBC_CIE sbc_cie_a;
  tA2D_SBC_CIE sbc_cie_b;

  // Check whether the codec info contains valid data
  tA2D_STATUS a2dp_status =
      A2D_ParsSbcInfo(&sbc_cie_a, (uint8_t*)p_codec_info_a, true);
  if (a2dp_status != A2D_SUCCESS) {
    if (a2dp_status != A2D_WRONG_CODEC)
      LOG_ERROR("%s: cannot decode codec information: %d", __func__, a2dp_status);
    return false;
  }
  a2dp_status = A2D_ParsSbcInfo(&sbc_cie_b, (uint8_t*)p_codec_info_b, true);
  if (a2dp_status != A2D_SUCCESS) {
      if (a2dp_status != A2D_WRONG_CODEC)
        LOG_ERROR("%s: cannot decode codec information: %d", __func__, a2dp_status);
    return false;
  }

  tA2D_CODEC_TYPE codec_type_a = A2DP_GetCodecType(p_codec_info_a);
  tA2D_CODEC_TYPE codec_type_b = A2DP_GetCodecType(p_codec_info_b);

  return (codec_type_a == codec_type_b) && (codec_type_a == A2D_MEDIA_CT_SBC);
}

bool A2DP_InitCodecConfigSbc(btav_a2dp_codec_index_t codec_index, UINT8 *p_result) {
  switch(codec_index) {
    case BTAV_A2DP_CODEC_INDEX_SINK_SBC:
      return A2DP_InitCodecConfigSbcSink(p_result);
    case BTAV_A2DP_CODEC_INDEX_SOURCE_SBC:
      return A2DP_InitCodecConfigSbcSrc(p_result);
    default:
      break;
  }

  return false;
}

bool A2DP_InitCodecConfigSbcSink(uint8_t* p_codec_info) {
  if (A2D_BldSbcInfo(A2D_MEDIA_TYPE_AUDIO, (tA2D_SBC_CIE *)&a2dp_sbc_sink_caps,
                     p_codec_info) != A2D_SUCCESS) {
    return false;
  }

  return true;
}

bool A2DP_InitCodecConfigSbcSrc(uint8_t* p_codec_info) {
  if (A2D_BldSbcInfo(A2D_MEDIA_TYPE_AUDIO, (tA2D_SBC_CIE *)&a2dp_sbc_src_caps,
                     p_codec_info) != A2D_SUCCESS) {
    return false;
  }

  return true;
}

bool A2DP_BuildCodecConfigSbc(UINT8 *p_src_cap, UINT8 *p_result) {
    tA2D_SBC_CIE    src_cap;
    tA2D_SBC_CIE    pref_cap = a2dp_sbc_default_config;
    UINT8           status = 0;

    /* now try to build a preferred one */
    /* parse configuration */
    if ((status = A2D_ParsSbcInfo(&src_cap, p_src_cap, TRUE)) != 0) {
        APPL_TRACE_DEBUG(" Cant parse src cap ret = %d", status);
        return false;
    }

    if (src_cap.samp_freq & A2D_SBC_IE_SAMP_FREQ_48) {
        pref_cap.samp_freq = A2D_SBC_IE_SAMP_FREQ_48;
    } else if (src_cap.samp_freq & A2D_SBC_IE_SAMP_FREQ_44) {
        pref_cap.samp_freq = A2D_SBC_IE_SAMP_FREQ_44;
    }

    if (src_cap.ch_mode & A2D_SBC_IE_CH_MD_JOINT) {
        pref_cap.ch_mode = A2D_SBC_IE_CH_MD_JOINT;
    } else if (src_cap.ch_mode & A2D_SBC_IE_CH_MD_STEREO) {
        pref_cap.ch_mode = A2D_SBC_IE_CH_MD_STEREO;
    } else if (src_cap.ch_mode & A2D_SBC_IE_CH_MD_DUAL) {
        pref_cap.ch_mode = A2D_SBC_IE_CH_MD_DUAL;
    } else if (src_cap.ch_mode & A2D_SBC_IE_CH_MD_MONO) {
        pref_cap.ch_mode = A2D_SBC_IE_CH_MD_MONO;
    }

    if (src_cap.block_len & A2D_SBC_IE_BLOCKS_16) {
        pref_cap.block_len = A2D_SBC_IE_BLOCKS_16;
    } else if (src_cap.block_len & A2D_SBC_IE_BLOCKS_12) {
        pref_cap.block_len = A2D_SBC_IE_BLOCKS_12;
    } else if (src_cap.block_len & A2D_SBC_IE_BLOCKS_8) {
        pref_cap.block_len = A2D_SBC_IE_BLOCKS_8;
    } else if (src_cap.block_len & A2D_SBC_IE_BLOCKS_4) {
        pref_cap.block_len = A2D_SBC_IE_BLOCKS_4;
    }

    if (src_cap.num_subbands & A2D_SBC_IE_SUBBAND_8) {
        pref_cap.num_subbands = A2D_SBC_IE_SUBBAND_8;
    } else if (src_cap.num_subbands & A2D_SBC_IE_SUBBAND_4) {
        pref_cap.num_subbands = A2D_SBC_IE_SUBBAND_4;
    }

    if (src_cap.alloc_mthd & A2D_SBC_IE_ALLOC_MD_L) {
        pref_cap.alloc_mthd = A2D_SBC_IE_ALLOC_MD_L;
    } else if (src_cap.alloc_mthd & A2D_SBC_IE_ALLOC_MD_S) {
        pref_cap.alloc_mthd = A2D_SBC_IE_ALLOC_MD_S;
    }

    if (A2D_SBC_IE_MIN_BITPOOL <= src_cap.max_bitpool &&
        src_cap.max_bitpool <= A2D_SBC_IE_MAX_BITPOOL) {
        pref_cap.max_bitpool = pref_cap.max_bitpool < src_cap.max_bitpool ?
                               pref_cap.max_bitpool : src_cap.max_bitpool;
    }

    if (A2D_SBC_IE_MIN_BITPOOL <= src_cap.min_bitpool &&
        src_cap.min_bitpool <= A2D_SBC_IE_MAX_BITPOOL) {
        pref_cap.min_bitpool = pref_cap.min_bitpool > src_cap.min_bitpool ?
                               pref_cap.min_bitpool : src_cap.min_bitpool;
    }

    status = A2D_BldSbcInfo(A2D_MEDIA_TYPE_AUDIO, (tA2D_SBC_CIE *) &pref_cap, p_result);
    return status == A2D_SUCCESS;
}

const tA2DP_DECODER_INTERFACE* A2DP_GetDecoderInterfaceSbc(
    const uint8_t* p_codec_info) {
  if (!A2DP_IsSinkCodecValidSbc(p_codec_info)) return NULL;

  return &a2dp_decoder_interface_sbc;
}

#endif /* #if (defined(A2D_INCLUDED) && A2D_INCLUDED == TRUE) */
