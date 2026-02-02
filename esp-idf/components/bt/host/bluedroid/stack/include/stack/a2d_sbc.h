/******************************************************************************
 *
 *  Copyright (C) 2000-2012 Broadcom Corporation
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
 *  nterface to low complexity subband codec (SBC)
 *
 ******************************************************************************/
#ifndef A2D_SBC_H
#define A2D_SBC_H

#include "bt_av.h"
#include "a2d_api.h"
#include "a2dp_codec_api.h"

#if (A2D_INCLUDED == TRUE)
/*****************************************************************************
**  Constants
*****************************************************************************/
/* the length of the SBC Media Payload header. */
#define A2D_SBC_MPL_HDR_LEN         1

/* CIE offset in the info byte sequence */
#define A2D_SBC_CIE_OFF             3
/* CIE length in the info byte sequence */
#define A2D_SBC_CIE_LEN             4

/* the LOSC of SBC media codec capability */
#define A2D_SBC_INFO_LEN            6

/* for Codec Specific Information Element */
#define A2D_SBC_IE_SAMP_FREQ_MSK    0xF0    /* b7-b4 sampling frequency */
#define A2D_SBC_IE_SAMP_FREQ_16     0x80    /* b7:16  kHz */
#define A2D_SBC_IE_SAMP_FREQ_32     0x40    /* b6:32  kHz */
#define A2D_SBC_IE_SAMP_FREQ_44     0x20    /* b5:44.1kHz */
#define A2D_SBC_IE_SAMP_FREQ_48     0x10    /* b4:48  kHz */

#define A2D_SBC_IE_CH_MD_MSK        0x0F    /* b3-b0 channel mode */
#define A2D_SBC_IE_CH_MD_MONO       0x08    /* b3: mono */
#define A2D_SBC_IE_CH_MD_DUAL       0x04    /* b2: dual */
#define A2D_SBC_IE_CH_MD_STEREO     0x02    /* b1: stereo */
#define A2D_SBC_IE_CH_MD_JOINT      0x01    /* b0: joint stereo */

#define A2D_SBC_IE_BLOCKS_MSK       0xF0    /* b7-b4 number of blocks */
#define A2D_SBC_IE_BLOCKS_4         0x80    /* 4 blocks */
#define A2D_SBC_IE_BLOCKS_8         0x40    /* 8 blocks */
#define A2D_SBC_IE_BLOCKS_12        0x20    /* 12blocks */
#define A2D_SBC_IE_BLOCKS_16        0x10    /* 16blocks */

#define A2D_SBC_IE_SUBBAND_MSK      0x0C    /* b3-b2 number of subbands */
#define A2D_SBC_IE_SUBBAND_4        0x08    /* b3: 4 */
#define A2D_SBC_IE_SUBBAND_8        0x04    /* b2: 8 */

#define A2D_SBC_IE_ALLOC_MD_MSK     0x03    /* b1-b0 allocation mode */
#define A2D_SBC_IE_ALLOC_MD_S       0x02    /* b1: SNR */
#define A2D_SBC_IE_ALLOC_MD_L       0x01    /* b0: loundess */

#define A2D_SBC_IE_MIN_BITPOOL      2
#define A2D_SBC_IE_MAX_BITPOOL      250

/* for media payload header */
#define A2D_SBC_HDR_F_MSK           0x80
#define A2D_SBC_HDR_S_MSK           0x40
#define A2D_SBC_HDR_L_MSK           0x20
#define A2D_SBC_HDR_NUM_MSK         0x0F

/*****************************************************************************
**  Type Definitions
*****************************************************************************/

/* data type for the SBC Codec Information Element*/
typedef struct {
    UINT8   samp_freq;      /* Sampling frequency */
    UINT8   ch_mode;        /* Channel mode */
    UINT8   block_len;      /* Block length */
    UINT8   num_subbands;   /* Number of subbands */
    UINT8   alloc_mthd;     /* Allocation method */
    UINT8   max_bitpool;    /* Maximum bitpool */
    UINT8   min_bitpool;    /* Minimum bitpool */
} tA2D_SBC_CIE;


/*****************************************************************************
**  External Function Declarations
*****************************************************************************/
#ifdef __cplusplus
extern "C"
{
#endif
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
extern tA2D_STATUS A2D_BldSbcInfo(UINT8 media_type, tA2D_SBC_CIE *p_ie,
                                  UINT8 *p_result);

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
extern tA2D_STATUS A2D_ParsSbcInfo(tA2D_SBC_CIE *p_ie, UINT8 *p_info,
                                   BOOLEAN for_caps);

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
extern void A2D_BldSbcMplHdr(UINT8 *p_dst, BOOLEAN frag, BOOLEAN start,
                             BOOLEAN last, UINT8 num);

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
extern void A2D_ParsSbcMplHdr(UINT8 *p_src, BOOLEAN *p_frag,
                              BOOLEAN *p_start, BOOLEAN *p_last,
                              UINT8 *p_num);

/******************************************************************************
**
** Function         A2DP_CodecNameSbc
**
** Description      Gets the A2DP SBC codec name for a given |p_codec_info|.
**
**                      p_codec_info:  Contains information about the codec capabilities.
**
** Returns          Codec name.
**
******************************************************************************/
const char* A2DP_CodecNameSbc(const uint8_t* p_codec_info);

// Checks whether the codec capabilities contain a valid A2DP SBC Sink codec.
// NOTE: only codecs that are implemented are considered valid.
// Returns true if |p_codec_info| contains information about a valid SBC codec,
// otherwise false.
/******************************************************************************
**
** Function         A2DP_IsSinkCodecValidSbc
**
** Description      Checks whether the codec capabilities contain a valid A2DP SBC Sink codec.
**                  NOTE: only codecs that are implemented are considered valid.
**
**                      p_codec_info:  Contains information about the codec capabilities.
**
** Returns          true if |p_codec_info| contains information about a valid codec,
**                  otherwise false.
**
******************************************************************************/
bool A2DP_IsSinkCodecValidSbc(const uint8_t* p_codec_info);

/******************************************************************************
**
** Function         A2DP_IsPeerSinkCodecValidSbc
**
** Description      Checks whether the codec capabilities contain a valid peer A2DP SBC Sink
**                  codec.
**                  NOTE: only codecs that are implemented are considered valid.
**
**                      p_codec_info:  contains information about the codec capabilities of the
**                                     peer device.
**
** Returns          true if |p_codec_info| contains information about a valid SBC codec,
**                  otherwise false.
**
******************************************************************************/
extern bool A2DP_IsPeerSinkCodecValidSbc(const uint8_t* p_codec_info);

/******************************************************************************
**
** Function         A2DP_IsSinkCodecSupportedSbc
**
** Description      Checks whether A2DP SBC Sink codec is supported.
**
**                      p_codec_info:  Contains information about the codec capabilities.
**
** Returns          true if the A2DP SBC Sink codec is supported, otherwise false.
**
******************************************************************************/
extern tA2D_STATUS A2DP_IsSinkCodecSupportedSbc(const uint8_t* p_codec_info);


/******************************************************************************
**
** Function         A2DP_IsPeerSourceCodecSupportedSbc
**
** Description      Checks whether an A2DP SBC Source codec for a peer Source device is
**                  supported
**
**                      p_codec_info:  contains information about the codec capabilities of the
**                                     peer device.
**
** Returns          true if the A2DP SBC Source codec for a peer Source device is
**                  supported, otherwise false.
**
******************************************************************************/
tA2D_STATUS A2DP_IsPeerSourceCodecSupportedSbc(const uint8_t* p_codec_info);

/******************************************************************************
**
** Function         A2DP_SinkCodecIndexSbc
**
** Description      Gets the A2DP SBC Sink codec index for a given |p_codec_info|.
**
**                      p_codec_info:  Contains information about the codec capabilities.
**
** Returns          the corresponding |btav_a2dp_codec_index_t| on success,
**                  otherwise |BTAV_A2DP_CODEC_INDEX_MAX|.
**
******************************************************************************/
extern btav_a2dp_codec_index_t A2DP_SinkCodecIndexSbc(const uint8_t* p_codec_info);

/******************************************************************************
**
** Function         A2DP_SourceCodecIndexSbc
**
** Description      Gets the A2DP SBC Source codec index for a given |p_codec_info|.
**
**                      p_codec_info:  Contains information about the codec capabilities.
**
** Returns          the corresponding |btav_a2dp_codec_index_t| on success,
**                  otherwise |BTAV_A2DP_CODEC_INDEX_MAX|.
**
******************************************************************************/
extern btav_a2dp_codec_index_t A2DP_SourceCodecIndexSbc(const uint8_t* p_codec_info);

/******************************************************************************
**
** Function
**
** Description      Checks whether two A2DP SBC codecs |p_codec_info_a| and |p_codec_info_b|
**                  have the same type.
**
**                      p_codec_info_a:  Contains information about the codec capabilities.
**
**                      p_codec_info_b:  Contains information about the codec capabilities.
**
** Returns          true if the two codecs have the same type, otherwise false.
**
******************************************************************************/
extern bool A2DP_CodecTypeEqualsSbc(const uint8_t* p_codec_info_a,
                                    const uint8_t* p_codec_info_b);

/******************************************************************************
**
** Function         A2DP_InitCodecConfigSbc
**
** Description      Initializes A2DP codec-specific information into |p_result|.
**                  The selected codec is defined by |codec_index|.
**
**                      codec_index:  Selected codec index.
**
**                  Output Parameters:
**                      p_result:  The resulting codec information element.
**
** Returns          true on success, otherwise false.
**
******************************************************************************/
bool A2DP_InitCodecConfigSbc(btav_a2dp_codec_index_t codec_index, UINT8 *p_result);

/******************************************************************************
**
** Function         A2DP_InitCodecConfigSbcSink
**
** Description      Initializes A2DP codec-specific information into |p_result|.
**                  The selected codec is defined by |codec_index|.
**
**                      p_codec_info:  Contains information about the codec capabilities.
**
** Returns          true on success, otherwise false.
**
******************************************************************************/
bool A2DP_InitCodecConfigSbcSink(uint8_t* p_codec_info);

/******************************************************************************
**
** Function         A2DP_InitCodecConfigSbcSrc
**
** Description      Initializes A2DP codec-specific information into |p_result|.
**                  The selected codec is defined by |codec_index|.
**
**                      p_codec_info:  Contains information about the codec capabilities.
**
** Returns          true on success, otherwise false.
**
******************************************************************************/
bool A2DP_InitCodecConfigSbcSrc(uint8_t* p_codec_info);

/******************************************************************************
**
** Function         A2DP_BuildCodecConfigSbc
**
** Description      Build codec info from a source config
**
**                      p_src_cap:  Codec capabilities of the A2DP source.
**
**                  Output Parameters:
**                      p_result:  The resulting codec capabilities.
**
** Returns          true on success, otherwise false.
**
******************************************************************************/
bool A2DP_BuildCodecConfigSbc(UINT8 *p_src_cap, UINT8 *p_result);

/******************************************************************************
**
** Function         A2DP_GetDecoderInterfaceSbc
**
** Description      Gets the A2DP SBC decoder interface that can be used to decode received A2DP
**                  packets
**
**                      p_codec_info:  contains the codec information.
**
** Returns          the A2DP SBC decoder interface if the |p_codec_info| is valid and
**                  supported, otherwise NULL.
**
******************************************************************************/
const tA2DP_DECODER_INTERFACE* A2DP_GetDecoderInterfaceSbc(
    const uint8_t* p_codec_info);

#ifdef __cplusplus
}
#endif

#endif  ///A2D_INCLUDED == TRUE

#endif /* A2D_SBC_H */
