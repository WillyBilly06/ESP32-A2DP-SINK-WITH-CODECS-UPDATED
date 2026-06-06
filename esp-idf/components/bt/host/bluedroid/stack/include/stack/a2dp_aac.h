/*
 * SPDX-FileCopyrightText: 2016 The Android Open Source Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef A2DP_AAC_H
#define A2DP_AAC_H

#include "bt_av.h"
#include "a2d_api.h"
#include "a2dp_codec_api.h"
#include "a2dp_aac_constants.h"

#if (A2D_INCLUDED == TRUE)

/*****************************************************************************
**  Type Definitions
*****************************************************************************/

// data type for the AAC Codec Information Element */
// NOTE: bits_per_sample is needed only for AAC encoder initialization.
typedef struct {
  uint8_t objectType;             /* Object Type */
  uint16_t sampleRate;            /* Sampling Frequency */
  uint8_t channelMode;            /* STEREO/MONO */
  uint8_t variableBitRateSupport; /* Variable Bit Rate Support*/
  uint32_t bitRate;               /* Bit rate */
  btav_a2dp_codec_bits_per_sample_t bits_per_sample;
} tA2DP_AAC_CIE;

/*****************************************************************************
**  External Function Declarations
*****************************************************************************/
#ifdef __cplusplus
extern "C"
{
#endif

// Builds the AAC Media Codec Capabilities byte sequence beginning from the
// LOSC octet. |media_type| is the media type |AVDT_MEDIA_TYPE_*|.
// |p_ie| is a pointer to the AAC Codec Information Element information.
// The result is stored in |p_result|. Returns A2DP_SUCCESS on success,
// otherwise the corresponding A2DP error status code.
tA2D_STATUS A2DP_BuildInfoAac(uint8_t media_type,
                                      const tA2DP_AAC_CIE* p_ie,
                                      uint8_t* p_result);

// Parses the AAC Media Codec Capabilities byte sequence beginning from the
// LOSC octet. The result is stored in |p_ie|. The byte sequence to parse is
// |p_codec_info|. If |is_capability| is true, the byte sequence is
// codec capabilities, otherwise is codec configuration.
// Returns A2DP_SUCCESS on success, otherwise the corresponding A2DP error
// status code.
tA2D_STATUS A2DP_ParseInfoAac(tA2DP_AAC_CIE* p_ie,
                                      const uint8_t* p_codec_info,
                                      bool is_capability);

// Gets the A2DP AAC codec name for a given |p_codec_info|.
const char* A2DP_CodecNameAac(const uint8_t* p_codec_info);

bool A2DP_IsPeerSinkCodecValidAac(const uint8_t* p_codec_info);

// Checks whether two A2DP AAC codecs |p_codec_info_a| and |p_codec_info_b|
// have the same type.
// Returns true if the two codecs have the same type, otherwise false.
bool A2DP_CodecTypeEqualsAac(const uint8_t* p_codec_info_a,
                             const uint8_t* p_codec_info_b);

// Checks whether the codec capabilities contain a valid A2DP AAC Sink codec.
// NOTE: only codecs that are implemented are considered valid.
// Returns true if |p_codec_info| contains information about a valid AAC codec,
// otherwise false.
bool A2DP_IsSinkCodecValidAac(const uint8_t* p_codec_info);

// Checks whether the codec capabilities contain a valid peer A2DP AAC Source
// codec.
// NOTE: only codecs that are implemented are considered valid.
// Returns true if |p_codec_info| contains information about a valid AAC codec,
// otherwise false.
bool A2DP_IsPeerSourceCodecValidAac(const uint8_t* p_codec_info);

// Checks whether A2DP AAC Sink codec is supported.
// |p_codec_info| contains information about the codec capabilities.
// Returns true if the A2DP AAC Sink codec is supported, otherwise false.
tA2D_STATUS A2DP_IsSinkCodecSupportedAac(const uint8_t* p_codec_info);

// Checks whether an A2DP AAC Source codec for a peer Source device is
// supported.
// |p_codec_info| contains information about the codec capabilities of the
// peer device.
// Returns true if the A2DP AAC Source codec for a peer Source device is
// supported, otherwise false.
tA2D_STATUS A2DP_IsPeerSourceCodecSupportedAac(const uint8_t* p_codec_info);

btav_a2dp_codec_index_t A2DP_SinkCodecIndexAac(const uint8_t* p_codec_info);

// Gets the A2DP AAC Source codec index for a given |p_codec_info|.
// Returns the corresponding |btav_a2dp_codec_index_t| on success,
// otherwise |BTAV_A2DP_CODEC_INDEX_MAX|.
btav_a2dp_codec_index_t A2DP_SourceCodecIndexAac(const uint8_t* p_codec_info);

// Gets the current A2DP AAC decoder interface that can be used to decode
// received A2DP packets - see |tA2DP_DECODER_INTERFACE|.
// |p_codec_info| contains the codec information.
// Returns the A2DP AAC decoder interface if the |p_codec_info| is valid and
// supported, otherwise NULL.
const tA2DP_DECODER_INTERFACE* A2DP_GetDecoderInterfaceAac(
    const uint8_t* p_codec_info);

// Initializes A2DP AAC Source codec information into |AvdtpSepConfig|
// configuration entry pointed by |p_cfg|.
bool A2DP_InitCodecConfigAac(btav_a2dp_codec_index_t codec_index, UINT8 *p_result);

// Initializes A2DP AAC Sink codec information into |AvdtpSepConfig|
// configuration entry pointed by |p_cfg|.
bool A2DP_InitCodecConfigAacSink(uint8_t* p_codec_info);

bool A2DP_BuildCodecConfigAac(UINT8 *p_src_cap, UINT8 *p_result);


#ifdef __cplusplus
}
#endif

#endif  ///A2D_INCLUDED == TRUE

#endif /* A2DP_AAC_H */
