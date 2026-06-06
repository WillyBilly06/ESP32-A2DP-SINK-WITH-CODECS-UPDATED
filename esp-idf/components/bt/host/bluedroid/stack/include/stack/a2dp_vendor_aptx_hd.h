/*
 * SPDX-FileCopyrightText: 2016 The Android Open Source Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

//
// A2DP Codec API for aptX-HD
//

#ifndef A2DP_VENDOR_APTX_HD_H
#define A2DP_VENDOR_APTX_HD_H

#if (A2D_INCLUDED == TRUE)

#include "a2d_api.h"
#include "a2dp_shim.h"
#include "a2dp_codec_api.h"
#include "a2dp_vendor_aptx_hd_constants.h"
#include "avdt_api.h"
#include "bt_av.h"

/*****************************************************************************
**  Type Definitions
*****************************************************************************/
// data type for the aptX-HD Codec Information Element */
typedef struct {
  uint32_t vendorId;
  uint16_t codecId;    /* Codec ID for aptX-HD */
  uint8_t sampleRate;  /* Sampling Frequency */
  uint8_t channelMode; /* STEREO/DUAL/MONO */
  uint8_t acl_sprint_reserved0;
  uint8_t acl_sprint_reserved1;
  uint8_t acl_sprint_reserved2;
  uint8_t acl_sprint_reserved3;
  btav_a2dp_codec_bits_per_sample_t bits_per_sample;
} tA2DP_APTX_HD_CIE;

/*****************************************************************************
**  External Function Declarations
*****************************************************************************/
#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************
**
** Function         A2DP_BuildInfoAptxHd
**
** Description      Builds the aptX Media Codec Capabilities byte sequence beginning from the
**                  LOSC octet.
**
**                      media_type:  the media type |AVDT_MEDIA_TYPE_*|.
**
**                      p_ie:  is a pointer to the aptX Codec Information Element information.
**
**                  Output Parameters:
**                      p_result:  Codec capabilities.
**
** Returns          A2D_SUCCESS on success, otherwise the corresponding A2DP error status code.
**
******************************************************************************/
tA2DP_STATUS A2DP_BuildInfoAptxHd(uint8_t media_type,
                                  const tA2DP_APTX_HD_CIE* p_ie,
                                  uint8_t* p_result);

/******************************************************************************
**
** Function         A2DP_ParseInfoAptxHd
**
** Description      This function is called by an application to parse
**                  the Media Codec Capabilities byte sequence
**                  beginning from the LOSC octet.
**                  Input Parameters:
**                      p_codec_info:  the byte sequence to parse.
**
**                      is_capability:  TRUE, if the byte sequence is for get capabilities response.
**
**                  Output Parameters:
**                      p_ie:  The Codec Information Element information.
**
** Returns          A2D_SUCCESS if function execution succeeded.
**                  Error status code, otherwise.
******************************************************************************/
tA2DP_STATUS A2DP_ParseInfoAptxHd(tA2DP_APTX_HD_CIE* p_ie,
                                  const uint8_t* p_codec_info,
                                  bool is_capability);

/******************************************************************************
**
** Function         A2DP_IsVendorPeerSourceCodecValidAptxHd
**
** Description      Checks whether the codec capabilities contain a valid peer A2DP aptX-HD Source
**                  codec.
**                  NOTE: only codecs that are implemented are considered valid.
**
**                      p_codec_info:  contains information about the codec capabilities of the
**                                     peer device.
**
** Returns          true if |p_codec_info| contains information about a valid aptX codec,
**                  otherwise false.
**
******************************************************************************/
tA2DP_STATUS A2DP_IsVendorPeerSourceCodecValidAptxHd(const uint8_t* p_codec_info);

/******************************************************************************
**
** Function         A2DP_IsVendorPeerSinkCodecValidAptxHd
**
** Description      Checks whether the codec capabilities contain a valid peer A2DP aptX-HD Sink
**                  codec.
**                  NOTE: only codecs that are implemented are considered valid.
**
**                      p_codec_info:  contains information about the codec capabilities of the
**                                     peer device.
**
** Returns          true if |p_codec_info| contains information about a valid aptX codec,
**                  otherwise false.
**
******************************************************************************/
bool A2DP_IsVendorPeerSinkCodecValidAptxHd(const uint8_t* p_codec_info);

/******************************************************************************
**
** Function         A2DP_VendorCodecNameAptxHd
**
** Description      Gets the A2DP aptX-HD codec name for a given |p_codec_info|.
**
**                      p_codec_info:  Contains information about the codec capabilities.
**
** Returns          Codec name.
**
******************************************************************************/
const char* A2DP_VendorCodecNameAptxHd(const uint8_t* p_codec_info);

/******************************************************************************
**
** Function         A2DP_CodecInfoMatchesCapabilityAptxHd
**
** Description      Checks whether A2DP aptX-HD codec configuration matches with a device's codec
**                  capabilities. |p_cap| is the aptX-HD codec configuration. |p_codec_info| is
**                  the device's codec capabilities.
**
**                      p_cap:  is the aptX-HD codec configuration.
**
**                      p_codec_info:  the device's codec capabilities
**
**                      is_peer_codec_info:  true, the byte sequence is codec capabilities,
**                                           otherwise is codec configuration.
**
** Returns          A2D_SUCCESS if the codec configuration matches with capabilities,
**                  otherwise the corresponding A2DP error status code.
**
******************************************************************************/
tA2DP_STATUS A2DP_CodecInfoMatchesCapabilityAptxHd(
    const tA2DP_APTX_HD_CIE* p_cap, const uint8_t* p_codec_info,
    bool is_capability);

/******************************************************************************
**
** Function         A2DP_VendorCodecTypeEqualsAptxHd
**
** Description      Checks whether two A2DP aptX-HD codecs |p_codec_info_a| and |p_codec_info_b|
**                  have the same type.
**
**                      p_codec_info_a:  Contains information about the codec capabilities.
**
**                      p_codec_info_b:  Contains information about the codec capabilities.
**
** Returns          true if the two codecs have the same type, otherwise false.
**
******************************************************************************/
bool A2DP_VendorCodecTypeEqualsAptxHd(const uint8_t* p_codec_info_a,
                                      const uint8_t* p_codec_info_b);

/******************************************************************************
**
** Function         A2DP_VendorSinkCodecIndexAptxHd
**
** Description      Gets the A2DP aptX-HD Sink codec index for a given |p_codec_info|.
**
**                      p_codec_info:  Contains information about the codec capabilities.
**
** Returns          the corresponding |btav_a2dp_codec_index_t| on success,
**                  otherwise |BTAV_A2DP_CODEC_INDEX_MAX|.
**
******************************************************************************/
btav_a2dp_codec_index_t A2DP_VendorSinkCodecIndexAptxHd(
    const uint8_t* p_codec_info);

/******************************************************************************
**
** Function         A2DP_VendorSourceCodecIndexAptxHd
**
** Description      Gets the A2DP aptX-HD Source codec index for a given |p_codec_info|.
**
**                      p_codec_info:  Contains information about the codec capabilities.
**
** Returns          the corresponding |btav_a2dp_codec_index_t| on success,
**                  otherwise |BTAV_A2DP_CODEC_INDEX_MAX|.
**
******************************************************************************/
btav_a2dp_codec_index_t A2DP_VendorSourceCodecIndexAptxHd(
    const uint8_t* p_codec_info);

/******************************************************************************
**
** Function         A2DP_VendorInitCodecConfigAptxHd
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
bool A2DP_VendorInitCodecConfigAptxHd(btav_a2dp_codec_index_t codec_index, UINT8 *p_result);

/******************************************************************************
**
** Function         A2DP_VendorInitCodecConfigAptxHdSink
**
** Description      Initializes A2DP codec-specific information into |p_result|.
**                  The selected codec is defined by |codec_index|.
**
**                      p_codec_info:  Contains information about the codec capabilities.
**
** Returns          true on success, otherwise false.
**
******************************************************************************/
bool A2DP_VendorInitCodecConfigAptxHdSink(uint8_t* p_codec_info);

/******************************************************************************
**
** Function         A2DP_VendorBuildCodecConfigAptxHd
**
** Description      Initializes A2DP codec-specific information into |p_result|.
**                  The selected codec is defined by |codec_index|.
**
**                      p_codec_info:  Contains information about the codec capabilities.
**
** Returns          true on success, otherwise false.
**
******************************************************************************/
bool A2DP_VendorBuildCodecConfigAptxHd(UINT8 *p_src_cap, UINT8 *p_result);


/******************************************************************************
**
** Function         A2DP_GetVendorDecoderInterfaceAptxHd
**
** Description      Gets the A2DP aptX-HD decoder interface that can be used to decode received A2DP
**                  packets
**
**                      p_codec_info:  contains the codec information.
**
** Returns          the A2DP aptX decoder interface if the |p_codec_info| is valid and
**                  supported, otherwise NULL.
**
******************************************************************************/
const tA2DP_DECODER_INTERFACE* A2DP_GetVendorDecoderInterfaceAptxHd(
    const uint8_t* p_codec_info);


#endif // A2D_INCLUDED == TRUE
#endif // A2DP_VENDOR_APTX_HD_H
