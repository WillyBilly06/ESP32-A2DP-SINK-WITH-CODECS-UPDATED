/*
 * Copyright 2016 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

//
// Vendor Specific A2DP Codecs Support
//

#ifndef A2DP_VENDOR_H
#define A2DP_VENDOR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bt_av.h"

/*****************************************************************************
**  Constants
*****************************************************************************/
/* Offset for A2DP vendor codec */
#define A2DP_VENDOR_CODEC_START_IDX 3

/* Offset for Vendor ID for A2DP vendor codec */
#define A2DP_VENDOR_CODEC_VENDOR_ID_START_IDX A2DP_VENDOR_CODEC_START_IDX

/* Offset for Codec ID for A2DP vendor codec */
#define A2DP_VENDOR_CODEC_CODEC_ID_START_IDX \
  (A2DP_VENDOR_CODEC_VENDOR_ID_START_IDX + sizeof(uint32_t))

/*****************************************************************************
**  External Function Declarations
*****************************************************************************/
#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************
**
** Function         A2DP_VendorParseInfo
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
tA2D_STATUS A2DP_VendorParseInfo(uint8_t* p_ie, const uint8_t* p_codec_info,
                                 bool is_capability);

/******************************************************************************
**
** Function         A2DP_IsVendorPeerSinkCodecValid
**
** Description      Checks whether the codec capabilities contain a valid peer A2DP
**                  vendor-specific Sink codec.
**                  NOTE: only codecs that are implemented are considered valid.
**
**                      p_codec_info:  contains information about the codec capabilities of the
**                                     peer device.
**
** Returns          true if |p_codec_info| contains information about a valid SBC codec,
**                  otherwise false.
**
******************************************************************************/
bool A2DP_IsVendorPeerSinkCodecValid(const uint8_t* p_codec_info);

/******************************************************************************
**
** Function         A2DP_IsVendorSinkCodecSupported
**
** Description      Checks whether a vendor-specific A2DP Sink codec is supported.
**
**                      p_codec_info:  Contains information about the codec capabilities.
**
** Returns          true if the vendor-specific A2DP Sink codec is supported, otherwise false.
**
******************************************************************************/
tA2D_STATUS A2DP_IsVendorSinkCodecSupported(const uint8_t* p_codec_info);

/******************************************************************************
**
** Function         A2DP_IsVendorPeerSourceCodecSupported
**
** Description      Checks whether a vendor-specific A2DP Source codec for a peer Source device
**                  is supported.
**
**                      p_codec_info:  contains information about the codec capabilities of the
**                                     peer device.
**
** Returns          true if the vendor-specific A2DP Source codec for a peer Source
**                  device is supported, otherwise false.
**
******************************************************************************/
tA2D_STATUS A2DP_IsVendorPeerSourceCodecSupported(const uint8_t* p_codec_info);

/******************************************************************************
**
** Function         A2DP_VendorCodecGetVendorId
**
** Description      Gets the Vendor ID for the vendor-specific A2DP codec.
**
**                      p_codec_info:  Contains information about the codec capabilities.
**
** Returns          the Vendor ID for the vendor-specific A2DP codec.
**
******************************************************************************/
uint32_t A2DP_VendorCodecGetVendorId(const uint8_t* p_codec_info);

/******************************************************************************
**
** Function         A2DP_VendorCodecGetCodecId
**
** Description      Gets the Codec ID for the vendor-specific A2DP codec.
**
**                      p_codec_info:  Contains information about the codec capabilities.
**
** Returns          the Codec ID for the vendor-specific A2DP codec.
**
******************************************************************************/
uint16_t A2DP_VendorCodecGetCodecId(const uint8_t* p_codec_info);

/******************************************************************************
**
** Function         A2DP_VendorSinkCodecIndex
**
** Description      Gets the A2DP vendor Sink codec index for a given |p_codec_info|.
**
**                      p_codec_info:  Contains information about the codec capabilities.
**
** Returns          the corresponding |btav_a2dp_codec_index_t| on success,
**                  otherwise |BTAV_A2DP_CODEC_INDEX_MAX|.
**
******************************************************************************/
btav_a2dp_codec_index_t A2DP_VendorSinkCodecIndex(
    const uint8_t* p_codec_info);

/******************************************************************************
**
** Function         A2DP_VendorSourceCodecIndex
**
** Description      Gets the A2DP vendor Source codec index for a given |p_codec_info|.
**
**                      p_codec_info:  Contains information about the codec capabilities.
**
** Returns          the corresponding |btav_a2dp_codec_index_t| on success,
**                  otherwise |BTAV_A2DP_CODEC_INDEX_MAX|.
**
******************************************************************************/
btav_a2dp_codec_index_t A2DP_VendorSourceCodecIndex(
    const uint8_t* p_codec_info);

/******************************************************************************
**
** Function         A2DP_VendorInitCodecConfig
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
bool A2DP_VendorInitCodecConfig(btav_a2dp_codec_index_t codec_index, UINT8 *p_result);

/******************************************************************************
**
** Function         A2DP_VendorBuildCodecConfig
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
bool A2DP_VendorBuildCodecConfig(UINT8 *p_src_cap, UINT8 *p_result);

/******************************************************************************
**
** Function         A2DP_VendorCodecName
**
** Description      Gets the A2DP vendor-specific codec name for a given |p_codec_info|.
**
**                      p_codec_info:  Contains information about the codec capabilities.
**
** Returns          Codec name.
**
******************************************************************************/
const char* A2DP_VendorCodecName(const uint8_t* p_codec_info);

/******************************************************************************
**
** Function
**
** Description      Checks whether two A2DP vendor-specific codecs |p_codec_info_a| and
**                  |p_codec_info_b| have the same type.
**
**                      p_codec_info_a:  Contains information about the codec capabilities.
**
**                      p_codec_info_b:  Contains information about the codec capabilities.
**
** Returns          true if the two codecs have the same type, otherwise false.
**
******************************************************************************/
bool A2DP_VendorCodecTypeEquals(const uint8_t* p_codec_info_a,
                                const uint8_t* p_codec_info_b);

/******************************************************************************
**
** Function         A2DP_GetVendorDecoderInterface
**
** Description      Gets the A2DP vendor-specific decoder interface that can be used to decode A2DP
**                  packets.
**
**                      p_codec_info:  contains the codec information.
**
** Returns          the A2DP SBC decoder interface if the |p_codec_info| is valid and
**                  supported, otherwise NULL.
**
******************************************************************************/
const tA2DP_DECODER_INTERFACE* A2DP_GetVendorDecoderInterface(
    const uint8_t* p_codec_info);

#ifdef __cplusplus
}
#endif

#endif  // A2DP_VENDOR_H
