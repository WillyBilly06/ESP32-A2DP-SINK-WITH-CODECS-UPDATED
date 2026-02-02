/*
 * SPDX-License-Identifier: Apache-2.0
 */

//
// A2DP Codec API for Opus
//

#ifndef A2DP_VENDOR_OPUS_H
#define A2DP_VENDOR_OPUS_H

#include "a2d_api.h"
#include "a2dp_codec_api.h"
#include "a2dp_vendor_opus_constants.h"
#include "avdt_api.h"
#include "bt_av.h"


#define min( a, b )             \
    ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
       _a < _b ? _a : _b; });

/*****************************************************************************
**  Type Definitions
*****************************************************************************/
// data type for the Opus Codec Information Element */
typedef struct {
  uint32_t vendorId;
  uint16_t codecId;

  uint8_t channels;
  uint8_t coupled_streams;
  uint32_t audio_location;
  uint8_t frame_duration;
  uint16_t maximum_bitrate;

  uint8_t bidi_channels;
  uint8_t bidi_coupled_streams;
  uint32_t bidi_audio_location;
  uint8_t bidi_frame_duration;
  uint16_t bidi_maximum_bitrate;
} tA2DP_OPUS_CIE;

/*****************************************************************************
**  External Function Declarations
*****************************************************************************/
#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************
**
** Function         A2DP_ParseInfoOpus
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
tA2D_STATUS A2DP_ParseInfoOpus(tA2DP_OPUS_CIE* p_ie,const uint8_t* p_codec_info,bool is_capability);

/******************************************************************************
**
** Function         A2DP_BuildInfoOpus
**
** Description      Builds the Opus Media Codec Capabilities byte sequence beginning from the
**                  LOSC octet.
**                  
**                      media_type:  the media type |AVDT_MEDIA_TYPE_*|.
**
**                      p_ie:  is a pointer to the Opus Codec Information Element information.
**
**                  Output Parameters:
**                      p_result:  Codec capabilities.
**
** Returns          A2D_SUCCESS on success, otherwise the corresponding A2DP error status code.
**
******************************************************************************/
tA2D_STATUS A2DP_BuildInfoOpus(uint8_t media_type, const tA2DP_OPUS_CIE* p_ie, uint8_t* p_result);

/******************************************************************************
**
** Function         A2DP_IsVendorPeerSourceCodecValidOpus
**
** Description      Checks whether the codec capabilities contain a valid peer A2DP Opus Source
**                  codec.
**                  NOTE: only codecs that are implemented are considered valid.
**
**                      p_codec_info:  contains information about the codec capabilities of the
**                                     peer device.
**
** Returns          true if |p_codec_info| contains information about a valid Opus codec,
**                  otherwise false.
**
******************************************************************************/
tA2D_STATUS A2DP_IsVendorPeerSourceCodecValidOpus(const uint8_t* p_codec_info);

/******************************************************************************
**
** Function         A2DP_IsVendorPeerSinkCodecValidOpus
**
** Description      Checks whether the codec capabilities contain a valid peer A2DP Opus Sink
**                  codec.
**                  NOTE: only codecs that are implemented are considered valid.
**
**                      p_codec_info:  contains information about the codec capabilities of the
**                                     peer device.
**
** Returns          true if |p_codec_info| contains information about a valid Opus codec,
**                  otherwise false.
**
******************************************************************************/
bool A2DP_IsVendorPeerSinkCodecValidOpus(const uint8_t* p_codec_info);

/******************************************************************************
**
** Function         A2DP_VendorSinkCodecIndexOpus
**
** Description      Gets the A2DP Opus Sink codec index for a given |p_codec_info|.
**
**                      p_codec_info:  Contains information about the codec capabilities.
**
** Returns          the corresponding |btav_a2dp_codec_index_t| on success,
**                  otherwise |BTAV_A2DP_CODEC_INDEX_MAX|.
**
******************************************************************************/
btav_a2dp_codec_index_t A2DP_VendorSinkCodecIndexOpus(
    const uint8_t* p_codec_info);

/******************************************************************************
**
** Function         A2DP_VendorSourceCodecIndexOpus
**
** Description      Gets the A2DP Opus Source codec index for a given |p_codec_info|.
**
**                      p_codec_info:  Contains information about the codec capabilities.
**
** Returns          the corresponding |btav_a2dp_codec_index_t| on success,
**                  otherwise |BTAV_A2DP_CODEC_INDEX_MAX|.
**
******************************************************************************/
btav_a2dp_codec_index_t A2DP_VendorSourceCodecIndexOpus(
    const uint8_t* p_codec_info);

/******************************************************************************
**
** Function         A2DP_VendorCodecNameOpus
**
** Description      Gets the A2DP Opus codec name for a given |p_codec_info|.
**
**                      p_codec_info:  Contains information about the codec capabilities.
**
** Returns          Codec name.
**
******************************************************************************/
const char* A2DP_VendorCodecNameOpus(const uint8_t* p_codec_info);

/******************************************************************************
**
** Function         A2DP_VendorCodecTypeEqualsOpus
**
** Description      Checks whether two A2DP Opus codecs |p_codec_info_a| and |p_codec_info_b|
**                  have the same type.
**
**                      p_codec_info_a:  Contains information about the codec capabilities.
**
**                      p_codec_info_b:  Contains information about the codec capabilities.
**
** Returns          true if the two codecs have the same type, otherwise false.
**
******************************************************************************/
bool A2DP_VendorCodecTypeEqualsOpus(const uint8_t* p_codec_info_a,
                                    const uint8_t* p_codec_info_b);

/******************************************************************************
**
** Function         A2DP_VendorInitCodecConfigOpus
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
bool A2DP_VendorInitCodecConfigOpus(btav_a2dp_codec_index_t codec_index, UINT8 *p_result);

/******************************************************************************
**
** Function         A2DP_VendorInitCodecConfigOpusSink
**
** Description      Initializes A2DP codec-specific information into |p_result|.
**                  The selected codec is defined by |codec_index|.
**
**                      p_codec_info:  Contains information about the codec capabilities.
**
** Returns          true on success, otherwise false.
**
******************************************************************************/
bool A2DP_VendorInitCodecConfigOpusSink(uint8_t* p_codec_info);

/******************************************************************************
**
** Function         A2DP_VendorBuildCodecConfigOpus
**
** Description      Initializes A2DP codec-specific information into |p_result|.
**                  The selected codec is defined by |codec_index|.
**
**                      p_codec_info:  Contains information about the codec capabilities.
**
** Returns          true on success, otherwise false.
**
******************************************************************************/
bool A2DP_VendorBuildCodecConfigOpus(UINT8 *p_src_cap, UINT8 *p_result);

/******************************************************************************
**
** Function         A2DP_GetVendorDecoderInterfaceOpus
**
** Description      Gets the A2DP Opus decoder interface that can be used to decode received A2DP
**                  packets
**
**                      p_codec_info:  contains the codec information.
**
** Returns          the A2DP Opus decoder interface if the |p_codec_info| is valid and
**                  supported, otherwise NULL.
**
******************************************************************************/
const tA2DP_DECODER_INTERFACE* A2DP_GetVendorDecoderInterfaceOpus(
    const uint8_t* p_codec_info);

#ifdef __cplusplus
}
#endif

#endif  // A2DP_VENDOR_OPUS_H
