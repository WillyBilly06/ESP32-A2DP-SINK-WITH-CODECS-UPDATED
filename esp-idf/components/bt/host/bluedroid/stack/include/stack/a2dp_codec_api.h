/*
 * SPDX-FileCopyrightText: 2016 The Android Open Source Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

//
// A2DP Codecs API
//

#ifndef A2DP_CODEC_API_H
#define A2DP_CODEC_API_H

#include "bt_av.h"
#include "stack/a2d_api.h"

// Prototype for a callback to receive decoded audio data from a
// tA2DP_DECODER_INTERFACE|.
// |buf| is a pointer to the data.
// |len| is the number of octets pointed to by |buf|.
typedef void (*decoded_data_callback_t)(uint8_t* buf, uint32_t len);

//
// A2DP decoder callbacks interface.
//
typedef struct {
  // Initialize the decoder. Can be called multiple times, will reinitalize.
  bool (*decoder_init)(decoded_data_callback_t decode_callback);

  // Cleanup the A2DP decoder.
  void (*decoder_cleanup)();

  // Reset the A2DP decoder.
  bool (*decoder_reset)();

  // Decodes codec header in |p_buf|.
  ssize_t (*decode_packet_header)(BT_HDR* p_buf);

  // Decodes |p_buf| and calls |decode_callback| passed into init for the
  // decoded data.
  bool (*decode_packet)(BT_HDR* p_buf, unsigned char* buf, size_t buf_len);

  // Start the A2DP decoder.
  void (*decoder_start)();

  // Suspend the A2DP decoder.
  void (*decoder_suspend)();

  // A2DP decoder configuration.
  void (*decoder_configure)(const uint8_t* p_codec_info);
} tA2DP_DECODER_INTERFACE;


/******************************************************************************
**
** Function         A2DP_GetCodecType
**
** Description      Gets the A2DP codec type.
**
**                      p_codec_info:  Contains information about the codec capabilities.
**
** Returns          tA2D_CODEC_TYPE codec type
**
******************************************************************************/
tA2D_CODEC_TYPE A2DP_GetCodecType(const uint8_t* p_codec_info);

/******************************************************************************
**
** Function         A2DP_ParseInfo
**
** Description      This function is called by an application to parse
**                  the Media Codec Capabilities byte sequence
**                  beginning from the LOSC octet.
**                  Input Parameters:
**                      p_info:  the byte sequence to parse.
**
**                      for_caps:  TRUE, if the byte sequence is for get capabilities response.
**
**                  Output Parameters:
**                      p_ie:  The Codec Information Element information.
**
** Returns          A2D_SUCCESS if function execution succeeded.
**                  Error status code, otherwise.
******************************************************************************/
tA2D_STATUS A2DP_ParseInfo(uint8_t* p_cie, const uint8_t* p_codec_info,
                           bool is_capability);

/******************************************************************************
**
** Function         A2DP_IsPeerSinkCodecValid
**
** Description      Checks whether the codec capabilities contain a valid peer A2DP Sink codec.
**                  NOTE: only codecs that are implemented are considered valid.
**
**                      p_codec_info:  Contains information about the codec capabilities.
**
** Returns         true if |p_codec_info| contains information about a valid codec,
**                 otherwise false.
**
******************************************************************************/
bool A2DP_IsPeerSinkCodecValid(const uint8_t* p_codec_info);

/******************************************************************************
**
** Function         A2DP_IsSinkCodecSupported
**
** Description      Checks whether an A2DP Sink codec is supported.
**
**                      p_codec_info:  Contains information about the codec capabilities.
**
** Returns         Returns true if the A2DP Sink codec is supported, otherwise false.
**
******************************************************************************/
tA2D_STATUS A2DP_IsSinkCodecSupported(const uint8_t* p_codec_info);

/******************************************************************************
**
** Function         A2DP_IsPeerSourceCodecSupported
**
** Description      Checks whether an A2DP Source codec for a peer Source device is supported.
**
**                      p_codec_info:  contains information about the codec capabilities of the
**                                     peer device.
**
** Returns         Returns true if the A2DP Source codec for a peer Source device is supported,
**                 otherwise false.
**
******************************************************************************/
tA2D_STATUS A2DP_IsPeerSourceCodecSupported(const uint8_t* p_codec_info);

/******************************************************************************
**
** Function         A2DP_GetDecoderInterface
**
** Description      Gets the A2DP decoder interface that can be used to decode received A2DP
**                  packets - see |tA2DP_DECODER_INTERFACE|.
**
**                      p_codec_info:  contains the codec information.
**
** Returns          the A2DP decoder interface if the |p_codec_info| is valid and
**                  supported, otherwise NULL.
**
******************************************************************************/
const tA2DP_DECODER_INTERFACE* A2DP_GetDecoderInterface(
    const uint8_t* p_codec_info);

/******************************************************************************
**
** Function         A2DP_SinkCodecIndex
**
** Description      Gets the A2DP Sink codec index for a given |p_codec_info|.
**
**                      p_codec_info:  Contains information about the codec capabilities.
**
** Returns          Returns the corresponding |btav_a2dp_codec_index_t| on success,
**                  otherwise |BTAV_A2DP_CODEC_INDEX_MAX|.
**
******************************************************************************/
btav_a2dp_codec_index_t A2DP_SinkCodecIndex(const uint8_t* p_codec_info);

/******************************************************************************
**
** Function         A2DP_SourceCodecIndex
**
** Description      Gets the A2DP Source codec index for a given |p_codec_info|.
**
**                      p_codec_info:  Contains information about the codec capabilities.
**
** Returns          Returns the corresponding |btav_a2dp_codec_index_t| on success,
**                  otherwise |BTAV_A2DP_CODEC_INDEX_MAX|.
**
******************************************************************************/
btav_a2dp_codec_index_t A2DP_SourceCodecIndex(const uint8_t* p_codec_info);

/******************************************************************************
**
** Function         A2DP_InitCodecConfig
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
bool A2DP_InitCodecConfig(btav_a2dp_codec_index_t codec_index, UINT8 *p_result);

/******************************************************************************
**
** Function         A2DP_BuildCodecConfig
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
bool A2DP_BuildCodecConfig(UINT8 *p_src_cap, UINT8 *p_result);

/******************************************************************************
**
** Function         A2DP_CodecName
**
** Description      Gets the A2DP codec name for a given |p_codec_info|.
**
**                      p_codec_info:  Contains information about the codec capabilities.
**
** Returns          Codec name
**
******************************************************************************/
const char* A2DP_CodecName(const uint8_t* p_codec_info);

/******************************************************************************
**
** Function
**
** Description      Checks whether two A2DP codecs |p_codec_info_a| and |p_codec_info_b| have
**                  the same type.
**
**                      p_codec_info_a:  Contains information about the codec capabilities.
**
**                      p_codec_info_b:  Contains information about the codec capabilities.
**
** Returns          true if equal, otherwise false.
**
******************************************************************************/
bool A2DP_CodecTypeEquals(const uint8_t* p_codec_info_a,
                          const uint8_t* p_codec_info_b);

#endif  // A2DP_CODEC_API_H
