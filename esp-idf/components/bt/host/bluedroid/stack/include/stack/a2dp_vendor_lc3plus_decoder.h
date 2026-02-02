/*
 * SPDX-License-Identifier: Apache-2.0
 */

//
// Interface to the A2DP Lc3Plus Decoder
//

#ifndef A2DP_VENDOR_LC3PLUS_DECODER_H
#define A2DP_VENDOR_LC3PLUS_DECODER_H

#include "a2dp_codec_api.h"
#include "a2dp_decoder.h"
#include "stack/bt_types.h"

/*****************************************************************************
**  External Function Declarations
*****************************************************************************/
#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************
**
** Function         a2dp_lc3plus_decoder_init
**
** Description      Initialize the A2DP Lc3Plus decoder
**
**                      decode_callback:  Callback function after decode completion
**
** Returns          true on success, false otherwise
**
******************************************************************************/
bool a2dp_lc3plus_decoder_init(decoded_data_callback_t decode_callback);

/******************************************************************************
**
** Function         a2dp_lc3plus_decoder_cleanup
**
** Description      Cleanup the A2DP Lc3Plus decoder.
**
******************************************************************************/
void a2dp_lc3plus_decoder_cleanup(void);

/******************************************************************************
**
** Function         a2dp_lc3plus_decoder_configure
**
** Description      Configure the Lc3Plus decoder.
**
**                      p_codec_info:  Codec capabilities
**
******************************************************************************/
void a2dp_lc3plus_decoder_configure(const uint8_t* p_codec_info);

/******************************************************************************
**
** Function         a2dp_lc3plus_decoder_decode_packet_header
**
** Description      Decode the Lc3Plus packet header.
**
** Returns          
**
******************************************************************************/
ssize_t a2dp_lc3plus_decoder_decode_packet_header(BT_HDR* p_data);

/******************************************************************************
**
** Function         a2dp_lc3plus_decoder_decode_packet
**
** Description      Decodes |p_buf|. Calls |decode_callback| passed into |a2dp_lc3plus_decoder_init|
**                  if decoded frames are available.
**
**                      p_buf:  Packet data
**
** Returns          true on success, false otherwise
**
******************************************************************************/
bool a2dp_lc3plus_decoder_decode_packet(BT_HDR* p_buf, unsigned char* buf, size_t buf_len);

#ifdef __cplusplus
}
#endif

#endif  // A2DP_VENDOR_LC3PLUS_DECODER_H
