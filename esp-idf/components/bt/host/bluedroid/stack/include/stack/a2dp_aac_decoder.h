/*
 * SPDX-FileCopyrightText: 2016 The Android Open Source Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

//
// Interface to the A2DP AAC Decoder
//

#ifndef A2DP_AAC_DECODER_H
#define A2DP_AAC_DECODER_H

#include "a2dp_codec_api.h"
#include "stack/bt_types.h"

// Initialize the A2DP AAC decoder.
bool a2dp_aac_decoder_init(decoded_data_callback_t decode_callback);

// Cleanup the A2DP AAC decoder.
void a2dp_aac_decoder_cleanup(void);

bool a2dp_aac_decoder_reset(void);

ssize_t a2dp_aac_decoder_decode_packet_header(BT_HDR* p_data);

// Decodes |p_buf|. Calls |decode_callback| passed into |a2dp_aac_decoder_init|
// if decoded frames are available.
bool a2dp_aac_decoder_decode_packet(BT_HDR* p_buf, unsigned char* buf, size_t buf_len);

void a2dp_aac_decoder_configure(const uint8_t* p_codec_info);

#endif  // A2DP_AAC_DECODER_H
