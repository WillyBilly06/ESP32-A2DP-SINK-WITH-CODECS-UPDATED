/*
 * SPDX-License-Identifier: Apache-2.0
 */

//
// A2DP constants for Opus codec
//

#ifndef A2DP_VENDOR_OPUS_CONSTANTS_H
#define A2DP_VENDOR_OPUS_CONSTANTS_H

// [Octet 0-3] Vendor ID
#define A2DP_OPUS_VENDOR_ID 0x000005f1
// [Octet 4-5] Vendor Specific Codec ID
#define A2DP_OPUS_CODEC_ID 0x1005

// Opus codec specific settings
#define A2DP_OPUS_CODEC_LEN 26

// Audio Location Configuration
#define A2DP_OPUS_AUDIO_LOCATION_FL 0x00000001
#define A2DP_OPUS_AUDIO_LOCATION_FR 0x00000002

// Limits Configuration
#define A2DP_OPUS_FRAME_DURATION_2_5 0x01
#define A2DP_OPUS_FRAME_DURATION_5_0 0x02
#define A2DP_OPUS_FRAME_DURATION_10  0x04
#define A2DP_OPUS_FRAME_DURATION_20  0x08
#define A2DP_OPUS_FRAME_DURATION_40  0x010

#define A2DP_OPUS_FRAME_BITRATE_ALL 0x0

#endif  // A2DP_VENDOR_OPUS_CONSTANTS_H
