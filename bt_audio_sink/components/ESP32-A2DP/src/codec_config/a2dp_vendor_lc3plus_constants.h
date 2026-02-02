/*
 * SPDX-License-Identifier: Apache-2.0
 */

//
// A2DP constants for LC3 Plus codec
//

#ifndef A2DP_VENDOR_LC3PLUS_CONSTANTS_H
#define A2DP_VENDOR_LC3PLUS_CONSTANTS_H

// [Octet 0-3] Vendor ID
#define A2DP_LC3PLUS_VENDOR_ID 0x000008a9
// [Octet 4-5] Vendor Specific Codec ID
#define A2DP_LC3PLUS_CODEC_ID 0x0001

// Lc3Plus codec specific settings
#define A2DP_LC3PLUS_CODEC_LEN 12

#define A2DP_LC3PLUS_FRAME_DURATION_10MS  (1 << 6)
#define A2DP_LC3PLUS_FRAME_DURATION_5MS   (1 << 5)
#define A2DP_LC3PLUS_FRAME_DURATION_2_5MS (1 << 4)

#define A2DP_LC3PLUS_CHANNELS_2       (1 << 6)
#define A2DP_LC3PLUS_CHANNELS_1       (1 << 7)

#define A2DP_LC3PLUS_SAMPLING_FREQ_96000  (1 << 7)
#define A2DP_LC3PLUS_SAMPLING_FREQ_48000  (1 << 8)

#endif  // A2DP_VENDOR_LC3PLUS_CONSTANTS_H
