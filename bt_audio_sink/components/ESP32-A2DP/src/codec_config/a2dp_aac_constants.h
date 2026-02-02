/*
 * SPDX-FileCopyrightText: 2016 The Android Open Source Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

//
// A2DP constants for AAC codec
//

#ifndef A2DP_AAC_CONSTANTS_H
#define A2DP_AAC_CONSTANTS_H

// AAC codec specific settings
#define A2DP_AAC_CODEC_LEN 8

// [Octet 0] Object Type
#define A2DP_AAC_OBJECT_TYPE_MPEG2_LC 0x80  /* MPEG-2 Low Complexity */
#define A2DP_AAC_OBJECT_TYPE_MPEG4_LC 0x40  /* MPEG-4 Low Complexity */
#define A2DP_AAC_OBJECT_TYPE_MPEG4_LTP 0x20 /* MPEG-4 Long Term Prediction */
#define A2DP_AAC_OBJECT_TYPE_MPEG4_SCALABLE 0x10
// [Octet 1] Sampling Frequency - 8000 to 44100
#define A2DP_AAC_SAMPLING_FREQ_MASK0 0xFF
#define A2DP_AAC_SAMPLING_FREQ_8000 0x80
#define A2DP_AAC_SAMPLING_FREQ_11025 0x40
#define A2DP_AAC_SAMPLING_FREQ_12000 0x20
#define A2DP_AAC_SAMPLING_FREQ_16000 0x10
#define A2DP_AAC_SAMPLING_FREQ_22050 0x08
#define A2DP_AAC_SAMPLING_FREQ_24000 0x04
#define A2DP_AAC_SAMPLING_FREQ_32000 0x02
#define A2DP_AAC_SAMPLING_FREQ_44100 0x01
// [Octet 2], [Bits 4-7] Sampling Frequency - 48000 to 96000
// NOTE: Bits offset for the higher-order octet 16-bit integer
#define A2DP_AAC_SAMPLING_FREQ_MASK1 (0xF0 << 8)
#define A2DP_AAC_SAMPLING_FREQ_48000 (0x80 << 8)
#define A2DP_AAC_SAMPLING_FREQ_64000 (0x40 << 8)
#define A2DP_AAC_SAMPLING_FREQ_88200 (0x20 << 8)
#define A2DP_AAC_SAMPLING_FREQ_96000 (0x10 << 8)
// [Octet 2], [Bits 2-3] Channel Mode
#define A2DP_AAC_CHANNEL_MODE_MASK 0x0C
#define A2DP_AAC_CHANNEL_MODE_MONO 0x08
#define A2DP_AAC_CHANNEL_MODE_STEREO 0x04
// [Octet 2], [Bits 0-1] RFA
// [Octet 3], [Bit 7] Variable Bit Rate Supported
#define A2DP_AAC_VARIABLE_BIT_RATE_MASK 0x80
#define A2DP_AAC_VARIABLE_BIT_RATE_ENABLED 0x80
#define A2DP_AAC_VARIABLE_BIT_RATE_DISABLED 0x00
// [Octet 3], [Bits 0-6] Bit Rate - Bits 16-22 in the 23-bit UiMsbf
#define A2DP_AAC_BIT_RATE_MASK0 (0x7F << 16)
#define A2DP_AAC_BIT_RATE_MASK1 (0xFF << 8)
#define A2DP_AAC_BIT_RATE_MASK2 0xFF
// [Octet 4], [Bits 0-7] Bit Rate - Bits 8-15 in the 23-bit UiMsfb
// [Octet 5], [Bits 0-7] Bit Rate - Bits 0-7 in the 23-bit UiMsfb

#endif  // A2DP_AAC_CONSTANTS_H
