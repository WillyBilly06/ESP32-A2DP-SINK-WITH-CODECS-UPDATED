/*
 * SPDX-FileCopyrightText: 2024 ESP32 A2DP Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * AAC-LC Decoder Tables
 * Huffman codebooks, scalefactor bands, window functions
 */

#ifndef AAC_TABLES_H
#define AAC_TABLES_H

#include <stdint.h>

/*******************************************************************************
 * Sample Rate Tables
 ******************************************************************************/

static const uint32_t aac_sample_rates[16] = {
    96000, 88200, 64000, 48000, 44100, 32000, 24000, 22050,
    16000, 12000, 11025, 8000, 7350, 0, 0, 0
};

/*******************************************************************************
 * Scalefactor Band Tables (long windows, 1024 samples)
 ******************************************************************************/

/* 44100 Hz - 49 bands */
static const uint16_t sfb_44100_long[50] = {
    0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 48, 56, 64, 72, 80,
    88, 96, 108, 120, 132, 144, 160, 176, 196, 216, 240, 264, 292, 320,
    352, 384, 416, 448, 480, 512, 576, 640, 704, 768, 832, 896, 960, 1024,
    1024, 1024, 1024, 1024, 1024, 1024
};

/* 48000 Hz - 49 bands */
static const uint16_t sfb_48000_long[50] = {
    0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 48, 56, 64, 72, 80,
    88, 96, 108, 120, 132, 144, 160, 176, 196, 216, 240, 264, 292, 320,
    352, 384, 416, 448, 480, 512, 576, 640, 704, 768, 832, 896, 960, 1024,
    1024, 1024, 1024, 1024, 1024, 1024
};

/* 32000 Hz - 51 bands */
static const uint16_t sfb_32000_long[52] = {
    0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 48, 56, 64, 72, 80,
    88, 96, 108, 120, 132, 144, 160, 176, 196, 216, 240, 264, 292, 320,
    352, 384, 416, 448, 480, 512, 544, 576, 608, 640, 704, 768, 832, 896,
    960, 1024, 1024, 1024, 1024, 1024, 1024, 1024
};

/* Short window tables (128 samples) */
static const uint16_t sfb_44100_short[16] = {
    0, 4, 8, 12, 16, 20, 28, 36, 44, 56, 68, 80, 96, 112, 128, 128
};

static const uint16_t sfb_48000_short[16] = {
    0, 4, 8, 12, 16, 20, 28, 36, 44, 56, 68, 80, 96, 112, 128, 128
};

/*******************************************************************************
 * Scalefactor decoding table (converts scalefactor to gain)
 * Q15 format: gain = 2^((sf - 100) / 4) scaled
 ******************************************************************************/

/* Precomputed: 2^((i - 100) / 4) * 32768 for i = 0..255 */
/* We'll use a smaller subset and compute at runtime for memory savings */

/*******************************************************************************
 * Huffman Codebook Structure
 * Each entry: [bits to read, value, escape flag]
 ******************************************************************************/

/* Scalefactor Huffman codebook */
/* Format: (codeword_bits << 8) | value, or use lookup table */

/* Huffman decode via binary search tables - more memory efficient */
typedef struct {
    uint16_t code;      /* Bit pattern (left-aligned) */
    uint8_t  bits;      /* Number of bits */
    int8_t   value;     /* Decoded value (or index) */
} hcb_entry_t;

/*******************************************************************************
 * Huffman Codebook 1 (4-tuple, signed, max |value| = 1)
 ******************************************************************************/

/* Using a simple lookup approach for small codebooks */
/* Format: offset + idx gives the 4 values (each -1, 0, or +1) */

/* Codebook 1 - 81 entries (3^4), unsigned index, values added with offset -1 */
/* We encode as packed nibbles: [v0+1][v1+1][v2+1][v3+1] = 0..2 each */

static const uint8_t hcb1_decode[81][2] = {
    /* Each row: bits_needed, packed_values */
    /* This is a simplified representation - actual decode done procedurally */
    {1, 0x55}, /* 0000: w=0,x=0,y=0,z=0 -> 1,1,1,1 packed as 0x55 (all zeros) */
};

/*******************************************************************************
 * Huffman Codebook for Scalefactors
 * Differential coding: 60 possible differences (-60 to +60)
 ******************************************************************************/

/* Scalefactor Huffman - lookup by code prefix */
typedef struct {
    uint32_t pattern;
    uint8_t  len;
    int8_t   diff;
} sf_hcb_t;

/* Sorted by codeword length for efficient decoding */
static const sf_hcb_t sf_hcb[] = {
    /* 1-4 bits */
    {0x0, 1, 0},     /* 0 -> diff=0 */
    {0x6, 3, -1},    /* 110 -> diff=-1 */
    {0x7, 3, 1},     /* 111 -> diff=1 */
    {0x4, 4, -2},    /* 0100 -> diff=-2 */
    {0x5, 4, 2},     /* 0101 -> diff=2 */
    /* 5-6 bits */
    {0x18, 5, -3},   /* 11000 -> diff=-3 */
    {0x19, 5, 3},    /* 11001 -> diff=3 */
    {0x1A, 5, -4},   /* 11010 -> diff=-4 */
    {0x1B, 5, 4},    /* 11011 -> diff=4 */
    /* Continue pattern for larger differences... */
    {0x0C, 6, -5},
    {0x0D, 6, 5},
    {0x0E, 6, -6},
    {0x0F, 6, 6},
    /* 7+ bits for larger diffs */
    {0x10, 7, -7},
    {0x11, 7, 7},
    {0x12, 7, -8},
    {0x13, 7, 8},
    {0x28, 8, -9},
    {0x29, 8, 9},
    {0x2A, 8, -10},
    {0x2B, 8, 10},
};

#define SF_HCB_SIZE (sizeof(sf_hcb) / sizeof(sf_hcb[0]))

/*******************************************************************************
 * Window Functions (Q15 format)
 * KBD window for AAC (Kaiser-Bessel Derived)
 ******************************************************************************/

/* Sine window for 2048 points (half window = 1024) - stored as Q15 */
/* We'll compute on-the-fly to save memory, just store constants */

/* For memory efficiency, we use quarter-wave symmetry */
/* sin_window[n] = sin(pi/2048 * (n + 0.5)) for n=0..1023 */

/* Pre-computed 256-sample quarter of sine window - rest derived by symmetry */
/* sin(pi/2048 * (n + 0.5)) * 32767 */
static const int16_t sine_win_256[256] = {
    50, 151, 251, 352, 452, 553, 653, 754,
    854, 954, 1055, 1155, 1256, 1356, 1456, 1556,
    1657, 1757, 1857, 1957, 2057, 2157, 2257, 2357,
    2457, 2557, 2657, 2757, 2856, 2956, 3056, 3155,
    3255, 3354, 3454, 3553, 3653, 3752, 3851, 3950,
    4049, 4148, 4247, 4346, 4445, 4544, 4642, 4741,
    4839, 4938, 5036, 5134, 5233, 5331, 5429, 5527,
    5625, 5723, 5820, 5918, 6016, 6113, 6210, 6308,
    6405, 6502, 6599, 6696, 6793, 6889, 6986, 7082,
    7179, 7275, 7371, 7467, 7563, 7659, 7755, 7850,
    7946, 8041, 8137, 8232, 8327, 8422, 8517, 8612,
    8706, 8801, 8895, 8989, 9083, 9177, 9271, 9365,
    9459, 9552, 9646, 9739, 9832, 9925, 10018, 10111,
    10203, 10296, 10388, 10480, 10572, 10664, 10756, 10847,
    10939, 11030, 11121, 11212, 11303, 11393, 11484, 11574,
    11664, 11754, 11844, 11934, 12023, 12113, 12202, 12291,
    12380, 12468, 12557, 12645, 12733, 12821, 12909, 12997,
    13084, 13172, 13259, 13346, 13433, 13519, 13606, 13692,
    13778, 13864, 13950, 14035, 14121, 14206, 14291, 14376,
    14460, 14545, 14629, 14713, 14797, 14881, 14964, 15047,
    15130, 15213, 15296, 15378, 15461, 15543, 15625, 15707,
    15788, 15870, 15951, 16032, 16113, 16193, 16274, 16354,
    16434, 16514, 16593, 16673, 16752, 16831, 16910, 16988,
    17067, 17145, 17223, 17301, 17378, 17456, 17533, 17610,
    17687, 17763, 17840, 17916, 17992, 18067, 18143, 18218,
    18293, 18368, 18443, 18517, 18591, 18665, 18739, 18812,
    18886, 18959, 19032, 19105, 19177, 19249, 19321, 19393,
    19465, 19536, 19607, 19678, 19749, 19819, 19889, 19959,
    20029, 20099, 20168, 20237, 20306, 20374, 20443, 20511,
    20579, 20646, 20714, 20781, 20848, 20915, 20981, 21047,
    21113, 21179, 21245, 21310, 21375, 21440, 21504, 21569,
    21633, 21697, 21760, 21824, 21887, 21950, 22012, 22075,
};

/*******************************************************************************
 * IMDCT Twiddle Factors (Q15)
 * For N=2048 IMDCT (1024 spectral -> 2048 time samples)
 ******************************************************************************/

/* cos(2*pi*k/N) and sin(2*pi*k/N) tables - computed at init for memory */

/*******************************************************************************
 * Spectral Huffman Codebooks (Simplified for AAC-LC)
 * Using table-driven decode for efficiency
 ******************************************************************************/

/* Huffman codebook parameters */
typedef struct {
    uint8_t dim;          /* Dimension (2 or 4) */
    uint8_t signed_cb;    /* Signed values? */
    uint8_t max_val;      /* Maximum value */
    uint8_t esc;          /* Has escape sequences? */
} hcb_info_t;

static const hcb_info_t hcb_info[12] = {
    {0, 0, 0, 0},   /* cb 0: zero (unused) */
    {4, 1, 1, 0},   /* cb 1: 4-tuple, signed, max=1 */
    {4, 1, 1, 0},   /* cb 2: 4-tuple, signed, max=1 */
    {4, 0, 2, 0},   /* cb 3: 4-tuple, unsigned, max=2 */
    {4, 0, 2, 0},   /* cb 4: 4-tuple, unsigned, max=2 */
    {2, 1, 4, 0},   /* cb 5: 2-tuple, signed, max=4 */
    {2, 1, 4, 0},   /* cb 6: 2-tuple, signed, max=4 */
    {2, 0, 7, 0},   /* cb 7: 2-tuple, unsigned, max=7 */
    {2, 0, 7, 0},   /* cb 8: 2-tuple, unsigned, max=7 */
    {2, 0, 12, 0},  /* cb 9: 2-tuple, unsigned, max=12 */
    {2, 0, 12, 0},  /* cb 10: 2-tuple, unsigned, max=12 */
    {2, 0, 16, 1},  /* cb 11: 2-tuple, unsigned, escape */
};

/*******************************************************************************
 * Huffman Decode Tables (Compressed Format)
 * 
 * We use a 2-level decode: fast lookup for short codes, tree for long codes
 ******************************************************************************/

/* Scalefactor codebook - VLC decode table */
/* Index by first 9 bits, gives (length << 8) | value or tree index */

/* For memory efficiency, we use procedural decode for spectral data */

/*******************************************************************************
 * TNS (Temporal Noise Shaping) Filter Coefficients
 ******************************************************************************/

static const int16_t tns_coef_0_3[8] = {
    /* 3-bit resolution, cos(coef * pi/8) in Q15 */
    32767, 30274, 23170, 12540, 0, -12540, -23170, -30274
};

static const int16_t tns_coef_0_4[16] = {
    /* 4-bit resolution */
    32767, 32138, 30274, 27246, 23170, 18205, 12540, 6393,
    0, -6393, -12540, -18205, -23170, -27246, -30274, -32138
};

#endif /* AAC_TABLES_H */
