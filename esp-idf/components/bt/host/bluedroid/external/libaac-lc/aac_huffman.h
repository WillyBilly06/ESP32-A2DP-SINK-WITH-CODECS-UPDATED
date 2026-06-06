/*
 * AAC Spectral Huffman Decoder
 * Complete Huffman tables for AAC-LC spectral data decoding
 */

#ifndef AAC_HUFFMAN_H
#define AAC_HUFFMAN_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Huffman codebook types */
#define HCB_ZERO       0   /* Zero spectral coefficients */
#define HCB_1          1   /* 4-tuples, signed, no ESC */
#define HCB_2          2   /* 4-tuples, signed, no ESC */
#define HCB_3          3   /* 4-tuples, unsigned, no ESC */
#define HCB_4          4   /* 4-tuples, unsigned, no ESC */
#define HCB_5          5   /* 2-tuples, signed, no ESC */
#define HCB_6          6   /* 2-tuples, signed, no ESC */
#define HCB_7          7   /* 2-tuples, unsigned, no ESC */
#define HCB_8          8   /* 2-tuples, unsigned, no ESC */
#define HCB_9          9   /* 2-tuples, unsigned, no ESC */
#define HCB_10        10   /* 2-tuples, unsigned, no ESC */
#define HCB_11        11   /* 2-tuples, unsigned, with ESC */
#define HCB_RESERVED  12   /* Reserved */
#define HCB_NOISE     13   /* Noise substitution */
#define HCB_INTENSITY 14   /* Intensity stereo */
#define HCB_INTENSITY2 15  /* Intensity stereo 2 */

/*******************************************************************************
 * Huffman Decode Functions
 ******************************************************************************/

/**
 * Decode scalefactor difference
 * @param data Bitstream data
 * @param bit_pos Current bit position (updated on return)
 * @param max_bits Maximum bits available
 * @return Decoded value (-60 to +60)
 */
int aac_huff_decode_sf(const uint8_t *data, int *bit_pos, int max_bits);

/**
 * Decode spectral quad (codebooks 1-4)
 * @param cb Codebook number (1-4)
 * @param data Bitstream data
 * @param bit_pos Current bit position (updated on return)
 * @param max_bits Maximum bits available
 * @param vals Output array for 4 decoded values
 * @return 1 on success, 0 on failure
 */
int aac_huff_decode_quad(int cb, const uint8_t *data, int *bit_pos, int max_bits, int vals[4]);

/**
 * Decode spectral pair (codebooks 5-11)
 * @param cb Codebook number (5-11)
 * @param data Bitstream data
 * @param bit_pos Current bit position (updated on return)
 * @param max_bits Maximum bits available
 * @param vals Output array for 2 decoded values
 * @return 1 on success, 0 on failure
 */
int aac_huff_decode_pair(int cb, const uint8_t *data, int *bit_pos, int max_bits, int vals[2]);

#ifdef __cplusplus
}
#endif

#endif /* AAC_HUFFMAN_H */
