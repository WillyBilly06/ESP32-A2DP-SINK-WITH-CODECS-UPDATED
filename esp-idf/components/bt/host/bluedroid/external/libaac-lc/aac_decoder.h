/*
 * SPDX-FileCopyrightText: 2024 ESP32 A2DP Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * AAC-LC Decoder Interface for ESP32
 * Uses libhelix-aac with static memory allocation (~27KB static buffer)
 */

#ifndef AAC_DECODER_H
#define AAC_DECODER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Constants
 ******************************************************************************/

#define AAC_MAX_CHANNELS        2
#define AAC_FRAME_LEN           1024

/*******************************************************************************
 * Error Codes
 ******************************************************************************/

#define AAC_OK              0
#define AAC_ERR_PARAM       -1
#define AAC_ERR_DATA        -2
#define AAC_ERR_UNSUPPORTED -3
#define AAC_ERR_NOMEM       -4
#define AAC_ERR_BITSTREAM   -5

/*******************************************************************************
 * Decoder State Structure - lightweight wrapper
 * 
 * The actual decoder state is in static buffers inside aac_decoder_helix.c
 * This struct is just for configuration and statistics (~16 bytes)
 ******************************************************************************/

typedef struct {
    /* Configuration */
    int sample_rate;
    int sample_rate_idx;
    int channels;
    
    /* Statistics */
    uint32_t frame_count;
    uint32_t error_count;
    
} aac_decoder_t;

/*******************************************************************************
 * Public API
 ******************************************************************************/

/**
 * Initialize decoder
 */
int aac_decoder_init(aac_decoder_t *dec);

/**
 * Set configuration
 */
int aac_decoder_configure(aac_decoder_t *dec, int sample_rate, int channels);

/**
 * Decode one raw AAC frame
 * @param dec       Decoder instance
 * @param data      Raw AAC frame data (not ADTS/LATM)
 * @param len       Length of data in bytes
 * @param pcm_out   Output buffer for interleaved PCM (must hold at least 1024*2 samples)
 * @param samples_out  Receives number of samples per channel
 */
int aac_decoder_decode(aac_decoder_t *dec,
                       const uint8_t *data, int len,
                       int16_t *pcm_out, int *samples_out);

/**
 * Deinitialize decoder
 */
void aac_decoder_deinit(aac_decoder_t *dec);

#ifdef __cplusplus
}
#endif

#endif /* AAC_DECODER_H */
