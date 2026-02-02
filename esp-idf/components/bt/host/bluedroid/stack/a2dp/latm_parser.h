/*
 * SPDX-FileCopyrightText: 2024 ESP32 A2DP Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * Lightweight LATM Parser for A2DP AAC
 * 
 * Parses LATM (Low-overhead MPEG-4 Audio Transport Multiplex) headers
 * used in A2DP AAC streams. Extracts raw AAC frames for decoding.
 * 
 * Based on ISO/IEC 14496-3 LATM specification.
 * 
 * Memory footprint: ~200 bytes for parser state
 */

#ifndef LATM_PARSER_H
#define LATM_PARSER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* LATM Parser Error Codes */
typedef enum {
    LATM_OK = 0,
    LATM_ERR_NULL_POINTER = -1,
    LATM_ERR_NOT_ENOUGH_DATA = -2,
    LATM_ERR_INVALID_SYNC = -3,
    LATM_ERR_UNSUPPORTED_CONFIG = -4,
    LATM_ERR_INVALID_FRAME = -5,
    LATM_ERR_BUFFER_TOO_SMALL = -6,
} latm_error_t;

/* Audio Specific Config extracted from LATM */
typedef struct {
    uint8_t  object_type;       /* AAC object type (1=Main, 2=LC, 3=SSR, 4=LTP) */
    uint32_t sample_rate;       /* Sample rate in Hz */
    uint8_t  channel_config;    /* Channel configuration (1=mono, 2=stereo) */
    uint8_t  frame_length_flag; /* 0=1024 samples, 1=960 samples */
    bool     config_valid;      /* True if config has been parsed */
} latm_audio_config_t;

/* LATM Frame Info */
typedef struct {
    uint32_t frame_length_bits;  /* Length of AAC frame in bits */
    uint32_t frame_length_bytes; /* Length of AAC frame in bytes (ceil) */
    uint32_t bit_offset;         /* Bit offset where AAC frame starts */
    bool     is_byte_aligned;    /* True if frame starts on byte boundary */
} latm_frame_info_t;

/* LATM Parser Context - minimal state */
typedef struct {
    /* StreamMuxConfig state */
    uint8_t  audio_mux_version;
    uint8_t  all_streams_same_time;
    uint8_t  num_sub_frames;
    uint8_t  num_program;
    uint8_t  num_layer;
    uint8_t  frame_length_type;
    uint32_t frame_length;        /* Fixed frame length for type 0 */
    
    /* Audio Specific Config */
    latm_audio_config_t audio_config;
    
    /* Parser state */
    bool     config_parsed;
    bool     use_same_stream_mux;
    
    /* Cached StreamMuxConfig for reuse */
    uint8_t  stream_mux_config[64];
    uint8_t  stream_mux_config_len;
} latm_parser_t;

/**
 * @brief Initialize LATM parser
 * @param parser Pointer to parser context
 * @return LATM_OK on success
 */
latm_error_t latm_parser_init(latm_parser_t *parser);

/**
 * @brief Reset LATM parser state
 * @param parser Pointer to parser context
 */
void latm_parser_reset(latm_parser_t *parser);

/**
 * @brief Parse LATM frame and extract raw AAC frame
 * 
 * This function parses the LATM header (AudioMuxElement) and returns
 * the position and length of the raw AAC frame data.
 * 
 * For A2DP, the LATM format is MCP1 (Multiplex Configuration Protocol 1)
 * which uses inband StreamMuxConfig signaling.
 * 
 * @param parser    Parser context
 * @param data      Input LATM data
 * @param data_len  Length of input data in bytes
 * @param aac_out   Output buffer for raw AAC frame (can be same as data for in-place)
 * @param aac_out_size Size of output buffer
 * @param aac_len   [out] Length of raw AAC frame
 * @param frame_info [out] Frame info (optional, can be NULL)
 * @return LATM_OK on success, error code otherwise
 */
latm_error_t latm_parse_frame(latm_parser_t *parser,
                               const uint8_t *data, size_t data_len,
                               uint8_t *aac_out, size_t aac_out_size,
                               size_t *aac_len,
                               latm_frame_info_t *frame_info);

/**
 * @brief Get audio configuration from parser
 * @param parser Parser context
 * @return Pointer to audio config, or NULL if not yet parsed
 */
const latm_audio_config_t *latm_get_audio_config(const latm_parser_t *parser);

/**
 * @brief Check if configuration is valid
 * @param parser Parser context
 * @return true if config has been parsed
 */
bool latm_config_valid(const latm_parser_t *parser);

#ifdef __cplusplus
}
#endif

#endif /* LATM_PARSER_H */
