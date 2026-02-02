/*
 * SPDX-FileCopyrightText: 2024 ESP32 A2DP Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * Lightweight LATM Parser for A2DP AAC
 * 
 * Based on ISO/IEC 14496-3 LATM specification.
 * Optimized for minimal memory usage on ESP32.
 */

#include "latm_parser.h"
#include <string.h>

/* Sample rate table for AAC (index 0-12) */
static const uint32_t sample_rate_table[13] = {
    96000, 88200, 64000, 48000, 44100, 32000,
    24000, 22050, 16000, 12000, 11025, 8000, 7350
};

/*******************************************************************************
 * Bitstream Reader - reads bits from byte buffer
 ******************************************************************************/
typedef struct {
    const uint8_t *data;
    size_t data_len;
    size_t bit_pos;
} bitstream_t;

static inline void bs_init(bitstream_t *bs, const uint8_t *data, size_t len)
{
    bs->data = data;
    bs->data_len = len;
    bs->bit_pos = 0;
}

static inline size_t bs_bits_left(const bitstream_t *bs)
{
    return (bs->data_len * 8) - bs->bit_pos;
}

static inline uint32_t bs_read_bits(bitstream_t *bs, uint8_t n)
{
    if (n == 0 || n > 32) return 0;
    if (bs_bits_left(bs) < n) return 0;
    
    uint32_t result = 0;
    while (n > 0) {
        size_t byte_pos = bs->bit_pos >> 3;
        size_t bit_in_byte = bs->bit_pos & 7;
        size_t bits_in_this_byte = 8 - bit_in_byte;
        
        if (bits_in_this_byte > n) bits_in_this_byte = n;
        
        uint8_t mask = (1 << bits_in_this_byte) - 1;
        uint8_t shift = 8 - bit_in_byte - bits_in_this_byte;
        uint8_t val = (bs->data[byte_pos] >> shift) & mask;
        
        result = (result << bits_in_this_byte) | val;
        bs->bit_pos += bits_in_this_byte;
        n -= bits_in_this_byte;
    }
    return result;
}

static inline uint8_t bs_read_bit(bitstream_t *bs)
{
    return (uint8_t)bs_read_bits(bs, 1);
}

static inline size_t bs_get_bit_pos(const bitstream_t *bs)
{
    return bs->bit_pos;
}

static inline void bs_skip_bits(bitstream_t *bs, size_t n)
{
    bs->bit_pos += n;
    if (bs->bit_pos > bs->data_len * 8) {
        bs->bit_pos = bs->data_len * 8;
    }
}

/*******************************************************************************
 * LATM Value reading (variable length encoding)
 ******************************************************************************/
static uint32_t latm_get_value(bitstream_t *bs)
{
    uint8_t bytes_for_value = (uint8_t)bs_read_bits(bs, 2);
    uint32_t value = 0;
    
    for (uint8_t i = 0; i <= bytes_for_value; i++) {
        value = (value << 8) | bs_read_bits(bs, 8);
    }
    return value;
}

/*******************************************************************************
 * Parse AudioSpecificConfig (GASpecificConfig for AAC-LC)
 ******************************************************************************/
static latm_error_t parse_audio_specific_config(bitstream_t *bs, latm_audio_config_t *config)
{
    /* audioObjectType (5 bits, may extend to 11) */
    uint8_t audio_object_type = (uint8_t)bs_read_bits(bs, 5);
    if (audio_object_type == 31) {
        audio_object_type = 32 + (uint8_t)bs_read_bits(bs, 6);
    }
    config->object_type = audio_object_type;
    
    /* samplingFrequencyIndex (4 bits) */
    uint8_t sf_index = (uint8_t)bs_read_bits(bs, 4);
    if (sf_index == 0x0F) {
        /* explicit sample rate (24 bits) */
        config->sample_rate = bs_read_bits(bs, 24);
    } else if (sf_index < 13) {
        config->sample_rate = sample_rate_table[sf_index];
    } else {
        return LATM_ERR_UNSUPPORTED_CONFIG;
    }
    
    /* channelConfiguration (4 bits) */
    config->channel_config = (uint8_t)bs_read_bits(bs, 4);
    
    /* For AAC-LC (objectType 2), parse GASpecificConfig */
    if (audio_object_type == 2) {
        /* frameLengthFlag */
        config->frame_length_flag = (uint8_t)bs_read_bit(bs);
        /* dependsOnCoreCoder */
        if (bs_read_bit(bs)) {
            /* coreCoderDelay */
            bs_skip_bits(bs, 14);
        }
        /* extensionFlag */
        bs_read_bit(bs);
    }
    
    config->config_valid = true;
    return LATM_OK;
}

/*******************************************************************************
 * Parse StreamMuxConfig
 ******************************************************************************/
static latm_error_t parse_stream_mux_config(bitstream_t *bs, latm_parser_t *parser)
{
    latm_error_t err;
    
    /* audioMuxVersion */
    parser->audio_mux_version = (uint8_t)bs_read_bit(bs);
    
    uint8_t audio_mux_version_a = 0;
    if (parser->audio_mux_version == 1) {
        audio_mux_version_a = (uint8_t)bs_read_bit(bs);
    }
    
    if (audio_mux_version_a != 0) {
        /* Future extension, not supported */
        return LATM_ERR_UNSUPPORTED_CONFIG;
    }
    
    uint32_t tara_buffer_fullness = 0;
    if (parser->audio_mux_version == 1) {
        tara_buffer_fullness = latm_get_value(bs);
        (void)tara_buffer_fullness;
    }
    
    /* allStreamsSameTimeFraming */
    parser->all_streams_same_time = (uint8_t)bs_read_bit(bs);
    
    /* numSubFrames */
    parser->num_sub_frames = (uint8_t)bs_read_bits(bs, 6) + 1;
    
    /* numProgram */
    parser->num_program = (uint8_t)bs_read_bits(bs, 4) + 1;
    
    /* For A2DP, we expect single program, single layer */
    if (parser->num_program != 1) {
        return LATM_ERR_UNSUPPORTED_CONFIG;
    }
    
    /* numLayer[0] */
    parser->num_layer = (uint8_t)bs_read_bits(bs, 3) + 1;
    if (parser->num_layer != 1) {
        return LATM_ERR_UNSUPPORTED_CONFIG;
    }
    
    /* AudioSpecificConfig */
    uint32_t asc_len = 0;
    if (parser->audio_mux_version == 0) {
        /* ASC is embedded directly */
        err = parse_audio_specific_config(bs, &parser->audio_config);
        if (err != LATM_OK) return err;
    } else {
        /* ascLen */
        asc_len = latm_get_value(bs);
        size_t asc_start = bs_get_bit_pos(bs);
        
        err = parse_audio_specific_config(bs, &parser->audio_config);
        if (err != LATM_OK) return err;
        
        /* Skip any remaining ASC bits */
        size_t asc_read = bs_get_bit_pos(bs) - asc_start;
        if (asc_read < asc_len) {
            bs_skip_bits(bs, asc_len - asc_read);
        }
    }
    
    /* frameLengthType */
    parser->frame_length_type = (uint8_t)bs_read_bits(bs, 3);
    
    if (parser->frame_length_type == 0) {
        /* latmBufferFullness (8 bits) - not needed */
        bs_read_bits(bs, 8);
    } else if (parser->frame_length_type == 1) {
        /* frameLength (9 bits) */
        parser->frame_length = bs_read_bits(bs, 9);
    } else {
        /* Types 3-7 not commonly used in A2DP */
        return LATM_ERR_UNSUPPORTED_CONFIG;
    }
    
    /* otherDataPresent */
    if (bs_read_bit(bs)) {
        /* otherDataLenBits - skip */
        if (parser->audio_mux_version == 1) {
            latm_get_value(bs);
        } else {
            /* otherDataLenEsc loop */
            uint8_t esc;
            do {
                esc = (uint8_t)bs_read_bit(bs);
                bs_skip_bits(bs, 8);
            } while (esc);
        }
    }
    
    /* crcCheckPresent */
    if (bs_read_bit(bs)) {
        /* crcCheckSum */
        bs_skip_bits(bs, 8);
    }
    
    parser->config_parsed = true;
    return LATM_OK;
}

/*******************************************************************************
 * Parse PayloadLengthInfo for frameLengthType 0
 ******************************************************************************/
static uint32_t parse_payload_length_info(bitstream_t *bs)
{
    uint32_t length = 0;
    uint8_t tmp;
    
    do {
        tmp = (uint8_t)bs_read_bits(bs, 8);
        length += tmp;
    } while (tmp == 255);
    
    return length;
}

/*******************************************************************************
 * Public API
 ******************************************************************************/

latm_error_t latm_parser_init(latm_parser_t *parser)
{
    if (parser == NULL) return LATM_ERR_NULL_POINTER;
    
    memset(parser, 0, sizeof(latm_parser_t));
    return LATM_OK;
}

void latm_parser_reset(latm_parser_t *parser)
{
    if (parser) {
        memset(parser, 0, sizeof(latm_parser_t));
    }
}

latm_error_t latm_parse_frame(latm_parser_t *parser,
                               const uint8_t *data, size_t data_len,
                               uint8_t *aac_out, size_t aac_out_size,
                               size_t *aac_len,
                               latm_frame_info_t *frame_info)
{
    if (parser == NULL || data == NULL || aac_out == NULL || aac_len == NULL) {
        return LATM_ERR_NULL_POINTER;
    }
    
    if (data_len < 2) {
        return LATM_ERR_NOT_ENOUGH_DATA;
    }
    
    bitstream_t bs;
    bs_init(&bs, data, data_len);
    latm_error_t err;
    
    /* AudioMuxElement - MCP1 format for A2DP */
    /* useSameStreamMux */
    parser->use_same_stream_mux = bs_read_bit(&bs) ? true : false;
    
    if (!parser->use_same_stream_mux) {
        /* Parse new StreamMuxConfig */
        err = parse_stream_mux_config(&bs, parser);
        if (err != LATM_OK) return err;
    } else if (!parser->config_parsed) {
        /* No config available yet */
        return LATM_ERR_UNSUPPORTED_CONFIG;
    }
    
    /* PayloadMux - extract AAC frame(s) */
    uint32_t aac_frame_len = 0;
    
    for (uint8_t sf = 0; sf < parser->num_sub_frames; sf++) {
        if (parser->frame_length_type == 0) {
            /* Variable length - parse PayloadLengthInfo */
            aac_frame_len = parse_payload_length_info(&bs);
        } else if (parser->frame_length_type == 1) {
            /* Fixed length */
            aac_frame_len = parser->frame_length;
        }
    }
    
    /* Get bit position where AAC frame starts */
    size_t frame_bit_offset = bs_get_bit_pos(&bs);
    size_t frame_byte_offset = frame_bit_offset >> 3;
    size_t bit_remainder = frame_bit_offset & 7;
    
    /* Fill in frame info if requested */
    if (frame_info) {
        frame_info->frame_length_bits = aac_frame_len * 8;
        frame_info->frame_length_bytes = aac_frame_len;
        frame_info->bit_offset = frame_bit_offset;
        frame_info->is_byte_aligned = (bit_remainder == 0);
    }
    
    /* Check output buffer size */
    if (aac_frame_len > aac_out_size) {
        return LATM_ERR_BUFFER_TOO_SMALL;
    }
    
    /* Check input data bounds */
    if (frame_byte_offset + aac_frame_len > data_len) {
        return LATM_ERR_NOT_ENOUGH_DATA;
    }
    
    /*
     * Extract AAC frame data
     * 
     * If byte-aligned, simple copy
     * If not byte-aligned, need to shift bits
     */
    if (bit_remainder == 0) {
        /* Byte aligned - simple copy */
        memcpy(aac_out, data + frame_byte_offset, aac_frame_len);
    } else {
        /* Not byte aligned - need to shift bits */
        /* This is the tricky case mentioned by the user */
        uint8_t left_shift = bit_remainder;
        uint8_t right_shift = 8 - bit_remainder;
        
        for (size_t i = 0; i < aac_frame_len; i++) {
            size_t src_byte = frame_byte_offset + i;
            uint8_t byte_val = (data[src_byte] << left_shift);
            if (src_byte + 1 < data_len) {
                byte_val |= (data[src_byte + 1] >> right_shift);
            }
            aac_out[i] = byte_val;
        }
    }
    
    *aac_len = aac_frame_len;
    return LATM_OK;
}

const latm_audio_config_t *latm_get_audio_config(const latm_parser_t *parser)
{
    if (parser && parser->audio_config.config_valid) {
        return &parser->audio_config;
    }
    return NULL;
}

bool latm_config_valid(const latm_parser_t *parser)
{
    return parser && parser->config_parsed && parser->audio_config.config_valid;
}
