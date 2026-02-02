/*
 * SPDX-FileCopyrightText: 2024 ESP32 A2DP Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * Hybrid AAC Decoder for A2DP
 * 
 * Uses a lightweight LATM parser to strip headers, then feeds raw AAC
 * frames to FDK-AAC in RAW mode for decoding.
 * 
 * This approach:
 * - Reduces memory by using minimal LATM parser instead of FDK's transport layer
 * - Uses FDK-AAC core decoder (proven, reliable) with TT_MP4_RAW mode
 * - Allows future swap of FDK decoder with lighter alternatives
 */

#include <string.h>
#include "common/bt_trace.h"
#include "latm_parser.h"
#include "libAACdec/aacdecoder_lib.h"
#include "stack/a2dp_aac.h"
#include "stack/a2dp_aac_decoder.h"
#include "stack/a2dp_decoder.h"
#include "osi/allocator.h"
#include "osi/future.h"

#define LOG_TAG "A2DP_AAC_DEC"

/* Decode buffer for PCM output: 1024 samples * 2 channels * sizeof(int16) */
#define DECODE_BUF_LEN (4 * 1024)

/* Raw AAC frame buffer (after LATM stripping) */
#define AAC_RAW_BUF_LEN (768 * 2)  /* Max AAC frame size */

typedef struct {
    /* FDK-AAC decoder in RAW mode */
    HANDLE_AACDECODER aac_handle;
    bool has_aac_handle;
    
    /* LATM parser for stripping transport headers */
    latm_parser_t latm_parser;
    bool latm_initialized;
    
    /* Working buffers */
    INT_PCM output_buffer[DECODE_BUF_LEN];
    uint8_t raw_aac_buffer[AAC_RAW_BUF_LEN];
    
    /* Callback for decoded data */
    decoded_data_callback_t decode_callback;
    
    /* Configuration state */
    bool config_sent;
    uint32_t sample_rate;
    uint8_t channels;
} tA2DP_AAC_DECODER_CB;

static tA2DP_AAC_DECODER_CB a2dp_aac_decoder_cb;

/**
 * @brief Send AudioSpecificConfig to FDK decoder
 * 
 * When using TT_MP4_RAW, we need to manually provide the ASC
 * extracted from the LATM StreamMuxConfig.
 */
static bool send_asc_to_decoder(const latm_audio_config_t *config)
{
    if (!a2dp_aac_decoder_cb.has_aac_handle || !config || !config->config_valid) {
        return false;
    }
    
    /*
     * Build AudioSpecificConfig (ASC) binary
     * 
     * ASC format for AAC-LC:
     *   audioObjectType:      5 bits  (2 for AAC-LC)
     *   samplingFrequencyIndex: 4 bits
     *   channelConfiguration: 4 bits
     *   GASpecificConfig:
     *     frameLengthFlag:    1 bit
     *     dependsOnCoreCoder: 1 bit (0)
     *     extensionFlag:      1 bit (0)
     * 
     * Total: 16 bits = 2 bytes minimum
     */
    uint8_t asc[2];
    uint8_t sf_index = 0;
    
    /* Find sample rate index */
    static const uint32_t sr_table[] = {
        96000, 88200, 64000, 48000, 44100, 32000,
        24000, 22050, 16000, 12000, 11025, 8000, 7350
    };
    for (int i = 0; i < 13; i++) {
        if (sr_table[i] == config->sample_rate) {
            sf_index = i;
            break;
        }
    }
    
    /* audioObjectType (5 bits) | samplingFrequencyIndex (4 bits) upper 3 */
    asc[0] = (config->object_type << 3) | (sf_index >> 1);
    /* samplingFrequencyIndex lower 1 | channelConfiguration (4 bits) | flags (3 bits) */
    asc[1] = ((sf_index & 1) << 7) | (config->channel_config << 3);
    
    /* Send ASC to decoder */
    UCHAR *conf_array[1] = { asc };
    UINT conf_len[1] = { 2 };
    
    AAC_DECODER_ERROR err = aacDecoder_ConfigRaw(a2dp_aac_decoder_cb.aac_handle,
                                                  conf_array, conf_len);
    if (err != AAC_DEC_OK) {
        LOG_ERROR("%s: aacDecoder_ConfigRaw failed: 0x%x", __func__, (int)err);
        return false;
    }
    
    a2dp_aac_decoder_cb.sample_rate = config->sample_rate;
    a2dp_aac_decoder_cb.channels = config->channel_config;
    a2dp_aac_decoder_cb.config_sent = true;
    
    LOG_INFO("%s: Configured decoder - SR:%lu CH:%d ObjType:%d",
             __func__, config->sample_rate, config->channel_config, config->object_type);
    
    return true;
}

bool a2dp_aac_decoder_init(decoded_data_callback_t decode_callback)
{
    a2dp_aac_decoder_cleanup();
    
    /* Initialize LATM parser */
    if (latm_parser_init(&a2dp_aac_decoder_cb.latm_parser) != LATM_OK) {
        LOG_ERROR("%s: Failed to initialize LATM parser", __func__);
        return false;
    }
    a2dp_aac_decoder_cb.latm_initialized = true;
    
    /*
     * Open FDK-AAC decoder in RAW mode
     * 
     * TT_MP4_RAW means we provide raw AAC access units without any
     * transport layer (ADTS/ADIF/LATM). We handle LATM ourselves.
     */
    a2dp_aac_decoder_cb.aac_handle = aacDecoder_Open(TT_MP4_RAW, 1 /* nrOfLayers */);
    if (a2dp_aac_decoder_cb.aac_handle == NULL) {
        LOG_ERROR("%s: Couldn't open AAC decoder in RAW mode", __func__);
        a2dp_aac_decoder_cb.has_aac_handle = false;
        return false;
    }
    
    a2dp_aac_decoder_cb.has_aac_handle = true;
    a2dp_aac_decoder_cb.decode_callback = decode_callback;
    a2dp_aac_decoder_cb.config_sent = false;
    
    /* Clear output buffer */
    memset(a2dp_aac_decoder_cb.output_buffer, 0, sizeof(a2dp_aac_decoder_cb.output_buffer));
    
    LOG_INFO("%s: Hybrid AAC decoder initialized (LATM parser + FDK RAW)", __func__);
    return true;
}

void a2dp_aac_decoder_cleanup(void)
{
    if (a2dp_aac_decoder_cb.has_aac_handle) {
        aacDecoder_Close(a2dp_aac_decoder_cb.aac_handle);
    }
    
    if (a2dp_aac_decoder_cb.latm_initialized) {
        latm_parser_reset(&a2dp_aac_decoder_cb.latm_parser);
    }
    
    memset(&a2dp_aac_decoder_cb, 0, sizeof(a2dp_aac_decoder_cb));
}

bool a2dp_aac_decoder_reset(void)
{
    if (a2dp_aac_decoder_cb.latm_initialized) {
        latm_parser_reset(&a2dp_aac_decoder_cb.latm_parser);
        latm_parser_init(&a2dp_aac_decoder_cb.latm_parser);
    }
    a2dp_aac_decoder_cb.config_sent = false;
    return true;
}

ssize_t a2dp_aac_decoder_decode_packet_header(BT_HDR *p_buf)
{
    /* Skip media packet header */
    size_t header_len = sizeof(struct media_packet_header);
    p_buf->offset += header_len;
    p_buf->len -= header_len;
    return 0;
}

bool a2dp_aac_decoder_decode_packet(BT_HDR *p_buf, unsigned char *buf,
                                     size_t buf_len)
{
    if (!a2dp_aac_decoder_cb.has_aac_handle) {
        LOG_ERROR("%s: Decoder handle not initialized", __func__);
        return false;
    }
    
    uint8_t *input_data = (uint8_t *)(p_buf->data + p_buf->offset);
    size_t input_len = p_buf->len;
    
    /*
     * Step 1: Parse LATM header and extract raw AAC frame
     */
    size_t raw_aac_len = 0;
    latm_frame_info_t frame_info;
    
    latm_error_t latm_err = latm_parse_frame(
        &a2dp_aac_decoder_cb.latm_parser,
        input_data, input_len,
        a2dp_aac_decoder_cb.raw_aac_buffer, AAC_RAW_BUF_LEN,
        &raw_aac_len,
        &frame_info
    );
    
    if (latm_err != LATM_OK) {
        LOG_ERROR("%s: LATM parse error: %d", __func__, latm_err);
        return false;
    }
    
    /*
     * Step 2: If we got new config from LATM, send ASC to FDK
     */
    if (!a2dp_aac_decoder_cb.config_sent) {
        const latm_audio_config_t *config = latm_get_audio_config(&a2dp_aac_decoder_cb.latm_parser);
        if (config && config->config_valid) {
            if (!send_asc_to_decoder(config)) {
                LOG_ERROR("%s: Failed to configure decoder", __func__);
                return false;
            }
        } else {
            /* Config not yet available, skip this frame */
            LOG_WARN("%s: Waiting for LATM config", __func__);
            return true;
        }
    }
    
    /*
     * Step 3: Feed raw AAC frame to FDK decoder
     */
    UCHAR *pBuffer = (UCHAR *)a2dp_aac_decoder_cb.raw_aac_buffer;
    UINT bufferSize = (UINT)raw_aac_len;
    UINT bytesValid = (UINT)raw_aac_len;
    
    AAC_DECODER_ERROR err = aacDecoder_Fill(
        a2dp_aac_decoder_cb.aac_handle,
        &pBuffer, &bufferSize, &bytesValid
    );
    
    if (err != AAC_DEC_OK) {
        LOG_ERROR("%s: aacDecoder_Fill failed: 0x%x", __func__, (int)err);
        return false;
    }
    
    /*
     * Step 4: Decode frames
     */
    while (true) {
        err = aacDecoder_DecodeFrame(
            a2dp_aac_decoder_cb.aac_handle,
            (INT_PCM *)a2dp_aac_decoder_cb.output_buffer,
            DECODE_BUF_LEN,
            0 /* flags */
        );
        
        if (err == AAC_DEC_NOT_ENOUGH_BITS) {
            /* Need more data */
            break;
        }
        
        if (err != AAC_DEC_OK) {
            LOG_ERROR("%s: aacDecoder_DecodeFrame failed: 0x%x", __func__, (int)err);
            break;
        }
        
        /* Get stream info for output size */
        CStreamInfo *info = aacDecoder_GetStreamInfo(a2dp_aac_decoder_cb.aac_handle);
        if (!info || info->sampleRate <= 0) {
            LOG_ERROR("%s: Invalid stream info", __func__);
            break;
        }
        
        /* Send decoded PCM to callback */
        size_t frame_len = info->frameSize * info->numChannels * sizeof(INT_PCM);
        a2dp_aac_decoder_cb.decode_callback(
            (uint8_t *)a2dp_aac_decoder_cb.output_buffer,
            frame_len
        );
    }
    
    return true;
}

void a2dp_aac_decoder_configure(const uint8_t *p_codec_info)
{
    tA2DP_AAC_CIE cie;
    tA2D_STATUS status;
    
    status = A2DP_ParseInfoAac(&cie, p_codec_info, FALSE);
    if (status != A2D_SUCCESS) {
        LOG_ERROR("%s: Failed to parse codec info", __func__);
        return;
    }
    
    /* Log configuration */
    if (cie.objectType & A2DP_AAC_OBJECT_TYPE_MPEG2_LC) {
        LOG_INFO("%s: AAC object type: MPEG-2 Low Complexity", __func__);
    } else if (cie.objectType & A2DP_AAC_OBJECT_TYPE_MPEG4_LC) {
        LOG_INFO("%s: AAC object type: MPEG-4 Low Complexity", __func__);
    }
    
    uint32_t sr = 0;
    if (cie.sampleRate == A2DP_AAC_SAMPLING_FREQ_44100) {
        sr = 44100;
    } else if (cie.sampleRate == A2DP_AAC_SAMPLING_FREQ_48000) {
        sr = 48000;
    }
    
    uint8_t ch = (cie.channelMode == A2DP_AAC_CHANNEL_MODE_STEREO) ? 2 : 1;
    
    LOG_INFO("%s: A2DP config - SR:%lu CH:%d VBR:%d BitRate:%lu",
             __func__, sr, ch, cie.variableBitRateSupport, cie.bitRate);
    
    /* The actual config will be extracted from LATM StreamMuxConfig */
}
