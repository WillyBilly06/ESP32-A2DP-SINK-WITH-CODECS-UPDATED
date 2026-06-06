/*
 * SPDX-FileCopyrightText: 2016 The Android Open Source Project
 * SPDX-FileCopyrightText: 2024 ESP32 A2DP Helix Integration
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * A2DP AAC Decoder using Helix Fixed-Point AAC Decoder
 * 
 * This implementation replaces FDK-AAC with the lightweight Helix AAC decoder
 * which is optimized for embedded systems and uses ~35-40KB of internal RAM.
 */

#include <string.h>
#include "common/bt_trace.h"
#include "stack/a2dp_aac.h"
#include "stack/a2dp_aac_decoder.h"
#include "stack/a2dp_decoder.h"
#include "osi/allocator.h"
#include "osi/future.h"

/* Helix AAC decoder header */
#include "libhelix-aac/aacdec.h"

/* We use a statically allocated output buffer to avoid heap fragmentation */
#define HELIX_DECODE_BUF_LEN (2 * 1024)  /* Max 1024 samples * 2 channels * 2 bytes */

typedef struct {
    HAACDecoder helix_handle;
    bool has_handle;
    short output_buffer[HELIX_DECODE_BUF_LEN];
    decoded_data_callback_t decode_callback;
    int sample_rate;
    int channels;
    bool configured;
} tA2DP_AAC_HELIX_CB;

static tA2DP_AAC_HELIX_CB a2dp_aac_helix_cb;

/**
 * @brief Initialize the Helix AAC decoder for A2DP
 */
bool a2dp_aac_decoder_init(decoded_data_callback_t decode_callback) {
    a2dp_aac_decoder_cleanup();
    
    LOG_INFO("%s: Initializing Helix AAC decoder (internal RAM optimized)", __func__);
    
    /* Initialize Helix decoder */
    a2dp_aac_helix_cb.helix_handle = AACInitDecoder();
    if (a2dp_aac_helix_cb.helix_handle == NULL) {
        LOG_ERROR("%s: Failed to initialize Helix AAC decoder!", __func__);
        LOG_ERROR("%s: Check internal RAM availability", __func__);
        a2dp_aac_helix_cb.has_handle = false;
        return false;
    }
    
    /* Clear output buffer */
    memset(a2dp_aac_helix_cb.output_buffer, 0, sizeof(a2dp_aac_helix_cb.output_buffer));
    
    a2dp_aac_helix_cb.has_handle = true;
    a2dp_aac_helix_cb.decode_callback = decode_callback;
    a2dp_aac_helix_cb.sample_rate = 44100;  /* Default, will be updated by configure */
    a2dp_aac_helix_cb.channels = 2;         /* Default stereo */
    a2dp_aac_helix_cb.configured = false;
    
    LOG_INFO("%s: Helix AAC decoder initialized successfully", __func__);
    return true;
}

/**
 * @brief Cleanup the Helix AAC decoder
 */
void a2dp_aac_decoder_cleanup(void) {
    if (a2dp_aac_helix_cb.has_handle && a2dp_aac_helix_cb.helix_handle) {
        AACFreeDecoder(a2dp_aac_helix_cb.helix_handle);
        LOG_INFO("%s: Helix AAC decoder freed", __func__);
    }
    memset(&a2dp_aac_helix_cb, 0, sizeof(a2dp_aac_helix_cb));
}

/**
 * @brief Reset the decoder state
 */
bool a2dp_aac_decoder_reset(void) {
    if (a2dp_aac_helix_cb.has_handle && a2dp_aac_helix_cb.helix_handle) {
        AACFlushCodec(a2dp_aac_helix_cb.helix_handle);
    }
    return true;
}

/**
 * @brief Parse the A2DP media packet header
 */
ssize_t a2dp_aac_decoder_decode_packet_header(BT_HDR* p_buf) {
    size_t header_len = sizeof(struct media_packet_header);
    p_buf->offset += header_len;
    p_buf->len -= header_len;
    return 0;
}

/**
 * @brief Decode an AAC packet using Helix decoder
 * 
 * A2DP AAC uses LATM framing (TT_MP4_LATM_MCP1), but Helix can also 
 * handle raw AAC frames. For LATM, we need to extract the raw frame first.
 */
bool a2dp_aac_decoder_decode_packet(BT_HDR* p_buf, unsigned char* buf,
                                    size_t buf_len)
{
    unsigned char* inbuf = (unsigned char*)(p_buf->data + p_buf->offset);
    int bytes_left = p_buf->len;
    int err;
    
    if (!a2dp_aac_helix_cb.has_handle || !a2dp_aac_helix_cb.helix_handle) {
        LOG_ERROR("%s: Decoder not initialized", __func__);
        return false;
    }
    
    /* Decode the AAC frame */
    while (bytes_left > 0) {
        /* Find sync word for ADTS if present */
        int sync_offset = AACFindSyncWord(inbuf, bytes_left);
        if (sync_offset < 0) {
            /* No sync word found - might be raw AAC or LATM */
            /* Try to decode anyway */
        } else if (sync_offset > 0) {
            /* Skip to sync word */
            inbuf += sync_offset;
            bytes_left -= sync_offset;
        }
        
        /* Decode the frame */
        err = AACDecode(a2dp_aac_helix_cb.helix_handle,
                        &inbuf,
                        &bytes_left,
                        a2dp_aac_helix_cb.output_buffer);
        
        if (err == ERR_AAC_NONE) {
            /* Get frame info */
            AACFrameInfo frame_info;
            AACGetLastFrameInfo(a2dp_aac_helix_cb.helix_handle, &frame_info);
            
            if (frame_info.outputSamps > 0) {
                /* Update sample rate if changed */
                if (frame_info.sampRateOut != a2dp_aac_helix_cb.sample_rate) {
                    a2dp_aac_helix_cb.sample_rate = frame_info.sampRateOut;
                    LOG_INFO("%s: Sample rate: %d Hz", __func__, frame_info.sampRateOut);
                }
                if (frame_info.nChans != a2dp_aac_helix_cb.channels) {
                    a2dp_aac_helix_cb.channels = frame_info.nChans;
                    LOG_INFO("%s: Channels: %d", __func__, frame_info.nChans);
                }
                
                /* Calculate output size in bytes (16-bit samples) */
                size_t frame_bytes = frame_info.outputSamps * sizeof(short);
                
                /* Deliver decoded PCM to callback */
                a2dp_aac_helix_cb.decode_callback(
                    (uint8_t*)a2dp_aac_helix_cb.output_buffer,
                    frame_bytes);
            }
        } else if (err == ERR_AAC_INDATA_UNDERFLOW) {
            /* Need more data - normal for last packet */
            break;
        } else {
            /* Decode error */
            LOG_WARN("%s: Helix AAC decode error: %d", __func__, err);
            /* Skip some bytes and try to recover */
            if (bytes_left > 1) {
                inbuf++;
                bytes_left--;
            } else {
                break;
            }
        }
    }
    
    return true;
}

/**
 * @brief Configure the decoder based on A2DP codec info
 */
void a2dp_aac_decoder_configure(const uint8_t* p_codec_info) {
    tA2DP_AAC_CIE cie;
    tA2D_STATUS status;
    
    status = A2DP_ParseInfoAac(&cie, p_codec_info, FALSE);
    if (status != A2D_SUCCESS) {
        LOG_ERROR("%s: Failed to parse AAC codec info", __func__);
        return;
    }
    
    /* Log AAC object type */
    if (cie.objectType & A2DP_AAC_OBJECT_TYPE_MPEG2_LC) {
        LOG_INFO("%s: AAC object type: MPEG-2 Low Complexity", __func__);
    } else if (cie.objectType & A2DP_AAC_OBJECT_TYPE_MPEG4_LC) {
        LOG_INFO("%s: AAC object type: MPEG-4 Low Complexity", __func__);
    } else if (cie.objectType & A2DP_AAC_OBJECT_TYPE_MPEG4_LTP) {
        LOG_WARN("%s: AAC object type: MPEG-4 LTP (may not be fully supported)", __func__);
    } else if (cie.objectType & A2DP_AAC_OBJECT_TYPE_MPEG4_SCALABLE) {
        LOG_WARN("%s: AAC object type: MPEG-4 Scalable (may not be fully supported)", __func__);
    }
    
    /* Determine sample rate */
    uint32_t sr = 0;
    if (cie.sampleRate == A2DP_AAC_SAMPLING_FREQ_8000) {
        sr = 8000;
    } else if (cie.sampleRate == A2DP_AAC_SAMPLING_FREQ_11025) {
        sr = 11025;
    } else if (cie.sampleRate == A2DP_AAC_SAMPLING_FREQ_12000) {
        sr = 12000;
    } else if (cie.sampleRate == A2DP_AAC_SAMPLING_FREQ_16000) {
        sr = 16000;
    } else if (cie.sampleRate == A2DP_AAC_SAMPLING_FREQ_22050) {
        sr = 22050;
    } else if (cie.sampleRate == A2DP_AAC_SAMPLING_FREQ_24000) {
        sr = 24000;
    } else if (cie.sampleRate == A2DP_AAC_SAMPLING_FREQ_32000) {
        sr = 32000;
    } else if (cie.sampleRate == A2DP_AAC_SAMPLING_FREQ_44100) {
        sr = 44100;
    } else if (cie.sampleRate == A2DP_AAC_SAMPLING_FREQ_48000) {
        sr = 48000;
    } else if (cie.sampleRate == A2DP_AAC_SAMPLING_FREQ_64000) {
        sr = 64000;
    } else if (cie.sampleRate == A2DP_AAC_SAMPLING_FREQ_88200) {
        sr = 88200;
    } else if (cie.sampleRate == A2DP_AAC_SAMPLING_FREQ_96000) {
        sr = 96000;
    } else {
        LOG_ERROR("%s: Invalid AAC sample rate config", __func__);
        sr = 44100;  /* Default fallback */
    }
    LOG_INFO("%s: AAC sample rate = %lu Hz", __func__, sr);
    a2dp_aac_helix_cb.sample_rate = sr;
    
    /* Determine channels */
    int channels = 2;
    switch (cie.channelMode) {
        case A2DP_AAC_CHANNEL_MODE_MONO:
            channels = 1;
            break;
        case A2DP_AAC_CHANNEL_MODE_STEREO:
            channels = 2;
            break;
        default:
            LOG_ERROR("%s: Invalid channel mode %u, defaulting to stereo", __func__, cie.channelMode);
            channels = 2;
            break;
    }
    LOG_INFO("%s: AAC channels = %d", __func__, channels);
    a2dp_aac_helix_cb.channels = channels;
    
    /* For raw AAC mode (non-ADTS), we need to set parameters explicitly */
    if (a2dp_aac_helix_cb.has_handle && a2dp_aac_helix_cb.helix_handle) {
        AACFrameInfo frame_info;
        memset(&frame_info, 0, sizeof(frame_info));
        frame_info.nChans = channels;
        frame_info.sampRateCore = sr;
        frame_info.profile = AAC_PROFILE_LC;
        
        /* Set raw block parameters for LATM parsing */
        int ret = AACSetRawBlockParams(a2dp_aac_helix_cb.helix_handle, 0, &frame_info);
        if (ret != ERR_AAC_NONE) {
            LOG_WARN("%s: AACSetRawBlockParams returned %d (may be OK for ADTS)", __func__, ret);
        }
    }
    
    /* Log VBR and bitrate info */
    bool vbr = !!(cie.variableBitRateSupport & A2DP_AAC_VARIABLE_BIT_RATE_ENABLED);
    LOG_INFO("%s: AAC VBR support = %s", __func__, vbr ? "enabled" : "disabled");
    LOG_INFO("%s: AAC bitrate = %lu bps", __func__, cie.bitRate);
    
    a2dp_aac_helix_cb.configured = true;
    LOG_INFO("%s: Helix AAC decoder configured successfully", __func__);
}
