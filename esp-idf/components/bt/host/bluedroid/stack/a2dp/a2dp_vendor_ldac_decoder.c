/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "common/bt_trace.h"
#include "stack/a2dp_vendor_ldac.h"
#include "stack/a2dp_vendor_ldac_constants.h"
#include "stack/a2dp_vendor_ldac_decoder.h"
#include "ldacdec.h"


#if (defined(LDAC_DEC_INCLUDED) && LDAC_DEC_INCLUDED == TRUE)

typedef struct {
  ldacdec_t decoder;
  decoded_data_callback_t decode_callback;
} tA2DP_LDAC_DECODER_CB;

static tA2DP_LDAC_DECODER_CB a2dp_ldac_decoder_cb;


bool a2dp_ldac_decoder_init(decoded_data_callback_t decode_callback) {
    int res;

    res = ldacdecInit(&a2dp_ldac_decoder_cb.decoder);
    if (res) {
        APPL_TRACE_ERROR("%s: decoder init failed %d", __func__, res);
        return false;
    }
    a2dp_ldac_decoder_cb.decode_callback = decode_callback;
    return true;
}

ssize_t a2dp_ldac_decoder_decode_packet_header(BT_HDR* p_buf) {
    const size_t header_len = 12 + 1;


    p_buf->offset += header_len;
    p_buf->len -= header_len;
    return 0;
}

static bool find_sync_word(unsigned char** buf, int *size) {
    while((*size) > 0 && **buf != 0xAA) {
        (*buf)++;
        (*size)--;
    }

    return (*size) > 0;
}

/* -------------- PATCHED FUNCTION -------------- */
bool a2dp_ldac_decoder_decode_packet(BT_HDR* p_buf, unsigned char* buf, size_t buf_len) {
    ldacdec_t* decoder = &a2dp_ldac_decoder_cb.decoder;
    int ret;

    unsigned char* src = ((unsigned char *)(p_buf + 1) + p_buf->offset);
    int src_size = p_buf->len;
    unsigned char* dst = buf;
    int32_t avail = buf_len;

    /* processed > 0 ensures we stop if the last decode consumed 0 bytes */
    int32_t processed = -1;

    while (src_size > 0 && avail > 0 && processed > 0) {
        int bytes_used = 0;
        int out_size   = 0;
        processed      = 0;

        if (!find_sync_word(&src, &src_size)) {
            break;
        }

        ret = ldacDecode(decoder, src, (void *)dst, &bytes_used);
        if (ret != 0) {
            APPL_TRACE_ERROR("%s: decoder error %d", __func__, ret);
            return false;
        }

        /* --- critical fix: stop if decoder didn’t consume anything --- */
        if (bytes_used <= 0) {
            LOG_ERROR("%s: decoder returned 0 bytes, stopping to avoid infinite loop.",
                      __func__);
            break;
        }

        processed  = bytes_used;
        src       += bytes_used;
        src_size  -= bytes_used;

        frame_t *frame = &decoder->frame;
        out_size = frame->frameSamples * frame->channelCount * sizeof(int16_t);

        /* extra safety: don’t overflow output buffer */
        if (out_size <= 0 || out_size > avail) {
            LOG_ERROR("%s: Insufficient output buffer size. frame=%d avail=%d",
                      __func__, out_size, avail);
            break;
        }

        dst   += out_size;
        avail -= out_size;
    }

    if (src_size > 0 && avail <= 0) {
        LOG_ERROR("%s: Insufficient output buffer size. %d bytes remain.",
                  __func__, src_size);
    }

    size_t len = buf_len - avail;
    if (len > buf_len) {
        len = buf_len;
    }

    a2dp_ldac_decoder_cb.decode_callback((uint8_t*)buf, len);
    return true;
}
/* -------------- END PATCH -------------- */

void a2dp_ldac_decoder_configure(const uint8_t* p_codec_info) {
    tA2DP_LDAC_CIE cie;
    tA2D_STATUS status;
    status = A2DP_ParseInfoLdac(&cie, (uint8_t*)p_codec_info, false);
    if (status != A2D_SUCCESS) {
        LOG_ERROR("%s: failed parsing codec info. %d", __func__, status);
        return;
    }

    uint32_t sr = 0;
    if (cie.sampleRate == A2DP_LDAC_SAMPLING_FREQ_44100) {
        sr = 44100;
    } else if (cie.sampleRate == A2DP_LDAC_SAMPLING_FREQ_48000) {
        sr = 48000;
    } else if (cie.sampleRate == A2DP_LDAC_SAMPLING_FREQ_88200) {
        sr = 88200;
    } else if (cie.sampleRate == A2DP_LDAC_SAMPLING_FREQ_96000) {
        sr = 96000;
    } else if (cie.sampleRate == A2DP_LDAC_SAMPLING_FREQ_176400) {
        sr = 176400;
    } else if (cie.sampleRate == A2DP_LDAC_SAMPLING_FREQ_192000) {
        sr = 192000;
    }
    LOG_INFO("%s: LDAC Sampling frequency = %lu", __func__, sr);

    if (cie.channelMode == A2DP_LDAC_CHANNEL_MODE_MONO) {
        LOG_INFO("%s: LDAC Channel mode: Mono", __func__);
    } else if (cie.channelMode == A2DP_LDAC_CHANNEL_MODE_DUAL) {
        LOG_INFO("%s: LDAC Channel mode: Dual", __func__);
    } else if (cie.channelMode == A2DP_LDAC_CHANNEL_MODE_STEREO) {
        LOG_INFO("%s: LDAC Channel mode: Stereo", __func__);
    }
}

#endif /* defined(LDAC_DEC_INCLUDED) && LDAC_DEC_INCLUDED == TRUE) */
