/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "common/bt_trace.h"
#include "stack/a2dp_vendor_ldac.h"
#include "stack/a2dp_vendor_ldac_constants.h"
#include "stack/a2dp_vendor_ldac_decoder.h"

#include <ldacBT.h>


#if (defined(LDAC_DEC_INCLUDED) && LDAC_DEC_INCLUDED == TRUE)

#define MAX_QUALITY (LDACBT_EQMID_HQ)

typedef struct {
    HANDLE_LDAC_BT ldac_handle;
    bool has_ldac_handle;
    LDACBT_SMPL_FMT_T pcm_fmt;

    decoded_data_callback_t decode_callback;
} tA2DP_LDAC_DECODER_CB;

static tA2DP_LDAC_DECODER_CB a2dp_ldac_decoder_cb;


bool a2dp_ldac_decoder_init(decoded_data_callback_t decode_callback) {

    HANDLE_LDAC_BT hndl = ldacBT_get_handle();
    if (!hndl) {
        APPL_TRACE_ERROR("%s: could not get decoder handle", __func__);
        a2dp_ldac_decoder_cb.has_ldac_handle = false;
        return false;
    }

    a2dp_ldac_decoder_cb.ldac_handle = hndl;
    a2dp_ldac_decoder_cb.has_ldac_handle = true;
    a2dp_ldac_decoder_cb.pcm_fmt = LDACBT_SMPL_FMT_S32;
    a2dp_ldac_decoder_cb.decode_callback = decode_callback;
    return true;
}

void a2dp_ldac_decoder_cleanup() {
    if (a2dp_ldac_decoder_cb.has_ldac_handle) {
        HANDLE_LDAC_BT hndl = a2dp_ldac_decoder_cb.ldac_handle;
        ldacBT_close_handle(hndl);
        ldacBT_free_handle(hndl);
    }

    memset(&a2dp_ldac_decoder_cb, 0, sizeof(a2dp_ldac_decoder_cb));
    a2dp_ldac_decoder_cb.has_ldac_handle = false;
}

ssize_t a2dp_ldac_decoder_decode_packet_header(BT_HDR* p_buf) {
    size_t header_len = sizeof(struct media_packet_header) +
                        A2DP_LDAC_MPL_HDR_LEN;
    p_buf->offset += header_len;
    p_buf->len -= header_len;
    return 0;
}

bool a2dp_ldac_decoder_decode_packet(BT_HDR* p_buf, unsigned char* buf, size_t buf_len) {
    HANDLE_LDAC_BT hndl = a2dp_ldac_decoder_cb.ldac_handle;
    const int32_t max_frame_size = (3 - MAX_QUALITY) * 110;

    if (!a2dp_ldac_decoder_cb.has_ldac_handle) {
        return false;
    }

    uint8_t *src = (uint8_t*)((unsigned char *)(p_buf + 1) + p_buf->offset);
    int32_t in_count = p_buf->len;
    int32_t in_used = 0;
    int32_t used_bytes = 0;

    int32_t out_used = 0;
    int32_t out_count = 0;

    int result = 0;

    /* Patched: Check buffer space BEFORE decoding to prevent overflow */
    /* LDAC decodes 256 samples per frame, max 32-bit stereo = 2048 bytes */
    const size_t max_frame_output = 256 * 4 * 2;

    while ((in_count - in_used) > 0) {
        /* Check if we have room for at least one more decoded frame */
        if ((size_t)(buf_len - out_count) < max_frame_output) {
            LOG_WARN("%s: buffer nearly full, stopping decode.", __func__);
            break;
        }

        result = ldacBT_decode(hndl,
                               src + in_used,
                               buf + out_count,
                               a2dp_ldac_decoder_cb.pcm_fmt,
                               max_frame_size,
                               (int *)&used_bytes,
                               (int *)&out_used);
        in_used += used_bytes;
        out_count += out_used;

        if (result != 0 || used_bytes <= 0 || out_used <= 0) {
            break;
        }

        if (out_count > buf_len) {
            LOG_ERROR("%s: buffer full.", __func__);
            out_count = buf_len;
            break;
        }
    }

    if (result != 0) {
        LOG_ERROR("%s: decode error. result = %d", __func__, result);
        return false;
    }

    a2dp_ldac_decoder_cb.decode_callback(buf, out_count);
    return true;
}

void a2dp_ldac_decoder_configure(const uint8_t* p_codec_info) {
    tA2DP_LDAC_CIE cie;
    tA2D_STATUS status;
    status = A2DP_ParseInfoLdac(&cie, (uint8_t*)p_codec_info, false);
    if (status != A2D_SUCCESS) {
        LOG_ERROR("%s: failed parsing codec info. %d", __func__, status);
        return;
    }

    uint32_t sf = 0;

    if (cie.sampleRate == A2DP_LDAC_SAMPLING_FREQ_44100) {
        sf = 44100;
    } else if (cie.sampleRate == A2DP_LDAC_SAMPLING_FREQ_48000) {
        sf = 48000;
    } else if (cie.sampleRate == A2DP_LDAC_SAMPLING_FREQ_88200) {
        sf = 88200;
    } else if (cie.sampleRate == A2DP_LDAC_SAMPLING_FREQ_96000) {
        sf = 96000;
    } else if (cie.sampleRate == A2DP_LDAC_SAMPLING_FREQ_176400) {
        sf = 176400;
    } else if (cie.sampleRate == A2DP_LDAC_SAMPLING_FREQ_192000) {
        sf = 192000;
    }
    LOG_INFO("%s: LDAC Sampling frequency = %lu", __func__, sf);


    int cm = 0;
    if (cie.channelMode == A2DP_LDAC_CHANNEL_MODE_MONO) {
        cm = LDACBT_CHANNEL_MODE_MONO;
        LOG_INFO("%s: LDAC Channel mode: Mono", __func__);
    } else if (cie.channelMode == A2DP_LDAC_CHANNEL_MODE_DUAL) {
        cm = LDACBT_CHANNEL_MODE_DUAL_CHANNEL;
        LOG_INFO("%s: LDAC Channel mode: Dual", __func__);
    } else if (cie.channelMode == A2DP_LDAC_CHANNEL_MODE_STEREO) {
        cm = LDACBT_CHANNEL_MODE_STEREO;
        // cm = LDACBT_CHANNEL_MODE_DUAL_CHANNEL;
        LOG_INFO("%s: LDAC Channel mode: Stereo", __func__);
    }

    int res;
    HANDLE_LDAC_BT hndl = a2dp_ldac_decoder_cb.ldac_handle;
    res = ldacBT_init_handle_decode(hndl, cm, sf, 0, 0, 0);
    if (res < 0) {
        int err = ldacBT_get_error_code(hndl);
        APPL_TRACE_ERROR("%s: decoder init failed res = %d, err = %d", __func__,
                         res, LDACBT_API_ERR(err));

        ldacBT_close_handle(hndl);
        ldacBT_free_handle(hndl);
        a2dp_ldac_decoder_cb.ldac_handle = NULL;
        a2dp_ldac_decoder_cb.has_ldac_handle = false;

        return;
    }
}

#endif /* defined(LDAC_DEC_INCLUDED) && LDAC_DEC_INCLUDED == TRUE) */
