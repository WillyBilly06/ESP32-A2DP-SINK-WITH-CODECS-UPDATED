/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Safer A2DP Opus decoder wrapper for ESP32 Bluedroid sink.
 *
 * Why this version exists:
 * - The previous wrapper used opus_multistream_decoder_* for a normal mono/stereo
 *   A2DP Opus stream. For A2DP Opus 05 from Linux/PipeWire this still crashed
 *   the A2DP_DECODER task. This version uses the normal OpusDecoder API, which
 *   is the correct lightweight path for raw mono/stereo Opus packets.
 * - It validates codec CIE fields before decoder creation.
 * - It validates RTP/media header length before modifying BT_HDR.
 * - It bounds fragmented frame accumulation.
 * - It never writes beyond the shared decode buffer.
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>

#include "common/bt_trace.h"
#include "stack/a2dp_vendor_opus_constants.h"
#include "stack/a2dp_vendor_opus_decoder.h"
#include "stack/a2dp_vendor_opus.h"
#include "opus.h"

#if (defined(OPUS_DEC_INCLUDED) && OPUS_DEC_INCLUDED == TRUE)

#define OPUS_A2DP_SAMPLE_RATE        48000
#define OPUS_A2DP_CHANNEL_MAX        2
#define OPUS_A2DP_FRAGMENT_BUF_SIZE  (6 * 1024)

/* 40 ms @ 48 kHz. This tree advertises 10/20/40 ms by default. */
#define OPUS_A2DP_MAX_FRAME_SAMPLES  1920

/* 40 ms stereo 16-bit PCM = 1920 * 2 * 2 = 7680 bytes. */
#define OPUS_A2DP_MAX_PCM_BYTES      (OPUS_A2DP_MAX_FRAME_SAMPLES * OPUS_A2DP_CHANNEL_MAX * sizeof(opus_int16))

typedef struct {
    OpusDecoder *decoder;
    int fragment_size;
    int fragment_count;
    uint8_t fragment[OPUS_A2DP_FRAGMENT_BUF_SIZE];
    uint8_t channels;
    decoded_data_callback_t decode_callback;
} tA2DP_OPUS_DECODER_CB;

static tA2DP_OPUS_DECODER_CB a2dp_opus_decoder_cb;

static bool opus_cie_is_supported(const tA2DP_OPUS_CIE *ie)
{
    if (!ie) return false;

    /* This A2DP sink only supports normal raw mono/stereo Opus. */
    if (ie->channels < 1 || ie->channels > OPUS_A2DP_CHANNEL_MAX) {
        APPL_TRACE_ERROR("%s: unsupported channel count: %u", __func__, ie->channels);
        return false;
    }

    /* For stereo, Linux should negotiate one coupled stream. For mono, zero is OK. */
    if (ie->channels == 1 && ie->coupled_streams != 0) {
        APPL_TRACE_ERROR("%s: invalid mono coupled_streams=%u", __func__, ie->coupled_streams);
        return false;
    }
    if (ie->channels == 2 && ie->coupled_streams != 1) {
        APPL_TRACE_ERROR("%s: invalid stereo coupled_streams=%u", __func__, ie->coupled_streams);
        return false;
    }

    switch (ie->frame_duration) {
    case A2DP_OPUS_FRAME_DURATION_2_5:
    case A2DP_OPUS_FRAME_DURATION_5_0:
    case A2DP_OPUS_FRAME_DURATION_10:
    case A2DP_OPUS_FRAME_DURATION_20:
    case A2DP_OPUS_FRAME_DURATION_40:
        return true;
    default:
        APPL_TRACE_ERROR("%s: unsupported frame duration id: %u", __func__, ie->frame_duration);
        return false;
    }
}

bool a2dp_opus_decoder_init(decoded_data_callback_t decode_callback)
{
    memset(&a2dp_opus_decoder_cb, 0, sizeof(a2dp_opus_decoder_cb));
    a2dp_opus_decoder_cb.decode_callback = decode_callback;
    return true;
}

void a2dp_opus_decoder_cleanup(void)
{
    tA2DP_OPUS_DECODER_CB *cb = &a2dp_opus_decoder_cb;
    if (cb->decoder) {
        opus_decoder_destroy(cb->decoder);
    }
    memset(cb, 0, sizeof(*cb));
}

void a2dp_opus_decoder_configure(const uint8_t *p_codec_info)
{
    tA2DP_OPUS_DECODER_CB *cb = &a2dp_opus_decoder_cb;

    if (cb->decoder) {
        opus_decoder_destroy(cb->decoder);
        cb->decoder = NULL;
    }
    cb->channels = 0;
    cb->fragment_size = 0;
    cb->fragment_count = 0;

    tA2DP_OPUS_CIE ie;
    memset(&ie, 0, sizeof(ie));

    if (A2DP_ParseInfoOpus(&ie, p_codec_info, false) != A2D_SUCCESS) {
        APPL_TRACE_ERROR("%s: failed to parse Opus CIE", __func__);
        return;
    }

    if (!opus_cie_is_supported(&ie)) {
        return;
    }

    int error = OPUS_OK;
    OpusDecoder *dec = opus_decoder_create(OPUS_A2DP_SAMPLE_RATE, (int)ie.channels, &error);
    if (error != OPUS_OK || !dec) {
        APPL_TRACE_ERROR("%s: opus_decoder_create failed: %d", __func__, error);
        return;
    }

    LOG_INFO("%s: Opus configured: channels=%u coupled=%u bitrate=%u frame_dur=0x%02x",
             __func__, ie.channels, ie.coupled_streams, ie.maximum_bitrate, ie.frame_duration);

    cb->decoder = dec;
    cb->channels = ie.channels;
}

ssize_t a2dp_opus_decoder_decode_packet_header(BT_HDR *p_buf)
{
    if (!p_buf) return -EINVAL;

    const size_t header_len = sizeof(struct media_packet_header) +
                              sizeof(struct media_payload_header);

    if (p_buf->len < header_len) {
        APPL_TRACE_ERROR("%s: packet too short: len=%u header=%u",
                         __func__, (unsigned)p_buf->len, (unsigned)header_len);
        return -EINVAL;
    }

    tA2DP_OPUS_DECODER_CB *cb = &a2dp_opus_decoder_cb;
    uint8_t *src = ((uint8_t *)(p_buf + 1)) + p_buf->offset;
    struct media_payload_header *payload =
        (struct media_payload_header *)(src + sizeof(struct media_packet_header));

    if (payload->is_fragmented) {
        if (payload->is_first_fragment) {
            cb->fragment_size = 0;
        } else if (payload->frame_count + 1 != cb->fragment_count ||
                   (payload->frame_count == 1 && !payload->is_last_fragment)) {
            APPL_TRACE_ERROR("%s: fragments out of order", __func__);
            cb->fragment_count = 0;
            cb->fragment_size = 0;
            return -EINVAL;
        }
        cb->fragment_count = payload->frame_count;
    } else {
        if (payload->frame_count != 1) {
            APPL_TRACE_ERROR("%s: unsupported frame_count=%u", __func__, payload->frame_count);
            return -EINVAL;
        }
        cb->fragment_count = 0;
        cb->fragment_size = 0;
    }

    p_buf->offset += header_len;
    p_buf->len -= header_len;
    return 0;
}

bool a2dp_opus_decoder_decode_packet(BT_HDR *p_buf, unsigned char *buf, size_t buf_len)
{
    tA2DP_OPUS_DECODER_CB *cb = &a2dp_opus_decoder_cb;

    if (!p_buf || !buf || buf_len == 0) return false;

    OpusDecoder *dec = cb->decoder;
    if (!dec || cb->channels < 1 || cb->channels > OPUS_A2DP_CHANNEL_MAX) {
        APPL_TRACE_ERROR("%s: decoder not ready", __func__);
        return false;
    }

    if (buf_len < OPUS_A2DP_MAX_PCM_BYTES) {
        APPL_TRACE_ERROR("%s: decode buffer too small: %u < %u",
                         __func__, (unsigned)buf_len, (unsigned)OPUS_A2DP_MAX_PCM_BYTES);
        return false;
    }

    uint8_t *src = ((uint8_t *)(p_buf + 1)) + p_buf->offset;
    size_t src_size = p_buf->len;

    if (cb->fragment_count > 0) {
        if (src_size > (sizeof(cb->fragment) - (size_t)cb->fragment_size)) {
            APPL_TRACE_ERROR("%s: fragmented Opus frame too large", __func__);
            cb->fragment_count = 0;
            cb->fragment_size = 0;
            return false;
        }

        memcpy(cb->fragment + cb->fragment_size, src, src_size);
        cb->fragment_size += (int)src_size;

        if (cb->fragment_count > 1) {
            return true;
        }

        src = cb->fragment;
        src_size = (size_t)cb->fragment_size;
        cb->fragment_count = 0;
        cb->fragment_size = 0;
    }

    int dst_samples = (int)(buf_len / (sizeof(opus_int16) * cb->channels));
    if (dst_samples > OPUS_A2DP_MAX_FRAME_SAMPLES) {
        dst_samples = OPUS_A2DP_MAX_FRAME_SAMPLES;
    }

    int res = opus_decode(dec,
                          (const unsigned char *)src,
                          (opus_int32)src_size,
                          (opus_int16 *)buf,
                          dst_samples,
                          0);

    if (res < OPUS_OK) {
        APPL_TRACE_ERROR("%s: opus_decode error=%d, using PLC", __func__, res);
        res = opus_decode(dec, NULL, 0, (opus_int16 *)buf, dst_samples, 0);
    }

    if (res < OPUS_OK) {
        APPL_TRACE_ERROR("%s: opus PLC failed=%d", __func__, res);
        return false;
    }

    const size_t out_bytes = (size_t)res * (size_t)cb->channels * sizeof(opus_int16);
    if (cb->decode_callback && out_bytes <= buf_len) {
        cb->decode_callback((uint8_t *)buf, out_bytes);
    }

    return true;
}

#endif /* defined(OPUS_DEC_INCLUDED) && OPUS_DEC_INCLUDED == TRUE */
