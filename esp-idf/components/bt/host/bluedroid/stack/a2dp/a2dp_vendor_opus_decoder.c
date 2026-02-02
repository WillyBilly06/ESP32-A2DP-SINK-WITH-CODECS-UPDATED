/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "common/bt_trace.h"
#include "stack/a2dp_vendor_opus_constants.h"
#include "stack/a2dp_vendor_opus_decoder.h"
#include "stack/a2dp_vendor_opus.h"
#include "opus_multistream.h"


#if (defined(OPUS_DEC_INCLUDED) && OPUS_DEC_INCLUDED == TRUE)

#define SAMPLE_RATE         48000
#define CHANNEL_MAX         2
#define FRAGMENT_BUF_SIZE   (5 * 1024)

typedef struct {
  OpusMSDecoder *decoder;
  int fragment_size;
  int fragment_count;
  uint8_t fragment[FRAGMENT_BUF_SIZE];
  uint8_t channels;
  decoded_data_callback_t decode_callback;
} tA2DP_OPUS_DECODER_CB;

static tA2DP_OPUS_DECODER_CB a2dp_opus_decoder_cb;


bool a2dp_opus_decoder_init(decoded_data_callback_t decode_callback) {
    a2dp_opus_decoder_cb.decode_callback = decode_callback;
    return true;
}

void a2dp_opus_decoder_cleanup(void) {
    tA2DP_OPUS_DECODER_CB* cb = &a2dp_opus_decoder_cb;
    OpusMSDecoder* st = cb->decoder;
    if (st) {
        opus_multistream_decoder_destroy(st);
    }
    cb->decoder = NULL;
}

void a2dp_opus_decoder_configure(const uint8_t* p_codec_info) {
    tA2DP_OPUS_DECODER_CB* cb = &a2dp_opus_decoder_cb;
    OpusMSDecoder* st = cb->decoder;

    if (st != NULL) {
        opus_multistream_decoder_destroy(st);
        st = NULL;
    }

    tA2DP_OPUS_CIE p_ie;
    A2DP_ParseInfoOpus(&p_ie, p_codec_info, false);

    opus_int32 Fs = SAMPLE_RATE;
    int channels = p_ie.channels;
    int streams = p_ie.channels - p_ie.coupled_streams;
    int coupled_streams = p_ie.coupled_streams;
    unsigned char mapping[CHANNEL_MAX];
    unsigned int i;
    int error;

    for (i = 0; i < channels; ++i)
        mapping[i] = i;



    LOG_INFO("%s: Opus Channels = %ld", __func__, channels);
    LOG_INFO("%s: Opus Streams = %ld", __func__, streams);
    LOG_INFO("%s: Opus Coupled Streams = %lu", __func__, coupled_streams);

    uint32_t frame_duration = 0;
    if (p_ie.frame_duration == A2DP_OPUS_FRAME_DURATION_2_5) {
        frame_duration = 2500;
    } else if (p_ie.frame_duration == A2DP_OPUS_FRAME_DURATION_5_0) {
        frame_duration = 5000;
    } else if (p_ie.frame_duration == A2DP_OPUS_FRAME_DURATION_10) {
        frame_duration = 10000;
    } else if (p_ie.frame_duration == A2DP_OPUS_FRAME_DURATION_20) {
        frame_duration = 20000;
    } else if (p_ie.frame_duration == A2DP_OPUS_FRAME_DURATION_40) {
        frame_duration = 40000;
    }
    LOG_INFO("%s: Opus Frame Duration = %lu us", __func__, frame_duration);
    LOG_INFO("%s: Opus Maximum Bitrate = %lu", __func__, p_ie.maximum_bitrate);

    st = (OpusMSDecoder*) opus_multistream_decoder_create(Fs, channels, streams,
                                                          coupled_streams,
                                                          (unsigned char*)&mapping,
                                                          &error);
    if (error != OPUS_OK) {
        APPL_TRACE_ERROR("%s: opus decoder create error %d", __func__, error);
        a2dp_opus_decoder_cb.decoder = NULL;
        return;
    }

    cb->decoder = st;
    cb->channels = channels;
}

ssize_t a2dp_opus_decoder_decode_packet_header(BT_HDR* p_buf) {
    tA2DP_OPUS_DECODER_CB* cb = &a2dp_opus_decoder_cb;

    size_t header_len = sizeof(struct media_packet_header) +
                           sizeof(struct media_payload_header);
    uint8_t* src = ((unsigned char *)(p_buf + 1) + p_buf->offset);
    struct media_payload_header* payload = (struct media_payload_header*)
        ((uint8_t*)src + (size_t)sizeof(struct media_packet_header));

    if (payload->is_fragmented) {
        if (payload->is_first_fragment) {
            cb->fragment_size = 0;
        } else if (payload->frame_count + 1 != cb->fragment_count ||
                   (payload->frame_count == 1 && !payload->is_last_fragment))
        {
            APPL_TRACE_ERROR("%s: Fragments not in right order: drop packet", __func__);
            return -EINVAL;
        }
        cb->fragment_count = payload->frame_count;
    } else {
        if (payload->frame_count != 1) {
            APPL_TRACE_ERROR("%s: payload->frame_count != 1. %d", __func__,
                             payload->frame_count);
            return -EINVAL;
        }
        cb->fragment_count = 0;
    }

    p_buf->offset += header_len;
    p_buf->len -= header_len;
    return 0;
}

bool a2dp_opus_decoder_decode_packet(BT_HDR* p_buf, unsigned char* buf, size_t buf_len) {
    tA2DP_OPUS_DECODER_CB* cb = &a2dp_opus_decoder_cb;
    OpusMSDecoder* dec = cb->decoder;

    if (!dec) {
        APPL_TRACE_ERROR("%s: opus decoder not allocated", __func__);
        return false;
    }

    unsigned char* src = ((unsigned char *)(p_buf + 1) + p_buf->offset);
    int src_size = p_buf->len;
    unsigned char* dst = buf;
    size_t dst_size = buf_len;

    int res;
    int dst_samples;

    if (cb->fragment_count > 0) {
        /* Fragmented frame */
        size_t avail;
        avail = min(sizeof(cb->fragment) - cb->fragment_size, src_size);
        memcpy(cb->fragment + cb->fragment_size, src, avail);

        cb->fragment_size += avail;

        if (cb->fragment_count > 1) {
            /* More fragments to come */
            APPL_TRACE_DEBUG("%s: More fragments to come", __func__);
            return true;
        }

        src = cb->fragment;
        src_size = cb->fragment_size;

        cb->fragment_count = 0;
        cb->fragment_size = 0;
    }

    if (cb->channels <= 0) {
        APPL_TRACE_ERROR("%s: opus decode error. channels = %d", __func__,
                         cb->channels);
        return false;
    }

    dst_samples = dst_size / (sizeof(opus_int16) * cb->channels);
    res = opus_multistream_decode(dec, src, src_size, (opus_int16*)dst, dst_samples, 0);

    if (res < OPUS_OK) {
        APPL_TRACE_ERROR("%s: opus decode error = %d. applying concealment",
                         __func__, res);
        res = opus_multistream_decode(dec, NULL, 0, (opus_int16*)dst,
                                      dst_samples, 0);
    }

    if (res < OPUS_OK) {
        APPL_TRACE_ERROR("%s: opus decode error = %d", __func__, res);
        return false;
    }

    size_t dst_out = (size_t)res * cb->channels * sizeof(opus_int16);
    cb->decode_callback((uint8_t*)dst, dst_out);
    return true;    
}

#endif /* defined(OPUS_DEC_INCLUDED) && OPUS_DEC_INCLUDED == TRUE) */
