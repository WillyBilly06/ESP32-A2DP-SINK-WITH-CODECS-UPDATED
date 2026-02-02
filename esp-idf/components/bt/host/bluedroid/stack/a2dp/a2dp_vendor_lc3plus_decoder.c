/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "common/bt_trace.h"
#include "stack/a2dp_vendor_lc3plus_constants.h"
#include "stack/a2dp_vendor_lc3plus_decoder.h"
#include "stack/a2dp_vendor_lc3plus.h"
#include "osi/allocator.h"

#include "lc3.h"

#if (defined(LC3PLUS_DEC_INCLUDED) && LC3PLUS_DEC_INCLUDED == TRUE)

// #define LC3PLUS_MAX_BYTES 1250
#define FRAGMENT_BUF_SIZE   (1250)

typedef struct {
    int fragment_size;
    int fragment_count;
    uint8_t fragment[FRAGMENT_BUF_SIZE];
    uint8_t channels;
    int sample_rate;
    int frame_duration;
    bool hr_mode;

    struct lc3_decoder* decoder[2];

    decoded_data_callback_t decode_callback;
} tA2DP_LC3PLUS_DECODER_CB;

static tA2DP_LC3PLUS_DECODER_CB a2dp_lc3plus_decoder_cb;


bool a2dp_lc3plus_decoder_init(decoded_data_callback_t decode_callback) {
    a2dp_lc3plus_decoder_cb.decode_callback = decode_callback;
    return true;
}

void a2dp_lc3plus_decoder_cleanup(void) {
    tA2DP_LC3PLUS_DECODER_CB* cb = &a2dp_lc3plus_decoder_cb;
    
    for (size_t i = 0; i < cb->channels; i++) {
        if (cb->decoder[i]) {
            osi_free(cb->decoder[i]);
        }
        cb->decoder[i] = NULL;
    }
}

void a2dp_lc3plus_decoder_configure(const uint8_t* p_codec_info) {
    tA2DP_LC3PLUS_DECODER_CB* cb = &a2dp_lc3plus_decoder_cb;

    tA2DP_LC3PLUS_CIE p_ie;
    A2DP_ParseInfoLc3Plus(&p_ie, p_codec_info, false);

    int frame_duration = 0;
    int channels = 0;
    int sample_rate = 0;

    if (p_ie.frame_duration & A2DP_LC3PLUS_FRAME_DURATION_10MS) {
        frame_duration = 10000;
    } else if (p_ie.frame_duration & A2DP_LC3PLUS_FRAME_DURATION_5MS) {
        frame_duration = 5000;
    } else if (p_ie.frame_duration & A2DP_LC3PLUS_FRAME_DURATION_2_5MS) {
        frame_duration = 2500;
    } else {
        APPL_TRACE_ERROR("%s: Invalid frame_duration = 0x%x", __func__, p_ie.frame_duration);
        return;
    }

    if (p_ie.channels & A2DP_LC3PLUS_CHANNELS_2) {
        channels = 2;
    } else if (p_ie.channels & A2DP_LC3PLUS_CHANNELS_1) {
        channels = 1;
    } else {
        APPL_TRACE_ERROR("%s: Invalid channels = 0x%x", __func__, p_ie.channels);
        return;
    }

    if (p_ie.frequency & A2DP_LC3PLUS_SAMPLING_FREQ_48000) {
        sample_rate = 48000;
    } else if (p_ie.frequency & A2DP_LC3PLUS_SAMPLING_FREQ_96000) {
        sample_rate = 96000;
    } else {
        APPL_TRACE_ERROR("%s: Invalid sample rate = 0x%x", __func__, p_ie.frequency);
        return;
    }

    LOG_INFO("%s: frame_duration = %ld", __func__, frame_duration);
    LOG_INFO("%s: channels = %ld", __func__, channels);
    LOG_INFO("%s: sample_rate = %ld", __func__, sample_rate);

    bool hr_mode = true;

    cb->frame_duration = frame_duration;
    cb->channels = channels;
    cb->sample_rate = sample_rate;
    cb->hr_mode = hr_mode;

    for (size_t i = 0; i < channels; i++) {
        if (cb->decoder[i]) {
            osi_free(cb->decoder[i]);
            cb->decoder[i] = NULL;
        }
    }

    for (size_t i = 0; i < channels; i++) {
        size_t sz = lc3_hr_decoder_size(hr_mode, frame_duration, sample_rate);
        struct lc3_decoder* dec = osi_malloc(sz);
        if (!dec) {
            APPL_TRACE_ERROR("%s: Failed to allocate decoder for ch %u", __func__, i);
        }

        cb->decoder[i] = dec;

        lc3_hr_setup_decoder(hr_mode, frame_duration, sample_rate, sample_rate, dec);
    }
}

ssize_t a2dp_lc3plus_decoder_decode_packet_header(BT_HDR* p_buf) {
    tA2DP_LC3PLUS_DECODER_CB* cb = &a2dp_lc3plus_decoder_cb;

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

bool a2dp_lc3plus_decoder_decode_packet(BT_HDR* p_buf, unsigned char* buf, size_t buf_len) {
    tA2DP_LC3PLUS_DECODER_CB* cb = &a2dp_lc3plus_decoder_cb;

    unsigned char* src = ((unsigned char *)(p_buf + 1) + p_buf->offset);
    int src_size = p_buf->len;


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

    enum lc3_pcm_format cfmt = LC3_PCM_FORMAT_S24;
    int channels = cb->channels;
    int32_t* pcm = (int32_t*)buf;
    int ret = 0;

    const uint8_t *in_ptr = src;
    for (size_t ich = 0; ich < channels; ich++) {
        struct lc3_decoder* dec = cb->decoder[ich];

        int frame_size = src_size / channels
                            + (ich < src_size % channels);

        int _ret = lc3_decode(dec, in_ptr, frame_size,
                                cfmt, pcm + ich, channels);
        if (_ret) {
            APPL_TRACE_ERROR("%s: channel %d decode error: %d", __func__, ich, _ret);
        }
        ret |= _ret;

        in_ptr += frame_size;
    }

    if (!ret) {
        int nsamples = lc3_hr_frame_samples(cb->hr_mode, cb->frame_duration, cb->sample_rate);

        if (nsamples > 0) {
            size_t dst_out = nsamples * cb->channels * sizeof(pcm[0]);
            cb->decode_callback((uint8_t*)buf, dst_out);
        } else if (nsamples == 0) {
            APPL_TRACE_WARNING("%s: No frames decoded. nsamples = %d", __func__, nsamples);
        } else {
            APPL_TRACE_ERROR("%s: Error getting frame samples: nsamples = %d", __func__, nsamples);
        }
    }

    return ret == 0;    
}

#endif /* defined(LC3PLUS_DEC_INCLUDED) && LC3PLUS_DEC_INCLUDED == TRUE) */
