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
#include "osi/allocator.h"
#include "stack/a2dp_vendor_opus_constants.h"
#include "stack/a2dp_vendor_opus_decoder.h"
#include "stack/a2dp_vendor_opus.h"
#include "opus.h"
#include "dbt_opus_scratch.h"
#include "dbt_opus_guarded_api.h"

#if (defined(OPUS_DEC_INCLUDED) && OPUS_DEC_INCLUDED == TRUE)

#define OPUS_A2DP_SAMPLE_RATE        48000
#define OPUS_A2DP_CHANNEL_MAX        2
#define OPUS_A2DP_FRAGMENT_BUF_SIZE  (6 * 1024)

/* 40 ms @ 48 kHz covers PipeWire Opus 05. Android Opus uses 20 ms. */
#define OPUS_A2DP_MAX_FRAME_SAMPLES  1920

/* 40 ms stereo 16-bit PCM = 1920 * 2 * 2 = 7680 bytes. */
#define OPUS_A2DP_MAX_PCM_BYTES      (OPUS_A2DP_MAX_FRAME_SAMPLES * OPUS_A2DP_CHANNEL_MAX * sizeof(opus_int16))

typedef struct {
    OpusDecoder *decoder;
    dbt_opus_scratch scratch;
    bool usable;
    int fragment_size;
    int fragment_count;
    uint8_t *fragment; /* Allocated only after a fragmented packet is observed. */
    uint8_t channels;
    uint16_t frame_samples;
    decoded_data_callback_t decode_callback;
} tA2DP_OPUS_DECODER_CB;

static tA2DP_OPUS_DECODER_CB a2dp_opus_decoder_cb;

static uint8_t opus_cie_channel_count(const tA2DP_OPUS_CIE *ie)
{
    if (!ie) return 0;

    if (ie->variant == A2DP_OPUS_VARIANT_ANDROID) {
        switch (ie->channelMode) {
        case A2DP_OPUS_CHANNEL_MODE_MONO:
            return 1;
        case A2DP_OPUS_CHANNEL_MODE_STEREO:
            return 2;
        default:
            return 0;
        }
    }

    return ie->channels;
}

static bool opus_cie_is_supported(const tA2DP_OPUS_CIE *ie)
{
    if (!ie) return false;

    uint8_t channels = opus_cie_channel_count(ie);
    if (channels < 1 || channels > OPUS_A2DP_CHANNEL_MAX) {
        APPL_TRACE_ERROR("%s: unsupported channel mode: 0x%02x", __func__,
                         ie->channelMode);
        return false;
    }

    if (ie->variant == A2DP_OPUS_VARIANT_ANDROID &&
        ie->samplingFreq != A2DP_OPUS_SAMPLING_FREQ_48000) {
        APPL_TRACE_ERROR("%s: unsupported sampling frequency: 0x%02x", __func__,
                         ie->samplingFreq);
        return false;
    }

    if (ie->variant == A2DP_OPUS_VARIANT_ANDROID) {
        switch (ie->frameSize) {
        case A2DP_OPUS_10MS_FRAMESIZE:
        case A2DP_OPUS_20MS_FRAMESIZE:
            return true;
        default:
            APPL_TRACE_ERROR("%s: unsupported frame size: 0x%02x", __func__, ie->frameSize);
            return false;
        }
    }

    if (ie->channels == 1 && ie->coupled_streams != 0) {
        APPL_TRACE_ERROR("%s: invalid mono coupled_streams=%u", __func__,
                         ie->coupled_streams);
        return false;
    }
    if (ie->channels == 2 && ie->coupled_streams != 1) {
        APPL_TRACE_ERROR("%s: invalid stereo coupled_streams=%u", __func__,
                         ie->coupled_streams);
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
        APPL_TRACE_ERROR("%s: unsupported frame duration id: %u", __func__,
                         ie->frame_duration);
        return false;
    }
}

static uint16_t opus_cie_frame_samples(const tA2DP_OPUS_CIE *ie)
{
    if (ie->variant == A2DP_OPUS_VARIANT_ANDROID) {
        return ie->frameSize == A2DP_OPUS_10MS_FRAMESIZE ? 480 :
               ie->frameSize == A2DP_OPUS_20MS_FRAMESIZE ? 960 : 0;
    }
    switch (ie->frame_duration) {
    case A2DP_OPUS_FRAME_DURATION_2_5: return 120;
    case A2DP_OPUS_FRAME_DURATION_5_0: return 240;
    case A2DP_OPUS_FRAME_DURATION_10: return 480;
    case A2DP_OPUS_FRAME_DURATION_20: return 960;
    case A2DP_OPUS_FRAME_DURATION_40: return 1920;
    default: return 0;
    }
}

size_t a2dp_opus_decoder_buffer_size(const uint8_t *codec_info)
{
    tA2DP_OPUS_CIE ie;
    memset(&ie, 0, sizeof(ie));
    if (!codec_info || A2DP_ParseInfoOpus(&ie, codec_info, false) != A2D_SUCCESS ||
        !opus_cie_is_supported(&ie)) return 0;
    return (size_t)opus_cie_frame_samples(&ie) * opus_cie_channel_count(&ie) * sizeof(opus_int16);
}

static void opus_clear_fragments(tA2DP_OPUS_DECODER_CB *cb, bool release)
{
    cb->fragment_size = cb->fragment_count = 0;
    if (release && cb->fragment) {
        free(cb->fragment);
        cb->fragment = NULL;
    }
}

/* The SDK dispatches configure, decode and cleanup on A2DP_DECODER. The
 * global scratch gate also rejects concurrent or recursive Opus decode calls. */
static void *opus_scratch_allocate(size_t size)
{
    /* Unlike osi_malloc_func, the base allocator reports OOM without asserting. */
    return osi_malloc_base(size);
}
static void opus_scratch_release(void *ptr) { free(ptr); }

static bool opus_release_decoder(tA2DP_OPUS_DECODER_CB *cb)
{
    /* Do not invalidate a live decode or its arena if an unexpected caller
     * violates the SDK owner-task contract. */
    if (dbt_opus_scratch_destroy(&cb->scratch) != DBT_OPUS_SCRATCH_OK) {
        return false;
    }
    free(cb->decoder);
    cb->decoder = NULL;
    cb->usable = false;
    cb->channels = 0;
    cb->frame_samples = 0;
    opus_clear_fragments(cb, true);
    return true;
}

bool a2dp_opus_decoder_init(decoded_data_callback_t decode_callback)
{
    tA2DP_OPUS_DECODER_CB *cb = &a2dp_opus_decoder_cb;
    if (!opus_release_decoder(cb)) return false;
    memset(cb, 0, sizeof(*cb));
    cb->decode_callback = decode_callback;
    return true;
}

void a2dp_opus_decoder_cleanup(void)
{
    tA2DP_OPUS_DECODER_CB *cb = &a2dp_opus_decoder_cb;
    if (!opus_release_decoder(cb)) return;
    memset(cb, 0, sizeof(*cb));
}

void a2dp_opus_decoder_configure(const uint8_t *p_codec_info)
{
    tA2DP_OPUS_DECODER_CB *cb = &a2dp_opus_decoder_cb;
    if (!opus_release_decoder(cb)) return;

    tA2DP_OPUS_CIE ie;
    memset(&ie, 0, sizeof(ie));
    if (!p_codec_info || A2DP_ParseInfoOpus(&ie, p_codec_info, false) != A2D_SUCCESS) {
        APPL_TRACE_ERROR("%s: failed to parse Opus CIE", __func__);
        return;
    }
    if (!opus_cie_is_supported(&ie)) return;

    uint8_t channels = opus_cie_channel_count(&ie);
    size_t arena_size = dbt_opus_scratch_capacity(channels);
    if (dbt_opus_scratch_init(&cb->scratch, arena_size, opus_scratch_allocate,
                              opus_scratch_release) != DBT_OPUS_SCRATCH_OK) {
        APPL_TRACE_ERROR("%s: Opus scratch allocation failed", __func__);
        return;
    }
    int decoder_size = opus_decoder_get_size(channels);
    OpusDecoder *dec = decoder_size > 0 ? opus_scratch_allocate((size_t)decoder_size) : NULL;
    int error = dec ? opus_decoder_init(dec, OPUS_A2DP_SAMPLE_RATE, (int)channels) : OPUS_ALLOC_FAIL;
    if (error != OPUS_OK) {
        free(dec);
        (void)dbt_opus_scratch_destroy(&cb->scratch);
        APPL_TRACE_ERROR("%s: Opus decoder allocation/init failed: %d", __func__, error);
        return;
    }

    cb->decoder = dec;
    cb->usable = true;
    cb->channels = channels;
    cb->frame_samples = opus_cie_frame_samples(&ie);
    LOG_INFO("%s: Opus configured: variant=%u channels=%u sample_rate=48000",
             __func__, ie.variant, channels);
}

typedef struct {
    tA2DP_OPUS_DECODER_CB *cb;
    const unsigned char *packet;
    opus_int32 packet_size;
    unsigned char *pcm;
    size_t pcm_size;
    int samples;
} opus_decode_job_t;

static int opus_decode_work(void *context)
{
    opus_decode_job_t *job = context;
    return dbt_opus_arena_decode(job->cb->decoder, job->packet, job->packet_size,
                                 (opus_int16 *)job->pcm, job->samples, 0);
}

static int opus_decode_recover(void *context)
{
    opus_decode_job_t *job = context;
    memset(job->pcm, 0, job->pcm_size);
    int status = opus_decoder_ctl(job->cb->decoder, OPUS_RESET_STATE);
    if (status != OPUS_OK) job->cb->usable = false;
    return status == OPUS_OK ? 0 : -1;
}

static bool opus_decode_guarded(opus_decode_job_t *job, int *samples)
{
    int status = dbt_opus_scratch_run(&job->cb->scratch, opus_decode_work, job,
                                      opus_decode_recover, job, samples);
    if (status == DBT_OPUS_SCRATCH_OK) return true;
    /* BUSY did not run decode; all other failures also discard partial PCM. */
    memset(job->pcm, 0, job->pcm_size);
    if (status == DBT_OPUS_SCRATCH_RESET_FAILED) job->cb->usable = false;
    return false;
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
            opus_clear_fragments(cb, true);
            return -EINVAL;
        }
        cb->fragment_count = payload->frame_count;
    } else {
        if (payload->frame_count != 1) {
            APPL_TRACE_ERROR("%s: unsupported frame_count=%u", __func__, payload->frame_count);
            return -EINVAL;
        }
        opus_clear_fragments(cb, true);
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
    if (!dec || !cb->usable || cb->channels < 1 || cb->channels > OPUS_A2DP_CHANNEL_MAX) {
        APPL_TRACE_ERROR("%s: decoder not ready", __func__);
        return false;
    }

    size_t required_pcm = (size_t)cb->frame_samples * cb->channels * sizeof(opus_int16);
    if (required_pcm == 0 || buf_len < required_pcm) {
        APPL_TRACE_ERROR("%s: decode buffer too small: %u < %u",
                         __func__, (unsigned)buf_len, (unsigned)required_pcm);
        return false;
    }

    uint8_t *src = ((uint8_t *)(p_buf + 1)) + p_buf->offset;
    size_t src_size = p_buf->len;

    if (cb->fragment_count > 0) {
        if (!cb->fragment) {
            cb->fragment = osi_malloc_base(OPUS_A2DP_FRAGMENT_BUF_SIZE);
            if (!cb->fragment) {
                opus_clear_fragments(cb, true);
                return false;
            }
        }
        if (src_size > (OPUS_A2DP_FRAGMENT_BUF_SIZE - (size_t)cb->fragment_size)) {
            APPL_TRACE_ERROR("%s: fragmented Opus frame too large", __func__);
            opus_clear_fragments(cb, true);
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
    if (dst_samples > cb->frame_samples) {
        dst_samples = cb->frame_samples;
    }

    opus_decode_job_t job = {cb, (const unsigned char *)src, (opus_int32)src_size,
                             buf, buf_len, dst_samples};
    int res = OPUS_INTERNAL_ERROR;
    if (!opus_decode_guarded(&job, &res)) return false;
    if (res < OPUS_OK) {
        APPL_TRACE_ERROR("%s: opus_decode error=%d, using PLC", __func__, res);
        job.packet = NULL;
        job.packet_size = 0;
        if (!opus_decode_guarded(&job, &res)) return false;
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
