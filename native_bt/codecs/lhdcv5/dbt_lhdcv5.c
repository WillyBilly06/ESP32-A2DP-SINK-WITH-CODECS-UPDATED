#include "dbt_lhdcv5.h"
#include "dbt_lhdc_alloc.h"
#include "lhdc_entropy_dec.h"
#include "lhdc_imdct.h"
#include "lhdc_dec_slot.h"
#include <stdlib.h>

#ifdef DBT_LHDC_ALLOC_TEST
extern void *dbt_lhdc_test_alloc(size_t bytes);
extern void dbt_lhdc_test_free(void *pointer);
#define WORKSPACE_ALLOC(bytes) dbt_lhdc_test_alloc(bytes)
#define WORKSPACE_FREE(pointer) dbt_lhdc_test_free(pointer)
#elif defined(ESP_PLATFORM)
#include "esp_heap_caps.h"
#define WORKSPACE_ALLOC(bytes) heap_caps_calloc(1, (bytes), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)
#define WORKSPACE_FREE(pointer) heap_caps_free(pointer)
#else
#define WORKSPACE_ALLOC(bytes) calloc(1, (bytes))
#define WORKSPACE_FREE(pointer) free(pointer)
#endif

static dbt_lhdcv5_session_t *active_session;
static bool allocation_failed;
_Static_assert(LHDC_NSLOTS == 1, "The native LHDC session requires one decoder slot");

/* The generated decoder shares caches with this one active session. */
void *dbt_lhdc_mem_alloc(size_t bytes) {
    if (!active_session || !bytes) { allocation_failed = true; return 0; }
    void *pointer = WORKSPACE_ALLOC(bytes);
    if (!pointer) allocation_failed = true;
    return pointer;
}
void dbt_lhdc_mem_free(void *pointer) {
    if (active_session && pointer) WORKSPACE_FREE(pointer);
}

static bool valid(const lhdc_dec_config_t *config) {
    return config != 0 &&
        (config->sample_rate == 44100 || config->sample_rate == 48000 ||
         config->sample_rate == 96000 || config->sample_rate == 192000) &&
        (config->bit_depth == 16 || config->bit_depth == 24 || config->bit_depth == 32) &&
        (config->channels == 1 || config->channels == 2) &&
        config->frame_duration == LHDC_DEC_FRAME_5MS &&
        config->max_frame_bytes != 0 && config->max_frame_bytes <= 1536;
}

static bool same_config(const lhdc_dec_config_t *left, const lhdc_dec_config_t *right) {
    return left->sample_rate == right->sample_rate && left->bit_depth == right->bit_depth &&
        left->channels == right->channels && left->frame_duration == right->frame_duration &&
        left->max_frame_bytes == right->max_frame_bytes && left->lossless_enable == right->lossless_enable;
}

static void release_caches(void) {
    lhdc_dec_free_slot_scratch();
    lhdc_entropy_free();
    lhdc_dec_free_window();
    lhdc_imdct_free_480();
    lhdc_imdct_free_960();
    lhdc_imdct_free_1920();
    lhdc_imdct_free_cos();
}

void dbt_lhdcv5_close(dbt_lhdcv5_session_t *session) {
    if (session == 0 || active_session != session) return;
    lhdc_dec_deinit((lhdc_decoder_t *)session->workspace);
    release_caches();
    WORKSPACE_FREE(session->workspace);
    session->workspace = 0;
    session->decoder = 0;
    session->workspace_bytes = 0;
    active_session = 0;
}

dbt_result_t dbt_lhdcv5_open(dbt_lhdcv5_session_t *session, const lhdc_dec_config_t *config) {
    if (session == 0 || !valid(config)) return DBT_INVALID;
    if (active_session != 0 && active_session != session) return DBT_BUSY;
    if (active_session == session && same_config(&session->config, config)) return DBT_OK;
    /* Caller has already fenced and joined its codec worker/media generation.
     * Drop the old working set before allocating the new configuration. */
    dbt_lhdcv5_close(session);
    size_t size = lhdc_dec_get_workspace_size(config->sample_rate, config->frame_duration);
    if (!size) return DBT_INVALID;
    void *workspace = WORKSPACE_ALLOC(size);
    if (workspace == 0) return DBT_NO_SPACE;
    session->workspace = workspace;
    active_session = session;
    allocation_failed = false;
    lhdc_decoder_t *decoder = lhdc_dec_init(workspace, config);
    if (decoder == 0 || allocation_failed || !dbt_lhdc_entropy_ready()) {
        dbt_lhdcv5_close(session);
        return DBT_NO_SPACE;
    }
    session->workspace_bytes = size;
    session->decoder = decoder;
    session->config = *config;
    ++session->generation;
    if (session->generation == 0) ++session->generation;
    return DBT_OK;
}

void dbt_lhdcv5_flush(dbt_lhdcv5_session_t *session) {
    if (session != 0 && active_session == session && session->decoder != 0) {
        lhdc_dec_reset(session->decoder);
        ++session->generation;
        if (session->generation == 0) ++session->generation;
    }
}

lhdc_dec_ret_t dbt_lhdcv5_decode(dbt_lhdcv5_session_t *session,
                                 const uint8_t *frame, size_t frame_bytes,
                                 void *pcm, size_t pcm_bytes,
                                 size_t *consumed, size_t *produced_bytes,
                                 lhdc_dec_frame_info_t *info) {
    if (consumed != 0) *consumed = 0;
    if (produced_bytes != 0) *produced_bytes = 0;
    if (session == 0 || active_session != session || session->decoder == 0) return LHDC_DEC_NOT_INITIALIZED;
    if (frame == 0 || pcm == 0 || consumed == 0 || produced_bytes == 0) return LHDC_DEC_INVALID_PARAM;
    if (frame_bytes < 2) return LHDC_DEC_NEED_MORE_DATA;
    size_t payload = (size_t)(frame[0] | ((uint16_t)frame[1] << 8)) & 0x3ffu;
    if (payload == 0 || payload * 2u > session->config.max_frame_bytes) return LHDC_DEC_BITSTREAM_ERROR;
    size_t width = session->config.bit_depth == 16 ? 2u : 4u;
    if ((uintptr_t)pcm % width != 0) return LHDC_DEC_INVALID_PARAM;
    size_t stride = width * session->config.channels;
    size_t samples = pcm_bytes / stride;
    if (samples == 0) return LHDC_DEC_BUF_NOT_ENOUGH;
    if (samples > UINT32_MAX) samples = UINT32_MAX;
    uint32_t generated = 0;
    allocation_failed = false;
    lhdc_dec_ret_t result = lhdc_dec_decode_frame(session->decoder, frame, frame_bytes, pcm,
                                                 (uint32_t)samples, consumed, &generated, info);
    if (allocation_failed) return LHDC_DEC_ERROR;
    if (result == LHDC_DEC_OK) *produced_bytes = (size_t)generated * stride;
    return result;
}
