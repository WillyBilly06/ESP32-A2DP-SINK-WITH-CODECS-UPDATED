#ifndef DBT_LHDCV5_H
#define DBT_LHDCV5_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "dbt_queue.h"
#include "lhdc_dec.h"

/* One codec-worker owner. Zero initialize; no decoder or tables at boot.
 * Shared caches in the supplied decoder currently permit one active instance.
 * This adapter consumes raw LHDC frames, not RTP/A2DP transport headers. */
typedef struct {
    void *workspace;
    lhdc_decoder_t *decoder;
    lhdc_dec_config_t config;
    size_t workspace_bytes;
    uint32_t generation;
} dbt_lhdcv5_session_t;

dbt_result_t dbt_lhdcv5_open(dbt_lhdcv5_session_t *session, const lhdc_dec_config_t *config);
void dbt_lhdcv5_close(dbt_lhdcv5_session_t *session);
void dbt_lhdcv5_flush(dbt_lhdcv5_session_t *session);
lhdc_dec_ret_t dbt_lhdcv5_decode(dbt_lhdcv5_session_t *session,
                                 const uint8_t *frame, size_t frame_bytes,
                                 void *pcm, size_t pcm_bytes,
                                 size_t *consumed, size_t *produced_bytes,
                                 lhdc_dec_frame_info_t *info);

#endif
