#ifndef PROBE_LHDC_PROFILE_H
#define PROBE_LHDC_PROFILE_H
#include <stdbool.h>
#include <stdint.h>

/* Diagnostic only. The decoder task exclusively owns reset/record/finish;
 * no locks, allocation, logging or cross-task readers occur during decoding.
 * Each record describes one channel, in elapsed CCOUNT cycles. */
typedef struct {
    uint64_t sum_cycles[6];
    uint32_t max_cycles[6],channel_calls;
} probe_lhdc_profile_t;
void probe_lhdc_profile_reset(bool collecting);
bool probe_lhdc_profile_finish(probe_lhdc_profile_t *result);
void dbt_lhdc_stage_record(uint32_t,uint32_t,uint32_t,uint32_t,uint32_t,uint32_t);

/* Live mirrors are lock-free words. Counter sums wrap modulo2^32; readers
 * compare within one generation and a bounded observation interval. The
 * decoder alone resets/records/ends. Background readers never touch counters. */
typedef struct {
    uint32_t revision,generation,rate,sample_bytes,collecting,channel_calls;
    uint32_t sum_cycles[6],max_cycles[6];
} probe_lhdc_live_t;
void probe_lhdc_profile_live_enable(void);
void dbt_lhdc_stage_live_begin(uint32_t rate,uint32_t sample_bytes);
void dbt_lhdc_stage_live_end(void);
bool probe_lhdc_profile_snapshot(probe_lhdc_live_t *result);
#endif
