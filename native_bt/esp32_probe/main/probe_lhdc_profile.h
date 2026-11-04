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
#endif
