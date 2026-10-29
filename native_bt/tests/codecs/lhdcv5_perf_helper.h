#ifndef DBT_LHDC_PERF_HELPER_H
#define DBT_LHDC_PERF_HELPER_H
#include "dbt_codec.h"

typedef uint64_t (*dbt_lhdc_perf_clock_fn)(void *context);
typedef struct {const uint8_t *bytes;uint16_t size;} dbt_lhdc_perf_packet_t;
typedef struct {
    void *context;
    uint8_t *(*allocate)(void *,uint32_t);
    void (*release)(void *,uint8_t *);
    dbt_lhdc_perf_clock_fn now_us;
} dbt_lhdc_perf_hooks_t;
typedef struct {
    uint32_t packets,callbacks,sample_frames,checksum;
    uint32_t packet_min_us,packet_max_us,decoder_estimate_min_us,decoder_estimate_max_us;
    uint64_t packet_sum_us,callback_sum_us,decoder_estimate_sum_us;
    uint64_t stage_sum_us[6];
    uint32_t stage_max_us[6],stage_calls;
} dbt_lhdc_perf_result_t;
/* Dedicated offline/target-check helper, never a host-loop callback. Uses one
 * active decoder, warmups followed by bounded repeated fixture packets. The
 * decoder estimate subtracts measured callback spans but includes timer and
 * wrapper overhead; it is not a cycle-accurate exclusive CPU measurement. */
dbt_result_t dbt_lhdc_perf_run(dbt_lhdc_perf_hooks_t,const uint8_t *,uint8_t,
    const dbt_lhdc_perf_packet_t *,uint8_t,uint16_t,uint16_t,dbt_lhdc_perf_result_t *);
/* Optional rate-scaled FIR test. Includes filter + one history push, excludes
 * DSP, rings and I2S. Caller invokes it outside protocol/interrupt context. */
dbt_result_t dbt_lhdc_perf_resampler(dbt_lhdc_perf_clock_fn,void *,uint32_t,uint32_t,
    uint64_t *,uint32_t *);
#endif
