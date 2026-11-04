#ifndef PROBE_DIAGNOSTIC_STATS_H
#define PROBE_DIAGNOSTIC_STATS_H
#include <stdbool.h>
#include <stdint.h>
#define PROBE_DIAGNOSTIC_SAMPLES_MAX 96u
typedef struct {
    uint64_t sum;
    uint32_t minimum,maximum,mean,p50,p95,p99,at_or_above_budget;
} probe_diagnostic_stats_t;
/* Sorts the caller's finite scratch array after measurement. No allocation,
 * floating point, logging or dependence on live codec state. */
bool probe_diagnostic_stats(uint32_t *samples,unsigned count,uint32_t budget,probe_diagnostic_stats_t *result);
#endif
