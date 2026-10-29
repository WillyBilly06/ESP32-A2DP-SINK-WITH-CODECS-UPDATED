#ifndef DBT_SMP_GUARD_H
#define DBT_SMP_GUARD_H
#include "dbt_queue.h"
/* One guard per claimed identity, retained across connections by the owner.
 * Times are wrapping monotonic milliseconds; poll at least once per 2^31 ms.
 * The owner must apply it before starting/responding to pairing and report
 * every failed pairing and verified success. Identity resolution and a
 * bounded table/eviction policy are external. */
typedef struct {
    uint32_t base_ms, max_ms, decay_ms, deadline, last_decay;
    uint8_t failures;
    bool waiting;
} dbt_smp_guard_t;
dbt_result_t dbt_smp_guard_init(dbt_smp_guard_t *g,uint32_t base_ms,
    uint32_t max_ms,uint32_t decay_ms,uint32_t now);
bool dbt_smp_guard_allowed(dbt_smp_guard_t *g,uint32_t now);
void dbt_smp_guard_failure(dbt_smp_guard_t *g,uint32_t now);
void dbt_smp_guard_success(dbt_smp_guard_t *g,uint32_t now);
#endif
