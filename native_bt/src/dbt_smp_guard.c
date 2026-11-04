#include "dbt_smp_guard.h"
static void decay(dbt_smp_guard_t *g, uint32_t now) {
  uint32_t age = now - g->last_decay;
  if (age >= 0x80000000u)
    return;
  uint32_t steps = age / g->decay_ms;
  if (steps) {
    g->failures = steps >= g->failures ? 0 : (uint8_t)(g->failures - steps);
    g->last_decay += steps * g->decay_ms;
  }
}
dbt_result_t dbt_smp_guard_init(dbt_smp_guard_t *g, uint32_t base, uint32_t max, uint32_t decay_ms,
                                uint32_t now) {
  if (!g || !base || max < base || max >= 0x80000000u || !decay_ms || decay_ms >= 0x80000000u)
    return DBT_INVALID;
  *g = (dbt_smp_guard_t){.base_ms = base, .max_ms = max, .decay_ms = decay_ms, .last_decay = now};
  return DBT_OK;
}
bool dbt_smp_guard_allowed(dbt_smp_guard_t *g, uint32_t now) {
  if (!g || !g->base_ms)
    return false;
  decay(g, now);
  if (g->waiting && (int32_t)(now - g->deadline) >= 0)
    g->waiting = false;
  return !g->waiting;
}
void dbt_smp_guard_failure(dbt_smp_guard_t *g, uint32_t now) {
  if (!g || !g->base_ms)
    return;
  decay(g, now);
  if (g->failures < 32)
    g->failures++;
  uint32_t delay = g->base_ms;
  for (unsigned i = 1; i < g->failures && delay < g->max_ms; i++) {
    delay = delay > g->max_ms / 2 ? g->max_ms : delay * 2;
  }
  g->deadline = now + delay;
  g->last_decay = now;
  g->waiting = true;
}
void dbt_smp_guard_success(dbt_smp_guard_t *g, uint32_t now) {
  if (!g || !g->base_ms)
    return;
  g->failures = 0;
  g->waiting = false;
  g->deadline = 0;
  g->last_decay = now;
}
