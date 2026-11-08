#ifndef PROBE_OWNER_STATE_H
#define PROBE_OWNER_STATE_H
#include <stdatomic.h>
#include <stdint.h>
#include <stdbool.h>
#if (defined(DBT_PROBE_SCHEDULER_TRACE) || defined(DBT_PROBE_STAY_NATIVE)) && defined(DBT_PROBE_AUDIO_OUTPUT) && defined(DBT_PROBE_AUDIO_TEST)
#define PROBE_OWNER_STATE_TRACE 1
#endif
enum {PROBE_OWNER_UNKNOWN,PROBE_OWNER_LOOP,PROBE_OWNER_RUNTIME,PROBE_OWNER_PROFILES,
      PROBE_OWNER_DEADLINES,PROBE_OWNER_SLEEP,PROBE_OWNER_RENDER_YIELD,PROBE_OWNER_READY_YIELD};
enum {PROBE_OWNER_BOUND=1u,PROBE_OWNER_PENDING=2u,PROBE_OWNER_NCP=4u,PROBE_OWNER_IN_RUNTIME=8u};
typedef struct {
    uint32_t coherent,phase,phase_at_us,wait_ms,credits_at_us,pending_counts,ncp_at_us,ncp_packets,flags,tx_owned;
} probe_owner_sample_t;
/* Init with a stable internal atomic flag before controller/I2S startup. No
 * reinitialization while readers run. Other mutations: host owner only. */
void probe_owner_state_init(_Atomic uint32_t *tx_owned);
/* Markers precede the named operation; they are intent, not an RTOS task-state
 * query. Returns previous phase for a nested render-service yield. A runtime
 * entry invalidates cached credits; profiles entry ends that mutation window. */
uint32_t probe_owner_state_mark(uint32_t phase,uint32_t at,uint32_t wait_ms);
/* Existing host-owned completion counts, not controller outstanding permits. */
void probe_owner_state_credits(uint32_t classic,uint32_t le,uint32_t at);
/* Observe only after the real SDK send call returns. No command is submitted
 * here. H4 Number Of Completed Packets is validated and bounded by DBT_MAX_LINKS. */
bool probe_owner_state_completed(const uint8_t *,uint16_t,uint32_t at);
/* One IRAM-safe attempt. A concurrent update gives coherent=false, never a
 * retry. tx_owned is an independent atomic observation with stable binding. */
void probe_owner_state_sample(probe_owner_sample_t *);
#endif
