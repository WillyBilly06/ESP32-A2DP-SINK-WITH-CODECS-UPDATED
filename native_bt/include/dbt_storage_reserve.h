#ifndef DBT_STORAGE_RESERVE_H
#define DBT_STORAGE_RESERVE_H
#include "dbt_audio.h"
#define DBT_STORAGE_RESERVE_PAGES 3u
#define DBT_STORAGE_RESERVE_MS 20u
#define DBT_STORAGE_GRANT_MAX_AGE_US 2000u
/* One serialized background provider; no task or queue is created. Each
 * request cookie must be non-NULL and at least two-byte aligned. Its owner
 * keeps it alive through retirement; cancel never dereferences the cookie. */
typedef enum {
    DBT_STORAGE_IDLE,DBT_STORAGE_REQUEST,DBT_STORAGE_ALLOCATE,DBT_STORAGE_READY,
    DBT_STORAGE_PREFILL,DBT_STORAGE_GRANTED,DBT_STORAGE_RUNNING,
    DBT_STORAGE_RELEASE,DBT_STORAGE_DETACHED
} dbt_storage_phase_t;
typedef struct {
    uint8_t *(*allocate)(void *,uint32_t);
    void (*release)(void *,uint8_t *);
    void (*wake_render)(void *);
    void (*wake_worker)(void *,uintptr_t);
    uint32_t (*clock_us)(void *);
    uint32_t (*guard_enter)(void *);
    void (*guard_exit)(void *,uint32_t);
} dbt_storage_reserve_ops_t;
typedef struct {
    _Atomic uint32_t state;
    _Atomic uint32_t published_word,published_us;
    _Atomic uintptr_t owner;
    const dbt_storage_reserve_ops_t *ops;void *context;
    uintptr_t waiter;uint8_t *pages[DBT_STORAGE_RESERVE_PAGES];
    uint32_t needed,base_frames,target,epoch,rate,grant_us,request_us,attached;
    dbt_result_t result;
    _Atomic uint32_t requests,claims,refused,regrants,live_bytes,peak_bytes,max_age_us,max_wait_us;
} dbt_storage_reserve_t;
typedef struct {
    uint32_t state,requests,claims,refused,regrants,live_bytes,peak_bytes,max_age_us,max_wait_us;
} dbt_storage_reserve_stats_t;
/* Initialize before either owner starts; ops/context remain live through all
 * retirements. All callbacks are nonblocking except worker allocation/release.
 * guard callbacks quiesce this ring's IRQ consumer on the renderer's CPU. */
dbt_result_t dbt_storage_reserve_init(dbt_storage_reserve_t *,const dbt_storage_reserve_ops_t *,void *);
/* Worker only. PENDING requires an event wait, never a polling spin. OK owns
 * permission to perform one storage job; only then may release be called.
 * NO_SPACE/INVALID transfer none and perform no storage operation. Allocation
 * is attempted once per request, exclusively on this worker. */
dbt_result_t dbt_storage_reserve_prepare(dbt_storage_reserve_t *,const void *,uintptr_t);
/* May run on a different owner. One strong attempt fences the exact cookie;
 * caller also checks its cancellation flag after registering prepare. */
void dbt_storage_reserve_cancel(dbt_storage_reserve_t *,const void *);
void dbt_storage_reserve_release(dbt_storage_reserve_t *);
/* Worker only, after release. PENDING retains context/waiter lifetime. OK
 * releases detached pages and ends the request; only then may worker retire. */
dbt_result_t dbt_storage_reserve_retire(dbt_storage_reserve_t *);
/* Assisted PCM producer only, after publishing a span and before releasing its
 * existing DSP borrow. The borrow prevents renderer lifecycle/target changes.
 * Publishes only two lock-free words; no callbacks or audio mutation. */
void dbt_storage_reserve_published(dbt_storage_reserve_t *,const dbt_audio_t *,uint32_t);
/* Render owner only, bounded one-state service. No allocation/release/copy.
 * running means DMA can consume PCM; stopping waits for actual close rather
 * than admitting a flash operation during the remaining fade. INVALID requires
 * the caller to close the audio object before cleanup can safely continue. */
dbt_result_t dbt_storage_reserve_render(dbt_storage_reserve_t *,dbt_audio_t *,uint32_t,bool,bool);
/* Render owner after close has freed attached pages; unattached worker pages
 * remain untouched. No worker ever reads the audio object. */
void dbt_storage_reserve_closed(dbt_storage_reserve_t *);
uint32_t dbt_storage_reserve_target(const dbt_storage_reserve_t *,uint32_t);
bool dbt_storage_reserve_blocks_start(const dbt_storage_reserve_t *);
/* Independent counters, not an atomic snapshot of all fields. */
void dbt_storage_reserve_stats(const dbt_storage_reserve_t *,dbt_storage_reserve_stats_t *);
#endif
