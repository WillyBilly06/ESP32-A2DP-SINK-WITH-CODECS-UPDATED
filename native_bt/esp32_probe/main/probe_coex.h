#ifndef PROBE_COEX_H
#define PROBE_COEX_H
#include <stdbool.h>
#include <stdint.h>
#include <stdatomic.h>
#define PROBE_COEX_STREAMING 0x10u
#define PROBE_COEX_PAUSED 0x20u
typedef enum {PROBE_COEX_QUEUED=1,PROBE_COEX_COMPLETED=2,PROBE_COEX_FAILED=3} probe_coex_stage_t;
enum {PROBE_COEX_WORK_IDLE,PROBE_COEX_WORK_READY,PROBE_COEX_WORK_RUNNING,PROBE_COEX_WORK_DONE};
typedef struct probe_coex probe_coex_t;
/* Owner-only bounded diagnostic callback; never wait or format here. */
typedef void (*probe_coex_note_fn)(const probe_coex_t *,probe_coex_stage_t,int32_t sdk_result);
struct probe_coex {
    probe_coex_note_fn note;
    uint32_t token;
    uint8_t known,applied,submitted_status,submitted_op;
    bool pending,failed;
    _Atomic uint32_t work_state;
    /* Single coherent owner publication for the logger, independent of event
     * ring loss: known8, applied8, pending bit16, failed bit17. */
    _Atomic uint32_t state_snapshot;
    int32_t sdk_result;
    uint32_t started_us,finished_us;
};
/* Zero-init once. Context lives until the auxiliary worker retires. Owner and
 * worker must not reset it while a job is borrowed. One immutable mailbox job;
 * newer desired state is reconciled after completion. No heap, wait or spin in
 * owner APIs. SDK calls can block and run ONLY on an existing background worker.
 * SDK success is API completion, not RF scheduling or independent bit readback. */
void probe_coex_poll(probe_coex_t *,bool controller_ready,bool streaming);
uint32_t probe_coex_wait_ms(const probe_coex_t *,bool controller_ready,bool streaming);
bool probe_coex_work_pending(const probe_coex_t *);
/* One strong claim, at most one SDK call; true means a completion was published.
 * The caller wakes the owner after true. Core metadata is the actual caller. */
bool probe_coex_worker_step(probe_coex_t *,unsigned core);
#endif
