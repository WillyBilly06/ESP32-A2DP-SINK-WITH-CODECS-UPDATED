#ifndef PROBE_SCHEDULER_H
#define PROBE_SCHEDULER_H
#ifdef DBT_PROBE_TASK_RUNTIME_ONLY
#define PROBE_TICK_SAMPLING_JSON "false"
#define PROBE_RUNTIME_ONLY_JSON "true"
#define PROBE_SCHEDULER_SAMPLE_HZ 0u
#else
#define PROBE_TICK_SAMPLING_JSON "true"
#define PROBE_RUNTIME_ONLY_JSON "false"
#define PROBE_SCHEDULER_SAMPLE_HZ ((unsigned)configTICK_RATE_HZ)
#endif
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>
#include <stdint.h>
#ifdef DBT_PROBE_TASK_CYCLE_ACCOUNTING
#include "probe_task_cycles.h"
#endif
enum { PROBE_SAMPLE_HOST,PROBE_SAMPLE_DECODER,PROBE_SAMPLE_RENDERER,
       PROBE_SAMPLE_LOGGER,PROBE_SAMPLE_CONTROLLER,PROBE_SAMPLE_IDLE,
       PROBE_SAMPLE_OTHER,PROBE_SAMPLE_IPC,PROBE_SAMPLE_TIMER,PROBE_SAMPLE_ROLES };
enum { PROBE_HOST_OTHER,PROBE_HOST_QUERY,PROBE_HOST_SUBMIT,PROBE_HOST_RUNTIME,
       PROBE_HOST_WAIT,PROBE_HOST_GATT,PROBE_HOST_SECURITY,PROBE_HOST_PRODUCT,
       PROBE_HOST_IO,PROBE_HOST_AUDIO,PROBE_HOST_AVRCP,PROBE_HOST_LOG,PROBE_HOST_PHASES };
typedef struct {
    uint32_t counts[PROBE_SAMPLE_ROLES],at_us,max_gap_us,max_gap_begin_us,max_gap_end_us;
    uint32_t host_phases[PROBE_HOST_PHASES];
    uint32_t max_gap_previous_role,max_gap_current_role,max_callback_cycles,known_task_mask;
    bool coherent;
} probe_scheduler_snapshot_t;
/* Init-only, once per boot, after all permanent audio/controller tasks exist.
 * No task/queue/allocation is added. Failed registration is rolled back. */
esp_err_t probe_scheduler_start(TaskHandle_t host,TaskHandle_t decoder,TaskHandle_t logger);
/* Tagging guard for helpers also used by background workers. No state change,
 * wait or kernel lock. A non-host caller must not alter the host's phase. */
bool probe_scheduler_host_context(void);
void probe_scheduler_snapshot(unsigned core,probe_scheduler_snapshot_t *out);
/* Bounded task-context observation of existing counters. No new ISR work.
 * One sequence check; inconsistent observations are explicit, never retried.
 * Counts describe interrupted-task samples, not precise CPU time. */
typedef struct {
    uint32_t counts[PROBE_SAMPLE_ROLES],host_phases[PROBE_HOST_PHASES];
    uint32_t at_us,known_task_mask,read_cycles,core;
    bool coherent;
#ifdef DBT_PROBE_TASK_CYCLE_ACCOUNTING
    probe_task_cycle_mark_t task;
#endif
} probe_scheduler_mark_t;
enum {PROBE_DELAY_LOCAL_RENDER,PROBE_DELAY_ASSIST_RETIRE,PROBE_DELAY_LOCAL_RENDER0,PROBE_DELAY_KINDS};
typedef struct {
    uint32_t revision,begin_us,end_us,wall_us,frames,begin_tick_us,end_tick_us,stream_serial,rate;
    uint32_t counts[PROBE_SAMPLE_ROLES],host_phases[PROBE_HOST_PHASES];
    uint32_t begin_read_cycles,end_read_cycles,known_task_mask;
    bool markers_valid,coherent;
} probe_scheduler_interval_t;
void probe_scheduler_mark(unsigned core,probe_scheduler_mark_t *);
/* One respective-core render/lifecycle writer per kind. Retains the longest wall interval; no log,
 * allocation, waits or retry. Start/end markers must describe the same core. */
void probe_scheduler_interval_observe(unsigned kind,const probe_scheduler_mark_t *,
    const probe_scheduler_mark_t *,uint32_t begin_us,uint32_t end_us,uint32_t frames,uint32_t stream_serial,uint32_t rate);
void probe_scheduler_interval_snapshot(unsigned kind,probe_scheduler_interval_t *);
#ifdef DBT_PROBE_TASK_CYCLE_ACCOUNTING
typedef struct {
    uint32_t revision,stream_serial,rate,calls,invalid,frames;
    uint32_t wall_cycles,charged_cycles,max_wall_cycles,charged_at_max_wall,max_charged_cycles;
    bool coherent;
} probe_render_account_t;
/* Only local-render kinds are counted. A cross-core helper-retirement marker
 * has no own-core running-tail observation and is deliberately not charged. */
void probe_scheduler_render_account(unsigned kind,probe_render_account_t *);
typedef struct {
    uint32_t revision,charged_cycles[PROBE_HOST_PHASES],intervals[PROBE_HOST_PHASES],invalid,wrong_context,wrong_pc,wrong_phase;
    bool coherent;
} probe_host_account_t;
/* Up to the last owner phase transition, not a running-tail read by logger. */
void probe_scheduler_host_account(probe_host_account_t *);
#endif
#ifdef DBT_PROBE_CODEC_AWARE_RENDER
/* Publish the optional permanent Core0 renderer handle after its lazy creation. */
void probe_scheduler_bind_renderer0(TaskHandle_t task);
#endif
#ifdef DBT_PROBE_TASK_RUNTIME_STATS
#define PROBE_RUNTIME_TASKS 11u
typedef struct {uint32_t counter_us,task_number,priority;int32_t core;} probe_task_runtime_t;
typedef struct {
    uint32_t begin_us,end_us,present_mask,max_query_us;
    probe_task_runtime_t tasks[PROBE_RUNTIME_TASKS];
} probe_task_runtime_snapshot_t;
/* Background logger only. At most11 public GetInfo calls on the permanent
 * handles registered above; skip stack scans and task-state queries. Each
 * call uses the SDK kernel lock. The snapshot has a reported read interval,
 * not an atomic simultaneous view. Runtime includes interrupted-task ISR time
 * and excludes the running task's unfinished quantum until its next switch. */
void probe_scheduler_task_runtime(probe_task_runtime_snapshot_t *);
#endif
/* Host-owner task only; returns previous phase for nested controller calls.
 * One writer. No timer, kernel lock, allocation or logging. The optional
 * task-cycle diagnostic adds one own-core interrupt-masked counter mark. */
unsigned probe_scheduler_host_phase(unsigned phase);
/* Optional task-context wall-cycle diagnostics. Nested categories are inclusive;
 * IRQ/preemption time and the small marker floor are included. No ISR hook.
 * A stream lifecycle has one logical publisher; only the host writes counters. */
#ifdef DBT_PROBE_SCHEDULER_TRACE
enum { PROBE_COST_RUNTIME_WAIT,PROBE_COST_PARAMETERS_WAIT,PROBE_COST_ADMISSION_WAIT,
       PROBE_COST_SMP_WAIT,PROBE_COST_AVRCP_WAIT,PROBE_COST_BUTTON_WAIT,
       PROBE_COST_RFCOMM_WAIT,PROBE_COST_GAP_WAIT,PROBE_COST_PRODUCT_WAIT,
       PROBE_COST_IO_WAIT,PROBE_COST_ATT_WAIT,PROBE_COST_SECURITY_WAIT,
       PROBE_COST_ECHO_WAIT,PROBE_COST_SDP_WAIT,PROBE_COST_AUDIO_WAIT,
       PROBE_COST_CLOCK_FLOOR,PROBE_COST_WAIT_TOTAL,PROBE_COST_POLL_BASE,
       PROBE_COST_SLOTS=PROBE_COST_POLL_BASE+PROBE_HOST_PHASES };
typedef struct {uint32_t epoch,cycles;} probe_owner_cost_mark_t;
typedef struct {uint32_t epoch,calls,cycles,min_cycles,max_cycles;} probe_owner_cost_t;
void probe_scheduler_owner_capture(bool active);
probe_owner_cost_mark_t probe_scheduler_owner_cost_begin(void);
void probe_scheduler_owner_cost_end(unsigned slot,probe_owner_cost_mark_t mark);
/* One attempt; false leaves output unchanged, never waits for the host. */
bool probe_scheduler_owner_cost_snapshot(unsigned slot,probe_owner_cost_t *out);
/* Completed epoch only: report after pause so diagnostics add no active UART load. */
uint32_t probe_scheduler_owner_cost_finished_epoch(uint32_t *seen_mask);
#define HOST_COST_VALUE(slot,destination,expression) do { \
    probe_owner_cost_mark_t probe_cost_value_mark=probe_scheduler_owner_cost_begin(); \
    (destination)=(expression); \
    probe_scheduler_owner_cost_end((slot),probe_cost_value_mark); \
} while(0)
#else
#define HOST_COST_VALUE(slot,destination,expression) do {(destination)=(expression);}while(0)
#endif
#endif
