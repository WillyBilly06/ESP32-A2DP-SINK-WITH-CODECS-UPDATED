#ifndef PROBE_SCHEDULER_H
#define PROBE_SCHEDULER_H
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
 * One writer, atomic load/store only. No timer, lock, allocation or logging. */
unsigned probe_scheduler_host_phase(unsigned phase);
#endif
