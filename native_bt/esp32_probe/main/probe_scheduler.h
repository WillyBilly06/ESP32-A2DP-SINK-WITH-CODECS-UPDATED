#ifndef PROBE_SCHEDULER_H
#define PROBE_SCHEDULER_H
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>
#include <stdint.h>
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
#ifdef DBT_PROBE_CODEC_AWARE_RENDER
/* Publish the optional permanent Core0 renderer handle after its lazy creation. */
void probe_scheduler_bind_renderer0(TaskHandle_t task);
#endif
/* Host-owner task only; returns previous phase for nested controller calls.
 * One writer, atomic load/store only. No timer, lock, allocation or logging. */
unsigned probe_scheduler_host_phase(unsigned phase);
#endif
