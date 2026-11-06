#ifndef PROBE_SCHEDULER_H
#define PROBE_SCHEDULER_H
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>
#include <stdint.h>
enum { PROBE_SAMPLE_HOST,PROBE_SAMPLE_DECODER,PROBE_SAMPLE_RENDERER,
       PROBE_SAMPLE_LOGGER,PROBE_SAMPLE_CONTROLLER,PROBE_SAMPLE_IDLE,
       PROBE_SAMPLE_OTHER,PROBE_SAMPLE_ROLES };
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
#ifdef DBT_PROBE_CODEC_AWARE_RENDER
/* Publish the optional permanent Core0 renderer handle after its lazy creation. */
void probe_scheduler_bind_renderer0(TaskHandle_t task);
#endif
/* Host-owner task only; returns previous phase for nested controller calls.
 * One writer, atomic load/store only. No timer, lock, allocation or logging. */
unsigned probe_scheduler_host_phase(unsigned phase);
#endif
