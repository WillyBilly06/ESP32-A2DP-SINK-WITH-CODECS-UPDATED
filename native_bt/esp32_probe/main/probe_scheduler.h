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
typedef struct {
    uint32_t counts[PROBE_SAMPLE_ROLES],at_us,max_gap_us,max_gap_begin_us,max_gap_end_us;
    uint32_t max_gap_previous_role,max_gap_current_role,max_callback_cycles,known_task_mask;
    bool coherent;
} probe_scheduler_snapshot_t;
/* Init-only, once per boot, after all permanent audio/controller tasks exist.
 * No task/queue/allocation is added. Failed registration is rolled back. */
esp_err_t probe_scheduler_start(TaskHandle_t host,TaskHandle_t decoder,TaskHandle_t logger);
void probe_scheduler_snapshot(unsigned core,probe_scheduler_snapshot_t *out);
#endif
