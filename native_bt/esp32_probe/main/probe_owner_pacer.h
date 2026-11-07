#ifndef PROBE_OWNER_PACER_H
#define PROBE_OWNER_PACER_H
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>
#include <stdint.h>
#define PROBE_OWNER_SLICE_US 1000u
#define PROBE_OWNER_RELEASE_US 500u
/* Host-owner task only except the atomic, independently readable snapshot.
 * On-demand one-shot release; callbacks never wait or perform protocol work. */
esp_err_t probe_owner_pacer_init(TaskHandle_t);
bool probe_owner_pacer_ready(void);
bool probe_owner_pacer_charge(bool immediate_work,bool audio_pressure);
void probe_owner_pacer_waited(uint32_t elapsed_us);
typedef struct {
    uint32_t enabled,waiting,failed,pauses,releases,wait_checks,max_slice_us,max_pause_us,arm_failures;
} probe_owner_pacer_stats_t;
void probe_owner_pacer_snapshot(probe_owner_pacer_stats_t *);
#endif
