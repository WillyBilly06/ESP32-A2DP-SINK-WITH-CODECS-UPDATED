#ifndef PROBE_PAIR_BUTTON_H
#define PROBE_PAIR_BUTTON_H
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Local BOOT/GPIO0 consent: release after startup, then hold for three seconds.
 * ISR only wakes the existing owner. No task, queue, timer or runtime allocation.
 * Poll/wait run only on that owner; true is one gesture, not authorization. */
bool probe_pair_button_init(TaskHandle_t owner);
bool probe_pair_button_poll(uint32_t now_ms);
uint32_t probe_pair_button_wait_ms(uint32_t now_ms);
#endif
