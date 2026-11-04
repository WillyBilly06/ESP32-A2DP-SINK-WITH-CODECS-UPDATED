#ifndef PROBE_CODEC_CHECK_H
#define PROBE_CODEC_CHECK_H
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Completion runs on the finite codec worker. Do not block in the callback.
 * A rejected start never calls completion; accepted starts call it once.
 * The callback/context must remain valid until completion, including timeout. */
typedef void (*probe_codec_complete_fn)(void *context, bool passed);
bool probe_codec_check_start(probe_codec_complete_fn complete, void *context);
/* Startup-only compatibility wrapper: sleeps on notifications, not periodic
 * polling. A timeout does not cancel the worker or free its state. */
bool probe_codec_check(TaskHandle_t owner);
#endif
