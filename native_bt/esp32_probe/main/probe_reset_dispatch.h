#ifndef PROBE_RESET_DISPATCH_H
#define PROBE_RESET_DISPATCH_H
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
/* Initialize once before controller enable. Owner calls request only after
 * selector/runtime/profile quiescence approval. No task wait or new stack. */
bool probe_reset_dispatch_init(TaskHandle_t owner);
bool probe_reset_dispatch_request(void);
bool probe_reset_dispatch_pending(void);
bool probe_reset_dispatch_failed(void);
#endif
