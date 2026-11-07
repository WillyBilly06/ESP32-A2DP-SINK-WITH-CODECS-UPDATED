#ifndef PROBE_CONSOLE_H
#define PROBE_CONSOLE_H
#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
/* Init before controller/worker startup. Preserve console pins, baud and
 * newline modes. No task or TX ring. Background writes wait on UART IRQs. */
esp_err_t probe_console_irq_init(bool *created);
/* Background logger only. One bounded, single-line record with final LF.
 * Preserve configured stdout newline conversion in caller-owned storage, then
 * use one driver write. May sleep for TX space; never call from callbacks/ISR. */
bool probe_console_write_record(char *record,size_t length,size_t capacity);
#endif
