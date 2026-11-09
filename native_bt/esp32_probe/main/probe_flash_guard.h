#ifndef PROBE_FLASH_GUARD_H
#define PROBE_FLASH_GUARD_H
#include <stdint.h>
/* Diagnostic token only, never a Bluetooth/codec flag: phase low2 bits
 * (0 finished,1 publishing,2 inside guard), initiator core bit2, serial above.
 * Active covers only after SDK disable returns through entry to SDK restore.
 * begin_us is sampled after disable returns, not the hardware cache edge.
 * The preparing/restore handshakes are conservatively excluded. */
typedef struct {uint32_t token,begin_us,scheduler_state,coherent;} probe_flash_guard_sample_t;
/* Enable once after timers/RTOS/audio clients are initialized. Earlier SDK
 * flash guards execute unchanged without reading an uninitialized timer. */
void probe_flash_guard_enable(void);
/* IRQ-safe one-attempt observation; no lock, retry, allocation or formatting. */
void probe_flash_guard_sample(probe_flash_guard_sample_t *);
/* Longest completed confirmed slice since arm; excludes SDK lock/IPC/restore
 * handshakes and end-wrapper observation overhead. Not a worst-case bound. */
typedef struct {uint32_t token,begin_us,elapsed_us,coherent;} probe_flash_guard_peak_t;
void probe_flash_guard_peak(probe_flash_guard_peak_t *);
#endif
