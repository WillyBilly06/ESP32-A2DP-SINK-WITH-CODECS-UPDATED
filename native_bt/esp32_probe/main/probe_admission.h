#ifndef PROBE_ADMISSION_H
#define PROBE_ADMISSION_H
#include "dbt_runtime.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Startup worker only, before the controller is enabled. Loads and validates
 * the bounded durable trust index and starts its single storage worker. */
dbt_result_t probe_admission_startup_worker(dbt_runtime_t *,TaskHandle_t);
/* The physical gesture grants new pairing for 120 seconds. Existing bonds
 * remain valid outside this window. All following calls run on the owner. */
void probe_admission_open(uint32_t now_ms);
/* Call after stores were successfully enumerated. An unprovisioned LE control
 * store may bootstrap independently of existing Classic audio bonds. */
void probe_admission_first_boot(uint32_t now_ms,unsigned classic_bonds);
bool probe_admission_window(uint32_t now_ms);
/* Normal speaker pairing is available while no Classic peer is encrypted.
 * This public audio policy never grants a BLE control/OTA trust cookie. */
bool probe_admission_classic_window(uint32_t now_ms);
/* 0 closed, 1 physical gesture, 2 unprovisioned LE power-on,
 * 4 unprovisioned LE connection. Idle public Classic pairing is separate. */
uint8_t probe_admission_origin(uint32_t now_ms);
/* Non-repeating physical/power-on prompt epoch, zero outside the window. */
uint32_t probe_admission_prompt_epoch(uint32_t now_ms);
/* Owner-only restart gate: block new pairing and reconnect starts while
 * allowing previously admitted security work and durable writes to finish. */
void probe_admission_freeze(bool frozen);
bool probe_admission_frozen(void);
/* Owner only, after full encrypted-session authorization and durable-key cache
 * validation. Renews one Classic procedure lease; opens no BLE/global window. */
bool probe_admission_classic_renew(const dbt_link_t *,uint32_t now_ms);
bool probe_admission_classic_pair(const dbt_link_t *,uint32_t now_ms);
bool probe_admission_classic_store(const uint8_t address[6],uint32_t now_ms);
bool probe_admission_le_begin(const dbt_link_t *,uint32_t now_ms);
void probe_admission_link(const dbt_link_change_t *);
void probe_admission_poll(uint32_t now_ms);
/* Owner-only: invalidate a published cookie as soon as link/security state
 * changes, before processing later owner events. */
void probe_admission_revalidate(void);
uint32_t probe_admission_wait_ms(uint32_t now_ms);
/* DBT_OK means live encrypted committed SMP bond AND matching committed local
 * admission, bound to exact handle, generation, identity and incarnation. */
dbt_result_t probe_admission_trusted_le(uint16_t,uint32_t,uint8_t *,uint8_t[6],uint8_t[32]);
/* Capture on owner; validate on any worker without reading mutable host state.
 * Cookies never repeat, and exhaustion disables further admission. */
uint32_t probe_admission_control_cookie(uint16_t,uint32_t);
bool probe_admission_control_cookie_valid(uint16_t,uint32_t,uint32_t);
/* Owner-only; false for any queued/running trust operation or pending grant. */
bool probe_admission_quiescent(void);
#endif
