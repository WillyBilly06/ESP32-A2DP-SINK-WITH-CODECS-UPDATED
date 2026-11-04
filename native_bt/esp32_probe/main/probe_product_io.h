#ifndef PROBE_PRODUCT_IO_H
#define PROBE_PRODUCT_IO_H
#include "dbt_ble_control.h"
#include "dbt_input.h"
#include "dbt_led.h"

/* Storage, output and board identity must be established before startup.
 * SPIFFS must already be mounted; no format or erase is attempted. Hooks and
 * the singleton live until worker retirement and owner-side input shutdown. */
typedef struct {
  void *context;
  bool (*peer_valid)(void *, const dbt_ble_control_peer_t *);
  void (*wake_owner)(void *); /* nonblocking, input task -> owner */
  bool led, input, sound;
  /* Default false: only two clean no-ACK probes may be optional. */
  bool input_optional;
} probe_product_io_hooks_t;

typedef enum {
  PROBE_PRODUCT_INPUT_NOT_REQUESTED,
  PROBE_PRODUCT_INPUT_NOT_ATTEMPTED,
  PROBE_PRODUCT_INPUT_AVAILABLE,
  PROBE_PRODUCT_INPUT_NO_ACK,
  PROBE_PRODUCT_INPUT_FAILED,
  PROBE_PRODUCT_INPUT_CLEANUP_FAILED,
  PROBE_PRODUCT_INPUT_STOPPED
} probe_product_input_status_t;
typedef struct {
  bool ready,led,sound;
  probe_product_input_status_t input;
} probe_product_io_capabilities_t;
/* One atomic snapshot, including after failed startup. NO_ACK means both
 * identity-register writes NACKed and both drivers were released; it is not
 * proof of physical absence, wiring, or device health. */
bool probe_product_io_capabilities(probe_product_io_capabilities_t *out);

/* Boot worker only; writes initial LED/sound snapshot for GATT configuration.
 * LED defaults SPI2/GPIO4/16x16, input I2C1/SDA23/SCL22/addresses 0x36/0x07 and
 * buttons 18/21/19; it probes 0x36 then 0x07 with Seesaw hardware/product ID
 * checks. Callers must verify the board's actual wiring before enabling. */
bool probe_product_io_startup_worker(dbt_ble_control_state_t *initial,
                                     const probe_product_io_hooks_t *hooks);
/* General GATT worker only. The callback context may be any immutable mux;
 * it is ignored. Completion ACK is returned only after the operation. */
bool probe_product_io_command(void *, const dbt_ble_control_peer_t *, uint8_t,
                              const uint8_t *, uint16_t, uint8_t *, uint8_t *);
/* General GATT worker only. Worker retirement quiesces jobs and loans but
 * preserves providers and the input task for the next worker. */
bool probe_product_io_worker_poll(void *, bool retiring);
/* Owner only, bounded/nonblocking. Generation is used to cancel sound uploads.
 */
void probe_product_io_peer_lost(void *, const dbt_ble_control_peer_t *);
/* Owner only: a popped event remains the owner's responsibility until applied.
 * EQ steps must be sent individually through the GATT EQ worker's immediate
 * persistence admission; retry on BUSY, never merge accepted EQ events. */
bool probe_product_io_take_input(dbt_input_action_t *out);
typedef struct {
  uint32_t read_errors, deferred_samples, feedback_errors, invalid_channels;
} probe_product_io_input_stats_t;
/* Atomic counters; invalid_channels counts explicit over-limit rebases, not
 * detents. */
bool probe_product_io_input_stats(probe_product_io_input_stats_t *out);
/* Owner only, nonblocking. Copied local LED requests are processed by the
 * general worker. Completion/result is returned through take_local_led. */
dbt_result_t probe_product_io_local_led(dbt_led_operation_t operation,
                                        uint8_t value, uint32_t token);
/* Relative requests are applied against the LED worker's latest state. */
dbt_result_t probe_product_io_local_led_relative(dbt_led_operation_t operation,
                                                 int8_t delta, uint32_t token);
bool probe_product_io_take_local_led(dbt_led_receipt_t *receipt);
/* Owner only, nonblocking; queue acceptance is not playback completion. */
dbt_result_t probe_product_io_play(uint8_t type, uint32_t rate, bool exclusive);
/* Cancel pending and playing UI sounds without freeing an output loan; the
 * worker performs the normal stop/retirement handshake. Owner only. */
void probe_product_io_cancel_sound(void);
/* Atomic nonblocking feedback publication. No settings persistence. */
void probe_product_io_pairing_mode(bool active);
void probe_product_io_pairing_success(void);
void probe_product_io_volume_feedback(uint8_t volume);
/* FIFO result for each accepted play request; OK means attached/prefilled,
 * not proof that the full sound was rendered or heard. */
typedef struct {
  uint8_t type;
  dbt_result_t result;
} probe_product_io_play_result_t;
bool probe_product_io_take_play_result(probe_product_io_play_result_t *result);
/* Atomic snapshot; true from immediately before play admission until both the
 * playback object and its output loan are retired. Gate product name/OTA while
 * true. */
bool probe_product_io_sound_busy(void);
/* Owner: whether the lazy general worker must be started/kept alive for
 * queued local work, an outstanding LED receipt, or a sound loan. */
bool probe_product_io_needs_worker(void);
/* Owner: receive copied asynchronous sound timeout notice, if any. */
bool probe_product_io_take_notice(dbt_ble_control_peer_t *peer, uint8_t out[3],
                                  uint8_t *size);
/* Owner: latest provider sound status after a successful change. */
uint8_t probe_product_io_sound_status(void);
/* Owner: bounded atomic snapshot of the latest applied or durable LED state,
 * including a durable write followed by transfer failure. */
bool probe_product_io_led_snapshot(dbt_led_settings_t *out);
/* Owner or output task, atomic publication only. */
void probe_product_io_meter(uint8_t bass, uint8_t mid, uint8_t treble,
                            bool active);
#endif
