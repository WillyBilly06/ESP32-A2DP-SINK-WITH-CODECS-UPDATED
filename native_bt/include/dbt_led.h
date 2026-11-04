#ifndef DBT_LED_H
#define DBT_LED_H
#include <stdatomic.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define DBT_LED_SETTINGS_SIZE 10u
#define DBT_LED_QUEUE_CAPACITY 4u
#define DBT_LED_EFFECT_COUNT 22u

/* BLE wire: effect, brightness (0..100), speed, RGB1, RGB2, gradient. */
typedef struct {
  uint8_t bytes[DBT_LED_SETTINGS_SIZE];
} dbt_led_settings_t;
typedef enum {
  DBT_LED_OK,
  DBT_LED_INVALID,
  DBT_LED_BUSY,
  DBT_LED_STORAGE_FAILED,
  DBT_LED_TRANSFER_FAILED,
  DBT_LED_NO_MEMORY
} dbt_led_result_t;
typedef enum {
  DBT_LED_SET_ALL,
  DBT_LED_SET_EFFECT,
  DBT_LED_SET_BRIGHTNESS,
  DBT_LED_PREVIEW_EFFECT,
  DBT_LED_ADJUST_BRIGHTNESS,
  DBT_LED_ADJUST_EFFECT,
  DBT_LED_PREVIEW_DELTA,
  DBT_LED_CONFIRM_PREVIEW
} dbt_led_operation_t;
typedef struct {
  uint32_t token;
  dbt_led_operation_t operation;
  dbt_led_settings_t settings;
  int8_t delta;
} dbt_led_request_t;
typedef struct {
  uint32_t token;
  dbt_led_result_t result;
  bool committed;
  dbt_led_settings_t settings;
} dbt_led_receipt_t;
typedef struct {
  /* Both callbacks run only on the LED worker, and borrow buffers only until
   * return. */
  bool (*commit)(void *context, const uint8_t legacy_blob[10]);
  bool (*show)(void *context, const uint8_t *grb, size_t bytes);
  void *context;
} dbt_led_io_t;
typedef struct {
  uint16_t width, height;
  uint8_t *frame;
  size_t frame_bytes;
  dbt_led_settings_t settings;
  uint8_t legacy[10];
  dbt_led_io_t io;
  atomic_uint meter;
  atomic_bool audio_active;
  atomic_uint pairing_event, volume_event,beat_event;
  uint32_t pairing_seen,volume_seen,beat_seen,render_now_ms,pairing_started,volume_started;
  uint32_t volume_smooth,overlay_frame;
  uint8_t pairing_state,volume_target;
  bool volume_visible;
  dbt_led_request_t requests[DBT_LED_QUEUE_CAPACITY];
  dbt_led_receipt_t receipts[DBT_LED_QUEUE_CAPACITY];
  atomic_uint request_head, request_tail, receipt_head, receipt_tail;
  uint32_t frame_number;
  uint16_t random_state;
  uint8_t previous_bass, pulse, active_effect, effect_hue;
  void *effect_dynamic;
  void *effect_alloc_context;
  void *(*effect_allocate)(void *context,size_t bytes);
  void (*effect_release)(void *context,void *memory);
  uint8_t effect_state[256];
  uint32_t inactive_since_ms;
  bool idle_timer_started;
} dbt_led_t;

/* One producer (control owner) and one LED worker; receipts flow in reverse.
 * Tokens must never be reused during this engine lifetime. No buffer is
 * retained by show(). Caller provides width*height*3 bytes for the lifetime of
 * the engine.
 */
bool dbt_led_init(dbt_led_t *led, uint16_t width, uint16_t height,
                  uint8_t *frame, size_t frame_bytes, const dbt_led_io_t *io,
                  const uint8_t legacy_blob[10]);
/* Install before first render. Only the active particle effect allocates;
 * switching releases it before allocating its successor. */
bool dbt_led_set_allocator(dbt_led_t *led,void *(*allocate)(void *,size_t),
                           void (*release)(void *,void *),void *context);
/* After worker has stopped and producers are quiescent. */
void dbt_led_deinit(dbt_led_t *led);
bool dbt_led_effect_supported(uint8_t effect);
bool dbt_led_settings_valid(const dbt_led_settings_t *settings);
void dbt_led_default(dbt_led_settings_t *settings);
void dbt_led_from_legacy(dbt_led_settings_t *out, const uint8_t legacy[10]);
void dbt_led_to_legacy(uint8_t out[10], const dbt_led_settings_t *settings);
/* Each band is an already computed 0..100 level, no DSP runs in this module. */
void dbt_led_meter(dbt_led_t *led, uint8_t bass, uint8_t mid, uint8_t treble);
/* Playback state for the baseline five-second idle demo timer. */
void dbt_led_audio_active(dbt_led_t *led, bool active);
/* Optional detector edge and its intensity (0..100); once used, this source
 * replaces the approximate onset heuristic until the engine is reinitialized. */
void dbt_led_beat(dbt_led_t *led,uint8_t intensity);
/* Single control owner publishes events; LED worker consumes newest state.
 * Values are A2DP absolute volume 0..127. These never change saved settings. */
void dbt_led_volume(dbt_led_t *led,uint8_t volume);
void dbt_led_pairing_mode(dbt_led_t *led,bool active);
void dbt_led_pairing_success(dbt_led_t *led);
dbt_led_result_t dbt_led_submit(dbt_led_t *led,
                                const dbt_led_request_t *request);
bool dbt_led_receipt(dbt_led_t *led, dbt_led_receipt_t *receipt);
/* LED worker only; returns true when it processed a job. It does not process a
 * job while its completion queue is full. At most one job per call. */
bool dbt_led_work(dbt_led_t *led);
/* LED worker only, e.g. 30 Hz. False indicates failed SPI transfer. */
bool dbt_led_render(dbt_led_t *led);
/* Worker monotonic time, wrapping naturally at 32 bits. */
bool dbt_led_render_at(dbt_led_t *led,uint32_t now_ms);
#endif
