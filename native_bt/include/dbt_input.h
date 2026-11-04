#ifndef DBT_INPUT_H
#define DBT_INPUT_H
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#define DBT_INPUT_QUEUE_CAPACITY 32u
/* EQ events are individual signed one-dB steps; owner must immediately persist
 * EACH accepted event through the same control/NVS path as a BLE EQ write. */
typedef enum {
  DBT_INPUT_VOLUME,
  DBT_INPUT_PLAY_PAUSE,
  DBT_INPUT_NEXT,
  DBT_INPUT_PREVIOUS,
  DBT_INPUT_EQ_BASS,
  DBT_INPUT_EQ_MID,
  DBT_INPUT_EQ_TREBLE,
  DBT_INPUT_BRIGHTNESS,
  DBT_INPUT_PAIR,
  DBT_INPUT_EFFECT_PREVIEW,
  DBT_INPUT_EFFECT_CONFIRM,
  DBT_INPUT_EFFECT_NEXT,
  DBT_INPUT_BASS_BOOST,
  DBT_INPUT_CHANNEL_FLIP,
  DBT_INPUT_BYPASS
} dbt_input_kind_t;
typedef struct {
  dbt_input_kind_t kind;
  int8_t delta;
} dbt_input_action_t;
/* Positions are signed Seesaw absolute counters; enc_buttons bit n is 1 when
 * pressed. gpio_buttons bits 0..2 are active high logical pressed states. */
typedef struct {
  int32_t position[4];
  uint8_t enc_buttons, gpio_buttons;
  uint32_t ms;
} dbt_input_sample_t;
typedef struct {
  int32_t last[4], accum[4];
  uint8_t debounce[4], enc_pressed, brightness_mode, effect_mode;
  uint8_t clicks[2];
  uint32_t click_time[2];
  uint8_t gpio_raw, gpio_stable, gpio_ignore;
  uint32_t gpio_changed[3], gpio_down;
  bool initialized;
} dbt_input_state_t;
typedef struct {
  dbt_input_state_t state;
  dbt_input_action_t queue[DBT_INPUT_QUEUE_CAPACITY];
  atomic_uint head, tail;
} dbt_input_t;
void dbt_input_init(dbt_input_t *input);
/* Single producer. On false, the sample was not accepted and state is
 * unchanged; retain and retry it. Huge/corrupt jumps are rejected and require a
 * deliberate reinitialization; no EQ actions are silently coalesced. */
bool dbt_input_feed(dbt_input_t *input, const dbt_input_sample_t *sample);
/* Partial draining processes at most 20 raw counts per encoder. fresh must be
 * true on the first attempt of a sampled snapshot and false on subsequent
 * accepted PARTIAL retries; BUSY leaves all state unchanged. Deltas larger
 * than 4096 counts or the ambiguous half-wrap return INVALID. */
typedef enum {
  DBT_INPUT_FEED_ACCEPTED,
  DBT_INPUT_FEED_PARTIAL,
  DBT_INPUT_FEED_BUSY,
  DBT_INPUT_FEED_INVALID
} dbt_input_feed_result_t;
dbt_input_feed_result_t
dbt_input_feed_ex(dbt_input_t *, const dbt_input_sample_t *, bool fresh);
/* Producer only. Explicitly rebase only invalid encoder positions, clear their
 * partial detents, return number rebased; these physical movements are lost. */
unsigned dbt_input_rebase_invalid(dbt_input_t *, const dbt_input_sample_t *);
/* Single owner consumer. A successful pop transfers one action to the owner. */
bool dbt_input_next(dbt_input_t *input, dbt_input_action_t *action);
#endif
