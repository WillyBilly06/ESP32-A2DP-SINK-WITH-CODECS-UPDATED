#include "dbt_input.h"
#include <string.h>
#define PUSH(k, d)                                                             \
  do {                                                                         \
    if (n >= DBT_INPUT_QUEUE_CAPACITY)                                         \
      return DBT_INPUT_FEED_BUSY;                                              \
    pending[n++] = (dbt_input_action_t){(k), (d)};                             \
  } while (0)
void dbt_input_init(dbt_input_t *i) {
  memset(i, 0, sizeof(*i));
  atomic_init(&i->head, 0);
  atomic_init(&i->tail, 0);
}
static bool delta_for(int32_t from, int32_t to, int32_t *delta) {
  uint32_t u = (uint32_t)to - (uint32_t)from;
  if (u <= 4096u) {
    *delta = (int32_t)u;
    return true;
  }
  if (u >= UINT32_MAX - 4095u) {
    *delta = -(int32_t)(0u - u);
    return true;
  }
  return false;
}
dbt_input_feed_result_t
dbt_input_feed_ex(dbt_input_t *i, const dbt_input_sample_t *v, bool fresh) {
  if (!i || !v)
    return DBT_INPUT_FEED_INVALID;
  dbt_input_state_t s = i->state;
  dbt_input_action_t pending[DBT_INPUT_QUEUE_CAPACITY];
  unsigned n = 0;
  if (!s.initialized) {
    for (unsigned j = 0; j < 4; j++)
      s.last[j] = v->position[j];
    s.gpio_raw = s.gpio_stable = v->gpio_buttons;
    s.gpio_ignore = v->gpio_buttons;
    s.initialized = true;
    i->state = s;
    return DBT_INPUT_FEED_ACCEPTED;
  }
  bool partial = false;
  for (unsigned j = 0; j < 4; j++) {
    int32_t delta;
    if (!delta_for(s.last[j], v->position[j], &delta))
      return DBT_INPUT_FEED_INVALID;
    if (delta > 20) {
      delta = 20;
      partial = true;
    } else if (delta < -20) {
      delta = -20;
      partial = true;
    }
    uint32_t next = (uint32_t)s.last[j] + (uint32_t)delta;
    memcpy(&s.last[j], &next, sizeof next);
    s.accum[j] += delta;
  }
  if (fresh)
    for (unsigned j = 0; j < 4; j++) {
      bool pressed = (v->enc_buttons & (1u << j)) != 0;
      if (pressed) {
        if (s.debounce[j] < 3)
          s.debounce[j]++;
      } else
        s.debounce[j] = 0;
      bool edge = s.debounce[j] >= 3 && !(s.enc_pressed & (1u << j));
      if (s.debounce[j] >= 3)
        s.enc_pressed |= (uint8_t)(1u << j);
      else
        s.enc_pressed &= (uint8_t)~(1u << j);
      if (edge) {
        if (j == 0 || j == 3) {
          unsigned c = j == 0 ? 0 : 1;
          if (s.clicks[c] < 3)
            s.clicks[c]++;
          s.click_time[c] = v->ms;
        } else if (j == 1)
          s.brightness_mode ^= 1u;
        else
          PUSH(DBT_INPUT_PAIR, 0);
      }
    }
  if (fresh && s.clicks[0] && v->ms - s.click_time[0] > 300u) {
    PUSH(s.clicks[0] == 1   ? DBT_INPUT_PLAY_PAUSE
         : s.clicks[0] == 2 ? DBT_INPUT_NEXT
                            : DBT_INPUT_PREVIOUS,
         0);
    s.clicks[0] = 0;
  }
  if (fresh && s.clicks[1] && v->ms - s.click_time[1] > 300u) {
    if (s.clicks[1] == 1) {
      s.effect_mode ^= 1u;
      if (!s.effect_mode)
        PUSH(DBT_INPUT_EFFECT_CONFIRM, 0);
    }
    s.clicks[1] = 0;
  }
  for (unsigned j = 0; j < 4; j++) {
    while (s.accum[j] >= 4 || s.accum[j] <= -4) {
      int8_t sign = s.accum[j] > 0 ? 1 : -1;
      s.accum[j] -= sign * 4;
      dbt_input_kind_t k = j == 0   ? DBT_INPUT_VOLUME
                           : j == 1 ? (s.brightness_mode ? DBT_INPUT_BRIGHTNESS
                                                         : DBT_INPUT_EQ_BASS)
                           : j == 2 ? DBT_INPUT_EQ_MID
                                    : (s.effect_mode ? DBT_INPUT_EFFECT_PREVIEW
                                                     : DBT_INPUT_EQ_TREBLE);
      PUSH(k, (int8_t)(sign *
                       (j == 0 ? 3 : (j == 1 && s.brightness_mode ? 5 : 1))));
    }
  }
  if (fresh)
    for (unsigned j = 0; j < 3; j++) {
      unsigned bit = 1u << j;
      bool raw = (v->gpio_buttons & bit) != 0;
      if (raw != ((s.gpio_raw & bit) != 0)) {
        s.gpio_raw ^= (uint8_t)bit;
        s.gpio_changed[j] = v->ms;
      }
      if (raw != ((s.gpio_stable & bit) != 0) &&
          v->ms - s.gpio_changed[j] > 25u) {
        s.gpio_stable ^= (uint8_t)bit;
        if (!raw && (s.gpio_ignore & bit)) {
          s.gpio_ignore &= (uint8_t)~bit;
          continue;
        }
        if (j == 0) {
          if (raw)
            s.gpio_down = v->ms;
          else {
            PUSH(v->ms - s.gpio_down < 1000u ? DBT_INPUT_BASS_BOOST
                                             : DBT_INPUT_CHANNEL_FLIP,
                 0);
          }
        } else if (!raw) {
          PUSH(j == 1 ? DBT_INPUT_BYPASS : DBT_INPUT_EFFECT_NEXT, 0);
        }
      }
    }
  unsigned h = atomic_load_explicit(&i->head, memory_order_relaxed),
           t = atomic_load_explicit(&i->tail, memory_order_acquire);
  if (n > DBT_INPUT_QUEUE_CAPACITY - (h - t))
    return DBT_INPUT_FEED_BUSY;
  for (unsigned j = 0; j < n; j++)
    i->queue[(h + j) % DBT_INPUT_QUEUE_CAPACITY] = pending[j];
  i->state = s;
  atomic_store_explicit(&i->head, h + n, memory_order_release);
  return partial ? DBT_INPUT_FEED_PARTIAL : DBT_INPUT_FEED_ACCEPTED;
}
bool dbt_input_feed(dbt_input_t *i, const dbt_input_sample_t *v) {
  if (!i || !v)
    return false;
  if (i->state.initialized)
    for (unsigned j = 0; j < 4; j++) {
      int32_t d;
      if (!delta_for(i->state.last[j], v->position[j], &d) || d > 20 || d < -20)
        return false;
    }
  return dbt_input_feed_ex(i, v, true) == DBT_INPUT_FEED_ACCEPTED;
}
unsigned dbt_input_rebase_invalid(dbt_input_t *i, const dbt_input_sample_t *v) {
  if (!i || !v || !i->state.initialized)
    return 0;
  unsigned count = 0;
  for (unsigned j = 0; j < 4; j++) {
    int32_t d;
    if (!delta_for(i->state.last[j], v->position[j], &d)) {
      i->state.last[j] = v->position[j];
      i->state.accum[j] = 0;
      count++;
    }
  }
  return count;
}
bool dbt_input_next(dbt_input_t *i, dbt_input_action_t *a) {
  if (!i || !a)
    return false;
  unsigned t = atomic_load_explicit(&i->tail, memory_order_relaxed),
           h = atomic_load_explicit(&i->head, memory_order_acquire);
  if (t == h)
    return false;
  *a = i->queue[t % DBT_INPUT_QUEUE_CAPACITY];
  atomic_store_explicit(&i->tail, t + 1, memory_order_release);
  return true;
}
