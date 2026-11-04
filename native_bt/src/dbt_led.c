#include "dbt_led.h"
#include <string.h>

static uint8_t clamp100(uint8_t n) { return n > 100 ? 100 : n; }
static uint8_t to255(uint8_t n) {
  unsigned v = ((unsigned)n * 41u) >> 4;
  return v > 255 ? 255 : (uint8_t)v;
}
void dbt_led_default(dbt_led_settings_t *s) {
  static const uint8_t d[10] = {0, 25, 50, 255, 0, 128, 0, 128, 255, 0};
  if (s)
    memcpy(s->bytes, d, 10);
}
bool dbt_led_effect_supported(uint8_t e) { return e < DBT_LED_EFFECT_COUNT; }
bool dbt_led_settings_valid(const dbt_led_settings_t *s) {
  return s && dbt_led_effect_supported(s->bytes[0]) && s->bytes[1] <= 100 &&
         s->bytes[9] <= 4;
}
void dbt_led_to_legacy(uint8_t o[10], const dbt_led_settings_t *s) {
  o[0] = to255(s->bytes[1]);
  memcpy(o + 1, s->bytes + 3, 6);
  o[7] = s->bytes[9];
  o[8] = s->bytes[2];
  o[9] = s->bytes[0];
}
void dbt_led_from_legacy(dbt_led_settings_t *s, const uint8_t p[10]) {
  s->bytes[0] = p[9];
  s->bytes[1] = (uint8_t)(((unsigned)p[0] * 100u + 127u) / 255u);
  s->bytes[2] = p[8];
  memcpy(s->bytes + 3, p + 1, 6);
  s->bytes[9] = p[7];
}
bool dbt_led_init(dbt_led_t *l, uint16_t w, uint16_t h, uint8_t *f, size_t n,
                  const dbt_led_io_t *io, const uint8_t legacy[10]) {
  if (!l || !f || !io || !io->show || !io->commit || !w || !h || w > 32 ||
      h > 32 || n < (size_t)w * h * 3)
    return false;
  memset(l, 0, sizeof(*l));
  l->width = w;
  l->height = h;
  l->frame = f;
  l->frame_bytes = n;
  l->io = *io;
  l->active_effect = 255;
  atomic_init(&l->meter, 0);
  atomic_init(&l->audio_active, false);
  atomic_init(&l->request_head, 0);
  atomic_init(&l->request_tail, 0);
  atomic_init(&l->receipt_head, 0);
  atomic_init(&l->receipt_tail, 0);
  dbt_led_default(&l->settings);
  dbt_led_to_legacy(l->legacy, &l->settings);
  if (legacy) {
    dbt_led_settings_t saved;
    dbt_led_from_legacy(&saved, legacy);
    if (!dbt_led_settings_valid(&saved))
      return false;
    l->settings = saved;
    memcpy(l->legacy, legacy, 10);
  }
  return true;
}
void dbt_led_meter(dbt_led_t *l, uint8_t b, uint8_t m, uint8_t t) {
  if (l)
    atomic_store_explicit(&l->meter,
                          (unsigned)clamp100(b) | ((unsigned)clamp100(m) << 8) |
                              ((unsigned)clamp100(t) << 16),
                          memory_order_release);
}
void dbt_led_audio_active(dbt_led_t *l, bool active) {
  if (l)
    atomic_store_explicit(&l->audio_active, active, memory_order_release);
}
dbt_led_result_t dbt_led_submit(dbt_led_t *l, const dbt_led_request_t *r) {
  if (!l || !r || !r->token || r->operation < DBT_LED_SET_ALL ||
      r->operation > DBT_LED_CONFIRM_PREVIEW)
    return DBT_LED_INVALID;
  if (r->operation == DBT_LED_SET_ALL && !dbt_led_settings_valid(&r->settings))
    return DBT_LED_INVALID;
  if ((r->operation == DBT_LED_SET_EFFECT ||
       r->operation == DBT_LED_PREVIEW_EFFECT) &&
      !dbt_led_effect_supported(r->settings.bytes[0]))
    return DBT_LED_INVALID;
  if (r->operation == DBT_LED_SET_BRIGHTNESS && r->settings.bytes[1] > 100)
    return DBT_LED_INVALID;
  if ((r->operation == DBT_LED_ADJUST_BRIGHTNESS && r->delta != 5 &&
       r->delta != -5) ||
      ((r->operation == DBT_LED_ADJUST_EFFECT ||
        r->operation == DBT_LED_PREVIEW_DELTA) &&
       r->delta != 1 && r->delta != -1) ||
      (r->operation == DBT_LED_CONFIRM_PREVIEW && r->delta != 0))
    return DBT_LED_INVALID;
  unsigned h = atomic_load_explicit(&l->request_head, memory_order_relaxed),
           t = atomic_load_explicit(&l->request_tail, memory_order_acquire);
  if (h - t >= DBT_LED_QUEUE_CAPACITY)
    return DBT_LED_BUSY;
  l->requests[h % DBT_LED_QUEUE_CAPACITY] = *r;
  atomic_store_explicit(&l->request_head, h + 1, memory_order_release);
  return DBT_LED_OK;
}
bool dbt_led_receipt(dbt_led_t *l, dbt_led_receipt_t *out) {
  if (!l || !out)
    return false;
  unsigned t = atomic_load_explicit(&l->receipt_tail, memory_order_relaxed),
           h = atomic_load_explicit(&l->receipt_head, memory_order_acquire);
  if (t == h)
    return false;
  *out = l->receipts[t % DBT_LED_QUEUE_CAPACITY];
  atomic_store_explicit(&l->receipt_tail, t + 1, memory_order_release);
  return true;
}
static unsigned tri(unsigned v) {
  v &= 255u;
  return v < 128u ? v * 2u : (255u - v) * 2u;
}
/* Integer parabolic sine matching the baseline's sin8 shape. */
static unsigned wave(unsigned v) {
  unsigned x = v & 127u;
  unsigned y = (x * (128u - x)) >> 5;
  if (y > 127)
    y = 127;
  return (v & 128u) ? 128u - y : 128u + y;
}
static unsigned absdiff(unsigned a, unsigned b) {
  return a > b ? a - b : b - a;
}
static unsigned root(unsigned v) {
  unsigned r = 0, bit = 1u << 14;
  while (bit > v)
    bit >>= 2;
  while (bit) {
    if (v >= r + bit) {
      v -= r + bit;
      r = (r >> 1) + bit;
    } else
      r >>= 1;
    bit >>= 2;
  }
  return r;
}
static uint8_t mix(uint8_t a, uint8_t b, unsigned t) {
  return (uint8_t)(((unsigned)a * (255u - t) + (unsigned)b * t + 127u) / 255u);
}
typedef struct {
  uint8_t r, g, b;
} colour_t;
static colour_t hsv(unsigned h, unsigned sat, unsigned val) {
  h &= 255u;
  unsigned region = (h * 6u) >> 8, rem = (h * 6u) & 255u;
  unsigned p = (val * (255u - sat) + 128u) >> 8,
           q = (val * (255u - ((sat * rem + 128u) >> 8)) + 128u) >> 8,
           t = (val * (255u - ((sat * (255u - rem) + 128u) >> 8)) + 128u) >> 8;
  switch (region) {
  case 0:
    return (colour_t){val, t, p};
  case 1:
    return (colour_t){q, val, p};
  case 2:
    return (colour_t){p, val, t};
  case 3:
    return (colour_t){p, q, val};
  case 4:
    return (colour_t){t, p, val};
  default:
    return (colour_t){val, p, q};
  }
}
static unsigned noise(unsigned x, unsigned y, unsigned t) {
  uint32_t z =
      (x + 1u) * 0x9e3779b9u ^ (y + 17u) * 0x85ebca6bu ^ (t + 3u) * 0xc2b2ae35u;
  z ^= z >> 16;
  z *= 0x7feb352du;
  z ^= z >> 15;
  return z & 255u;
}
/* probe_output_meter is a linear mapping of -60..0 dB, not amplitude. */
static const uint8_t meter_linear[101] = {
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
    1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,  3,  3,  3,  3,
    3,  4,  4,  4,  4,  5,  5,  5,  6,  6,  7,  7,  8,  8,  9,  10, 10,
    11, 12, 13, 13, 14, 15, 17, 18, 19, 20, 22, 23, 25, 27, 29, 31, 33,
    35, 38, 41, 44, 47, 50, 54, 58, 62, 66, 71, 76, 81, 87, 93, 100};
static bool frame(dbt_led_t *l, const dbt_led_settings_t *s, uint8_t bright) {
  unsigned m = atomic_load_explicit(&l->meter, memory_order_acquire),
           bass = m & 255u, mid = (m >> 8) & 255u, high = (m >> 16) & 255u;
  unsigned raw_bass = bass, raw_mid = mid, raw_high = high;
  bass = meter_linear[bass];
  mid = meter_linear[mid];
  high = meter_linear[high];
  unsigned e = s->bytes[0], w = l->width, h = l->height;
  if (l->active_effect != e) {
    l->active_effect = (uint8_t)e;
    l->frame_number = 0;
    l->effect_hue = 0;
    memset(l->effect_state, 0, sizeof l->effect_state);
  }
  unsigned f = ++l->frame_number;
  if (atomic_load_explicit(&l->audio_active, memory_order_acquire))
    l->inactive_frames = 0;
  else if (l->inactive_frames < 151)
    l->inactive_frames++;
  bool demo = l->inactive_frames >= 151;
  if (demo) {
    bass = 30u + wave(f * 2u) * 30u / 255u;
    mid = 20u + wave(f * 3u + 85u) * 20u / 255u;
    high = 10u + wave(f * 4u + 170u) * 20u / 255u;
    raw_bass = 75u + bass / 4u;
    raw_mid = 70u + mid / 4u;
    raw_high = 65u + high / 4u;
  }
  bool beat =
      (bass > l->previous_bass + 8u && bass > 35u) || (demo && f % 30u == 0);
  if (beat) {
    l->pulse = 255;
    l->effect_hue = (uint8_t)(l->effect_hue + 32u);
  } else
    l->pulse = (uint8_t)(l->pulse * 217u / 256u);
  l->previous_bass = (uint8_t)bass;
  if (e == 14) {
    for (unsigned k = 7; k > 0; k--)
      l->effect_state[k] = (uint8_t)(l->effect_state[k - 1] * 230u / 256u);
    l->effect_state[0] = (uint8_t)(bass * 255u / 100u);
  }
  if (e == 18) {
    for (unsigned k = 0; k < 15; k++)
      l->effect_state[k] = l->effect_state[k + 1];
    l->effect_state[15] =
        (uint8_t)(8u + (bass * 5u + mid * 3u + high * 2u) * 7u / 1000u);
  }
  if (e == 16) {
    unsigned target = (bass * 5u + mid * 3u + high * 2u) * 255u / 1000u;
    unsigned v = l->effect_state[0];
    if (v < target)
      v += (target - v + 3u) / 4u;
    else
      v -= (v - target + 7u) / 8u;
    l->effect_state[0] = (uint8_t)v;
  }
  if (e == 3) {
    for (unsigned yy = 0; yy < 15; yy++)
      for (unsigned xx = 0; xx < 16; xx++) {
        unsigned a = l->effect_state[(yy + 1) * 16 + xx],
                 b = l->effect_state[(yy + 1) * 16 + (xx + 1) % 16],
                 cool = noise(xx, yy, f) % 20;
        unsigned heat = (a * 2u + b) / 3u;
        l->effect_state[yy * 16 + xx] =
            (uint8_t)(heat > cool ? heat - cool : 0);
      }
    for (unsigned xx = 0; xx < 16; xx++)
      if (noise(xx, 15, f) < 80u + bass * 120u / 100u)
        l->effect_state[240 + xx] = (uint8_t)(160u + noise(xx, 14, f) % 96u);
  }
  for (unsigned y = 0; y < h; y++)
    for (unsigned x = 0; x < w; x++) {
      unsigned X = x * 16u / w, Y = y * 16u / h, dx = absdiff(2u * X, 15u),
               dy = absdiff(2u * Y, 15u), dist = root(dx * dx + dy * dy) / 2u;
      colour_t c = {0, 0, 0};
      unsigned hue = 0, value = 0;
      switch (e) {
      case 0: {
        unsigned band = X < 5 ? (raw_bass * (5u - X) + raw_mid * X) / 5u
                        : X < 10
                            ? (raw_mid * (10u - X) + raw_high * (X - 5u)) / 5u
                            : raw_high;
        unsigned bar = band * 16u / 100u;
        if (15u - Y < bar)
          c = hsv(Y >= 11 ? 96 : Y >= 6 ? 48 : 0, 255, 255);
        break;
      }
      case 1:
        c = hsv(l->effect_hue, 255, 20u + l->pulse * 235u / 255u);
        break;
      case 2: {
        unsigned radius = (255u - l->pulse) / 16u;
        if (l->pulse && absdiff(dist, radius) <= 1)
          c = hsv(l->effect_hue, 255, l->pulse);
        break;
      }
      case 3: {
        unsigned heat = l->effect_state[Y * 16u + X];
        c = (colour_t){heat < 85 ? heat * 3u : 255u,
                       heat < 85    ? 0
                       : heat < 170 ? (heat - 85u) * 3u
                                    : 255u,
                       heat < 170 ? 0 : (heat - 170u) * 3u};
        break;
      }
      case 4:
        hue = (wave(X * 16u + f) + wave(Y * 16u + 2u * f) +
               wave((X + Y) * 8u + f) + wave(dist * 8u - f)) /
              4u;
        c = hsv(hue, 255, 128u + wave(hue + f) / 2u);
        break;
      case 5: {
        unsigned head = (f * (3u + bass / 20u) / 8u + noise(X, 0, 0)) % 24u,
                 trail = (head + 24u - Y) % 24u;
        if (trail < 6)
          c = trail == 0 ? (colour_t){255, 255, 255}
                         : (colour_t){0, 255u - trail * 40u, 0};
        break;
      }
      case 6: {
        unsigned level = (bass * 6u + mid * 3u + high) * 2u / 10u;
        if (level > 100)
          level = 100;
        if (X == 7 || X == 8)
          c = hsv(f + Y * 16u + (X == 8 ? 128 : 0), 255, 100);
        else if ((15u - Y) * 100u < 16u * level)
          c = hsv(Y >= 11 ? 96 : Y >= 6 ? 48 : 0, 255, 255);
        break;
      }
      case 7: {
        unsigned star = noise(X, Y, 0);
        if (star > 215)
          c = (colour_t){wave(f * (2u + star % 6u) + star),
                         wave(f * (2u + star % 6u) + star),
                         wave(f * (2u + star % 6u) + star)};
        else
          c = hsv(160, 255, bass / 5u);
        break;
      }
      case 8: {
        int yy = (int)Y;
        int y1 = 8 + ((int)wave(f * 4u + X * 16u) - 128) *
                         (3 + (int)bass * 4 / 100) / 128;
        int y2 = 8 + ((int)wave(f * 6u + X * 24u + 40u) - 128) *
                         (2 + (int)mid * 3 / 100) / 128;
        int y3 = 8 + ((int)wave(f * 8u + X * 32u + 80u) - 128) *
                         (1 + (int)high * 2 / 100) / 128;
        if (yy == y1)
          c = (colour_t){255, 0, 0};
        if (yy == y2)
          c = (colour_t){0, 255, 0};
        if (yy == y3)
          c = (colour_t){0, 0, 255};
        break;
      }
      case 9: {
        unsigned age = (255u - l->pulse) / 6u;
        unsigned seed = noise(X, Y, age / 12u);
        if (l->pulse &&
            ((age < 16 && X == 7 && Y == 15 - age / 2u) ||
             (age >= 16 && absdiff(dist, (age - 16) / 3u) <= 1 && seed > 100)))
          c = hsv(l->effect_hue + seed / 8u, 255, l->pulse);
        break;
      }
      case 10:
        hue = f + X * 8u + Y * 8u + wave(f + X * 16u) / 4u;
        c = hsv(hue, 255, 128u + bass * 127u / 100u);
        break;
      case 11: {
        unsigned radius = (255u - l->pulse) / 14u;
        if (l->pulse && absdiff(dist, radius) <= 1 && noise(X, Y, f / 4u) > 90)
          c = hsv(l->effect_hue + noise(X, Y, 1) / 8u, 255, l->pulse);
        if (dist <= 1)
          c = hsv(l->effect_hue, 255, 50u + bass);
        break;
      }
      case 12: {
        unsigned xx = X < 8 ? 7 - X : X - 8, yy = Y < 8 ? 7 - Y : Y - 8;
        unsigned d = root(xx * xx + yy * yy);
        hue = f + mid / 2u + d * 20u + xx * 9u + yy * 5u;
        value = 100u + wave(d * 30u - f * 2u) / 2u + bass * 80u / 100u;
        if (value > 255)
          value = 255;
        c = hsv(hue, 255, value);
        break;
      }
      case 13: {
        unsigned a = (X * 17u + Y * 29u + dist * 24u + f * 3u) % 85u;
        if (a < 12u && dist < 2u + (bass + mid + high) * 6u / 300u)
          c = hsv((a * 3u + dist * 10u + f) & 255u, 255, 255u - dist * 20u);
        break;
      }
      case 14:
        if (dist < 8)
          c = hsv(f + dist * 20u, 255,
                  l->effect_state[dist] *
                      (200u + wave(f * 3u + dist * 30u) / 5u) / 256u);
        break;
      case 15: {
        unsigned head = (f * 3u + X * 5u + noise(X, 0, 1)) % 26u;
        unsigned trail = (head + 26u - Y) % 26u;
        if (trail < 4 && bass > 20)
          c = hsv(25u + X * 8u, 255, 255u - trail * 60u);
        break;
      }
      case 16:
        value =
            l->effect_state[0] * (dist < 11u ? 255u - dist * 22u : 25u) / 255u;
        c = hsv(f + (beat ? 15u : 0u) + dist * 3u, 220, value);
        break;
      case 17: {
        int amp = 4 + (int)bass * 3 / 100;
        int x1 =
            8 + ((int)wave(f * (2u + mid / 20u) + Y * 20u) - 128) * amp / 128;
        int x2 = 16 - x1;
        if ((int)X == x1)
          c = hsv(bass * 60u / 100u, 255, 180u + bass * 75u / 100u);
        else if ((int)X == x2)
          c = hsv(160u + high * 60u / 100u, 255, 180u + high * 75u / 100u);
        else if ((Y + f / 4u) % 4u == 0 && (int)X > (x1 < x2 ? x1 : x2) &&
                 (int)X < (x1 > x2 ? x1 : x2))
          c = hsv(96, 200, 60u + mid);
        break;
      }
      case 18: {
        unsigned yy = l->effect_state[X];
        if (Y == yy)
          c = hsv(160u - absdiff(Y, 8u) * 15u, 255,
                  150u + absdiff(Y, 8u) * 13u);
        else if (Y == 8)
          c = (colour_t){15, 15, 15};
        break;
      }
      case 19: {
        for (unsigned k = 0; k < 3; k++) {
          unsigned xx = (k * 5u + 3u) % 16u,
                   yy = 15u - tri(f * (3u + k) + k * 70u) * 12u / 255u;
          if (absdiff(X, xx) <= 1 && absdiff(Y, yy) <= 1)
            c = hsv(k * 85u + f, 255, 255);
        }
        break;
      }
      case 20: {
        unsigned influence = 0, hues = 0;
        for (unsigned k = 0; k < 4; k++) {
          unsigned cx = 3u + wave(f * (1u + k) + k * 55u) * 9u / 255u,
                   cy = 3u + wave(f * (2u + k) + k * 87u) * 9u / 255u;
          unsigned dd = root(absdiff(X, cx) * absdiff(X, cx) +
                             absdiff(Y, cy) * absdiff(Y, cy));
          unsigned q = (3u + bass / 50u) * 64u / (dd + 1u);
          influence += q;
          hues += q * (k * 55u + f);
        }
        if (influence > 100u)
          c = hsv(hues / (influence ? influence : 1), 200,
                  influence > 200u ? 255u : (influence - 100u) * 255u / 100u);
        break;
      }
      case 21: {
        unsigned g = 0;
        switch (s->bytes[9]) {
        case 1:
          g = w > 1 ? x * 255u / (w - 1u) : 0;
          break;
        case 2:
          g = h > 1 ? y * 255u / (h - 1u) : 0;
          break;
        case 3: {
          unsigned xx = absdiff(2 * x, w - 1), yy = absdiff(2 * y, h - 1);
          unsigned max = root((w - 1) * (w - 1) + (h - 1) * (h - 1));
          g = max ? root(xx * xx + yy * yy) * 255u / max : 0;
          break;
        }
        case 4:
          g = (x + y) * 255u / (w + h - 2u + (w + h == 2));
          break;
        default:
          break;
        }
        if (s->bytes[9])
          g = (g + (f * s->bytes[2] / 10u)) % 255u;
        c = (colour_t){mix(s->bytes[3], s->bytes[6], g),
                       mix(s->bytes[4], s->bytes[7], g),
                       mix(s->bytes[5], s->bytes[8], g)};
        break;
      }
      default:
        break;
      }
      unsigned pos = (y * w + ((y & 1u) ? w - 1u - x : x)) * 3u;
      l->frame[pos] = (uint8_t)((unsigned)c.g * bright / 255u);
      l->frame[pos + 1] = (uint8_t)((unsigned)c.r * bright / 255u);
      l->frame[pos + 2] = (uint8_t)((unsigned)c.b * bright / 255u);
    }
  return l->io.show(l->io.context, l->frame, (size_t)w * h * 3u);
}
bool dbt_led_render(dbt_led_t *l) {
  return l && frame(l, &l->settings, l->legacy[0]);
}
bool dbt_led_work(dbt_led_t *l) {
  if (!l)
    return false;
  unsigned rt = atomic_load_explicit(&l->request_tail, memory_order_relaxed),
           rh = atomic_load_explicit(&l->request_head, memory_order_acquire);
  unsigned wh = atomic_load_explicit(&l->receipt_head, memory_order_relaxed),
           wt = atomic_load_explicit(&l->receipt_tail, memory_order_acquire);
  if (rt == rh || wh - wt >= DBT_LED_QUEUE_CAPACITY)
    return false;
  dbt_led_request_t r = l->requests[rt % DBT_LED_QUEUE_CAPACITY];
  uint8_t legacy[10];
  memcpy(legacy, l->legacy, 10);
  if (r.operation == DBT_LED_SET_ALL)
    dbt_led_to_legacy(legacy, &r.settings);
  else if (r.operation == DBT_LED_SET_BRIGHTNESS)
    legacy[0] = to255(r.settings.bytes[1]);
  else if (r.operation == DBT_LED_ADJUST_BRIGHTNESS) {
    int b = (int)legacy[0] + r.delta;
    if (b < 0)
      b = 0;
    if (b > 255)
      b = 255;
    legacy[0] = (uint8_t)b;
  } else if (r.operation == DBT_LED_ADJUST_EFFECT ||
             r.operation == DBT_LED_PREVIEW_DELTA) {
    int e = (int)l->settings.bytes[0] + r.delta;
    if (e < 0)
      e += DBT_LED_EFFECT_COUNT;
    if (e >= (int)DBT_LED_EFFECT_COUNT)
      e -= DBT_LED_EFFECT_COUNT;
    legacy[9] = (uint8_t)e;
  } else if (r.operation == DBT_LED_CONFIRM_PREVIEW)
    legacy[9] = l->settings.bytes[0];
  else
    legacy[9] = r.settings.bytes[0];
  dbt_led_settings_t s;
  dbt_led_from_legacy(&s, legacy);
  if (r.operation == DBT_LED_SET_BRIGHTNESS ||
      r.operation == DBT_LED_ADJUST_BRIGHTNESS)
    s.bytes[0] = l->settings.bytes[0];
  bool preview = r.operation == DBT_LED_PREVIEW_EFFECT ||
                 r.operation == DBT_LED_PREVIEW_DELTA;
  dbt_led_receipt_t result = {0};
  result.token = r.token;
  if (!preview && !l->io.commit(l->io.context, legacy))
    result.result = DBT_LED_STORAGE_FAILED;
  else {
    result.committed = !preview;
    /* A failed show can follow a successful durable commit; retain that state.
     */
    if (result.committed) {
      l->settings = s;
      memcpy(l->legacy, legacy, 10);
    }
    if (frame(l, &s, legacy[0])) {
      result.result = DBT_LED_OK;
      if (!result.committed)
        l->settings = s;
    } else
      result.result = DBT_LED_TRANSFER_FAILED;
  }
  result.settings = l->settings;
  l->receipts[wh % DBT_LED_QUEUE_CAPACITY] = result;
  atomic_store_explicit(&l->receipt_head, wh + 1, memory_order_release);
  atomic_store_explicit(&l->request_tail, rt + 1, memory_order_release);
  return true;
}
