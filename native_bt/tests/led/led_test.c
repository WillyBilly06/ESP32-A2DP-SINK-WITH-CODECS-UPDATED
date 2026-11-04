#include "dbt_led.h"
#include <stddef.h>
#ifdef _WIN32
#define EX __declspec(dllexport)
void *memcpy(void *d, const void *s, size_t n) {
  unsigned char *a = d;
  const unsigned char *b = s;
  for (size_t i = 0; i < n; i++)
    a[i] = b[i];
  return d;
}
void *memset(void *d, int v, size_t n) {
  unsigned char *a = d;
  for (size_t i = 0; i < n; i++)
    a[i] = (unsigned char)v;
  return d;
}
#else
#define EX
#endif
#define CHECK(x)                                                               \
  do {                                                                         \
    if (!(x))                                                                  \
      return __LINE__;                                                         \
  } while (0)
typedef struct {
  int commits, shows;
  bool commit_ok, show_ok;
  uint8_t last[10];
  uint8_t frame[32 * 32 * 3];
} fake_t;
static bool commit(void *p, const uint8_t *b) {
  fake_t *f = p;
  f->commits++;
  for (unsigned i = 0; i < 10; i++)
    f->last[i] = b[i];
  return f->commit_ok;
}
static bool show(void *p, const uint8_t *b, size_t n) {
  fake_t *f = p;
  f->shows++;
  if (n > sizeof f->frame)
    return false;
  for (size_t i = 0; i < n; i++)
    f->frame[i] = b[i];
  return f->show_ok;
}
EX int test_led(void) {
  dbt_led_t l;
  uint8_t pixels[48];
  static fake_t f;
  f.commit_ok = true;
  f.show_ok = true;
  dbt_led_io_t io = {commit, show, &f};
  uint8_t legacy[10] = {63, 12, 34, 56, 78, 90, 123, 4, 75, 21};
  CHECK(dbt_led_init(&l, 4, 4, pixels, sizeof pixels, &io, legacy));
  CHECK(l.legacy[0] == 63);
  dbt_led_request_t r = {0};
  r.token = 1;
  r.operation = DBT_LED_SET_EFFECT;
  r.settings.bytes[0] = 6;
  CHECK(dbt_led_submit(&l, &r) == DBT_LED_OK);
  dbt_led_receipt_t result;
  CHECK(!dbt_led_receipt(&l, &result));
  CHECK(dbt_led_work(&l));
  CHECK(dbt_led_receipt(&l, &result));
  CHECK(result.result == DBT_LED_OK && result.committed && result.token == 1);
  CHECK(f.commits == 1 && f.shows == 1);
  CHECK(f.last[0] == 63 && f.last[9] == 6 && f.last[7] == 4);
  r.token = 2;
  r.operation = DBT_LED_SET_BRIGHTNESS;
  r.settings.bytes[1] = 100;
  f.commit_ok = false;
  CHECK(dbt_led_submit(&l, &r) == DBT_LED_OK);
  CHECK(dbt_led_work(&l));
  CHECK(dbt_led_receipt(&l, &result));
  CHECK(result.result == DBT_LED_STORAGE_FAILED && !result.committed &&
        l.legacy[0] == 63 && f.shows == 1);
  f.commit_ok = true;
  f.show_ok = false;
  r.token = 3;
  CHECK(dbt_led_submit(&l, &r) == DBT_LED_OK);
  CHECK(dbt_led_work(&l));
  CHECK(dbt_led_receipt(&l, &result));
  CHECK(result.result == DBT_LED_TRANSFER_FAILED && result.committed &&
        l.legacy[0] == 255);
  f.show_ok = true;
  r.token = 4;
  r.operation = DBT_LED_PREVIEW_EFFECT;
  r.settings.bytes[0] = 1;
  CHECK(dbt_led_submit(&l, &r) == DBT_LED_OK);
  CHECK(dbt_led_work(&l));
  CHECK(dbt_led_receipt(&l, &result));
  CHECK(result.result == DBT_LED_OK && !result.committed && l.legacy[9] == 6 &&
        l.settings.bytes[0] == 1);
  r.token = 5;
  r.operation = DBT_LED_SET_EFFECT;
  r.settings.bytes[0] = 22;
  CHECK(dbt_led_submit(&l, &r) == DBT_LED_INVALID);
  r.operation = DBT_LED_SET_BRIGHTNESS;
  r.settings.bytes[1] = 101;
  CHECK(dbt_led_submit(&l, &r) == DBT_LED_INVALID);
  r.settings.bytes[1] = 50;
  for (unsigned i = 0; i < 4; i++) {
    r.token = 10 + i;
    CHECK(dbt_led_submit(&l, &r) == DBT_LED_OK);
  }
  CHECK(dbt_led_submit(&l, &r) == DBT_LED_BUSY);
  for (unsigned i = 0; i < 4; i++)
    CHECK(dbt_led_work(&l));
  CHECK(!dbt_led_work(&l));
  for (unsigned i = 0; i < 4; i++) {
    CHECK(dbt_led_receipt(&l, &result));
    CHECK(result.token == 10 + i);
  }
  dbt_led_meter(&l, 255, 50, 0);
  CHECK((atomic_load(&l.meter) & 255) == 100);
  CHECK(dbt_led_render(&l));
  r.token = 99;
  r.operation = DBT_LED_SET_ALL;
  const uint8_t ambient[10] = {21, 100, 0, 255, 0, 0, 0, 0, 255, 1};
  for (unsigned j = 0; j < 10; j++)
    r.settings.bytes[j] = ambient[j];
  CHECK(dbt_led_submit(&l, &r) == DBT_LED_OK);
  CHECK(dbt_led_work(&l));
  CHECK(dbt_led_receipt(&l, &result) && result.result == DBT_LED_OK);
  CHECK(f.last[0] == 255 && f.last[1] == 255 && f.last[6] == 255 &&
        f.last[7] == 1 && f.last[9] == 21);
  CHECK(f.frame[0] == 0 && f.frame[1] == 255 && f.frame[2] == 0);
  CHECK(f.frame[9] == 0 && f.frame[10] == 255 && f.frame[11] == 0);
  CHECK(f.frame[12] == 0 && f.frame[13] == 255 && f.frame[14] == 0);
  CHECK(f.frame[21] == 0 && f.frame[22] == 255 && f.frame[23] == 0);
  uint8_t unknown[10] = {64, 1, 2, 3, 4, 5, 6, 0, 50, 22};
  CHECK(!dbt_led_init(&l, 4, 4, pixels, sizeof pixels, &io, unknown));
  return 0;
}

EX int test_all_effects(void) {
  dbt_led_t l;
  uint8_t pixels[16 * 16 * 3];
  static fake_t f;
  f.commit_ok = f.show_ok = true;
  dbt_led_io_t io = {commit, show, &f};
  CHECK(dbt_led_init(&l, 16, 16, pixels, sizeof pixels, &io, 0));
  dbt_led_audio_active(&l, true);
  uint32_t hashes[22] = {0};
  for (unsigned e = 0; e < 22; e++) {
    dbt_led_request_t request = {0};
    request.token = e + 1;
    request.operation = DBT_LED_SET_ALL;
    uint8_t wire[10] = {(uint8_t)e, 100, 50, 255, 0, 128, 0, 128, 255, 1};
    for (unsigned i = 0; i < 10; i++)
      request.settings.bytes[i] = wire[i];
    CHECK(dbt_led_submit(&l, &request) == DBT_LED_OK);
    CHECK(dbt_led_work(&l));
    dbt_led_receipt_t receipt;
    CHECK(dbt_led_receipt(&l, &receipt) && receipt.result == DBT_LED_OK &&
          receipt.committed && receipt.token == e + 1);
    unsigned lit = 0;
    uint32_t hash = 2166136261u;
    for (unsigned n = 0; n < 80; n++) {
      dbt_led_meter(&l, (n / 8u) % 2u ? 100 : 20, 50, 70);
      CHECK(dbt_led_render(&l));
      for (unsigned k = 0; k < sizeof pixels; k++) {
        lit += pixels[k] != 0;
        hash = (hash ^ pixels[k]) * 16777619u;
      }
    }
    CHECK(lit > 0);
    hashes[e] = hash;
    for (unsigned prior = 0; prior < e; prior++)
      CHECK(hashes[prior] != hash);
  }
  /* Bass Reactor ID 14: one loud sample appears at the center, then moves to
   * the next ring after a quiet sample, matching the baseline ring history. */
  dbt_led_request_t ring = {0};
  ring.token = 77;
  ring.operation = DBT_LED_SET_EFFECT;
  ring.settings.bytes[0] = 14;
  dbt_led_meter(&l, 100, 0, 0);
  CHECK(dbt_led_submit(&l, &ring) == DBT_LED_OK);
  CHECK(dbt_led_work(&l));
  dbt_led_receipt_t rr;
  CHECK(dbt_led_receipt(&l, &rr));
  unsigned center = (7u * 16u + 8u) * 3u, outer = (6u * 16u + 7u) * 3u;
  CHECK((pixels[center] | pixels[center + 1] | pixels[center + 2]) != 0);
  CHECK((pixels[outer] | pixels[outer + 1] | pixels[outer + 2]) == 0);
  dbt_led_meter(&l, 0, 0, 0);
  CHECK(dbt_led_render(&l));
  CHECK((pixels[center] | pixels[center + 1] | pixels[center + 2]) == 0);
  CHECK((pixels[outer] | pixels[outer + 1] | pixels[outer + 2]) != 0);
  return 0;
}

EX int test_matrix_bounds(void) {
  dbt_led_t l;
  static uint8_t storage[32 * 32 * 3 + 2];
  static fake_t f;
  f.commit_ok = f.show_ok = true;
  dbt_led_io_t io = {commit, show, &f};
  for (unsigned size_index = 0; size_index < 2; size_index++) {
    unsigned side = size_index ? 32 : 1;
    storage[0] = 0x5a;
    storage[side * side * 3 + 1] = 0xa5;
    CHECK(!dbt_led_init(&l, 0, (uint16_t)side, storage + 1, side * side * 3,
                        &io, 0));
    CHECK(!dbt_led_init(&l, 33, (uint16_t)side, storage + 1, side * side * 3,
                        &io, 0));
    CHECK(!dbt_led_init(&l, (uint16_t)side, (uint16_t)side, storage + 1,
                        side * side * 3 - 1, &io, 0));
    CHECK(dbt_led_init(&l, (uint16_t)side, (uint16_t)side, storage + 1,
                       side * side * 3, &io, 0));
    dbt_led_audio_active(&l, true);
    for (unsigned e = 0; e < 22; e++)
      for (unsigned g = 0; g < 5; g++) {
        dbt_led_request_t r = {0};
        r.token = 1 + e * 5 + g;
        r.operation = DBT_LED_SET_ALL;
        uint8_t wire[10] = {(uint8_t)e, 100, 255, 255, 255,
                            255,        0,   0,   0,   (uint8_t)g};
        for (unsigned k = 0; k < 10; k++)
          r.settings.bytes[k] = wire[k];
        CHECK(dbt_led_submit(&l, &r) == DBT_LED_OK);
        CHECK(dbt_led_work(&l));
        dbt_led_receipt_t rec;
        CHECK(dbt_led_receipt(&l, &rec) && rec.result == DBT_LED_OK);
        CHECK(storage[0] == 0x5a && storage[side * side * 3 + 1] == 0xa5);
      }
  }
  return 0;
}
