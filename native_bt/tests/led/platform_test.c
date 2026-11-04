#include "dbt_led_esp32.h"
#include "nvs.h"
#include <stddef.h>
#include <stdint.h>
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
static int step, fail, transmit_count;
static uint8_t sent[64];
static size_t sent_bytes;
static uint8_t journal[36], saved[10], bright, audio_effect;
static int32_t effect;
static bool has_journal, has_settings, has_bright, has_effect, has_audio;
static int load_error;
static bool same(const char *a, const char *b) {
  while (*a && *a == *b) {
    a++;
    b++;
  }
  return *a == *b;
}
static bool gate(void) { return ++step != fail; }
static void reset_store(void) {
  has_journal = has_settings = has_bright = has_effect = has_audio = false;
  step = fail = load_error = 0;
}
int nvs_open(const char *n, int mode, nvs_handle_t *h) {
  (void)mode;
  if (!gate())
    return 88;
  *h = n[0] == 'a' ? 1 : 2;
  return 0;
}
void nvs_close(nvs_handle_t h) { (void)h; }
int nvs_set_u8(nvs_handle_t h, const char *k, uint8_t v) {
  if (!gate())
    return 88;
  if (h == 1 && same(k, "led_effect")) {
    audio_effect = v;
    has_audio = true;
  } else if (h == 2 && same(k, "bright")) {
    bright = v;
    has_bright = true;
  } else
    return 89;
  return 0;
}
int nvs_set_i32(nvs_handle_t h, const char *k, int32_t v) {
  if (!gate())
    return 88;
  if (h != 2 || !same(k, "effect"))
    return 89;
  effect = v;
  has_effect = true;
  return 0;
}
int nvs_set_blob(nvs_handle_t h, const char *k, const void *v, size_t z) {
  if (!gate())
    return 88;
  if (h != 2)
    return 89;
  if (same(k, "dbt_intent") && z == 36) {
    memcpy(journal, v, z);
    has_journal = true;
  } else if (same(k, "settings") && z == 10) {
    memcpy(saved, v, z);
    has_settings = true;
  } else
    return 89;
  return 0;
}
int nvs_commit(nvs_handle_t h) {
  (void)h;
  return gate() ? 0 : 88;
}
int nvs_get_blob(nvs_handle_t h, const char *k, void *v, size_t *z) {
  (void)h;
  if (load_error)
    return load_error;
  uint8_t *src;
  size_t n;
  if (same(k, "dbt_intent") && has_journal) {
    src = journal;
    n = 36;
  } else if (same(k, "settings") && has_settings) {
    src = saved;
    n = 10;
  } else
    return ESP_ERR_NVS_NOT_FOUND;
  if (*z < n)
    return 90;
  *z = n;
  memcpy(v, src, n);
  return 0;
}
int nvs_get_i32(nvs_handle_t h, const char *k, int32_t *v) {
  (void)h;
  (void)k;
  if (!has_effect)
    return ESP_ERR_NVS_NOT_FOUND;
  *v = effect;
  return 0;
}
int nvs_get_u8(nvs_handle_t h, const char *k, uint8_t *v) {
  (void)h;
  (void)k;
  if (h == 1) {
    if (!has_audio)
      return ESP_ERR_NVS_NOT_FOUND;
    *v = audio_effect;
    return 0;
  }
  if (!has_bright)
    return ESP_ERR_NVS_NOT_FOUND;
  *v = bright;
  return 0;
}
int spi_device_transmit(spi_device_handle_t h, spi_transaction_t *t) {
  (void)h;
  transmit_count++;
  sent_bytes = t->length / 8;
  if (sent_bytes > sizeof sent)
    return 1;
  for (size_t i = 0; i < sent_bytes; i++)
    sent[i] = ((const uint8_t *)t->tx_buffer)[i];
  return 0;
}
int spi_bus_remove_device(spi_device_handle_t h) {
  (void)h;
  return 0;
}
int spi_bus_free(spi_host_device_t h) {
  (void)h;
  return 0;
}
int spi_bus_initialize(spi_host_device_t h, const spi_bus_config_t *c, int d) {
  (void)h;
  (void)c;
  (void)d;
  return 0;
}
int spi_bus_add_device(spi_host_device_t h,
                       const spi_device_interface_config_t *c,
                       spi_device_handle_t *d) {
  (void)h;
  (void)c;
  *d = (void *)1;
  return 0;
}
void *heap_caps_malloc(size_t n, unsigned f) {
  (void)n;
  (void)f;
  return 0;
}
void heap_caps_free(void *p) { (void)p; }
int64_t esp_timer_get_time(void) { return 0; }
int xTaskCreate(void (*f)(void *), const char *n, unsigned z, void *p,
                unsigned pr, TaskHandle_t *h) {
  (void)f;
  (void)n;
  (void)z;
  (void)p;
  (void)pr;
  (void)h;
  return 0;
}
unsigned ulTaskNotifyTake(int b, unsigned t) {
  (void)b;
  (void)t;
  return 0;
}
void xTaskNotifyGive(TaskHandle_t t) { (void)t; }
void vTaskDelete(TaskHandle_t t) { (void)t; }
void vTaskSuspend(TaskHandle_t t) { (void)t; }
SemaphoreHandle_t xSemaphoreCreateBinary(void) { return 0; }
int xSemaphoreGive(SemaphoreHandle_t s) {
  (void)s;
  return 1;
}
int xSemaphoreTake(SemaphoreHandle_t s, unsigned t) {
  (void)s;
  (void)t;
  return 1;
}
void vSemaphoreDelete(SemaphoreHandle_t s) { (void)s; }
#include "../../src/esp32/dbt_led_esp32.c"
#define CHECK(x)                                                               \
  do {                                                                         \
    if (!(x))                                                                  \
      return __LINE__;                                                         \
  } while (0)
EX int test_platform(void) {
  const uint8_t target[10] = {64, 1, 2, 3, 4, 5, 6, 1, 50, 14};
  uint8_t blob[10];
  dbt_led_esp32_t context = {0};
  reset_store();
  CHECK(commit(&context, target));
  CHECK(step == 16);
  CHECK(has_audio && audio_effect == 14 && has_effect && effect == 14 &&
        has_bright && bright == 64 && has_settings);
  CHECK(has_journal && journal[4] == 2);
  for (int cut = 1; cut <= 16; cut++) {
    reset_store();
    context.storage_fault = false;
    fail = cut;
    CHECK(!commit(&context, target));
    CHECK(context.storage_fault);
    int old_step = step;
    CHECK(!commit(&context, target));
    CHECK(step == old_step);
    fail = 0;
    CHECK(recover());
    if (cut <= 4) {
      CHECK(!has_audio && !has_settings && !has_effect && !has_bright);
    } else {
      CHECK(has_audio && audio_effect == 14 && has_effect && effect == 14 &&
            has_bright && bright == 64 && has_settings);
      for (unsigned j = 0; j < 10; j++)
        CHECK(saved[j] == target[j]);
      CHECK(journal[4] == 2);
    }
  }
  /* Repeated interruption of recovery always converges when no external edit
   * occurs. */
  for (int cut = 1; cut <= 14; cut++) {
    reset_store();
    context.storage_fault = false;
    fail = 5;
    CHECK(!commit(&context, target));
    CHECK(has_journal && journal[4] == 1);
    step = 0;
    fail = cut;
    (void)recover();
    fail = 0;
    CHECK(recover());
    CHECK(has_audio && audio_effect == 14 && has_effect && effect == 14 &&
          has_bright && bright == 64 && has_settings && journal[4] == 2);
    for (unsigned k = 0; k < 10; k++)
      CHECK(saved[k] == target[k]);
  }
  /* An intervening unrelated baseline edit is not overwritten by replay. */
  reset_store();
  context.storage_fault = false;
  fail = 8;
  CHECK(!commit(&context, target));
  fail = 0;
  audio_effect = 42;
  has_audio = true;
  CHECK(!recover());
  CHECK(audio_effect == 42 && journal[4] == 1);
  reset_store();
  context.storage_fault = false;
  CHECK(commit(&context, target));
  /* A completed old journal must not overwrite a later baseline application
   * edit. */
  saved[9] = 7;
  CHECK(recover());
  CHECK(saved[9] == 7);
  journal[8] ^= 1;
  CHECK(!recover());
  journal[8] ^= 1;
  reset_store();
  load_error = 88;
  CHECK(load_settings(blob) == -1);
  load_error = 0;
  CHECK(load_settings(blob) == 1);
  CHECK(blob[0] == 64 && blob[9] == 0);
  uint8_t dma[64], rgb[3] = {0x80, 0x00, 0xff};
  dbt_led_esp32_t p = {0};
  p.spi = (void *)1;
  p.dma = dma;
  p.dma_bytes = sizeof dma;
  CHECK(show(&p, rgb, 3));
  CHECK(transmit_count == 1 && sent_bytes == 34);
  CHECK(sent[0] == 0 && sent[1] == 0);
  CHECK(sent[2] == 0x17 && sent[3] == 0x11 && sent[4] == 0x11 &&
        sent[5] == 0x11);
  CHECK(sent[10] == 0x77 && sent[13] == 0x77);
  for (size_t i = 14; i < 34; i++)
    CHECK(sent[i] == 0);
  return 0;
}
