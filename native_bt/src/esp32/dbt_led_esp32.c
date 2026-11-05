#include "dbt_esp32_affinity.h"
#include "dbt_led_esp32.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "nvs.h"
#include <string.h>

/* 0 absent, 1 loaded, -1 read/corruption error. Never replace a failed read. */
static int load_settings(uint8_t b[10]) {
  nvs_handle_t n;
  esp_err_t e = nvs_open("led", NVS_READONLY, &n);
  if (e == ESP_ERR_NVS_NOT_FOUND)
    return 0;
  if (e != ESP_OK)
    return -1;
  size_t z = 10;
  e = nvs_get_blob(n, "settings", b, &z);
  if (e == ESP_OK && z == 10) {
    nvs_close(n);
    return 1;
  }
  if (e != ESP_ERR_NVS_NOT_FOUND) {
    nvs_close(n);
    return -1;
  }
  int32_t effect = 0;
  uint8_t brightness = 64;
  e = nvs_get_i32(n, "effect", &effect);
  if (e != ESP_OK && e != ESP_ERR_NVS_NOT_FOUND) {
    nvs_close(n);
    return -1;
  }
  e = nvs_get_u8(n, "bright", &brightness);
  nvs_close(n);
  if (e != ESP_OK && e != ESP_ERR_NVS_NOT_FOUND)
    return -1;
  if (effect < 0 || effect >= (int32_t)DBT_LED_EFFECT_COUNT)
    return -1;
  dbt_led_settings_t settings;
  dbt_led_default(&settings);
  dbt_led_to_legacy(b, &settings);
  b[0] = brightness;
  b[9] = (uint8_t)effect;
  return 1;
}
/* Intent is one checksummed, versioned NVS blob. DONE is retained rather than
 * erased. Only PENDING is replayed; DONE never overwrites later baseline edits.
 */
#define JOURNAL_BYTES 36u
/* Snapshot the exact prior existence/value of all baseline keys. A pending
 * intent is replayed only if every key still equals its old or desired value.
 */
typedef struct {
  uint8_t flags, blob[10], bright, audio;
  int32_t effect;
} key_snapshot_t;
static uint32_t journal_crc(const uint8_t *p, size_t n) {
  uint32_t c = UINT32_MAX;
  for (size_t i = 0; i < n; i++) {
    c ^= p[i];
    for (unsigned b = 0; b < 8; b++)
      c = (c >> 1) ^ (0xedb88320u & (0u - (c & 1u)));
  }
  return ~c;
}
static bool snapshot(key_snapshot_t *v) {
  memset(v, 0, sizeof(*v));
  nvs_handle_t n;
  esp_err_t e = nvs_open("led", NVS_READONLY, &n);
  if (e != ESP_OK && e != ESP_ERR_NVS_NOT_FOUND)
    return false;
  if (e == ESP_OK) {
    size_t z = 10;
    e = nvs_get_blob(n, "settings", v->blob, &z);
    if (e == ESP_OK) {
      if (z != 10) {
        nvs_close(n);
        return false;
      }
      v->flags |= 1;
    } else if (e != ESP_ERR_NVS_NOT_FOUND) {
      nvs_close(n);
      return false;
    }
    e = nvs_get_i32(n, "effect", &v->effect);
    if (e == ESP_OK)
      v->flags |= 2;
    else if (e != ESP_ERR_NVS_NOT_FOUND) {
      nvs_close(n);
      return false;
    }
    e = nvs_get_u8(n, "bright", &v->bright);
    if (e == ESP_OK)
      v->flags |= 4;
    else if (e != ESP_ERR_NVS_NOT_FOUND) {
      nvs_close(n);
      return false;
    }
    nvs_close(n);
  }
  e = nvs_open("audio", NVS_READONLY, &n);
  if (e == ESP_ERR_NVS_NOT_FOUND)
    return true;
  if (e != ESP_OK)
    return false;
  e = nvs_get_u8(n, "led_effect", &v->audio);
  if (e == ESP_OK)
    v->flags |= 8;
  else if (e != ESP_ERR_NVS_NOT_FOUND) {
    nvs_close(n);
    return false;
  }
  nvs_close(n);
  return true;
}
static void journal_finish(uint8_t j[JOURNAL_BYTES], uint8_t state) {
  j[4] = state;
  uint32_t c = journal_crc(j, 32);
  for (unsigned k = 0; k < 4; k++)
    j[32 + k] = (uint8_t)(c >> (k * 8));
}
static bool journal_write(const uint8_t j[JOURNAL_BYTES]) {
  nvs_handle_t n;
  if (nvs_open("led", NVS_READWRITE, &n) != ESP_OK)
    return false;
  esp_err_t e = nvs_set_blob(n, "dbt_intent", j, JOURNAL_BYTES);
  if (e == ESP_OK)
    e = nvs_commit(n);
  nvs_close(n);
  return e == ESP_OK;
}
static bool write_keys(const uint8_t b[10]) {
  nvs_handle_t n;
  if (nvs_open("audio", NVS_READWRITE, &n) != ESP_OK)
    return false;
  esp_err_t e = nvs_set_u8(n, "led_effect", b[9]);
  if (e == ESP_OK)
    e = nvs_commit(n);
  nvs_close(n);
  if (e != ESP_OK)
    return false;
  if (nvs_open("led", NVS_READWRITE, &n) != ESP_OK)
    return false;
  e = nvs_set_i32(n, "effect", b[9]);
  if (e == ESP_OK)
    e = nvs_set_u8(n, "bright", b[0]);
  if (e == ESP_OK)
    e = nvs_set_blob(n, "settings", b, 10);
  if (e == ESP_OK)
    e = nvs_commit(n);
  nvs_close(n);
  return e == ESP_OK;
}
static bool equal10(const uint8_t *a, const uint8_t *b) {
  for (unsigned i = 0; i < 10; i++)
    if (a[i] != b[i])
      return false;
  return true;
}
static bool old_or_new(bool present, bool old_present, bool old_eq,
                       bool new_eq) {
  return present ? (new_eq || (old_present && old_eq)) : !old_present;
}
static bool compatible(const uint8_t j[JOURNAL_BYTES]) {
  key_snapshot_t now;
  if (!snapshot(&now))
    return false;
  uint8_t old = j[5];
  const uint8_t *b = j + 6;
  uint32_t e = (uint32_t)j[26] | ((uint32_t)j[27] << 8) |
               ((uint32_t)j[28] << 16) | ((uint32_t)j[29] << 24);
  return old_or_new(now.flags & 1, old & 1, equal10(now.blob, j + 16),
                    equal10(now.blob, b)) &&
         old_or_new(now.flags & 2, old & 2, (uint32_t)now.effect == e,
                    now.effect == b[9]) &&
         old_or_new(now.flags & 4, old & 4, now.bright == j[30],
                    now.bright == b[0]) &&
         old_or_new(now.flags & 8, old & 8, now.audio == j[31],
                    now.audio == b[9]);
}
static bool recover(void) {
  nvs_handle_t n;
  esp_err_t e = nvs_open("led", NVS_READONLY, &n);
  if (e == ESP_ERR_NVS_NOT_FOUND)
    return true;
  if (e != ESP_OK)
    return false;
  uint8_t j[JOURNAL_BYTES];
  size_t size = sizeof j;
  e = nvs_get_blob(n, "dbt_intent", j, &size);
  nvs_close(n);
  if (e == ESP_ERR_NVS_NOT_FOUND)
    return true;
  if (e != ESP_OK || size != sizeof j || j[0] != 'D' || j[1] != 'L' ||
      j[2] != 'J' || j[3] != 2 || (j[4] != 1 && j[4] != 2) || (j[5] & 0xf0))
    return false;
  uint32_t c = (uint32_t)j[32] | ((uint32_t)j[33] << 8) |
               ((uint32_t)j[34] << 16) | ((uint32_t)j[35] << 24);
  if (c != journal_crc(j, 32))
    return false;
  dbt_led_settings_t check;
  dbt_led_from_legacy(&check, j + 6);
  if (!dbt_led_settings_valid(&check))
    return false;
  if (j[4] == 2)
    return true;
  if (!compatible(j) || !write_keys(j + 6))
    return false;
  journal_finish(j, 2);
  return journal_write(j);
}
static bool commit(void *ctx, const uint8_t b[10]) {
  dbt_led_esp32_t *p = ctx;
  if (!p || p->storage_fault)
    return false;
  key_snapshot_t old;
  uint8_t j[JOURNAL_BYTES] = {0};
  if (!snapshot(&old)) {
    p->storage_fault = true;
    return false;
  }
  j[0] = 'D';
  j[1] = 'L';
  j[2] = 'J';
  j[3] = 2;
  j[5] = old.flags;
  memcpy(j + 6, b, 10);
  memcpy(j + 16, old.blob, 10);
  for (unsigned k = 0; k < 4; k++)
    j[26 + k] = (uint8_t)((uint32_t)old.effect >> (k * 8));
  j[30] = old.bright;
  j[31] = old.audio;
  journal_finish(j, 1);
  bool ok = journal_write(j) && write_keys(b);
  if (ok) {
    journal_finish(j, 2);
    ok = journal_write(j);
  }
  if (!ok)
    p->storage_fault = true;
  return ok;
}
static bool show(void *ctx, const uint8_t *grb, size_t bytes) {
  dbt_led_esp32_t *p = ctx;
  if (!p || !p->spi || bytes * 4u + 22u > p->dma_bytes)
    return false;
  size_t n = 0;
  p->dma[n++] = 0;
  p->dma[n++] = 0;
  for (size_t i = 0; i < bytes; i++) {
    uint8_t c = grb[i];
    for (int shift = 6; shift >= 0; shift -= 2) {
      /* SPI sends LSB first; source bit 7 first, pulse 1 or 7. */
      uint8_t a = (c & (1u << (shift + 1))) ? 7u : 1u,
              b = (c & (1u << shift)) ? 7u : 1u;
      p->dma[n++] = (uint8_t)(a | (b << 4));
    }
  }
  memset(p->dma + n, 0, 20);
  n += 20;
  spi_transaction_t t = {0};
  t.length = n * 8u;
  t.tx_buffer = p->dma;
  return spi_device_transmit(p->spi, &t) == ESP_OK;
}
static void *effect_allocate(void *ctx,size_t n) {
  (void)ctx;return heap_caps_malloc(n,MALLOC_CAP_8BIT);
}
static void effect_release(void *ctx,void *memory) {
  (void)ctx;heap_caps_free(memory);
}
static void worker(void *arg) {
  dbt_led_esp32_t *p = arg;
  uint64_t next = 0;
  while (!atomic_load_explicit(&p->stopping, memory_order_acquire)) {
    (void)dbt_led_work(&p->led);
    uint64_t now = (uint64_t)esp_timer_get_time() / 1000u;
    if (now >= next) {
      (void)dbt_led_render_at(&p->led,(uint32_t)now);
      next = now + 33u;
    }
    /* timeout also retries display failures without blocking the host */
    (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
  }
  xSemaphoreGive(p->stopped);
  vTaskSuspend(NULL);
}
static void release(dbt_led_esp32_t *p) {
  dbt_led_deinit(&p->led);
  if (p->spi) {
    spi_bus_remove_device(p->spi);
    p->spi = NULL;
  }
  if (p->bus_owned) {
    spi_bus_free(p->host);
    p->bus_owned = false;
  }
  if (p->dma) {
    heap_caps_free(p->dma);
    p->dma = NULL;
  }
  if (p->pixels) {
    heap_caps_free(p->pixels);
    p->pixels = NULL;
  }
  if (p->stopped) {
    vSemaphoreDelete(p->stopped);
    p->stopped = NULL;
  }
}
bool dbt_led_esp32_start(dbt_led_esp32_t *p, spi_host_device_t host, int gpio,
                         uint16_t w, uint16_t h) {
  if (!p || !w || !h || w > 32 || h > 32 || gpio < 0)
    return false;
  memset(p, 0, sizeof(*p));
  if (!recover())
    return false;
  atomic_init(&p->stopping, false);
  p->host = host;
  size_t n = (size_t)w * h * 3u;
  p->dma_bytes = n * 4u + 22u;
  p->pixels = heap_caps_malloc(n, MALLOC_CAP_8BIT);
  p->dma = heap_caps_malloc(p->dma_bytes, MALLOC_CAP_DMA);
  p->stopped = xSemaphoreCreateBinary();
  if (!p->pixels || !p->dma || !p->stopped) {
    release(p);
    return false;
  }
  spi_bus_config_t bus = {0};
  bus.mosi_io_num = gpio;
  bus.miso_io_num = -1;
  bus.sclk_io_num = -1;
  bus.quadwp_io_num = -1;
  bus.quadhd_io_num = -1;
  bus.max_transfer_sz = (int)p->dma_bytes;
  if (spi_bus_initialize(host, &bus, SPI_DMA_CH_AUTO) != ESP_OK) {
    release(p);
    return false;
  }
  p->bus_owned = true;
  spi_device_interface_config_t dev = {0};
  dev.clock_speed_hz = 3200000;
  dev.spics_io_num = -1;
  dev.flags = SPI_DEVICE_TXBIT_LSBFIRST;
  dev.queue_size = 1;
  if (spi_bus_add_device(host, &dev, &p->spi) != ESP_OK) {
    release(p);
    return false;
  }
  uint8_t saved[10];
  int present = load_settings(saved);
  if (present < 0) {
    release(p);
    return false;
  }
  dbt_led_io_t io = {commit, show, p};
  if (!dbt_led_init(&p->led, w, h, p->pixels, n, &io,
                    present == 1 ? saved : NULL)) {
    release(p);
    return false;
  }
  if(!dbt_led_set_allocator(&p->led,effect_allocate,effect_release,NULL)) {
    release(p);return false;
  }
  if (xTaskCreatePinnedToCore(worker, "dbt_led", 3072, p, 2, &p->task, DBT_ESP32_BACKGROUND_CORE) != pdPASS) {
    release(p);
    return false;
  }
  return true;
}
dbt_led_result_t dbt_led_esp32_submit(dbt_led_esp32_t *p,
                                      const dbt_led_request_t *r) {
  if (!p || !p->task ||
      atomic_load_explicit(&p->stopping, memory_order_acquire))
    return DBT_LED_INVALID;
  dbt_led_result_t e = dbt_led_submit(&p->led, r);
  if (e == DBT_LED_OK)
    xTaskNotifyGive(p->task);
  return e;
}
bool dbt_led_esp32_receipt(dbt_led_esp32_t *p, dbt_led_receipt_t *r) {
  return p && dbt_led_receipt(&p->led, r);
}
bool dbt_led_esp32_stop(dbt_led_esp32_t *p) {
  if (!p || !p->task)
    return false;
  if (atomic_load(&p->led.request_head) != atomic_load(&p->led.request_tail) ||
      atomic_load(&p->led.receipt_head) != atomic_load(&p->led.receipt_tail))
    return false;
  atomic_store_explicit(&p->stopping, true, memory_order_release);
  xTaskNotifyGive(p->task);
  xSemaphoreTake(p->stopped, portMAX_DELAY);
  vTaskDelete(p->task);
  p->task = NULL;
  release(p);
  return true;
}
