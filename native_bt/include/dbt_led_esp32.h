#ifndef DBT_LED_ESP32_H
#define DBT_LED_ESP32_H
#include "dbt_led.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

typedef struct {
  dbt_led_t led;
  spi_device_handle_t spi;
  spi_host_device_t host;
  uint8_t *pixels, *dma;
  size_t dma_bytes;
  TaskHandle_t task;
  SemaphoreHandle_t stopped;
  atomic_bool stopping;
  bool bus_owned, storage_fault;
} dbt_led_esp32_t;
/* Call start/stop on a boot/shutdown or control worker, never the host loop.
 * Start initializes GPIO/SPI and reads NVS. No namespace is erased.
 * Defaults: SPI2_HOST, GPIO4, 16x16, 30 frames/sec. */
bool dbt_led_esp32_start(dbt_led_esp32_t *p, spi_host_device_t host, int gpio,
                         uint16_t width, uint16_t height);
/* Stop only after all submitted jobs have produced and had receipts drained. */
bool dbt_led_esp32_stop(dbt_led_esp32_t *p);
/* Submit and receipt are single-owner nonblocking; token matched receipt is
 * the only authorization to ACK success. Meter publication is atomic. */
dbt_led_result_t dbt_led_esp32_submit(dbt_led_esp32_t *p,
                                      const dbt_led_request_t *r);
bool dbt_led_esp32_receipt(dbt_led_esp32_t *p, dbt_led_receipt_t *r);
#endif
