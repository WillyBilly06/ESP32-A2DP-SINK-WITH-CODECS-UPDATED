#include "dbt_input_esp32.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include <string.h>
/* Reserve before touching the driver. A failed delete deliberately retains
 * this reservation, even if the driver state is ambiguous. */
static atomic_uint owned_ports;
static atomic_uintptr_t port_owner[I2C_NUM_MAX];
static bool readreg(dbt_input_esp32_t *p, uint8_t module, uint8_t reg,
                    uint8_t *b, size_t n, unsigned delay) {
  uint8_t prefix[2] = {module, reg};
  if (i2c_master_write_to_device(p->port, p->address, prefix, 2,
                                 pdMS_TO_TICKS(25)) != ESP_OK)
    return false;
  esp_rom_delay_us(delay);
  return i2c_master_read_from_device(p->port, p->address, b, n,
                                     pdMS_TO_TICKS(25)) == ESP_OK;
}
static bool gpio_mode(dbt_input_esp32_t *p, uint8_t reg, uint32_t mask) {
  uint8_t b[6] = {1,
                  reg,
                  (uint8_t)(mask >> 24),
                  (uint8_t)(mask >> 16),
                  (uint8_t)(mask >> 8),
                  (uint8_t)mask};
  return i2c_master_write_to_device(p->port, p->address, b, sizeof(b),
                                    pdMS_TO_TICKS(25)) == ESP_OK;
}
static bool pixel_write(dbt_input_esp32_t *p, uint8_t reg, const uint8_t *data,
                        size_t len) {
  if (len > 5)
    return false;
  uint8_t cmd[7] = {0x0e, reg};
  if (len)
    memcpy(cmd + 2, data, len);
  return i2c_master_write_to_device(p->port, p->address, cmd, len + 2,
                                    pdMS_TO_TICKS(25)) == ESP_OK;
}
static bool pixel(dbt_input_esp32_t *p, uint8_t index, uint8_t r, uint8_t g,
                  uint8_t b) {
  uint8_t d[5] = {0, (uint8_t)(index * 3u), g, r, b};
  return pixel_write(p, 4, d, 5);
}
static bool pixels_init(dbt_input_esp32_t *p) {
  const uint8_t pin = 18, speed = 1, len[2] = {0, 12};
  return pixel_write(p, 1, &pin, 1) && pixel_write(p, 2, &speed, 1) &&
         pixel_write(p, 3, len, 2) && pixel(p, 0, 0, 100, 0) &&
         pixel(p, 1, 100, 0, 0) && pixel(p, 2, 0, 0, 100) &&
         pixel(p, 3, 100, 100, 0) && pixel_write(p, 5, NULL, 0);
}
static void pixels_update(dbt_input_esp32_t *p) {
  uint8_t flags = (p->input.state.brightness_mode ? 1u : 0u) |
                  (p->input.state.effect_mode ? 2u : 0u);
  if ((p->input.state.enc_pressed & 4u) && !(p->previous_buttons & 4u))
    p->feedback |= 4u;
  p->previous_buttons = p->input.state.enc_pressed;
  flags |= p->feedback & 4u;
  p->feedback = flags;
  if (flags == p->feedback_applied)
    return;
  if (pixel(p, 1, 100, flags & 1u ? 100 : 0, flags & 1u ? 100 : 0) &&
      pixel(p, 2, 0, flags & 4u ? 100 : 0, 100) &&
      pixel(p, 3, 100, flags & 2u ? 0 : 100, flags & 2u ? 100 : 0) &&
      pixel_write(p, 5, NULL, 0))
    p->feedback_applied = flags;
  else
    p->feedback_errors++;
}
static uint32_t be32(const uint8_t b[4]) {
  return ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) |
         ((uint32_t)b[2] << 8) | b[3];
}
dbt_input_start_result_t dbt_input_esp32_start_checked(dbt_input_esp32_t *p, i2c_port_t port, int sda,
                           int scl, uint8_t address, int b1, int b2, int effect) {
  if (!p || address > 0x7f || sda < 0 || scl < 0 || b1 < 0 || b2 < 0 ||
      effect < 0 || b1 >= 64 || b2 >= 64 || effect >= 64 || b1 == b2 ||
      b1 == effect || b2 == effect || sda == scl || b1 == sda || b1 == scl ||
      b2 == sda || b2 == scl || effect == sda || effect == scl || port < 0 ||
      port >= I2C_NUM_MAX || (unsigned)port>=sizeof(unsigned)*8u)
    return DBT_INPUT_START_FAILED;
  unsigned mask_port=1u<<(unsigned)port;
  if(atomic_fetch_or_explicit(&owned_ports,mask_port,memory_order_acq_rel)&mask_port)
    return DBT_INPUT_START_FAILED;
  memset(p, 0, sizeof(*p));
  atomic_init(&p->read_errors, 0);
  atomic_init(&p->deferred_samples, 0);
  atomic_init(&p->feedback_errors, 0);
  atomic_init(&p->invalid_channels, 0);
  dbt_input_init(&p->input);
  p->port = port;
  p->address = address;
  p->gpio[0] = b1;
  p->gpio[1] = b2;
  p->gpio[2] = effect;
  i2c_config_t conf = {0};
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = sda;
  conf.scl_io_num = scl;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = 400000;
  if (i2c_param_config(port, &conf) != ESP_OK ||
      i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0) != ESP_OK) {
    atomic_fetch_and_explicit(&owned_ports,~mask_port,memory_order_release);
    return DBT_INPUT_START_FAILED;
  }
  atomic_store_explicit(&port_owner[port],(uintptr_t)p,memory_order_release);
  p->installed = true;
  uint8_t hardware_id, version[4];
  uint8_t first_read[2]={0,1};
  int first=i2c_master_write_to_device(p->port,p->address,first_read,2,pdMS_TO_TICKS(25));
  if(first!=ESP_OK) {
    if(!dbt_input_esp32_stop_checked(p))return DBT_INPUT_START_CLEANUP_FAILED;
    return first==ESP_FAIL?DBT_INPUT_START_NO_ACK:DBT_INPUT_START_FAILED;
  }
  esp_rom_delay_us(250);
  if (i2c_master_read_from_device(p->port,p->address,&hardware_id,1,pdMS_TO_TICKS(25))!=ESP_OK ||
      (hardware_id != 0x55 && (hardware_id < 0x84 || hardware_id > 0x89)) ||
      !readreg(p, 0, 2, version, 4, 250) || ((be32(version) >> 16) != 5752u)) {
    return dbt_input_esp32_stop_checked(p)?DBT_INPUT_START_FAILED:DBT_INPUT_START_CLEANUP_FAILED;
  }
  uint32_t mask = (1u << 12) | (1u << 14) | (1u << 17) | (1u << 9);
  if (!gpio_mode(p, 3, mask) || !gpio_mode(p, 11, mask) ||
      !gpio_mode(p, 5, mask)) {
    return dbt_input_esp32_stop_checked(p)?DBT_INPUT_START_FAILED:DBT_INPUT_START_CLEANUP_FAILED;
  }
  gpio_config_t io = {0};
  io.mode = GPIO_MODE_INPUT;
  io.pull_up_en = GPIO_PULLUP_ENABLE;
  io.pin_bit_mask =
      (UINT64_C(1) << b1) | (UINT64_C(1) << b2) | (UINT64_C(1) << effect);
  if (!pixels_init(p)) {
    return dbt_input_esp32_stop_checked(p)?DBT_INPUT_START_FAILED:DBT_INPUT_START_CLEANUP_FAILED;
  }
  if (gpio_config(&io) != ESP_OK) {
    return dbt_input_esp32_stop_checked(p)?DBT_INPUT_START_FAILED:DBT_INPUT_START_CLEANUP_FAILED;
  }
  return DBT_INPUT_START_READY;
}
bool dbt_input_esp32_start(dbt_input_esp32_t *p,i2c_port_t port,int sda,int scl,
                           uint8_t address,int b1,int b2,int effect) {
  return dbt_input_esp32_start_checked(p,port,sda,scl,address,b1,b2,effect)==DBT_INPUT_START_READY;
}
static bool feed_sample(dbt_input_esp32_t *p, const dbt_input_sample_t *s,
                        bool fresh) {
  dbt_input_feed_result_t r = dbt_input_feed_ex(&p->input, s, fresh);
  if (r == DBT_INPUT_FEED_INVALID) {
    unsigned lost = dbt_input_rebase_invalid(&p->input, s);
    if (!lost) {
      p->pending = false;
      return false;
    }
    atomic_fetch_add_explicit(&p->invalid_channels, lost, memory_order_relaxed);
    r = dbt_input_feed_ex(&p->input, s, fresh);
  }
  if (r == DBT_INPUT_FEED_BUSY || r == DBT_INPUT_FEED_PARTIAL) {
    p->retry = *s;
    p->pending = true;
    p->retry_fresh = (r == DBT_INPUT_FEED_BUSY) ? fresh : false;
    atomic_fetch_add_explicit(&p->deferred_samples, 1, memory_order_relaxed);
    if (r == DBT_INPUT_FEED_PARTIAL)
      pixels_update(p);
    return false;
  }
  if (r != DBT_INPUT_FEED_ACCEPTED) {
    p->pending = false;
    return false;
  }
  p->pending = false;
  pixels_update(p);
  return true;
}
dbt_input_drain_result_t dbt_input_esp32_drain_pending(dbt_input_esp32_t *p) {
  if (!p || !p->installed) return DBT_INPUT_DRAIN_FAULT;
  if (!p->pending) return DBT_INPUT_DRAIN_DONE;
  if (feed_sample(p,&p->retry,p->retry_fresh)) return DBT_INPUT_DRAIN_DONE;
  return p->pending ? DBT_INPUT_DRAIN_BUSY : DBT_INPUT_DRAIN_FAULT;
}
bool dbt_input_esp32_poll(dbt_input_esp32_t *p) {
  if (!p || !p->installed)
    return false;
  if (p->pending)
    return feed_sample(p, &p->retry, p->retry_fresh);
  dbt_input_sample_t s = {0};
  uint8_t b[4];
  for (unsigned j = 0; j < 4; j++) {
    if (!readreg(p, 0x11, (uint8_t)(0x30 + j), b, 4, 500)) {
      p->read_errors++;
      return false;
    }
    uint32_t x = be32(b);
    memcpy(&s.position[j], &x, 4);
  }
  if (!readreg(p, 1, 4, b, 4, 250)) {
    p->read_errors++;
    return false;
  }
  uint32_t buttons = be32(b);
  static const unsigned pins[4] = {12, 14, 17, 9};
  for (unsigned j = 0; j < 4; j++)
    if (!(buttons & (1u << pins[j])))
      s.enc_buttons |= (uint8_t)(1u << j);
  for (unsigned j = 0; j < 3; j++)
    if (!gpio_get_level((gpio_num_t)p->gpio[j]))
      s.gpio_buttons |= (uint8_t)(1u << j);
  s.ms = (uint32_t)(esp_timer_get_time() / 1000u);
  return feed_sample(p, &s, true);
}
void dbt_input_esp32_stop(dbt_input_esp32_t *p) {
  (void)dbt_input_esp32_stop_checked(p);
}
bool dbt_input_esp32_stop_checked(dbt_input_esp32_t *p) {
  if(!p)return false;
  if(!p->installed)return true;
  if(p->port<0 || p->port>=I2C_NUM_MAX ||
     atomic_load_explicit(&port_owner[p->port],memory_order_acquire)!=(uintptr_t)p)return false;
  if(i2c_driver_delete(p->port)!=ESP_OK)return false;
  atomic_store_explicit(&port_owner[p->port],(uintptr_t)NULL,memory_order_release);
  atomic_fetch_and_explicit(&owned_ports,~(1u<<(unsigned)p->port),memory_order_release);
  p->installed=false;return true;
}
