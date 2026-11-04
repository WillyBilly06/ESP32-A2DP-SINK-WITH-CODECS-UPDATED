#include "dbt_input_esp32.h"
#include "driver/gpio.h"
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
static uint8_t module, reg;
static uint32_t buttons = 0xffffffffu;
static int32_t positions[4];
static uint32_t ms;
static uint8_t commands[128][8], sizes[128];
static unsigned count, installs, deletes;
static unsigned sensor_reads;
static int fail_config,fail_install,probe_write_result,fail_delete,fail_identity_read;
static uint8_t fake_id = 0x55;
static uint32_t fake_product = 5752;

int i2c_param_config(i2c_port_t p, const i2c_config_t *c) {
  (void)p;
  (void)c;
  return fail_config?1:0;
}
int i2c_driver_install(i2c_port_t p, int m, int x, int y, int z) {
  (void)p;
  (void)m;
  (void)x;
  (void)y;
  (void)z;
  installs++;
  return fail_install?1:0;
}
int i2c_driver_delete(i2c_port_t p) {
  (void)p;
  deletes++;
  return fail_delete?1:0;
}
int gpio_config(const gpio_config_t *c) {
  (void)c;
  return 0;
}
int gpio_get_level(gpio_num_t n) {
  (void)n;
  return 1;
}
void esp_rom_delay_us(unsigned n) { (void)n; }
int64_t esp_timer_get_time(void) { return (int64_t)ms * 1000; }
int i2c_master_write_to_device(i2c_port_t p, uint8_t a, const uint8_t *b,
                               size_t n, unsigned t) {
  (void)p;
  (void)a;
  (void)t;
  if (n < 2)
    return 1;
  if(b[0]==0 && b[1]==1 && probe_write_result)return probe_write_result;
  module = b[0];
  reg = b[1];
  if (module == 0x0e && count < 128 && n <= 8) {
    sizes[count] = (uint8_t)n;
    for (size_t j = 0; j < n; j++)
      commands[count][j] = b[j];
    count++;
  }
  return 0;
}
int i2c_master_read_from_device(i2c_port_t p, uint8_t a, uint8_t *b, size_t n,
                                unsigned t) {
  sensor_reads++;
  (void)p;
  (void)a;
  (void)t;
  if (module == 0 && reg == 1 && n == 1) {
    if(fail_identity_read)return 1;
    b[0] = fake_id;
    return 0;
  }
  if (n != 4)
    return 1;
  uint32_t v;
  if (module == 0 && reg == 2)
    v = fake_product << 16;
  else if (module == 1 && reg == 4)
    v = buttons;
  else if (module == 0x11 && reg >= 0x30 && reg < 0x34)
    v = (uint32_t)positions[reg - 0x30];
  else
    return 1;
  for (unsigned i = 0; i < 4; i++)
    b[i] = (uint8_t)(v >> (24 - 8 * i));
  return 0;
}
#define CHECK(x)                                                               \
  do {                                                                         \
    if (!(x))                                                                  \
      return __LINE__;                                                         \
  } while (0)
static bool found(unsigned off, uint8_t g, uint8_t r, uint8_t b) {
  for (unsigned i = 0; i < count; i++)
    if (sizes[i] == 7 && commands[i][0] == 14 && commands[i][1] == 4 &&
        commands[i][2] == 0 && commands[i][3] == off && commands[i][4] == g &&
        commands[i][5] == r && commands[i][6] == b)
      return true;
  return false;
}
EX int test_input_identity(void) {
  dbt_input_esp32_t p;
  probe_write_result=ESP_FAIL;
  CHECK(dbt_input_esp32_start_checked(&p,1,23,22,0x36,18,21,19)==DBT_INPUT_START_NO_ACK);
  CHECK(!p.installed && installs==1 && deletes==1);
  fail_delete=1;
  CHECK(dbt_input_esp32_start_checked(&p,1,23,22,0x36,18,21,19)==DBT_INPUT_START_CLEANUP_FAILED);
  CHECK(p.installed && installs==2 && deletes==2);
  dbt_input_esp32_t other;
  CHECK(dbt_input_esp32_start_checked(&other,1,23,22,7,18,21,19)==DBT_INPUT_START_FAILED);
  CHECK(installs==2 && deletes==2 && p.installed);
  CHECK(!dbt_input_esp32_stop_checked(&p) && p.installed);
  fail_delete=0;CHECK(dbt_input_esp32_stop_checked(&p) && !p.installed);
  probe_write_result=1;
  CHECK(dbt_input_esp32_start_checked(&p,1,23,22,7,18,21,19)==DBT_INPUT_START_FAILED && !p.installed);
  probe_write_result=0;fail_config=1;
  CHECK(dbt_input_esp32_start_checked(&p,1,23,22,7,18,21,19)==DBT_INPUT_START_FAILED);
  fail_config=0;fail_install=1;
  CHECK(dbt_input_esp32_start_checked(&p,1,23,22,7,18,21,19)==DBT_INPUT_START_FAILED);
  fail_install=0;fail_identity_read=1;
  CHECK(dbt_input_esp32_start_checked(&p,1,23,22,7,18,21,19)==DBT_INPUT_START_FAILED && !p.installed);
  fail_identity_read=0;installs=deletes=0;
  fake_id = 0;
  CHECK(!dbt_input_esp32_start(&p, 1, 23, 22, 0x36, 18, 21, 19));
  CHECK(!p.installed && installs == 1 && deletes == 1 && count == 0);
  fake_id = 0x55;
  fake_product = 1234;
  CHECK(!dbt_input_esp32_start(&p, 1, 23, 22, 0x07, 18, 21, 19));
  CHECK(!p.installed && installs == 2 && deletes == 2 && count == 0);
  fake_product = 5752;
  CHECK(dbt_input_esp32_start(&p, 1, 23, 22, 0x36, 18, 21, 19));
  CHECK(p.installed && p.address == 0x36);
  dbt_input_esp32_stop(&p);
  CHECK(deletes == 3);
  count = 0;
  installs = 0;
  deletes = 0;
  return 0;
}
EX int test_input_platform(void) {
  dbt_input_esp32_t p;
  CHECK(dbt_input_esp32_start(&p, 1, 23, 22, 7, 18, 21, 19));
  CHECK(count == 8);
  CHECK(found(0, 100, 0, 0) && found(3, 0, 100, 0) && found(6, 0, 0, 100) &&
        found(9, 100, 100, 0));
  CHECK(dbt_input_esp32_poll(&p));
  buttons &= ~(1u << 14);
  for (int i = 0; i < 3; i++) {
    ms += 25;
    CHECK(dbt_input_esp32_poll(&p));
  }
  CHECK(found(3, 100, 100, 100));
  buttons = 0xffffffffu;
  ms += 25;
  CHECK(dbt_input_esp32_poll(&p));
  buttons &= ~(1u << 17);
  for (int i = 0; i < 3; i++) {
    ms += 25;
    CHECK(dbt_input_esp32_poll(&p));
  }
  CHECK(found(6, 100, 0, 100));
  dbt_input_action_t a;
  CHECK(dbt_input_next(&p.input, &a) && a.kind == DBT_INPUT_PAIR);
  buttons = 0xffffffffu;
  ms += 25;
  CHECK(dbt_input_esp32_poll(&p));
  buttons &= ~(1u << 9);
  for (int i = 0; i < 3; i++) {
    ms += 25;
    CHECK(dbt_input_esp32_poll(&p));
  }
  buttons = 0xffffffffu;
  ms += 301;
  CHECK(dbt_input_esp32_poll(&p));
  CHECK(found(9, 0, 100, 100));
  /* A large but plausible movement drains across bounded samples without
   * treating one captured button state as repeated debounce samples. */
  p.input.state.brightness_mode = 0;
  positions[1] = 80;
  ms += 25;
  CHECK(!dbt_input_esp32_poll(&p) && p.pending);
  unsigned reads_at_pause=sensor_reads;
  unsigned steps = 0;
  while (dbt_input_next(&p.input, &a)) {
    CHECK(a.kind == DBT_INPUT_EQ_BASS);
    steps++;
  }
  for (unsigned k = 0; k < 8 && p.pending; k++) {
    ms += 25;
    dbt_input_drain_result_t drained=dbt_input_esp32_drain_pending(&p);
    CHECK(drained==DBT_INPUT_DRAIN_BUSY || drained==DBT_INPUT_DRAIN_DONE);
    CHECK(sensor_reads==reads_at_pause);
    while (dbt_input_next(&p.input, &a)) {
      CHECK(a.kind == DBT_INPUT_EQ_BASS && a.delta == 1);
      steps++;
    }
  }
  CHECK(!p.pending && steps == 20);
  CHECK(dbt_input_esp32_drain_pending(&p)==DBT_INPUT_DRAIN_DONE && sensor_reads==reads_at_pause);
  /* An implausible counter jump is diagnosed/rebased without blocking other
   * encoder channels or wedging subsequent samples. */
  positions[1] = 10000;
  positions[2] = 4;
  ms += 25;
  CHECK(dbt_input_esp32_poll(&p));
  CHECK(atomic_load(&p.invalid_channels) == 1);
  CHECK(dbt_input_next(&p.input, &a) && a.kind == DBT_INPUT_EQ_MID &&
        a.delta == 1);
  positions[1] += 4;
  ms += 25;
  CHECK(dbt_input_esp32_poll(&p));
  CHECK(dbt_input_next(&p.input, &a) && a.kind == DBT_INPUT_EQ_BASS &&
        a.delta == 1);
  dbt_input_esp32_stop(&p);
  CHECK(!p.installed);
  return 0;
}
