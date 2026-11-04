#include "dbt_input_esp32.h"
#include "dbt_led_esp32.h"
#include "dbt_sound_esp32.h"
#include "probe_product_io.h"
#include "probe_sound.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#ifdef _WIN32
#define EX __declspec(dllexport)
void *memcpy(void *d, const void *s, size_t n) {
  uint8_t *a = d;
  const uint8_t *b = s;
  for (size_t i = 0; i < n; i++)
    a[i] = b[i];
  return d;
}
void *memset(void *d, int v, size_t n) {
  uint8_t *a = d;
  for (size_t i = 0; i < n; i++)
    a[i] = (uint8_t)v;
  return d;
}
int memcmp(const void *a, const void *b, size_t n) {
  const uint8_t *x = a, *y = b;
  for (size_t i = 0; i < n; i++)
    if (x[i] != y[i])
      return x[i] < y[i] ? -1 : 1;
  return 0;
}
#else
#define EX
#endif
static uint8_t pixels[768];
static uint32_t ms;
static bool mounted = true, peer_ok = true, commit_ok = true, show_ok = true,
            delay_led = false, retire_play = false, disconnect_on_wait = false,
            fail_play = false, disconnect_in_commit = false,
            stale_on_wait = false;
static bool fail_input_start, allow_address36, fail_task_create,
    fail_sound_bind;
static dbt_input_start_result_t forced_input_result;
static dbt_input_start_result_t forced_alt_result;
static bool fail_input_stop;
static unsigned led_stops, input_stops, task_deletes, input_tries;
static unsigned input_reads;static dbt_input_drain_result_t input_drain_result=DBT_INPUT_DRAIN_DONE;
static uint8_t tried[4];
static void fake_task_advance(void);
static unsigned wake_owner_count, wake_worker_count, disconnects, commits,
    shows;
static dbt_led_esp32_t *provider;
static bool commit_fake(void *p, const uint8_t *b) {
  (void)p;
  (void)b;
  commits++;
  if (disconnect_in_commit) {
    dbt_ble_control_peer_t peer = {1, 2};
    peer_ok = false;
    probe_product_io_peer_lost(NULL, &peer);
    disconnect_in_commit = false;
  }
  return commit_ok;
}
static bool show_fake(void *p, const uint8_t *b, size_t n) {
  (void)p;
  (void)b;
  (void)n;
  shows++;
  return show_ok;
}
bool esp_spiffs_mounted(const char *label) {
  (void)label;
  return mounted;
}
int64_t esp_timer_get_time(void) { return (int64_t)ms * 1000; }
void probe_gatt_control_wake(void) { wake_worker_count++; }
static void wake_owner(void *p) {
  (void)p;
  wake_owner_count++;
}
static bool valid(void *p, const dbt_ble_control_peer_t *peer) {
  (void)p;
  return peer_ok && peer->generation == 2;
}
void vTaskDelay(unsigned t) {
  ms += t;
  fake_task_advance();
  if (disconnect_on_wait) {
    dbt_ble_control_peer_t peer = {1, 2};
    peer_ok = false;
    probe_product_io_peer_lost(NULL, &peer);
    delay_led = false;
    disconnect_on_wait = false;
  }
  if (stale_on_wait) {
    dbt_ble_control_peer_t peer = {1, 1};
    probe_product_io_peer_lost(NULL, &peer);
    delay_led = false;
    stale_on_wait = false;
  }
  if (provider && !delay_led)
    (void)dbt_led_work(&provider->led);
}
void vTaskDelete(TaskHandle_t t) {
  (void)t;
  task_deletes++;
}
void vTaskSuspend(TaskHandle_t t) { (void)t; }
int xTaskCreate(void (*f)(void *), const char *n, unsigned z, void *p,
                unsigned pr, TaskHandle_t *h) {
  (void)f;
  (void)n;
  (void)z;
  (void)p;
  (void)pr;
  if (fail_task_create)
    return 0;
  *h = (void *)1;
  return pdPASS;
}
bool dbt_led_esp32_start(dbt_led_esp32_t *p, spi_host_device_t host, int pin,
                         uint16_t w, uint16_t h) {
  (void)host;
  (void)pin;
  dbt_led_io_t hooks = {commit_fake, show_fake, 0};
  provider = p;
  p->task = (void *)1;
  return dbt_led_init(&p->led, w, h, pixels, sizeof pixels, &hooks, 0);
}
bool dbt_led_esp32_stop(dbt_led_esp32_t *p) {
  led_stops++;
  p->task = 0;
  return true;
}
dbt_led_result_t dbt_led_esp32_submit(dbt_led_esp32_t *p,
                                      const dbt_led_request_t *r) {
  dbt_led_result_t x = dbt_led_submit(&p->led, r);
  if (x == DBT_LED_OK && !delay_led)
    (void)dbt_led_work(&p->led);
  return x;
}
bool dbt_led_esp32_receipt(dbt_led_esp32_t *p, dbt_led_receipt_t *r) {
  return dbt_led_receipt(&p->led, r);
}
dbt_input_start_result_t dbt_input_esp32_start_checked(dbt_input_esp32_t *p, i2c_port_t port, int a, int b,
                           uint8_t addr, int c, int d, int e) {
  (void)port;
  (void)a;
  (void)b;
  if (input_tries < 4)
    tried[input_tries] = addr;
  input_tries++;
  (void)c;
  (void)d;
  (void)e;
  if(forced_input_result!=DBT_INPUT_START_READY){
    if(forced_input_result==DBT_INPUT_START_CLEANUP_FAILED)p->installed=true;
    return forced_input_result;
  }
  if(addr==0x07 && forced_alt_result!=DBT_INPUT_START_READY) {
    if(forced_alt_result==DBT_INPUT_START_CLEANUP_FAILED)p->installed=true;
    return forced_alt_result;
  }
  if (fail_input_start || (addr == 0x36 && !allow_address36))
    return DBT_INPUT_START_NO_ACK;
  dbt_input_init(&p->input);
  atomic_init(&p->read_errors, 0);
  atomic_init(&p->deferred_samples, 0);
  atomic_init(&p->feedback_errors, 0);
  atomic_init(&p->invalid_channels, 0);
  p->installed = true;
  return DBT_INPUT_START_READY;
}
bool dbt_input_esp32_poll(dbt_input_esp32_t *p) {
  (void)p;
  input_reads++;
  return true;
}
dbt_input_drain_result_t dbt_input_esp32_drain_pending(dbt_input_esp32_t *p){
  if(input_drain_result==DBT_INPUT_DRAIN_DONE)p->pending=false;
  return input_drain_result;
}
void dbt_input_esp32_stop(dbt_input_esp32_t *p) {
  (void)dbt_input_esp32_stop_checked(p);
}
bool dbt_input_esp32_stop_checked(dbt_input_esp32_t *p) {
  input_stops++;
  if(fail_input_stop)return false;
  p->installed = false;return true;
}
static uint8_t temp[128], file_data[4][128];
static uint32_t file_size[4] = {46, 46, 0, 0}, temp_size;
static uint8_t upload_type;
static bool begin_file(void *c, uint8_t type, uint32_t n) {
  (void)c;
  if (n > sizeof temp)
    return false;
  temp_size = n;
  upload_type = type;
  return true;
}
static bool write_file(void *c, uint32_t at, const uint8_t *d, uint16_t n) {
  (void)c;
  if (at + n > temp_size)
    return false;
  memcpy(temp + at, d, n);
  return true;
}
static bool seal_file(void *c) {
  (void)c;
  return true;
}
static bool read_temp(void *c, uint32_t at, uint8_t *d, uint16_t n) {
  (void)c;
  if (at + n > temp_size)
    return false;
  memcpy(d, temp + at, n);
  return true;
}
static bool commit_file(void *c, uint8_t type) {
  (void)c;
  if (type != upload_type)
    return false;
  memcpy(file_data[type], temp, temp_size);
  file_size[type] = temp_size;
  return true;
}
static void abort_file(void *c) {
  (void)c;
  temp_size = 0;
}
static dbt_sound_recovery_state_t recovery_state[4];
static dbt_result_t size_file(void *c, uint8_t t, uint32_t *n) {
  (void)c;
  if (recovery_state[t] != DBT_SOUND_RECOVERY_CLEAN)
    return DBT_INVALID;
  if (!file_size[t])
    return DBT_EMPTY;
  *n = file_size[t];
  return DBT_OK;
}
static bool read_file(void *c, uint8_t t, uint32_t at, uint8_t *d, uint16_t n) {
  (void)c;
  if (at + n > file_size[t])
    return false;
  memcpy(d, file_data[t] + at, n);
  return true;
}
static bool remove_file(void *c, uint8_t t) {
  (void)c;
  file_size[t] = 0;
  return true;
}
static bool set_mute(void *c, bool m) {
  (void)c;
  (void)m;
  return true;
}
static uint32_t store_now(void *c) {
  (void)c;
  return ms;
}
static bool store_valid(void *c, dbt_sound_peer_t p) {
  (void)c;
  return peer_ok && p.generation == 2;
}
static int inspect_error = -1, reconcile_error = -1;
static unsigned inspected, reconciled;
dbt_result_t dbt_sound_esp32_inspect(dbt_sound_esp32_t *p, uint8_t t,
                                     dbt_sound_recovery_t *r) {
  (void)p;
  inspected++;
  if (t == (uint8_t)inspect_error)
    return DBT_INVALID;
  memset(r, 0, sizeof(*r));
  r->state = recovery_state[t];
  return DBT_OK;
}
dbt_result_t dbt_sound_esp32_reconcile(dbt_sound_esp32_t *p, uint8_t t,
                                       dbt_sound_recovery_t *r) {
  (void)p;
  reconciled++;
  if (t == (uint8_t)reconcile_error)
    return DBT_INVALID;
  recovery_state[t] = DBT_SOUND_RECOVERY_CLEAN;
  memset(r, 0, sizeof(*r));
  r->state = DBT_SOUND_RECOVERY_CLEAN;
  return DBT_OK;
}
dbt_result_t dbt_sound_esp32_init(dbt_sound_esp32_t *p, void *c,
                                  bool (*v)(void *, dbt_sound_peer_t),
                                  dbt_sound_store_t *store, bool *muted) {
  (void)p;
  (void)c;
  (void)v;
  *store = (dbt_sound_store_t){0,          begin_file,  write_file, seal_file,
                               read_temp,  commit_file, abort_file, size_file,
                               read_file,  remove_file, set_mute,   store_now,
                               store_valid};
  *muted = false;
  return DBT_OK;
}
dbt_result_t probe_sound_init_with_wake(probe_sound_t *p, dbt_sound_t *s,
                                        void (*w)(void *), void *c) {
  if (fail_sound_bind)
    return DBT_INVALID;
  p->provider = s;
  p->wake_external = w;
  p->wake_context = c;
  return DBT_OK;
}
dbt_result_t probe_sound_play(probe_sound_t *p, uint8_t t, uint32_t rate,
                              bool x) {
  (void)t;
  (void)rate;
  (void)x;
  if (fail_play)
    return DBT_INVALID;
  p->play = (dbt_sound_play_t *)1;
  p->token = 1;
  return DBT_OK;
}
dbt_result_t probe_sound_poll(probe_sound_t *p, uint32_t now,
                              dbt_sound_peer_t *expired) {
  bool timed_out = dbt_sound_poll(p->provider, now, expired);
  if (retire_play) {
    p->play = 0;
    p->token = 0;
  }
  return timed_out ? DBT_EMPTY : DBT_OK;
}
void probe_sound_stop(probe_sound_t *p) {
  (void)p;
  retire_play = true;
}
dbt_result_t probe_sound_disconnect(probe_sound_t *p, dbt_sound_peer_t peer) {
  disconnects++;
  return dbt_sound_disconnect(p->provider, peer);
}
#include "../../esp32_probe/main/probe_product_io.c"
static void fake_task_advance(void) {
  if (atomic_load(&io.retiring) && io.input_task)
    atomic_store(&io.input_done, true);
}
#define CHECK(x)                                                               \
  do {                                                                         \
    if (!(x))                                                                  \
      return __LINE__;                                                         \
  } while (0)
EX int test_startup_snapshot(void) {
  dbt_ble_control_state_t initial = {0};
  for (unsigned j = 0; j < 10; j++)
    initial.led[j] = (uint8_t)(40 + j);
  initial.sound = 0x91;
  probe_product_io_hooks_t hooks = {0, valid, wake_owner, true, true, false, false};
  fail_input_start = true;
  input_tries = led_stops = 0;
  CHECK(!probe_product_io_startup_worker(&initial, &hooks));
  CHECK(input_tries == 2 && tried[0] == 0x36 && tried[1] == 0x07 &&
        led_stops == 1);
  CHECK(initial.sound == 0x91);
  probe_product_io_capabilities_t caps;
  CHECK(probe_product_io_capabilities(&caps) && !caps.ready && caps.input==PROBE_PRODUCT_INPUT_NO_ACK);
  for (unsigned j = 0; j < 10; j++)
    CHECK(initial.led[j] == 40 + j);
  /* Test-only reset: no fake task was spawned; production never reinitializes
   * a live singleton this way. */
  memset(&io, 0, sizeof io);
  atomic_init(&io.ready, false);
  atomic_init(&io.retiring, false);
  /* Optional policy continues only after two clean no-ACK results. */
  hooks.input_optional=true;hooks.sound=true;input_tries=led_stops=0;
  CHECK(probe_product_io_startup_worker(&initial,&hooks));
  CHECK(input_tries==2 && !led_stops && probe_product_io_capabilities(&caps) &&
        caps.ready && caps.led && caps.sound && caps.input==PROBE_PRODUCT_INPUT_NO_ACK);
  dbt_input_action_t action;probe_product_io_input_stats_t stats;
  CHECK(!probe_product_io_take_input(&action) && !probe_product_io_input_stats(&stats));
  CHECK(initial.sound==3 && initial.led[0]==0);
  dbt_ble_control_peer_t peer={1,2};uint8_t data=45,response[8],size=sizeof(response);
  CHECK(probe_product_io_command(NULL,&peer,7,&data,1,response,&size) && size==2 && response[0]==0x10);
  size=sizeof(response);
  CHECK(probe_product_io_command(NULL,&peer,0x14,NULL,0,response,&size) && size==2 && response[0]==0x33);
  atomic_store(&io.ready,false);close_startup();
  memset(&io,0,sizeof io);atomic_init(&io.ready,false);atomic_init(&io.retiring,false);
  /* A bus/configuration error or a failed cleanup is never optional, and no
   * alternate address is attempted after an ambiguous retained driver. */
  forced_input_result=DBT_INPUT_START_FAILED;input_tries=0;
  CHECK(!probe_product_io_startup_worker(&initial,&hooks) && input_tries==1);
  CHECK(probe_product_io_capabilities(&caps) && !caps.ready && caps.input==PROBE_PRODUCT_INPUT_FAILED);
  memset(&io,0,sizeof io);atomic_init(&io.ready,false);atomic_init(&io.retiring,false);
  forced_input_result=DBT_INPUT_START_READY;forced_alt_result=DBT_INPUT_START_CLEANUP_FAILED;input_tries=0;
  CHECK(!probe_product_io_startup_worker(&initial,&hooks) && input_tries==2 && io.input.installed);
  CHECK(probe_product_io_capabilities(&caps) && !caps.ready && caps.input==PROBE_PRODUCT_INPUT_CLEANUP_FAILED);
  memset(&io,0,sizeof io);atomic_init(&io.ready,false);atomic_init(&io.retiring,false);
  forced_alt_result=DBT_INPUT_START_READY;
  forced_input_result=DBT_INPUT_START_CLEANUP_FAILED;input_tries=0;
  CHECK(!probe_product_io_startup_worker(&initial,&hooks) && input_tries==1 && io.input.installed);
  CHECK(probe_product_io_capabilities(&caps) && !caps.ready && caps.input==PROBE_PRODUCT_INPUT_CLEANUP_FAILED);
  memset(&io,0,sizeof io);atomic_init(&io.ready,false);atomic_init(&io.retiring,false);
  forced_input_result=DBT_INPUT_START_READY;
  provider = 0;
  fail_input_start = false;
  return 0;
}
EX int test_startup_cleanup(void) {
  dbt_ble_control_state_t initial = {0};
  initial.led[0] = 55;
  initial.sound = 77;
  probe_product_io_hooks_t hooks = {0, valid, wake_owner, true, true, true, false};
  allow_address36 = true;
  fail_task_create = true;
  led_stops = input_stops = task_deletes = input_tries = 0;
  CHECK(!probe_product_io_startup_worker(&initial, &hooks));
  CHECK(led_stops == 1 && input_stops == 1 && task_deletes == 0 &&
        input_tries == 1 && tried[0] == 0x36);
  CHECK(initial.led[0] == 55 && initial.sound == 77);
  probe_product_io_capabilities_t caps;
  CHECK(probe_product_io_capabilities(&caps) && !caps.ready && caps.input==PROBE_PRODUCT_INPUT_STOPPED);
  memset(&io, 0, sizeof io);
  atomic_init(&io.ready, false);
  atomic_init(&io.retiring, false);
  fail_input_stop=true;led_stops=input_stops=input_tries=0;
  CHECK(!probe_product_io_startup_worker(&initial,&hooks));
  CHECK(input_tries==1 && input_stops==1 && io.input.installed &&
        probe_product_io_capabilities(&caps) && !caps.ready && caps.input==PROBE_PRODUCT_INPUT_CLEANUP_FAILED);
  fail_input_stop=false;CHECK(dbt_input_esp32_stop_checked(&io.input));
  memset(&io, 0, sizeof io);
  atomic_init(&io.ready, false);
  atomic_init(&io.retiring, false);
  fail_task_create = false;
  fail_sound_bind = true;
  led_stops = input_stops = task_deletes = input_tries = 0;
  CHECK(!probe_product_io_startup_worker(&initial, &hooks));
  CHECK(led_stops == 1 && input_stops == 1 && task_deletes == 1 &&
        input_tries == 1);
  CHECK(initial.led[0] == 55 && initial.sound == 77);
  memset(&io, 0, sizeof io);
  atomic_init(&io.ready, false);
  atomic_init(&io.retiring, false);
  provider = 0;
  fail_sound_bind = false;
  allow_address36 = false;
  input_tries = 0;
  return 0;
}
EX int test_product_io(void) {
  dbt_ble_control_state_t initial = {0};
  probe_product_io_hooks_t hooks = {0, valid, wake_owner, true, true, true, false};
  mounted = false;
  CHECK(!probe_product_io_startup_worker(&initial, &hooks));
  mounted = true;
  recovery_state[0] = DBT_SOUND_RECOVERY_NEEDED;
  recovery_state[2] = DBT_SOUND_RECOVERY_NEEDED;
  recovery_state[3] = DBT_SOUND_RECOVERY_BLOCKED;
  CHECK(!probe_product_io_startup_worker(&initial, &hooks));
  CHECK(reconciled == 0 && provider == 0);
  recovery_state[3] = DBT_SOUND_RECOVERY_CLEAN;
  inspect_error = 3;
  CHECK(!probe_product_io_startup_worker(&initial, &hooks));
  CHECK(reconciled == 0 && provider == 0);
  inspect_error = -1;
  reconcile_error = 0;
  CHECK(!probe_product_io_startup_worker(&initial, &hooks));
  CHECK(reconciled == 1 && provider == 0);
  reconcile_error = -1;
  input_tries = 0;
  CHECK(probe_product_io_startup_worker(&initial, &hooks));
  probe_product_io_capabilities_t caps;
  CHECK(probe_product_io_capabilities(&caps) && caps.ready && caps.led && caps.sound &&
        caps.input==PROBE_PRODUCT_INPUT_AVAILABLE);
  CHECK(input_tries == 2 && tried[0] == 0x36 && tried[1] == 0x07);
  CHECK(reconciled == 3 && recovery_state[0] == DBT_SOUND_RECOVERY_CLEAN &&
        recovery_state[2] == DBT_SOUND_RECOVERY_CLEAN && inspected >= 12);
  CHECK(initial.sound == 3 && initial.led[0] == 0);
  /* Pause waits for a copied partial sample and then freezes new sensor
   * reads; already-published actions remain the owner's responsibility. */
  input_reads=0;input_tick();CHECK(input_reads==1);
  io.input.pending=true;input_drain_result=DBT_INPUT_DRAIN_BUSY;
  CHECK(probe_product_io_input_pause(true)==DBT_OK && probe_product_io_input_paused()==DBT_BUSY);
  input_tick();CHECK(input_reads==1 && probe_product_io_input_paused()==DBT_BUSY);
  io.input.input.queue[0]=(dbt_input_action_t){DBT_INPUT_EQ_BASS,1};atomic_store(&io.input.input.head,1);
  input_drain_result=DBT_INPUT_DRAIN_DONE;input_tick();
  CHECK(probe_product_io_input_paused()==DBT_OK && !probe_product_io_input_drained() && input_reads==1);
  dbt_input_action_t input_action;CHECK(probe_product_io_take_input(&input_action) && input_action.kind==DBT_INPUT_EQ_BASS);
  CHECK(probe_product_io_input_drained());input_tick();CHECK(input_reads==1);
  CHECK(probe_product_io_input_pause(false)==DBT_OK);input_tick();CHECK(input_reads==2);
  input_drain_result=DBT_INPUT_DRAIN_FAULT;CHECK(probe_product_io_input_pause(true)==DBT_OK);input_tick();
  CHECK(probe_product_io_input_paused()==DBT_INVALID && !probe_product_io_input_drained());
  CHECK(probe_product_io_input_pause(false)==DBT_OK);input_drain_result=DBT_INPUT_DRAIN_DONE;
  probe_product_io_input_stats_t stats;
  atomic_store(&io.input.invalid_channels, 2);
  CHECK(probe_product_io_input_stats(&stats) && stats.invalid_channels == 2);
  dbt_ble_control_peer_t peer = {1, 2};
  uint8_t response[16], size = sizeof response, bright = 75;
  CHECK(probe_product_io_command(0, &peer, 7, &bright, 1, response, &size));
  CHECK(size == 2 && response[0] == 0x10 && response[1] == 7);
  dbt_led_settings_t snap;
  CHECK(probe_product_io_led_snapshot(&snap) && snap.bytes[1] == 75);
  commit_ok = false;
  size = sizeof response;
  bright = 55;
  CHECK(!probe_product_io_command(0, &peer, 7, &bright, 1, response, &size));
  commit_ok = true;
  show_ok = false;
  size = sizeof response;
  CHECK(!probe_product_io_command(0, &peer, 7, &bright, 1, response, &size));
  CHECK(probe_product_io_led_snapshot(&snap) && snap.bytes[1] == 55);
  show_ok = true;
  delay_led = true;
  size = sizeof response;
  bright = 65;
  CHECK(!probe_product_io_command(0, &peer, 7, &bright, 1, response, &size));
  CHECK(io.led_waiting);
  delay_led = false;
  CHECK(dbt_led_work(&provider->led));
  CHECK(probe_product_io_worker_poll(0, false));
  CHECK(!io.led_waiting);
  size = sizeof response;
  bright = 70;
  CHECK(probe_product_io_command(0, &peer, 7, &bright, 1, response, &size));
  unsigned committed_before = commits, shown_before = shows;
  delay_led = true;
  disconnect_on_wait = true;
  size = sizeof response;
  bright = 72;
  CHECK(!probe_product_io_command(0, &peer, 7, &bright, 1, response, &size));
  CHECK(commits == committed_before && shows == shown_before);
  CHECK(probe_product_io_led_snapshot(&snap) && snap.bytes[1] == 70);
  peer_ok = true;
  delay_led = true;
  stale_on_wait = true;
  size = sizeof response;
  CHECK(probe_product_io_command(0, &peer, 7, &bright, 1, response, &size));
  CHECK(probe_product_io_led_snapshot(&snap) && snap.bytes[1] == 72);
  disconnect_in_commit = true;
  size = sizeof response;
  bright = 73;
  CHECK(!probe_product_io_command(0, &peer, 7, &bright, 1, response, &size));
  CHECK(probe_product_io_led_snapshot(&snap) && snap.bytes[1] == 73);
  peer_ok = true;
  disconnects = 0;
  /* An earlier local detent resolves against an intervening completed BLE
   * write, including both raw five-count brightness steps. */
  CHECK(probe_product_io_local_led_relative(DBT_LED_ADJUST_BRIGHTNESS, 5,
                                            900) == DBT_OK);
  bright = 40;
  size = sizeof response;
  CHECK(probe_product_io_command(0, &peer, 7, &bright, 1, response, &size));
  CHECK(probe_product_io_worker_poll(0, false));
  CHECK(probe_product_io_worker_poll(0, false));
  dbt_led_receipt_t relative;
  CHECK(probe_product_io_take_local_led(&relative) && relative.token == 900 &&
        relative.committed && relative.settings.bytes[1] == 42);
  CHECK(probe_product_io_local_led_relative(DBT_LED_ADJUST_BRIGHTNESS, 5,
                                            901) == DBT_OK);
  CHECK(probe_product_io_worker_poll(0, false));
  CHECK(probe_product_io_worker_poll(0, false));
  CHECK(probe_product_io_take_local_led(&relative) && relative.token == 901 &&
        relative.settings.bytes[1] == 44);
  CHECK(probe_product_io_local_led_relative(DBT_LED_PREVIEW_DELTA, -1, 902) ==
        DBT_OK);
  CHECK(probe_product_io_worker_poll(0, false));
  CHECK(probe_product_io_worker_poll(0, false));
  CHECK(probe_product_io_take_local_led(&relative) && !relative.committed &&
        relative.settings.bytes[0] == 21);
  CHECK(probe_product_io_local_led_relative(DBT_LED_ADJUST_BRIGHTNESS, 5,
                                            904) == DBT_OK);
  CHECK(probe_product_io_worker_poll(0, false));
  CHECK(probe_product_io_worker_poll(0, false));
  CHECK(probe_product_io_take_local_led(&relative) && relative.committed &&
        relative.settings.bytes[0] == 21);
  CHECK(io.led.led.legacy[9] == 0); /* brightness did not persist preview */
  CHECK(probe_product_io_local_led_relative(DBT_LED_CONFIRM_PREVIEW, 0, 903) ==
        DBT_OK);
  CHECK(probe_product_io_worker_poll(0, false));
  CHECK(probe_product_io_worker_poll(0, false));
  CHECK(probe_product_io_take_local_led(&relative) && relative.committed &&
        relative.settings.bytes[0] == 21);
  for (unsigned k = 0; k < 4; k++)
    CHECK(probe_product_io_local_led(DBT_LED_SET_EFFECT, (uint8_t)k, 100 + k) ==
          DBT_OK);
  CHECK(probe_product_io_local_led(DBT_LED_SET_EFFECT, 5, 200) == DBT_BUSY);
  for (unsigned k = 0; k < 5; k++)
    CHECK(probe_product_io_worker_poll(0, false));
  for (unsigned k = 0; k < 4; k++) {
    dbt_led_receipt_t r;
    CHECK(probe_product_io_take_local_led(&r) && r.token == 100 + k &&
          r.committed && r.result == DBT_LED_OK);
  }
  /* Real BLE parser and sound upload state machine with in-memory storage. */
  dbt_ble_control_state_t state = initial;
  state.valid = true;
  dbt_ble_control_hooks_t control = {0};
  control.command = probe_product_io_command;
  dbt_ble_control_result_t result;
  const uint8_t start[] = {0x12, 2, 46, 0, 0};
  CHECK(dbt_ble_control_execute(&state, &control, &peer, start, sizeof start,
                                &result) == DBT_OK);
  CHECK(result.size == 1 && result.data[0] == 0x31);
  const uint8_t wav[46] = {'R', 'I', 'F', 'F', 38,   0,    0,   0,   'W',  'A',
                           'V', 'E', 'f', 'm', 't',  ' ',  16,  0,   0,    0,
                           1,   0,   1,   0,   0x40, 0x1f, 0,   0,   0x80, 0x3e,
                           0,   0,   2,   0,   16,   0,    'd', 'a', 't',  'a',
                           2,   0,   0,   0,   0,    0};
  uint8_t chunk[48] = {0x13, 0};
  memcpy(chunk + 2, wav, 46);
  CHECK(dbt_ble_control_execute(&state, &control, &peer, chunk, sizeof chunk,
                                &result) == DBT_OK);
  CHECK(result.size == 1 && result.data[0] == 0x31);
  const uint8_t end[] = {0x14};
  CHECK(dbt_ble_control_execute(&state, &control, &peer, end, 1, &result) ==
        DBT_OK);
  CHECK(result.size == 1 && result.data[0] == 0x32 && file_size[2] == 46 &&
        probe_product_io_sound_status() == 7);
  const uint8_t start_later[] = {0x12, 3, 46, 0, 0};
  CHECK(dbt_ble_control_execute(&state, &control, &peer, start_later,
                                sizeof start_later, &result) == DBT_OK &&
        result.data[0] == 0x31);
  ms += 30001;
  CHECK(probe_product_io_worker_poll(0, false));
  dbt_ble_control_peer_t notice_peer;
  uint8_t notice[3], notice_size;
  CHECK(probe_product_io_take_notice(&notice_peer, notice, &notice_size));
  CHECK(notice_peer.handle == 1 && notice_peer.generation == 2 &&
        notice_size == 2 && notice[0] == 0x33 && notice[1] == 0x0b);
  fail_play = true;
  CHECK(probe_product_io_play(1, 48000, false) == DBT_OK);
  CHECK(probe_product_io_worker_poll(0, false));
  probe_product_io_play_result_t failed_start;
  CHECK(probe_product_io_take_play_result(&failed_start) &&
        failed_start.type == 1 && failed_start.result == DBT_INVALID);
  CHECK(!probe_product_io_sound_busy());
  fail_play = false;
  /* Owner invalidation cancels a copied, not-yet-opened job. */
  CHECK(probe_product_io_play(0, 48000, true) == DBT_OK);
  probe_product_io_cancel_sound();
  CHECK(probe_product_io_worker_poll(0, false));
  probe_product_io_play_result_t cancelled;
  CHECK(probe_product_io_take_play_result(&cancelled) && cancelled.type == 0 && cancelled.result == DBT_EMPTY);
  CHECK(!probe_product_io_sound_busy());retire_play = false;
  CHECK(probe_product_io_play(0, 48000, false) == DBT_OK);
  CHECK(probe_product_io_worker_poll(0, false));
  CHECK(probe_product_io_sound_busy());
  probe_product_io_play_result_t played;
  CHECK(probe_product_io_take_play_result(&played) && played.type == 0 &&
        played.result == DBT_OK);
  size = sizeof response;
  CHECK(!probe_product_io_command(0, &peer, 7, &bright, 1, response, &size));
  CHECK(probe_product_io_local_led(DBT_LED_SET_EFFECT, 14, 500) == DBT_OK);
  CHECK(probe_product_io_worker_poll(0, false));
  dbt_led_receipt_t r;
  CHECK(!probe_product_io_take_local_led(&r));
  probe_product_io_cancel_sound();
  CHECK(probe_product_io_worker_poll(0, false));
  CHECK(!probe_product_io_sound_busy());
  CHECK(probe_product_io_worker_poll(0, false));
  CHECK(probe_product_io_take_local_led(&r) && r.token == 500);
  size = sizeof response;
  uint8_t mute = 1;
  CHECK(probe_product_io_command(0, &peer, 0x10, &mute, 1, response, &size));
  CHECK(probe_product_io_sound_status() == 135);
  size = sizeof response;
  uint8_t upload_start[4] = {3, 46, 0, 0};
  CHECK(probe_product_io_command(0, &peer, 0x12, upload_start, 4, response,
                                 &size) &&
        response[0] == 0x31);
  dbt_ble_control_peer_t stale = {1, 1};
  probe_product_io_peer_lost(0, &stale);
  CHECK(io.sound.active);
  probe_product_io_peer_lost(0, &peer);
  CHECK(probe_product_io_worker_poll(0, false));
  CHECK(!io.sound.active);
  CHECK(disconnects == 2);
  dbt_input_sample_t sample = {0};
  CHECK(dbt_input_feed(&io.input.input, &sample));
  sample.position[1] = 4;
  CHECK(dbt_input_feed(&io.input.input, &sample));
  CHECK(probe_product_io_worker_poll(0, true));
  dbt_input_action_t act;
  CHECK(probe_product_io_take_input(&act) && act.kind == DBT_INPUT_EQ_BASS);
  CHECK(probe_product_io_local_led(DBT_LED_SET_EFFECT, 14, 700) == DBT_OK);
  CHECK(probe_product_io_needs_worker());
  CHECK(probe_product_io_worker_poll(0, false));
  CHECK(probe_product_io_worker_poll(0, false));
  CHECK(probe_product_io_take_local_led(&r) && r.token == 700);
  CHECK(probe_product_io_worker_poll(0, true));
  CHECK(wake_worker_count && wake_owner_count);
  return 0;
}
