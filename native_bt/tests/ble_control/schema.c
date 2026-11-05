/* Actual ATT server and product adapter with deterministic worker/NVS shims. */
#include "../../esp32_probe/main/probe_gatt.c"
#include "dbt_ble_nvs.h"
#include "nvs.h"
#include <assert.h>
#include <setjmp.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
  size_t item;
  unsigned head, count, capacity;
  uint8_t *bytes;
} queue_t;
static unsigned allocations, commits, published, publish_failures,
    control_published, control_failures, provider_calls;
static jmp_buf worker_exit;
static bool running_worker,fail_spawn,resume_test,resume_injected;
static unsigned spawn_calls;
static unsigned normal_polls,retire_polls,retire_delays,retire_waits,peer_losses;
static bool worker_context, fail_commit, load_fixture, load_error,
    bad_eq_fixture;
static bool committed_bond;
static uint32_t fake_cookie=1;
static uint32_t capture_cookie(uint16_t h,uint32_t generation){return h && generation?fake_cookie:0;}
static bool cookie_valid(uint16_t h,uint32_t generation,uint32_t cookie){return h && generation && cookie==fake_cookie;}
static bool ota_enabled,ota_selected,ota_valid=true,ota_finish_error;
static bool ota_prepared=true,ota_prepare_error;
static unsigned ota_delivered_count,ota_abandoned,ota_finish_calls;
static uint32_t ota_admit(void *ctx,const dbt_ble_control_peer_t *p,uint8_t cmd,const uint8_t *d,uint16_t n) {
  (void)ctx;(void)cmd;(void)d;(void)n;return ota_enabled && p->generation?1u:0u;
}
static void ota_abandon(void *ctx,const dbt_ble_control_peer_t *p,uint32_t epoch,uint8_t cmd) {
  (void)ctx;(void)p;(void)epoch;(void)cmd;ota_abandoned++;
}
static bool ota_check(void *ctx,const dbt_ble_control_peer_t *p,uint32_t epoch) {
  (void)ctx;return ota_valid && p->generation && epoch==1;
}
static bool ota_command(void *ctx,const dbt_ble_control_peer_t *p,uint32_t epoch,uint8_t cmd,
                        const uint8_t *data,uint16_t n,uint8_t *out,uint8_t *size) {
  (void)ctx;(void)p;(void)epoch;(void)data;(void)n;assert(worker_context && *size>=2);
  out[0]=cmd==0x20?0x21:cmd==0x21?0x20:0x22;out[1]=100;*size=cmd==0x21?2:1;return true;
}
static dbt_result_t ota_finish(void *ctx,const dbt_ble_control_peer_t *p,uint32_t epoch) {
  (void)ctx;(void)p;(void)epoch;ota_finish_calls++;
  return ota_finish_error?DBT_INVALID:ota_selected?DBT_OK:DBT_BUSY;
}
static void ota_delivered(void *ctx,const dbt_ble_control_peer_t *p,uint32_t epoch) {
  (void)ctx;(void)p;(void)epoch;ota_delivered_count++;
}
static dbt_result_t ota_prepare(void *ctx,const dbt_ble_control_peer_t *p,uint32_t epoch) {
  (void)ctx;(void)p;(void)epoch;
  if(ota_prepare_error)return DBT_INVALID;
  return ota_prepared && probe_gatt_restart_prepare_quiescent()?DBT_OK:DBT_BUSY;
}
static bool fail_heap;
static uint8_t test_bond_tag=1;
static bool cccd_fail;
static uint16_t cccd_saved[4][PROBE_CCCD_COUNT];
static bool cccd_present[4];
static dbt_result_t fake_security(uint16_t handle,uint32_t generation,dbt_smp_session_security_t *out) {
  assert(handle==1 && generation);memset(out,0,sizeof(*out));out->bond_committed=committed_bond;
  out->encrypted=committed_bond;out->key_size=committed_bond?16:0;
  return DBT_OK;
}
static dbt_result_t fake_bond(uint16_t handle,uint32_t generation,uint8_t *type,uint8_t id[6],uint8_t tag[32]) {
  assert(handle==1 && generation);if(!committed_bond)return DBT_INVALID;
  *type=0;memset(id,1,6);memset(tag,test_bond_tag,32);return DBT_OK;
}
dbt_result_t probe_cccd_load(uint8_t type,const uint8_t id[6],const uint8_t tag[32],uint16_t values[4]) {
  assert(worker_context && type==0 && id[0]==1 && tag[0]<4);
  memcpy(values,cccd_saved[tag[0]],sizeof(cccd_saved[0]));return cccd_present[tag[0]]?DBT_OK:DBT_EMPTY;
}
dbt_result_t probe_cccd_save(uint8_t type,const uint8_t id[6],const uint8_t tag[32],const uint16_t values[4]) {
  assert(worker_context && type==0 && id[0]==1 && tag[0]<4);
  if(cccd_fail)return DBT_INVALID;
  memcpy(cccd_saved[tag[0]],values,sizeof(cccd_saved[0]));cccd_present[tag[0]]=true;return DBT_OK;
}
static uint32_t durable_eq, staged_eq;
static uint8_t durable_control, staged_control;
static bool eq_staged, control_staged;
QueueHandle_t xQueueCreate(UBaseType_t n, UBaseType_t size) {
  queue_t *q = calloc(1, sizeof(*q));
  assert(q);
  q->capacity = n;
  q->item = size;
  q->bytes = calloc(n, size);
  assert(q->bytes);
  return q;
}
BaseType_t xQueueSend(QueueHandle_t v, const void *p, TickType_t timeout) {
  (void)timeout;
  queue_t *q = v;
  if (q->count == q->capacity)
    return 0;
  memcpy(q->bytes + ((q->head + q->count) % q->capacity) * q->item, p, q->item);
  q->count++;
  return pdTRUE;
}
BaseType_t xQueueReceive(QueueHandle_t v, void *p, TickType_t timeout) {
  if(worker_context && v==control_signal)assert(timeout);
  if(worker_context && v==control_signal && timeout && timeout!=portMAX_DELAY && retire_polls){
    assert(timeout==(pdMS_TO_TICKS(20)?pdMS_TO_TICKS(20):1));retire_waits++;
    if(resume_test && !resume_injected){
      resume_injected=true;worker_context=false;
      const dbt_link_t remote={.handle=7,.generation=1,.kind=DBT_LINK_LE};
      const uint8_t ping[]={0xff};const dbt_att_write_t w={.data=ping,.size=1};
      assert(control_submit(&remote,&w,true,0)==0);worker_context=true;
    }
  }
  queue_t *q = v;
  if (!q->count)
    return 0;
  memcpy(p, q->bytes + q->head * q->item, q->item);
  memset(q->bytes + q->head * q->item, 0, q->item);
  q->head = (q->head + 1) % q->capacity;
  q->count--;
  return pdTRUE;
}
UBaseType_t uxQueueMessagesWaiting(QueueHandle_t v) {
  return ((queue_t *)v)->count;
}
void vQueueDelete(QueueHandle_t v) {
  queue_t *q = v;
  free(q->bytes);
  free(q);
}
TaskHandle_t xTaskGetCurrentTaskHandle(void) { return (TaskHandle_t)1; }
BaseType_t xTaskCreate(void (*fn)(void *), const char *name, uint32_t stack,
                       void *arg, UBaseType_t pri, TaskHandle_t *out) {
  (void)fn;
  (void)name;
  (void)arg;
  (void)pri;
  (void)out;
  assert(stack == CONTROL_STACK_BYTES);
  spawn_calls++;return fail_spawn?0:pdPASS;
}
void xTaskNotifyGive(TaskHandle_t h) { assert(h); }
void vTaskDelete(TaskHandle_t h) {
  (void)h;
  if(running_worker)longjmp(worker_exit,1);
  abort();
}
void taskYIELD(void) { assert(worker_context); }
void vTaskDelay(TickType_t t) {
  assert(worker_context && t);retire_delays++;
}
void *heap_caps_malloc(size_t n, uint32_t flags) {
  (void)flags;
  if(fail_heap)return NULL;
  void *p = malloc(n);
  if (p)
    allocations++;
  return p;
}
void heap_caps_free(void *p) {
  if (p) {
    assert(allocations);
    allocations--;
    free(p);
  }
}
esp_err_t nvs_open(const char *name, int mode, nvs_handle_t *h) {
  assert(worker_context);
  if (!strcmp(name, "audio")) {
    *h = 1;
    return ESP_OK;
  }
  if (!strcmp(name, "led") && mode == NVS_READONLY) {
    *h = 2;
    return ESP_OK;
  }
  return ESP_ERR_NVS_NOT_FOUND;
}
esp_err_t nvs_set_u32(nvs_handle_t h, const char *key, uint32_t v) {
  assert(worker_context && h == 1 && !strcmp(key, "eq_v1"));
  staged_eq = v;
  eq_staged = true;
  return ESP_OK;
}
esp_err_t nvs_set_u8(nvs_handle_t h, const char *key, uint8_t v) {
  assert(worker_context && h == 1 && !strcmp(key, "ctrl"));
  staged_control = v;
  control_staged = true;
  return ESP_OK;
}
esp_err_t nvs_commit(nvs_handle_t h) {
  assert(worker_context && h == 1);
  commits++;
  if (fail_commit)
    return 1;
  if (eq_staged)
    durable_eq = staged_eq;
  if (control_staged)
    durable_control = staged_control;
  eq_staged = control_staged = false;
  return ESP_OK;
}
void nvs_close(nvs_handle_t h) { assert(worker_context && (h == 1 || h == 2)); }
esp_err_t nvs_get_str(nvs_handle_t h, const char *key, char *out, size_t *n) {
  assert(worker_context && h == 1 && !strcmp(key, "devname"));
  if (load_error)
    return 1;
  if (!load_fixture)
    return ESP_ERR_NVS_NOT_FOUND;
  assert(*n >= 6);
  memcpy(out, "Saved", 6);
  *n = 6;
  return ESP_OK;
}
esp_err_t nvs_get_u8(nvs_handle_t h, const char *key, uint8_t *out) {
  assert(worker_context);
  if (!load_fixture)
    return ESP_ERR_NVS_NOT_FOUND;
  if (h == 1 && !strcmp(key, "ctrl"))
    *out = 0x85;
  else if (h == 1 && !strcmp(key, "led_effect"))
    *out = 3;
  else if (h == 1 && !strcmp(key, "sound_muted"))
    *out = 1;
  else
    return ESP_ERR_NVS_NOT_FOUND;
  return ESP_OK;
}
esp_err_t nvs_get_u32(nvs_handle_t h, const char *key, uint32_t *out) {
  assert(worker_context && h == 1 && !strcmp(key, "eq_v1"));
  if (!load_fixture)
    return ESP_ERR_NVS_NOT_FOUND;
  *out = bad_eq_fixture ? 0x01040380u : 0x010403feu;
  return ESP_OK;
}
esp_err_t nvs_get_i8(nvs_handle_t h, const char *key, int8_t *out) {
  (void)out;
  assert(worker_context && h == 1 && key);
  return ESP_ERR_NVS_NOT_FOUND;
}
esp_err_t nvs_get_blob(nvs_handle_t h, const char *key, void *out, size_t *n) {
  assert(worker_context && h == 2 && !strcmp(key, "settings"));
  if (!load_fixture)
    return ESP_ERR_NVS_NOT_FOUND;
  assert(*n >= 10);
  for (unsigned i = 0; i < 10; i++)
    ((uint8_t *)out)[i] = (uint8_t)i;
  ((uint8_t *)out)[0] = 128;
  *n = 10;
  return ESP_OK;
}
static bool authorize(void *ctx, const dbt_link_t *l, uint8_t command) {
  (void)ctx;
  (void)command;
  return l && l->generation == 1;
}
static bool publish_eq(void *ctx, int8_t b, int8_t m, int8_t t) {
  (void)ctx;
  (void)b;
  (void)m;
  (void)t;
  if (publish_failures) {
    publish_failures--;
    return false;
  }
  published++;
  return true;
}
static bool publish_control(void *ctx, uint8_t c) {
  (void)ctx;
  (void)c;
  if (control_failures) {
    control_failures--;
    return false;
  }
  control_published++;
  return true;
}
static bool provider(void *ctx, const dbt_ble_control_peer_t *peer, uint8_t cmd,
                     const uint8_t *p, uint16_t n, uint8_t *out,
                     uint8_t *size) {
  (void)ctx;
  (void)peer;
  (void)p;
  (void)n;
  assert(worker_context);
  provider_calls++;
  out[0] = 0x10;
  out[1] = cmd;
  *size = 2;
  return true;
}
static uint8_t *alloc_runtime(void *p, uint16_t n) {
  (void)p;
  return malloc(n);
}
static void free_runtime(void *p, uint8_t *x) {
  (void)p;
  free(x);
}
static void fixed(void *ctx, const dbt_link_t *l, uint16_t cid,
                  const uint8_t *p, uint16_t n) {
  (void)ctx;
  (void)l;
  (void)cid;
  (void)p;
  (void)n;
}
static dbt_runtime_t runtime;
static dbt_gap_t gap_state;
static dbt_link_t *link0;
static void execute_one(void) {
  control_job_t *j = 0;
  QueueHandle_t q = uxQueueMessagesWaiting(eq_work) ? eq_work : control_work;
  assert(xQueueReceive(q, &j, 0) == pdTRUE && j);
  worker_context = true;
  control_execute_job(j);
  worker_context = false;
  assert(xQueueSend(control_done, &j, 0) == pdTRUE);
}
static void tx(uint8_t *out, unsigned *n) {
  dbt_att_view_t view;
  assert(dbt_att_tx_peek(&server, link0->handle, link0->generation, &view) ==
         DBT_OK);
  *n = view.size;
  memcpy(out, view.data, view.size);
  assert(dbt_att_tx_admitted(&server, link0->handle, link0->generation, 1) ==
         DBT_OK);
}
static void write_req(uint16_t h, const uint8_t *value, unsigned n,
                      bool command) {
  uint8_t p[520] = {0};
  assert(n <= 512);
  p[0] = command ? 0x52 : 0x12;
  p[1] = (uint8_t)h;
  p[2] = (uint8_t)(h >> 8);
  memcpy(p + 3, value, n);
  assert(dbt_att_receive(&server, link0, p, (uint16_t)(n + 3), 1) == DBT_OK);
}
static bool worker_poll(void *ctx,bool retiring){
  (void)ctx;assert(worker_context);
  if(retiring)return ++retire_polls>=3;
  normal_polls++;
  /* Model an idle sound/meter wake; a null queue item requests retirement. */
  probe_gatt_control_wake();
  control_job_t *stop=0;assert(xQueueSend(control_work,&stop,0)==pdTRUE);
  return true;
}
static void peer_lost(void *ctx,const dbt_ble_control_peer_t *peer){
  (void)ctx;assert(!worker_context && peer && peer->generation);peer_losses++;
}
static void run_general_worker(void){
  worker_context=running_worker=true;
  if(!setjmp(worker_exit))control_task(0);
  worker_context=running_worker=false;
}
int main(void) {
  runtime.ready = true;
  runtime.host.state = DBT_HOST_IDENTIFIED;
  runtime.maximum_mtu = 1024;
  runtime.hooks.allocate = alloc_runtime;
  runtime.hooks.release = free_runtime;
  runtime.channels.hooks.fixed_data = fixed;
  link0 = &runtime.acl.links[0];
  link0->used = true;
  link0->kind = DBT_LINK_LE;
  link0->handle = 1;
  link0->generation = 1;
  link0->encrypted = true;
  dbt_ble_control_state_t state = {0};
  state.valid = true;
  state.name_size = 4;
  memcpy(state.name, "Test", 4);
  state.firmware_size = 2;
  memcpy(state.firmware, "v1", 2);
  state.led[1] = 50;
  dbt_ble_control_state_t loaded = state;
  loaded.sound = 5;
  load_fixture = true;
  worker_context = true;
  assert(dbt_ble_nvs_load_state(&loaded) == DBT_OK && loaded.valid &&
         loaded.eq[0] == -2 && loaded.eq[1] == 3 && loaded.eq[2] == 4 &&
         loaded.control == 5 && loaded.name_size == 5 &&
         !memcmp(loaded.name, "Saved", 5) && loaded.led[0] == 3 &&
         loaded.led[1] == 50 && loaded.led[2] == 8 && loaded.led[3] == 1 &&
         loaded.led[8] == 6 && loaded.led[9] == 7 && loaded.sound == 0x85 &&
         !commits);
  dbt_ble_control_state_t unchanged = loaded;
  load_error = true;
  assert(dbt_ble_nvs_load_state(&loaded) == DBT_INVALID &&
         !memcmp(&loaded, &unchanged, sizeof(loaded)) && !commits);
  load_error = false;
  bad_eq_fixture = true;
  assert(dbt_ble_nvs_load_state(&loaded) == DBT_INVALID &&
         !memcmp(&loaded, &unchanged, sizeof(loaded)) && !commits);
  bad_eq_fixture = false;
  load_error = load_fixture = false;
  worker_context = false;
  dbt_ble_control_hooks_t hooks = {0};
  hooks.authorize = authorize;
  hooks.peer_lost=peer_lost;
  hooks.commit_eq = dbt_ble_nvs_commit_eq;
  hooks.commit_control = dbt_ble_nvs_commit_control;
  hooks.publish_eq = publish_eq;
  hooks.publish_control = publish_control;
  assert(probe_gatt_security_query_configure(fake_security)==DBT_OK &&
         probe_gatt_bond_query_configure(fake_bond)==DBT_OK &&
         probe_gatt_control_trust_configure(capture_cookie,cookie_valid)==DBT_OK &&
         probe_gatt_ota_configure(&(probe_gatt_ota_hooks_t){NULL,ota_admit,ota_abandon,ota_check,ota_command,ota_finish,ota_delivered,ota_prepare})==DBT_OK &&
         probe_gatt_control_configure(&state, &hooks) == DBT_OK &&
         probe_gatt_init(&runtime, &gap_state) == DBT_OK);
  dbt_gap_config_t ad = {0};
  probe_gatt_advertisement(&ad);
  assert(ad.advertising_size == 21 &&
         !memcmp(ad.advertising + 5, product_service, 16));
  uint8_t out[128];
  unsigned n;
  /* Actual ATT Read By Group Type returns the product service UUID. */
  uint8_t group[] = {0x10, 0x20, 0, 0xff, 0xff, 0, 0x28};
  assert(dbt_att_receive(&server, link0, group, sizeof(group), 0) == DBT_OK);
  tx(out, &n);
  assert(n == 22 && out[0] == 0x11 && out[1] == 20 &&
         !memcmp(out + 6, product_service, 16));
  uint8_t enabled[] = {1, 0};
  write_req(0x25, enabled, 2, false);
  tx(out, &n);
  assert(n == 1 && out[0] == 0x13);
  pump_control();
  tx(out, &n);
  assert(out[0] == 0x1b && out[1] == 0x24 && out[3] == 0xf0);
  uint8_t full[82];
  unsigned fulln = n - 3;
  memcpy(full, out + 3, fulln);
  pump_control();
  tx(out, &n);
  assert(out[0] == 0x1b && out[1] == 0x24);
  memcpy(full + fulln, out + 3, n - 3);
  fulln += n - 3;
  assert(fulln == 24 && full[0] == 0xf0 && full[5] == 0 && full[6] == 50 &&
         full[16] == 4 && !memcmp(full + 17, "Test", 4));
  uint8_t eq[] = {1, 0xfe, 3, 4};
  write_req(0x22, eq, sizeof(eq), false);
  assert(!commits && server.connections[0].deferred);
  memset(eq, 0, sizeof(eq));
  pump_control();
  assert(eq_worker && !commits);
  execute_one();
  assert(commits == 1 && published == 1 && durable_eq == 0x010403feu);
  pump_control();
  tx(out, &n);
  assert(n == 1 && out[0] == 0x13);
  tx(out, &n);
  assert(n == 5 && out[3] == 0x10 && out[4] == 1);
  /* Commands are copied, serialized and committed once each, even unchanged. */
  uint8_t preset[] = {2, 1}, again[] = {1, 6, 2, 0};
  write_req(0x22, preset, sizeof(preset), true);
  write_req(0x22, again, sizeof(again), true);
  memset(preset, 0, sizeof(preset));
  memset(again, 0, sizeof(again));
  execute_one();
  execute_one();
  assert(commits == 3 && published == 3 && durable_eq == 0x01000206u);
  pump_control();
  tx(out, &n);
  assert(out[3] == 0x10 && out[4] == 2);
  pump_control();
  tx(out, &n);
  assert(out[3] == 0x10 && out[4] == 1);
  fail_commit = true;
  uint8_t eq2[] = {1, 1, 2, 3};
  write_req(0x22, eq2, sizeof(eq2), false);
  execute_one();
  pump_control();
  tx(out, &n);
  assert(out[0] == 1 && out[4] == DBT_ATT_UNLIKELY);
  tx(out, &n);
  assert(out[3] == 0x11 && out[4] == 1);
  assert(commits == 4 && published == 3 && durable_eq == 0x01000206u);
  fail_commit = false;
  eq_staged = false;
  /* Publication can fail after a durable commit. Retries never recommit. */
  publish_failures = 3;
  uint8_t eq_retry[] = {1, 7, 8, 9};
  write_req(0x22, eq_retry, sizeof(eq_retry), false);
  execute_one();
  pump_control();
  tx(out, &n);
  assert(out[0] == 1);
  tx(out, &n);
  assert(out[3] == 0x11);
  assert(commits == 5 && published == 3 && durable_eq == 0x01090807u &&
         eq_publication_pending);
  assert(probe_gatt_retry_eq_publication() == DBT_OK && commits == 5 &&
         published == 4 && !eq_publication_pending);
  control_failures = 3;
  uint8_t flags[] = {3, 5};
  write_req(0x22, flags, sizeof(flags), false);
  execute_one();
  pump_control();
  tx(out, &n);
  assert(out[0] == 1);
  tx(out, &n);
  assert(out[3] == 0x11);
  assert(commits == 6 && durable_control == 5 && !control_published &&
         control_publication_pending);
  assert(probe_gatt_retry_control_publication() == DBT_OK && commits == 6 &&
         control_published == 1);
  /* Missing OTA provider yields error; it cannot report success. */
  uint8_t ota[] = {0x14};
  write_req(0x22, ota, sizeof(ota), true);
  pump_control();
  execute_one();
  pump_control();
  tx(out, &n);
  assert(out[3] == 0x11 && out[4] == 0x14);
  /* Meter subscription is independent of status. */
  write_req(0x28, enabled, 2, false);
  tx(out, &n);
  probe_gatt_meter(9, 8, 7);
  tx(out, &n);
  assert(n == 6 && out[1] == 0x27 && out[3] == 9 && out[4] == 8 && out[5] == 7);
  server.connections[0].bonded = true;
  write_req(0x28, enabled, 2, false);
  tx(out, &n);
  assert(out[0] == 1 && out[4] == DBT_ATT_UNLIKELY);
  server.connections[0].bonded = false;
  /* SMP can have a committed bond even though ATT has not been promoted. */
  committed_bond=true;
  write_req(0x28, enabled, 2, false);
  tx(out, &n);
  assert(out[0] == 1 && out[4] == DBT_ATT_UNLIKELY);
  committed_bond=false;
  /* An accepted EQ still commits on disconnect; a reused handle gets no ACK. */
  control_hooks.command = provider;
  uint8_t setname[] = {4, 'N'};
  write_req(0x22, setname, sizeof(setname), true);
  uint8_t eq3[] = {1, 4, 5, 6};
  write_req(0x22, eq3, sizeof(eq3), true);
  dbt_link_change_t remove = {DBT_LINK_REMOVED, 1, 1};
  link0->used = false;
  probe_gatt_link(NULL, &remove);
  link0->used = true;
  link0->generation = 2;
  dbt_link_change_t added = {DBT_LINK_ADDED, 1, 2};
  probe_gatt_link(NULL, &added);
  execute_one();
  execute_one();
  pump_control();
  assert(commits == 7 && published == 5 && durable_eq == 0x01060504u &&
         !control_jobs && !provider_calls);
  assert(dbt_att_tx_peek(&server, 1, 2, (dbt_att_view_t[1]){{0}}) == DBT_EMPTY);
  /* No command implies notification consent. Hold the response until an
   * actual status CCCD write and keep the meter independently disabled. */
  uint8_t ping[] = {0xff};
  write_req(0x22, ping, sizeof(ping), true);
  execute_one();
  pump_control();
  assert(control_jobs == 1 &&
         dbt_att_tx_peek(&server, 1, 2, (dbt_att_view_t[1]){{0}}) == DBT_EMPTY);
  write_req(0x22, ping, sizeof(ping), true);
  execute_one();
  pump_control();
  write_req(0x22, ping, sizeof(ping), true);
  execute_one();
  pump_control();
  uint32_t ignored = server.ignored_commands;
  write_req(0x22, ping, sizeof(ping), true);
  assert(control_jobs == 3 && server.ignored_commands == ignored + 1);
  uint8_t read_cccd[] = {0x0a, 0x25, 0};
  assert(dbt_att_receive(&server, link0, read_cccd, sizeof(read_cccd), 1) ==
         DBT_OK);
  tx(out, &n);
  assert(n == 3 && out[0] == 0x0b && !out[1] && !out[2]);
  probe_gatt_meter(1, 2, 3);
  assert(dbt_att_tx_peek(&server, 1, 2, (dbt_att_view_t[1]){{0}}) == DBT_EMPTY);
  write_req(0x25, enabled, 2, false);
  tx(out, &n);
  assert(out[0] == 0x13);
  pump_control();
  tx(out, &n);
  assert(n == 4 && out[0] == 0x1b && out[1] == 0x24 && out[3] == 0xff);
  pump_control();
  tx(out, &n);
  assert(out[3] == 0xff);
  pump_control();
  tx(out, &n);
  assert(out[3] == 0xff);
  pump_control();
  tx(out, &n);
  assert(out[3] == 0xf0);
  pump_control();
  tx(out, &n);
  uint8_t disabled[] = {0, 0};
  write_req(0x25, disabled, 2, false);
  tx(out, &n);
  assert(out[0] == 0x13);
  write_req(0x22, ping, sizeof(ping), true);
  execute_one();
  pump_control();
  assert(control_jobs == 1 &&
         dbt_att_tx_peek(&server, 1, 2, (dbt_att_view_t[1]){{0}}) == DBT_EMPTY);
  write_req(0x25, enabled, 2, false);
  tx(out, &n);
  assert(out[0] == 0x13);
  pump_control();
  tx(out, &n);
  assert(out[3] == 0xff);
  pump_control();
  tx(out, &n);
  assert(out[3] == 0xf0);
  pump_control();
  tx(out, &n);
  write_req(0x25, disabled, 2, false);
  tx(out, &n);
  write_req(0x22, ping, sizeof(ping), true);
  execute_one();
  pump_control();
  assert(control_jobs == 1);
  /* Discard a held reply when the connection generation changes. */
  link0->used = false;
  remove.generation = 2;
  probe_gatt_link(NULL, &remove);
  link0->used = true;
  link0->generation = 3;
  added.generation = 3;
  probe_gatt_link(NULL, &added);
  pump_control();
  assert(!control_jobs &&
         dbt_att_tx_peek(&server, 1, 3, (dbt_att_view_t[1]){{0}}) == DBT_EMPTY);
  write_req(0x25, enabled, 2, false);
  tx(out, &n);
  pump_control();
  tx(out, &n);
  assert(out[3] == 0xf0);
  pump_control();
  tx(out, &n);
  link0->used = false;
  remove.generation = 3;
  probe_gatt_link(NULL, &remove);
  pump_control();
  assert(eq_retiring && control_retiring);
  control_job_t *stop = (void *)1;
  assert(xQueueReceive(control_work, &stop, 0) == pdTRUE && !stop);
  stop = (void *)(uintptr_t)2;
  assert(xQueueSend(control_done, &stop, 0) == pdTRUE);
  assert(xQueueReceive(eq_work, &stop, 0) == pdTRUE && !stop);
  stop = (void *)(uintptr_t)1;
  assert(xQueueSend(control_done, &stop, 0) == pdTRUE);
  pump_control();
  assert(!control_worker && !eq_worker && !control_jobs);
  assert(allocations == 0 && peer_losses);
  /* Local detents and BLE presets share one FIFO and never coalesce commits. */
  memset(desired_eq,0,3);memset(control_display_state.eq,0,3);
  unsigned before_local=commits;
  assert(probe_gatt_control_eq_delta(0,1)==DBT_OK);
  const dbt_link_t remote={.handle=7,.generation=1,.kind=DBT_LINK_LE,.encrypted=true};
  uint8_t remote_eq[]={1,5,0,0};const dbt_att_write_t change_eq={.data=remote_eq,.size=4};
  assert(control_submit(&remote,&change_eq,true,0)==0);
  assert(probe_gatt_control_eq_delta(0,1)==DBT_OK && desired_eq[0]==6);
  assert(probe_gatt_control_eq_delta(0,-1)==DBT_OK && desired_eq[0]==5);
  assert(probe_gatt_control_eq_delta(1,1)==DBT_BUSY && desired_eq[1]==0);
  fail_spawn=true;unsigned before_eq_spawn=spawn_calls;
  pump_control();assert(!eq_worker && eq_retry_pending && control_jobs==4 && uxQueueMessagesWaiting(eq_work)==4 && commits==before_local);
  assert(probe_gatt_wait_ms(0)>0);
  for(unsigned i=0;i<4;i++)pump_control();
  assert(spawn_calls==before_eq_spawn+1 && control_jobs==4);
  fail_spawn=false;control_now_ms=100;pump_control();
  assert(eq_worker && !eq_retry_pending && control_jobs==4);
  const int8_t applied[]={1,5,6,5};
  for(unsigned i=0;i<4;i++){
    execute_one();assert(commits==before_local+i+1);pump_control();
    assert(control_display_state.eq[0]==applied[i]);
  }
  assert(eq_retiring && !control_jobs);
  worker_context=running_worker=true;
  if(!setjmp(worker_exit))control_task((void *)(uintptr_t)1);
  worker_context=running_worker=false;pump_control();
  assert(!eq_worker && !eq_retiring && !allocations);
  /* A failed final commit restores relative input's base to durable state. */
  assert(probe_gatt_control_eq_delta(0,1)==DBT_OK && desired_eq[0]==6);
  fail_commit=true;execute_one();fail_commit=false;pump_control();
  assert(desired_eq[0]==5 && control_display_state.eq[0]==5);
  assert(probe_gatt_control_eq_delta(3,1)==DBT_INVALID && probe_gatt_control_eq_delta(0,2)==DBT_INVALID);
  assert(!eq_worker && !control_jobs && !allocations);
  /* Local flag toggles share the fast persistence FIFO with BLE flag writes. */
  desired_control=control_display_state.control=0;unsigned before_toggle=commits;
  assert(probe_gatt_control_toggle(1)==DBT_OK);
  const uint8_t remote_flags[]={3,7};const dbt_att_write_t change_flags={.data=remote_flags,.size=2};
  assert(control_submit(&remote,&change_flags,true,0)==0);
  assert(probe_gatt_control_toggle(2)==DBT_OK && desired_control==5);
  assert(probe_gatt_control_toggle(4)==DBT_OK && desired_control==1);
  assert(probe_gatt_control_toggle(1)==DBT_BUSY && desired_control==1);
  control_publication_pending=true;
  assert(probe_gatt_retry_control_publication()==DBT_BUSY);
  const uint8_t flags_applied[]={1,7,5,1};
  for(unsigned i=0;i<4;i++){
    execute_one();assert(commits==before_toggle+i+1);pump_control();
    assert(control_display_state.control==flags_applied[i]);
  }
  assert(!control_writes_running && eq_retiring);
  worker_context=running_worker=true;
  if(!setjmp(worker_exit))control_task((void *)(uintptr_t)1);
  worker_context=running_worker=false;pump_control();
  assert(!eq_worker && !eq_retiring);
  assert(probe_gatt_control_toggle(1)==DBT_OK && desired_control==0);
  fail_commit=true;execute_one();fail_commit=false;pump_control();
  assert(desired_control==1 && control_display_state.control==1 && !control_writes_running);
  assert(probe_gatt_control_toggle(0)==DBT_INVALID && probe_gatt_control_toggle(3)==DBT_INVALID);
  /* The real worker loop remains alive while an asynchronous provider retires. */
  assert(probe_gatt_control_worker_required(true)==DBT_UNSUPPORTED);
  control_now_ms=0;control_hooks.worker_poll=worker_poll;
  assert(probe_gatt_control_worker_required(true)==DBT_OK && probe_gatt_wait_ms(0)==0);
  fail_spawn=true;unsigned before_spawn=spawn_calls;
  pump_control();assert(!control_worker && control_retry_pending && spawn_calls==before_spawn+1);
  assert(probe_gatt_wait_ms(0)>0 && probe_gatt_wait_ms(0)<=100);
  for(unsigned i=0;i<4;i++)pump_control();
  assert(spawn_calls==before_spawn+1);
  fail_spawn=false;control_now_ms=100;
  pump_control();assert(control_worker && !control_retiring && !control_jobs && !control_retry_pending);
  pump_control();assert(control_worker && !control_retiring);
  assert(probe_gatt_control_worker_required(false)==DBT_OK);
  run_general_worker();
  assert(normal_polls==1 && retire_polls==3 && !retire_delays && retire_waits==2);
  assert(probe_gatt_control_worker_required(false)==DBT_OK);
  pump_control();assert(!control_worker && !control_jobs);
  normal_polls=retire_polls=retire_delays=retire_waits=0;resume_test=true;
  control_worker=control_retiring=true;control_job_t *retire_item=0;
  assert(xQueueSend(control_work,&retire_item,0)==pdTRUE);
  run_general_worker();
  assert(resume_injected && normal_polls==1 && retire_polls==3 && control_jobs==1);
  assert(!retire_delays && retire_waits==2);
  pump_control();assert(!control_worker && !control_retiring && !control_jobs && !allocations);
  resume_test=false;
  /* Wakes remain safe after the task deleted itself; no task handle is kept. */
  probe_gatt_control_wake();probe_gatt_control_wake();
  assert(uxQueueMessagesWaiting(control_signal)==1);
  dbt_ble_control_state_t displayed;
  const uint8_t renamed[]={'N',0xe2,0x82,0xac};
  assert(probe_gatt_control_name_applied(renamed,sizeof(renamed))==DBT_OK);
  assert(probe_gatt_control_state(&displayed)==DBT_OK && displayed.name_size==4 && !memcmp(displayed.name,renamed,4));
  uint8_t led_applied[]={21,100,8,1,2,3,4,5,6,4};
  assert(probe_gatt_control_led_applied(led_applied)==DBT_OK && probe_gatt_control_sound_applied(0x8f)==DBT_OK);
  assert(probe_gatt_control_state(&displayed)==DBT_OK && displayed.sound==0x8f && !memcmp(displayed.led,led_applied,10));
  led_applied[0]=22;assert(probe_gatt_control_led_applied(led_applied)==DBT_INVALID);
  assert(probe_gatt_control_sound_applied(0x10)==DBT_INVALID && probe_gatt_control_state(&displayed)==DBT_OK && displayed.sound==0x8f && displayed.led[0]==21);
  const uint8_t invalid_name[]={0xed,0xa0,0x80};
  assert(probe_gatt_control_name_applied(invalid_name,sizeof(invalid_name))==DBT_INVALID);
  assert(probe_gatt_control_state(&displayed)==DBT_OK && !memcmp(displayed.name,renamed,4));
  uint16_t written,total;uint8_t read_name[32];
  assert(read(0,link0,&attributes[2],0,read_name,sizeof(read_name),&written,&total)==0 && total==4 && written==4 && !memcmp(read_name,renamed,4));
  const dbt_ble_control_peer_t stale={1,2};const uint8_t response[]={0x11,0x20,3};
  assert(probe_gatt_control_notify(&stale,response,sizeof(response))==DBT_UNSUPPORTED);
  link0->used=true;dbt_ble_control_peer_t current={link0->handle,link0->generation};
  assert(probe_gatt_control_notify(&current,response,sizeof(response))==DBT_OK && control_jobs==1);
  link0->generation++;pump_control();assert(!control_jobs && !allocations);
  /* Committed bond loads off-owner, saves before ACK, and is isolated from a
   * later pairing with the same identity but a different bond incarnation. */
  dbt_att_clear(&server);link0->used=true;link0->encrypted=true;link0->generation++;
  assert(dbt_att_open(&server,link0)==DBT_OK);committed_bond=true;test_bond_tag=1;
  sync_cccd();assert(cccd_binding(link0)->phase==CCCD_LOADING && !server.connections[0].bonded);
  execute_one();pump_control();sync_cccd();assert(server.connections[0].bonded);
  write_req(0x28,enabled,2,false);assert(server.connections[0].deferred && !cccd_present[1]);
  assert(dbt_att_tx_peek(&server,1,link0->generation,(dbt_att_view_t[1]){{0}})==DBT_EMPTY);
  execute_one();assert(cccd_present[1]);pump_control();tx(out,&n);assert(out[0]==0x13);
  assert(server.connections[0].cccds[3]==1);
  uint32_t before_ignored=server.ignored_commands;
  write_req(0x28,disabled,2,true);
  assert(server.ignored_commands==before_ignored+1 && server.connections[0].cccds[3]==1 && cccd_saved[1][3]==1);
  dbt_att_close(&server,1,link0->generation);link0->generation++;
  assert(dbt_att_open(&server,link0)==DBT_OK);sync_cccd();execute_one();pump_control();sync_cccd();
  assert(server.connections[0].bonded && server.connections[0].cccds[3]==1);
  /* A stale save may reach storage but cannot ACK or attach to a reused
   * handle; the following load observes the serialized committed state. */
  write_req(0x28,disabled,2,false);execute_one();
  dbt_att_close(&server,1,link0->generation);link0->generation++;
  assert(dbt_att_open(&server,link0)==DBT_OK);sync_cccd();pump_control();
  assert(!server.connections[0].bonded && dbt_att_tx_peek(&server,1,link0->generation,(dbt_att_view_t[1]){{0}})==DBT_EMPTY);
  control_now_ms+=100;sync_cccd();
  execute_one();pump_control();sync_cccd();
  assert(server.connections[0].bonded && !server.connections[0].cccds[3]);
  dbt_att_close(&server,1,link0->generation);link0->generation++;
  assert(dbt_att_open(&server,link0)==DBT_OK);control_now_ms=1000;fail_heap=true;sync_cccd();
  assert(cccd_binding(link0)->phase==CCCD_NONE && cccd_binding(link0)->retry_pending);
  fail_heap=false;control_now_ms=1099;sync_cccd();assert(!uxQueueMessagesWaiting(control_work));
  control_now_ms=1100;sync_cccd();assert(cccd_binding(link0)->phase==CCCD_LOADING);
  execute_one();pump_control();sync_cccd();assert(server.connections[0].bonded);
  dbt_att_close(&server,1,link0->generation);link0->generation++;test_bond_tag=2;
  assert(dbt_att_open(&server,link0)==DBT_OK);sync_cccd();execute_one();pump_control();sync_cccd();
  assert(server.connections[0].bonded && !server.connections[0].cccds[3]);
  cccd_fail=true;write_req(0x28,enabled,2,false);execute_one();pump_control();tx(out,&n);
  assert(out[0]==1 && out[4]==DBT_ATT_UNLIKELY && !server.connections[0].cccds[3]);cccd_fail=false;
  dbt_att_close(&server,1,link0->generation);link0->generation++;test_bond_tag=1;
  assert(dbt_att_open(&server,link0)==DBT_OK);sync_cccd();execute_one();pump_control();sync_cccd();
  assert(server.connections[0].bonded && !server.connections[0].cccds[3]);
  /* A reconnect can disable a previously saved CCCD before encryption.
   * The same-connection disable wins, and is durable before bonded attach. */
  dbt_att_clear(&server);link0->generation++;link0->encrypted=false;committed_bond=false;test_bond_tag=1;
  cccd_saved[1][3]=1;cccd_present[1]=true;assert(dbt_att_open(&server,link0)==DBT_OK);
  write_req(0x28,disabled,2,false);tx(out,&n);assert(out[0]==0x13);
  link0->encrypted=true;committed_bond=true;sync_cccd();execute_one();pump_control();sync_cccd();
  assert(cccd_binding(link0)->phase==CCCD_SAVING && !server.connections[0].bonded);
  execute_one();pump_control();sync_cccd();assert(server.connections[0].bonded && !server.connections[0].cccds[3] && !cccd_saved[1][3]);
  /* Phone subscribes before pairing, then uses the same link for OTA. Both
   * touched subscriptions are merged and committed before ATT is bonded. */
  dbt_att_clear(&server);link0->generation++;link0->encrypted=false;committed_bond=false;test_bond_tag=3;
  assert(dbt_att_open(&server,link0)==DBT_OK);
  write_req(0x25,enabled,2,false);tx(out,&n);assert(out[0]==0x13);
  write_req(0x28,enabled,2,false);tx(out,&n);assert(out[0]==0x13);
  uint8_t early_eq[]={1,0,0,0};write_req(0x22,early_eq,sizeof(early_eq),false);tx(out,&n);
  assert(out[0]==1 && out[4]==DBT_ATT_ENCRYPTION);
  uint32_t early_ignored=server.ignored_commands;
  write_req(0x22,early_eq,sizeof(early_eq),true);assert(server.ignored_commands==early_ignored+1);
  link0->encrypted=true;write_req(0x22,early_eq,sizeof(early_eq),false);tx(out,&n);
  assert(out[0]==1 && out[4]==DBT_ATT_AUTHORIZATION);link0->encrypted=false;
  status_pending_generation[0]=0;ota_enabled=true;link0->encrypted=true;committed_bond=true;
  /* Three already-admitted replies cannot starve restore, and the EQ lane
   * still has its independent fourth admission while the load is queued. */
  for(unsigned i=0;i<3;i++){write_req(0x22,ping,sizeof(ping),true);assert(uxQueueMessagesWaiting(control_work));execute_one();pump_control();}
  assert(control_jobs==3 && !cccd_jobs);sync_cccd();assert(control_jobs==4 && cccd_jobs==1);
  assert(probe_gatt_control_eq_delta(0,1)==DBT_OK && control_jobs==5);
  assert(uxQueueMessagesWaiting(eq_work));execute_one();pump_control();assert(control_jobs==4 && cccd_jobs==1);
  assert(uxQueueMessagesWaiting(control_work));execute_one();pump_control();fail_heap=true;sync_cccd();
  assert(cccd_binding(link0)->phase==CCCD_MERGE && cccd_binding(link0)->retry_pending && !server.connections[0].bonded);
  fail_heap=false;control_now_ms+=100;sync_cccd();
  assert(cccd_binding(link0)->phase==CCCD_SAVING && !server.connections[0].bonded && !cccd_present[3]);
  uint8_t early_begin[]={0x20,9,0,0,0};write_req(0x22,early_begin,sizeof(early_begin),false);tx(out,&n);
  assert(out[0]==1 && out[4]==DBT_ATT_AUTHORIZATION);
  execute_one();pump_control();sync_cccd();
  assert(server.connections[0].bonded && server.connections[0].cccds[2]==1 && server.connections[0].cccds[3]==1 &&
         cccd_present[3] && cccd_saved[3][2]==1 && cccd_saved[3][3]==1 && !cccd_saved[3][0] && !cccd_saved[3][1]);
  for(unsigned i=0;i<30 && (control_jobs || status_pending_generation[0] || server.connections[0].notice_ready);i++){
    pump_control();if(server.connections[0].notice_ready)tx(out,&n);
  }
  assert(!control_jobs && !status_pending_generation[0] && !server.connections[0].notice_ready);
  /* OTA complete is held until selector success; delivery means runtime TX
   * admission of the completion notice, never queue admission. */
  ota_enabled=true;ota_selected=false;uint8_t begin[]={0x20,9,0,0,0};
  write_req(0x22,begin,sizeof(begin),false);execute_one();pump_control();
  tx(out,&n);assert(out[0]==0x13);tx(out,&n);assert(out[3]==0x21);
  uint8_t end[]={0x22};write_req(0x22,end,sizeof(end),false);execute_one();pump_control();
  assert(ota_finish_job && server.connections[0].deferred && !ota_delivered_count &&
         dbt_att_tx_peek(&server,1,link0->generation,(dbt_att_view_t[1]){{0}})==DBT_EMPTY);
  assert(probe_gatt_restart_fenced() && probe_gatt_restart_prepare_quiescent());
  unsigned before_drain=commits;
  assert(probe_gatt_control_eq_delta(0,1)==DBT_BUSY &&
         probe_gatt_control_eq_delta_drain(0,1)==DBT_OK && !probe_gatt_restart_prepare_quiescent());
  execute_one();pump_control();assert(commits==before_drain+1);
  for(unsigned i=0;i<10 && !probe_gatt_restart_prepare_quiescent();i++){
    if(server.connections[0].notice_ready)tx(out,&n);pump_control();
  }
  assert(probe_gatt_restart_prepare_quiescent());
  ota_selected=true;pump_control();assert(!ota_finish_job && !ota_delivered_count);
  tx(out,&n);assert(out[0]==0x13);tx(out,&n);assert(out[3]==0x22 && !ota_delivered_count);
  pump_control();assert(ota_delivered_count==1);
  assert(probe_gatt_restart_fenced() && probe_gatt_restart_quiescent() &&
         probe_gatt_control_eq_delta(0,1)==DBT_BUSY);
  probe_gatt_restart_cancel();assert(!probe_gatt_restart_fenced());
  unsigned before_finish_calls=ota_finish_calls;ota_prepare_error=true;
  write_req(0x22,end,sizeof(end),false);execute_one();pump_control();
  assert(ota_finish_calls==before_finish_calls && !probe_gatt_restart_fenced());
  tx(out,&n);assert(out[0]==1 && out[4]==DBT_ATT_UNLIKELY);
  tx(out,&n);assert(out[3]==0x23 && out[4]==0x12 && ota_delivered_count==1);
  ota_prepare_error=false;
  ota_selected=false;ota_finish_error=true;
  write_req(0x22,end,sizeof(end),false);execute_one();pump_control();
  tx(out,&n);assert(out[0]==1 && out[4]==DBT_ATT_UNLIKELY);
  tx(out,&n);assert(out[3]==0x23 && out[4]==0x12 && ota_delivered_count==1 && ota_abandoned);
  ota_finish_error=false;ota_selected=false;
  write_req(0x22,end,sizeof(end),false);execute_one();pump_control();assert(ota_finish_job);
  ota_valid=false;pump_control();assert(!ota_finish_job && !probe_gatt_restart_fenced());
  tx(out,&n);assert(out[0]==1 && out[4]==DBT_ATT_UNLIKELY && ota_delivered_count==1);
  ota_valid=true;ota_selected=false;
  write_req(0x22,end,sizeof(end),false);execute_one();pump_control();assert(ota_finish_job);
  control_now_ms=40000;pump_control();assert(!ota_finish_job && !probe_gatt_restart_fenced());
  tx(out,&n);assert(out[0]==1 && out[4]==DBT_ATT_UNLIKELY && ota_delivered_count==1);
  control_now_ms=0;ota_selected=true;
  const uint8_t prior_notice[]={0x77};assert(dbt_att_notify(&server,link0,0x24,prior_notice,1,false)==DBT_OK);
  write_req(0x22,end,sizeof(end),false);execute_one();pump_control();
  assert(probe_gatt_restart_fenced() && !probe_gatt_restart_quiescent() && ota_delivered_count==1);
  control_now_ms=30001;pump_control();assert(!probe_gatt_restart_fenced() && ota_delivered_count==1);
  tx(out,&n);assert(out[0]==1 && out[4]==DBT_ATT_UNLIKELY);tx(out,&n);assert(out[3]==0x77);
  control_now_ms=0;ota_selected=false;
  control_worker=false;control_retiring=false;control_retry_pending=false;fail_spawn=true;
  write_req(0x22,begin,sizeof(begin),false);pump_control();
  tx(out,&n);assert(out[0]==1 && out[4]==DBT_ATT_UNLIKELY);
  tx(out,&n);assert(out[3]==0x11 && out[4]==0x20 && ota_delivered_count==1);
  fail_spawn=false;ota_enabled=false;
  /* A failed transition save never marks ATT bonded, admits OTA, or emits a
   * notification using its now-uncommitted pre-pairing subscription. */
  dbt_att_clear(&server);link0->generation++;link0->encrypted=false;committed_bond=false;test_bond_tag=2;
  memset(cccd_saved[2],0,sizeof(cccd_saved[2]));cccd_present[2]=false;
  assert(dbt_att_open(&server,link0)==DBT_OK);
  write_req(0x25,enabled,2,false);tx(out,&n);write_req(0x28,enabled,2,false);tx(out,&n);
  status_pending_generation[0]=0;link0->encrypted=true;committed_bond=true;ota_enabled=true;
  sync_cccd();execute_one();pump_control();sync_cccd();assert(cccd_binding(link0)->phase==CCCD_SAVING);
  cccd_fail=true;execute_one();pump_control();sync_cccd();cccd_fail=false;
  assert(cccd_binding(link0)->phase==CCCD_ERROR && !server.connections[0].bonded && !cccd_present[2]);
  probe_gatt_meter(1,2,3);assert(dbt_att_tx_peek(&server,1,link0->generation,(dbt_att_view_t[1]){{0}})==DBT_EMPTY);
  write_req(0x22,early_begin,sizeof(early_begin),false);tx(out,&n);
  assert(out[0]==1 && out[4]==DBT_ATT_AUTHORIZATION);ota_enabled=false;committed_bond=false;
  /* A load completion from an old generation cannot carry that link's
   * pre-pair subscription into a reused handle. */
  dbt_att_clear(&server);link0->generation++;link0->encrypted=false;
  assert(dbt_att_open(&server,link0)==DBT_OK);write_req(0x25,enabled,2,false);tx(out,&n);
  status_pending_generation[0]=0;link0->encrypted=true;committed_bond=true;
  sync_cccd();assert(cccd_jobs==1);dbt_att_close(&server,1,link0->generation);link0->generation++;
  assert(dbt_att_open(&server,link0)==DBT_OK);sync_cccd();execute_one();pump_control();
  control_now_ms+=100;sync_cccd();execute_one();pump_control();sync_cccd();
  assert(server.connections[0].bonded && !server.connections[0].cccds[2] && !cccd_saved[2][2]);
  committed_bond=false;
  /* Same-generation revoke/regrant invalidates queued provider work before
   * execution, while an admitted EQ still commits but gets no stale ACK. */
  dbt_att_clear(&server);link0->generation=1;assert(dbt_att_open(&server,link0)==DBT_OK);
  control_worker=true;control_retiring=false;control_retry_pending=false;
  uint8_t stale_name[]={4,'S'};unsigned before_provider=provider_calls;
  write_req(0x22,stale_name,sizeof(stale_name),false);fake_cookie++;
  assert(uxQueueMessagesWaiting(control_work));
  execute_one();pump_control();tx(out,&n);
  assert(out[0]==1 && out[4]==DBT_ATT_AUTHORIZATION && provider_calls==before_provider);
  uint8_t stale_eq[]={1,2,3,4};unsigned before_durable=commits;
  write_req(0x22,stale_eq,sizeof(stale_eq),false);fake_cookie++;
  assert(uxQueueMessagesWaiting(eq_work));
  execute_one();pump_control();tx(out,&n);
  assert(commits==before_durable+1 && durable_eq==0x01040302u &&
         out[0]==1 && out[4]==DBT_ATT_AUTHORIZATION);
  uint8_t loaned_eq[]={1,4,3,4};before_durable=commits;
  write_req(0x22,loaned_eq,sizeof(loaned_eq),false);execute_one();pump_control();
  assert(commits==before_durable+1 && server.connections[0].response_ready);
  bearer.bindings[0].active=true;bearer.bindings[0].handle=1;bearer.bindings[0].generation=1;
  fake_cookie++;guard_pending_acks();assert(!server.connections[0].failed);
  tx(out,&n);bearer.bindings[0].active=false;guard_pending_acks();pump_control();
  uint8_t later_eq[]={1,3,3,4};before_durable=commits;
  write_req(0x22,later_eq,sizeof(later_eq),false);execute_one();pump_control();
  assert(commits==before_durable+1 && server.connections[0].response_ready);
  uint32_t old_token=server.connections[0].token,old_ignored=server.ignored_commands;
  uint8_t intervening[]={0x52,0x22,0,0xff};
  probe_gatt_receive(NULL,link0,4,intervening,sizeof(intervening));
  assert(server.connections[0].token==old_token && server.ignored_commands==old_ignored+1);
  fake_cookie++;guard_pending_acks();assert(server.connections[0].failed);
  pump_control();assert(!control_jobs);
  link0->used=false;
  dbt_att_clear(&server);
  vQueueDelete(control_work);
  vQueueDelete(eq_work);
  vQueueDelete(control_done);
  vSemaphoreDelete(control_signal);vSemaphoreDelete(eq_signal);
  control_signal=eq_signal=0;
  control_work = eq_work = control_done = 0;
  return 0;
}
