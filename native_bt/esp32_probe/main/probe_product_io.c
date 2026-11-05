#include "dbt_esp32_affinity.h"
#include "probe_product_io.h"
#include "dbt_input_esp32.h"
#include "dbt_led_esp32.h"
#include "dbt_sound_esp32.h"
#include "esp_spiffs.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "probe_gatt.h"
#include "probe_sound.h"
#include <stdatomic.h>
#include <string.h>

#define QUEUE_CAP 4u
#define LED_WAIT_MS 3000u

typedef struct {
  uint8_t type;
  uint32_t rate;
  uint32_t epoch;
  bool exclusive;
} play_job_t;
typedef struct {
  dbt_ble_control_peer_t peer;
  uint8_t data[3], size;
} notice_t;
typedef struct {
  probe_product_io_hooks_t hooks;
  dbt_led_esp32_t led;
  dbt_input_esp32_t input;
  dbt_sound_esp32_t fs;
  dbt_sound_t sound;
  probe_sound_t player;
  atomic_bool ready, retiring, input_done, sound_busy;
  atomic_bool input_pause,input_pause_ack,input_pause_fault;
  atomic_uint sound_status;
  atomic_uint sound_epoch;
  uint32_t sound_worker_epoch;
  TaskHandle_t input_task;
  atomic_bool led_started, input_started, sound_started;
  probe_product_input_status_t input_status;
  atomic_uint led_sequence, led_words[3];
  atomic_uint led_grant, led_peer_generation, led_peer_handle;
  bool (*led_commit_original)(void *, const uint8_t[10]);
  dbt_led_request_t local_jobs[QUEUE_CAP];
  atomic_uint lj_head, lj_tail;
  dbt_led_receipt_t local_done[QUEUE_CAP];
  atomic_uint ld_head, ld_tail;
  play_job_t plays[QUEUE_CAP];
  atomic_uint play_head, play_tail;
  probe_product_io_play_result_t play_results[QUEUE_CAP];
  atomic_uint pr_head, pr_tail;
  bool play_waiting;
  notice_t notices[QUEUE_CAP];
  atomic_uint notice_head, notice_tail;
  atomic_bool led_waiting;
  bool led_local, ble_done;
  uint32_t sequence, led_internal, led_external;
  dbt_led_receipt_t ble_receipt;
} product_io_t;
static product_io_t io;
static atomic_uint capabilities_word;
static void publish_capabilities(void) {
  unsigned v=(unsigned)io.input_status;
  bool ready=atomic_load_explicit(&io.ready,memory_order_acquire);
  if(ready) {
    v|=1u<<8;
    if(atomic_load_explicit(&io.led_started,memory_order_acquire))v|=1u<<9;
    if(atomic_load_explicit(&io.sound_started,memory_order_acquire))v|=1u<<10;
  }
  atomic_store_explicit(&capabilities_word,v,memory_order_release);
}
bool probe_product_io_capabilities(probe_product_io_capabilities_t *out) {
  if(!out)return false;
  unsigned v=atomic_load_explicit(&capabilities_word,memory_order_acquire);
  out->ready=(v&(1u<<8))!=0;out->led=(v&(1u<<9))!=0;out->sound=(v&(1u<<10))!=0;
  out->input=(probe_product_input_status_t)(v&255u);return true;
}
static uint32_t now_ms(void) {
  return (uint32_t)((uint64_t)esp_timer_get_time() / 1000u);
}
static bool valid_peer(dbt_sound_peer_t p) {
  dbt_ble_control_peer_t peer = {p.handle, p.generation};
  return p.generation && io.hooks.peer_valid(io.hooks.context, &peer);
}
static bool fs_peer(void *ignored, dbt_sound_peer_t p) {
  (void)ignored;
  return valid_peer(p);
}
static void wake_sound(void *ignored) {
  (void)ignored;
  probe_gatt_control_wake();
}
static void led_snapshot_publish(const dbt_led_settings_t *settings) {
  uint8_t bytes[12] = {0};
  memcpy(bytes, settings->bytes, 10);
  atomic_fetch_add_explicit(&io.led_sequence, 1, memory_order_seq_cst);
  for (unsigned i = 0; i < 3; i++) {
    uint32_t w;
    memcpy(&w, bytes + 4 * i, 4);
    atomic_store_explicit(&io.led_words[i], w, memory_order_seq_cst);
  }
  atomic_fetch_add_explicit(&io.led_sequence, 1, memory_order_seq_cst);
}
/* One general worker submits at most one LED job at a time. The unique grant
 * is consumed either by cancellation or by the LED task's irreversible commit
 * admission. Metadata is published before the grant and never read unless the
 * matching grant was acquired. No peer/host state is touched on the LED task.
 */
static bool led_commit_fenced(void *context, const uint8_t blob[10]) {
  uint32_t token = atomic_load_explicit(&io.led_grant, memory_order_acquire);
  if (!token ||
      !atomic_compare_exchange_strong_explicit(
          &io.led_grant, &token, 0, memory_order_acq_rel, memory_order_acquire))
    return false;
  return io.led_commit_original(context, blob);
}
static void led_grant_prepare(uint32_t token,
                              const dbt_ble_control_peer_t *peer) {
  atomic_store_explicit(&io.led_peer_handle, peer ? peer->handle : 0,
                        memory_order_relaxed);
  atomic_store_explicit(&io.led_peer_generation, peer ? peer->generation : 0,
                        memory_order_relaxed);
  atomic_store_explicit(&io.led_grant, token, memory_order_release);
}
static void led_grant_clear(uint32_t token) {
  (void)atomic_compare_exchange_strong_explicit(
      &io.led_grant, &token, 0, memory_order_acq_rel, memory_order_acquire);
}
static void input_tick(void) {
    if (atomic_load_explicit(&io.ready, memory_order_acquire)) {
      if(atomic_load_explicit(&io.input_pause,memory_order_acquire)){
        dbt_input_drain_result_t r=dbt_input_esp32_drain_pending(&io.input);
        if(r==DBT_INPUT_DRAIN_FAULT)atomic_store_explicit(&io.input_pause_fault,true,memory_order_release);
        if(r!=DBT_INPUT_DRAIN_BUSY){
          if(!atomic_exchange_explicit(&io.input_pause_ack,true,memory_order_acq_rel))io.hooks.wake_owner(io.hooks.context);
        }
      }else {
        atomic_store_explicit(&io.input_pause_ack,false,memory_order_release);
        (void)dbt_input_esp32_poll(&io.input);
      }
      if (atomic_load_explicit(&io.input.input.head, memory_order_acquire) !=
          atomic_load_explicit(&io.input.input.tail, memory_order_acquire))
        io.hooks.wake_owner(io.hooks.context);
    }
}
static void input_worker(void *unused) {
  (void)unused;
  while (!atomic_load_explicit(&io.retiring, memory_order_acquire)) {
    input_tick();
    vTaskDelay(pdMS_TO_TICKS(25));
  }
  atomic_store_explicit(&io.input_done, true, memory_order_release);
  vTaskSuspend(NULL);
}
static void close_startup(void) {
  atomic_store(&io.retiring, true);
  if (io.input_task) {
    while (!atomic_load_explicit(&io.input_done, memory_order_acquire))
      vTaskDelay(1);
    vTaskDelete(io.input_task);
    io.input_task = NULL;
  }
  if (io.input_started) {
    if(dbt_input_esp32_stop_checked(&io.input)) {
      io.input_started = false;io.input_status=PROBE_PRODUCT_INPUT_STOPPED;
    } else io.input_status=PROBE_PRODUCT_INPUT_CLEANUP_FAILED;
  }
  if (io.led_started) {
    (void)dbt_led_esp32_stop(&io.led);
    io.led_started = false;
  }
}
/* Inspect every slot before changing any of them. A blocked slot or read error
 * prevents recovery of all slots and preserves the evidence for diagnosis. */
static bool recover_sound_slots(dbt_sound_esp32_t *backend) {
  dbt_sound_recovery_state_t states[4];
  for (uint8_t i = 0; i < 4; i++) {
    dbt_sound_recovery_t r;
    if (dbt_sound_esp32_inspect(backend, i, &r) != DBT_OK ||
        (r.state != DBT_SOUND_RECOVERY_CLEAN &&
         r.state != DBT_SOUND_RECOVERY_NEEDED))
      return false;
    states[i] = r.state;
  }
  for (uint8_t i = 0; i < 4; i++)
    if (states[i] == DBT_SOUND_RECOVERY_NEEDED) {
      dbt_sound_recovery_t r;
      if (dbt_sound_esp32_reconcile(backend, i, &r) != DBT_OK ||
          r.state != DBT_SOUND_RECOVERY_CLEAN)
        return false;
    }
  return true;
}
bool probe_product_io_startup_worker(dbt_ble_control_state_t *initial,
                                     const probe_product_io_hooks_t *hooks) {
  if (!initial || !hooks || !hooks->peer_valid || !hooks->wake_owner ||
      atomic_load_explicit(&io.ready, memory_order_acquire) ||
      atomic_load_explicit(&io.retiring, memory_order_acquire))
    return false;
  memset(&io, 0, sizeof io);
  io.hooks = *hooks;
  io.input_status=hooks->input?PROBE_PRODUCT_INPUT_NOT_ATTEMPTED:PROBE_PRODUCT_INPUT_NOT_REQUESTED;
  atomic_init(&io.ready, false);
  atomic_init(&io.retiring, false);
  atomic_init(&io.input_done, false);
  atomic_init(&io.input_pause,false);
  atomic_init(&io.input_pause_ack,false);
  atomic_init(&io.input_pause_fault,false);
  atomic_init(&io.led_waiting, false);
  atomic_init(&io.sound_busy, false);
  atomic_init(&io.sound_status, 0);
  atomic_init(&io.sound_epoch, 1);
  io.sound_worker_epoch = 1;
  atomic_init(&io.led_started, false);
  atomic_init(&io.input_started, false);
  atomic_init(&io.sound_started, false);
  atomic_init(&io.led_sequence, 0);
  atomic_init(&io.led_grant, 0);
  atomic_init(&io.led_peer_generation, 0);
  atomic_init(&io.led_peer_handle, 0);
  for (unsigned i = 0; i < 3; i++)
    atomic_init(&io.led_words[i], 0);
  atomic_init(&io.lj_head, 0);
  atomic_init(&io.lj_tail, 0);
  atomic_init(&io.ld_head, 0);
  atomic_init(&io.ld_tail, 0);
  atomic_init(&io.play_head, 0);
  atomic_init(&io.play_tail, 0);
  atomic_init(&io.pr_head, 0);
  atomic_init(&io.pr_tail, 0);
  atomic_init(&io.notice_head, 0);
  atomic_init(&io.notice_tail, 0);
  publish_capabilities();
  dbt_sound_store_t sound_store;
  bool muted = false;
  if (hooks->sound) {
    if (!esp_spiffs_mounted("spiffs") ||
        dbt_sound_esp32_init(&io.fs, NULL, fs_peer, &sound_store, &muted) !=
            DBT_OK ||
        !recover_sound_slots(&io.fs) ||
        dbt_sound_init(&io.sound, sound_store, muted) != DBT_OK)
      return false;
  }
  if (hooks->led) {
    if (!dbt_led_esp32_start(&io.led, SPI2_HOST, 4, 16, 16))
      return false;
    io.led_commit_original = io.led.led.io.commit;
    io.led.led.io.commit = led_commit_fenced;
    io.led_started = true;
    led_snapshot_publish(&io.led.led.settings);
  }
  if (hooks->input) {
    /* Only a clean NACK is eligible for an alternate probe or optional mode. */
    dbt_input_start_result_t input=dbt_input_esp32_start_checked(&io.input,I2C_NUM_1,23,22,0x36,18,21,19);
    if(input==DBT_INPUT_START_NO_ACK)
      input=dbt_input_esp32_start_checked(&io.input,I2C_NUM_1,23,22,0x07,18,21,19);
    if(input==DBT_INPUT_START_NO_ACK && hooks->input_optional)
      io.input_status=PROBE_PRODUCT_INPUT_NO_ACK;
    else if(input!=DBT_INPUT_START_READY) {
      io.input_status=input==DBT_INPUT_START_CLEANUP_FAILED?PROBE_PRODUCT_INPUT_CLEANUP_FAILED:
                      input==DBT_INPUT_START_NO_ACK?PROBE_PRODUCT_INPUT_NO_ACK:PROBE_PRODUCT_INPUT_FAILED;
      close_startup();
      publish_capabilities();
      return false;
    }
    if(input==DBT_INPUT_START_READY) {
      io.input_started = true;io.input_status=PROBE_PRODUCT_INPUT_AVAILABLE;
      if (xTaskCreatePinnedToCore(input_worker, "dbt_inputs", 3072, NULL, 2,
                      &io.input_task, DBT_ESP32_BACKGROUND_CORE) != pdPASS) {
        close_startup();publish_capabilities();
        return false;
      }
    }
  }
  if (hooks->sound) {
    if (probe_sound_init_with_wake(&io.player, &io.sound, wake_sound, NULL) !=
        DBT_OK) {
      close_startup();
      publish_capabilities();
      return false;
    }
    io.sound_started = true;
    atomic_store(&io.sound_status, dbt_sound_status(&io.sound));
  }
  /* Publish the caller's combined initial state only after every enabled
   * provider succeeded. A failed start leaves the supplied snapshot intact. */
  if (io.led_started)
    memcpy(initial->led, io.led.led.settings.bytes, 10);
  if (io.sound_started)
    initial->sound =
        (uint8_t)atomic_load_explicit(&io.sound_status, memory_order_relaxed);
  atomic_store_explicit(&io.ready, true, memory_order_release);
  publish_capabilities();
  return true;
}
static void sound_notice(dbt_sound_peer_t p) {
  unsigned h = atomic_load_explicit(&io.notice_head, memory_order_relaxed),
           t = atomic_load_explicit(&io.notice_tail, memory_order_acquire);
  if (h - t >= QUEUE_CAP)
    return;
  notice_t *n = &io.notices[h % QUEUE_CAP];
  n->peer = (dbt_ble_control_peer_t){p.handle, p.generation};
  n->data[0] = 0x33;
  n->data[1] = 0x0b;
  n->size = 2;
  atomic_store_explicit(&io.notice_head, h + 1, memory_order_release);
  io.hooks.wake_owner(io.hooks.context);
}
static void sound_tick(void) {
  if (!io.sound_started)
    return;
  dbt_sound_peer_t expired = {0};
  dbt_result_t r = probe_sound_poll(&io.player, now_ms(), &expired);
  if (r == DBT_EMPTY && expired.generation && valid_peer(expired))
    sound_notice(expired);
  bool busy = probe_sound_busy(&io.player);
  bool was_busy =
      atomic_exchange_explicit(&io.sound_busy, busy, memory_order_acq_rel);
  if (was_busy != busy)
    io.hooks.wake_owner(io.hooks.context);
  atomic_store_explicit(&io.sound_status, dbt_sound_status(&io.sound),
                        memory_order_release);
}
static void led_tick(void) {
  if (!io.led_started || !io.led_waiting)
    return;
  dbt_led_receipt_t r;
  if (!dbt_led_esp32_receipt(&io.led, &r))
    return;
  if (r.token != io.led_internal)
    return; /* Never convert an uncorrelated receipt into success. */
  led_grant_clear(r.token);
  io.led_waiting = false;
  io.hooks.wake_owner(io.hooks.context);
  if (r.committed || r.result == DBT_LED_OK) {
    led_snapshot_publish(&r.settings);
    io.hooks.wake_owner(io.hooks.context);
  }
  if (io.led_local) {
    unsigned h = atomic_load_explicit(&io.ld_head, memory_order_relaxed);
    r.token = io.led_external;
    io.local_done[h % QUEUE_CAP] = r;
    atomic_store_explicit(&io.ld_head, h + 1, memory_order_release);
    io.hooks.wake_owner(io.hooks.context);
  } else {
    io.ble_receipt = r;
    io.ble_done = true;
  }
}
static void local_tick(void) {
  if (!io.led_started || io.led_waiting ||
      atomic_load_explicit(&io.sound_busy, memory_order_acquire))
    return;
  unsigned rt = atomic_load_explicit(&io.lj_tail, memory_order_relaxed),
           rh = atomic_load_explicit(&io.lj_head, memory_order_acquire);
  unsigned h = atomic_load_explicit(&io.ld_head, memory_order_relaxed),
           t = atomic_load_explicit(&io.ld_tail, memory_order_acquire);
  if (rt == rh || h - t >= QUEUE_CAP)
    return;
  dbt_led_request_t req = io.local_jobs[rt % QUEUE_CAP];
  uint32_t external = req.token;
  if (io.sequence == UINT32_MAX) {
    dbt_led_receipt_t done = {0};
    done.token = external;
    done.result = DBT_LED_INVALID;
    io.local_done[h % QUEUE_CAP] = done;
    atomic_store_explicit(&io.lj_tail, rt + 1, memory_order_release);
    atomic_store_explicit(&io.ld_head, h + 1, memory_order_release);
    io.hooks.wake_owner(io.hooks.context);
    return;
  }
  req.token = ++io.sequence;
  led_grant_prepare(req.token, NULL);
  dbt_led_result_t r = dbt_led_esp32_submit(&io.led, &req);
  if (r != DBT_LED_OK)
    led_grant_clear(req.token);
  if (r == DBT_LED_BUSY)
    return;
  if (r != DBT_LED_OK) {
    atomic_store_explicit(&io.lj_tail, rt + 1, memory_order_release);
    dbt_led_receipt_t done = {0};
    done.token = external;
    done.result = r;
    io.local_done[h % QUEUE_CAP] = done;
    atomic_store_explicit(&io.ld_head, h + 1, memory_order_release);
    io.hooks.wake_owner(io.hooks.context);
    return;
  }
  io.led_local = true;
  io.led_internal = req.token;
  io.led_external = external;
  atomic_store_explicit(&io.led_waiting, true, memory_order_release);
  atomic_store_explicit(&io.lj_tail, rt + 1, memory_order_release);
}
static void play_tick(bool retiring) {
  if (!io.sound_started)
    return;
  unsigned t = atomic_load_explicit(&io.play_tail, memory_order_relaxed),
           h = atomic_load_explicit(&io.play_head, memory_order_acquire);
  unsigned rh = atomic_load_explicit(&io.pr_head, memory_order_relaxed),
           rt = atomic_load_explicit(&io.pr_tail, memory_order_acquire);
  if (t == h || rh - rt >= QUEUE_CAP)
    return;
  play_job_t job = io.plays[t % QUEUE_CAP];
  dbt_result_t result;
  if(io.play_waiting){
    if(retiring || job.epoch!=atomic_load_explicit(&io.sound_epoch,memory_order_acquire))probe_sound_stop(&io.player);
    if(probe_sound_start_pending(&io.player))return;
    result=io.player.last_result;io.play_waiting=false;
  }else if(retiring)result=DBT_EMPTY;
  else {
    if(io.led_waiting || atomic_load_explicit(&io.sound_busy,memory_order_acquire) || io.sound.active)return;
    atomic_store_explicit(&io.sound_busy,true,memory_order_release);
    result=job.epoch==atomic_load_explicit(&io.sound_epoch,memory_order_acquire)
        ?probe_sound_play(&io.player,job.type,job.rate,job.exclusive):DBT_EMPTY;
    if(result==DBT_PENDING){io.play_waiting=true;return;}
  }
  if(retiring || job.epoch!=atomic_load_explicit(&io.sound_epoch,memory_order_acquire)){
    probe_sound_stop(&io.player);
    result=DBT_EMPTY;
  }
  if (!probe_sound_busy(&io.player))
    atomic_store_explicit(&io.sound_busy, false, memory_order_release);
  io.play_results[rh % QUEUE_CAP] =
      (probe_product_io_play_result_t){job.type, result};
  atomic_store_explicit(&io.play_tail, t + 1, memory_order_release);
  atomic_store_explicit(&io.pr_head, rh + 1, memory_order_release);
  io.hooks.wake_owner(io.hooks.context);
}
bool probe_product_io_command(void *unused, const dbt_ble_control_peer_t *peer,
                              uint8_t cmd, const uint8_t *data, uint16_t n,
                              uint8_t *out, uint8_t *size) {
  (void)unused;
  if (!atomic_load_explicit(&io.ready, memory_order_acquire) ||
      atomic_load_explicit(&io.retiring, memory_order_acquire) || !peer ||
      !peer->generation || (!data && n) || !out || !size ||
      !io.hooks.peer_valid(io.hooks.context, peer))
    return false;
  sound_tick();
  led_tick();
  if (cmd >= 5 && cmd <= 7) {
    if (!io.led_started || io.led_waiting ||
        atomic_load_explicit(&io.sound_busy, memory_order_acquire) ||
        *size < 2 || io.sequence == UINT32_MAX)
      return false;
    dbt_led_request_t r = {0};
    r.token = ++io.sequence;
    r.operation = cmd == 5   ? DBT_LED_SET_ALL
                  : cmd == 6 ? DBT_LED_SET_EFFECT
                             : DBT_LED_SET_BRIGHTNESS;
    if ((cmd == 5 && n < 10) || (cmd != 5 && n < 1))
      return false;
    if (cmd == 5)
      memcpy(r.settings.bytes, data, 10);
    else
      r.settings.bytes[cmd == 6 ? 0 : 1] = data[0];
    led_grant_prepare(r.token, peer);
    /* Recheck after publication: a loss just before publication must not be
     * missed, and a later loss can atomically cancel the unique grant. */
    if (!io.hooks.peer_valid(io.hooks.context, peer)) {
      led_grant_clear(r.token);
      return false;
    }
    if (dbt_led_esp32_submit(&io.led, &r) != DBT_LED_OK) {
      led_grant_clear(r.token);
      return false;
    }
    io.led_waiting = true;
    io.led_local = false;
    io.led_internal = r.token;
    io.ble_done = false;
    uint32_t start = now_ms();
    while (io.led_waiting && (uint32_t)(now_ms() - start) < LED_WAIT_MS) {
      led_tick();
      if (io.led_waiting)
        vTaskDelay(1);
    }
    if (!io.ble_done || io.ble_receipt.result != DBT_LED_OK ||
        !io.ble_receipt.committed ||
        !io.hooks.peer_valid(io.hooks.context, peer))
      return false;
    out[0] = 0x10;
    out[1] = cmd;
    *size = 2;
    return true;
  }
  if (cmd >= 0x10 && cmd <= 0x14) {
    if (!io.sound_started ||
        atomic_load_explicit(&io.sound_busy, memory_order_acquire))
      return false;
    unsigned h = atomic_load_explicit(&io.notice_head, memory_order_relaxed),
             t = atomic_load_explicit(&io.notice_tail, memory_order_acquire);
    if (h - t >= QUEUE_CAP)
      return false;
    dbt_sound_response_t r;
    dbt_sound_peer_t p = {peer->handle, peer->generation};
    if (dbt_sound_command(&io.sound, p, cmd, data, n, now_ms(), &r) != DBT_OK ||
        !r.size || *size < r.size ||
        !io.hooks.peer_valid(io.hooks.context, peer))
      return false;
    memcpy(out, r.data, r.size);
    *size = r.size;
    atomic_store_explicit(&io.sound_status, dbt_sound_status(&io.sound),
                          memory_order_release);
    io.hooks.wake_owner(io.hooks.context);
    return true;
  }
  return false;
}
void probe_product_io_peer_lost(void *unused, const dbt_ble_control_peer_t *p) {
  (void)unused;
  if (!p || !p->generation ||
      !atomic_load_explicit(&io.ready, memory_order_acquire))
    return;
  uint32_t token = atomic_load_explicit(&io.led_grant, memory_order_acquire);
  if (token &&
      atomic_load_explicit(&io.led_peer_generation, memory_order_relaxed) ==
          p->generation &&
      atomic_load_explicit(&io.led_peer_handle, memory_order_relaxed) ==
          p->handle)
    led_grant_clear(token);
  if (io.sound_started) {
    dbt_sound_peer_t peer = {p->handle, p->generation};
    (void)probe_sound_disconnect(&io.player, peer);
  }
  probe_gatt_control_wake();
}
bool probe_product_io_worker_poll(void *unused, bool retiring) {
  (void)unused;
  if (!atomic_load_explicit(&io.ready, memory_order_acquire))
    return false;
  uint32_t epoch=atomic_load_explicit(&io.sound_epoch,memory_order_acquire);
  if(io.sound_worker_epoch!=epoch){io.sound_worker_epoch=epoch;probe_sound_stop(&io.player);}
  if (retiring && io.sound_started) {
    probe_sound_stop(&io.player);
    if (io.sound.active)
      (void)dbt_sound_disconnect(&io.sound, io.sound.peer);
  }
  sound_tick();
  led_tick();
  local_tick();
  play_tick(retiring);
  if (!retiring)
    return true;
  /* Worker lifetime is shorter than the provider lifetime. No destructors or
   * manager reinitialization on ordinary BLE-worker retirement. */
  if (io.sound_started &&
      (probe_sound_busy(&io.player) || io.play_waiting || io.sound.active))
    return false;
  if (atomic_load_explicit(&io.lj_head, memory_order_acquire) !=
          atomic_load_explicit(&io.lj_tail, memory_order_acquire) ||
      atomic_load_explicit(&io.led_waiting, memory_order_acquire))
    return false;
  return true;
}
bool probe_product_io_take_input(dbt_input_action_t *out) {
  return out && atomic_load_explicit(&io.ready, memory_order_acquire) &&
         io.input_started && dbt_input_next(&io.input.input, out);
}
dbt_result_t probe_product_io_input_pause(bool pause){
  if(!atomic_load_explicit(&io.ready,memory_order_acquire))return DBT_INVALID;
  if(pause){
    if(!atomic_load_explicit(&io.input_pause,memory_order_acquire)){
      atomic_store_explicit(&io.input_pause_fault,false,memory_order_relaxed);
      atomic_store_explicit(&io.input_pause_ack,!io.input_started,memory_order_relaxed);
      atomic_store_explicit(&io.input_pause,true,memory_order_release);
    }
  }else {
    atomic_store_explicit(&io.input_pause,false,memory_order_release);
    atomic_store_explicit(&io.input_pause_ack,false,memory_order_release);
  }
  return DBT_OK;
}
dbt_result_t probe_product_io_input_paused(void){
  if(!atomic_load_explicit(&io.ready,memory_order_acquire) || !atomic_load_explicit(&io.input_pause,memory_order_acquire))return DBT_INVALID;
  if(atomic_load_explicit(&io.input_pause_fault,memory_order_acquire))return DBT_INVALID;
  return atomic_load_explicit(&io.input_pause_ack,memory_order_acquire)?DBT_OK:DBT_BUSY;
}
bool probe_product_io_input_drained(void){
  return probe_product_io_input_paused()==DBT_OK && (!io.input_started ||
    atomic_load_explicit(&io.input.input.head,memory_order_acquire)==atomic_load_explicit(&io.input.input.tail,memory_order_acquire));
}
static dbt_result_t enqueue_local(const dbt_led_request_t *r) {
  if (!atomic_load_explicit(&io.ready, memory_order_acquire) ||
      atomic_load_explicit(&io.retiring, memory_order_acquire) ||
      !io.led_started || !r->token)
    return DBT_INVALID;
  unsigned h = atomic_load_explicit(&io.lj_head, memory_order_relaxed),
           t = atomic_load_explicit(&io.lj_tail, memory_order_acquire);
  if (h - t >= QUEUE_CAP)
    return DBT_BUSY;
  io.local_jobs[h % QUEUE_CAP] = *r;
  atomic_store_explicit(&io.lj_head, h + 1, memory_order_release);
  probe_gatt_control_wake();
  io.hooks.wake_owner(io.hooks.context);
  return DBT_OK;
}
dbt_result_t probe_product_io_local_led(dbt_led_operation_t op, uint8_t value,
                                        uint32_t token) {
  if (op != DBT_LED_SET_EFFECT && op != DBT_LED_SET_BRIGHTNESS &&
      op != DBT_LED_PREVIEW_EFFECT)
    return DBT_INVALID;
  if ((op == DBT_LED_SET_BRIGHTNESS && value > 100) ||
      (op != DBT_LED_SET_BRIGHTNESS && !dbt_led_effect_supported(value)))
    return DBT_INVALID;
  dbt_led_request_t r = {0};
  r.token = token;
  r.operation = op;
  r.settings.bytes[op == DBT_LED_SET_BRIGHTNESS ? 1 : 0] = value;
  return enqueue_local(&r);
}
dbt_result_t probe_product_io_local_led_relative(dbt_led_operation_t op,
                                                 int8_t delta, uint32_t token) {
  if ((op == DBT_LED_ADJUST_BRIGHTNESS && delta != 5 && delta != -5) ||
      ((op == DBT_LED_ADJUST_EFFECT || op == DBT_LED_PREVIEW_DELTA) &&
       delta != 1 && delta != -1) ||
      (op == DBT_LED_CONFIRM_PREVIEW && delta != 0) ||
      (op != DBT_LED_ADJUST_BRIGHTNESS && op != DBT_LED_ADJUST_EFFECT &&
       op != DBT_LED_PREVIEW_DELTA && op != DBT_LED_CONFIRM_PREVIEW))
    return DBT_INVALID;
  dbt_led_request_t r = {0};
  r.token = token;
  r.operation = op;
  r.delta = delta;
  return enqueue_local(&r);
}
bool probe_product_io_take_local_led(dbt_led_receipt_t *r) {
  if (!r)
    return false;
  unsigned t = atomic_load_explicit(&io.ld_tail, memory_order_relaxed),
           h = atomic_load_explicit(&io.ld_head, memory_order_acquire);
  if (t == h)
    return false;
  *r = io.local_done[t % QUEUE_CAP];
  atomic_store_explicit(&io.ld_tail, t + 1, memory_order_release);
  return true;
}
dbt_result_t probe_product_io_play(uint8_t type, uint32_t rate,
                                   bool exclusive) {
  if (!atomic_load_explicit(&io.ready, memory_order_acquire) ||
      atomic_load_explicit(&io.retiring, memory_order_acquire) ||
      !io.sound_started || type > 3 || rate < 8000 || rate > 192000)
    return DBT_INVALID;
  unsigned h = atomic_load_explicit(&io.play_head, memory_order_relaxed),
           t = atomic_load_explicit(&io.play_tail, memory_order_acquire);
  if (h - t >= QUEUE_CAP)
    return DBT_BUSY;
  uint32_t epoch=atomic_load_explicit(&io.sound_epoch,memory_order_acquire);
  if(!epoch)return DBT_INVALID; /* exhausted epochs stay fail-closed */
  io.plays[h % QUEUE_CAP] = (play_job_t){type, rate, epoch, exclusive};
  atomic_store_explicit(&io.play_head, h + 1, memory_order_release);
  probe_gatt_control_wake();
  io.hooks.wake_owner(io.hooks.context);
  return DBT_OK;
}
void probe_product_io_cancel_sound(void) {
  if (!atomic_load_explicit(&io.ready,memory_order_acquire)) return;
  uint32_t epoch=atomic_load_explicit(&io.sound_epoch,memory_order_relaxed);
  atomic_store_explicit(&io.sound_epoch,epoch==UINT32_MAX?0:epoch?epoch+1u:0,memory_order_release);
  probe_gatt_control_wake();
}
void probe_product_io_pairing_mode(bool active) {
  if (atomic_load_explicit(&io.ready,memory_order_acquire) && io.led_started)
    dbt_led_pairing_mode(&io.led.led,active);
}
void probe_product_io_pairing_success(void) {
  if (atomic_load_explicit(&io.ready,memory_order_acquire) && io.led_started)
    dbt_led_pairing_success(&io.led.led);
}
void probe_product_io_volume_feedback(uint8_t volume) {
  if (atomic_load_explicit(&io.ready,memory_order_acquire) && io.led_started && volume<=127)
    dbt_led_volume(&io.led.led,volume);
}
bool probe_product_io_sound_busy(void) {
  return atomic_load_explicit(&io.sound_busy, memory_order_acquire);
}
uint8_t probe_product_io_sound_status(void) {
  return (uint8_t)atomic_load_explicit(&io.sound_status, memory_order_acquire);
}
bool probe_product_io_take_notice(dbt_ble_control_peer_t *peer, uint8_t out[3],
                                  uint8_t *size) {
  if (!peer || !out || !size)
    return false;
  unsigned t = atomic_load_explicit(&io.notice_tail, memory_order_relaxed),
           h = atomic_load_explicit(&io.notice_head, memory_order_acquire);
  if (t == h)
    return false;
  notice_t n = io.notices[t % QUEUE_CAP];
  *peer = n.peer;
  memcpy(out, n.data, n.size);
  *size = n.size;
  atomic_store_explicit(&io.notice_tail, t + 1, memory_order_release);
  return true;
}
void probe_product_io_meter(uint8_t b, uint8_t m, uint8_t t, bool active) {
  if (!atomic_load_explicit(&io.ready, memory_order_acquire) || !io.led_started)
    return;
  dbt_led_meter(&io.led.led, b, m, t);
  dbt_led_audio_active(&io.led.led, active);
}

bool probe_product_io_led_snapshot(dbt_led_settings_t *out) {
  if (!out || !atomic_load_explicit(&io.ready, memory_order_acquire) ||
      !atomic_load_explicit(&io.led_started, memory_order_acquire))
    return false;
  for (unsigned attempt = 0; attempt < 3; attempt++) {
    unsigned a = atomic_load_explicit(&io.led_sequence, memory_order_seq_cst);
    if (a & 1u)
      continue;
    uint8_t bytes[12];
    for (unsigned i = 0; i < 3; i++) {
      uint32_t w = atomic_load_explicit(&io.led_words[i], memory_order_seq_cst);
      memcpy(bytes + 4 * i, &w, 4);
    }
    unsigned b = atomic_load_explicit(&io.led_sequence, memory_order_seq_cst);
    if (a == b) {
      memcpy(out->bytes, bytes, 10);
      return true;
    }
  }
  return false;
}

bool probe_product_io_needs_worker(void) {
  if (!atomic_load_explicit(&io.ready, memory_order_acquire))
    return false;
  /* Check in-flight states again after queue indices: a worker may move a job
   * from its queue to a loan between the first state read and the queue read.
   */
  if (atomic_load_explicit(&io.sound_busy, memory_order_acquire) ||
      atomic_load_explicit(&io.led_waiting, memory_order_acquire))
    return true;
  if (atomic_load_explicit(&io.lj_head, memory_order_acquire) !=
          atomic_load_explicit(&io.lj_tail, memory_order_acquire) ||
      atomic_load_explicit(&io.play_head, memory_order_acquire) !=
          atomic_load_explicit(&io.play_tail, memory_order_acquire))
    return true;
  return atomic_load_explicit(&io.sound_busy, memory_order_acquire) ||
         atomic_load_explicit(&io.led_waiting, memory_order_acquire);
}
bool probe_product_io_take_play_result(probe_product_io_play_result_t *out) {
  if (!out)
    return false;
  unsigned t = atomic_load_explicit(&io.pr_tail, memory_order_relaxed),
           h = atomic_load_explicit(&io.pr_head, memory_order_acquire);
  if (t == h)
    return false;
  *out = io.play_results[t % QUEUE_CAP];
  atomic_store_explicit(&io.pr_tail, t + 1, memory_order_release);
  return true;
}

bool probe_product_io_input_stats(probe_product_io_input_stats_t *out) {
  if (!out || !atomic_load_explicit(&io.ready, memory_order_acquire) ||
      !atomic_load_explicit(&io.input_started, memory_order_acquire))
    return false;
  out->read_errors =
      atomic_load_explicit(&io.input.read_errors, memory_order_relaxed);
  out->deferred_samples =
      atomic_load_explicit(&io.input.deferred_samples, memory_order_relaxed);
  out->feedback_errors =
      atomic_load_explicit(&io.input.feedback_errors, memory_order_relaxed);
  out->invalid_channels =
      atomic_load_explicit(&io.input.invalid_channels, memory_order_relaxed);
  return true;
}
