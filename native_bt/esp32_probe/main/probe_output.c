#include "probe_output.h"
#include "dbt_async_ticket.h"
#include "driver/i2s_std.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#ifdef DBT_PROBE_SOUND_BUILD
#include "dbt_sound.h"
#endif
#ifdef DBT_OUTPUT_TEST_HOOKS
extern void dbt_output_test_step(unsigned);
#define OUTPUT_TEST_STEP(x) dbt_output_test_step(x)
#else
#define OUTPUT_TEST_STEP(x) ((void)0)
#endif

#if !CONFIG_I2S_ISR_IRAM_SAFE
#error "Native output requires an IRAM-safe I2S ISR for immediate NVS commits"
#endif
#define DMA_FRAMES 128u
#define DMA_DESCRIPTORS 3u
static TickType_t event_wait_ticks(uint32_t ms) {
  TickType_t ticks = pdMS_TO_TICKS(ms);
  return ticks ? ticks : 1;
}
typedef struct {
  bool start;
  uint8_t kind;
  bool legacy;
  uint32_t rate;
  uint8_t channels;
  dbt_pcm_format_t format;
  uint32_t id;
#ifdef DBT_PROBE_SOUND_BUILD
  uint32_t sound_token;
  dbt_sound_play_t *sound;
  bool exclusive;
#endif
} request_t;
typedef struct {
  uint32_t id;
  dbt_result_t result;
} response_t;
typedef struct {
  void *address;
  uint32_t valid;
  bool silent;
} descriptor_t;
typedef struct {
  dbt_audio_t audio;
  dbt_audio_fade_t fade;
  descriptor_t descriptors[DMA_DESCRIPTORS];
  i2s_chan_handle_t channel;
  TaskHandle_t task;
  QueueHandle_t requests;
  void (*wake_decoder)(void *);
  void *wake_context;
  bool active, enabled, stopping;
  uint32_t descriptor_count, next_descriptor, zero_completions, last_eof,
      clock_at;
  uint32_t stop_request, stop_at;
  dbt_async_ticket_t ticket;
  _Atomic uint32_t request_fault,stale_receipts;
  _Atomic bool restart_hold;
  request_t inflight;
#ifdef DBT_PROBE_SOUND_BUILD
  _Atomic uint32_t sound_token;
  _Atomic uint32_t sound_retired,active_rate;
  uint32_t next_sound_token;
  dbt_sound_play_t *sound;
  _Atomic(void (*)(void *)) wake_sound;
  _Atomic(void *) sound_context;
  bool sound_standalone,transition_sound,sound_silence;
  request_t pending_sound_transition;
  bool restart_after_stop;
#endif
  int32_t held_left, held_right, meter_db[3];
  uint32_t meter_at;
  _Atomic uint32_t meter_value;
  _Atomic uint32_t stop_requested, stop_finished;
  _Atomic uint32_t running, failed, rate, requested_bytes, accepted_frames,
      rendered_frames, ready_frames, input_ready_frames;
  _Atomic uint32_t dma_eofs, dma_source_frames, underrun_frames,
      descriptor_errors, max_eof_gap_us;
  _Atomic uint32_t maximum_render_us, stop_fill_frames;
  _Atomic uint32_t render_total_us,render_calls;
  _Atomic uint32_t allocation_failed_bytes,allocation_failure_free,allocation_failure_largest;
  _Atomic uint32_t first_underrun_accepted_frames, first_underrun_dma_frames;
} output_t;
static DRAM_ATTR output_t output;
#ifdef DBT_PROBE_SOUND_BUILD
static void notify_sound(void){
  void (*wake)(void *)=atomic_load_explicit(&output.wake_sound,memory_order_acquire);
  if(wake)wake(atomic_load_explicit(&output.sound_context,memory_order_relaxed));
}
#endif
static uint8_t *allocate(void *context, uint32_t bytes) {
  (void)context;
  uint8_t *p=heap_caps_malloc(bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if(!p){
    atomic_store(&output.allocation_failed_bytes,bytes);
#ifdef ESP_PLATFORM
    atomic_store(&output.allocation_failure_free,heap_caps_get_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT));
    atomic_store(&output.allocation_failure_largest,heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT));
#endif
  }
  return p;
}
static void release(void *context, uint8_t *bytes) {
  (void)context;
  heap_caps_free(bytes);
}
static uint32_t power2(uint32_t n) {
  uint32_t result = 2;
  while (result < n)
    result <<= 1;
  return result;
}
static void reclaim_cancelled(const request_t *,dbt_result_t);
static void notify_clients(void){
  if(output.wake_decoder)output.wake_decoder(output.wake_context);
#ifdef DBT_PROBE_SOUND_BUILD
  notify_sound();
#endif
}
static void finish_cancel(uint32_t id,dbt_result_t result){
  bool legacy=output.inflight.legacy;
  reclaim_cancelled(&output.inflight,result);
  if(legacy)atomic_store_explicit(&output.request_fault,0,memory_order_release);
  if(!dbt_ticket_reclaimed(&output.ticket,id,legacy))atomic_fetch_add(&output.stale_receipts,1);
  notify_clients();
}
static void report_done(uint32_t id, dbt_result_t result) {
  if(dbt_ticket_state(&output.ticket,id)==DBT_TICKET_STALE){atomic_fetch_add(&output.stale_receipts,1);return;}
  if(dbt_ticket_done_begin(&output.ticket,id)){
#ifdef DBT_PROBE_SOUND_BUILD
    if(output.inflight.kind==2)OUTPUT_TEST_STEP(4);
#endif
    if(dbt_ticket_done_end(&output.ticket,id,result)){notify_clients();return;}
  }
  if(dbt_ticket_state(&output.ticket,id)==DBT_TICKET_CANCEL)finish_cancel(id,result);
  else atomic_fetch_add(&output.stale_receipts,1);
}
#ifdef DBT_OUTPUT_TEST_HOOKS
void dbt_output_test_bad_receipt(uint32_t id){report_done(id,DBT_OK);}
#endif

static bool IRAM_ATTR sent(i2s_chan_handle_t channel, i2s_event_data_t *event,
                           void *context) {
  (void)channel;
  output_t *o = context;
  BaseType_t wake = pdFALSE;
  if (!event->dma_buf || event->size != DMA_FRAMES * 8u) {
    atomic_store(&o->descriptor_errors,
                 atomic_load(&o->descriptor_errors) + 1u);
    atomic_store(&o->failed, 1);
    vTaskNotifyGiveFromISR(o->task,&wake);
    return wake==pdTRUE;
  }
  uint32_t now = (uint32_t)esp_timer_get_time();
  if (o->last_eof) {
    uint32_t gap = now - o->last_eof;
    if (gap > atomic_load(&o->max_eof_gap_us))
      atomic_store(&o->max_eof_gap_us, gap);
  }
  o->last_eof = now;
  unsigned index = 0;
  while (index < o->descriptor_count &&
         o->descriptors[index].address != event->dma_buf)
    ++index;
  if (index == o->descriptor_count) {
    if (index == DMA_DESCRIPTORS) {
      atomic_store(&o->descriptor_errors,
                   atomic_load(&o->descriptor_errors) + 1u);
      atomic_store(&o->failed, 1);
      vTaskNotifyGiveFromISR(o->task,&wake);
      return wake==pdTRUE;
    }
    o->descriptors[index] = (descriptor_t){event->dma_buf, DMA_FRAMES, false};
    ++o->descriptor_count;
  }
  if (index != o->next_descriptor) {
    atomic_store(&o->descriptor_errors,
                 atomic_load(&o->descriptor_errors) + 1u);
    atomic_store(&o->failed, 1);
  }
  o->next_descriptor = index + 1u;
  if (o->next_descriptor == DMA_DESCRIPTORS)
    o->next_descriptor = 0;
  descriptor_t *descriptor = &o->descriptors[index];
  atomic_store(&o->dma_eofs, atomic_load(&o->dma_eofs) + 1u);
  atomic_store(&o->dma_source_frames,
               atomic_load(&o->dma_source_frames) + descriptor->valid);
  bool stopping =
      atomic_load_explicit(&o->stop_requested, memory_order_acquire) != 0;
  if (stopping && descriptor->silent)
    ++o->zero_completions;
  else
    o->zero_completions = 0;
  if (stopping && o->zero_completions >= DMA_DESCRIPTORS)
    atomic_store_explicit(&o->stop_finished, 1, memory_order_release);
  int32_t *buffer = event->dma_buf;
  uint32_t wanted = stopping && o->fade.remaining < DMA_FRAMES
                        ? o->fade.remaining
                        : DMA_FRAMES;
  uint32_t taken = wanted ? dbt_audio_take(&o->audio, buffer, wanted) : 0;
  if (stopping) {
    for (unsigned i = taken; i < wanted; ++i) {
      buffer[2 * i] = o->held_left;
      buffer[2 * i + 1u] = o->held_right;
    }
    atomic_store(&o->stop_fill_frames,
                 atomic_load(&o->stop_fill_frames) + wanted - taken);
  } else {
    if (taken < DMA_FRAMES && !atomic_load(&o->underrun_frames)) {
      atomic_store(&o->first_underrun_accepted_frames,
                   atomic_load(&o->accepted_frames));
      atomic_store(&o->first_underrun_dma_frames,
                   atomic_load(&o->dma_source_frames));
    }
    atomic_store(&o->underrun_frames,
                 atomic_load(&o->underrun_frames) + DMA_FRAMES - taken);
  }
  if (taken) {
    o->held_left = buffer[2 * (taken - 1u)];
    o->held_right = buffer[2 * (taken - 1u) + 1u];
  }
  if (!stopping && taken < DMA_FRAMES)
    o->held_left = o->held_right = 0;
  for (unsigned i = wanted; i < DMA_FRAMES; ++i)
    buffer[2 * i] = buffer[2 * i + 1u] = 0;
  descriptor->silent = stopping && !o->fade.remaining;
  if (stopping)
    dbt_audio_fade(&o->fade, buffer, DMA_FRAMES);
  descriptor->valid = taken;
  vTaskNotifyGiveFromISR(o->task, &wake);
  return wake == pdTRUE;
}
static void close_output(void) {
#ifdef DBT_PROBE_SOUND_BUILD
  dbt_audio_set_mix(&output.audio,0,0);
  output.sound=0;
  uint32_t token=atomic_exchange_explicit(&output.sound_token,0,memory_order_acq_rel);
  if(token)atomic_store_explicit(&output.sound_retired,token,memory_order_release);
  atomic_store_explicit(&output.active_rate,0,memory_order_release);
  output.sound_silence=false;
  if(token)notify_sound();
#endif
  if (output.channel) {
    if (output.enabled)
      (void)i2s_channel_disable(output.channel);
    (void)i2s_del_channel(output.channel);
    output.channel = 0;
  }
  /* All I2S operations and its ISR live on core 0. No callback can still be
   * running on another core when this task frees the internal-SRAM rings. */
  dbt_audio_close(&output.audio);
  output.active = output.enabled = output.stopping = false;
  atomic_store(&output.meter_value, 0);
  for (unsigned i = 0; i < 3; ++i)
    output.meter_db[i] = -60 * 256;
  atomic_store(&output.running, 0);
  atomic_store(&output.requested_bytes, 0);
  atomic_store(&output.ready_frames, 0);
  atomic_store(&output.input_ready_frames, 0);
}
static esp_err_t create_channel(uint32_t rate, bool apll) {
  i2s_chan_config_t channel =
      I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
  channel.dma_desc_num = DMA_DESCRIPTORS;
  channel.dma_frame_num = DMA_FRAMES;
  channel.auto_clear_after_cb = false;
  channel.auto_clear_before_cb = false;
  channel.intr_priority = 1;
  esp_err_t result = i2s_new_channel(&channel, &output.channel, 0);
  if (result != ESP_OK)
    return result;
  i2s_std_config_t config = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(rate),
      .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT,
                                                      I2S_SLOT_MODE_STEREO),
      .gpio_cfg = {.mclk = I2S_GPIO_UNUSED,
                   .bclk = CONFIG_NATIVE_AUDIO_BCLK,
                   .ws = CONFIG_NATIVE_AUDIO_LRCK,
                   .dout = CONFIG_NATIVE_AUDIO_DATA,
                   .din = I2S_GPIO_UNUSED}};
  if (apll)
    config.clk_cfg.clk_src = I2S_CLK_SRC_APLL;
  result = i2s_channel_init_std_mode(output.channel, &config);
  if (result == ESP_OK) {
    i2s_event_callbacks_t callbacks = {.on_sent = sent};
    result = i2s_channel_register_event_callback(output.channel, &callbacks,
                                                 &output);
  }
  if (result != ESP_OK) {
    (void)i2s_del_channel(output.channel);
    output.channel = 0;
  }
  return result;
}
static dbt_result_t start_output(uint32_t rate, uint8_t channels,
                                 dbt_pcm_format_t format) {
  if (output.active)
    return DBT_BUSY;
  atomic_store(&output.allocation_failed_bytes,0);
  atomic_store(&output.allocation_failure_free,0);
  atomic_store(&output.allocation_failure_largest,0);
  uint32_t target = (rate * 64u + 999u) / 1000u;
  dbt_audio_config_t config = {
      rate,
      power2((rate * 32u + 999u) / 1000u),
      power2(target + DMA_DESCRIPTORS * DMA_FRAMES + 128u),
      target,
      format,
      channels,
      true};
  dbt_result_t result = dbt_audio_open(&output.audio, config);
  if (result != DBT_OK)
    return result;
  if (create_channel(rate, true) != ESP_OK &&
      create_channel(rate, false) != ESP_OK) {
    close_output();
    return DBT_INVALID;
  }
  for (unsigned i = 0; i < DMA_DESCRIPTORS; ++i)
    output.descriptors[i] = (descriptor_t){0};
  output.descriptor_count = output.next_descriptor = output.zero_completions =
      output.last_eof = 0;
  output.held_left = output.held_right = 0;
  output.clock_at = (uint32_t)(esp_timer_get_time() / 1000);
  dbt_audio_fade_init(&output.fade, rate / 50u);
  atomic_store(&output.stop_requested, 0);
  atomic_store(&output.stop_finished, 0);
  atomic_store(&output.failed, 0);
  atomic_store(&output.rate, rate);
  atomic_store(&output.accepted_frames, 0);
  atomic_store(&output.rendered_frames, 0);
  atomic_store(&output.dma_eofs, 0);
  atomic_store(&output.dma_source_frames, 0);
  atomic_store(&output.underrun_frames, 0);
  atomic_store(&output.descriptor_errors, 0);
  atomic_store(&output.max_eof_gap_us, 0);
  atomic_store(&output.maximum_render_us, 0);
  atomic_store(&output.render_total_us,0);
  atomic_store(&output.render_calls,0);
  atomic_store(&output.stop_fill_frames, 0);
  atomic_store(&output.requested_bytes, output.audio.allocated_bytes +
                                            DMA_DESCRIPTORS * DMA_FRAMES * 8u);
  atomic_store(&output.first_underrun_accepted_frames, 0);
  atomic_store(&output.first_underrun_dma_frames, 0);
  output.active = true;
#ifdef DBT_PROBE_SOUND_BUILD
  atomic_store_explicit(&output.active_rate,rate,memory_order_release);
#endif
  return DBT_OK;
}
static dbt_result_t enable_output(void) {
  int32_t block[DMA_FRAMES * 2];
  for (unsigned i = 0; i < DMA_DESCRIPTORS; ++i) {
    if (dbt_audio_take(&output.audio, block, DMA_FRAMES) != DMA_FRAMES)
      return DBT_INVALID;
    size_t loaded = 0;
    if (i2s_channel_preload_data(output.channel, block, sizeof(block),
                                 &loaded) != ESP_OK ||
        loaded != sizeof(block))
      return DBT_INVALID;
  }
  output.enabled = i2s_channel_enable(output.channel) == ESP_OK;
  if (output.enabled) {
    output.clock_at = (uint32_t)(esp_timer_get_time() / 1000);
    dbt_audio_clock_reset(&output.audio, output.clock_at);
  }
  atomic_store(&output.running, output.enabled);
  return output.enabled ? DBT_OK : DBT_INVALID;
}
static void codec_request(request_t request){
  if (request.start && atomic_load_explicit(&output.restart_hold,memory_order_acquire))report_done(request.id,DBT_BUSY);
  else if (output.stopping)report_done(request.id, DBT_BUSY);
  else if (request.start){
#ifdef DBT_PROBE_SOUND_BUILD
    if(!output.active)output.sound_standalone=false;
#endif
    report_done(request.id,start_output(request.rate,request.channels,request.format));
  }
  else if (!output.active || !output.enabled){close_output();report_done(request.id,DBT_OK);}
  else {
    dbt_audio_begin_drain(&output.audio);output.stopping=true;output.stop_request=request.id;
    output.stop_at=(uint32_t)(esp_timer_get_time()/1000);
    atomic_store_explicit(&output.stop_requested,1,memory_order_release);
  }
}
#ifdef DBT_PROBE_SOUND_BUILD
enum { REQ_CODEC=0, REQ_SOUND_RESERVE=1, REQ_SOUND_ATTACH=2, REQ_SOUND_RELEASE=3 };
static void mix_sound(void *p,int32_t *pcm,uint32_t frames){dbt_sound_mix(p,pcm,(uint16_t)frames);}
static void retire_sound(void){
  dbt_audio_set_mix(&output.audio,0,0);output.sound=0;
  uint32_t token=atomic_exchange_explicit(&output.sound_token,0,memory_order_acq_rel);
  if(token)atomic_store_explicit(&output.sound_retired,token,memory_order_release);
  if(token)notify_sound();
}
static void reclaim_cancelled(const request_t *request,dbt_result_t result){
  if(result!=DBT_OK)return;
  if(request->kind==REQ_CODEC && request->start){close_output();return;}
  uint32_t token=atomic_load_explicit(&output.sound_token,memory_order_acquire);
  if((request->kind==REQ_SOUND_RESERVE && token==request->id) ||
     (request->kind==REQ_SOUND_ATTACH && token==request->sound_token && output.sound==request->sound)){
    bool standalone=output.sound_standalone;retire_sound();if(standalone)close_output();
  }
}
static void handle_request(request_t request){
  if(atomic_load_explicit(&output.restart_hold,memory_order_acquire) &&
     (request.kind==REQ_SOUND_RESERVE || request.kind==REQ_SOUND_ATTACH || (request.kind==REQ_CODEC && request.start))){report_done(request.id,DBT_BUSY);return;}
  if(output.stopping || output.transition_sound){report_done(request.id,DBT_BUSY);return;}
  uint32_t token=atomic_load_explicit(&output.sound_token,memory_order_acquire);
  if(request.kind==REQ_SOUND_RESERVE){
    if(token || (output.active && (output.audio.config.rate!=request.rate || request.exclusive))){report_done(request.id,DBT_BUSY);return;}
    output.sound_standalone=!output.active;
    dbt_result_t result=output.active?DBT_OK:start_output(request.rate,2,DBT_PCM_S32_LE);
    if(result==DBT_OK)atomic_store_explicit(&output.sound_token,request.id,memory_order_release);
    report_done(request.id,result);return;
  }
  if(request.kind==REQ_SOUND_ATTACH){
    OUTPUT_TEST_STEP(1);
    if(dbt_ticket_state(&output.ticket,request.id)==DBT_TICKET_CANCEL){report_done(request.id,DBT_BUSY);return;}
    if(!token || token!=request.sound_token || !request.sound || output.sound || !dbt_sound_play_ready(request.sound)){
      report_done(request.id,DBT_INVALID);return;
    }
    output.sound=request.sound;dbt_audio_set_mix(&output.audio,mix_sound,output.sound);
    OUTPUT_TEST_STEP(2);
    report_done(request.id,DBT_OK);return;
  }
  if(request.kind==REQ_SOUND_RELEASE){
    if(!token && request.sound_token==atomic_load_explicit(&output.sound_retired,memory_order_acquire)){report_done(request.id,DBT_OK);return;}
    if(!token || token!=request.sound_token){report_done(request.id,DBT_INVALID);return;}
    if(atomic_load(&output.failed)){
      bool standalone=output.sound_standalone;retire_sound();
      if(standalone)close_output();
      report_done(request.id,DBT_OK);return;
    }
    if(output.sound && !dbt_sound_play_drained(output.sound)){report_done(request.id,DBT_BUSY);return;}
    if(!output.sound){bool standalone=output.sound_standalone;retire_sound();if(standalone)close_output();report_done(request.id,DBT_OK);return;}
    if(!output.sound_standalone){retire_sound();report_done(request.id,DBT_OK);return;}
    output.transition_sound=true;output.pending_sound_transition=request;return;
  }
  if(request.kind!=REQ_CODEC){report_done(request.id,DBT_INVALID);return;}
  if(!token){codec_request(request);return;}
  if(request.start && !output.sound_standalone){report_done(request.id,DBT_BUSY);return;}
  if(!output.sound){bool standalone=output.sound_standalone;retire_sound();if(standalone)close_output();codec_request(request);return;}
  dbt_sound_play_request_stop(output.sound);
  notify_sound();
  output.transition_sound=true;output.pending_sound_transition=request;
}
static void advance_sound_transition(void){
  if(!output.transition_sound || !output.sound || !dbt_sound_play_drained(output.sound))return;
  if(output.sound_standalone && (!output.enabled || dbt_audio_output_frames(&output.audio)))return;
  request_t pending=output.pending_sound_transition;bool standalone=output.sound_standalone;
  retire_sound();output.transition_sound=false;
  if(standalone){
    if(!output.enabled){close_output();if(pending.kind==REQ_CODEC)codec_request(pending);else report_done(pending.id,DBT_OK);return;}
    output.restart_after_stop=pending.kind==REQ_CODEC && pending.start;
    output.pending_sound_transition=pending;
    request_t stop=pending;stop.start=false;codec_request(stop);
  }else if(pending.kind==REQ_CODEC)codec_request(pending);
  else report_done(pending.id,DBT_OK);
}
#else
static void reclaim_cancelled(const request_t *request,dbt_result_t result){if(result==DBT_OK && request->start)close_output();}
static void handle_request(request_t request){codec_request(request);}
#endif
static bool render_step(void) {
#ifdef DBT_PROBE_SOUND_BUILD
    bool made_silence=false,blocked_sound=false;
#endif
    uint32_t tag=atomic_load_explicit(&output.ticket.tag,memory_order_acquire);
    if((tag&7u)==DBT_TICKET_RECLAIM){
      finish_cancel(tag>>3,(dbt_result_t)atomic_load_explicit(&output.ticket.result,memory_order_relaxed));
      return true;
    }
    request_t request;
    if (xQueueReceive(output.requests, &request, 0) == pdTRUE) {
      unsigned state=dbt_ticket_state(&output.ticket,request.id);
      if(state==DBT_TICKET_CANCEL)
        report_done(request.id,DBT_BUSY);
      else if(state==DBT_TICKET_WAITING)handle_request(request);
      else atomic_fetch_add(&output.stale_receipts,1);
    }
    if (output.active) {
      if (output.stopping &&
          atomic_load_explicit(&output.stop_finished, memory_order_acquire)) {
        close_output();
#ifdef DBT_PROBE_SOUND_BUILD
        if(output.restart_after_stop){
          output.restart_after_stop=false;request_t restart=output.pending_sound_transition;
          output.sound_standalone=false;
          OUTPUT_TEST_STEP(3);
          if(dbt_ticket_state(&output.ticket,output.stop_request)==DBT_TICKET_CANCEL){report_done(output.stop_request,DBT_BUSY);return true;}
          report_done(output.stop_request,start_output(restart.rate,restart.channels,restart.format));return true;
        }
#endif
        report_done(output.stop_request, DBT_OK);
        return true;
      }
      if (output.stopping &&
          (uint32_t)(esp_timer_get_time() / 1000) - output.stop_at > 500) {
        close_output();
#ifdef DBT_PROBE_SOUND_BUILD
        output.restart_after_stop=false;
#endif
        atomic_store(&output.failed, 1);
        report_done(output.stop_request, DBT_INVALID);
        return true;
      }
      if (atomic_load(&output.failed)) {
#ifdef DBT_PROBE_SOUND_BUILD
        if(output.transition_sound){uint32_t id=output.pending_sound_transition.id;output.transition_sound=false;close_output();report_done(id,DBT_INVALID);}
#endif
        return false;
      }
      uint32_t target = output.audio.config.target_frames +
                        (output.enabled ? 0 : DMA_DESCRIPTORS * DMA_FRAMES);
      uint32_t available = dbt_audio_output_frames(&output.audio);
      if (available < target) {
        uint32_t began = (uint32_t)esp_timer_get_time();
        uint32_t budget = target - available;
        if (budget > 128)
          budget = 128;
#ifdef DBT_PROBE_SOUND_BUILD
        if(output.sound){
          uint16_t sound_frames=dbt_sound_play_mix_frames(output.sound,(uint16_t)budget);
          /* Give a live WAV producer a turn while DMA has a safe reserve (or
           * has not started). Do not pad a normal file read with silent gaps.
           * At a genuine low-water emergency, music/DMA still makes progress. */
          if(!sound_frames && (!output.enabled || available>=2u*DMA_FRAMES)){
            notify_sound();budget=0;blocked_sound=true;
          }
          if(sound_frames && sound_frames<budget)budget=sound_frames;
        }
#endif
        uint32_t made = budget?dbt_audio_render(&output.audio, budget):0;
#ifdef DBT_PROBE_SOUND_BUILD
        if(made && output.sound_silence){
          output.clock_at=(uint32_t)(esp_timer_get_time()/1000);
          dbt_audio_clock_reset(&output.audio,output.clock_at);output.sound_silence=false;
        }
        if(!made && output.sound &&
           (output.sound_standalone || output.transition_sound ||
            (!dbt_audio_input_frames(&output.audio) && (!output.enabled || !available))) &&
           (!output.enabled || !dbt_sound_play_drained(output.sound))){
          uint16_t sound_budget=dbt_sound_play_mix_frames(output.sound,(uint16_t)budget);
          made=sound_budget?dbt_audio_render_silence(&output.audio,sound_budget):0;
          made_silence=made!=0;
          if(made && !output.sound_standalone && !output.sound_silence){
            output.clock_at=(uint32_t)(esp_timer_get_time()/1000);
            dbt_audio_clock_reset(&output.audio,output.clock_at);output.sound_silence=true;
          }
        }
#endif
        if (made) {
          uint32_t elapsed = (uint32_t)esp_timer_get_time() - began;
          if (elapsed > atomic_load(&output.maximum_render_us))
            atomic_store(&output.maximum_render_us, elapsed);
          atomic_fetch_add(&output.render_total_us,elapsed);
          atomic_fetch_add(&output.render_calls,1);
          atomic_store(&output.rendered_frames,
                       atomic_load(&output.audio.rendered_frames));
          if (output.wake_decoder)
            output.wake_decoder(output.wake_context);
#ifdef DBT_PROBE_SOUND_BUILD
          if(output.sound)notify_sound();
#endif
        }
      }
      available = dbt_audio_output_frames(&output.audio);
      atomic_store(&output.ready_frames, available);
      atomic_store(&output.input_ready_frames,
                   dbt_audio_input_frames(&output.audio));
      if (!output.enabled && available >= target) {
        if (enable_output() != DBT_OK)
          atomic_store(&output.failed, 1);
      }
      uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
      if (now - output.meter_at >= 40) {
        output.meter_at = now;
        uint32_t packed = 0;
        for (unsigned i = 0; i < 3; ++i) {
          int32_t a =
              fixed_dsp_level_db_q8(output.audio.dsp.goertzel_levels[i]);
          int32_t b = fixed_dsp_level_db_q8(output.audio.dsp.meter_levels[i]);
          int32_t target = a > b ? a : b;
          if (target < -60 * 256)
            target = -60 * 256;
          if (target > 0)
            target = 0;
          output.meter_db[i] +=
              (int32_t)(((int64_t)target - output.meter_db[i]) * 115 / 256);
          int32_t value =
              ((output.meter_db[i] + 60 * 256) * 100 + 30 * 256) / (60 * 256);
          if (value < 0)
            value = 0;
          if (value > 100)
            value = 100;
          packed |= (uint32_t)value << (i * 8u);
        }
        atomic_store_explicit(&output.meter_value, packed,
                              memory_order_release);
      }
      if (output.enabled && !output.stopping && now - output.clock_at >= 500
#ifdef DBT_PROBE_SOUND_BUILD
          && !output.sound_standalone && !output.sound_silence && !output.transition_sound
#endif
      ) {
        dbt_audio_clock_update(&output.audio, now);
        output.clock_at = now;
      }
#ifdef DBT_PROBE_SOUND_BUILD
      if(blocked_sound)return false;
#endif
      if (output.active && dbt_audio_output_frames(&output.audio) < target &&
          dbt_audio_input_frames(&output.audio) > 1)
        return true;
#ifdef DBT_PROBE_SOUND_BUILD
      if(output.active && output.sound && output.sound_standalone &&
         dbt_audio_output_frames(&output.audio)<target && dbt_sound_play_ready(output.sound))return true;
      /* A stop/fade or pre-enable silence refill is runnable work even after
       * its WAV ring drains. Do not depend on periodic polling to finish it. */
      if(made_silence && dbt_audio_output_frames(&output.audio)<target &&
         dbt_sound_play_mix_frames(output.sound,1))return true;
#endif
    }
#ifdef DBT_PROBE_SOUND_BUILD
    advance_sound_transition();
#endif
    return false;
}
static TickType_t render_wait_ticks(void){
  uint32_t now=(uint32_t)(esp_timer_get_time()/1000),ms=UINT32_MAX;
  if(output.stopping){uint32_t elapsed=now-output.stop_at;ms=elapsed>500u?1u:501u-elapsed;}
  if(output.active && !atomic_load(&output.failed)){
    uint32_t elapsed=now-output.meter_at,remaining=elapsed>=40u?1u:40u-elapsed;
    if(remaining<ms)ms=remaining;
    if(output.enabled && !output.stopping
#ifdef DBT_PROBE_SOUND_BUILD
       && !output.sound_standalone && !output.sound_silence && !output.transition_sound
#endif
    ){
      elapsed=now-output.clock_at;remaining=elapsed>=500u?1u:500u-elapsed;
      if(remaining<ms)ms=remaining;
    }
  }
  if(ms==UINT32_MAX)return portMAX_DELAY;
  TickType_t ticks=pdMS_TO_TICKS(ms);return ticks?ticks:1;
}
static void render_loop(void *unused) {
  (void)unused;
  for (;;) {
    if(!render_step())ulTaskNotifyTake(pdTRUE,render_wait_ticks());
  }
}
dbt_result_t probe_output_init(void (*wake_decoder)(void *), void *context) {
  output.wake_decoder = wake_decoder;
  output.wake_context = context;
  dbt_ticket_init(&output.ticket);
  for (unsigned i = 0; i < 3; ++i)
    output.meter_db[i] = -60 * 256;
  dbt_audio_memory_t memory = {0, allocate, release};
  if (dbt_audio_init(&output.audio, memory) != DBT_OK)
    return DBT_INVALID;
  output.requests = xQueueCreate(2, sizeof(request_t));
  if (!output.requests)
    return DBT_NO_SPACE;
  return xTaskCreatePinnedToCore(render_loop, "native_render", 6144, 0, 6,
                                 &output.task, 0) == pdPASS
             ? DBT_OK
             : DBT_NO_SPACE;
}
/* One accepted request retains the gate until its own receipt arrives. This
 * prevents competing workers consuming each other's completion or a timed-out
 * request later changing the output behind its caller's back. */
static dbt_result_t request_begin(request_t *request,uint32_t *ticket,bool legacy) {
  if(!ticket)return DBT_INVALID;
  *ticket=0;
  if(!output.task || !output.requests)return DBT_INVALID;
  bool starts=request->start;
#ifdef DBT_PROBE_SOUND_BUILD
  starts=starts || request->kind==REQ_SOUND_RESERVE || request->kind==REQ_SOUND_ATTACH;
#endif
  if(starts && atomic_load_explicit(&output.restart_hold,memory_order_acquire))return DBT_BUSY;
  uint32_t avoid1=0,avoid2=0;
#ifdef DBT_PROBE_SOUND_BUILD
  avoid1=atomic_load_explicit(&output.sound_token,memory_order_acquire);
  avoid2=atomic_load_explicit(&output.sound_retired,memory_order_acquire);
#endif
  uint32_t id;dbt_result_t reserved=dbt_ticket_reserve(&output.ticket,avoid1,avoid2,&id);
  if(reserved!=DBT_OK)return reserved;
#ifdef DBT_PROBE_SOUND_BUILD
  /* Refresh identities after reservation: another operation could have
   * completed between the optimistic snapshot above and the gate CAS. */
  if(id==atomic_load_explicit(&output.sound_token,memory_order_acquire) ||
     id==atomic_load_explicit(&output.sound_retired,memory_order_acquire)){
    (void)dbt_ticket_abort(&output.ticket,id);notify_clients();return DBT_BUSY;
  }
#endif
  /* Close the producer/owner fence race before any request is published. */
  if(starts && atomic_load_explicit(&output.restart_hold,memory_order_acquire)){
    (void)dbt_ticket_abort(&output.ticket,id);notify_clients();return DBT_BUSY;
  }
  request->id=id;request->legacy=legacy;
  output.inflight=*request;
  if(!dbt_ticket_publish(&output.ticket,id))return DBT_INVALID;
  if (xQueueSend(output.requests, request, 0) != pdTRUE){
    (void)dbt_ticket_abort(&output.ticket,id);notify_clients();
    xTaskNotifyGive(output.task);
    return DBT_BUSY;
  }
  *ticket=id;
  xTaskNotifyGive(output.task);
  return DBT_PENDING;
}
dbt_result_t probe_output_request_poll(uint32_t ticket){
  unsigned state=dbt_ticket_state(&output.ticket,ticket);
  dbt_result_t result=dbt_ticket_poll(&output.ticket,ticket);
  if(result!=DBT_PENDING && (state==DBT_TICKET_DONE || state==DBT_TICKET_CANCELLED))notify_clients();
  return result;
}
dbt_result_t probe_output_request_cancel(uint32_t ticket){
  dbt_result_t result=dbt_ticket_cancel(&output.ticket,ticket);
  if(result==DBT_PENDING && output.task)xTaskNotifyGive(output.task);
  return result;
}
/* Compatibility for unconverted clients only. New runtime clients use the
 * ticket API; this wrapper retains the old bounded-wait/late-reclaim policy. */
static dbt_result_t request_output(request_t *request) {
  unsigned waits=0,max_waits=50;uint32_t ticket=0;
#ifdef DBT_PROBE_SOUND_BUILD
  if(request->kind==REQ_SOUND_RELEASE)max_waits=1;
#endif
  for(;;){dbt_result_t result=request_begin(request,&ticket,true);
    if(result==DBT_PENDING)break;
    if(result!=DBT_BUSY || ++waits>=max_waits)return result;
    ulTaskNotifyTake(pdTRUE,event_wait_ticks(2));
  }
  uint32_t began=(uint32_t)(esp_timer_get_time()/1000);
  for (;;) {
    dbt_result_t result=probe_output_request_poll(ticket);
    if(result!=DBT_PENDING)return result;
    if((uint32_t)(esp_timer_get_time()/1000)-began>=1500u){
      atomic_store_explicit(&output.request_fault,1,memory_order_release);
      if(probe_output_request_cancel(ticket)==DBT_PENDING)return DBT_BUSY;
      atomic_store_explicit(&output.request_fault,0,memory_order_release);
    }
    ulTaskNotifyTake(pdTRUE,event_wait_ticks(2));
  }
}
#ifdef DBT_PROBE_SOUND_BUILD
dbt_result_t probe_output_sound_reserve_begin(uint32_t rate,bool exclusive,uint32_t *ticket){
  if(ticket)*ticket=0;
  if(!ticket || rate<8000 || rate>192000)return DBT_INVALID;
  request_t request={.kind=REQ_SOUND_RESERVE,.rate=rate,.exclusive=exclusive};return request_begin(&request,ticket,false);
}
dbt_result_t probe_output_sound_attach_begin(uint32_t token,dbt_sound_play_t *play,uint32_t *ticket){
  if(ticket)*ticket=0;
  if(!token || !play)return DBT_INVALID;
  request_t request={.kind=REQ_SOUND_ATTACH,.sound_token=token,.sound=play};return request_begin(&request,ticket,false);
}
dbt_result_t probe_output_sound_release_begin(uint32_t token,uint32_t *ticket){
  if(ticket)*ticket=0;
  if(!token)return DBT_INVALID;
  request_t request={.kind=REQ_SOUND_RELEASE,.sound_token=token};return request_begin(&request,ticket,false);
}
dbt_result_t probe_output_sound_reserve(uint32_t rate,bool exclusive,uint32_t *token){
  if(!token || rate<8000 || rate>192000)return DBT_INVALID;
  *token=0;request_t request={.kind=REQ_SOUND_RESERVE,.rate=rate,.exclusive=exclusive};
  dbt_result_t result=request_output(&request);if(result==DBT_OK)*token=request.id;return result;
}
dbt_result_t probe_output_sound_attach(uint32_t token,dbt_sound_play_t *play){
  if(!token || !play)return DBT_INVALID;
  request_t request={.kind=REQ_SOUND_ATTACH,.sound_token=token,.sound=play};return request_output(&request);
}
dbt_result_t probe_output_sound_release(uint32_t token){
  if(!token)return DBT_INVALID;
  request_t request={.kind=REQ_SOUND_RELEASE,.sound_token=token};return request_output(&request);
}
bool probe_output_sound_alive(uint32_t token){return token && atomic_load_explicit(&output.sound_token,memory_order_acquire)==token;}
uint32_t probe_output_active_rate(void){return atomic_load_explicit(&output.active_rate,memory_order_acquire);}
void probe_output_sound_wake(void (*wake)(void *),void *context){
  atomic_store_explicit(&output.sound_context,context,memory_order_relaxed);
  atomic_store_explicit(&output.wake_sound,wake,memory_order_release);
}
void probe_output_sound_data_ready(void){if(output.task)xTaskNotifyGive(output.task);}
#endif
dbt_result_t probe_output_stream_begin(bool start,uint32_t rate,uint8_t channels,dbt_pcm_format_t format,uint32_t *ticket){
  if(ticket)*ticket=0;
  if(start && (format<DBT_PCM_S16_LE || format>DBT_PCM_S32_LE || channels<1 || channels>2 || rate<8000 || rate>192000))return DBT_INVALID;
  request_t request={.start=start,.rate=rate,.channels=channels,.format=format};return request_begin(&request,ticket,false);
}
dbt_result_t probe_output_stream_format(bool start, uint32_t rate,
                                        uint8_t channels,
                                        dbt_pcm_format_t format) {
  if (start && (format < DBT_PCM_S16_LE || format > DBT_PCM_S32_LE ||
                channels < 1 || channels > 2 || rate < 8000u || rate > 192000u))
    return DBT_INVALID;
  request_t request = {.start = start,.rate = rate,.channels = channels,.format = format};
  return request_output(&request);
}
dbt_result_t probe_output_stream(bool start, uint32_t rate, uint8_t channels) {
  return probe_output_stream_format(start, rate, channels, DBT_PCM_S16_LE);
}
dbt_result_t probe_output_try_pcm_format(const void *samples, uint16_t frames,
                                     uint8_t channels, uint32_t rate,
                                     dbt_pcm_format_t format) {
  if (!samples || !frames || format < DBT_PCM_S16_LE ||
      format > DBT_PCM_S32_LE || atomic_load(&output.failed) || atomic_load(&output.request_fault) ||
      !output.active || output.audio.config.rate != rate ||
      output.audio.config.channels != channels ||
      output.audio.config.format != format)
    return DBT_INVALID;
  dbt_result_t result=dbt_audio_submit(&output.audio,samples,frames);
  if(result==DBT_OK){atomic_fetch_add(&output.accepted_frames,frames);xTaskNotifyGive(output.task);}
  else if(result==DBT_BUSY)xTaskNotifyGive(output.task);
  return result;
}
dbt_result_t probe_output_pcm_format(const void *samples,uint16_t frames,uint8_t channels,uint32_t rate,dbt_pcm_format_t format){
  uint32_t began = (uint32_t)(esp_timer_get_time() / 1000);
  for (;;) {
    dbt_result_t result=probe_output_try_pcm_format(samples,frames,channels,rate,format);
    if(result==DBT_OK)return result;
    if (result != DBT_BUSY ||
        (uint32_t)(esp_timer_get_time() / 1000) - began >= 100)
      return result;
    xTaskNotifyGive(output.task);
    ulTaskNotifyTake(pdTRUE, event_wait_ticks(2));
  }
}
dbt_result_t probe_output_pcm(const int16_t *samples, uint16_t frames,
                              uint8_t channels, uint32_t rate) {
  return probe_output_pcm_format(samples, frames, channels, rate,
                                 DBT_PCM_S16_LE);
}
bool probe_output_failed(void){return atomic_load(&output.failed) || atomic_load(&output.request_fault);}
void probe_output_stats(probe_output_stats_t *s) {
  if (!s)
    return;
#define LOAD(field) s->field = atomic_load(&output.field)
  LOAD(running);
  LOAD(failed);
  LOAD(rate);
  LOAD(requested_bytes);
  LOAD(accepted_frames);
  LOAD(rendered_frames);
  LOAD(ready_frames);
  LOAD(input_ready_frames);
  LOAD(dma_eofs);
  LOAD(dma_source_frames);
  LOAD(underrun_frames);
  LOAD(descriptor_errors);
  LOAD(max_eof_gap_us);
  LOAD(maximum_render_us);
  LOAD(render_total_us);
  LOAD(render_calls);
  LOAD(allocation_failed_bytes);
  LOAD(allocation_failure_free);
  LOAD(allocation_failure_largest);
  LOAD(stop_fill_frames);
  LOAD(first_underrun_accepted_frames);
  LOAD(first_underrun_dma_frames);
  LOAD(request_fault);
  LOAD(stale_receipts);
#undef LOAD
#define PROFILE_LOAD(field) s->profile_##field=atomic_load(&output.audio.profile_##field)
  PROFILE_LOAD(conversion);PROFILE_LOAD(dsp);PROFILE_LOAD(tail);PROFILE_LOAD(controls);PROFILE_LOAD(frames);
  PROFILE_LOAD(conversion_max);PROFILE_LOAD(dsp_max);PROFILE_LOAD(tail_max);
#undef PROFILE_LOAD
  s->render_stack_low_water_bytes=output.task?(uint32_t)uxTaskGetStackHighWaterMark(output.task):0;
  s->failed|=s->request_fault;
  dbt_audio_clock_stats_t clock = {0};
  s->clock_snapshot_valid = dbt_audio_clock_snapshot(&output.audio, &clock);
  s->clock_ppm = clock.ppm;
  s->clock_observed_ppm = clock.observed_ppm;
  s->clock_error_frames = clock.error_frames;
  s->clock_state = clock.state;
  s->clock_target_frames = clock.target_frames;
  s->clock_updates = clock.updates;
  s->clock_disturbances = clock.disturbances;
  s->clock_observed_valid = clock.observed_valid;
}
uint16_t probe_output_delay_100us(uint32_t rate) {
  if (!rate)
    return 0;
  uint32_t frames = atomic_load(&output.ready_frames) +
                    atomic_load(&output.input_ready_frames) +
                    DMA_DESCRIPTORS * DMA_FRAMES + 80u;
  if (!atomic_load(&output.running))
    frames = (rate * 64u + 999u) / 1000u + DMA_DESCRIPTORS * DMA_FRAMES + 80u;
  frames += dbt_resampler_delay_frames(rate);
  uint32_t value = (uint32_t)(((uint64_t)frames * 10000u + rate - 1u) / rate);
  return value > UINT16_MAX ? UINT16_MAX : (uint16_t)value;
}
dbt_result_t probe_output_controls(int8_t bass, int8_t mid, int8_t treble,
                                   uint8_t volume, uint8_t flags) {
  return dbt_audio_controls(&output.audio, bass, mid, treble, volume, flags);
}

dbt_result_t probe_output_volume(uint8_t volume) {
  return dbt_audio_volume(&output.audio, volume);
}

dbt_result_t probe_output_eq(int8_t bass, int8_t mid, int8_t treble) {
  return dbt_audio_eq(&output.audio, bass, mid, treble);
}
dbt_result_t probe_output_control(uint8_t flags) {
  return dbt_audio_control_flags(&output.audio, flags);
}
dbt_result_t probe_output_analysis(bool enabled) {
  return dbt_audio_analysis(&output.audio, enabled);
}

void probe_output_meter(uint8_t values[3]) {
  if (!values)
    return;
  uint32_t v = atomic_load_explicit(&output.meter_value, memory_order_acquire);
  for (unsigned i = 0; i < 3; ++i)
    values[i] = (uint8_t)(v >> (i * 8u));
}

void probe_output_restart_hold(bool hold){
  atomic_store_explicit(&output.restart_hold,hold,memory_order_release);
  if(output.task)xTaskNotifyGive(output.task);
}
bool probe_output_restart_quiescent(void){
  if(!atomic_load_explicit(&output.restart_hold,memory_order_acquire) ||
     !dbt_ticket_idle(&output.ticket) ||
     atomic_load(&output.failed) || atomic_load(&output.request_fault) ||
     atomic_load(&output.running) || atomic_load(&output.requested_bytes))return false;
#ifdef DBT_PROBE_SOUND_BUILD
  if(atomic_load_explicit(&output.sound_token,memory_order_acquire))return false;
#endif
  return dbt_ticket_idle(&output.ticket);
}
