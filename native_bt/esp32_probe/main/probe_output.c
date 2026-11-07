#include "probe_schedule_limits.h"
#include "probe_output.h"
#include "dbt_async_ticket.h"
#ifdef DBT_PROBE_DSP_ASSIST
#include "dbt_dsp_job.h"
#include "probe_audio.h"
#ifdef __XTENSA__
#include <xtensa/core-macros.h>
#endif
#endif
#include <stddef.h>
#ifdef __XTENSA__
#include <xtensa/xtruntime.h>
#endif
#if defined(DBT_PROBE_CODEC_AWARE_RENDER) && defined(DBT_PROBE_SCHEDULER_TRACE)
#include "probe_scheduler.h"
#endif
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
#define DMA_FRAMES PROBE_OUTPUT_DMA_FRAMES
#define DMA_DESCRIPTORS PROBE_OUTPUT_DMA_DESCRIPTORS
#define DMA_IRQ_PRIORITY 2
#define REFILL_GUARD_US 64u
#define REFILL_COPY_BUDGET_US 32u
/* The working Bluedroid product consumes exactly one decoded source frame per
 * output frame. Its drift drop/insert path is explicitly disabled. Preserve
 * that PCM behavior: the added always-on FIR is not part of codec decoding or
 * product DSP. Finite resampler diagnostics retain their existing mode. */
#ifdef DBT_PROBE_STAY_NATIVE
#define OUTPUT_ADAPTIVE_CLOCK 0
#else
#define OUTPUT_ADAPTIVE_CLOCK 1
#endif
#if defined(DBT_PROBE_PCM_LOANS) && !defined(DBT_PROBE_ASYNC_AUDIO)
#error "PCM loans require the asynchronous audio client"
#endif
enum {REQ_CODEC=0,REQ_SOUND_RESERVE=1,REQ_SOUND_ATTACH=2,REQ_SOUND_RELEASE=3,REQ_PCM_LOAN=4};
static TickType_t event_wait_ticks(uint32_t ms) {
  TickType_t ticks = pdMS_TO_TICKS(ms);
  return ticks ? ticks : 1;
}
typedef struct {
  bool start;
  uint8_t kind;
  bool legacy;
  bool share_decoder_core;
  uint32_t rate;
  uint8_t channels;
  uint16_t priming_ms; /* Fits existing request padding on ESP32. */
  dbt_pcm_format_t format;
  uint32_t id;
#ifdef DBT_PROBE_PCM_LOANS
  const void *pcm;
  uint16_t frames;
#endif
#ifdef DBT_PROBE_SOUND_BUILD
  uint32_t sound_token;
  dbt_sound_play_t *sound;
  bool exclusive;
#endif
} request_t;
_Static_assert(offsetof(request_t,format)==12 && offsetof(request_t,id)==16,"Priming must reuse the existing request prefix padding");
typedef struct {
  uint32_t id;
  dbt_result_t result;
} response_t;
typedef struct {
  void *address;
  _Atomic uint32_t valid;
  bool silent;
} descriptor_t;
typedef struct {
  dbt_audio_t audio;
  dbt_audio_fade_t fade;
  descriptor_t descriptors[DMA_DESCRIPTORS];
  i2s_chan_handle_t channel;
  _Atomic(TaskHandle_t) task;
#ifdef DBT_PROBE_CODEC_AWARE_RENDER
  TaskHandle_t workers[2]; /* Handles stay valid when a notification races handoff. */
  _Atomic bool transfer_pending;
  _Atomic uint32_t owner_transfers,auxiliary_stack_bytes,auxiliary_allocation_failures;
#endif
  QueueHandle_t requests;
  void (*wake_decoder)(void *);
  void *wake_context;
  bool active, enabled, stopping;
  _Atomic uint32_t priming_state,priming_at_ms;
  uint8_t descriptor_order_seen; /* Complete repeat verifies discovered ring order. */
  uint32_t descriptor_count, next_descriptor, zero_completions, last_eof,
      clock_at;
  uint32_t stop_request, stop_at;
  dbt_async_ticket_t ticket;
  _Atomic uint32_t request_fault,stale_receipts;
  _Atomic bool restart_hold;
  request_t inflight;
#ifdef DBT_PROBE_PCM_LOANS
  uint32_t pcm_loan_id,pcm_loan_began;
#endif
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
  _Atomic int32_t held_left,held_right;
  int32_t meter_db[3];
  uint32_t meter_at;
  _Atomic uint32_t meter_value;
  _Atomic uint32_t stop_requested, stop_finished;
  _Atomic uint32_t running, failed, rate, requested_bytes, accepted_frames,
      rendered_frames, ready_frames, input_ready_frames;
  _Atomic uint32_t dma_eofs, dma_source_frames, underrun_frames,
      descriptor_errors, max_eof_gap_us;
  _Atomic uint32_t descriptor_shape_errors,descriptor_pool_errors,descriptor_order_errors;
  _Atomic uint32_t descriptor_order_recoveries,descriptor_missed_eofs_min,descriptor_recovered_source_frames;
  _Atomic uint32_t descriptor_expected_index,descriptor_observed_index,descriptor_observed_size;
  _Atomic uint32_t descriptor_fault_at_us,descriptor_fault_gap_us,descriptor_fault_stopping;
  _Atomic uint32_t callback_calls,callback_elapsed_us,callback_max_us,callback_core,render_core;
  _Atomic uint32_t input_ring_bytes_avoided,pcm_loans_begun,pcm_loans_retired,pcm_loans_cancelled,pcm_loan_active,pcm_loan_max_us;
  _Atomic uint32_t input_capacity_frames,input_ring_bytes,actual_target_frames;
  _Atomic uint32_t maximum_render_us, stop_fill_frames;
  _Atomic uint32_t render_total_us,render_calls;
  _Atomic uint32_t allocation_failed_bytes,allocation_failure_free,allocation_failure_largest;
  _Atomic uint32_t allocation_failure_rate,allocation_failure_channels,allocation_failure_sample_bytes,allocation_failure_at_us;
  _Atomic uint32_t allocation_failure_audio_requested,allocation_failure_audio_allocated,allocation_failure_pages;
  _Atomic uint32_t allocation_failure_free_before,allocation_failure_free_after_cleanup;
  _Atomic uint32_t allocation_failure_free_32bit,allocation_failure_largest_32bit;
  /* Render-owner startup context, read only by its allocator callback. */
  uint32_t allocation_request_rate,allocation_request_channels,allocation_request_sample_bytes;
  uint32_t allocation_audio_requested,allocation_input_requested,allocation_success_bytes,allocation_success_calls;
  _Atomic uint32_t first_underrun_accepted_frames, first_underrun_dma_frames;
  _Atomic uint32_t refill_period_us,refill_pending_index,refill_deadline_us;
  _Atomic uint32_t refill_recovered_frames,refill_recovered_calls,refill_unfilled_frames,refill_canceled_frames;
  _Atomic uint32_t refill_copy_max_us,refill_mask_max_us,refill_contract_errors;
  /* Logger requests a best-effort snapshot. The render owner reads DSP only
   * after prepared work retires; logger and ISR never read live filter state. */
  _Atomic uint32_t dsp_requested,dsp_generation,dsp_at_us,dsp_open,dsp_rate,dsp_gains,dsp_options;
  _Atomic uint32_t dsp_peak_q27,dsp_limited,dsp_over_full_scale,dsp_frames;
} output_t;
static DRAM_ATTR output_t output;
#ifdef DBT_PROBE_DSP_ASSIST
static DRAM_ATTR struct {
  dbt_dsp_job_lane_t lane;
  uint32_t began_us,submitted_us,committed_us,committed_frames,budget;
  uint32_t extra_cycles,block_max_cycles,commit_delay_max_us,completed_blocks;
#ifdef DBT_PROBE_SCHEDULER_TRACE
  probe_scheduler_mark_t committed_mark;
#endif
  _Atomic uint32_t jobs,blocks,frames,cycles,max_cycles,max_job_frames,queue_max_us,finish_wait_max_us,retire_wait_max_us,wall_max_us,failures;
} assist;
#endif

#ifdef DBT_PROBE_SCHEDULER_TRACE
/* No queue or borrowed object: immutable first-gap metadata survives UART
 * backpressure. All shared cells are atomic. Seq-cst versions and cells give
 * bounded readers an explicit coherent/incoherent result without retry. */
typedef struct {
  _Atomic uint32_t version,valid;
  _Atomic uint32_t stream_serial,at_us,taken_frames,input_frames,output_frames,accepted_frames,rendered_frames,dma_source_frames,acl_us,host_poll_us,media_us,media_admit_us,media_epoch,media_sequence,media_timestamp,media_ssrc,media_result,media_gap_us,media_max_gap_us,media_coherent,decoder_phase,decoder_at_us,decoder_coherent,render_phase,render_at_us,render_coherent,decoder_queued,dsp_assist_state;
} starvation_capture_t;
static DRAM_ATTR struct {
  _Atomic uint32_t stream_serial,acl_us,host_poll_us;
  _Atomic uint32_t media_version,media_us,media_admit_us,media_epoch,media_sequence;
  _Atomic uint32_t media_timestamp,media_ssrc,media_result,media_gap_us,media_max_gap_us;
  _Atomic uint32_t decoder_version,decoder_phase,decoder_at_us,decoder_queued;
  _Atomic uint32_t render_version,render_phase,render_at_us;
  starvation_capture_t first;
} timeline;
void IRAM_ATTR probe_output_trace_acl(uint32_t at){atomic_store(&timeline.acl_us,at);}
void probe_output_trace_host_poll(uint32_t at){atomic_store(&timeline.host_poll_us,at);}
void probe_output_trace_media(uint32_t began,uint32_t ended,uint32_t epoch,uint16_t sequence,uint32_t timestamp,uint32_t ssrc,dbt_result_t result){
  uint32_t version=atomic_load(&timeline.media_version);
  uint32_t previous=atomic_load(&timeline.media_us);
  bool same=epoch==atomic_load(&timeline.media_epoch) && ssrc==atomic_load(&timeline.media_ssrc);
  bool completion=same && began==previous && sequence==atomic_load(&timeline.media_sequence);
  uint32_t gap=completion?atomic_load(&timeline.media_gap_us):same && previous?began-previous:0;
  uint32_t maximum=same?atomic_load(&timeline.media_max_gap_us):0;
  atomic_store(&timeline.media_version,version+1u);
  atomic_store(&timeline.media_us,began);atomic_store(&timeline.media_admit_us,ended);
  atomic_store(&timeline.media_epoch,epoch);atomic_store(&timeline.media_sequence,sequence);
  atomic_store(&timeline.media_timestamp,timestamp);atomic_store(&timeline.media_ssrc,ssrc);
  atomic_store(&timeline.media_result,(uint32_t)result);atomic_store(&timeline.media_gap_us,gap);
  atomic_store(&timeline.media_max_gap_us,gap>maximum?gap:maximum);
  atomic_store(&timeline.media_version,version+2u);
}
void probe_output_trace_decoder(uint32_t phase,uint32_t at,uint32_t queued){
  uint32_t version=atomic_load(&timeline.decoder_version);
  atomic_store(&timeline.decoder_version,version+1u);
  atomic_store(&timeline.decoder_phase,phase);atomic_store(&timeline.decoder_at_us,at);
  atomic_store(&timeline.decoder_queued,queued);
  atomic_store(&timeline.decoder_version,version+2u);
}
static void trace_render(uint32_t phase,uint32_t at){
  uint32_t version=atomic_load(&timeline.render_version);
  atomic_store(&timeline.render_version,version+1u);
  atomic_store(&timeline.render_phase,phase);atomic_store(&timeline.render_at_us,at);
  atomic_store(&timeline.render_version,version+2u);
}
static void IRAM_ATTR capture_starvation(output_t *o,uint32_t now,uint32_t taken){
  starvation_capture_t *f=&timeline.first;uint32_t version=atomic_load(&f->version);
  atomic_store(&f->version,version+1u);
  atomic_store(&f->stream_serial,atomic_load(&timeline.stream_serial));
  atomic_store(&f->at_us,now);atomic_store(&f->taken_frames,taken);
  /* Consumer read precedes producer written; capacity bounds remain visible.
   * These are adjacent atomic observations, not a simultaneous queue lock. */
  uint32_t read=atomic_load(&o->audio.input.read),written=atomic_load(&o->audio.input.written);
  atomic_store(&f->input_frames,written-read);
  read=atomic_load(&o->audio.output.read);written=atomic_load(&o->audio.output.written);
  atomic_store(&f->output_frames,written-read);
  atomic_store(&f->accepted_frames,atomic_load(&o->accepted_frames));
  atomic_store(&f->rendered_frames,atomic_load(&o->audio.rendered_frames));
  atomic_store(&f->dma_source_frames,atomic_load(&o->dma_source_frames));
  atomic_store(&f->acl_us,atomic_load(&timeline.acl_us));
  atomic_store(&f->host_poll_us,atomic_load(&timeline.host_poll_us));
  uint32_t before=atomic_load(&timeline.media_version);
  atomic_store(&f->media_us,atomic_load(&timeline.media_us));
  atomic_store(&f->media_admit_us,atomic_load(&timeline.media_admit_us));
  atomic_store(&f->media_epoch,atomic_load(&timeline.media_epoch));
  atomic_store(&f->media_sequence,atomic_load(&timeline.media_sequence));
  atomic_store(&f->media_timestamp,atomic_load(&timeline.media_timestamp));
  atomic_store(&f->media_ssrc,atomic_load(&timeline.media_ssrc));
  atomic_store(&f->media_result,atomic_load(&timeline.media_result));
  atomic_store(&f->media_gap_us,atomic_load(&timeline.media_gap_us));
  atomic_store(&f->media_max_gap_us,atomic_load(&timeline.media_max_gap_us));
  atomic_store(&f->media_coherent,!(before&1u) && before==atomic_load(&timeline.media_version));
  before=atomic_load(&timeline.decoder_version);
  atomic_store(&f->decoder_phase,atomic_load(&timeline.decoder_phase));atomic_store(&f->decoder_at_us,atomic_load(&timeline.decoder_at_us));
  atomic_store(&f->decoder_queued,atomic_load(&timeline.decoder_queued));
  atomic_store(&f->decoder_coherent,!(before&1u) && before==atomic_load(&timeline.decoder_version));
  before=atomic_load(&timeline.render_version);
  atomic_store(&f->render_phase,atomic_load(&timeline.render_phase));atomic_store(&f->render_at_us,atomic_load(&timeline.render_at_us));
  atomic_store(&f->render_coherent,!(before&1u) && before==atomic_load(&timeline.render_version));
#ifdef DBT_PROBE_DSP_ASSIST
  /* One lock-free word only; never call flash-resident job code from ISR. */
  atomic_store(&f->dsp_assist_state,atomic_load_explicit(&assist.lane.state,memory_order_acquire));
#else
  atomic_store(&f->dsp_assist_state,0);
#endif
  atomic_store(&f->version,version+2u);atomic_store(&f->valid,1);
}
void probe_output_first_starvation(probe_output_starvation_t *s){
  if(!s)return;
  *s=(probe_output_starvation_t){0};
  starvation_capture_t *f=&timeline.first;uint32_t serial=atomic_load(&timeline.stream_serial);
  s->stream_serial=serial;s->coherent=1;
  if(!atomic_load(&f->valid)){
    s->coherent=serial==atomic_load(&timeline.stream_serial) && !atomic_load(&f->valid);return;
  }
  uint32_t before=atomic_load(&f->version);
  s->stream_serial=atomic_load(&f->stream_serial);
  s->at_us=atomic_load(&f->at_us);
  s->taken_frames=atomic_load(&f->taken_frames);
  s->input_frames=atomic_load(&f->input_frames);
  s->output_frames=atomic_load(&f->output_frames);
  s->accepted_frames=atomic_load(&f->accepted_frames);
  s->rendered_frames=atomic_load(&f->rendered_frames);
  s->dma_source_frames=atomic_load(&f->dma_source_frames);
  s->acl_us=atomic_load(&f->acl_us);
  s->host_poll_us=atomic_load(&f->host_poll_us);
  s->media_us=atomic_load(&f->media_us);
  s->media_admit_us=atomic_load(&f->media_admit_us);
  s->media_epoch=atomic_load(&f->media_epoch);
  s->media_sequence=atomic_load(&f->media_sequence);
  s->media_timestamp=atomic_load(&f->media_timestamp);
  s->media_ssrc=atomic_load(&f->media_ssrc);
  s->media_result=atomic_load(&f->media_result);
  s->media_gap_us=atomic_load(&f->media_gap_us);
  s->media_max_gap_us=atomic_load(&f->media_max_gap_us);
  s->media_coherent=atomic_load(&f->media_coherent);
  s->decoder_phase=atomic_load(&f->decoder_phase);
  s->decoder_at_us=atomic_load(&f->decoder_at_us);
  s->decoder_coherent=atomic_load(&f->decoder_coherent);
  s->render_phase=atomic_load(&f->render_phase);
  s->render_at_us=atomic_load(&f->render_at_us);
  s->render_coherent=atomic_load(&f->render_coherent);
  s->dsp_assist_state=atomic_load(&f->dsp_assist_state);
  s->decoder_queued=atomic_load(&f->decoder_queued);
  s->captured=atomic_load(&f->valid);
  s->coherent=s->captured && !(before&1u) && before==atomic_load(&f->version) &&
      serial==s->stream_serial && serial==atomic_load(&timeline.stream_serial);
}
#endif

#ifdef DBT_PROBE_CODEC_AWARE_RENDER
#define AUX_RENDER_STACK_BYTES 6144u
_Static_assert(ATOMIC_POINTER_LOCK_FREE==2,"Output owner needs lock-free handles");
_Static_assert(DBT_ESP32_RENDER_CORE==1,"Codec-aware output requires the CPU1 default");
static void render_loop(void *unused);
static bool owns_output(void){
  return atomic_load_explicit(&output.task,memory_order_acquire)==xTaskGetCurrentTaskHandle();
}
static bool ensure_auxiliary_render0(void){
  if(output.workers[0])return true;
  TaskHandle_t worker=NULL;
  if(xTaskCreatePinnedToCore(render_loop,"native_render0",AUX_RENDER_STACK_BYTES,0,
      DBT_PROBE_RENDER_PRIORITY,&worker,0)!=pdPASS){
    atomic_fetch_add(&output.auxiliary_allocation_failures,1);return false;
  }
  output.workers[0]=worker;atomic_store(&output.auxiliary_stack_bytes,AUX_RENDER_STACK_BYTES);
#ifdef DBT_PROBE_SCHEDULER_TRACE
  probe_scheduler_bind_renderer0(worker);
#endif
  return true;
}
#endif
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
    atomic_store(&output.allocation_failure_rate,output.allocation_request_rate);
    atomic_store(&output.allocation_failure_channels,output.allocation_request_channels);
    atomic_store(&output.allocation_failure_sample_bytes,output.allocation_request_sample_bytes);
    atomic_store(&output.allocation_failure_audio_requested,output.allocation_audio_requested);
    atomic_store(&output.allocation_failure_audio_allocated,output.allocation_success_bytes);
    atomic_store(&output.allocation_failure_pages,output.allocation_success_calls-
      (output.allocation_input_requested && output.allocation_success_calls?1u:0u));
    atomic_store(&output.allocation_failure_at_us,(uint32_t)esp_timer_get_time());
#ifdef ESP_PLATFORM
    atomic_store(&output.allocation_failure_free,heap_caps_get_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT));
    atomic_store(&output.allocation_failure_largest,heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT));
    /* These capability sets overlap; never add 8-bit and 32-bit totals.
     * Audio pages still require 8-bit memory. This only measures the pool. */
    atomic_store(&output.allocation_failure_free_32bit,heap_caps_get_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_32BIT));
    atomic_store(&output.allocation_failure_largest_32bit,heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL|MALLOC_CAP_32BIT));
#endif
  }else{
    output.allocation_success_bytes+=bytes;++output.allocation_success_calls;
  }
  return p;
}
static void release(void *context, uint8_t *bytes) {
  (void)context;
  heap_caps_free(bytes);
}
/* The working product's16384-byte Q31 ring has a1023-frame start target
 * and1024-frame producer watermark. Round this reserve to one AAC unit
 * at44.1/48k;16ms alone is
 * shorter than one AAC1024-frame access unit. Existing1536-frame output
 * pages already cover this watermark at both rates; no added audio memory. */
static uint32_t pcm_target_frames(uint32_t rate){
  uint32_t target=(rate*DBT_PROBE_PCM_TARGET_MS+999u)/1000u;
#ifdef DBT_PROBE_STAY_NATIVE
  if(target<1024u)target=1024u;
#endif
  return target;
}
#define PRIMING_ARMED 0x10000u
#define PRIMING_DONE 0x20000u
#define PRIMING_BUDGET 0xffffu
/* Single producer arms before notifying; only render retires the deadline.
 * DONE prevents an expired deadline from creating periodic one-tick work. */
static bool priming_ready(uint32_t now){
  uint32_t state=atomic_load(&output.priming_state),budget=state&PRIMING_BUDGET;
  if(!budget || (state&PRIMING_DONE))return true;
  if(!(state&PRIMING_ARMED) || now-atomic_load(&output.priming_at_ms)<budget)return false;
  atomic_store(&output.priming_state,state|PRIMING_DONE);return true;
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
#ifdef DBT_PROBE_PCM_LOANS
    if(output.inflight.kind==REQ_PCM_LOAN)OUTPUT_TEST_STEP(6);
#endif
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
uint32_t dbt_output_test_ready_pcm(void){return dbt_audio_output_frames(&output.audio);}
#endif
#ifdef DBT_PROBE_PCM_LOANS
static bool retire_pcm_loan(bool cancel){
  uint32_t id=output.pcm_loan_id;if(!id)return false;
  if(cancel)(void)dbt_audio_input_cancel(&output.audio);
  else if(!dbt_audio_input_release(&output.audio))return false;
  output.pcm_loan_id=0;output.inflight.pcm=NULL;atomic_store(&output.pcm_loan_active,0);
  uint32_t elapsed=(uint32_t)esp_timer_get_time()-output.pcm_loan_began;
  if(elapsed>atomic_load(&output.pcm_loan_max_us))atomic_store(&output.pcm_loan_max_us,elapsed);
  if(cancel)atomic_fetch_add(&output.pcm_loans_cancelled,1);
  else atomic_fetch_add(&output.pcm_loans_retired,1);
  report_done(id,cancel?DBT_CANCELLED:DBT_OK);return true;
}
static bool reclaim_pcm_loan(const request_t *request,dbt_result_t result){
  if(request->kind!=REQ_PCM_LOAN)return false;
  output.inflight.pcm=NULL;
  if(result==DBT_OK){(void)dbt_audio_input_cancel(&output.audio);output.pcm_loan_id=0;atomic_store(&output.pcm_loan_active,0);}
  return true;
}
static bool handle_pcm_loan(const request_t *request){
  if(request->kind!=REQ_PCM_LOAN)return false;
  bool unavailable=!output.active || output.stopping || atomic_load(&output.failed) || output.pcm_loan_id ||
      atomic_load_explicit(&output.restart_hold,memory_order_acquire);
#ifdef DBT_PROBE_SOUND_BUILD
  unavailable=unavailable || output.transition_sound || output.sound_standalone;
#endif
  if(unavailable || request->rate!=output.audio.config.rate || request->channels!=output.audio.config.channels ||
      request->format!=output.audio.config.format || !request->pcm || !request->frames || request->frames>PROBE_PCM_LOAN_FRAMES ||
      dbt_audio_input_borrow(&output.audio,request->pcm,request->frames)!=DBT_OK){
    output.inflight.pcm=NULL;
    report_done(request->id,DBT_CANCELLED);return true;
  }
  output.pcm_loan_id=request->id;output.pcm_loan_began=(uint32_t)esp_timer_get_time();
  atomic_store(&output.pcm_loan_active,1);atomic_fetch_add(&output.pcm_loans_begun,1);
  atomic_fetch_add(&output.accepted_frames,request->frames);return true;
}
#endif

/* A pending shortage belongs only to the most recent completed descriptor.
 * Every subsequent EOF closes it BEFORE consuming newer output, preserving
 * sample order even if the notification task could not run in time. */
static void IRAM_ATTR close_refill_window(output_t *o){
  uint32_t index=atomic_load_explicit(&o->refill_pending_index,memory_order_acquire);
  if(index==UINT32_MAX)return;
  if(index<DMA_DESCRIPTORS){
    uint32_t valid=atomic_load(&o->descriptors[index].valid);
    if(valid<=DMA_FRAMES)atomic_store(&o->refill_unfilled_frames,
        atomic_load(&o->refill_unfilled_frames)+DMA_FRAMES-valid);
    else atomic_store(&o->failed,1);
  }else atomic_store(&o->failed,1);
  atomic_store_explicit(&o->refill_pending_index,UINT32_MAX,memory_order_release);
}
/* Render owner only, same CPU as the ISR. A verified three-descriptor ring
 * gives at least one conservative buffer period after a fresh normal EOF.
 * No newer EOF is allowed to leave an older window open. The local mask never
 * waits for another CPU; only a <=1024-byte copy and atomic metadata run here. */
static bool IRAM_ATTR service_refill_window(void){
#ifdef DBT_PROBE_STAY_NATIVE
  if(atomic_load(&output.refill_pending_index)==UINT32_MAX ||
     atomic_load(&output.failed) || atomic_load(&output.stop_requested) ||
     atomic_load(&output.callback_core)!=(uint32_t)xPortGetCoreID())return false;
#ifdef __XTENSA__
  uint32_t mask=XTOS_SET_MIN_INTLEVEL(5); /* Also exclude the ordinary high-level IPC interrupt during this bounded copy. */
#endif
  uint32_t mask_began=(uint32_t)esp_timer_get_time();
  bool copied=false;
  uint32_t index=atomic_load_explicit(&output.refill_pending_index,memory_order_acquire);
  uint32_t now=(uint32_t)esp_timer_get_time();
  uint32_t deadline=atomic_load(&output.refill_deadline_us);
  if(index<DMA_DESCRIPTORS && !atomic_load(&output.failed) && !atomic_load(&output.stop_requested) &&
     (int32_t)(deadline-now)>(int32_t)REFILL_COPY_BUDGET_US){
    descriptor_t *d=&output.descriptors[index];uint32_t valid=atomic_load(&d->valid);
    uint32_t missing=valid<=DMA_FRAMES?DMA_FRAMES-valid:0;
    if(missing && dbt_audio_output_frames(&output.audio)>=missing){
      uint32_t began=(uint32_t)esp_timer_get_time();
      int32_t *buffer=d->address;
      uint32_t taken=dbt_audio_take(&output.audio,buffer+2u*valid,missing);
      uint32_t ended=(uint32_t)esp_timer_get_time(),elapsed=ended-began;
      atomic_store(&d->valid,valid+taken);
      atomic_store(&output.refill_recovered_frames,atomic_load(&output.refill_recovered_frames)+taken);
      atomic_store(&output.refill_recovered_calls,atomic_load(&output.refill_recovered_calls)+1u);
      if(elapsed>atomic_load(&output.refill_copy_max_us))atomic_store(&output.refill_copy_max_us,elapsed);
      if(taken==missing){
        atomic_store(&output.held_left,buffer[2u*(DMA_FRAMES-1u)]);
        atomic_store(&output.held_right,buffer[2u*(DMA_FRAMES-1u)+1u]);
        atomic_store_explicit(&output.refill_pending_index,UINT32_MAX,memory_order_release);
      }
      if(taken!=missing || elapsed>REFILL_COPY_BUDGET_US || (int32_t)(ended-deadline)>=0){
        atomic_store(&output.refill_contract_errors,atomic_load(&output.refill_contract_errors)+1u);
        atomic_store(&output.failed,1);
      }
      copied=taken!=0;
    }
  }
  uint32_t masked=(uint32_t)esp_timer_get_time()-mask_began;
  if(masked>atomic_load(&output.refill_mask_max_us))atomic_store(&output.refill_mask_max_us,masked);
  if(masked>REFILL_GUARD_US){atomic_store(&output.refill_contract_errors,atomic_load(&output.refill_contract_errors)+1u);atomic_store(&output.failed,1);}
#ifdef __XTENSA__
  XTOS_RESTORE_JUST_INTLEVEL(mask);
#endif
  return copied;
#else
  return false;
#endif
}
static bool IRAM_ATTR finish_sent(output_t *o,uint32_t began,BaseType_t wake){
  uint32_t elapsed=(uint32_t)esp_timer_get_time()-began;
  if(!atomic_load(&o->callback_calls)){
#ifdef ESP_PLATFORM
    atomic_store(&o->callback_core,(uint32_t)xPortGetCoreID());
#else
    atomic_store(&o->callback_core,0);
#endif
  }
  atomic_store(&o->callback_calls,atomic_load(&o->callback_calls)+1u);
  atomic_store(&o->callback_elapsed_us,atomic_load(&o->callback_elapsed_us)+elapsed);
  if(elapsed>atomic_load(&o->callback_max_us))atomic_store(&o->callback_max_us,elapsed);
  return wake==pdTRUE;
}
static bool IRAM_ATTR sent(i2s_chan_handle_t channel, i2s_event_data_t *event,
                           void *context) {
  (void)channel;
  output_t *o = context;
  BaseType_t wake = pdFALSE;
  uint32_t now = (uint32_t)esp_timer_get_time();
  uint32_t gap = o->last_eof?now-o->last_eof:0;
  if (!event->dma_buf || event->size != DMA_FRAMES * 8u) {
    atomic_store(&o->descriptor_fault_at_us,now);
    atomic_store(&o->descriptor_fault_gap_us,gap);
    atomic_store(&o->descriptor_fault_stopping,atomic_load(&o->stop_requested)!=0);
    atomic_store(&o->descriptor_shape_errors,atomic_load(&o->descriptor_shape_errors)+1u);
    atomic_store(&o->descriptor_expected_index,o->next_descriptor);
    atomic_store(&o->descriptor_observed_index,UINT32_MAX);
    atomic_store(&o->descriptor_observed_size,event->size);
    atomic_store(&o->descriptor_errors,
                 atomic_load(&o->descriptor_errors) + 1u);
    atomic_store(&o->failed, 1);
    vTaskNotifyGiveFromISR(o->task,&wake);
    return finish_sent(o,now,wake);
  }
  close_refill_window(o);
  if (o->last_eof) {
    if (gap > atomic_load(&o->max_eof_gap_us))
      atomic_store(&o->max_eof_gap_us, gap);
  }
  o->last_eof = now;
  unsigned index = 0;
  while (index < o->descriptor_count &&
         o->descriptors[index].address != event->dma_buf)
    ++index;
  bool known=index<o->descriptor_count;
  if (index == o->descriptor_count) {
    if (index == DMA_DESCRIPTORS) {
      atomic_store(&o->descriptor_fault_at_us,now);
      atomic_store(&o->descriptor_fault_gap_us,gap);
      atomic_store(&o->descriptor_fault_stopping,atomic_load(&o->stop_requested)!=0);
      atomic_store(&o->descriptor_pool_errors,atomic_load(&o->descriptor_pool_errors)+1u);
      atomic_store(&o->descriptor_expected_index,o->next_descriptor);
      atomic_store(&o->descriptor_observed_index,index);
      atomic_store(&o->descriptor_observed_size,event->size);
      atomic_store(&o->descriptor_errors,
                   atomic_load(&o->descriptor_errors) + 1u);
      atomic_store(&o->failed, 1);
      vTaskNotifyGiveFromISR(o->task,&wake);
      return finish_sent(o,now,wake);
    }
    o->descriptors[index] = (descriptor_t){event->dma_buf, DMA_FRAMES, false};
    ++o->descriptor_count;
  }
  if (index != o->next_descriptor) {
    atomic_store(&o->descriptor_fault_at_us,now);
    atomic_store(&o->descriptor_fault_gap_us,gap);
    atomic_store(&o->descriptor_fault_stopping,atomic_load(&o->stop_requested)!=0);
    atomic_store(&o->descriptor_order_errors,atomic_load(&o->descriptor_order_errors)+1u);
    atomic_store(&o->descriptor_expected_index,o->next_descriptor);
    atomic_store(&o->descriptor_observed_index,index);
    atomic_store(&o->descriptor_observed_size,event->size);
    atomic_store(&o->descriptor_errors,
                 atomic_load(&o->descriptor_errors) + 1u);
    if(known && o->descriptor_count==DMA_DESCRIPTORS &&
       o->descriptor_order_seen==DMA_DESCRIPTORS && o->next_descriptor<DMA_DESCRIPTORS &&
       !atomic_load(&o->failed)){
      /* ESP32 reports the latest completed descriptor once per ISR. Coalesced
       * EOFs need not visit each index. Require a verified stable ring first.
       * Pending valid counts are DMA-owned copies, not borrowed PCM storage.
       * Credit skipped copies once; extra complete laps remain unknown. */
      uint32_t missed=0,recovered=0;unsigned prior=o->next_descriptor;
      while(prior!=index && missed<DMA_DESCRIPTORS){
        recovered+=o->descriptors[prior].valid;
        o->descriptors[prior].valid=0;
        ++missed;if(++prior==DMA_DESCRIPTORS)prior=0;
      }
      atomic_store(&o->descriptor_order_recoveries,atomic_load(&o->descriptor_order_recoveries)+1u);
      atomic_store(&o->descriptor_missed_eofs_min,atomic_load(&o->descriptor_missed_eofs_min)+missed);
      atomic_store(&o->descriptor_recovered_source_frames,atomic_load(&o->descriptor_recovered_source_frames)+recovered);
      atomic_store(&o->dma_source_frames,atomic_load(&o->dma_source_frames)+recovered);
      /* Stop requires a fresh complete consecutive silent traversal. */
      o->zero_completions=0;
    }else atomic_store(&o->failed,1);
  }else if(known && o->descriptor_count==DMA_DESCRIPTORS &&
           o->descriptor_order_seen<DMA_DESCRIPTORS)++o->descriptor_order_seen;
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
    int32_t held_left=atomic_load(&o->held_left),held_right=atomic_load(&o->held_right);
    for (unsigned i = taken; i < wanted; ++i) {
      buffer[2 * i] = held_left;
      buffer[2 * i + 1u] = held_right;
    }
    atomic_store(&o->stop_fill_frames,
                 atomic_load(&o->stop_fill_frames) + wanted - taken);
  } else {
    if (taken < DMA_FRAMES && !atomic_load(&o->underrun_frames)) {
#ifdef DBT_PROBE_SCHEDULER_TRACE
      capture_starvation(o,now,taken);
#endif
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
  if(!stopping && taken<DMA_FRAMES){
    uint32_t period=atomic_load(&o->refill_period_us);
    bool eligible=false;
#ifdef DBT_PROBE_STAY_NATIVE
    eligible=known && o->descriptor_count==DMA_DESCRIPTORS &&
        o->descriptor_order_seen==DMA_DESCRIPTORS && !atomic_load(&o->descriptor_errors) &&
        !atomic_load(&o->failed) && period>REFILL_GUARD_US+REFILL_COPY_BUDGET_US &&
        gap>=period/2u && gap<=period+REFILL_GUARD_US &&
        atomic_load(&o->render_core)==(uint32_t)xPortGetCoreID();
#endif
    if(eligible){
      atomic_store(&o->refill_deadline_us,now+period-REFILL_GUARD_US);
      atomic_store_explicit(&o->refill_pending_index,index,memory_order_release);
    }else atomic_store(&o->refill_unfilled_frames,atomic_load(&o->refill_unfilled_frames)+DMA_FRAMES-taken);
  }
  vTaskNotifyGiveFromISR(o->task, &wake);
  return finish_sent(o,now,wake);
}
static void close_output(void) {
#ifdef DBT_PROBE_PCM_LOANS
  (void)retire_pcm_loan(true);
#endif
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
  uint32_t pending=atomic_exchange_explicit(&output.refill_pending_index,UINT32_MAX,memory_order_acq_rel);
  if(pending<DMA_DESCRIPTORS){
    uint32_t valid=atomic_load(&output.descriptors[pending].valid);
    if(valid<=DMA_FRAMES)atomic_store(&output.refill_canceled_frames,atomic_load(&output.refill_canceled_frames)+DMA_FRAMES-valid);
  }
  /* I2S operations and its ISR belong to this pinned render core. After
   * disable/delete no callback can run while its owner frees the SRAM rings. */
  dbt_audio_close(&output.audio);
  output.active = output.enabled = output.stopping = false;
  atomic_store(&output.meter_value, 0);
  for (unsigned i = 0; i < 3; ++i)
    output.meter_db[i] = -60 * 256;
  atomic_store(&output.running, 0);
  atomic_store(&output.requested_bytes, 0);
  atomic_store(&output.input_capacity_frames,0);atomic_store(&output.input_ring_bytes,0);atomic_store(&output.actual_target_frames,0);
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
  channel.intr_priority = DMA_IRQ_PRIORITY;
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
  atomic_store(&output.allocation_failure_rate,0);atomic_store(&output.allocation_failure_channels,0);
  atomic_store(&output.allocation_failure_sample_bytes,0);atomic_store(&output.allocation_failure_at_us,0);
  atomic_store(&output.allocation_failure_audio_requested,0);atomic_store(&output.allocation_failure_audio_allocated,0);atomic_store(&output.allocation_failure_pages,0);
  atomic_store(&output.allocation_failure_free_after_cleanup,0);atomic_store(&output.allocation_failure_free_32bit,0);atomic_store(&output.allocation_failure_largest_32bit,0);
#ifdef ESP_PLATFORM
  atomic_store(&output.allocation_failure_free_before,heap_caps_get_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT));
#endif
  output.allocation_success_bytes=output.allocation_success_calls=0;
  output.allocation_request_rate=rate;output.allocation_request_channels=channels;output.allocation_request_sample_bytes=(uint32_t)format;
  uint32_t target = pcm_target_frames(rate);
  uint32_t input_frames=power2((rate * 32u + 999u) / 1000u),avoided=0;
#ifdef DBT_PROBE_PCM_COPIED
  input_frames=power2((rate*4u+999u)/1000u);
  /* Positive clock correction can retain one source frame while waiting for
   * a two-source-frame advance. A complete decoder chunk must still fit:
   * capacity==chunk would leave 255 free for a 256-frame submission forever. */
  if(input_frames<=PROBE_PCM_TRANSFER_FRAMES)
    input_frames=power2(PROBE_PCM_TRANSFER_FRAMES+1u);
#endif
#ifdef DBT_PROBE_HIGH_RATE_PCM_REBALANCE
  /* Match the baseline two-frame decoded reserve without adding PCM bytes.
   * S32 stereo uses8 bytes per frame on both sides; move1024 frames from
   * the processed reserve into the decoded ring. Page rounding stays exact. */
  if(rate==192000u && channels==2u && format==DBT_PCM_S32_LE && input_frames==1024u && target>=2048u){
    input_frames=2048u;target-=1024u;
  }
#endif
  atomic_store(&output.input_ring_bytes_avoided,0);
#ifdef DBT_PROBE_PCM_LOANS
  avoided=input_frames*(uint32_t)format*channels;input_frames=PROBE_PCM_LOAN_FRAMES;
#endif
  dbt_audio_config_t config = {
      rate,
      input_frames,
      /* The renderer never produces beyond target+DMA prefill. Reserve the
       * existing extra 128 frames, then round only to its allocation page. */
      ((target + DMA_DESCRIPTORS * DMA_FRAMES + 128u + DBT_AUDIO_OUTPUT_PAGE_FRAMES-1u)/DBT_AUDIO_OUTPUT_PAGE_FRAMES)*DBT_AUDIO_OUTPUT_PAGE_FRAMES,
      target,
      format,
      channels,
      OUTPUT_ADAPTIVE_CLOCK};
  output.allocation_input_requested=avoided?0:input_frames*(uint32_t)format*channels;
  output.allocation_audio_requested=output.allocation_input_requested+config.output_frames*8u;
  dbt_result_t result =
#ifdef DBT_PROBE_PCM_LOANS
      dbt_audio_open_borrowed(&output.audio,config);
#else
      dbt_audio_open(&output.audio, config);
#endif
  if (result != DBT_OK){
#ifdef ESP_PLATFORM
    if(atomic_load(&output.allocation_failed_bytes))
      atomic_store(&output.allocation_failure_free_after_cleanup,heap_caps_get_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT));
#endif
    return result;
  }
  if (create_channel(rate, true) != ESP_OK &&
      create_channel(rate, false) != ESP_OK) {
    close_output();
    return DBT_INVALID;
  }
  for (unsigned i = 0; i < DMA_DESCRIPTORS; ++i)
    output.descriptors[i] = (descriptor_t){0};
  output.descriptor_count = output.next_descriptor = output.zero_completions =
      output.last_eof = 0;
  output.descriptor_order_seen=0;
  output.held_left = output.held_right = 0;
  output.clock_at = (uint32_t)(esp_timer_get_time() / 1000);
  dbt_audio_fade_init(&output.fade, rate / 50u);
  atomic_store(&output.stop_requested, 0);
  atomic_store(&output.stop_finished, 0);
  atomic_store(&output.failed, 0);
  atomic_store(&output.rate, rate);
  atomic_store(&output.accepted_frames, 0);
  atomic_store(&output.priming_state,0);atomic_store(&output.priming_at_ms,0);
  atomic_store(&output.rendered_frames, 0);
  atomic_store(&output.dma_eofs, 0);
  atomic_store(&output.dma_source_frames, 0);
  atomic_store(&output.underrun_frames, 0);
#ifdef DBT_PROBE_SCHEDULER_TRACE
  atomic_store(&timeline.first.valid,0);
  atomic_store(&timeline.stream_serial,atomic_load(&timeline.stream_serial)+1u);
#endif
  atomic_store(&output.descriptor_errors, 0);
  atomic_store(&output.descriptor_shape_errors,0);
  atomic_store(&output.descriptor_pool_errors,0);
  atomic_store(&output.descriptor_order_errors,0);
  atomic_store(&output.descriptor_order_recoveries,0);
  atomic_store(&output.descriptor_missed_eofs_min,0);
  atomic_store(&output.descriptor_recovered_source_frames,0);
  atomic_store(&output.descriptor_expected_index,0);
  atomic_store(&output.descriptor_observed_index,0);
  atomic_store(&output.descriptor_observed_size,0);
  atomic_store(&output.descriptor_fault_at_us,0);
  atomic_store(&output.descriptor_fault_gap_us,0);
  atomic_store(&output.descriptor_fault_stopping,0);
  atomic_store(&output.callback_calls,0);atomic_store(&output.callback_elapsed_us,0);
  atomic_store(&output.callback_max_us,0);atomic_store(&output.callback_core,UINT32_MAX);
  atomic_store(&output.max_eof_gap_us, 0);
  atomic_store(&output.maximum_render_us, 0);
  atomic_store(&output.render_total_us,0);
  atomic_store(&output.render_calls,0);
  atomic_store(&output.input_ring_bytes_avoided,avoided);
  atomic_store(&output.input_capacity_frames,config.input_frames);
  atomic_store(&output.actual_target_frames,config.target_frames);
  atomic_store(&output.input_ring_bytes,output.audio.borrowed_input?0:config.input_frames*(uint32_t)format*channels);
  atomic_store(&output.pcm_loans_begun,0);atomic_store(&output.pcm_loans_retired,0);atomic_store(&output.pcm_loans_cancelled,0);
  atomic_store(&output.pcm_loan_active,0);atomic_store(&output.pcm_loan_max_us,0);
  atomic_store(&output.stop_fill_frames, 0);
  atomic_store(&output.requested_bytes, output.audio.allocated_bytes +
                                            DMA_DESCRIPTORS * DMA_FRAMES * 8u);
  atomic_store(&output.refill_period_us,DMA_FRAMES*1000000u/rate);
  atomic_store(&output.refill_pending_index,UINT32_MAX);atomic_store(&output.refill_deadline_us,0);
  atomic_store(&output.refill_recovered_frames,0);atomic_store(&output.refill_recovered_calls,0);
  atomic_store(&output.refill_unfilled_frames,0);atomic_store(&output.refill_canceled_frames,0);
  atomic_store(&output.refill_copy_max_us,0);atomic_store(&output.refill_mask_max_us,0);atomic_store(&output.refill_contract_errors,0);
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
#ifdef DBT_PROBE_CODEC_AWARE_RENDER
/* Only a closed I2S object can move. Inflight remains held by its ticket. */
static bool transfer_output_request(const request_t *request){
  bool creates=(request->kind==REQ_CODEC && request->start) || request->kind==REQ_SOUND_RESERVE;
  if(!creates || output.active || output.channel || output.stopping ||
     atomic_load_explicit(&output.restart_hold,memory_order_acquire))return false;
  unsigned core=request->kind==REQ_CODEC && request->share_decoder_core?0u:DBT_ESP32_RENDER_CORE;
  if((unsigned)xPortGetCoreID()==core)return false;
  if(!output.workers[core]){
    if(core!=0){atomic_fetch_add(&output.auxiliary_allocation_failures,1);report_done(request->id,DBT_NO_SPACE);return true;}
    if(!ensure_auxiliary_render0()){report_done(request->id,DBT_NO_SPACE);return true;}
  }
  TaskHandle_t target=output.workers[core];
  atomic_store_explicit(&output.transfer_pending,true,memory_order_release);
  atomic_fetch_add(&output.owner_transfers,1);
  atomic_store_explicit(&output.task,target,memory_order_release);
  xTaskNotifyGive(target);return true;
}
#endif
static dbt_result_t start_codec_output(request_t request){
#ifdef DBT_PROBE_DSP_ASSIST
  if(request.rate>96000u && request.format==DBT_PCM_S32_LE && request.channels==2 &&
     !ensure_auxiliary_render0())return DBT_NO_SPACE;
#endif
  dbt_result_t result=start_output(request.rate,request.channels,request.format);
  if(result==DBT_OK)atomic_store(&output.priming_state,request.priming_ms);
  return result;
}
static void codec_request(request_t request){
#ifdef DBT_PROBE_CODEC_AWARE_RENDER
  if(transfer_output_request(&request))return;
#endif
  if (request.start && atomic_load_explicit(&output.restart_hold,memory_order_acquire))report_done(request.id,DBT_BUSY);
  else if (output.stopping)report_done(request.id, DBT_BUSY);
  else if (request.start){
#ifdef DBT_PROBE_SOUND_BUILD
    if(!output.active)output.sound_standalone=false;
#endif
    report_done(request.id,start_codec_output(request));
  }
  else if (!output.active){close_output();report_done(request.id,DBT_OK);}
  else {
    if(!output.enabled){
      /* A stop bypasses only the new guard, draining a safely preloadable
       * accepted clip. The prior sub-DMA short-clip behavior is unchanged. */
      if(!(atomic_load(&output.priming_state)&PRIMING_BUDGET) ||
         dbt_audio_output_frames(&output.audio)<DMA_DESCRIPTORS*DMA_FRAMES){
        close_output();report_done(request.id,DBT_OK);return;
      }
      if(enable_output()!=DBT_OK){close_output();atomic_store(&output.failed,1);report_done(request.id,DBT_INVALID);return;}
    }
    dbt_audio_begin_drain(&output.audio);output.stopping=true;output.stop_request=request.id;
    output.stop_at=(uint32_t)(esp_timer_get_time()/1000);
    atomic_store_explicit(&output.stop_requested,1,memory_order_release);
  }
}
#ifdef DBT_PROBE_SOUND_BUILD
static void mix_sound(void *p,int32_t *pcm,uint32_t frames){dbt_sound_mix(p,pcm,(uint16_t)frames);}
static void retire_sound(void){
  dbt_audio_set_mix(&output.audio,0,0);output.sound=0;
  uint32_t token=atomic_exchange_explicit(&output.sound_token,0,memory_order_acq_rel);
  if(token)atomic_store_explicit(&output.sound_retired,token,memory_order_release);
  if(token)notify_sound();
}
static void reclaim_cancelled(const request_t *request,dbt_result_t result){
#ifdef DBT_PROBE_PCM_LOANS
  if(reclaim_pcm_loan(request,result))return;
#endif
  if(result!=DBT_OK)return;
  if(request->kind==REQ_CODEC && request->start){close_output();return;}
  uint32_t token=atomic_load_explicit(&output.sound_token,memory_order_acquire);
  if((request->kind==REQ_SOUND_RESERVE && token==request->id) ||
     (request->kind==REQ_SOUND_ATTACH && token==request->sound_token && output.sound==request->sound)){
    bool standalone=output.sound_standalone;retire_sound();if(standalone)close_output();
  }
}
static void handle_request(request_t request){
#ifdef DBT_PROBE_CODEC_AWARE_RENDER
  if(transfer_output_request(&request))return;
#endif
#ifdef DBT_PROBE_PCM_LOANS
  if(handle_pcm_loan(&request))return;
#endif
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
static void reclaim_cancelled(const request_t *request,dbt_result_t result){
#ifdef DBT_PROBE_PCM_LOANS
  if(reclaim_pcm_loan(request,result))return;
#endif
  if(result==DBT_OK && request->start)close_output();
}
static void handle_request(request_t request){
#ifdef DBT_PROBE_CODEC_AWARE_RENDER
  if(transfer_output_request(&request))return;
#endif
#ifdef DBT_PROBE_PCM_LOANS
  if(handle_pcm_loan(&request))return;
#endif
  codec_request(request);
}
#endif
static void consume_request(request_t request){
  unsigned state=dbt_ticket_state(&output.ticket,request.id);
  if(state==DBT_TICKET_CANCEL)report_done(request.id,DBT_BUSY);
  else if(state==DBT_TICKET_WAITING)handle_request(request);
  else atomic_fetch_add(&output.stale_receipts,1);
}
static void publish_dsp_quality(void){
    if(!atomic_load_explicit(&output.dsp_requested,memory_order_acquire))return;
    atomic_store_explicit(&output.dsp_requested,0,memory_order_release);
    uint32_t generation=atomic_load_explicit(&output.dsp_generation,memory_order_relaxed);
    atomic_store_explicit(&output.dsp_generation,generation+1u,memory_order_release);
    atomic_thread_fence(memory_order_seq_cst);
    const fixed_dsp_t *d=&output.audio.dsp;bool open=output.audio.opened;
    uint32_t gains=open?((uint32_t)(uint8_t)d->gains[0]|((uint32_t)(uint8_t)d->gains[1]<<8)|((uint32_t)(uint8_t)d->gains[2]<<16)):0;
    atomic_store_explicit(&output.dsp_at_us,(uint32_t)esp_timer_get_time(),memory_order_relaxed);
    atomic_store_explicit(&output.dsp_open,open,memory_order_relaxed);
    atomic_store_explicit(&output.dsp_rate,open?output.audio.config.rate:0,memory_order_relaxed);
    atomic_store_explicit(&output.dsp_gains,gains,memory_order_relaxed);
    atomic_store_explicit(&output.dsp_options,open?((uint32_t)d->flags|((uint32_t)d->requested_volume<<8)):0,memory_order_relaxed);
    atomic_store_explicit(&output.dsp_peak_q27,open?d->peak_q27:0,memory_order_relaxed);
    atomic_store_explicit(&output.dsp_limited,open?d->limited:0,memory_order_relaxed);
    atomic_store_explicit(&output.dsp_over_full_scale,open?d->over_full_scale:0,memory_order_relaxed);
    atomic_store_explicit(&output.dsp_frames,open?atomic_load(&output.audio.rendered_frames):0,memory_order_relaxed);
    atomic_store_explicit(&output.dsp_generation,generation+2u,memory_order_release);
}
#ifdef DBT_PROBE_EARLY_INPUT_RELEASE
static void copied_input_ready(void *context){
  (void)context;
  if(output.wake_decoder)output.wake_decoder(output.wake_context);
  OUTPUT_TEST_STEP(10); /* test-only hold after input release, before DSP */
}
#endif
static void render_completed(uint32_t began,bool input_wake_owned){
  uint32_t elapsed=(uint32_t)esp_timer_get_time()-began;
  if(elapsed>atomic_load(&output.maximum_render_us))atomic_store(&output.maximum_render_us,elapsed);
  atomic_fetch_add(&output.render_total_us,elapsed);atomic_fetch_add(&output.render_calls,1);
  atomic_store(&output.rendered_frames,atomic_load(&output.audio.rendered_frames));
  (void)service_refill_window();
  if(!input_wake_owned)notify_clients();
#ifdef DBT_PROBE_SOUND_BUILD
  else notify_sound(); /* Mixer space is released only after DSP/mixing. */
#endif
}
#ifdef DBT_PROBE_DSP_ASSIST
static void assist_add(_Atomic uint32_t *value,uint32_t n){atomic_store(value,atomic_load(value)+n);}
static void assist_max(_Atomic uint32_t *value,uint32_t n){if(n>atomic_load(value))atomic_store(value,n);}
static uint32_t assist_cycles(void *context){
  (void)context;OUTPUT_TEST_STEP(4);
#ifdef __XTENSA__
  return (uint32_t)XTHAL_GET_CCOUNT();
#else
  return 0;
#endif
}
static uint32_t assist_time_us(void *context){(void)context;return (uint32_t)esp_timer_get_time();}
/* At most four canonical blocks per lease. Publish every completed span
 * immediately, then return on empty input/output or any error. This is bounded
 * work, never a wait for PCM. Core1 retains lifecycle ownership through DONE. */
static dbt_result_t dsp_assist_commit(void *context,const dbt_audio_render_job_t *first,bool success){
  (void)context;
  dbt_audio_render_job_t next;const dbt_audio_render_job_t *block=first;
  uint32_t math_end=assist.lane.finished_us;
  for(unsigned i=0;i<4u;++i){
    uint32_t made=0;
    dbt_result_t result=dbt_audio_render_finish(&output.audio,block,success,&made);
    assist.committed_us=(uint32_t)esp_timer_get_time();
    uint32_t delay=assist.committed_us-math_end;
    if(delay>assist.commit_delay_max_us)assist.commit_delay_max_us=delay;
    if(result!=DBT_OK)return result;
    assist.committed_frames+=made;++assist.completed_blocks;
    if(i==3u || assist.committed_frames>=assist.budget)break;
    result=dbt_audio_render_prepare(&output.audio,assist.budget-assist.committed_frames,&next);
    if(result==DBT_EMPTY)break;
    if(result!=DBT_OK)return result;
    block=&next;
    uint32_t cycles=assist_cycles(NULL);
    success=fixed_dsp_process_block(block->dsp,block->pcm,(uint16_t)block->frames);
    cycles=assist_cycles(NULL)-cycles;
    assist.extra_cycles+=cycles;
    if(cycles>assist.block_max_cycles)assist.block_max_cycles=cycles;
    math_end=(uint32_t)esp_timer_get_time();
  }
  OUTPUT_TEST_STEP(8);
#ifdef DBT_PROBE_SCHEDULER_TRACE
  probe_scheduler_mark(1,&assist.committed_mark);
#endif
  return DBT_OK;
}
static bool dsp_assist_work(void){
  if((unsigned)xPortGetCoreID()!=0u)return false;
  if(!dbt_dsp_job_work_commit(&assist.lane,(dbt_dsp_job_clock_t){NULL,assist_cycles,assist_time_us},dsp_assist_commit,NULL))return false;
  /* PCM/input publications precede DONE. Wake the producer immediately;
   * it must not wait for a busy Core1 merely to observe available space. */
  notify_clients();
  TaskHandle_t owner=atomic_load_explicit(&output.task,memory_order_acquire);
  if(owner)xTaskNotifyGive(owner);
  return true;
}
/* Return1 for a completed bounded transaction, -1 while borrowed, 0 idle.
 * This runs before every raw DSP read or lifecycle/control consumption. */
static int dsp_assist_complete(void){
  dbt_dsp_job_state_t state=dbt_dsp_job_state(&assist.lane);
  if(state==DBT_DSP_JOB_IDLE)return 0;
  if(state!=DBT_DSP_JOB_DONE)return -1;
  const dbt_audio_render_job_t *block=NULL;uint32_t cycles=0,start=0,end=0;
  dbt_result_t result=dbt_dsp_job_poll(&assist.lane,&block,&cycles,&start,&end);
  if(!block){atomic_store(&output.failed,1);assist_add(&assist.failures,1);return -1;}
  uint32_t token=block->token;
  uint32_t made=assist.committed_frames;
  dbt_result_t retired=dbt_dsp_job_retire(&assist.lane,token);
  uint32_t now=(uint32_t)esp_timer_get_time();
#ifdef DBT_PROBE_SCHEDULER_TRACE
  if((unsigned)xPortGetCoreID()==1u){
    probe_scheduler_mark_t end_mark;probe_scheduler_mark(1,&end_mark);
    probe_scheduler_interval_observe(PROBE_DELAY_ASSIST_RETIRE,&assist.committed_mark,&end_mark,
        assist.committed_us,now,made,atomic_load(&timeline.stream_serial),atomic_load(&output.rate));
  }
#endif
  assist_add(&assist.jobs,1);assist_add(&assist.blocks,assist.completed_blocks);assist_add(&assist.frames,made);
  assist_add(&assist.cycles,cycles+assist.extra_cycles);
  assist_max(&assist.max_cycles,cycles>assist.block_max_cycles?cycles:assist.block_max_cycles);
  assist_max(&assist.max_job_frames,made);assist_max(&assist.queue_max_us,start-assist.submitted_us);
  assist_max(&assist.finish_wait_max_us,assist.commit_delay_max_us);
  assist_max(&assist.retire_wait_max_us,now-assist.committed_us);assist_max(&assist.wall_max_us,now-assist.began_us);
  if(result!=DBT_OK || retired!=DBT_OK || !made || made>assist.budget){
    atomic_store(&output.failed,1);assist_add(&assist.failures,1);notify_clients();
    return retired==DBT_OK?1:-1;
  }
#ifdef DBT_PROBE_SOUND_BUILD
  if(output.sound_silence){output.clock_at=(uint32_t)(esp_timer_get_time()/1000);dbt_audio_clock_reset(&output.audio,output.clock_at);output.sound_silence=false;}
#endif
  /* Include dispatch/wait/retirement in the existing render wall record. */
  render_completed(assist.began_us,false);return 1;
}
static bool dsp_assist_submit(uint32_t budget,uint32_t began){
  if(!budget || (unsigned)xPortGetCoreID()!=1u || !output.workers[0] ||
     atomic_load(&output.rate)<=96000u || atomic_load(&output.restart_hold) ||
     dbt_dsp_job_state(&assist.lane)!=DBT_DSP_JOB_IDLE || !probe_audio_dsp_assist_ready())return false;
  dbt_audio_render_job_t block;dbt_result_t result=dbt_audio_render_prepare(&output.audio,budget,&block);
  if(result==DBT_EMPTY || result==DBT_UNSUPPORTED)return false;
  if(result!=DBT_OK){atomic_store(&output.failed,1);assist_add(&assist.failures,1);notify_clients();return true;}
  assist.began_us=began;assist.submitted_us=(uint32_t)esp_timer_get_time();
  assist.committed_frames=assist.committed_us=assist.extra_cycles=assist.block_max_cycles=0;
  assist.commit_delay_max_us=assist.completed_blocks=0;assist.budget=budget>512u?512u:budget;
  if(dbt_dsp_job_submit(&assist.lane,&block)!=DBT_OK){
    /* Submit transferred no borrow. Retire the matching preparation as a
     * failed stream, never run local math over an ambiguously owned block. */
    (void)dbt_audio_render_finish(&output.audio,&block,false,NULL);
    atomic_store(&output.failed,1);assist_add(&assist.failures,1);notify_clients();return true;
  }
  xTaskNotifyGive(output.workers[0]);return true;
}
#endif
void probe_output_dsp_assist_snapshot(probe_dsp_assist_stats_t *stats){
  if(!stats)return;
  *stats=(probe_dsp_assist_stats_t){0};
#ifdef DBT_PROBE_DSP_ASSIST
  stats->enabled=1;stats->state=(uint32_t)dbt_dsp_job_state(&assist.lane);
#define ASSIST_LOAD(name) stats->name=atomic_load(&assist.name)
  ASSIST_LOAD(jobs);ASSIST_LOAD(blocks);ASSIST_LOAD(frames);ASSIST_LOAD(cycles);ASSIST_LOAD(max_cycles);ASSIST_LOAD(max_job_frames);
  ASSIST_LOAD(queue_max_us);ASSIST_LOAD(finish_wait_max_us);ASSIST_LOAD(retire_wait_max_us);ASSIST_LOAD(wall_max_us);ASSIST_LOAD(failures);
#undef ASSIST_LOAD
#endif
}

static bool render_step(void) {
#ifdef DBT_PROBE_CODEC_AWARE_RENDER
    if(!owns_output())return false;
#endif
#ifdef DBT_PROBE_DSP_ASSIST
    int assisted=dsp_assist_complete();
    if(assisted<0)return false; /* READY/BUSY retains every DSP/lifecycle borrow. */
    /* DONE is only bookkeeping. Service controls/refill and at most one local
     * block before yielding; let Core0 resume decoding before another lease. */
#endif
    publish_dsp_quality();
#ifdef DBT_PROBE_SOUND_BUILD
    bool made_silence=false,blocked_sound=false;
#endif
    uint32_t tag=atomic_load_explicit(&output.ticket.tag,memory_order_acquire);
    if((tag&7u)==DBT_TICKET_RECLAIM){
      finish_cancel(tag>>3,(dbt_result_t)atomic_load_explicit(&output.ticket.result,memory_order_relaxed));
      return true;
    }
    request_t request;
    if(xQueueReceive(output.requests,&request,0)==pdTRUE)consume_request(request);
#ifdef DBT_PROBE_CODEC_AWARE_RENDER
    if(!owns_output())return false;
#endif
#ifdef DBT_PROBE_PCM_LOANS
    if(output.pcm_loan_id){
      unsigned state=dbt_ticket_state(&output.ticket,output.pcm_loan_id);
      (void)retire_pcm_loan(state==DBT_TICKET_CANCEL || !output.active || atomic_load(&output.failed));
    }
#endif
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
#ifdef DBT_PROBE_CODEC_AWARE_RENDER
          if(transfer_output_request(&restart))return false;
#endif
          report_done(output.stop_request,start_codec_output(restart));return true;
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
      (void)service_refill_window();
      uint32_t target = output.audio.config.target_frames +
                        (output.enabled ? 0 : DMA_DESCRIPTORS * DMA_FRAMES);
      uint32_t available = dbt_audio_output_frames(&output.audio);
      if (available < target) {
        uint32_t began = (uint32_t)esp_timer_get_time();
        uint32_t budget = target - available;
#ifdef DBT_PROBE_DSP_ASSIST
        uint32_t assist_budget=budget>512u?512u:budget;
#endif
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
#ifdef DBT_PROBE_SCHEDULER_TRACE
        trace_render(1,began);
#endif
#ifdef DBT_PROBE_DSP_ASSIST
        /* An attached WAV mixer retains its existing per-block readiness
         * boundary. Pure music may amortize the lease across four blocks. */
#ifdef DBT_PROBE_SOUND_BUILD
        if(output.sound || !budget)assist_budget=budget;
#endif
        if(!assisted && dsp_assist_submit(assist_budget,began)){
#ifdef DBT_PROBE_SCHEDULER_TRACE
          trace_render(0,(uint32_t)esp_timer_get_time());
#endif
          return false; /* no raw DSP/meter reads or lifecycle work while lent */
        }
#endif
#ifdef DBT_PROBE_SCHEDULER_TRACE
        probe_scheduler_mark_t local_mark;
        unsigned local_core=(unsigned)xPortGetCoreID();
        bool mark_local=budget!=0;
        if(mark_local)probe_scheduler_mark(local_core,&local_mark);
        uint32_t local_began=(uint32_t)esp_timer_get_time();
#endif
#ifdef DBT_PROBE_EARLY_INPUT_RELEASE
        bool input_wake_owned=(unsigned)xPortGetCoreID()==1u;
        uint32_t made=budget?(input_wake_owned?
            dbt_audio_render_with_input_wake(&output.audio,budget,copied_input_ready,NULL):
            dbt_audio_render(&output.audio,budget)):0;
#else
        const bool input_wake_owned=false;
        uint32_t made=budget?dbt_audio_render(&output.audio,budget):0;
#endif
#ifdef DBT_PROBE_SCHEDULER_TRACE
        uint32_t local_end=(uint32_t)esp_timer_get_time();
        if(mark_local && made){
          probe_scheduler_mark_t end_mark;probe_scheduler_mark(local_core,&end_mark);
          probe_scheduler_interval_observe(local_core?PROBE_DELAY_LOCAL_RENDER:PROBE_DELAY_LOCAL_RENDER0,&local_mark,&end_mark,local_began,local_end,made,atomic_load(&timeline.stream_serial),atomic_load(&output.rate));
        }
#endif
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
#ifdef DBT_PROBE_SCHEDULER_TRACE
        trace_render(0,(uint32_t)esp_timer_get_time());
#endif
        if(made)render_completed(began,input_wake_owned);
      }
      available = dbt_audio_output_frames(&output.audio);
      atomic_store(&output.ready_frames, available);
      atomic_store(&output.input_ready_frames,
                   dbt_audio_input_frames(&output.audio));
#ifdef DBT_PROBE_PCM_LOANS
      (void)retire_pcm_loan(false);
#endif
      bool prime_complete=output.enabled;
#ifdef DBT_PROBE_SOUND_BUILD
      prime_complete=prime_complete || output.sound || output.sound_standalone || output.transition_sound;
#endif
      prime_complete=prime_complete || priming_ready((uint32_t)(esp_timer_get_time()/1000));
      if (!output.enabled && prime_complete && available >= target) {
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
#ifdef DBT_PROBE_DSP_ASSIST
  if(dbt_dsp_job_state(&assist.lane)!=DBT_DSP_JOB_IDLE)return portMAX_DELAY;
#endif
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
  if(output.active && !output.enabled){
    uint32_t state=atomic_load(&output.priming_state),budget=state&PRIMING_BUDGET;
    if(budget && (state&PRIMING_ARMED) && !(state&PRIMING_DONE)){
      uint32_t elapsed=now-atomic_load(&output.priming_at_ms);
      if(elapsed<budget && budget-elapsed<ms)ms=budget-elapsed;
    }
  }
  if(ms==UINT32_MAX)return portMAX_DELAY;
  TickType_t ticks=pdMS_TO_TICKS(ms);return ticks?ticks:1;
}
static void render_loop(void *unused) {
  (void)unused;
#ifdef DBT_PROBE_CODEC_AWARE_RENDER
  for(;;){
    if(!owns_output()){
#ifdef DBT_PROBE_DSP_ASSIST
      if(dsp_assist_work())continue;
#endif
      (void)ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
      if(!owns_output()){
        TaskHandle_t target=atomic_load_explicit(&output.task,memory_order_acquire);
        if(target)xTaskNotifyGive(target);
      }
      continue;
    }
    atomic_store_explicit(&output.render_core,(uint32_t)xPortGetCoreID(),memory_order_relaxed);
    if(atomic_exchange_explicit(&output.transfer_pending,false,memory_order_acq_rel)){
      request_t transferred=output.inflight;consume_request(transferred);
      if(!owns_output())continue;
    }
    bool again=render_step();
    if(!owns_output())continue;
    if(!again)ulTaskNotifyTake(pdTRUE,render_wait_ticks());
#ifdef DBT_PROBE_RENDER_FAIR_SHARE
    else if((unsigned)xPortGetCoreID()==1u)taskYIELD(); /* one bounded step before equal-host service */
#endif
  }
#else


#ifdef ESP_PLATFORM
  atomic_store(&output.render_core,(uint32_t)xPortGetCoreID());
#else
  atomic_store(&output.render_core,DBT_ESP32_RENDER_CORE);
#endif
  for (;;) {
    if(!render_step())ulTaskNotifyTake(pdTRUE,render_wait_ticks());
  }

#endif
}

dbt_result_t probe_output_init(void (*wake_decoder)(void *), void *context) {
  if(output.task || output.requests)return DBT_BUSY;
  atomic_store(&output.refill_pending_index,UINT32_MAX);
#ifdef DBT_PROBE_DSP_ASSIST
  dbt_dsp_job_init(&assist.lane);
#endif
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
  TaskHandle_t base=NULL;
  if(xTaskCreatePinnedToCore(render_loop,"native_render",6144,0,DBT_PROBE_RENDER_CORE1_PRIORITY,
                              &base,DBT_ESP32_RENDER_CORE)!=pdPASS){
    vQueueDelete(output.requests);output.requests=NULL;output.task=NULL;
    return DBT_NO_SPACE;
  }
#ifdef DBT_PROBE_CODEC_AWARE_RENDER
  output.workers[DBT_ESP32_RENDER_CORE]=base;
#endif
  atomic_store_explicit(&output.task,base,memory_order_release);xTaskNotifyGive(base);
  return DBT_OK;
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
#ifdef DBT_PROBE_PCM_LOANS
dbt_result_t probe_output_pcm_begin(const void *samples,uint16_t frames,uint8_t channels,uint32_t rate,dbt_pcm_format_t format,uint32_t *ticket){
  if(ticket)*ticket=0;
  if(!ticket || !samples || !frames || frames>PROBE_PCM_LOAN_FRAMES || channels<1 || channels>2 ||
      rate<8000 || rate>192000 || format<DBT_PCM_S16_LE || format>DBT_PCM_S32_LE)return DBT_INVALID;
  request_t request={.kind=REQ_PCM_LOAN,.start=true,.pcm=samples,.frames=frames,.channels=channels,.rate=rate,.format=format};
  return request_begin(&request,ticket,false);
}
dbt_result_t probe_output_pcm_poll(uint32_t ticket){
  unsigned state=dbt_ticket_state(&output.ticket,ticket);
  dbt_result_t result=probe_output_request_poll(ticket);
  if(result!=DBT_PENDING && result!=DBT_OK && (state==DBT_TICKET_DONE || state==DBT_TICKET_CANCELLED))return DBT_CANCELLED;
  return result;
}
dbt_result_t probe_output_pcm_cancel(uint32_t ticket){return probe_output_request_cancel(ticket);}
#endif
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
dbt_result_t probe_output_stream_begin_primed(bool start,uint32_t rate,uint8_t channels,dbt_pcm_format_t format,bool share_decoder_core,uint16_t priming_ms,uint32_t *ticket){
  if(ticket)*ticket=0;
#ifndef DBT_PROBE_CODEC_AWARE_RENDER
  if(share_decoder_core)return DBT_UNSUPPORTED;
#endif
  if(start && (format<DBT_PCM_S16_LE || format>DBT_PCM_S32_LE || channels<1 || channels>2 || rate<8000 || rate>192000))return DBT_INVALID;
  if(priming_ms>100u)return DBT_INVALID;
  request_t request={.start=start,.share_decoder_core=share_decoder_core,.priming_ms=priming_ms,.rate=rate,.channels=channels,.format=format};return request_begin(&request,ticket,false);
}
dbt_result_t probe_output_stream_begin_shared(bool start,uint32_t rate,uint8_t channels,dbt_pcm_format_t format,bool share_decoder_core,uint32_t *ticket){
  return probe_output_stream_begin_primed(start,rate,channels,format,share_decoder_core,0,ticket);
}
dbt_result_t probe_output_stream_begin(bool start,uint32_t rate,uint8_t channels,dbt_pcm_format_t format,uint32_t *ticket){
  return probe_output_stream_begin_shared(start,rate,channels,format,false,ticket);
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
uint16_t probe_output_decode_budget(void){
#ifdef DBT_PROBE_DSP_ASSIST
  /* One lower-priority Core0 job already owns decoded input. Do not start a
   * new frame ahead of it; controls still retire at a zero media budget.
   * DONE wakes this decoder directly, without a Core1 round trip. */
  if((unsigned)xPortGetCoreID()==0u){
    dbt_dsp_job_state_t state=dbt_dsp_job_state(&assist.lane);
    if(state==DBT_DSP_JOB_READY || state==DBT_DSP_JOB_BUSY)return 0;
  }
#endif
#if defined(DBT_PROBE_CODEC_AWARE_RENDER) && defined(DBT_PROBE_PCM_COPIED)
  /* Decoder-only. Same-core owners cannot close/reinitialize these atomic
   * ring counters while this higher-priority observer runs. No buffer or
   * mutable configuration is borrowed. Disabled/different-core output keeps
   * the existing two-operation quantum, including LHDC/Opus on Core1. */
  if(!atomic_load_explicit(&output.running,memory_order_acquire) ||
     atomic_load(&output.failed) || atomic_load(&output.stop_requested) ||
     atomic_load(&output.render_core)!=(uint32_t)xPortGetCoreID())return 2;
  uint32_t input_read=atomic_load_explicit(&output.audio.input.read,memory_order_acquire);
  uint32_t input_written=atomic_load_explicit(&output.audio.input.written,memory_order_acquire);
  uint32_t output_read=atomic_load_explicit(&output.audio.output.read,memory_order_acquire);
  uint32_t output_written=atomic_load_explicit(&output.audio.output.written,memory_order_acquire);
  uint32_t reserve=2u*DMA_FRAMES;
  uint32_t rate=atomic_load_explicit(&output.rate,memory_order_relaxed);
  if(rate>96000u){
    /* A 192k LHDC frame is 5ms; measured decoding can occupy CPU0 for
     * >3ms before the lower-priority renderer runs again. Two DMA blocks
     * cover only 1.33ms here. Render already-decoded input first, retaining
     * one frame interval plus the original DMA margin before another decode.
     * This is admission within existing storage, never a larger buffer. */
    reserve+=(rate+199u)/200u;
    uint32_t target=atomic_load(&output.actual_target_frames);if(!target)target=pcm_target_frames(rate);
    if(reserve>target)reserve=target; /* renderer never fills beyond target */
  }
  if(input_written!=input_read && output_written-output_read<reserve){
    TaskHandle_t render=atomic_load_explicit(&output.task,memory_order_acquire);
    if(render)xTaskNotifyGive(render);
    return 0; /* control retirement still runs; wait on existing notification */
  }
  /* Stop at each PCM/decode boundary so another decode cannot hide the need
   * for this handoff inside the old two-operation quantum. */
  return 1;
#else
  return 2;
#endif
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
  if(result==DBT_OK){
    uint32_t previous=atomic_fetch_add(&output.accepted_frames,frames);
    uint32_t state=atomic_load(&output.priming_state);
    if(!previous && (state&PRIMING_BUDGET)){
      atomic_store(&output.priming_at_ms,(uint32_t)(esp_timer_get_time()/1000));
      atomic_store(&output.priming_state,state|PRIMING_ARMED);
    }
    xTaskNotifyGive(output.task);
  }
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
bool probe_output_running(void){return atomic_load(&output.running)!=0;}
bool probe_output_owner_needs_time(void){
  if(!atomic_load(&output.running) || atomic_load(&output.failed) || atomic_load(&output.stop_requested) ||
     atomic_load(&output.render_core)!=1u)return false;
  uint32_t rate=atomic_load(&output.rate);if(rate<=96000u)return false;
  uint32_t read=atomic_load_explicit(&output.audio.output.read,memory_order_acquire);
  uint32_t written=atomic_load_explicit(&output.audio.output.written,memory_order_acquire);
  return written-read<(rate+199u)/200u+2u*DMA_FRAMES;
}
void probe_output_stats(probe_output_stats_t *s) {
  if (!s)
    return;
  /* One read attempt. A concurrent publication is invalid for this cycle;
   * there is no retry loop and no wait for the render owner. */
  uint32_t dsp_generation=atomic_load_explicit(&output.dsp_generation,memory_order_acquire);
  s->dsp_valid=dsp_generation && !(dsp_generation&1u) && atomic_load(&output.dsp_open);
#define DSP_LOAD(field) s->dsp_##field=atomic_load_explicit(&output.dsp_##field,memory_order_relaxed)
  DSP_LOAD(at_us);DSP_LOAD(rate);DSP_LOAD(gains);DSP_LOAD(options);DSP_LOAD(peak_q27);
  DSP_LOAD(limited);DSP_LOAD(over_full_scale);DSP_LOAD(frames);
#undef DSP_LOAD
  atomic_thread_fence(memory_order_acquire);
  if(dsp_generation!=atomic_load_explicit(&output.dsp_generation,memory_order_acquire))s->dsp_valid=0;
  atomic_store_explicit(&output.dsp_requested,1,memory_order_release);
  TaskHandle_t dsp_owner=atomic_load_explicit(&output.task,memory_order_acquire);
  if(dsp_owner)xTaskNotifyGive(dsp_owner);
  s->main_render_priority=DBT_PROBE_RENDER_CORE1_PRIORITY;s->auxiliary_render_priority=DBT_PROBE_RENDER_PRIORITY;
  s->pcm_target_ms=DBT_PROBE_PCM_TARGET_MS;
  uint32_t prime=atomic_load(&output.priming_state),guard=prime&PRIMING_BUDGET;
  s->priming_ms=guard;s->priming_remaining_ms=0;
  if(guard && atomic_load(&output.requested_bytes) && !atomic_load(&output.running) && !(prime&PRIMING_DONE)){
    uint32_t elapsed=(uint32_t)(esp_timer_get_time()/1000)-atomic_load(&output.priming_at_ms);
    s->priming_remaining_ms=!(prime&PRIMING_ARMED)?guard:elapsed<guard?guard-elapsed:0;
  }
  s->adaptive_clock=OUTPUT_ADAPTIVE_CLOCK;
  s->pcm_target_frames=atomic_load(&output.actual_target_frames);
#if defined(DBT_PROBE_PCM_COPIED)
  s->pcm_input_mode=2;
#elif defined(DBT_PROBE_PCM_LOANS)
  s->pcm_input_mode=1;
#else
  s->pcm_input_mode=0;
#endif
#ifdef DBT_PROBE_BLOCK_DSP
  s->block_dsp=1;
#else
  s->block_dsp=0;
#endif
#ifdef DBT_PROBE_WORD_DSP
  s->word_dsp=1;
#else
  s->word_dsp=0;
#endif
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
  LOAD(descriptor_shape_errors);LOAD(descriptor_pool_errors);LOAD(descriptor_order_errors);
  LOAD(descriptor_order_recoveries);LOAD(descriptor_missed_eofs_min);LOAD(descriptor_recovered_source_frames);
  LOAD(descriptor_expected_index);LOAD(descriptor_observed_index);LOAD(descriptor_observed_size);
  LOAD(descriptor_fault_at_us);LOAD(descriptor_fault_gap_us);LOAD(descriptor_fault_stopping);
  LOAD(callback_calls);LOAD(callback_elapsed_us);LOAD(callback_max_us);LOAD(callback_core);LOAD(render_core);
#ifdef DBT_PROBE_PCM_LOANS
  s->pcm_loan_mode=1;
#else
  s->pcm_loan_mode=0;
#endif
  LOAD(input_ring_bytes_avoided);LOAD(pcm_loans_begun);LOAD(pcm_loans_retired);LOAD(pcm_loans_cancelled);LOAD(pcm_loan_active);LOAD(pcm_loan_max_us);
  LOAD(refill_period_us);LOAD(refill_recovered_frames);LOAD(refill_recovered_calls);LOAD(refill_unfilled_frames);LOAD(refill_canceled_frames);
  LOAD(refill_copy_max_us);LOAD(refill_mask_max_us);LOAD(refill_contract_errors);LOAD(refill_pending_index);
  LOAD(input_capacity_frames);LOAD(input_ring_bytes);
  s->configured_irq_priority=DMA_IRQ_PRIORITY;
  LOAD(max_eof_gap_us);
  LOAD(maximum_render_us);
  LOAD(render_total_us);
  LOAD(render_calls);
  LOAD(allocation_failed_bytes);
  LOAD(allocation_failure_free);
  LOAD(allocation_failure_largest);
  LOAD(allocation_failure_rate);LOAD(allocation_failure_channels);LOAD(allocation_failure_sample_bytes);LOAD(allocation_failure_at_us);
  LOAD(allocation_failure_audio_requested);LOAD(allocation_failure_audio_allocated);LOAD(allocation_failure_pages);
  LOAD(allocation_failure_free_before);LOAD(allocation_failure_free_after_cleanup);
  LOAD(allocation_failure_free_32bit);LOAD(allocation_failure_largest_32bit);
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
#ifdef DBT_PROBE_CODEC_AWARE_RENDER
  s->render_owner_transfers=atomic_load(&output.owner_transfers);
  s->auxiliary_render_stack_bytes=atomic_load(&output.auxiliary_stack_bytes);
  s->auxiliary_render_allocation_failures=atomic_load(&output.auxiliary_allocation_failures);
#else
  s->render_owner_transfers=s->auxiliary_render_stack_bytes=s->auxiliary_render_allocation_failures=0;
#endif
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
  if (!atomic_load(&output.running)){
    uint32_t target=atomic_load(&output.actual_target_frames);
    if(!target || atomic_load(&output.rate)!=rate)target=pcm_target_frames(rate);
    frames=target+DMA_DESCRIPTORS*DMA_FRAMES+80u;
  }
  if(OUTPUT_ADAPTIVE_CLOCK)frames += dbt_resampler_delay_frames(rate);
  uint32_t value = (uint32_t)(((uint64_t)frames * 10000u + rate - 1u) / rate);
  uint32_t guard=(atomic_load(&output.priming_state)&PRIMING_BUDGET)*10u;
  if(value<guard)value=guard; /* Minimum deliberate priming component. */
  return value > UINT16_MAX ? UINT16_MAX : (uint16_t)value;
}
dbt_result_t probe_output_controls(int8_t bass, int8_t mid, int8_t treble,
                                   uint8_t volume, uint8_t flags) {
  return dbt_audio_controls(&output.audio, bass, mid, treble, volume, flags);
}

/* Task-context publishers wake the existing owner immediately. Gain/EQ is
 * still applied only at a safe render boundary, with its existing ramps. */
static dbt_result_t control_published(dbt_result_t result){
  if(result==DBT_OK){TaskHandle_t owner=atomic_load_explicit(&output.task,memory_order_acquire);if(owner)xTaskNotifyGive(owner);}
  return result;
}
dbt_result_t probe_output_volume(uint8_t volume) {
  return control_published(dbt_audio_volume(&output.audio, volume));
}

dbt_result_t probe_output_eq(int8_t bass, int8_t mid, int8_t treble) {
  return control_published(dbt_audio_eq(&output.audio, bass, mid, treble));
}
dbt_result_t probe_output_control(uint8_t flags) {
  return control_published(dbt_audio_control_flags(&output.audio, flags));
}
dbt_result_t probe_output_analysis(bool enabled) {
  return control_published(dbt_audio_analysis(&output.audio, enabled));
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
#ifdef DBT_PROBE_DSP_ASSIST
  if(dbt_dsp_job_state(&assist.lane)!=DBT_DSP_JOB_IDLE)return false;
#endif
  if(!atomic_load_explicit(&output.restart_hold,memory_order_acquire) ||
     !dbt_ticket_idle(&output.ticket) ||
     atomic_load(&output.failed) || atomic_load(&output.request_fault) ||
     atomic_load(&output.running) || atomic_load(&output.requested_bytes))return false;
#ifdef DBT_PROBE_SOUND_BUILD
  if(atomic_load_explicit(&output.sound_token,memory_order_acquire))return false;
#endif
  return dbt_ticket_idle(&output.ticket);
}
