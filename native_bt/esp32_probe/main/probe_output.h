#ifndef PROBE_OUTPUT_H
#define PROBE_OUTPUT_H
#include "dbt_audio.h"
/* Native runtime keeps bounded first-gap evidence without enabling the tick
 * sampler or per-owner cost profiler. Detailed tracing remains opt-in. */
#if defined(DBT_PROBE_SCHEDULER_TRACE) || defined(DBT_PROBE_STAY_NATIVE)
#define PROBE_OUTPUT_GAP_TRACE 1
#endif
/* Keep 384 total DMA frames while reducing IRQ cadence at high rates.
 * Two descriptors are supported by IDF; refill still closes at the next EOF. */
#define PROBE_OUTPUT_DMA_FRAMES 192u
#define PROBE_OUTPUT_DMA_DESCRIPTORS 2u
#ifdef DBT_PROBE_DSP_ASSIST
#ifndef DBT_PROBE_DSP_ASSIST_BLOCKS
#define DBT_PROBE_DSP_ASSIST_BLOCKS 4
#endif
#if DBT_PROBE_DSP_ASSIST_BLOCKS != 4 && DBT_PROBE_DSP_ASSIST_BLOCKS != 5 && DBT_PROBE_DSP_ASSIST_BLOCKS != 6
#error DSP batch must be4,5 or6 canonical blocks
#endif
#elif defined(DBT_PROBE_DSP_ASSIST_BLOCKS)
#error DSP batch override requires DSP assistance
#endif
#ifndef DBT_PROBE_PCM_TARGET_MS
#define DBT_PROBE_PCM_TARGET_MS 64
#endif
#if DBT_PROBE_PCM_TARGET_MS != 16 && DBT_PROBE_PCM_TARGET_MS != 32 && DBT_PROBE_PCM_TARGET_MS != 64
#error "PCM target must be 16, 32 or 64 ms"
#endif
#if DBT_PROBE_PCM_TARGET_MS != 64 && ((!defined(DBT_PROBE_PCM_LOANS) && !defined(DBT_PROBE_PCM_COPIED)) || !defined(DBT_PROBE_ASYNC_AUDIO))
#error "Short PCM target requires a finite async PCM input experiment"
#endif
#if defined(DBT_PROBE_PCM_COPIED) && (defined(DBT_PROBE_PCM_LOANS) || !defined(DBT_PROBE_ASYNC_AUDIO))
#error "Compact copied PCM requires async output and excludes loans"
#endif
#define PROBE_PCM_TRANSFER_FRAMES 256u
#if defined(DBT_PROBE_WORD_DSP) && !defined(DBT_PROBE_BLOCK_DSP)
#error Word DSP requires the block renderer
#endif
typedef struct {
    uint32_t running,failed,rate,requested_bytes,accepted_frames,rendered_frames,ready_frames,input_ready_frames;
    uint32_t priming_ms,priming_remaining_ms;
    uint32_t startup_coherent,startup_first_pcm_us,startup_enable_us,startup_elapsed_us;
    uint32_t startup_input_frames,startup_preload_frames,startup_guard_state,startup_encoded_slots;
    uint32_t pcm_target_ms,block_dsp,word_dsp,pcm_input_mode,input_capacity_frames,input_ring_bytes,adaptive_clock,pcm_target_frames;
    /* underrun_frames retains its original initial-refill-shortage meaning. */
    uint32_t dma_eofs,dma_source_frames,underrun_frames,descriptor_errors,max_eof_gap_us;
    uint32_t refill_period_us,refill_recovered_frames,refill_recovered_calls,refill_unfilled_frames,refill_canceled_frames;
    uint32_t refill_copy_max_us,refill_mask_max_us,refill_contract_errors,refill_pending_index;
    uint32_t descriptor_shape_errors,descriptor_pool_errors,descriptor_order_errors;
    /* Recoveries preserve the error count. Missed EOFs are a lower bound;
     * additional complete DMA laps cannot be inferred from the latest IRQ. */
    uint32_t descriptor_order_recoveries,descriptor_missed_eofs_min,descriptor_recovered_source_frames;
    uint32_t descriptor_expected_index,descriptor_observed_index,descriptor_observed_size;
    uint32_t descriptor_fault_at_us,descriptor_fault_gap_us,descriptor_fault_stopping;
    uint32_t configured_irq_priority,callback_calls,callback_elapsed_us,callback_max_us,callback_core,render_core;
    uint32_t pcm_loan_mode,input_ring_bytes_avoided,pcm_loans_begun,pcm_loans_retired,pcm_loans_cancelled,pcm_loan_active,pcm_loan_max_us;
    uint32_t maximum_render_us,stop_fill_frames;
    uint32_t render_total_us,render_calls,render_stack_low_water_bytes;
    uint32_t render_owner_transfers,auxiliary_render_stack_bytes,auxiliary_render_allocation_failures;
    uint32_t main_render_priority,auxiliary_render_priority;
    uint32_t allocation_failed_bytes,allocation_failure_free,allocation_failure_largest;
    uint32_t allocation_failure_rate,allocation_failure_channels,allocation_failure_sample_bytes,allocation_failure_at_us;
    uint32_t allocation_failure_audio_requested,allocation_failure_audio_allocated,allocation_failure_pages;
    uint32_t allocation_failure_free_before,allocation_failure_free_after_cleanup;
    uint32_t allocation_failure_free_32bit,allocation_failure_largest_32bit;
    uint32_t profile_conversion,profile_dsp,profile_tail,profile_controls,profile_frames;
    uint32_t profile_conversion_max,profile_dsp_max,profile_tail_max;
    uint32_t dsp_valid,dsp_at_us,dsp_rate,dsp_gains,dsp_options,dsp_peak_q27,dsp_limited,dsp_over_full_scale,dsp_frames;
    uint32_t first_underrun_accepted_frames,first_underrun_dma_frames;
    int32_t clock_ppm,clock_observed_ppm,clock_error_frames;
    uint32_t clock_state,clock_target_frames,clock_updates,clock_disturbances;
    uint32_t clock_observed_valid,clock_snapshot_valid;
    uint32_t request_fault,stale_receipts;
} probe_output_stats_t;
/* Startup owner only, before clients. BUSY preserves an existing instance.
 * Failed task creation retires the new queue so initialization may be retried. */
dbt_result_t probe_output_init(void (*wake_decoder)(void *),void *context);
/* Accepted begin returns PENDING and one opaque ticket; BUSY accepts nothing.
 * Poll consumes exactly one terminal result. Cancel remains pending until the
 * render owner has reclaimed any accepted side effect. A stale id is INVALID,
 * never evidence that a borrowed object can be freed. Clients receive wakeups
 * through their registered decoder/sound notifications. */
/* Closed output can transfer to a lower-priority CPU0 owner for light codecs.
 * Active I2S never migrates; the existing ticket retains request ownership. */
/* Minimum first-PCM startup guard, at most100ms, using existing storage.
 * Starts return the existing async ticket; timer expiry is render-owner work. */
dbt_result_t probe_output_stream_begin_primed(bool,uint32_t,uint8_t,dbt_pcm_format_t,bool,uint16_t,uint32_t *);
dbt_result_t probe_output_stream_begin_shared(bool start,uint32_t rate,uint8_t channels,dbt_pcm_format_t format,bool share_decoder_core,uint32_t *ticket);
dbt_result_t probe_output_stream_begin(bool,uint32_t,uint8_t,dbt_pcm_format_t,uint32_t *);
dbt_result_t probe_output_request_poll(uint32_t);
dbt_result_t probe_output_request_cancel(uint32_t);
#ifdef DBT_PROBE_PCM_LOANS
#define PROBE_PCM_LOAN_FRAMES PROBE_PCM_TRANSFER_FRAMES
/* PENDING lends immutable storage until an OK/CANCELLED poll. A stale id does
 * not prove retirement. Cancel is a request, not an acknowledgement. */
dbt_result_t probe_output_pcm_begin(const void *,uint16_t,uint8_t,uint32_t,dbt_pcm_format_t,uint32_t *);
dbt_result_t probe_output_pcm_poll(uint32_t);
dbt_result_t probe_output_pcm_cancel(uint32_t);
#endif
/* Decoder worker only; bounded synchronous handshake with the render task. */
dbt_result_t probe_output_stream(bool start,uint32_t rate,uint8_t channels);
dbt_result_t probe_output_pcm(const int16_t *samples,uint16_t frames,uint8_t channels,uint32_t rate);
/* Decoder worker only. Format 2/3/4 means interleaved signed little-endian
 * 16-bit, packed 24-bit, or 32-bit slots, respectively. */
dbt_result_t probe_output_stream_format(bool start,uint32_t rate,uint8_t channels,dbt_pcm_format_t format);
dbt_result_t probe_output_pcm_format(const void *samples,uint16_t frames,uint8_t channels,uint32_t rate,dbt_pcm_format_t format);
/* Decoder producer only. One bounded attempt; BUSY copies/publishes nothing.
 * Stream control must keep the producer quiescent through format/stop tickets. */
dbt_result_t probe_output_try_pcm_format(const void *,uint16_t,uint8_t,uint32_t,dbt_pcm_format_t);
/* Decoder-only bounded scheduling hint:0=let same-core render make progress,
 *1=one media operation at a time on a shared core,2=existing separate-core
 * quantum. A zero budget still permits worker control/loan retirement. The
 * decoder waits on its existing notification; the render consumer wakes it. */
uint16_t probe_output_decode_budget(void);
void probe_output_stats(probe_output_stats_t *stats);
/* Independent owner-published counters since boot; cycles are helper Core0
 * only. Queue/finish/wall spans use the shared microsecond clock. */
typedef struct {
    uint32_t enabled,state,jobs,blocks,frames,cycles,max_cycles,max_job_frames,queue_max_us,finish_wait_max_us,retire_wait_max_us,wall_max_us,failures;
} probe_dsp_assist_stats_t;
void probe_output_dsp_assist_snapshot(probe_dsp_assist_stats_t *);
#ifdef PROBE_OUTPUT_GAP_TRACE
/* Passive stage stamps. Each producer has one writer; snapshots never spin.
 * Phase0=between operations;1=decode;2=reset;3=PCM delivery;
 *4/5/6/7/8=waiting for PCM space/loan/control/encoded input/shared renderer. */
typedef struct {
    uint32_t captured,coherent,deadline_us,pending_acl_metadata;
    uint32_t stream_serial,at_us,taken_frames,input_frames,output_frames,accepted_frames,rendered_frames,dma_source_frames,acl_us,host_poll_us,media_us,media_admit_us,media_epoch,media_sequence,media_timestamp,media_ssrc,media_result,media_gap_us,media_max_gap_us,media_arrival_us,media_arrival_cursor,media_arrival_flags,media_arrival_valid,media_coherent,decoder_phase,decoder_at_us,decoder_coherent,render_phase,render_at_us,render_coherent,decoder_queued,dsp_assist_state;
    uint32_t vhci_coherent,vhci_at_us,vhci_flags,vhci_submitted_us,vhci_packet,vhci_pending_acks,vhci_owned_now;
} probe_output_starvation_t;
void probe_output_trace_media(uint32_t began,uint32_t ended,uint32_t epoch,uint16_t sequence,uint32_t timestamp,uint32_t ssrc,dbt_result_t result);
/* Matched current complete ACL/L2CAP/RTP packet, not RF arrival. Fragmented,
 * deferred and raw media remain uncorrelated. All fields share the version. */
void probe_output_trace_media_arrival(uint32_t,uint32_t,uint32_t,uint16_t,uint32_t,uint32_t,dbt_result_t,bool,uint32_t,uint32_t,uint32_t);
void probe_output_trace_acl(uint32_t at);
void probe_output_trace_host_poll(uint32_t at);
void probe_output_trace_decoder(uint32_t phase,uint32_t at,uint32_t queued);
void probe_output_first_starvation(probe_output_starvation_t *snapshot);
/* First finalized unfilled descriptor: captured at closure, not at the initial
 * shortage. deadline_us is the guarded copy cutoff, or zero if ineligible.
 * Ring counts are adjacent atomic observations; no queue lock is taken. */
void probe_output_first_unfilled(probe_output_starvation_t *snapshot);
#endif

/* Cheap failure fence for hot-path monitoring; no stack scanning/snapshot. */
bool probe_output_failed(void);
bool probe_output_running(void);
/* Atomic low-output hint only; no buffer pointers, stack scan or full stats. */
bool probe_output_owner_needs_time(void);
uint16_t probe_output_delay_100us(uint32_t rate);
dbt_result_t probe_output_controls(int8_t bass,int8_t mid,int8_t treble,uint8_t volume,uint8_t flags);
dbt_result_t probe_output_volume(uint8_t volume);
dbt_result_t probe_output_eq(int8_t,int8_t,int8_t);
dbt_result_t probe_output_control(uint8_t);
dbt_result_t probe_output_analysis(bool);
void probe_output_meter(uint8_t values[3]);
/* Owner-only restart fence: prevents new starts/borrows, permits retirement.
 * A latched failure or unresolved ticket is never quiescent. */
void probe_output_restart_hold(bool);
bool probe_output_restart_quiescent(void);
#ifdef DBT_PROBE_SOUND_BUILD
#include "dbt_sound.h"
/* A successful attach lends play beyond the attach ticket's completion. Keep
 * it live until its sound token is retired. Cancelling a release retires that
 * request, not an earlier successful attachment; sound_alive remains decisive. */
dbt_result_t probe_output_sound_reserve_begin(uint32_t,bool,uint32_t *);
dbt_result_t probe_output_sound_attach_begin(uint32_t,dbt_sound_play_t *,uint32_t *);
dbt_result_t probe_output_sound_release_begin(uint32_t,uint32_t *);
/* Sound worker only. Reserve, open/prefill, attach, then release after drain.
 * The render task owns the borrowed playback pointer until token retirement. */
dbt_result_t probe_output_sound_reserve(uint32_t rate,bool exclusive,uint32_t *token);
dbt_result_t probe_output_sound_attach(uint32_t token,dbt_sound_play_t *play);
dbt_result_t probe_output_sound_release(uint32_t token);
bool probe_output_sound_alive(uint32_t token);
uint32_t probe_output_active_rate(void);
void probe_output_sound_wake(void (*wake)(void *),void *context);
/* Sound worker: wake render after publishing PCM or requesting stop. */
void probe_output_sound_data_ready(void);
#endif
#endif
