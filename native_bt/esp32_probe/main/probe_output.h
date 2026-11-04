#ifndef PROBE_OUTPUT_H
#define PROBE_OUTPUT_H
#include "dbt_audio.h"
typedef struct {
    uint32_t running,failed,rate,requested_bytes,accepted_frames,rendered_frames,ready_frames,input_ready_frames;
    uint32_t dma_eofs,dma_source_frames,underrun_frames,descriptor_errors,max_eof_gap_us;
    uint32_t maximum_render_us,stop_fill_frames;
    uint32_t render_total_us,render_calls,render_stack_low_water_bytes;
    uint32_t allocation_failed_bytes,allocation_failure_free,allocation_failure_largest;
    uint32_t profile_conversion,profile_dsp,profile_tail,profile_controls,profile_frames;
    uint32_t profile_conversion_max,profile_dsp_max,profile_tail_max;
    uint32_t first_underrun_accepted_frames,first_underrun_dma_frames;
    int32_t clock_ppm,clock_observed_ppm,clock_error_frames;
    uint32_t clock_state,clock_target_frames,clock_updates,clock_disturbances;
    uint32_t clock_observed_valid,clock_snapshot_valid;
    uint32_t request_fault,stale_receipts;
} probe_output_stats_t;
dbt_result_t probe_output_init(void (*wake_decoder)(void *),void *context);
/* Accepted begin returns PENDING and one opaque ticket; BUSY accepts nothing.
 * Poll consumes exactly one terminal result. Cancel remains pending until the
 * render owner has reclaimed any accepted side effect. A stale id is INVALID,
 * never evidence that a borrowed object can be freed. Clients receive wakeups
 * through their registered decoder/sound notifications. */
dbt_result_t probe_output_stream_begin(bool,uint32_t,uint8_t,dbt_pcm_format_t,uint32_t *);
dbt_result_t probe_output_request_poll(uint32_t);
dbt_result_t probe_output_request_cancel(uint32_t);
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
void probe_output_stats(probe_output_stats_t *stats);
/* Cheap failure fence for hot-path monitoring; no stack scanning/snapshot. */
bool probe_output_failed(void);
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
