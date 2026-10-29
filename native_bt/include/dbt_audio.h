#ifndef DBT_AUDIO_H
#define DBT_AUDIO_H
#include "dbt_queue.h"
#include "fixed_dsp.h"
#include "dbt_resampler.h"

typedef enum { DBT_PCM_S16_LE=2, DBT_PCM_S24_LE=3, DBT_PCM_S32_LE=4 } dbt_pcm_format_t;
typedef struct {
    uint8_t *bytes;
    uint32_t frames,mask,stride;
    _Atomic uint32_t written,read;
} dbt_audio_ring_t;
typedef struct {
    void *context;
    uint8_t *(*allocate)(void *context,uint32_t bytes);
    void (*release)(void *context,uint8_t *bytes);
} dbt_audio_memory_t;
typedef struct {
    uint32_t rate,input_frames,output_frames,target_frames;
    dbt_pcm_format_t format;
    uint8_t channels;
    bool adaptive_clock;
} dbt_audio_config_t;
typedef enum {
    DBT_AUDIO_CLOCK_OFF=0, DBT_AUDIO_CLOCK_CALIBRATING,
    DBT_AUDIO_CLOCK_TRACKING, DBT_AUDIO_CLOCK_RECOVERING
} dbt_audio_clock_state_t;
typedef struct {
    int32_t ppm, observed_ppm, error_frames;
    uint32_t state,target_frames,updates,disturbances,observed_valid;
} dbt_audio_clock_stats_t;
typedef struct {
    dbt_audio_memory_t memory;
    dbt_audio_config_t config;
    dbt_audio_ring_t input,output;
    fixed_dsp_t dsp;
    dbt_resampler_t resampler;
    _Atomic uint32_t controls_generation,controls_gains,controls_options,controls_lock;
    _Atomic uint32_t submitted_frames,rendered_frames,taken_frames,zero_frames;
    uint32_t applied_generation,fraction;
    int32_t step_adjust,filtered_error_q8,integral_ppm;
    int32_t clock_integral_q16,clock_ppm,clock_observed_ppm;
    uint32_t clock_last_ms,clock_submitted,clock_rendered,clock_taken,clock_zero,clock_busy;
    uint32_t clock_target,clock_calibration_ms,clock_clean_ms,clock_window_ms,clock_updates,clock_disturbances;
    uint64_t clock_calibration_sum,clock_window_source,clock_window_sink;
    dbt_audio_clock_state_t clock_state;
    bool clock_primed,clock_observed_valid;
    _Atomic uint32_t producer_backpressure,clock_version;
    _Atomic int32_t clock_pub_ppm,clock_pub_observed_ppm,clock_pub_error;
    _Atomic uint32_t clock_pub_state,clock_pub_target,clock_pub_updates,clock_pub_disturbances,clock_pub_observed_valid;
    uint32_t allocated_bytes;
    uint16_t drain_remaining;
    bool opened,draining_input;
} dbt_audio_t;

/* Decoder -> render -> DMA/ISR are distinct SPSC boundaries. Initialize before
 * owners start. open/close require the decoder and DMA consumer quiesced; only
 * the render owner mutates filter/history/clock state. All large buffers are
 * lazy and configuration-specific. No floating-point signal processing. */
dbt_result_t dbt_audio_init(dbt_audio_t *audio,dbt_audio_memory_t memory);
dbt_result_t dbt_audio_open(dbt_audio_t *audio,dbt_audio_config_t config);
void dbt_audio_close(dbt_audio_t *audio);
/* Producer only; whole block or BUSY, with no partial publication. */
dbt_result_t dbt_audio_submit(dbt_audio_t *audio,const void *pcm,uint32_t frames);
/* Settings/control writers publish a complete snapshot. BUSY never spins.
 * On ESP these writers run in task context using an internal-RAM audio object
 * and stack. They briefly mask local interrupts while publishing; they never
 * block or hold the mask across persistence.
 * Persistence is separate and must retain every required immediate NVS commit. */
dbt_result_t dbt_audio_controls(dbt_audio_t *audio,int8_t bass,int8_t mid,int8_t treble,uint8_t volume,uint8_t flags);
/* Volume-only publication preserves EQ and flags; BUSY never waits. */
dbt_result_t dbt_audio_volume(dbt_audio_t *audio,uint8_t volume);
/* Named-field updates preserve concurrent independent control fields. */
dbt_result_t dbt_audio_eq(dbt_audio_t *,int8_t,int8_t,int8_t);
dbt_result_t dbt_audio_control_flags(dbt_audio_t *,uint8_t);
dbt_result_t dbt_audio_analysis(dbt_audio_t *,bool);
/* Render owner. Input conversion, optional fractional clock correction, then DSP.
 * The caller controls output watermarks; this function performs bounded work. */
uint32_t dbt_audio_render(dbt_audio_t *audio,uint32_t budget);
/* Render owner only, after the producer is quiesced: extend the final sample
 * through the finite causal filter delay. The next open resets this state. */
void dbt_audio_begin_drain(dbt_audio_t *audio);
/* Render owner. Reset at the transition to a running consumer, then update at
 * approximately 500 ms using wrap-safe monotonic milliseconds. The loop
 * calibrates occupancy, limits correction to +/-500 ppm and limits slew to
 * 50 ppm/s. Underruns, producer backpressure and scheduler gaps freeze the
 * integrator and invalidate measured clock estimates until recovery. */
void dbt_audio_clock_reset(dbt_audio_t *audio,uint32_t now_ms);
void dbt_audio_clock_update(dbt_audio_t *audio,uint32_t now_ms);
/* Compatibility entry point assumes consecutive 500 ms intervals. */
void dbt_audio_clock(dbt_audio_t *audio);
/* Bounded, race-free reader usable by a different task. False means retry. */
bool dbt_audio_clock_snapshot(const dbt_audio_t *audio,dbt_audio_clock_stats_t *stats);
/* One consumer (render task before enable, then DMA ISR). This path copies only
 * already-processed internal-SRAM samples; missing frames are explicitly zeroed.
 * It does not run DSP, allocate, block, log or dereference flash tables. */
uint32_t dbt_audio_take(dbt_audio_t *audio,int32_t *stereo,uint32_t frames);
uint32_t dbt_audio_input_frames(const dbt_audio_t *audio);
uint32_t dbt_audio_output_frames(const dbt_audio_t *audio);
typedef struct {
    uint32_t total,remaining;
    int32_t gain,step,remainder,error;
} dbt_audio_fade_t;
/* Initialize while the consumer is quiescent. The consume-side fade has no
 * division, 64-bit multiplication, allocation or flash-table accesses. */
void dbt_audio_fade_init(dbt_audio_fade_t *fade,uint32_t frames);
void dbt_audio_fade(dbt_audio_fade_t *fade,int32_t *stereo,uint32_t frames);

#endif
