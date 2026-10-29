#ifndef DBT_AUDIO_H
#define DBT_AUDIO_H
#include "dbt_queue.h"
#include "fixed_dsp.h"

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
typedef struct {
    dbt_audio_memory_t memory;
    dbt_audio_config_t config;
    dbt_audio_ring_t input,output;
    fixed_dsp_t dsp;
    _Atomic uint32_t controls_generation,controls_gains,controls_options,controls_lock;
    _Atomic uint32_t submitted_frames,rendered_frames,taken_frames,zero_frames;
    uint32_t applied_generation,fraction;
    int32_t step_adjust,filtered_error_q8,integral_ppm;
    uint32_t allocated_bytes;
    bool opened;
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
 * Persistence is separate and must retain every required immediate NVS commit. */
dbt_result_t dbt_audio_controls(dbt_audio_t *audio,int8_t bass,int8_t mid,int8_t treble,uint8_t volume,uint8_t flags);
/* Render owner. Input conversion, optional fractional clock correction, then DSP.
 * The caller controls output watermarks; this function performs bounded work. */
uint32_t dbt_audio_render(dbt_audio_t *audio,uint32_t budget);
/* Render owner, normally at a slow control interval (e.g. 500 ms). Clamped
 * +/-500 ppm adaptive interpolation avoids hard sample insertion/deletion. */
void dbt_audio_clock(dbt_audio_t *audio);
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
