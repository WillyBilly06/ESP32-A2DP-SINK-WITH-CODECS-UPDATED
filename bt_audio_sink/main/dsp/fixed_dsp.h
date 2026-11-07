#ifndef FIXED_DSP_H
#define FIXED_DSP_H

#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

enum { FDSP_BASS_BOOST = 1, FDSP_CHANNEL_FLIP = 2, FDSP_BYPASS = 4, FDSP_ANALYSIS = 8 };
typedef struct {
    int32_t value[5], target[5], step[5], remainder[5], error[5];
    uint32_t remaining, duration;
} fdsp_coefficients_t;
typedef struct { int32_t x1, x2, y1, y2; uint32_t fraction; } fdsp_history_t;
typedef struct { int32_t value, target, step, remainder, error; uint32_t remaining, duration; } fdsp_ramp_t;
typedef struct {
    fdsp_coefficients_t filters[5]; /* bass/mid/treble, compensation, boost */
    int32_t cross_coefficients[2][5]; /* fixed until the sample rate changes */
    fdsp_history_t channel_history[4][2], boost_history[3], cross_history[2];
    fdsp_ramp_t volume, bypass, flip;
    uint32_t sample_rate, fade_frames, flags;
    uint32_t peak_q27, limited, over_full_scale;
    int rate_index;
    int8_t gains[3];
    uint8_t requested_volume;
    uint16_t decimation, analysis_count, window_count, window_frames;
    int64_t analysis_sum, resonators[3][2];
    int32_t analysis_coefficients[13], meter_states[3];
    uint32_t envelopes[3], goertzel_levels[3], meter_levels[3];
} fixed_dsp_t;

/* Owner-only APIs. Controls must be delivered as one complete block snapshot.
 * Fully inaudible full/split branches pause at stable bypass endpoints and
 * clear on reactivation under the existing crossfade. Both run during a fade.
 * No heap allocation and no floating-point operations in this implementation. */
bool fixed_dsp_init(fixed_dsp_t *dsp, uint32_t sample_rate, uint32_t low_pass_hz,
                     uint32_t high_pass_hz, uint32_t goertzel_reference_frames);
void fixed_dsp_controls(fixed_dsp_t *dsp, int8_t bass, int8_t mid, int8_t treble,
                         uint8_t volume, uint8_t flags, bool immediate);
void fixed_dsp_process(fixed_dsp_t *dsp, int32_t *left_q31, int32_t *right_q31);
#define FIXED_DSP_BLOCK_FRAMES 128u
#ifdef FIXED_DSP_BLOCK_API
/* Experimental owner-only block API. Controls cannot change during the call.
 * Accepts 0..128 interleaved, word-aligned stereo frames; false rejects the
 * whole call. Caller storage must not overlap dsp. No heap/persistent scratch.
 * The canonical per-frame path remains available for exact comparisons. */
bool fixed_dsp_process_block(fixed_dsp_t *dsp,int32_t *pcm,uint16_t frames);
#if defined(FIXED_DSP_VOLUME_RAMP_COMPARE) || defined(FIXED_DSP_MATCHED_FEEDBACK_COMPARE)
typedef struct {uint32_t revision,rate,policy,calls,cycles,minimum,maximum,clipped;bool coherent;} fixed_dsp_volume_stats_t;
/* Optional task-context, independent per-core/mode counters; one read attempt.
 * Calls are full128-frame, nonclipped blocks in the admitted volume-ramp or
 * matched-feedback steady comparison. Cycles include
 * preemption and wrap modulo uint32. No allocation or new DSP state. */
void fixed_dsp_volume_stats(unsigned core,unsigned mode,fixed_dsp_volume_stats_t *);
#endif
#endif
void fixed_dsp_zero_analysis(fixed_dsp_t *dsp);
int32_t fixed_dsp_level_db_q8(uint32_t level_q31);

#ifdef __cplusplus
}
#endif
#endif
