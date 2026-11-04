#ifndef DBT_RESAMPLER_H
#define DBT_RESAMPLER_H
#include <stdbool.h>
#include <stdint.h>

/* Single render owner. Q31 interleaved audio; 128 stereo frames of bounded
 * history live with the active audio object. Immutable phase banks are flash. */
#define DBT_RESAMPLER_HISTORY 128u
typedef struct {
    int32_t history[2][DBT_RESAMPLER_HISTORY],first[2];
    uint8_t next,count,taps;
    bool primed;
} dbt_resampler_t;
void dbt_resampler_init(dbt_resampler_t *state,uint32_t rate);
void dbt_resampler_push(dbt_resampler_t *state,int32_t left,int32_t right);
void dbt_resampler_filter(dbt_resampler_t *state,int32_t left,int32_t right,
                          uint32_t phase,int32_t *out_left,int32_t *out_right);
/* Fractional filter causal group delay in input frames at the given rate. */
uint8_t dbt_resampler_delay_frames(uint32_t rate);
/* Immutable row is borrowed for the program lifetime; phase is 0..128. */
const int32_t *dbt_resampler_phase(uint32_t rate,uint8_t phase,uint8_t *taps);
#endif
