/* Kept diagnostic-only: portable folding is exact but slower on this target. */
#define DBT_RESAMPLER_ENABLE_FOLDED_ZERO 1
#define dbt_resampler_init probe_fir_fold_init
#define dbt_resampler_push probe_fir_fold_push
#define dbt_resampler_filter probe_fir_fold_filter
#define dbt_resampler_phase probe_fir_fold_phase
#define dbt_resampler_delay_frames probe_fir_fold_delay_frames
#include "../../src/dbt_resampler.c"
