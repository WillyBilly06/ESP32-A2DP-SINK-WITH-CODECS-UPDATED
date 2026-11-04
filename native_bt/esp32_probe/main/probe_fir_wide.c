/* Original interpolation arithmetic, compiled at the retained production O3. */
#define DBT_RESAMPLER_FORCE_WIDE_DELTA 1
#define dbt_resampler_init probe_fir_wide_init
#define dbt_resampler_push probe_fir_wide_push
#define dbt_resampler_filter probe_fir_wide_filter
#define dbt_resampler_phase probe_fir_wide_phase
#define dbt_resampler_delay_frames probe_fir_wide_delay_frames
#include "../../src/dbt_resampler.c"
