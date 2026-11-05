/* Diagnostic-only until the actual Xtensa result and speed are measured. */
#define DBT_RESAMPLER_PAIRED_PRODUCTS 1
#define dbt_resampler_init probe_fir_pair_init
#define dbt_resampler_push probe_fir_pair_push
#define dbt_resampler_filter probe_fir_pair_filter
#define dbt_resampler_phase probe_fir_pair_phase
#define dbt_resampler_delay_frames probe_fir_pair_delay_frames
#include "../../src/dbt_resampler.c"
