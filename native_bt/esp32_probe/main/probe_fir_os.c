#define DBT_RESAMPLER_FORCE_UNFOLDED_ZERO 1
#define dbt_resampler_init probe_fir_os_init
#define dbt_resampler_push probe_fir_os_push
#define dbt_resampler_filter probe_fir_os_filter
#define dbt_resampler_phase probe_fir_os_phase
#define dbt_resampler_delay_frames probe_fir_os_delay_frames
#include "../../src/dbt_resampler.c"
