/* Os reference retaining the general split-product accumulation. */
#define FIXED_DSP_FORCE_SPLIT_SUM 1
#define fixed_dsp_init probe_dsp_os_init
#define fixed_dsp_controls probe_dsp_os_controls
#define fixed_dsp_process probe_dsp_os_process
#define fixed_dsp_zero_analysis probe_dsp_os_zero_analysis
#define fixed_dsp_level_db_q8 probe_dsp_os_level_db_q8
#include "../../../bt_audio_sink/main/dsp/fixed_dsp.c"
