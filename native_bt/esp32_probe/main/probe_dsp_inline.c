/* Diagnostic only: arithmetic matches the current Os product kernel. */
#define FIXED_DSP_FORCE_FILTER_INLINE 1
#define fixed_dsp_init probe_dsp_inline_init
#define fixed_dsp_controls probe_dsp_inline_controls
#define fixed_dsp_process probe_dsp_inline_process
#define fixed_dsp_zero_analysis probe_dsp_inline_zero_analysis
#define fixed_dsp_level_db_q8 probe_dsp_inline_level_db_q8
#include "../../../bt_audio_sink/main/dsp/fixed_dsp.c"
