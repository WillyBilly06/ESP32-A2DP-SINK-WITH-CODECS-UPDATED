/* Diagnostic-only block kernel. No renderer selection is changed here. */
#define FIXED_DSP_BLOCK_API 1
#define fixed_dsp_init probe_dsp_block_init
#define fixed_dsp_controls probe_dsp_block_controls
#define fixed_dsp_process probe_dsp_block_process
#define fixed_dsp_process_block probe_dsp_block_process_block
#define fixed_dsp_zero_analysis probe_dsp_block_zero_analysis
#define fixed_dsp_level_db_q8 probe_dsp_block_level_db_q8
#include "../../../bt_audio_sink/main/dsp/fixed_dsp.c"
