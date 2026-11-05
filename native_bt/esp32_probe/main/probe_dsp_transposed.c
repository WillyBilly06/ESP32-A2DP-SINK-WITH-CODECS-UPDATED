/* Diagnostic-only exact product-state block kernel. No renderer selection. */
#define FIXED_DSP_BLOCK_API 1
#define FIXED_DSP_TRANSPOSED_BLOCK 1
#define fixed_dsp_init probe_dsp_transposed_init
#define fixed_dsp_controls probe_dsp_transposed_controls
#define fixed_dsp_process probe_dsp_transposed_process
#define fixed_dsp_process_block probe_dsp_transposed_process_block
#define fixed_dsp_zero_analysis probe_dsp_transposed_zero_analysis
#define fixed_dsp_level_db_q8 probe_dsp_transposed_level_db_q8
#include "../../../bt_audio_sink/main/dsp/fixed_dsp.c"
