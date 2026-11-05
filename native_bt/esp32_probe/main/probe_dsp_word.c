/* Diagnostic-only exact steady/ramp upper-word quantization comparison. */
#define FIXED_DSP_BLOCK_API 1
#define FIXED_DSP_WORD_QUANT 1
#define FIXED_DSP_WORD_RAMP 1
#define fixed_dsp_init probe_dsp_word_init
#define fixed_dsp_controls probe_dsp_word_controls
#define fixed_dsp_process probe_dsp_word_process
#define fixed_dsp_process_block probe_dsp_word_process_block
#define fixed_dsp_zero_analysis probe_dsp_word_zero_analysis
#define fixed_dsp_level_db_q8 probe_dsp_word_level_db_q8
#include "../../../bt_audio_sink/main/dsp/fixed_dsp.c"
