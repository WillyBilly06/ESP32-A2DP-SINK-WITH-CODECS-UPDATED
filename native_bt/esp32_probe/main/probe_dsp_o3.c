/* Same source and interfaces as the Os variant; only compiler policy differs. */
#define fixed_dsp_init probe_dsp_o3_init
#define fixed_dsp_controls probe_dsp_o3_controls
#define fixed_dsp_process probe_dsp_o3_process
#define fixed_dsp_zero_analysis probe_dsp_o3_zero_analysis
#define fixed_dsp_level_db_q8 probe_dsp_o3_level_db_q8
#include "../../../bt_audio_sink/main/dsp/fixed_dsp.c"
