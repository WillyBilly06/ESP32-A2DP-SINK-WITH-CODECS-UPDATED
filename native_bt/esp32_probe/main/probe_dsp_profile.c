#include "probe_dsp_profile.h"
#include "esp_cpu.h"
static probe_dsp_profile_t profile;
static uint32_t last;
static inline __attribute__((always_inline)) void mark(unsigned stage){
    /* Compiler fences, not hardware waits: prevent moving kernel memory
     * operations across the public CCOUNT read. */
    __asm__ __volatile__("" ::: "memory");
    uint32_t now=esp_cpu_get_cycle_count();
    __asm__ __volatile__("" ::: "memory");
    if(stage)profile.cycles[stage-1u]+=now-last;
    last=now;if(stage==5u)++profile.frames;
}
#define FIXED_DSP_STAGE_MARK(stage) mark(stage)
#define fixed_dsp_init probe_dsp_profile_init
#define fixed_dsp_controls probe_dsp_profile_controls
#define fixed_dsp_process probe_dsp_profile_process
#define fixed_dsp_zero_analysis probe_dsp_profile_zero_analysis
#define fixed_dsp_level_db_q8 probe_dsp_profile_level_db_q8
#include "../../../bt_audio_sink/main/dsp/fixed_dsp.c"
void probe_dsp_profile_reset(void){
    for(unsigned i=0;i<5;++i)profile.cycles[i]=0;
    profile.frames=0;last=0;
}
void probe_dsp_profile_snapshot(probe_dsp_profile_t *out){if(out)*out=profile;}
