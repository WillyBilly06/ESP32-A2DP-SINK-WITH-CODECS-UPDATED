#include "probe_lhdc_profile.h"
static probe_lhdc_profile_t counters;
static bool active;
void probe_lhdc_profile_reset(bool collecting){
    counters=(probe_lhdc_profile_t){0};active=collecting;
}
void dbt_lhdc_stage_record(uint32_t pre,uint32_t entropy,uint32_t mantissa,
                         uint32_t synthesis,uint32_t imdct,uint32_t overlap){
    if(!active)return;
    const uint32_t values[6]={pre,entropy,mantissa,synthesis,imdct,overlap};
    for(unsigned i=0;i<6;++i){
        counters.sum_cycles[i]+=values[i];
        if(values[i]>counters.max_cycles[i])counters.max_cycles[i]=values[i];
    }
    ++counters.channel_calls;
}
bool probe_lhdc_profile_finish(probe_lhdc_profile_t *result){
    if(!result)return false;
    active=false;*result=counters;return counters.channel_calls!=0;
}
