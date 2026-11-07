#include "probe_lhdc_profile.h"
#include <stdatomic.h>
_Static_assert(ATOMIC_INT_LOCK_FREE==2 && sizeof(unsigned)==4,"lock-free stage snapshots");
static probe_lhdc_profile_t counters;
static bool active;
static uint64_t frame_sum[4];
static uint32_t frame_max[4],frame_calls;
static _Atomic uint32_t live_enabled;
void probe_lhdc_profile_live_enable(void){atomic_store(&live_enabled,1u);}
static uint32_t live_rate,live_bytes,generation;
static struct {
    _Atomic uint32_t revision,generation,rate,sample_bytes,collecting,channel_calls;
    _Atomic uint32_t sum_cycles[6],max_cycles[6];
    _Atomic uint32_t frame_calls,frame_sum[4],frame_max[4];
} published;
static void publish(void){
    uint32_t r=atomic_load(&published.revision);
    atomic_store(&published.revision,r+1u);
#ifdef PROBE_LHDC_PROFILE_TEST
    extern void probe_lhdc_profile_test_hold(void);probe_lhdc_profile_test_hold();
#endif
    atomic_store(&published.generation,generation);atomic_store(&published.rate,atomic_load(&live_enabled)?live_rate:0u);
    atomic_store(&published.sample_bytes,live_bytes);atomic_store(&published.collecting,active);
    atomic_store(&published.channel_calls,counters.channel_calls);
    for(unsigned i=0;i<6;++i){atomic_store(&published.sum_cycles[i],(uint32_t)counters.sum_cycles[i]);atomic_store(&published.max_cycles[i],counters.max_cycles[i]);}
    atomic_store(&published.frame_calls,frame_calls);
    for(unsigned i=0;i<4;++i){atomic_store(&published.frame_sum[i],(uint32_t)frame_sum[i]);atomic_store(&published.frame_max[i],frame_max[i]);}
    uint32_t next=r+2u;atomic_store(&published.revision,next?next:2u);
}
void probe_lhdc_profile_reset(bool collecting){
    counters=(probe_lhdc_profile_t){0};active=collecting;live_rate=live_bytes=frame_calls=0;
    for(unsigned i=0;i<4;++i){frame_sum[i]=0;frame_max[i]=0;}
    publish();
}
void dbt_lhdc_stage_live_begin(uint32_t rate,uint32_t sample_bytes){
    probe_lhdc_profile_reset(true);if(++generation==0)++generation;
    live_rate=rate;live_bytes=sample_bytes;publish();
}
void dbt_lhdc_stage_live_end(void){if(live_rate){active=false;publish();}}
void dbt_lhdc_stage_record(uint32_t pre,uint32_t entropy,uint32_t mantissa,uint32_t synthesis,uint32_t imdct,uint32_t overlap){
    if(!active)return;
    const uint32_t values[6]={pre,entropy,mantissa,synthesis,imdct,overlap};
    for(unsigned i=0;i<6;++i){counters.sum_cycles[i]+=values[i];if(values[i]>counters.max_cycles[i])counters.max_cycles[i]=values[i];}
    ++counters.channel_calls;

}
void dbt_lhdc_frame_stage_record(uint32_t prepare,uint32_t channels,uint32_t scan,uint32_t output){
    if(!active)return;
    const uint32_t values[4]={prepare,channels,scan,output};
    for(unsigned i=0;i<4;++i){frame_sum[i]+=values[i];if(values[i]>frame_max[i])frame_max[i]=values[i];}
    ++frame_calls;
    if(live_rate && !(frame_calls&15u))publish();
}
bool probe_lhdc_profile_finish(probe_lhdc_profile_t *result){
    if(!result)return false;
    active=false;*result=counters;
    if(live_rate)publish();
    return counters.channel_calls!=0;
}
bool probe_lhdc_profile_snapshot(probe_lhdc_live_t *result){
    if(!result)return false;
    for(unsigned attempt=0;attempt<3;++attempt){
        uint32_t r=atomic_load(&published.revision);if(!r || r&1u)continue;
        probe_lhdc_live_t v={0};v.revision=r;v.generation=atomic_load(&published.generation);v.rate=atomic_load(&published.rate);
        v.sample_bytes=atomic_load(&published.sample_bytes);v.collecting=atomic_load(&published.collecting);v.channel_calls=atomic_load(&published.channel_calls);
        for(unsigned i=0;i<6;++i){v.sum_cycles[i]=atomic_load(&published.sum_cycles[i]);v.max_cycles[i]=atomic_load(&published.max_cycles[i]);}
        v.frame_calls=atomic_load(&published.frame_calls);
        for(unsigned i=0;i<4;++i){v.frame_sum_cycles[i]=atomic_load(&published.frame_sum[i]);v.frame_max_cycles[i]=atomic_load(&published.frame_max[i]);}
        if(r==atomic_load(&published.revision) && v.rate && v.generation){*result=v;return true;}
    }
    return false;
}
