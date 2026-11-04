#include "probe_fir_compare.h"
#include "probe_diagnostic_stats.h"
#include "dbt_resampler.h"
#include "esp_cpu.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>
#define DECLARE(name) \
void probe_fir_##name##_init(dbt_resampler_t *,uint32_t); \
void probe_fir_##name##_push(dbt_resampler_t *,int32_t,int32_t); \
void probe_fir_##name##_filter(dbt_resampler_t *,int32_t,int32_t,uint32_t,int32_t *,int32_t *)
DECLARE(wide);DECLARE(os);DECLARE(o2);DECLARE(o3);DECLARE(fold);
#undef DECLARE
enum {VARIANTS=5,BLOCK_FRAMES=128,BLOCKS=20,RAW_FRAMES=257};
typedef struct {
    void (*init)(dbt_resampler_t *,uint32_t);
    void (*push)(dbt_resampler_t *,int32_t,int32_t);
    void (*filter)(dbt_resampler_t *,int32_t,int32_t,uint32_t,int32_t *,int32_t *);
    const char *name;
} variant_t;
static const variant_t variants[]={
    {probe_fir_wide_init,probe_fir_wide_push,probe_fir_wide_filter,"O3_wide_reference"},
    {probe_fir_os_init,probe_fir_os_push,probe_fir_os_filter,"Os"},
    {probe_fir_o2_init,probe_fir_o2_push,probe_fir_o2_filter,"O2"},
    {probe_fir_o3_init,probe_fir_o3_push,probe_fir_o3_filter,"O3"},
    {probe_fir_fold_init,probe_fir_fold_push,probe_fir_fold_filter,"O3_folded_zero"}};
typedef struct {
    uint64_t alignment;
    dbt_resampler_t state[VARIANTS];
    int32_t raw[RAW_FRAMES*2],pcm[VARIANTS][BLOCK_FRAMES*2];
    uint32_t elapsed[VARIANTS][BLOCKS],phase[VARIANTS],consumed[VARIANTS];
} context_t;
static uint32_t random32(uint32_t *seed){uint32_t x=*seed;x^=x<<13;x^=x>>17;x^=x<<5;*seed=x;return x;}
static uint32_t process(context_t *c,unsigned variant,int32_t adjustment){
    const variant_t *v=&variants[variant];unsigned input=0;uint32_t phase=c->phase[variant];
    uint32_t began=esp_cpu_get_cycle_count();
    for(unsigned i=0;i<BLOCK_FRAMES;++i){
        v->filter(&c->state[variant],c->raw[2*input],c->raw[2*input+1u],phase,
                  &c->pcm[variant][2*i],&c->pcm[variant][2*i+1u]);
        int64_t next=(int64_t)phase+4294967296ll+adjustment;
        unsigned consumed=(unsigned)(next>>32);phase=(uint32_t)next;
        for(unsigned j=0;j<consumed;++j){v->push(&c->state[variant],c->raw[2*input],c->raw[2*input+1u]);++input;}
    }
    uint32_t elapsed=esp_cpu_get_cycle_count()-began;
    c->phase[variant]=phase;c->consumed[variant]=input;return elapsed;
}
static bool one_case(context_t *c,unsigned trial,uint32_t rate,int ppm,int64_t deadline){
    memset(c,0,sizeof(*c));
    for(unsigned v=0;v<VARIANTS;++v)variants[v].init(&c->state[v],rate);
    int32_t adjustment=(int32_t)(((int64_t)ppm*4294967296ll)/1000000ll);
    uint32_t seed=0x751391abu,hash=2166136261u,total_input=0;bool passed=true;unsigned completed=0;
    for(unsigned block=0;passed && block<BLOCKS;++block){
        if(esp_timer_get_time()>=deadline){passed=false;break;}
        for(unsigned i=0;i<RAW_FRAMES*2u;++i){
            uint32_t value=random32(&seed);
            if(block%3u==0)value&=0xffffff00u;
            if(block%3u==1)value=i&1u?0x7fffffffu:0x80000000u;
            c->raw[i]=(int32_t)value;
        }
        /* Rotate order per block and trial to reduce order/cache bias. */
        for(unsigned order=0;order<VARIANTS;++order){
            unsigned v=(order+block+trial)%VARIANTS;c->elapsed[v][block]=process(c,v,adjustment);
        }
        for(unsigned v=1;v<VARIANTS;++v)
            if(c->phase[v]!=c->phase[0] || c->consumed[v]!=c->consumed[0] ||
               memcmp(c->pcm[0],c->pcm[v],sizeof(c->pcm[0])) ||
               memcmp(&c->state[0],&c->state[v],sizeof(c->state[0])))passed=false;
        total_input+=c->consumed[0];
        const uint8_t *bytes=(const uint8_t *)c->pcm[0];
        for(unsigned i=0;i<sizeof(c->pcm[0]);++i)hash=(hash^bytes[i])*16777619u;
        ++completed;TickType_t ticks=pdMS_TO_TICKS(1);vTaskDelay(ticks?ticks:1);
    }
    if(completed==BLOCKS){
        for(unsigned v=0;v<VARIANTS;++v){
            uint32_t first=c->elapsed[v][0];probe_diagnostic_stats_t steady={0};
            if(!probe_diagnostic_stats(c->elapsed[v]+1,BLOCKS-1,160000,&steady))passed=false;
            printf("{\"probe\":\"fir_variant_case\",\"trial\":%u,\"rate\":%u,\"ppm\":%d,\"variant\":\"%s\",\"equivalent\":%s,\"frames\":2560,\"input_frames\":%u,\"block_frames\":128,\"first_block_cycles\":%u,\"steady_blocks\":19,\"steady_mean_cycles\":%u,\"steady_p95_cycles\":%u,\"steady_max_cycles\":%u,\"pcm_hash\":%u,\"includes_phase_and_history_wrapper\":true,\"elapsed_core_cycles\":true,\"cpu_mhz\":240}\n",
                trial,(unsigned)rate,ppm,variants[v].name,passed?"true":"false",(unsigned)total_input,
                (unsigned)first,(unsigned)steady.mean,(unsigned)steady.p95,(unsigned)steady.maximum,(unsigned)hash);
        }
    }
    return passed && completed==BLOCKS;
}
bool probe_fir_compare(void){
    context_t *c=heap_caps_aligned_alloc(8,sizeof(*c),MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
    if(!c){printf("{\"probe\":\"fir_variant_summary\",\"passed\":false,\"allocation_failed\":true}\n");return false;}
    const uint32_t rates[]={48000,96000,192000};const int drift[]={0,500,-500};
    bool passed=true;unsigned completed=0;int64_t deadline=esp_timer_get_time()+15000000;
    for(unsigned trial=0;passed && trial<3;++trial)
        for(unsigned r=0;passed && r<3;++r)
            for(unsigned d=0;passed && d<3;++d){passed=one_case(c,trial,rates[r],drift[d],deadline);if(passed)++completed;}
    heap_caps_free(c);
    printf("{\"probe\":\"fir_variant_summary\",\"passed\":%s,\"cases\":%u,\"expected_cases\":27,\"workspace_bytes\":%u,\"stack_low_water_bytes\":%u,\"controller_enabled\":false,\"i2s_test\":false,\"speed_is_not_a_pass_gate\":true}\n",
        passed?"true":"false",completed,(unsigned)sizeof(*c),(unsigned)uxTaskGetStackHighWaterMark(NULL));
    return passed && completed==27;
}
