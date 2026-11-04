#include "probe_dsp_compare.h"
#include "probe_diagnostic_stats.h"
#include "fixed_dsp.h"
#include "esp_cpu.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>

#define DECLARE_VARIANT(name) \
bool probe_dsp_##name##_init(fixed_dsp_t *,uint32_t,uint32_t,uint32_t,uint32_t); \
void probe_dsp_##name##_controls(fixed_dsp_t *,int8_t,int8_t,int8_t,uint8_t,uint8_t,bool); \
void probe_dsp_##name##_process(fixed_dsp_t *,int32_t *,int32_t *)
DECLARE_VARIANT(os);
DECLARE_VARIANT(o3);
#undef DECLARE_VARIANT
enum {BLOCK_FRAMES=128,BLOCKS=40};
typedef struct {
    fixed_dsp_t state[2];
    int32_t pcm[2][BLOCK_FRAMES*2];
    uint32_t elapsed[2][BLOCKS];
} context_t;
typedef void (*process_fn)(fixed_dsp_t *,int32_t *,int32_t *);
static uint32_t random32(uint32_t *seed){
    uint32_t x=*seed;x^=x<<13;x^=x>>17;x^=x<<5;*seed=x;return x;
}
static uint32_t hash_bytes(uint32_t hash,const void *bytes,size_t count){
    const uint8_t *p=bytes;for(size_t i=0;i<count;++i)hash=(hash^p[i])*16777619u;return hash;
}
static uint32_t process(context_t *c,unsigned variant){
    process_fn fn=variant?probe_dsp_o3_process:probe_dsp_os_process;
    uint32_t began=esp_cpu_get_cycle_count();
    for(unsigned i=0;i<BLOCK_FRAMES;++i)fn(&c->state[variant],&c->pcm[variant][2*i],&c->pcm[variant][2*i+1u]);
    return esp_cpu_get_cycle_count()-began;
}
static bool one_case(context_t *c,unsigned trial,uint32_t rate,uint8_t flags,int64_t deadline){
    bool passed=probe_dsp_os_init(&c->state[0],rate,90,500,512) &&
                probe_dsp_o3_init(&c->state[1],rate,90,500,512);
    if(!passed)return false;
    probe_dsp_os_controls(&c->state[0],3,-2,2,50,flags,false);
    probe_dsp_o3_controls(&c->state[1],3,-2,2,50,flags,false);
    uint32_t seed=0x718392abu;uint32_t hash=2166136261u;
    unsigned completed=0;
    for(unsigned block=0;passed && block<BLOCKS;++block){
        if(esp_timer_get_time()>=deadline){passed=false;break;}
        for(unsigned i=0;i<BLOCK_FRAMES*2u;++i){
            /* Full-width deterministic signals, including rail alternation.
             * Both variants receive the exact same block and control state. */
            uint32_t value=random32(&seed);
            if(block%3u==0)value&=0xffffff00u;
            if(block%3u==1)value=i&1u?0x7fffffffu:0x80000000u;
            c->pcm[0][i]=c->pcm[1][i]=(int32_t)value;
        }
        /* Alternate execution order to reduce systematic cache/order bias. */
        unsigned first=(block+trial)&1u,second=first^1u;
        c->elapsed[first][block]=process(c,first);
        c->elapsed[second][block]=process(c,second);
        passed=!memcmp(c->pcm[0],c->pcm[1],sizeof(c->pcm[0])) &&
               !memcmp(&c->state[0],&c->state[1],sizeof(c->state[0]));
        hash=hash_bytes(hash,c->pcm[0],sizeof(c->pcm[0]));++completed;
        /* No waiting callback or queue polling. Let idle/timers run between
         * finite CPU-only work units; the sleep is outside measured spans. */
        TickType_t ticks=pdMS_TO_TICKS(1);vTaskDelay(ticks?ticks:1);
    }
    unsigned warm_blocks=(rate/50u+BLOCK_FRAMES-1u)/BLOCK_FRAMES;
    if(warm_blocks>=BLOCKS)return false;
    if(completed==BLOCKS){
        for(unsigned variant=0;variant<2;++variant){
            probe_diagnostic_stats_t warm={0},steady={0};
            bool stats=probe_diagnostic_stats(c->elapsed[variant],warm_blocks,160000,&warm) &&
                probe_diagnostic_stats(c->elapsed[variant]+warm_blocks,BLOCKS-warm_blocks,160000,&steady);
            passed=passed && stats;
            printf("{\"probe\":\"dsp_variant_case\",\"trial\":%u,\"rate\":%u,\"flags\":%u,\"variant\":\"%s\",\"equivalent\":%s,\"frames\":5120,\"block_frames\":128,\"ramp_containing_blocks\":%u,\"ramp_mean_cycles\":%u,\"ramp_max_cycles\":%u,\"steady_blocks\":%u,\"steady_mean_cycles\":%u,\"steady_p95_cycles\":%u,\"steady_max_cycles\":%u,\"pcm_hash\":%u,\"elapsed_core_cycles\":true,\"cpu_mhz\":240}\n",
                trial,(unsigned)rate,flags,variant?"O3":"Os",passed?"true":"false",warm_blocks,
                (unsigned)warm.mean,(unsigned)warm.maximum,BLOCKS-warm_blocks,(unsigned)steady.mean,
                (unsigned)steady.p95,(unsigned)steady.maximum,(unsigned)hash);
        }
    }
    return passed && completed==BLOCKS;
}
bool probe_dsp_compare(void){
    /* Allocate once after the real pipeline has retired; this allocation is
     * diagnostic workspace, never part of a live render/transport callback. */
    context_t *c=heap_caps_aligned_alloc(8,sizeof(*c),MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
    if(!c){printf("{\"probe\":\"dsp_variant_summary\",\"passed\":false,\"allocation_failed\":true}\n");return false;}
    const uint32_t rates[]={48000,96000,192000};unsigned completed=0;bool passed=true;
    int64_t deadline=esp_timer_get_time()+6000000;
    for(unsigned trial=0;passed && trial<3;++trial)
        for(unsigned i=0;passed && i<3;++i)
            for(unsigned policy=0;passed && policy<2;++policy){
                passed=one_case(c,trial,rates[i],policy?13:9,deadline);if(passed)++completed;
            }
    heap_caps_free(c);
    printf("{\"probe\":\"dsp_variant_summary\",\"passed\":%s,\"cases\":%u,\"expected_cases\":18,\"workspace_bytes\":%u,\"stack_low_water_bytes\":%u,\"controller_enabled\":false,\"i2s_test\":false,\"speed_is_not_a_pass_gate\":true}\n",
        passed?"true":"false",completed,(unsigned)sizeof(*c),(unsigned)uxTaskGetStackHighWaterMark(NULL));
    return passed && completed==18;
}
