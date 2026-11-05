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
#ifdef DBT_DSP_STAGE_PROFILE
DECLARE_VARIANT(profile);
#include "probe_dsp_profile.h"
#define SECOND_INIT probe_dsp_profile_init
#define SECOND_CONTROLS probe_dsp_profile_controls
#define SECOND_PROCESS probe_dsp_profile_process
#define SECOND_NAME "Os_profiled"
#define REFERENCE_NAME "Os_split_reference"
#elif defined(DBT_PROBE_DSP_INLINE_CHECK)
DECLARE_VARIANT(inline);
#define SECOND_INIT probe_dsp_inline_init
#define SECOND_CONTROLS probe_dsp_inline_controls
#define SECOND_PROCESS probe_dsp_inline_process
#define SECOND_NAME "Os_force_inline"
#define REFERENCE_NAME "Os_current"
#else
DECLARE_VARIANT(o3);
DECLARE_VARIANT(block);
DECLARE_VARIANT(word);
bool probe_dsp_block_process_block(fixed_dsp_t *,int32_t *,uint16_t);
bool probe_dsp_word_process_block(fixed_dsp_t *,int32_t *,uint16_t);
#define HAVE_BLOCK_VARIANT 1
#define SECOND_INIT probe_dsp_o3_init
#define SECOND_CONTROLS probe_dsp_o3_controls
#define SECOND_PROCESS probe_dsp_o3_process
#define SECOND_NAME "O3"
#define REFERENCE_NAME "Os"
#endif
#undef DECLARE_VARIANT
enum {BLOCK_FRAMES=128,BLOCKS=40,
#ifdef HAVE_BLOCK_VARIANT
    VARIANTS=4,POLICIES=3
#else
    VARIANTS=2,POLICIES=2
#endif
};
typedef struct {
    fixed_dsp_t state[VARIANTS];
    int32_t pcm[VARIANTS][BLOCK_FRAMES*2];
    uint32_t elapsed[VARIANTS][BLOCKS];
    bool block_valid;
#ifdef DBT_DSP_STAGE_PROFILE
    uint32_t stages[5][BLOCKS];
    bool stages_valid;
#endif
} context_t;
typedef void (*process_fn)(fixed_dsp_t *,int32_t *,int32_t *);
static uint32_t random32(uint32_t *seed){
    uint32_t x=*seed;x^=x<<13;x^=x>>17;x^=x<<5;*seed=x;return x;
}
static uint32_t hash_bytes(uint32_t hash,const void *bytes,size_t count){
    const uint8_t *p=bytes;for(size_t i=0;i<count;++i)hash=(hash^p[i])*16777619u;return hash;
}
static uint32_t process(context_t *c,unsigned variant,unsigned block){
    (void)block;
    process_fn fn=variant?SECOND_PROCESS:probe_dsp_os_process;
#ifdef DBT_DSP_STAGE_PROFILE
    if(variant)probe_dsp_profile_reset();
#endif
    uint32_t began=esp_cpu_get_cycle_count();
#ifdef HAVE_BLOCK_VARIANT
    if(variant==2){if(!probe_dsp_block_process_block(&c->state[variant],c->pcm[variant],BLOCK_FRAMES))c->block_valid=false;}
    else if(variant==3){if(!probe_dsp_word_process_block(&c->state[variant],c->pcm[variant],BLOCK_FRAMES))c->block_valid=false;}
    else
#endif
    for(unsigned i=0;i<BLOCK_FRAMES;++i)fn(&c->state[variant],&c->pcm[variant][2*i],&c->pcm[variant][2*i+1u]);
    uint32_t elapsed=esp_cpu_get_cycle_count()-began;
#ifdef DBT_DSP_STAGE_PROFILE
    if(variant){
        probe_dsp_profile_t data;probe_dsp_profile_snapshot(&data);
        if(data.frames!=BLOCK_FRAMES)c->stages_valid=false;
        for(unsigned i=0;i<5;++i)c->stages[i][block]=data.cycles[i];
    }
#endif
    return elapsed;
}
static bool one_case(context_t *c,unsigned trial,uint32_t rate,uint8_t flags,bool flat,int64_t deadline){
    bool passed=probe_dsp_os_init(&c->state[0],rate,90,500,512) &&
                SECOND_INIT(&c->state[1],rate,90,500,512);
    if(!passed)return false;
    c->block_valid=true;
#ifdef HAVE_BLOCK_VARIANT
    if(!probe_dsp_block_init(&c->state[2],rate,90,500,512))return false;
    if(!probe_dsp_word_init(&c->state[3],rate,90,500,512))return false;
#endif
#ifdef DBT_DSP_STAGE_PROFILE
    /* Match the real audio-open muted baseline before applying its snapshot. */
    probe_dsp_os_controls(&c->state[0],0,0,0,0,0,true);
    SECOND_CONTROLS(&c->state[1],0,0,0,0,0,true);
    c->stages_valid=true;
#endif
    int8_t bass=flat?0:3,mid=flat?0:-2,treble=flat?0:2;
    probe_dsp_os_controls(&c->state[0],bass,mid,treble,50,flags,false);
    SECOND_CONTROLS(&c->state[1],bass,mid,treble,50,flags,false);
#ifdef HAVE_BLOCK_VARIANT
    probe_dsp_block_controls(&c->state[2],bass,mid,treble,50,flags,false);
    probe_dsp_word_controls(&c->state[3],bass,mid,treble,50,flags,false);
#endif
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
            for(unsigned v=0;v<VARIANTS;++v)c->pcm[v][i]=(int32_t)value;
        }
        /* Alternate execution order to reduce systematic cache/order bias. */
        unsigned first=(block+trial)%VARIANTS;
        for(unsigned i=0;i<VARIANTS;++i){unsigned v=(first+i)%VARIANTS;c->elapsed[v][block]=process(c,v,block);}
        passed=c->block_valid;
        for(unsigned v=1;v<VARIANTS;++v)passed=passed && !memcmp(c->pcm[0],c->pcm[v],sizeof(c->pcm[0])) &&
               !memcmp(&c->state[0],&c->state[v],sizeof(c->state[0]));
        hash=hash_bytes(hash,c->pcm[0],sizeof(c->pcm[0]));++completed;
        /* No waiting callback or queue polling. Let idle/timers run between
         * finite CPU-only work units; the sleep is outside measured spans. */
        TickType_t ticks=pdMS_TO_TICKS(1);vTaskDelay(ticks?ticks:1);
    }
    unsigned warm_blocks=(rate/50u+BLOCK_FRAMES-1u)/BLOCK_FRAMES;
    if(warm_blocks>=BLOCKS)return false;
    if(completed==BLOCKS){
        for(unsigned variant=0;variant<VARIANTS;++variant){
            probe_diagnostic_stats_t warm={0},steady={0};
            bool stats=probe_diagnostic_stats(c->elapsed[variant],warm_blocks,160000,&warm) &&
                probe_diagnostic_stats(c->elapsed[variant]+warm_blocks,BLOCKS-warm_blocks,160000,&steady);
            passed=passed && stats;
            const char *name=variant?SECOND_NAME:REFERENCE_NAME;
#ifdef HAVE_BLOCK_VARIANT
            if(variant==2)name="Os_block";
            if(variant==3)name="Os_word_ramp_block";
#endif
            printf("{\"probe\":\"dsp_variant_case\",\"trial\":%u,\"rate\":%u,\"flags\":%u,\"gains\":[%d,%d,%d],\"variant\":\"%s\",\"equivalent\":%s,\"frames\":5120,\"block_frames\":128,\"ramp_containing_blocks\":%u,\"ramp_mean_cycles\":%u,\"ramp_max_cycles\":%u,\"steady_blocks\":%u,\"steady_mean_cycles\":%u,\"steady_p95_cycles\":%u,\"steady_max_cycles\":%u,\"pcm_hash\":%u,\"elapsed_core_cycles\":true,\"cpu_mhz\":240}\n",
                trial,(unsigned)rate,flags,bass,mid,treble,name,passed?"true":"false",warm_blocks,
                (unsigned)warm.mean,(unsigned)warm.maximum,BLOCKS-warm_blocks,(unsigned)steady.mean,
                (unsigned)steady.p95,(unsigned)steady.maximum,(unsigned)hash);
        }
#ifdef DBT_DSP_STAGE_PROFILE
        passed=passed && c->stages_valid;
        uint32_t warm_mean[5],steady_mean[5],steady_max[5];
        for(unsigned i=0;i<5;++i){
            probe_diagnostic_stats_t w={0},s={0};
            if(!probe_diagnostic_stats(c->stages[i],warm_blocks,160000,&w) ||
               !probe_diagnostic_stats(c->stages[i]+warm_blocks,BLOCKS-warm_blocks,160000,&s))passed=false;
            warm_mean[i]=w.mean;steady_mean[i]=s.mean;steady_max[i]=s.maximum;
        }
        printf("{\"probe\":\"dsp_internal_stages\",\"trial\":%u,\"rate\":%u,\"flags\":%u,\"valid\":%s,\"stages\":[\"analysis\",\"ramps_input\",\"eq_comp\",\"cross_boost\",\"mix_limit\"],\"ramp_mean_cycles\":[%u,%u,%u,%u,%u],\"steady_mean_cycles\":[%u,%u,%u,%u,%u],\"steady_max_cycles\":[%u,%u,%u,%u,%u],\"block_frames\":128,\"includes_marker_overhead\":true,\"pipeline_muted_start\":true,\"proven_sum_enabled\":%u}\n",
            trial,(unsigned)rate,flags,passed?"true":"false",
            (unsigned)warm_mean[0],(unsigned)warm_mean[1],(unsigned)warm_mean[2],(unsigned)warm_mean[3],(unsigned)warm_mean[4],
            (unsigned)steady_mean[0],(unsigned)steady_mean[1],(unsigned)steady_mean[2],(unsigned)steady_mean[3],(unsigned)steady_mean[4],
            (unsigned)steady_max[0],(unsigned)steady_max[1],(unsigned)steady_max[2],(unsigned)steady_max[3],(unsigned)steady_max[4],probe_dsp_profile_sum_enabled());
#endif
    }
    return passed && completed==BLOCKS;
}
bool probe_dsp_compare(void){
    /* Allocate once after the real pipeline has retired; this allocation is
     * diagnostic workspace, never part of a live render/transport callback. */
    context_t *c=heap_caps_aligned_alloc(8,sizeof(*c),MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
    if(!c){printf("{\"probe\":\"dsp_variant_summary\",\"passed\":false,\"allocation_failed\":true}\n");return false;}
    const uint32_t rates[]={48000,96000,192000};unsigned completed=0;bool passed=true;
    int64_t deadline=esp_timer_get_time()+(POLICIES==3?14000000:6000000);
    for(unsigned trial=0;passed && trial<3;++trial)
        for(unsigned i=0;passed && i<3;++i)
            for(unsigned policy=0;passed && policy<POLICIES;++policy){
                passed=one_case(c,trial,rates[i],policy==2?8:policy?13:9,policy==2,deadline);if(passed)++completed;
            }
    heap_caps_free(c);
    printf("{\"probe\":\"dsp_variant_summary\",\"passed\":%s,\"cases\":%u,\"expected_cases\":%u,\"variants\":%u,\"workspace_bytes\":%u,\"stack_low_water_bytes\":%u,\"controller_enabled\":false,\"i2s_test\":false,\"speed_is_not_a_pass_gate\":true}\n",
        passed?"true":"false",completed,9u*POLICIES,VARIANTS,(unsigned)sizeof(*c),(unsigned)uxTaskGetStackHighWaterMark(NULL));
    return passed && completed==9u*POLICIES;
}
