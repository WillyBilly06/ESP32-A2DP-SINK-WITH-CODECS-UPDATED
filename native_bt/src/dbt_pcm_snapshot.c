#include "dbt_pcm_snapshot.h"
#ifdef DBT_APTX_SNAPSHOT
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stddef.h>
#define CONTEXT_CAP 4608u
#define INPUT_CAP 1800u
#define PCM_CAP 1024u
/* Deliberate temporary diagnostic cost: fixed storage only in opted-in builds. */
static uint8_t context_blob[CONTEXT_CAP],input_blob[INPUT_CAP],raw_blob[PCM_CAP],ring_blob[PCM_CAP];
static fixed_dsp_t dsp_blob;
static int32_t dsp_input[256],dsp_output[256],final_output[256];
static _Atomic unsigned armed,decode_stage,ring_stage,dsp_stage,final_stage;
static bool started;static uint32_t started_ms;
static const void *origin;static const int32_t *final_origin;static const fixed_dsp_t *dsp_owner;
static uint32_t context_bytes,context_schema,input_bytes,processed_bytes,pcm_bytes,rate,decode_ms;
static uint32_t decode_copy_cycles,ring_copy_cycles,dsp_copy_cycles,final_copy_cycles;
static bool mix_present;
static uint32_t cycles(void){
#ifdef __XTENSA__
    uint32_t value;__asm__ volatile("rsr.ccount %0":"=a"(value));return value;
#else
    return 0;
#endif
}
static uint32_t hash_word(uint32_t h,size_t value){return (h^(uint32_t)value)*16777619u;}
uint32_t dbt_pcm_snapshot_dsp_schema(void){
    uint32_t h=2166136261u;
#define T(t) h=hash_word(h,sizeof(t))
#define F(t,f) h=hash_word(h,offsetof(t,f));h=hash_word(h,sizeof(((t*)0)->f))
    T(fixed_dsp_t);T(fdsp_coefficients_t);T(fdsp_history_t);T(fdsp_ramp_t);
    F(fdsp_coefficients_t,value);F(fdsp_coefficients_t,target);F(fdsp_coefficients_t,step);F(fdsp_coefficients_t,remainder);F(fdsp_coefficients_t,error);F(fdsp_coefficients_t,remaining);F(fdsp_coefficients_t,duration);
    F(fdsp_history_t,x1);F(fdsp_history_t,x2);F(fdsp_history_t,y1);F(fdsp_history_t,y2);F(fdsp_history_t,fraction);
    F(fdsp_ramp_t,value);F(fdsp_ramp_t,target);F(fdsp_ramp_t,step);F(fdsp_ramp_t,remainder);F(fdsp_ramp_t,error);F(fdsp_ramp_t,remaining);F(fdsp_ramp_t,duration);
    F(fixed_dsp_t,filters);F(fixed_dsp_t,cross_coefficients);F(fixed_dsp_t,channel_history);F(fixed_dsp_t,boost_history);F(fixed_dsp_t,cross_history);
    F(fixed_dsp_t,volume);F(fixed_dsp_t,bypass);F(fixed_dsp_t,flip);F(fixed_dsp_t,sample_rate);F(fixed_dsp_t,fade_frames);F(fixed_dsp_t,flags);
    F(fixed_dsp_t,peak_q27);F(fixed_dsp_t,limited);F(fixed_dsp_t,over_full_scale);F(fixed_dsp_t,rate_index);F(fixed_dsp_t,gains);F(fixed_dsp_t,requested_volume);
    F(fixed_dsp_t,decimation);F(fixed_dsp_t,analysis_count);F(fixed_dsp_t,window_count);F(fixed_dsp_t,window_frames);F(fixed_dsp_t,analysis_sum);F(fixed_dsp_t,resonators);
    F(fixed_dsp_t,analysis_coefficients);F(fixed_dsp_t,meter_states);F(fixed_dsp_t,envelopes);F(fixed_dsp_t,goertzel_levels);F(fixed_dsp_t,meter_levels);
#undef F
#undef T
    return h;
}
void dbt_pcm_snapshot_arm(void){atomic_store_explicit(&armed,1,memory_order_release);}
void dbt_pcm_snapshot_retire(void){
    /* Decoder owner only. Never let a reused codec scratch address match an old
     * pending handoff. Published snapshot storage has an independent lifetime. */
    started=false;
    if(atomic_load_explicit(&decode_stage,memory_order_acquire)==2 && !atomic_load(&ring_stage)){
        origin=NULL;atomic_store_explicit(&ring_stage,3,memory_order_release);
    }
}
bool dbt_pcm_snapshot_wanted(bool hd,uint32_t now){
    if(!hd || !atomic_load_explicit(&armed,memory_order_acquire) || atomic_load(&decode_stage))return false;
    if(!started){started=true;started_ms=now;return false;}
    return (uint32_t)(now-started_ms)>=2500u;
}
bool dbt_pcm_snapshot_decode_begin(const void *ctx,size_t size,uint32_t schema,
    const uint8_t *input,size_t count,const void *output,uint32_t hz,uint32_t now){
    if(atomic_load(&decode_stage) || !ctx || size>CONTEXT_CAP || !size || !input || !output || !count || count>INPUT_CAP)return false;
    atomic_store(&decode_stage,1);uint32_t begin=cycles();
    memcpy(context_blob,ctx,size);memcpy(input_blob,input,count);
    context_bytes=(uint32_t)size;context_schema=schema;input_bytes=(uint32_t)count;
    origin=output;rate=hz;decode_ms=now;decode_copy_cycles=cycles()-begin;return true;
}
void dbt_pcm_snapshot_decode_end(size_t used,size_t written){
    if(atomic_load(&decode_stage)!=1)return;
    processed_bytes=(uint32_t)used;
    if(!used || used>input_bytes || !written || written>PCM_CAP || written%8u){origin=NULL;atomic_store_explicit(&decode_stage,3,memory_order_release);return;}
    uint32_t begin=cycles();memcpy(raw_blob,origin,written);pcm_bytes=(uint32_t)written;
    decode_copy_cycles+=cycles()-begin;atomic_store_explicit(&decode_stage,2,memory_order_release);
}
void dbt_pcm_snapshot_handoff(const void *source,uint32_t frames,uint32_t stride,
    const void *first,size_t first_size,const void *second,size_t second_size){
    if(atomic_load_explicit(&decode_stage,memory_order_acquire)!=2 || atomic_load(&ring_stage) || source!=origin)return;
    atomic_store(&ring_stage,1);
    if(stride!=8 || frames!=pcm_bytes/8u || first_size>pcm_bytes || second_size!=pcm_bytes-first_size || (first_size && !first) || (second_size && !second)){origin=NULL;atomic_store_explicit(&ring_stage,3,memory_order_release);return;}
    uint32_t begin=cycles();memcpy(ring_blob,first,first_size);if(second_size)memcpy(ring_blob+first_size,second,second_size);
    ring_copy_cycles=cycles()-begin;origin=NULL;atomic_store_explicit(&ring_stage,2,memory_order_release);
}
bool dbt_pcm_snapshot_dsp_begin(const fixed_dsp_t *dsp,int32_t *pcm,unsigned frames,bool mix){
    if(!dsp || !pcm || frames!=128 || atomic_load_explicit(&ring_stage,memory_order_acquire)!=2 || atomic_load(&dsp_stage))return false;
    atomic_store(&dsp_stage,1);uint32_t begin=cycles();memcpy(&dsp_blob,dsp,sizeof(dsp_blob));memcpy(dsp_input,pcm,PCM_CAP);
    dsp_owner=dsp;final_origin=pcm;mix_present=mix;dsp_copy_cycles=cycles()-begin;return true;
}
void dbt_pcm_snapshot_dsp_end(const int32_t *pcm){
    if(atomic_load(&dsp_stage)!=1 || pcm!=final_origin)return;
    uint32_t begin=cycles();memcpy(dsp_output,pcm,PCM_CAP);dsp_copy_cycles+=cycles()-begin;
    atomic_store_explicit(&dsp_stage,2,memory_order_release);
}
void dbt_pcm_snapshot_final(const fixed_dsp_t *dsp){
    if(dsp!=dsp_owner || atomic_load_explicit(&dsp_stage,memory_order_acquire)!=2 || atomic_load(&final_stage))return;
    atomic_store(&final_stage,1);uint32_t begin=cycles();memcpy(final_output,final_origin,PCM_CAP);
    final_copy_cycles=cycles()-begin;final_origin=NULL;atomic_store_explicit(&final_stage,2,memory_order_release);
}
void dbt_pcm_snapshot_log(bool paused){
    static unsigned section,offset,mask;static bool header,done;
    if(!paused || done)return;
    unsigned ds=atomic_load_explicit(&decode_stage,memory_order_acquire),rs=atomic_load_explicit(&ring_stage,memory_order_acquire);
    unsigned ps=atomic_load_explicit(&dsp_stage,memory_order_acquire),fs=atomic_load_explicit(&final_stage,memory_order_acquire);
    if(!header){
        if(ds!=3 && !(ds==2 && (rs==3 || (rs==2 && ps==2 && fs==2))))return;
        mask=ds==2?7u:0u;if(rs==2)mask|=8u;if(ps==2)mask|=112u;if(fs==2)mask|=128u;
        printf("{\"probe\":\"aptx_pcm_snapshot\",\"schema\":1,\"codec_stage\":%u,\"ring_stage\":%u,\"dsp_stage\":%u,\"final_stage\":%u,\"sections\":%u,\"context_bytes\":%" PRIu32 ",\"context_schema\":%" PRIu32 ",\"dsp_bytes\":%u,\"dsp_schema\":%" PRIu32 ",\"input_bytes\":%" PRIu32 ",\"processed_bytes\":%" PRIu32 ",\"pcm_bytes\":%" PRIu32 ",\"rate\":%" PRIu32 ",\"decode_ms\":%" PRIu32 ",\"mix_hook\":%s,\"decode_copy_cycles\":%" PRIu32 ",\"ring_copy_cycles\":%" PRIu32 ",\"dsp_copy_cycles\":%" PRIu32 ",\"final_copy_cycles\":%" PRIu32 "}\n",ds,rs,ps,fs,mask,context_bytes,context_schema,(unsigned)sizeof(dsp_blob),dbt_pcm_snapshot_dsp_schema(),input_bytes,processed_bytes,pcm_bytes,rate,decode_ms,mix_present?"true":"false",decode_copy_cycles,ring_copy_cycles,dsp_copy_cycles,final_copy_cycles);
        header=true;
    }
    const uint8_t *data[8]={context_blob,input_blob,raw_blob,ring_blob,(const uint8_t *)&dsp_blob,(const uint8_t *)dsp_input,(const uint8_t *)dsp_output,(const uint8_t *)final_output};
    unsigned sizes[8]={context_bytes,input_bytes,pcm_bytes,pcm_bytes,sizeof(dsp_blob),PCM_CAP,PCM_CAP,PCM_CAP};
    static const char *const names[8]={"context","encoded","decoded","input_ring","dsp_state","dsp_input","dsp_output","final_output"};
    for(unsigned emitted=0;emitted<2;++emitted){
        /* Fixed eight-section traversal, no waiting for producers. */
        while(section<8 && (!(mask&(1u<<section)) || offset==sizes[section])){++section;offset=0;}
        if(section==8){printf("{\"probe\":\"aptx_pcm_snapshot_end\",\"sections\":%u}\n",mask);done=true;break;}
        unsigned count=sizes[section]-offset;if(count>64)count=64;
        static const char digits[]="0123456789abcdef";char hex[129];
        for(unsigned i=0;i<count;++i){uint8_t b=data[section][offset+i];hex[2*i]=digits[b>>4];hex[2*i+1]=digits[b&15];}hex[2*count]=0;
        printf("{\"probe\":\"aptx_pcm_snapshot_data\",\"section\":\"%s\",\"offset\":%u,\"hex\":\"%s\"}\n",names[section],offset,hex);offset+=count;
    }
}
#endif
