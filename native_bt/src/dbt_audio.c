#include "dbt_audio.h"
#ifdef ESP_PLATFORM
#include "esp_attr.h"
#define AUDIO_ISR IRAM_ATTR
#else
#define AUDIO_ISR
#endif

static bool power2(uint32_t n){return n>=2 && !(n&(n-1u));}
static void zero(void *object,unsigned n){uint8_t *p=object;for(unsigned i=0;i<n;++i)p[i]=0;}
static void ring_init(dbt_audio_ring_t *ring,uint8_t *bytes,uint32_t frames,uint32_t stride){
    ring->bytes=bytes;ring->frames=frames;ring->mask=frames-1u;ring->stride=stride;
    atomic_init(&ring->written,0);atomic_init(&ring->read,0);
}
dbt_result_t dbt_audio_init(dbt_audio_t *a,dbt_audio_memory_t memory){
    if(!a || !memory.allocate || !memory.release)return DBT_INVALID;
    zero(a,sizeof(*a));a->memory=memory;
    atomic_init(&a->controls_generation,0);atomic_init(&a->controls_gains,0);
    atomic_init(&a->controls_options,6);atomic_init(&a->controls_lock,0);
    atomic_init(&a->submitted_frames,0);atomic_init(&a->rendered_frames,0);
    atomic_init(&a->taken_frames,0);atomic_init(&a->zero_frames,0);
    return DBT_OK;
}
void dbt_audio_close(dbt_audio_t *a){
    if(!a)return;
    if(a->input.bytes)a->memory.release(a->memory.context,a->input.bytes);
    if(a->output.bytes)a->memory.release(a->memory.context,a->output.bytes);
    a->input.bytes=a->output.bytes=0;a->opened=false;a->allocated_bytes=0;
}
dbt_result_t dbt_audio_open(dbt_audio_t *a,dbt_audio_config_t config){
    if(!a || !a->memory.allocate || !power2(config.input_frames) || !power2(config.output_frames) ||
       config.input_frames>32768 || config.output_frames>32768 || config.target_frames>=config.output_frames ||
       config.channels<1 || config.channels>2 || config.format<DBT_PCM_S16_LE || config.format>DBT_PCM_S32_LE)return DBT_INVALID;
    fixed_dsp_t check;
    if(!fixed_dsp_init(&check,config.rate,90,500,512))return DBT_UNSUPPORTED;
    dbt_audio_close(a);
    uint32_t stride=(uint32_t)config.format*config.channels;
    uint8_t *input=a->memory.allocate(a->memory.context,config.input_frames*stride);
    if(!input)return DBT_NO_SPACE;
    uint8_t *output=a->memory.allocate(a->memory.context,config.output_frames*8u);
    if(!output){a->memory.release(a->memory.context,input);return DBT_NO_SPACE;}
    if((uintptr_t)output%4u){a->memory.release(a->memory.context,input);a->memory.release(a->memory.context,output);return DBT_INVALID;}
    ring_init(&a->input,input,config.input_frames,stride);ring_init(&a->output,output,config.output_frames,8);
    a->config=config;a->allocated_bytes=config.input_frames*stride+config.output_frames*8u;
    (void)fixed_dsp_init(&a->dsp,config.rate,90,500,512);
    fixed_dsp_controls(&a->dsp,0,0,0,0,0,true);
    a->fraction=0;a->step_adjust=a->filtered_error_q8=a->integral_ppm=0;
    a->applied_generation=UINT32_MAX;a->opened=true;
    atomic_store(&a->submitted_frames,0);atomic_store(&a->rendered_frames,0);
    atomic_store(&a->taken_frames,0);atomic_store(&a->zero_frames,0);
    return DBT_OK;
}
uint32_t dbt_audio_input_frames(const dbt_audio_t *a){
    if(!a || !a->opened)return 0;
    return atomic_load_explicit(&a->input.written,memory_order_acquire)-atomic_load_explicit(&a->input.read,memory_order_acquire);
}
uint32_t dbt_audio_output_frames(const dbt_audio_t *a){
    if(!a || !a->opened)return 0;
    return atomic_load_explicit(&a->output.written,memory_order_acquire)-atomic_load_explicit(&a->output.read,memory_order_acquire);
}
dbt_result_t dbt_audio_submit(dbt_audio_t *a,const void *pcm,uint32_t frames){
    if(!a || !a->opened || (!pcm && frames) || frames>a->input.frames)return DBT_INVALID;
    uint32_t written=atomic_load_explicit(&a->input.written,memory_order_relaxed);
    uint32_t read=atomic_load_explicit(&a->input.read,memory_order_acquire);
    if(frames>a->input.frames-(written-read))return DBT_BUSY;
    const uint8_t *source=pcm;
    for(uint32_t i=0;i<frames;++i){
        uint8_t *to=a->input.bytes+((written+i)&a->input.mask)*a->input.stride;
        for(unsigned j=0;j<a->input.stride;++j)to[j]=source[i*a->input.stride+j];
    }
    atomic_store_explicit(&a->input.written,written+frames,memory_order_release);
    atomic_fetch_add_explicit(&a->submitted_frames,frames,memory_order_relaxed);return DBT_OK;
}
dbt_result_t dbt_audio_controls(dbt_audio_t *a,int8_t bass,int8_t mid,int8_t treble,uint8_t volume,uint8_t flags){
    if(!a || bass < -12 || bass>12 || mid < -12 || mid>12 || treble < -12 || treble>12 || volume>127 || (flags&~15u))return DBT_INVALID;
    uint32_t expected=0;
    if(!atomic_compare_exchange_strong_explicit(&a->controls_lock,&expected,1,memory_order_acquire,memory_order_relaxed))return DBT_BUSY;
    atomic_fetch_add_explicit(&a->controls_generation,1,memory_order_seq_cst);
    atomic_store_explicit(&a->controls_gains,(uint8_t)bass|((uint32_t)(uint8_t)mid<<8)|((uint32_t)(uint8_t)treble<<16),memory_order_seq_cst);
    atomic_store_explicit(&a->controls_options,volume|((uint32_t)flags<<8),memory_order_seq_cst);
    atomic_fetch_add_explicit(&a->controls_generation,1,memory_order_seq_cst);
    atomic_store_explicit(&a->controls_lock,0,memory_order_release);return DBT_OK;
}
static void controls(dbt_audio_t *a){
    for(unsigned i=0;i<3;++i){
        uint32_t generation=atomic_load_explicit(&a->controls_generation,memory_order_seq_cst);
        if((generation&1u) || generation==a->applied_generation)return;
        uint32_t gains=atomic_load_explicit(&a->controls_gains,memory_order_seq_cst);
        uint32_t options=atomic_load_explicit(&a->controls_options,memory_order_seq_cst);
        if(generation!=atomic_load_explicit(&a->controls_generation,memory_order_seq_cst))continue;
        fixed_dsp_controls(&a->dsp,(int8_t)gains,(int8_t)(gains>>8),(int8_t)(gains>>16),(uint8_t)options,(uint8_t)(options>>8),false);
        a->applied_generation=generation;return;
    }
}
static int32_t sample(const dbt_audio_t *a,uint32_t index,unsigned channel){
    const uint8_t *p=a->input.bytes+(index&a->input.mask)*a->input.stride+channel*(unsigned)a->config.format;
    uint32_t value=(uint32_t)p[0]|((uint32_t)p[1]<<8);
    if(a->config.format==DBT_PCM_S16_LE)return (int32_t)(int16_t)value*65536;
    value|=(uint32_t)p[2]<<16;
    if(a->config.format==DBT_PCM_S24_LE){if(value&0x800000u)value|=0xff000000u;return (int32_t)((int64_t)(int32_t)value*256);}
    return (int32_t)(value|((uint32_t)p[3]<<24));
}
static int32_t interpolate(int32_t current,int32_t next,uint32_t fraction){
    if(!fraction)return current;
    int64_t value=(int64_t)current*(int64_t)(4294967296ull-fraction)+(int64_t)next*fraction;
    return (int32_t)(value>>32);
}
uint32_t dbt_audio_render(dbt_audio_t *a,uint32_t budget){
    if(!a || !a->opened)return 0;
    controls(a);
    uint32_t in=atomic_load_explicit(&a->input.read,memory_order_relaxed);
    uint32_t end=atomic_load_explicit(&a->input.written,memory_order_acquire);
    uint32_t out=atomic_load_explicit(&a->output.written,memory_order_relaxed);
    uint32_t free=a->output.frames-(out-atomic_load_explicit(&a->output.read,memory_order_acquire));
    uint32_t produced=0;int32_t *destination=(int32_t *)a->output.bytes;
    while(produced<budget && produced<free && in!=end){
        int64_t next=(int64_t)a->fraction+4294967296ll+a->step_adjust;
        uint32_t consumed=(uint32_t)(next>>32);
        if((a->fraction || consumed==2) && end-in<2)break;
        int32_t left=sample(a,in,0),right=sample(a,in,a->config.channels==1?0:1);
        if(a->fraction){
            left=interpolate(left,sample(a,in+1u,0),a->fraction);
            right=interpolate(right,sample(a,in+1u,a->config.channels==1?0:1),a->fraction);
        }
        fixed_dsp_process(&a->dsp,&left,&right);
        uint32_t slot=((out+produced)&a->output.mask)*2u;destination[slot]=left;destination[slot+1u]=right;
        in+=consumed;a->fraction=(uint32_t)next;++produced;
    }
    atomic_store_explicit(&a->input.read,in,memory_order_release);
    atomic_store_explicit(&a->output.written,out+produced,memory_order_release);
    atomic_fetch_add_explicit(&a->rendered_frames,produced,memory_order_relaxed);return produced;
}
void dbt_audio_clock(dbt_audio_t *a){
    if(!a || !a->opened || !a->config.adaptive_clock)return;
    int32_t target=(int32_t)a->config.target_frames+(int32_t)(a->config.rate/250u);
    int32_t error=(int32_t)(dbt_audio_input_frames(a)+dbt_audio_output_frames(a))-target;
    a->filtered_error_q8+=(error*256-a->filtered_error_q8)/8;
    int32_t filtered=a->filtered_error_q8/256;
    a->integral_ppm+=filtered/64;
    if(a->integral_ppm>200)a->integral_ppm=200;
    if(a->integral_ppm < -200)a->integral_ppm=-200;
    int32_t ppm=filtered/4+a->integral_ppm;
    if(ppm>500)ppm=500;
    if(ppm < -500)ppm=-500;
    a->step_adjust=(int32_t)((int64_t)ppm*4294967296ll/1000000);
}
uint32_t AUDIO_ISR dbt_audio_take(dbt_audio_t *a,int32_t *destination,uint32_t frames){
    if(!a || !destination || !a->opened)return 0;
    uint32_t read=atomic_load_explicit(&a->output.read,memory_order_relaxed);
    uint32_t count=atomic_load_explicit(&a->output.written,memory_order_acquire)-read;
    if(count>frames)count=frames;
    const int32_t *source=(const int32_t *)a->output.bytes;
    for(uint32_t i=0;i<count;++i){uint32_t index=((read+i)&a->output.mask)*2u;destination[2*i]=source[index];destination[2*i+1u]=source[index+1u];}
    for(uint32_t i=count;i<frames;++i){destination[2*i]=0;destination[2*i+1u]=0;}
    atomic_store_explicit(&a->output.read,read+count,memory_order_release);
    atomic_store_explicit(&a->taken_frames,atomic_load_explicit(&a->taken_frames,memory_order_relaxed)+count,memory_order_relaxed);
    atomic_store_explicit(&a->zero_frames,atomic_load_explicit(&a->zero_frames,memory_order_relaxed)+frames-count,memory_order_relaxed);return count;
}
void dbt_audio_fade_init(dbt_audio_fade_t *fade,uint32_t frames){
    if(!fade)return;
    if(!frames || frames>192000u)frames=1;
    fade->total=fade->remaining=frames;fade->gain=32768;
    fade->step=32768/(int32_t)frames;fade->remainder=32768%(int32_t)frames;fade->error=0;
}
static inline __attribute__((always_inline)) int32_t gain_q15(int32_t sample_value,int32_t gain){
    int32_t high=sample_value>>15;uint32_t low=(uint32_t)sample_value&32767u;
    return high*gain+(int32_t)((low*(uint32_t)gain)>>15);
}
void AUDIO_ISR dbt_audio_fade(dbt_audio_fade_t *fade,int32_t *stereo,uint32_t frames){
    if(!fade || !stereo)return;
    for(uint32_t i=0;i<frames;++i){
        if(!fade->remaining){stereo[2*i]=stereo[2*i+1u]=0;continue;}
        stereo[2*i]=gain_q15(stereo[2*i],fade->gain);
        stereo[2*i+1u]=gain_q15(stereo[2*i+1u],fade->gain);
        fade->gain-=fade->step;fade->error+=fade->remainder;
        if(fade->error>=(int32_t)fade->total){--fade->gain;fade->error-=(int32_t)fade->total;}
        if(!--fade->remaining)fade->gain=0;
    }
}
