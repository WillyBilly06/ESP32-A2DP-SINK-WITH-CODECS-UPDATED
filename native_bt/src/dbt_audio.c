#include "dbt_audio.h"
#ifdef ESP_PLATFORM
#include "esp_attr.h"
#define AUDIO_ISR IRAM_ATTR
#define AUDIO_CONTROL IRAM_ATTR
#else
#define AUDIO_ISR
#define AUDIO_CONTROL
#endif

/* A task that owns controls_lock must not be descheduled on its own CPU
 * before publication completes. Local interrupt masking is nested and does
 * not wait for the other CPU; the single CAS still reports BUSY immediately.
 * The actual protected section contains only bounded atomic operations. */
#if defined(DBT_AUDIO_CONTROL_TEST_IRQ)
extern uint32_t dbt_audio_test_mask_enter(void);
extern void dbt_audio_test_mask_exit(uint32_t);
extern void dbt_audio_test_publication_step(dbt_audio_t *);
#define CONTROL_MASK_ENTER() dbt_audio_test_mask_enter()
#define CONTROL_MASK_EXIT(x) dbt_audio_test_mask_exit(x)
#define CONTROL_TEST_STEP(a) dbt_audio_test_publication_step(a)
#elif defined(ESP_PLATFORM) && defined(__XTENSA__)
#include <xtensa/xtruntime.h>
#define CONTROL_MASK_ENTER() XTOS_SET_MIN_INTLEVEL(XCHAL_EXCM_LEVEL)
#define CONTROL_MASK_EXIT(x) XTOS_RESTORE_JUST_INTLEVEL(x)
#define CONTROL_TEST_STEP(a) ((void)0)
#elif defined(ESP_PLATFORM)
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#define CONTROL_MASK_ENTER() portSET_INTERRUPT_MASK_FROM_ISR()
#define CONTROL_MASK_EXIT(x) portCLEAR_INTERRUPT_MASK_FROM_ISR(x)
#define CONTROL_TEST_STEP(a) ((void)0)
#else
#define CONTROL_MASK_ENTER() 0u
#define CONTROL_MASK_EXIT(x) ((void)(x))
#define CONTROL_TEST_STEP(a) ((void)0)
#endif

static AUDIO_CONTROL bool controls_enter(dbt_audio_t *a,uint32_t *mask){
    *mask=CONTROL_MASK_ENTER();
    uint32_t expected=0;
    if(atomic_compare_exchange_strong_explicit(&a->controls_lock,&expected,1,memory_order_acquire,memory_order_relaxed))return true;
    CONTROL_MASK_EXIT(*mask);return false;
}
static AUDIO_CONTROL void controls_exit(dbt_audio_t *a,uint32_t mask){
    atomic_store_explicit(&a->controls_lock,0,memory_order_release);
    CONTROL_MASK_EXIT(mask);
}

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
    atomic_init(&a->producer_backpressure,0);atomic_init(&a->clock_version,0);
    atomic_init(&a->clock_pub_ppm,0);atomic_init(&a->clock_pub_observed_ppm,0);atomic_init(&a->clock_pub_error,0);
    atomic_init(&a->clock_pub_state,DBT_AUDIO_CLOCK_OFF);atomic_init(&a->clock_pub_target,0);atomic_init(&a->clock_pub_updates,0);
    atomic_init(&a->clock_pub_disturbances,0);atomic_init(&a->clock_pub_observed_valid,0);
    return DBT_OK;
}
void dbt_audio_close(dbt_audio_t *a){
    if(!a)return;
    if(a->input.bytes)a->memory.release(a->memory.context,a->input.bytes);
    if(a->output.bytes)a->memory.release(a->memory.context,a->output.bytes);
    a->input.bytes=a->output.bytes=0;a->opened=false;a->allocated_bytes=0;
    dbt_audio_clock_reset(a,0);
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
    dbt_resampler_init(&a->resampler,config.rate);a->draining_input=false;a->drain_remaining=0;
    a->applied_generation=UINT32_MAX;a->opened=true;
    atomic_store(&a->submitted_frames,0);atomic_store(&a->rendered_frames,0);
    atomic_store(&a->taken_frames,0);atomic_store(&a->zero_frames,0);atomic_store(&a->producer_backpressure,0);
    dbt_audio_clock_reset(a,0);a->clock_primed=false;
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
    if(frames>a->input.frames-(written-read)){
        atomic_fetch_add_explicit(&a->producer_backpressure,1,memory_order_relaxed);return DBT_BUSY;
    }
    const uint8_t *source=pcm;
    for(uint32_t i=0;i<frames;++i){
        uint8_t *to=a->input.bytes+((written+i)&a->input.mask)*a->input.stride;
        for(unsigned j=0;j<a->input.stride;++j)to[j]=source[i*a->input.stride+j];
    }
    atomic_store_explicit(&a->input.written,written+frames,memory_order_release);
    atomic_fetch_add_explicit(&a->submitted_frames,frames,memory_order_relaxed);return DBT_OK;
}
dbt_result_t AUDIO_CONTROL dbt_audio_controls(dbt_audio_t *a,int8_t bass,int8_t mid,int8_t treble,uint8_t volume,uint8_t flags){
    if(!a || bass < -12 || bass>12 || mid < -12 || mid>12 || treble < -12 || treble>12 || volume>127 || (flags&~15u))return DBT_INVALID;
    uint32_t irq_mask;if(!controls_enter(a,&irq_mask))return DBT_BUSY;
    atomic_fetch_add_explicit(&a->controls_generation,1,memory_order_seq_cst);
    CONTROL_TEST_STEP(a);
    atomic_store_explicit(&a->controls_gains,(uint8_t)bass|((uint32_t)(uint8_t)mid<<8)|((uint32_t)(uint8_t)treble<<16),memory_order_seq_cst);
    atomic_store_explicit(&a->controls_options,volume|((uint32_t)flags<<8),memory_order_seq_cst);
    atomic_fetch_add_explicit(&a->controls_generation,1,memory_order_seq_cst);
    controls_exit(a,irq_mask);return DBT_OK;
}
dbt_result_t AUDIO_CONTROL dbt_audio_volume(dbt_audio_t *a,uint8_t volume){
    if(!a || volume>127)return DBT_INVALID;
    uint32_t irq_mask;if(!controls_enter(a,&irq_mask))return DBT_BUSY;
    atomic_fetch_add_explicit(&a->controls_generation,1,memory_order_seq_cst);
    CONTROL_TEST_STEP(a);
    uint32_t options=atomic_load_explicit(&a->controls_options,memory_order_seq_cst);
    atomic_store_explicit(&a->controls_options,(options&~255u)|volume,memory_order_seq_cst);
    atomic_fetch_add_explicit(&a->controls_generation,1,memory_order_seq_cst);
    controls_exit(a,irq_mask);return DBT_OK;
}
dbt_result_t AUDIO_CONTROL dbt_audio_eq(dbt_audio_t *a,int8_t bass,int8_t mid,int8_t treble){
    if(!a || bass < -12 || bass>12 || mid < -12 || mid>12 || treble < -12 || treble>12)return DBT_INVALID;
    uint32_t irq_mask;if(!controls_enter(a,&irq_mask))return DBT_BUSY;
    atomic_fetch_add_explicit(&a->controls_generation,1,memory_order_seq_cst);
    CONTROL_TEST_STEP(a);
    atomic_store_explicit(&a->controls_gains,(uint8_t)bass|((uint32_t)(uint8_t)mid<<8)|((uint32_t)(uint8_t)treble<<16),memory_order_seq_cst);
    atomic_fetch_add_explicit(&a->controls_generation,1,memory_order_seq_cst);
    controls_exit(a,irq_mask);return DBT_OK;
}
static AUDIO_CONTROL dbt_result_t update_options(dbt_audio_t *a,uint32_t mask,uint32_t value){
    if(!a)return DBT_INVALID;
    uint32_t irq_mask;if(!controls_enter(a,&irq_mask))return DBT_BUSY;
    atomic_fetch_add_explicit(&a->controls_generation,1,memory_order_seq_cst);
    CONTROL_TEST_STEP(a);
    uint32_t previous=atomic_load_explicit(&a->controls_options,memory_order_seq_cst);
    atomic_store_explicit(&a->controls_options,(previous&~mask)|(value&mask),memory_order_seq_cst);
    atomic_fetch_add_explicit(&a->controls_generation,1,memory_order_seq_cst);
    controls_exit(a,irq_mask);return DBT_OK;
}
dbt_result_t dbt_audio_control_flags(dbt_audio_t *a,uint8_t flags){
    return flags>7?DBT_INVALID:update_options(a,7u<<8,(uint32_t)flags<<8);
}
dbt_result_t dbt_audio_analysis(dbt_audio_t *a,bool enabled){return update_options(a,8u<<8,enabled?8u<<8:0);}
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
    while(produced<budget && produced<free){
        uint32_t available=end-in;
        if(!available && !a->drain_remaining)break;
        int64_t next=(int64_t)a->fraction+4294967296ll+a->step_adjust;
        uint32_t consumed=(uint32_t)(next>>32);
        if(consumed>available+a->drain_remaining ||
           (!a->config.adaptive_clock && a->fraction && available<2))break;
        uint8_t last=(uint8_t)((a->resampler.next-1u)&(DBT_RESAMPLER_HISTORY-1u));
        int32_t left=available?sample(a,in,0):a->resampler.history[0][last];
        int32_t right=available?sample(a,in,a->config.channels==1?0:1):a->resampler.history[1][last];
        if(a->config.adaptive_clock){
            int32_t filtered_left,filtered_right;
            dbt_resampler_filter(&a->resampler,left,right,a->fraction,&filtered_left,&filtered_right);
            for(unsigned i=0;i<consumed;i++){
                int32_t history_left,history_right;
                if(in!=end){history_left=sample(a,in,0);history_right=sample(a,in,a->config.channels==1?0:1);++in;}
                else {uint8_t index=(uint8_t)((a->resampler.next-1u)&(DBT_RESAMPLER_HISTORY-1u));
                    history_left=a->resampler.history[0][index];history_right=a->resampler.history[1][index];--a->drain_remaining;}
                dbt_resampler_push(&a->resampler,history_left,history_right);
            }
            left=filtered_left;right=filtered_right;
        }else{
            if(a->fraction){
                left=interpolate(left,sample(a,in+1u,0),a->fraction);
                right=interpolate(right,sample(a,in+1u,a->config.channels==1?0:1),a->fraction);
            }
            in+=consumed;
        }
        fixed_dsp_process(&a->dsp,&left,&right);
        uint32_t slot=((out+produced)&a->output.mask)*2u;destination[slot]=left;destination[slot+1u]=right;
        a->fraction=(uint32_t)next;++produced;
    }
    atomic_store_explicit(&a->input.read,in,memory_order_release);
    atomic_store_explicit(&a->output.written,out+produced,memory_order_release);
    atomic_fetch_add_explicit(&a->rendered_frames,produced,memory_order_relaxed);return produced;
}
void dbt_audio_begin_drain(dbt_audio_t *a){
    if(!a || !a->opened || a->draining_input)return;
    a->draining_input=true;
    if(a->config.adaptive_clock && (a->resampler.primed || dbt_audio_input_frames(a)))
        a->drain_remaining=dbt_resampler_delay_frames(a->config.rate);
}

/* Clock policy is owner-only. Source/consumer counters are atomic because
 * producer and ISR run independently. Occupancy is calibrated to the actual
 * packet cadence, rather than assuming a codec frame is four milliseconds. */
static void clock_publish(dbt_audio_t *a,int32_t error){
    atomic_fetch_add_explicit(&a->clock_version,1,memory_order_acq_rel);
    atomic_store_explicit(&a->clock_pub_ppm,a->clock_ppm,memory_order_relaxed);
    atomic_store_explicit(&a->clock_pub_observed_ppm,a->clock_observed_ppm,memory_order_relaxed);
    atomic_store_explicit(&a->clock_pub_error,error,memory_order_relaxed);
    atomic_store_explicit(&a->clock_pub_state,(uint32_t)a->clock_state,memory_order_relaxed);
    atomic_store_explicit(&a->clock_pub_target,a->clock_target,memory_order_relaxed);
    atomic_store_explicit(&a->clock_pub_updates,a->clock_updates,memory_order_relaxed);
    atomic_store_explicit(&a->clock_pub_disturbances,a->clock_disturbances,memory_order_relaxed);
    atomic_store_explicit(&a->clock_pub_observed_valid,a->clock_observed_valid,memory_order_relaxed);
    atomic_fetch_add_explicit(&a->clock_version,1,memory_order_release);
}
bool dbt_audio_clock_snapshot(const dbt_audio_t *a,dbt_audio_clock_stats_t *out){
    if(!a || !out)return false;
    for(unsigned i=0;i<3;++i){
        uint32_t before=atomic_load_explicit(&a->clock_version,memory_order_acquire);
        if(before&1u)continue;
        out->ppm=atomic_load_explicit(&a->clock_pub_ppm,memory_order_relaxed);
        out->observed_ppm=atomic_load_explicit(&a->clock_pub_observed_ppm,memory_order_relaxed);
        out->error_frames=atomic_load_explicit(&a->clock_pub_error,memory_order_relaxed);
        out->state=atomic_load_explicit(&a->clock_pub_state,memory_order_relaxed);
        out->target_frames=atomic_load_explicit(&a->clock_pub_target,memory_order_relaxed);
        out->updates=atomic_load_explicit(&a->clock_pub_updates,memory_order_relaxed);
        out->disturbances=atomic_load_explicit(&a->clock_pub_disturbances,memory_order_relaxed);
        out->observed_valid=atomic_load_explicit(&a->clock_pub_observed_valid,memory_order_relaxed);
        atomic_thread_fence(memory_order_acquire);
        if(before==atomic_load_explicit(&a->clock_version,memory_order_relaxed))return true;
    }
    return false;
}
void dbt_audio_clock_reset(dbt_audio_t *a,uint32_t now){
    if(!a)return;
    a->clock_last_ms=now;a->clock_submitted=atomic_load_explicit(&a->submitted_frames,memory_order_acquire);
    a->clock_rendered=atomic_load_explicit(&a->rendered_frames,memory_order_acquire);
    a->clock_taken=atomic_load_explicit(&a->taken_frames,memory_order_acquire);
    a->clock_zero=atomic_load_explicit(&a->zero_frames,memory_order_acquire);
    a->clock_busy=atomic_load_explicit(&a->producer_backpressure,memory_order_acquire);
    a->clock_target=a->clock_calibration_ms=a->clock_clean_ms=a->clock_window_ms=0;
    a->clock_calibration_sum=a->clock_window_source=a->clock_window_sink=0;
    a->clock_integral_q16=a->integral_ppm=a->filtered_error_q8=a->clock_ppm=a->clock_observed_ppm=a->step_adjust=0;
    a->clock_updates=a->clock_disturbances=0;a->clock_observed_valid=false;
    a->clock_primed=a->opened && a->config.adaptive_clock;
    a->clock_state=a->clock_primed?DBT_AUDIO_CLOCK_CALIBRATING:DBT_AUDIO_CLOCK_OFF;clock_publish(a,0);
}
static int32_t clamp_ppm(int32_t p){return p>500?500:p< -500?-500:p;}
static void clock_slew(dbt_audio_t *a,int32_t wanted,uint32_t dt){
    int32_t limit=(int32_t)((50u*dt)/1000u);if(!limit)limit=1;
    int32_t delta=clamp_ppm(wanted)-a->clock_ppm;
    if(delta>limit)delta=limit;else if(delta< -limit)delta= -limit;
    a->clock_ppm+=delta;
    a->step_adjust=(int32_t)((int64_t)a->clock_ppm*4294967296ll/1000000ll);
}
void dbt_audio_clock_update(dbt_audio_t *a,uint32_t now){
    if(!a || !a->opened || !a->config.adaptive_clock)return;
    if(!a->clock_primed){dbt_audio_clock_reset(a,now);return;}
    uint32_t dt=now-a->clock_last_ms;if(dt<100u)return;
    uint32_t submitted=atomic_load_explicit(&a->submitted_frames,memory_order_acquire);
    uint32_t rendered=atomic_load_explicit(&a->rendered_frames,memory_order_acquire);
    uint32_t taken=atomic_load_explicit(&a->taken_frames,memory_order_acquire);
    uint32_t zeroes=atomic_load_explicit(&a->zero_frames,memory_order_acquire);
    uint32_t busy=atomic_load_explicit(&a->producer_backpressure,memory_order_acquire);
    uint32_t source_delta=submitted-a->clock_submitted,render_delta=rendered-a->clock_rendered;
    uint32_t take_delta=taken-a->clock_taken;
    bool backpressure=busy!=a->clock_busy;
    bool underrun=zeroes!=a->clock_zero;
    bool disturbed=dt>2000u || underrun || backpressure || !source_delta || !render_delta || !take_delta;
    a->clock_last_ms=now;a->clock_submitted=submitted;a->clock_rendered=rendered;a->clock_taken=taken;a->clock_zero=zeroes;a->clock_busy=busy;
    ++a->clock_updates;
    uint32_t queue=dbt_audio_input_frames(a)+dbt_audio_output_frames(a);
    int32_t error=(int32_t)queue-(int32_t)a->clock_target;
    if(disturbed){
        ++a->clock_disturbances;a->clock_clean_ms=0;a->clock_window_ms=0;
        a->clock_window_source=a->clock_window_sink=0;a->clock_observed_valid=false;
        if(!a->clock_target){a->clock_calibration_ms=0;a->clock_calibration_sum=0;}
        a->clock_state=DBT_AUDIO_CLOCK_RECOVERING;
        /* A full producer queue with an advancing consumer can be relieved by
         * proportional correction, but is never integrated or used as a clock
         * observation. A stopped or underrunning consumer instead coasts to 0. */
        int32_t wanted=0;
        if(backpressure && !underrun && render_delta && take_delta && a->clock_target && error>0)
            wanted=(int32_t)((int64_t)error*48000ll/a->config.rate);
        clock_slew(a,wanted,dt>2000u?2000u:dt);clock_publish(a,error);return;
    }
    if(a->clock_state==DBT_AUDIO_CLOCK_RECOVERING){
        a->clock_clean_ms+=dt;clock_slew(a,0,dt);
        if(a->clock_clean_ms<2000u){clock_publish(a,error);return;}
        a->filtered_error_q8=0;a->clock_state=a->clock_target?DBT_AUDIO_CLOCK_TRACKING:DBT_AUDIO_CLOCK_CALIBRATING;
    }
    if(a->clock_state==DBT_AUDIO_CLOCK_CALIBRATING){
        a->clock_calibration_sum+=(uint64_t)queue*dt;a->clock_calibration_ms+=dt;clock_slew(a,0,dt);
        if(a->clock_calibration_ms>=4000u){
            a->clock_target=(uint32_t)(a->clock_calibration_sum/a->clock_calibration_ms);
            if(!a->clock_target)a->clock_target=1;
            a->filtered_error_q8=0;a->clock_state=DBT_AUDIO_CLOCK_TRACKING;
        }
        clock_publish(a,0);return;
    }
    a->clock_window_ms+=dt;a->clock_window_source+=source_delta;a->clock_window_sink+=take_delta;
    if(a->clock_window_ms>=8000u && a->clock_window_sink){
        int64_t difference=(int64_t)a->clock_window_source-(int64_t)a->clock_window_sink;
        int64_t observed=difference*1000000ll/(int64_t)a->clock_window_sink;
        if(observed>=-2000 && observed<=2000){a->clock_observed_ppm=(int32_t)observed;a->clock_observed_valid=true;}
        else a->clock_observed_valid=false;
        a->clock_window_ms=0;a->clock_window_source=a->clock_window_sink=0;
    }
    error=(int32_t)queue-(int32_t)a->clock_target;
    int64_t filtered=(int64_t)a->filtered_error_q8+((int64_t)error*256-a->filtered_error_q8)*dt/(4000u+dt);
    a->filtered_error_q8=(int32_t)filtered;
    int64_t increment=filtered*(int64_t)dt*65536ll/(256ll*a->config.rate);
    int32_t proportional=(int32_t)(filtered*48000ll/(256ll*a->config.rate));
    int32_t current=a->clock_integral_q16/65536;
    if(!((proportional+current>=500 && increment>0) || (proportional+current<=-500 && increment<0))){
        int64_t next=(int64_t)a->clock_integral_q16+increment;
        if(next>500ll*65536)next=500ll*65536;else if(next< -500ll*65536)next= -500ll*65536;
        a->clock_integral_q16=(int32_t)next;
    }
    a->integral_ppm=a->clock_integral_q16/65536;
    clock_slew(a,proportional+a->integral_ppm,dt);clock_publish(a,error);
}
void dbt_audio_clock(dbt_audio_t *a){if(a)dbt_audio_clock_update(a,a->clock_last_ms+500u);}
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
