#include "dbt_audio.h"
#include "dbt_pcm_snapshot.h"
#include <string.h>
#if defined(DBT_AUDIO_BLOCK_DSP) && !defined(FIXED_DSP_BLOCK_API)
#error "Block rendering requires the exact bounded DSP block API"
#endif
#ifdef DBT_AUDIO_STAGE_PROFILE
#ifdef __XTENSA__
/* The SDK esp_cpu header pulls GNU-only asm spelling into this C11 component.
 * Its Xtensa HAL accessor reads the same per-core cycle counter with __asm__. */
#include <xtensa/core-macros.h>
#define audio_cycles() ((uint32_t)XTHAL_GET_CCOUNT())
#else
#include "esp_cpu.h"
#define audio_cycles() esp_cpu_get_cycle_count()
#endif
#endif
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

/* Each counter below has one writer while open. Reset/open require all
 * owners quiescent; readers are atomic but never modify these counters.
 * Keep publication atomic without a read/modify/write CAS retry loop. */
static inline void owned_add(_Atomic uint32_t *counter,uint32_t value){
    uint32_t previous=atomic_load_explicit(counter,memory_order_relaxed);
    atomic_store_explicit(counter,previous+value,memory_order_relaxed);
}
static bool power2(uint32_t n){return n>=2 && !(n&(n-1u));}
static void zero(void *object,unsigned n){uint8_t *p=object;for(unsigned i=0;i<n;++i)p[i]=0;}
static void ring_init(dbt_audio_ring_t *ring,uint8_t *bytes,uint32_t frames,uint32_t stride){
    ring->bytes=bytes;ring->frames=frames;ring->mask=power2(frames)?frames-1u:0;ring->stride=stride;
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
    atomic_init(&a->profile_conversion,0);atomic_init(&a->profile_dsp,0);atomic_init(&a->profile_tail,0);
    atomic_init(&a->profile_controls,0);atomic_init(&a->profile_frames,0);
    atomic_init(&a->profile_conversion_max,0);atomic_init(&a->profile_dsp_max,0);atomic_init(&a->profile_tail_max,0);
    atomic_init(&a->clock_pub_ppm,0);atomic_init(&a->clock_pub_observed_ppm,0);atomic_init(&a->clock_pub_error,0);
    atomic_init(&a->clock_pub_state,DBT_AUDIO_CLOCK_OFF);atomic_init(&a->clock_pub_target,0);atomic_init(&a->clock_pub_updates,0);
    atomic_init(&a->clock_pub_disturbances,0);atomic_init(&a->clock_pub_observed_valid,0);
    return DBT_OK;
}
void dbt_audio_close(dbt_audio_t *a){
    if(!a || a->render_pending)return;
    if(a->input.bytes)a->memory.release(a->memory.context,a->input.bytes);
    for(unsigned i=0;i<DBT_AUDIO_OUTPUT_PAGES;++i){
        if(a->output_pages[i])a->memory.release(a->memory.context,a->output_pages[i]);
        a->output_pages[i]=0;
    }
    a->input.bytes=a->output.bytes=0;a->opened=false;a->allocated_bytes=0;a->mix=0;a->mix_context=0;
    a->input_view=0;a->borrowed_input=a->input_carry_valid=false;
    a->render_pending_frames=0;a->render_failed=false;
    dbt_audio_clock_reset(a,0);
}
static dbt_result_t audio_open(dbt_audio_t *a,dbt_audio_config_t config,bool borrowed){
    if(!a || !a->memory.allocate || !power2(config.input_frames) || config.output_frames<2u ||
       config.input_frames>32768 || config.output_frames>32768 || config.target_frames>=config.output_frames ||
       config.channels<1 || config.channels>2 || config.format<DBT_PCM_S16_LE || config.format>DBT_PCM_S32_LE)return DBT_INVALID;
    if(a->render_pending)return DBT_BUSY;
    fixed_dsp_t check;
    if(!fixed_dsp_init(&check,config.rate,90,500,512))return DBT_UNSUPPORTED;
    dbt_audio_close(a);
    uint32_t stride=(uint32_t)config.format*config.channels;
    uint8_t *input=borrowed?0:a->memory.allocate(a->memory.context,config.input_frames*stride);
    if(!borrowed && !input)return DBT_NO_SPACE;
    ring_init(&a->input,input,config.input_frames+(borrowed?1u:0u),stride);
    a->borrowed_input=borrowed;
    unsigned pages=(config.output_frames+DBT_AUDIO_OUTPUT_PAGE_FRAMES-1u)/DBT_AUDIO_OUTPUT_PAGE_FRAMES;
    for(unsigned i=0;i<pages;++i){
        uint32_t page_frames=config.output_frames-i*DBT_AUDIO_OUTPUT_PAGE_FRAMES;
        if(page_frames>DBT_AUDIO_OUTPUT_PAGE_FRAMES)page_frames=DBT_AUDIO_OUTPUT_PAGE_FRAMES;
        a->output_pages[i]=a->memory.allocate(a->memory.context,page_frames*8u);
        if(!a->output_pages[i]){dbt_audio_close(a);return DBT_NO_SPACE;}
        if((uintptr_t)a->output_pages[i]%4u){dbt_audio_close(a);return DBT_INVALID;}
    }
    ring_init(&a->output,a->output_pages[0],config.output_frames,8);
    a->output_write_slot=a->output_read_slot=0;
    a->config=config;a->allocated_bytes=(borrowed?0:config.input_frames*stride)+config.output_frames*8u;
    (void)fixed_dsp_init(&a->dsp,config.rate,90,500,512);
    fixed_dsp_controls(&a->dsp,0,0,0,0,0,true);
    a->fraction=0;a->step_adjust=a->filtered_error_q8=a->integral_ppm=0;
    dbt_resampler_init(&a->resampler,config.rate);a->draining_input=false;a->drain_remaining=0;
    a->applied_generation=UINT32_MAX;a->opened=true;a->dsp_started=false;
    atomic_store(&a->submitted_frames,0);atomic_store(&a->rendered_frames,0);
    atomic_store(&a->taken_frames,0);atomic_store(&a->zero_frames,0);atomic_store(&a->producer_backpressure,0);
    atomic_store(&a->profile_conversion,0);atomic_store(&a->profile_dsp,0);atomic_store(&a->profile_tail,0);
    atomic_store(&a->profile_controls,0);atomic_store(&a->profile_frames,0);
    atomic_store(&a->profile_conversion_max,0);atomic_store(&a->profile_dsp_max,0);atomic_store(&a->profile_tail_max,0);
    dbt_audio_clock_reset(a,0);a->clock_primed=false;
    return DBT_OK;
}
/* Page indices are physical cursors, independent of wrapping publication
 * counters. Changing capacity is lossless only when the occupied span is a
 * single physical interval. The caller quiesces both owners for this check. */
static dbt_result_t output_linear(const dbt_audio_t *a,uint32_t *queued){
    uint32_t n=atomic_load_explicit(&a->output.written,memory_order_acquire)-
        atomic_load_explicit(&a->output.read,memory_order_acquire);
    *queued=n;
    uint32_t r=a->output_read_slot,w=a->output_write_slot,frames=a->output.frames;
    if(n>frames || r>=frames || w>=frames)return DBT_INVALID;
    uint32_t distance=w>=r?w-r:frames-r+w;
    if(n!=distance && !(n==frames && r==w))return DBT_INVALID;
    return w>=r && n==w-r?DBT_OK:DBT_BUSY;
}
dbt_result_t dbt_audio_output_extend(dbt_audio_t *a,uint8_t *const *pages,uint32_t count){
    if(!a || !a->opened || !pages || !count || count>DBT_AUDIO_OUTPUT_PAGES ||
       a->output.frames!=a->config.output_frames)return DBT_INVALID;
    if(a->render_pending)return DBT_BUSY;
    if(a->output.frames%DBT_AUDIO_OUTPUT_PAGE_FRAMES)return DBT_UNSUPPORTED;
    uint32_t old=a->output.frames/DBT_AUDIO_OUTPUT_PAGE_FRAMES;
    if(old>DBT_AUDIO_OUTPUT_PAGES || count>DBT_AUDIO_OUTPUT_PAGES-old)return DBT_NO_SPACE;
    /* All validation precedes ownership or metadata changes. New pages must
     * be distinct complete allocations compatible with memory.release. */
    for(uint32_t i=0;i<count;++i){
        if(!pages[i] || ((uintptr_t)pages[i]&3u) || a->output_pages[old+i])return DBT_INVALID;
        for(uint32_t j=0;j<old;++j)if(pages[i]==a->output_pages[j])return DBT_INVALID;
        for(uint32_t j=0;j<i;++j)if(pages[i]==pages[j])return DBT_INVALID;
    }
    uint32_t queued;dbt_result_t shape=output_linear(a,&queued);if(shape!=DBT_OK)return shape;
    for(uint32_t i=0;i<count;++i)a->output_pages[old+i]=pages[i];
    uint32_t frames=(old+count)*DBT_AUDIO_OUTPUT_PAGE_FRAMES;
    a->output.frames=a->config.output_frames=frames;a->output.mask=power2(frames)?frames-1u:0;
    a->allocated_bytes+=count*DBT_AUDIO_OUTPUT_PAGE_FRAMES*8u;return DBT_OK;
}
dbt_result_t dbt_audio_output_trim(dbt_audio_t *a,uint32_t frames,uint8_t **pages,uint32_t capacity,uint32_t *count){
    if(count)*count=0;
    if(!a || !a->opened || !count || frames<2u || frames>a->output.frames ||
       frames<=a->config.target_frames || a->output.frames!=a->config.output_frames)return DBT_INVALID;
    if(a->render_pending)return DBT_BUSY;
    if(frames%DBT_AUDIO_OUTPUT_PAGE_FRAMES || a->output.frames%DBT_AUDIO_OUTPUT_PAGE_FRAMES)return DBT_UNSUPPORTED;
    uint32_t old=a->output.frames/DBT_AUDIO_OUTPUT_PAGE_FRAMES,n=(a->output.frames-frames)/DBT_AUDIO_OUTPUT_PAGE_FRAMES;
    if(n && (!pages || capacity<n))return DBT_NO_SPACE;
    if(!n)return DBT_OK;
    uint32_t queued;dbt_result_t shape=output_linear(a,&queued);if(shape==DBT_INVALID)return shape;
    /* An empty ring can restart at slot zero even above the retained pages.
     * Nonempty data must lie wholly within the retained physical prefix. */
    if(queued && (shape!=DBT_OK || a->output_write_slot>frames))return DBT_BUSY;
    if(!queued)a->output_read_slot=a->output_write_slot=0;
    else if(a->output_write_slot==frames)a->output_write_slot=0;
    for(uint32_t i=0;i<n;++i){pages[i]=a->output_pages[old-n+i];a->output_pages[old-n+i]=0;}
    a->output.frames=a->config.output_frames=frames;a->output.mask=power2(frames)?frames-1u:0;
    a->allocated_bytes-=n*DBT_AUDIO_OUTPUT_PAGE_FRAMES*8u;*count=n;return DBT_OK;
}
dbt_result_t dbt_audio_open(dbt_audio_t *a,dbt_audio_config_t config){return audio_open(a,config,false);}
dbt_result_t dbt_audio_open_borrowed(dbt_audio_t *a,dbt_audio_config_t config){return audio_open(a,config,true);}
uint32_t dbt_audio_input_frames(const dbt_audio_t *a){
    if(!a || !a->opened)return 0;
    return atomic_load_explicit(&a->input.written,memory_order_acquire)-atomic_load_explicit(&a->input.read,memory_order_acquire);
}
uint32_t AUDIO_ISR dbt_audio_output_frames(const dbt_audio_t *a){
    if(!a || !a->opened)return 0;
    return atomic_load_explicit(&a->output.written,memory_order_acquire)-atomic_load_explicit(&a->output.read,memory_order_acquire);
}
dbt_result_t dbt_audio_input_borrow(dbt_audio_t *a,const void *pcm,uint32_t frames){
    if(!a || !a->opened || !a->borrowed_input || !pcm || !frames || frames>a->config.input_frames || a->draining_input)return DBT_INVALID;
    if(a->input_view)return DBT_BUSY;
    uint32_t written=atomic_load_explicit(&a->input.written,memory_order_relaxed);
    uint32_t remaining=written-atomic_load_explicit(&a->input.read,memory_order_relaxed);
    if(remaining>1u || (remaining && !a->input_carry_valid))return DBT_BUSY;
    a->input_view=pcm;a->input_view_begin=written;
    atomic_store_explicit(&a->input.written,written+frames,memory_order_release);
    owned_add(&a->submitted_frames,frames);return DBT_OK;
}
static inline const uint8_t *input_frame(const dbt_audio_t *a,uint32_t index){
    if(!a->borrowed_input)return a->input.bytes+(index&a->input.mask)*a->input.stride;
    if(a->input_carry_valid && index==a->input_carry_index)return a->input_carry;
    return a->input_view+(index-a->input_view_begin)*a->input.stride;
}
bool dbt_audio_input_release(dbt_audio_t *a){
    if(!a || !a->opened || !a->borrowed_input)return false;
    if(!a->input_view)return true;
    uint32_t read=atomic_load_explicit(&a->input.read,memory_order_relaxed);
    uint32_t remaining=atomic_load_explicit(&a->input.written,memory_order_relaxed)-read;
    if(remaining>1u)return false;
    if(remaining){
        const uint8_t *source=input_frame(a,read);
        for(unsigned i=0;i<a->input.stride;++i)a->input_carry[i]=source[i];
        a->input_carry_index=read;a->input_carry_valid=true;
    }else a->input_carry_valid=false;
    a->input_view=0;return true;
}
uint32_t dbt_audio_input_cancel(dbt_audio_t *a){
    if(!a || !a->opened || !a->borrowed_input)return 0;
    uint32_t written=atomic_load_explicit(&a->input.written,memory_order_relaxed);
    uint32_t remaining=written-atomic_load_explicit(&a->input.read,memory_order_relaxed);
    a->input_view=0;a->input_carry_valid=false;
    atomic_store_explicit(&a->input.read,written,memory_order_release);return remaining;
}
dbt_result_t dbt_audio_submit(dbt_audio_t *a,const void *pcm,uint32_t frames){
    if(!a || !a->opened || a->borrowed_input || (!pcm && frames) || frames>a->input.frames)return DBT_INVALID;
    uint32_t written=atomic_load_explicit(&a->input.written,memory_order_relaxed);
    uint32_t read=atomic_load_explicit(&a->input.read,memory_order_acquire);
    if(frames>a->input.frames-(written-read)){
        owned_add(&a->producer_backpressure,1);return DBT_BUSY;
    }
    if(frames){
        /* A whole submission spans at most one power-of-two ring wrap. The
         * immutable decoder view is separate from this owned ring. */
        const uint8_t *source=pcm;uint32_t slot=written&a->input.mask;
        uint32_t first=a->input.frames-slot;if(first>frames)first=frames;
        uint32_t bytes=first*a->input.stride;
        memcpy(a->input.bytes+slot*a->input.stride,source,bytes);
        if(first<frames)memcpy(a->input.bytes,source+bytes,(frames-first)*a->input.stride);
#ifdef DBT_APTX_SNAPSHOT
        dbt_pcm_snapshot_handoff(pcm,frames,a->input.stride,a->input.bytes+slot*a->input.stride,bytes,a->input.bytes,(frames-first)*a->input.stride);
#endif
    }
    atomic_store_explicit(&a->input.written,written+frames,memory_order_release);
    owned_add(&a->submitted_frames,frames);return DBT_OK;
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
        /* A fresh stream has no audible history to crossfade. Restore its
         * first policy before DSP begins; live changes keep the normal ramps.
         * The persistent flag also remains correct across counter wrap. */
        fixed_dsp_controls(&a->dsp,(int8_t)gains,(int8_t)(gains>>8),(int8_t)(gains>>16),(uint8_t)options,(uint8_t)(options>>8),!a->dsp_started);
        a->applied_generation=generation;return;
    }
}
static int32_t sample(const dbt_audio_t *a,uint32_t index,unsigned channel){
    const uint8_t *p=input_frame(a,index)+channel*(unsigned)a->config.format;
#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    if(a->config.format==DBT_PCM_S32_LE && !((uintptr_t)p&3u)){
        int32_t value;__builtin_memcpy(&value,__builtin_assume_aligned(p,4),sizeof(value));return value;
    }
#endif
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
void dbt_audio_set_mix(dbt_audio_t *a,void (*mix)(void *,int32_t *,uint32_t),void *context){if(a && !a->render_pending){a->mix=mix;a->mix_context=context;}}
static inline __attribute__((always_inline)) int32_t *output_frame(const dbt_audio_t *a,uint32_t slot){
    return (int32_t *)a->output_pages[slot/DBT_AUDIO_OUTPUT_PAGE_FRAMES]+2u*(slot&(DBT_AUDIO_OUTPUT_PAGE_FRAMES-1u));
}
/* Count is <= capacity. No division, shared cursor or cross-owner access. */
static inline __attribute__((always_inline)) uint32_t output_advance(const dbt_audio_t *a,uint32_t slot,uint32_t count){
    slot+=count;return slot>=a->output.frames?slot-a->output.frames:slot;
}
/* Bound a contiguous page/ring span before copying. No shared cursor or
 * counter is advanced until the complete original block is ready. */
static inline __attribute__((always_inline)) uint32_t output_span(const dbt_audio_t *a,uint32_t slot,uint32_t count){
    uint32_t span=DBT_AUDIO_OUTPUT_PAGE_FRAMES-(slot&(DBT_AUDIO_OUTPUT_PAGE_FRAMES-1u));
    uint32_t ring_left=a->output.frames-slot;if(span>ring_left)span=ring_left;
    return span<count?span:count;
}
static void apply_mix(dbt_audio_t *a,uint32_t out,uint32_t count){
    if(!a->mix || !count)return;
    /* At most 65 spans for the maximum legal ring, including one wrap. Normal
     * <=128-frame units need at most three spans when a partial final page
     * places a distinct page edge immediately before the ring wrap. */
    while(count){
        uint32_t slot=out;
        uint32_t first=output_span(a,slot,count);
        a->mix(a->mix_context,output_frame(a,out),first);out=output_advance(a,out,first);count-=first;
    }
}
#ifdef DBT_AUDIO_BLOCK_DSP
static uint32_t render_dsp_block(dbt_audio_t *a,int32_t *pcm,unsigned frames,uint32_t slot
#ifdef DBT_AUDIO_STAGE_PROFILE
                                ,uint32_t *dsp_cycles,uint32_t *tail_cycles
#endif
){
#ifdef DBT_AUDIO_STAGE_PROFILE
    uint32_t began=audio_cycles();
#endif
    /* pcm is a naturally aligned, unpublished output-ring span. The caller
     * bounds it by one page/ring edge and FIXED_DSP_BLOCK_FRAMES. DSP mutates
     * only renderer-owned free slots; the whole render call publishes later. */
#ifdef DBT_APTX_SNAPSHOT
    bool capture=dbt_pcm_snapshot_dsp_begin(&a->dsp,pcm,frames,a->mix!=NULL);
#endif
    (void)fixed_dsp_process_block(&a->dsp,pcm,(uint16_t)frames);
#ifdef DBT_APTX_SNAPSHOT
    if(capture)dbt_pcm_snapshot_dsp_end(pcm);
#endif
#ifdef DBT_AUDIO_STAGE_PROFILE
    uint32_t at=audio_cycles();*dsp_cycles+=at-began;
#endif
    slot=output_advance(a,slot,frames);
#ifdef DBT_AUDIO_STAGE_PROFILE
    *tail_cycles+=audio_cycles()-at;
#endif
    return slot;
}
#endif
static uint32_t render_publish(dbt_audio_t *a,uint32_t in,uint32_t out,uint32_t slot,uint32_t frames){
    if(frames)a->dsp_started=true;
    a->output_write_slot=slot;
    if(a->borrowed_input && a->input_carry_valid && in!=a->input_carry_index)a->input_carry_valid=false;
    atomic_store_explicit(&a->input.read,in,memory_order_release);
    atomic_store_explicit(&a->output.written,out,memory_order_release);
    owned_add(&a->rendered_frames,frames);return frames;
}
bool dbt_audio_render_inflight(const dbt_audio_t *a){return a && a->render_pending!=0;}
dbt_result_t dbt_audio_render_prepare(dbt_audio_t *a,uint32_t budget,dbt_audio_render_job_t *job){
    if(!a || !job || !a->opened || a->render_failed)return DBT_INVALID;
    if(a->render_pending)return DBT_BUSY;
#if defined(DBT_AUDIO_UNITY_COPY) && defined(DBT_AUDIO_BLOCK_DSP) && defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    if(a->borrowed_input || a->config.adaptive_clock || a->fraction || a->step_adjust || a->drain_remaining ||
       a->config.format!=DBT_PCM_S32_LE || a->config.channels!=2 || a->input.stride!=8)return DBT_UNSUPPORTED;
#ifdef DBT_AUDIO_STAGE_PROFILE
    uint32_t began=audio_cycles();
#endif
    controls(a);
    uint32_t controls_cycles=0,conversion_cycles=0;
#ifdef DBT_AUDIO_STAGE_PROFILE
    uint32_t mark=audio_cycles();controls_cycles=mark-began;
#endif
    uint32_t in=atomic_load_explicit(&a->input.read,memory_order_relaxed);
    uint32_t end=atomic_load_explicit(&a->input.written,memory_order_acquire);
    uint32_t out=atomic_load_explicit(&a->output.written,memory_order_relaxed);
    uint32_t free=a->output.frames-(out-atomic_load_explicit(&a->output.read,memory_order_acquire));
    uint32_t count=output_span(a,a->output_write_slot,FIXED_DSP_BLOCK_FRAMES);
    if(count>budget)count=budget;
    if(count>free)count=free;
    if(count>end-in)count=end-in;
    if(!count)return DBT_EMPTY;
    int32_t *pcm=output_frame(a,a->output_write_slot);
    uint32_t source=in&a->input.mask,first=a->input.frames-source;if(first>count)first=count;
    memcpy(pcm,a->input.bytes+source*8u,first*8u);
    if(first<count)memcpy(pcm+2u*first,a->input.bytes,(count-first)*8u);
#ifdef DBT_AUDIO_STAGE_PROFILE
    conversion_cycles=audio_cycles()-mark;
#endif
    uint32_t token=a->render_serial+1u;if(!token)token=1;
    a->render_serial=token;a->render_pending=token;a->render_pending_frames=count;
    *job=(dbt_audio_render_job_t){&a->dsp,pcm,token,count,in+count,out+count,
        output_advance(a,a->output_write_slot,count),conversion_cycles,controls_cycles};
    return DBT_OK;
#else
    (void)budget;return DBT_UNSUPPORTED;
#endif
}
dbt_result_t dbt_audio_render_finish(dbt_audio_t *a,const dbt_audio_render_job_t *job,bool success,uint32_t *frames){
    if(frames)*frames=0;
    if(!a || !job || !a->opened || !a->render_pending || job->token!=a->render_pending)return DBT_INVALID;
    uint32_t in=atomic_load_explicit(&a->input.read,memory_order_relaxed);
    uint32_t out=atomic_load_explicit(&a->output.written,memory_order_relaxed);
    bool valid=job->frames && job->frames==a->render_pending_frames && job->frames<=FIXED_DSP_BLOCK_FRAMES &&
        job->dsp==&a->dsp && job->pcm==output_frame(a,a->output_write_slot) &&
        job->input_after-in==job->frames && job->output_after-out==job->frames &&
        job->output_slot==output_advance(a,a->output_write_slot,job->frames);
    if(!success || !valid){a->render_pending=a->render_pending_frames=0;a->render_failed=true;return DBT_INVALID;}
#ifdef DBT_AUDIO_STAGE_PROFILE
    uint32_t finish_began=audio_cycles();
#endif
    apply_mix(a,a->output_write_slot,job->frames);
#ifdef DBT_APTX_SNAPSHOT
    dbt_pcm_snapshot_final(&a->dsp);
#endif
#ifdef DBT_AUDIO_STAGE_PROFILE
    owned_add(&a->profile_controls,job->controls_cycles);owned_add(&a->profile_frames,job->frames);
    owned_add(&a->profile_conversion,job->conversion_cycles);
    if(job->conversion_cycles>atomic_load(&a->profile_conversion_max))atomic_store(&a->profile_conversion_max,job->conversion_cycles);
    /* Assisted math runs on another core and is logged by its worker. Keep
     * the existing DSP-cycle counter local-only instead of mixing domains. */
#endif
    uint32_t made=render_publish(a,job->input_after,job->output_after,job->output_slot,job->frames);
    a->render_pending=a->render_pending_frames=0;
#ifdef DBT_AUDIO_STAGE_PROFILE
    uint32_t finish_cycles=audio_cycles()-finish_began;owned_add(&a->profile_tail,finish_cycles);
    if(finish_cycles>atomic_load(&a->profile_tail_max))atomic_store(&a->profile_tail_max,finish_cycles);
#endif
    if(frames)*frames=made;
    return DBT_OK;
}

/* Inline the implementation into both public entries. The ordinary entry
 * passes a constant NULL hook and retains its original publication timing. */
static inline __attribute__((always_inline)) uint32_t audio_render(dbt_audio_t *a,uint32_t budget,
    void (*input_ready)(void *),void *wake_context){
    if(!a || !a->opened || a->render_pending || a->render_failed)return 0;
#ifdef DBT_AUDIO_BLOCK_DSP
    int32_t *pcm_block;unsigned block_frames=0,block_capacity;
#endif
#ifdef DBT_AUDIO_STAGE_PROFILE
    uint32_t stage=audio_cycles(),conversion=0,dsp=0,tail=0;
#endif
    controls(a);
#ifdef DBT_AUDIO_STAGE_PROFILE
    uint32_t control_cycles=audio_cycles()-stage;
#endif
    uint32_t in=atomic_load_explicit(&a->input.read,memory_order_relaxed);
    uint32_t end=atomic_load_explicit(&a->input.written,memory_order_acquire);
    uint32_t out=atomic_load_explicit(&a->output.written,memory_order_relaxed);
    uint32_t free=a->output.frames-(out-atomic_load_explicit(&a->output.read,memory_order_acquire));
    uint32_t produced=0,initial_in=in;bool input_notified=false;
    uint32_t slot=a->output_write_slot;
#ifdef DBT_AUDIO_BLOCK_DSP
    pcm_block=output_frame(a,slot);block_capacity=output_span(a,slot,FIXED_DSP_BLOCK_FRAMES);
#endif
#if defined(DBT_AUDIO_UNITY_COPY) && defined(DBT_AUDIO_BLOCK_DSP) && defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    /* The product's copied S32 stereo unity clock needs no per-sample format,
     * fraction or resampler work. Copy only the captured input publication into
     * one unpublished DSP/output span; keep identical block boundaries and the
     * existing final input/output release publications and mix ordering. */
    const bool copy_unity=!a->borrowed_input && !a->config.adaptive_clock &&
        !a->fraction && !a->step_adjust && !a->drain_remaining &&
        a->config.format==DBT_PCM_S32_LE && a->config.channels==2 && a->input.stride==8;
#endif
    while(produced<budget && produced<free){
#ifdef DBT_AUDIO_STAGE_PROFILE
        stage=audio_cycles();
#endif
        uint32_t available=end-in;
#if defined(DBT_AUDIO_UNITY_COPY) && defined(DBT_AUDIO_BLOCK_DSP) && defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
        if(copy_unity){
            uint32_t count=block_capacity;
            if(count>budget-produced)count=budget-produced;
            if(count>free-produced)count=free-produced;
            if(count>available)count=available;
            if(!count)break;
            uint32_t source_slot=in&a->input.mask,first=a->input.frames-source_slot;
            if(first>count)first=count;
            memcpy(pcm_block,a->input.bytes+source_slot*8u,first*8u);
            if(first<count)memcpy(pcm_block+2u*first,a->input.bytes,(count-first)*8u);
            if(input_ready){
                /* The complete source span is now copied into unpublished
                 * output storage. The SPSC producer may reuse it during DSP;
                 * DMA still cannot see this output until final publication. */
                atomic_store_explicit(&a->input.read,in+count,memory_order_release);
                input_ready(wake_context);input_notified=true;
            }
#ifdef DBT_AUDIO_STAGE_PROFILE
            conversion+=audio_cycles()-stage;
#endif
            slot=render_dsp_block(a,pcm_block,count,slot
#ifdef DBT_AUDIO_STAGE_PROFILE
                                  ,&dsp,&tail
#endif
            );
#ifdef DBT_AUDIO_STAGE_PROFILE
            stage=audio_cycles();
#endif
            in+=count;produced+=count;
            pcm_block=output_frame(a,slot);block_capacity=output_span(a,slot,FIXED_DSP_BLOCK_FRAMES);
#ifdef DBT_AUDIO_STAGE_PROFILE
            tail+=audio_cycles()-stage;
#endif
            continue;
        }
#endif
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
                if(in!=end){
                    if(!i){history_left=left;history_right=right;}
                    else {history_left=sample(a,in,0);history_right=sample(a,in,a->config.channels==1?0:1);}
                    ++in;
                }
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
#ifdef DBT_AUDIO_STAGE_PROFILE
        uint32_t at=audio_cycles();conversion+=at-stage;stage=at;
#endif
#ifdef DBT_AUDIO_BLOCK_DSP
        pcm_block[2u*block_frames]=left;pcm_block[2u*block_frames+1u]=right;++block_frames;
#else
        fixed_dsp_process(&a->dsp,&left,&right);
#ifdef DBT_AUDIO_STAGE_PROFILE
        at=audio_cycles();dsp+=at-stage;stage=at;
#endif
        int32_t *destination=output_frame(a,slot);destination[0]=left;destination[1]=right;
        slot=output_advance(a,slot,1);
#endif
        a->fraction=(uint32_t)next;++produced;
#ifdef DBT_AUDIO_STAGE_PROFILE
        tail+=audio_cycles()-stage;
#endif
#ifdef DBT_AUDIO_BLOCK_DSP
        if(block_frames==block_capacity){
            slot=render_dsp_block(a,pcm_block,block_frames,slot
#ifdef DBT_AUDIO_STAGE_PROFILE
                                  ,&dsp,&tail
#endif
            );block_frames=0;
            pcm_block=output_frame(a,slot);block_capacity=output_span(a,slot,FIXED_DSP_BLOCK_FRAMES);
        }
#endif
    }
#ifdef DBT_AUDIO_BLOCK_DSP
    if(block_frames)slot=render_dsp_block(a,pcm_block,block_frames,slot
#ifdef DBT_AUDIO_STAGE_PROFILE
                                        ,&dsp,&tail
#endif
    );
#endif
    apply_mix(a,a->output_write_slot,produced);
#ifdef DBT_APTX_SNAPSHOT
    dbt_pcm_snapshot_final(&a->dsp);
#endif
    a->output_write_slot=slot;
#ifdef DBT_AUDIO_STAGE_PROFILE
    atomic_fetch_add(&a->profile_controls,control_cycles);atomic_fetch_add(&a->profile_frames,produced);
#define PROFILE_PUBLISH(field,value) do {atomic_fetch_add(&a->profile_##field,value);if(value>atomic_load(&a->profile_##field##_max))atomic_store(&a->profile_##field##_max,value);}while(0)
    PROFILE_PUBLISH(conversion,conversion);PROFILE_PUBLISH(dsp,dsp);PROFILE_PUBLISH(tail,tail);
#undef PROFILE_PUBLISH
#endif
    uint32_t made=render_publish(a,in,out+produced,slot,produced);
    if(input_ready && !input_notified && in!=initial_in)input_ready(wake_context);
    return made;
}
uint32_t dbt_audio_render(dbt_audio_t *a,uint32_t budget){return audio_render(a,budget,NULL,NULL);}
uint32_t dbt_audio_render_with_input_wake(dbt_audio_t *a,uint32_t budget,
    void (*input_ready)(void *),void *context){return audio_render(a,budget,input_ready,context);}
uint32_t dbt_audio_render_silence(dbt_audio_t *a,uint32_t budget){
    if(!a || !a->opened || !a->mix || a->render_pending || a->render_failed)return 0;
    controls(a);uint32_t out=atomic_load_explicit(&a->output.written,memory_order_relaxed);
    uint32_t free=a->output.frames-(out-atomic_load_explicit(&a->output.read,memory_order_acquire));
    uint32_t count=budget<free?budget:free;
    uint32_t slot=a->output_write_slot;
    for(uint32_t i=0;i<count;++i){int32_t *destination=output_frame(a,slot);destination[0]=destination[1]=0;slot=output_advance(a,slot,1);}
    apply_mix(a,a->output_write_slot,count);
    if(count)a->dsp_started=true;
    a->output_write_slot=slot;
    atomic_store_explicit(&a->output.written,out+count,memory_order_release);
    owned_add(&a->rendered_frames,count);return count;
}
void dbt_audio_begin_drain(dbt_audio_t *a){
    if(!a || !a->opened || a->draining_input || a->render_pending)return;
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
    uint32_t slot=a->output_read_slot;
    uint32_t copied=0;
    while(copied<count){
        uint32_t span=output_span(a,slot,count-copied);
        memcpy(destination+2u*copied,output_frame(a,slot),span*8u);
        slot=output_advance(a,slot,span);copied+=span;
    }
    a->output_read_slot=slot;
    if(count<frames)memset(destination+2u*count,0,(frames-count)*8u);
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
