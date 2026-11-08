#ifndef DBT_LHDC_PCM_S32_H
#define DBT_LHDC_PCM_S32_H
#include <stdint.h>
#include <limits.h>
#include <math.h>
#include <float.h>
#if defined(__XTENSA__) && defined(CONFIG_IDF_TARGET_ESP32) && CONFIG_IDF_TARGET_ESP32
#include "xtensa/config/core-isa.h"
#if !XCHAL_HAVE_FP
#error ESP32 PCM register transfer requires the configured FP option
#endif
#endif
_Static_assert(sizeof(float)==sizeof(uint32_t) && FLT_RADIX==2 && FLT_MANT_DIG==24 && FLT_MAX_EXP==128,
    "LHDC PCM specialization requires binary32");
typedef union {float value;uint32_t bits;} dbt_lhdc_pcm_word_t;
static inline __attribute__((always_inline)) uint32_t dbt_lhdc_pcm_bits(float value){
#if defined(__XTENSA__) && defined(CONFIG_IDF_TARGET_ESP32) && CONFIG_IDF_TARGET_ESP32
    /* Documented FR-to-AR bit move, also used by the pinned GCC movsf pattern:
     * espressif/gcc esp-15.2.0_20251204, gcc/config/xtensa/xtensa.md. No memory. */
    uint32_t bits;__asm__("rfr %0, %1" : "=a"(bits) : "f"(value));return bits;
#else
    dbt_lhdc_pcm_word_t word={.value=value};return word.bits;
#endif
}
static inline __attribute__((always_inline)) int32_t dbt_lhdc_s32_sample(float value){
    uint32_t bits=dbt_lhdc_pcm_bits(value),magnitude=bits&UINT32_C(0x7fffffff);
    if(magnitude>=UINT32_C(0x4f000000)){
        if(magnitude>UINT32_C(0x7f800000))return 0;
        return bits>>31?INT32_MIN:INT32_MAX;
    }
    return (int32_t)value;
}
typedef struct {float peak;int clipped;} dbt_lhdc_s32_quality_t;
#if defined(__GNUC__) && !defined(__clang__)
#define DBT_LHDC_PCM_COMPACT __attribute__((optimize("no-thread-jumps")))
#else
#define DBT_LHDC_PCM_COMPACT
#endif
static inline __attribute__((always_inline)) int32_t dbt_lhdc_s32_accumulate(float value,uint32_t *peak,int *clipped){
    uint32_t bits=dbt_lhdc_pcm_bits(value),magnitude=bits&UINT32_C(0x7fffffff);
    if(magnitude>UINT32_C(0x7f800000)){*peak=UINT32_C(0x7f800000);return 0;}
    *clipped+=magnitude>UINT32_C(0x4f000000);
    if(magnitude>*peak)*peak=magnitude;
    if(magnitude>=UINT32_C(0x4f000000))return bits>>31?INT32_MIN:INT32_MAX;
    return (int32_t)value;
}
/* Inputs/output are disjoint private transform scratch until frame acceptance.
 * A NaN selects sticky +Inf without a clip count. Inf/finite rails preserve the
 * original rounded 2^31 comparisons; representable samples use hardware trunc. */
static LHDC_HOT __attribute__((noinline)) DBT_LHDC_PCM_COMPACT dbt_lhdc_s32_quality_t dbt_lhdc_scan_emit_s32(
    const float *left,const float *right,int32_t *out,int frames,float scale){
    uint32_t peak=0;int clipped=0;
    for(int i=0;i<frames;++i){
        out[2*i]=dbt_lhdc_s32_accumulate(left[i]*scale,&peak,&clipped);
        out[2*i+1]=dbt_lhdc_s32_accumulate(right[i]*scale,&peak,&clipped);
    }
    dbt_lhdc_pcm_word_t maximum={.bits=peak};return (dbt_lhdc_s32_quality_t){maximum.value,clipped};
}
#undef DBT_LHDC_PCM_COMPACT
#endif
