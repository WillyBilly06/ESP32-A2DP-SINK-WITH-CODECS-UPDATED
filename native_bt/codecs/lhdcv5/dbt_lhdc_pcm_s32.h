#ifndef DBT_LHDC_PCM_S32_H
#define DBT_LHDC_PCM_S32_H
#include <stdint.h>
#include <limits.h>
#include <math.h>
#include <float.h>
_Static_assert(sizeof(float)==sizeof(uint32_t) && FLT_RADIX==2 && FLT_MANT_DIG==24 && FLT_MAX_EXP==128,
    "LHDC PCM specialization requires binary32");
/* float(INT32_MAX) rounds to2^31. Every cast below has a representable
 * operand. Unordered input becomes0, matching the original output macro. */
static inline __attribute__((always_inline)) int32_t dbt_lhdc_s32_sample(float value){
    if(value < -2147483648.0f)return INT32_MIN;
    if(!(value < 2147483648.0f))return value==value?INT32_MAX:0;
    return (int32_t)value;
}
typedef struct {float peak;int clipped;} dbt_lhdc_s32_quality_t;
#if defined(__GNUC__) && !defined(__clang__)
/* Threading the rare rail/NaN paths duplicates this small IRAM loop. Keep the
 * regular O3 arithmetic/loop optimizations without that code expansion. */
#define DBT_LHDC_PCM_COMPACT __attribute__((optimize("no-thread-jumps")))
#else
#define DBT_LHDC_PCM_COMPACT
#endif
/* Output is private, disjoint transform scratch until the whole frame's quality
 * decision. Concealment still overwrites every sample and clears overlap in the
 * caller. For the admitted positive finite scale, taking the magnitude after
 * multiplication preserves its quality comparison under the decoder's normal
 * floating-point environment. Other scales retain the original caller path. */
static LHDC_HOT __attribute__((noinline)) DBT_LHDC_PCM_COMPACT dbt_lhdc_s32_quality_t dbt_lhdc_scan_emit_s32(
    const float *left,const float *right,int32_t *out,int frames,float scale){
    dbt_lhdc_s32_quality_t quality={0.0f,0};
    for(int i=0;i<frames;++i){
        float l=left[i]*scale,r=right[i]*scale;
        float al=fabsf(l),ar=fabsf(r);
        if(!(al<=quality.peak))quality.peak=al>=0.0f?al:INFINITY;
        if(!(ar<=quality.peak))quality.peak=ar>=0.0f?ar:INFINITY;
        if(al>2147483647.0f)++quality.clipped;
        if(ar>2147483647.0f)++quality.clipped;
        out[2*i]=dbt_lhdc_s32_sample(l);
        out[2*i+1]=dbt_lhdc_s32_sample(r);
    }
    return quality;
}
#undef DBT_LHDC_PCM_COMPACT
#endif
