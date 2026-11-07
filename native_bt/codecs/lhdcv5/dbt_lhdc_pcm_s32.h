#ifndef DBT_LHDC_PCM_S32_H
#define DBT_LHDC_PCM_S32_H
#include <stdint.h>
#include <limits.h>
/* float(INT32_MAX) rounds to2^31. Every cast below has a representable
 * operand. Unordered input becomes0, matching the original output macro. */
static inline __attribute__((always_inline)) int32_t dbt_lhdc_s32_sample(float value){
    if(value < -2147483648.0f)return INT32_MIN;
    if(!(value < 2147483648.0f))return value==value?INT32_MAX:0;
    return (int32_t)value;
}
static LHDC_HOT __attribute__((noinline)) void dbt_lhdc_emit_s32(const float *left,const float *right,int32_t *out,int frames,float scale){
    for(int i=0;i<frames;++i){
        float l=left[i]*scale,r=right[i]*scale;
        out[2*i]=dbt_lhdc_s32_sample(l);out[2*i+1]=dbt_lhdc_s32_sample(r);
    }
}
#endif
