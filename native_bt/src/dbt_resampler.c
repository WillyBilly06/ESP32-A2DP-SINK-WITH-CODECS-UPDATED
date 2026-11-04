#include "dbt_resampler.h"
#include <limits.h>

#include "dbt_resampler_coeffs.inc"

static uint8_t tap_count(uint32_t rate){return rate<=48000u?96u:rate<=96000u?48u:24u;}
uint8_t dbt_resampler_delay_frames(uint32_t rate){return (uint8_t)(tap_count(rate)/2u-1u);}
const int32_t *dbt_resampler_phase(uint32_t rate,uint8_t phase,uint8_t *taps){
    uint8_t n=tap_count(rate);if(taps)*taps=n;
    if(phase>128u)return 0;
    return n==96u?coefficients_96[phase]:n==48u?coefficients_48[phase]:coefficients_24[phase];
}
void dbt_resampler_init(dbt_resampler_t *s,uint32_t rate){
    if(!s)return;
    s->next=s->count=0;s->taps=tap_count(rate);s->primed=false;s->first[0]=s->first[1]=0;
}
void dbt_resampler_push(dbt_resampler_t *s,int32_t left,int32_t right){
    if(!s)return;
    if(!s->primed){s->first[0]=left;s->first[1]=right;s->primed=true;}
    s->history[0][s->next]=left;s->history[1][s->next]=right;
    s->next=(uint8_t)((s->next+1u)&(DBT_RESAMPLER_HISTORY-1u));
    if(s->count<DBT_RESAMPLER_HISTORY)++s->count;
}
static int32_t saturated(int64_t value){
    value>>=30;
    if(value>INT32_MAX)return INT32_MAX;
    if(value<INT32_MIN)return INT32_MIN;
    return (int32_t)value;
}
static inline __attribute__((always_inline)) int32_t coefficient_at(int32_t first,int32_t next,uint32_t remainder){
#if defined(DBT_RESAMPLER_ADJACENT_DELTA_MAX) && DBT_RESAMPLER_ADJACENT_DELTA_MAX <= 2147483647u && !defined(DBT_RESAMPLER_FORCE_WIDE_DELTA)
    /* Generator proves every adjacent-row subtraction fits int32. Remainder
     * is <2^25; truncation keeps the interpolated value between the endpoints.
     * Keep the same signed division/rounding and complete Q30 precision. */
    int32_t delta=next-first;
    return first+(int32_t)(((int64_t)delta*(int32_t)remainder)/33554432ll);
#else
    return (int32_t)((int64_t)first+(((int64_t)next-first)*remainder)/33554432ll);
#endif
}
#if defined(DBT_RESAMPLER_ENABLE_FOLDED_ZERO) && defined(DBT_RESAMPLER_ZERO_PHASE_SYMMETRIC) && DBT_RESAMPLER_ZERO_PHASE_SYMMETRIC && !defined(DBT_RESAMPLER_FORCE_UNFOLDED_ZERO) && !defined(DBT_RESAMPLER_FORCE_WIDE_DELTA)
#define DBT_RESAMPLER_FOLDED_ZERO_ENABLED 1
static __attribute__((noinline)) void folded_zero(const dbt_resampler_t *s,const int32_t *coefficients,int32_t *left,int32_t *right){
    /* Complete history only. The generated row is symmetric around taps/2-1,
     * with current-input and final two coefficients zero. Pair sums retain
     * 33 signed bits. The original absolute-sum bound also bounds every folded
     * partial sum, so regrouping cannot overflow or alter rounding. */
    unsigned middle=s->taps/2u-1u;
    unsigned near=(s->next-1u)&(DBT_RESAMPLER_HISTORY-1u);
    unsigned far=(s->next-(s->taps-3u))&(DBT_RESAMPLER_HISTORY-1u);
    int64_t l=0,r=0;
    for(unsigned k=1;k<middle;++k){
        int32_t c=coefficients[k];
        int64_t pair_l=(int64_t)s->history[0][near]+s->history[0][far];
        int64_t pair_r=(int64_t)s->history[1][near]+s->history[1][far];
        l+=pair_l*c;r+=pair_r*c;
        near=(near-1u)&(DBT_RESAMPLER_HISTORY-1u);
        far=(far+1u)&(DBT_RESAMPLER_HISTORY-1u);
    }
    l+=(int64_t)s->history[0][near]*coefficients[middle];
    r+=(int64_t)s->history[1][near]*coefficients[middle];
    *left=saturated(l);*right=saturated(r);
}
#else
#define DBT_RESAMPLER_FOLDED_ZERO_ENABLED 0
#endif
void dbt_resampler_filter(dbt_resampler_t *s,int32_t left,int32_t right,uint32_t phase,int32_t *out_left,int32_t *out_right){
    if(!s || !out_left || !out_right)return;
    if(!s->primed){s->first[0]=left;s->first[1]=right;s->primed=true;}
    uint8_t p=(uint8_t)(phase>>25);uint32_t remainder=phase&0x1ffffffu;
    const int32_t *first=s->taps==96?coefficients_96[p]:s->taps==48?coefficients_48[p]:coefficients_24[p];
#if DBT_RESAMPLER_FOLDED_ZERO_ENABLED
    if(!phase && s->count>=s->taps && (s->taps==24u || s->taps==48u || s->taps==96u)){
        folded_zero(s,first,out_left,out_right);return;
    }
#endif
    const int32_t *next=s->taps==96?coefficients_96[p+1u]:s->taps==48?coefficients_48[p+1u]:coefficients_24[p+1u];
    int64_t l=0,r=0;
    if(s->count>=s->taps){
        int32_t coefficient=first[0];
        if(remainder)coefficient=coefficient_at(coefficient,next[0],remainder);
        l=(int64_t)left*coefficient;r=(int64_t)right*coefficient;
        unsigned index=(s->next-1u)&(DBT_RESAMPLER_HISTORY-1u);
        if(!remainder){
            for(unsigned k=1;k<s->taps;++k){
                int32_t c=first[k];
                l+=(int64_t)s->history[0][index]*c;r+=(int64_t)s->history[1][index]*c;
                index=(index-1u)&(DBT_RESAMPLER_HISTORY-1u);
            }
        }else{
            for(unsigned k=1;k<s->taps;++k){
                int32_t c=coefficient_at(first[k],next[k],remainder);
                l+=(int64_t)s->history[0][index]*c;r+=(int64_t)s->history[1][index]*c;
                index=(index-1u)&(DBT_RESAMPLER_HISTORY-1u);
            }
        }
    }else for(unsigned k=0;k<s->taps;++k){
        int32_t coefficient=first[k];
        if(remainder)coefficient=coefficient_at(coefficient,next[k],remainder);
        unsigned index=(s->next-k)&(DBT_RESAMPLER_HISTORY-1u);
        int32_t x=k==0?left:k<=s->count?s->history[0][index]:s->first[0];
        int32_t y=k==0?right:k<=s->count?s->history[1][index]:s->first[1];
        l+=(int64_t)x*coefficient;r+=(int64_t)y*coefficient;
    }
    *out_left=saturated(l);*out_right=saturated(r);
}
