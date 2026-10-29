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
void dbt_resampler_filter(dbt_resampler_t *s,int32_t left,int32_t right,uint32_t phase,int32_t *out_left,int32_t *out_right){
    if(!s || !out_left || !out_right)return;
    if(!s->primed){s->first[0]=left;s->first[1]=right;s->primed=true;}
    uint8_t p=(uint8_t)(phase>>25);uint32_t remainder=phase&0x1ffffffu;
    const int32_t *first=s->taps==96?coefficients_96[p]:s->taps==48?coefficients_48[p]:coefficients_24[p];
    const int32_t *next=s->taps==96?coefficients_96[p+1u]:s->taps==48?coefficients_48[p+1u]:coefficients_24[p+1u];
    int64_t l=0,r=0;
    for(unsigned k=0;k<s->taps;++k){
        int32_t coefficient=(int32_t)((int64_t)first[k]+(((int64_t)next[k]-first[k])*remainder)/33554432ll);
        unsigned index=(s->next-k)&(DBT_RESAMPLER_HISTORY-1u);
        int32_t x=k==0?left:k<=s->count?s->history[0][index]:s->first[0];
        int32_t y=k==0?right:k<=s->count?s->history[1][index]:s->first[1];
        l+=(int64_t)x*coefficient;r+=(int64_t)y*coefficient;
    }
    *out_left=saturated(l);*out_right=saturated(r);
}
