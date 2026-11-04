#include "dbt_sound.h"
#include <limits.h>
dbt_result_t dbt_sound_wav(dbt_sound_t *s,uint8_t t,bool tmp,uint32_t n,dbt_sound_wav_t *out){(void)s;(void)t;(void)tmp;(void)n;(void)out;return DBT_INVALID;}
#include "../../src/dbt_sound_mix.c"
int main(void){
    if(clamp64((int64_t)INT32_MAX+1)!=INT32_MAX)return 1;
    if(clamp64((int64_t)INT32_MIN-1)!=INT32_MIN)return 2;
    if(clamp64(-123456789)!=-123456789)return 3;
    if(interpolate(INT32_MIN,INT32_MAX,0x80000000u)!=-1)return 4;
    return 0;
}
