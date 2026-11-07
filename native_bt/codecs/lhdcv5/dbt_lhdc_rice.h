#ifndef DBT_LHDC_RICE_H
#define DBT_LHDC_RICE_H
#include <stdint.h>

/* Included after the private lazy FAC cursor. Keep the current symbol locally:
 * rereading it cannot consume another FAC symbol or change the model. In the
 * short-three case do not look ahead: that byte/range state belongs to the
 * mantissa boundary. Invalid escapes return a checked error, never shift UB. */
static LHDC_HOT int rice_read_quotient(s2_lazy_t *z,int *jp)
{
    int j=*jp,symbol=s2_at(z,j);
    if(symbol==2){
        uint32_t low=0;unsigned shift=0;
        do {
            if(shift>=32u){z->error=1;*jp=j;return 0;}
            ++j;
            int v1=s2_at(z,j);uint32_t b1=v1<0?0u:(uint32_t)v1;++j;
            int v0=s2_at(z,j);uint32_t b0=v0<0?0u:(uint32_t)v0;++j;
            low|=((b1<<1)|b0)<<shift;shift+=2u;
            symbol=s2_at(z,j);
        }while(symbol==2);
        int ones=0;
        while(symbol==1){++ones;++j;symbol=s2_at(z,j);}
        if(symbol==0)++j;
        *jp=j;
        if(shift>=31u || (uint32_t)(ones+1)>((uint32_t)INT32_MAX>>shift)){
            z->error=1;return 0;
        }
        uint32_t value=low|((uint32_t)(ones+1)<<shift);
        if(value>(uint32_t)INT32_MAX){z->error=1;return 0;}
        return (int)value;
    }
    int ones=0;
    while(ones<3 && symbol==1){
        ++ones;++j;
        if(ones<3)symbol=s2_at(z,j);
    }
    if(ones==3){*jp=j;return 3;}
    if(symbol==0)++j;
    *jp=j;return ones;
}
#endif
