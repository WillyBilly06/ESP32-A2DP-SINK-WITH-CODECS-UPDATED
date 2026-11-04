#ifndef DBT_UTF8_H
#define DBT_UTF8_H
#include <stdbool.h>
#include <stdint.h>
/* Bounded UTF-8 without embedded NUL, overlong/surrogate/out-of-range scalars. */
static inline bool dbt_utf8_valid(const uint8_t *p,uint16_t size){
    if(!p || !size)return false;
    for(unsigned i=0;i<size;){
        unsigned n;uint32_t cp;uint8_t first=p[i];
        if(first && first<0x80){n=1;cp=first;}
        else if(first>=0xc2 && first<=0xdf){n=2;cp=first&31u;}
        else if(first>=0xe0 && first<=0xef){n=3;cp=first&15u;}
        else if(first>=0xf0 && first<=0xf4){n=4;cp=first&7u;}else return false;
        if(i+n>size)return false;
        for(unsigned j=1;j<n;j++){if((p[i+j]&0xc0u)!=0x80u)return false;cp=(cp<<6)|(p[i+j]&63u);}
        if((n==2 && cp<0x80) || (n==3 && cp<0x800) || (n==4 && cp<0x10000) || cp>0x10ffff || (cp>=0xd800 && cp<=0xdfff))return false;
        i+=n;
    }
    return true;
}
#endif
