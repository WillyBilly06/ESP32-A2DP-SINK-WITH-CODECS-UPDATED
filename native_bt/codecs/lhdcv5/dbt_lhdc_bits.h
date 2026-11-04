#ifndef DBT_LHDC_BITS_H
#define DBT_LHDC_BITS_H
#include "lhdc_bit_reader.h"
#include <string.h>

/* Equivalent to the bounded FAC loop's repeated read(br,8), including the
 * reader state left behind. Drain prefetched bytes, then keep only the 0..7
 * residual bits in a 32-bit carry. The caller supplies disjoint FAC scratch
 * with its existing fixed capacity (768 bytes in the current decoder slot).
 * No direct unaligned word access, extra input reads or padded-byte synthesis. */
static __attribute__((always_inline)) inline int dbt_lhdc_read_fac_bytes(
    lhdc_dec_bit_reader_t *br,uint8_t *output,int capacity){
    if(!br || !output || capacity<=0 || br->byte_pos>br->data_bytes ||
       br->cache_bits<0 || br->cache_bits>64 || (!br->data && br->data_bytes))return 0;
    size_t left=br->data_bytes-br->byte_pos;
    size_t count=left>(size_t)capacity?(size_t)capacity:left;
    unsigned cached=(unsigned)br->cache_bits/8u;
    if(count<(size_t)capacity){size_t extra=(size_t)capacity-count;
        count+=cached<extra?cached:extra;}
    size_t copied=0;
    while(copied<count && br->cache_bits>=8){
        unsigned shift=(unsigned)br->cache_bits-8u;
        output[copied++]=(uint8_t)(br->cache>>shift);
        br->cache_bits=(int)shift;br->cache&=(UINT64_C(1)<<shift)-1u;
    }
    if(copied==count)return (int)count;
    unsigned bits=(unsigned)br->cache_bits;
    size_t bytes=count-copied;
    const uint8_t *source=br->data+br->byte_pos;
    if(!bits){memcpy(output+copied,source,bytes);br->cache=0;}
    else {
        uint32_t carry=(uint32_t)br->cache,mask=(1u<<bits)-1u;
        for(size_t i=0;i<bytes;++i){uint32_t value=(carry<<8)|source[i];
            output[copied+i]=(uint8_t)(value>>bits);carry=value&mask;}
        br->cache=carry;
    }
    br->byte_pos+=bytes;
    return (int)count;
}
#endif
