#ifndef DBT_LHDC_MANTISSA_H
#define DBT_LHDC_MANTISSA_H
#include <stdint.h>
/* One channel, at most the configured coefficient count. Valid decoder calls
 * supply 0<=init_shift<=15, a nonnegative start/extent, and disjoint coeff/out.
 * Retain the original zero-filled reads beyond the encoded slice. The sliding
 * word uses byte loads only; no unaligned access or input overread is required.
 * All intermediate magnitudes and peak comparisons use defined 32-bit words.
 * No persistent storage, allocation, floating-point arithmetic or waits. */
static __attribute__((always_inline)) inline int64_t dbt_lhdc_mantissa_fast(const int32_t *coeff,int32_t *out,int count,
    const uint8_t *payload,int total_bits,int start_bit,int pred_mode,int init_shift){
    int p=start_bit,pred=init_shift<<7,cached_byte=-4;
    uint32_t word=0,peak=0;
    for(int k=0;k<count;++k){
        const int shift=(pred+0x40)>>7;
        uint32_t mant=0;
        if(shift){
            const int byte=p>>3,off=p&7;
            if(shift<=24 && byte+4<=total_bits/8){
                const int advance=byte-cached_byte;
                if(advance==1)word=(word<<8)|payload[byte+3];
                else if(advance==0){ /* keep the already loaded word */ }
                else if(advance==2)word=(word<<16)|((uint32_t)payload[byte+2]<<8)|payload[byte+3];
                else if(advance==3)word=(word<<24)|((uint32_t)payload[byte+1]<<16)|((uint32_t)payload[byte+2]<<8)|payload[byte+3];
                else word=((uint32_t)payload[byte]<<24)|((uint32_t)payload[byte+1]<<16)|((uint32_t)payload[byte+2]<<8)|payload[byte+3];
                cached_byte=byte;mant=(word<<off)>>(32-shift);p+=shift;
            }else{
                for(int bit=0;bit<shift;++bit){
                    const uint32_t value=p<total_bits?(payload[p>>3]>>(7-(p&7)))&1u:0u;
                    ++p;mant=(mant<<1)|value;
                }
            }
        }
        const uint32_t magnitude=((uint32_t)coeff[k]<<shift)|mant;
        uint32_t value=magnitude;
        if(magnitude){
            const int relative=p-cached_byte*8;
            const uint32_t negative=relative>=0 && relative<32?(word>>(31-relative))&1u:
                (p<total_bits?(payload[p>>3]>>(7-(p&7)))&1u:0u);
            ++p;if(negative)value=0u-magnitude;
        }
        out[k]=(int32_t)value;
        const uint32_t absolute=(int32_t)magnitude<0?0u-magnitude:magnitude;
        if(absolute>peak)peak=absolute;
        unsigned width=absolute?32u-(unsigned)__builtin_clz(absolute):0u;
        if(width>31u)width=31u;
        const int x=(int)width<<7;
        pred=pred_mode?((7*pred+x)>>3):((5*x+3*pred)>>3);
    }
    return (int64_t)peak;
}
#endif
