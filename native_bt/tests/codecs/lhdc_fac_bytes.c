#include <stdint.h>
#include <string.h>
#include "dbt_lhdc_bits.h"
static uint32_t random_state=0x13579bdf;
static uint32_t random_word(void){random_state=random_state*1664525u+1013904223u;return random_state;}
static uint64_t cases;
static int compare(lhdc_dec_bit_reader_t initial,int capacity){
    lhdc_dec_bit_reader_t a=initial,b=initial;
    uint8_t actual[776],expected[776];memset(actual,0xa5,sizeof(actual));memset(expected,0xa5,sizeof(expected));
    int n=0;
    while(lhdc_bit_reader_remaining(&b)>=8 && n<capacity)expected[4+n++]=(uint8_t)lhdc_bit_reader_read(&b,8);
    int got=dbt_lhdc_read_fac_bytes(&a,actual+4,capacity);
    if(got!=n || memcmp(actual,expected,sizeof(actual)) || a.data!=b.data || a.data_bytes!=b.data_bytes ||
       a.byte_pos!=b.byte_pos || a.cache!=b.cache || a.cache_bits!=b.cache_bits)return __LINE__;
    ++cases;return 0;
}
#ifdef _WIN32
__declspec(dllexport)
#endif
int lhdc_fac_bytes_test(void){
    uint8_t data[1540];for(unsigned i=0;i<sizeof(data);++i)data[i]=(uint8_t)random_word();
    static const int capacities[]={0,1,2,3,4,7,8,9,15,16,31,64,127,255,512,767,768};
    for(unsigned alignment=0;alignment<4;++alignment)for(unsigned length=0;length<264;++length){
        size_t bytes=length<=257?length:(size_t[]){383,511,767,768,1023,1536}[length-258];
        for(unsigned bits=0;bits<=64;++bits)for(unsigned position=0;position<3;++position){
            lhdc_dec_bit_reader_t br={0};br.data=data+alignment;br.data_bytes=bytes;
            br.byte_pos=position==0?0:position==1?bytes/2:bytes;br.cache_bits=(int)bits;
            uint64_t word=((uint64_t)random_word()<<32)|random_word();
            br.cache=bits==64?word:word&((UINT64_C(1)<<bits)-1u);
            for(unsigned c=0;c<sizeof(capacities)/sizeof(capacities[0]);++c){int line=compare(br,capacities[c]);if(line)return line;}
        }
    }
    uint8_t output[16]={0};lhdc_dec_bit_reader_t br={0};
    /* Exhaust every possible residual-bit carry and following input byte. */
    for(unsigned bits=0;bits<8;++bits)for(unsigned carry=0;carry<(1u<<bits);++carry)
        for(unsigned byte=0;byte<256;++byte){uint8_t next=(uint8_t)byte;
            lhdc_dec_bit_reader_t edge={.data=&next,.data_bytes=1,.cache=carry,.cache_bits=(int)bits};
            if(compare(edge,1))return __LINE__;}
    if(dbt_lhdc_read_fac_bytes(NULL,output,8) || dbt_lhdc_read_fac_bytes(&br,NULL,8) || dbt_lhdc_read_fac_bytes(&br,output,-1))return __LINE__;
    br.cache_bits=64;br.cache=UINT64_C(0x0123456789abcdef);
    if(compare(br,8))return __LINE__;
    br.cache_bits=65;if(dbt_lhdc_read_fac_bytes(&br,output,8))return __LINE__;
    br.cache_bits=-1;if(dbt_lhdc_read_fac_bytes(&br,output,8))return __LINE__;
    br.cache_bits=0;br.byte_pos=1;if(dbt_lhdc_read_fac_bytes(&br,output,8))return __LINE__;
    return 0;
}
#ifdef _WIN32
__declspec(dllexport)
#endif
uint64_t lhdc_fac_bytes_cases(void){return cases;}
#ifdef DBT_LHDC_BITS_SANITIZER_MAIN
#include <stdio.h>
int main(void){int line=lhdc_fac_bytes_test();
    if(line){printf("failed_line=%d\n",line);return 1;}
    printf("cases=%llu\n",(unsigned long long)cases);return 0;}
#endif
