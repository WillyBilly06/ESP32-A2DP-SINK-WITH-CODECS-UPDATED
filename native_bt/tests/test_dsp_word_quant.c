/* Independent division/remainder oracle over the complete signed64 domain. */
#define FIXED_DSP_BLOCK_API 1
#define FIXED_DSP_WORD_QUANT 1
#include "../../bt_audio_sink/main/dsp/fixed_dsp.c"
#include <assert.h>
#include <stdio.h>
static void check(int64_t sum){
    const int64_t scale=INT64_C(268435456);
    int64_t quotient=sum/scale,remainder=sum%scale;
    if(remainder<0){--quotient;remainder+=scale;}
    int32_t expected;
    uint32_t carried;
    if(quotient>INT32_MAX){expected=INT32_MAX;carried=0;}
    else if(quotient<INT32_MIN){expected=INT32_MIN;carried=0;}
    else{expected=(int32_t)quotient;carried=(uint32_t)remainder;}
    uint32_t fraction=0xffffffffu;
    int32_t actual=block_word_quantize(sum,&fraction);
    assert(actual==expected && fraction==carried);
}
int main(void){
    const int64_t scale=INT64_C(268435456);
    const int64_t edges[]={INT64_MIN,INT64_MAX,0,-1,1,
        (int64_t)INT32_MIN*scale,(int64_t)INT32_MAX*scale,
        ((int64_t)INT32_MAX+1)*scale,((int64_t)INT32_MIN-1)*scale};
    uint64_t count=0;
    for(unsigned i=0;i<sizeof(edges)/sizeof(edges[0]);++i){
        check(edges[i]);++count;
        for(int delta=-64;delta<=64;delta++){
            if((delta<0 && edges[i]<INT64_MIN-delta) || (delta>0 && edges[i]>INT64_MAX-delta))continue;
            check(edges[i]+delta);++count;
        }
    }
    uint64_t random=UINT64_C(0x8392746a13ce59b7);
    for(unsigned i=0;i<2000000u;++i){
        random^=random<<13;random^=random>>7;random^=random<<17;
        check((int64_t)random);++count;
    }
    printf("PASS %llu signed64 word-quantization cases\n",(unsigned long long)count);
    return 0;
}
