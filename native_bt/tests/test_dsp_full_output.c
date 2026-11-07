/* Independent wide-division oracle for steady gain/limiting/output scaling. */
#define FIXED_DSP_BLOCK_API 1
#include "../../bt_audio_sink/main/dsp/fixed_dsp.c"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static uint32_t seed=UINT32_C(0xa3c64891);
static uint32_t next_word(void){seed^=seed<<13;seed^=seed>>17;seed^=seed<<5;return seed;}
static int32_t expected_sample(fixed_dsp_t *state,int32_t input,int32_t gain){
    int64_t value=input;
    if(gain!=INT32_MAX){
        int64_t product=value*gain,scale=INT64_C(2147483648);
        value=product/scale;
        if(product<0 && product%scale)--value;
        if(value>INT32_MAX)value=INT32_MAX;
        if(value<INT32_MIN)value=INT32_MIN;
    }
    int64_t magnitude=value<0?-value:value;
    if((uint64_t)magnitude>state->peak_q27)state->peak_q27=(uint32_t)magnitude;
    const int64_t knee=INT64_C(114085069),unity=INT64_C(134217728),range=unity-knee;
    if(magnitude>knee){
        ++state->limited;
        if(magnitude>=unity)++state->over_full_scale;
        int64_t excess=magnitude-knee;
        int64_t bounded=knee+range*excess/(range+excess);
        value=value<0?-bounded:bounded;
    }
    assert(value>-unity && value<unity);
    value*=16;
    assert(value>=INT32_MIN && value<=INT32_MAX);
    return (int32_t)value;
}
int main(void){
    const int32_t gains[]={0,1,2,INT32_MAX,INT32_MAX-1,INT32_MIN,INT32_MIN+1,-1,-2,
        INT32_C(134217728),INT32_C(1073741824),-INT32_C(1073741824),INT32_C(2000000000),
        -INT32_C(2000000000)};
    const int32_t edges[]={INT32_MIN,INT32_MIN+1,INT32_MAX,INT32_MAX-1,0,1,-1,
        114085068,114085069,114085070,-114085068,-114085069,-114085070,
        134217727,134217728,134217729,-134217727,-134217728,-134217729};
    uint32_t samples=0,blocks=0;
    for(unsigned g=0;g<sizeof(gains)/sizeof(gains[0])+2u;++g)for(unsigned batch=0;batch<1024u;++batch){
        fixed_dsp_t actual,expected;
        assert(fixed_dsp_init(&actual,192000,90,500,512));
        fixed_dsp_controls(&actual,2,8,12,68,12,true);
        int32_t gain=g<sizeof(gains)/sizeof(gains[0])?gains[g]:(int32_t)next_word();
        actual.volume.value=actual.volume.target=gain;
        actual.volume.remaining=actual.bypass.remaining=actual.flip.remaining=0;
        actual.peak_q27=batch%3u==0?UINT32_MAX:batch%3u==1?0:next_word();
        actual.limited=UINT32_MAX-7u;actual.over_full_scale=UINT32_MAX-3u;
        expected=actual;
        unsigned frames=batch%8u==0?0u:batch%8u==1?1u:batch%8u==2?2u:
            batch%8u==3?127u:batch%8u==4?128u:next_word()%129u;
        int32_t pcm[256],reference[256];
        for(unsigned i=0;i<2u*frames;++i){
            pcm[i]=batch%2u==0?edges[(batch+i)%(sizeof(edges)/sizeof(edges[0]))]:(int32_t)next_word();
            reference[i]=expected_sample(&expected,pcm[i],gain);
        }
        finish_full_block(&actual,pcm,frames);
        assert(!memcmp(pcm,reference,2u*frames*sizeof(int32_t)));
        assert(!memcmp(&actual,&expected,sizeof(actual)));
        samples+=2u*frames;++blocks;
    }
    printf("PASS %u full-range finalizer channel samples / %u blocks: wide-division oracle, gain and limiter boundaries, counter wraps, complete state\n",samples,blocks);
    return 0;
}
