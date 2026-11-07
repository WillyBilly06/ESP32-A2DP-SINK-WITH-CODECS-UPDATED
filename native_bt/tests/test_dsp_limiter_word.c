#include <assert.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "../../bt_audio_sink/main/dsp/fixed_dsp.c"
static int32_t original_limit(fixed_dsp_t *state,int32_t value) {
    uint32_t magnitude=(uint32_t)(value<0?-(int64_t)value:value);
    if(magnitude>state->peak_q27)state->peak_q27=magnitude;
    if(magnitude<=CLIP_THRESHOLD)return value;
    ++state->limited;
    if(magnitude>=AUDIO_ONE)++state->over_full_scale;
    uint64_t excess=magnitude-CLIP_THRESHOLD;
    int32_t output=CLIP_THRESHOLD+(int32_t)((uint64_t)CLIP_RANGE*excess/(CLIP_RANGE+excess));
    return value<0?-output:output;
}
static void state_case(int32_t value,uint32_t serial) {
    fixed_dsp_t candidate,reference;
    memset(&candidate,0x5a,sizeof(candidate));
    candidate.peak_q27=serial&1u?UINT32_MAX:0;
    candidate.limited=UINT32_MAX-(serial&3u);candidate.over_full_scale=UINT32_MAX-(serial&7u);
    reference=candidate;
    int32_t actual=limit(&candidate,value),expected=original_limit(&reference,value);
    assert(actual==expected && !memcmp(&candidate,&reference,sizeof(candidate)));
}
static void magnitude_case(uint32_t magnitude) {
    assert(magnitude>CLIP_THRESHOLD && magnitude<=2147483648u);
    uint32_t excess=magnitude-CLIP_THRESHOLD;
    uint32_t expected=CLIP_THRESHOLD+(uint32_t)((uint64_t)CLIP_RANGE*excess/(CLIP_RANGE+excess));
    uint32_t actual=AUDIO_ONE-limiter_tail_ceil(CLIP_RANGE+excess);
    if(actual!=expected){fprintf(stderr,"FAIL magnitude=%" PRIu32 " expected=%" PRIu32 " actual=%" PRIu32 "\n",magnitude,expected,actual);abort();}
}
int main(int argc,char **argv) {
    const int32_t edges[]={INT32_MIN,INT32_MIN+1,-AUDIO_ONE-1,-AUDIO_ONE,-AUDIO_ONE+1,-CLIP_THRESHOLD-1,-CLIP_THRESHOLD,-CLIP_THRESHOLD+1,-1,0,1,CLIP_THRESHOLD-1,CLIP_THRESHOLD,CLIP_THRESHOLD+1,AUDIO_ONE-1,AUDIO_ONE,AUDIO_ONE+1,INT32_MAX-1,INT32_MAX};
    uint32_t serial=0;
    for(unsigned repeat=0;repeat<16;++repeat)for(unsigned i=0;i<sizeof(edges)/sizeof(edges[0]);++i)state_case(edges[i],serial++);
    uint32_t seed=0x7395bc81u;
    for(unsigned i=0;i<1000000;++i){seed^=seed<<13;seed^=seed>>17;seed^=seed<<5;int32_t value;memcpy(&value,&seed,sizeof(value));state_case(value,serial++);}
    for(unsigned bit=25;bit<32;++bit)for(int offset=-1024;offset<=1024;++offset){
        int64_t divisor=(int64_t)(1u<<bit)+offset;
        int64_t magnitude=divisor-CLIP_RANGE+CLIP_THRESHOLD;
        if(magnitude>CLIP_THRESHOLD && magnitude<=2147483648LL)magnitude_case((uint32_t)magnitude);
    }
    bool exhaustive=argc==1;
    if(!exhaustive)assert(argc==2 && !strcmp(argv[1],"--quick"));
    uint32_t checked=0;
    if(exhaustive){
        for(uint32_t magnitude=CLIP_THRESHOLD+1u;magnitude<=2147483648u;++magnitude){
            magnitude_case(magnitude);++checked;
            if((checked&0x1fffffffu)==0){printf("checked=%" PRIu32 "\n",checked);fflush(stdout);}
        }
        assert(checked==2147483648u-CLIP_THRESHOLD);
    }
    printf("PASS %" PRIu32 " exhaustive limiter magnitudes; %" PRIu32 " signed full-state cases\n",checked,serial);
    return 0;
}
