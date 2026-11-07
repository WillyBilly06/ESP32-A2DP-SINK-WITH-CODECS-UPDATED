/* Actual matched helper versus the independent five-product scalar filter. */
#define FIXED_DSP_BLOCK_API 1
#define FIXED_DSP_WORD_QUANT 1
#define FIXED_DSP_MATCHED_FEEDBACK 1
#include "../../bt_audio_sink/main/dsp/fixed_dsp.c"
#include <stdio.h>
#include <string.h>
#define CHECK(x) do{if(!(x)){printf("FAIL matched feedback line %u\n",(unsigned)__LINE__);return 1;}}while(0)
static uint32_t seed=0x8432987au;
static uint32_t random32(void){seed^=seed<<13;seed^=seed>>17;seed^=seed<<5;return seed;}
int main(void){
    uint32_t cases=0,tables=0,same=0,opposite=0,matching=0,ordinary=0;
    for(unsigned rate=0;rate<FDSP_RATE_COUNT;++rate)for(unsigned band=0;band<3;++band)for(unsigned gain=0;gain<25;++gain){
        const int32_t *c=fdsp_eq[rate][band][gain];
        if(c[0]==COEFF_ONE && !c[1] && !c[2] && !c[3] && !c[4])continue;
        ++tables;if(c[1]==c[3])++matching;else ++ordinary;
        for(unsigned i=0;i<2048;++i){
            fdsp_history_t a={(int32_t)random32(),(int32_t)random32(),(int32_t)random32(),(int32_t)random32(),random32()&0x0fffffffu};
            if(i%8u==0){a.x1=INT32_MIN;a.y1=INT32_MAX;}
            if(i%8u==1){a.x1=INT32_MAX;a.y1=INT32_MIN;}
            if(i%8u==2){a.x1=INT32_MIN;a.y1=-1;}
            if(i%8u==3){a.x1=INT32_MAX;a.y1=0;}
            if(((uint32_t)a.x1^(uint32_t)a.y1)&UINT32_C(0x80000000))++opposite;else ++same;
            fdsp_history_t b=a;int32_t pcm[2]={(int32_t)random32(),0x13579bdf};int32_t expected=filter(c,&b,pcm[0]);
            if(c[1]==c[3])filter_matched_feedback_block(c,&a,pcm,1);else filter_steady_block(c,&a,pcm,1);CHECK(pcm[0]==expected && pcm[1]==0x13579bdf && !memcmp(&a,&b,sizeof a));++cases;
        }
    }
    CHECK(cases && tables && matching && ordinary && same && opposite);
    printf("matching tables=%u ordinary tables=%u\n",matching,ordinary);
#ifdef FIXED_DSP_MATCHED_FEEDBACK_COMPARE
    fixed_dsp_t d;CHECK(fixed_dsp_init(&d,192000,90,500,512));fixed_dsp_controls(&d,2,8,12,93,12,true);
    for(unsigned i=0;i<16;++i){int32_t pcm[256]={0};CHECK(fixed_dsp_process_block(&d,pcm,128));}
    for(unsigned mode=0;mode<2;++mode){fixed_dsp_volume_stats_t s;fixed_dsp_volume_stats(0,mode,&s);CHECK(s.coherent && s.calls==8 && !s.clipped && s.rate==192000);}
#endif
    printf("PASS %u matched-feedback histories, %u generated nonidentity tables, %u same-sign and %u opposite-sign cases\n",cases,tables,same,opposite);return 0;
}
