#include "fixed_dsp.h"
#include <limits.h>
#include <stdio.h>
#include <string.h>
bool reference_init(fixed_dsp_t *,uint32_t,uint32_t,uint32_t,uint32_t);
void reference_controls(fixed_dsp_t *,int8_t,int8_t,int8_t,uint8_t,uint8_t,bool);
void reference_process(fixed_dsp_t *,int32_t *,int32_t *);
static uint32_t seed=0x193751abu;
static uint32_t random32(void){seed^=seed<<13;seed^=seed>>17;seed^=seed<<5;return seed;}
#define CHECK(x) do{if(!(x)){printf("FAIL block DSP line %u\n",(unsigned)__LINE__);return 1;}}while(0)
static void controls(fixed_dsp_t *a,fixed_dsp_t *b,unsigned phase,unsigned within){
    if(within && phase!=0 && phase!=3)return;
    int8_t bass=(int8_t)(random32()%25u)-12,mid=(int8_t)(random32()%25u)-12,treble=(int8_t)(random32()%25u)-12;
    uint8_t volume=(uint8_t)(random32()%128u),flags=(uint8_t)(random32()&15u);bool immediate=within==0 && (phase==0 || phase==2);
    if(phase==1){bass=8;mid=-1;treble=-2;volume=50;flags=9;}
    if(phase==2){bass=mid=treble=0;volume=127;flags=12;}
    fixed_dsp_controls(a,bass,mid,treble,volume,flags,immediate);
    reference_controls(b,bass,mid,treble,volume,flags,immediate);
}
int main(void){
    const uint32_t rates[]={8000,11025,12000,16000,22050,24000,32000,44100,48000,88200,96000,176400,192000};
    uint32_t total_frames=0,total_blocks=0;
    for(unsigned rate=0;rate<13;++rate){
        fixed_dsp_t a,b;CHECK(fixed_dsp_init(&a,rates[rate],90,500,512));CHECK(reference_init(&b,rates[rate],90,500,512));
        int32_t actual[FIXED_DSP_BLOCK_FRAMES*2u],expected[FIXED_DSP_BLOCK_FRAMES*2u];
        CHECK(fixed_dsp_process_block(&a,0,0));
        fixed_dsp_t saved=a;
        CHECK(!fixed_dsp_process_block(&a,actual,FIXED_DSP_BLOCK_FRAMES+1u));
        CHECK(!fixed_dsp_process_block(&a,0,1) && !fixed_dsp_process_block(0,actual,1));
        CHECK(!fixed_dsp_process_block(&a,(int32_t *)((uint8_t *)actual+1),1));
        CHECK(!memcmp(&a,&saved,sizeof a));
        for(unsigned block=0;block<2048;++block){
            unsigned within=block%128u,phase=(block/128u)%4u;
            controls(&a,&b,phase,within);
            unsigned frames=within==0?1u:within==1?2u:within==2?128u:random32()%129u;
            if(block%257u==0){
                /* General int32 histories exercise saturation and the exact
                 * identity transition where prior x and y histories differ. */
                for(unsigned f=0;f<4;++f)for(unsigned ch=0;ch<2;++ch){
                    fdsp_history_t h={(int32_t)random32(),(int32_t)random32(),(int32_t)random32(),(int32_t)random32(),random32()&0x0fffffffu};
                    a.channel_history[f][ch]=b.channel_history[f][ch]=h;
                }
            }
            for(unsigned i=0;i<frames*2u;++i){
                int32_t value=block%3u==0?(i&1u?INT32_MIN:INT32_MAX):block%3u==1?(int32_t)(random32()&0xffffff00u):(int32_t)random32();
                actual[i]=expected[i]=value;
            }
            CHECK(fixed_dsp_process_block(&a,actual,(uint16_t)frames));
            for(unsigned i=0;i<frames;++i)reference_process(&b,&expected[2u*i],&expected[2u*i+1u]);
            if(memcmp(actual,expected,frames*2u*sizeof(int32_t)) || memcmp(&a,&b,sizeof a)){
                printf("Mismatch rate=%u block=%u frames=%u phase=%u\n",rates[rate],block,frames,phase);return 2;
            }
            total_frames+=frames;++total_blocks;
        }
    }
    printf("PASS exact block DSP: %u blocks, %u frames, 13 rates, full PCM/state equality\n",total_blocks,total_frames);
    return 0;
}
