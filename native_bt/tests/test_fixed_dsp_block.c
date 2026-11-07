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
    uint32_t boundary_blocks=0;
    for(unsigned ri=0;ri<13;++ri){
        fixed_dsp_t initial;CHECK(fixed_dsp_init(&initial,rates[ri],90,500,512));
        fixed_dsp_controls(&initial,0,0,0,127,12,true);
        for(unsigned offset=0;offset<initial.decimation;++offset)for(unsigned frames=0;frames<=128;++frames){
            fixed_dsp_t a=initial,b=initial;
            /* Complete the next Goertzel window at the first decimation edge.
             * Odd signed inputs exercise truncation toward zero, not shifting. */
            a.window_count=b.window_count=(uint16_t)(initial.window_frames-1u);
            for(unsigned i=0;i<offset;++i){
                int32_t l=(i&1u)?INT32_MIN:INT32_MAX,r=(i&1u)?-3:2;
                int32_t el=l,er=r;fixed_dsp_process(&a,&l,&r);reference_process(&b,&el,&er);
            }
            int32_t actual[256],expected[256];
            for(unsigned i=0;i<2u*frames;++i){
                int32_t value=i%4u==0?INT32_MIN:i%4u==1?INT32_MAX:i%4u==2?-3:2;
                actual[i]=expected[i]=value;
            }
            CHECK(fixed_dsp_process_block(&a,actual,(uint16_t)frames));
            for(unsigned i=0;i<frames;++i)reference_process(&b,&expected[2u*i],&expected[2u*i+1u]);
            CHECK(!memcmp(actual,expected,2u*frames*sizeof(int32_t)) && !memcmp(&a,&b,sizeof a));
            ++boundary_blocks;++total_blocks;total_frames+=frames;
        }
    }
    printf("analysis boundaries=%u: every valid decimation offset, 0..128 frames, signed odd/rail PCM, Goertzel rollover\n",boundary_blocks);
    uint32_t volume_blocks=0;
    for(unsigned rate=0;rate<13;++rate)for(unsigned flags=0;flags<16;++flags){
        fixed_dsp_t a,b;CHECK(fixed_dsp_init(&a,rates[rate],90,500,512));CHECK(reference_init(&b,rates[rate],90,500,512));
        fixed_dsp_controls(&a,12,-3,9,127,(uint8_t)flags,true);reference_controls(&b,12,-3,9,127,(uint8_t)flags,true);
        int32_t actual[256],expected[256];
        for(unsigned block=0;block<192;++block){
            if(block<64?block%3u==0:block==64){
                uint8_t v=(uint8_t)(block==64?35:random32()%128u);
                fixed_dsp_controls(&a,12,-3,9,v,(uint8_t)flags,false);reference_controls(&b,12,-3,9,v,(uint8_t)flags,false);
            }
            unsigned frames=block%5u==0?128u:block%5u==1?1u:block%5u==2?2u:random32()%129u;
            for(unsigned f=0;f<5;++f)if(f!=3u)CHECK(!a.filters[f].remaining);
            if(a.filters[3].remaining && frames)++volume_blocks;
            for(unsigned i=0;i<2u*frames;++i){
                int32_t value=block%4u==0?0:block%4u==1?(i&1u?INT32_MIN:INT32_MAX):block%4u==2?(int32_t)random32():(i<2u?INT32_MAX/4:0);
                actual[i]=expected[i]=value;
            }
            CHECK(fixed_dsp_process_block(&a,actual,(uint16_t)frames));
            for(unsigned i=0;i<frames;++i)reference_process(&b,&expected[2u*i],&expected[2u*i+1u]);
            if(memcmp(actual,expected,frames*2u*sizeof(int32_t)) || memcmp(&a,&b,sizeof(a))){
                printf("Mismatch volume-only rate=%u flags=%u block=%u frames=%u\n",rates[rate],flags,block,frames);return 3;
            }
            total_frames+=frames;++total_blocks;
        }
    }
    CHECK(volume_blocks>1000u);printf("volume-only ramp blocks=%u\n",volume_blocks);
    const uint32_t stagger_rates[]={48000,96000,192000};
    for(unsigned ri=0;ri<3;++ri){
        fixed_dsp_t a,b;uint8_t flags=12,volume=127;int8_t bass=0,mid=0,treble=0;
        CHECK(fixed_dsp_init(&a,stagger_rates[ri],90,500,512));CHECK(reference_init(&b,stagger_rates[ri],90,500,512));
        fixed_dsp_controls(&a,bass,mid,treble,volume,flags,true);reference_controls(&b,bass,mid,treble,volume,flags,true);
        for(unsigned block=0;block<160;++block){
            bool change=false;
            if(!block){bass=12;mid=8;treble=12;change=true;}
            if(block==1){volume=90;change=true;}
            if(block==34){volume=30;change=true;}
            if(block==35){bass=-12;mid=12;treble=5;change=true;}
            if(block==68){flags=13;volume=80;change=true;}
            if(block==76){volume=50;change=true;}
            if(block==100){flags=15;volume=110;change=true;}
            if(block==132){volume=60;change=true;}
            if(change){fixed_dsp_controls(&a,bass,mid,treble,volume,flags,false);reference_controls(&b,bass,mid,treble,volume,flags,false);}
            int32_t actual[256],expected[256];
            for(unsigned i=0;i<256;++i){int32_t value=(int32_t)(random32()>>6);if(i&1u)value=-value;actual[i]=expected[i]=value;}
            CHECK(fixed_dsp_process_block(&a,actual,128));
            for(unsigned i=0;i<128;++i)reference_process(&b,&expected[2u*i],&expected[2u*i+1u]);
            CHECK(!memcmp(actual,expected,sizeof(actual)) && !memcmp(&a,&b,sizeof(a)));
            ++total_blocks;total_frames+=128;
        }
    }
#ifdef FIXED_DSP_VOLUME_RAMP_COMPARE
    fixed_dsp_volume_stats_t profile0,profile1;fixed_dsp_volume_stats(0,0,&profile0);fixed_dsp_volume_stats(0,1,&profile1);
    CHECK(profile0.coherent && profile1.coherent && profile0.calls>10 && profile1.calls>10 && profile0.rate==192000 && profile1.rate==192000);
    printf("ABBA modes exercised: canonical=%u candidate=%u (host timing stub)\n",profile0.calls,profile1.calls);
#endif
    printf("PASS exact block DSP: %u blocks, %u frames, 13 rates, full PCM/state equality\n",total_blocks,total_frames);
    return 0;
}
