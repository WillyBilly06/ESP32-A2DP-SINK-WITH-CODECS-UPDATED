#include "dbt_audio.h"
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#if defined(__has_feature)
#if __has_feature(address_sanitizer)
#include <sanitizer/asan_interface.h>
#define POISON(p,n) __asan_poison_memory_region(p,n)
#define UNPOISON(p,n) __asan_unpoison_memory_region(p,n)
#endif
#endif
#ifndef POISON
#define POISON(p,n) ((void)0)
#define UNPOISON(p,n) ((void)0)
#endif
#define CHECK(x) do{if(!(x)){printf("FAIL line %u\n",(unsigned)__LINE__);return 1;}}while(0)
typedef struct {unsigned live,calls,fail;uint32_t bytes;} memory_t;
static uint8_t *allocate(void *context,uint32_t bytes){
    memory_t *m=context;if(++m->calls==m->fail)return 0;
    uint64_t *p=malloc((size_t)bytes+8u);if(!p)return 0;*p=bytes;
    ++m->live;m->bytes+=bytes;return (uint8_t *)(p+1);
}
static void release(void *context,uint8_t *bytes){
    memory_t *m=context;uint64_t *p=(uint64_t *)bytes-1;
    --m->live;m->bytes-=(uint32_t)*p;free(p);
}
static uint32_t rng=0x517ca391u;
static uint32_t random32(void){rng^=rng<<13;rng^=rng>>17;rng^=rng<<5;return rng;}
static int compare_step(dbt_audio_t *borrowed,dbt_audio_t *copied){
    int32_t actual[256],expected[256];
    uint32_t a=dbt_audio_render(borrowed,128),b=dbt_audio_render(copied,128);
    if(a!=b || dbt_audio_input_frames(borrowed)!=dbt_audio_input_frames(copied))return -1;
    if(dbt_audio_take(borrowed,actual,a)!=a || dbt_audio_take(copied,expected,b)!=b)return -1;
    if(memcmp(actual,expected,a*8u) || memcmp(&borrowed->dsp,&copied->dsp,sizeof(borrowed->dsp)) ||
       memcmp(&borrowed->resampler,&copied->resampler,sizeof(borrowed->resampler)) ||
       borrowed->fraction!=copied->fraction || borrowed->drain_remaining!=copied->drain_remaining)return -1;
    return (int)a;
}
static int compare_stream(uint32_t rate,unsigned width,unsigned channels,int32_t adjustment,bool adaptive){
    dbt_audio_t a,b;memory_t ma={0},mb={0};
    dbt_audio_config_t cfg={rate,256,513,128,(dbt_pcm_format_t)width,(uint8_t)channels,adaptive};
    CHECK(dbt_audio_init(&a,(dbt_audio_memory_t){&ma,allocate,release})==DBT_OK);
    CHECK(dbt_audio_init(&b,(dbt_audio_memory_t){&mb,allocate,release})==DBT_OK);
    CHECK(dbt_audio_open_borrowed(&a,cfg)==DBT_OK);
    cfg.input_frames=512;CHECK(dbt_audio_open(&b,cfg)==DBT_OK);
    CHECK(ma.bytes==513u*8u && mb.bytes==513u*8u+512u*width*channels && !a.input.bytes);
    a.fraction=b.fraction=UINT32_MAX-1u;a.step_adjust=b.step_adjust=adjustment;
    atomic_store(&a.input.read,UINT32_MAX-5u);atomic_store(&a.input.written,UINT32_MAX-5u);
    atomic_store(&b.input.read,UINT32_MAX-5u);atomic_store(&b.input.written,UINT32_MAX-5u);
    CHECK(dbt_audio_controls(&a,3,-2,2,80,9)==DBT_OK && dbt_audio_controls(&b,3,-2,2,80,9)==DBT_OK);
    uint8_t storage[256*8+8];unsigned submitted=0,round=0;
    while(submitted<4096){
        unsigned frames=round%5u==0?1u:round%11u==1?256u:1u+random32()%256u;
        if(frames>4096u-submitted)frames=4096u-submitted;
        UNPOISON(storage,sizeof(storage));uint8_t *pcm=storage+(round&1u);
        for(unsigned i=0;i<frames*channels;++i){
            uint32_t v=i%3u?random32():i&1u?0x80000000u:0x7fffffffu;
            for(unsigned byte=0;byte<width;++byte)pcm[i*width+byte]=(uint8_t)(v>>(byte*8u));
        }
        CHECK(dbt_audio_input_borrow(&a,pcm,frames)==DBT_OK && dbt_audio_submit(&b,pcm,frames)==DBT_OK);
        CHECK(dbt_audio_input_borrow(&a,pcm,1)==DBT_BUSY);
        unsigned steps=0;
        while(!dbt_audio_input_release(&a)){
            CHECK(compare_step(&a,&b)>=0 && ++steps<=4u);
        }
        /* The provider may now acknowledge reuse. No sample, including the
         * boundary carry, may read this storage until the next explicit loan. */
        POISON(storage,sizeof(storage));
        CHECK(compare_step(&a,&b)>=0);
        submitted+=frames;++round;
        if(round%7u==0){
            int8_t gain=(int8_t)(round%25u)-12;
            CHECK(dbt_audio_controls(&a,gain,-gain,gain,64,13)==DBT_OK && dbt_audio_controls(&b,gain,-gain,gain,64,13)==DBT_OK);
        }
    }
    dbt_audio_begin_drain(&a);dbt_audio_begin_drain(&b);
    for(unsigned i=0;i<4;++i){int made=compare_step(&a,&b);CHECK(made>=0);if(!made)break;}
    CHECK(!a.drain_remaining && !a.input_view);
    if(adaptive)CHECK(!dbt_audio_input_frames(&a));
    else {
        /* The legacy linear path retains its final lookahead sample when
         * manually given a nonzero fraction. Verify identical pending state,
         * not an invented drain guarantee absent from that existing path. */
        CHECK(dbt_audio_input_frames(&a)==dbt_audio_input_frames(&b) && dbt_audio_input_frames(&a)<=1u);
        CHECK(dbt_audio_input_cancel(&a)<=1u);
    }
    UNPOISON(storage,sizeof(storage));dbt_audio_close(&a);dbt_audio_close(&b);
    CHECK(!ma.live && !mb.live && !ma.bytes && !mb.bytes);return 0;
}
static int boundaries(void){
    dbt_audio_t a;memory_t m={0};uint8_t pcm[32]={0};
    dbt_audio_config_t cfg={192000,256,513,128,DBT_PCM_S32_LE,2,true};
    CHECK(dbt_audio_init(&a,(dbt_audio_memory_t){&m,allocate,release})==DBT_OK);
    for(unsigned fail=1;fail<=2;++fail){
        m=(memory_t){.fail=fail};CHECK(dbt_audio_open_borrowed(&a,cfg)==DBT_NO_SPACE && !m.live && !m.bytes && !a.opened);
    }
    m=(memory_t){0};CHECK(dbt_audio_open_borrowed(&a,cfg)==DBT_OK);
    CHECK(dbt_audio_submit(&a,pcm,1)==DBT_INVALID);
    CHECK(dbt_audio_input_borrow(&a,0,1)==DBT_INVALID && dbt_audio_input_borrow(&a,pcm,0)==DBT_INVALID);
    CHECK(dbt_audio_input_borrow(&a,pcm,257)==DBT_INVALID);
    CHECK(dbt_audio_input_borrow(&a,pcm,3)==DBT_OK && !dbt_audio_input_release(&a));
    CHECK(dbt_audio_input_cancel(&a)==3 && dbt_audio_input_release(&a) && !dbt_audio_input_frames(&a));
    CHECK(dbt_audio_input_borrow(&a,pcm,1)==DBT_OK && dbt_audio_input_release(&a) && a.input_carry_valid);
    memset(pcm,0xa5,sizeof(pcm));CHECK(dbt_audio_input_cancel(&a)==1 && !a.input_carry_valid);
    CHECK(dbt_audio_input_borrow(&a,pcm,2)==DBT_OK);dbt_audio_close(&a);
    CHECK(!a.input_view && !m.live && !m.bytes && !dbt_audio_input_release(&a));
    CHECK(dbt_audio_open(&a,cfg)==DBT_OK && !a.borrowed_input);
    CHECK(dbt_audio_input_borrow(&a,pcm,1)==DBT_INVALID && !dbt_audio_input_release(&a));dbt_audio_close(&a);
    CHECK(!m.live && !m.bytes);return 0;
}
int main(void){
    CHECK(!boundaries());const uint32_t rates[]={48000,96000,192000};const int32_t adjustments[]={0,2147483,-2147483};
    unsigned cases=0;
    for(unsigned mode=0;mode<2;++mode)
    for(unsigned r=0;r<3;++r)for(unsigned w=2;w<=4;++w)for(unsigned ch=1;ch<=2;++ch)for(unsigned p=0;p<3;++p){
        CHECK(!compare_stream(rates[r],w,ch,adjustments[p],mode!=0));++cases;
    }
    printf("PASS %u borrowed/copied stream pairs, poisoned retired views, +/-500ppm, formats, wrap, controls, OOM and cancellation\n",cases);return 0;
}
