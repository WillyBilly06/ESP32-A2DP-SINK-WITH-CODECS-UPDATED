#include "dbt_audio.h"
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
#define STEP_CHECK(x) do{if(!(x)){printf("FAIL step line %u\n",(unsigned)__LINE__);return -1;}}while(0)
#define CHECK(x) do{if(!(x)){printf("FAIL line %u\n",(unsigned)__LINE__);return 1;}}while(0)
extern uint32_t fast_dbt_audio_render(dbt_audio_t *,uint32_t);
static uint64_t copied_bytes,compared_frames;
void *unity_test_copy(void *to,const void *from,size_t count){copied_bytes+=count;return memcpy(to,from,count);}
typedef struct {unsigned live;uint32_t bytes;} memory_t;
static uint8_t *allocate(void *context,uint32_t bytes){
    memory_t *m=context;uint64_t *p=malloc((size_t)bytes+8);if(!p)return NULL;
    *p=bytes;++m->live;m->bytes+=bytes;return (uint8_t *)(p+1);
}
static void release(void *context,uint8_t *bytes){
    memory_t *m=context;uint64_t *p=(uint64_t *)bytes-1;--m->live;m->bytes-=(uint32_t)*p;free(p);
}
static uint32_t rng=0x61c0ffeeu;
static uint32_t random32(void){rng^=rng<<13;rng^=rng>>17;rng^=rng<<5;return rng;}
static void mix(void *context,int32_t *pcm,uint32_t frames){
    uint32_t *index=context;for(uint32_t i=0;i<frames;++i){uint32_t bits=(++*index&0xffffu)<<8;pcm[2*i]=(int32_t)((uint32_t)pcm[2*i]^bits);pcm[2*i+1]=(int32_t)((uint32_t)pcm[2*i+1]^bits);}
}
static int step(dbt_audio_t *a,dbt_audio_t *b,unsigned budget){
    int32_t actual[600],expected[600];uint64_t before=copied_bytes;
    bool eligible=!a->config.adaptive_clock && !a->fraction && !a->step_adjust && !a->drain_remaining &&
        a->config.format==DBT_PCM_S32_LE && a->config.channels==2;
    unsigned made=fast_dbt_audio_render(a,budget),reference=dbt_audio_render(b,budget);
    STEP_CHECK(made==reference && made<=300);
    STEP_CHECK(dbt_audio_input_frames(a)==dbt_audio_input_frames(b));
    STEP_CHECK(dbt_audio_take(a,actual,made)==made && dbt_audio_take(b,expected,made)==made);
    STEP_CHECK(!memcmp(actual,expected,made*8u));
    STEP_CHECK(!memcmp(&a->dsp,&b->dsp,sizeof(a->dsp)) && !memcmp(&a->resampler,&b->resampler,sizeof(a->resampler)));
    STEP_CHECK(a->fraction==b->fraction && a->drain_remaining==b->drain_remaining && a->output_read_slot==b->output_read_slot && a->output_write_slot==b->output_write_slot);
    STEP_CHECK(atomic_load(&a->submitted_frames)==atomic_load(&b->submitted_frames) && atomic_load(&a->rendered_frames)==atomic_load(&b->rendered_frames));
    STEP_CHECK(copied_bytes-before==(eligible?(uint64_t)made*8u:0));compared_frames+=made;return (int)made;
}
static int stream(uint32_t rate,unsigned width,unsigned channels,unsigned mode,bool mixing){
    dbt_audio_t a,b;memory_t ma={0},mb={0};uint32_t mix_a=0,mix_b=0;
    unsigned capacity=mixing?6656u:513u;
    dbt_audio_config_t config={rate,512,capacity,128,(dbt_pcm_format_t)width,(uint8_t)channels,mode==2};
    CHECK(dbt_audio_init(&a,(dbt_audio_memory_t){&ma,allocate,release})==DBT_OK);
    CHECK(dbt_audio_init(&b,(dbt_audio_memory_t){&mb,allocate,release})==DBT_OK);
    CHECK(dbt_audio_open(&a,config)==DBT_OK && dbt_audio_open(&b,config)==DBT_OK);
    a.fraction=b.fraction=mode==1?1u:0u;a.step_adjust=b.step_adjust=mode==3?2147483:0;
    atomic_store(&a.input.read,UINT32_MAX-63);atomic_store(&a.input.written,UINT32_MAX-63);
    atomic_store(&b.input.read,UINT32_MAX-63);atomic_store(&b.input.written,UINT32_MAX-63);
    atomic_store(&a.output.read,UINT32_MAX-63);atomic_store(&a.output.written,UINT32_MAX-63);
    atomic_store(&b.output.read,UINT32_MAX-63);atomic_store(&b.output.written,UINT32_MAX-63);
    a.output_read_slot=a.output_write_slot=b.output_read_slot=b.output_write_slot=capacity-63;
    if(mixing){dbt_audio_set_mix(&a,mix,&mix_a);dbt_audio_set_mix(&b,mix,&mix_b);}
    CHECK(dbt_audio_controls(&a,8,8,11,127,12)==DBT_OK && dbt_audio_controls(&b,8,8,11,127,12)==DBT_OK);
    uint8_t storage[256*8+8];unsigned total=0,round=0;
    while(total<4096){
        unsigned frames=round%9==0?1u:1u+random32()%256u;if(frames>4096-total)frames=4096-total;
        UNPOISON(storage,sizeof(storage));uint8_t *pcm=storage+(round&1u);
        for(unsigned i=0;i<frames*channels;++i){uint32_t value=i%3?random32():i&1?0x80000000u:0x7fffffffu;for(unsigned j=0;j<width;++j)pcm[i*width+j]=(uint8_t)(value>>(8*j));}
        CHECK(dbt_audio_submit(&a,pcm,frames)==DBT_OK && dbt_audio_submit(&b,pcm,frames)==DBT_OK);
        POISON(storage,sizeof(storage));
        for(unsigned n=0;n<512;++n){unsigned budget=n%9==0?0u:1u+random32()%300u;int made=step(&a,&b,budget);CHECK(made>=0);if(!made && budget)break;CHECK(n<511);}
        total+=frames;++round;
        if(round%7==0){int8_t gain=(int8_t)(round%25)-12;CHECK(dbt_audio_controls(&a,gain,-gain,gain,64,(uint8_t)(round&15u))==DBT_OK && dbt_audio_controls(&b,gain,-gain,gain,64,(uint8_t)(round&15u))==DBT_OK);}
    }
    dbt_audio_begin_drain(&a);dbt_audio_begin_drain(&b);
    for(unsigned i=0;i<8;++i){int made=step(&a,&b,300);CHECK(made>=0);if(!made)break;}
    CHECK(mix_a==mix_b);UNPOISON(storage,sizeof(storage));dbt_audio_close(&a);dbt_audio_close(&b);
    CHECK(!ma.live && !mb.live && !ma.bytes && !mb.bytes);return 0;
}
int main(void){
    const uint32_t rates[]={44100,48000,96000,192000};unsigned cases=0;
    for(unsigned r=0;r<4;++r)for(unsigned width=2;width<=4;++width)for(unsigned channels=1;channels<=2;++channels)
    for(unsigned mode=0;mode<4;++mode)for(unsigned mixing=0;mixing<2;++mixing){CHECK(!stream(rates[r],width,channels,mode,mixing!=0));++cases;}
    CHECK(copied_bytes>0);printf("PASS %u unity/general stream pairs, %llu exact frames, %llu fast-copy bytes; DSP state, mixer, formats, phase fallbacks, page/ring/counter wraps\n",cases,(unsigned long long)compared_frames,(unsigned long long)copied_bytes);return 0;
}
