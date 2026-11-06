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
#ifdef DBT_TEST_PREPARED_DSP
extern dbt_result_t fast_dbt_audio_render_prepare(dbt_audio_t *,uint32_t,dbt_audio_render_job_t *);
extern dbt_result_t fast_dbt_audio_render_finish(dbt_audio_t *,const dbt_audio_render_job_t *,bool,uint32_t *);
static unsigned prepared_jobs;
#endif
static uint64_t copied_bytes,compared_frames;
void *unity_test_copy(void *to,const void *from,size_t count){copied_bytes+=count;return memcpy(to,from,count);}

#ifdef DBT_TEST_DSP_LANE
#include "dbt_dsp_job.h"
#ifdef _WIN32
#include <windows.h>
static HANDLE lane_start,lane_done,lane_thread;
static _Atomic bool lane_stop;
static _Atomic unsigned lane_errors;
#endif
static dbt_dsp_job_lane_t test_lane;
static uint32_t lane_clock=UINT32_MAX-123u;
static uint32_t test_clock(void *context){(void)context;return ++lane_clock;}
#ifdef _WIN32
static DWORD WINAPI lane_worker(LPVOID context){
    (void)context;
    for(unsigned jobs=0;jobs<100000u;++jobs){
        if(WaitForSingleObject(lane_start,10000)!=WAIT_OBJECT_0){atomic_store(&lane_errors,1);return 1;}
        if(atomic_load_explicit(&lane_stop,memory_order_acquire))return 0;
        if(!dbt_dsp_job_work(&test_lane,(dbt_dsp_job_clock_t){NULL,test_clock,test_clock}))atomic_store(&lane_errors,2);
        SetEvent(lane_done);
    }
    atomic_store(&lane_errors,3);return 1;
}
#endif
static int lane_initialize(void){
    dbt_dsp_job_init(&test_lane);
#ifdef _WIN32
    lane_start=CreateEventA(NULL,FALSE,FALSE,NULL);lane_done=CreateEventA(NULL,FALSE,FALSE,NULL);
    if(!lane_start || !lane_done)return 1;
    lane_thread=CreateThread(NULL,0,lane_worker,NULL,0,NULL);if(!lane_thread)return 1;
#endif
    return 0;
}
static int lane_begin(const dbt_audio_render_job_t *job){
    STEP_CHECK(dbt_dsp_job_state(&test_lane)==DBT_DSP_JOB_IDLE);
    STEP_CHECK(dbt_dsp_job_submit(&test_lane,job)==DBT_OK);
    STEP_CHECK(dbt_dsp_job_submit(&test_lane,job)==DBT_BUSY);
    const dbt_audio_render_job_t *not_ready=(const dbt_audio_render_job_t *)(uintptr_t)1;
    STEP_CHECK(dbt_dsp_job_poll(&test_lane,&not_ready,NULL,NULL,NULL)==DBT_PENDING && !not_ready);
    STEP_CHECK(dbt_dsp_job_retire(&test_lane,job->token)==DBT_INVALID);
#ifdef _WIN32
    STEP_CHECK(SetEvent(lane_start));
#else
    STEP_CHECK(dbt_dsp_job_work(&test_lane,(dbt_dsp_job_clock_t){NULL,test_clock,test_clock}));
#endif
    return 0;
}
static int lane_wait(const dbt_audio_render_job_t *job){
#ifdef _WIN32
    STEP_CHECK(WaitForSingleObject(lane_done,2000)==WAIT_OBJECT_0 && !atomic_load(&lane_errors));
#endif
    const dbt_audio_render_job_t *ready=NULL;uint32_t cycles=0,start=0,end=0;
    STEP_CHECK(dbt_dsp_job_poll(&test_lane,&ready,&cycles,&start,&end)==DBT_OK && ready && ready->token==job->token);
    STEP_CHECK(cycles==2u && end-start==2u);
    STEP_CHECK(!dbt_dsp_job_work(&test_lane,(dbt_dsp_job_clock_t){0}));
    STEP_CHECK(dbt_dsp_job_retire(&test_lane,job->token==UINT32_MAX?1u:job->token+1u)==DBT_INVALID);
    return 0;
}
static int lane_shutdown(void){
    STEP_CHECK(dbt_dsp_job_state(&test_lane)==DBT_DSP_JOB_IDLE);
#ifdef _WIN32
    atomic_store_explicit(&lane_stop,true,memory_order_release);STEP_CHECK(SetEvent(lane_start));
    STEP_CHECK(WaitForSingleObject(lane_thread,2000)==WAIT_OBJECT_0 && !atomic_load(&lane_errors));
    CloseHandle(lane_thread);CloseHandle(lane_start);CloseHandle(lane_done);
#endif
    return 0;
}
#endif

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
    unsigned made=0,reference=0;
#ifdef DBT_TEST_PREPARED_DSP
    if(budget>128)budget=128;
    dbt_audio_render_job_t job;
    uint32_t old_in=atomic_load(&a->input.read),old_out=atomic_load(&a->output.written);
    dbt_result_t result=fast_dbt_audio_render_prepare(a,budget,&job);
    if(result==DBT_OK){
        STEP_CHECK(eligible && dbt_audio_render_inflight(a));
        STEP_CHECK(atomic_load(&a->input.read)==old_in && atomic_load(&a->output.written)==old_out);
        uint8_t *input=a->input.bytes,*output=a->output.bytes;
        dbt_audio_close(a);STEP_CHECK(a->opened && a->input.bytes==input && a->output.bytes==output);
        STEP_CHECK(dbt_audio_open(a,a->config)==DBT_BUSY && dbt_audio_render(a,budget)==0);
        void (*old_mix)(void *,int32_t *,uint32_t)=a->mix;void *old_context=a->mix_context;
        dbt_audio_set_mix(a,NULL,NULL);STEP_CHECK(a->mix==old_mix && a->mix_context==old_context);
        bool was_draining=a->draining_input;dbt_audio_begin_drain(a);STEP_CHECK(a->draining_input==was_draining);
        dbt_audio_render_job_t stale=job;stale.token=job.token==UINT32_MAX?1u:job.token+1u;
        STEP_CHECK(fast_dbt_audio_render_finish(a,&stale,true,&made)==DBT_INVALID && !made && dbt_audio_render_inflight(a));
        reference=dbt_audio_render(b,job.frames);
        if(++prepared_jobs%5u==0){
            int8_t gain=(int8_t)(prepared_jobs%25u)-12;
            STEP_CHECK(dbt_audio_controls(a,gain,-gain,gain,91,13)==DBT_OK && dbt_audio_controls(b,gain,-gain,gain,91,13)==DBT_OK);
        }
#ifdef DBT_TEST_DSP_LANE
        STEP_CHECK(!lane_begin(&job));
        /* The worker may now be actively mutating DSP/PCM. Close/open must
         * still retain the reservation and buffers without touching DSP. */
        dbt_audio_close(a);STEP_CHECK(a->opened && a->input.bytes==input && a->output.bytes==output);
        STEP_CHECK(dbt_audio_open(a,a->config)==DBT_BUSY);
        STEP_CHECK(!lane_wait(&job));
#else
        STEP_CHECK(fixed_dsp_process_block(job.dsp,job.pcm,(uint16_t)job.frames));
#endif
        STEP_CHECK(atomic_load(&a->input.read)==old_in && atomic_load(&a->output.written)==old_out);
        STEP_CHECK(fast_dbt_audio_render_finish(a,&job,true,&made)==DBT_OK && !dbt_audio_render_inflight(a));
        uint32_t again=99;STEP_CHECK(fast_dbt_audio_render_finish(a,&job,true,&again)==DBT_INVALID && !again);
#ifdef DBT_TEST_DSP_LANE
        STEP_CHECK(dbt_dsp_job_retire(&test_lane,job.token)==DBT_OK);
#endif
    }else{
        STEP_CHECK(result==DBT_EMPTY || result==DBT_UNSUPPORTED);
        made=fast_dbt_audio_render(a,budget);reference=dbt_audio_render(b,budget);
    }
#else
    made=fast_dbt_audio_render(a,budget);reference=dbt_audio_render(b,budget);
#endif
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

#ifdef DBT_TEST_PREPARED_DSP
static int reservation_edges(void){
    dbt_audio_t a;memory_t memory={0};int32_t samples[256]={0};
    dbt_audio_config_t config={192000,512,1536,512,DBT_PCM_S32_LE,2,false};
    CHECK(dbt_audio_init(&a,(dbt_audio_memory_t){&memory,allocate,release})==DBT_OK);
    CHECK(dbt_audio_open(&a,config)==DBT_OK && dbt_audio_submit(&a,samples,128)==DBT_OK);
    a.render_serial=UINT32_MAX-1u;dbt_audio_render_job_t old,new_job;
    CHECK(fast_dbt_audio_render_prepare(&a,128,&old)==DBT_OK && old.token==UINT32_MAX);
    CHECK(fast_dbt_audio_render_finish(&a,&old,false,NULL)==DBT_INVALID && !dbt_audio_render_inflight(&a));
    CHECK(a.render_failed && !atomic_load(&a.input.read) && !atomic_load(&a.output.written));
    CHECK(!dbt_audio_render(&a,128) && fast_dbt_audio_render_prepare(&a,128,&new_job)==DBT_INVALID);
    dbt_audio_close(&a);CHECK(!memory.live);
    CHECK(dbt_audio_open(&a,config)==DBT_OK && dbt_audio_submit(&a,samples,128)==DBT_OK);
    CHECK(fast_dbt_audio_render_prepare(&a,128,&new_job)==DBT_OK && new_job.token==1);
    CHECK(fast_dbt_audio_render_finish(&a,&old,true,NULL)==DBT_INVALID && dbt_audio_render_inflight(&a));
    CHECK(fixed_dsp_process_block(new_job.dsp,new_job.pcm,(uint16_t)new_job.frames));
    uint32_t made=0;CHECK(fast_dbt_audio_render_finish(&a,&new_job,true,&made)==DBT_OK && made==128);
    int32_t output[256];CHECK(dbt_audio_take(&a,output,128)==128);
    CHECK(dbt_audio_submit(&a,samples,128)==DBT_OK && fast_dbt_audio_render_prepare(&a,128,&new_job)==DBT_OK);
    new_job.frames++;CHECK(fast_dbt_audio_render_finish(&a,&new_job,true,NULL)==DBT_INVALID && a.render_failed && !dbt_audio_render_inflight(&a));
    CHECK(atomic_load(&a.input.read)==128 && atomic_load(&a.output.written)==128);
    dbt_audio_close(&a);CHECK(!memory.live && !memory.bytes);return 0;
}
#endif

int main(void){
#ifdef DBT_TEST_DSP_LANE
    CHECK(!lane_initialize());
#endif
#ifdef DBT_TEST_PREPARED_DSP
    CHECK(!reservation_edges());
#endif
    const uint32_t rates[]={44100,48000,96000,192000};unsigned cases=0;
    for(unsigned r=0;r<4;++r)for(unsigned width=2;width<=4;++width)for(unsigned channels=1;channels<=2;++channels)
    for(unsigned mode=0;mode<4;++mode)for(unsigned mixing=0;mixing<2;++mixing){CHECK(!stream(rates[r],width,channels,mode,mixing!=0));++cases;}
    CHECK(copied_bytes>0);
#ifdef DBT_TEST_DSP_LANE
    CHECK(!lane_shutdown());
#endif
#ifdef DBT_TEST_PREPARED_DSP
    CHECK(prepared_jobs>0);printf("prepared DSP jobs=%u\n",prepared_jobs);
#endif
    printf("PASS %u unity/general stream pairs, %llu exact frames, %llu fast-copy bytes; DSP state, mixer, formats, phase fallbacks, page/ring/counter wraps\n",cases,(unsigned long long)compared_frames,(unsigned long long)copied_bytes);return 0;
}
