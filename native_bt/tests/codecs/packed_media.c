/* Actual packed worker: bounded record ownership, not a codec/RF benchmark. */
#include "dbt_codec.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef _WIN32
#include <windows.h>
#endif
#define CHECK(x) do{if(!(x)){fprintf(stderr,"FAIL packed line %u\n",(unsigned)__LINE__);return 1;}}while(0)
typedef struct {const uint8_t *input;uint16_t size;uint32_t ordinal;unsigned left;int16_t pcm[600];} model_t;
static dbt_codec_worker_t w;
static unsigned live,allocations,releases,wakes,decoded,completed,pcm_calls;
static bool packed=true,deferred,busy;
static uint8_t selected_depth=8;
static _Atomic unsigned errors;
static uint8_t pattern(uint32_t ordinal,unsigned i){return (uint8_t)(ordinal*37u+i*13u+(i>>5));}
static uint8_t *allocate(void *ctx,uint32_t n){(void)ctx;uint8_t *p=malloc(n+8u);if(!p)return NULL;memcpy(p,&n,4);live+=n;++allocations;return p+8;}
static void release(void *ctx,uint8_t *p){(void)ctx;unsigned n;memcpy(&n,p-8,4);live-=n;++releases;free(p-8);}
static void wake(void *ctx){(void)ctx;++wakes;}
static dbt_result_t pcm(void *ctx,const void *p,uint16_t n,const dbt_codec_config_t *c,uint32_t timestamp){
    (void)ctx;(void)timestamp;if(!p||!n||n>256||c->sample_bytes!=2)atomic_fetch_add(&errors,1);++pcm_calls;return busy?DBT_BUSY:DBT_OK;
}
static dbt_result_t config(const uint8_t *p,uint16_t n,dbt_codec_config_t *c){
    if(n!=6||p[0]||p[1])return DBT_UNSUPPORTED;
    *c=(dbt_codec_config_t){0};c->rate=48000;c->channels=2;c->sample_bytes=2;c->samples=300;c->cie_size=6;memcpy(c->cie,p,6);return DBT_OK;
}
static dbt_result_t open_codec(dbt_codec_decoder_t *d){d->state=dbt_codec_allocate(d,sizeof(model_t));if(!d->state)return DBT_NO_SPACE;memset(d->state,0,sizeof(model_t));return DBT_OK;}
static void close_codec(dbt_codec_decoder_t *d){dbt_codec_release(d,d->state);d->state=NULL;}
static dbt_result_t reset_codec(dbt_codec_decoder_t *d){model_t *m=d->state;m->input=NULL;m->left=0;return DBT_OK;}
static bool verify(model_t *m){
    if((uintptr_t)m->input&7u)return false;
    for(unsigned i=0;i<m->size;++i)if(m->input[i]!=pattern(m->ordinal,i))return false;
    return true;
}
static dbt_result_t resume_codec(dbt_codec_decoder_t *d,uint32_t now){
    (void)now;model_t *m=d->state;
    if(!m->input||!verify(m)){atomic_fetch_add(&errors,1);return DBT_INVALID;}
    if(--m->left)return DBT_PROGRESS;
    m->input=NULL;++completed;return DBT_OK;
}
static dbt_result_t media_codec(dbt_codec_decoder_t *d,const dbt_avdtp_media_t *p,uint32_t now){
    (void)now;model_t *m=d->state;
    if(m->input||p->sequence!=(uint16_t)decoded||p->timestamp!=(uint32_t)decoded*960u||p->ssrc!=7u||p->marker!=(bool)(decoded&1u)){
        atomic_fetch_add(&errors,1);return DBT_INVALID;
    }
    m->input=p->payload;m->size=p->size;m->ordinal=decoded++;m->left=deferred?1u:3u;
    if(!verify(m)){atomic_fetch_add(&errors,1);return DBT_INVALID;}
    if(deferred)return d->hooks.pcm(d->hooks.context,m->pcm,300,&d->config,p->timestamp);
    return DBT_PROGRESS;
}
static const dbt_codec_ops_t codec={"packed ownership",config,open_codec,close_codec,reset_codec,media_codec,NULL,resume_codec,NULL};
static const dbt_codec_ops_t *const codecs[]={&codec};
static dbt_codec_queue_t policy(void *ctx,const dbt_codec_config_t *c){(void)ctx;(void)c;return (dbt_codec_queue_t){(uint8_t)(selected_depth|(packed?DBT_CODEC_QUEUE_PACKED:0)),(uint8_t)(selected_depth/2u)};}
static int control(dbt_sink_operation_t op){
    const uint8_t caps[]={1,0,7,6,0,0,0x21,0x15,2,53};dbt_sink_job_t job={1,1,op,1,caps,sizeof caps};
    CHECK(dbt_codec_worker_submit(&w,&job)==DBT_OK);dbt_codec_worker_step(&w,1,0);dbt_codec_receipt_t r;
    CHECK(dbt_codec_worker_receipt(&w,&r)==DBT_OK&&r.result==DBT_SINK_WORK_READY);return 0;
}
static int setup(bool defer){
    CHECK(!live);allocations=releases=wakes=decoded=completed=pcm_calls=0;atomic_store(&errors,0);deferred=defer;busy=false;packed=true;selected_depth=8;
    dbt_codec_worker_hooks_t h={0};h.decoder=(dbt_codec_hooks_t){NULL,allocate,release,pcm};h.codecs=codecs;h.codec_count=1;h.queue_policy=policy;h.deferred_pcm=defer;h.wake_owner=wake;
    CHECK(dbt_codec_worker_init(&w,h,1800,8)==DBT_OK);CHECK(!control(DBT_SINK_CONFIGURE)&&!control(DBT_SINK_START));
    CHECK(w.packed_media&&w.depth==8&&w.stride==1816);CHECK(live==sizeof(model_t)+8u+14536u);return 0;
}
static dbt_result_t packet(unsigned ordinal,uint16_t size){
    uint8_t bytes[1801];for(unsigned i=0;i<size&&i<sizeof bytes;++i)bytes[i]=pattern(ordinal,i);
    dbt_avdtp_media_t p={bytes,ordinal*960u,7,size,(uint16_t)ordinal,96,(bool)(ordinal&1u)};
    dbt_result_t r=dbt_codec_worker_media(&w,&p);memset(bytes,0,sizeof bytes);return r;
}
static int drain(unsigned count){
    for(unsigned i=0;i<count*12u+12u&&atomic_load(&w.media_read)!=atomic_load(&w.media_written);++i)CHECK(dbt_codec_worker_step(&w,1,1)<=1);
    CHECK(atomic_load(&w.media_read)==atomic_load(&w.media_written));CHECK(completed==count&&!atomic_load(&errors));
    CHECK(!atomic_load(&w.decode_rejects)&&!atomic_load(&w.faults));return 0;
}
static int drop(void){CHECK(!control(DBT_SINK_DROP)&&!live&&allocations==releases&&!atomic_load(&errors));return 0;}
static int capacity(void){
    CHECK(!setup(false));unsigned before=allocations;
    for(unsigned i=0;i<24;++i)CHECK(packet(i,566)==DBT_OK);
    CHECK(packet(24,566)==DBT_NO_SPACE&&atomic_load(&w.queue_high_water)==24);
    CHECK(!dbt_codec_worker_free_slots(&w)&&allocations==before);CHECK(!drain(24));CHECK(!drop());
    CHECK(!setup(false));for(unsigned i=0;i<8;++i)CHECK(packet(i,1800)==DBT_OK);
    CHECK(packet(8,1800)==DBT_NO_SPACE&&packet(8,1801)==DBT_INVALID);
    CHECK(atomic_load(&w.input_oversize_rejects)==1&&allocations==2);CHECK(!drain(8)&&!drop());return 0;
}
static int mixed_and_wrap(void){
    CHECK(!setup(false));const uint16_t sizes[]={1,8,17,566,1000,1799,1800,24,1023,1500};unsigned accepted=0;
    atomic_store(&w.media_written,UINT32_MAX-2u);atomic_store(&w.media_read,UINT32_MAX-2u);
    atomic_store(&w.media_page_written,UINT32_MAX-1u);atomic_store(&w.media_page_read,UINT32_MAX-1u);
    for(unsigned i=0;i<30000;++i){
        dbt_result_t r=packet(accepted,sizes[(i*7u)%10u]);CHECK(r==DBT_OK||r==DBT_NO_SPACE);if(r==DBT_OK)++accepted;
        CHECK(dbt_codec_worker_step(&w,1,1)<=1&&dbt_codec_worker_free_slots(&w)<=8);
    }
    CHECK(accepted>1000&&!drain(accepted)&&allocations==2);CHECK(!drop());return 0;
}
static int borrowed_and_lifecycle(void){
    const dbt_sink_operation_t stops[]={DBT_SINK_SUSPEND,DBT_SINK_CLOSE,DBT_SINK_ABORT,DBT_SINK_DROP};
    for(unsigned op=0;op<4;++op){
        CHECK(!setup(true)&&packet(0,566)==DBT_OK&&dbt_codec_worker_step(&w,1,1)==1&&w.pending_pcm);
        busy=true;CHECK(!dbt_codec_worker_step(&w,2,1));
        for(unsigned i=1;i<24;++i)CHECK(packet(i,566)==DBT_OK);
        CHECK(verify(w.decoder.state)&&w.pending_pcm&&allocations==2);
        if(op==0){busy=false;CHECK(!drain(24));}
        CHECK(!control(stops[op]));
        if(op==0){decoded=completed=0;CHECK(!control(DBT_SINK_START)&&packet(0,1800)==DBT_OK&&!drain(1));CHECK(!drop());}
        else CHECK(!live&&allocations==releases);
    }
    CHECK(!setup(false));packed=false;selected_depth=4;CHECK(!control(DBT_SINK_RECONFIGURE)&&!w.packed_media&&w.depth==4);
    CHECK(!control(DBT_SINK_START));for(unsigned i=0;i<4;++i)CHECK(packet(i,566)==DBT_OK);
    CHECK(packet(4,566)==DBT_NO_SPACE&&!drain(4)&&!drop());return 0;
}
#ifdef _WIN32
static _Atomic bool producer_done;
static DWORD WINAPI producer(void *p){
    (void)p;unsigned accepted=0;const uint16_t lengths[]={566,1800,1,1023,8,1799};
    for(unsigned attempt=0;attempt<20000000u&&accepted<20000u;++attempt){
        dbt_result_t r=packet(accepted,lengths[accepted%6u]);if(r==DBT_OK)++accepted;else if(r!=DBT_NO_SPACE){atomic_fetch_add(&errors,1);break;}
        if(attempt%32u==0)SwitchToThread();
    }
    if(accepted!=20000u)atomic_fetch_add(&errors,1);atomic_store_explicit(&producer_done,true,memory_order_release);return 0;
}
static DWORD WINAPI consumer(void *p){
    (void)p;for(unsigned i=0;i<20000000u;++i){
        dbt_codec_worker_step(&w,1,1);
        if(atomic_load_explicit(&producer_done,memory_order_acquire)&&atomic_load(&w.media_read)==atomic_load(&w.media_written))return 0;
        if(i%32u==0)SwitchToThread();
    }atomic_fetch_add(&errors,1);return 0;
}
static int concurrency(void){
    CHECK(!setup(false));atomic_store(&producer_done,false);HANDLE h[2]={CreateThread(NULL,0,producer,NULL,0,NULL),CreateThread(NULL,0,consumer,NULL,0,NULL)};
    CHECK(h[0]&&h[1]);CHECK(WaitForMultipleObjects(2,h,TRUE,30000)==WAIT_OBJECT_0);CloseHandle(h[0]);CloseHandle(h[1]);
    CHECK(decoded==20000u&&completed==20000u&&!atomic_load(&errors)&&allocations==2);CHECK(!drop());return 0;
}
#endif
int test_codec_async_pcm_completion_and_exact_order(void);
int test_codec_async_pcm_cancel_before_codec_lifecycle(void);
int test_codec_async_pcm_deadline_and_stale_retirement(void);
int test_codec_async_pcm_failed_completion_and_busy_begin(void);
int main(void){
    int (*const loans[])(void)={test_codec_async_pcm_completion_and_exact_order,
        test_codec_async_pcm_cancel_before_codec_lifecycle,
        test_codec_async_pcm_deadline_and_stale_retirement,
        test_codec_async_pcm_failed_completion_and_busy_begin};
    for(unsigned i=0;i<4;++i){int line=loans[i]();if(line){fprintf(stderr,"FAIL packed PCM loan group %u line %d\n",i,line);return 1;}}
    CHECK(!capacity()&&!mixed_and_wrap()&&!borrowed_and_lifecycle());
#ifdef _WIN32
    CHECK(!concurrency());
#endif
    puts("PASS actual packed worker:24x566 in unchanged14528 pool,8x1800 untruncated, mixed/wrap, borrowed PCM/control teardown, fixed-mode restore,4 packed PCM-loan groups,20000 concurrent packets");return 0;
}
