#include "dbt_codec.h"
#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif
#define CHECK(x) do {if(!(x))return __LINE__;}while(0)
static void zero(void *v,unsigned n){uint8_t *p=v;for(unsigned i=0;i<n;++i)p[i]=0;}
typedef struct {
    int16_t samples[600*2];
    const uint8_t *batch;
    uint32_t timestamp;
    unsigned cursor,count;
} model_t;
typedef struct {
    dbt_codec_worker_t worker;
    _Alignas(8) uint8_t pool[4][8192];
    uint32_t sizes[4],live,allocations,releases,errors,decodes,resumes,resets,pcm_calls,pcm_frames;
    bool used[4],busy,pcm_failure,reset_failure;
    int16_t output[4096*2];uint32_t timestamps[4096];
    uint32_t ticket,next_ticket,begins,polls,cancels;
    bool begin_busy,async_done,async_cancel,async_start,output_active;
    dbt_result_t async_result;
} fixture_t;
static fixture_t f;
static uint8_t *allocate(void *context,uint32_t bytes){
    fixture_t *p=context;++p->allocations;
    if(bytes>8192)return 0;
    for(unsigned i=0;i<4;++i)if(!p->used[i]){p->used[i]=true;p->sizes[i]=bytes;p->live+=bytes;return p->pool[i];}
    return 0;
}
static void release(void *context,uint8_t *bytes){
    fixture_t *p=context;
    for(unsigned i=0;i<4;++i)if(bytes==p->pool[i]){
        if(!p->used[i]){++p->errors;return;}
        p->used[i]=false;p->live-=p->sizes[i];++p->releases;return;
    }
    ++p->errors;
}
static dbt_result_t output(void *context,const void *samples,uint16_t frames,const dbt_codec_config_t *config,uint32_t timestamp){
    fixture_t *p=context;++p->pcm_calls;
    if(p->pcm_failure)return DBT_INVALID;
    if(p->busy)return DBT_BUSY;
    if(!samples || !frames || frames>DBT_CODEC_PCM_CHUNK_FRAMES || config->channels!=2 || config->sample_bytes!=2 || p->pcm_frames+frames>4096){++p->errors;return DBT_INVALID;}
    const int16_t *in=samples;
    for(unsigned i=0;i<frames;++i){
        unsigned at=p->pcm_frames+i;p->output[2*at]=in[2*i];p->output[2*at+1]=in[2*i+1];p->timestamps[at]=timestamp+i;
    }
    p->pcm_frames+=frames;return DBT_OK;
}
static dbt_result_t configure(const uint8_t *cie,uint16_t size,dbt_codec_config_t *c){
    if(!cie || size!=6 || cie[0] || cie[1])return DBT_UNSUPPORTED;
    *c=(dbt_codec_config_t){0};c->rate=48000;c->samples=600;c->channels=2;c->sample_bytes=2;c->cie_size=6;
    for(unsigned i=0;i<6;++i)c->cie[i]=cie[i];return DBT_OK;
}
static dbt_result_t open_model(dbt_codec_decoder_t *d){
    d->state=dbt_codec_allocate(d,sizeof(model_t));if(!d->state)return DBT_NO_SPACE;
    zero(d->state,sizeof(model_t));return DBT_OK;
}
static void close_model(dbt_codec_decoder_t *d){if(d->state){dbt_codec_release(d,d->state);d->state=0;}}
static dbt_result_t reset_model(dbt_codec_decoder_t *d){
    ++f.resets;if(f.reset_failure)return DBT_INVALID;
    if(!d->state)return DBT_INVALID;
    ((model_t *)d->state)->batch=0;return DBT_OK;
}
static dbt_result_t resume_model(dbt_codec_decoder_t *d,uint32_t now){
    (void)now;++f.resumes;model_t *s=d->state;if(!s)return DBT_INVALID;
    if(!s->batch)return DBT_OK;
    while(s->cursor<s->count){
        unsigned frame=s->cursor;
        for(unsigned i=0;i<600;++i)for(unsigned ch=0;ch<2;++ch)
            s->samples[2*i+ch]=(int16_t)(100u*s->batch[0]+frame*600u+i+ch);
        ++f.decodes;
        dbt_result_t r=d->hooks.pcm(d->hooks.context,s->samples,600,&d->config,s->timestamp+frame*600u);
        if(r!=DBT_OK && r!=DBT_PENDING){s->batch=0;return r;}
        ++s->cursor;++d->frames;if(r==DBT_PENDING)return r;
    }
    s->batch=0;return DBT_OK;
}
static dbt_result_t media_model(dbt_codec_decoder_t *d,const dbt_avdtp_media_t *p,uint32_t now){
    model_t *s=d->state;if(!s || !p || p->size!=2 || !p->payload[1] || p->payload[1]>2 || s->batch)return DBT_INVALID;
    s->batch=p->payload;s->cursor=0;s->count=p->payload[1];s->timestamp=p->timestamp;return resume_model(d,now);
}
static dbt_result_t stream(void *context,bool start,const dbt_codec_config_t *config){(void)context;(void)start;(void)config;return DBT_OK;}
static const dbt_codec_ops_t codec={"deferred ownership model",configure,open_model,close_model,reset_model,media_model,0,resume_model,0};
static const dbt_codec_ops_t *const codecs[]={&codec};
static bool setup(void){
    zero(&f,sizeof(f));dbt_codec_worker_hooks_t hooks={{&f,allocate,release,output},codecs,1,0,0,stream,true,{0}};
    return dbt_codec_worker_init(&f.worker,hooks,64,2)==DBT_OK;
}
static dbt_result_t submit(dbt_sink_operation_t op){
    static const uint8_t config[]={1,0,7,6,0,0,0x21,0x15,2,53};
    dbt_sink_job_t job={1,1,op,1,config,sizeof(config)};return dbt_codec_worker_submit(&f.worker,&job);
}
static bool control(dbt_sink_operation_t op){
    if(submit(op)!=DBT_OK)return false;
    dbt_codec_worker_step(&f.worker,0,0);dbt_codec_receipt_t receipt;
    return dbt_codec_worker_receipt(&f.worker,&receipt)==DBT_OK && receipt.result==DBT_SINK_WORK_READY;
}
static bool start(void){return setup() && control(DBT_SINK_CONFIGURE) && control(DBT_SINK_START);}
static dbt_result_t packet(uint8_t *bytes,uint16_t sequence,uint32_t timestamp){
    dbt_avdtp_media_t p={bytes,timestamp,1,2,sequence,96,false};return dbt_codec_worker_media(&f.worker,&p);
}
EXPORT int test_codec_deferred_pcm_backpressure_and_exact_order(void){
    CHECK(start());uint8_t bytes[]={5,2};uint32_t ts=0xfffffff0u;
    CHECK(packet(bytes,65535,ts)==DBT_OK);bytes[0]=99;
    CHECK(dbt_codec_worker_step(&f.worker,0,1)==1 && f.decodes==1 && !f.pcm_calls && f.worker.pending_pcm);
    CHECK(!atomic_load(&f.worker.media_read) && dbt_codec_worker_runnable(&f.worker));
    f.busy=true;CHECK(!dbt_codec_worker_step(&f.worker,1,2) && f.pcm_calls==1 && !f.pcm_frames);
    uint32_t when;CHECK(!dbt_codec_worker_runnable(&f.worker) && dbt_codec_worker_deadline(&f.worker,&when) && when==200);
    uint8_t next[]={8,2};CHECK(packet(next,0,ts+1200u)==DBT_OK);
    CHECK(packet(next,1,ts+2400u)==DBT_NO_SPACE && atomic_load(&f.worker.media_drops)==1);
    next[0]=77;f.busy=false;
    for(unsigned i=0;i<40 && atomic_load(&f.worker.media_read)!=atomic_load(&f.worker.media_written);++i)
        CHECK(dbt_codec_worker_step(&f.worker,2+i,1)<=1);
    CHECK(f.pcm_frames==2400 && f.decodes==4 && atomic_load(&f.worker.packets)==2 && !f.worker.pending_pcm);
    CHECK(!dbt_codec_worker_runnable(&f.worker) && !dbt_codec_worker_deadline(&f.worker,&when));
    for(unsigned i=0;i<2400;++i){unsigned base=i<1200?500:800,at=i%1200;
        CHECK(f.output[2*i]==(int16_t)(base+at) && f.output[2*i+1]==(int16_t)(base+at+1));
        CHECK(f.timestamps[i]==ts+i);}
    CHECK(atomic_load(&f.worker.pcm_delivered_frames)==2400 && !atomic_load(&f.worker.resets));
    CHECK(control(DBT_SINK_DROP) && !f.live && f.allocations==f.releases && !f.errors);
    return 0;
}
EXPORT int test_codec_deferred_cancel_suspend_and_reconfigure(void){
    const dbt_sink_operation_t operations[]={DBT_SINK_CLOSE,DBT_SINK_ABORT,DBT_SINK_DROP,DBT_SINK_SUSPEND};
    for(unsigned n=0;n<4;++n){
        CHECK(start());uint8_t bytes[]={3,2};CHECK(packet(bytes,1,0)==DBT_OK);
        CHECK(dbt_codec_worker_step(&f.worker,0,2)==2 && f.pcm_frames==256);
        unsigned calls=f.pcm_calls;CHECK(control(operations[n]));
        CHECK(!f.worker.pending_pcm && !f.worker.media_active && !dbt_codec_worker_step(&f.worker,1,8) && f.pcm_calls==calls);
        if(operations[n]==DBT_SINK_SUSPEND){
            CHECK(control(DBT_SINK_RECONFIGURE) && control(DBT_SINK_START));
            f.pcm_frames=0;bytes[0]=9;bytes[1]=1;CHECK(packet(bytes,2,123)==DBT_OK);
            for(unsigned i=0;i<10 && atomic_load(&f.worker.media_read)!=atomic_load(&f.worker.media_written);++i)dbt_codec_worker_step(&f.worker,2+i,2);
            CHECK(f.pcm_frames==600 && f.output[0]==900 && f.timestamps[0]==123 && control(DBT_SINK_DROP));
        }
        CHECK(!f.live && !f.errors && f.allocations==f.releases);
    }
    return 0;
}
EXPORT int test_codec_deferred_deadline_and_failure_retirement(void){
    for(unsigned bad=0;bad<2;++bad){
        CHECK(start());uint8_t bytes[]={1,1};uint32_t began=0xfffffff0u;
        CHECK(packet(bytes,1,0)==DBT_OK && dbt_codec_worker_step(&f.worker,began,1)==1);
        unsigned resets=f.resets;f.busy=true;
        CHECK(!dbt_codec_worker_step(&f.worker,began+199u,2) && !atomic_load(&f.worker.faults));
        if(bad)f.pcm_failure=true;
        CHECK(dbt_codec_worker_step(&f.worker,began+200u,2)<=1 && atomic_load(&f.worker.faults)==1);
        CHECK(!f.worker.pending_pcm && !f.worker.media_active && f.resets==resets && atomic_load(&f.worker.media_drops)==1);
        CHECK(atomic_load(&f.worker.pcm_timeouts)==(bad?0u:1u));
        CHECK(!dbt_codec_worker_runnable(&f.worker) && packet(bytes,2,600)==DBT_BUSY);
        CHECK(control(DBT_SINK_DROP) && !f.live && !f.errors);
    }
    return 0;
}
static dbt_result_t async_begin(void *context,bool start,const dbt_codec_config_t *config,uint32_t *ticket){
    fixture_t *p=context;(void)config;*ticket=0;
    if(p->begin_busy || p->ticket)return DBT_BUSY;
    ++p->begins;p->ticket=++p->next_ticket;p->async_start=start;p->async_done=p->async_cancel=false;
    *ticket=p->ticket;return DBT_PENDING;
}
static dbt_result_t async_poll(void *context,uint32_t ticket){
    fixture_t *p=context;++p->polls;
    if(ticket!=p->ticket || !ticket)return DBT_INVALID;
    if(!p->async_done)return DBT_PENDING;
    dbt_result_t result=p->async_cancel?DBT_CANCELLED:p->async_result;
    if(result==DBT_OK)p->output_active=p->async_start;
    if(result==DBT_CANCELLED)p->output_active=false;
    p->ticket=0;return result;
}
static dbt_result_t async_cancel(void *context,uint32_t ticket){
    fixture_t *p=context;if(!ticket || ticket!=p->ticket)return DBT_INVALID;
    ++p->cancels;p->async_cancel=true;return DBT_PENDING;
}
static bool setup_async(void){
    zero(&f,sizeof(f));dbt_codec_worker_hooks_t hooks={{&f,allocate,release,output},codecs,1,0,0,0,true,{async_begin,async_poll,async_cancel}};
    return dbt_codec_worker_init(&f.worker,hooks,64,2)==DBT_OK && control(DBT_SINK_CONFIGURE);
}
static bool complete_async(dbt_sink_operation_t op,uint32_t now){
    if(submit(op)!=DBT_OK)return false;
    dbt_codec_worker_step(&f.worker,now,0);if(!f.ticket)return false;
    f.async_result=DBT_OK;f.async_done=true;dbt_codec_worker_step(&f.worker,now+1u,0);
    dbt_codec_receipt_t receipt;
    return dbt_codec_worker_receipt(&f.worker,&receipt)==DBT_OK && receipt.result==DBT_SINK_WORK_READY;
}
EXPORT int test_codec_async_stream_completion_and_media_fence(void){
    CHECK(setup_async() && submit(DBT_SINK_START)==DBT_OK);
    CHECK(!dbt_codec_worker_step(&f.worker,0,2) && f.begins==1 && !f.worker.streaming);
    dbt_codec_receipt_t receipt;CHECK(dbt_codec_worker_receipt(&f.worker,&receipt)==DBT_EMPTY);
    uint32_t when;CHECK(!dbt_codec_worker_runnable(&f.worker) && dbt_codec_worker_deadline(&f.worker,&when) && when==1500);
    uint8_t bytes[]={4,1};CHECK(packet(bytes,1,0)==DBT_BUSY);
    f.async_result=DBT_OK;f.async_done=true;CHECK(!dbt_codec_worker_step(&f.worker,10,0));
    CHECK(dbt_codec_worker_receipt(&f.worker,&receipt)==DBT_OK && receipt.result==DBT_SINK_WORK_READY && f.output_active);
    CHECK(packet(bytes,1,0)==DBT_OK && dbt_codec_worker_step(&f.worker,11,1)==1 && f.worker.pending_pcm);
    unsigned calls=f.pcm_calls;
    CHECK(submit(DBT_SINK_SUSPEND)==DBT_OK && !dbt_codec_worker_step(&f.worker,12,4) && f.pcm_calls==calls);
    CHECK(dbt_codec_worker_receipt(&f.worker,&receipt)==DBT_EMPTY && f.worker.pending_pcm);
    f.async_result=DBT_OK;f.async_done=true;dbt_codec_worker_step(&f.worker,13,4);
    CHECK(dbt_codec_worker_receipt(&f.worker,&receipt)==DBT_OK && receipt.result==DBT_SINK_WORK_READY && !f.worker.pending_pcm && !f.output_active);
    CHECK(complete_async(DBT_SINK_DROP,14) && !f.live && !f.errors);
    return 0;
}
EXPORT int test_codec_async_stream_cancel_before_drop(void){
    CHECK(setup_async() && submit(DBT_SINK_START)==DBT_OK);
    dbt_codec_worker_step(&f.worker,0,0);CHECK(f.ticket && f.begins==1);
    CHECK(submit(DBT_SINK_DROP)==DBT_OK);dbt_codec_worker_step(&f.worker,1,0);
    dbt_codec_receipt_t receipt;CHECK(f.cancels==1 && dbt_codec_worker_receipt(&f.worker,&receipt)==DBT_EMPTY && f.live);
    f.async_done=true;dbt_codec_worker_step(&f.worker,2,0);
    CHECK(dbt_codec_worker_receipt(&f.worker,&receipt)==DBT_OK && receipt.result==DBT_SINK_WORK_REJECTED);
    CHECK(f.begins==2 && f.ticket && !f.output_active && !f.worker.streaming && f.live);
    CHECK(dbt_codec_worker_receipt(&f.worker,&receipt)==DBT_EMPTY);
    f.async_result=DBT_OK;f.async_done=true;dbt_codec_worker_step(&f.worker,3,0);
    CHECK(dbt_codec_worker_receipt(&f.worker,&receipt)==DBT_OK && receipt.result==DBT_SINK_WORK_READY && !f.live && !f.errors);
    return 0;
}
EXPORT int test_codec_async_stream_deadline_and_receipt_reservation(void){
    CHECK(setup_async());f.begin_busy=true;CHECK(submit(DBT_SINK_START)==DBT_OK);
    dbt_codec_worker_step(&f.worker,0,0);CHECK(!f.ticket && !dbt_codec_worker_runnable(&f.worker));
    dbt_codec_worker_step(&f.worker,1500,0);CHECK(atomic_load(&f.worker.faults)==1);
    uint32_t when;CHECK(!dbt_codec_worker_deadline(&f.worker,&when) && !f.ticket && !f.begins);
    dbt_codec_receipt_t receipt;
    CHECK(dbt_codec_worker_receipt(&f.worker,&receipt)==DBT_OK && receipt.result==DBT_SINK_WORK_FATAL);
    f.begin_busy=false;CHECK(complete_async(DBT_SINK_DROP,1501) && !f.live);
    CHECK(setup_async() && submit(DBT_SINK_START)==DBT_OK);dbt_codec_worker_step(&f.worker,0,0);
    CHECK(f.ticket);dbt_codec_worker_step(&f.worker,1500,0);
    CHECK(f.cancels==1 && atomic_load(&f.worker.faults)==1 && f.live && !dbt_codec_worker_deadline(&f.worker,&when));
    f.async_done=true;dbt_codec_worker_step(&f.worker,1501,0);
    CHECK(dbt_codec_worker_receipt(&f.worker,&receipt)==DBT_OK && receipt.result==DBT_SINK_WORK_REJECTED);
    CHECK(complete_async(DBT_SINK_DROP,1502) && !f.live);
    CHECK(setup_async());
    for(unsigned i=0;i<4;++i)CHECK(submit(DBT_SINK_OPEN)==DBT_OK);
    dbt_codec_worker_step(&f.worker,0,0);CHECK(submit(DBT_SINK_START)==DBT_OK);
    dbt_codec_worker_step(&f.worker,1,0);CHECK(!f.begins);
    CHECK(dbt_codec_worker_receipt(&f.worker,&receipt)==DBT_OK);dbt_codec_worker_step(&f.worker,2,0);CHECK(f.begins==1);
    for(unsigned i=0;i<3;++i)CHECK(dbt_codec_worker_receipt(&f.worker,&receipt)==DBT_OK);
    f.async_result=DBT_OK;f.async_done=true;dbt_codec_worker_step(&f.worker,3,0);
    CHECK(dbt_codec_worker_receipt(&f.worker,&receipt)==DBT_OK && receipt.result==DBT_SINK_WORK_READY);
    CHECK(complete_async(DBT_SINK_DROP,4) && !f.live && !f.errors);
    return 0;
}
