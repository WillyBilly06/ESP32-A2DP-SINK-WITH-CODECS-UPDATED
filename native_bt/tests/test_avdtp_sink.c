#include "dbt_avdtp_sink.h"

#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif
#define CHECK(x) do { if(!(x))return __LINE__; } while(0)

static const uint8_t sbc_caps[]={1,0,7,6,0,0,0xff,0xff,2,53,8,0};
static const dbt_avdtp_endpoint_t endpoints[]={
    {sbc_caps,sizeof(sbc_caps),1,dbt_a2dp_sbc_validate},
    {sbc_caps,sizeof(sbc_caps),2,dbt_a2dp_sbc_validate}};
static const uint8_t selection[]={4,8,1,0,7,6,0,0,0x21,0x15,2,53,8,0};
typedef struct {
    dbt_avdtp_sink_t sink;
    uint8_t allocations[8][1026],job_configuration[1024],wire[1024];
    uint16_t allocation_sizes[8],job_size,wire_size;
    bool used[8],oom,work_busy,work_refuse,media_busy;
    unsigned allocated,released,errors,jobs,events,close_media,media_calls,drained;
    uint32_t job_token,job_binding;
    dbt_sink_operation_t operation;
    dbt_stream_state_t event_state;
    uint8_t next_label;
    uint32_t now;
} test_sink_t;
static test_sink_t f;
static void zero(void *v,unsigned n){uint8_t *p=v;for(unsigned i=0;i<n;++i)p[i]=0;}
static void copy(uint8_t *a,const uint8_t *b,unsigned n){for(unsigned i=0;i<n;++i)a[i]=b[i];}
static uint8_t *allocate(void *context,uint16_t n){
    test_sink_t *p=context;if(p->oom || !n || n>1024)return 0;
    for(unsigned i=0;i<8;++i)if(!p->used[i]){
        p->used[i]=true;p->allocation_sizes[i]=n;p->allocations[i][0]=0x5a;p->allocations[i][n+1u]=0xa5;
        ++p->allocated;return p->allocations[i]+1;
    }
    return 0;
}
static void release(void *context,uint8_t *bytes){
    test_sink_t *p=context;
    for(unsigned i=0;i<8;++i)if(bytes==p->allocations[i]+1){
        if(!p->used[i] || p->allocations[i][0]!=0x5a || p->allocations[i][p->allocation_sizes[i]+1u]!=0xa5)++p->errors;
        p->used[i]=false;++p->released;return;
    }
    ++p->errors;
}
static dbt_result_t work(void *context,const dbt_sink_job_t *job){
    test_sink_t *p=context;if(p->work_busy)return DBT_BUSY;if(p->work_refuse)return DBT_NO_SPACE;
    ++p->jobs;p->job_token=job->token;p->job_binding=job->binding;p->operation=job->operation;
    p->job_size=job->size;copy(p->job_configuration,job->configuration,job->size);return DBT_OK;
}
static void event(void *context,dbt_sink_event_t kind,dbt_stream_state_t state){
    test_sink_t *p=context;++p->events;p->event_state=state;
    if(kind==DBT_SINK_CLOSE_MEDIA)++p->close_media;
    if(kind==DBT_SINK_DRAINED)++p->drained;
}
static dbt_result_t media(void *context,const dbt_avdtp_media_t *view){
    test_sink_t *p=context;++p->media_calls;
    if(view->size!=2 || view->payload[0]!=1 || view->sequence!=0x1234)++p->errors;
    return p->media_busy?DBT_NO_SPACE:DBT_OK;
}
static bool fixture(void){
    zero(&f,sizeof(f));f.now=10;
    dbt_avdtp_sink_hooks_t hooks={&f,allocate,release,work,event,media};
    return dbt_avdtp_sink_init(&f.sink,hooks,endpoints,2,123,672,672,128,5000,1000,2000)==DBT_OK;
}
static dbt_result_t request(uint8_t signal,const uint8_t *body,uint16_t size){
    uint8_t wire[1026];wire[0]=(uint8_t)((f.next_label++&15u)<<4);wire[1]=signal;copy(wire+2,body,size);
    return dbt_avdtp_receive(&f.sink.signal,123,wire,(uint16_t)(size+2),++f.now);
}
static dbt_result_t output(bool commit){
    const uint8_t *wire=0;uint16_t size=0;
    dbt_result_t result=dbt_avdtp_tx_peek(&f.sink.signal,&wire,&size);
    if(result!=DBT_OK)return result;
    copy(f.wire,wire,size);f.wire_size=size;
    if(commit)return dbt_avdtp_tx_commit(&f.sink.signal,++f.now);
    return DBT_OK;
}
static bool ready(void){
    return dbt_avdtp_sink_poll(&f.sink,++f.now)==DBT_OK &&
        dbt_avdtp_sink_complete(&f.sink,f.job_binding,f.job_token,DBT_SINK_WORK_READY,++f.now)==DBT_OK;
}
static bool configured(void){
    return request(3,selection,sizeof(selection))==DBT_OK && ready() && output(true)==DBT_OK &&
        f.sink.state==DBT_STREAM_CONFIGURED;
}
static bool opened(void){
    uint8_t seid=4;
    return configured() && request(6,&seid,1)==DBT_OK && ready() && output(true)==DBT_OK &&
        dbt_avdtp_sink_media_open(&f.sink,456,++f.now)==DBT_OK;
}
static bool shutdown(void){
    dbt_result_t result=dbt_avdtp_sink_shutdown(&f.sink,++f.now);
    if(result==DBT_OK)return f.allocated==f.released && !f.errors;
    if(result!=DBT_BUSY || dbt_avdtp_sink_poll(&f.sink,++f.now)!=DBT_OK)return false;
    if(f.operation!=DBT_SINK_DROP)return false;
    if(dbt_avdtp_sink_complete(&f.sink,f.job_binding,f.job_token,DBT_SINK_WORK_READY,++f.now)!=DBT_OK)return false;
    return dbt_avdtp_sink_shutdown(&f.sink,++f.now)==DBT_OK && f.allocated==f.released && !f.errors;
}

EXPORT int test_avdtp_sink_discovery_flash_caps_and_validation(void){
    CHECK(fixture() && f.allocated==0);
    CHECK(request(1,0,0)==DBT_OK && output(true)==DBT_OK);
    CHECK(f.wire_size==6 && f.wire[2]==4 && f.wire[3]==8 && f.wire[4]==8 && !f.jobs);
    uint8_t seid=4;
    CHECK(request(2,&seid,1)==DBT_OK && output(true)==DBT_OK && f.wire_size==12);
    CHECK(request(12,&seid,1)==DBT_OK && output(true)==DBT_OK && f.wire_size==14);
    CHECK(f.allocated==f.released && !f.sink.configuration);
    uint8_t invalid[sizeof(selection)];copy(invalid,selection,sizeof(invalid));invalid[8]=0x31;
    CHECK(request(3,invalid,sizeof(invalid))==DBT_OK && output(true)==DBT_OK);
    CHECK((f.wire[0]&3)==3 && f.wire[2]==7 && f.wire[3]==0xc3 && !f.jobs && f.sink.state==DBT_STREAM_IDLE);
    CHECK(shutdown());return 0;
}
EXPORT int test_avdtp_sink_configuration_worker_and_admission_order(void){
    CHECK(fixture());CHECK(request(3,selection,sizeof(selection))==DBT_OK);
    CHECK(f.sink.state==DBT_STREAM_IDLE && f.sink.configuration==0 && !f.jobs);
    CHECK(dbt_avdtp_sink_wait_ms(&f.sink,f.now)==0);
    CHECK(output(true)==DBT_EMPTY);
    f.work_busy=true;CHECK(dbt_avdtp_sink_poll(&f.sink,++f.now)==DBT_OK && !f.jobs);
    CHECK(dbt_avdtp_sink_wait_ms(&f.sink,f.now)>0);
    f.work_busy=false;CHECK(dbt_avdtp_sink_poll(&f.sink,++f.now)==DBT_OK && f.jobs==1);
    CHECK(f.operation==DBT_SINK_CONFIGURE && f.job_size==12 && f.job_configuration[6]==0x21);
    CHECK(dbt_avdtp_sink_complete(&f.sink,124,f.job_token,DBT_SINK_WORK_READY,++f.now)==DBT_EMPTY);
    CHECK(dbt_avdtp_sink_complete(&f.sink,123,f.job_token+1,DBT_SINK_WORK_READY,++f.now)==DBT_EMPTY);
    CHECK(dbt_avdtp_sink_complete(&f.sink,123,f.job_token,DBT_SINK_WORK_READY,++f.now)==DBT_OK);
    CHECK(f.sink.state==DBT_STREAM_IDLE && output(false)==DBT_OK);
    CHECK(f.sink.state==DBT_STREAM_IDLE && dbt_avdtp_sink_shutdown(&f.sink,f.now)==DBT_BUSY);
    CHECK(dbt_avdtp_tx_commit(&f.sink.signal,++f.now)==DBT_OK && f.sink.state==DBT_STREAM_CONFIGURED);
    uint8_t seid=4;CHECK(request(4,&seid,1)==DBT_OK && output(true)==DBT_OK && f.wire_size==14);
    CHECK(request(1,0,0)==DBT_OK && output(true)==DBT_OK && f.wire[2]==6);
    CHECK(shutdown());return 0;
}
EXPORT int test_avdtp_sink_start_suspend_close_and_media_gate(void){
    CHECK(fixture() && opened());uint8_t seid=4;
    uint8_t rtp[]={0x80,96,0x12,0x34,0,0,0,1,0,0,0,1,1,0x9c};
    CHECK(dbt_avdtp_sink_media_receive(&f.sink,456,rtp,sizeof(rtp))==DBT_BUSY);
    CHECK(request(7,&seid,1)==DBT_OK && ready());
    CHECK(f.sink.state==DBT_STREAM_OPEN && output(true)==DBT_OK && f.sink.state==DBT_STREAM_STREAMING);
    CHECK(dbt_avdtp_sink_media_receive(&f.sink,457,rtp,sizeof(rtp))==DBT_EMPTY);
    CHECK(dbt_avdtp_sink_media_receive(&f.sink,456,rtp,sizeof(rtp))==DBT_OK && f.sink.media_packets==1);
    f.media_busy=true;CHECK(dbt_avdtp_sink_media_receive(&f.sink,456,rtp,sizeof(rtp))==DBT_NO_SPACE && f.sink.media_rejected==1);
    f.media_busy=false;CHECK(request(9,&seid,1)==DBT_OK);
    CHECK(dbt_avdtp_sink_media_receive(&f.sink,456,rtp,sizeof(rtp))==DBT_BUSY);
    CHECK(ready() && output(true)==DBT_OK && f.sink.state==DBT_STREAM_OPEN);
    CHECK(request(8,&seid,1)==DBT_OK && ready() && output(true)==DBT_OK);
    CHECK(f.sink.state==DBT_STREAM_CLOSING && f.close_media==1 && !f.sink.worker_live);
    CHECK(dbt_avdtp_sink_media_closed(&f.sink,456)==DBT_OK && f.sink.state==DBT_STREAM_IDLE && !f.sink.configuration);
    CHECK(shutdown());return 0;
}
EXPORT int test_avdtp_sink_ordered_list_prefix_and_rejection(void){
    CHECK(fixture() && opened());uint8_t list[]={4,8};
    CHECK(request(7,list,sizeof(list))==DBT_OK && ready() && output(true)==DBT_OK);
    CHECK((f.wire[0]&3)==3 && f.wire[2]==8 && f.wire[3]==0x31 && f.sink.state==DBT_STREAM_STREAMING);
    uint8_t duplicate[]={4,4};
    CHECK(request(9,duplicate,sizeof(duplicate))==DBT_OK && ready() && output(true)==DBT_OK);
    CHECK((f.wire[0]&3)==3 && f.wire[2]==4 && f.sink.state==DBT_STREAM_OPEN);
    list[0]=8;list[1]=4;unsigned jobs=f.jobs;
    CHECK(request(7,list,sizeof(list))==DBT_OK && output(true)==DBT_OK && f.jobs==jobs);
    CHECK(f.sink.state==DBT_STREAM_OPEN && f.wire[2]==8);
    CHECK(shutdown());return 0;
}
EXPORT int test_avdtp_sink_reconfigure_rollback_and_fatal_cleanup(void){
    CHECK(fixture() && opened());uint8_t reconfigure[]={4,7,6,0,0,0x11,0x15,2,51};
    CHECK(request(5,reconfigure,sizeof(reconfigure))==DBT_OK);
    CHECK(dbt_avdtp_sink_poll(&f.sink,++f.now)==DBT_OK && f.operation==DBT_SINK_RECONFIGURE);
    CHECK(f.sink.configuration[6]==0x21 && f.job_configuration[6]==0x11);
    CHECK(dbt_avdtp_sink_complete(&f.sink,123,f.job_token,DBT_SINK_WORK_REJECTED,++f.now)==DBT_OK);
    CHECK(output(true)==DBT_OK && f.sink.configuration[6]==0x21 && f.sink.state==DBT_STREAM_OPEN && f.sink.worker_live);
    CHECK(request(5,reconfigure,sizeof(reconfigure))==DBT_OK && ready());
    CHECK(f.sink.configuration[6]==0x21 && output(true)==DBT_OK && f.sink.configuration[6]==0x11 && f.sink.delay_enabled);
    CHECK(request(5,reconfigure,sizeof(reconfigure))==DBT_OK);
    CHECK(dbt_avdtp_sink_poll(&f.sink,++f.now)==DBT_OK);
    CHECK(dbt_avdtp_sink_complete(&f.sink,123,f.job_token,DBT_SINK_WORK_FATAL,++f.now)==DBT_INVALID && f.sink.failed);
    CHECK(dbt_avdtp_sink_shutdown(&f.sink,++f.now)==DBT_BUSY);
    CHECK(dbt_avdtp_sink_poll(&f.sink,++f.now)==DBT_INVALID && f.operation==DBT_SINK_DROP);
    CHECK(dbt_avdtp_sink_complete(&f.sink,123,f.job_token,DBT_SINK_WORK_READY,++f.now)==DBT_OK);
    CHECK(dbt_avdtp_sink_shutdown(&f.sink,++f.now)==DBT_OK && f.allocated==f.released && !f.errors);
    return 0;
}
EXPORT int test_avdtp_sink_disconnect_waits_for_worker_drop(void){
    CHECK(fixture());CHECK(request(3,selection,sizeof(selection))==DBT_OK);
    CHECK(dbt_avdtp_sink_poll(&f.sink,++f.now)==DBT_OK);uint32_t old=f.job_token;
    CHECK(dbt_avdtp_sink_shutdown(&f.sink,++f.now)==DBT_BUSY && f.sink.retiring && !f.sink.drained);
    CHECK(dbt_avdtp_sink_complete(&f.sink,123,old,DBT_SINK_WORK_READY,++f.now)==DBT_EMPTY);
    CHECK(dbt_avdtp_sink_poll(&f.sink,++f.now)==DBT_OK && f.operation==DBT_SINK_DROP && f.job_token!=old);
    CHECK(dbt_avdtp_sink_shutdown(&f.sink,f.now)==DBT_BUSY);
    CHECK(dbt_avdtp_sink_complete(&f.sink,123,old,DBT_SINK_WORK_READY,++f.now)==DBT_EMPTY);
    CHECK(dbt_avdtp_sink_complete(&f.sink,123,f.job_token,DBT_SINK_WORK_READY,++f.now)==DBT_OK);
    CHECK(f.sink.drained && !f.sink.worker_live && f.drained==1 && f.allocated==f.released && !f.errors);
    return 0;
}

EXPORT int test_avdtp_sink_abort_idle_invalid_and_active(void){
    CHECK(fixture());uint8_t seid=0;
    CHECK(request(10,&seid,1)==DBT_OK && !f.sink.signal.remote_pending && output(true)==DBT_EMPTY);
    CHECK(request(10,0,0)==DBT_OK && !f.sink.signal.remote_pending);
    uint8_t malformed[]={4,8};
    CHECK(request(10,malformed,2)==DBT_OK && !f.sink.signal.remote_pending);
    seid=4;CHECK(request(10,&seid,1)==DBT_OK && output(true)==DBT_OK && f.sink.state==DBT_STREAM_IDLE && !f.jobs);
    CHECK(opened());seid=8;unsigned jobs=f.jobs;
    CHECK(request(10,&seid,1)==DBT_OK && output(true)==DBT_OK && f.sink.state==DBT_STREAM_OPEN && f.jobs==jobs);
    seid=4;CHECK(request(10,&seid,1)==DBT_OK && ready() && output(true)==DBT_OK);
    CHECK(f.sink.state==DBT_STREAM_ABORTING && f.close_media==1 && !f.sink.worker_live);
    CHECK(dbt_avdtp_sink_media_closed(&f.sink,456)==DBT_OK && f.sink.state==DBT_STREAM_IDLE);
    CHECK(shutdown());return 0;
}
EXPORT int test_avdtp_sink_oom_refusal_and_resource_rejection(void){
    CHECK(fixture());f.oom=true;uint8_t seid=4;
    CHECK(request(12,&seid,1)==DBT_OK && output(true)==DBT_OK && (f.wire[0]&3)==3 && f.wire[2]==0x31);
    CHECK(request(3,selection,sizeof(selection))==DBT_OK && output(true)==DBT_OK && f.wire[2]==7 && f.wire[3]==0x29);
    CHECK(!f.jobs && !f.sink.worker_live && !f.sink.configuration);
    f.oom=false;f.work_refuse=true;
    CHECK(request(3,selection,sizeof(selection))==DBT_OK && dbt_avdtp_sink_poll(&f.sink,++f.now)==DBT_OK);
    CHECK(output(true)==DBT_OK && !f.jobs && !f.sink.worker_live && !f.sink.configuration);
    f.work_refuse=false;CHECK(configured());
    uint8_t other[sizeof(selection)];copy(other,selection,sizeof(other));other[0]=8;
    unsigned jobs=f.jobs;
    CHECK(request(3,other,sizeof(other))==DBT_OK && output(true)==DBT_OK && (f.wire[0]&3)==3 && f.jobs==jobs);
    CHECK(f.sink.endpoint==0 && f.sink.configuration_size==12);
    CHECK(shutdown());return 0;
}
EXPORT int test_avdtp_sink_deadlines_media_wrap_and_late_receipts(void){
    CHECK(fixture());f.work_busy=true;
    CHECK(request(3,selection,sizeof(selection))==DBT_OK && dbt_avdtp_sink_poll(&f.sink,++f.now)==DBT_OK);
    f.now=f.sink.pending.deadline;
    CHECK(dbt_avdtp_sink_poll(&f.sink,f.now)==DBT_INVALID && f.sink.failed && !f.jobs);
    CHECK(dbt_avdtp_sink_shutdown(&f.sink,++f.now)==DBT_OK && f.allocated==f.released);
    CHECK(fixture() && configured());f.now=UINT32_MAX-2003u;uint8_t seid=4;
    CHECK(request(6,&seid,1)==DBT_OK && ready() && output(true)==DBT_OK);
    CHECK(f.sink.media_deadline==0 && dbt_avdtp_sink_wait_ms(&f.sink,f.now)==2000);
    CHECK(dbt_avdtp_sink_poll(&f.sink,0)==DBT_INVALID && f.sink.failed);
    CHECK(dbt_avdtp_sink_shutdown(&f.sink,1)==DBT_BUSY);
    CHECK(dbt_avdtp_sink_poll(&f.sink,2)==DBT_INVALID && f.operation==DBT_SINK_DROP);
    CHECK(dbt_avdtp_sink_complete(&f.sink,123,f.job_token,DBT_SINK_WORK_READY,3)==DBT_OK && f.allocated==f.released);
    CHECK(fixture());CHECK(request(3,selection,sizeof(selection))==DBT_OK);
    CHECK(dbt_avdtp_sink_poll(&f.sink,++f.now)==DBT_OK);
    CHECK(dbt_avdtp_sink_complete(&f.sink,123,f.job_token,DBT_SINK_WORK_READY,f.sink.pending.deadline)==DBT_INVALID);
    CHECK(dbt_avdtp_sink_shutdown(&f.sink,f.now)==DBT_BUSY);
    CHECK(dbt_avdtp_sink_poll(&f.sink,++f.now)==DBT_INVALID && f.operation==DBT_SINK_DROP);
    CHECK(dbt_avdtp_sink_complete(&f.sink,123,f.job_token,DBT_SINK_WORK_READY,++f.now)==DBT_OK && f.allocated==f.released);
    return 0;
}
EXPORT int test_avdtp_sink_delay_reporting_and_close_deadline(void){
    CHECK(fixture() && opened());
    CHECK(dbt_avdtp_sink_delay(&f.sink,1234,++f.now)==DBT_OK && output(true)==DBT_OK);
    CHECK(f.wire_size==5 && f.wire[1]==13 && f.wire[2]==8 && f.wire[3]==4 && f.wire[4]==0xd2);
    uint8_t reply[]={(uint8_t)((f.wire[0]&0xf0u)|2u),13};
    CHECK(dbt_avdtp_receive(&f.sink.signal,123,reply,sizeof(reply),++f.now)==DBT_OK);
    uint8_t seid=4;CHECK(request(8,&seid,1)==DBT_OK && ready() && output(true)==DBT_OK);
    f.now=f.sink.media_deadline;
    CHECK(dbt_avdtp_sink_poll(&f.sink,f.now)==DBT_INVALID && f.sink.failed);
    CHECK(dbt_avdtp_sink_shutdown(&f.sink,++f.now)==DBT_OK && f.allocated==f.released);
    return 0;
}
EXPORT int test_a2dp_sbc_selection_errors_and_all_fields(void){
    uint8_t cie[]={0,0,0x21,0x15,2,53};dbt_avdtp_cap_t cap={cie,7,6};
    CHECK(dbt_a2dp_sbc_validate(&endpoints[0],&cap)==0);
    cie[2]=0x31;CHECK(dbt_a2dp_sbc_validate(&endpoints[0],&cap)==0xc3);cie[2]=0x23;
    CHECK(dbt_a2dp_sbc_validate(&endpoints[0],&cap)==0xc5);cie[2]=0x21;cie[3]=0x35;
    CHECK(dbt_a2dp_sbc_validate(&endpoints[0],&cap)==0xdd);cie[3]=0x1d;
    CHECK(dbt_a2dp_sbc_validate(&endpoints[0],&cap)==0xc7);cie[3]=0x17;
    CHECK(dbt_a2dp_sbc_validate(&endpoints[0],&cap)==0xc9);cie[3]=0x15;cie[4]=1;
    CHECK(dbt_a2dp_sbc_validate(&endpoints[0],&cap)==0xcb);cie[4]=2;cie[5]=251;
    CHECK(dbt_a2dp_sbc_validate(&endpoints[0],&cap)==0xcd);cie[5]=54;
    CHECK(dbt_a2dp_sbc_validate(&endpoints[0],&cap)==0xce);cie[5]=53;
    for(unsigned rate=0;rate<4;++rate)for(unsigned channel=0;channel<4;++channel)
    for(unsigned block=0;block<4;++block)for(unsigned bands=0;bands<2;++bands)for(unsigned allocation=0;allocation<2;++allocation){
        cie[2]=(uint8_t)((0x80u>>rate)|(8u>>channel));
        cie[3]=(uint8_t)((0x80u>>block)|(8u>>bands)|(2u>>allocation));
        CHECK(dbt_a2dp_sbc_validate(&endpoints[0],&cap)==0);
    }
    return 0;
}
EXPORT int test_avdtp_sink_generated_requests_leave_no_allocations(void){
    uint32_t random=0x998701u;uint8_t body[32];
    for(unsigned n=0;n<2048;++n){
        CHECK(fixture());
        for(unsigned j=0;j<sizeof(body);++j){random=random*1664525u+1013904223u;body[j]=(uint8_t)(random>>24);}
        CHECK(request((uint8_t)(1+n%13),body,(uint16_t)(n%sizeof(body)))==DBT_OK);
        /* No decoder jobs have been submitted yet, so shutdown is synchronous. */
        CHECK(dbt_avdtp_sink_shutdown(&f.sink,++f.now)==DBT_OK && f.allocated==f.released && !f.errors);
    }
    return 0;
}
EXPORT int test_avdtp_sink_abort_supersedes_pending_worker_safely(void){
    CHECK(fixture());uint8_t selected[sizeof(selection)];copy(selected,selection,sizeof(selected));selected[0]=8;
    CHECK(request(3,selected,sizeof(selected))==DBT_OK);
    CHECK(dbt_avdtp_sink_poll(&f.sink,++f.now)==DBT_OK && f.jobs==1);uint32_t stale=f.job_token;
    uint8_t seid=4;CHECK(request(10,&seid,1)==DBT_BUSY && f.sink.pending.token==stale);
    seid=8;CHECK(request(10,&seid,1)==DBT_OK && f.sink.pending.token!=stale);
    CHECK(dbt_avdtp_sink_complete(&f.sink,123,stale,DBT_SINK_WORK_READY,++f.now)==DBT_EMPTY);
    CHECK(output(true)==DBT_EMPTY && ready() && f.operation==DBT_SINK_ABORT);
    CHECK(output(true)==DBT_OK && f.sink.state==DBT_STREAM_IDLE && !f.sink.worker_live);
    CHECK(shutdown() && f.allocated==f.released);
    CHECK(fixture());CHECK(request(3,selection,sizeof(selection))==DBT_OK);
    seid=4;CHECK(request(10,&seid,1)==DBT_OK && output(true)==DBT_OK && !f.jobs && !f.sink.pending.used);
    CHECK(shutdown());return 0;
}
