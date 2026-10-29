#include "dbt_avdtp_sink.h"

static void zero(void *data, unsigned size) { uint8_t *p=data; for(unsigned i=0;i<size;++i)p[i]=0; }
static void copy(uint8_t *to,const uint8_t *from,uint16_t size) { for(uint16_t i=0;i<size;++i)to[i]=from[i]; }
static bool due(uint32_t now,uint32_t deadline) { return (int32_t)(now-deadline)>=0; }
static uint32_t until(uint32_t now,uint32_t deadline) { return due(now,deadline)?0:deadline-now; }
static void emit(dbt_avdtp_sink_t *s,dbt_sink_event_t event) {
    if(s->hooks.event)s->hooks.event(s->hooks.context,event,s->state);
}
static void fault(dbt_avdtp_sink_t *s) { if(!s->failed){s->failed=true;emit(s,DBT_SINK_FAULT);} }
static uint8_t *allocate_signal(void *context,uint16_t size) {
    dbt_avdtp_sink_t *s=context;return s->hooks.allocate(s->hooks.context,size);
}
static void release_signal(void *context,uint8_t *data) {
    dbt_avdtp_sink_t *s=context;s->hooks.release(s->hooks.context,data);
}
static int endpoint_index(const dbt_avdtp_sink_t *s,uint8_t seid) {
    for(unsigned i=0;i<s->count;++i)if(s->endpoints[i].seid==seid)return (int)i;
    return -1;
}
static void drop_pending(dbt_avdtp_sink_t *s) {
    if(s->pending.configuration)s->hooks.release(s->hooks.context,s->pending.configuration);
    zero(&s->pending,sizeof(s->pending));
}
static void idle(dbt_avdtp_sink_t *s) {
    if(s->configuration)s->hooks.release(s->hooks.context,s->configuration);
    s->configuration=0;s->configuration_size=0;s->state=DBT_STREAM_IDLE;
    s->media_binding=0;s->media_deadline=0;s->delay_enabled=false;s->remote_seid=0;
}
static uint32_t token(dbt_avdtp_sink_t *s) { if(!++s->next_token)++s->next_token;return s->next_token; }

static dbt_result_t respond(dbt_avdtp_sink_t *s,dbt_avdtp_type_t type,const uint8_t *body,uint16_t size) {
    return dbt_avdtp_respond(&s->signal,s->signal.remote_label,s->signal.remote_signal,
                            type,body,size,s->signal.now_ms);
}
static void reject(dbt_avdtp_sink_t *s,uint8_t detail,uint8_t error) {
    uint8_t body[2]={detail,error};uint8_t command=s->signal.remote_signal;
    bool detailed=command==3 || command==5 || command==7 || command==9;
    if(respond(s,DBT_AVDTP_REJECT,detailed?body:body+1,detailed?2:1)!=DBT_OK)fault(s);
}
static void reply_pending(dbt_avdtp_sink_t *s) {
    dbt_sink_pending_t *p=&s->pending;
    if(!p->used || p->queued || !p->done || s->retiring || s->failed)return;
    uint8_t body[2]={p->error_detail,p->error};
    bool detailed=p->signal==3 || p->signal==5 || p->signal==7 || p->signal==9;
    dbt_result_t result=respond(s,p->error?DBT_AVDTP_REJECT:DBT_AVDTP_ACCEPT,
        p->error?(detailed?body:body+1):0,p->error?(detailed?2:1):0);
    if(result==DBT_OK){p->queued=true;p->reply=p->error?DBT_AVDTP_REJECT:DBT_AVDTP_ACCEPT;}
    else fault(s);
}
static void begin(dbt_avdtp_sink_t *s,dbt_sink_operation_t operation,uint8_t endpoint,
                   bool work,bool apply) {
    dbt_sink_pending_t *p=&s->pending;
    zero(p,sizeof(*p));p->used=true;p->operation=operation;p->endpoint=endpoint;
    p->label=s->signal.remote_label;p->signal=s->signal.remote_signal;
    p->deadline=s->signal.now_ms+s->worker_ms;p->apply=apply;p->done=!work;
    if(work)p->token=token(s);
}

static void capabilities(dbt_avdtp_sink_t *s,const dbt_avdtp_endpoint_t *ep,bool all) {
    if(all){
        if(respond(s,DBT_AVDTP_ACCEPT,ep->capabilities,ep->size)!=DBT_OK)reject(s,0,0x31);
        return;
    }
    uint16_t cursor=0,size=0;dbt_avdtp_cap_t cap;
    while(dbt_avdtp_cap_next(ep->capabilities,ep->size,&cursor,&cap)==DBT_OK)
        if(cap.category<8)size=(uint16_t)(size+2u+cap.size);
    if(size==ep->size){
        if(respond(s,DBT_AVDTP_ACCEPT,ep->capabilities,size)!=DBT_OK)reject(s,0,0x31);
        return;
    }
    uint8_t small[16];uint8_t *bytes=size<=sizeof(small)?small:s->hooks.allocate(s->hooks.context,size);
    if(!bytes){reject(s,0,0x31);return;}
    uint16_t offset=0;cursor=0;
    while(dbt_avdtp_cap_next(ep->capabilities,ep->size,&cursor,&cap)==DBT_OK)if(cap.category<8){
        bytes[offset++]=cap.category;bytes[offset++]=cap.size;copy(bytes+offset,cap.data,cap.size);offset=(uint16_t)(offset+cap.size);
    }
    dbt_result_t result=respond(s,DBT_AVDTP_ACCEPT,bytes,size);
    if(bytes!=small)s->hooks.release(s->hooks.context,bytes);
    if(result!=DBT_OK)reject(s,0,0x31);
}

static void configure(dbt_avdtp_sink_t *s,const dbt_avdtp_message_t *m,unsigned index,bool reconfigure) {
    uint16_t header=reconfigure?1:2;
    if(m->size<header){reject(s,0,0x11);return;}
    if((reconfigure && (s->state!=DBT_STREAM_OPEN || index!=s->endpoint)) ||
       (!reconfigure && s->state!=DBT_STREAM_IDLE)){
        reject(s,0,!reconfigure && index==s->endpoint?0x13:0x31);return;
    }
    uint8_t remote=reconfigure?s->remote_seid:m->data[1]>>2;
    if(!remote || remote>62){reject(s,0,0x18);return;}
    dbt_avdtp_caps_t caps,local;dbt_avdtp_cap_error_t error;
    const dbt_avdtp_endpoint_t *ep=&s->endpoints[index];
    if(dbt_avdtp_caps_validate(m->data+header,(uint16_t)(m->size-header),
        reconfigure?DBT_AVDTP_RECONFIGURATION:DBT_AVDTP_CONFIGURATION,&caps,&error)!=DBT_OK){
        reject(s,error.category,error.code);return;
    }
    (void)dbt_avdtp_caps_validate(ep->capabilities,ep->size,DBT_AVDTP_CAPABILITIES,&local,&error);
    uint16_t cursor=0;dbt_avdtp_cap_t cap;
    while(dbt_avdtp_cap_next(m->data+header,(uint16_t)(m->size-header),&cursor,&cap)==DBT_OK)
        if(!(local.present&(1u<<cap.category))){reject(s,cap.category,0x29);return;}
    if(!caps.codec.data){reject(s,7,0x1a);return;}
    uint8_t code=ep->validate(ep,&caps.codec);
    if(code){reject(s,7,code);return;}
    uint16_t bytes=(uint16_t)(4u+caps.codec.size);
    bool delay=reconfigure?s->delay_enabled:(caps.present&(1u<<8))!=0;
    if(delay)bytes=(uint16_t)(bytes+2u);
    if(bytes>s->maximum_configuration){reject(s,7,0x29);return;}
    uint8_t *configuration=s->hooks.allocate(s->hooks.context,bytes);
    if(!configuration){reject(s,7,0x29);return;}
    configuration[0]=1;configuration[1]=0;configuration[2]=7;configuration[3]=caps.codec.size;
    copy(configuration+4,caps.codec.data,caps.codec.size);
    if(delay){configuration[bytes-2]=8;configuration[bytes-1]=0;}
    begin(s,reconfigure?DBT_SINK_RECONFIGURE:DBT_SINK_CONFIGURE,(uint8_t)index,true,true);
    s->pending.configuration=configuration;s->pending.size=bytes;s->pending.remote_seid=remote;
}

static void list_control(dbt_avdtp_sink_t *s,const dbt_avdtp_message_t *m,bool start) {
    if(!m->size){reject(s,0,0x11);return;}
    bool prefix=false;uint8_t bad=0,error=0;
    for(unsigned i=0;i<m->size;++i){
        uint8_t seid=m->data[i]>>2;int index=endpoint_index(s,seid);
        if(index<0){bad=(uint8_t)(seid<<2);error=0x12;break;}
        if(prefix || (unsigned)index!=s->endpoint ||
           s->state!=(start?DBT_STREAM_OPEN:DBT_STREAM_STREAMING) ||
           !s->worker_live || (start && !s->media_binding)){
            bad=(uint8_t)(seid<<2);error=0x31;break;
        }
        prefix=true;
    }
    if(!prefix){reject(s,bad,error);return;}
    /* With one active stream the valid prefix has at most one SEP. It is acted
     * on even if a later list item is rejected, as required by ordered lists. */
    begin(s,start?DBT_SINK_START:DBT_SINK_SUSPEND,s->endpoint,true,true);
    s->pending.error_detail=bad;s->pending.error=error;
}

static void command(void *context,const dbt_avdtp_message_t *m) {
    dbt_avdtp_sink_t *s=context;
    if(s->failed || s->retiring){reject(s,0,0x31);return;}
    if(m->signal==DBT_AVDTP_DISCOVER){
        if(m->size){reject(s,0,0x11);return;}
        uint8_t bytes[2*DBT_AVDTP_SINK_ENDPOINT_LIMIT];
        for(unsigned i=0;i<s->count;++i){
            bytes[2*i]=(uint8_t)((s->endpoints[i].seid<<2)|((s->state!=DBT_STREAM_IDLE && s->endpoint==i)?2:0));
            bytes[2*i+1]=8; /* Audio, sink */
        }
        if(respond(s,DBT_AVDTP_ACCEPT,bytes,(uint16_t)(2u*s->count))!=DBT_OK)reject(s,0,0x31);
        return;
    }
    if(m->signal==DBT_AVDTP_START || m->signal==DBT_AVDTP_SUSPEND){
        list_control(s,m,m->signal==DBT_AVDTP_START);return;
    }
    if(!m->size){
        if(m->signal!=DBT_AVDTP_ABORT)reject(s,0,0x11);
        else (void)dbt_avdtp_ignore_abort(&s->signal,m->label);
        return;
    }
    uint8_t seid=m->data[0]>>2;int index=endpoint_index(s,seid);
    if(index<0){
        if(m->signal!=DBT_AVDTP_ABORT)reject(s,0,0x12);
        else (void)dbt_avdtp_ignore_abort(&s->signal,m->label);
        return;
    }
    if(m->signal==DBT_AVDTP_SET_CONFIGURATION || m->signal==DBT_AVDTP_RECONFIGURE){
        configure(s,m,(unsigned)index,m->signal==DBT_AVDTP_RECONFIGURE);return;
    }
    if(m->signal==DBT_AVDTP_SECURITY_CONTROL || m->signal==DBT_AVDTP_DELAY_REPORT){reject(s,0,0x19);return;}
    if(m->size!=1){
        if(m->signal!=DBT_AVDTP_ABORT)reject(s,0,0x11);
        else (void)dbt_avdtp_ignore_abort(&s->signal,m->label);
        return;
    }
    switch(m->signal){
    case DBT_AVDTP_GET_CAPABILITIES:case DBT_AVDTP_GET_ALL_CAPABILITIES:
        capabilities(s,&s->endpoints[index],m->signal==DBT_AVDTP_GET_ALL_CAPABILITIES);return;
    case DBT_AVDTP_GET_CONFIGURATION:
        if(s->state==DBT_STREAM_IDLE || (unsigned)index!=s->endpoint){reject(s,0,0x31);return;}
        if(respond(s,DBT_AVDTP_ACCEPT,s->configuration,s->configuration_size)!=DBT_OK)reject(s,0,0x31);
        return;
    case DBT_AVDTP_OPEN:
        if(s->state!=DBT_STREAM_CONFIGURED || (unsigned)index!=s->endpoint){reject(s,0,0x31);return;}
        begin(s,DBT_SINK_OPEN,s->endpoint,true,true);return;
    case DBT_AVDTP_CLOSE:
        if((s->state!=DBT_STREAM_OPEN && s->state!=DBT_STREAM_STREAMING) || (unsigned)index!=s->endpoint){reject(s,0,0x31);return;}
        begin(s,DBT_SINK_CLOSE,s->endpoint,true,true);return;
    case DBT_AVDTP_ABORT:
        begin(s,DBT_SINK_ABORT,(uint8_t)index,(s->worker_live || s->state!=DBT_STREAM_IDLE) && (unsigned)index==s->endpoint,
              (s->worker_live || s->state!=DBT_STREAM_IDLE) && (unsigned)index==s->endpoint);
        reply_pending(s);return;
    default:reject(s,0,0x19);return;
    }
}

static void responded(void *context,uint8_t label,uint8_t signal,dbt_avdtp_type_t type) {
    dbt_avdtp_sink_t *s=context;dbt_sink_pending_t *p=&s->pending;
    if(!p->used || !p->queued || p->label!=label || p->signal!=signal || p->reply!=type)return;
    bool close_media=false;
    if(p->apply){
        switch(p->operation){
        case DBT_SINK_CONFIGURE:case DBT_SINK_RECONFIGURE:{
            if(s->configuration)s->hooks.release(s->hooks.context,s->configuration);
            s->configuration=p->configuration;s->configuration_size=p->size;p->configuration=0;
            s->endpoint=p->endpoint;s->remote_seid=p->remote_seid;
            dbt_avdtp_caps_t caps;dbt_avdtp_cap_error_t error;
            (void)dbt_avdtp_caps_validate(s->configuration,s->configuration_size,DBT_AVDTP_CONFIGURATION,&caps,&error);
            s->delay_enabled=(caps.present&(1u<<8))!=0;
            if(p->operation==DBT_SINK_CONFIGURE)s->state=DBT_STREAM_CONFIGURED;
            break;}
        case DBT_SINK_OPEN:s->state=DBT_STREAM_OPEN;s->media_deadline=s->signal.now_ms+s->media_ms;break;
        case DBT_SINK_START:s->state=DBT_STREAM_STREAMING;break;
        case DBT_SINK_SUSPEND:s->state=DBT_STREAM_OPEN;break;
        case DBT_SINK_CLOSE:case DBT_SINK_ABORT:
            s->state=p->operation==DBT_SINK_CLOSE?DBT_STREAM_CLOSING:DBT_STREAM_ABORTING;
            if(s->media_binding){close_media=true;s->media_deadline=s->signal.now_ms+s->media_ms;}
            else idle(s);
            break;
        default:break;
        }
        emit(s,DBT_SINK_STATE);
    }
    drop_pending(s);
    if(close_media)emit(s,DBT_SINK_CLOSE_MEDIA);
}
static void local_complete(void *context,dbt_avdtp_outcome_t outcome,const dbt_avdtp_message_t *m) {
    dbt_avdtp_sink_t *s=context;(void)m;
    if(!s->retiring && outcome!=DBT_AVDTP_RESPONSE)fault(s);
    if(!s->retiring)emit(s,DBT_SINK_DELAY_RESPONSE);
}
static bool interrupt(void *context,const dbt_avdtp_message_t *m) {
    dbt_avdtp_sink_t *s=context;dbt_sink_pending_t *p=&s->pending;
    if(s->failed || s->retiring || m->size!=1 || !p->used || p->queued ||
       (m->data[0]>>2)!=s->endpoints[p->endpoint].seid)return false;
    if(s->state==DBT_STREAM_IDLE && s->worker_live)s->endpoint=p->endpoint;
    /* Worker copied its job. A later serialized Abort releases any resources
     * it creates; the superseded completion token must no longer affect state. */
    drop_pending(s);
    return true;
}

dbt_result_t dbt_avdtp_sink_init(dbt_avdtp_sink_t *s,dbt_avdtp_sink_hooks_t hooks,
    const dbt_avdtp_endpoint_t *endpoints,uint8_t count,uint32_t binding,
    uint16_t rx_mtu,uint16_t tx_mtu,uint16_t maximum_configuration,
    uint32_t signal_ms,uint32_t worker_ms,uint32_t media_ms) {
    if(!s || !hooks.allocate || !hooks.release || !hooks.work || !hooks.media ||
       !endpoints || !count || count>DBT_AVDTP_SINK_ENDPOINT_LIMIT || maximum_configuration<6 ||
       maximum_configuration>1024 || !worker_ms || worker_ms>INT32_MAX || !media_ms || media_ms>INT32_MAX)return DBT_INVALID;
    for(unsigned i=0;i<count;++i){
        const dbt_avdtp_endpoint_t *ep=&endpoints[i];dbt_avdtp_caps_t caps;dbt_avdtp_cap_error_t error;
        if(!ep->seid || ep->seid>62 || !ep->validate || ep->size>1024 ||
            dbt_avdtp_caps_validate(ep->capabilities,ep->size,DBT_AVDTP_CAPABILITIES,&caps,&error)!=DBT_OK ||
            (caps.present&0x82u)!=0x82u || (caps.present&~0x182u) || caps.codec.data[0]>>4)return DBT_INVALID;
        for(unsigned j=0;j<i;++j)if(endpoints[j].seid==ep->seid)return DBT_INVALID;
    }
    zero(s,sizeof(*s));s->hooks=hooks;s->endpoints=endpoints;s->count=count;s->binding=binding;
    s->maximum_configuration=maximum_configuration;s->worker_ms=worker_ms;s->media_ms=media_ms;
    dbt_avdtp_hooks_t signaling={s,allocate_signal,release_signal,command,local_complete,responded,interrupt};
    return dbt_avdtp_init(&s->signal,signaling,binding,rx_mtu,tx_mtu,1024,signal_ms,signal_ms);
}

dbt_result_t dbt_avdtp_sink_complete(dbt_avdtp_sink_t *s,uint32_t binding,uint32_t job,
    dbt_sink_work_result_t result,uint32_t now) {
    if(!s || s->in_work || result>DBT_SINK_WORK_FATAL || result<DBT_SINK_WORK_READY)return DBT_INVALID;
    dbt_sink_pending_t *p=&s->pending;
    if(binding!=s->binding || !p->used || !p->submitted || p->done || job!=p->token)return DBT_EMPTY;
    if(p->operation==DBT_SINK_DROP){
        if(result!=DBT_SINK_WORK_READY){fault(s);return DBT_INVALID;}
        s->worker_live=false;drop_pending(s);idle(s);s->drained=true;emit(s,DBT_SINK_DRAINED);return DBT_OK;
    }
    if(s->retiring)return DBT_EMPTY;
    if(dbt_avdtp_poll(&s->signal,now)!=DBT_OK || due(now,p->deadline)){fault(s);return DBT_INVALID;}
    if(result==DBT_SINK_WORK_FATAL){fault(s);return DBT_INVALID;}
    p->done=true;
    if(result==DBT_SINK_WORK_REJECTED){
        p->apply=false;p->error=(p->operation==DBT_SINK_CONFIGURE || p->operation==DBT_SINK_RECONFIGURE)?0x29:0x31;
        p->error_detail=(p->signal==3 || p->signal==5)?7:(uint8_t)(s->endpoints[p->endpoint].seid<<2);
        if(p->operation==DBT_SINK_CONFIGURE)s->worker_live=false;
    }else if(p->operation==DBT_SINK_CLOSE || p->operation==DBT_SINK_ABORT)s->worker_live=false;
    reply_pending(s);return s->failed?DBT_INVALID:DBT_OK;
}

dbt_result_t dbt_avdtp_sink_poll(dbt_avdtp_sink_t *s,uint32_t now) {
    if(!s || !s->binding || s->in_work)return DBT_INVALID;
    if(s->drained)return DBT_OK;
    if(!s->retiring && (s->failed || dbt_avdtp_poll(&s->signal,now)!=DBT_OK)){fault(s);return DBT_INVALID;}
    bool media_wait=(s->state==DBT_STREAM_OPEN && !s->media_binding) ||
        ((s->state==DBT_STREAM_CLOSING || s->state==DBT_STREAM_ABORTING) && s->media_binding);
    if(!s->retiring && media_wait && due(now,s->media_deadline)){
        fault(s);return DBT_INVALID;
    }
    dbt_sink_pending_t *p=&s->pending;
    if(p->used && !p->done && due(now,p->deadline)){fault(s);return DBT_INVALID;}
    if(p->used && !p->submitted && !p->done){
        const uint8_t *configuration=p->configuration?p->configuration:s->configuration;
        uint16_t size=p->configuration?p->size:s->configuration_size;
        dbt_sink_job_t request={s->binding,p->token,p->operation,s->endpoints[p->endpoint].seid,configuration,size};
        p->attempted=true;
        s->in_work=true;dbt_result_t result=s->hooks.work(s->hooks.context,&request);s->in_work=false;
        if(result==DBT_OK){p->submitted=true;s->worker_live=true;}
        else if(result!=DBT_BUSY){
            if(p->operation==DBT_SINK_DROP){fault(s);return DBT_INVALID;}
            p->submitted=true;
            /* Enqueue refusal accepted no worker state; CONFIGURE has none. */
            (void)dbt_avdtp_sink_complete(s,s->binding,p->token,DBT_SINK_WORK_REJECTED,now);
        }
    }
    reply_pending(s);return s->failed?DBT_INVALID:DBT_OK;
}

dbt_result_t dbt_avdtp_sink_media_open(dbt_avdtp_sink_t *s,uint32_t binding,uint32_t now) {
    if(!s || !binding || s->retiring || s->failed || s->state!=DBT_STREAM_OPEN || s->pending.used || s->media_binding)return DBT_INVALID;
    if(due(now,s->media_deadline)){fault(s);return DBT_INVALID;}
    s->media_binding=binding;s->media_deadline=0;s->media_packets=s->media_rejected=0;return DBT_OK;
}
dbt_result_t dbt_avdtp_sink_media_closed(dbt_avdtp_sink_t *s,uint32_t binding) {
    if(!s || !binding || s->media_binding!=binding)return DBT_EMPTY;
    s->media_binding=0;
    if(s->state==DBT_STREAM_CLOSING || s->state==DBT_STREAM_ABORTING){idle(s);emit(s,DBT_SINK_STATE);return DBT_OK;}
    fault(s);return DBT_INVALID;
}
dbt_result_t dbt_avdtp_sink_media_receive(dbt_avdtp_sink_t *s,uint32_t binding,const uint8_t *packet,uint16_t size) {
    if(!s || !binding || binding!=s->media_binding)return DBT_EMPTY;
    if(s->failed || s->retiring || s->state!=DBT_STREAM_STREAMING ||
       (s->pending.used && s->pending.operation!=DBT_SINK_START))return DBT_BUSY;
    dbt_avdtp_media_t media;dbt_result_t result=dbt_avdtp_media_decode(packet,size,&media);
    if(result==DBT_OK)result=s->hooks.media(s->hooks.context,&media);
    if(result==DBT_OK)++s->media_packets;else ++s->media_rejected;
    return result;
}
dbt_result_t dbt_avdtp_sink_delay(dbt_avdtp_sink_t *s,uint16_t delay,uint32_t now) {
    if(!s || s->failed || s->retiring || !s->delay_enabled || s->state==DBT_STREAM_IDLE ||
       s->state==DBT_STREAM_CLOSING || s->state==DBT_STREAM_ABORTING)return DBT_INVALID;
    uint8_t body[3]={(uint8_t)(s->remote_seid<<2),(uint8_t)(delay>>8),(uint8_t)delay},label=0;
    return dbt_avdtp_command(&s->signal,DBT_AVDTP_DELAY_REPORT,body,3,now,&label);
}
uint32_t dbt_avdtp_sink_wait_ms(const dbt_avdtp_sink_t *s,uint32_t now) {
    if(!s || s->drained || (!s->retiring && s->failed))return UINT32_MAX;
    uint32_t wait=s->retiring?UINT32_MAX:dbt_avdtp_wait_ms(&s->signal,now);
    const dbt_sink_pending_t *p=&s->pending;
    if(p->used && !p->done){
        if(!p->attempted)return 0;
        uint32_t next=until(now,p->deadline);if(next<wait)wait=next;
    }
    bool media_wait=(s->state==DBT_STREAM_OPEN && !s->media_binding) ||
        ((s->state==DBT_STREAM_CLOSING || s->state==DBT_STREAM_ABORTING) && s->media_binding);
    if(!s->retiring && media_wait){
        uint32_t next=until(now,s->media_deadline);if(next<wait)wait=next;
    }
    /* Work queue availability is a provider wakeup; BUSY must not spin. */
    return wait;
}
dbt_result_t dbt_avdtp_sink_shutdown(dbt_avdtp_sink_t *s,uint32_t now) {
    if(!s || !s->binding || s->in_work || s->signal.in_callback)return DBT_INVALID;
    if(s->drained)return DBT_OK;
    if(s->signal.pinned)return DBT_BUSY;
    if(!s->retiring){
        s->retiring=true;
        if(s->signal.binding && dbt_avdtp_clear(&s->signal)!=DBT_OK)return DBT_BUSY;
        drop_pending(s);
        if(!s->worker_live){idle(s);s->drained=true;emit(s,DBT_SINK_DRAINED);return DBT_OK;}
        dbt_sink_pending_t *p=&s->pending;p->used=true;p->operation=DBT_SINK_DROP;
        p->endpoint=s->endpoint;p->token=token(s);p->deadline=now+s->worker_ms;
    }
    return DBT_BUSY;
}
