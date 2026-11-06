#include "dbt_a2dp_acceptor.h"

static void zero(void *object,unsigned n){uint8_t *p=object;for(unsigned i=0;i<n;++i)p[i]=0;}
static bool matches(const dbt_a2dp_channel_t *key,const dbt_channel_t *channel){
    return key->serial && channel && channel->serial==key->serial &&
        channel->link_generation==key->generation && channel->handle==key->handle && channel->local_cid==key->cid;
}
static const dbt_channel_t *live(const dbt_a2dp_acceptor_t *p,const dbt_a2dp_channel_t *key){
    const dbt_channel_t *channel=dbt_channels_find(&p->runtime->channels,key->handle,key->cid);
    return matches(key,channel)?channel:0;
}
static void remember(dbt_a2dp_channel_t *key,const dbt_channel_t *channel){
    *key=(dbt_a2dp_channel_t){channel->serial,channel->link_generation,channel->handle,
                           channel->local_cid,channel->rx_mtu,channel->tx_mtu};
}
static uint8_t *allocate(void *context,uint16_t size){
    dbt_a2dp_acceptor_t *p=context;return p->client.allocate(p->client.context,size);
}
static void release(void *context,uint8_t *bytes){
    dbt_a2dp_acceptor_t *p=context;p->client.release(p->client.context,bytes);
}
static dbt_result_t work(void *context,const dbt_sink_job_t *job){
    dbt_a2dp_acceptor_t *p=context;return p->client.work(p->client.context,job);
}
static dbt_result_t media(void *context,const dbt_avdtp_media_t *view){
    dbt_a2dp_acceptor_t *p=context;return p->client.media(p->client.context,view);
}
static void event(void *context,dbt_sink_event_t kind,dbt_stream_state_t state){
    dbt_a2dp_acceptor_t *p=context;
    if(kind==DBT_SINK_CLOSE_MEDIA)p->close_media=true;
    if(kind==DBT_SINK_FAULT)p->draining=true;
    if(p->client.event)p->client.event(p->client.context,kind,state);
}
dbt_result_t dbt_a2dp_acceptor_init(dbt_a2dp_acceptor_t *p,dbt_runtime_t *runtime,
    dbt_avdtp_sink_hooks_t worker,const dbt_avdtp_endpoint_t *endpoints,uint8_t count,
    uint16_t mtu,uint16_t maximum_configuration,uint32_t signal_ms,uint32_t worker_ms,uint32_t media_ms){
    if(!p || !runtime || !runtime->ready || mtu<335 || mtu>runtime->maximum_mtu ||
        !worker.allocate || !worker.release || !worker.work || !worker.media)return DBT_INVALID;
    /* Validate descriptors/timers without retaining a dummy signaling binding. */
    dbt_avdtp_sink_t check;
    dbt_result_t result=dbt_avdtp_sink_init(&check,worker,endpoints,count,1,mtu,mtu,
                                         maximum_configuration,signal_ms,worker_ms,media_ms);
    if(result!=DBT_OK)return result;
    dbt_service_t service={0x19,mtu,335,DBT_SECURITY_ENCRYPTED};
    result=dbt_channels_register(&runtime->channels,service);
    if(result!=DBT_OK)return result;
    zero(p,sizeof(*p));p->runtime=runtime;p->client=worker;p->endpoints=endpoints;p->count=count;
    p->mtu=mtu;p->maximum_configuration=maximum_configuration;p->signal_ms=signal_ms;
    p->worker_ms=worker_ms;p->media_ms=media_ms;p->enabled=true;return DBT_OK;
}
bool dbt_a2dp_acceptor_accept(const dbt_a2dp_acceptor_t *p,const dbt_link_t *link){
    if(!p || !p->runtime || !p->enabled || p->draining || !link || !link->used || link->kind!=DBT_LINK_CLASSIC)return false;
    unsigned staged=0;
    for(unsigned i=0;i<DBT_CHANNEL_LIMIT;++i){
        const dbt_channel_t *channel=&p->runtime->channels.channels[i];
        if(channel->state!=DBT_CH_FREE && channel->psm==0x19)++staged;
    }
    if(!p->active)return staged==0;
    return staged==1 && p->signaling.handle==link->handle && p->signaling.generation==link->generation &&
        p->sink.state==DBT_STREAM_OPEN && !p->sink.pending.used && !p->sink.media_binding &&
        !p->media.serial && !p->sink.failed && !p->sink.media_initiator;
}
dbt_result_t dbt_a2dp_acceptor_event_ex(dbt_a2dp_acceptor_t *p,dbt_channel_event_t kind,const dbt_channel_t *channel,
    dbt_channel_reason_t reason,uint16_t wire_result){
    if(!p || !p->runtime || !channel)return DBT_INVALID;
    if(channel->psm!=0x19)return DBT_UNSUPPORTED;
    if(kind==DBT_CH_EVENT_OPEN){
        const dbt_link_t *link=dbt_acl_find(&p->runtime->acl,channel->handle);
        if(!link || link->kind!=DBT_LINK_CLASSIC || !link->encrypted ||
            link->generation!=channel->link_generation || channel->rx_mtu<335 || channel->tx_mtu<335 ||
            !p->enabled || p->draining)return DBT_INVALID;
        if(!p->active){
            dbt_avdtp_sink_hooks_t hooks={p,allocate,release,work,event,media};
            dbt_result_t result=dbt_avdtp_sink_init(&p->sink,hooks,p->endpoints,p->count,channel->serial,
                channel->rx_mtu,channel->tx_mtu,p->maximum_configuration,p->signal_ms,p->worker_ms,p->media_ms);
            if(result!=DBT_OK)return result;
            result=dbt_avdtp_bearer_init(&p->bearer,p->runtime,&p->sink.signal,channel);
            if(result!=DBT_OK){(void)dbt_avdtp_sink_shutdown(&p->sink,p->runtime->now_ms);return result;}
            remember(&p->signaling,channel);p->active=true;return DBT_OK;
        }
        if(p->signaling.handle!=channel->handle || p->signaling.generation!=channel->link_generation ||
           ((p->media.serial || p->sink.media_initiator) && !matches(&p->media,channel)))return DBT_INVALID;
        if(p->sink.local && p->sink.local->result.operation==DBT_SINK_ABORT &&
           p->sink.local->result.media_was_pending && p->sink.local->result.media_binding==channel->serial){
            p->close_media=true;return DBT_OK; /* Retire; never bind an aborted setup. */
        }
        dbt_result_t result=dbt_avdtp_sink_media_open(&p->sink,channel->serial,p->runtime->now_ms);
        if(result==DBT_OK)remember(&p->media,channel);
        return result;
    }
    if(kind==DBT_CH_EVENT_CLOSED || kind==DBT_CH_EVENT_FAILED){
        if(matches(&p->signaling,channel)){
            dbt_avdtp_bearer_event(&p->bearer,kind,channel);zero(&p->signaling,sizeof(p->signaling));
            p->draining=true;return DBT_OK;
        }
        if(matches(&p->media,channel)){
            bool bound=p->sink.media_binding==channel->serial;
            bool pending_abort=p->sink.local && p->sink.local->result.operation==DBT_SINK_ABORT &&
                p->sink.local->result.media_was_pending && p->sink.local->result.media_binding==channel->serial;
            if((bound || pending_abort) && p->sink.local){
                dbt_avdtp_local_result_t *r=&p->sink.local->result;r->media_event_valid=true;
                r->media_event=(uint8_t)kind;r->media_reason=(uint8_t)reason;r->media_status=wire_result;
            }
            bool failed=(kind==DBT_CH_EVENT_FAILED && !pending_abort) || reason!=DBT_CH_REASON_NORMAL || wire_result!=0;
            dbt_result_t result=(bound || pending_abort)?dbt_avdtp_sink_media_closed_ex(&p->sink,channel->serial,failed):DBT_OK;
            zero(&p->media,sizeof(p->media));p->close_media=false;
            if((!bound && !pending_abort) || result!=DBT_OK)p->draining=true;
            return DBT_OK;
        }
        return DBT_EMPTY;
    }
    if(kind==DBT_CH_EVENT_RECONFIGURED && (matches(&p->signaling,channel) || matches(&p->media,channel))){
        const dbt_a2dp_channel_t *key=matches(&p->signaling,channel)?&p->signaling:&p->media;
        if(key->rx_mtu!=channel->rx_mtu || key->tx_mtu!=channel->tx_mtu)p->draining=true;
    }
    return DBT_OK;
}
dbt_result_t dbt_a2dp_acceptor_event(dbt_a2dp_acceptor_t *p,dbt_channel_event_t kind,const dbt_channel_t *channel){
    return dbt_a2dp_acceptor_event_ex(p,kind,channel,DBT_CH_REASON_NORMAL,0);
}
dbt_result_t dbt_a2dp_acceptor_receive(dbt_a2dp_acceptor_t *p,const dbt_channel_t *channel,const uint8_t *data,uint16_t size){
    if(!p || !p->runtime || !channel)return DBT_INVALID;
    if(channel->psm!=0x19)return DBT_UNSUPPORTED;
    if(!p->active || p->draining)return DBT_BUSY;
    if(matches(&p->signaling,channel)){
        uint32_t discarded=p->sink.signal.discarded;
        dbt_result_t result=dbt_avdtp_bearer_receive(&p->bearer,channel,data,size);
        /* AVDTP integrity failures discard only that message (8.4.6). Preserve
         * live media and pending valid transactions; API/lifetime/resource and
         * actual transaction failures still require retirement. */
        bool wire_discard=result==DBT_INVALID && !p->sink.signal.failed &&
            p->sink.signal.discarded!=discarded;
        if(result!=DBT_OK && result!=DBT_EMPTY && !wire_discard)p->draining=true;
        return result;
    }
    if(matches(&p->media,channel)){
        const dbt_channel_t *current=live(p,&p->media);
        if(!current || current->state!=DBT_CH_OPEN || size>current->rx_mtu)return DBT_INVALID;
        return dbt_avdtp_sink_media_receive(&p->sink,channel->serial,data,size);
    }
    return DBT_EMPTY;
}
static void close_channel(dbt_a2dp_acceptor_t *p,dbt_a2dp_channel_t *key,uint32_t now){
    if(!key->serial)return;
    const dbt_channel_t *channel=live(p,key);
    if(!channel){zero(key,sizeof(*key));return;}
    if(channel->state!=DBT_CH_DISCONNECTING)(void)dbt_channels_close(&p->runtime->channels,key->handle,key->cid,now);
}
static bool staged(const dbt_a2dp_acceptor_t *p){
    for(unsigned i=0;i<DBT_CHANNEL_LIMIT;++i)
        if(p->runtime->channels.channels[i].state!=DBT_CH_FREE && p->runtime->channels.channels[i].psm==0x19)return true;
    return false;
}
dbt_result_t dbt_a2dp_acceptor_connect_begin(dbt_a2dp_acceptor_t *p,const dbt_link_t *link,
                                           uint32_t now,dbt_a2dp_channel_t *key){
    if(!p || !p->runtime || !link || !key || !link->used || link->kind!=DBT_LINK_CLASSIC)return DBT_INVALID;
    if(!p->runtime->ready || p->runtime->host.state!=DBT_HOST_IDENTIFIED ||
       !p->enabled || p->active || p->draining || staged(p))return DBT_BUSY;
    const dbt_link_t *current=dbt_acl_find(&p->runtime->acl,link->handle);
    if(!current || current->generation!=link->generation || current->kind!=DBT_LINK_CLASSIC)return DBT_EMPTY;
    uint16_t cid;
    dbt_result_t result=dbt_channels_open(&p->runtime->channels,current->handle,0x19,p->mtu,335,
                                         DBT_SECURITY_ENCRYPTED,now,&cid);
    if(result!=DBT_OK)return result;
    const dbt_channel_t *channel=dbt_channels_find(&p->runtime->channels,current->handle,cid);
    if(!channel || !channel->serial){dbt_runtime_abort(p->runtime,DBT_FAULT_BAD_RESPONSE);return DBT_INVALID;}
    remember(key,channel);return DBT_PENDING;
}
dbt_result_t dbt_a2dp_acceptor_connect_status(const dbt_a2dp_acceptor_t *p,const dbt_a2dp_channel_t *key){
    if(!p || !p->runtime || !key || !key->serial || !key->generation)return DBT_INVALID;
    const dbt_channel_t *channel=live(p,key);
    if(!channel || channel->psm!=0x19)return DBT_EMPTY;
    if(p->runtime->host.state==DBT_HOST_FAILED)return DBT_INVALID;
    if(channel->state!=DBT_CH_OPEN || p->draining)return DBT_PENDING;
    return p->active && matches(&p->signaling,channel) && !p->sink.failed && !p->sink.signal.failed?DBT_OK:DBT_INVALID;
}
dbt_result_t dbt_a2dp_acceptor_connect_cancel(dbt_a2dp_acceptor_t *p,const dbt_a2dp_channel_t *key,uint32_t now){
    if(!p || !p->runtime || !key || !key->serial || !key->generation)return DBT_INVALID;
    const dbt_channel_t *channel=live(p,key);
    if(!channel || channel->psm!=0x19)return DBT_EMPTY;
    if(p->active && matches(&p->signaling,channel)){
        dbt_result_t result=dbt_a2dp_acceptor_retire(p,now);return result==DBT_BUSY?DBT_PENDING:result;
    }
    dbt_result_t result=dbt_channels_request_close(&p->runtime->channels,channel);
    return result==DBT_OK?DBT_PENDING:result;
}
static bool overlap(const void *a,unsigned an,const void *b,unsigned bn){
    uintptr_t x=(uintptr_t)a,y=(uintptr_t)b;return an && bn && (x<=y?y-x<an:x-y<bn);
}
dbt_result_t dbt_a2dp_acceptor_abort_begin(dbt_a2dp_acceptor_t *p,const dbt_a2dp_channel_t *signaling,
    dbt_avdtp_local_t *context,uint32_t now,uint32_t *id){
    if(!p || !p->runtime || !signaling || !context || !id || overlap(p,sizeof(*p),context,sizeof(*context)) ||
       overlap(p,sizeof(*p),id,sizeof(*id)) || overlap(p->runtime,sizeof(*p->runtime),context,sizeof(*context)) ||
       overlap(p->runtime,sizeof(*p->runtime),id,sizeof(*id)) || overlap(signaling,sizeof(*signaling),context,sizeof(*context)) ||
       overlap(signaling,sizeof(*signaling),id,sizeof(*id)))return DBT_INVALID;
    dbt_result_t ready=dbt_a2dp_acceptor_connect_status(p,signaling);if(ready!=DBT_OK)return ready==DBT_PENDING?DBT_BUSY:ready;
    const dbt_channel_t *selected=0;
    for(unsigned i=0;i<DBT_CHANNEL_LIMIT;++i){
        const dbt_channel_t *ch=&p->runtime->channels.channels[i];
        if(ch->state==DBT_CH_FREE || ch->psm!=0x19 || matches(&p->signaling,ch))continue;
        if(selected || ch->handle!=p->signaling.handle || ch->link_generation!=p->signaling.generation)return DBT_BUSY;
        selected=ch;
    }
    if(p->media.serial && (!selected || !matches(&p->media,selected)))return DBT_BUSY;
    if(p->sink.media_binding && (!selected || selected->serial!=p->sink.media_binding))return DBT_INVALID;
    dbt_result_t result=dbt_avdtp_sink_local_abort_begin(&p->sink,context,selected?selected->serial:0,now,id);
    if(result==DBT_PENDING && selected && !p->media.serial)remember(&p->media,selected);
    return result;
}
dbt_result_t dbt_a2dp_acceptor_media_begin(dbt_a2dp_acceptor_t *p,const dbt_a2dp_channel_t *signal,
    uint32_t now,dbt_a2dp_channel_t *key){
    if(!p || !p->runtime || !signal || !key || overlap(p,sizeof(*p),key,sizeof(*key)) ||
       overlap(p->runtime,sizeof(*p->runtime),key,sizeof(*key)) || overlap(signal,sizeof(*signal),key,sizeof(*key)))return DBT_INVALID;
    dbt_result_t ready=dbt_a2dp_acceptor_connect_status(p,signal);if(ready!=DBT_OK)return ready==DBT_PENDING?DBT_BUSY:ready;
    if(!p->enabled || p->sink.state!=DBT_STREAM_OPEN || !p->sink.media_initiator || !p->sink.worker_live || p->sink.pending.used ||
       p->sink.local || p->sink.media_binding || p->media.serial || p->close_media)return DBT_BUSY;
    if((int32_t)(now-p->sink.media_deadline)>=0)return DBT_INVALID;
    for(unsigned i=0;i<DBT_CHANNEL_LIMIT;++i){
        const dbt_channel_t *channel=&p->runtime->channels.channels[i];
        if(channel->state!=DBT_CH_FREE && channel->psm==0x19 && !matches(&p->signaling,channel))return DBT_BUSY;
    }
    uint16_t cid;dbt_result_t result=dbt_channels_open(&p->runtime->channels,p->signaling.handle,0x19,
        p->mtu,335,DBT_SECURITY_ENCRYPTED,now,&cid);
    if(result!=DBT_OK)return result;
    const dbt_channel_t *channel=dbt_channels_find(&p->runtime->channels,p->signaling.handle,cid);
    if(!channel || !channel->serial || channel->link_generation!=p->signaling.generation){
        dbt_runtime_abort(p->runtime,DBT_FAULT_BAD_RESPONSE);return DBT_INVALID;
    }
    remember(&p->media,channel);*key=p->media;return DBT_PENDING;
}
dbt_result_t dbt_a2dp_acceptor_media_status(const dbt_a2dp_acceptor_t *p,const dbt_a2dp_channel_t *key){
    if(!p || !p->runtime || !key || !key->serial || !key->generation)return DBT_INVALID;
    const dbt_channel_t *channel=live(p,key);if(!channel || channel->psm!=0x19 || !matches(&p->media,channel))return DBT_EMPTY;
    if(p->runtime->host.state==DBT_HOST_FAILED)return DBT_INVALID;
    if(p->draining || p->close_media || channel->state!=DBT_CH_OPEN || channel->app_close_requested)return DBT_PENDING;
    return p->active && p->sink.media_binding==channel->serial && !p->sink.failed?DBT_OK:DBT_INVALID;
}
dbt_result_t dbt_a2dp_acceptor_media_cancel(dbt_a2dp_acceptor_t *p,const dbt_a2dp_channel_t *key,uint32_t now){
    if(!p || !p->runtime || !key || !key->serial || !key->generation)return DBT_INVALID;
    const dbt_channel_t *channel=live(p,key);if(!channel || !matches(&p->media,channel))return DBT_EMPTY;
    (void)now;p->draining=true;return DBT_PENDING;
}
static void close_all(dbt_a2dp_acceptor_t *p,uint32_t now){
    for(unsigned i=0;i<DBT_CHANNEL_LIMIT;++i){
        const dbt_channel_t *channel=&p->runtime->channels.channels[i];
        if(channel->state!=DBT_CH_FREE && channel->state!=DBT_CH_DISCONNECTING && channel->psm==0x19)
            (void)dbt_channels_close(&p->runtime->channels,channel->handle,channel->local_cid,now);
    }
}
dbt_result_t dbt_a2dp_acceptor_poll(dbt_a2dp_acceptor_t *p,uint32_t now){
    if(!p || !p->runtime)return DBT_INVALID;
    if(!p->enabled || p->draining)close_all(p,now);
    if(!p->active){
        if(p->draining && !staged(p))p->draining=false;
        return DBT_OK;
    }
    if(p->runtime->host.state==DBT_HOST_FAILED)p->draining=true;
    if(!p->draining){
        if(dbt_avdtp_sink_poll(&p->sink,now)!=DBT_OK || !p->bearer.attached ||
           dbt_avdtp_bearer_poll(&p->bearer,now)!=DBT_OK)p->draining=true;
        if(p->close_media)close_channel(p,&p->media,now);
    }
    if(p->draining){
        close_channel(p,&p->media,now);close_channel(p,&p->signaling,now);
        if(!p->signaling.serial && !p->media.serial && !p->bearer.active && !staged(p)){
            if(p->bearer.attached)(void)dbt_avdtp_bearer_clear(&p->bearer);
            dbt_result_t result=dbt_avdtp_sink_shutdown(&p->sink,now);
            if(result==DBT_OK){p->active=p->draining=p->close_media=false;return DBT_OK;}
            (void)dbt_avdtp_sink_poll(&p->sink,now); /* retiring worker drain */
        }
    }
    return DBT_OK;
}
uint32_t dbt_a2dp_acceptor_wait_ms(const dbt_a2dp_acceptor_t *p,uint32_t now){
    if(!p)return UINT32_MAX;
    if(!p->active){
        if(!p->enabled || p->draining)for(unsigned i=0;i<DBT_CHANNEL_LIMIT;++i){
            const dbt_channel_t *channel=&p->runtime->channels.channels[i];
            if(channel->psm==0x19 && dbt_channels_close_ready(&p->runtime->channels,channel->handle,channel->local_cid))return 0;
        }
        return UINT32_MAX;
    }
    if(p->draining){
        const dbt_channel_t *signal=live(p,&p->signaling),*media_channel=live(p,&p->media);
        if((p->signaling.serial && !signal) || (p->media.serial && !media_channel) ||
            (signal && dbt_channels_close_ready(&p->runtime->channels,signal->handle,signal->local_cid)) ||
            (media_channel && dbt_channels_close_ready(&p->runtime->channels,media_channel->handle,media_channel->local_cid)))return 0;
        if(!p->signaling.serial && !p->media.serial && (!p->sink.retiring || p->sink.drained) && !staged(p))return 0;
        return p->sink.retiring?dbt_avdtp_sink_wait_ms(&p->sink,now):UINT32_MAX;
    }
    if(p->close_media){const dbt_channel_t *channel=live(p,&p->media);if(channel && dbt_channels_close_ready(&p->runtime->channels,channel->handle,channel->local_cid))return 0;}
    uint32_t wait=dbt_avdtp_sink_wait_ms(&p->sink,now),signal=dbt_avdtp_bearer_wait_ms(&p->bearer,now);
    return signal<wait?signal:wait;
}
dbt_result_t dbt_a2dp_acceptor_complete(dbt_a2dp_acceptor_t *p,uint32_t binding,uint32_t token,
    dbt_sink_work_result_t result,uint32_t now){
    if(!p || !p->active)return DBT_EMPTY;
    return dbt_avdtp_sink_complete(&p->sink,binding,token,result,now);
}
dbt_result_t dbt_a2dp_acceptor_retire(dbt_a2dp_acceptor_t *p,uint32_t now){
    if(!p || !p->runtime)return DBT_INVALID;
    (void)now;
    p->draining=p->active || staged(p);
    return p->draining?DBT_BUSY:DBT_OK;
}
dbt_result_t dbt_a2dp_acceptor_stop(dbt_a2dp_acceptor_t *p,uint32_t now){
    if(!p || !p->runtime)return DBT_INVALID;
    p->enabled=false;(void)dbt_a2dp_acceptor_retire(p,now);
    (void)dbt_a2dp_acceptor_poll(p,now);return p->active || staged(p)?DBT_BUSY:DBT_OK;
}
