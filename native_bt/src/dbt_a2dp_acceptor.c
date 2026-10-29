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
        !p->media.serial && !p->sink.failed;
}
dbt_result_t dbt_a2dp_acceptor_event(dbt_a2dp_acceptor_t *p,dbt_channel_event_t kind,const dbt_channel_t *channel){
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
        if(p->signaling.handle!=channel->handle || p->signaling.generation!=channel->link_generation || p->media.serial)return DBT_INVALID;
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
            dbt_result_t result=dbt_avdtp_sink_media_closed(&p->sink,channel->serial);
            zero(&p->media,sizeof(p->media));p->close_media=false;
            if(result!=DBT_OK)p->draining=true;
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
dbt_result_t dbt_a2dp_acceptor_receive(dbt_a2dp_acceptor_t *p,const dbt_channel_t *channel,const uint8_t *data,uint16_t size){
    if(!p || !p->runtime || !channel)return DBT_INVALID;
    if(channel->psm!=0x19)return DBT_UNSUPPORTED;
    if(!p->active || p->draining)return DBT_BUSY;
    if(matches(&p->signaling,channel)){
        dbt_result_t result=dbt_avdtp_bearer_receive(&p->bearer,channel,data,size);
        if(result!=DBT_OK && result!=DBT_EMPTY)p->draining=true;
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
    if(channel->state==DBT_CH_OPEN)(void)dbt_channels_close(&p->runtime->channels,key->handle,key->cid,now);
}
static bool staged(const dbt_a2dp_acceptor_t *p){
    for(unsigned i=0;i<DBT_CHANNEL_LIMIT;++i)
        if(p->runtime->channels.channels[i].state!=DBT_CH_FREE && p->runtime->channels.channels[i].psm==0x19)return true;
    return false;
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
    if(!p->active)return DBT_OK;
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
        if(!p->enabled)for(unsigned i=0;i<DBT_CHANNEL_LIMIT;++i){
            const dbt_channel_t *channel=&p->runtime->channels.channels[i];
            if(channel->psm==0x19 && channel->state!=DBT_CH_FREE && channel->state!=DBT_CH_DISCONNECTING)return 0;
        }
        return UINT32_MAX;
    }
    if(p->draining){
        const dbt_channel_t *signal=live(p,&p->signaling),*media_channel=live(p,&p->media);
        if((p->signaling.serial && !signal) || (p->media.serial && !media_channel) ||
            (signal && signal->state==DBT_CH_OPEN) || (media_channel && media_channel->state==DBT_CH_OPEN))return 0;
        if(!p->signaling.serial && !p->media.serial && !p->sink.retiring && !staged(p))return 0;
        return p->sink.retiring?dbt_avdtp_sink_wait_ms(&p->sink,now):UINT32_MAX;
    }
    if(p->close_media){const dbt_channel_t *channel=live(p,&p->media);if(channel && channel->state==DBT_CH_OPEN)return 0;}
    uint32_t wait=dbt_avdtp_sink_wait_ms(&p->sink,now),signal=dbt_avdtp_bearer_wait_ms(&p->bearer,now);
    return signal<wait?signal:wait;
}
dbt_result_t dbt_a2dp_acceptor_complete(dbt_a2dp_acceptor_t *p,uint32_t binding,uint32_t token,
    dbt_sink_work_result_t result,uint32_t now){
    if(!p || !p->active)return DBT_EMPTY;
    return dbt_avdtp_sink_complete(&p->sink,binding,token,result,now);
}
dbt_result_t dbt_a2dp_acceptor_stop(dbt_a2dp_acceptor_t *p,uint32_t now){
    if(!p || !p->runtime)return DBT_INVALID;
    p->enabled=false;
    if(!p->active){(void)dbt_a2dp_acceptor_poll(p,now);return staged(p)?DBT_BUSY:DBT_OK;}
    p->draining=true;(void)dbt_a2dp_acceptor_poll(p,now);return p->active?DBT_BUSY:DBT_OK;
}
