#include "dbt_sdp_discovery.h"
#include <string.h>

enum { DISC_OPENING=1,DISC_QUERY,DISC_CLOSING,DISC_DONE };
static bool due(uint32_t now,uint32_t deadline){return (int32_t)(now-deadline)>=0;}
static bool overlap(const void *a,uint32_t an,const void *b,uint32_t bn){
    uintptr_t x=(uintptr_t)a,y=(uintptr_t)b;return an && bn && (x<=y?y-x<an:x-y<bn);
}
bool dbt_sdp_discovery_matches(const dbt_sdp_discovery_t *d,const dbt_channel_t *channel){
    return d && d->runtime && d->state && channel && channel->psm==1 && channel->serial==d->serial &&
        channel->handle==d->handle && channel->local_cid==d->cid && channel->link_generation==d->generation;
}
static const dbt_channel_t *current(const dbt_sdp_discovery_t *d){
    const dbt_channel_t *channel=dbt_channels_find(&d->runtime->channels,d->handle,d->cid);
    return dbt_sdp_discovery_matches(d,channel)?channel:0;
}
static void closing(dbt_sdp_discovery_t *d,dbt_sdp_discovery_outcome_t outcome){
    d->outcome=(uint8_t)outcome;d->state=DISC_CLOSING;
}
void dbt_sdp_discovery_init(dbt_sdp_discovery_t *d){if(d)*d=(dbt_sdp_discovery_t){0};}
dbt_result_t dbt_sdp_discovery_begin(dbt_sdp_discovery_t *d,dbt_runtime_t *runtime,const dbt_link_t *link,
    dbt_sdp_discovery_config_t config,uint32_t now,uint32_t *token){
    if(!d || !runtime || !link || !token || !runtime->ready || runtime->host.state!=DBT_HOST_IDENTIFIED ||
       !config.request || !config.response || config.request_capacity<46 || config.request_capacity>DBT_SDP_RECORD_BYTES ||
       config.response_capacity<2 || config.response_capacity>DBT_SDP_RECORD_BYTES || config.rx_mtu<48 ||
       config.rx_mtu>runtime->maximum_mtu || config.rx_mtu>DBT_SDP_RECORD_BYTES ||
       !config.phase_ms || config.phase_ms>60000u || !config.total_ms || config.total_ms>60000u ||
       overlap(d,sizeof(*d),config.request,config.request_capacity) || overlap(d,sizeof(*d),config.response,config.response_capacity) ||
       overlap(config.request,config.request_capacity,config.response,config.response_capacity))return DBT_INVALID;
    if(d->state && d->state!=DISC_DONE)return DBT_BUSY;
    const dbt_link_t *live=dbt_acl_find(&runtime->acl,link->handle);
    if(!live || live->kind!=DBT_LINK_CLASSIC || live->generation!=link->generation)return DBT_EMPTY;
    uint16_t handle=live->handle,cid;uint32_t generation=live->generation;
    dbt_result_t result=dbt_channels_open(&runtime->channels,handle,1,config.rx_mtu,48,DBT_SECURITY_NONE,now,&cid);
    if(result!=DBT_OK)return result;
    const dbt_channel_t *channel=dbt_channels_find(&runtime->channels,handle,cid);
    if(!channel || !channel->serial){dbt_runtime_abort(runtime,DBT_FAULT_BAD_RESPONSE);return DBT_INVALID;}
    *d=(dbt_sdp_discovery_t){0};d->runtime=runtime;d->config=config;d->handle=handle;d->generation=generation;
    d->cid=cid;d->serial=channel->serial;d->deadline=now+config.total_ms;d->state=DISC_OPENING;
    *token=d->serial;return DBT_OK;
}
static void opened(dbt_sdp_discovery_t *d,const dbt_channel_t *channel,uint32_t now){
    if(due(now,d->deadline)){d->timeout_seen=true;closing(d,DBT_SDP_DISCOVERY_TIMEOUT);return;}
    dbt_sdp_client_config_t config={d->serial,d->config.phase_ms,d->deadline-now,
        channel->rx_mtu,channel->tx_mtu,d->config.request_capacity,d->config.response_capacity,d->config.request,d->config.response};
    if(dbt_sdp_client_init(&d->client,config)!=DBT_OK ||
       dbt_sdp_client_bearer_init(&d->bearer,d->runtime,&d->client,channel)!=DBT_OK){closing(d,DBT_SDP_DISCOVERY_PROTOCOL_ERROR);return;}
    d->client_ready=true;
    uint8_t prefix[]={0x35,3,0x19,0x11,0x0a,0,0,0x35,15,
        9,0,0,9,0,1,9,0,4,9,0,9,9,3,0x11};
    uint16_t maximum=d->config.response_capacity<7?7:d->config.response_capacity;
    prefix[5]=(uint8_t)(maximum>>8);prefix[6]=(uint8_t)maximum;
    if(dbt_sdp_client_begin(&d->client,6,prefix,sizeof(prefix),now,&d->query)!=DBT_OK){closing(d,DBT_SDP_DISCOVERY_QUERY_ERROR);return;}
    d->state=DISC_QUERY;
}
void dbt_sdp_discovery_event(dbt_sdp_discovery_t *d,dbt_channel_event_t event,const dbt_channel_t *channel,
    dbt_channel_reason_t reason,uint16_t result,uint32_t now){
    if(!dbt_sdp_discovery_matches(d,channel))return;
    if(d->client_ready)dbt_sdp_client_bearer_event(&d->bearer,event,channel);
    if(event==DBT_CH_EVENT_OPEN && d->state==DISC_OPENING)opened(d,channel,now);
    if(event==DBT_CH_EVENT_CLOSED || event==DBT_CH_EVENT_FAILED){
        d->channel_reason=reason;d->channel_result=result;
        bool completed=d->client_ready && d->client.result.outcome!=DBT_SDP_CLIENT_NONE && reason!=DBT_CH_REASON_LINK_LOST;
        if(d->state!=DISC_CLOSING && d->state!=DISC_DONE && !(d->state==DISC_QUERY && completed))
            closing(d,DBT_SDP_DISCOVERY_CHANNEL_ERROR);
    }
}
dbt_result_t dbt_sdp_discovery_receive(dbt_sdp_discovery_t *d,const dbt_channel_t *channel,const uint8_t *data,uint16_t size,uint32_t now){
    if(!dbt_sdp_discovery_matches(d,channel))return DBT_UNSUPPORTED;
    if(d->state==DISC_CLOSING || d->state==DISC_DONE)return DBT_EMPTY;
    if(!d->client_ready){closing(d,DBT_SDP_DISCOVERY_PROTOCOL_ERROR);return DBT_INVALID;}
    return dbt_sdp_client_bearer_receive(&d->bearer,channel,data,size,now);
}
static void select_service(dbt_sdp_discovery_t *d,const uint8_t *data,uint16_t size){
    dbt_sdp_service_cursor_t cursor={0};dbt_sdp_av_service_t info;bool matched=false,selected=false;
    dbt_result_t result;
    while((result=dbt_sdp_av_service_next(data,size,true,0x110a,0x0019,0x110d,&cursor,&info))==DBT_OK){
        matched=true;
        if(info.psm!=0x0019 || (info.protocol_version>>8)!=1 || !(info.present&DBT_SDP_SERVICE_PROFILE) ||
           (info.profile_version>>8)!=1)continue;
        if(!selected || info.protocol_version>d->service.protocol_version ||
           (info.protocol_version==d->service.protocol_version && info.profile_version>d->service.profile_version)){
            d->service=info;selected=true;
        }
    }
    closing(d,result!=DBT_EMPTY?DBT_SDP_DISCOVERY_PROTOCOL_ERROR:selected?DBT_SDP_DISCOVERY_SUCCESS:
        matched?DBT_SDP_DISCOVERY_UNSUPPORTED:DBT_SDP_DISCOVERY_NOT_FOUND);
}
static dbt_sdp_discovery_outcome_t query_failure(dbt_sdp_client_outcome_t outcome){
    return outcome==DBT_SDP_CLIENT_TIMEOUT?DBT_SDP_DISCOVERY_TIMEOUT:
        outcome==DBT_SDP_CLIENT_CANCELLED?DBT_SDP_DISCOVERY_CANCELLED:DBT_SDP_DISCOVERY_QUERY_ERROR;
}
void dbt_sdp_discovery_poll(dbt_sdp_discovery_t *d,uint32_t now){
    if(!d || !d->runtime || !d->state || d->state==DISC_DONE)return;
    const dbt_channel_t *channel=current(d);
    if(!d->timeout_seen && due(now,d->deadline)){
        d->timeout_seen=true;closing(d,DBT_SDP_DISCOVERY_TIMEOUT);
        if(d->client_ready)(void)dbt_sdp_client_cancel(&d->client,d->query);
    }
    if(d->runtime->host.state==DBT_HOST_FAILED)closing(d,DBT_SDP_DISCOVERY_CHANNEL_ERROR);
    if(d->state==DISC_OPENING && channel && channel->state==DBT_CH_OPEN)opened(d,channel,now);
    if(d->state==DISC_QUERY){
        (void)dbt_sdp_client_bearer_poll(&d->bearer,now);
        dbt_sdp_client_result_t result={0};const uint8_t *data=0;
        dbt_result_t status=dbt_sdp_client_result(&d->client,d->query,&result,&data);
        if(status==DBT_OK){
            if(result.outcome==DBT_SDP_CLIENT_SUCCESS)select_service(d,data,result.bytes);
            else closing(d,query_failure(result.outcome));
        }else if(status!=DBT_PENDING)closing(d,DBT_SDP_DISCOVERY_PROTOCOL_ERROR);
        else if(result.must_close)closing(d,query_failure(result.outcome));
    }
    if(!channel && d->state!=DISC_CLOSING)closing(d,DBT_SDP_DISCOVERY_CHANNEL_ERROR);
    if(d->state==DISC_CLOSING){
        if(d->client_ready){
            (void)dbt_sdp_client_bearer_close(&d->bearer);
            (void)dbt_sdp_client_bearer_poll(&d->bearer,now);
            if(dbt_sdp_client_bearer_clear(&d->bearer)==DBT_OK)d->state=DISC_DONE;
        }else if(!channel)d->state=DISC_DONE;
        else if(dbt_channels_close_ready(&d->runtime->channels,d->handle,d->cid))
            (void)dbt_channels_close(&d->runtime->channels,d->handle,d->cid,now);
    }
}
uint32_t dbt_sdp_discovery_wait_ms(const dbt_sdp_discovery_t *d,uint32_t now){
    if(!d || !d->runtime || !d->state || d->state==DISC_DONE)return UINT32_MAX;
    uint32_t wait=d->timeout_seen?UINT32_MAX:due(now,d->deadline)?0:d->deadline-now;
    const dbt_channel_t *channel=current(d);
    if(d->state==DISC_QUERY && d->client_ready && (d->client.result.must_close ||
       (d->client.result.outcome!=DBT_SDP_CLIENT_NONE && !d->client.active && !d->client.pinned)))return 0;
    if(d->client_ready){uint32_t next=dbt_sdp_client_bearer_wait_ms(&d->bearer,now);if(next<wait)wait=next;}
    else if(!channel || (d->state==DISC_OPENING && channel->state==DBT_CH_OPEN) ||
            (d->state==DISC_CLOSING && dbt_channels_close_ready(&d->runtime->channels,d->handle,d->cid)))return 0;
    return wait;
}
dbt_result_t dbt_sdp_discovery_cancel(dbt_sdp_discovery_t *d,uint32_t token){
    if(!d || !token)return DBT_INVALID;
    if(!d->state || d->serial!=token)return DBT_EMPTY;
    if(d->state==DISC_DONE)return DBT_OK;
    d->cancel_requested=true;closing(d,DBT_SDP_DISCOVERY_CANCELLED);
    if(d->client_ready)(void)dbt_sdp_client_cancel(&d->client,d->query);
    return DBT_PENDING;
}
dbt_result_t dbt_sdp_discovery_result(const dbt_sdp_discovery_t *d,uint32_t token,dbt_sdp_discovery_result_t *result){
    if(!d || !token || !result)return DBT_INVALID;
    if(!d->state || d->serial!=token)return DBT_EMPTY;
    *result=(dbt_sdp_discovery_result_t){0};result->token=d->serial;result->generation=d->generation;result->handle=d->handle;
    result->outcome=(dbt_sdp_discovery_outcome_t)d->outcome;result->channel_reason=d->channel_reason;
    result->channel_result=d->channel_result;result->service=d->service;result->cancel_requested=d->cancel_requested;
    if(d->client_ready)result->query=d->client.result;
    return d->state==DISC_DONE?DBT_OK:DBT_PENDING;
}
