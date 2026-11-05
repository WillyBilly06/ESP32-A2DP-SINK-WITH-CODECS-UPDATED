#include "dbt_sdp_client_bearer.h"

static bool key(const dbt_sdp_client_bearer_t *b,const dbt_channel_t *channel){
    return channel && b->serial && channel->psm==1 && channel->serial==b->serial &&
        channel->handle==b->handle && channel->local_cid==b->cid && channel->link_generation==b->generation;
}
static const dbt_channel_t *current(const dbt_sdp_client_bearer_t *b){
    const dbt_channel_t *channel=dbt_channels_find(&b->runtime->channels,b->handle,b->cid);
    return key(b,channel)?channel:0;
}
static void sent(void *context,dbt_tx_outcome_t outcome){
    dbt_sdp_client_bearer_t *b=context;b->active=false;
    dbt_sdp_client_tx_outcome_t result=outcome==DBT_TX_ADMITTED?DBT_SDP_TX_ADMITTED:
        outcome==DBT_TX_LINK_LOST?DBT_SDP_TX_LINK_LOST:DBT_SDP_TX_ABORTED;
    (void)dbt_sdp_client_tx_done(b->client,b->serial,b->transaction,result,b->runtime->now_ms);
}
dbt_result_t dbt_sdp_client_bearer_init(dbt_sdp_client_bearer_t *b,dbt_runtime_t *runtime,
                                      dbt_sdp_client_t *client,const dbt_channel_t *channel){
    if(!b || !runtime || !client || !channel || !runtime->ready ||
       runtime->host.state!=DBT_HOST_IDENTIFIED || channel->psm!=1 || channel->state!=DBT_CH_OPEN ||
       !channel->serial || client->config.binding!=channel->serial || client->active || client->pinned ||
       client->result.must_close || client->config.rx_mtu!=channel->rx_mtu || client->config.tx_mtu!=channel->tx_mtu)return DBT_INVALID;
    const dbt_channel_t *live=dbt_channels_find(&runtime->channels,channel->handle,channel->local_cid);
    const dbt_link_t *link=dbt_acl_find(&runtime->acl,channel->handle);
    if(!live || live->serial!=channel->serial || live->link_generation!=channel->link_generation ||
       live->state!=DBT_CH_OPEN || live->config_epoch!=channel->config_epoch ||
       !link || link->kind!=DBT_LINK_CLASSIC || link->generation!=channel->link_generation)return DBT_INVALID;
    *b=(dbt_sdp_client_bearer_t){0};b->runtime=runtime;b->client=client;
    b->serial=channel->serial;b->generation=channel->link_generation;b->handle=channel->handle;
    b->cid=channel->local_cid;b->config_epoch=channel->config_epoch;b->attached=true;return DBT_OK;
}
bool dbt_sdp_client_bearer_matches(const dbt_sdp_client_bearer_t *b,const dbt_channel_t *channel){
    return b && b->runtime && b->client && b->attached && key(b,channel);
}
dbt_result_t dbt_sdp_client_bearer_receive(dbt_sdp_client_bearer_t *b,const dbt_channel_t *channel,
                                         const uint8_t *data,uint16_t size,uint32_t now){
    if(!dbt_sdp_client_bearer_matches(b,channel))return DBT_UNSUPPORTED;
    if(b->closing)return DBT_EMPTY;
    const dbt_channel_t *live=current(b);
    if(!live || live->state!=DBT_CH_OPEN || live->config_epoch!=b->config_epoch || size>live->rx_mtu){
        b->closing=true;dbt_sdp_client_abort(b->client,b->serial);return DBT_INVALID;
    }
    dbt_result_t result=dbt_sdp_client_receive(b->client,b->serial,data,size,now);
    if(b->client->result.must_close)b->closing=true;
    return result;
}
void dbt_sdp_client_bearer_event(dbt_sdp_client_bearer_t *b,dbt_channel_event_t event,const dbt_channel_t *channel){
    if(!b || !b->runtime || !key(b,channel))return;
    if(event==DBT_CH_EVENT_CLOSED || event==DBT_CH_EVENT_FAILED){
        b->attached=false;b->closing=true;dbt_sdp_client_abort(b->client,b->serial);
    }else if(event==DBT_CH_EVENT_RECONFIGURED &&
             (channel->config_epoch!=b->config_epoch || channel->rx_mtu!=b->client->config.rx_mtu || channel->tx_mtu!=b->client->config.tx_mtu)){
        b->closing=true;dbt_sdp_client_abort(b->client,b->serial);
    }
}
static bool tx_capacity(const dbt_sdp_client_bearer_t *b){
    for(unsigned i=0;i<DBT_MAX_LINKS;++i){
        const dbt_link_t *link=&b->runtime->acl.links[i];
        if(link->used && link->handle==b->handle && link->generation==b->generation)return !b->runtime->tx[i].active;
    }
    return false;
}
dbt_result_t dbt_sdp_client_bearer_poll(dbt_sdp_client_bearer_t *b,uint32_t now){
    if(!b || !b->runtime || !b->client)return DBT_INVALID;
    b->send_blocked=false;
    dbt_sdp_client_poll(b->client,now);
    const dbt_channel_t *channel=current(b);
    if(!channel || b->runtime->host.state==DBT_HOST_FAILED){
        b->attached=false;b->closing=true;dbt_sdp_client_abort(b->client,b->serial);return DBT_OK;
    }
    if(channel->config_epoch!=b->config_epoch || b->client->result.must_close || channel->state!=DBT_CH_OPEN)
        b->closing=true;
    if(b->closing){
        dbt_sdp_client_abort(b->client,b->serial);
        if(dbt_channels_close_ready(&b->runtime->channels,b->handle,b->cid))
            (void)dbt_channels_close(&b->runtime->channels,b->handle,b->cid,now);
        return DBT_OK;
    }
    if(b->active || !dbt_sdp_client_tx_ready(b->client) || !tx_capacity(b))return DBT_OK;
    const uint8_t *data;uint16_t size,transaction;
    dbt_result_t result=dbt_sdp_client_tx_peek(b->client,now,&data,&size,&transaction);
    if(result!=DBT_OK)return result==DBT_EMPTY?DBT_OK:result;
    b->transaction=transaction;
    result=dbt_runtime_send(b->runtime,b->handle,b->cid,data,size,sent,b);
    if(result==DBT_OK)b->active=true;
    else{
        (void)dbt_sdp_client_tx_done(b->client,b->serial,transaction,DBT_SDP_TX_UNSENT,now);
        if(result==DBT_BUSY)b->send_blocked=true;
        else{b->closing=true;dbt_sdp_client_abort(b->client,b->serial);}
    }
    return DBT_OK;
}
uint32_t dbt_sdp_client_bearer_wait_ms(const dbt_sdp_client_bearer_t *b,uint32_t now){
    if(!b || !b->runtime || !b->client)return UINT32_MAX;
    uint32_t wait=dbt_sdp_client_wait_ms(b->client,now);
    const dbt_channel_t *channel=current(b);
    if(b->attached && (!channel || b->runtime->host.state==DBT_HOST_FAILED))return 0;
    if(b->attached && channel && channel->config_epoch!=b->config_epoch && !b->closing)return 0;
    if(b->attached && channel && (b->closing || b->client->result.must_close) &&
       dbt_channels_close_ready(&b->runtime->channels,b->handle,b->cid))return 0;
    if(b->attached && !b->closing && !b->client->result.must_close && channel && channel->state==DBT_CH_OPEN &&
       !b->active && !b->send_blocked && dbt_sdp_client_tx_ready(b->client) && tx_capacity(b))return 0;
    return wait;
}
dbt_result_t dbt_sdp_client_bearer_close(dbt_sdp_client_bearer_t *b){
    if(!b || !b->runtime || !b->client)return DBT_INVALID;
    b->closing=true;dbt_sdp_client_abort(b->client,b->serial);
    return b->active || b->attached?DBT_PENDING:DBT_OK;
}
dbt_result_t dbt_sdp_client_bearer_clear(dbt_sdp_client_bearer_t *b){
    if(!b)return DBT_INVALID;
    if(b->active || b->attached || (b->client && b->client->pinned))return DBT_BUSY;
    *b=(dbt_sdp_client_bearer_t){0};return DBT_OK;
}
