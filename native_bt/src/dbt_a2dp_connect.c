#include "dbt_a2dp_connect.h"
#include <string.h>

static bool due(uint32_t now,uint32_t deadline){return (int32_t)(now-deadline)>=0;}
static bool overlap(const void *a,uint32_t an,const void *b,uint32_t bn){
    uintptr_t x=(uintptr_t)a,y=(uintptr_t)b;return an && bn && (x<=y?y-x<an:x-y<bn);
}
static const dbt_link_t *peer(const dbt_a2dp_connect_t *c){
    const dbt_acl_t *acl=&c->profile->runtime->acl;
    for(unsigned i=0;i<DBT_MAX_LINKS;++i){const dbt_link_t *l=&acl->links[i];
        if(l->used && l->kind==DBT_LINK_CLASSIC && !memcmp(l->peer,c->config.peer.address_le,6))return l;}
    return 0;
}
static const dbt_link_t *current(const dbt_a2dp_connect_t *c){
    const dbt_link_t *l=dbt_acl_find(&c->profile->runtime->acl,c->handle);
    return l && l->kind==DBT_LINK_CLASSIC && l->generation==c->generation &&
        !memcmp(l->peer,c->config.peer.address_le,6)?l:0;
}
static bool capacity(const dbt_a2dp_connect_t *c){
    const dbt_channels_t *m=&c->profile->runtime->channels;
    if(m->tx_count==DBT_SIGNAL_TX_SLOTS)return false;
    for(unsigned i=0;i<DBT_CHANNEL_LIMIT;++i)if(m->channels[i].state==DBT_CH_FREE)return true;
    return false;
}
static void stop(dbt_a2dp_connect_t *c,dbt_a2dp_connect_outcome_t outcome){
    if(c->outcome==DBT_A2DP_CONNECT_NONE)c->outcome=(uint8_t)outcome;
    c->phase=DBT_A2DP_CONNECT_RETIRING;
}
static void invariant(dbt_a2dp_connect_t *c,dbt_result_t error){
    c->last_error=error;c->ownership_fault=true;stop(c,DBT_A2DP_CONNECT_INTERNAL_ERROR);
    /* A lost accepted token is an ownership fault, never permission to free
     * caller storage. Keep the retirement fences and fail the host closed. */
    if(c->profile->runtime->host.state!=DBT_HOST_FAILED)
        dbt_runtime_abort(c->profile->runtime,DBT_FAULT_BAD_RESPONSE);
}
void dbt_a2dp_connect_init(dbt_a2dp_connect_t *c){if(c)*c=(dbt_a2dp_connect_t){0};}
bool dbt_a2dp_connect_done(const dbt_a2dp_connect_t *c){return c && c->phase==DBT_A2DP_CONNECT_DONE;}
static bool config_valid(const dbt_a2dp_connect_t *c,const dbt_a2dp_acceptor_t *p,
                         const dbt_gap_t *gap,dbt_a2dp_connect_config_t v,const uint32_t *token){
    const dbt_sdp_discovery_config_t *d=&v.discovery;
    return c && p && p->runtime && gap && token && gap->host==&p->runtime->host &&
        d->request && d->response && d->request_capacity>=46 && d->request_capacity<=DBT_SDP_RECORD_BYTES &&
        d->response_capacity>=2 && d->response_capacity<=DBT_SDP_RECORD_BYTES &&
        d->rx_mtu>=48 && d->rx_mtu<=p->runtime->maximum_mtu && d->rx_mtu<=DBT_SDP_RECORD_BYTES &&
        d->phase_ms && d->phase_ms<=60000u && d->total_ms && d->total_ms<=60000u &&
        memcmp(v.peer.address_le,p->runtime->host.controller.address_le,6)!=0 &&
        v.peer.page_repetition<=2 && v.page_ms && v.page_ms<=60000u && v.page_retire_ms &&
        v.page_retire_ms<=DBT_GAP_PAGE_RETIRE_MAX_MS && v.total_ms && v.total_ms<=180000u &&
        !overlap(c,sizeof(*c),token,sizeof(*token)) &&
        !overlap(c,sizeof(*c),d->request,d->request_capacity) &&
        !overlap(c,sizeof(*c),d->response,d->response_capacity) &&
        !overlap(d->request,d->request_capacity,d->response,d->response_capacity) &&
        !overlap(token,sizeof(*token),d->request,d->request_capacity) &&
        !overlap(token,sizeof(*token),d->response,d->response_capacity) &&
        !overlap(c,sizeof(*c),p,sizeof(*p)) && !overlap(c,sizeof(*c),gap,sizeof(*gap)) &&
        !overlap(c,sizeof(*c),p->runtime,sizeof(*p->runtime)) &&
        !overlap(d->request,d->request_capacity,p,sizeof(*p)) &&
        !overlap(d->response,d->response_capacity,p,sizeof(*p)) &&
        !overlap(d->request,d->request_capacity,gap,sizeof(*gap)) &&
        !overlap(d->response,d->response_capacity,gap,sizeof(*gap)) &&
        !overlap(d->request,d->request_capacity,p->runtime,sizeof(*p->runtime)) &&
        !overlap(d->response,d->response_capacity,p->runtime,sizeof(*p->runtime));
}
dbt_result_t dbt_a2dp_connect_begin(dbt_a2dp_connect_t *c,dbt_a2dp_acceptor_t *p,dbt_gap_t *gap,
    dbt_a2dp_connect_config_t config,uint32_t now,uint32_t *token){
    if(!config_valid(c,p,gap,config,token))return DBT_INVALID;
    if(c->phase && c->phase!=DBT_A2DP_CONNECT_DONE)return DBT_BUSY;
    if(!p->runtime->ready || p->runtime->host.state!=DBT_HOST_IDENTIFIED || gap->state!=DBT_GAP_READY ||
       !p->enabled || p->active || p->draining)return DBT_BUSY;
    for(unsigned i=0;i<DBT_CHANNEL_LIMIT;++i)if(p->runtime->channels.channels[i].state!=DBT_CH_FREE &&
        p->runtime->channels.channels[i].psm==0x19)return DBT_BUSY;
    if(c->serial==UINT32_MAX)return DBT_NO_SPACE;
    uint32_t serial=c->serial;
    *c=(dbt_a2dp_connect_t){0};c->serial=serial;c->profile=p;c->gap=gap;c->config=config;
    c->deadline=now+config.total_ms;
    if(dbt_page_init(&c->page,&p->runtime->host)!=DBT_OK)return DBT_INVALID;
    dbt_sdp_discovery_init(&c->discovery);
    const dbt_link_t *l=peer(c);
    if(l){c->handle=l->handle;c->generation=l->generation;c->used_existing_acl=true;c->phase=DBT_A2DP_CONNECT_DISCOVERY_BEGIN;}
    else {
        dbt_result_t result=dbt_gap_connect(gap,&c->page,&p->runtime->acl,&config.peer,now,
                                            config.page_ms,config.page_retire_ms,&c->page_token);
        if(result!=DBT_OK)return result;
        c->phase=DBT_A2DP_CONNECT_PAGING;
    }
    c->serial=serial+1u;*token=c->serial;return DBT_PENDING;
}
void dbt_a2dp_connect_event(dbt_a2dp_connect_t *c,dbt_channel_event_t event,const dbt_channel_t *ch,
    dbt_channel_reason_t reason,uint16_t result,uint32_t now){
    if(!c || !c->phase || !ch || !c->signaling.serial || ch->psm!=0x19 ||
       ch->serial!=c->signaling.serial || ch->link_generation!=c->signaling.generation ||
       ch->handle!=c->signaling.handle || ch->local_cid!=c->signaling.cid)return;
    c->signaling_event=event;c->signaling_reason=reason;c->signaling_result=result;
    c->signaling_event_ms=now;c->signaling_event_seen=true;
}
static bool signal_retired(const dbt_a2dp_connect_t *c){
    if(!c->owns_signaling || !c->signaling.serial)return true;
    if(dbt_a2dp_acceptor_connect_status(c->profile,&c->signaling)!=DBT_EMPTY)return false;
    return !c->profile->active || c->profile->sink.binding!=c->signaling.serial;
}
static void retire(dbt_a2dp_connect_t *c,uint32_t now){
    if(c->page_token){
        if(!c->page_cancelled){
            dbt_result_t r=dbt_gap_connect_cancel(c->gap,c->page_token,now);
            if(r!=DBT_OK && r!=DBT_PENDING){invariant(c,r);return;}
            c->page_cancelled=true;
        }
        dbt_result_t r=dbt_gap_connect_result(c->gap,c->page_token,&c->page_result);
        if(r==DBT_OK){c->page_token=0;if(c->page_result.outcome==DBT_PAGE_CONNECTED){
            c->handle=c->page_result.handle;c->generation=c->page_result.generation;}}
        else if(r!=DBT_PENDING){invariant(c,r);return;}
    }
    if(c->discovery_token && !c->discovery_done){
        if(!c->discovery_cancelled){
            dbt_result_t r=dbt_sdp_discovery_cancel(&c->discovery,c->discovery_token);
            if(r!=DBT_OK && r!=DBT_PENDING){invariant(c,r);return;}
            c->discovery_cancelled=true;
        }
        dbt_sdp_discovery_poll(&c->discovery,now);
        dbt_sdp_discovery_result_t result;
        dbt_result_t r=dbt_sdp_discovery_result(&c->discovery,c->discovery_token,&result);
        if(r==DBT_OK)c->discovery_done=true;
        else if(r!=DBT_PENDING){invariant(c,r);return;}
    }
    if(c->owns_signaling && !c->signal_cancelled){
        dbt_result_t r=dbt_a2dp_acceptor_connect_cancel(c->profile,&c->signaling,now);
        if(r!=DBT_OK && r!=DBT_PENDING && r!=DBT_EMPTY){invariant(c,r);return;}
        c->signal_cancelled=true;
    }
    if(!c->page_token && (!c->discovery_token || c->discovery_done) && signal_retired(c))
        c->phase=DBT_A2DP_CONNECT_DONE;
}
static void signal_begin(dbt_a2dp_connect_t *c,uint32_t now){
    const dbt_link_t *l=current(c);if(!l){stop(c,DBT_A2DP_CONNECT_LINK_LOST);return;}
    const dbt_a2dp_acceptor_t *p=c->profile;
    /* Incoming setup may have won while SDP was running. Observe the exact
     * same-peer channel, but never claim cancellation ownership of it. */
    for(unsigned i=0;i<DBT_CHANNEL_LIMIT;++i){const dbt_channel_t *ch=&p->runtime->channels.channels[i];
        if(ch->state==DBT_CH_FREE || ch->psm!=0x19)continue;
        if(ch->handle!=c->handle || ch->link_generation!=c->generation || p->draining){
            stop(c,DBT_A2DP_CONNECT_PROFILE_BUSY);return;}
        c->signaling=(dbt_a2dp_channel_t){ch->serial,ch->link_generation,ch->handle,ch->local_cid,ch->rx_mtu,ch->tx_mtu};
        if(p->active)c->signaling=p->signaling;
        c->phase=DBT_A2DP_CONNECT_SIGNALING;return;
    }
    if(!p->enabled || p->active || p->draining){stop(c,DBT_A2DP_CONNECT_PROFILE_BUSY);return;}
    if(!capacity(c))return;
    dbt_result_t r=dbt_a2dp_acceptor_connect_begin(c->profile,l,now,&c->signaling);
    if(r==DBT_PENDING){c->owns_signaling=true;c->phase=DBT_A2DP_CONNECT_SIGNALING;}
    else {c->last_error=r;stop(c,DBT_A2DP_CONNECT_SIGNAL_ERROR);}
}
void dbt_a2dp_connect_poll(dbt_a2dp_connect_t *c,uint32_t now){
    if(!c || !c->profile || !c->phase || dbt_a2dp_connect_done(c) || c->ownership_fault)return;
    if(!c->expired && due(now,c->deadline)){c->expired=true;stop(c,DBT_A2DP_CONNECT_TIMEOUT);}
    if(c->profile->runtime->host.state==DBT_HOST_FAILED)stop(c,DBT_A2DP_CONNECT_INTERNAL_ERROR);
    switch(c->phase){
    case DBT_A2DP_CONNECT_PAGING:{
        dbt_result_t r=dbt_gap_connect_result(c->gap,c->page_token,&c->page_result);
        if(r==DBT_OK){c->page_token=0;
            if(c->page_result.outcome!=DBT_PAGE_CONNECTED)stop(c,DBT_A2DP_CONNECT_PAGE_ERROR);
            else {c->handle=c->page_result.handle;c->generation=c->page_result.generation;c->phase=DBT_A2DP_CONNECT_DISCOVERY_BEGIN;}}
        else if(r!=DBT_PENDING)invariant(c,r);
        break;}
    case DBT_A2DP_CONNECT_DISCOVERY_BEGIN:{
        const dbt_link_t *l=current(c);if(!l){stop(c,DBT_A2DP_CONNECT_LINK_LOST);break;}
        if(!capacity(c))break;
        dbt_result_t r=dbt_sdp_discovery_begin(&c->discovery,c->profile->runtime,l,c->config.discovery,now,&c->discovery_token);
        if(r==DBT_OK)c->phase=DBT_A2DP_CONNECT_DISCOVERING;
        else {c->last_error=r;stop(c,DBT_A2DP_CONNECT_DISCOVERY_ERROR);}
        break;}
    case DBT_A2DP_CONNECT_DISCOVERING:{
        dbt_sdp_discovery_poll(&c->discovery,now);dbt_sdp_discovery_result_t result;
        dbt_result_t r=dbt_sdp_discovery_result(&c->discovery,c->discovery_token,&result);
        if(r==DBT_OK){c->discovery_done=true;
            if(result.outcome==DBT_SDP_DISCOVERY_SUCCESS)c->phase=DBT_A2DP_CONNECT_SIGNAL_BEGIN;
            else stop(c,DBT_A2DP_CONNECT_DISCOVERY_ERROR);}
        else if(r!=DBT_PENDING)invariant(c,r);
        break;}
    case DBT_A2DP_CONNECT_SIGNAL_BEGIN:signal_begin(c,now);break;
    case DBT_A2DP_CONNECT_SIGNALING:{
        dbt_result_t r=dbt_a2dp_acceptor_connect_status(c->profile,&c->signaling);
        if(r==DBT_OK){c->outcome=DBT_A2DP_CONNECT_SIGNALING_READY;c->phase=DBT_A2DP_CONNECT_DONE;}
        else if(r!=DBT_PENDING){c->last_error=r;stop(c,DBT_A2DP_CONNECT_SIGNAL_ERROR);}
        break;}
    case DBT_A2DP_CONNECT_RETIRING:retire(c,now);break;
    default:invariant(c,DBT_INVALID);break;
    }
}
dbt_result_t dbt_a2dp_connect_cancel(dbt_a2dp_connect_t *c,uint32_t token){
    if(!c || !token)return DBT_INVALID;
    if(!c->phase || c->serial!=token)return DBT_EMPTY;
    c->cancel_requested=true;
    if(dbt_a2dp_connect_done(c))return DBT_OK; /* A completed connection may win. */
    stop(c,DBT_A2DP_CONNECT_CANCELLED);return DBT_PENDING;
}
uint32_t dbt_a2dp_connect_wait_ms(const dbt_a2dp_connect_t *c,uint32_t now){
    if(!c || !c->profile || !c->phase || dbt_a2dp_connect_done(c) || c->ownership_fault)return UINT32_MAX;
    uint32_t wait=c->expired?UINT32_MAX:due(now,c->deadline)?0:c->deadline-now;
    if(c->phase==DBT_A2DP_CONNECT_PAGING || (c->phase==DBT_A2DP_CONNECT_RETIRING && c->page_token)){
        if(dbt_gap_connect_ready(c->gap,c->page_token))return 0;
        uint32_t n=dbt_gap_wait_ms(c->gap);if(n<wait)wait=n;
    }
    if(c->phase==DBT_A2DP_CONNECT_DISCOVERY_BEGIN && (!current(c) || capacity(c)))return 0;
    if(c->phase==DBT_A2DP_CONNECT_SIGNAL_BEGIN){
        if(!current(c) || !c->profile->enabled || c->profile->active || c->profile->draining || capacity(c))return 0;
        for(unsigned i=0;i<DBT_CHANNEL_LIMIT;++i)if(c->profile->runtime->channels.channels[i].state!=DBT_CH_FREE &&
            c->profile->runtime->channels.channels[i].psm==0x19)return 0;
        return wait;
    }
    if(c->phase==DBT_A2DP_CONNECT_DISCOVERING ||
       (c->phase==DBT_A2DP_CONNECT_RETIRING && c->discovery_token && !c->discovery_done)){
        uint32_t n=dbt_sdp_discovery_wait_ms(&c->discovery,now);if(n<wait)wait=n;
    }
    if(c->phase==DBT_A2DP_CONNECT_SIGNALING &&
       dbt_a2dp_acceptor_connect_status(c->profile,&c->signaling)!=DBT_PENDING)return 0;
    if(c->phase==DBT_A2DP_CONNECT_RETIRING){
        if((c->page_token && !c->page_cancelled) ||
           (c->discovery_token && !c->discovery_done && !c->discovery_cancelled) ||
           (c->owns_signaling && !c->signal_cancelled))return 0;
        if(!c->page_token && (!c->discovery_token || c->discovery_done) && signal_retired(c))return 0;
        uint32_t n=dbt_a2dp_acceptor_wait_ms(c->profile,now);if(n<wait)wait=n;
    }
    return wait;
}
dbt_result_t dbt_a2dp_connect_result(const dbt_a2dp_connect_t *c,uint32_t token,dbt_a2dp_connect_result_t *r){
    if(!c || !token || !r)return DBT_INVALID;
    if(!c->phase || c->serial!=token)return DBT_EMPTY;
    if(overlap(c,sizeof(*c),r,sizeof(*r)) ||
       overlap(r,sizeof(*r),c->config.discovery.request,c->config.discovery.request_capacity) ||
       overlap(r,sizeof(*r),c->config.discovery.response,c->config.discovery.response_capacity))return DBT_INVALID;
    *r=(dbt_a2dp_connect_result_t){0};r->token=c->serial;r->handle=c->handle;r->generation=c->generation;
    r->outcome=(dbt_a2dp_connect_outcome_t)c->outcome;r->phase=(dbt_a2dp_connect_phase_t)c->phase;
    r->last_error=c->last_error;r->page=c->page_result;r->signaling=c->signaling;
    r->signaling_event=c->signaling_event;r->signaling_reason=c->signaling_reason;
    r->signaling_result=c->signaling_result;r->signaling_event_ms=c->signaling_event_ms;
    r->signaling_event_seen=c->signaling_event_seen;
    r->cancel_requested=c->cancel_requested;r->used_existing_acl=c->used_existing_acl;r->owns_signaling=c->owns_signaling;
    r->ownership_fault=c->ownership_fault;
    r->link_live=c->generation && current(c)!=0;
    r->signaling_live=c->signaling.serial && dbt_a2dp_acceptor_connect_status(c->profile,&c->signaling)==DBT_OK;
    if(c->discovery_token)(void)dbt_sdp_discovery_result(&c->discovery,c->discovery_token,&r->discovery);
    return dbt_a2dp_connect_done(c)?DBT_OK:DBT_PENDING;
}
