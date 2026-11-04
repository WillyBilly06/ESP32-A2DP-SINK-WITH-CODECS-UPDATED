#include "dbt_rfcomm_spp.h"
static const uint8_t service_class[]={0x35,3,0x19,0x11,1};
static const uint8_t browse[]={0x35,3,0x19,0x10,2};
static const uint8_t profile[]={0x35,8,0x35,6,0x19,0x11,1,9,1,2};
dbt_result_t dbt_rfcomm_spp_init(dbt_rfcomm_spp_t *s,dbt_runtime_t *r,uint8_t scn,uint16_t mtu,uint16_t max_frame,
                                 uint8_t credits,uint32_t handle,dbt_rfcomm_dlc_hooks_t hooks){
    if(!s || !r || !r->ready || r->in_poll || !hooks.event || !hooks.data || scn<1 || scn>30 || mtu<48 || mtu>r->maximum_mtu ||
       max_frame<23 || (uint32_t)max_frame+5u>mtu || max_frame+5u>DBT_RFCOMM_BEARER_FRAME_MAX ||
       credits>DBT_RFCOMM_BEARER_RX_CREDITS || handle<0x10000u)return DBT_INVALID;
    if(s->installed)return DBT_BUSY;
    *s=(dbt_rfcomm_spp_t){0};s->runtime=r;s->scn=scn;s->max_frame=max_frame;s->initial_credits=credits;s->hooks=hooks;
    const uint8_t protocol[]={0x35,12,0x35,3,0x19,1,0,0x35,5,0x19,0,3,8,scn};
    for(unsigned i=0;i<sizeof(protocol);++i)s->protocols[i]=protocol[i];
    s->attributes[0]=(dbt_sdp_attribute_t){1,sizeof(service_class),service_class};
    s->attributes[1]=(dbt_sdp_attribute_t){4,sizeof(s->protocols),s->protocols};
    s->attributes[2]=(dbt_sdp_attribute_t){5,sizeof(browse),browse};
    s->attributes[3]=(dbt_sdp_attribute_t){9,sizeof(profile),profile};
    s->record=(dbt_sdp_record_t){handle,s->attributes,4};dbt_sdp_t check;
    if(dbt_sdp_init(&check,&s->record,1,1)!=DBT_OK)return DBT_INVALID;
    dbt_service_t service={3,mtu,48,DBT_SECURITY_ENCRYPTED};dbt_result_t result=dbt_channels_register(&r->channels,service);
    if(result==DBT_OK)s->installed=true;
    return result;
}
bool dbt_rfcomm_spp_accept(const dbt_rfcomm_spp_t *s,const dbt_link_t *link,uint16_t psm){
    return s && s->installed && link && link->kind==DBT_LINK_CLASSIC && psm==3 && !s->pending && !s->bound && !s->failed;
}
void dbt_rfcomm_spp_event(dbt_rfcomm_spp_t *s,dbt_channel_event_t event,const dbt_channel_t *c){
    if(!s || !s->installed || !c || c->psm!=3)return;
    if(event==DBT_CH_EVENT_OPEN && c->incoming_connect_id && !s->pending && !s->bound){
        s->pending=true;s->pending_handle=c->handle;s->pending_cid=c->local_cid;s->pending_generation=c->link_generation;s->pending_serial=c->serial;
    }else if(event==DBT_CH_EVENT_OPEN && c->incoming_connect_id){
        for(unsigned i=0;i<DBT_CHANNEL_LIMIT;++i)if(!s->rejected[i].serial){
            s->rejected[i].handle=c->handle;s->rejected[i].cid=c->local_cid;
            s->rejected[i].generation=c->link_generation;s->rejected[i].serial=c->serial;break;
        }
    }else if((event==DBT_CH_EVENT_CLOSED || event==DBT_CH_EVENT_FAILED) && s->pending &&
             c->serial==s->pending_serial && c->link_generation==s->pending_generation){s->pending=false;s->early_valid=false;s->failed=false;}
}
dbt_result_t dbt_rfcomm_spp_receive(dbt_rfcomm_spp_t *s,const dbt_channel_t *c,const uint8_t *data,uint16_t size){
    if(!s || !s->installed || !c || !data || c->psm!=3)return DBT_INVALID;
    if(s->bound)return dbt_rfcomm_bearer_enqueue(&s->bearer,c,data,size);
    if(!s->pending || c->serial!=s->pending_serial || c->link_generation!=s->pending_generation)return DBT_INVALID;
    /* Before binding, a peer can only send the initial control SABM; one
     * bounded early frame bridges event and owner poll without allocating in
     * the host callback. Unexpected pipelining fails the channel closed. */
    if(size!=4 || s->early_valid){s->failed=true;return DBT_NO_SPACE;}
    for(unsigned i=0;i<4;++i)s->early[i]=data[i];
    s->early_valid=true;return DBT_OK;
}
void dbt_rfcomm_spp_poll(dbt_rfcomm_spp_t *s,uint32_t now){
    if(!s || !s->installed || !s->runtime || s->runtime->in_poll)return;
    for(unsigned i=0;i<DBT_CHANNEL_LIMIT;++i)if(s->rejected[i].serial){
        const dbt_channel_t *c=dbt_channels_find(&s->runtime->channels,s->rejected[i].handle,s->rejected[i].cid);
        if(!c || c->serial!=s->rejected[i].serial || c->link_generation!=s->rejected[i].generation || c->state==DBT_CH_DISCONNECTING)
            s->rejected[i].serial=0;
        else (void)dbt_channels_close(&s->runtime->channels,c->handle,c->local_cid,now);
    }
    if(s->pending){
        const dbt_channel_t *c=dbt_channels_find(&s->runtime->channels,s->pending_handle,s->pending_cid);
        if(!c || c->serial!=s->pending_serial || c->link_generation!=s->pending_generation || c->state==DBT_CH_DISCONNECTING){s->pending=false;s->early_valid=false;s->failed=false;}
        else if(s->failed)(void)dbt_channels_close(&s->runtime->channels,c->handle,c->local_cid,now);
        else {
            dbt_result_t r=dbt_rfcomm_bearer_bind(&s->bearer,s->runtime,c->handle,c->local_cid,false,10000);
            if(r!=DBT_OK){s->failed=true;(void)dbt_channels_close(&s->runtime->channels,c->handle,c->local_cid,now);}
            else {s->bound=true;s->pending=false;s->listen_set=false;
                if(s->early_valid && dbt_rfcomm_bearer_enqueue(&s->bearer,c,s->early,4)!=DBT_OK)s->failed=true;
                s->early_valid=false;}
        }
    }
    if(s->bound){
        if(!s->listen_set && s->bearer.mux.state==DBT_RFCOMM_MUX_OPEN){
            uint16_t n1=s->max_frame;if(n1>s->bearer.frame_capacity-5u)n1=(uint16_t)(s->bearer.frame_capacity-5u);
            if(dbt_rfcomm_bearer_listen(&s->bearer,s->scn,n1,s->initial_credits,s->hooks)!=DBT_OK)s->failed=true;
            else s->listen_set=true;
        }
        if(s->failed)s->bearer.faulted=true;
        dbt_rfcomm_bearer_poll(&s->bearer,now);
        if(s->bearer.closed){s->bound=false;s->listen_set=false;s->failed=false;}
    }
}
const dbt_sdp_record_t *dbt_rfcomm_spp_record(const dbt_rfcomm_spp_t *s){
    return s && s->installed && s->runtime && s->runtime->ready && s->runtime->host.state==DBT_HOST_IDENTIFIED?&s->record:0;
}
