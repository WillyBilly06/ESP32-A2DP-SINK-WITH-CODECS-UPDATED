#include "probe_rfcomm.h"
#define ECHO_FRAME 180u
static struct {
    dbt_runtime_t *runtime;dbt_rfcomm_spp_t spp;dbt_sdp_attribute_t attributes[5];dbt_sdp_record_t record;
    uint8_t *echo;uint16_t echo_size;bool initialized,hold,pending,active,grant_pending,failed;
} state;
static const uint8_t service_name[]={0x25,18,'N','a','t','i','v','e',' ','R','F','C','O','M','M',' ','E','c','h','o'};
static void echo_event(void *context,dbt_rfcomm_dlc_event_t event,uint8_t dlci){
    (void)context;(void)dlci;
    if(event==DBT_RFCOMM_DLC_EVENT_CLOSED || event==DBT_RFCOMM_DLC_EVENT_FAILED)
        state.pending=state.grant_pending=false;
    if(event==DBT_RFCOMM_DLC_EVENT_FAILED && state.spp.bound)state.failed=true;
}
static void echo_data(void *context,uint8_t dlci,const uint8_t *data,uint16_t size){
    (void)context;(void)dlci;
    if(state.hold)return; /* The channel is being retired; no new echo starts. */
    if(!state.echo || state.pending || state.active || size>ECHO_FRAME){state.failed=true;return;}
    for(unsigned i=0;i<size;++i)state.echo[i]=data[i];
    state.echo_size=size;state.pending=true;
}
static void echo_done(void *context,dbt_tx_outcome_t outcome){
    (void)context;state.active=false;
    if(outcome==DBT_TX_ADMITTED)state.grant_pending=true;
    else if(outcome==DBT_TX_ABORTED && !state.hold)state.failed=true;
}
dbt_result_t probe_rfcomm_init(dbt_runtime_t *r,uint8_t scn,uint32_t handle){
    if(!r || state.initialized)return DBT_INVALID;
    dbt_rfcomm_dlc_hooks_t hooks={0,echo_event,echo_data};
    dbt_result_t result=dbt_rfcomm_spp_init(&state.spp,r,scn,256,ECHO_FRAME,1,handle,hooks);
    if(result!=DBT_OK)return result;
    const dbt_sdp_record_t *base=dbt_rfcomm_spp_record(&state.spp);
    if(!base || base->count!=4)return DBT_INVALID;
    for(unsigned i=0;i<4;++i)state.attributes[i]=base->attributes[i];
    state.attributes[4]=(dbt_sdp_attribute_t){0x100,sizeof(service_name),service_name};
    state.record=(dbt_sdp_record_t){base->handle,state.attributes,5};
    state.runtime=r;state.initialized=true;return DBT_OK;
}
bool probe_rfcomm_accept(const dbt_link_t *link,uint16_t psm){
    if(psm!=3 || !state.initialized)return true;
    return !state.hold && !state.failed && dbt_rfcomm_spp_accept(&state.spp,link,psm);
}
void probe_rfcomm_event(dbt_channel_event_t event,const dbt_channel_t *c){if(state.initialized)dbt_rfcomm_spp_event(&state.spp,event,c);}
dbt_result_t probe_rfcomm_data(const dbt_channel_t *c,const uint8_t *data,uint16_t size){
    if(!state.initialized || !c || c->psm!=3)return DBT_UNSUPPORTED;
    dbt_result_t r=dbt_rfcomm_spp_receive(&state.spp,c,data,size);
    if(r!=DBT_OK)state.failed=true;
    return r;
}
static void free_echo(void){
    if(state.echo){state.runtime->hooks.release(state.runtime->hooks.context,state.echo);state.echo=0;}
    state.echo_size=0;state.pending=state.grant_pending=false;
}
static void retire_channels(uint32_t now){
    dbt_channels_t *manager=&state.runtime->channels;
    for(unsigned i=0;i<DBT_CHANNEL_LIMIT;++i){
        dbt_channel_t *c=&manager->channels[i];if(c->state==DBT_CH_FREE || c->state==DBT_CH_DISCONNECTING || c->psm!=3)continue;
        if(state.spp.bound && c->serial==state.spp.bearer.serial && c->link_generation==state.spp.bearer.mux.generation){
            dbt_rfcomm_bearer_t *b=&state.spp.bearer;
            if(b->tx_active || b->app_pending)continue;
            if(b->dlc.state==DBT_RFCOMM_DLC_OPEN){
                (void)dbt_rfcomm_bearer_close_dlc(b);continue;
            }
            if(b->dlc.state==DBT_RFCOMM_DLC_DISC_QUEUED || b->dlc.state==DBT_RFCOMM_DLC_WAIT_DISC ||
               b->dlc.state==DBT_RFCOMM_DLC_CLOSE_REPLY)continue;
        }
        (void)dbt_channels_close(manager,c->handle,c->local_cid,now);
    }
}
void probe_rfcomm_poll(uint32_t now){
    if(!state.initialized || !state.runtime || state.runtime->in_poll)return;
    if(state.hold && state.spp.pending)state.spp.failed=true;
    if(state.hold && state.spp.bound && state.spp.bearer.app_pending && !state.spp.bearer.tx_active)
        state.spp.bearer.faulted=true; /* abort a copied, unsubmitted echo */
    if(state.failed && state.spp.bound)state.spp.bearer.faulted=true;
    dbt_rfcomm_spp_poll(&state.spp,now);
    if(state.spp.bound && !state.echo && !state.hold){
        state.echo=state.runtime->hooks.allocate(state.runtime->hooks.context,ECHO_FRAME);
        if(!state.echo){state.failed=true;state.spp.bearer.faulted=true;}
    }
    if(!state.spp.bound && !state.active){free_echo();state.failed=false;}
    if(!state.hold && !state.failed && state.spp.bound && state.spp.bearer.dlc.state==DBT_RFCOMM_DLC_OPEN){
        dbt_rfcomm_bearer_t *b=&state.spp.bearer;
        if(state.grant_pending && b->dlc.credit_mode){
            dbt_result_t r=dbt_rfcomm_bearer_grant(b,1);
            if(r==DBT_OK)state.grant_pending=false;else if(r!=DBT_BUSY)state.failed=true;
        }else if(state.grant_pending)state.grant_pending=false;
        if(state.pending && !state.active && !state.grant_pending){
            dbt_result_t r=dbt_rfcomm_bearer_send(b,state.echo,state.echo_size,echo_done,0);
            if(r==DBT_OK){state.pending=false;state.active=true;}else if(r!=DBT_BUSY)state.failed=true;
        }
    }
    if(state.hold)retire_channels(now);
}
uint32_t probe_rfcomm_wait_ms(uint32_t now){
    if(!state.initialized || !state.runtime)return UINT32_MAX;
    if(state.spp.pending)return 0;
    if(!state.spp.bound){
        if(state.hold)for(unsigned i=0;i<DBT_CHANNEL_LIMIT;++i){
            const dbt_channel_t *c=&state.runtime->channels.channels[i];
            if(c->psm==3 && c->state!=DBT_CH_FREE && c->state!=DBT_CH_DISCONNECTING)return 0;
        }
        return UINT32_MAX;
    }
    const dbt_rfcomm_bearer_t *b=&state.spp.bearer;
    if(state.failed && !b->tx_active)return 0;
    if(!state.echo && !state.hold)return 0;
    bool runtime_busy=false;
    for(unsigned i=0;i<DBT_MAX_LINKS;++i)if(state.runtime->tx[i].active &&
       state.runtime->tx[i].pdu.handle==b->mux.handle)runtime_busy=true;
    if(!b->tx_active && !runtime_busy && (b->mux.tx_ready || b->dlc.tx_ready))return 0;
    if(b->dlc.state==DBT_RFCOMM_DLC_MSC && !b->dlc.msc_sent && !b->dlc.tx_ready)return 0;
    if(state.hold && !b->tx_active && !b->app_pending && !b->dlc.tx_ready &&
       b->dlc.state!=DBT_RFCOMM_DLC_WAIT_DISC && b->dlc.state!=DBT_RFCOMM_DLC_CLOSE_REPLY)return 0;
    bool data_ready=!b->dlc.remote_flow_off && !b->dlc.aggregate_flow_off &&
        (!b->dlc.credit_mode || b->dlc.credits.tx_available);
    if(b->dlc.state==DBT_RFCOMM_DLC_OPEN && (state.grant_pending || (state.pending && data_ready)) &&
       !state.hold && !b->tx_active && !b->dlc.tx_ready)return 0;
    if(b->count && !b->tx_active && !b->mux.tx_ready && !b->dlc.tx_ready)return 0;
    if(b->mux.state==DBT_RFCOMM_MUX_WAIT_UA || b->mux.state==DBT_RFCOMM_MUX_WAIT_DISC){
        uint32_t n=b->mux.deadline-now;return n<0x80000000u?n:0;
    }
    if(b->dlc.state==DBT_RFCOMM_DLC_WAIT_PN || b->dlc.state==DBT_RFCOMM_DLC_WAIT_SABM ||
       b->dlc.state==DBT_RFCOMM_DLC_WAIT_UA || b->dlc.state==DBT_RFCOMM_DLC_WAIT_DISC ||
       (b->dlc.state==DBT_RFCOMM_DLC_MSC && b->dlc.msc_sent)){
        uint32_t n=b->dlc.deadline-now;return n<0x80000000u?n:0;
    }
    return UINT32_MAX;
}
const dbt_sdp_record_t *probe_rfcomm_record(void){
    return state.initialized && dbt_rfcomm_spp_record(&state.spp)?&state.record:0;
}
void probe_rfcomm_restart_begin(void){state.hold=true;}
bool probe_rfcomm_restart_quiescent(void){
    if(!state.initialized)return true;
    if(!state.hold || state.spp.pending || state.spp.bound || state.active || state.echo)return false;
    for(unsigned i=0;i<DBT_CHANNEL_LIMIT;++i)if(state.runtime->channels.channels[i].state!=DBT_CH_FREE &&
       state.runtime->channels.channels[i].psm==3)return false;
    return true;
}
void probe_rfcomm_restart_cancel(void){state.hold=false;}
