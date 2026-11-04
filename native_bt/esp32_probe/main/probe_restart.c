#include "probe_restart.h"
#include "probe_admission.h"
#include "probe_security.h"
#include "probe_smp.h"
#include "probe_audio.h"
#include "probe_output.h"
#include "probe_avrcp.h"
#include "probe_gatt.h"
#ifdef DBT_PROBE_RFCOMM
#include "probe_rfcomm.h"
#endif
#include <string.h>

bool probe_restart_init(probe_restart_t *s,dbt_runtime_t *r,probe_product_io_owner_t *input){
    if(!s || !r || !input)return false;
    memset(s,0,sizeof(*s));s->runtime=r;s->input=input;return true;
}
static bool transport_idle(const dbt_runtime_t *r){
    if(r->in_poll || r->host.in_poll || r->channels.in_callback || r->host.pending_opcode ||
       r->host.command_count || r->host.tx_retained || r->control_active ||
       r->channels.tx_count || r->channels.rx_count || r->rx_flow.command_pending ||
       dbt_queue_used(&r->host.ingress))return false;
    for(unsigned i=0;i<DBT_MAX_LINKS;i++)if(r->tx[i].active || r->ertm_tx[i].active ||
        r->acl.links[i].outstanding || r->rx.partial[i].data ||
        (r->rx_flow.pending[i].used && r->rx_flow.pending[i].packets))return false;
    return true;
}
static dbt_result_t progress(probe_restart_t *s,uint32_t now,bool prepared){
    dbt_runtime_t *r=s->runtime;
    if(s->failed || !r->ready || r->host.state==DBT_HOST_FAILED || r->host.fault!=DBT_FAULT_NONE || r->rx_flow.state==DBT_RX_FLOW_FAILED ||
       (int32_t)(now-s->deadline)>=0)return DBT_INVALID;
    if(r->in_poll || r->channels.in_callback)return DBT_BUSY;
    probe_output_stats_t out;probe_output_stats(&out);
    if(out.failed || out.request_fault)return DBT_INVALID;
    dbt_result_t input=probe_product_io_owner_restart_poll(s->input,now);
    dbt_result_t audio=probe_audio_restart_hold(true,now);
    /* Let an accepted physical key release use AVRCP before sealing it. */
    dbt_result_t avrcp=DBT_BUSY;
    if(input==DBT_OK || s->avrcp_held){s->avrcp_held=true;avrcp=probe_avrcp_restart_hold(true,now);}
    if(input==DBT_INVALID || audio==DBT_INVALID || avrcp==DBT_INVALID)return DBT_INVALID;
    bool channels=false;
    for(unsigned i=0;i<DBT_CHANNEL_LIMIT;i++){
        const dbt_channel_t *ch=&r->channels.channels[i];
        if(ch->state==DBT_CH_FREE)continue;
        channels=true;
        if(ch->psm==0x17 && !s->avrcp_held)continue;
        dbt_result_t result=dbt_channels_close(&r->channels,ch->handle,ch->local_cid,now);
        if(result!=DBT_OK && result!=DBT_BUSY && result!=DBT_NO_SPACE)return DBT_INVALID;
    }
    bool ready=input==DBT_OK && !channels && probe_admission_quiescent() &&
        probe_security_quiescent() && probe_smp_quiescent() && probe_audio_restart_quiescent() &&
        probe_avrcp_restart_quiescent() && probe_output_restart_quiescent() &&
        !probe_product_io_sound_busy() && !probe_product_io_needs_worker() && transport_idle(r);
#ifdef DBT_PROBE_RFCOMM
    ready=ready && probe_rfcomm_restart_quiescent();
#endif
    ready=ready && (prepared?probe_gatt_restart_quiescent():probe_gatt_restart_prepare_quiescent());
    return ready?DBT_OK:DBT_BUSY;
}
dbt_result_t probe_restart_prepare(probe_restart_t *s,uint32_t epoch,uint32_t now){
    if(!s || !s->runtime || !s->input || !epoch || !probe_gatt_restart_fenced())return DBT_INVALID;
    if(s->active && s->epoch!=epoch)return DBT_INVALID;
    if(s->runtime->in_poll || s->runtime->channels.in_callback)return DBT_BUSY;
    if(!s->active){
        s->active=true;s->epoch=epoch;s->deadline=now+10000u;s->failed=false;s->prepared=false;s->avrcp_held=false;
        probe_admission_freeze(true);probe_smp_security_request_preflight(now);probe_output_restart_hold(true);
#ifdef DBT_PROBE_RFCOMM
        probe_rfcomm_restart_begin();
#endif
        if(probe_product_io_owner_restart_begin(s->input)==DBT_INVALID)s->failed=true;
    }
    dbt_result_t result=progress(s,now,false);
    if(result==DBT_INVALID)s->failed=true;
    if(result==DBT_OK && !s->prepared){s->prepared=true;s->deadline=now+10000u;}
    return result;
}
bool probe_restart_quiescent(probe_restart_t *s,uint32_t now){
    if(!s || !s->active || !probe_gatt_restart_fenced())return false;
    dbt_result_t result=progress(s,now,true);
    if(result==DBT_INVALID)s->failed=true;
    return result==DBT_OK;
}
void probe_restart_cancel(probe_restart_t *s,uint32_t now){
    if(!s || !s->active)return;
    probe_product_io_owner_restart_cancel(s->input);probe_admission_freeze(false);
    probe_output_restart_hold(false);(void)probe_audio_restart_hold(false,now);
    (void)probe_avrcp_restart_hold(false,now);
#ifdef DBT_PROBE_RFCOMM
    probe_rfcomm_restart_cancel();
#endif
    s->active=false;s->epoch=0;s->failed=false;s->prepared=false;s->avrcp_held=false;
}
bool probe_restart_active(const probe_restart_t *s){return s && s->active;}
