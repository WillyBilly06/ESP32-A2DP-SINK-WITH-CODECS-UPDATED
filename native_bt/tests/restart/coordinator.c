#include "probe_restart.h"
#include "probe_output.h"
#include <assert.h>
#include <string.h>
static dbt_runtime_t runtime;
static probe_product_io_owner_t input;
static bool fenced,admission_idle,security_idle,smp_idle,audio_idle,avrcp_idle,output_idle,gatt_prepare_idle,gatt_final_idle,sound_busy,io_work;
static bool admission_frozen,output_held,audio_held,avrcp_held,input_paused;
static unsigned ingress_used,closes;static dbt_result_t input_result,audio_result,avrcp_result;
static probe_output_stats_t stats;
void probe_admission_freeze(bool v){admission_frozen=v;}
bool probe_admission_quiescent(void){return admission_idle;}
bool probe_security_quiescent(void){return security_idle;}
bool probe_smp_quiescent(void){return smp_idle;}
void probe_smp_security_request_preflight(uint32_t now){(void)now;assert(admission_frozen);}
void probe_output_restart_hold(bool v){output_held=v;}
bool probe_output_restart_quiescent(void){return output_held && output_idle;}
void probe_output_stats(probe_output_stats_t *s){*s=stats;}
dbt_result_t probe_audio_restart_hold(bool v,uint32_t n){(void)n;audio_held=v;return audio_result;}
bool probe_audio_restart_quiescent(void){return audio_held && audio_idle;}
dbt_result_t probe_avrcp_restart_hold(bool v,uint32_t n){(void)n;avrcp_held=v;return avrcp_result;}
bool probe_avrcp_restart_quiescent(void){return avrcp_held && avrcp_idle;}
dbt_result_t probe_product_io_owner_restart_begin(probe_product_io_owner_t *s){assert(s==&input);input_paused=true;return DBT_OK;}
dbt_result_t probe_product_io_owner_restart_poll(probe_product_io_owner_t *s,uint32_t n){(void)n;assert(s==&input && input_paused);return input_result;}
void probe_product_io_owner_restart_cancel(probe_product_io_owner_t *s){assert(s==&input);input_paused=false;}
bool probe_product_io_sound_busy(void){return sound_busy;}
bool probe_product_io_needs_worker(void){return io_work;}
bool probe_gatt_restart_fenced(void){return fenced;}
bool probe_gatt_restart_prepare_quiescent(void){return gatt_prepare_idle;}
bool probe_gatt_restart_quiescent(void){return gatt_final_idle;}
uint32_t dbt_queue_used(const dbt_queue_t *q){assert(q==&runtime.host.ingress);return ingress_used;}
dbt_result_t dbt_channels_close(dbt_channels_t *c,uint16_t h,uint16_t cid,uint32_t now){
    (void)now;assert(c==&runtime.channels && !c->in_callback);closes++;
    assert(h==10 && cid==64);c->channels[0].state=DBT_CH_DISCONNECTING;return DBT_BUSY;
}
static void reset(probe_restart_t *s){
    memset(&runtime,0,sizeof(runtime));memset(&input,0,sizeof(input));memset(&stats,0,sizeof(stats));
    runtime.ready=true;runtime.host.state=DBT_HOST_IDENTIFIED;
    admission_idle=security_idle=smp_idle=audio_idle=avrcp_idle=output_idle=gatt_prepare_idle=gatt_final_idle=fenced=true;
    sound_busy=io_work=admission_frozen=output_held=audio_held=avrcp_held=input_paused=false;
    input_result=audio_result=avrcp_result=DBT_OK;ingress_used=closes=0;assert(probe_restart_init(s,&runtime,&input));
}
static void check_canceled(probe_restart_t *s){probe_restart_cancel(s,77);assert(!probe_restart_active(s) && !admission_frozen && !output_held && !audio_held && !avrcp_held && !input_paused);}
int main(void){
    probe_restart_t s;reset(&s);fenced=false;assert(probe_restart_prepare(&s,1,1)==DBT_INVALID && !s.active);
    fenced=true;runtime.in_poll=true;assert(probe_restart_prepare(&s,1,1)==DBT_BUSY && !s.active);runtime.in_poll=false;
    assert(probe_restart_prepare(&s,1,1)==DBT_OK && admission_frozen && output_held && input_paused);
    assert(probe_restart_prepare(&s,2,2)==DBT_INVALID && s.epoch==1);
    gatt_final_idle=false;assert(!probe_restart_quiescent(&s,2));gatt_final_idle=true;assert(probe_restart_quiescent(&s,3));check_canceled(&s);
    bool *gates[]={&admission_idle,&security_idle,&smp_idle,&audio_idle,&avrcp_idle,&output_idle,&gatt_prepare_idle};
    for(unsigned i=0;i<sizeof(gates)/sizeof(gates[0]);i++){
        reset(&s);*gates[i]=false;assert(probe_restart_prepare(&s,1,1)==DBT_BUSY && !s.failed);*gates[i]=true;
        assert(probe_restart_prepare(&s,1,2)==DBT_OK);check_canceled(&s);
    }
    for(unsigned i=0;i<12;i++){
        reset(&s);
        switch(i){case 0:runtime.host.command_count=1;break;case 1:runtime.host.pending_opcode=0xc35;break;
        case 2:runtime.host.tx_retained=true;break;case 3:runtime.control_active=true;break;
        case 4:runtime.tx[0].active=true;break;case 5:runtime.ertm_tx[0].active=true;break;
        case 6:runtime.acl.links[0].outstanding=1;break;case 7:runtime.rx.partial[0].data=(uint8_t *)1;break;
        case 8:runtime.rx_flow.pending[0].used=true;runtime.rx_flow.pending[0].packets=1;break;
        case 9:ingress_used=1;break;case 10:sound_busy=true;break;case 11:io_work=true;break;}
        assert(probe_restart_prepare(&s,1,1)==DBT_BUSY && !s.failed);check_canceled(&s);
    }
    reset(&s);input_result=DBT_BUSY;assert(probe_restart_prepare(&s,1,1)==DBT_BUSY && !avrcp_held);input_result=DBT_INVALID;assert(probe_restart_prepare(&s,1,2)==DBT_INVALID && s.failed);check_canceled(&s);
    reset(&s);runtime.channels.channels[0].state=DBT_CH_OPEN;runtime.channels.channels[0].handle=10;runtime.channels.channels[0].local_cid=64;
    assert(probe_restart_prepare(&s,1,1)==DBT_BUSY && closes==1);runtime.channels.channels[0].state=DBT_CH_FREE;assert(probe_restart_prepare(&s,1,2)==DBT_OK);check_canceled(&s);
    reset(&s);input_result=DBT_BUSY;runtime.channels.channels[0].state=DBT_CH_OPEN;runtime.channels.channels[0].psm=0x17;runtime.channels.channels[0].handle=10;runtime.channels.channels[0].local_cid=64;
    assert(probe_restart_prepare(&s,1,1)==DBT_BUSY && !closes && !avrcp_held);
    input_result=DBT_OK;assert(probe_restart_prepare(&s,1,71)==DBT_BUSY && closes==1 && avrcp_held);check_canceled(&s);
    reset(&s);stats.request_fault=1;assert(probe_restart_prepare(&s,1,1)==DBT_INVALID && s.failed);stats.request_fault=0;assert(!probe_restart_quiescent(&s,2));check_canceled(&s);
    reset(&s);input_result=DBT_BUSY;uint32_t start=UINT32_MAX-100;assert(probe_restart_prepare(&s,1,start)==DBT_BUSY);
    assert(probe_restart_prepare(&s,1,start+9999u)==DBT_BUSY);assert(probe_restart_prepare(&s,1,start+10000u)==DBT_INVALID);check_canceled(&s);
    return 0;
}
