#define DBT_OTA_SHIM_LIB 1
#include "../ota/esp_shim.c"
#include "probe_product.h"
#include "probe_product_runtime.h"
#include "dbt_ble_nvs.h"
#include "nvs.h"

static probe_product_t *owner_product;static uint32_t test_clock;static int apply_name;static unsigned commits,owner_wakes,worker_wakes;
static bool fail_nvs;static char durable_name[33],staged_name[33];
static probe_product_runtime_t owner_runtime;static dbt_runtime_t runtime_model;static dbt_gap_t gap_model;
static void revoke_during_select(void){probe_product_ota_epoch(owner_product,0);}
static bool use_runtime,secure_peer=true,security_bond=true,security_encrypt=true;static uint8_t security_key_size=16;static unsigned gap_polls,gatt_updates;
static bool trust_peer=true;static uint8_t bond_version=1;
static dbt_result_t trust(uint16_t h,uint32_t generation,uint8_t *type,uint8_t id[6],uint8_t tag[32]) {
    if(!trust_peer || h!=7 || generation!=runtime_model.acl.links[0].generation)return DBT_INVALID;
    *type=0;memset(id,0x42,6);memset(tag,bond_version,32);return DBT_OK;
}
esp_err_t nvs_open(const char *n,int mode,nvs_handle_t *h){assert(!strcmp(n,"audio") && mode==NVS_READWRITE);*h=1;return ESP_OK;}
esp_err_t nvs_set_str(nvs_handle_t h,const char *key,const char *v){assert(h==1 && !strcmp(key,"devname") && strlen(v)<sizeof(staged_name));memcpy(staged_name,v,strlen(v)+1);return ESP_OK;}
esp_err_t nvs_commit(nvs_handle_t h){assert(h==1);commits++;if(fail_nvs)return 1;memcpy(durable_name,staged_name,sizeof(durable_name));return ESP_OK;}
void nvs_close(nvs_handle_t h){assert(h==1);}
dbt_result_t dbt_ble_nvs_load_state(dbt_ble_control_state_t *s){s->valid=true;return DBT_OK;}
static uint32_t time_ms(void *c){(void)c;return test_clock;}
static void wake_owner(void *c){(void)c;owner_wakes++;}
static void wake_worker(void *c){(void)c;worker_wakes++;}
void vTaskDelay(uint32_t n){
    test_clock+=n;
    if(use_runtime){probe_product_runtime_poll(&owner_runtime);return;}
    if(apply_name) {probe_product_name_t request;
        if(probe_product_take_name(owner_product,&request)) {
            assert(!strcmp(request.name,durable_name));
            assert(probe_product_complete_name(owner_product,request.token,apply_name==1));
        }
    }
}
const dbt_link_t *dbt_acl_find(const dbt_acl_t *acl,uint16_t h){for(unsigned i=0;i<DBT_MAX_LINKS;i++)if(acl->links[i].used && acl->links[i].handle==h)return &acl->links[i];return 0;}
static dbt_result_t security(uint16_t h,uint32_t generation,dbt_smp_session_security_t *out){
    if(!secure_peer || h!=7 || generation!=runtime_model.acl.links[0].generation)return DBT_INVALID;
    memset(out,0,sizeof(*out));out->encrypted=security_encrypt;out->bond_committed=security_bond;out->key_size=security_key_size;return DBT_OK;
}
dbt_result_t dbt_gap_rename(dbt_gap_t *g,const uint8_t *p,uint8_t n,uint32_t *token){
    assert(g==&gap_model && n && p);if(g->rename_active)return DBT_BUSY;g->rename_active=true;g->rename_serial++;*token=g->rename_serial;gap_polls=0;return DBT_OK;
}
dbt_result_t dbt_gap_rename_result(const dbt_gap_t *g,uint32_t token,bool *applied,uint8_t *status){
    assert(g==&gap_model && token==g->rename_serial);if(!gap_polls++){return DBT_BUSY;}
    gap_model.rename_active=false;*applied=apply_name==1;*status=*applied?0:1;return DBT_OK;
}
dbt_result_t probe_gatt_control_name_applied(const uint8_t *p,uint8_t n){assert(p && n);gatt_updates++;return DBT_OK;}
static void grant(probe_product_t *s,uint32_t generation) {
    dbt_product_peer_value_t v={0};v.peer=(dbt_ble_control_peer_t){7,generation};v.encrypted=v.bonded=v.allow_name=v.allow_ota=true;
    memcpy(v.ota.project,"native_bt_probe",16);assert(probe_product_publish_peer(s,&v));probe_product_ota_epoch(s,generation);
}
static void command(dbt_ble_control_state_t *display,dbt_ble_control_hooks_t *hooks,const dbt_ble_control_peer_t *peer,
                    const uint8_t *data,uint16_t n,uint8_t expected) {
    dbt_ble_control_result_t result;
    if(data[0]>=0x20 && data[0]<=0x23) {
        uint8_t size=DBT_BLE_CONTROL_MAX_STATUS;
        uint32_t epoch=atomic_load(&((probe_product_t *)hooks->context)->admission_epoch);
        if(!probe_product_ota_command_epoch(hooks->context,peer,epoch,data[0],data+1,n-1,result.data,&size)) {
            result.data[0]=0x11;size=1;
        }
        assert(size && result.data[0]==expected);return;
    }
    assert(dbt_ble_control_execute(display,hooks,peer,data,n,&result)==DBT_OK && result.size && result.data[0]==expected);
}
static uint8_t runtime_ota(probe_product_runtime_t *r,const dbt_ble_control_peer_t *p,uint32_t epoch,
                           const uint8_t *bytes,uint16_t n) {
    uint8_t out[DBT_BLE_CONTROL_MAX_STATUS],size=sizeof(out);
    if(bytes[0]!=0x20 && probe_product_runtime_ota_admit(r,p,bytes[0],bytes+1,n-1)!=epoch)return 0x11;
    return probe_product_runtime_ota_worker(r,p,epoch,bytes[0],bytes+1,n-1,out,&size) && size?out[0]:0x11;
}
int main(void) {
    probe_product_t s;owner_product=&s;dbt_ble_control_state_t state={0};
    probe_product_hooks_t hooks={0};hooks.now_ms=time_ms;hooks.wake_owner=wake_owner;hooks.wake_worker=wake_worker;
    hooks.ota_timeout_ms=100;hooks.name_timeout_ms=60;
    assert(probe_product_startup_worker(&s,&state,&hooks) && state.valid);
    dbt_ble_control_hooks_t control={0};control.context=&s;control.command=probe_product_command;
    dbt_ble_control_peer_t peer={7,1};uint8_t start[]={0x20,9,0,0,0},data[]={0x21,1,'1','2','3','4','5','6','7','8','9'},end[]={0x22};
    command(&state,&control,&peer,start,sizeof(start),0x11);grant(&s,1);slot_recorded=true;
    command(&state,&control,&peer,start,sizeof(start),0x21);command(&state,&control,&peer,data,sizeof(data),0x20);
    command(&state,&control,&peer,end,sizeof(end),0x22);assert(s.ota.state==DBT_OTA_VERIFIED && boot==&running && !boot_writes);
    assert(probe_product_request_activation(&s,&peer));probe_product_worker_tick(&s);
    assert(probe_product_take_activation_result(&s)==PROBE_PRODUCT_SELECTED && boot==&update && boot_writes==1);
    boot=&running;command(&state,&control,&peer,start,sizeof(start),0x21);
    probe_product_peer_lost(&s,&peer);
    command(&state,&control,&peer,data,sizeof(data),0x11);probe_product_worker_tick(&s);
    assert(s.ota.state==DBT_OTA_IDLE && !s.esp.session);grant(&s,2);
    dbt_ble_control_peer_t peer2={7,2};command(&state,&control,&peer,start,sizeof(start),0x11);
    command(&state,&control,&peer2,start,sizeof(start),0x21);test_clock+=100;probe_product_worker_tick(&s);
    assert(s.ota.state==DBT_OTA_IDLE && !s.esp.session);
    apply_name=1;uint8_t name[]={4,'N','e','w'};command(&state,&control,&peer2,name,sizeof(name),0x10);
    assert(commits==1 && !strcmp(durable_name,"New") && state.name_size==3 && !memcmp(state.name,"New",3));
    apply_name=0;uint8_t later[]={4,'L','a','t','e'};command(&state,&control,&peer2,later,sizeof(later),0x11);
    assert(commits==2 && !strcmp(durable_name,"Late"));probe_product_name_t pending;
    assert(probe_product_take_name(&s,&pending) && !strcmp(pending.name,"Late"));
    assert(!probe_product_complete_name(&s,pending.token+1,true));assert(probe_product_complete_name(&s,pending.token,true));
    apply_name=2;command(&state,&control,&peer2,name,sizeof(name),0x11);
    assert(commits==3 && !strcmp(durable_name,"New"));assert(probe_product_take_name(&s,&pending));
    assert(probe_product_complete_name(&s,pending.token,false));assert(probe_product_take_name(&s,&pending));
    assert(probe_product_complete_name(&s,pending.token,true) && commits==3);
    apply_name=1;fail_nvs=true;command(&state,&control,&peer2,name,sizeof(name),0x11);assert(commits==4 && !strcmp(durable_name,"New"));
    assert(owner_wakes && worker_wakes && probe_product_worker_quiescent(&s));
    fail_nvs=false;command(&state,&control,&peer2,start,sizeof(start),0x21);
    command(&state,&control,&peer2,data,sizeof(data),0x20);command(&state,&control,&peer2,end,sizeof(end),0x22);
    select_failure=2;assert(probe_product_request_activation(&s,&peer2));assert(probe_product_worker_poll(&s,false));
    assert(probe_product_take_activation_result(&s)==PROBE_PRODUCT_SELECTION_AMBIGUOUS && boot==&update);
    assert(probe_product_worker_poll(&s,true));
    /* Owner runtime: real security snapshot, explicit grant and correlated
     * GAP completion before the name ACK and GATT name update. */
    boot=&running;select_failure=0;test_clock=0;state.valid=false;assert(probe_product_startup_worker(&s,&state,&hooks));
    memset(&runtime_model,0,sizeof(runtime_model));runtime_model.acl.links[0].used=true;runtime_model.acl.links[0].kind=DBT_LINK_LE;
    runtime_model.acl.links[0].handle=7;runtime_model.acl.links[0].generation=10;runtime_model.acl.links[0].encrypted=true;
    memset(&gap_model,0,sizeof(gap_model));gap_model.state=DBT_GAP_READY;
    assert(probe_product_runtime_init(&owner_runtime,&s,&runtime_model,&gap_model,security));owner_product=&s;use_runtime=true;
    assert(probe_product_runtime_set_trust(&owner_runtime,trust));
    probe_product_runtime_poll(&owner_runtime);dbt_ble_control_peer_t peer10={7,10};
    command(&state,&control,&peer10,start,sizeof(start),0x11);
    assert(probe_product_runtime_grant_ota(&owner_runtime,&peer10,0)==DBT_OK);
    command(&state,&control,&peer10,start,sizeof(start),0x21);command(&state,&control,&peer10,data,sizeof(data),0x20);
    command(&state,&control,&peer10,end,sizeof(end),0x22);assert(boot==&running);
    assert(probe_product_runtime_request_activation(&owner_runtime,&peer10)==DBT_OK);
    assert(probe_product_worker_poll_guarded(&s,false,false) && boot==&running &&
           probe_product_runtime_take_activation(&owner_runtime)==PROBE_PRODUCT_NO_RESULT);
    probe_product_worker_tick(&s);
    assert(probe_product_runtime_take_activation(&owner_runtime)==PROBE_PRODUCT_SELECTED);boot=&running;
    apply_name=1;command(&state,&control,&peer10,name,sizeof(name),0x10);assert(gatt_updates==1);
    apply_name=2;unsigned before_commits=commits;
    command(&state,&control,&peer10,later,sizeof(later),0x11);
    assert(commits==before_commits+1 && owner_runtime.retry_pending && gatt_updates==1);
    test_clock+=1000;apply_name=1;probe_product_runtime_poll(&owner_runtime);probe_product_runtime_poll(&owner_runtime);
    assert(gatt_updates==2 && commits==before_commits+1 && !strcmp(durable_name,"Late"));
    runtime_model.acl.links[0].encrypted=false;probe_product_runtime_refresh_security(&owner_runtime);
    assert(probe_product_runtime_grant_ota(&owner_runtime,&peer10,0)==DBT_INVALID);
    command(&state,&control,&peer10,start,sizeof(start),0x11);
    runtime_model.acl.links[0].encrypted=true;secure_peer=false;probe_product_runtime_poll(&owner_runtime);
    assert(probe_product_runtime_grant_ota(&owner_runtime,&peer10,0)==DBT_INVALID);secure_peer=true;
    security_bond=false;assert(probe_product_runtime_grant_ota(&owner_runtime,&peer10,0)==DBT_INVALID);security_bond=true;
    security_key_size=15;assert(probe_product_runtime_grant_ota(&owner_runtime,&peer10,0)==DBT_INVALID);security_key_size=16;
    runtime_model.acl.links[0].generation=11;probe_product_runtime_poll(&owner_runtime);
    dbt_ble_control_peer_t peer11={7,11};command(&state,&control,&peer11,start,sizeof(start),0x11);
    assert(probe_product_runtime_grant_ota(&owner_runtime,&peer10,0)==DBT_INVALID);
    atomic_store(&s.peer.sequence,UINT32_MAX);dbt_product_peer_value_t snap;assert(!dbt_product_peer_read(&s.peer,&snap));
    /* Full automatic owner handoff: a committed encrypted but untrusted peer
     * cannot start, and a re-pair revokes already queued work by epoch. */
    boot=&running;test_clock=0;trust_peer=false;bond_version=1;
    assert(probe_product_startup_worker(&s,&state,&hooks));runtime_model.acl.links[0].generation=20;
    assert(probe_product_runtime_init(&owner_runtime,&s,&runtime_model,&gap_model,security));
    assert(probe_product_runtime_set_trust(&owner_runtime,trust));dbt_ble_control_peer_t peer20={7,20};
    assert(!probe_product_runtime_ota_admit(&owner_runtime,&peer20,0x20,start+1,4));
    probe_product_runtime_poll(&owner_runtime);assert(!dbt_product_peer_read(&s.peer,&snap));
    trust_peer=true;
    dbt_ota_manifest_t explicit_manifest={0};memcpy(explicit_manifest.project,"native_bt_probe",16);
    explicit_manifest.length=9;explicit_manifest.trusted_digest=true;explicit_manifest.crc32=0xcbf43926u;
    memcpy(explicit_manifest.sha256,known_sha,32);
    assert(probe_product_runtime_grant_ota(&owner_runtime,&peer20,&explicit_manifest)==DBT_OK);
    const uint8_t wrong_length[]={8,0,0,0};
    assert(!probe_product_runtime_ota_admit(&owner_runtime,&peer20,0x20,wrong_length,4));
    uint32_t explicit_epoch=probe_product_runtime_ota_admit(&owner_runtime,&peer20,0x20,start+1,4);
    assert(explicit_epoch && explicit_epoch==owner_runtime.grant_epoch);
    probe_product_runtime_ota_abandon(&owner_runtime,&peer20,explicit_epoch,0x20);
    uint32_t epoch=probe_product_runtime_ota_admit(&owner_runtime,&peer20,0x20,start+1,4);
    assert(epoch && runtime_ota(&owner_runtime,&peer20,epoch,start,sizeof(start))==0x21);
    bond_version=2;assert(!probe_product_runtime_ota_valid(&owner_runtime,&peer20,epoch));
    assert(runtime_ota(&owner_runtime,&peer20,epoch,data,sizeof(data))==0x11);
    probe_product_worker_tick(&s);assert(s.ota.state==DBT_OTA_IDLE && !s.esp.session);
    epoch=probe_product_runtime_ota_admit(&owner_runtime,&peer20,0x20,start+1,4);
    assert(epoch && runtime_ota(&owner_runtime,&peer20,epoch,start,sizeof(start))==0x21);
    test_clock=100;probe_product_worker_tick(&s);probe_product_runtime_poll(&owner_runtime);
    assert(!owner_runtime.grant_epoch && s.ota.state==DBT_OTA_IDLE);test_clock=0;
    uint32_t next=probe_product_runtime_ota_admit(&owner_runtime,&peer20,0x20,start+1,4);
    assert(next>epoch && runtime_ota(&owner_runtime,&peer20,epoch,start,sizeof(start))==0x11);
    oom=true;assert(runtime_ota(&owner_runtime,&peer20,next,start,sizeof(start))==0x23 && !s.esp.session);
    oom=false;probe_product_runtime_ota_abandon(&owner_runtime,&peer20,next,0x20);
    epoch=probe_product_runtime_ota_admit(&owner_runtime,&peer20,0x20,start+1,4);
    assert(epoch>next && runtime_ota(&owner_runtime,&peer20,epoch,start,sizeof(start))==0x21);
    assert(runtime_ota(&owner_runtime,&peer20,epoch,data,sizeof(data))==0x20);
    assert(runtime_ota(&owner_runtime,&peer20,epoch,end,sizeof(end))==0x22 && boot==&running);
    assert(!probe_product_runtime_ota_admit(&owner_runtime,&peer20,0x21,data+1,sizeof(data)-1));
    assert(!probe_product_runtime_restart_ready(&owner_runtime,test_clock,true));
    assert(probe_product_runtime_ota_finish(&owner_runtime,&peer20,epoch)==DBT_BUSY);
    assert(probe_product_worker_poll_guarded(&s,false,false) && boot==&running);
    assert(probe_product_runtime_ota_finish(&owner_runtime,&peer20,epoch)==DBT_BUSY);
    probe_product_worker_tick(&s);assert(probe_product_runtime_ota_finish(&owner_runtime,&peer20,epoch)==DBT_OK && boot==&update);
    assert(!probe_product_runtime_restart_ready(&owner_runtime,test_clock,true));
    probe_product_runtime_ota_delivered(&owner_runtime,&peer20,epoch);
    assert(!probe_product_runtime_restart_ready(&owner_runtime,499,true));
    assert(!probe_product_runtime_restart_ready(&owner_runtime,500,false));
    assert(probe_product_runtime_restart_ready(&owner_runtime,500,true));
    assert(!probe_product_runtime_restart_ready(&owner_runtime,501,true));
    /* Selector failure can have changed boot metadata, but never produces a
     * selected result or restart permission. */
    boot=&running;assert(probe_product_startup_worker(&s,&state,&hooks));
    assert(probe_product_runtime_init(&owner_runtime,&s,&runtime_model,&gap_model,security));
    assert(probe_product_runtime_set_trust(&owner_runtime,trust));
    epoch=probe_product_runtime_ota_admit(&owner_runtime,&peer20,0x20,start+1,4);
    assert(epoch && runtime_ota(&owner_runtime,&peer20,epoch,start,sizeof(start))==0x21 &&
           runtime_ota(&owner_runtime,&peer20,epoch,data,sizeof(data))==0x20 &&
           runtime_ota(&owner_runtime,&peer20,epoch,end,sizeof(end))==0x22);
    select_failure=2;assert(probe_product_runtime_ota_finish(&owner_runtime,&peer20,epoch)==DBT_BUSY);
    probe_product_worker_tick(&s);assert(boot==&update && probe_product_runtime_ota_finish(&owner_runtime,&peer20,epoch)==DBT_INVALID);
    probe_product_runtime_ota_delivered(&owner_runtime,&peer20,epoch);
    assert(!probe_product_runtime_restart_ready(&owner_runtime,1000,true));
    select_failure=0;boot=&running;
    /* Concurrent revocation after selector admission may leave the selector
     * changed, but is never reported as selected or restartable. */
    assert(probe_product_startup_worker(&s,&state,&hooks));
    assert(probe_product_runtime_init(&owner_runtime,&s,&runtime_model,&gap_model,security));
    assert(probe_product_runtime_set_trust(&owner_runtime,trust));
    epoch=probe_product_runtime_ota_admit(&owner_runtime,&peer20,0x20,start+1,4);
    assert(epoch && runtime_ota(&owner_runtime,&peer20,epoch,start,sizeof(start))==0x21 &&
           runtime_ota(&owner_runtime,&peer20,epoch,data,sizeof(data))==0x20 &&
           runtime_ota(&owner_runtime,&peer20,epoch,end,sizeof(end))==0x22);
    assert(probe_product_runtime_ota_finish(&owner_runtime,&peer20,epoch)==DBT_BUSY);
    during_select=revoke_during_select;probe_product_worker_tick(&s);during_select=NULL;
    assert(boot==&update && probe_product_runtime_ota_finish(&owner_runtime,&peer20,epoch)==DBT_INVALID);
    probe_product_runtime_ota_delivered(&owner_runtime,&peer20,epoch);
    assert(!probe_product_runtime_restart_ready(&owner_runtime,1000,true));boot=&running;
    assert(probe_product_startup_worker(&s,&state,&hooks));
    assert(probe_product_runtime_init(&owner_runtime,&s,&runtime_model,&gap_model,security));
    assert(probe_product_runtime_set_trust(&owner_runtime,trust));
    assert(!probe_product_request_activation(&s,&peer20));
    epoch=probe_product_runtime_ota_admit(&owner_runtime,&peer20,0x20,start+1,4);
    assert(epoch && runtime_ota(&owner_runtime,&peer20,epoch,start,sizeof(start))==0x21 &&
           runtime_ota(&owner_runtime,&peer20,epoch,data,sizeof(data))==0x20 &&
           runtime_ota(&owner_runtime,&peer20,epoch,end,sizeof(end))==0x22);
    unsigned before_boot=boot_writes;runtime_model.acl.links[0].used=false;
    assert(probe_product_runtime_ota_finish(&owner_runtime,&peer20,epoch)==DBT_INVALID);
    probe_product_worker_tick(&s);assert(boot_writes==before_boot && !s.esp.session);
    runtime_model.acl.links[0].used=true;boot=&running;test_clock=0;
    assert(probe_product_startup_worker(&s,&state,&hooks));
    assert(probe_product_runtime_init(&owner_runtime,&s,&runtime_model,&gap_model,security));
    assert(probe_product_runtime_set_trust(&owner_runtime,trust));
    epoch=probe_product_runtime_ota_admit(&owner_runtime,&peer20,0x20,start+1,4);
    assert(epoch && runtime_ota(&owner_runtime,&peer20,epoch,start,sizeof(start))==0x21 &&
           runtime_ota(&owner_runtime,&peer20,epoch,data,sizeof(data))==0x20 &&
           runtime_ota(&owner_runtime,&peer20,epoch,end,sizeof(end))==0x22);
    assert(probe_product_runtime_ota_finish(&owner_runtime,&peer20,epoch)==DBT_BUSY);
    probe_product_worker_tick(&s);assert(probe_product_runtime_ota_finish(&owner_runtime,&peer20,epoch)==DBT_OK);
    probe_product_runtime_ota_delivered(&owner_runtime,&peer20,epoch);
    assert(!probe_product_runtime_restart_ready(&owner_runtime,10000,true) && !owner_runtime.restart_pending);
    return 0;
}
