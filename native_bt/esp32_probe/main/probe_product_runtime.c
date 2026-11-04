#include "probe_product_runtime.h"
#include "probe_gatt.h"
#include <string.h>
static bool same_peer(const dbt_ble_control_peer_t *a,const dbt_ble_control_peer_t *b){return a->handle==b->handle && a->generation && a->generation==b->generation;}
static bool eligible(probe_product_runtime_t *s,const dbt_link_t *link) {
    dbt_smp_session_security_t sec;
    return link && link->used && link->kind==DBT_LINK_LE && link->encrypted && s->security &&
           s->security(link->handle,link->generation,&sec)==DBT_OK && sec.encrypted && sec.bond_committed && sec.key_size>=16;
}
static bool trusted(probe_product_runtime_t *s,const dbt_ble_control_peer_t *peer,uint8_t *type,uint8_t id[6],uint8_t tag[32]) {
    const dbt_link_t *link=dbt_acl_find(&s->runtime->acl,peer->handle);
    return link && link->generation==peer->generation && eligible(s,link) && s->trust &&
           s->trust(peer->handle,peer->generation,type,id,tag)==DBT_OK;
}
static void revoke(probe_product_runtime_t *s) {
    if(s->ota_granted || s->grant_epoch)probe_product_ota_epoch(s->product,0);
    s->ota_granted=false;s->grant_epoch=0;s->selected_epoch=0;s->restart_pending=false;
    s->begin_admitted=false;s->finish_admitted=false;
    memset(&s->grant,0,sizeof(s->grant));memset(s->trust_identity,0,6);memset(s->trust_tag,0,32);
}
static bool current_trust(probe_product_runtime_t *s,const dbt_ble_control_peer_t *peer) {
    uint8_t type,id[6],tag[32];
    return s->ota_granted && s->grant_epoch &&
           atomic_load_explicit(&s->product->admission_epoch,memory_order_acquire)==s->grant_epoch &&
           same_peer(peer,&s->granted_peer) && trusted(s,peer,&type,id,tag) &&
           type==s->trust_type && !memcmp(id,s->trust_identity,6) && !memcmp(tag,s->trust_tag,32);
}
static bool same_value(const dbt_product_peer_value_t *a,const dbt_product_peer_value_t *b){
    return a->peer.handle==b->peer.handle && a->peer.generation==b->peer.generation && a->encrypted==b->encrypted &&
           a->bonded==b->bonded && a->allow_name==b->allow_name && a->allow_ota==b->allow_ota &&
           a->ota.length==b->ota.length && a->ota.crc32==b->ota.crc32 && a->ota.trusted_digest==b->ota.trusted_digest &&
           !memcmp(a->ota.sha256,b->ota.sha256,32) && !memcmp(a->ota.project,b->ota.project,32);
}
bool probe_product_runtime_init(probe_product_runtime_t *s,probe_product_t *product,dbt_runtime_t *runtime,dbt_gap_t *gap,
                                dbt_result_t (*security)(uint16_t,uint32_t,dbt_smp_session_security_t *)) {
    if(!s || !product || !product->ready || !runtime || !gap || !security)return false;
    memset(s,0,sizeof(*s));s->product=product;s->runtime=runtime;s->gap=gap;s->security=security;return true;
}
bool probe_product_runtime_set_trust(probe_product_runtime_t *s,
    dbt_result_t (*query)(uint16_t,uint32_t,uint8_t *,uint8_t[6],uint8_t[32])) {
    if(!s || !s->product || !query || s->ota_granted)return false;
    s->trust=query;return true;
}
static void update_security(probe_product_runtime_t *s) {
    if(s->grant_epoch && atomic_load_explicit(&s->product->terminal_epoch,memory_order_acquire)==s->grant_epoch)
        revoke(s);
    dbt_product_peer_value_t value={0};unsigned eligible_count=0;
    for(unsigned i=0;i<DBT_MAX_LINKS;i++) {
        const dbt_link_t *link=&s->runtime->acl.links[i];
        if(!eligible(s,link))continue;
        ++eligible_count;value.peer.handle=link->handle;value.peer.generation=link->generation;
    }
    uint8_t peer_type,peer_id[6],peer_tag[32];
    if(eligible_count==1 && trusted(s,&value.peer,&peer_type,peer_id,peer_tag)) {
        value.encrypted=value.bonded=value.allow_name=true;
        if(s->ota_granted && current_trust(s,&value.peer)){value.allow_ota=true;value.ota=s->grant;}
    } else memset(&value,0,sizeof(value));
    if(s->ota_granted && !value.allow_ota)revoke(s);
    if(!s->ota_granted && s->activation_started &&
       probe_product_take_activation_result(s->product)!=PROBE_PRODUCT_NO_RESULT)s->activation_started=false;
    if(!s->have_current || !same_value(&value,&s->current)) {
        if(probe_product_publish_peer(s->product,&value)) {s->current=value;s->have_current=true;}
        else {s->have_current=false;revoke(s);}
    }
}
static void update_name(probe_product_runtime_t *s) {
    uint32_t now=s->product->hooks.now_ms(s->product->hooks.context);
    if(s->retry_pending) {if((int32_t)(now-s->retry_at)<0)return;s->retry_pending=false;}
    if(!s->have_name) {if(!probe_product_take_name(s->product,&s->name))return;s->have_name=true;s->rename_token=0;}
    if(!s->rename_token) {
        dbt_result_t r=dbt_gap_rename(s->gap,(const uint8_t *)s->name.name,s->name.length,&s->rename_token);
        if(r==DBT_BUSY)return;
        if(r!=DBT_OK) {if(probe_product_complete_name(s->product,s->name.token,false)){s->have_name=false;s->retry_at=now+1000u;s->retry_pending=true;}return;}
    }
    bool applied=false;uint8_t status=0;
    dbt_result_t result=dbt_gap_rename_result(s->gap,s->rename_token,&applied,&status);
    if(result==DBT_BUSY || (result==DBT_EMPTY && s->gap->rename_serial==s->rename_token))return;
    if(result==DBT_OK && applied)applied=probe_gatt_control_name_applied((const uint8_t *)s->name.name,s->name.length)==DBT_OK;
    else applied=false;
    if(probe_product_complete_name(s->product,s->name.token,applied)){
        s->have_name=false;s->rename_token=0;if(!applied){s->retry_at=now+1000u;s->retry_pending=true;}
    }
}
void probe_product_runtime_refresh_security(probe_product_runtime_t *s) {if(s && s->product)update_security(s);}
void probe_product_runtime_poll(probe_product_runtime_t *s) {if(s && s->product){update_security(s);update_name(s);}}
uint32_t probe_product_runtime_wait_ms(const probe_product_runtime_t *s,uint32_t now) {
    if(!s || !s->product)return UINT32_MAX;
    uint32_t wait=s->retry_pending?((int32_t)(now-s->retry_at)>=0?0u:(uint32_t)(s->retry_at-now)):UINT32_MAX;
    if(s->restart_pending) {
        uint32_t restart=(int32_t)(now-s->restart_at)>=0?20u:(uint32_t)(s->restart_at-now);
        if(restart<wait)wait=restart;
    }
    return wait;
}
dbt_result_t probe_product_runtime_grant_ota(probe_product_runtime_t *s,const dbt_ble_control_peer_t *peer,const dbt_ota_manifest_t *manifest) {
    if(!s || !s->product || !peer)return DBT_INVALID;
    uint8_t type,id[6],tag[32];
    if(!trusted(s,peer,&type,id,tag))return DBT_INVALID;
    if(s->ota_granted || s->grant_serial==UINT32_MAX || s->activation_started ||
       atomic_load_explicit(&s->product->action_phase,memory_order_acquire)!=0)return DBT_BUSY;
    dbt_ota_manifest_t m={0};memcpy(m.project,"native_bt_probe",16);
    if(manifest) {
        if(memcmp(manifest->project,m.project,32) || (manifest->trusted_digest && !manifest->length))return DBT_INVALID;
        m=*manifest;
    }
    s->grant=m;s->granted_peer=*peer;s->ota_granted=true;s->trust_type=type;
    memcpy(s->trust_identity,id,6);memcpy(s->trust_tag,tag,32);
    s->grant_epoch=++s->grant_serial;probe_product_ota_epoch(s->product,s->grant_epoch);update_security(s);
    if(!s->have_current || !s->current.allow_ota || !same_peer(&s->current.peer,peer))return DBT_BUSY;
    return DBT_OK;
}
uint32_t probe_product_runtime_ota_admit(void *context,const dbt_ble_control_peer_t *peer,uint8_t cmd,const uint8_t *p,uint16_t n) {
    probe_product_runtime_t *s=context;if(!s || !peer || cmd<0x20 || cmd>0x23)return 0;
    update_security(s);
    if(cmd==0x20) {
        if(!p || n!=4 || !(p[0]|p[1]|p[2]|p[3]) || s->begin_admitted)return 0;
        if(!s->ota_granted && probe_product_runtime_grant_ota(s,peer,NULL)!=DBT_OK)return 0;
        uint32_t length=(uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);
        if(!current_trust(s,peer) || (s->grant.length && s->grant.length!=length))return 0;
        s->begin_admitted=true;
    } else if(!s->begin_admitted || !current_trust(s,peer) || s->activation_started || s->finish_admitted ||
              atomic_load_explicit(&s->product->verified_epoch,memory_order_acquire)==s->grant_epoch)return 0;
    if((cmd==0x21 && (!p || n<2)) || ((cmd==0x22 || cmd==0x23) && n))return 0;
    if(cmd==0x22)s->finish_admitted=true;
    return s->grant_epoch;
}
void probe_product_runtime_ota_abandon(void *context,const dbt_ble_control_peer_t *peer,uint32_t epoch,uint8_t cmd) {
    probe_product_runtime_t *s=context;(void)cmd;if(s && epoch && epoch==s->grant_epoch && same_peer(peer,&s->granted_peer)){
        revoke(s);update_security(s);
    }
}
bool probe_product_runtime_ota_valid(void *context,const dbt_ble_control_peer_t *peer,uint32_t epoch) {
    probe_product_runtime_t *s=context;
    if(!s || !peer || !epoch || epoch!=s->grant_epoch || !current_trust(s,peer)) {
        if(s){revoke(s);update_security(s);}return false;
    }
    return true;
}
bool probe_product_runtime_ota_worker(void *context,const dbt_ble_control_peer_t *peer,uint32_t epoch,
    uint8_t cmd,const uint8_t *p,uint16_t n,uint8_t *out,uint8_t *size) {
    probe_product_runtime_t *s=context;
    return s && probe_product_ota_command_epoch(s->product,peer,epoch,cmd,p,n,out,size);
}
dbt_result_t probe_product_runtime_ota_finish(void *context,const dbt_ble_control_peer_t *peer,uint32_t epoch) {
    probe_product_runtime_t *s=context;
    if(!s || !peer || !epoch || epoch!=s->grant_epoch || !current_trust(s,peer)) {
        if(s){revoke(s);update_security(s);}return DBT_INVALID;
    }
    if(!s->activation_started) {
        if(!probe_product_request_activation(s->product,peer))return DBT_BUSY;
        s->activation_started=true;return DBT_BUSY;
    }
    probe_product_action_result_t r=probe_product_take_activation_result(s->product);
    if(r==PROBE_PRODUCT_NO_RESULT)return DBT_BUSY;
    s->activation_started=false;
    if(r!=PROBE_PRODUCT_SELECTED){revoke(s);update_security(s);return DBT_INVALID;}
    if(!current_trust(s,peer)){revoke(s);update_security(s);return DBT_INVALID;}
    s->selected_epoch=epoch;return DBT_OK;
}
void probe_product_runtime_ota_delivered(void *context,const dbt_ble_control_peer_t *peer,uint32_t epoch) {
    probe_product_runtime_t *s=context;
    if(!s || !peer || !epoch || s->selected_epoch!=epoch || s->grant_epoch!=epoch || !current_trust(s,peer))return;
    uint32_t now=s->product->hooks.now_ms(s->product->hooks.context);
    s->restart_at=now+500u;s->restart_deadline=now+10000u;s->restart_pending=true;
}
bool probe_product_runtime_restart_ready(probe_product_runtime_t *s,uint32_t now,bool all_quiescent) {
    if(!s || !s->restart_pending)return false;
    if(!current_trust(s,&s->granted_peer) || !s->selected_epoch || s->selected_epoch!=s->grant_epoch) {
        revoke(s);update_security(s);return false;
    }
    if((int32_t)(now-s->restart_deadline)>=0){revoke(s);update_security(s);return false;}
    if(!all_quiescent || (int32_t)(now-s->restart_at)<0)return false;
    s->restart_pending=false;s->selected_epoch=0;return true;
}
dbt_result_t probe_product_runtime_request_activation(probe_product_runtime_t *s,const dbt_ble_control_peer_t *peer) {
    if(!s || !s->product || !peer)return DBT_INVALID;
    update_security(s);
    if(!s->ota_granted || !s->have_current || !s->current.allow_ota || !same_peer(&s->granted_peer,peer))return DBT_INVALID;
    return probe_product_request_activation(s->product,peer)?DBT_OK:DBT_BUSY;
}
probe_product_action_result_t probe_product_runtime_take_activation(probe_product_runtime_t *s) {
    return s && s->product && !s->activation_started?probe_product_take_activation_result(s->product):PROBE_PRODUCT_NO_RESULT;
}
