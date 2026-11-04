#include "probe_product_runtime.h"
#include "probe_gatt.h"
#include <string.h>
static bool same_peer(const dbt_ble_control_peer_t *a,const dbt_ble_control_peer_t *b){return a->handle==b->handle && a->generation && a->generation==b->generation;}
static bool eligible(probe_product_runtime_t *s,const dbt_link_t *link) {
    dbt_smp_session_security_t sec;
    return link && link->used && link->kind==DBT_LINK_LE && link->encrypted && s->security &&
           s->security(link->handle,link->generation,&sec)==DBT_OK && sec.encrypted && sec.bond_committed && sec.key_size>=16;
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
static void update_security(probe_product_runtime_t *s) {
    dbt_product_peer_value_t value={0};unsigned eligible_count=0;
    for(unsigned i=0;i<DBT_MAX_LINKS;i++) {
        const dbt_link_t *link=&s->runtime->acl.links[i];
        if(!eligible(s,link))continue;
        ++eligible_count;value.peer.handle=link->handle;value.peer.generation=link->generation;
    }
    if(eligible_count==1) {
        value.encrypted=value.bonded=value.allow_name=true;
        if(s->ota_granted && same_peer(&value.peer,&s->granted_peer)){value.allow_ota=true;value.ota=s->grant;}
    } else memset(&value,0,sizeof(value));
    if(s->ota_granted && !same_peer(&value.peer,&s->granted_peer)) {s->ota_granted=false;memset(&s->grant,0,sizeof(s->grant));}
    if(!s->have_current || !same_value(&value,&s->current)) {
        if(probe_product_publish_peer(s->product,&value)) {s->current=value;s->have_current=true;}
        else {s->have_current=false;s->ota_granted=false;}
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
    if(s->retry_pending)return (int32_t)(now-s->retry_at)>=0?0u:(uint32_t)(s->retry_at-now);
    return UINT32_MAX;
}
dbt_result_t probe_product_runtime_grant_ota(probe_product_runtime_t *s,const dbt_ble_control_peer_t *peer,const dbt_ota_manifest_t *manifest) {
    if(!s || !s->product || !peer)return DBT_INVALID;
    const dbt_link_t *link=dbt_acl_find(&s->runtime->acl,peer->handle);
    if(!link || link->generation!=peer->generation || !eligible(s,link))return DBT_INVALID;
    dbt_ota_manifest_t m={0};memcpy(m.project,"native_bt_probe",16);
    if(manifest) {
        if(memcmp(manifest->project,m.project,32) || (manifest->trusted_digest && !manifest->length))return DBT_INVALID;
        m=*manifest;
    }
    s->grant=m;s->granted_peer=*peer;s->ota_granted=true;update_security(s);
    if(!s->have_current || !s->current.allow_ota || !same_peer(&s->current.peer,peer))return DBT_BUSY;
    return DBT_OK;
}
dbt_result_t probe_product_runtime_request_activation(probe_product_runtime_t *s,const dbt_ble_control_peer_t *peer) {
    if(!s || !s->product || !peer)return DBT_INVALID;
    update_security(s);
    if(!s->ota_granted || !s->have_current || !s->current.allow_ota || !same_peer(&s->granted_peer,peer))return DBT_INVALID;
    return probe_product_request_activation(s->product,peer)?DBT_OK:DBT_BUSY;
}
probe_product_action_result_t probe_product_runtime_take_activation(probe_product_runtime_t *s) {
    return s && s->product?probe_product_take_activation_result(s->product):PROBE_PRODUCT_NO_RESULT;
}
