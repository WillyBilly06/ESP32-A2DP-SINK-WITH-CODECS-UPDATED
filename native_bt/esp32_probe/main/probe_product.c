#include "probe_product.h"
#include "dbt_utf8.h"
#include "dbt_ble_nvs.h"
#include "dbt_boot_policy.h"
#include "esp_ota_ops.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

enum {NAME_IDLE,NAME_READY,NAME_TAKEN,NAME_OK,NAME_FAILED,NAME_ABANDONED,NAME_TAKEN_ABANDONED};
enum {ACTION_IDLE,ACTION_QUEUED,ACTION_RUNNING,ACTION_OK,ACTION_REJECTED,ACTION_AMBIGUOUS};
static bool same_peer(const dbt_ble_control_peer_t *a,const dbt_ble_control_peer_t *b) {
    return a->handle==b->handle && a->generation==b->generation && a->generation;
}
static bool authorized(probe_product_t *s,const dbt_ble_control_peer_t *p,bool ota,dbt_product_peer_value_t *out) {
    dbt_product_peer_value_t value;
    if(!dbt_product_peer_read(&s->peer,&value) || !same_peer(&value.peer,p) || !value.encrypted || !value.bonded ||
       (ota?!value.allow_ota:!value.allow_name))return false;
    if(out)*out=value;
    return true;
}
static bool ota_grant(void *context,const dbt_ble_control_peer_t *peer,uint32_t length,dbt_ota_manifest_t *m) {
    probe_product_t *s=context;dbt_product_peer_value_t v;
    uint32_t epoch=atomic_load_explicit(&s->admission_epoch,memory_order_acquire);
    if(!epoch || ((s->ota.state==DBT_OTA_RECEIVING || s->ota.state==DBT_OTA_VERIFIED) &&
                  s->ota_session_epoch!=epoch))return false;
    if(!authorized(s,peer,true,&v) || (v.ota.length && v.ota.length!=length))return false;
    *m=v.ota;m->length=length;return true;
}
static uint32_t clock_ms(void *context) {probe_product_t *s=context;return s->hooks.now_ms(s->hooks.context);}
static bool ota_begin(void *v,const dbt_ota_manifest_t *m){return dbt_ota_esp_begin(&((probe_product_t *)v)->esp,m);}
static bool ota_write(void *v,uint32_t n,const uint8_t *p,uint16_t z){return dbt_ota_esp_write(&((probe_product_t *)v)->esp,n,p,z);}
static bool ota_finish(void *v,const dbt_ota_manifest_t *m){return dbt_ota_esp_finish(&((probe_product_t *)v)->esp,m);}
static bool ota_abort(void *v){return dbt_ota_esp_abort(&((probe_product_t *)v)->esp);}
static bool ota_activate(void *v,const dbt_ota_manifest_t *m){return dbt_ota_esp_activate(&((probe_product_t *)v)->esp,m);}
bool probe_product_startup_worker(probe_product_t *s,dbt_ble_control_state_t *initial,const probe_product_hooks_t *h) {
    if(!s || !initial || !h || !h->now_ms || !h->wake_owner || !h->wake_worker ||
       !h->ota_timeout_ms || !h->name_timeout_ms || h->name_timeout_ms>0x7fffffffu)return false;
    memset(s,0,sizeof(*s));s->hooks=*h;dbt_product_peer_init(&s->peer);
    atomic_init(&s->name_phase,0);atomic_init(&s->name_token,0);atomic_init(&s->name_handle,0);
    atomic_init(&s->name_generation,0);atomic_init(&s->name_length,0);
    for(unsigned i=0;i<8;i++)atomic_init(&s->name_words[i],0);
    atomic_init(&s->action_phase,0);atomic_init(&s->action_handle,0);atomic_init(&s->action_generation,0);
    atomic_init(&s->admission_epoch,0);atomic_init(&s->verified_epoch,0);atomic_init(&s->action_epoch,0);
    atomic_init(&s->terminal_epoch,0);
    if(dbt_ble_nvs_load_state(initial)!=DBT_OK)return false;
    const esp_partition_t *running=esp_ota_get_running_partition(),*boot=esp_ota_get_boot_partition();
    esp_app_desc_t desc;
    if(!running || !boot || running->address!=boot->address || running->size!=boot->size ||
       esp_ota_get_partition_description(running,&desc)!=ESP_OK ||
       !dbt_boot_owned_slot(running->type,running->subtype,running->label,desc.project_name))return false;
    const char *other=running->subtype==ESP_PARTITION_SUBTYPE_APP_OTA_MIN?"ota_1":"ota_0";
    if(!dbt_ota_esp_init(&s->esp,other))return false;
    dbt_ota_hooks_t hooks={s,ota_grant,clock_ms,ota_begin,ota_write,ota_finish,ota_abort,ota_activate};
    if(!dbt_ota_init(&s->ota,&hooks,h->ota_timeout_ms))return false;
    s->ready=true;return true;
}
bool probe_product_publish_peer(probe_product_t *s,const dbt_product_peer_value_t *v) {
    if(!s || !s->ready || !v)return false;
    dbt_product_peer_value_t previous;bool had=dbt_product_peer_read(&s->peer,&previous);
    if(!dbt_product_peer_publish(&s->peer,v)) {dbt_ota_request_cancel(&s->ota);s->hooks.wake_worker(s->hooks.context);return false;}
    if(!had || !same_peer(&previous.peer,&v->peer) || previous.encrypted!=v->encrypted || previous.bonded!=v->bonded ||
       previous.allow_ota!=v->allow_ota || previous.ota.length!=v->ota.length || previous.ota.crc32!=v->ota.crc32 ||
       previous.ota.trusted_digest!=v->ota.trusted_digest || memcmp(previous.ota.sha256,v->ota.sha256,32) ||
       memcmp(previous.ota.project,v->ota.project,32))dbt_ota_request_cancel(&s->ota);
    s->hooks.wake_worker(s->hooks.context);return true;
}
static bool name_commit(const uint8_t *p,uint16_t n) {
    char value[33];if(!n || n>32 || memchr(p,0,n))return false;
    memcpy(value,p,n);value[n]=0;nvs_handle_t h;
    if(nvs_open("audio",NVS_READWRITE,&h)!=ESP_OK)return false;
    esp_err_t e=nvs_set_str(h,"devname",value);if(e==ESP_OK)e=nvs_commit(h);nvs_close(h);
    memset(value,0,sizeof(value));return e==ESP_OK;
}
static bool name_command(probe_product_t *s,const dbt_ble_control_peer_t *peer,const uint8_t *p,uint16_t n,uint8_t *out,uint8_t *size) {
    if(!authorized(s,peer,false,0) || !n || n>32 || !dbt_utf8_valid(p,n) ||
       atomic_load_explicit(&s->name_phase,memory_order_seq_cst)!=NAME_IDLE || s->next_name_token==UINT32_MAX)return false;
    if(!name_commit(p,n))return false;
    uint8_t name[32]={0};memcpy(name,p,n);
    for(unsigned i=0;i<8;i++){uint32_t word;memcpy(&word,name+4*i,4);atomic_store_explicit(&s->name_words[i],word,memory_order_seq_cst);}
    uint32_t token=++s->next_name_token;
    atomic_store_explicit(&s->name_token,token,memory_order_seq_cst);
    atomic_store_explicit(&s->name_handle,peer->handle,memory_order_seq_cst);
    atomic_store_explicit(&s->name_generation,peer->generation,memory_order_seq_cst);
    atomic_store_explicit(&s->name_length,n,memory_order_seq_cst);
    atomic_store_explicit(&s->name_phase,NAME_READY,memory_order_seq_cst);s->hooks.wake_owner(s->hooks.context);
    uint32_t start=clock_ms(s);
    for(;;) {
        uint32_t phase=atomic_load_explicit(&s->name_phase,memory_order_seq_cst);
        if(phase==NAME_OK || phase==NAME_FAILED) {
            atomic_store_explicit(&s->name_phase,phase==NAME_OK?NAME_IDLE:NAME_ABANDONED,memory_order_seq_cst);
            if(phase!=NAME_OK){s->hooks.wake_owner(s->hooks.context);return false;}
            if(!authorized(s,peer,false,0))return false;
            out[0]=0x10;out[1]=4;*size=2;return true;
        }
        if((uint32_t)(clock_ms(s)-start)>=s->hooks.name_timeout_ms) {
            uint32_t expected=phase;
            if((phase==NAME_READY || phase==NAME_TAKEN) && atomic_compare_exchange_strong_explicit(&s->name_phase,&expected,
                phase==NAME_READY?NAME_ABANDONED:NAME_TAKEN_ABANDONED,memory_order_seq_cst,memory_order_seq_cst)) {
                s->hooks.wake_owner(s->hooks.context);
                return false;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
bool probe_product_command(void *context,const dbt_ble_control_peer_t *peer,uint8_t cmd,const uint8_t *data,uint16_t n,uint8_t *out,uint8_t *size) {
    probe_product_t *s=context;
    if(!s || !s->ready || !peer || (!data && n) || !out || !size || *size<2)return false;
    if(cmd==4)return name_command(s,peer,data,n,out,size);
    if(cmd>=0x20 && cmd<=0x23) {
        /* OTA must use the epoch-aware worker entry point. */
        return false;
    }
    if(!authorized(s,peer,false,0) || !s->hooks.other_command)return false;
    return s->hooks.other_command(s->hooks.context,peer,cmd,data,n,out,size);
}
void probe_product_ota_epoch(probe_product_t *s,uint32_t epoch) {
    if(!s || !s->ready)return;
    if(epoch)atomic_store_explicit(&s->terminal_epoch,0,memory_order_release);
    atomic_store_explicit(&s->admission_epoch,epoch,memory_order_release);
    if(!epoch){dbt_ota_request_cancel(&s->ota);s->hooks.wake_worker(s->hooks.context);}
}
static void ota_tick(probe_product_t *s) {
    dbt_ota_state_t before=s->ota.state;dbt_ota_tick(&s->ota);
    if((before==DBT_OTA_RECEIVING || before==DBT_OTA_VERIFIED) &&
       (s->ota.state==DBT_OTA_IDLE || s->ota.state==DBT_OTA_FAULT)) {
        atomic_store_explicit(&s->terminal_epoch,s->ota_session_epoch,memory_order_release);
        s->hooks.wake_owner(s->hooks.context);
    }
}
bool probe_product_ota_command_epoch(void *context,const dbt_ble_control_peer_t *peer,uint32_t epoch,
                                    uint8_t cmd,const uint8_t *data,uint16_t n,uint8_t *out,uint8_t *size) {
    probe_product_t *s=context;
    if(!s || !s->ready || !peer || !epoch || cmd<0x20 || cmd>0x23 ||
       epoch!=atomic_load_explicit(&s->admission_epoch,memory_order_acquire) || !authorized(s,peer,true,0))return false;
    if(cmd!=0x20 && s->ota_session_epoch!=epoch)return false;
    bool ok=dbt_ota_command(&s->ota,peer,cmd,data,n,out,size);
    if(ok && *size && cmd==0x20 && out[0]==0x21 && s->ota.state==DBT_OTA_RECEIVING)
        s->ota_session_epoch=epoch;
    ota_tick(s);
    if(!ok || !*size || epoch!=atomic_load_explicit(&s->admission_epoch,memory_order_acquire) ||
       !authorized(s,peer,true,0))return false;
    if(out[0]!=0x23 && cmd!=0x23 &&
       ((cmd==0x22 && s->ota.state!=DBT_OTA_VERIFIED) || (cmd!=0x22 && s->ota.state!=DBT_OTA_RECEIVING)))return false;
    if(cmd==0x22 && out[0]==0x22 && s->ota.state==DBT_OTA_VERIFIED) {
        atomic_store_explicit(&s->verified_epoch,epoch,memory_order_release);s->hooks.wake_owner(s->hooks.context);
    }
    return true;
}
void probe_product_worker_tick(probe_product_t *s) {
    if(!s || !s->ready)return;
    ota_tick(s);uint32_t expected=ACTION_QUEUED;
    if(atomic_compare_exchange_strong_explicit(&s->action_phase,&expected,ACTION_RUNNING,memory_order_seq_cst,memory_order_seq_cst)) {
        dbt_ble_control_peer_t peer={(uint16_t)atomic_load_explicit(&s->action_handle,memory_order_seq_cst),
                                     atomic_load_explicit(&s->action_generation,memory_order_seq_cst)};
        uint32_t epoch=atomic_load_explicit(&s->action_epoch,memory_order_acquire);
        bool admitted=epoch && epoch==atomic_load_explicit(&s->admission_epoch,memory_order_acquire) &&
                      epoch==atomic_load_explicit(&s->verified_epoch,memory_order_acquire);
        bool ok=admitted && authorized(s,&peer,true,0) && dbt_ota_activate(&s->ota,&peer);
        bool stale=epoch!=atomic_load_explicit(&s->admission_epoch,memory_order_acquire) || !authorized(s,&peer,true,0);
        uint32_t result=ok && !stale?ACTION_OK:(s->ota.state==DBT_OTA_FAULT || (ok && stale)?ACTION_AMBIGUOUS:ACTION_REJECTED);
        atomic_store_explicit(&s->action_phase,result,memory_order_seq_cst);s->hooks.wake_owner(s->hooks.context);
    }
}
bool probe_product_worker_quiescent(const probe_product_t *s) {
    return s && s->ready && (s->ota.state==DBT_OTA_IDLE || s->ota.state==DBT_OTA_FAULT) &&
           !s->esp.session && atomic_load_explicit(&s->action_phase,memory_order_seq_cst)!=ACTION_QUEUED;
}
bool probe_product_worker_retire(probe_product_t *s) {
    if(!s || !s->ready)return false;
    if(s->ota.state==DBT_OTA_FAULT && s->esp.session && !dbt_ota_esp_abort(&s->esp))return false;
    return probe_product_worker_quiescent(s);
}
bool probe_product_worker_poll(void *context,bool retiring) {
    return probe_product_worker_poll_guarded(context,retiring,true);
}
bool probe_product_worker_poll_guarded(void *context,bool retiring,bool allow_activation) {
    probe_product_t *s=context;if(!s || !s->ready)return false;
    if(retiring)dbt_ota_request_cancel(&s->ota);
    if(allow_activation)probe_product_worker_tick(s);else ota_tick(s);
    return !retiring || probe_product_worker_retire(s);
}
void probe_product_peer_lost(void *context,const dbt_ble_control_peer_t *peer) {
    probe_product_t *s=context;dbt_product_peer_value_t current,empty={0};
    if(s && s->ready && peer && dbt_product_peer_read(&s->peer,&current) && same_peer(peer,&current.peer))
        (void)probe_product_publish_peer(s,&empty);
}
bool probe_product_take_name(probe_product_t *s,probe_product_name_t *out) {
    if(!s || !s->ready || !out)return false;
    uint32_t phase=atomic_load_explicit(&s->name_phase,memory_order_seq_cst);
    if(phase!=NAME_READY && phase!=NAME_ABANDONED)return false;
    uint32_t desired=phase==NAME_READY?NAME_TAKEN:NAME_TAKEN_ABANDONED;
    if(!atomic_compare_exchange_strong_explicit(&s->name_phase,&phase,desired,memory_order_seq_cst,memory_order_seq_cst))return false;
    memset(out,0,sizeof(*out));out->token=atomic_load_explicit(&s->name_token,memory_order_seq_cst);
    out->handle=(uint16_t)atomic_load_explicit(&s->name_handle,memory_order_seq_cst);
    out->generation=atomic_load_explicit(&s->name_generation,memory_order_seq_cst);
    out->length=(uint8_t)atomic_load_explicit(&s->name_length,memory_order_seq_cst);
    for(unsigned i=0;i<8;i++){uint32_t word=atomic_load_explicit(&s->name_words[i],memory_order_seq_cst);memcpy(out->name+4*i,&word,4);}
    out->name[out->length]=0;return true;
}
bool probe_product_complete_name(probe_product_t *s,uint32_t token,bool applied) {
    if(!s || !s->ready || !token || token!=atomic_load_explicit(&s->name_token,memory_order_seq_cst))return false;
    for(unsigned i=0;i<3;i++) {
        uint32_t phase=atomic_load_explicit(&s->name_phase,memory_order_seq_cst);
        if(phase!=NAME_TAKEN && phase!=NAME_TAKEN_ABANDONED)return false;
        uint32_t desired=phase==NAME_TAKEN_ABANDONED?(applied?NAME_IDLE:NAME_ABANDONED):(applied?NAME_OK:NAME_FAILED);
        if(atomic_compare_exchange_strong_explicit(&s->name_phase,&phase,desired,memory_order_seq_cst,memory_order_seq_cst))return true;
    }
    return false;
}
bool probe_product_request_activation(probe_product_t *s,const dbt_ble_control_peer_t *peer) {
    if(!s || !s->ready || !peer || !authorized(s,peer,true,0) ||
       atomic_load_explicit(&s->action_phase,memory_order_seq_cst)!=ACTION_IDLE)return false;
    uint32_t epoch=atomic_load_explicit(&s->admission_epoch,memory_order_acquire);
    if(!epoch || epoch!=atomic_load_explicit(&s->verified_epoch,memory_order_acquire))return false;
    atomic_store_explicit(&s->action_epoch,epoch,memory_order_release);
    atomic_store_explicit(&s->action_handle,peer->handle,memory_order_seq_cst);
    atomic_store_explicit(&s->action_generation,peer->generation,memory_order_seq_cst);
    atomic_store_explicit(&s->action_phase,ACTION_QUEUED,memory_order_seq_cst);s->hooks.wake_worker(s->hooks.context);return true;
}
probe_product_action_result_t probe_product_take_activation_result(probe_product_t *s) {
    if(!s || !s->ready)return PROBE_PRODUCT_NO_RESULT;
    uint32_t phase=atomic_load_explicit(&s->action_phase,memory_order_seq_cst);
    if(phase!=ACTION_OK && phase!=ACTION_REJECTED && phase!=ACTION_AMBIGUOUS)return PROBE_PRODUCT_NO_RESULT;
    atomic_store_explicit(&s->action_phase,ACTION_IDLE,memory_order_seq_cst);
    return phase==ACTION_OK?PROBE_PRODUCT_SELECTED:phase==ACTION_REJECTED?PROBE_PRODUCT_REJECTED:PROBE_PRODUCT_SELECTION_AMBIGUOUS;
}
