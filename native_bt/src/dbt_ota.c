#include "dbt_ota.h"
#include <string.h>

static bool same_peer(const dbt_ble_control_peer_t *a,const dbt_ble_control_peer_t *b) {
    return a->handle==b->handle && a->generation==b->generation;
}
static bool same_manifest(const dbt_ota_manifest_t *a,const dbt_ota_manifest_t *b) {
    return a->length==b->length && a->trusted_digest==b->trusted_digest &&
           (!a->trusted_digest || (a->crc32==b->crc32 && !memcmp(a->sha256,b->sha256,32))) &&
           !memcmp(a->project,b->project,32);
}
static bool valid_grant(dbt_ota_t *s,const dbt_ble_control_peer_t *p) {
    dbt_ota_manifest_t candidate={0};
    return s->hooks.authorize(s->hooks.context,p,s->manifest.length,&candidate) && same_manifest(&candidate,&s->manifest);
}
static uint32_t crc_update(uint32_t crc,const uint8_t *data,uint16_t n) {
    while(n--) {crc^=*data++;for(unsigned i=0;i<8;i++)crc=(crc>>1)^((0u-(crc&1u))&0xedb88320u);}
    return crc;
}
static void cleanup(dbt_ota_t *s) {
    if((s->state==DBT_OTA_RECEIVING || s->state==DBT_OTA_VERIFIED) && !s->hooks.abort(s->hooks.context))s->state=DBT_OTA_FAULT;
    else s->state=DBT_OTA_IDLE;
    memset(&s->manifest,0,sizeof(s->manifest));memset(&s->peer,0,sizeof(s->peer));s->offset=0;s->crc=0;s->sequence=0;s->sequence_started=false;
}
bool dbt_ota_init(dbt_ota_t *s,const dbt_ota_hooks_t *h,uint32_t timeout_ms) {
    if(!s || !h || !h->authorize || !h->now_ms || !h->begin || !h->write || !h->finish || !h->abort || !h->activate ||
       !timeout_ms || timeout_ms>0x7fffffffu)return false;
    memset(s,0,sizeof(*s));s->hooks=*h;s->timeout_ms=timeout_ms;atomic_init(&s->cancel,false);return true;
}
void dbt_ota_request_cancel(dbt_ota_t *s) {if(s)atomic_store_explicit(&s->cancel,true,memory_order_release);}
void dbt_ota_tick(dbt_ota_t *s) {
    if(!s)return;
    bool cancel=atomic_exchange_explicit(&s->cancel,false,memory_order_acq_rel);
    if((s->state==DBT_OTA_RECEIVING || s->state==DBT_OTA_VERIFIED) &&
       (cancel || (uint32_t)(s->hooks.now_ms(s->hooks.context)-s->last_ms)>=s->timeout_ms || !valid_grant(s,&s->peer)))cleanup(s);
}
bool dbt_ota_command(dbt_ota_t *s,const dbt_ble_control_peer_t *p,uint8_t cmd,const uint8_t *data,uint16_t n,uint8_t *out,uint8_t *size) {
    if(!s || !p || !p->generation || (!data && n) || !out || !size || *size<2 || cmd<0x20 || cmd>0x23)return false;
    dbt_ota_tick(s);out[0]=0x23;out[1]=0x12;*size=2;
    if(cmd==0x20) {
        if(n!=4 || s->state!=DBT_OTA_IDLE)return true;
        uint32_t length=(uint32_t)data[0]|((uint32_t)data[1]<<8)|((uint32_t)data[2]<<16)|((uint32_t)data[3]<<24);
        dbt_ota_manifest_t m={0};out[1]=0x10;
        if(!length || !s->hooks.authorize(s->hooks.context,p,length,&m) || m.length!=length || !m.project[0] ||
           !memchr(m.project,0,sizeof(m.project)) || !s->hooks.begin(s->hooks.context,&m))return true;
        s->manifest=m;s->peer=*p;s->offset=0;s->sequence=0;s->sequence_started=false;s->crc=0xffffffffu;
        s->last_ms=s->hooks.now_ms(s->hooks.context);s->state=DBT_OTA_RECEIVING;out[0]=0x21;*size=1;return true;
    }
    if(s->state!=DBT_OTA_RECEIVING || !same_peer(&s->peer,p) || !valid_grant(s,p))return true;
    if(cmd==0x23) {
        if(n)return true;
        cleanup(s);
        if(s->state==DBT_OTA_FAULT)return true;
        out[0]=0x10;out[1]=0x23;*size=2;return true;
    }
    if(cmd==0x21) {
        out[1]=0x11;
        if(n<2 || (s->sequence_started && data[0]!=s->sequence) || (uint32_t)(n-1)>s->manifest.length-s->offset) {cleanup(s);return true;}
        if(!s->hooks.write(s->hooks.context,s->offset,data+1,(uint16_t)(n-1))) {cleanup(s);return true;}
        s->crc=crc_update(s->crc,data+1,(uint16_t)(n-1));s->offset+=(uint32_t)n-1u;s->sequence=(uint8_t)(data[0]+1u);s->sequence_started=true;
        s->last_ms=s->hooks.now_ms(s->hooks.context);out[0]=0x20;out[1]=(uint8_t)(((uint64_t)s->offset*100u)/s->manifest.length);return true;
    }
    if(n || s->offset!=s->manifest.length || (s->manifest.trusted_digest && (s->crc^0xffffffffu)!=s->manifest.crc32)) {cleanup(s);return true;}
    /* finish owns and closes the backend handle even on failure. */
    bool ok=s->hooks.finish(s->hooks.context,&s->manifest);
    if(!ok) {s->state=DBT_OTA_FAULT;return true;}
    s->state=DBT_OTA_VERIFIED;s->last_ms=s->hooks.now_ms(s->hooks.context);out[0]=0x22;*size=1;return true;
}
bool dbt_ota_activate(dbt_ota_t *s,const dbt_ble_control_peer_t *p) {
    if(!s || !p)return false;
    dbt_ota_tick(s);
    if(s->state!=DBT_OTA_VERIFIED || !same_peer(&s->peer,p) || !valid_grant(s,p))return false;
    if(!s->hooks.activate(s->hooks.context,&s->manifest)) {
        (void)s->hooks.abort(s->hooks.context);s->state=DBT_OTA_FAULT;return false;
    }
    cleanup(s);return s->state==DBT_OTA_IDLE;
}
bool dbt_ota_control_command(void *context,const dbt_ble_control_peer_t *peer,uint8_t command,
                             const uint8_t *data,uint16_t size,uint8_t *out,uint8_t *out_size) {
    return dbt_ota_command(context,peer,command,data,size,out,out_size);
}
