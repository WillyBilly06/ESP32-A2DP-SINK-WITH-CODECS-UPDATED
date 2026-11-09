#include "probe_cccd_store.h"
#include "dbt_sha256.h"
#include <nvs.h>
#include <string.h>

static bool poisoned;
/* One serialized worker owns this 48-byte verified durable snapshot. A load
 * always revalidates NVS; saves for the same identity/incarnation can avoid
 * re-reading the blob they just loaded or committed. Other peers fall back
 * to the original checked read. No key material is retained here. */
static struct {
    uint8_t type,identity[6],tag[32];
    bool valid;
    uint16_t values[4];
} cccd_cache;
_Static_assert(sizeof(cccd_cache)==48,"CCCD cache budget");
static bool cache_matches(uint8_t t,const uint8_t id[6],const uint8_t tag[32]) {
    return cccd_cache.valid && cccd_cache.type==t &&
        !memcmp(cccd_cache.identity,id,6) && !memcmp(cccd_cache.tag,tag,32);
}
static void cache_verified(uint8_t t,const uint8_t id[6],const uint8_t tag[32],const uint16_t v[4]) {
    cccd_cache.type=t;memcpy(cccd_cache.identity,id,6);memcpy(cccd_cache.tag,tag,32);
    memcpy(cccd_cache.values,v,sizeof(cccd_cache.values));cccd_cache.valid=true;
}
static void cccd_wipe(void *p,size_t n) { volatile uint8_t *v=p;while(n--)*v++=0; }
static bool valid_identity(uint8_t t,const uint8_t id[6]) {
    return id && (t==0 || (t==1 && (id[5]&0xc0u)==0xc0u));
}
bool probe_cccd_bond_tag(const dbt_smp_bond_record_t *r,uint8_t out[32]) {
    if(!r || !out || !valid_identity(r->identity_type,r->identity) || r->key_size<7 || r->key_size>16)
        return false;
    uint8_t bytes[75]={ 'D','B','T','C','C','C','D',1 }; unsigned p=8;
    bytes[p++]=r->identity_type;memcpy(bytes+p,r->identity,6);p+=6;
    bytes[p++]=r->key_size;bytes[p++]=(r->secure_connections?1:0)|(r->have_peer_ltk?2:0)|(r->have_local_ltk?4:0);
    memcpy(bytes+p,r->ltk,16);p+=16;memcpy(bytes+p,r->local_ltk,16);p+=16;
    memcpy(bytes+p,r->rand,8);p+=8;memcpy(bytes+p,r->local_rand,8);p+=8;
    bytes[p++]=(uint8_t)r->ediv;bytes[p++]=(uint8_t)(r->ediv>>8);
    bytes[p++]=(uint8_t)r->local_ediv;bytes[p++]=(uint8_t)(r->local_ediv>>8);
    bool ok=dbt_sha256(bytes,p,out,0)==0;cccd_wipe(bytes,sizeof(bytes));return ok;
}
static void key_name(uint8_t type,const uint8_t id[6],char key[14]) {
    static const char h[]="0123456789abcdef";
    key[0]=(char)('0'+type);
    for(unsigned i=0;i<6;i++){key[1+2*i]=h[id[i]>>4];key[2+2*i]=h[id[i]&15];}key[13]=0;
}
/* Exact identity key avoids accumulating one blob for every re-pair. The full
 * incarnation digest in the blob prevents a new bond inheriting old values.
 * GATT uses one FIFO worker, so an old accepted save precedes a new load. */
#define RECORD_SIZE 51u
static bool encode(uint8_t t,const uint8_t id[6],const uint8_t tag[32],const uint16_t v[4],uint8_t b[RECORD_SIZE]) {
    if(!valid_identity(t,id) || !tag || !v || (v[0]&~2u) || (v[1]&~1u) || (v[2]&~1u) || (v[3]&~1u))return false;
    memcpy(b,"CC1\0",4);b[4]=t;memcpy(b+5,id,6);memcpy(b+11,tag,32);
    for(unsigned i=0;i<4;i++){b[43+2*i]=(uint8_t)v[i];b[44+2*i]=(uint8_t)(v[i]>>8);}return true;
}
static dbt_result_t read_blob(nvs_handle_t h,uint8_t t,const uint8_t id[6],const uint8_t tag[32],uint16_t v[4]) {
    char key[14];uint8_t b[RECORD_SIZE];size_t n=sizeof(b);key_name(t,id,key);
    esp_err_t rc=nvs_get_blob(h,key,b,&n);
    if(rc==ESP_ERR_NVS_NOT_FOUND){memset(v,0,8);return DBT_EMPTY;}
    if(rc!=ESP_OK || n!=sizeof(b) || memcmp(b,"CC1\0",4) || b[4]!=t || memcmp(b+5,id,6))return DBT_INVALID;
    for(unsigned i=0;i<4;i++)v[i]=(uint16_t)(b[43+2*i]|((uint16_t)b[44+2*i]<<8));
    if((v[0]&~2u) || (v[1]&~1u) || (v[2]&~1u) || (v[3]&~1u))return DBT_INVALID;
    if(memcmp(b+11,tag,32)){memset(v,0,8);return DBT_EMPTY;}
    return DBT_OK;
}
dbt_result_t probe_cccd_load(uint8_t t,const uint8_t id[6],const uint8_t tag[32],uint16_t v[4]) {
    if(poisoned || !valid_identity(t,id) || !tag || !v)return DBT_INVALID;
    cccd_cache.valid=false;
    nvs_handle_t h;if(nvs_open("native_cccd",NVS_READWRITE,&h)!=ESP_OK)return DBT_INVALID;
    dbt_result_t rc=read_blob(h,t,id,tag,v);nvs_close(h);
    if(rc==DBT_OK)cache_verified(t,id,tag,v);
    return rc;
}
dbt_result_t probe_cccd_save(uint8_t t,const uint8_t id[6],const uint8_t tag[32],const uint16_t v[4]) {
    uint8_t b[RECORD_SIZE];if(poisoned || !encode(t,id,tag,v,b))return DBT_INVALID;
    bool verified=cache_matches(t,id,tag);
    if(verified && !memcmp(cccd_cache.values,v,sizeof(cccd_cache.values))){cccd_wipe(b,sizeof(b));return DBT_OK;}
    nvs_handle_t h;if(nvs_open("native_cccd",NVS_READWRITE,&h)!=ESP_OK){cccd_cache.valid=false;cccd_wipe(b,sizeof(b));return DBT_INVALID;}
    uint16_t previous[4];dbt_result_t rc=verified?DBT_OK:read_blob(h,t,id,tag,previous);
    if(rc==DBT_OK || rc==DBT_EMPTY) {
        char key[14];key_name(t,id,key);
        if(nvs_set_blob(h,key,b,sizeof(b))!=ESP_OK || nvs_commit(h)!=ESP_OK) {poisoned=true;cccd_cache.valid=false;rc=DBT_INVALID;}
        else {cache_verified(t,id,tag,v);rc=DBT_OK;}
    }
    nvs_close(h);cccd_wipe(b,sizeof(b));return rc;
}
