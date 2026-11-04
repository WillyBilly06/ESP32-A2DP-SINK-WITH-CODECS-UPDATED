#include "probe_admission.h"
#include "probe_smp.h"
#include "freertos/queue.h"
#include "nvs.h"
#include <string.h>
#include <stdatomic.h>

#define LIMIT 8u
#define WINDOW_MS 120000u
typedef struct { bool used;uint8_t type,id[6],tag[32]; } trust_t;
typedef struct { uint8_t type,id[6],tag[32];bool committed; } job_t;
typedef struct { bool used,bound;uint16_t handle;uint32_t generation,window;uint8_t address[6],type,id[6],tag[32]; } lease_t;
static dbt_runtime_t *runtime;
static TaskHandle_t owner;
static nvs_handle_t storage;
static QueueHandle_t jobs,results;
static trust_t trust[LIMIT];
static lease_t classic[DBT_MAX_LINKS],le;
static uint32_t serial,deadline,prompt_serial;
static bool opened,poisoned,write_pending,storage_open,first_boot_empty,frozen,prompt_exhausted;
static uint8_t window_origin; /* 1 physical gesture, 2 empty-store power-on */
static _Atomic uint32_t published_cookie,published_handle,published_generation;
static uint32_t cookie_serial;
static struct {uint16_t handle;uint32_t generation;uint8_t type,id[6],tag[32];} cookie_binding;
static void wipe(void *p,size_t n){volatile uint8_t *v=p;while(n--)*v++=0;}
static void revoke_cookie(void){atomic_store_explicit(&published_cookie,0,memory_order_release);wipe(&cookie_binding,sizeof(cookie_binding));}
static bool same(const uint8_t *a,const uint8_t *b,size_t n){return memcmp(a,b,n)==0;}
static int find(uint8_t type,const uint8_t id[6]){for(unsigned i=0;i<LIMIT;i++)if(trust[i].used && trust[i].type==type && same(trust[i].id,id,6))return (int)i;return -1;}
static int available(void){for(unsigned i=0;i<LIMIT;i++)if(!trust[i].used)return (int)i;return -1;}
static void name(uint8_t type,const uint8_t id[6],char key[15]){
    static const char hex[]="0123456789abcdef";key[0]='l';key[1]=(char)('0'+type);
    for(unsigned i=0;i<6;i++){key[2+2*i]=hex[id[i]>>4];key[3+2*i]=hex[id[i]&15];}key[14]=0;
}
static void encode(const job_t *j,uint8_t b[40]){b[0]=1;b[1]=j->type;memcpy(b+2,j->id,6);memcpy(b+8,j->tag,32);}
static bool decode(const char *key,const uint8_t b[40],trust_t *out){
    if(b[0]!=1 || b[1]>1)return false;
    char expected[15];name(b[1],b+2,expected);
    if(strcmp(key,expected))return false;
    out->used=true;out->type=b[1];memcpy(out->id,b+2,6);memcpy(out->tag,b+8,32);return true;
}
static bool preload(void){
    if(nvs_open("native_trust",NVS_READWRITE,&storage)!=ESP_OK)return false;
    storage_open=true;
    nvs_iterator_t it=0;esp_err_t e=nvs_entry_find_in_handle(storage,NVS_TYPE_ANY,&it);
    if(e==ESP_ERR_NVS_NOT_FOUND)return true;
    while(e==ESP_OK){
        nvs_entry_info_t info;e=nvs_entry_info(it,&info);if(e!=ESP_OK)break;
        int idx=available();uint8_t b[40];size_t n=sizeof(b);
        if(idx<0 || info.type!=NVS_TYPE_BLOB || !strcmp(info.key,"intent") ||
           nvs_get_blob(storage,info.key,b,&n)!=ESP_OK || n!=sizeof(b) || !decode(info.key,b,&trust[idx])){
            wipe(b,sizeof(b));e=ESP_ERR_INVALID_STATE;break;
        }
        wipe(b,sizeof(b));e=nvs_entry_next(&it);
    }
    nvs_release_iterator(it);return e==ESP_ERR_NVS_NOT_FOUND;
}
/* Presence of any entry, including an intent or unknown record, is never
 * interpreted as an empty bond store. Enumeration errors also fail closed. */
static bool namespace_empty(const char *space,bool *empty){
    nvs_handle_t h=0;esp_err_t e=nvs_open(space,NVS_READONLY,&h);
    if(e==ESP_ERR_NVS_NOT_FOUND){*empty=true;return true;}
    if(e!=ESP_OK)return false;
    nvs_iterator_t it=0;e=nvs_entry_find_in_handle(h,NVS_TYPE_ANY,&it);unsigned count=0;
    while(e==ESP_OK){nvs_entry_info_t info;e=nvs_entry_info(it,&info);if(e!=ESP_OK)break;
        if(++count>32){e=ESP_ERR_INVALID_STATE;break;}e=nvs_entry_next(&it);}
    nvs_release_iterator(it);nvs_close(h);
    if(e!=ESP_ERR_NVS_NOT_FOUND)return false;
    *empty=count==0;return true;
}
static bool commit(const job_t *j){
    uint8_t b[40],check[40];char key[15];size_t n;encode(j,b);name(j->type,j->id,key);
    esp_err_t e=nvs_set_blob(storage,"intent",b,sizeof(b));if(e==ESP_OK)e=nvs_commit(storage);
    n=sizeof(check);if(e==ESP_OK)e=nvs_get_blob(storage,"intent",check,&n);
    if(e!=ESP_OK || n!=sizeof(check) || !same(b,check,sizeof(b)))goto fail;
    e=nvs_set_blob(storage,key,b,sizeof(b));if(e==ESP_OK)e=nvs_commit(storage);
    n=sizeof(check);if(e==ESP_OK)e=nvs_get_blob(storage,key,check,&n);
    if(e!=ESP_OK || n!=sizeof(check) || !same(b,check,sizeof(b)))goto fail;
    e=nvs_erase_key(storage,"intent");if(e==ESP_OK)e=nvs_commit(storage);
    n=0;if(e==ESP_OK)e=nvs_get_blob(storage,"intent",NULL,&n);
    wipe(b,sizeof(b));wipe(check,sizeof(check));return e==ESP_ERR_NVS_NOT_FOUND;
fail:
    wipe(b,sizeof(b));wipe(check,sizeof(check));return false;
}
static void worker(void *unused){
    (void)unused;for(;;){job_t j;if(xQueueReceive(jobs,&j,portMAX_DELAY)!=pdTRUE)continue;
        j.committed=commit(&j);(void)xQueueSend(results,&j,portMAX_DELAY);wipe(&j,sizeof(j));xTaskNotifyGive(owner);}
}
dbt_result_t probe_admission_startup_worker(dbt_runtime_t *r,TaskHandle_t task){
    if(!r || !task || runtime)return DBT_INVALID;
    if(!preload()){if(storage_open)nvs_close(storage);storage_open=false;wipe(trust,sizeof(trust));return DBT_INVALID;}
    bool classic_empty=false,le_empty=false;
    if(!namespace_empty("native_bonds",&classic_empty) || !namespace_empty("native_le_bonds",&le_empty)){
        nvs_close(storage);storage_open=false;wipe(trust,sizeof(trust));return DBT_INVALID;
    }
    first_boot_empty=classic_empty && le_empty;
    for(unsigned i=0;i<LIMIT;i++)if(trust[i].used)first_boot_empty=false;
    jobs=xQueueCreate(1,sizeof(job_t));results=xQueueCreate(1,sizeof(job_t));
    if(!jobs || !results){if(jobs)vQueueDelete(jobs);if(results)vQueueDelete(results);jobs=results=NULL;nvs_close(storage);storage_open=false;wipe(trust,sizeof(trust));return DBT_NO_SPACE;}
    runtime=r;owner=task;
    if(xTaskCreatePinnedToCore(worker,"native_trust",3072,NULL,1,NULL,0)!=pdPASS){
        runtime=NULL;owner=NULL;vQueueDelete(jobs);vQueueDelete(results);jobs=results=NULL;nvs_close(storage);storage_open=false;wipe(trust,sizeof(trust));return DBT_NO_SPACE;}
    return DBT_OK;
}
static bool lease_window(uint32_t now){return runtime && opened && !poisoned && (int32_t)(now-deadline)<0;}
bool probe_admission_window(uint32_t now){return !frozen && lease_window(now);}
uint8_t probe_admission_origin(uint32_t now){return probe_admission_window(now)?window_origin:0;}
uint32_t probe_admission_prompt_epoch(uint32_t now){return !prompt_exhausted && probe_admission_window(now)?prompt_serial:0;}
void probe_admission_freeze(bool value){frozen=value;}
bool probe_admission_frozen(void){return frozen;}
static bool secure_bond(uint16_t handle,uint32_t generation,job_t *job){
    dbt_smp_session_security_t sec;
    return probe_smp_security(handle,generation,&sec)==DBT_OK && sec.encrypted && sec.bond_committed && sec.key_size>=16 &&
        probe_smp_committed_bond(handle,generation,&job->type,job->id,job->tag)==DBT_OK;
}
void probe_admission_open(uint32_t now){
    if(frozen)return;
    if(!probe_admission_window(now)){
        if(!runtime || poisoned || serial==UINT32_MAX){opened=false;return;}
        serial++;opened=true;memset(classic,0,sizeof(classic));memset(&le,0,sizeof(le));
    }
    if(prompt_serial!=UINT32_MAX)++prompt_serial;else prompt_exhausted=true;
    deadline=now+WINDOW_MS;window_origin=1;
    /* A gesture can elevate exactly one already-encrypted committed LE bond;
     * never infer which of multiple candidates the physical gesture meant. */
    if(!le.used){
        const dbt_link_t *chosen=NULL;job_t selected={0};unsigned count=0;
        for(unsigned i=0;i<DBT_MAX_LINKS;i++){
            const dbt_link_t *l=&runtime->acl.links[i];job_t j={0};
            if(l->used && l->kind==DBT_LINK_LE && l->encrypted && secure_bond(l->handle,l->generation,&j)){
                chosen=l;selected=j;count++;
            }
            wipe(&j,sizeof(j));
        }
        if(count==1){le=(lease_t){.used=true,.bound=true,.handle=chosen->handle,.generation=chosen->generation,.window=serial,.type=selected.type};
            memcpy(le.address,chosen->peer,6);memcpy(le.id,selected.id,6);memcpy(le.tag,selected.tag,32);}
        wipe(&selected,sizeof(selected));
    }
}
void probe_admission_first_boot(uint32_t now,unsigned classic_bonds){
    if(!first_boot_empty || classic_bonds || !runtime || poisoned || serial==UINT32_MAX)return;
    if(prompt_serial!=UINT32_MAX)++prompt_serial;else prompt_exhausted=true;
    serial++;deadline=now+WINDOW_MS;opened=true;window_origin=2;first_boot_empty=false;
}
static bool valid_link(const dbt_link_t *l,dbt_link_kind_t kind){
    if(!runtime || !l || !l->used || l->kind!=kind || !l->generation)return false;
    const dbt_link_t *current=dbt_acl_find(&runtime->acl,l->handle);
    return current && current->generation==l->generation && current->kind==kind && same(current->peer,l->peer,6);
}
bool probe_admission_classic_pair(const dbt_link_t *l,uint32_t now){
    if(!lease_window(now) || !valid_link(l,DBT_LINK_CLASSIC))return false;
    for(unsigned i=0;i<DBT_MAX_LINKS;i++)if(classic[i].used && classic[i].handle==l->handle && classic[i].generation==l->generation)return true;
    if(frozen)return false;
    for(unsigned i=0;i<DBT_MAX_LINKS;i++)if(!classic[i].used){classic[i]=(lease_t){.used=true,.handle=l->handle,.generation=l->generation,.window=serial};memcpy(classic[i].address,l->peer,6);return true;}
    return false;
}
bool probe_admission_classic_store(const uint8_t address[6],uint32_t now){
    if(!address || !lease_window(now))return false;
    for(unsigned i=0;i<DBT_MAX_LINKS;i++)if(classic[i].used && classic[i].window==serial && same(classic[i].address,address,6)){
        const dbt_link_t *l=dbt_acl_find(&runtime->acl,classic[i].handle);
        return l && l->generation==classic[i].generation && valid_link(l,DBT_LINK_CLASSIC);
    }
    return false;
}
bool probe_admission_le_begin(const dbt_link_t *l,uint32_t now){
    if(!probe_admission_window(now) || !valid_link(l,DBT_LINK_LE) || le.used)return false;
    if(cookie_binding.handle==l->handle && cookie_binding.generation==l->generation)revoke_cookie();
    le=(lease_t){.used=true,.handle=l->handle,.generation=l->generation,.window=serial};memcpy(le.address,l->peer,6);return true;
}
void probe_admission_link(const dbt_link_change_t *c){
    if(!c || c->kind!=DBT_LINK_REMOVED)return;
    if(cookie_binding.handle==c->handle && cookie_binding.generation==c->generation)revoke_cookie();
    for(unsigned i=0;i<DBT_MAX_LINKS;i++)if(classic[i].used && classic[i].handle==c->handle && classic[i].generation==c->generation)classic[i].used=false;
    if(le.used && le.handle==c->handle && le.generation==c->generation)le.used=false;
}
void probe_admission_revalidate(void){
    if(atomic_load_explicit(&published_cookie,memory_order_acquire)){
        uint8_t type,id[6],tag[32];
        if(probe_admission_trusted_le(cookie_binding.handle,cookie_binding.generation,&type,id,tag)!=DBT_OK ||
           type!=cookie_binding.type || !same(id,cookie_binding.id,6) || !same(tag,cookie_binding.tag,32))revoke_cookie();
        wipe(tag,sizeof(tag));
    }
}
void probe_admission_poll(uint32_t now){
    if(!runtime)return;
    job_t done;
    if(xQueueReceive(results,&done,0)==pdTRUE){
        write_pending=false;
        if(!done.committed)poisoned=true;
        else {int i=find(done.type,done.id);if(i<0)i=available();
            if(i<0)poisoned=true;else {trust[i].used=true;trust[i].type=done.type;memcpy(trust[i].id,done.id,6);memcpy(trust[i].tag,done.tag,32);}}
        wipe(&done,sizeof(done));
    }
    probe_admission_revalidate();
    if(opened && !lease_window(now)){opened=false;memset(classic,0,sizeof(classic));le.used=false;}
    if(!le.used || write_pending || !lease_window(now) || le.window!=serial)return;
    const dbt_link_t *l=dbt_acl_find(&runtime->acl,le.handle);
    if(!l || l->generation!=le.generation || !valid_link(l,DBT_LINK_LE)){le.used=false;return;}
    if(!l->encrypted)return;
    job_t job={0};
    if(!secure_bond(le.handle,le.generation,&job))return;
    if(le.bound && (le.type!=job.type || !same(le.id,job.id,6) || !same(le.tag,job.tag,32))){le.used=false;wipe(&job,sizeof(job));return;}
    if(!le.bound){le.bound=true;le.type=job.type;memcpy(le.id,job.id,6);memcpy(le.tag,job.tag,32);}
    int existing=find(job.type,job.id);
    if(existing>=0 && same(trust[existing].tag,job.tag,32)){le.used=false;wipe(&job,sizeof(job));return;}
    if(existing<0 && available()<0){le.used=false;wipe(&job,sizeof(job));return;}
    if(xQueueSend(jobs,&job,0)==pdTRUE){write_pending=true;le.used=false;}
    wipe(&job,sizeof(job));
}
uint32_t probe_admission_wait_ms(uint32_t now){
    if(!runtime)return UINT32_MAX;
    if(uxQueueMessagesWaiting(results))return 0;
    if(le.used || write_pending)return 20;
    if(opened)return (int32_t)(now-deadline)>=0?0:deadline-now;
    return UINT32_MAX;
}
dbt_result_t probe_admission_trusted_le(uint16_t handle,uint32_t generation,uint8_t *type,uint8_t id[6],uint8_t tag[32]){
    if(!runtime || poisoned || !type || !id || !tag || !generation)return DBT_INVALID;
    const dbt_link_t *l=dbt_acl_find(&runtime->acl,handle);dbt_smp_session_security_t sec;
    if(!l || l->generation!=generation || !l->encrypted || !valid_link(l,DBT_LINK_LE) ||
       probe_smp_security(handle,generation,&sec)!=DBT_OK || !sec.encrypted || !sec.bond_committed || sec.key_size<16)return DBT_INVALID;
    uint8_t t,i[6],b[32];dbt_result_t r=probe_smp_committed_bond(handle,generation,&t,i,b);
    if(r!=DBT_OK)return r;
    int n=find(t,i);
    if(n<0 || !same(trust[n].tag,b,32)){wipe(b,sizeof(b));return DBT_INVALID;}
    *type=t;memcpy(id,i,6);memcpy(tag,b,32);wipe(b,sizeof(b));return DBT_OK;
}
uint32_t probe_admission_control_cookie(uint16_t handle,uint32_t generation){
    uint8_t type,id[6],tag[32];
    if(probe_admission_trusted_le(handle,generation,&type,id,tag)!=DBT_OK){revoke_cookie();return 0;}
    uint32_t current=atomic_load_explicit(&published_cookie,memory_order_acquire);
    if(current && cookie_binding.handle==handle && cookie_binding.generation==generation && cookie_binding.type==type &&
       same(cookie_binding.id,id,6) && same(cookie_binding.tag,tag,32)){wipe(tag,sizeof(tag));return current;}
    revoke_cookie();
    if(cookie_serial==UINT32_MAX){wipe(tag,sizeof(tag));return 0;}
    uint32_t next=++cookie_serial;
    cookie_binding.handle=handle;cookie_binding.generation=generation;cookie_binding.type=type;
    memcpy(cookie_binding.id,id,6);memcpy(cookie_binding.tag,tag,32);
    atomic_store_explicit(&published_handle,handle,memory_order_relaxed);
    atomic_store_explicit(&published_generation,generation,memory_order_relaxed);
    atomic_store_explicit(&published_cookie,next,memory_order_release);wipe(tag,sizeof(tag));return next;
}
bool probe_admission_control_cookie_valid(uint16_t handle,uint32_t generation,uint32_t cookie){
    if(!cookie || !generation)return false;
    uint32_t first=atomic_load_explicit(&published_cookie,memory_order_acquire);
    if(first!=cookie)return false;
    uint32_t h=atomic_load_explicit(&published_handle,memory_order_relaxed);
    uint32_t g=atomic_load_explicit(&published_generation,memory_order_relaxed);
    return h==handle && g==generation && atomic_load_explicit(&published_cookie,memory_order_acquire)==cookie;
}
bool probe_admission_quiescent(void){
    return runtime && !poisoned && !le.used && !write_pending && jobs && results &&
        !uxQueueMessagesWaiting(jobs) && !uxQueueMessagesWaiting(results);
}
