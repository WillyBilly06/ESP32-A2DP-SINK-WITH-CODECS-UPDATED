#define _CRT_SECURE_NO_WARNINGS
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#include <pthread.h>
#endif
#undef assert
#define assert(x) do { if(!(x)){fprintf(stderr,"failed line %d: %s\n",__LINE__,#x);exit(1);} } while(0)
#include "probe_smp.h"
#include "freertos/queue.h"
#include "nvs.h"
#define ESP_ERR_INVALID_STATE 0x103
#define NVS_READONLY 0
BaseType_t xTaskCreatePinnedToCore(void (*)(void *),const char *,uint32_t,void *,UBaseType_t,TaskHandle_t *,int);
#include "../../esp32_probe/main/probe_admission.c"

typedef struct {size_t size;bool full;uint8_t bytes[128];} queue_t;
typedef struct {bool used;char key[16];uint8_t bytes[40];size_t size;} entry_t;
static entry_t entries[12];static int queue_fail,send_fail,commit_fail,task_fail,iterator,enumeration_fail,classic_entries,le_entries,iterator_handle;
static uint8_t identity[6]={1,2,3,4,5,6},tag_now[32];static bool smp_ready;
static dbt_runtime_t rt;
static _Atomic bool race_done;static _Atomic unsigned race_reads;
#ifdef _WIN32
static DWORD WINAPI cookie_reader(void *unused)
#else
static void *cookie_reader(void *unused)
#endif
{
    (void)unused;
    while(!atomic_load_explicit(&race_done,memory_order_acquire)){
        uint32_t c=atomic_load_explicit(&published_cookie,memory_order_acquire);
        if(c){uint16_t expected=(uint16_t)(1u+(c&1u));
            assert(!probe_admission_control_cookie_valid((uint16_t)(3u-expected),expected,c));
            if(probe_admission_control_cookie_valid(expected,expected,c))atomic_fetch_add(&race_reads,1);}
    }
#ifdef _WIN32
    return 0;
#else
    return NULL;
#endif
}
static void cookie_race(void){
    atomic_store(&race_done,false);atomic_store(&race_reads,0);
#ifdef _WIN32
    HANDLE reader=CreateThread(NULL,0,cookie_reader,NULL,0,NULL);assert(reader);
#else
    pthread_t reader;assert(!pthread_create(&reader,NULL,cookie_reader,NULL));
#endif
    for(uint32_t c=1;c<=30000;c++){
        atomic_store_explicit(&published_cookie,0,memory_order_release);
        uint32_t h=1u+(c&1u);atomic_store_explicit(&published_handle,h,memory_order_relaxed);
        atomic_store_explicit(&published_generation,h,memory_order_relaxed);
        atomic_store_explicit(&published_cookie,c,memory_order_release);
        if(c==1){unsigned spins=0;while(!atomic_load(&race_reads) && spins++<10000000u){}
            assert(atomic_load(&race_reads)>0);}
    }
    atomic_store_explicit(&race_done,true,memory_order_release);
#ifdef _WIN32
    assert(WaitForSingleObject(reader,INFINITE)==WAIT_OBJECT_0);CloseHandle(reader);
#else
    assert(!pthread_join(reader,NULL));
#endif
    assert(atomic_load(&race_reads)>0);
}
static entry_t *entry(const char *key,bool create){
    for(unsigned i=0;i<12;i++)if(entries[i].used && !strcmp(entries[i].key,key))return &entries[i];
    if(create)for(unsigned i=0;i<12;i++)if(!entries[i].used){entries[i].used=true;strcpy(entries[i].key,key);return &entries[i];}return NULL;
}
esp_err_t nvs_open(const char *ns,int mode,nvs_handle_t *h){(void)mode;if(!strcmp(ns,"native_trust"))*h=1;else if(!strcmp(ns,"native_bonds"))*h=2;else if(!strcmp(ns,"native_le_bonds"))*h=3;else return ESP_ERR_INVALID_STATE;return ESP_OK;}
void nvs_close(nvs_handle_t h){assert(h>=1 && h<=3);}
esp_err_t nvs_get_blob(nvs_handle_t h,const char *key,void *data,size_t *n){
    (void)h;entry_t *e=entry(key,false);if(!e)return ESP_ERR_NVS_NOT_FOUND;
    if(data){if(*n<e->size)return ESP_ERR_INVALID_STATE;memcpy(data,e->bytes,e->size);}*n=e->size;return ESP_OK;
}
esp_err_t nvs_set_blob(nvs_handle_t h,const char *key,const void *data,size_t n){(void)h;entry_t *e=entry(key,true);if(!e || n>40)return ESP_ERR_INVALID_STATE;memcpy(e->bytes,data,n);e->size=n;return ESP_OK;}
esp_err_t nvs_erase_key(nvs_handle_t h,const char *key){(void)h;entry_t *e=entry(key,false);if(!e)return ESP_ERR_NVS_NOT_FOUND;e->used=false;return ESP_OK;}
esp_err_t nvs_commit(nvs_handle_t h){(void)h;if(commit_fail){commit_fail--;return ESP_ERR_INVALID_STATE;}return ESP_OK;}
esp_err_t nvs_entry_find_in_handle(nvs_handle_t h,nvs_type_t t,nvs_iterator_t *it){(void)t;if(enumeration_fail && h!=1)return ESP_ERR_INVALID_STATE;iterator=-1;iterator_handle=(int)h;return nvs_entry_next(it);}
esp_err_t nvs_entry_next(nvs_iterator_t *it){
    if(iterator_handle!=1){iterator++;if(iterator<(iterator_handle==2?classic_entries:le_entries)){*it=(nvs_iterator_t)1;return ESP_OK;}*it=NULL;return ESP_ERR_NVS_NOT_FOUND;}
    for(iterator++;iterator<12;iterator++)if(entries[iterator].used){*it=(nvs_iterator_t)1;return ESP_OK;}*it=NULL;return ESP_ERR_NVS_NOT_FOUND;
}
esp_err_t nvs_entry_info(nvs_iterator_t it,nvs_entry_info_t *info){(void)it;memset(info,0,sizeof(*info));strcpy(info->key,iterator_handle==1?entries[iterator].key:"bond");info->type=NVS_TYPE_BLOB;return ESP_OK;}
void nvs_release_iterator(nvs_iterator_t it){(void)it;}
QueueHandle_t xQueueCreate(UBaseType_t count,UBaseType_t size){assert(count==1 && size<=128);if(queue_fail){queue_fail--;return NULL;}queue_t *q=calloc(1,sizeof(*q));q->size=size;return q;}
void vQueueDelete(QueueHandle_t q){free(q);}
BaseType_t xQueueSend(QueueHandle_t q,const void *v,TickType_t wait){(void)wait;queue_t *p=q;if(send_fail){send_fail--;return 0;}if(p->full)return 0;memcpy(p->bytes,v,p->size);p->full=true;return pdTRUE;}
BaseType_t xQueueReceive(QueueHandle_t q,void *v,TickType_t wait){(void)wait;queue_t *p=q;if(!p->full)return 0;memcpy(v,p->bytes,p->size);p->full=false;return pdTRUE;}
UBaseType_t uxQueueMessagesWaiting(QueueHandle_t q){return ((queue_t *)q)->full?1:0;}
BaseType_t xTaskCreatePinnedToCore(void (*f)(void *),const char *n,uint32_t bytes,void *a,UBaseType_t prio,TaskHandle_t *t,int core){(void)f;(void)n;(void)bytes;(void)a;(void)prio;(void)t;(void)core;return task_fail?0:pdPASS;}
void xTaskNotifyGive(TaskHandle_t task){assert(task);}
const dbt_link_t *dbt_acl_find(const dbt_acl_t *acl,uint16_t h){for(unsigned i=0;i<DBT_MAX_LINKS;i++)if(acl->links[i].used && acl->links[i].handle==h)return &acl->links[i];return NULL;}
dbt_result_t probe_smp_security(uint16_t h,uint32_t g,dbt_smp_session_security_t *s){(void)h;(void)g;if(!smp_ready)return DBT_BUSY;memset(s,0,sizeof(*s));s->encrypted=s->bond_committed=true;s->key_size=16;return DBT_OK;}
dbt_result_t probe_smp_committed_bond(uint16_t h,uint32_t g,uint8_t *t,uint8_t id[6],uint8_t tag[32]){(void)h;(void)g;if(!smp_ready)return DBT_BUSY;*t=0;memcpy(id,identity,6);memcpy(tag,tag_now,32);return DBT_OK;}
static void reset(bool preserve){
    if(jobs)vQueueDelete(jobs);if(results)vQueueDelete(results);jobs=results=NULL;
    runtime=NULL;owner=NULL;storage=0;storage_open=false;opened=poisoned=write_pending=first_boot_empty=frozen=prompt_exhausted=false;serial=deadline=prompt_serial=window_origin=0;
    atomic_store(&published_cookie,0);atomic_store(&published_handle,0);atomic_store(&published_generation,0);cookie_serial=0;memset(&cookie_binding,0,sizeof(cookie_binding));
    memset(trust,0,sizeof(trust));memset(classic,0,sizeof(classic));memset(&le,0,sizeof(le));memset(&rt,0,sizeof(rt));
    if(!preserve)memset(entries,0,sizeof(entries));queue_fail=send_fail=commit_fail=task_fail=enumeration_fail=classic_entries=le_entries=0;smp_ready=false;memset(tag_now,7,sizeof(tag_now));
}
static dbt_link_t *link(unsigned i,dbt_link_kind_t kind,uint16_t h,uint32_t gen){dbt_link_t *l=&rt.acl.links[i];*l=(dbt_link_t){.used=true,.kind=kind,.handle=h,.generation=gen};memcpy(l->peer,identity,6);return l;}
static void complete_job(bool fail,uint32_t now){job_t j;assert(xQueueReceive(jobs,&j,0));if(fail)commit_fail=1;j.committed=commit(&j);assert(xQueueSend(results,&j,0));probe_admission_poll(now);}
int main(void){
    reset(false);assert(probe_admission_startup_worker(&rt,(void *)1)==DBT_OK);
    assert(probe_admission_quiescent());
    probe_admission_first_boot(0,0);assert(probe_admission_window(0) && probe_admission_origin(0)==2 && probe_admission_prompt_epoch(0)==1);
    probe_admission_poll(120000);assert(!probe_admission_window(120000));
    dbt_link_t *c=link(0,DBT_LINK_CLASSIC,4,10),*l=link(1,DBT_LINK_LE,5,11);
    assert(!probe_admission_classic_pair(c,0) && !probe_admission_le_begin(l,0));
    probe_admission_open(120100);assert(probe_admission_origin(120100)==1 && probe_admission_prompt_epoch(120100)==2 && probe_admission_classic_pair(c,120101) && probe_admission_classic_store(c->peer,120101));
    assert(probe_admission_le_begin(l,120102));probe_admission_poll(120103);assert(le.used && !probe_admission_quiescent()); /* encryption still pending */
    probe_admission_freeze(true);assert(probe_admission_frozen() && !probe_admission_window(120103));
    assert(probe_admission_classic_pair(c,120103) && !probe_admission_le_begin(l,120103));
    l->encrypted=true;smp_ready=true;send_fail=1;probe_admission_poll(120104);assert(le.used && !write_pending);
    probe_admission_freeze(false);
    probe_admission_open(120105);assert(probe_admission_prompt_epoch(120105)==3);probe_admission_poll(120106);assert(!le.used && write_pending); /* repeated gesture preserves lease */
    uint8_t t,id[6],tag[32];assert(probe_admission_trusted_le(5,11,&t,id,tag)!=DBT_OK);
    complete_job(false,120107);assert(probe_admission_trusted_le(5,11,&t,id,tag)==DBT_OK && !memcmp(tag,tag_now,32) && probe_admission_quiescent());
    uint32_t first=probe_admission_control_cookie(5,11);assert(first && probe_admission_control_cookie_valid(5,11,first));
    assert(!probe_admission_control_cookie_valid(5,12,first));
    memset(tag_now,8,32);probe_admission_poll(120108);assert(!probe_admission_control_cookie_valid(5,11,first));
    memset(tag_now,7,32);uint32_t renewed=probe_admission_control_cookie(5,11);
    assert(renewed && renewed!=first && !probe_admission_control_cookie_valid(5,11,first));
    dbt_link_change_t removed={DBT_LINK_REMOVED,5,11};probe_admission_link(&removed);
    assert(!probe_admission_control_cookie_valid(5,11,renewed));l->generation=12;assert(probe_admission_trusted_le(5,11,&t,id,tag)!=DBT_OK);
    assert(probe_admission_trusted_le(5,12,&t,id,tag)==DBT_OK); /* committed known bond reconnect */
    uint32_t second=probe_admission_control_cookie(5,12);assert(second && second!=first && probe_admission_control_cookie_valid(5,12,second));
    memset(tag_now,8,32);probe_admission_poll(120109);assert(!probe_admission_control_cookie_valid(5,12,second));
    assert(probe_admission_trusted_le(5,12,&t,id,tag)!=DBT_OK); /* replaced bond is untrusted */
    l->encrypted=false;assert(probe_admission_le_begin(l,120110));l->encrypted=true;probe_admission_poll(120111);assert(write_pending);complete_job(false,120112);
    assert(probe_admission_trusted_le(5,12,&t,id,tag)==DBT_OK && tag[0]==8);
    uint32_t third=probe_admission_control_cookie(5,12);assert(third && third!=second && !probe_admission_control_cookie_valid(5,12,second));
    probe_admission_poll(240106);assert(!probe_admission_window(240106));assert(probe_admission_trusted_le(5,12,&t,id,tag)==DBT_OK);
    assert(!probe_admission_classic_pair(c,240107) && !probe_admission_le_begin(l,240107));
    /* Backpressure through expiry must not publish trust or tear down a link. */
    memset(tag_now,10,32);probe_admission_open(241000);assert(le.used && le.bound);send_fail=1;
    probe_admission_poll(241001);assert(le.used && !write_pending);
    probe_admission_poll(361000);assert(!le.used && l->used && l->encrypted && !write_pending);
    assert(probe_admission_trusted_le(5,12,&t,id,tag)!=DBT_OK);
    /* A committed trust record survives restart, but does not create a
     * first-boot window or trust a replacement bond incarnation. */
    reset(true);le_entries=1;assert(probe_admission_startup_worker(&rt,(void *)1)==DBT_OK);
    probe_admission_first_boot(1,0);assert(!probe_admission_window(1));
    l=link(1,DBT_LINK_LE,5,13);l->encrypted=true;smp_ready=true;memset(tag_now,8,32);
    assert(probe_admission_trusted_le(5,13,&t,id,tag)==DBT_OK);
    /* Queued storage error leaves the intent and cannot authorize after reboot. */
    memset(tag_now,9,32);probe_admission_open(370000);assert(le.used && le.bound);probe_admission_poll(370002);assert(write_pending);
    complete_job(true,370003);assert(poisoned && probe_admission_trusted_le(5,13,&t,id,tag)!=DBT_OK);
    reset(true);assert(probe_admission_startup_worker(&rt,(void *)1)==DBT_INVALID);
    reset(false);enumeration_fail=1;assert(probe_admission_startup_worker(&rt,(void *)1)==DBT_INVALID && !runtime);
    reset(false);classic_entries=1;assert(probe_admission_startup_worker(&rt,(void *)1)==DBT_OK);probe_admission_first_boot(1,1);assert(!probe_admission_window(1));
    reset(false);le_entries=1;assert(probe_admission_startup_worker(&rt,(void *)1)==DBT_OK);probe_admission_first_boot(1,0);assert(!probe_admission_window(1));
    reset(false);queue_fail=1;assert(probe_admission_startup_worker(&rt,(void *)1)==DBT_NO_SPACE && !runtime);
    reset(false);task_fail=1;assert(probe_admission_startup_worker(&rt,(void *)1)==DBT_NO_SPACE && !runtime);
    reset(false);assert(probe_admission_startup_worker(&rt,(void *)1)==DBT_OK);
    l=link(0,DBT_LINK_LE,5,20);l->encrypted=true;smp_ready=true;trust[0].used=true;
    memcpy(trust[0].id,identity,6);memcpy(trust[0].tag,tag_now,32);cookie_serial=UINT32_MAX;
    assert(!probe_admission_control_cookie(5,20) && !probe_admission_control_cookie_valid(5,20,UINT32_MAX));
    reset(false);cookie_race();return 0;
}
