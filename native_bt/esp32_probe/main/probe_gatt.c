#include "probe_heap_alloc_trace.h"
#include "dbt_esp32_affinity.h"
#include "probe_gatt.h"
#include "probe_cccd_store.h"
#include "dbt_att_bearer.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "dbt_utf8.h"
#include "freertos/task.h"
#include <string.h>
#include <stdatomic.h>
#if defined(DBT_PROBE_AUDIO_TEST) && (defined(DBT_PROBE_STAY_NATIVE) || defined(DBT_PROBE_SCHEDULER_TRACE))
#include "probe_audio.h"
#endif

/* Diagnostic echo/counter and baseline product-control services. */
static dbt_att_server_t server;
static dbt_att_bearer_t bearer;
static dbt_gap_t *gap;
static dbt_result_t (*security_query)(uint16_t,uint32_t,dbt_smp_session_security_t *);
static dbt_result_t (*bond_query)(uint16_t,uint32_t,uint8_t *,uint8_t[6],uint8_t[32]);
static probe_gatt_ota_hooks_t ota_hooks;
static uint32_t (*control_trust_capture)(uint16_t,uint32_t);
static bool (*control_trust_valid)(uint16_t,uint32_t,uint32_t);
typedef enum { CCCD_NONE,CCCD_LOADING,CCCD_MERGE,CCCD_SAVING,CCCD_READY,CCCD_BOUND,CCCD_ERROR } cccd_phase_t;
typedef struct {
    uint16_t handle;uint32_t generation;uint8_t type,identity[6],tag[32];
    uint16_t values[PROBE_CCCD_COUNT];cccd_phase_t phase;uint32_t retry_at,pending_token;bool retry_pending;
} cccd_binding_t;
static cccd_binding_t cccd_bindings[DBT_MAX_LINKS];

/* Owner-only lifecycle observation. Reuse the existing bounded host ring;
 * never format, allocate, wait or record bond keys on the ATT path. */
static void cccd_record(uint16_t handle,uint32_t generation,uint32_t token,uint32_t stage,
                        uint32_t phase,uint32_t result,uint32_t kind) {
#if defined(DBT_PROBE_AUDIO_TEST) && (defined(DBT_PROBE_STAY_NATIVE) || defined(DBT_PROBE_SCHEDULER_TRACE))
    const probe_host_record_t r={.kind=PROBE_HOST_CCCD,
        .value={handle,generation,token,stage,phase,result,kind}};
    (void)probe_audio_host_record(&r);
#else
    (void)handle;(void)generation;(void)token;(void)stage;(void)phase;(void)result;(void)kind;
#endif
}

typedef struct {uint16_t handle,values[PROBE_CCCD_COUNT];uint32_t generation;uint8_t mask;} cccd_touched_t;
static cccd_touched_t cccd_touched[DBT_MAX_LINKS];
static uint8_t *echo;
static uint16_t echo_size;
static uint32_t writes, notified[DBT_MAX_LINKS], notified_generation[DBT_MAX_LINKS];
/* CCCD jobs do not use command reply storage. Byte-aligned clock copies keep
 * the existing job size/alignment; neither keys nor NVS payloads enter logs. */
typedef struct {uint8_t clock[3][sizeof(uint32_t)],worker_core;bool executed;} cccd_work_timing_t;
_Static_assert(sizeof(cccd_work_timing_t)<=sizeof(dbt_ble_control_result_t) &&
    _Alignof(cccd_work_timing_t)==_Alignof(dbt_ble_control_result_t),"Reuse CCCD reply storage without job growth");
typedef struct control_job {
    struct control_job *next;
    uint32_t generation,token;
    uint32_t ota_epoch,ota_deadline,trust_cookie;
    uint16_t handle,size,offset;
    bool request,eq_lane,local;
    uint8_t cccd_kind,identity_type,identity[6],bond_tag[32];
    uint16_t cccd_values[PROBE_CCCD_COUNT],cccd_error_handle;
    dbt_result_t cccd_result;
    _Atomic bool cancelled;
    union {dbt_ble_control_result_t result;cccd_work_timing_t cccd_timing;};
    dbt_ble_control_state_t state;
    uint8_t data[];
} control_job_t;
#define CONTROL_JOBS 4u
#define CONTROL_REGISTRY_CAPACITY (CONTROL_JOBS+1u)
#define CONTROL_STACK_BYTES 4096u
static QueueHandle_t control_work,eq_work,control_done;
static TaskHandle_t control_owner;
static SemaphoreHandle_t control_signal,eq_signal;
static dbt_ble_control_hooks_t control_hooks;
static dbt_ble_control_state_t control_display_state;
static int8_t desired_eq[3];
static uint8_t desired_control;
static control_job_t *control_head,*control_tail;
static control_job_t *ota_finish_job;
static struct {uint16_t handle;uint32_t generation,epoch,deadline;} ota_notice;
static struct {uint16_t handle;uint32_t generation,epoch;} ota_fence;
static struct {uint16_t handle;uint32_t generation,cookie,token;} guarded_acks[DBT_MAX_LINKS];
static void guard_pending_acks(void);
static control_job_t *control_registry[CONTROL_REGISTRY_CAPACITY];
static unsigned control_jobs,cccd_jobs,eq_running,control_running,control_writes_running;
static bool control_configured,control_worker,control_retiring,eq_worker,eq_retiring;
static bool eq_publication_pending,control_publication_pending;
static _Atomic bool external_worker_required;
static uint32_t control_now_ms,control_retry_at,eq_retry_at;
static bool control_retry_pending,eq_retry_pending;
static uint32_t status_pending_generation[DBT_MAX_LINKS];
static const uint8_t primary[] = {0, 0x28}, declaration[] = {3, 0x28}, cccd[] = {2, 0x29};
static const uint8_t extended_properties[] = {0, 0x29}, reliable_write[] = {1, 0};
static const uint8_t gap_uuid[] = {0, 0x18}, gatt_uuid[] = {1, 0x18};
static const uint8_t name_uuid[] = {0, 0x2a}, changed_uuid[] = {5, 0x2a};
static const uint8_t name[] = "NATIVE_BT_PROBE";
static const uint8_t name_declaration[] = {2, 3, 0, 0, 0x2a};
static const uint8_t changed_declaration[] = {0x20, 6, 0, 5, 0x2a};
static const uint8_t changed_value[] = {1, 0, 0xff, 0xff};
/* 12345678-1234-1234-1234-12345678f000, ...f001, ...f002. */
static const uint8_t service_uuid[] = {0,    0xf0, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12,
                                       0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12};
static const uint8_t echo_uuid[] = {1,    0xf0, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12,
                                    0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12};
static const uint8_t counter_uuid[] = {2,    0xf0, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12,
                                       0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12};
/* Baseline app service 12345678-1234-1234-1234-123456789000..9003. */
static const uint8_t product_service[]={0x00,0x90,0x78,0x56,0x34,0x12,0x34,0x12,0x34,0x12,0x34,0x12,0x78,0x56,0x34,0x12};
static const uint8_t product_cmd[]={0x01,0x90,0x78,0x56,0x34,0x12,0x34,0x12,0x34,0x12,0x34,0x12,0x78,0x56,0x34,0x12};
static const uint8_t product_status[]={0x02,0x90,0x78,0x56,0x34,0x12,0x34,0x12,0x34,0x12,0x34,0x12,0x78,0x56,0x34,0x12};
static const uint8_t product_meter[]={0x03,0x90,0x78,0x56,0x34,0x12,0x34,0x12,0x34,0x12,0x34,0x12,0x78,0x56,0x34,0x12};
static const uint8_t product_cmd_decl[]={0x0c,0x22,0,0x01,0x90,0x78,0x56,0x34,0x12,0x34,0x12,0x34,0x12,0x34,0x12,0x78,0x56,0x34,0x12};
static const uint8_t product_status_decl[]={0x12,0x24,0,0x02,0x90,0x78,0x56,0x34,0x12,0x34,0x12,0x34,0x12,0x34,0x12,0x78,0x56,0x34,0x12};
static const uint8_t product_meter_decl[]={0x12,0x27,0,0x03,0x90,0x78,0x56,0x34,0x12,0x34,0x12,0x34,0x12,0x34,0x12,0x78,0x56,0x34,0x12};
static uint8_t meter_value[3];
static const uint8_t echo_declaration[] = {0x8e, 0x12, 0,    1,    0xf0, 0x78, 0x56,
                                           0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34,
                                           0x12, 0x78, 0x56, 0x34, 0x12};
static const uint8_t counter_declaration[] = {0x12, 0x15, 0,    2,    0xf0, 0x78, 0x56,
                                              0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34,
                                              0x12, 0x78, 0x56, 0x34, 0x12};
static const dbt_attribute_t attributes[] = {
    {1, DBT_ATT_READ, 2, 2, 0, 2, 0, primary, gap_uuid},
    {2, DBT_ATT_READ, 5, 5, 0, 2, 0, declaration, name_declaration},
    {3, DBT_ATT_READ | DBT_ATT_DYNAMIC, 32, 0, 0, 2, 0, name_uuid, 0},
    {4, DBT_ATT_READ, 2, 2, 0, 2, 0, primary, gatt_uuid},
    {5, DBT_ATT_READ, 5, 5, 0, 2, 0, declaration, changed_declaration},
    {6, DBT_ATT_INDICATE, 4, 4, 0, 2, 0, changed_uuid, changed_value},
    {7, DBT_ATT_READ | DBT_ATT_WRITE, 2, 2, 6, 2, 0, cccd, 0},
    {0x10, DBT_ATT_READ, 16, 16, 0, 2, 0, primary, service_uuid},
    {0x11, DBT_ATT_READ, 19, 19, 0, 2, 0, declaration, echo_declaration},
    {0x12, DBT_ATT_READ | DBT_ATT_WRITE | DBT_ATT_DYNAMIC, 512, 0, 0, 16, 0, echo_uuid, 0},
    {0x13, DBT_ATT_READ, 2, 2, 0, 2, 0, extended_properties, reliable_write},
    {0x14, DBT_ATT_READ, 19, 19, 0, 2, 0, declaration, counter_declaration},
    {0x15, DBT_ATT_READ | DBT_ATT_NOTIFY | DBT_ATT_DYNAMIC, 8, 0, 0, 16, 0, counter_uuid, 0},
    {0x16, DBT_ATT_READ | DBT_ATT_WRITE, 2, 2, 0x15, 2, 0, cccd, 0},
    {0x20, DBT_ATT_READ, 16, 16, 0, 2, 0, primary, product_service},
    {0x21, DBT_ATT_READ, 19, 19, 0, 2, 0, declaration, product_cmd_decl},
    {0x22, DBT_ATT_WRITE | DBT_ATT_DYNAMIC, 512, 0, 0, 16, 0, product_cmd, 0},
    {0x23, DBT_ATT_READ, 19, 19, 0, 2, 0, declaration, product_status_decl},
    {0x24, DBT_ATT_READ | DBT_ATT_NOTIFY | DBT_ATT_DYNAMIC, 82, 0, 0, 16, 0, product_status, 0},
    {0x25, DBT_ATT_READ | DBT_ATT_WRITE, 2, 2, 0x24, 2, 0, cccd, 0},
    {0x26, DBT_ATT_READ, 19, 19, 0, 2, 0, declaration, product_meter_decl},
    {0x27, DBT_ATT_READ | DBT_ATT_NOTIFY | DBT_ATT_DYNAMIC, 3, 0, 0, 16, 0, product_meter, 0},
    {0x28, DBT_ATT_READ | DBT_ATT_WRITE, 2, 2, 0x27, 2, 0, cccd, 0}};
static void copy(uint8_t *out, const uint8_t *in, unsigned size) {
    for (unsigned i = 0; i < size; ++i)
        out[i] = in[i];
}
static void put16(uint8_t *out, uint16_t value) {
    out[0] = (uint8_t)value;
    out[1] = (uint8_t)(value >> 8);
}
static void counter(uint8_t out[8]) {
    for (unsigned i = 0; i < 4; ++i)
        out[i] = (uint8_t)(writes >> (8u * i));
    put16(out + 4, echo_size);
    put16(out + 6, 1); /* Diagnostic protocol version. */
}
static uint8_t *allocate(void *context, uint16_t size) {
    (void)context;
    return heap_caps_malloc(size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
}
static void release(void *context, uint8_t *bytes) {
    (void)context;
    heap_caps_free(bytes);
}
dbt_result_t probe_gatt_security_query_configure(
    dbt_result_t (*query)(uint16_t,uint32_t,dbt_smp_session_security_t *)) {
    if (bearer.runtime || !query) return DBT_INVALID;
    security_query=query;
    return DBT_OK;
}
dbt_result_t probe_gatt_bond_query_configure(
    dbt_result_t (*query)(uint16_t,uint32_t,uint8_t *,uint8_t[6],uint8_t[32])) {
    if(bearer.runtime || !query)return DBT_INVALID;
    bond_query=query;return DBT_OK;
}
dbt_result_t probe_gatt_ota_configure(const probe_gatt_ota_hooks_t *hooks) {
    if(bearer.runtime || !hooks || !hooks->admit || !hooks->abandon || !hooks->valid ||
       !hooks->command || !hooks->finish || !hooks->delivered || !hooks->prepare)return DBT_INVALID;
    ota_hooks=*hooks;return DBT_OK;
}
dbt_result_t probe_gatt_control_trust_configure(uint32_t (*capture)(uint16_t,uint32_t),
                                                bool (*valid)(uint16_t,uint32_t,uint32_t)) {
    if(bearer.runtime || !capture || !valid)return DBT_INVALID;
    control_trust_capture=capture;control_trust_valid=valid;return DBT_OK;
}
dbt_result_t probe_gatt_control_configure(const dbt_ble_control_state_t *initial,
                                          const dbt_ble_control_hooks_t *hooks) {
    if (bearer.runtime || !initial || !hooks || initial->name_size>32 || initial->firmware_size>32)
        return DBT_INVALID;
    control_hooks=*hooks; control_display_state=*initial;memcpy(desired_eq,initial->eq,3);desired_control=initial->control;
    control_configured=true; return DBT_OK;
}
static void cccd_clock_set(control_job_t *job,unsigned phase,uint32_t value){
    memcpy(job->cccd_timing.clock[phase],&value,sizeof(value));
}
static void cccd_work_record(const control_job_t *job){
#if defined(DBT_PROBE_AUDIO_TEST) && (defined(DBT_PROBE_STAY_NATIVE) || defined(DBT_PROBE_SCHEDULER_TRACE))
    uint32_t clocks[3];memcpy(clocks,job->cccd_timing.clock,sizeof(clocks));
    const probe_host_record_t record={.kind=PROBE_HOST_CCCD_WORK,.value={
        job->handle,job->generation,job->token,(uint32_t)job->cccd_kind|
        ((uint32_t)(job->cccd_timing.worker_core&1u)<<PROBE_CCCD_WORK_CORE_SHIFT)|
        (job->cccd_timing.executed?PROBE_CCCD_WORK_EXECUTED:0u),clocks[0],clocks[1],clocks[2]}};
    (void)probe_audio_host_record(&record);
#else
    (void)job;
#endif
}
static void control_execute_job(control_job_t *job) {
    if(job->cccd_kind) {
#ifdef DBT_PROBE_HEAP_ALLOC_TRACE
        probe_heap_mark_t mark=probe_heap_alloc_begin(job->cccd_kind);
#endif
        cccd_clock_set(job,1,(uint32_t)esp_timer_get_time());
        job->cccd_timing.worker_core=(uint8_t)xPortGetCoreID();
        job->cccd_result=job->cccd_kind==1
            ? probe_cccd_load(job->identity_type,job->identity,job->bond_tag,job->cccd_values)
            : probe_cccd_save(job->identity_type,job->identity,job->bond_tag,job->cccd_values);
        cccd_clock_set(job,2,(uint32_t)esp_timer_get_time());job->cccd_timing.executed=true;
#ifdef DBT_PROBE_HEAP_ALLOC_TRACE
        probe_heap_alloc_end(mark);
#endif
        return;
    }
    dbt_ble_control_peer_t peer={job->handle,job->generation};
    if(job->ota_epoch) {
        uint8_t n=DBT_BLE_CONTROL_MAX_STATUS;
        if(atomic_load_explicit(&job->cancelled,memory_order_acquire) ||
           !ota_hooks.command(ota_hooks.context,&peer,job->ota_epoch,job->data[0],job->data+1,
                              (uint16_t)(job->size-1u),job->result.data,&n) || !n || n>DBT_BLE_CONTROL_MAX_STATUS) {
            job->result.data[0]=0x11;job->result.data[1]=job->data[0];job->result.data[2]=3;job->result.size=3;
        } else job->result.size=n;
        return;
    }
    bool persistent_eq=job->size && (job->data[0]==1 || job->data[0]==2 || job->data[0]==3);
    if(job->trust_cookie && !persistent_eq &&
       (!control_trust_valid || !control_trust_valid(job->handle,job->generation,job->trust_cookie))) {
        job->result.data[0]=0x11;job->result.data[1]=job->data[0];job->result.data[2]=2;job->result.size=3;return;
    }
    if(atomic_load_explicit(&job->cancelled,memory_order_acquire) ||
       dbt_ble_control_execute(&job->state,&control_hooks,&peer,job->data,job->size,&job->result)!=DBT_OK) {
        job->result.data[0]=0x11;job->result.data[1]=job->size?job->data[0]:0;job->result.data[2]=2;job->result.size=3;
    }
    /* The write has already committed. Retry only nonblocking publication,
     * never the NVS operation, and bound scheduling work. */
    if(job->result.durable_eq && job->result.data[0]==0x11 && control_hooks.publish_eq)
        for(unsigned i=0;i<2;i++) {
            taskYIELD();
            if(control_hooks.publish_eq(control_hooks.context,job->state.eq[0],job->state.eq[1],job->state.eq[2])) {
                job->result.data[0]=0x10;job->result.data[1]=job->data[0];job->result.size=2;break;
            }
        }
    if(job->result.durable_control && job->result.data[0]==0x11 && control_hooks.publish_control)
        for(unsigned i=0;i<2;i++) {
            taskYIELD();
            if(control_hooks.publish_control(control_hooks.context,job->state.control)) {
                job->result.data[0]=0x10;job->result.data[1]=job->data[0];job->result.size=2;break;
            }
        }
}
static void control_task(void *unused) {
    bool eq=(uintptr_t)unused!=0,retire=false;QueueHandle_t queue=eq?eq_work:control_work;
    probe_heap_alloc_task(eq?PROBE_HEAP_PERSIST:PROBE_HEAP_GENERAL,true);
    SemaphoreHandle_t signal=eq?eq_signal:control_signal;
    TickType_t maintenance_wait=pdMS_TO_TICKS(20);
    if(!maintenance_wait)maintenance_wait=1;
    for (;;) {
        if(retire){
            if(!eq && (uxQueueMessagesWaiting(queue) || atomic_load_explicit(&external_worker_required,memory_order_acquire))){
                control_job_t *resumed=(control_job_t *)(uintptr_t)3;
                (void)xQueueSend(control_done,&resumed,portMAX_DELAY);xTaskNotifyGive(control_owner);
                retire=false;continue;
            }
            if(!eq && control_hooks.worker_poll && !control_hooks.worker_poll(control_hooks.context,true)){
                /* A command or provider completion wakes retirement early.
                 * The persistent binary semaphore also retains a wake that
                 * arrives between the level checks and this wait. */
                (void)xSemaphoreTake(signal,maintenance_wait);continue;
            }
            control_job_t *done=(control_job_t *)(uintptr_t)(eq?1u:2u);
            (void)xQueueSend(control_done,&done,portMAX_DELAY);xTaskNotifyGive(control_owner);
            probe_heap_alloc_task(eq?PROBE_HEAP_PERSIST:PROBE_HEAP_GENERAL,false);vTaskDelete(NULL);return;
        }
        control_job_t *job=0;
        TickType_t wait=(!eq && control_hooks.worker_poll)?maintenance_wait:portMAX_DELAY;
        if(xQueueReceive(queue,&job,0)!=pdTRUE){
            if(!eq && control_hooks.worker_poll)(void)control_hooks.worker_poll(control_hooks.context,false);
            (void)xSemaphoreTake(signal,wait);continue;
        }
        if(!job){retire=true;continue;}
        control_execute_job(job);
        if(!eq && control_hooks.worker_poll)(void)control_hooks.worker_poll(control_hooks.context,false);
        (void)xQueueSend(control_done,&job,portMAX_DELAY);xTaskNotifyGive(control_owner);
    }
}
static void control_free(control_job_t *job) {
    if(!job)return;
    for(unsigned i=0;i<CONTROL_REGISTRY_CAPACITY;i++)if(control_registry[i]==job)control_registry[i]=0;
    if(job->cccd_kind && cccd_jobs)--cccd_jobs;
    volatile uint8_t *p=(volatile uint8_t *)job;
    size_t n=sizeof(*job)+job->size;while(n--)*p++=0;
    heap_caps_free(job); if(control_jobs)--control_jobs;
}
static void control_append(control_job_t *job) {
    job->next=0;if(control_tail)control_tail->next=job;else control_head=job;control_tail=job;
}
static cccd_binding_t *cccd_binding(const dbt_link_t *link) {
    for(unsigned i=0;i<DBT_MAX_LINKS;i++)if(cccd_bindings[i].generation==link->generation &&
        cccd_bindings[i].handle==link->handle)return &cccd_bindings[i];
    return NULL;
}
static bool cccd_delivery_ready(const dbt_link_t *link) {
    dbt_att_connection_t *c=NULL;
    for(unsigned i=0;i<DBT_MAX_LINKS;i++)if(server.connections[i].used &&
        server.connections[i].handle==link->handle && server.connections[i].generation==link->generation)c=&server.connections[i];
    if(!c || c->failed)return false;
    dbt_smp_session_security_t sec={0};
    bool has_security=security_query && security_query(link->handle,link->generation,&sec)==DBT_OK;
    if(!c->bonded)return !has_security || !sec.bond_committed;
    cccd_binding_t *b=cccd_binding(link);uint8_t type,id[6],tag[32];
    return has_security && sec.encrypted && sec.bond_committed && b && b->phase==CCCD_BOUND && bond_query &&
           bond_query(link->handle,link->generation,&type,id,tag)==DBT_OK && type==b->type &&
           !memcmp(id,b->identity,6) && !memcmp(tag,b->tag,32);
}
static cccd_touched_t *touch_for(const dbt_link_t *link) {
    for(unsigned i=0;i<DBT_MAX_LINKS;i++) {
        dbt_att_connection_t *c=&server.connections[i];
        if(c->used && c->handle==link->handle && c->generation==link->generation) {
            cccd_touched_t *t=&cccd_touched[i];
            if(t->handle!=link->handle || t->generation!=link->generation){memset(t,0,sizeof(*t));t->handle=link->handle;t->generation=link->generation;}
            return t;
        }
    }
    return NULL;
}
static int cccd_number(uint16_t handle) {
    static const uint16_t handles[PROBE_CCCD_COUNT]={7,0x16,0x25,0x28};
    for(unsigned i=0;i<PROBE_CCCD_COUNT;i++)if(handles[i]==handle)return (int)i;
    return -1;
}

/* Reuse the per-connection touched values; never retain an ATT batch pointer. */
static bool cccd_touch_batch(const dbt_link_t *link,const dbt_att_batch_t *batch) {
    cccd_touched_t *t=touch_for(link);if(!t)return false;
    for(unsigned i=0;i<batch->cccd_count;i++)if(cccd_number(batch->cccds[i].handle)<0)return false;
    for(unsigned i=0;i<batch->cccd_count;i++){
        unsigned k=(unsigned)cccd_number(batch->cccds[i].handle);
        t->mask|=(uint8_t)(1u<<k);t->values[k]=batch->cccds[i].value;
    }
    return true;
}
static bool cccd_merge_touched(const dbt_link_t *link,cccd_binding_t *b) {
    cccd_touched_t *t=touch_for(link);if(!t)return false;
    for(unsigned i=0;i<PROBE_CCCD_COUNT;i++)if(t->mask&(1u<<i))b->values[i]=t->values[i];
    return true;
}
/* Only outside ATT callbacks. The token fences the single deferred request;
 * restoration/persistence must finish before ATT's bonded state is attached. */
static bool cccd_complete_pending(const dbt_link_t *link,cccd_binding_t *b,
                                  dbt_att_connection_t *c,uint8_t error) {
    uint32_t token=b->pending_token;if(!token)return true;b->pending_token=0;
    bool current=c && c->used && c->handle==b->handle && c->generation==b->generation &&
        c->deferred && !c->failed && c->token==token &&
        (int32_t)(control_now_ms-c->request_deadline)<0;
    if(!current || error){
        cccd_touched_t *t=touch_for(link);if(t)t->mask=0;
        b->phase=CCCD_ERROR;
    }
    if(!current)return false;
    if(!error)memcpy(c->staged_cccds,b->values,sizeof(b->values));
    dbt_result_t rc=dbt_att_complete(&server,b->handle,b->generation,token,error,c->error_handle);
    cccd_record(b->handle,b->generation,token,8,b->phase,error?error:(uint32_t)rc,0);
    return rc==DBT_OK;
}

static uint16_t cccd_enqueue(const dbt_link_t *link,cccd_binding_t *binding,uint8_t kind,
                             uint32_t token,uint16_t error_handle,const uint16_t *values) {
    /* One bounded extra CCCD job is independent of the four control slots;
     * status replies waiting for bond restore cannot starve that restore. */
    if(cccd_jobs)return DBT_ATT_RESOURCES;
    control_job_t *job=heap_caps_malloc(sizeof(*job),MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
    if(!job)return DBT_ATT_RESOURCES;
    memset(job,0,sizeof(*job));atomic_init(&job->cancelled,false);job->cccd_kind=kind;
    job->handle=link->handle;job->generation=link->generation;job->token=token;
    job->cccd_result=DBT_INVALID;
    job->cccd_error_handle=error_handle;job->identity_type=binding->type;
    memcpy(job->identity,binding->identity,6);memcpy(job->bond_tag,binding->tag,32);
    if(values)memcpy(job->cccd_values,values,sizeof(job->cccd_values));
    /* Serialize CCCDs with EQ/control persistence. General provider/OTA work
     * retains its own Core1 lane and cannot hold this NVS mailbox hostage. */
    cccd_clock_set(job,0,(uint32_t)esp_timer_get_time());
    if(xQueueSend(eq_work,&job,0)!=pdTRUE){heap_caps_free(job);return DBT_ATT_RESOURCES;}
    ++control_jobs;++cccd_jobs;for(unsigned i=0;i<CONTROL_REGISTRY_CAPACITY;i++)if(!control_registry[i]){control_registry[i]=job;break;}
    cccd_record(link->handle,link->generation,token,1,binding->phase,DBT_OK,kind);
    (void)xSemaphoreGive(eq_signal);return 0;
}
static dbt_result_t load_cccd(void *context,const dbt_link_t *link,uint16_t handle,uint16_t *value) {
    (void)context;static const uint16_t handles[4]={7,0x16,0x25,0x28};
    cccd_binding_t *b=cccd_binding(link);if(!b || b->phase!=CCCD_READY)return DBT_INVALID;
    for(unsigned i=0;i<PROBE_CCCD_COUNT;i++)if(handles[i]==handle){*value=b->values[i];return DBT_OK;}
    return DBT_INVALID;
}
static uint16_t control_enqueue(const dbt_link_t *link,const dbt_att_write_t *w,bool command,uint32_t token,bool local) {
    if(!link || !w || w->offset || !w->size || w->size>DBT_BLE_CONTROL_MAX_COMMAND) return DBT_ATT_INVALID_LENGTH;
    bool eq=w->data[0]==1 || w->data[0]==2 || w->data[0]==3;
    unsigned ordinary_jobs=control_jobs-cccd_jobs;
    if(ordinary_jobs>=CONTROL_JOBS || (!eq && ordinary_jobs>=CONTROL_JOBS-1u)) return DBT_ATT_RESOURCES;
    control_job_t *job=heap_caps_malloc(sizeof(*job)+w->size,MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
    if(!job)return DBT_ATT_RESOURCES;
    memset(job,0,sizeof(*job));atomic_init(&job->cancelled,false);job->handle=link->handle;job->generation=link->generation;
    job->local=local;job->token=token;job->request=!command;job->size=w->size;memcpy(job->data,w->data,w->size);
    bool ota=!local && w->data[0]>=0x20 && w->data[0]<=0x23;
    bool privileged=!local && !ota && w->data[0]!=0xf0 && w->data[0]!=0xff;
    if(privileged && (!control_trust_capture ||
       !(job->trust_cookie=control_trust_capture(link->handle,link->generation)))) {
        heap_caps_free(job);return DBT_ATT_AUTHORIZATION;
    }
    if(ota){
        dbt_ble_control_peer_t peer={link->handle,link->generation};
        if(!ota_hooks.admit || !(job->ota_epoch=ota_hooks.admit(ota_hooks.context,&peer,w->data[0],w->data+1,(uint16_t)(w->size-1u)))) {
            heap_caps_free(job);return DBT_ATT_AUTHORIZATION;
        }
    }
    job->eq_lane=eq;job->state=control_display_state;
    ++control_jobs;
    if(xQueueSend(job->eq_lane?eq_work:control_work,&job,0)!=pdTRUE){
        if(ota && (w->data[0]==0x20 || w->data[0]==0x22)){dbt_ble_control_peer_t peer={link->handle,link->generation};
            ota_hooks.abandon(ota_hooks.context,&peer,job->ota_epoch,w->data[0]);}
        control_free(job);return DBT_ATT_RESOURCES;}
    (void)xSemaphoreGive(eq?eq_signal:control_signal);
    if(w->data[0]==3)++control_writes_running;
    else if(job->eq_lane)++eq_running;else ++control_running;
    for(unsigned i=0;i<CONTROL_REGISTRY_CAPACITY;i++)if(!control_registry[i]){control_registry[i]=job;break;}
    int8_t values[3];
    if(eq && dbt_ble_control_eq_values(w->data,w->size,values)==DBT_OK)memcpy(desired_eq,values,3);
    if(w->data[0]==3 && w->size>=2)desired_control=w->data[1]&7u;
    return command?0:DBT_ATT_DEFERRED;
}
static uint16_t control_submit(const dbt_link_t *link,const dbt_att_write_t *w,bool command,uint32_t token){
    if(!link || !w || !w->data || !w->size)return DBT_ATT_INVALID_LENGTH;
    if(ota_fence.epoch && w->data[0]!=0xf0 && w->data[0]!=0xff)return DBT_ATT_RESOURCES;
    bool ota=w->data[0]>=0x20 && w->data[0]<=0x23;
    if(w->data[0]!=0xf0 && w->data[0]!=0xff && !link->encrypted)return DBT_ATT_ENCRYPTION;
    if(ota) {
        bool enabled=false;
        for(unsigned i=0;i<DBT_MAX_LINKS;i++){dbt_att_connection_t *c=&server.connections[i];
            cccd_binding_t *b=cccd_binding(link);
            if(c->used && !c->failed && c->bonded && b && b->phase==CCCD_BOUND &&
               c->handle==link->handle && c->generation==link->generation && (c->cccds[2]&1u))enabled=true;}
        if(!enabled || !cccd_delivery_ready(link) || !ota_hooks.admit)return DBT_ATT_AUTHORIZATION;
    }
    if(!ota && w->data[0]!=0xf0 && w->data[0]!=0xff &&
       (!control_hooks.authorize || !control_hooks.authorize(control_hooks.context,link,w->data[0])))
        return DBT_ATT_AUTHORIZATION;
    return control_enqueue(link,w,command,token,false);
}

static dbt_result_t control_eq_delta(uint8_t band,int8_t delta,bool drain){
    if(!bearer.runtime || !control_configured || band>2 || (delta!=-1 && delta!=1))return DBT_INVALID;
    if(drain?!ota_fence.epoch:ota_fence.epoch!=0)return drain?DBT_INVALID:DBT_BUSY;
    int next=(int)desired_eq[band]+delta;if(next<-12 || next>12)return DBT_OK;
    uint8_t data[]={1,(uint8_t)desired_eq[0],(uint8_t)desired_eq[1],(uint8_t)desired_eq[2]};data[band+1]=(uint8_t)next;
    const dbt_link_t local={.handle=UINT16_MAX,.generation=1};
    const dbt_att_write_t w={.data=data,.size=sizeof(data)};
    uint16_t e=control_enqueue(&local,&w,true,0,true);
    return !e?DBT_OK:e==DBT_ATT_RESOURCES?DBT_BUSY:DBT_INVALID;
}
dbt_result_t probe_gatt_control_eq_delta(uint8_t band,int8_t delta){return control_eq_delta(band,delta,false);}
dbt_result_t probe_gatt_control_eq_delta_drain(uint8_t band,int8_t delta){return control_eq_delta(band,delta,true);}
static dbt_result_t control_toggle(uint8_t flag,bool drain){
    if(!bearer.runtime || !control_configured || (flag!=1 && flag!=2 && flag!=4))return DBT_INVALID;
    if(drain?!ota_fence.epoch:ota_fence.epoch!=0)return drain?DBT_INVALID:DBT_BUSY;
    const uint8_t data[]={3,(uint8_t)(desired_control^flag)};
    const dbt_link_t local={.handle=UINT16_MAX,.generation=1};
    const dbt_att_write_t w={.data=data,.size=sizeof(data)};
    uint16_t e=control_enqueue(&local,&w,true,0,true);
    return !e?DBT_OK:e==DBT_ATT_RESOURCES?DBT_BUSY:DBT_INVALID;
}
dbt_result_t probe_gatt_control_toggle(uint8_t flag){return control_toggle(flag,false);}
dbt_result_t probe_gatt_control_toggle_drain(uint8_t flag){return control_toggle(flag,true);}
static uint8_t read(void *context, const dbt_link_t *link, const dbt_attribute_t *attribute,
                    uint16_t offset, uint8_t *out, uint16_t capacity, uint16_t *written,
                    uint16_t *total) {
    (void)context;
    (void)link;
    uint8_t status[DBT_BLE_CONTROL_MAX_STATUS];
    const uint8_t *source = echo;
    *total = echo_size;
    if(attribute->handle==3){source=control_configured?control_display_state.name:name;*total=control_configured?control_display_state.name_size:sizeof(name)-1;}
    else if(attribute->handle==0x24) {
        uint8_t n=0;if(dbt_ble_control_status(&control_display_state,status,&n)!=DBT_OK)return DBT_ATT_UNLIKELY;
        source=status;*total=n;
    } else if(attribute->handle==0x27) { source=meter_value;*total=3; }
    if (attribute->handle == 0x15) {
        counter(status);
        source = status;
        *total = 8;
    }
    if (offset > *total)
        return DBT_ATT_INVALID_OFFSET;
    *written = (uint16_t)(*total - offset);
    if (*written > capacity)
        *written = capacity;
    if (*written)
        copy(out, source + offset, *written);
    return 0;
}
static uint16_t write(void *context, const dbt_link_t *link, const dbt_att_batch_t *batch,
                      uint32_t token, uint16_t *error_handle) {
    (void)context;
    if(ota_fence.epoch && batch->count){*error_handle=batch->writes[0].handle;return DBT_ATT_RESOURCES;}
    if(batch->cccd_count && security_query) {
        dbt_smp_session_security_t security={0};
        if(security_query(link->handle,link->generation,&security)==DBT_OK && security.bond_committed) {
            *error_handle=batch->cccds[0].handle;
            cccd_binding_t *b=cccd_binding(link);dbt_att_connection_t *c=NULL;
            for(unsigned i=0;i<DBT_MAX_LINKS;i++)if(server.connections[i].used &&
                server.connections[i].handle==link->handle && server.connections[i].generation==link->generation)c=&server.connections[i];
            if(!c || c->failed || !security.encrypted || !link->encrypted || security.key_size<7 ||
               batch->command || batch->count!=batch->cccd_count){
                cccd_record(link->handle,link->generation,token,3,b?b->phase:UINT32_MAX,DBT_ATT_UNLIKELY,0);return DBT_ATT_UNLIKELY;
            }
            uint8_t type,id[6],tag[32];
            if(!bond_query || bond_query(link->handle,link->generation,&type,id,tag)!=DBT_OK)
                return DBT_ATT_UNLIKELY;
            /* A controller event and the first write may share an owner poll,
             * before sync_cccd has installed the new binding. No NVS work here. */
            if(!b && !c->bonded)for(unsigned i=0;i<DBT_MAX_LINKS;i++){
                const dbt_link_t *l=&bearer.runtime->acl.links[i];
                if(!l->used || l->handle!=link->handle || l->generation!=link->generation)continue;
                b=&cccd_bindings[i];memset(b,0,sizeof(*b));b->handle=l->handle;b->generation=l->generation;
                b->type=type;memcpy(b->identity,id,6);memcpy(b->tag,tag,32);break;
            }
            if(!b || type!=b->type || memcmp(id,b->identity,6) || memcmp(tag,b->tag,32)){
                cccd_record(link->handle,link->generation,token,4,b?b->phase:UINT32_MAX,DBT_ATT_UNLIKELY,0);return DBT_ATT_UNLIKELY;
            }
            if(b->phase==CCCD_BOUND && c->bonded){
                uint16_t rc=cccd_enqueue(link,b,2,token,*error_handle,c->staged_cccds);
                return rc?rc:DBT_ATT_DEFERRED;
            }
            if(c->bonded || b->phase==CCCD_BOUND || b->phase==CCCD_ERROR || b->pending_token ||
               !cccd_touch_batch(link,batch))return DBT_ATT_UNLIKELY;
            b->pending_token=token;
            if(b->phase==CCCD_READY || b->phase==CCCD_MERGE){
                (void)cccd_merge_touched(link,b);b->phase=CCCD_MERGE;
            }
            cccd_record(link->handle,link->generation,token,7,b->phase,DBT_ATT_DEFERRED,0);
            return DBT_ATT_DEFERRED;
        }
    }
    /* If ATT still believes the connection bonded but the security snapshot
     * no longer matches, never fall back to ephemeral success. */
    if(batch->cccd_count) for(unsigned i=0;i<DBT_MAX_LINKS;i++) {
        const dbt_att_connection_t *c=&server.connections[i];
        if(c->used && c->handle==link->handle && c->generation==link->generation && c->bonded) {
            *error_handle=batch->cccds[0].handle;
            cccd_record(link->handle,link->generation,token,5,UINT32_MAX,DBT_ATT_UNLIKELY,0);return DBT_ATT_UNLIKELY;
        }
    }
    for(unsigned i=0;i<batch->count;i++) if(batch->writes[i].handle==0x22) {
        *error_handle=0x22;
        if(batch->count!=1 || batch->cccd_count || batch->prepared)return DBT_ATT_NOT_SUPPORTED;
        return control_submit(link,&batch->writes[i],batch->command,token);
    }
    bool changed = false;
    uint16_t size = batch->prepared ? echo_size : 0;
    for (unsigned i = 0; i < batch->count; ++i) {
        const dbt_att_write_t *w = &batch->writes[i];
        if (w->handle != 0x12)
            continue;
        changed = true;
        *error_handle = w->handle;
        if (w->offset > size || (uint32_t)w->offset + w->size > 512)
            return DBT_ATT_INVALID_OFFSET;
        if (w->offset + w->size > size)
            size = (uint16_t)(w->offset + w->size);
    }
    uint8_t *candidate = 0;
    if (changed && size) {
        candidate = allocate(0, size);
        if (!candidate)
            return DBT_ATT_RESOURCES;
        for (unsigned i = 0; i < size; ++i)
            candidate[i] = i < echo_size ? echo[i] : 0;
        for (unsigned i = 0; i < batch->count; ++i) {
            const dbt_att_write_t *w = &batch->writes[i];
            if (w->handle == 0x12)
                copy(candidate + w->offset, w->data, w->size);
        }
    }
    if (changed) {
        release(0, echo);
        echo = candidate;
        echo_size = size;
    }
    if(batch->cccd_count) {
        cccd_touched_t *t=touch_for(link);
        if(!t)return DBT_ATT_UNLIKELY;
        for(unsigned i=0;i<batch->cccd_count;i++){
            int k=cccd_number(batch->cccds[i].handle);
            if(k<0)return DBT_ATT_UNLIKELY;
            t->mask|=(uint8_t)(1u<<(unsigned)k);t->values[k]=batch->cccds[i].value;
            if(batch->cccds[i].handle==0x25)for(unsigned j=0;j<DBT_MAX_LINKS;j++){
                const dbt_link_t *l=&bearer.runtime->acl.links[j];
                if(l->used && l->handle==link->handle && l->generation==link->generation)
                    status_pending_generation[j]=(batch->cccds[i].value&1)?link->generation:0;
            }
        }
    }
    ++writes;
    return 0;
}
static void failed(void *context, uint16_t handle, uint32_t generation) {
    (void)context;
    const dbt_link_t *link = dbt_acl_find(&bearer.runtime->acl, handle);
    if (link && link->generation == generation)
        (void)dbt_gap_disconnect(gap, link, 0x13);
}
dbt_result_t probe_gatt_init(dbt_runtime_t *runtime, dbt_gap_t *configuration) {
    if(!control_configured) memset(&control_display_state,0,sizeof(control_display_state));
    dbt_att_hooks_t hooks = {0};
    hooks.allocate = allocate;
    hooks.release = release;
    hooks.read = read;
    hooks.write = write;
    hooks.load_cccd = load_cccd;
    hooks.failed = failed;
    control_work=xQueueCreate(CONTROL_JOBS,sizeof(control_job_t *));
    eq_work=xQueueCreate(CONTROL_JOBS+1u,sizeof(control_job_t *)); /* Reserved CCCD obligation. */
    control_done=xQueueCreate(CONTROL_JOBS,sizeof(control_job_t *));
    control_signal=xSemaphoreCreateBinary();eq_signal=xSemaphoreCreateBinary();
    control_owner=xTaskGetCurrentTaskHandle();
    if(!control_work || !eq_work || !control_done || !control_signal || !eq_signal || !control_owner) {
        if(control_work)vQueueDelete(control_work);
        if(eq_work)vQueueDelete(eq_work);
        if(control_done)vQueueDelete(control_done);
        if(control_signal)vSemaphoreDelete(control_signal);
        if(eq_signal)vSemaphoreDelete(eq_signal);
        control_signal=eq_signal=0;control_work=eq_work=control_done=0;return DBT_NO_SPACE;
    }
    dbt_result_t result = dbt_att_init(
        &server, attributes, sizeof(attributes) / sizeof(attributes[0]), hooks, 517, 1024);
    if (result != DBT_OK) goto failed_init;
    gap = configuration;
    result=dbt_att_bearer_init(&bearer, runtime, &server);
    if(result==DBT_OK)return result;
    dbt_att_clear(&server);
failed_init:
    vQueueDelete(control_work);vQueueDelete(eq_work);vQueueDelete(control_done);
    vSemaphoreDelete(control_signal);vSemaphoreDelete(eq_signal);control_signal=eq_signal=0;
    control_work=eq_work=control_done=0;return result;
}
void probe_gatt_advertisement(dbt_gap_config_t *configuration) {
    configuration->advertising[0] = 2;
    configuration->advertising[1] = 1;
    configuration->advertising[2] = 2;
    configuration->advertising[3] = 17;
    configuration->advertising[4] = 7;
    copy(configuration->advertising + 5, product_service, 16);
    configuration->advertising_size = 21;
    const uint8_t *local=configuration->name[0]?(const uint8_t *)configuration->name:name;
    unsigned length=0;while(length<(local==name?sizeof(name):sizeof(configuration->name)) && local[length])++length;
    if(!length){local=name;length=sizeof(name)-1;}
    unsigned used=length>29?29:length;
    if(used<length)while(used && (local[used]&0xc0u)==0x80u)--used;
    configuration->scan_response[0]=(uint8_t)(used+1u);
    configuration->scan_response[1]=used<length?8:9;
    copy(configuration->scan_response+2,local,used);
    configuration->scan_response_size=(uint8_t)(used+2u);
}
void probe_gatt_link(void *context, const dbt_link_change_t *change) {
    (void)context;
    if (!bearer.runtime)
        return;
    if(change && change->kind==DBT_LINK_REMOVED && control_hooks.peer_lost){dbt_ble_control_peer_t peer={change->handle,change->generation};control_hooks.peer_lost(control_hooks.context,&peer);}
    if(change && change->kind==DBT_LINK_REMOVED) for(unsigned i=0;i<CONTROL_REGISTRY_CAPACITY;i++) {
        control_job_t *job=control_registry[i];
        if(job && job->ota_epoch && job->handle==change->handle && job->generation==change->generation) {
            dbt_ble_control_peer_t peer={job->handle,job->generation};
            ota_hooks.abandon(ota_hooks.context,&peer,job->ota_epoch,job->data[0]);
        }
        /* An accepted EQ change must still commit even if its originating
         * link vanishes; other unstarted jobs can be cancelled. */
        if(job && !job->cccd_kind && job->handle==change->handle && job->generation==change->generation &&
           job->data[0]!=1 && job->data[0]!=2 && job->data[0]!=3)
            atomic_store_explicit(&job->cancelled,true,memory_order_release);
    }
    if(change && change->kind==DBT_LINK_REMOVED)for(unsigned i=0;i<DBT_MAX_LINKS;i++)
        if(cccd_bindings[i].handle==change->handle && cccd_bindings[i].generation==change->generation)
            memset(&cccd_bindings[i],0,sizeof(cccd_bindings[i]));
    if(change && change->kind==DBT_LINK_REMOVED)for(unsigned i=0;i<DBT_MAX_LINKS;i++)
        if(cccd_touched[i].handle==change->handle && cccd_touched[i].generation==change->generation)
            memset(&cccd_touched[i],0,sizeof(cccd_touched[i]));
    if(change && change->kind==DBT_LINK_REMOVED)for(unsigned i=0;i<DBT_MAX_LINKS;i++)
        if(guarded_acks[i].handle==change->handle && guarded_acks[i].generation==change->generation)
            memset(&guarded_acks[i],0,sizeof(guarded_acks[i]));
    dbt_result_t result = dbt_att_bearer_link(&bearer, change);
    if (result != DBT_OK)
        dbt_host_abort(&bearer.runtime->host, DBT_FAULT_BAD_RESPONSE);
}
dbt_result_t probe_gatt_receive_oversized(const dbt_link_t *link,uint8_t opcode,uint16_t declared){
    if(!bearer.runtime)return DBT_INVALID;
    guard_pending_acks();
    return dbt_att_receive_oversized(&server,link,opcode,declared,bearer.runtime->now_ms);
}
void probe_gatt_receive(void *context, const dbt_link_t *link, uint16_t cid, const uint8_t *data,
                        uint16_t size) {
    (void)context;
    guard_pending_acks();
    /* A command cannot replace the token of a request whose successful ACK
     * is still waiting on a borrowed/unavailable transport. */
    if(link && cid==4 && data && size && data[0]==0x52)for(unsigned i=0;i<DBT_MAX_LINKS;i++)
        if(guarded_acks[i].cookie && guarded_acks[i].handle==link->handle &&
           guarded_acks[i].generation==link->generation){++server.ignored_commands;return;}
    dbt_result_t result = dbt_att_bearer_receive(&bearer, link, cid, data, size);
    if (result == DBT_INVALID || result == DBT_BUSY)
        (void)dbt_gap_disconnect(gap, link, 0x13);
}
static void finish_cccd(control_job_t *job) {
    const dbt_link_t *link=dbt_acl_find(&bearer.runtime->acl,job->handle);
    cccd_binding_t *b=link && link->generation==job->generation?cccd_binding(link):NULL;
    cccd_record(job->handle,job->generation,job->token,2,b?b->phase:UINT32_MAX,job->cccd_result,job->cccd_kind);
    cccd_work_record(job);
    uint8_t type,id[6],tag[32];
    bool current=b && bond_query && bond_query(job->handle,job->generation,&type,id,tag)==DBT_OK &&
        type==job->identity_type && type==b->type && !memcmp(id,job->identity,6) &&
        !memcmp(id,b->identity,6) && !memcmp(tag,job->bond_tag,32) && !memcmp(tag,b->tag,32);
    if(job->cccd_kind==1) {
        if(current && b->phase==CCCD_LOADING) {
            if(job->cccd_result==DBT_OK || job->cccd_result==DBT_EMPTY) {
                memcpy(b->values,job->cccd_values,sizeof(b->values));
                cccd_touched_t *t=touch_for(link);
                if(!cccd_merge_touched(link,b))b->phase=CCCD_ERROR;
                else b->phase=t->mask?CCCD_MERGE:CCCD_READY;
            } else if(job->cccd_result==DBT_NO_SPACE) {
                b->phase=CCCD_NONE;b->retry_pending=true;b->retry_at=control_now_ms+100u;
            } else b->phase=CCCD_ERROR;
        }
    } else if(job->cccd_kind==3) {
        if(current && b->phase==CCCD_SAVING) {
            if(job->cccd_result==DBT_OK){
                /* A pending write may arrive after this save was copied.
                 * Merge it and save again before completing that request. */
                memcpy(b->values,job->cccd_values,sizeof(b->values));
                if(!cccd_merge_touched(link,b))b->phase=CCCD_ERROR;
                else b->phase=memcmp(b->values,job->cccd_values,sizeof(b->values))?CCCD_MERGE:CCCD_READY;
            }
            else if(job->cccd_result==DBT_NO_SPACE){b->phase=CCCD_MERGE;b->retry_pending=true;b->retry_at=control_now_ms+100u;}
            else b->phase=CCCD_ERROR;
        }
    } else if(current && b->phase==CCCD_BOUND) {
        uint8_t error=job->cccd_result==DBT_OK?0:DBT_ATT_UNLIKELY;
        if(dbt_att_complete(&server,job->handle,job->generation,job->token,error,job->cccd_error_handle)==DBT_OK && !error) {
            memcpy(b->values,job->cccd_values,sizeof(b->values));
            for(unsigned i=0;i<DBT_MAX_LINKS;i++)if(&bearer.runtime->acl.links[i]==link)
                status_pending_generation[i]=(b->values[2]&1u)?link->generation:0;
        }
    } else if(link && link->generation==job->generation) {
        (void)dbt_att_complete(&server,job->handle,job->generation,job->token,DBT_ATT_UNLIKELY,job->cccd_error_handle);
    }
    control_free(job);
}
static void finish_job(control_job_t *job) {
    if(job->cccd_kind){finish_cccd(job);return;}
    bool stale_trust=job->trust_cookie && (!control_trust_valid ||
        !control_trust_valid(job->handle,job->generation,job->trust_cookie));
    if(job->eq_lane && job->data[0]==3){if(control_writes_running)--control_writes_running;}
    else if(job->eq_lane && eq_running)--eq_running;
    if(!job->eq_lane && control_running)--control_running;
    if(job->result.durable_eq) {
        memcpy(control_display_state.eq,job->state.eq,3);
        eq_publication_pending=job->result.data[0]!=0x10;
    }
    if(job->eq_lane && job->data[0]!=3 && !eq_running)memcpy(desired_eq,control_display_state.eq,3);
    if(job->result.durable_control) {
        control_display_state.control=job->state.control;
        control_publication_pending=job->result.data[0]!=0x10;
    }
    if(job->eq_lane && job->data[0]==3 && !control_writes_running)desired_control=control_display_state.control;
    if(!stale_trust && job->size && job->data[0]==4 && job->result.size==2 && job->result.data[0]==0x10 && job->result.data[1]==4) {
        control_display_state.name_size=job->state.name_size;
        memcpy(control_display_state.name,job->state.name,job->state.name_size);
    }
    if(!stale_trust && job->size && (job->data[0]==5 || job->data[0]==6 || job->data[0]==7) &&
       job->result.size==2 && job->result.data[0]==0x10 && job->result.data[1]==job->data[0]) {
        if(job->data[0]==5)memcpy(control_display_state.led,job->state.led,10);
        else if(job->data[0]==6)control_display_state.led[0]=job->state.led[0];
        else control_display_state.led[1]=job->state.led[1];
    }
    if(!stale_trust && job->result.size==2 && job->result.data[0]==0x06) control_display_state.sound=job->state.sound;
    if(job->local){
        if(job->result.durable_eq || job->result.durable_control)for(unsigned i=0;i<DBT_MAX_LINKS;i++){
            const dbt_link_t *l=&bearer.runtime->acl.links[i];
            if(l->used && l->kind==DBT_LINK_LE)status_pending_generation[i]=l->generation;
        }
        control_free(job);return;
    }
    const dbt_link_t *link=dbt_acl_find(&bearer.runtime->acl,job->handle);
    if(!link || link->generation!=job->generation || link->kind!=DBT_LINK_LE) {
        if(job->ota_epoch){dbt_ble_control_peer_t peer={job->handle,job->generation};
            ota_hooks.abandon(ota_hooks.context,&peer,job->ota_epoch,job->data[0]);}
        control_free(job);return;
    }
    if(stale_trust) {
        if(job->request)(void)dbt_att_complete(&server,job->handle,job->generation,job->token,DBT_ATT_AUTHORIZATION,0x22);
        control_free(job);return;
    }
    if(job->ota_epoch) {
        dbt_ble_control_peer_t peer={job->handle,job->generation};
        bool valid=ota_hooks.valid(ota_hooks.context,&peer,job->ota_epoch);
        if(!valid){if(job->request)(void)dbt_att_complete(&server,job->handle,job->generation,job->token,DBT_ATT_UNLIKELY,0x22);
            control_free(job);return;}
        if(job->data[0]==0x22 && job->result.size==1 && job->result.data[0]==0x22) {
            if(ota_finish_job){ota_hooks.abandon(ota_hooks.context,&peer,job->ota_epoch,job->data[0]);
                if(job->request)(void)dbt_att_complete(&server,job->handle,job->generation,job->token,DBT_ATT_UNLIKELY,0x22);
                control_free(job);return;}
            ota_finish_job=job;job->ota_deadline=control_now_ms+30000u;
            ota_fence.handle=job->handle;ota_fence.generation=job->generation;ota_fence.epoch=job->ota_epoch;
            return;
        }
        if(job->result.data[0]==0x11 || job->result.data[0]==0x23 || job->data[0]==0x23)
            ota_hooks.abandon(ota_hooks.context,&peer,job->ota_epoch,job->data[0]);
    }
    if(job->request) {
        uint8_t e=job->result.size && job->result.data[0]!=0x11 &&
                  !(job->ota_epoch && job->result.data[0]==0x23)?0:DBT_ATT_UNLIKELY;
        if(dbt_att_complete(&server,job->handle,job->generation,job->token,e,0x22)!=DBT_OK) {
            if(job->ota_epoch){dbt_ble_control_peer_t peer={job->handle,job->generation};
                ota_hooks.abandon(ota_hooks.context,&peer,job->ota_epoch,job->data[0]);}
            control_free(job);return;
        }
        if(!e && job->trust_cookie)for(unsigned i=0;i<DBT_MAX_LINKS;i++) {
            dbt_att_connection_t *c=&server.connections[i];
            if(c->used && c->handle==job->handle && c->generation==job->generation) {
                guarded_acks[i].handle=job->handle;guarded_acks[i].generation=job->generation;
                guarded_acks[i].cookie=job->trust_cookie;guarded_acks[i].token=job->token;break;
            }
        }
    }
    control_append(job);
}
static dbt_att_connection_t *control_connection(const dbt_link_t *link) {
    for(unsigned i=0;i<DBT_MAX_LINKS;i++)
        if(server.connections[i].used && server.connections[i].handle==link->handle &&
           server.connections[i].generation==link->generation) return &server.connections[i];
    return 0;
}
static void guard_pending_acks(void) {
    for(unsigned i=0;i<DBT_MAX_LINKS;i++) {
        if(!guarded_acks[i].cookie)continue;
        dbt_att_connection_t *c=&server.connections[i];
        if(!c->used || c->handle!=guarded_acks[i].handle || c->generation!=guarded_acks[i].generation ||
           c->token!=guarded_acks[i].token || !c->response_ready){memset(&guarded_acks[i],0,sizeof(guarded_acks[i]));continue;}
        if(control_trust_valid && control_trust_valid(c->handle,c->generation,guarded_acks[i].cookie))continue;
        bool loaned=false;
        for(unsigned k=0;k<DBT_MAX_LINKS;k++)if(bearer.bindings[k].active &&
           bearer.bindings[k].handle==c->handle && bearer.bindings[k].generation==c->generation)loaned=true;
        if(!loaned){c->failed=true;c->deferred=false;failed(NULL,c->handle,c->generation);
            memset(&guarded_acks[i],0,sizeof(guarded_acks[i]));}
    }
}
static int status_cccd_index(void) {
    unsigned index=0;
    for(unsigned i=0;i<sizeof(attributes)/sizeof(attributes[0]);i++) {
        const dbt_attribute_t *a=&attributes[i];
        if(a->uuid_size==2 && a->uuid[0]==2 && a->uuid[1]==0x29) {
            if(a->handle==0x25)return (int)index;
            ++index;
        }
    }
    return -1;
}
static void pump_ota(void) {
    if(ota_fence.epoch) {
        dbt_ble_control_peer_t peer={ota_fence.handle,ota_fence.generation};
        if(!ota_hooks.valid(ota_hooks.context,&peer,ota_fence.epoch))probe_gatt_restart_cancel();
    }
    if(ota_notice.epoch) {
        const dbt_link_t *link=dbt_acl_find(&bearer.runtime->acl,ota_notice.handle);
        dbt_att_connection_t *c=link && link->generation==ota_notice.generation?control_connection(link):NULL;
        dbt_ble_control_peer_t peer={ota_notice.handle,ota_notice.generation};
        if(!c || c->failed || (int32_t)(control_now_ms-ota_notice.deadline)>=0 ||
           !ota_hooks.valid(ota_hooks.context,&peer,ota_notice.epoch)) {
            ota_hooks.abandon(ota_hooks.context,&peer,ota_notice.epoch,0x22);memset(&ota_notice,0,sizeof(ota_notice));
        } else if(!c->notice_ready) {
            ota_hooks.delivered(ota_hooks.context,&peer,ota_notice.epoch);memset(&ota_notice,0,sizeof(ota_notice));
        }
    }
    control_job_t *job=ota_finish_job;if(!job)return;
    const dbt_link_t *link=dbt_acl_find(&bearer.runtime->acl,job->handle);
    dbt_att_connection_t *c=link && link->generation==job->generation?control_connection(link):NULL;
    dbt_ble_control_peer_t peer={job->handle,job->generation};
    bool expired=(int32_t)(control_now_ms-job->ota_deadline)>=0 ||
        (job->request && c && (int32_t)(control_now_ms-c->request_deadline)>=0);
    bool usable=c && !c->failed && (c->cccds[2]&1u) && (!job->request || c->deferred) && !expired &&
                ota_hooks.valid(ota_hooks.context,&peer,job->ota_epoch);
    dbt_result_t r=usable?ota_hooks.prepare(ota_hooks.context,&peer,job->ota_epoch):DBT_INVALID;
    if(r==DBT_OK)r=ota_hooks.finish(ota_hooks.context,&peer,job->ota_epoch);
    if(r==DBT_BUSY)return;
    ota_finish_job=NULL;
    if(r!=DBT_OK) {
        ota_hooks.abandon(ota_hooks.context,&peer,job->ota_epoch,job->data[0]);
        probe_gatt_restart_cancel();
        if(job->request && c)(void)dbt_att_complete(&server,job->handle,job->generation,job->token,DBT_ATT_UNLIKELY,0x22);
        if(usable){job->result.data[0]=0x23;job->result.data[1]=0x12;job->result.size=2;control_append(job);}
        else control_free(job);
        return;
    }
    if(job->request && dbt_att_complete(&server,job->handle,job->generation,job->token,0,0x22)!=DBT_OK) {
        ota_hooks.abandon(ota_hooks.context,&peer,job->ota_epoch,job->data[0]);
        probe_gatt_restart_cancel();control_free(job);return;
    }
    ota_fence.handle=job->handle;ota_fence.generation=job->generation;ota_fence.epoch=job->ota_epoch;
    job->ota_deadline=control_now_ms+30000u;
    control_append(job);
}
static void pump_control(void) {
    control_job_t *job=0;
    while(xQueueReceive(control_done,&job,0)==pdTRUE) {
        if((uintptr_t)job==1u) {eq_worker=false;eq_retiring=false;continue;}
        if((uintptr_t)job==2u) {control_worker=false;control_retiring=false;continue;}
        if((uintptr_t)job==3u) {control_retiring=false;continue;}
        finish_job(job);
    }
    pump_ota();
    if(control_retry_pending && (int32_t)(control_now_ms-control_retry_at)>=0)control_retry_pending=false;
    if(!control_worker && !control_retiring && !control_retry_pending && (uxQueueMessagesWaiting(control_work) || external_worker_required)) {
        probe_heap_alloc_owner_phase(PROBE_HEAP_CREATE_GENERAL);
        BaseType_t created=xTaskCreatePinnedToCore(control_task,"ble_control_worker",CONTROL_STACK_BYTES,NULL,4,NULL,DBT_ESP32_BACKGROUND_CORE);
        probe_heap_alloc_owner_phase(0);
        if(created==pdPASS)control_worker=true;
        else {
          control_retry_pending=true;control_retry_at=control_now_ms+100u;
          while(xQueueReceive(control_work,&job,0)==pdTRUE) if(job) {
            if(job->cccd_kind)job->cccd_result=DBT_NO_SPACE;
            else {job->state=control_display_state;job->result.data[0]=0x11;job->result.data[1]=job->data[0];
                job->result.data[2]=3;job->result.size=3;}
            finish_job(job);
          }
        }
    }
    if(eq_retry_pending && (int32_t)(control_now_ms-eq_retry_at)>=0)eq_retry_pending=false;
    if(!eq_worker && !eq_retiring && !eq_retry_pending && uxQueueMessagesWaiting(eq_work)) {
        probe_heap_alloc_owner_phase(PROBE_HEAP_CREATE_PERSIST);
        BaseType_t created=xTaskCreatePinnedToCore(control_task,"ble_persist_worker",CONTROL_STACK_BYTES,(void *)(uintptr_t)1,4,NULL,DBT_ESP32_PERSISTENCE_WORKER_CORE);
        probe_heap_alloc_owner_phase(0);
        if(created==pdPASS)eq_worker=true;
        else {eq_retry_pending=true;eq_retry_at=control_now_ms+100u;}
        /* Retain every admitted persistence obligation; capacity supplies
         * backpressure until a worker can be allocated. Never ACK success here. */
    }
    for(unsigned i=0;i<DBT_MAX_LINKS;i++) {
        const dbt_link_t *l=&bearer.runtime->acl.links[i];
        if(!status_pending_generation[i])continue;
        if(!l->used || l->kind!=DBT_LINK_LE || l->generation!=status_pending_generation[i]) {
            status_pending_generation[i]=0;continue;
        }
        if(!cccd_delivery_ready(l) || control_jobs-cccd_jobs>=CONTROL_JOBS-1u)continue;
        uint8_t bytes[DBT_BLE_CONTROL_MAX_STATUS],n=0;
        if(dbt_ble_control_status(&control_display_state,bytes,&n)!=DBT_OK) {status_pending_generation[i]=0;continue;}
        job=heap_caps_malloc(sizeof(*job),MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
        if(!job)continue;
        memset(job,0,sizeof(*job));job->handle=l->handle;job->generation=l->generation;
        memcpy(job->result.data,bytes,n);job->result.size=n;++control_jobs;control_append(job);
        status_pending_generation[i]=0;
    }
    control_job_t *previous=0;
    job=control_head;
    while(job) {
        const dbt_link_t *l=dbt_acl_find(&bearer.runtime->acl,job->handle);
        bool discard=!l || l->kind!=DBT_LINK_LE || l->generation!=job->generation || !job->result.size;
        if(job->trust_cookie && (!control_trust_valid ||
           !control_trust_valid(job->handle,job->generation,job->trust_cookie)))discard=true;
        bool attempted=false,sent=false;
        bool ota_complete=job->ota_epoch && job->data[0]==0x22 && job->result.data[0]==0x22;
        if(ota_complete && ((int32_t)(control_now_ms-job->ota_deadline)>=0 ||
           !ota_hooks.valid(ota_hooks.context,&(dbt_ble_control_peer_t){job->handle,job->generation},job->ota_epoch)))discard=true;
        if(!discard) {
            dbt_att_connection_t *c=control_connection(l);
            if(!c || c->failed || c->mtu<4) discard=true;
            else {
                if(!cccd_delivery_ready(l)){previous=job;job=job->next;continue;}
                int index=status_cccd_index();
                if(index<0 || (unsigned)index>=DBT_ATT_CCCD_LIMIT)discard=true;
                else if(!(c->cccds[index]&1u)) {previous=job;job=job->next;continue;}
                uint16_t left=(uint16_t)(job->result.size-job->offset),chunk=(uint16_t)(c->mtu-3u);
                if(chunk>left)chunk=left;
                if(!discard) {
                    attempted=true;
                    dbt_result_t rc=dbt_att_notify(&server,l,0x24,job->result.data+job->offset,chunk,false);
                    if(rc==DBT_OK) {sent=true;job->offset=(uint16_t)(job->offset+chunk);discard=job->offset==job->result.size;
                        if(discard && job->ota_epoch && job->data[0]==0x22 && job->result.data[0]==0x22) {
                            ota_notice.handle=job->handle;ota_notice.generation=job->generation;
                            ota_notice.epoch=job->ota_epoch;ota_notice.deadline=control_now_ms+30000u;
                        }
                    }
                    else if(rc!=DBT_BUSY && rc!=DBT_NO_SPACE && rc!=DBT_UNSUPPORTED)discard=true;
                }
            }
        }
        if(discard) {
            if(ota_complete && !sent)probe_gatt_restart_cancel();
            else if(job->ota_epoch && !sent){dbt_ble_control_peer_t peer={job->handle,job->generation};
                ota_hooks.abandon(ota_hooks.context,&peer,job->ota_epoch,job->data[0]);}
            control_job_t *next=job->next;
            if(previous)previous->next=next;else control_head=next;
            if(control_tail==job)control_tail=previous;
            control_free(job);job=next;
        } else {previous=job;job=job->next;}
        if(attempted)break;
    }
    bool any=false;
    for(unsigned i=0;i<DBT_MAX_LINKS;i++) if(bearer.runtime->acl.links[i].used && bearer.runtime->acl.links[i].kind==DBT_LINK_LE) any=true;
    if(!any && !control_jobs && !external_worker_required && control_worker && !control_retiring) {
        job=0;if(xQueueSend(control_work,&job,0)==pdTRUE){control_retiring=true;(void)xSemaphoreGive(control_signal);}
    }
    if(!any && !control_jobs && eq_worker && !eq_retiring) {
        job=0;if(xQueueSend(eq_work,&job,0)==pdTRUE){eq_retiring=true;(void)xSemaphoreGive(eq_signal);}
    }
}
void probe_gatt_meter(uint8_t a,uint8_t b,uint8_t c) {
    if(!bearer.runtime || ota_fence.epoch)return;
    meter_value[0]=a;meter_value[1]=b;meter_value[2]=c;
    for(unsigned i=0;i<DBT_MAX_LINKS;i++) {
        const dbt_link_t *l=&bearer.runtime->acl.links[i];
        if(l->used && l->kind==DBT_LINK_LE && cccd_delivery_ready(l)) (void)dbt_att_notify(&server,l,0x27,meter_value,3,false);
    }
}
static void sync_cccd(void) {
    if(!security_query || !bond_query)return;
    for(unsigned i=0;i<DBT_MAX_LINKS;i++) {
        const dbt_link_t *l=&bearer.runtime->acl.links[i];cccd_binding_t *b=&cccd_bindings[i];
        if(!l->used || l->kind!=DBT_LINK_LE) {memset(b,0,sizeof(*b));continue;}
        dbt_att_connection_t *c=control_connection(l);if(!c)continue;
        dbt_smp_session_security_t sec={0};uint8_t type,id[6],tag[32];
        bool active=security_query(l->handle,l->generation,&sec)==DBT_OK && sec.encrypted && sec.bond_committed &&
            bond_query(l->handle,l->generation,&type,id,tag)==DBT_OK;
        bool same=active && b->generation==l->generation && b->handle==l->handle && b->type==type &&
            !memcmp(b->identity,id,6) && !memcmp(b->tag,tag,32);
        if(!same) {
            if(b->handle==l->handle && b->generation==l->generation && b->pending_token)
                (void)cccd_complete_pending(l,b,c,DBT_ATT_UNLIKELY);
            if(c->bonded) {
                if(l->encrypted){(void)dbt_gap_disconnect(gap,l,0x13);continue;}
                if(dbt_att_security(&server,l,false,0,false)!=DBT_OK)continue;
            }
            memset(b,0,sizeof(*b));
            if(!active)continue;
            b->handle=l->handle;b->generation=l->generation;b->type=type;
            memcpy(b->identity,id,6);memcpy(b->tag,tag,32);
        }
        if(b->pending_token && (c->failed || !c->deferred || c->token!=b->pending_token ||
           (int32_t)(control_now_ms-c->request_deadline)>=0))
            (void)cccd_complete_pending(l,b,c,DBT_ATT_UNLIKELY);
        if(c->failed)continue;
        if(b->phase==CCCD_ERROR && b->pending_token)
            (void)cccd_complete_pending(l,b,c,DBT_ATT_UNLIKELY);
        if(b->phase==CCCD_NONE) {
            if(b->retry_pending && (int32_t)(control_now_ms-b->retry_at)<0)continue;
            b->retry_pending=false;
            if(!cccd_enqueue(l,b,1,0,0,NULL))b->phase=CCCD_LOADING;
            else {b->retry_pending=true;b->retry_at=control_now_ms+100u;}
        } else if(b->phase==CCCD_MERGE) {
            if(b->retry_pending && (int32_t)(control_now_ms-b->retry_at)<0)continue;
            b->retry_pending=false;
            if(!cccd_enqueue(l,b,3,0,0,b->values))b->phase=CCCD_SAVING;
            else {b->retry_pending=true;b->retry_at=control_now_ms+100u;}
        } else if(b->phase==CCCD_READY) {
            if(b->pending_token && !cccd_complete_pending(l,b,c,0))continue;
            dbt_result_t rc=dbt_att_security(&server,l,sec.authenticated,sec.key_size,true);
            if(rc==DBT_OK) {
                b->phase=CCCD_BOUND;
                cccd_record(l->handle,l->generation,0,6,b->phase,DBT_OK,0);
                cccd_touched_t *t=touch_for(l);if(t)t->mask=0;
                if(b->values[2]&1u)status_pending_generation[i]=l->generation;
            } else if(rc!=DBT_BUSY)b->phase=CCCD_ERROR;
        }
    }
}
dbt_result_t probe_gatt_retry_eq_publication(void) {
    if(!bearer.runtime)return DBT_INVALID;
    if(!eq_publication_pending)return DBT_EMPTY;
    if(eq_running)return DBT_BUSY;
    if(!control_hooks.publish_eq || !control_hooks.publish_eq(control_hooks.context,
        control_display_state.eq[0],control_display_state.eq[1],control_display_state.eq[2])) return DBT_BUSY;
    eq_publication_pending=false;return DBT_OK;
}
dbt_result_t probe_gatt_retry_control_publication(void) {
    if(!bearer.runtime)return DBT_INVALID;
    if(!control_publication_pending)return DBT_EMPTY;
    if(control_writes_running)return DBT_BUSY;
    if(!control_hooks.publish_control || !control_hooks.publish_control(control_hooks.context,
        control_display_state.control)) return DBT_BUSY;
    control_publication_pending=false;return DBT_OK;
}
void probe_gatt_poll(uint32_t now_ms) {
    if (!bearer.runtime)
        return;
    control_now_ms=now_ms;pump_control();sync_cccd();guard_pending_acks();
    dbt_att_bearer_poll(&bearer, now_ms);
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i) {
        const dbt_link_t *link = &bearer.runtime->acl.links[i];
        if (!link->used || link->kind != DBT_LINK_LE)
            continue;
        if(!cccd_delivery_ready(link))continue;
        if (notified_generation[i] != link->generation) {
            notified_generation[i] = link->generation;
            notified[i] = UINT32_MAX;
        }
        if (notified[i] == writes)
            continue;
        uint8_t bytes[8];
        counter(bytes);
        dbt_result_t result = dbt_att_notify(&server, link, 0x15, bytes, 8, false);
        if (result == DBT_OK || result == DBT_UNSUPPORTED)
            notified[i] = writes;
    }
}
uint32_t probe_gatt_wait_ms(uint32_t now_ms) {
    if(!bearer.runtime)return UINT32_MAX;
    if(uxQueueMessagesWaiting(control_done))return 0;
    uint32_t wait=dbt_att_bearer_wait_ms(&bearer, now_ms);
    if(!eq_worker && !eq_retiring && uxQueueMessagesWaiting(eq_work)){
        uint32_t retry=eq_retry_pending && (int32_t)(eq_retry_at-now_ms)>0?eq_retry_at-now_ms:0;
        if(retry<wait)wait=retry;
    }
    if(!control_worker && !control_retiring && (uxQueueMessagesWaiting(control_work) || external_worker_required)){
        uint32_t retry=control_retry_pending && (int32_t)(control_retry_at-now_ms)>0?control_retry_at-now_ms:0;
        if(retry<wait)wait=retry;
    }
    if((control_head || control_jobs) && wait>20)wait=20;
    if((ota_finish_job || ota_notice.epoch) && wait>20)wait=20;
    for(unsigned i=0;i<DBT_MAX_LINKS;i++) {
        const cccd_binding_t *b=&cccd_bindings[i];
        if(b->generation && (b->phase==CCCD_NONE || b->phase==CCCD_MERGE)) {
            uint32_t retry=b->retry_pending && (int32_t)(b->retry_at-now_ms)>0?b->retry_at-now_ms:0;
            if(retry<wait)wait=retry;
        } else if(b->generation && b->phase==CCCD_READY && wait>20)wait=20;
    }
    return wait;
}
bool probe_gatt_connected(void) {
    if (!bearer.runtime)
        return false;
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i)
        if (bearer.runtime->acl.links[i].used && bearer.runtime->acl.links[i].kind == DBT_LINK_LE)
            return true;
    return false;
}
unsigned probe_gatt_bytes(void) {
    /* Mutable static storage only; FreeRTOS queues, jobs, stacks and echo are dynamic. */
    return sizeof(server)+sizeof(bearer)+sizeof(gap)+sizeof(echo)+sizeof(echo_size)+sizeof(writes)+
           sizeof(security_query)+sizeof(bond_query)+sizeof(control_trust_capture)+sizeof(control_trust_valid)+sizeof(cccd_bindings)+sizeof(cccd_touched)+sizeof(ota_hooks)+sizeof(ota_finish_job)+sizeof(ota_notice)+sizeof(ota_fence)+sizeof(guarded_acks)+
           sizeof(control_work)+sizeof(eq_work)+sizeof(control_done)+sizeof(control_owner)+sizeof(control_signal)+sizeof(eq_signal)+
           sizeof(control_hooks)+sizeof(control_display_state)+sizeof(desired_eq)+sizeof(control_head)+sizeof(control_tail)+
           sizeof(control_now_ms)+sizeof(control_retry_at)+sizeof(control_retry_pending)+sizeof(eq_retry_at)+sizeof(eq_retry_pending)+sizeof(control_jobs)+sizeof(cccd_jobs)+sizeof(eq_running)+sizeof(control_running)+sizeof(control_writes_running)+sizeof(desired_control)+sizeof(control_configured)+
           sizeof(control_worker)+sizeof(control_retiring)+sizeof(eq_worker)+sizeof(eq_retiring)+
           sizeof(eq_publication_pending)+sizeof(control_publication_pending)+sizeof(external_worker_required)+
           sizeof(control_registry)+sizeof(status_pending_generation)+
           sizeof(notified)+sizeof(notified_generation)+sizeof(meter_value);
}

bool probe_gatt_restart_fenced(void){return ota_fence.epoch!=0;}
void probe_gatt_restart_cancel(void){
    if(!ota_fence.epoch)return;
    dbt_ble_control_peer_t peer={ota_fence.handle,ota_fence.generation};
    ota_hooks.abandon(ota_hooks.context,&peer,ota_fence.epoch,0x22);
    memset(&ota_fence,0,sizeof(ota_fence));
}
bool probe_gatt_restart_prepare_quiescent(void){
    control_job_t *job=ota_finish_job;
    if(!bearer.runtime || !ota_fence.epoch || !job || ota_notice.epoch || control_jobs!=1 || control_head ||
       uxQueueMessagesWaiting(control_work) || uxQueueMessagesWaiting(eq_work) ||
       uxQueueMessagesWaiting(control_done) || atomic_load_explicit(&external_worker_required,memory_order_acquire))return false;
    for(unsigned i=0;i<DBT_MAX_LINKS;i++){
        const dbt_att_connection_t *c=&server.connections[i];
        if(status_pending_generation[i])return false;
        if(bearer.bindings[i].active)return false;
        if(!c->used)continue;
        bool held=job->request && c->handle==job->handle && c->generation==job->generation &&
                  c->request_busy && c->deferred && c->token==job->token;
        if(c->response_ready || c->notice_ready || (c->request_busy && !held) || (c->deferred && !held))return false;
    }
    return true;
}
bool probe_gatt_restart_quiescent(void){
    if(!bearer.runtime || !ota_fence.epoch || ota_finish_job || ota_notice.epoch || control_jobs ||
       control_head || uxQueueMessagesWaiting(control_work) || uxQueueMessagesWaiting(eq_work) ||
       uxQueueMessagesWaiting(control_done) || atomic_load_explicit(&external_worker_required,memory_order_acquire))return false;
    for(unsigned i=0;i<DBT_MAX_LINKS;i++) {
        const dbt_att_connection_t *c=&server.connections[i];
        if(status_pending_generation[i])return false;
        if((c->used && (c->request_busy || c->response_ready || c->notice_ready || c->deferred)) || bearer.bindings[i].active)return false;
    }
    return true;
}

dbt_result_t probe_gatt_control_state(dbt_ble_control_state_t *out){
    if(!out || !control_configured)return DBT_INVALID;
    *out=control_display_state;return DBT_OK;
}
dbt_result_t probe_gatt_control_notify(const dbt_ble_control_peer_t *peer,const uint8_t *data,uint8_t size){
    if(!peer || !data || !size || size>DBT_BLE_CONTROL_MAX_STATUS || !bearer.runtime)return DBT_INVALID;
    const dbt_link_t *link=dbt_acl_find(&bearer.runtime->acl,peer->handle);
    if(!link || link->generation!=peer->generation || link->kind!=DBT_LINK_LE)return DBT_UNSUPPORTED;
    if(control_jobs-cccd_jobs>=CONTROL_JOBS-1u)return DBT_BUSY;
    control_job_t *job=heap_caps_malloc(sizeof(*job)+1u,MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
    if(!job)return DBT_NO_SPACE;
    memset(job,0,sizeof(*job));atomic_init(&job->cancelled,false);job->size=1;job->data[0]=0xfe;
    job->handle=peer->handle;job->generation=peer->generation;job->result.size=size;memcpy(job->result.data,data,size);
    for(unsigned i=0;i<CONTROL_REGISTRY_CAPACITY;i++)if(!control_registry[i]){control_registry[i]=job;break;}
    ++control_jobs;control_append(job);return DBT_OK;
}

static void product_status_changed(void){
    if(!bearer.runtime)return;
    for(unsigned i=0;i<DBT_MAX_LINKS;i++){
        const dbt_link_t *l=&bearer.runtime->acl.links[i];
        if(l->used && l->kind==DBT_LINK_LE)status_pending_generation[i]=l->generation;
    }
}
dbt_result_t probe_gatt_control_led_applied(const uint8_t values[10]){
    if(!control_configured || !values || values[0]>=22 || values[1]>100 || values[9]>4)return DBT_INVALID;
    if(memcmp(control_display_state.led,values,10)){memcpy(control_display_state.led,values,10);product_status_changed();}
    return DBT_OK;
}
dbt_result_t probe_gatt_control_sound_applied(uint8_t flags){
    if(!control_configured || (flags&0x70u))return DBT_INVALID;
    if(control_display_state.sound!=flags){control_display_state.sound=flags;product_status_changed();}
    return DBT_OK;
}

dbt_result_t probe_gatt_control_name_applied(const uint8_t *name,uint8_t size){
    if(!control_configured || !name || !size || size>32)return DBT_INVALID;
    if(!dbt_utf8_valid(name,size))return DBT_INVALID;
    memcpy(control_display_state.name,name,size);control_display_state.name_size=size;product_status_changed();return DBT_OK;
}

void probe_gatt_control_wake(void){
    /* Persistent semaphore outlives lazy workers, avoiding stale task handles. */
    if(control_signal)(void)xSemaphoreGive(control_signal);
}

dbt_result_t probe_gatt_control_worker_required(bool required){
    if(!bearer.runtime)return DBT_INVALID;
    if(required && !control_hooks.worker_poll)return DBT_UNSUPPORTED;
    if(external_worker_required!=required){external_worker_required=required;probe_gatt_control_wake();}
    return DBT_OK;
}
