#include "probe_gatt.h"
#include "dbt_att_bearer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <string.h>
#include <stdatomic.h>

/* Diagnostic echo/counter and baseline product-control services. */
static dbt_att_server_t server;
static dbt_att_bearer_t bearer;
static dbt_gap_t *gap;
static uint8_t *echo;
static uint16_t echo_size;
static uint32_t writes, notified[DBT_MAX_LINKS], notified_generation[DBT_MAX_LINKS];
typedef struct control_job {
    struct control_job *next;
    uint32_t generation,token;
    uint16_t handle,size,offset;
    bool request,eq_lane;
    _Atomic bool cancelled;
    dbt_ble_control_result_t result;
    dbt_ble_control_state_t state;
    uint8_t data[];
} control_job_t;
#define CONTROL_JOBS 4u
#define CONTROL_STACK_BYTES 4096u
static QueueHandle_t control_work,eq_work,control_done;
static TaskHandle_t control_owner;
static dbt_ble_control_hooks_t control_hooks;
static dbt_ble_control_state_t control_display_state;
static control_job_t *control_head,*control_tail;
static control_job_t *control_registry[CONTROL_JOBS];
static unsigned control_jobs,eq_running,control_running;
static bool control_configured,control_worker,control_retiring,eq_worker,eq_retiring;
static bool eq_publication_pending,control_publication_pending;
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
    {3, DBT_ATT_READ, sizeof(name) - 1, sizeof(name) - 1, 0, 2, 0, name_uuid, name},
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
dbt_result_t probe_gatt_control_configure(const dbt_ble_control_state_t *initial,
                                          const dbt_ble_control_hooks_t *hooks) {
    if (bearer.runtime || !initial || !hooks || initial->name_size>32 || initial->firmware_size>32)
        return DBT_INVALID;
    control_hooks=*hooks; control_display_state=*initial;
    control_configured=true; return DBT_OK;
}
static void control_execute_job(control_job_t *job) {
    dbt_ble_control_peer_t peer={job->handle,job->generation};
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
    bool eq=(uintptr_t)unused!=0;QueueHandle_t queue=eq?eq_work:control_work;
    for (;;) {
        control_job_t *job=0;
        if(xQueueReceive(queue,&job,portMAX_DELAY)!=pdTRUE) continue;
        if(!job) { job=(control_job_t *)(uintptr_t)(eq?1u:2u);
            (void)xQueueSend(control_done,&job,portMAX_DELAY);xTaskNotifyGive(control_owner);vTaskDelete(NULL);return; }
        control_execute_job(job);
        (void)xQueueSend(control_done,&job,portMAX_DELAY);xTaskNotifyGive(control_owner);
    }
}
static void control_free(control_job_t *job) {
    if(!job)return;
    for(unsigned i=0;i<CONTROL_JOBS;i++)if(control_registry[i]==job)control_registry[i]=0;
    volatile uint8_t *p=(volatile uint8_t *)job;
    size_t n=sizeof(*job)+job->size;while(n--)*p++=0;
    heap_caps_free(job); if(control_jobs)--control_jobs;
}
static void control_append(control_job_t *job) {
    job->next=0;if(control_tail)control_tail->next=job;else control_head=job;control_tail=job;
}
static uint16_t control_submit(const dbt_link_t *link,const dbt_att_write_t *w,bool command,uint32_t token) {
    if(!link || !w || w->offset || !w->size || w->size>DBT_BLE_CONTROL_MAX_COMMAND) return DBT_ATT_INVALID_LENGTH;
    if(w->data[0]!=0xf0 && w->data[0]!=0xff &&
       (!control_hooks.authorize || !control_hooks.authorize(control_hooks.context,link,w->data[0])))
        return DBT_ATT_AUTHORIZATION;
    bool eq=w->data[0]==1 || w->data[0]==2;
    if(control_jobs>=CONTROL_JOBS || (!eq && control_jobs>=CONTROL_JOBS-1u)) return DBT_ATT_RESOURCES;
    control_job_t *job=heap_caps_malloc(sizeof(*job)+w->size,MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
    if(!job)return DBT_ATT_RESOURCES;
    memset(job,0,sizeof(*job));atomic_init(&job->cancelled,false);job->handle=link->handle;job->generation=link->generation;
    job->token=token;job->request=!command;job->size=w->size;memcpy(job->data,w->data,w->size);
    job->eq_lane=eq;job->state=control_display_state;
    ++control_jobs;
    if(xQueueSend(job->eq_lane?eq_work:control_work,&job,0)!=pdTRUE){control_free(job);return DBT_ATT_RESOURCES;}
    if(job->eq_lane)++eq_running;else ++control_running;
    for(unsigned i=0;i<CONTROL_JOBS;i++)if(!control_registry[i]){control_registry[i]=job;break;}
    return command?0:DBT_ATT_DEFERRED;
}
static uint8_t read(void *context, const dbt_link_t *link, const dbt_attribute_t *attribute,
                    uint16_t offset, uint8_t *out, uint16_t capacity, uint16_t *written,
                    uint16_t *total) {
    (void)context;
    (void)link;
    uint8_t status[DBT_BLE_CONTROL_MAX_STATUS];
    const uint8_t *source = echo;
    *total = echo_size;
    if(attribute->handle==0x24) {
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
    /* The ATT core expects this callback to durably save bonded CCCDs. No
     * provider is wired yet, so fail rather than claim persistent success. */
    if(batch->cccd_count) for(unsigned i=0;i<DBT_MAX_LINKS;i++) {
        const dbt_att_connection_t *c=&server.connections[i];
        if(c->used && c->handle==link->handle && c->generation==link->generation && c->bonded) {
            *error_handle=batch->cccds[0].handle;return DBT_ATT_UNLIKELY;
        }
    }
    for(unsigned i=0;i<batch->count;i++) if(batch->writes[i].handle==0x22) {
        *error_handle=0x22;
        if(batch->count!=1 || batch->cccd_count || batch->prepared)return DBT_ATT_NOT_SUPPORTED;
        return control_submit(link,&batch->writes[i],batch->command,token);
    }
    for(unsigned i=0;i<batch->cccd_count;i++) if(batch->cccds[i].handle==0x25) {
        for(unsigned j=0;j<DBT_MAX_LINKS;j++) if(bearer.runtime->acl.links[j].used &&
            bearer.runtime->acl.links[j].handle==link->handle && bearer.runtime->acl.links[j].generation==link->generation) {
            if(batch->cccds[i].value&1)status_pending_generation[j]=link->generation;
            else status_pending_generation[j]=0;
        }
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
    hooks.failed = failed;
    control_work=xQueueCreate(CONTROL_JOBS,sizeof(control_job_t *));
    eq_work=xQueueCreate(CONTROL_JOBS,sizeof(control_job_t *));
    control_done=xQueueCreate(CONTROL_JOBS,sizeof(control_job_t *));
    control_owner=xTaskGetCurrentTaskHandle();
    if(!control_work || !eq_work || !control_done || !control_owner) {
        if(control_work)vQueueDelete(control_work);
        if(eq_work)vQueueDelete(eq_work);
        if(control_done)vQueueDelete(control_done);
        control_work=eq_work=control_done=0;return DBT_NO_SPACE;
    }
    dbt_result_t result = dbt_att_init(
        &server, attributes, sizeof(attributes) / sizeof(attributes[0]), hooks, 517, 1024);
    if (result != DBT_OK) goto failed_init;
    gap = configuration;
    result=dbt_att_bearer_init(&bearer, runtime, &server);
    if(result==DBT_OK)return result;
    dbt_att_clear(&server);
failed_init:
    vQueueDelete(control_work);vQueueDelete(eq_work);vQueueDelete(control_done);control_work=eq_work=control_done=0;return result;
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
    if(change && change->kind==DBT_LINK_REMOVED) for(unsigned i=0;i<CONTROL_JOBS;i++) {
        control_job_t *job=control_registry[i];
        /* An accepted EQ change must still commit even if its originating
         * link vanishes; other unstarted jobs can be cancelled. */
        if(job && job->handle==change->handle && job->generation==change->generation &&
           job->data[0]!=1 && job->data[0]!=2)
            atomic_store_explicit(&job->cancelled,true,memory_order_release);
    }
    dbt_result_t result = dbt_att_bearer_link(&bearer, change);
    if (result != DBT_OK)
        dbt_host_abort(&bearer.runtime->host, DBT_FAULT_BAD_RESPONSE);
}
void probe_gatt_receive(void *context, const dbt_link_t *link, uint16_t cid, const uint8_t *data,
                        uint16_t size) {
    (void)context;
    dbt_result_t result = dbt_att_bearer_receive(&bearer, link, cid, data, size);
    if (result == DBT_INVALID || result == DBT_BUSY)
        (void)dbt_gap_disconnect(gap, link, 0x13);
}
static void finish_job(control_job_t *job) {
    if(job->eq_lane && eq_running)--eq_running;
    if(!job->eq_lane && control_running)--control_running;
    if(job->result.durable_eq) {
        memcpy(control_display_state.eq,job->state.eq,3);
        eq_publication_pending=job->result.data[0]!=0x10;
    }
    if(job->result.durable_control) {
        control_display_state.control=job->state.control;
        control_publication_pending=job->result.data[0]!=0x10;
    }
    if(job->size && job->data[0]==4 && job->result.size==2 && job->result.data[0]==0x10 && job->result.data[1]==4) {
        control_display_state.name_size=job->state.name_size;
        memcpy(control_display_state.name,job->state.name,job->state.name_size);
    }
    if(job->size && (job->data[0]==5 || job->data[0]==6 || job->data[0]==7) &&
       job->result.size==2 && job->result.data[0]==0x10 && job->result.data[1]==job->data[0]) {
        if(job->data[0]==5)memcpy(control_display_state.led,job->state.led,10);
        else if(job->data[0]==6)control_display_state.led[0]=job->state.led[0];
        else control_display_state.led[1]=job->state.led[1];
    }
    if(job->result.size==2 && job->result.data[0]==0x06) control_display_state.sound=job->state.sound;
    const dbt_link_t *link=dbt_acl_find(&bearer.runtime->acl,job->handle);
    if(!link || link->generation!=job->generation || link->kind!=DBT_LINK_LE) { control_free(job);return; }
    if(job->request) {
        uint8_t e=job->result.size && job->result.data[0]!=0x11 ? 0 : DBT_ATT_UNLIKELY;
        if(dbt_att_complete(&server,job->handle,job->generation,job->token,e,0x22)!=DBT_OK) {
            control_free(job);return;
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
static void pump_control(void) {
    control_job_t *job=0;
    while(xQueueReceive(control_done,&job,0)==pdTRUE) {
        if((uintptr_t)job==1u) {eq_worker=false;eq_retiring=false;continue;}
        if((uintptr_t)job==2u) {control_worker=false;control_retiring=false;continue;}
        finish_job(job);
    }
    if(!control_worker && !control_retiring && uxQueueMessagesWaiting(control_work)) {
        if(xTaskCreate(control_task,"ble_control_worker",CONTROL_STACK_BYTES,NULL,4,NULL)==pdPASS) control_worker=true;
        else while(xQueueReceive(control_work,&job,0)==pdTRUE) if(job) {
            job->state=control_display_state;job->result.data[0]=0x11;job->result.data[1]=job->data[0];
            job->result.data[2]=3;job->result.size=3;finish_job(job);
        }
    }
    if(!eq_worker && !eq_retiring && uxQueueMessagesWaiting(eq_work)) {
        if(xTaskCreate(control_task,"ble_eq_worker",CONTROL_STACK_BYTES,(void *)(uintptr_t)1,4,NULL)==pdPASS) eq_worker=true;
        else while(xQueueReceive(eq_work,&job,0)==pdTRUE) if(job) {
            job->result.data[0]=0x11;job->result.data[1]=job->data[0];job->result.data[2]=3;job->result.size=3;finish_job(job);
        }
    }
    for(unsigned i=0;i<DBT_MAX_LINKS;i++) {
        const dbt_link_t *l=&bearer.runtime->acl.links[i];
        if(!status_pending_generation[i])continue;
        if(!l->used || l->kind!=DBT_LINK_LE || l->generation!=status_pending_generation[i]) {
            status_pending_generation[i]=0;continue;
        }
        if(control_jobs>=CONTROL_JOBS-1u)continue;
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
        bool attempted=false;
        if(!discard) {
            dbt_att_connection_t *c=control_connection(l);
            if(!c || c->mtu<4) discard=true;
            else {
                int index=status_cccd_index();
                if(index<0 || (unsigned)index>=DBT_ATT_CCCD_LIMIT)discard=true;
                else if(!(c->cccds[index]&1u)) {previous=job;job=job->next;continue;}
                uint16_t left=(uint16_t)(job->result.size-job->offset),chunk=(uint16_t)(c->mtu-3u);
                if(chunk>left)chunk=left;
                if(!discard) {
                    attempted=true;
                    dbt_result_t rc=dbt_att_notify(&server,l,0x24,job->result.data+job->offset,chunk,false);
                    if(rc==DBT_OK) {job->offset=(uint16_t)(job->offset+chunk);discard=job->offset==job->result.size;}
                    else if(rc!=DBT_BUSY && rc!=DBT_NO_SPACE && rc!=DBT_UNSUPPORTED)discard=true;
                }
            }
        }
        if(discard) {
            control_job_t *next=job->next;
            if(previous)previous->next=next;else control_head=next;
            if(control_tail==job)control_tail=previous;
            control_free(job);job=next;
        } else {previous=job;job=job->next;}
        if(attempted)break;
    }
    bool any=false;
    for(unsigned i=0;i<DBT_MAX_LINKS;i++) if(bearer.runtime->acl.links[i].used && bearer.runtime->acl.links[i].kind==DBT_LINK_LE) any=true;
    if(!any && !control_jobs && control_worker && !control_retiring) {
        job=0;if(xQueueSend(control_work,&job,0)==pdTRUE)control_retiring=true;
    }
    if(!any && !control_jobs && eq_worker && !eq_retiring) {
        job=0;if(xQueueSend(eq_work,&job,0)==pdTRUE)eq_retiring=true;
    }
}
void probe_gatt_meter(uint8_t a,uint8_t b,uint8_t c) {
    if(!bearer.runtime)return;
    meter_value[0]=a;meter_value[1]=b;meter_value[2]=c;
    for(unsigned i=0;i<DBT_MAX_LINKS;i++) {
        const dbt_link_t *l=&bearer.runtime->acl.links[i];
        if(l->used && l->kind==DBT_LINK_LE) (void)dbt_att_notify(&server,l,0x27,meter_value,3,false);
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
    if(control_running)return DBT_BUSY;
    if(!control_hooks.publish_control || !control_hooks.publish_control(control_hooks.context,
        control_display_state.control)) return DBT_BUSY;
    control_publication_pending=false;return DBT_OK;
}
void probe_gatt_poll(uint32_t now_ms) {
    if (!bearer.runtime)
        return;
    pump_control();
    dbt_att_bearer_poll(&bearer, now_ms);
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i) {
        const dbt_link_t *link = &bearer.runtime->acl.links[i];
        if (!link->used || link->kind != DBT_LINK_LE)
            continue;
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
    if(uxQueueMessagesWaiting(control_done) || (!control_worker && !control_retiring && uxQueueMessagesWaiting(control_work)) ||
       (!eq_worker && !eq_retiring && uxQueueMessagesWaiting(eq_work)))return 0;
    uint32_t wait=dbt_att_bearer_wait_ms(&bearer, now_ms);
    if((control_head || control_jobs) && wait>20)wait=20;
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
           sizeof(control_work)+sizeof(eq_work)+sizeof(control_done)+sizeof(control_owner)+
           sizeof(control_hooks)+sizeof(control_display_state)+sizeof(control_head)+sizeof(control_tail)+
           sizeof(control_jobs)+sizeof(eq_running)+sizeof(control_running)+sizeof(control_configured)+
           sizeof(control_worker)+sizeof(control_retiring)+sizeof(eq_worker)+sizeof(eq_retiring)+
           sizeof(eq_publication_pending)+sizeof(control_publication_pending)+
           sizeof(control_registry)+sizeof(status_pending_generation)+
           sizeof(notified)+sizeof(notified_generation)+sizeof(meter_value);
}
