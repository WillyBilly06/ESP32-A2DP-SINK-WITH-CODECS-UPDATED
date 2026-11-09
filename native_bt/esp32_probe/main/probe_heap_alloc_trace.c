#include "probe_heap_alloc_trace.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include <stdatomic.h>
#include <stddef.h>
#ifndef DBT_PROBE_HEAP_ALLOC_TRACE
#error Allocation trace module is opt-in only
#endif
#ifndef CONFIG_HEAP_USE_HOOKS
#error Public SDK allocation hooks must be enabled
#endif
_Static_assert(ATOMIC_INT_LOCK_FREE==2 && ATOMIC_LONG_LOCK_FREE==2,"Native word atomics required");
#ifdef ESP_PLATFORM
_Static_assert(sizeof(void *)==4,"Original ESP32 diagnostic");
#endif
#ifdef PROBE_HEAP_ALLOC_TEST
void probe_heap_trace_test_step(unsigned);
#define PROBE_HEAP_TRACE_TEST_STEP(index) probe_heap_trace_test_step(index)
#else
#define PROBE_HEAP_TRACE_TEST_STEP(index) ((void)(index))
#endif
enum {IDLE,ARMED,CLOSING,FROZEN};
enum {ROW_ALLOC=1,ROW_HEAP,ROW_FREE,ROW_RETIRE};
enum {END_DISCONNECTED=1,END_TIMEOUT,END_OVERFLOW};
#define ADMISSION_OPEN UINT32_C(0x80000000)
#define ADMISSION_COUNT UINT32_C(0x7fffffff)
/* Static zero storage, one capture and no slot reuse: unused fields stay zero. */
typedef struct {_Atomic uint32_t ready;uint32_t a,b,c,begin_us,end_us,actor,flags;} heap_row_t;
static DRAM_ATTR heap_row_t alloc_trace_rows[PROBE_HEAP_ALLOC_ROWS];
static DRAM_ATTR _Atomic uint32_t alloc_trace_cursor,alloc_trace_phase,alloc_trace_overflow,alloc_trace_collision;
static DRAM_ATTR _Atomic uint32_t alloc_trace_tasks[5],alloc_trace_owner_phase,alloc_trace_stop_pending,alloc_trace_stop_at;
static DRAM_ATTR uint32_t alloc_trace_began_us,alloc_trace_stopped_us,alloc_trace_frozen_us,alloc_trace_reason,alloc_trace_handle,alloc_trace_generation;
static uint32_t IRAM_ATTR heap_clock_us(void){return (uint32_t)esp_timer_get_time();}
/* One CAS attempt reserves an immutable slot. The same word closes admission,
 * so snapshot checks reserved slots once and never waits for another task.
 * Failed admission is observational loss, not protocol backpressure. */
static heap_row_t *IRAM_ATTR heap_reserve(unsigned *index){
    uint32_t word=atomic_load(&alloc_trace_cursor);if(!(word&ADMISSION_OPEN))return NULL;
    uint32_t n=word&ADMISSION_COUNT;
    if(n>=PROBE_HEAP_ALLOC_ROWS){atomic_store(&alloc_trace_overflow,1u);return NULL;}
    if(!atomic_compare_exchange_strong(&alloc_trace_cursor,&word,word+1u)){atomic_store(&alloc_trace_collision,1u);return NULL;}
    *index=(unsigned)n;return &alloc_trace_rows[n];
}
static uint32_t IRAM_ATTR heap_actor(unsigned *role){
    *role=0;uint32_t id=xPortInIsrContext()?0u:(uint32_t)(uintptr_t)xTaskGetCurrentTaskHandle();
    /* The armed owner is non-null. Other task IDs are resolved by retirement
     * markers/snapshot, avoiding role-table scans in an allocation callback. */
    if(atomic_load(&alloc_trace_tasks[PROBE_HEAP_OWNER])==id)*role=atomic_load(&alloc_trace_owner_phase);
    return id;
}
void esp_heap_trace_alloc_hook(void *ptr,size_t size,uint32_t caps){
    if(!ptr || !(atomic_load(&alloc_trace_cursor)&ADMISSION_OPEN) || (size<2048u && !atomic_load(&alloc_trace_owner_phase)))return;
    unsigned role;uint32_t id=heap_actor(&role);
    if(size<2048u && !(role>=PROBE_HEAP_CREATE_SMP && role<=PROBE_HEAP_CREATE_GENERAL))return;
    unsigned index;heap_row_t *r=heap_reserve(&index);if(!r)return;
    r->a=(uint32_t)(uintptr_t)ptr;r->b=(uint32_t)size;r->c=caps;r->begin_us=heap_clock_us();
    r->actor=id;r->flags=role|((unsigned)xPortGetCoreID()<<8)|(id?0u:512u);
    PROBE_HEAP_TRACE_TEST_STEP(index);atomic_store(&r->ready,ROW_ALLOC);
}
/* SDK callbacks follow the allocator operation. Reuse can reorder callbacks:
 * keep free observations independent, with no invented generation pairing. */
void esp_heap_trace_free_hook(void *ptr){
    uint32_t word=atomic_load(&alloc_trace_cursor);if(!ptr || !(word&ADMISSION_OPEN))return;
    uint32_t address=(uint32_t)(uintptr_t)ptr;unsigned count=word&ADMISSION_COUNT;
    /* Admission reserves at most PROBE_HEAP_ALLOC_ROWS slots. */
    bool sampled=false;
    for(unsigned i=0;i<count;i++)if(atomic_load(&alloc_trace_rows[i].ready)==ROW_ALLOC && alloc_trace_rows[i].a==address){sampled=true;break;}
    if(!sampled)return;
    unsigned index;heap_row_t *r=heap_reserve(&index);if(!r)return;
    r->a=address;r->begin_us=heap_clock_us();r->flags=(unsigned)xPortGetCoreID()<<8;
    atomic_store(&r->ready,ROW_FREE);
}
bool probe_heap_alloc_arm(uint16_t handle,uint32_t generation,TaskHandle_t owner){
    if(!generation || !owner || atomic_load(&alloc_trace_phase)!=IDLE)return false;
    alloc_trace_handle=handle;alloc_trace_generation=generation;alloc_trace_began_us=heap_clock_us();
    atomic_store(&alloc_trace_tasks[PROBE_HEAP_OWNER],(uint32_t)(uintptr_t)owner);
    atomic_store(&alloc_trace_phase,ARMED);atomic_store(&alloc_trace_cursor,ADMISSION_OPEN);return true;
}
void probe_heap_alloc_task(unsigned role,bool active){
    if(role<PROBE_HEAP_SMP || role>PROBE_HEAP_GENERAL)return;
    uint32_t id=(uint32_t)(uintptr_t)xTaskGetCurrentTaskHandle();
    if(active){atomic_store(&alloc_trace_tasks[role],id);return;}
    uint32_t previous=id;(void)atomic_compare_exchange_strong(&alloc_trace_tasks[role],&previous,0u);
    if(atomic_load(&alloc_trace_phase)!=ARMED)return;
    uint32_t f=(uint32_t)heap_caps_get_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
    uint32_t m=(uint32_t)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
    uint32_t stack_low=(uint32_t)uxTaskGetStackHighWaterMark(NULL);
    unsigned index;heap_row_t *r=heap_reserve(&index);if(!r)return;
    r->a=id;r->b=role;r->c=f;r->begin_us=heap_clock_us();r->end_us=stack_low;r->actor=m;r->flags=(unsigned)xPortGetCoreID()<<8;
    atomic_store(&r->ready,ROW_RETIRE);
}
void probe_heap_alloc_owner_phase(unsigned value){atomic_store(&alloc_trace_owner_phase,value);}
void probe_heap_alloc_link_removed(uint16_t handle,uint32_t generation){
    if(atomic_load(&alloc_trace_phase)!=ARMED || handle!=alloc_trace_handle || generation!=alloc_trace_generation)return;
    atomic_store(&alloc_trace_stop_at,heap_clock_us()+2000000u);atomic_store(&alloc_trace_stop_pending,1u);
}
probe_heap_mark_t probe_heap_alloc_begin(unsigned kind){
    probe_heap_mark_t m={0};if(!(atomic_load(&alloc_trace_cursor)&ADMISSION_OPEN))return m;
    m.free_bytes=(uint32_t)heap_caps_get_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
    m.minimum_bytes=(uint32_t)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
    m.at_us=heap_clock_us();m.kind=kind;m.valid=true;return m;
}
void probe_heap_alloc_end(probe_heap_mark_t m){
    if(!m.valid)return;
    uint32_t f=(uint32_t)heap_caps_get_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
    uint32_t low=(uint32_t)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
    unsigned index;heap_row_t *r=heap_reserve(&index);if(!r)return;
    r->a=m.free_bytes;r->b=m.minimum_bytes;r->c=f;r->begin_us=m.at_us;r->end_us=heap_clock_us();r->actor=low;r->flags=m.kind|((unsigned)xPortGetCoreID()<<8);
    atomic_store(&r->ready,ROW_HEAP);
}
void probe_heap_alloc_poll(void){
    if(atomic_load(&alloc_trace_phase)!=ARMED)return;
    uint32_t now=heap_clock_us(),reason=0;
    if(atomic_load(&alloc_trace_overflow))reason=END_OVERFLOW;
    else if(atomic_load(&alloc_trace_stop_pending) && (int32_t)(now-atomic_load(&alloc_trace_stop_at))>=0)reason=END_DISCONNECTED;
    else if((uint32_t)(now-alloc_trace_began_us)>=30000000u)reason=END_TIMEOUT;
    if(!reason)return;
    uint32_t word=atomic_load(&alloc_trace_cursor);
    if(!(word&ADMISSION_OPEN) || !atomic_compare_exchange_strong(&alloc_trace_cursor,&word,word&ADMISSION_COUNT))return;
    alloc_trace_stopped_us=now;alloc_trace_reason=reason;atomic_store(&alloc_trace_phase,CLOSING);
}
bool probe_heap_alloc_snapshot(probe_heap_capture_t *out){
    if(!out)return false;
    unsigned count=atomic_load(&alloc_trace_cursor)&ADMISSION_COUNT;
    if(atomic_load(&alloc_trace_phase)==CLOSING){
        for(unsigned i=0;i<count;i++)if(!atomic_load(&alloc_trace_rows[i].ready))return false;
        alloc_trace_frozen_us=heap_clock_us();atomic_store(&alloc_trace_phase,FROZEN);
    }
    if(atomic_load(&alloc_trace_phase)!=FROZEN)return false;
    *out=(probe_heap_capture_t){.count=count,.overflow=atomic_load(&alloc_trace_overflow),.collision=atomic_load(&alloc_trace_collision),
        .begin_us=alloc_trace_began_us,.stop_requested_us=alloc_trace_stopped_us,.frozen_us=alloc_trace_frozen_us,.reason=alloc_trace_reason,
        .handle=alloc_trace_handle,.generation=alloc_trace_generation,.bytes=sizeof(alloc_trace_rows),
        .owner_task_id=atomic_load(&alloc_trace_tasks[PROBE_HEAP_OWNER]),.smp_task_id=atomic_load(&alloc_trace_tasks[PROBE_HEAP_SMP]),
        .persist_task_id=atomic_load(&alloc_trace_tasks[PROBE_HEAP_PERSIST]),.general_task_id=atomic_load(&alloc_trace_tasks[PROBE_HEAP_GENERAL])};return true;
}
bool probe_heap_alloc_row(unsigned index,probe_heap_row_t *out){
    if(!out || atomic_load(&alloc_trace_phase)!=FROZEN || index>=(atomic_load(&alloc_trace_cursor)&ADMISSION_COUNT) || index>=PROBE_HEAP_ALLOC_ROWS)return false;
    const heap_row_t *r=&alloc_trace_rows[index];unsigned kind=atomic_load(&r->ready);if(!kind)return false;
    *out=(probe_heap_row_t){.kind=kind,.a=r->a,.b=r->b,.c=r->c,.begin_us=r->begin_us,.end_us=r->end_us,.actor=r->actor,.flags=r->flags};return true;
}
