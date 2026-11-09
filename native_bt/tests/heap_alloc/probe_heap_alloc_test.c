#include "probe_heap_alloc_trace.h"
#include "esp_heap_caps.h"
#include <assert.h>
#include <stdatomic.h>
#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>
#include <windows.h>
static _Atomic uint32_t now_us=100,free_bytes=30000,minimum_bytes=20000;
static _Atomic bool held;
static HANDLE entered,released;
static _Thread_local uintptr_t task_id=0x11;
static _Thread_local unsigned current_core=1;
static _Thread_local bool isr,in_hook;
int64_t esp_timer_get_time(void){return atomic_fetch_add(&now_us,1u);}
TaskHandle_t xTaskGetCurrentTaskHandle(void){assert(!isr);return (TaskHandle_t)task_id;}
BaseType_t xPortGetCoreID(void){return (BaseType_t)current_core;}
BaseType_t xPortInIsrContext(void){return isr;}
unsigned uxTaskGetStackHighWaterMark(TaskHandle_t task){assert(!in_hook && !task);return 1234;}
size_t heap_caps_get_free_size(uint32_t caps){assert(!in_hook && caps==(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT));return atomic_load(&free_bytes);}
size_t heap_caps_get_minimum_free_size(uint32_t caps){assert(!in_hook && caps==(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT));return atomic_load(&minimum_bytes);}
void probe_heap_trace_test_step(unsigned index){
    (void)index;
    if(atomic_load(&held)){SetEvent(entered);assert(WaitForSingleObject(released,3000)==WAIT_OBJECT_0);}
}
static void alloc(uintptr_t ptr,size_t size){in_hook=true;esp_heap_trace_alloc_hook((void *)ptr,size,1);in_hook=false;}
static void drop(uintptr_t ptr){in_hook=true;esp_heap_trace_free_hook((void *)ptr);in_hook=false;}
static void stop(void){probe_heap_alloc_link_removed(1,2);atomic_fetch_add(&now_us,2000001u);probe_heap_alloc_poll();}
static bool probe_output_running(void){return false;}
#define PROBE_SNAPSHOT_LOG(...) printf(__VA_ARGS__)
#include "actual_heap_logger.inc"
static void basic(void){
    assert(!probe_heap_alloc_begin(1).valid);alloc(0x1000,4096);
    assert(probe_heap_alloc_arm(1,2,(TaskHandle_t)0x11));assert(!probe_heap_alloc_arm(1,2,(TaskHandle_t)0x11));
    alloc(0x9000,256);probe_heap_alloc_owner_phase(PROBE_HEAP_CREATE_PERSIST);
    alloc(0x1000,144);alloc(0x2000,4096);probe_heap_alloc_owner_phase(0);drop(0x1000);drop(0x2000);
    alloc(0x2000,4096);drop(0x2000);
    task_id=0x22;current_core=0;probe_heap_alloc_task(PROBE_HEAP_PERSIST,true);
    probe_heap_mark_t m=probe_heap_alloc_begin(1);alloc(0x3000,4096);drop(0x3000);
    atomic_store(&free_bytes,24000);atomic_store(&minimum_bytes,17000);probe_heap_alloc_end(m);
    probe_heap_alloc_task(PROBE_HEAP_PERSIST,false);
    probe_heap_alloc_link_removed(1,3);atomic_fetch_add(&now_us,2000001u);probe_heap_alloc_poll();
    probe_heap_capture_t s;assert(!probe_heap_alloc_snapshot(&s));stop();assert(probe_heap_alloc_snapshot(&s));
    assert(s.count==10 && !s.overflow && !s.collision && s.reason==1 && s.bytes==1536);
    probe_heap_row_t r;assert(probe_heap_alloc_row(0,&r) && r.kind==1 && r.b==144 && (r.flags&255u)==PROBE_HEAP_CREATE_PERSIST);
    assert(probe_heap_alloc_row(2,&r) && r.kind==3 && r.a==0x1000);
    assert(probe_heap_alloc_row(4,&r) && r.kind==1 && r.a==0x2000);
    assert(probe_heap_alloc_row(6,&r) && r.kind==1 && (r.flags&255u)==0 && r.actor==0x22);
    assert(probe_heap_alloc_row(8,&r) && r.kind==2 && r.a==30000 && r.b==20000 && r.c==24000 && r.actor==17000);
    assert(probe_heap_alloc_row(9,&r) && r.kind==4 && r.a==0x22 && r.b==PROBE_HEAP_PERSIST && r.c==24000 && r.actor==17000 && r.end_us==1234);
    assert(!probe_heap_alloc_row(10,&r));log_heap_alloc_capture();
}

static void overflow_test(void){
    assert(probe_heap_alloc_arm(1,2,(TaskHandle_t)0x11));
    isr=true;alloc(0x90000,2048);isr=false;
    for(unsigned i=0;i<48;i++)alloc(0x1000u+i*0x100u,4096);
    probe_heap_capture_t s;assert(!probe_heap_alloc_snapshot(&s));probe_heap_alloc_poll();
    assert(probe_heap_alloc_snapshot(&s) && s.count==48 && s.overflow && s.reason==3);
    probe_heap_row_t r;assert(probe_heap_alloc_row(0,&r) && r.actor==0 && (r.flags&512u));
    assert(!probe_heap_alloc_arm(1,2,(TaskHandle_t)0x11));
}
static DWORD WINAPI blocked_writer(void *arg){
    (void)arg;current_core=0;task_id=0x22;probe_heap_alloc_task(PROBE_HEAP_SMP,true);alloc(0x1000,12288);return 0;
}
static void closing_writer(void){
    assert(probe_heap_alloc_arm(1,2,(TaskHandle_t)0x11));
    entered=CreateEventA(NULL,TRUE,FALSE,NULL);released=CreateEventA(NULL,TRUE,FALSE,NULL);assert(entered&&released);
    atomic_store(&held,true);HANDLE t=CreateThread(NULL,0,blocked_writer,NULL,0,NULL);assert(t);
    assert(WaitForSingleObject(entered,3000)==WAIT_OBJECT_0);stop();probe_heap_capture_t s;
    assert(!probe_heap_alloc_snapshot(&s));atomic_store(&held,false);SetEvent(released);
    assert(WaitForSingleObject(t,3000)==WAIT_OBJECT_0);assert(probe_heap_alloc_snapshot(&s) && s.count==1);
    probe_heap_row_t r;assert(probe_heap_alloc_row(0,&r) && r.a==0x1000 && r.b==12288 && r.kind==1);
    drop(0x1000);assert(probe_heap_alloc_row(0,&r) && r.kind==1 && r.a==0x1000);
    CloseHandle(t);CloseHandle(entered);CloseHandle(released);
}
static DWORD WINAPI parallel_writer(void *arg){
    current_core=(unsigned)(uintptr_t)arg;task_id=0x20u+current_core;
    for(unsigned i=0;i<10;i++){uintptr_t p=0x1000u+current_core*0x10000u+i*0x100u;alloc(p,4096);drop(p);}return 0;
}
static void parallel_test(void){
    assert(probe_heap_alloc_arm(1,2,(TaskHandle_t)0x11));
    HANDLE a=CreateThread(NULL,0,parallel_writer,(void *)0,0,NULL),b=CreateThread(NULL,0,parallel_writer,(void *)1,0,NULL);assert(a&&b);
    assert(WaitForSingleObject(a,3000)==WAIT_OBJECT_0&&WaitForSingleObject(b,3000)==WAIT_OBJECT_0);stop();
    probe_heap_capture_t s;assert(probe_heap_alloc_snapshot(&s) && s.count>0 && s.count<=40 && !s.overflow);
    if(s.count<40)assert(s.collision);
    for(unsigned i=0;i<s.count;i++){probe_heap_row_t r;assert(probe_heap_alloc_row(i,&r) && (r.kind==1 || r.kind==3));if(r.kind==1)assert(r.b==4096);else{bool seen=false;for(unsigned j=0;j<i;j++){probe_heap_row_t a;assert(probe_heap_alloc_row(j,&a));if(a.kind==1 && a.a==r.a)seen=true;}assert(seen);}}
    CloseHandle(a);CloseHandle(b);
}
static void delayed_free_callback(void){
    assert(probe_heap_alloc_arm(1,2,(TaskHandle_t)0x11));
    /* The first block was physically released before the second allocation,
     * but its SDK callback was delayed on the other core. Keep observations,
     * never invent a generation pairing from the address alone. */
    alloc(0x2000,4096);alloc(0x2000,4096);drop(0x2000);drop(0x2000);stop();
    probe_heap_capture_t s;assert(probe_heap_alloc_snapshot(&s) && s.count==4);
    for(unsigned i=0;i<4;i++){probe_heap_row_t r;assert(probe_heap_alloc_row(i,&r) && r.a==0x2000 && r.kind==(i<2?1u:3u));}
}
int main(int argc,char **argv){assert(argc==2);switch(atoi(argv[1])){case 0:basic();break;case 1:overflow_test();break;case 2:closing_writer();break;case 3:parallel_test();break;case 4:delayed_free_callback();break;default:abort();}puts("PASS bounded allocation sampler");return 0;}
