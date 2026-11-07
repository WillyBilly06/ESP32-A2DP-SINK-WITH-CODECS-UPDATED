#include <stdio.h>
#include <string.h>
#include "../../esp32_probe/main/probe_scheduler.c"
#define CHECK(x) do{if(!(x)){printf("FAIL sampler line %u\n",(unsigned)__LINE__);return 1;}}while(0)
static esp_freertos_tick_cb_t hooks[2];
static int fail_core=-1;
static unsigned registrations,deregistrations;
static uint32_t clock_us,cycles;
static TaskHandle_t running;
static bool running_isr;
#ifdef DBT_PROBE_TASK_RUNTIME_STATS
static unsigned info_calls,info_errors;
void vTaskGetInfo(TaskHandle_t handle,TaskStatus_t *info,int stack,eTaskState state){
    if(!handle || stack!=pdFALSE || state!=eReady)++info_errors;
    unsigned n=(unsigned)(uintptr_t)handle;
    *info=(TaskStatus_t){handle,NULL,n==1?0u:n+100u,20u+n,UINT32_MAX-n,(int32_t)(n&1u)};
    ++info_calls;clock_us+=3;
}
#endif
static bool missing_controller,have_system_tasks;
static TaskHandle_t task(unsigned n){return (TaskHandle_t)(uintptr_t)n;}
uint32_t esp_cpu_get_cycle_count(void){uint32_t before=cycles;cycles+=16;return before;}
int64_t esp_timer_get_time(void){return clock_us;}
TaskHandle_t xTaskGetCurrentTaskHandle(void){return running;}
int xPortInIsrContext(void){return running_isr;}
TaskHandle_t xTaskGetHandle(const char *name){return !strcmp(name,"native_render")?task(3):!strcmp(name,"btController")&&!missing_controller?task(5):have_system_tasks&&!strcmp(name,"ipc0")?task(11):have_system_tasks&&!strcmp(name,"ipc1")?task(12):have_system_tasks&&!strcmp(name,"esp_timer")?task(13):0;}
TaskHandle_t xTaskGetIdleTaskHandleForCore(unsigned core){return task(6+core);}
esp_err_t esp_register_freertos_tick_hook_for_cpu(esp_freertos_tick_cb_t hook,unsigned core){
    ++registrations;if((int)core==fail_core)return ESP_ERR_NO_MEM;hooks[core]=hook;return ESP_OK;
}
void esp_deregister_freertos_tick_hook_for_cpu(esp_freertos_tick_cb_t hook,unsigned core){
    ++deregistrations;if(hooks[core]==hook)hooks[core]=0;
}
static void fixture(void){
    memset(samples,0,sizeof(samples));memset(intervals,0,sizeof(intervals));memset(tasks,0,sizeof(tasks));memset(idle_tasks,0,sizeof(idle_tasks));
    memset(hooks,0,sizeof(hooks));atomic_store(&enabled,false);atomic_store(&host_phase,PROBE_HOST_OTHER);attempted=false;known_task_mask=0;
    #ifdef DBT_PROBE_CODEC_AWARE_RENDER
    atomic_store(&renderer0,0);
    #endif
    #ifdef DBT_PROBE_TASK_RUNTIME_STATS
    info_calls=info_errors=0;
    #endif
    fail_core=-1;registrations=deregistrations=clock_us=cycles=0;missing_controller=have_system_tasks=running_isr=false;running=timer_task=0;ipc_tasks[0]=ipc_tasks[1]=0;
}
static void tick(unsigned core,unsigned identity,uint32_t at){running=task(identity);clock_us=at;hooks[core]();}
int main(void){
    probe_scheduler_snapshot_t a,b;
    fixture();probe_scheduler_snapshot(0,&a);CHECK(!a.coherent);
    running=task(1);CHECK(!probe_scheduler_host_context());
    CHECK(probe_scheduler_start(task(1),task(2),task(4))==ESP_OK && registrations==2 && hooks[0] && hooks[1]);
    CHECK(probe_scheduler_host_context());
    running_isr=true;CHECK(!probe_scheduler_host_context());running_isr=false;
    running=task(3);CHECK(!probe_scheduler_host_context());
    running=task(4);CHECK(!probe_scheduler_host_context());
    running=0;CHECK(!probe_scheduler_host_context());
    const unsigned identities[]={1,2,3,4,5,6,8};
    for(unsigned i=0;i<7;++i)tick(0,identities[i],1000u*(i+1u));
    tick(1,7,1000);tick(1,2,2000);
    probe_scheduler_snapshot(0,&a);probe_scheduler_snapshot(1,&b);
    CHECK(a.coherent && b.coherent && a.known_task_mask==31 && b.known_task_mask==31);
    for(unsigned i=0;i<7;++i)CHECK(a.counts[i]==1);
    CHECK(a.host_phases[PROBE_HOST_OTHER]==1);
    for(unsigned i=1;i<PROBE_HOST_PHASES;++i)CHECK(!a.host_phases[i]);
    CHECK(b.counts[PROBE_SAMPLE_IDLE]==1 && b.counts[PROBE_SAMPLE_DECODER]==1 && !b.counts[PROBE_SAMPLE_HOST]);
    CHECK(a.max_gap_us==1000 && a.max_callback_cycles==16);
    tick(0,2,19000);probe_scheduler_snapshot(0,&a);
    CHECK(a.max_gap_us==12000 && a.max_gap_begin_us==7000 && a.max_gap_end_us==19000);
    CHECK(a.max_gap_previous_role==PROBE_SAMPLE_OTHER && a.max_gap_current_role==PROBE_SAMPLE_DECODER);
    atomic_store(&samples[0].sequence,101);probe_scheduler_snapshot(0,&a);CHECK(!a.coherent);
    probe_scheduler_snapshot(2,&a);CHECK(!a.coherent);probe_scheduler_snapshot(0,0);
    CHECK(probe_scheduler_start(task(1),task(2),task(4))==ESP_ERR_INVALID_STATE);
    fixture();CHECK(probe_scheduler_start(task(1),task(2),task(4))==ESP_OK);
    tick(0,1,UINT32_MAX-499u);atomic_store(&samples[0].sequence,UINT32_MAX-1u);
    tick(0,2,500);CHECK(atomic_load(&samples[0].sequence)==0);
    tick(0,2,1500);probe_scheduler_snapshot(0,&a);
    CHECK(a.coherent && a.max_gap_us==1000 && a.max_gap_begin_us==UINT32_MAX-499u && a.max_gap_end_us==500);
    atomic_store(&samples[0].counts[PROBE_SAMPLE_DECODER],UINT32_MAX);tick(0,2,2500);
    probe_scheduler_snapshot(0,&a);CHECK(!a.counts[PROBE_SAMPLE_DECODER]);
    fixture();CHECK(probe_scheduler_start(task(1),task(2),task(4))==ESP_OK);
    unsigned previous=probe_scheduler_host_phase(PROBE_HOST_RUNTIME);CHECK(previous==PROBE_HOST_OTHER);
    tick(1,1,1000);
    unsigned nested=probe_scheduler_host_phase(PROBE_HOST_SUBMIT);CHECK(nested==PROBE_HOST_RUNTIME);
    tick(1,1,2000);CHECK(probe_scheduler_host_phase(nested)==PROBE_HOST_SUBMIT);
    tick(1,1,3000);CHECK(probe_scheduler_host_phase(previous)==PROBE_HOST_RUNTIME);
    tick(1,1,4000);probe_scheduler_host_phase(PROBE_HOST_GATT);tick(1,2,5000);
    CHECK(probe_scheduler_host_phase(UINT32_MAX)==PROBE_HOST_GATT);tick(1,1,6000);
    probe_scheduler_snapshot(1,&a);
    CHECK(a.coherent && a.host_phases[PROBE_HOST_RUNTIME]==2 && a.host_phases[PROBE_HOST_SUBMIT]==1 && a.host_phases[PROBE_HOST_OTHER]==2);
    CHECK(!a.host_phases[PROBE_HOST_GATT] && a.counts[PROBE_SAMPLE_DECODER]==1);
    uint32_t total=0;for(unsigned i=0;i<PROBE_HOST_PHASES;++i)total+=a.host_phases[i];CHECK(total==a.counts[PROBE_SAMPLE_HOST]);
    for(unsigned phase=0;phase<PROBE_HOST_PHASES;++phase){probe_scheduler_host_phase(phase);tick(0,1,1000u*(phase+1u));}
    probe_scheduler_snapshot(0,&b);for(unsigned phase=0;phase<PROBE_HOST_PHASES;++phase)CHECK(b.host_phases[phase]==1);
    /* Task-context interval attribution reuses the existing tick writer. */
    fixture();CHECK(probe_scheduler_start(task(1),task(2),task(4))==ESP_OK);
    probe_scheduler_mark_t begin,end;probe_scheduler_interval_t interval;
    probe_scheduler_interval_snapshot(PROBE_DELAY_LOCAL_RENDER,&interval);CHECK(!interval.coherent);
    probe_scheduler_mark(1,&begin);CHECK(begin.coherent && begin.core==1 && begin.read_cycles==16);
    tick(1,5,1000);probe_scheduler_host_phase(PROBE_HOST_QUERY);tick(1,1,2000);tick(0,2,2500);
    probe_scheduler_mark(1,&end);CHECK(end.coherent && end.read_cycles==16);
    probe_scheduler_interval_observe(PROBE_DELAY_LOCAL_RENDER,&begin,&end,100,4100,128,1,192000);
    probe_scheduler_interval_snapshot(PROBE_DELAY_LOCAL_RENDER,&interval);
    CHECK(interval.coherent && interval.markers_valid && interval.revision==2 && interval.wall_us==4000 && interval.frames==128);
    CHECK(interval.counts[PROBE_SAMPLE_CONTROLLER]==1 && interval.counts[PROBE_SAMPLE_HOST]==1 && !interval.counts[PROBE_SAMPLE_DECODER]);
    CHECK(interval.host_phases[PROBE_HOST_QUERY]==1 && interval.known_task_mask==31 && interval.end_tick_us==2000);
    probe_scheduler_interval_observe(PROBE_DELAY_LOCAL_RENDER,&begin,&end,100,200,64,1,192000);
    probe_scheduler_interval_snapshot(PROBE_DELAY_LOCAL_RENDER,&interval);CHECK(interval.revision==2 && interval.wall_us==4000 && interval.frames==128);
    begin.coherent=false;probe_scheduler_interval_observe(PROBE_DELAY_ASSIST_RETIRE,&begin,&end,0,5000,512,1,192000);
    probe_scheduler_interval_snapshot(PROBE_DELAY_ASSIST_RETIRE,&interval);CHECK(interval.coherent && !interval.markers_valid && !interval.counts[0]);
    begin.coherent=true;begin.core=0;probe_scheduler_interval_observe(PROBE_DELAY_ASSIST_RETIRE,&begin,&end,0,6000,512,1,192000);
    probe_scheduler_interval_snapshot(PROBE_DELAY_ASSIST_RETIRE,&interval);CHECK(!interval.markers_valid && !interval.counts[PROBE_SAMPLE_CONTROLLER]);
    begin.core=1;begin.known_task_mask=15;probe_scheduler_interval_observe(PROBE_DELAY_ASSIST_RETIRE,&begin,&end,0,7000,512,1,192000);
    probe_scheduler_interval_snapshot(PROBE_DELAY_ASSIST_RETIRE,&interval);CHECK(!interval.markers_valid);
    begin.known_task_mask=31;begin.counts[PROBE_SAMPLE_HOST]=UINT32_MAX-1u;end.counts[PROBE_SAMPLE_HOST]=1;
    begin.host_phases[PROBE_HOST_QUERY]=UINT32_MAX;end.host_phases[PROBE_HOST_QUERY]=1;
    atomic_store(&intervals[PROBE_DELAY_LOCAL_RENDER].wall_us,0);atomic_store(&intervals[PROBE_DELAY_LOCAL_RENDER].sequence,UINT32_MAX-1u);
    probe_scheduler_interval_observe(PROBE_DELAY_LOCAL_RENDER,&begin,&end,UINT32_MAX-99u,101,128,1,192000);
    probe_scheduler_interval_snapshot(PROBE_DELAY_LOCAL_RENDER,&interval);
    CHECK(interval.coherent && interval.revision==2 && interval.wall_us==201 && interval.markers_valid);
    CHECK(interval.counts[PROBE_SAMPLE_HOST]==3 && interval.host_phases[PROBE_HOST_QUERY]==2);
    atomic_store(&intervals[PROBE_DELAY_LOCAL_RENDER].sequence,3);probe_scheduler_interval_snapshot(PROBE_DELAY_LOCAL_RENDER,&interval);CHECK(!interval.coherent);
    atomic_store(&samples[1].sequence,3);probe_scheduler_mark(1,&begin);CHECK(!begin.coherent);
    probe_scheduler_mark(2,&begin);CHECK(!begin.coherent);probe_scheduler_mark(1,NULL);
    probe_scheduler_interval_snapshot(PROBE_DELAY_KINDS,&interval);CHECK(!interval.coherent);
    probe_scheduler_interval_observe(PROBE_DELAY_KINDS,&begin,&end,0,9999,1,1,192000);
    probe_scheduler_interval_observe(0,NULL,&end,0,9999,1,1,192000);probe_scheduler_interval_snapshot(0,NULL);
    for(int core=0;core<2;++core){
        fixture();fail_core=core;CHECK(probe_scheduler_start(task(1),task(2),task(4))==ESP_ERR_NO_MEM);
        CHECK(!hooks[0] && !hooks[1] && deregistrations==(unsigned)core);
        sample_tick0();sample_tick1();probe_scheduler_snapshot(0,&a);CHECK(!a.coherent && !atomic_load(&samples[0].sequence));
    }
    fixture();CHECK(probe_scheduler_start(0,task(2),task(4))==ESP_ERR_INVALID_ARG && !registrations);
    fixture();missing_controller=true;CHECK(probe_scheduler_start(task(1),task(2),task(4))==ESP_OK);
    tick(1,5,1000);probe_scheduler_snapshot(1,&a);CHECK(a.known_task_mask==15 && a.counts[PROBE_SAMPLE_OTHER]==1);
    fixture();have_system_tasks=true;CHECK(probe_scheduler_start(task(1),task(2),task(4))==ESP_OK);
    tick(0,11,1000);tick(1,12,1000);tick(0,13,2000);
    probe_scheduler_mark(0,&begin);CHECK(begin.coherent && begin.core==0 && begin.known_task_mask==415);
    tick(0,11,3000);probe_scheduler_mark(0,&end);
    probe_scheduler_interval_observe(PROBE_DELAY_LOCAL_RENDER0,&begin,&end,2100,3100,128,1,192000);
    probe_scheduler_interval_snapshot(PROBE_DELAY_LOCAL_RENDER0,&interval);
    CHECK(interval.coherent && interval.markers_valid && interval.counts[PROBE_SAMPLE_IPC]==1 && !interval.counts[PROBE_SAMPLE_TIMER]);
    probe_scheduler_interval_observe(PROBE_DELAY_LOCAL_RENDER0,&begin,&end,2100,2200,128,2,96000);
    probe_scheduler_interval_snapshot(PROBE_DELAY_LOCAL_RENDER0,&interval);
    CHECK(interval.coherent && interval.stream_serial==2 && interval.rate==96000 && interval.wall_us==100 && interval.revision==4);
    probe_scheduler_snapshot(0,&a);CHECK(a.counts[PROBE_SAMPLE_IPC]==2 && a.counts[PROBE_SAMPLE_TIMER]==1);
    probe_scheduler_snapshot(1,&b);CHECK(b.counts[PROBE_SAMPLE_IPC]==1);
#ifdef DBT_PROBE_TASK_RUNTIME_STATS
    probe_task_runtime_snapshot_t runtime;
    fixture();probe_scheduler_task_runtime(&runtime);CHECK(!runtime.present_mask && !info_calls);
    probe_scheduler_task_runtime(NULL);CHECK(!info_calls);
    have_system_tasks=true;CHECK(probe_scheduler_start(task(1),task(2),task(4))==ESP_OK);
    probe_scheduler_bind_renderer0(task(14));clock_us=UINT32_MAX-10u;
    probe_scheduler_task_runtime(&runtime);
    CHECK(runtime.present_mask==2047u && info_calls==11 && !info_errors);
    CHECK(runtime.max_query_us==3 && runtime.end_us-runtime.begin_us==33 && runtime.end_us<runtime.begin_us);
    CHECK(runtime.tasks[0].task_number==0 && runtime.tasks[0].counter_us==UINT32_MAX-1u && runtime.tasks[0].core==1);
    CHECK(runtime.tasks[10].task_number==114 && runtime.tasks[10].priority==34 && runtime.tasks[10].core==0);
    fixture();missing_controller=true;CHECK(probe_scheduler_start(task(1),task(2),task(4))==ESP_OK);
    probe_scheduler_task_runtime(&runtime);CHECK(info_calls==6 && !info_errors && runtime.present_mask==111u && !runtime.tasks[4].counter_us);
    puts("PASS runtime observer:11 permanent roles, raw counter/clock wrap, missing handles, no stack/state scans, no allocation or ISR collection");
#endif
    puts("PASS sampler: per-core identities, tick gaps, sequence/counter/time wrap, nonspinning snapshots, Core0/Core1 maximum intervals, IPC/timer attribution, invalid/mixed-core marks, counter/time/publication wrap, missing tasks and registration rollback");return 0;
}
