#include <stdio.h>
#include <string.h>
#include "../../esp32_probe/main/probe_scheduler.c"
#define CHECK(x) do{if(!(x)){printf("FAIL sampler line %u\n",(unsigned)__LINE__);return 1;}}while(0)
static esp_freertos_tick_cb_t hooks[2];
static int fail_core=-1;
static unsigned registrations,deregistrations;
static uint32_t clock_us,cycles;
static TaskHandle_t running;
static bool missing_controller;
static TaskHandle_t task(unsigned n){return (TaskHandle_t)(uintptr_t)n;}
uint32_t esp_cpu_get_cycle_count(void){uint32_t before=cycles;cycles+=16;return before;}
int64_t esp_timer_get_time(void){return clock_us;}
TaskHandle_t xTaskGetCurrentTaskHandle(void){return running;}
TaskHandle_t xTaskGetHandle(const char *name){return !strcmp(name,"native_render")?task(3):!strcmp(name,"btController")&&!missing_controller?task(5):0;}
TaskHandle_t xTaskGetIdleTaskHandleForCore(unsigned core){return task(6+core);}
esp_err_t esp_register_freertos_tick_hook_for_cpu(esp_freertos_tick_cb_t hook,unsigned core){
    ++registrations;if((int)core==fail_core)return ESP_ERR_NO_MEM;hooks[core]=hook;return ESP_OK;
}
void esp_deregister_freertos_tick_hook_for_cpu(esp_freertos_tick_cb_t hook,unsigned core){
    ++deregistrations;if(hooks[core]==hook)hooks[core]=0;
}
static void fixture(void){
    memset(samples,0,sizeof(samples));memset(tasks,0,sizeof(tasks));memset(idle_tasks,0,sizeof(idle_tasks));
    memset(hooks,0,sizeof(hooks));atomic_store(&enabled,false);atomic_store(&host_phase,PROBE_HOST_OTHER);attempted=false;known_task_mask=0;
    fail_core=-1;registrations=deregistrations=clock_us=cycles=0;missing_controller=false;running=0;
}
static void tick(unsigned core,unsigned identity,uint32_t at){running=task(identity);clock_us=at;hooks[core]();}
int main(void){
    probe_scheduler_snapshot_t a,b;
    fixture();probe_scheduler_snapshot(0,&a);CHECK(!a.coherent);
    CHECK(probe_scheduler_start(task(1),task(2),task(4))==ESP_OK && registrations==2 && hooks[0] && hooks[1]);
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
    for(int core=0;core<2;++core){
        fixture();fail_core=core;CHECK(probe_scheduler_start(task(1),task(2),task(4))==ESP_ERR_NO_MEM);
        CHECK(!hooks[0] && !hooks[1] && deregistrations==(unsigned)core);
        sample_tick0();sample_tick1();probe_scheduler_snapshot(0,&a);CHECK(!a.coherent && !atomic_load(&samples[0].sequence));
    }
    fixture();CHECK(probe_scheduler_start(0,task(2),task(4))==ESP_ERR_INVALID_ARG && !registrations);
    fixture();missing_controller=true;CHECK(probe_scheduler_start(task(1),task(2),task(4))==ESP_OK);
    tick(1,5,1000);probe_scheduler_snapshot(1,&a);CHECK(a.known_task_mask==15 && a.counts[PROBE_SAMPLE_OTHER]==1);
    puts("PASS sampler: per-core identities, tick gaps, sequence/counter/time wrap, nonspinning snapshots, missing tasks and registration rollback");return 0;
}
