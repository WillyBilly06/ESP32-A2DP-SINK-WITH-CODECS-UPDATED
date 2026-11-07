#include "probe_scheduler.h"
#include "esp_attr.h"
#include "esp_cpu.h"
#include "esp_freertos_hooks.h"
#include "esp_timer.h"
#include <stdatomic.h>
#include <stddef.h>

/* Tick sampling observes interrupted task identity, not precise CPU runtime.
 * Interrupt masking, higher-level ISRs and periodic-work aliasing bias it.
 * Each core is its counters' only writer. Logger reads atomically once and
 * reports a failed sequence check instead of spinning for a coherent snapshot.
 * No task priority query: that API takes the kernel's cross-core lock. */
typedef struct {
    _Atomic uint32_t sequence,counts[PROBE_SAMPLE_ROLES],at_us,max_gap_us;
    _Atomic uint32_t host_phases[PROBE_HOST_PHASES];
    _Atomic uint32_t max_gap_begin_us,max_gap_end_us,max_gap_previous_role,max_gap_current_role;
    _Atomic uint32_t max_callback_cycles;
    uint32_t previous_role;
    bool have_tick;
} sample_core_t;
static DRAM_ATTR sample_core_t samples[2];
static DRAM_ATTR TaskHandle_t tasks[5],idle_tasks[2],ipc_tasks[2],timer_task;
#ifdef DBT_PROBE_CODEC_AWARE_RENDER
static DRAM_ATTR _Atomic(TaskHandle_t) renderer0;
void probe_scheduler_bind_renderer0(TaskHandle_t task){atomic_store_explicit(&renderer0,task,memory_order_release);}
#endif
static DRAM_ATTR _Atomic bool enabled;
static DRAM_ATTR _Atomic uint32_t host_phase;
static uint32_t known_task_mask;
static bool attempted;

static void IRAM_ATTR sample_tick(unsigned core){
    uint32_t began=esp_cpu_get_cycle_count();
    if(!atomic_load_explicit(&enabled,memory_order_acquire))return;
    TaskHandle_t current=xTaskGetCurrentTaskHandle();
    unsigned role=PROBE_SAMPLE_OTHER;
    for(unsigned i=0;i<5;++i)if(tasks[i] && current==tasks[i]){role=i;break;}
    if(role==PROBE_SAMPLE_OTHER && current==idle_tasks[core])role=PROBE_SAMPLE_IDLE;
#ifdef DBT_PROBE_CODEC_AWARE_RENDER
    if(role==PROBE_SAMPLE_OTHER && current==atomic_load_explicit(&renderer0,memory_order_acquire))role=PROBE_SAMPLE_RENDERER;
#endif
    if(role==PROBE_SAMPLE_OTHER && current==ipc_tasks[core] && current)role=PROBE_SAMPLE_IPC;
    if(role==PROBE_SAMPLE_OTHER && current==timer_task && current)role=PROBE_SAMPLE_TIMER;
    sample_core_t *s=&samples[core];
    uint32_t sequence=atomic_load_explicit(&s->sequence,memory_order_relaxed);
    atomic_store_explicit(&s->sequence,sequence+1u,memory_order_seq_cst);
    uint32_t now=(uint32_t)esp_timer_get_time();
    uint32_t previous=atomic_load_explicit(&s->at_us,memory_order_relaxed),gap=now-previous;
    if(s->have_tick && gap>atomic_load_explicit(&s->max_gap_us,memory_order_relaxed)){
        atomic_store_explicit(&s->max_gap_us,gap,memory_order_relaxed);
        atomic_store_explicit(&s->max_gap_begin_us,previous,memory_order_relaxed);
        atomic_store_explicit(&s->max_gap_end_us,now,memory_order_relaxed);
        atomic_store_explicit(&s->max_gap_previous_role,s->previous_role,memory_order_relaxed);
        atomic_store_explicit(&s->max_gap_current_role,role,memory_order_relaxed);
    }
    atomic_store_explicit(&s->at_us,now,memory_order_relaxed);s->previous_role=role;s->have_tick=true;
    atomic_store_explicit(&s->counts[role],atomic_load_explicit(&s->counts[role],memory_order_relaxed)+1u,memory_order_relaxed);
    if(role==PROBE_SAMPLE_HOST){
        unsigned phase=atomic_load_explicit(&host_phase,memory_order_relaxed);
        if(phase>=PROBE_HOST_PHASES)phase=PROBE_HOST_OTHER;
        atomic_store_explicit(&s->host_phases[phase],atomic_load_explicit(&s->host_phases[phase],memory_order_relaxed)+1u,memory_order_relaxed);
    }
    uint32_t elapsed=esp_cpu_get_cycle_count()-began;
    if(elapsed>atomic_load_explicit(&s->max_callback_cycles,memory_order_relaxed))
        atomic_store_explicit(&s->max_callback_cycles,elapsed,memory_order_relaxed);
    atomic_store_explicit(&s->sequence,sequence+2u,memory_order_release);
}
static void IRAM_ATTR sample_tick0(void){sample_tick(0);}
static void IRAM_ATTR sample_tick1(void){sample_tick(1);}

esp_err_t probe_scheduler_start(TaskHandle_t host,TaskHandle_t decoder,TaskHandle_t logger){
    if(attempted)return ESP_ERR_INVALID_STATE;
    attempted=true;
    if(!host || !decoder || !logger)return ESP_ERR_INVALID_ARG;
    tasks[PROBE_SAMPLE_HOST]=host;tasks[PROBE_SAMPLE_DECODER]=decoder;
    tasks[PROBE_SAMPLE_LOGGER]=logger;tasks[PROBE_SAMPLE_RENDERER]=xTaskGetHandle("native_render");
    tasks[PROBE_SAMPLE_CONTROLLER]=xTaskGetHandle("btController");
    for(unsigned i=0;i<5;++i)if(tasks[i])known_task_mask|=1u<<i;
    for(unsigned core=0;core<2;++core)idle_tasks[core]=xTaskGetIdleTaskHandleForCore(core);
    ipc_tasks[0]=xTaskGetHandle("ipc0");ipc_tasks[1]=xTaskGetHandle("ipc1");timer_task=xTaskGetHandle("esp_timer");
    if(ipc_tasks[0] || ipc_tasks[1])known_task_mask|=1u<<PROBE_SAMPLE_IPC;
    if(timer_task)known_task_mask|=1u<<PROBE_SAMPLE_TIMER;
    atomic_store_explicit(&enabled,true,memory_order_release);
    esp_err_t result=esp_register_freertos_tick_hook_for_cpu(sample_tick0,0);
    if(result==ESP_OK){
        result=esp_register_freertos_tick_hook_for_cpu(sample_tick1,1);
        if(result!=ESP_OK)esp_deregister_freertos_tick_hook_for_cpu(sample_tick0,0);
    }
    if(result!=ESP_OK)atomic_store_explicit(&enabled,false,memory_order_release);
    return result;
}
void probe_scheduler_snapshot(unsigned core,probe_scheduler_snapshot_t *out){
    if(!out)return;
    *out=(probe_scheduler_snapshot_t){0};
    if(core>=2 || !atomic_load_explicit(&enabled,memory_order_acquire))return;
    sample_core_t *s=&samples[core];uint32_t before=atomic_load_explicit(&s->sequence,memory_order_acquire);
    for(unsigned i=0;i<PROBE_SAMPLE_ROLES;++i)out->counts[i]=atomic_load_explicit(&s->counts[i],memory_order_relaxed);
    for(unsigned i=0;i<PROBE_HOST_PHASES;++i)out->host_phases[i]=atomic_load_explicit(&s->host_phases[i],memory_order_relaxed);
#define READ(field) out->field=atomic_load_explicit(&s->field,memory_order_relaxed)
    READ(at_us);READ(max_gap_us);READ(max_gap_begin_us);READ(max_gap_end_us);
    READ(max_gap_previous_role);READ(max_gap_current_role);READ(max_callback_cycles);
#undef READ
    atomic_thread_fence(memory_order_seq_cst);
    out->known_task_mask=known_task_mask;
    out->coherent=!(before&1u) && before==atomic_load_explicit(&s->sequence,memory_order_acquire);
}
unsigned probe_scheduler_host_phase(unsigned phase){
    unsigned previous=atomic_load_explicit(&host_phase,memory_order_relaxed);
    atomic_store_explicit(&host_phase,phase<PROBE_HOST_PHASES?phase:PROBE_HOST_OTHER,memory_order_relaxed);
    return previous;
}

/* Each kind has one render/lifecycle writer on its respective core. Public cells remain
 * atomic so the background logger can make one nonblocking read attempt. */
typedef struct {
    _Atomic uint32_t sequence,begin_us,end_us,wall_us,frames,begin_tick_us,end_tick_us,stream_serial,rate;
    _Atomic uint32_t counts[PROBE_SAMPLE_ROLES],host_phases[PROBE_HOST_PHASES];
    _Atomic uint32_t begin_read_cycles,end_read_cycles,known_task_mask,markers_valid;
} interval_record_t;
static DRAM_ATTR interval_record_t intervals[PROBE_DELAY_KINDS];
void probe_scheduler_mark(unsigned core,probe_scheduler_mark_t *out){
    if(!out)return;
    *out=(probe_scheduler_mark_t){0};
    if(core>=2 || !atomic_load_explicit(&enabled,memory_order_acquire))return;
    uint32_t began=esp_cpu_get_cycle_count();
    sample_core_t *s=&samples[core];uint32_t before=atomic_load_explicit(&s->sequence,memory_order_acquire);
    for(unsigned i=0;i<PROBE_SAMPLE_ROLES;++i)out->counts[i]=atomic_load_explicit(&s->counts[i],memory_order_relaxed);
    for(unsigned i=0;i<PROBE_HOST_PHASES;++i)out->host_phases[i]=atomic_load_explicit(&s->host_phases[i],memory_order_relaxed);
    out->at_us=atomic_load_explicit(&s->at_us,memory_order_relaxed);out->known_task_mask=known_task_mask;out->core=core;
    atomic_thread_fence(memory_order_seq_cst);
    out->coherent=!(before&1u) && before==atomic_load_explicit(&s->sequence,memory_order_acquire);
    out->read_cycles=esp_cpu_get_cycle_count()-began;
}
void probe_scheduler_interval_observe(unsigned kind,const probe_scheduler_mark_t *begin,
    const probe_scheduler_mark_t *end,uint32_t begin_us,uint32_t end_us,uint32_t frames,uint32_t stream_serial,uint32_t rate){
    if(kind>=PROBE_DELAY_KINDS || !begin || !end || !atomic_load_explicit(&enabled,memory_order_acquire))return;
    interval_record_t *r=&intervals[kind];uint32_t wall=end_us-begin_us;
    if(stream_serial==atomic_load_explicit(&r->stream_serial,memory_order_relaxed) && wall<=atomic_load_explicit(&r->wall_us,memory_order_relaxed))return;
    uint32_t sequence=atomic_load_explicit(&r->sequence,memory_order_relaxed);
    unsigned core=kind==PROBE_DELAY_LOCAL_RENDER0?0u:1u;
    bool valid=begin->coherent && end->coherent && begin->core==core && end->core==core && begin->known_task_mask==end->known_task_mask;
    atomic_store_explicit(&r->sequence,sequence+1u,memory_order_seq_cst);
#define INTERVAL_SET(field,value) atomic_store_explicit(&r->field,(value),memory_order_relaxed)
    INTERVAL_SET(stream_serial,stream_serial);INTERVAL_SET(rate,rate);
    INTERVAL_SET(begin_us,begin_us);INTERVAL_SET(end_us,end_us);INTERVAL_SET(wall_us,wall);INTERVAL_SET(frames,frames);
    INTERVAL_SET(begin_tick_us,begin->at_us);INTERVAL_SET(end_tick_us,end->at_us);
    INTERVAL_SET(begin_read_cycles,begin->read_cycles);INTERVAL_SET(end_read_cycles,end->read_cycles);
    INTERVAL_SET(known_task_mask,end->known_task_mask);INTERVAL_SET(markers_valid,valid);
#undef INTERVAL_SET
    for(unsigned i=0;i<PROBE_SAMPLE_ROLES;++i)atomic_store_explicit(&r->counts[i],valid?end->counts[i]-begin->counts[i]:0,memory_order_relaxed);
    for(unsigned i=0;i<PROBE_HOST_PHASES;++i)atomic_store_explicit(&r->host_phases[i],valid?end->host_phases[i]-begin->host_phases[i]:0,memory_order_relaxed);
    uint32_t published=sequence+2u;if(!published)published=2u; /* zero means unpublished */
    atomic_store_explicit(&r->sequence,published,memory_order_release);
}
void probe_scheduler_interval_snapshot(unsigned kind,probe_scheduler_interval_t *out){
    if(!out)return;
    *out=(probe_scheduler_interval_t){0};if(kind>=PROBE_DELAY_KINDS)return;
    interval_record_t *r=&intervals[kind];uint32_t before=atomic_load_explicit(&r->sequence,memory_order_acquire);
#define INTERVAL_GET(field) out->field=atomic_load_explicit(&r->field,memory_order_relaxed)
    INTERVAL_GET(stream_serial);INTERVAL_GET(rate);
    INTERVAL_GET(begin_us);INTERVAL_GET(end_us);INTERVAL_GET(wall_us);INTERVAL_GET(frames);
    INTERVAL_GET(begin_tick_us);INTERVAL_GET(end_tick_us);INTERVAL_GET(begin_read_cycles);INTERVAL_GET(end_read_cycles);
    INTERVAL_GET(known_task_mask);INTERVAL_GET(markers_valid);
#undef INTERVAL_GET
    for(unsigned i=0;i<PROBE_SAMPLE_ROLES;++i)out->counts[i]=atomic_load_explicit(&r->counts[i],memory_order_relaxed);
    for(unsigned i=0;i<PROBE_HOST_PHASES;++i)out->host_phases[i]=atomic_load_explicit(&r->host_phases[i],memory_order_relaxed);
    atomic_thread_fence(memory_order_seq_cst);out->revision=before;
    out->coherent=before && !(before&1u) && before==atomic_load_explicit(&r->sequence,memory_order_acquire);
}

#ifdef DBT_PROBE_TASK_RUNTIME_STATS
#if configGENERATE_RUN_TIME_STATS != 1 || configUSE_TRACE_FACILITY != 1 || configTASKLIST_INCLUDE_COREID != 1 || !CONFIG_FREERTOS_RUN_TIME_STATS_USING_ESP_TIMER
#error Task-runtime observation requires SDK ESP-timer statistics, trace and core IDs
#endif
_Static_assert(sizeof(configRUN_TIME_COUNTER_TYPE)==sizeof(uint32_t),"Runtime records require uint32 microseconds");
void probe_scheduler_task_runtime(probe_task_runtime_snapshot_t *out){
    if(!out)return;
    *out=(probe_task_runtime_snapshot_t){0};
    if(!atomic_load_explicit(&enabled,memory_order_acquire))return;
    out->begin_us=(uint32_t)esp_timer_get_time();
    for(unsigned i=0;i<PROBE_RUNTIME_TASKS;++i){
        TaskHandle_t task=NULL;
        if(i<5u)task=tasks[i];
        else if(i<7u)task=idle_tasks[i-5u];
        else if(i<9u)task=ipc_tasks[i-7u];
        else if(i==9u)task=timer_task;
#ifdef DBT_PROBE_CODEC_AWARE_RENDER
        else task=atomic_load_explicit(&renderer0,memory_order_acquire);
#endif
        if(!task)continue;
        TaskStatus_t info;
        uint32_t began=(uint32_t)esp_timer_get_time();
        /* Permanent handles live until reset. Never call this from the tick,
         * decoder, controller or render paths; no stack walk or state scan. */
        vTaskGetInfo(task,&info,pdFALSE,eReady);
        uint32_t elapsed=(uint32_t)esp_timer_get_time()-began;
        if(elapsed>out->max_query_us)out->max_query_us=elapsed;
        out->tasks[i]=(probe_task_runtime_t){(uint32_t)info.ulRunTimeCounter,
            (uint32_t)info.xTaskNumber,(uint32_t)info.uxCurrentPriority,(int32_t)info.xCoreID};
        out->present_mask|=1u<<i;
    }
    out->end_us=(uint32_t)esp_timer_get_time();
}
#endif
