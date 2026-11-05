#include "probe_scheduler.h"
#include "esp_attr.h"
#include "esp_cpu.h"
#include "esp_freertos_hooks.h"
#include "esp_timer.h"
#include <stdatomic.h>

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
static DRAM_ATTR TaskHandle_t tasks[5],idle_tasks[2];
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
