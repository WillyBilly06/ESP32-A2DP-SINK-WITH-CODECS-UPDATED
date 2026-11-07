#include "probe_task_cycles.h"
#include "esp_attr.h"
#include <stdatomic.h>
#include <stddef.h>
#ifdef DBT_TASK_CYCLES_TEST
extern unsigned probe_task_test_core(void);
extern uint32_t probe_task_test_cycle(void),probe_task_test_mask(void);
extern void probe_task_test_unmask(uint32_t);
#define CYCLE_NOW() probe_task_test_cycle()
#define OWN_CORE() probe_task_test_core()
#define MASK_ENTER() probe_task_test_mask()
#define MASK_EXIT(x) probe_task_test_unmask(x)
#else
#include "esp_cpu.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <xtensa/xtruntime.h>
#if CONFIG_PM_ENABLE || CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ != 240
#error Task-cycle accounting requires the fixed 240 MHz ESP32 clock
#endif
#define CYCLE_NOW() esp_cpu_get_cycle_count()
#define OWN_CORE() ((unsigned)xPortGetCoreID())
#define MASK_ENTER() XTOS_SET_MIN_INTLEVEL(XCHAL_EXCM_LEVEL)
#define MASK_EXIT(x) XTOS_RESTORE_JUST_INTLEVEL(x)
#endif
typedef struct {
    _Atomic uintptr_t target,current;
    _Atomic uint32_t began,total,faults;
} cycle_core_t;
static DRAM_ATTR cycle_core_t cycle_cores[2];
static DRAM_ATTR _Atomic uint32_t cycle_enabled;
_Static_assert(ATOMIC_INT_LOCK_FREE==2 && ATOMIC_POINTER_LOCK_FREE==2,"Trace counters must be lock-free words");
void probe_task_cycles_bind(unsigned core,void *task){
    if(core<2u)atomic_store_explicit(&cycle_cores[core].target,(uintptr_t)task,memory_order_release);
}
void probe_task_cycles_enable(void){atomic_store_explicit(&cycle_enabled,1,memory_order_release);}
void IRAM_ATTR probe_task_cycles_in(unsigned core,void *task){
    if(core>=2u || !atomic_load_explicit(&cycle_enabled,memory_order_acquire))return;
    cycle_core_t *s=&cycle_cores[core];
    if(!task || atomic_load_explicit(&s->current,memory_order_relaxed))
        atomic_store_explicit(&s->faults,atomic_load_explicit(&s->faults,memory_order_relaxed)+1u,memory_order_relaxed);
    atomic_store_explicit(&s->began,CYCLE_NOW(),memory_order_relaxed);
    atomic_store_explicit(&s->current,(uintptr_t)task,memory_order_relaxed);
}
void IRAM_ATTR probe_task_cycles_out(unsigned core,void *task){
    if(core>=2u || !atomic_load_explicit(&cycle_enabled,memory_order_acquire))return;
    cycle_core_t *s=&cycle_cores[core];
    uintptr_t current=atomic_load_explicit(&s->current,memory_order_relaxed);
    if(current && current!=(uintptr_t)task)
        atomic_store_explicit(&s->faults,atomic_load_explicit(&s->faults,memory_order_relaxed)+1u,memory_order_relaxed);
    if(current && current==(uintptr_t)task && current==atomic_load_explicit(&s->target,memory_order_acquire)){
        uint32_t elapsed=CYCLE_NOW()-atomic_load_explicit(&s->began,memory_order_relaxed);
        atomic_store_explicit(&s->total,atomic_load_explicit(&s->total,memory_order_relaxed)+elapsed,memory_order_relaxed);
    }
    atomic_store_explicit(&s->current,0,memory_order_relaxed);
}
void probe_task_cycles_mark(unsigned expected_core,probe_task_cycle_mark_t *out){
    if(!out)return;
    *out=(probe_task_cycle_mark_t){0};
    unsigned core=OWN_CORE();
    if(core>=2u || core!=expected_core || !atomic_load_explicit(&cycle_enabled,memory_order_acquire))return;
    uint32_t mask=MASK_ENTER();cycle_core_t *s=&cycle_cores[core];
    uintptr_t target=atomic_load_explicit(&s->target,memory_order_acquire);
    uintptr_t current=atomic_load_explicit(&s->current,memory_order_relaxed);
    uint32_t faults=atomic_load_explicit(&s->faults,memory_order_relaxed);
    if(target && current==target && !faults){
        uint32_t now=CYCLE_NOW();
        out->wall_cycles=now;out->charged_cycles=atomic_load_explicit(&s->total,memory_order_relaxed)+now-atomic_load_explicit(&s->began,memory_order_relaxed);
        out->owner=target;out->valid=true;
    }
    MASK_EXIT(mask);
}
void probe_task_cycles_faults(uint32_t out[2]){
    if(out)for(unsigned core=0;core<2u;++core)out[core]=atomic_load_explicit(&cycle_cores[core].faults,memory_order_relaxed);
}
