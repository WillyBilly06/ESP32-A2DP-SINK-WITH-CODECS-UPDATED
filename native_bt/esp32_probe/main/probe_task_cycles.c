#include "probe_task_cycles.h"
#include "esp_attr.h"
#include <stdatomic.h>
#include <stddef.h>
#ifdef DBT_TASK_CYCLES_TEST
extern unsigned probe_task_test_core(void),probe_task_test_isr(void);
extern uint32_t probe_task_test_cycle(void),probe_task_test_mask(void);
extern void probe_task_test_unmask(uint32_t);
#define CYCLE_NOW() probe_task_test_cycle()
#define OWN_CORE() probe_task_test_core()
#define IN_ISR() probe_task_test_isr()
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
#define IN_ISR() xPortInIsrContext()
#define MASK_ENTER() XTOS_SET_MIN_INTLEVEL(XCHAL_EXCM_LEVEL)
#define MASK_EXIT(x) XTOS_RESTORE_JUST_INTLEVEL(x)
#endif
typedef struct {
    _Atomic uintptr_t target[2],current;
    _Atomic uint32_t began,total[2],faults;
} cycle_core_t;
static DRAM_ATTR cycle_core_t cycle_cores[2];
static DRAM_ATTR _Atomic uint32_t cycle_enabled;
_Static_assert(ATOMIC_INT_LOCK_FREE==2 && ATOMIC_POINTER_LOCK_FREE==2,"Trace counters must be lock-free words");
void probe_task_cycles_bind(unsigned core,void *task){
    if(core<2u)atomic_store_explicit(&cycle_cores[core].target[0],(uintptr_t)task,memory_order_release);
}
void probe_task_cycles_bind_host(void *task){atomic_store_explicit(&cycle_cores[1].target[1],(uintptr_t)task,memory_order_release);}
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
    if(current && current==(uintptr_t)task){
        /* Exactly two identity comparisons, not a task-table traversal. */
        unsigned slot=current==atomic_load_explicit(&s->target[0],memory_order_acquire)?0u:
            current==atomic_load_explicit(&s->target[1],memory_order_acquire)?1u:2u;
        if(slot<2u){
            uint32_t elapsed=CYCLE_NOW()-atomic_load_explicit(&s->began,memory_order_relaxed);
            atomic_store_explicit(&s->total[slot],atomic_load_explicit(&s->total[slot],memory_order_relaxed)+elapsed,memory_order_relaxed);
        }
    }
    atomic_store_explicit(&s->current,0,memory_order_relaxed);
}
static void mark_slot(unsigned expected_core,unsigned slot,probe_task_cycle_mark_t *out){
    if(!out)return;
    *out=(probe_task_cycle_mark_t){0};
    unsigned core=OWN_CORE();
    if(!atomic_load_explicit(&cycle_enabled,memory_order_acquire))return;
    if(core>=2u || core!=expected_core){out->context_known=true;return;}
    uint32_t mask=MASK_ENTER();
    /* The public context query nests its own mask. Keep it inside this outer
     * span, so identity, state and counter reads share one protected window. */
    if(IN_ISR()){out->context_known=true;MASK_EXIT(mask);return;}
    cycle_core_t *s=&cycle_cores[core];
    uintptr_t target=atomic_load_explicit(&s->target[slot],memory_order_acquire);
    uintptr_t current=atomic_load_explicit(&s->current,memory_order_relaxed);
    out->context_known=current!=0;
    uint32_t faults=atomic_load_explicit(&s->faults,memory_order_relaxed);
    if(target && current==target){
        out->owner=target;
        if(!faults){
            uint32_t now=CYCLE_NOW();
            out->wall_cycles=now;out->charged_cycles=atomic_load_explicit(&s->total[slot],memory_order_relaxed)+now-atomic_load_explicit(&s->began,memory_order_relaxed);
            out->valid=true;
        }
    }
    MASK_EXIT(mask);
}
void probe_task_cycles_mark(unsigned expected_core,probe_task_cycle_mark_t *out){mark_slot(expected_core,0,out);}
void probe_task_cycles_mark_host(probe_task_cycle_mark_t *out){mark_slot(1,1,out);}
void probe_task_cycles_faults(uint32_t out[2]){
    if(out)for(unsigned core=0;core<2u;++core)out[core]=atomic_load_explicit(&cycle_cores[core].faults,memory_order_relaxed);
}
