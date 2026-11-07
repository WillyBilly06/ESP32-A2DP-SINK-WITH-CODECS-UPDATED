#ifndef PROBE_TASK_CYCLES_H
#define PROBE_TASK_CYCLES_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    uint32_t charged_cycles,wall_cycles;
    uintptr_t owner;
    bool valid;
} probe_task_cycle_mark_t;
/* Permanent pinned render handles; bind before their first measured work.
 * No task object is dereferenced. Enable only after scheduler registration. */
void probe_task_cycles_bind(unsigned core,void *task);
void probe_task_cycles_enable(void);
/* SDK trace hooks: counter/pointer operations only; no kernel or logging API. */
void probe_task_cycles_in(unsigned core,void *task);
void probe_task_cycles_out(unsigned core,void *task);
/* Own-core task context only. Includes the current running tail and IRQ time;
 * excludes intervals switched away and the OUT-to-IN selection gap. One local
 * interrupt-mask span, no kernel lock, stack scan, allocation or retry. */
void probe_task_cycles_mark(unsigned expected_core,probe_task_cycle_mark_t *);
void probe_task_cycles_faults(uint32_t faults[2]);
#endif
