#ifndef PROBE_HEAP_ALLOC_TRACE_H
#define PROBE_HEAP_ALLOC_TRACE_H
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>
#include <stdint.h>
enum {PROBE_HEAP_OWNER=1,PROBE_HEAP_SMP,PROBE_HEAP_PERSIST,PROBE_HEAP_GENERAL,
      PROBE_HEAP_CREATE_SMP,PROBE_HEAP_CREATE_PERSIST,PROBE_HEAP_CREATE_GENERAL,
      PROBE_HEAP_CCCD_LOAD,PROBE_HEAP_CCCD_SAVE};
#define PROBE_HEAP_ALLOC_ROWS 48u
typedef struct {uint32_t at_us,free_bytes,minimum_bytes,kind;bool valid;} probe_heap_mark_t;
typedef struct {uint32_t count,overflow,collision,begin_us,stop_requested_us,frozen_us,reason,handle,generation,bytes,owner_task_id,smp_task_id,persist_task_id,general_task_id;} probe_heap_capture_t;
typedef struct {uint32_t kind,a,b,c,begin_us,end_us,actor,flags;} probe_heap_row_t;
#ifdef DBT_PROBE_HEAP_ALLOC_TRACE
/* One capture per boot. No reset/retry, task, allocator or UART work in hooks. */
bool probe_heap_alloc_arm(uint16_t,uint32_t,TaskHandle_t);
void probe_heap_alloc_link_removed(uint16_t,uint32_t);
void probe_heap_alloc_task(unsigned,bool);
void probe_heap_alloc_owner_phase(unsigned);
probe_heap_mark_t probe_heap_alloc_begin(unsigned);
void probe_heap_alloc_end(probe_heap_mark_t);
/* Logger only: close after link removal/timeout, one attempt to freeze. */
void probe_heap_alloc_poll(void);
bool probe_heap_alloc_snapshot(probe_heap_capture_t *);
bool probe_heap_alloc_row(unsigned,probe_heap_row_t *);
#else
#define probe_heap_alloc_task(role,active) ((void)0)
#define probe_heap_alloc_owner_phase(phase) ((void)0)
#endif
#endif
