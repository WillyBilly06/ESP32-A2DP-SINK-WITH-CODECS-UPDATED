#include "probe_dispatch_trace.h"
#include <stdatomic.h>
#include <stddef.h>
#ifdef DBT_DISPATCH_TRACE_TEST
#define IRAM_ATTR
#define DRAM_ATTR
extern unsigned dispatch_test_core(void);
extern uint32_t dispatch_test_time(void),dispatch_test_mask(void);
extern void dispatch_test_unmask(uint32_t);
#define OWN_CORE() dispatch_test_core()
#define CLOCK_US() dispatch_test_time()
#define MASK_ENTER() dispatch_test_mask()
#define MASK_EXIT(x) dispatch_test_unmask(x)
#else
#include "esp_attr.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include <xtensa/xtruntime.h>
#define OWN_CORE() ((unsigned)xPortGetCoreID())
#define CLOCK_US() ((uint32_t)esp_timer_get_time())
#define MASK_ENTER() XTOS_SET_MIN_INTLEVEL(XCHAL_EXCM_LEVEL)
#define MASK_EXIT(x) XTOS_RESTORE_JUST_INTLEVEL(x)
#endif
#ifndef DISPATCH_SNAPSHOT_STEP
#define DISPATCH_SNAPSHOT_STEP() ((void)0)
#endif
typedef struct {
    _Atomic uint32_t version,head;
    struct {_Atomic uint32_t at_us,info;} events[PROBE_DISPATCH_SLOTS];
} dispatch_ring_t;
static DRAM_ATTR dispatch_ring_t rings[2];
static DRAM_ATTR _Atomic uintptr_t roles[PROBE_DISPATCH_ROLES];
static DRAM_ATTR struct {_Atomic uint32_t version,serial,begin_us,gap_us,enabled,frozen,begin_head[2];} trace;
_Static_assert(ATOMIC_INT_LOCK_FREE==2 && ATOMIC_POINTER_LOCK_FREE==2,"Trace uses lock-free words");
void probe_dispatch_bind(unsigned role,void *task){
    if(role && role<PROBE_DISPATCH_ROLES)atomic_store_explicit(&roles[role],(uintptr_t)task,memory_order_release);
}
static unsigned IRAM_ATTR task_role(void *task){
    if(task)for(unsigned i=1;i<PROBE_DISPATCH_ROLES;++i)
        if((uintptr_t)task==atomic_load_explicit(&roles[i],memory_order_relaxed))return i;
    return PROBE_DISPATCH_OTHER;
}
static void IRAM_ATTR record(unsigned core,uint32_t at,unsigned info){
    if(core>=2u)return;
    uint32_t mask=MASK_ENTER();
    if(atomic_load_explicit(&trace.enabled,memory_order_relaxed)){
        dispatch_ring_t *r=&rings[core];uint32_t v=atomic_load_explicit(&r->version,memory_order_relaxed);
        atomic_store_explicit(&r->version,v+1u,memory_order_seq_cst);
        uint32_t head=atomic_load_explicit(&r->head,memory_order_relaxed),i=head&(PROBE_DISPATCH_SLOTS-1u);
        if((info&15u)!=PROBE_DISPATCH_EOF)at=CLOCK_US();
        atomic_store_explicit(&r->events[i].at_us,at,memory_order_relaxed);
        atomic_store_explicit(&r->events[i].info,info,memory_order_relaxed);
        atomic_store_explicit(&r->head,head+1u,memory_order_relaxed);
        atomic_store_explicit(&r->version,v+2u,memory_order_seq_cst);
    }
    MASK_EXIT(mask);
}
void IRAM_ATTR probe_dispatch_switch(unsigned core,void *task){
    if(core<2u && atomic_load_explicit(&trace.enabled,memory_order_relaxed))
        record(core,0,PROBE_DISPATCH_SWITCH|(task_role(task)<<4));
}
void IRAM_ATTR probe_dispatch_mark(unsigned kind,unsigned role,uint32_t value){
    if(atomic_load_explicit(&trace.enabled,memory_order_relaxed))
        record(OWN_CORE(),0,(kind&15u)|((role&15u)<<4)|(value<<8));
}
void IRAM_ATTR probe_dispatch_eof(uint32_t at,void *task){
    if(atomic_load_explicit(&trace.enabled,memory_order_relaxed))
        record(OWN_CORE(),at,PROBE_DISPATCH_EOF|(task_role(task)<<4));
}
void probe_dispatch_begin(uint32_t serial,uint32_t at){
    uint32_t mask=MASK_ENTER();atomic_store_explicit(&trace.enabled,0,memory_order_relaxed);
    uint32_t v=atomic_load_explicit(&trace.version,memory_order_relaxed);
    atomic_store_explicit(&trace.version,v+1u,memory_order_seq_cst);
    atomic_store_explicit(&trace.frozen,0,memory_order_relaxed);
    atomic_store_explicit(&trace.serial,serial,memory_order_relaxed);
    atomic_store_explicit(&trace.begin_us,at,memory_order_relaxed);
    for(unsigned core=0;core<2;++core)atomic_store_explicit(&trace.begin_head[core],atomic_load_explicit(&rings[core].head,memory_order_relaxed),memory_order_relaxed);
    atomic_store_explicit(&trace.version,v+2u,memory_order_seq_cst);
    atomic_store_explicit(&trace.enabled,1,memory_order_release);MASK_EXIT(mask);
}
void probe_dispatch_stop(void){atomic_store_explicit(&trace.enabled,0,memory_order_release);}
void IRAM_ATTR probe_dispatch_freeze(uint32_t serial,uint32_t at){
    if(serial!=atomic_load_explicit(&trace.serial,memory_order_relaxed) ||
       !atomic_load_explicit(&trace.enabled,memory_order_relaxed))return;
    probe_dispatch_mark(PROBE_DISPATCH_FREEZE,PROBE_DISPATCH_OTHER,0);
    atomic_store_explicit(&trace.enabled,0,memory_order_release);
    atomic_store_explicit(&trace.gap_us,at,memory_order_relaxed);
    atomic_store_explicit(&trace.frozen,1,memory_order_release);
}
bool probe_dispatch_snapshot(unsigned core,probe_dispatch_snapshot_t *out){
    if(!out || core>=2u)return false;
    *out=(probe_dispatch_snapshot_t){0};
    uint32_t meta=atomic_load_explicit(&trace.version,memory_order_seq_cst);
    if((meta&1u) || !atomic_load_explicit(&trace.frozen,memory_order_acquire))return false;
    out->serial=atomic_load_explicit(&trace.serial,memory_order_relaxed);
    out->begin_us=atomic_load_explicit(&trace.begin_us,memory_order_relaxed);
    out->gap_us=atomic_load_explicit(&trace.gap_us,memory_order_relaxed);out->core=core;
    /* At fixed240MHz a15s window cannot contain2^32 per-core writes even
     * at one write/cycle. Modular head distance therefore also proves fill. */
    if(out->gap_us-out->begin_us>15000000u)return false;
    out->begin_head=atomic_load_explicit(&trace.begin_head[core],memory_order_relaxed);
    dispatch_ring_t *r=&rings[core];uint32_t v=atomic_load_explicit(&r->version,memory_order_seq_cst);
    if(v&1u)return false;
    out->head=atomic_load_explicit(&r->head,memory_order_relaxed);
    uint32_t available=out->head-out->begin_head;
    out->count=available<PROBE_DISPATCH_SLOTS?available:PROBE_DISPATCH_SLOTS;
    for(unsigned n=0;n<out->count;++n){unsigned i=(out->head-out->count+n)&(PROBE_DISPATCH_SLOTS-1u);
        out->events[n].at_us=atomic_load_explicit(&r->events[i].at_us,memory_order_relaxed);
        out->events[n].info=atomic_load_explicit(&r->events[i].info,memory_order_relaxed);
    }
    for(unsigned i=1;i<PROBE_DISPATCH_ROLES;++i)if(atomic_load_explicit(&roles[i],memory_order_relaxed))out->bindings|=1u<<i;
    DISPATCH_SNAPSHOT_STEP();
    out->coherent=v==atomic_load_explicit(&r->version,memory_order_seq_cst) &&
        meta==atomic_load_explicit(&trace.version,memory_order_seq_cst) && atomic_load_explicit(&trace.frozen,memory_order_acquire);
    return out->coherent;
}
