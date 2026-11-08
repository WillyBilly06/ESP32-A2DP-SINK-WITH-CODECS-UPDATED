#ifndef PROBE_DISPATCH_TRACE_H
#define PROBE_DISPATCH_TRACE_H
#include <stdbool.h>
#include <stdint.h>
#define PROBE_DISPATCH_SLOTS 16u
enum {PROBE_DISPATCH_OTHER,PROBE_DISPATCH_HOST,PROBE_DISPATCH_RENDER,PROBE_DISPATCH_DECODE,
      PROBE_DISPATCH_HELPER,PROBE_DISPATCH_CONTROLLER,PROBE_DISPATCH_IPC1,
      PROBE_DISPATCH_IDLE0,PROBE_DISPATCH_IDLE1,PROBE_DISPATCH_LOGGER,PROBE_DISPATCH_ROLES};
enum {PROBE_DISPATCH_SWITCH=1,PROBE_DISPATCH_EOF,PROBE_DISPATCH_RENDER_ENTER,
      PROBE_DISPATCH_RENDER_LEAVE,PROBE_DISPATCH_HELPER_QUEUED,PROBE_DISPATCH_PCM_READY,
      PROBE_DISPATCH_HELPER_DONE,PROBE_DISPATCH_DECODE_ENTER,PROBE_DISPATCH_DECODE_LEAVE,
      PROBE_DISPATCH_HOST_YIELD,PROBE_DISPATCH_FREEZE,PROBE_DISPATCH_HELPER_SUBMITTED};
typedef struct {uint32_t at_us,info;} probe_dispatch_event_t;
typedef struct {
    uint32_t serial,begin_us,gap_us,core,head,begin_head,count,bindings;
    probe_dispatch_event_t events[PROBE_DISPATCH_SLOTS];
    bool coherent;
} probe_dispatch_snapshot_t;
#ifdef DBT_PROBE_DISPATCH_TRACE
/* Permanent task identities only; no dereference or name lookup in trace/ISR. */
void probe_dispatch_bind(unsigned,void *);
/* Stream owner only. Monotonic ring heads are never reset across owners. */
void probe_dispatch_begin(uint32_t serial,uint32_t at_us);
void probe_dispatch_stop(void);
/* Fixed work under a local IRQ mask; no cross-core lock, allocation or UART. */
void probe_dispatch_switch(unsigned core,void *selected);
void probe_dispatch_mark(unsigned kind,unsigned role,uint32_t value);
void probe_dispatch_eof(uint32_t at_us,void *interrupted);
/* First finalized gap freezes future writes. An already-entered other-core
 * write may finish; its timestamp remains visible, including after-gap records. */
void probe_dispatch_freeze(uint32_t serial,uint32_t at_us);
/* One attempt over frozen atomic cells, only gaps within15s of begin. Longer
 * epochs are unavailable. Fixed240MHz bounds the modular per-core write count.
 * False means unavailable/torn; no spin. */
bool probe_dispatch_snapshot(unsigned core,probe_dispatch_snapshot_t *);
#else
#define probe_dispatch_bind(...) ((void)0)
#define probe_dispatch_begin(...) ((void)0)
#define probe_dispatch_stop(...) ((void)0)
#define probe_dispatch_switch(...) ((void)0)
#define probe_dispatch_mark(...) ((void)0)
#define probe_dispatch_eof(...) ((void)0)
#define probe_dispatch_freeze(...) ((void)0)
#endif
#endif
