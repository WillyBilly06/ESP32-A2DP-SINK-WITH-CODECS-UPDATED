#ifndef PROBE_VHCI_READY_H
#define PROBE_VHCI_READY_H
#include <stdbool.h>
#include <stdint.h>
#include <stdatomic.h>
enum {PROBE_VHCI_SAMPLE=1u,PROBE_VHCI_HELD=2u,PROBE_VHCI_SDK_READY=4u,PROBE_VHCI_SUBMITTED=8u};
typedef struct {uint32_t coherent,at_us,flags,submitted_us,packet,pending_acks,owned_now;} probe_vhci_ready_sample_t;
typedef struct {uint32_t observations,held,ready_while_held,max_ready_span_us,max_slot_age_us,bytes;} probe_vhci_ready_stats_t;
/* One host-owner writer. Initialize before callbacks/IRQs; flag must outlive
 * this diagnostic. No controller callback writes this diagnostic object. */
void probe_vhci_ready_init(_Atomic uint32_t *owned_flag);
void probe_vhci_ready_submitted(uint32_t at_us,uint32_t packet);
void probe_vhci_ready_observe(bool held,bool sdk_ready,uint32_t at_us,uint32_t pending_acks);
/* Bounded IRAM snapshot, no SDK call or retry. owned_now is an independent
 * atomic read; the version fence covers only the last owner observation. */
void probe_vhci_ready_sample(probe_vhci_ready_sample_t *);
void probe_vhci_ready_stats(probe_vhci_ready_stats_t *);
#endif
