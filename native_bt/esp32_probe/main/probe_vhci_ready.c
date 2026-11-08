#include "probe_vhci_ready.h"
#ifdef ESP_PLATFORM
#include "esp_attr.h"
#define READY_IRAM IRAM_ATTR
#else
#define READY_IRAM
#endif
_Static_assert(ATOMIC_INT_LOCK_FREE==2,"VHCI observation requires lock-free words");
static struct {
    _Atomic uint32_t version,at_us,flags,submitted_us,packet,pending_acks;
    _Atomic uint32_t observations,held,ready_while_held,max_ready_span_us,max_slot_age_us;
    _Atomic uint32_t *owned;
    uint32_t first_ready_us;
    bool first_ready,submitted;
} ready;
static void maximum(_Atomic uint32_t *cell,uint32_t value){
    /* Only the owner writes these counters. No CAS retry or contention loop. */
    if(value>atomic_load_explicit(cell,memory_order_relaxed))atomic_store_explicit(cell,value,memory_order_relaxed);
}
static void increment(_Atomic uint32_t *cell){
    atomic_store_explicit(cell,atomic_load_explicit(cell,memory_order_relaxed)+1u,memory_order_relaxed);
}
void probe_vhci_ready_init(_Atomic uint32_t *owned){
#define INIT(name) atomic_init(&ready.name,0)
    INIT(version);INIT(at_us);INIT(flags);INIT(submitted_us);INIT(packet);INIT(pending_acks);
    INIT(observations);INIT(held);INIT(ready_while_held);INIT(max_ready_span_us);INIT(max_slot_age_us);
#undef INIT
    ready.owned=owned;ready.first_ready_us=0;ready.first_ready=false;ready.submitted=false;
}
void probe_vhci_ready_submitted(uint32_t at,uint32_t packet){
    uint32_t version=atomic_load(&ready.version);atomic_store(&ready.version,version+1u);
    atomic_store(&ready.submitted_us,at);atomic_store(&ready.packet,packet);
    atomic_store(&ready.at_us,0);atomic_store(&ready.flags,PROBE_VHCI_SUBMITTED);
    atomic_store(&ready.pending_acks,UINT32_MAX);
    ready.first_ready=false;ready.submitted=true;
    atomic_store(&ready.version,version+2u);
}
void probe_vhci_ready_observe(bool held,bool sdk,uint32_t at,uint32_t pending){
    uint32_t version=atomic_load(&ready.version);atomic_store(&ready.version,version+1u);
    increment(&ready.observations);if(held)increment(&ready.held);
    if(held && sdk){
        increment(&ready.ready_while_held);
        if(!ready.first_ready){ready.first_ready=true;ready.first_ready_us=at;}
        uint32_t span=at-ready.first_ready_us;if(span<0x80000000u)maximum(&ready.max_ready_span_us,span);
        if(ready.submitted){uint32_t age=at-atomic_load(&ready.submitted_us);if(age<0x80000000u)maximum(&ready.max_slot_age_us,age);}
    }else ready.first_ready=false;
    atomic_store(&ready.at_us,at);atomic_store(&ready.pending_acks,pending);
    atomic_store(&ready.flags,PROBE_VHCI_SAMPLE|(held?PROBE_VHCI_HELD:0u)|(sdk?PROBE_VHCI_SDK_READY:0u)|(ready.submitted?PROBE_VHCI_SUBMITTED:0u));
    atomic_store(&ready.version,version+2u);
}
void READY_IRAM probe_vhci_ready_sample(probe_vhci_ready_sample_t *out){
    if(!out)return;
    uint32_t before=atomic_load(&ready.version);
    out->at_us=atomic_load(&ready.at_us);out->flags=atomic_load(&ready.flags);
    out->submitted_us=atomic_load(&ready.submitted_us);out->packet=atomic_load(&ready.packet);
    out->pending_acks=atomic_load(&ready.pending_acks);
    out->owned_now=ready.owned?atomic_load_explicit(ready.owned,memory_order_acquire):UINT32_MAX;
    out->coherent=!(before&1u) && before==atomic_load(&ready.version);
}
void probe_vhci_ready_stats(probe_vhci_ready_stats_t *out){
    if(!out)return;
    out->observations=atomic_load(&ready.observations);out->held=atomic_load(&ready.held);
    out->ready_while_held=atomic_load(&ready.ready_while_held);
    out->max_ready_span_us=atomic_load(&ready.max_ready_span_us);out->max_slot_age_us=atomic_load(&ready.max_slot_age_us);
    out->bytes=(uint32_t)sizeof ready;
}
