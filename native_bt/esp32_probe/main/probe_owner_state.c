#include "probe_owner_state.h"
#include "dbt_hci.h"
#include "dbt_acl.h"
#include <stddef.h>
#ifdef ESP_PLATFORM
#include "esp_attr.h"
#define OWNER_IRAM IRAM_ATTR
#else
#define OWNER_IRAM
#endif
#ifndef PROBE_OWNER_TEST_READ_PART
#define PROBE_OWNER_TEST_READ_PART(part) ((void)(part))
#endif
_Static_assert(ATOMIC_INT_LOCK_FREE==2,"Owner state requires lock-free words");
static struct {
    _Atomic uint32_t version,phase,phase_at_us,wait_ms,credits_at_us,pending_counts,ncp_at_us,ncp_packets,flags;
    _Atomic uint32_t *tx_owned;
} owner_state;
static uint32_t begin(void){uint32_t v=atomic_load(&owner_state.version);atomic_store(&owner_state.version,v+1u);return v;}
static void end(uint32_t v){atomic_store(&owner_state.version,v+2u);}
void probe_owner_state_init(_Atomic uint32_t *owned){
    atomic_init(&owner_state.version,0);atomic_init(&owner_state.phase,PROBE_OWNER_UNKNOWN);
    atomic_init(&owner_state.phase_at_us,0);atomic_init(&owner_state.wait_ms,UINT32_MAX);
    atomic_init(&owner_state.credits_at_us,0);atomic_init(&owner_state.pending_counts,0);
    atomic_init(&owner_state.ncp_at_us,0);atomic_init(&owner_state.ncp_packets,0);
    atomic_init(&owner_state.flags,owned?PROBE_OWNER_BOUND:0);owner_state.tx_owned=owned;
}
uint32_t probe_owner_state_mark(uint32_t phase,uint32_t at,uint32_t wait){
    uint32_t v=begin(),old=atomic_load(&owner_state.phase),flags=atomic_load(&owner_state.flags);
    if(phase==PROBE_OWNER_RUNTIME)flags=(flags|PROBE_OWNER_IN_RUNTIME)&~PROBE_OWNER_PENDING;
    else if(phase==PROBE_OWNER_PROFILES)flags&=~PROBE_OWNER_IN_RUNTIME;
    atomic_store(&owner_state.phase,phase);atomic_store(&owner_state.phase_at_us,at);
    atomic_store(&owner_state.wait_ms,phase==PROBE_OWNER_SLEEP?wait:UINT32_MAX);
    atomic_store(&owner_state.flags,flags);end(v);return old;
}
void probe_owner_state_credits(uint32_t classic,uint32_t le,uint32_t at){
    uint32_t v=begin(),flags=atomic_load(&owner_state.flags)&~PROBE_OWNER_PENDING;
    if(classic<=UINT16_MAX && le<=UINT16_MAX && !(flags&PROBE_OWNER_IN_RUNTIME)){
        atomic_store(&owner_state.pending_counts,classic|(le<<16));
        atomic_store(&owner_state.credits_at_us,at);flags|=PROBE_OWNER_PENDING;
    }
    atomic_store(&owner_state.flags,flags);end(v);
}
bool probe_owner_state_completed(const uint8_t *p,uint16_t n,uint32_t at){
    if(!p || n<5u || p[0]!=DBT_H4_COMMAND || dbt_read_le16(p+1)!=0x0c35u ||
       n!=(uint16_t)(4u+p[3]) || !p[4] || p[4]>DBT_MAX_LINKS || p[3]!=1u+4u*p[4])return false;
    uint32_t total=0;
    for(unsigned i=0;i<p[4];++i)total+=dbt_read_le16(p+7u+4u*i);
    uint32_t v=begin();atomic_store(&owner_state.ncp_at_us,at);atomic_store(&owner_state.ncp_packets,total);
    atomic_store(&owner_state.flags,atomic_load(&owner_state.flags)|PROBE_OWNER_NCP);end(v);return true;
}
void OWNER_IRAM probe_owner_state_sample(probe_owner_sample_t *s){
    if(!s)return;
    uint32_t before=atomic_load(&owner_state.version);
    s->phase=atomic_load(&owner_state.phase);s->phase_at_us=atomic_load(&owner_state.phase_at_us);
    s->wait_ms=atomic_load(&owner_state.wait_ms);s->credits_at_us=atomic_load(&owner_state.credits_at_us);
    s->pending_counts=atomic_load(&owner_state.pending_counts);
    PROBE_OWNER_TEST_READ_PART(0);
    s->ncp_at_us=atomic_load(&owner_state.ncp_at_us);s->ncp_packets=atomic_load(&owner_state.ncp_packets);
    s->flags=atomic_load(&owner_state.flags);
    s->coherent=!(before&1u) && before==atomic_load(&owner_state.version);
    s->tx_owned=owner_state.tx_owned?atomic_load_explicit(owner_state.tx_owned,memory_order_acquire):UINT32_MAX;
    if(!s->coherent)s->flags&=PROBE_OWNER_BOUND;
}
