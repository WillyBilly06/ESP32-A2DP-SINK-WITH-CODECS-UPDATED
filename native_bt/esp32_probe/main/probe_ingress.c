#include "probe_ingress.h"
#include <stdatomic.h>

#define SLOTS 64u
typedef struct {
    uint16_t size;
    uint8_t prefix[6];
    uint32_t before_hash;
    atomic_uint after,after_hash;
} snapshot_t;
static snapshot_t snapshots[SLOTS];
static atomic_uint published, consumed, lost;
static bool invalid;
static probe_ingress_fault_t fault;
static uint32_t fingerprint(const uint8_t *data,uint16_t size) {
    uint32_t value=2166136261u;
    for(unsigned i=0;i<size;++i)value=(value^data[i])*16777619u;
    return value;
}
static void invalidate(uint8_t reason) {
    if(invalid)return;
    fault.reason=reason;
    fault.published=atomic_load_explicit(&published,memory_order_acquire);
    fault.consumed=atomic_load_explicit(&consumed,memory_order_relaxed);
    fault.lost=atomic_load_explicit(&lost,memory_order_acquire);
    invalid=true;
}

uint32_t probe_ingress_begin(const uint8_t *data, uint16_t size) {
    if (size < 6 || size>DBT_MAX_H4_BYTES || data[0] != DBT_H4_ACL) return UINT32_MAX;
    uint32_t write = atomic_load_explicit(&published, memory_order_relaxed);
    uint32_t read = atomic_load_explicit(&consumed, memory_order_acquire);
    if (write - read >= SLOTS) {
        atomic_fetch_add_explicit(&lost, 1, memory_order_relaxed);
        return UINT32_MAX;
    }
    snapshot_t *slot = &snapshots[write & (SLOTS - 1u)];
    slot->size = size;
    for (unsigned i = 0; i < 6; ++i) slot->prefix[i] = data[i];
    slot->before_hash=fingerprint(data+5,(uint16_t)(size-5));
    atomic_store_explicit(&slot->after, 0, memory_order_relaxed);
    /* Publish before the main H4 queue so the owner can always match a header.
     * Post-copy sampling may still be pending; the owner never waits for it. */
    atomic_store_explicit(&published, write + 1u, memory_order_release);
    return write;
}
void probe_ingress_end(uint32_t token, const uint8_t *data, uint16_t size) {
    if (token == UINT32_MAX || size < 6) return;
    atomic_store_explicit(&snapshots[token&(SLOTS-1u)].after_hash,
                          fingerprint(data+5,(uint16_t)(size-5)),memory_order_relaxed);
    atomic_store_explicit(&snapshots[token & (SLOTS - 1u)].after,
                          0x100u | data[5], memory_order_release);
}
bool probe_ingress_observe(const dbt_hci_packet_t *packet,probe_ingress_sample_t *sample) {
    if (packet->type != DBT_H4_ACL || !sample) return false;
    if(invalid)return false;
    if(atomic_load_explicit(&lost,memory_order_acquire)){invalidate(1);return false;}
    uint32_t read = atomic_load_explicit(&consumed, memory_order_relaxed);
    uint32_t write = atomic_load_explicit(&published, memory_order_acquire);
    if (read == write) { invalidate(2); return false; }
    const snapshot_t *slot = &snapshots[read & (SLOTS - 1u)];
    uint16_t flags = dbt_read_le16(slot->prefix + 1);
    bool matches = slot->size == 5u + packet->size &&
        dbt_read_le16(slot->prefix + 3) == packet->size &&
        (flags & 0xfffu) == packet->handle && ((flags >> 12) & 3u) == packet->pb;
    sample->before = slot->prefix[5];sample->before_hash=slot->before_hash;sample->ordinal=read;
    sample->owner_hash=fingerprint(packet->payload,packet->size);
    uint32_t post = atomic_load_explicit(&slot->after, memory_order_acquire);
    sample->after_ready = (post & 0x100u) != 0;sample->after = (uint8_t)post;
    sample->after_hash=sample->after_ready?atomic_load_explicit(&slot->after_hash,memory_order_relaxed):0;
    atomic_store_explicit(&consumed, read + 1u, memory_order_release);
    if (!matches) {
        fault.snapshot_size=slot->size;fault.packet_size=packet->size;
        fault.snapshot_flags=flags;
        fault.packet_flags=(uint16_t)(packet->handle | (packet->pb<<12) | (packet->bc<<14));
        invalidate(3);
    }
    return matches;
}
unsigned probe_ingress_bytes(void) {
    return sizeof(snapshots) + sizeof(published) + sizeof(consumed) + sizeof(lost) + sizeof(invalid) + sizeof(fault);
}
void probe_ingress_fault(probe_ingress_fault_t *out) { if(out)*out=fault; }
