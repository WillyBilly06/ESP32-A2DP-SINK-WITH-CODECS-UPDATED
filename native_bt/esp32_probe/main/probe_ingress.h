#ifndef PROBE_INGRESS_H
#define PROBE_INGRESS_H
#include "dbt_hci.h"

/* Optional, bounded ACL-header snapshots. No allocation, logging, waiting,
 * application policy, or key-event capture in the controller callback. */
uint32_t probe_ingress_begin(const uint8_t *data, uint16_t size);
void probe_ingress_end(uint32_t token, const uint8_t *data, uint16_t size);
typedef struct {
    uint32_t before_hash,after_hash,owner_hash,ordinal;
    uint8_t before,after;
    bool after_ready;
} probe_ingress_sample_t;
bool probe_ingress_observe(const dbt_hci_packet_t *packet,probe_ingress_sample_t *sample);
unsigned probe_ingress_bytes(void);
typedef struct {
    uint32_t published,consumed,lost;
    uint16_t snapshot_size,packet_size,snapshot_flags,packet_flags;
    uint8_t reason;
} probe_ingress_fault_t;
/* Owner-only first-failure metadata; headers contain no key-event payloads. */
void probe_ingress_fault(probe_ingress_fault_t *fault);
#endif
