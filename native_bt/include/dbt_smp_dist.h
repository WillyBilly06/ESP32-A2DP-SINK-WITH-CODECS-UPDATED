#ifndef DBT_SMP_DIST_H
#define DBT_SMP_DIST_H
#include "dbt_queue.h"
/* Offline phase-3 engine. Masks: 1 legacy LTK+EDIV/Rand, 2 IRK+identity,
 * 4 CSRK. LE SC LTK is generated in phase 2 and never distributed here.
 * All byte arrays are SMP/HCI order. Identity received from the peer is
 * unverified until reconnection proves it; do not key a bond solely by it. */
typedef struct {
    uint8_t ltk[16], rand[8], irk[16], csrk[16], identity[6];
    uint16_t ediv;
    uint8_t identity_type;
} dbt_smp_dist_keys_t;
typedef struct {
    uint32_t generation;
    dbt_smp_dist_keys_t local,peer;
    uint8_t local_mask,peer_mask,key_size,step,phase,tx_size,tx[17];
    bool initiator,secure_connections,failed,taken;
    uint32_t tx_serial,final_serial,hci_buffers_completed,local_pdus_admitted;
    bool final_wait_ack,final_ll_acked,peer_final_received,early_final_acked;
} dbt_smp_dist_t;
/* The caller supplies freshly generated local keys and must have observed a
 * correlated encryption event using this pairing's phase-2 key. */
dbt_result_t dbt_smp_dist_open(dbt_smp_dist_t *s,uint32_t generation,bool initiator,
    bool secure_connections,uint8_t key_size,uint8_t local_mask,uint8_t peer_mask,
    bool encryption_verified,const dbt_smp_dist_keys_t *local);
void dbt_smp_dist_close(dbt_smp_dist_t *s);
dbt_result_t dbt_smp_dist_tx(dbt_smp_dist_t *s,const uint8_t **p,uint8_t *n);
/* Admission means all fragments of this PDU were accepted by the ordered,
 * non-automatically-flushable transport. It is never RF or peer evidence.
 * DBT_OK means the callback was accepted; admitted=false fails the procedure. */
dbt_result_t dbt_smp_dist_tx_token(const dbt_smp_dist_t *s,uint32_t *serial);
dbt_result_t dbt_smp_dist_tx_admitted(dbt_smp_dist_t *s,uint32_t generation,uint32_t serial,bool admitted);
/* Only the sender of the globally final key waits here. A provider may report
 * final acknowledgment only when it can prove every link fragment carrying
 * that key was acknowledged; an HCI credit or VHCI send-ready is insufficient.
 * DBT_OK means the event was accepted; acknowledged=false fails the procedure. */
dbt_result_t dbt_smp_dist_final_pending(const dbt_smp_dist_t *s,uint32_t *serial);
dbt_result_t dbt_smp_dist_final_ack(dbt_smp_dist_t *s,uint32_t generation,uint32_t serial,bool acknowledged);
/* Diagnostic HCI buffer completion counts do not advance phase 3. The caller
 * must correlate the counts to this link/generation and avoid double counting. */
dbt_result_t dbt_smp_dist_hci_buffers_completed(dbt_smp_dist_t *s,uint32_t generation,uint16_t count);
dbt_result_t dbt_smp_dist_receive(dbt_smp_dist_t *s,const uint8_t *p,uint8_t n);
dbt_result_t dbt_smp_dist_encryption_lost(dbt_smp_dist_t *s,uint32_t generation);
dbt_result_t dbt_smp_dist_take(dbt_smp_dist_t *s,dbt_smp_dist_keys_t *peer);
#endif
