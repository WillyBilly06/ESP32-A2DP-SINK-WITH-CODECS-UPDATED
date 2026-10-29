#ifndef PROBE_SMP_H
#define PROBE_SMP_H

#include "dbt_smp_session.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* One active SMP procedure; all functions except the worker are host-owner only.
 * The local address must be the actual address used to establish this link.
 * Configure and register CID 6 before advertising or initiating LE links. */
typedef struct {
    dbt_smp_session_config_t pairing;
    uint8_t local_address_type;
    uint8_t local_address[6];
    /* Request only the peer's identity keys, with role-correct key masks. */
    bool peer_identity_only;
} probe_smp_config_t;

dbt_result_t probe_smp_init(dbt_runtime_t *, TaskHandle_t, const probe_smp_config_t *);
/* A fresh application-supplied six-digit value is consumed by one procedure.
 * Only use with a real UI matching the configured I/O capability. */
dbt_result_t probe_smp_set_passkey(uint32_t passkey);
/* Forward from the shared runtime's fixed-data hook only for LE CID 6.
 * A non-OK return means that PDU was not accepted; the caller must fail the
 * affected SMP procedure explicitly. Other CIDs still go to their owners. */
dbt_result_t probe_smp_receive(const dbt_link_t *, uint16_t cid, const uint8_t *, uint16_t);
/* Central initiation is explicit. Peripheral requests arrive via receive. */
dbt_result_t probe_smp_start(const dbt_link_t *);
/* Explicitly start encryption on an existing central-role LE connection using
 * its stored peer LTK. Never initiates a connection or falls back to pairing. */
dbt_result_t probe_smp_reconnect_start(const dbt_link_t *);
void probe_smp_link(const dbt_link_change_t *);
/* true claims an LE LTK Request and suppresses the GAP negative reply. With
 * bonding enabled this may start a peripheral reconnect for a public/static
 * peer, loading its local committed record on the worker. Encryption Change
 * is observed but is not claimed from other consumers. */
bool probe_smp_event(const dbt_hci_packet_t *);
void probe_smp_poll(uint32_t now_ms);
uint32_t probe_smp_wait_ms(uint32_t now_ms);
/* Reports a failed procedure once; policy decides whether to disconnect. */
bool probe_smp_take_failure(uint16_t *handle, uint32_t *generation);
dbt_result_t probe_smp_security(uint16_t handle, uint32_t generation,
                                dbt_smp_session_security_t *);
dbt_result_t probe_smp_numeric_value(uint32_t *);
dbt_result_t probe_smp_numeric_confirm(bool accepted, uint32_t now_ms);
/* Explicit maintenance; never automatic. Starts only when no SMP procedure is
 * active. Results are consumed after the worker has retired. Inspection gives
 * the pending intent identity and operation. Reconcile verifies the exact
 * before/after record and never rewrites it. status reports provider outcome;
 * applied is meaningful only for successful reconciliation. */
dbt_result_t probe_smp_storage_inspect_start(void);
dbt_result_t probe_smp_storage_reconcile_start(dbt_smp_bond_op_t,uint8_t,const uint8_t[6]);
dbt_result_t probe_smp_storage_result(dbt_result_t *status,dbt_smp_bond_op_t *operation,
                                     uint8_t *identity_type,uint8_t identity[6],bool *applied);

#endif
