#ifndef DBT_RUNTIME_H
#define DBT_RUNTIME_H

#include "dbt_channels.h"
#include "dbt_rx_flow.h"

typedef enum { DBT_TX_ADMITTED, DBT_TX_LINK_LOST, DBT_TX_ABORTED } dbt_tx_outcome_t;
/* ADMITTED means all fragments copied to HCI transport, never RF delivery. */
typedef void (*dbt_tx_done_t)(void *context, dbt_tx_outcome_t outcome);
typedef struct {
    void *context;
    uint8_t *(*allocate)(void *context, uint16_t bytes);
    void (*release)(void *context, uint8_t *data);
    dbt_channel_hooks_t channels;
    void (*link)(void *context, const dbt_link_change_t *change);
    dbt_observer_t packet;
    /* Borrowed SCO frame/status, owner only. The application must validate its
     * synchronous handle and copy data for a voice worker before returning. */
    dbt_observer_t sco;
} dbt_runtime_hooks_t;

typedef struct {
    dbt_l2cap_tx_t pdu;
    dbt_tx_done_t done;
    void *context;
    uint16_t local_cid;
    bool active,credit_channel,frame_ready;
} dbt_runtime_tx_t;

typedef enum { DBT_RUNTIME_FAULT_NONE, DBT_RUNTIME_FAULT_ACL_EVENT,
    DBT_RUNTIME_FAULT_L2CAP, DBT_RUNTIME_FAULT_CHANNEL_PDU,
    DBT_RUNTIME_FAULT_RX_FLOW } dbt_runtime_fault_origin_t;
/* First owner-side parser failure only. Header dimensions/control identifiers
 * are diagnostic metadata; no audio, application values or key bytes retained. */
typedef struct {
    uint32_t at_ms, generation;
    uint16_t handle, cid, bytes, declared;
    uint8_t origin, result, packet_type, event, pb, bc, code, identifier;
    bool link_present;
} dbt_runtime_failure_t;

typedef struct {
    dbt_host_t host;
    dbt_rx_flow_t rx_flow;
    dbt_acl_t acl;
    dbt_l2cap_rx_t rx;
    dbt_channels_t channels;
    dbt_runtime_hooks_t hooks;
    dbt_runtime_failure_t failure;
    dbt_runtime_tx_t tx[DBT_MAX_LINKS];
    dbt_l2cap_tx_t control;
    uint32_t now_ms;
    uint16_t maximum_mtu;
    uint8_t next_link;
    bool ready, control_active, in_poll, cleared;
    uint8_t tx_packet[DBT_MAX_H4_BYTES];
} dbt_runtime_t;

/* One owner for every call except receive. Allocator is used only for fragmented RX PDUs, active
 * LE channel metadata and fragmented LE SDUs. There are no permanent per-codec or per-link payload pools.
 * Hooks are borrowed/nonblocking; poll is never reentrant. */
dbt_result_t dbt_runtime_init(dbt_runtime_t *runtime, dbt_transport_t transport,
    dbt_runtime_hooks_t hooks, uint16_t maximum_mtu, uint32_t command_timeout_ms);
dbt_result_t dbt_runtime_start(dbt_runtime_t *runtime, uint32_t now_ms);
dbt_result_t dbt_runtime_enable_rx_flow(dbt_runtime_t *runtime, uint16_t packet_bytes,
                                       uint16_t packet_count);
dbt_result_t dbt_runtime_receive(dbt_runtime_t *runtime, const uint8_t *data, uint16_t size);
void dbt_runtime_poll(dbt_runtime_t *runtime, uint32_t now_ms, uint16_t rx_budget, uint16_t tx_budget);
uint32_t dbt_runtime_wait_ms(const dbt_runtime_t *runtime, uint32_t now_ms);
/* One borrowed outbound SDU per connection. Keep it unchanged until done, even
 * during controller credit stalls. Channel callbacks may enqueue fixed replies;
 * dynamic channel operations follow the channel manager's non-reentrant rules. */
dbt_result_t dbt_runtime_send(dbt_runtime_t *runtime, uint16_t handle, uint16_t local_cid,
    const uint8_t *data, uint16_t size, dbt_tx_done_t done, void *context);
dbt_result_t dbt_runtime_send_fixed(dbt_runtime_t *runtime, uint16_t handle, uint16_t cid,
    const uint8_t *data, uint16_t size, dbt_tx_done_t done, void *context);
/* Host owner only; release all borrowed SDUs and partial RX allocations. */
void dbt_runtime_abort(dbt_runtime_t *runtime, dbt_fault_t fault);

#endif
