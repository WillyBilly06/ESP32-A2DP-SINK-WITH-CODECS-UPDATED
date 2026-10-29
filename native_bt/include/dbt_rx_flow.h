#ifndef DBT_RX_FLOW_H
#define DBT_RX_FLOW_H

#include "dbt_acl.h"

typedef enum {
    DBT_RX_FLOW_DISABLED,
    DBT_RX_FLOW_BUFFER,
    DBT_RX_FLOW_ENABLE,
    DBT_RX_FLOW_ACTIVE,
    DBT_RX_FLOW_FAILED
} dbt_rx_flow_state_t;

typedef struct {
    uint32_t generation;
    uint16_t handle, packets;
    bool used;
} dbt_rx_completed_t;

typedef struct {
    dbt_host_t *host;
    dbt_acl_t *acl;
    dbt_rx_completed_t pending[DBT_MAX_LINKS];
    uint32_t acknowledged, commands_submitted, retired_on_disconnect;
    uint16_t packet_bytes, packet_count;
    uint16_t failed_opcode;
    uint8_t failed_status;
    dbt_rx_flow_state_t state;
    bool command_pending;
    uint8_t tx[5u + 4u * DBT_MAX_LINKS];
} dbt_rx_flow_t;

/* Initialize before transport use; never clear/reinitialize retained TX storage.
 * One host owner. ACL receive flow is enabled. One maximum-size SCO HCI frame
 * also fits in the shared FIFO, for controllers requiring nonzero SCO fields;
 * synchronous flow control and voice connection/profile ownership are separate. */
dbt_result_t dbt_rx_flow_init(dbt_rx_flow_t *flow, dbt_host_t *host, dbt_acl_t *acl);
/* Start before accepting any links. Capacity must fit the existing H4 FIFO plus
 * its event reserve. ACTIVE requires actual Host Buffer Size / Set Flow replies. */
dbt_result_t dbt_rx_flow_start(dbt_rx_flow_t *flow, uint16_t packet_bytes, uint16_t packet_count);
void dbt_rx_flow_poll(dbt_rx_flow_t *flow, uint32_t now_ms);
/* Call after consuming the borrowed HCI packet, including profile callbacks.
 * Each packet is counted once, for its live connection generation. */
dbt_result_t dbt_rx_flow_completed(dbt_rx_flow_t *flow, const dbt_link_t *link);
void dbt_rx_flow_link_change(dbt_rx_flow_t *flow, const dbt_link_change_t *change);
/* Observes a possible error reply to the normally unacknowledged 0x0c35 command. */
void dbt_rx_flow_event(dbt_rx_flow_t *flow, const dbt_hci_packet_t *packet);
uint32_t dbt_rx_flow_wait_ms(const dbt_rx_flow_t *flow);
/* Fail closed without modifying a packet still retained by transport. */
void dbt_rx_flow_abort(dbt_rx_flow_t *flow);

#endif
