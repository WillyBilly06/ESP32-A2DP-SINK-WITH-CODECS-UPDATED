#ifndef DBT_L2CAP_H
#define DBT_L2CAP_H

#include "dbt_acl.h"

typedef struct {
    void *context;
    uint8_t *(*allocate)(void *context, uint16_t bytes);
    void (*release)(void *context, uint8_t *data);
    /* Borrowed data, valid only during this callback on the host owner. */
    void (*pdu)(void *context, uint16_t handle, uint32_t generation,
                uint16_t cid, const uint8_t *data, uint16_t bytes);
} dbt_l2cap_hooks_t;
typedef struct {
    uint8_t *data;
    uint32_t generation;
    uint32_t deadline;
    uint16_t handle, cid, expected, received;
} dbt_l2cap_partial_t;
typedef struct {
    dbt_l2cap_partial_t partial[DBT_MAX_LINKS];
    dbt_l2cap_hooks_t hooks;
    uint32_t timeout_ms;
    uint32_t delivered, abandoned;
    uint16_t max_pdu_payload;
} dbt_l2cap_rx_t;
typedef struct {
    const uint8_t *data;
    uint32_t generation, cursor;
    uint16_t handle, cid, size, pending;
    uint8_t prefix[2],prefix_size;
} dbt_l2cap_tx_t;

/* Basic PDU framing only. CID negotiation, signaling/security and channel modes
 * are separate layers; these functions do not establish an L2CAP channel. */
dbt_result_t dbt_l2cap_rx_init(dbt_l2cap_rx_t *rx, dbt_l2cap_hooks_t hooks,
                              uint16_t max_payload, uint32_t timeout_ms);
dbt_result_t dbt_l2cap_receive(dbt_l2cap_rx_t *rx, const dbt_acl_t *acl,
                              const dbt_hci_packet_t *packet, uint32_t now_ms);
void dbt_l2cap_link_change(dbt_l2cap_rx_t *rx, const dbt_link_change_t *change);
unsigned dbt_l2cap_expire(dbt_l2cap_rx_t *rx, uint32_t now_ms);
void dbt_l2cap_rx_clear(dbt_l2cap_rx_t *rx);
dbt_result_t dbt_l2cap_tx_begin(dbt_l2cap_tx_t *tx, const dbt_link_t *link,
                               uint16_t cid, const uint8_t *data, uint16_t size);
/* Optional copied SDU-length prefix plus borrowed K-frame payload. */
dbt_result_t dbt_l2cap_tx_begin_credit(dbt_l2cap_tx_t *tx,const dbt_link_t *link,
    uint16_t cid,const uint8_t *data,uint16_t size,bool first,uint16_t sdu_length);
/* Peek is repeatable under backpressure. Commit only after dbt_acl_submit OK. */
dbt_result_t dbt_l2cap_tx_peek(dbt_l2cap_tx_t *tx, const dbt_acl_t *acl,
                              uint8_t *output, uint16_t capacity, uint16_t *written);
dbt_result_t dbt_l2cap_tx_commit(dbt_l2cap_tx_t *tx);
/* Gated convenience path: output must be this transport's reusable TX storage. */
dbt_result_t dbt_l2cap_send_next(dbt_l2cap_tx_t *tx, dbt_acl_t *acl,
                                dbt_transport_t transport, uint8_t *output, uint16_t capacity);

#endif
