#ifndef DBT_AVCTP_H
#define DBT_AVCTP_H

#include "dbt_queue.h"

/* One owner per channel. All storage is supplied by the caller and remains
 * exclusive to this object until clear succeeds. No allocation at init. */
typedef struct {
    const uint8_t *data;
    uint16_t size, pid;
    uint8_t label;
    bool response, invalid_pid;
} dbt_avctp_message_t;
/* Data is borrowed only during this callback. Return true on a matched interim
 * response to retain the local transaction; false completes it. The profile
 * must validate response identity before accepting or completing it. */
typedef bool (*dbt_avctp_message_fn)(void *, const dbt_avctp_message_t *);
typedef struct {
    void *context;
    dbt_avctp_message_fn message;
    /* Optional owner callback once a whole AVCTP message is admitted by the
     * bearer, never evidence of peer receipt. No blocking work. */
    void (*transmitted)(void *, uint8_t label, bool response, uint32_t now);
} dbt_avctp_hooks_t;
typedef struct {
    dbt_avctp_hooks_t hooks;
    uint8_t *tx, *rx, *packet;
    uint32_t binding, tx_deadline, rx_deadline;
    uint32_t response_deadline[16], response_timeout[16];
    uint16_t local_pending, local_sent;
    uint32_t transmit_ms, response_ms;
    uint16_t tx_capacity, rx_capacity, tx_mtu, rx_mtu;
    uint16_t tx_size, tx_offset, packet_size, packet_body, rx_size, rx_pid, tx_pid;
    uint8_t tx_label, tx_count, tx_index, next_label;
    uint8_t rx_label, rx_remaining;
    bool browsing, tx_used, tx_response, tx_ipid, pinned;
    bool rx_active, rx_response, failed, in_callback;
} dbt_avctp_t;

dbt_result_t dbt_avctp_init(dbt_avctp_t *, dbt_avctp_hooks_t, uint32_t binding,
    uint16_t rx_mtu, uint16_t tx_mtu, bool browsing,
    uint8_t *rx, uint16_t rx_capacity, uint8_t *tx, uint16_t tx_capacity,
    uint8_t *packet, uint32_t transmit_ms, uint32_t response_ms);
/* packet must point to at least tx_mtu bytes; all three buffers must not
 * overlap. Owner must keep them alive until clear and all borrows return. */
/* Copies bytes. Up to sixteen local transactions are outstanding. Responses may be
 * queued while local transactions wait. Response labels originate at peer. */
dbt_result_t dbt_avctp_command(dbt_avctp_t *, const uint8_t *, uint16_t,
    uint32_t now, uint8_t *label);
dbt_result_t dbt_avctp_respond(dbt_avctp_t *, uint8_t label, bool invalid_pid,
    const uint8_t *, uint16_t, uint32_t now);
/* Echo the rejected profile identifier in an IPID response. */
dbt_result_t dbt_avctp_reject_pid(dbt_avctp_t *, uint8_t label, uint16_t pid, uint32_t now);
dbt_result_t dbt_avctp_receive(dbt_avctp_t *, uint32_t binding,
    const uint8_t *, uint16_t, uint32_t now);
/* Set a label timeout in ms. Zero permits a long-lived notification. Call
 * before final TX commit or from an interim-response callback. A timeout
 * closes the entire channel rather than recycling an ambiguous label. */
dbt_result_t dbt_avctp_transaction_timeout(dbt_avctp_t *, uint8_t label, uint32_t ms, uint32_t now);
/* Peek pins until commit or cancel, including while runtime borrows the bytes. */
dbt_result_t dbt_avctp_tx_peek(dbt_avctp_t *, const uint8_t **, uint16_t *);
dbt_result_t dbt_avctp_tx_commit(dbt_avctp_t *, uint32_t now);
dbt_result_t dbt_avctp_tx_cancel(dbt_avctp_t *);
dbt_result_t dbt_avctp_poll(dbt_avctp_t *, uint32_t now);
uint32_t dbt_avctp_wait_ms(const dbt_avctp_t *, uint32_t now);
dbt_result_t dbt_avctp_clear(dbt_avctp_t *);
#endif
