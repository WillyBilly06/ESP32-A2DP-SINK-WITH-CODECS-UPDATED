#ifndef DBT_AVDTP_H
#define DBT_AVDTP_H

#include "dbt_hci.h"

enum {
    DBT_AVDTP_DISCOVER = 1, DBT_AVDTP_GET_CAPABILITIES = 2,
    DBT_AVDTP_SET_CONFIGURATION = 3, DBT_AVDTP_GET_CONFIGURATION = 4,
    DBT_AVDTP_RECONFIGURE = 5, DBT_AVDTP_OPEN = 6, DBT_AVDTP_START = 7,
    DBT_AVDTP_CLOSE = 8, DBT_AVDTP_SUSPEND = 9, DBT_AVDTP_ABORT = 10,
    DBT_AVDTP_SECURITY_CONTROL = 11, DBT_AVDTP_GET_ALL_CAPABILITIES = 12,
    DBT_AVDTP_DELAY_REPORT = 13
};
typedef enum { DBT_AVDTP_COMMAND, DBT_AVDTP_GENERAL_REJECT,
               DBT_AVDTP_ACCEPT, DBT_AVDTP_REJECT } dbt_avdtp_type_t;
typedef enum { DBT_AVDTP_RESPONSE, DBT_AVDTP_TIMEOUT,
               DBT_AVDTP_CANCELLED } dbt_avdtp_outcome_t;

typedef struct {
    const uint8_t *data;
    uint16_t size;
    uint8_t label, signal;
    dbt_avdtp_type_t type;
} dbt_avdtp_message_t;

typedef struct {
    void *context;
    uint8_t *(*allocate)(void *context, uint16_t size);
    void (*release)(void *context, uint8_t *bytes);
    /* Command payload is borrowed only during this callback. respond() is
     * permitted here, including a later owner-thread response for worker work.
     * Decoder allocation/work must run off the host owner. */
    void (*command)(void *context, const dbt_avdtp_message_t *message);
    /* A response is borrowed; message is NULL on timeout/cancellation. */
    void (*complete)(void *context, dbt_avdtp_outcome_t outcome,
                     const dbt_avdtp_message_t *message);
    /* Full response admitted to L2CAP/HCI; never proof of peer receipt. */
    void (*responded)(void *context, uint8_t label, uint8_t signal,
                      dbt_avdtp_type_t type);
    /* Optional owner policy for an Abort received while a prior remote command
     * awaits its worker and has no response queued/pinned. True must invalidate
     * old application work for the same SEP before the Abort command callback.
     * Return false without changing state for unrelated/invalid Abort requests. */
    bool (*interrupt)(void *context, const dbt_avdtp_message_t *abort_command);
} dbt_avdtp_hooks_t;

typedef struct {
    uint8_t *data;
    uint32_t deadline;
    uint16_t size, offset;
    uint8_t label, signal, type, count, index;
    bool used, waiting;
    uint8_t small[8];
} dbt_avdtp_tx_t;

typedef struct {
    dbt_avdtp_hooks_t hooks;
    dbt_avdtp_tx_t tx[2]; /* local command and remote response */
    uint8_t *rx, *packet;
    uint32_t binding, now_ms, rx_deadline, remote_deadline, transmit_ms, response_ms;
    uint32_t discarded;
    uint16_t rx_size, rx_capacity, rx_packet_size, packet_size;
    uint16_t tx_mtu, rx_mtu, maximum_message;
    uint8_t rx_label, rx_signal, rx_type, rx_remaining;
    uint8_t next_label, remote_label, remote_signal, selected;
    bool remote_pending, pinned, selected_valid, failed, in_callback;
    uint8_t small_packet[16];
} dbt_avdtp_t;

/* One instance per signaling channel, one owner, one local transaction plus one
 * remote command at a time. binding is a nonzero channel-lifetime ID. MTUs >=48.
 * max_message bounds the message BODY (excluding signaling headers). No dynamic
 * allocation at init; no codec state or media buffering belongs to this object.
 * Initializing a live/pinned instance is forbidden: first quiesce and clear it. */
dbt_result_t dbt_avdtp_init(dbt_avdtp_t *signal, dbt_avdtp_hooks_t hooks,
    uint32_t binding, uint16_t rx_mtu, uint16_t tx_mtu, uint16_t maximum_message,
    uint32_t transmit_ms, uint32_t response_ms);

/* Copies the body before returning. A successful call reserves a transaction;
 * it does not establish a stream. Small common controls allocate nothing. */
dbt_result_t dbt_avdtp_command(dbt_avdtp_t *signal, uint8_t command,
    const uint8_t *body, uint16_t size, uint32_t now_ms, uint8_t *label);
dbt_result_t dbt_avdtp_respond(dbt_avdtp_t *signal, uint8_t label, uint8_t command,
    dbt_avdtp_type_t type, const uint8_t *body, uint16_t size, uint32_t now_ms);
/* Invalid Abort SEIDs require no response. Retire that exact remote Abort only,
 * without leaving an unanswered-command timer or fabricating an Abort Reject.
 * Permitted from the command callback before a response has been queued. */
dbt_result_t dbt_avdtp_ignore_abort(dbt_avdtp_t *signal, uint8_t label);

/* Exactly one complete signaling L2CAP SDU, not an ACL fragment. Stale binding
 * and unmatched responses return EMPTY. INVALID means malformed/discarded;
 * BUSY means a pipelined command was not accepted. Never silently retry it. */
dbt_result_t dbt_avdtp_receive(dbt_avdtp_t *signal, uint32_t binding,
    const uint8_t *packet, uint16_t size, uint32_t now_ms);

/* Peek pins one immutable signaling SDU until commit/cancel, including across
 * every underlying ACL fragment. Commit only after all those fragments are
 * admitted. cancel returns the pin without advancing; a live retry is identical.
 * Fragmented messages are never interleaved with another outgoing message. */
dbt_result_t dbt_avdtp_tx_peek(dbt_avdtp_t *signal, const uint8_t **packet, uint16_t *size);
dbt_result_t dbt_avdtp_tx_commit(dbt_avdtp_t *signal, uint32_t now_ms);
dbt_result_t dbt_avdtp_tx_cancel(dbt_avdtp_t *signal);
dbt_result_t dbt_avdtp_poll(dbt_avdtp_t *signal, uint32_t now_ms);
uint32_t dbt_avdtp_wait_ms(const dbt_avdtp_t *signal, uint32_t now_ms);
/* Fails closed after any transaction/reassembly deadline. Caller must close the
 * signaling channel, quiesce its transport and clear. clear returns BUSY while
 * a view is pinned; it must never release bytes still borrowed by the runtime. */
dbt_result_t dbt_avdtp_clear(dbt_avdtp_t *signal);

/* Bounded RTP view for the unmultiplexed, uncompressed media transport service.
 * Handles CSRCs, extensions and padding; leaves codec-specific payload intact.
 * A successful parse is not a decoded-frame or continuity claim. */
typedef struct {
    const uint8_t *payload;
    uint32_t timestamp, ssrc;
    uint16_t size, sequence;
    uint8_t payload_type;
    bool marker;
} dbt_avdtp_media_t;
dbt_result_t dbt_avdtp_media_decode(const uint8_t *packet, uint16_t size,
                                   dbt_avdtp_media_t *media);

#endif
