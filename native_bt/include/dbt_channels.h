#ifndef DBT_CHANNELS_H
#define DBT_CHANNELS_H

#include "dbt_l2cap.h"

#define DBT_CHANNEL_LIMIT 8u
#define DBT_SERVICE_LIMIT 8u
#define DBT_SIGNAL_BYTES 48u
#define DBT_SIGNAL_TX_SLOTS 8u
#define DBT_SIGNAL_RX_SLOTS 4u

typedef enum { DBT_CH_FREE, DBT_CH_CONNECTING, DBT_CH_CONFIGURING,
               DBT_CH_OPEN, DBT_CH_DISCONNECTING, DBT_CH_SECURING } dbt_channel_state_t;
typedef enum { DBT_SECURITY_NONE, DBT_SECURITY_ENCRYPTED,
               DBT_SECURITY_AUTHENTICATED } dbt_security_t;
typedef enum { DBT_CH_EVENT_OPEN, DBT_CH_EVENT_CLOSED, DBT_CH_EVENT_FAILED,
               DBT_CH_EVENT_RECONFIGURED } dbt_channel_event_t;
typedef enum { DBT_CH_REASON_NORMAL, DBT_CH_REASON_REFUSED, DBT_CH_REASON_PROTOCOL,
               DBT_CH_REASON_LOCAL_TIMEOUT, DBT_CH_REASON_RESPONSE_TIMEOUT,
               DBT_CH_REASON_LINK_LOST, DBT_CH_REASON_SECURITY } dbt_channel_reason_t;

typedef struct {
    uint16_t psm, rx_mtu, minimum_mtu;
    dbt_security_t security;
} dbt_service_t;

typedef struct {
    uint32_t serial, link_generation, deadline, pending_limit, config_limit;
    uint16_t handle, local_cid, remote_cid, psm, rx_mtu, rx_capacity, minimum_mtu, tx_mtu;
    uint16_t peer_mtu_staged, config_epoch;
    dbt_channel_state_t state;
    dbt_security_t security;
    uint8_t request_code, request_id, incoming_connect_id, attempts;
    uint8_t cached_request_size, cached_reply_size;
    bool request_sent, pending_extended, connection_ready, local_configured, peer_configured;
    bool incoming_more, incoming_failed, response_more, response_failed, reconfiguring, opened_once;
    bool security_incoming, security_pending_sent;
    uint8_t cached_request[DBT_SIGNAL_BYTES], cached_reply[DBT_SIGNAL_BYTES];
} dbt_channel_t;

typedef struct {
    void *context;
    /* Required for authenticated policy; encryption alone is not authentication.
     * Optional for NONE/ENCRYPTED. False refuses a channel before allocation. */
    bool (*authorize)(void *context, const dbt_link_t *link, uint16_t psm, dbt_security_t security);
    /* Optional async provider. start=true requests work once; false only reads
     * status. BUSY means pending, OK means ready, anything else refuses. The
     * authorize callback and actual encryption are checked before success.
     * Wake the owner when provider status changes outside runtime_poll. */
    dbt_result_t (*security_request)(void *context, const dbt_link_t *link, uint16_t psm,
                                     dbt_security_t security, bool start);
    /* A reconfiguration may start only after the current outgoing SDU is done.
     * The application must provide this if it transmits channel data. */
    bool (*tx_idle)(void *context, uint16_t handle, uint16_t local_cid);
    void (*event)(void *context, dbt_channel_event_t event,
                   const dbt_channel_t *snapshot, dbt_channel_reason_t reason, uint16_t result);
    void (*data)(void *context, const dbt_channel_t *channel, const uint8_t *data, uint16_t size);
    void (*fixed_data)(void *context, const dbt_link_t *link, uint16_t cid,
                       const uint8_t *data, uint16_t size);
    /* True only when policy accepts and the controller update was queued. */
    bool (*le_parameters)(void *context, const dbt_link_t *link, uint16_t minimum,
                          uint16_t maximum, uint16_t latency, uint16_t timeout);
    /* New incoming Classic channel only, after duplicate/CID checks and before
     * allocation/security work. Pure bounded admission policy, no mutations of
     * this manager. It does not replace authorization or encryption checks. */
    bool (*accept)(void *context, const dbt_link_t *link, uint16_t psm);
} dbt_channel_hooks_t;

typedef struct {
    uint32_t generation;
    uint16_t handle;
    uint8_t next_id;
    bool used;
} dbt_signal_link_t;
typedef struct {
    uint32_t generation, serial;
    uint16_t handle, cid, local_cid, config_epoch, peer_mtu;
    uint8_t size, effect;
    bool valid;
    uint8_t bytes[DBT_SIGNAL_BYTES];
} dbt_signal_tx_t;
typedef struct {
    uint32_t generation;
    uint16_t handle, cid;
    uint8_t size, cursor;
    bool oversized;
    uint8_t bytes[DBT_SIGNAL_BYTES];
} dbt_signal_rx_t;
typedef struct {
    uint16_t handle, cid, size;
    uint32_t generation;
    const uint8_t *data;
} dbt_signal_view_t;

typedef struct {
    dbt_acl_t *acl;
    dbt_channel_hooks_t hooks;
    dbt_service_t services[DBT_SERVICE_LIMIT];
    dbt_channel_t channels[DBT_CHANNEL_LIMIT];
    dbt_signal_link_t links[DBT_MAX_LINKS];
    dbt_signal_tx_t tx[DBT_SIGNAL_TX_SLOTS];
    dbt_signal_rx_t rx[DBT_SIGNAL_RX_SLOTS];
    uint64_t fixed_mask[2];
    uint32_t serial, rtx_ms, ertx_ms, now_ms;
    uint32_t stale_pdus, unknown_responses, malformed_pdus;
    uint16_t maximum_mtu;
    uint8_t tx_head, tx_count, rx_head, rx_count;
    bool tx_locked, in_callback, rx_deferred;
} dbt_channels_t;

/* Single host owner. Callbacks are borrowed, nonblocking and non-reentrant.
 * Basic Classic channels are implemented here. ERTM/LE credit channels remain
 * separate work; neither is advertised by the information response. */
dbt_result_t dbt_channels_init(dbt_channels_t *manager, dbt_acl_t *acl,
                               dbt_channel_hooks_t hooks, uint16_t maximum_mtu,
                               uint32_t rtx_ms, uint32_t ertx_ms);
dbt_result_t dbt_channels_register(dbt_channels_t *manager, dbt_service_t service);
dbt_result_t dbt_channels_register_fixed(dbt_channels_t *manager, dbt_link_kind_t kind, uint16_t cid);
dbt_result_t dbt_channels_open(dbt_channels_t *manager, uint16_t handle, uint16_t psm,
                               uint16_t rx_mtu, uint16_t minimum_mtu, dbt_security_t security,
                               uint32_t now_ms, uint16_t *local_cid);
dbt_result_t dbt_channels_close(dbt_channels_t *manager, uint16_t handle,
                                uint16_t local_cid, uint32_t now_ms);
const dbt_channel_t *dbt_channels_find(const dbt_channels_t *manager, uint16_t handle, uint16_t local_cid);
/* Signaling is copied into a bounded inbox. NO_SPACE means not accepted; the
 * caller must apply RX flow control or terminate the affected link. Dynamic
 * data/fixed-channel callbacks borrow bytes only for the duration of this call. */
dbt_result_t dbt_channels_receive(dbt_channels_t *manager, uint16_t handle, uint32_t generation,
                                  uint16_t cid, const uint8_t *data, uint16_t size);
uint16_t dbt_channels_poll(dbt_channels_t *manager, uint32_t now_ms, uint16_t budget);
/* Peek pins bytes until commit/cancel. Commit after the complete signaling PDU
 * has been admitted to HCI, not after preparing its first ACL fragment. */
dbt_result_t dbt_channels_tx_peek(dbt_channels_t *manager, dbt_signal_view_t *view);
/* Select the first queued PDU for this link before pinning. Preserves order
 * within each link while allowing independent Classic/LE credit pools to run. */
dbt_result_t dbt_channels_tx_select(dbt_channels_t *manager, uint16_t handle, dbt_signal_view_t *view);
bool dbt_channels_tx_pending(const dbt_channels_t *manager, uint16_t handle);
dbt_result_t dbt_channels_tx_commit(dbt_channels_t *manager, uint32_t now_ms);
dbt_result_t dbt_channels_tx_cancel(dbt_channels_t *manager);
void dbt_channels_link_change(dbt_channels_t *manager, const dbt_link_change_t *change);
dbt_result_t dbt_channels_prepare_data(dbt_channels_t *manager, uint16_t handle, uint16_t local_cid,
                                       const uint8_t *data, uint16_t size, dbt_l2cap_tx_t *tx);
uint32_t dbt_channels_wait_ms(const dbt_channels_t *manager, uint32_t now_ms);

#endif
