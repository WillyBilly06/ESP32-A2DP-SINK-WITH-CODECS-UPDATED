#ifndef DBT_CHANNELS_H
#define DBT_CHANNELS_H

#include "dbt_l2cap.h"
#include "dbt_credit.h"

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
    uint16_t spsm;
    dbt_credit_parameters_t receive;
    dbt_security_t security;
    uint8_t minimum_key_size;
} dbt_le_service_t;
struct dbt_le_channel;

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
    struct dbt_le_channel *le; /* Lazy active-channel state; NULL for Basic. */
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
    /* Optional LE CoC state/reassembly allocator, host owner only. */
    uint8_t *(*allocate)(void *context,uint16_t bytes);
    void (*release)(void *context,uint8_t *bytes);
    /* Pure LE security qualification, returns wire result 0 or 5..8. Required
     * for authenticated policy or verified minimum key size; encryption alone
     * is insufficient. Incoming CoC requests are refused while security lacks
     * qualification; this does not implement SMP or claim successful pairing. */
    uint16_t (*le_security)(void *context,const dbt_link_t *link,uint16_t spsm,
        dbt_security_t security,uint8_t minimum_key_size);
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
    const dbt_le_service_t *le_services;
    dbt_channel_t channels[DBT_CHANNEL_LIMIT];
    dbt_signal_link_t links[DBT_MAX_LINKS];
    dbt_signal_tx_t tx[DBT_SIGNAL_TX_SLOTS];
    dbt_signal_rx_t rx[DBT_SIGNAL_RX_SLOTS];
    uint64_t fixed_mask[2];
    uint32_t serial, rtx_ms, ertx_ms, now_ms;
    uint32_t stale_pdus, unknown_responses, malformed_pdus, ignored_dynamic_pdus, oversized_dynamic_pdus, oversized_signaling_pdus;
    uint16_t maximum_mtu;
    uint8_t tx_head, tx_count, rx_head, rx_count;
    uint8_t le_service_count;
    bool tx_locked, in_callback, rx_deferred;
} dbt_channels_t;

/* Single host owner. Callbacks are borrowed, nonblocking and non-reentrant.
 * Classic Basic and LE credit channels share signaling/ownership. ERTM and
 * enhanced credit mode are not advertised by the information response. */
dbt_result_t dbt_channels_init(dbt_channels_t *manager, dbt_acl_t *acl,
                               dbt_channel_hooks_t hooks, uint16_t maximum_mtu,
                               uint32_t rtx_ms, uint32_t ertx_ms);
dbt_result_t dbt_channels_register(dbt_channels_t *manager, dbt_service_t service);
/* One immutable descriptor list, borrowed for manager lifetime (flash suitable).
 * No per-service heap/payload allocation. Must register before accepting LE CoC. */
dbt_result_t dbt_channels_register_le(dbt_channels_t *manager,const dbt_le_service_t *services,uint8_t count);
dbt_result_t dbt_channels_open_le(dbt_channels_t *manager,uint16_t handle,const dbt_le_service_t *selection,
    uint32_t now_ms,uint16_t *local_cid);
dbt_result_t dbt_channels_grant_le(dbt_channels_t *manager,uint16_t handle,uint16_t local_cid,uint16_t credits);
dbt_result_t dbt_channels_credit_begin(dbt_channels_t *manager,uint16_t handle,uint16_t local_cid,const uint8_t *data,uint16_t size);
dbt_result_t dbt_channels_credit_frame(dbt_channels_t *manager,uint16_t handle,uint16_t local_cid,dbt_l2cap_tx_t *frame);
dbt_result_t dbt_channels_credit_commit(dbt_channels_t *manager,uint16_t handle,uint16_t local_cid,bool *complete);
bool dbt_channels_credit_ready(const dbt_channels_t *manager,uint16_t handle,uint16_t local_cid);
/* Abort all channels and release lazy state after transport has been quiesced. */
void dbt_channels_clear(dbt_channels_t *manager);
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
