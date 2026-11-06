#ifndef DBT_GAP_H
#define DBT_GAP_H

#include "dbt_runtime.h"
#include "dbt_page.h"

#define DBT_GAP_REPLY_SLOTS 4u
#define DBT_GAP_CLASSIC_SERVICES 6u
#define DBT_GAP_CLASSIC_SETUP_MS 60000u
#define DBT_GAP_CLASSIC_ACCEPT_MS 5000u
#define DBT_GAP_CLASSIC_DEFER_MS 4500u
#define DBT_GAP_PAGE_RETIRE_MAX_MS 4000u
typedef enum { DBT_GAP_IDLE, DBT_GAP_CONFIGURING, DBT_GAP_READY, DBT_GAP_FAILED } dbt_gap_state_t;
typedef struct {
    char name[33];
    uint8_t class_of_device[3];
    uint16_t classic_services[DBT_GAP_CLASSIC_SERVICES];
    uint8_t classic_service_count;
    uint8_t advertising[31], advertising_size;
    uint8_t scan_response[31], scan_response_size;
    uint16_t advertising_minimum, advertising_maximum;
} dbt_gap_config_t;
typedef struct {
    void *context;
    /* Explicit admission policy; default refuses all incoming Classic links. */
    bool (*accept_classic)(void *context, const uint8_t address_le[6], const uint8_t device_class[3]);
    /* Return true only if a security owner accepted responsibility for replying.
     * Without one, pairing/key requests receive the standard negative reply.
     * There is no implicit trust or automatic Just Works confirmation. */
    bool (*security)(void *context, const dbt_hci_packet_t *packet);
} dbt_gap_hooks_t;
typedef struct {
    uint16_t opcode;
    uint8_t bytes[7], size, incoming;
    dbt_command_response_t response;
} dbt_gap_reply_t;
struct dbt_gap;
typedef struct {
    struct dbt_gap *gap;
    uint32_t deadline,token;
    uint8_t address_le[6],status,flags;
} dbt_gap_incoming_t;
typedef enum {
    DBT_GAP_INCOMING_QUEUED=1,DBT_GAP_INCOMING_SUBMITTED,DBT_GAP_INCOMING_ACK,
    DBT_GAP_INCOMING_FINAL,DBT_GAP_INCOMING_RETIRED,DBT_GAP_INCOMING_EXPIRED,
    DBT_GAP_INCOMING_RESOLVED
} dbt_gap_incoming_phase_t;
/* Optional owner-only bounded observer. Entry is borrowed for this callback;
 * copy metadata without waiting or reentering GAP. Outcome is valid only for
 * ACK; status is valid for FINAL or a successful/rejected HCI ACK. */
typedef void (*dbt_gap_incoming_observer_t)(void *,dbt_gap_incoming_phase_t,
    const dbt_gap_incoming_t *,uint16_t,dbt_command_outcome_t);
typedef struct {
    dbt_page_result_t outgoing;
    dbt_page_outcome_t outcome;
    uint32_t generation;
    uint16_t handle;
    uint8_t incoming_command_status,incoming_connection_status,incoming_status_mask;
    bool via_incoming,user_cancel_requested;
} dbt_gap_connect_result_t;
enum { DBT_GAP_CONNECT_ACK=1,DBT_GAP_CONNECT_FINAL=2 };
typedef struct dbt_gap {
    dbt_host_t *host;
    dbt_gap_config_t config;
    dbt_gap_hooks_t hooks;
    dbt_gap_reply_t replies[DBT_GAP_REPLY_SLOTS];
    dbt_gap_state_t state;
    uint32_t now_ms, rejected_commands;
    uint8_t step, reply_head, reply_count, last_status,reply_inflight;
    uint8_t classic_scan, wanted_scan;
    bool command_pending, classic_pending, advertising_pending;
    bool advertising, wanted_advertising;
    char rename_name[33];uint8_t rename_scan[31],rename_scan_size,rename_step,rename_status;
    uint32_t rename_serial,rename_done;
    bool rename_active,rename_pending,rename_applied,rename_partial;
    dbt_gap_incoming_t incoming[DBT_MAX_LINKS];
    dbt_gap_incoming_observer_t incoming_observer;
    void *incoming_context;
    dbt_page_t *page;
    const dbt_acl_t *page_acl;
    uint32_t page_generation;
    uint16_t page_incoming_handle;
    uint8_t page_flags,page_incoming_mask,page_incoming_ack,page_incoming_status,page_incoming_outcome;
} dbt_gap_t;

/* Controller configuration and legacy advertising/page scanning. Does not
 * implement bonding, initiator scanning, privacy or audio/GATT profiles.
 * Initialize fresh storage only. Queued replies, command callbacks and accepted
 * controller procedures must retire before reuse; host/controller recovery
 * must also finish before reinitializing a failed instance. */
dbt_result_t dbt_gap_init(dbt_gap_t *gap, dbt_host_t *host, dbt_gap_config_t config, dbt_gap_hooks_t hooks);
dbt_result_t dbt_gap_start(dbt_gap_t *gap);
void dbt_gap_poll(dbt_gap_t *gap, uint32_t now_ms);
dbt_result_t dbt_gap_event(dbt_gap_t *gap, const dbt_hci_packet_t *packet);
/* Prefer explicit current owner time for setup deadlines. The legacy entry
 * uses the last poll time. Call only from the same host owner as poll. */
dbt_result_t dbt_gap_event_at(dbt_gap_t *,const dbt_hci_packet_t *,uint32_t now_ms);
uint32_t dbt_gap_wait_ms(const dbt_gap_t *gap);
/* Incoming identity remains pinned through queued command/callback retirement.
 * Accepted controller procedures additionally await Connection Complete.
 * Reservations exclude peers already published into the matching ACL runtime,
 * avoiding double counting during its earlier link-added callback. These are
 * owner queries, not outgoing-connection admission or collision arbitration.
 * Invalid reservation-query arguments fail closed with DBT_MAX_LINKS. */
bool dbt_gap_classic_pending(const dbt_gap_t *,const uint8_t address_le[6]);
uint8_t dbt_gap_classic_reservations(const dbt_gap_t *,const dbt_acl_t *);
void dbt_gap_set_incoming_observer(dbt_gap_t *,dbt_gap_incoming_observer_t,void *);
/* Caller authorizes peer selection; this grants no pairing window or protected
 * access. The ACL view must be this host's runtime. Page storage must already be
 * initialized for this host and remain valid until connect_result returns OK.
 * Use the GAP result, not the standalone page result, as the release fence.
 * One operation per GAP. Reserved incoming/outgoing ACL capacity is respected.
 * A matching incoming request cancels/retires the outgoing page before its
 * decision is submitted. retire_ms must be 1..DBT_GAP_PAGE_RETIRE_MAX_MS. */
dbt_result_t dbt_gap_connect(dbt_gap_t *,dbt_page_t *,const dbt_acl_t *,const dbt_page_config_t *,
                             uint32_t now_ms,uint32_t timeout_ms,uint32_t retire_ms,uint32_t *token);
dbt_result_t dbt_gap_connect_cancel(dbt_gap_t *,uint32_t token,uint32_t now_ms);
/* Copies progress while PENDING. A terminal OK consumes the completion and
 * releases caller page storage. CONNECTED may win cancellation: the caller
 * must then disconnect the exact handle/generation if still desired. */
dbt_result_t dbt_gap_connect_result(dbt_gap_t *,uint32_t token,dbt_gap_connect_result_t *);
/* Pure owner query; true means result can consume the retirement receipt. */
bool dbt_gap_connect_ready(const dbt_gap_t *,uint32_t token);
/* READY does not imply either bearer is discoverable. Enable explicitly only
 * after profile/security initialization. Properties change only on successful
 * command completion; on LE connection complete advertising becomes false. */
dbt_result_t dbt_gap_set_classic_scan(dbt_gap_t *gap, bool discoverable, bool connectable);
dbt_result_t dbt_gap_set_advertising(dbt_gap_t *gap, bool enabled);
/* Owner-only copied UTF-8 rename. Completion covers HCI name, LE scan data and Classic EIR;
 * failed partial controller updates require retry, never a success ACK. */
dbt_result_t dbt_gap_rename(dbt_gap_t *,const uint8_t *,uint8_t,uint32_t *);
dbt_result_t dbt_gap_rename_result(const dbt_gap_t *,uint32_t,bool *,uint8_t *);

dbt_result_t dbt_gap_disconnect(dbt_gap_t *gap, const dbt_link_t *link, uint8_t reason);

#endif
