#ifndef DBT_PAGE_H
#define DBT_PAGE_H

#include "dbt_acl.h"

typedef enum {
    DBT_PAGE_NONE, DBT_PAGE_CONNECTED, DBT_PAGE_REJECTED,
    DBT_PAGE_CANCELLED, DBT_PAGE_FAILED
} dbt_page_outcome_t;
enum { DBT_PAGE_STATUS_CREATE=1, DBT_PAGE_STATUS_CONNECTION=2, DBT_PAGE_STATUS_CANCEL=4 };
typedef struct {
    uint32_t token;
    dbt_page_outcome_t outcome;
    uint16_t handle;
    uint8_t command_status,connection_status,cancel_status,status_mask;
    bool cancel_requested,timed_out;
} dbt_page_result_t;
typedef struct {
    uint8_t address_le[6],page_repetition;
    uint16_t clock_offset; /* HCI bit 15 marks a valid cached clock offset. */
    bool allow_role_switch;
} dbt_page_config_t;
typedef struct {
    dbt_host_t *host;
    dbt_page_result_t result;
    uint32_t cancel_token,deadline,retire_deadline,retire_ms;
    uint8_t address_le[6];
    bool active,create_pending,admitted,cancel_pending,cancel_attempted,final_seen;
} dbt_page_t;

/* Single-owner HCI procedure, not a GAP admission/reconnect policy. The caller
 * reserves one ACL slot and exclusive establishment ownership for this peer
 * until terminal completion. A connection manager must arbitrate simultaneous
 * incoming establishment before routing its events here; address matching alone
 * cannot distinguish two competing procedures. Only one page per host at once.
 * This module grants no bond trust or protected-profile access.
 *
 * Initialize fresh storage, or retired storage after all callbacks complete.
 * Never reinitialize the host/object while its commands/procedure remain live.
 * Poll on procedure/event progress; combine wait_ms with runtime/host deadlines.
 * Forward the arbitrated final Connection Complete event. Host command callbacks
 * and controller ingress notifications already wake the same owner.
 * No heap, callbacks to application code, waits, tasks or controller resets. */
dbt_result_t dbt_page_init(dbt_page_t *,dbt_host_t *);
/* Maximum locally supported BR/EDR types; EDR exclusion bits have inverse sense.
 * This reads the identified controller's feature page, not guessed hardware. */
uint16_t dbt_page_packet_types(const dbt_controller_info_t *);
/* Copies parameters into the existing tracked HCI queue. Token is unchanged on
 * refusal; accepted tokens never repeat across host reinitialization. The ACL
 * snapshot must belong to this host, and the caller must maintain its capacity
 * reservation through completion. timeout includes initial command queuing;
 * retire_ms bounds cancel admission/final-event waiting. Submitted command
 * callbacks additionally retain the host's bounded queue/response deadlines.
 * Both intervals are 1..60000. Missing final retirement fails the host closed;
 * normal rejection/cancellation does not. Host failure requires normal complete
 * controller/transport recovery before another procedure, not object reinit. */
dbt_result_t dbt_page_begin(dbt_page_t *,const dbt_acl_t *,const dbt_page_config_t *,
                          uint32_t now_ms,uint32_t timeout_ms,uint32_t retire_ms,uint32_t *token);
/* Requests cancellation; PENDING still owns the context. OK means the result is
 * terminal, not necessarily canceled: establishment may win and return a live
 * handle. The caller must explicitly disconnect that exact runtime link. */
dbt_result_t dbt_page_cancel(dbt_page_t *,uint32_t token,uint32_t now_ms);
/* Always copies a snapshot for the matching token, including while PENDING.
 * A handle is meaningful only for CONNECTED; reconcile it with the runtime's
 * current link generation before using it. Status mask distinguishes absent
 * events from a genuine zero HCI status. Result remains until the next begin. */
dbt_result_t dbt_page_result(const dbt_page_t *,uint32_t token,dbt_page_result_t *);
dbt_result_t dbt_page_event(dbt_page_t *,const dbt_hci_packet_t *);
void dbt_page_poll(dbt_page_t *,uint32_t now_ms);
uint32_t dbt_page_wait_ms(const dbt_page_t *,uint32_t now_ms);

#endif
