#ifndef DBT_AVRCP_BROWSE_H
#define DBT_AVRCP_BROWSE_H
#include "dbt_avrcp.h"
/* Experimental wire primitives only. Do not advertise browsing until ERTM,
 * full item semantics and conformance are integrated. */
typedef struct {
    dbt_avctp_t *transport;
    dbt_avrcp_message_fn message;
    void *context;
    uint32_t now, next_token, timeout, deadline[16];
    uint64_t token[16];
    uint16_t remote_pending, remote_auto, local_mutating, remote_mutating, state_change_admission;
    uint32_t local_epoch, target_epoch, local_snapshot[16], remote_snapshot[16];
    uint16_t local_player, target_player, local_uid, target_uid, local_addressed_uid, target_addressed_uid;
    uint16_t local_arg[16], remote_arg[16], auto_pid[16];
    uint8_t local_pdu[16], remote_pdu[16], auto_pdu[16], auto_status[16];
    uint8_t local_scope[16], remote_scope[16], local_depth, target_depth;
    bool local_selected, target_selected, local_uid_valid, target_uid_valid;
    bool local_addressed_valid, target_addressed_valid;
    bool local_search_valid, target_search_valid;
    dbt_result_t status;
    bool failed, in_callback;
} dbt_avrcp_browse_t;
dbt_result_t dbt_avrcp_browse_init(dbt_avrcp_browse_t *,dbt_avctp_t *,dbt_avrcp_message_fn,void *,uint32_t);
dbt_avctp_hooks_t dbt_avrcp_browse_hooks(dbt_avrcp_browse_t *);
dbt_result_t dbt_avrcp_browse_receive(dbt_avrcp_browse_t *,uint32_t,const uint8_t *,uint16_t,uint32_t);
/* Input must be disjoint from the AVCTP TX buffer. Copy bound is the caller's
 * TX capacity and negotiated browsing MTU; no AVCTP fragmentation is used. */
dbt_result_t dbt_avrcp_browse_command(dbt_avrcp_browse_t *,uint8_t,const uint8_t *,uint16_t,uint32_t,uint8_t *);
dbt_result_t dbt_avrcp_browse_reply(dbt_avrcp_browse_t *,uint64_t,const uint8_t *,uint16_t,uint32_t);
/* Forward control-channel UID change to the appropriate role. Each call
 * invalidates prior work and search results. unavailable=true clears the
 * browsed player. After the successful player response is fully admitted,
 * state_change notifies the owner to complete any outstanding UID notification
 * on the control channel. Admission is not proof of peer receipt. */
dbt_result_t dbt_avrcp_browse_uids_changed(dbt_avrcp_browse_t *, bool target, uint16_t uid, bool unavailable);
/* Invalidate or publish the addressed player's UID counter separately from the
 * browsed player's counter; scope 3 belongs to the addressed player. */
dbt_result_t dbt_avrcp_browse_addressed_uids_changed(dbt_avrcp_browse_t *, bool target, uint16_t uid, bool valid);
dbt_result_t dbt_avrcp_browse_poll(dbt_avrcp_browse_t *,uint32_t);
uint32_t dbt_avrcp_browse_wait_ms(const dbt_avrcp_browse_t *,uint32_t);
#endif
