#ifndef DBT_AVRCP_H
#define DBT_AVRCP_H
#include "dbt_avctp.h"

/* Control-channel subset. Every call is made on the single host owner. Callback
 * views are borrowed only for the duration of the call. Token identifies one
 * remote command in one binding; workers must return the full token. */
typedef struct {
    const uint8_t *parameters;
    uint16_t size;
    uint64_t token;
    uint8_t label, opcode, pdu, code;
    bool response, invalid_pid, stale, aborted, state_change;
} dbt_avrcp_message_t;
typedef void (*dbt_avrcp_message_fn)(void *, const dbt_avrcp_message_t *);
typedef struct {
    dbt_avctp_t *transport;
    dbt_avrcp_message_fn message;
    void *context;
    uint32_t next_token, remote_deadline[16], now;
    uint64_t remote_token[16];
    uint32_t remote_timeout;
    uint16_t remote_pending, remote_interim, local_interim, remote_auto;
    uint8_t remote_opcode[16], remote_pdu[16], remote_event[16], remote_auto_kind[16];
    uint16_t remote_pid[16];
    uint8_t local_opcode[16], local_pdu[16], local_event[16], local_operation[16];
    uint8_t local_control[16][11], local_control_size[16];
    uint8_t remote_control[16][11], remote_control_size[16];
    uint8_t *echo_buffer;
    uint16_t echo_capacity, echo_size;
    uint8_t echo_label;
    bool echo_active;
    uint8_t *continuation_buffer;
    uint16_t continuation_capacity, continuation_size;
    uint8_t continuation_fragments;
    uint8_t *target_buffer;
    uint16_t target_capacity, target_size, target_offset;
    uint32_t target_hold_ms, target_deadline;
    bool target_active;
    uint16_t local_continuation, local_abandoned, local_abort;
    uint8_t continuation_label;
    bool continuation_active, continuation_needed, abort_needed;
    dbt_result_t input_status;
    bool in_callback, failed;
} dbt_avrcp_t;
/* Initialize profile first, then initialize control AVCTP with these hooks.
 * Binding must be unique over channel lifetimes. remote_timeout bounds deferred
 * work; notification INTERIM may remain pending until CHANGED or disconnect. */
dbt_result_t dbt_avrcp_init(dbt_avrcp_t *, dbt_avctp_t *, dbt_avrcp_message_fn, void *, uint32_t remote_timeout);
/* Optional disjoint caller-owned 512-byte storage for one queued generic AV/C
 * NOT IMPLEMENTED echo. A second pending echo reports BUSY; absent/too-small
 * storage reports NO_SPACE while transport is busy. */
dbt_result_t dbt_avrcp_set_echo_buffer(dbt_avrcp_t *, uint8_t *, uint16_t);
/* Optional caller-owned metadata reassembly storage. Configure before using
 * GetElementAttributes; held until profile/channel quiescence and disjoint
 * from transport buffers. */
dbt_result_t dbt_avrcp_set_continuation_buffer(dbt_avrcp_t *, uint8_t *, uint16_t);
/* Optional independent storage for target-side long metadata replies. hold_ms
 * bounds the retained snapshot if the controller never requests continuation.
 * Storage must remain disjoint from transport and controller reassembly buffers. */
dbt_result_t dbt_avrcp_set_target_buffer(dbt_avrcp_t *, uint8_t *, uint16_t, uint32_t hold_ms);
dbt_avctp_hooks_t dbt_avrcp_hooks(dbt_avrcp_t *);
dbt_result_t dbt_avrcp_receive(dbt_avrcp_t *, uint32_t binding, const uint8_t *, uint16_t, uint32_t now);
dbt_result_t dbt_avrcp_passthrough(dbt_avrcp_t *, uint8_t operation, bool release, uint32_t now, uint8_t *label);
/* Supported PDUs: 0x10 capabilities, 0x20 element attributes, 0x30 play
 * status, 0x31 notification, 0x50 absolute volume, 0x60 addressed player,
 * 0x74 play item and 0x90 add to now playing. Parameters copied; the owner
 * validates UID and player state against the browsing/control context. */
dbt_result_t dbt_avrcp_command(dbt_avrcp_t *, uint8_t pdu, const uint8_t *, uint16_t, uint32_t now, uint8_t *label);
/* Token must match the request callback. Reply code is AV/C response code;
 * parameters are copied. For RegisterNotification use INTERIM (0xf) then
 * CHANGED (0xd). A remote-origin volume set must not itself produce CHANGED. */
dbt_result_t dbt_avrcp_reply(dbt_avrcp_t *, uint64_t token, uint8_t code, const uint8_t *, uint16_t, uint32_t now);
/* Queue REJECTED for pending target notification(s), invalidating their
 * asynchronous tokens. The owner calls this for a browsed-player/UID reset
 * and, after queuing AddressedPlayerChanged, for player-specific events. */
dbt_result_t dbt_avrcp_cancel_notification(dbt_avrcp_t *, uint8_t event, uint8_t error, uint32_t now);
dbt_result_t dbt_avrcp_addressed_player_changed(dbt_avrcp_t *, uint32_t now);
/* Abort a partially received metadata response. A previously sent continuation
 * keeps its label quarantined until its matching reply arrives or the channel
 * times out; no stale bytes are delivered as metadata. */
dbt_result_t dbt_avrcp_abort_continuation(dbt_avrcp_t *);
dbt_result_t dbt_avrcp_poll(dbt_avrcp_t *, uint32_t now);
uint32_t dbt_avrcp_wait_ms(const dbt_avrcp_t *,uint32_t now);
#endif
