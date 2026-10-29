#ifndef DBT_HOST_H
#define DBT_HOST_H

#include "dbt_hci.h"

typedef enum {
    DBT_HOST_IDLE,
    DBT_HOST_PROBING,
    DBT_HOST_IDENTIFIED,
    DBT_HOST_FAILED
} dbt_host_state_t;

typedef enum {
    DBT_FAULT_NONE,
    DBT_FAULT_RX_OVERRUN,
    DBT_FAULT_BAD_PACKET,
    DBT_FAULT_UNSUPPORTED_PACKET,
    DBT_FAULT_TIMEOUT,
    DBT_FAULT_COMMAND_REJECTED,
    DBT_FAULT_BAD_RESPONSE,
    DBT_FAULT_CONTROLLER,
    DBT_FAULT_NOT_DUAL_MODE
} dbt_fault_t;

typedef struct {
    void *context;
    bool (*can_send)(void *context);
    /* True means accepted by transport only. Keep data valid until next can_send. */
    bool (*send)(void *context, uint8_t *data, uint16_t size);
} dbt_transport_t;

typedef struct {
    uint8_t hci_version;
    uint16_t hci_revision;
    uint8_t lmp_version;
    uint16_t manufacturer;
    uint16_t lmp_subversion;
    uint8_t supported_commands[64];
    uint8_t features[8];
    uint8_t le_features[8];
    uint8_t address_le[6];
    uint16_t classic_acl_bytes;
    uint16_t classic_acl_packets;
    uint16_t le_acl_bytes;
    uint16_t le_acl_packets;
    bool shared_acl_pool;
} dbt_controller_info_t;

/* Runs on the owner only; packet/view lifetime ends on callback return.
 * May dispatch protocol state and enqueue commands. Never reenter poll/start,
 * block, decode audio, or retain packet pointers. */
typedef void (*dbt_observer_t)(void *context, const dbt_hci_packet_t *packet);

#define DBT_COMMAND_SLOTS 4u
typedef enum { DBT_COMMAND_COMPLETE, DBT_COMMAND_STATUS } dbt_command_response_t;
typedef enum { DBT_COMMAND_SUCCESS, DBT_COMMAND_REJECTED,
               DBT_COMMAND_TIMEOUT, DBT_COMMAND_ABORTED } dbt_command_outcome_t;
typedef enum { DBT_COMMAND_CANCELLED, DBT_COMMAND_ALREADY_SUBMITTED,
               DBT_COMMAND_NOT_FOUND } dbt_command_cancel_result_t;
/* STATUS success means controller procedure admission, not connection completion.
 * Return parameters (including Status) are borrowed only during this callback. */
typedef void (*dbt_command_done_t)(void *context, uint16_t opcode,
    dbt_command_outcome_t outcome, uint8_t status, const uint8_t *returns, uint8_t size);
typedef struct {
    void *context;
    dbt_command_done_t done;
    uint32_t deadline, token;
    uint16_t opcode;
    uint8_t size;
    dbt_command_response_t response;
    uint8_t parameters[255];
} dbt_command_t;

typedef struct {
    dbt_queue_t ingress;
    _Atomic uint32_t ingress_fault;
    dbt_transport_t transport;
    dbt_observer_t observer;
    void *observer_context;
    dbt_controller_info_t controller;
    dbt_host_state_t state;
    dbt_fault_t fault;
    uint32_t timeout_ms;
    uint32_t deadline_ms;
    uint32_t packets_processed;
    uint32_t unexpected_completions;
    uint32_t commands_submitted;
    uint16_t pending_opcode;
    uint8_t command_credits;
    uint8_t probe_index;
    uint8_t controller_status;
    /* Owner-only failure metadata. No packet payload or key bytes retained.
     * Stages: 1 ingress, 2 queue, 3 H4 decode, 4 HCI event, 5 observer. */
    uint16_t fault_packet_size;
    uint8_t fault_packet_type, fault_event, fault_stage;
    uint8_t rx_packet[DBT_MAX_H4_BYTES];
    uint8_t tx_command[259];
    dbt_command_t commands[DBT_COMMAND_SLOTS];
    uint8_t command_head, command_count;
    bool in_poll, tx_retained;
} dbt_host_t;

/* All APIs except receive are single-owner, called from the host task. Before
 * reinitializing a used host, abort outstanding work, complete its callbacks,
 * and wait until the transport has returned any retained TX storage. */
dbt_result_t dbt_host_init(dbt_host_t *host, dbt_transport_t transport,
                          dbt_observer_t observer, void *context, uint32_t timeout_ms);
dbt_result_t dbt_host_start(dbt_host_t *host, uint32_t now_ms);
/* Controller task only: copy, atomically publish, return; caller wakes owner. */
dbt_result_t dbt_host_receive(dbt_host_t *host, const uint8_t *data, uint16_t size);
/* Bounded number of RX callbacks per invocation; also advances probe and timeout. */
uint16_t dbt_host_poll(dbt_host_t *host, uint32_t now_ms, uint16_t rx_budget);
uint32_t dbt_host_wait_ms(const dbt_host_t *host, uint32_t now_ms);
/* Copies bounded parameters; valid after identification. At most one command is
 * outstanding even when the controller grants more credits. Unsent expiry is
 * recoverable; an outstanding timeout fails the host to prevent late-response
 * aliasing. Queue wait and response each receive timeout_ms. */
dbt_result_t dbt_host_command(dbt_host_t *host, uint16_t opcode,
    const uint8_t *parameters, uint8_t size, dbt_command_response_t response,
    dbt_command_done_t done, void *context, uint32_t now_ms);
/* Identical enqueue semantics, additionally yielding a process-unique nonzero
 * token on success. Output is unchanged on failure. Tokens never wrap or repeat
 * across host reinitialization; exhaustion fails closed. */
dbt_result_t dbt_host_command_tracked(dbt_host_t *host, uint16_t opcode,
    const uint8_t *parameters, uint8_t size, dbt_command_response_t response,
    dbt_command_done_t done, void *context, uint32_t now_ms, uint32_t *token);
/* Owner only, including during a host observer callback. An unsent command is
 * removed and wiped before its synchronous ABORTED callback. An HCI-admitted
 * command stays pending until its normal completion or host failure. */
dbt_command_cancel_result_t dbt_host_cancel_command(dbt_host_t *host,uint32_t token);
void dbt_host_abort(dbt_host_t *host, dbt_fault_t fault);

#endif
