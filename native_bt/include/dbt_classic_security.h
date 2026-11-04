#ifndef DBT_CLASSIC_SECURITY_H
#define DBT_CLASSIC_SECURITY_H

#include "dbt_channels.h"

#define DBT_CLASSIC_SECURITY_COMMANDS 8u
#define DBT_CLASSIC_SECURITY_INLINE 23u

typedef struct {
    uint8_t bytes[16], type, pin_length;
} dbt_classic_key_t;
typedef enum {
    DBT_CLASSIC_INPUT_NONE,
    DBT_CLASSIC_INPUT_PIN,
    DBT_CLASSIC_INPUT_CONFIRM,
    DBT_CLASSIC_INPUT_PASSKEY,
    DBT_CLASSIC_INPUT_OOB
} dbt_classic_input_t;
typedef enum {
    DBT_CLASSIC_SECURITY_READY,
    DBT_CLASSIC_BOND_STORED,
    DBT_CLASSIC_BOND_REMOVED,
    DBT_CLASSIC_SECURITY_FAILED,
    DBT_CLASSIC_SECURITY_LINK_LOST
} dbt_classic_security_event_t;
typedef enum {
    DBT_CLASSIC_REASON_NONE,
    DBT_CLASSIC_REASON_POLICY,
    DBT_CLASSIC_REASON_AUTHENTICATION,
    DBT_CLASSIC_REASON_ENCRYPTION,
    DBT_CLASSIC_REASON_KEY_SIZE,
    DBT_CLASSIC_REASON_KEY_TYPE,
    DBT_CLASSIC_REASON_STORAGE,
    DBT_CLASSIC_REASON_TIMEOUT,
    DBT_CLASSIC_REASON_RESOURCES,
    DBT_CLASSIC_REASON_CONTROLLER
} dbt_classic_security_reason_t;
typedef struct {
    uint8_t address[6], key_type, key_size;
    uint16_t handle;
    uint32_t generation;
    bool connected, authenticated, secure_connections, bonded, encrypted;
} dbt_classic_security_info_t;

typedef struct {
    uint32_t procedure_timeout_ms, store_timeout_ms;
    uint8_t io_capability, authentication_requirements, minimum_key_size, oob_present;
    bool allow_just_works, allow_legacy, require_secure_connections;
    bool controller_secure_connections_enabled, require_durable_bond;
} dbt_classic_security_config_t;

typedef struct {
    void *context;
    bool (*pairing_allowed)(void *context, const uint8_t address_le[6]);
    /* Nonblocking lookup from a previously validated durable bond cache.
     * EMPTY is an unknown peer. Never read flash or block in this callback. */
    dbt_result_t (*load)(void *context, const uint8_t address_le[6], dbt_classic_key_t *key);
    /* Copy before returning. A NULL key requests deletion. True means only
     * queued; complete_store on the owner acknowledges the actual commit.
     * Jobs for one address must commit/complete in submission order. */
    bool (*store)(void *context, const uint8_t address_le[6], const dbt_classic_key_t *key,
                  uint32_t token);
    /* Only OOB commands larger than 23 bytes need allocation. */
    uint8_t *(*allocate)(void *context, uint16_t size);
    void (*release)(void *context, uint8_t *bytes);
    /* Borrowed metadata. Reply later on the owner; do not reenter or wait.
     * Value is the six-digit number for confirmation, otherwise zero. */
    void (*input)(void *context, const uint8_t address_le[6], dbt_classic_input_t input,
                  uint32_t token, uint32_t value);
    void (*display_passkey)(void *context, const uint8_t address_le[6], uint32_t value);
    void (*event)(void *context, dbt_classic_security_event_t event,
                  const dbt_classic_security_info_t *info, dbt_classic_security_reason_t reason,
                  uint8_t controller_status);
} dbt_classic_security_hooks_t;

typedef struct {
    dbt_classic_key_t key;
    uint32_t serial, generation, deadline, store_deadline, input_token, store_token;
    uint32_t key_epoch, encryption_key_epoch, cipher_epoch, verified_cipher_epoch;
    uint16_t handle;
    uint8_t address[6], remote_io, remote_auth, key_size, encryption_mode;
    dbt_security_t wanted;
    dbt_classic_input_t input;
    bool used, connected, accepted_pairing, key_known, bonded, dirty_key, ready, failed;
    bool auth_requested, auth_complete, pairing_complete, encryption_requested, size_requested;
    bool force_pair, deleting, store_expired, io_replied, auto_confirmed;
    bool locally_required;
} dbt_classic_peer_t;

typedef struct {
    uint8_t *extended;
    uint32_t peer_serial, generation;
    uint32_t key_epoch, cipher_epoch;
    uint16_t opcode, handle;
    uint8_t address[6], size;
    dbt_command_response_t response;
    uint8_t parameters[DBT_CLASSIC_SECURITY_INLINE];
} dbt_classic_security_command_t;

typedef struct {
    dbt_host_t *host;
    dbt_acl_t *acl;
    dbt_classic_security_config_t config;
    dbt_classic_security_hooks_t hooks;
    dbt_classic_peer_t peers[DBT_MAX_LINKS];
    dbt_classic_security_command_t commands[DBT_CLASSIC_SECURITY_COMMANDS], active;
    uint32_t serial, token, now_ms, rejected_commands, stale_completions;
    uint8_t head, count;
    bool command_active, pairable, in_callback;
} dbt_classic_security_t;

bool dbt_classic_key_valid(const dbt_classic_key_t *key);
bool dbt_classic_key_authenticated(const dbt_classic_key_t *key);
bool dbt_classic_key_secure_connections(const dbt_classic_key_t *key);

/* One host owner for every call; callbacks are nonblocking and borrowed.
 * The controller performs SSP/link authentication cryptography. This module
 * owns policy, interactions, key lifecycle, encryption/key-size verification
 * and durable-store acknowledgement. It is not BLE SMP. Clear before reinit. */
dbt_result_t dbt_classic_security_init(dbt_classic_security_t *security, dbt_host_t *host,
                                       dbt_acl_t *acl, dbt_classic_security_config_t config,
                                       dbt_classic_security_hooks_t hooks);
void dbt_classic_security_pairable(dbt_classic_security_t *security, bool enabled);
dbt_result_t dbt_classic_security_link(dbt_classic_security_t *security,
                                       const dbt_link_change_t *change);
/* Dispatch before GAP. UNSUPPORTED means GAP/another owner must handle it. */
dbt_result_t dbt_classic_security_receive(dbt_classic_security_t *security,
                                          const dbt_hci_packet_t *packet, uint32_t now_ms);
dbt_result_t dbt_classic_security_require(dbt_classic_security_t *security, const dbt_link_t *link,
                                          dbt_security_t level, uint32_t now_ms);
bool dbt_classic_security_authorize(const dbt_classic_security_t *security, const dbt_link_t *link,
                                    dbt_security_t level);
/* PIN: 1..16 bytes. CONFIRM: no data. PASSKEY: four little-endian bytes <=999999.
 * OOB: 32 bytes (C,R) for P192-only or P256-only, 64 for both, as configured.
 * Refusal accepts no data. Unusable/late tokens never affect a reused peer. */
dbt_result_t dbt_classic_security_reply(dbt_classic_security_t *security, uint32_t token,
                                        bool accepted, const uint8_t *data, uint8_t size);
dbt_result_t dbt_classic_security_complete_store(dbt_classic_security_t *security, uint32_t token,
                                                 bool committed);
dbt_result_t dbt_classic_security_forget(dbt_classic_security_t *security,
                                         const uint8_t address_le[6], uint32_t now_ms);
void dbt_classic_security_poll(dbt_classic_security_t *security, uint32_t now_ms);
uint32_t dbt_classic_security_wait_ms(const dbt_classic_security_t *security, uint32_t now_ms);
/* Quiesce the worker and abort/drain host commands before clearing. */
dbt_result_t dbt_classic_security_clear(dbt_classic_security_t *security);

#endif
