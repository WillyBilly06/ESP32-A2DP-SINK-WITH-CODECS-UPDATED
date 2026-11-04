#ifndef DBT_ATT_H
#define DBT_ATT_H

#include "dbt_acl.h"

#define DBT_ATT_MAX_MTU 517u
#define DBT_ATT_MAX_VALUE 512u
#define DBT_ATT_CCCD_LIMIT 16u
#define DBT_ATT_PREPARE_LIMIT 8u
#define DBT_ATT_DEFERRED 0x100u

enum {
    DBT_ATT_READ = 1,
    DBT_ATT_WRITE = 2,
    DBT_ATT_READ_ENCRYPT = 4,
    DBT_ATT_WRITE_ENCRYPT = 8,
    DBT_ATT_READ_AUTHENTICATE = 16,
    DBT_ATT_WRITE_AUTHENTICATE = 32,
    DBT_ATT_READ_AUTHORIZE = 64,
    DBT_ATT_WRITE_AUTHORIZE = 128,
    DBT_ATT_NOTIFY = 256,
    DBT_ATT_INDICATE = 512,
    DBT_ATT_DYNAMIC = 1024
};
enum {
    DBT_ATT_INVALID_HANDLE = 1,
    DBT_ATT_READ_DENIED = 2,
    DBT_ATT_WRITE_DENIED = 3,
    DBT_ATT_INVALID_PDU = 4,
    DBT_ATT_AUTHENTICATION = 5,
    DBT_ATT_NOT_SUPPORTED = 6,
    DBT_ATT_INVALID_OFFSET = 7,
    DBT_ATT_AUTHORIZATION = 8,
    DBT_ATT_PREPARE_FULL = 9,
    DBT_ATT_NOT_FOUND = 10,
    DBT_ATT_NOT_LONG = 11,
    DBT_ATT_KEY_SIZE = 12,
    DBT_ATT_INVALID_LENGTH = 13,
    DBT_ATT_UNLIKELY = 14,
    DBT_ATT_ENCRYPTION = 15,
    DBT_ATT_GROUP_TYPE = 16,
    DBT_ATT_RESOURCES = 17
};

/* Ordered immutable database, suitable for flash. UUID bytes use ATT little
 * endian wire order. Service values are 2/16-byte UUIDs. Characteristic values
 * carry NOTIFY/INDICATE flags; each CCCD refers to its value handle. */
typedef struct {
    uint16_t handle, flags, maximum_size, value_size, cccd_for;
    uint8_t uuid_size, minimum_key_size;
    const uint8_t *uuid, *value;
} dbt_attribute_t;

typedef struct {
    uint16_t handle, offset, size;
    const uint8_t *data;
} dbt_att_write_t;
typedef struct {
    uint16_t handle, value;
} dbt_att_cccd_t;
typedef struct {
    const dbt_att_write_t *writes;
    const dbt_att_cccd_t *cccds;
    uint8_t count, cccd_count;
    bool command, prepared;
} dbt_att_batch_t;

typedef struct {
    void *context;
    uint8_t *(*allocate)(void *context, uint16_t size);
    void (*release)(void *context, uint8_t *bytes);
    uint8_t (*authorize)(void *context, const dbt_link_t *link, const dbt_attribute_t *attribute,
                         bool write);
    /* Read must fill min(capacity,total-offset) bytes. Total is the complete
     * current value size, <= maximum_size. No blocking or retained pointers. */
    uint8_t (*read)(void *context, const dbt_link_t *link, const dbt_attribute_t *attribute,
                    uint16_t offset, uint8_t *output, uint16_t capacity, uint16_t *written,
                    uint16_t *total);
    /* One atomic batch, never sequential per-attribute commits. Validate every
     * write before applying any. Return ATT error without mutations on failure.
     * For slow request work, copy all descriptors/data before returning DEFERRED
     * and call complete on the owner after the worker commits. Commands have no
     * response and may return success after bounded worker admission; they must
     * not return DEFERRED. No borrowed pointer survives this callback. */
    uint16_t (*write)(void *context, const dbt_link_t *link, const dbt_att_batch_t *batch,
                      uint32_t token, uint16_t *error_handle);
    /* Bond identity resolution is the security owner's responsibility. Called
     * only when marking a connection bonded. EMPTY means no saved subscription.
     * Bonded CCCD writes require write() to persist the batch. */
    dbt_result_t (*load_cccd)(void *context, const dbt_link_t *link, uint16_t handle,
                              uint16_t *value);
    void (*confirmed)(void *context, uint16_t handle, uint32_t generation, uint16_t attribute);
    void (*failed)(void *context, uint16_t handle, uint32_t generation);
} dbt_att_hooks_t;

typedef struct dbt_att_prepared {
    struct dbt_att_prepared *next;
    uint16_t handle, offset, size;
    uint8_t data[];
} dbt_att_prepared_t;

typedef struct {
    dbt_att_prepared_t *prepared;
    uint8_t *response, *notice;
    uint32_t generation, request_deadline, indication_deadline, token;
    uint16_t handle, mtu, pending_mtu, response_size, notice_size;
    uint16_t prepared_bytes, error_handle, indication_handle;
    uint16_t cccds[DBT_ATT_CCCD_LIMIT], staged_cccds[DBT_ATT_CCCD_LIMIT];
    uint8_t prepared_count, locked, pending_opcode;
    bool used, request_busy, response_ready, notice_ready, mtu_exchanged;
    bool deferred, indication_wait, authenticated, bonded, failed;
    uint8_t key_size;
    uint8_t short_response[23], short_notice[23];
} dbt_att_connection_t;

typedef struct {
    const dbt_attribute_t *attributes;
    dbt_att_hooks_t hooks;
    dbt_att_connection_t connections[DBT_MAX_LINKS];
    uint32_t token, ignored_commands;
    uint16_t attribute_count, mtu, prepare_budget;
    uint8_t cccd_count;
    bool in_callback;
} dbt_att_server_t;
typedef struct {
    const uint8_t *data;
    uint16_t size;
} dbt_att_view_t;

/* Legacy unenhanced ATT server over LE fixed CID 4. EATT, signed commands and
 * GATT client procedures are not advertised/implemented by this module. All
 * calls use one host owner. Allocation occurs only for long TX/prepare data. */
dbt_result_t dbt_att_init(dbt_att_server_t *server, const dbt_attribute_t *attributes,
                          uint16_t count, dbt_att_hooks_t hooks, uint16_t mtu,
                          uint16_t prepare_budget);
dbt_result_t dbt_att_open(dbt_att_server_t *server, const dbt_link_t *link);
void dbt_att_close(dbt_att_server_t *server, uint16_t handle, uint32_t generation);
void dbt_att_clear(dbt_att_server_t *server);
/* Only a verified security owner may assert authentication/key size/bonding. */
dbt_result_t dbt_att_security(dbt_att_server_t *server, const dbt_link_t *link, bool authenticated,
                              uint8_t key_size, bool bonded);
dbt_result_t dbt_att_receive(dbt_att_server_t *server, const dbt_link_t *link, const uint8_t *data,
                             uint16_t size, uint32_t now_ms);
/* Owner continuation for an accepted DEFERRED write. Token/generation are both
 * checked; stale worker completions never affect a reused connection handle. */
dbt_result_t dbt_att_complete(dbt_att_server_t *server, uint16_t handle, uint32_t generation,
                              uint32_t token, uint8_t error, uint16_t error_handle);
dbt_result_t dbt_att_notify(dbt_att_server_t *server, const dbt_link_t *link, uint16_t value_handle,
                            const uint8_t *data, uint16_t size, bool indication);
/* Pull TX only when runtime_send_fixed can accept it. Keep a pinned view until
 * all its fragments are admitted, then call tx_admitted. Close only after the
 * runtime has returned its borrowed SDU on disconnect/abort. */
dbt_result_t dbt_att_tx_peek(dbt_att_server_t *server, uint16_t handle, uint32_t generation,
                             dbt_att_view_t *view);
dbt_result_t dbt_att_tx_admitted(dbt_att_server_t *server, uint16_t handle, uint32_t generation,
                                 uint32_t now_ms);
void dbt_att_poll(dbt_att_server_t *server, uint32_t now_ms);
uint32_t dbt_att_wait_ms(const dbt_att_server_t *server, uint32_t now_ms);

#endif
