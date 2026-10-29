#ifndef DBT_GAP_H
#define DBT_GAP_H

#include "dbt_runtime.h"

#define DBT_GAP_REPLY_SLOTS 4u
typedef enum { DBT_GAP_IDLE, DBT_GAP_CONFIGURING, DBT_GAP_READY, DBT_GAP_FAILED } dbt_gap_state_t;
typedef struct {
    char name[33];
    uint8_t class_of_device[3];
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
    uint8_t bytes[7], size;
    dbt_command_response_t response;
} dbt_gap_reply_t;
typedef struct {
    dbt_host_t *host;
    dbt_gap_config_t config;
    dbt_gap_hooks_t hooks;
    dbt_gap_reply_t replies[DBT_GAP_REPLY_SLOTS];
    dbt_gap_state_t state;
    uint32_t now_ms, rejected_commands;
    uint8_t step, reply_head, reply_count, last_status;
    uint8_t classic_scan, wanted_scan;
    bool command_pending, classic_pending, advertising_pending;
    bool advertising, wanted_advertising;
} dbt_gap_t;

/* Controller configuration and legacy advertising/page scanning. Does not
 * implement bonding, initiator scanning, privacy or audio/GATT profiles. */
dbt_result_t dbt_gap_init(dbt_gap_t *gap, dbt_host_t *host, dbt_gap_config_t config, dbt_gap_hooks_t hooks);
dbt_result_t dbt_gap_start(dbt_gap_t *gap);
void dbt_gap_poll(dbt_gap_t *gap, uint32_t now_ms);
dbt_result_t dbt_gap_event(dbt_gap_t *gap, const dbt_hci_packet_t *packet);
uint32_t dbt_gap_wait_ms(const dbt_gap_t *gap);
/* READY does not imply either bearer is discoverable. Enable explicitly only
 * after profile/security initialization. Properties change only on successful
 * command completion; on LE connection complete advertising becomes false. */
dbt_result_t dbt_gap_set_classic_scan(dbt_gap_t *gap, bool discoverable, bool connectable);
dbt_result_t dbt_gap_set_advertising(dbt_gap_t *gap, bool enabled);
dbt_result_t dbt_gap_disconnect(dbt_gap_t *gap, const dbt_link_t *link, uint8_t reason);

#endif
