#ifndef DBT_HCI_H
#define DBT_HCI_H

#include "dbt_queue.h"

enum {
    DBT_H4_COMMAND = 0x01,
    DBT_H4_ACL = 0x02,
    DBT_H4_SCO = 0x03,
    DBT_H4_EVENT = 0x04,
    DBT_EVT_COMMAND_COMPLETE = 0x0e,
    DBT_EVT_COMMAND_STATUS = 0x0f,
    DBT_EVT_HARDWARE_ERROR = 0x10
};

typedef struct {
    uint8_t type;
    uint8_t event;
    uint16_t handle;
    uint8_t pb;
    uint8_t bc;
    uint8_t packet_status; /* SCO Packet_Status_Flag; zero for other H4 types. */
    const uint8_t *payload;
    uint16_t size;
} dbt_hci_packet_t;

uint16_t dbt_read_le16(const uint8_t *data);
/* Exactly one complete H4 controller-to-host packet. ISO is not implemented.
 * SCO framing/status is supported; connection ownership and voice profiles are
 * separate from this decoder. */
dbt_result_t dbt_hci_decode(const uint8_t *data, uint16_t size, dbt_hci_packet_t *packet);
dbt_result_t dbt_hci_command(uint16_t opcode, const uint8_t *parameters,
                            uint8_t size, uint8_t *output,
                            uint16_t capacity, uint16_t *written);

#endif
