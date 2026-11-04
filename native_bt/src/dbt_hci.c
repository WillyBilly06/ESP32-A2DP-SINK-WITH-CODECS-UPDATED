#include "dbt_hci.h"

uint16_t dbt_read_le16(const uint8_t *data) {
    return (uint16_t)(data[0] | ((uint16_t)data[1] << 8));
}

dbt_result_t dbt_hci_decode(const uint8_t *data, uint16_t size, dbt_hci_packet_t *packet) {
    if (data == 0 || packet == 0 || size == 0 || size > DBT_MAX_H4_BYTES) return DBT_INVALID;
    packet->type = data[0];
    packet->event = 0;
    packet->handle = 0;
    packet->pb = 0;
    packet->bc = 0;
    packet->packet_status = 0;
    packet->payload = 0;
    packet->size = 0;
    if (data[0] == DBT_H4_EVENT) {
        if (size < 3u || (uint16_t)(3u + data[2]) != size) return DBT_INVALID;
        packet->event = data[1];
        packet->payload = data + 3;
        packet->size = data[2];
        return DBT_OK;
    }
    if (data[0] == DBT_H4_ACL) {
        if (size < 5u) return DBT_INVALID;
        uint16_t payload_size = dbt_read_le16(data + 3);
        if ((uint32_t)payload_size + 5u != size) return DBT_INVALID;
        uint16_t flags = dbt_read_le16(data + 1);
        packet->handle = flags & 0x0fffu;
        packet->pb = (uint8_t)((flags >> 12) & 3u);
        packet->bc = (uint8_t)(flags >> 14);
        packet->payload = data + 5;
        packet->size = payload_size;
        return DBT_OK;
    }
    if(data[0]==DBT_H4_SCO) {
        if(size<4u || (uint16_t)(4u+data[3])!=size)return DBT_INVALID;
        uint16_t flags=dbt_read_le16(data+1);
        if(flags&0xc000u)return DBT_INVALID;
        packet->handle=flags&0x0fffu;
        packet->packet_status=(uint8_t)((flags>>12)&3u);
        packet->payload=data+4;packet->size=data[3];return DBT_OK;
    }
    return DBT_UNSUPPORTED;
}

dbt_result_t dbt_hci_command(uint16_t opcode, const uint8_t *parameters,
                            uint8_t size, uint8_t *output,
                            uint16_t capacity, uint16_t *written) {
    if (output == 0 || written == 0 || (size != 0 && parameters == 0)) return DBT_INVALID;
    if (capacity < (uint16_t)(4u + size)) return DBT_NO_SPACE;
    output[0] = DBT_H4_COMMAND;
    output[1] = (uint8_t)opcode;
    output[2] = (uint8_t)(opcode >> 8);
    output[3] = size;
    for (uint16_t i = 0; i < size; ++i) output[4u + i] = parameters[i];
    *written = (uint16_t)(4u + size);
    return DBT_OK;
}
