#include "dbt_rx_flow.h"

/* Core Vol 4 Part E 4.2/4.3 and 7.3.38..40, independently authored. */
static void put16(uint8_t *out, uint16_t value) {
    out[0] = (uint8_t)value;
    out[1] = (uint8_t)(value >> 8);
}
static void fail(dbt_rx_flow_t *flow, dbt_fault_t fault, uint8_t status) {
    flow->state = DBT_RX_FLOW_FAILED;
    if (status)
        flow->host->controller_status = status;
    dbt_host_abort(flow->host, fault);
}
static void configured(void *context, uint16_t opcode, dbt_command_outcome_t outcome,
                       uint8_t status, const uint8_t *returns, uint8_t size) {
    dbt_rx_flow_t *flow = context;
    flow->command_pending = false;
    if (outcome != DBT_COMMAND_SUCCESS) {
        flow->failed_opcode=opcode;flow->failed_status=status;
        fail(flow, outcome == DBT_COMMAND_TIMEOUT ? DBT_FAULT_TIMEOUT : DBT_FAULT_COMMAND_REJECTED,
             status);
        return;
    }
    if (!returns || size != 1 || returns[0] ||
        (flow->state == DBT_RX_FLOW_BUFFER   ? opcode != 0x0c33
         : flow->state == DBT_RX_FLOW_ENABLE ? opcode != 0x0c31
                                             : true)) {
        fail(flow, DBT_FAULT_BAD_RESPONSE, 0);
        return;
    }
    flow->state = flow->state == DBT_RX_FLOW_BUFFER ? DBT_RX_FLOW_ENABLE : DBT_RX_FLOW_ACTIVE;
}

dbt_result_t dbt_rx_flow_init(dbt_rx_flow_t *flow, dbt_host_t *host, dbt_acl_t *acl) {
    if (!flow || !host || !acl)
        return DBT_INVALID;
    uint8_t *bytes = (uint8_t *)flow;
    for (unsigned i = 0; i < sizeof(*flow); ++i)
        bytes[i] = 0;
    flow->host = host;
    flow->acl = acl;
    return DBT_OK;
}
dbt_result_t dbt_rx_flow_start(dbt_rx_flow_t *flow, uint16_t packet_bytes, uint16_t packet_count) {
    if (!flow || !flow->host || !flow->acl || !flow->host->transport.can_send ||
        !flow->host->transport.send || packet_bytes < 4 ||
        packet_bytes > DBT_MAX_H4_BYTES - 5u || !packet_count)
        return DBT_INVALID;
    if (flow->state != DBT_RX_FLOW_DISABLED || flow->host->state != DBT_HOST_IDENTIFIED)
        return DBT_BUSY;
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i)
        if (flow->acl->links[i].used)
            return DBT_BUSY;
    if ((flow->host->controller.supported_commands[10] & 0xe0u) != 0xe0u)
        return DBT_UNSUPPORTED;
    /* One 255-byte SCO payload + four-byte H4 header + two-byte ring tag.
     * No separate SCO payload pool is allocated. */
    if ((uint32_t)packet_count * (packet_bytes + 7u) + 261u + DBT_CONTROL_RESERVE_BYTES > DBT_QUEUE_BYTES)
        return DBT_NO_SPACE;
    flow->packet_bytes = packet_bytes;
    flow->packet_count = packet_count;
    flow->state = DBT_RX_FLOW_BUFFER;
    return DBT_OK;
}

void dbt_rx_flow_poll(dbt_rx_flow_t *flow, uint32_t now_ms) {
    if (!flow || !flow->host || flow->state == DBT_RX_FLOW_DISABLED ||
        flow->state == DBT_RX_FLOW_FAILED)
        return;
    if (flow->host->state == DBT_HOST_FAILED) {
        flow->state = DBT_RX_FLOW_FAILED;
        return;
    }
    if (flow->state != DBT_RX_FLOW_ACTIVE) {
        if (flow->command_pending)
            return;
        uint8_t parameters[7];
        uint8_t size;
        uint16_t opcode;
        if (flow->state == DBT_RX_FLOW_BUFFER) {
            put16(parameters, flow->packet_bytes);
            parameters[2] = 255;
            put16(parameters + 3, flow->packet_count);
            put16(parameters + 5, 1);
            size = 7;
            opcode = 0x0c33;
        } else {
            parameters[0] = 1;
            size = 1;
            opcode = 0x0c31;
        }
        dbt_result_t result = dbt_host_command(flow->host, opcode, parameters, size,
                                               DBT_COMMAND_COMPLETE, configured, flow, now_ms);
        if (result == DBT_OK)
            flow->command_pending = true;
        else if (result != DBT_NO_SPACE && result != DBT_BUSY)
            fail(flow, DBT_FAULT_BAD_RESPONSE, 0);
        return;
    }
    dbt_transport_t transport = flow->host->transport;
    if (!transport.can_send(transport.context))
        return;
    unsigned count = 0;
    uint32_t packets = 0;
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i) {
        dbt_rx_completed_t *entry = &flow->pending[i];
        if (!entry->used || !entry->packets)
            continue;
        const dbt_link_t *link = dbt_acl_find(flow->acl, entry->handle);
        if (!link || link->generation != entry->generation) {
            flow->retired_on_disconnect += entry->packets;
            entry->used = false;
            entry->packets = 0;
            continue;
        }
        put16(flow->tx + 5u + count * 4u, entry->handle);
        put16(flow->tx + 7u + count * 4u, entry->packets);
        packets += entry->packets;
        ++count;
    }
    if (!count)
        return;
    flow->tx[0] = DBT_H4_COMMAND;
    flow->tx[1] = 0x35;
    flow->tx[2] = 0x0c;
    flow->tx[3] = (uint8_t)(1u + count * 4u);
    flow->tx[4] = (uint8_t)count;
    /* This command consumes no normal command credit and expects no response.
     * Its storage remains immutable until transport.can_send releases it. */
    if (!transport.send(transport.context, flow->tx, (uint16_t)(5u + count * 4u)))
        return;
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i) {
        flow->pending[i].packets = 0;
        flow->pending[i].used = false;
    }
    flow->acknowledged += packets;
    ++flow->commands_submitted;
}

dbt_result_t dbt_rx_flow_completed(dbt_rx_flow_t *flow, const dbt_link_t *link) {
    if (!flow || !link || !link->used)
        return DBT_INVALID;
    if (flow->state != DBT_RX_FLOW_ACTIVE)
        return DBT_BUSY;
    const dbt_link_t *current = dbt_acl_find(flow->acl, link->handle);
    if (!current || current->generation != link->generation || current->kind != link->kind)
        return DBT_INVALID;
    dbt_rx_completed_t *slot = 0;
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i) {
        dbt_rx_completed_t *entry = &flow->pending[i];
        if (entry->used && entry->handle == link->handle) {
            if (entry->generation != link->generation)
                return DBT_INVALID;
            slot = entry;
            break;
        }
        if (!entry->used && !slot)
            slot = entry;
    }
    if (!slot || (slot->used && slot->packets == UINT16_MAX))
        return DBT_NO_SPACE;
    if (!slot->used) {
        slot->handle = link->handle;
        slot->generation = link->generation;
        slot->packets = 0;
        slot->used = true;
    }
    ++slot->packets;
    return DBT_OK;
}
void dbt_rx_flow_link_change(dbt_rx_flow_t *flow, const dbt_link_change_t *change) {
    if (!flow || !change || change->kind != DBT_LINK_REMOVED)
        return;
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i) {
        dbt_rx_completed_t *entry = &flow->pending[i];
        if (entry->used && entry->handle == change->handle &&
            entry->generation == change->generation) {
            flow->retired_on_disconnect += entry->packets;
            entry->used = false;
            entry->packets = 0;
        }
    }
}
void dbt_rx_flow_event(dbt_rx_flow_t *flow, const dbt_hci_packet_t *packet) {
    if (!flow || !flow->host || flow->state == DBT_RX_FLOW_DISABLED ||
        flow->state == DBT_RX_FLOW_FAILED || !packet || packet->type != DBT_H4_EVENT ||
        packet->event != DBT_EVT_COMMAND_COMPLETE || !packet->payload || packet->size < 3 ||
        dbt_read_le16(packet->payload + 1) != 0x0c35)
        return;
    if (packet->size != 4)
        fail(flow, DBT_FAULT_BAD_RESPONSE, 0);
    else if (packet->payload[3]) {
        flow->failed_opcode=0x0c35;flow->failed_status=packet->payload[3];
        fail(flow, DBT_FAULT_CONTROLLER, packet->payload[3]);
    }
}
uint32_t dbt_rx_flow_wait_ms(const dbt_rx_flow_t *flow) {
    if (!flow || !flow->host || flow->host->state == DBT_HOST_FAILED)
        return UINT32_MAX;
    if ((flow->state == DBT_RX_FLOW_BUFFER || flow->state == DBT_RX_FLOW_ENABLE) &&
        !flow->command_pending && flow->host->command_count < DBT_COMMAND_SLOTS)
        return 0;
    if (flow->state == DBT_RX_FLOW_ACTIVE &&
        flow->host->transport.can_send(flow->host->transport.context))
        for (unsigned i = 0; i < DBT_MAX_LINKS; ++i)
            if (flow->pending[i].used)
                return 0;
    return UINT32_MAX;
}
void dbt_rx_flow_abort(dbt_rx_flow_t *flow) {
    if (!flow || flow->state == DBT_RX_FLOW_DISABLED)
        return;
    flow->state = DBT_RX_FLOW_FAILED;
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i) {
        flow->pending[i].packets = 0;
        flow->pending[i].used = false;
    }
}
