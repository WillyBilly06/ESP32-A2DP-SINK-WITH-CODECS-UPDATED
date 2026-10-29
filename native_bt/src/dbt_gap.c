#include "dbt_gap.h"

static void copy(uint8_t *out, const uint8_t *in, unsigned size) {
    for (unsigned i = 0; i < size; ++i) out[i] = in[i];
}
static void put16(uint8_t *out, uint16_t value) { out[0] = (uint8_t)value; out[1] = (uint8_t)(value >> 8); }
static void mask_event(uint8_t mask[8], uint8_t event) {
    mask[(event - 1u) / 8u] |= (uint8_t)(1u << ((event - 1u) % 8u));
}
static void configured(void *context, uint16_t opcode, dbt_command_outcome_t outcome,
                         uint8_t status, const uint8_t *returns, uint8_t size) {
    dbt_gap_t *gap = context;
    (void)opcode; (void)returns;
    gap->command_pending = false; gap->last_status = status;
    if (outcome != DBT_COMMAND_SUCCESS || size != 1) { gap->state = DBT_GAP_FAILED; return; }
    if (++gap->step == 9) gap->state = DBT_GAP_READY;
}
static void replied(void *context, uint16_t opcode, dbt_command_outcome_t outcome,
                      uint8_t status, const uint8_t *returns, uint8_t size) {
    dbt_gap_t *gap = context;
    (void)opcode; (void)returns; (void)size;
    if (outcome != DBT_COMMAND_SUCCESS) { ++gap->rejected_commands; gap->last_status = status; }
    if (outcome == DBT_COMMAND_ABORTED || outcome == DBT_COMMAND_TIMEOUT) gap->state = DBT_GAP_FAILED;
}
static void visibility(void *context, uint16_t opcode, dbt_command_outcome_t outcome,
                        uint8_t status, const uint8_t *returns, uint8_t size) {
    dbt_gap_t *gap = context;
    (void)returns;
    bool success = outcome == DBT_COMMAND_SUCCESS && size == 1;
    gap->last_status = status;
    if (opcode == 0x0c1a) {
        gap->classic_pending = false;
        if (success) gap->classic_scan = gap->wanted_scan;
    } else {
        gap->advertising_pending = false;
        if (success) gap->advertising = gap->wanted_advertising;
    }
    if (!success) ++gap->rejected_commands;
    if (outcome == DBT_COMMAND_ABORTED || outcome == DBT_COMMAND_TIMEOUT) gap->state = DBT_GAP_FAILED;
}
dbt_result_t dbt_gap_init(dbt_gap_t *gap, dbt_host_t *host, dbt_gap_config_t config, dbt_gap_hooks_t hooks) {
    if (!gap || !host || config.advertising_size > 31 || config.scan_response_size > 31 ||
        config.advertising_minimum < 0x20 || config.advertising_maximum > 0x4000 ||
        config.advertising_minimum > config.advertising_maximum) return DBT_INVALID;
    bool terminated = false;
    for (unsigned i = 0; i < sizeof(config.name); ++i) if (!config.name[i]) terminated = true;
    if (!terminated) return DBT_INVALID;
    uint8_t *bytes = (uint8_t *)gap;
    for (unsigned i = 0; i < sizeof(*gap); ++i) bytes[i] = 0;
    gap->host = host; gap->config = config; gap->hooks = hooks;
    return DBT_OK;
}
dbt_result_t dbt_gap_start(dbt_gap_t *gap) {
    if (!gap || gap->state != DBT_GAP_IDLE || gap->host->state != DBT_HOST_IDENTIFIED) return DBT_BUSY;
    gap->state = DBT_GAP_CONFIGURING;
    return DBT_OK;
}
static dbt_result_t configure_next(dbt_gap_t *gap) {
    uint8_t data[248] = {0}; uint8_t size = 0; uint16_t opcode = 0;
    switch (gap->step) {
    case 0: {
        opcode = 0x0c01; size = 8;
        static const uint8_t events[] = {3,4,5,6,7,8,0x0b,0x0c,0x0e,0x0f,0x10,0x12,0x13,
            0x16,0x17,0x18,0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x3b,0x3e};
        for (unsigned i = 0; i < sizeof(events); ++i) mask_event(data, events[i]);
        break;
    }
    case 1: opcode = 0x2001; size = 8; data[0] = 0x1f; break; /* LE v4.0 events 1..5. */
    case 2: opcode = 0x0c6d; size = 2; data[0] = 1; break; /* LE Supported Host. */
    case 3:
        opcode = 0x0c13; size = 248;
        for (unsigned i = 0; i < sizeof(gap->config.name) && gap->config.name[i]; ++i) data[i] = (uint8_t)gap->config.name[i];
        break;
    case 4: opcode = 0x0c24; size = 3; copy(data, gap->config.class_of_device, 3); break;
    case 5: opcode = 0x0c56; size = 1; data[0] = 1; break; /* SSP enabled; security owner still required. */
    case 6:
        opcode = 0x2006; size = 15;
        put16(data, gap->config.advertising_minimum); put16(data + 2, gap->config.advertising_maximum);
        data[13] = 7; break; /* ADV_IND, public address, all primary channels, no filter. */
    case 7:
        opcode = 0x2008; size = 32; data[0] = gap->config.advertising_size;
        copy(data + 1, gap->config.advertising, data[0]); break;
    case 8:
        opcode = 0x2009; size = 32; data[0] = gap->config.scan_response_size;
        copy(data + 1, gap->config.scan_response, data[0]); break;
    default: return DBT_INVALID;
    }
    return dbt_host_command(gap->host, opcode, data, size, DBT_COMMAND_COMPLETE, configured, gap, gap->now_ms);
}
void dbt_gap_poll(dbt_gap_t *gap, uint32_t now_ms) {
    if (!gap) return;
    gap->now_ms = now_ms;
    if (gap->host->state == DBT_HOST_FAILED) { gap->state = DBT_GAP_FAILED; return; }
    if (gap->state == DBT_GAP_FAILED || gap->state == DBT_GAP_IDLE) return;
    /* Security and connection decisions precede nonurgent setup commands. */
    if (gap->reply_count) {
        dbt_gap_reply_t *reply = &gap->replies[gap->reply_head];
        dbt_result_t result = dbt_host_command(gap->host, reply->opcode, reply->bytes, reply->size,
                                              reply->response, replied, gap, now_ms);
        if (result == DBT_OK) {
            gap->reply_head = (uint8_t)((gap->reply_head + 1u) % DBT_GAP_REPLY_SLOTS); --gap->reply_count;
        }
    }
    if (gap->state == DBT_GAP_CONFIGURING && !gap->command_pending) {
        dbt_result_t result = configure_next(gap);
        if (result == DBT_OK) gap->command_pending = true;
        else if (result != DBT_NO_SPACE && result != DBT_BUSY) gap->state = DBT_GAP_FAILED;
    }
}
static dbt_result_t reply(dbt_gap_t *gap, uint16_t opcode, const uint8_t *data, uint8_t size,
                            dbt_command_response_t response) {
    if (gap->reply_count == DBT_GAP_REPLY_SLOTS) return DBT_NO_SPACE;
    dbt_gap_reply_t *slot = &gap->replies[(gap->reply_head + gap->reply_count) % DBT_GAP_REPLY_SLOTS];
    slot->opcode = opcode; slot->response = response; slot->size = size; copy(slot->bytes, data, size);
    ++gap->reply_count;
    return DBT_OK;
}
dbt_result_t dbt_gap_event(dbt_gap_t *gap, const dbt_hci_packet_t *packet) {
    if (!gap || !packet || packet->type != DBT_H4_EVENT) return DBT_INVALID;
    if (gap->state == DBT_GAP_IDLE || gap->state == DBT_GAP_FAILED) return DBT_BUSY;
    const uint8_t *data = packet->payload;
    if (packet->size && !data) return DBT_INVALID;
    uint8_t bytes[7] = {0}; uint16_t opcode = 0; uint16_t expected = 0;
    switch (packet->event) {
    case 0x04: { /* Connection Request. Status is admission, Connection Complete is final. */
        if (packet->size != 10) return DBT_INVALID;
        copy(bytes, data, 6);
        bool accepted = data[9] == 1 && gap->state == DBT_GAP_READY && gap->hooks.accept_classic &&
            gap->hooks.accept_classic(gap->hooks.context, data, data + 6);
        bytes[6] = accepted ? 1 : 0x0f;
        return reply(gap, accepted ? 0x0409 : 0x040a, bytes, 7, DBT_COMMAND_STATUS);
    }
    case 0x16: opcode = 0x040e; expected = 6; break; /* PIN negative. */
    case 0x17: opcode = 0x040c; expected = 6; break; /* Link key negative. */
    case 0x31: opcode = 0x0434; expected = 6; break; /* IO capability negative. */
    case 0x33: opcode = 0x042d; expected = 10; break; /* User confirmation negative. */
    case 0x34: opcode = 0x042f; expected = 6; break; /* Passkey negative. */
    case 0x35: opcode = 0x0433; expected = 6; break; /* OOB negative. */
    case 0x3e:
        if (!packet->size) return DBT_INVALID;
        if (data[0] == 1 || data[0] == 0x0a) {
            if (packet->size != (data[0] == 1 ? 19 : 31)) return DBT_INVALID;
            /* Advertising stops when a peripheral connection is established. */
            if (data[1] == 0 && data[4] == 1) gap->advertising = false;
            return DBT_OK;
        }
        if (data[0] != 5) return DBT_UNSUPPORTED;
        if (packet->size != 13) return DBT_INVALID;
        if (gap->hooks.security && gap->hooks.security(gap->hooks.context, packet)) return DBT_OK;
        return reply(gap, 0x201b, data + 1, 2, DBT_COMMAND_COMPLETE);
    default: return DBT_UNSUPPORTED;
    }
    if (packet->size != expected) return DBT_INVALID;
    if (gap->hooks.security && gap->hooks.security(gap->hooks.context, packet)) return DBT_OK;
    copy(bytes, data, 6); bytes[6] = 0x18; /* Pairing Not Allowed. */
    return reply(gap, opcode, bytes, opcode == 0x0434 ? 7 : 6, DBT_COMMAND_COMPLETE);
}
uint32_t dbt_gap_wait_ms(const dbt_gap_t *gap) {
    if (!gap || gap->state == DBT_GAP_FAILED || gap->state == DBT_GAP_IDLE) return UINT32_MAX;
    if (gap->host->command_count == DBT_COMMAND_SLOTS) return UINT32_MAX;
    return gap->reply_count || (gap->state == DBT_GAP_CONFIGURING && !gap->command_pending) ? 0 : UINT32_MAX;
}
dbt_result_t dbt_gap_set_classic_scan(dbt_gap_t *gap, bool discoverable, bool connectable) {
    if (!gap || gap->state != DBT_GAP_READY || gap->classic_pending) return DBT_BUSY;
    uint8_t scan = (uint8_t)((discoverable ? 1 : 0) | (connectable ? 2 : 0));
    dbt_result_t result = dbt_host_command(gap->host, 0x0c1a, &scan, 1, DBT_COMMAND_COMPLETE, visibility, gap, gap->now_ms);
    if (result == DBT_OK) { gap->wanted_scan = scan; gap->classic_pending = true; }
    return result;
}
dbt_result_t dbt_gap_set_advertising(dbt_gap_t *gap, bool enabled) {
    if (!gap || gap->state != DBT_GAP_READY || gap->advertising_pending) return DBT_BUSY;
    uint8_t flag = enabled ? 1 : 0;
    dbt_result_t result = dbt_host_command(gap->host, 0x200a, &flag, 1, DBT_COMMAND_COMPLETE, visibility, gap, gap->now_ms);
    if (result == DBT_OK) { gap->wanted_advertising = enabled; gap->advertising_pending = true; }
    return result;
}
dbt_result_t dbt_gap_disconnect(dbt_gap_t *gap, const dbt_link_t *link, uint8_t reason) {
    if (!gap || !link || !link->used || gap->state != DBT_GAP_READY ||
        (reason != 0x05 && reason != 0x13 && reason != 0x14 && reason != 0x15 && reason != 0x1a && reason != 0x29 && reason != 0x3b)) return DBT_INVALID;
    uint8_t bytes[3]; put16(bytes, link->handle); bytes[2] = reason;
    return dbt_host_command(gap->host, 0x0406, bytes, 3, DBT_COMMAND_STATUS, replied, gap, gap->now_ms);
}
