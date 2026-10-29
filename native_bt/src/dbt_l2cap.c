#include "dbt_l2cap.h"

static void release_partial(dbt_l2cap_rx_t *rx, dbt_l2cap_partial_t *partial) {
    if (partial->data != 0) rx->hooks.release(rx->hooks.context, partial->data);
    partial->data = 0;
    partial->received = partial->expected = 0;
}

dbt_result_t dbt_l2cap_rx_init(dbt_l2cap_rx_t *rx, dbt_l2cap_hooks_t hooks,
                              uint16_t max_payload, uint32_t timeout_ms) {
    if (rx == 0 || hooks.allocate == 0 || hooks.release == 0 || hooks.pdu == 0 ||
        max_payload == 0 || timeout_ms == 0 || timeout_ms >= 0x80000000u) return DBT_INVALID;
    uint8_t *bytes = (uint8_t *)rx;
    for (unsigned i = 0; i < sizeof(*rx); ++i) bytes[i] = 0;
    rx->hooks = hooks;
    rx->max_pdu_payload = max_payload;
    rx->timeout_ms = timeout_ms;
    return DBT_OK;
}

void dbt_l2cap_rx_clear(dbt_l2cap_rx_t *rx) {
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i) release_partial(rx, &rx->partial[i]);
}

void dbt_l2cap_link_change(dbt_l2cap_rx_t *rx, const dbt_link_change_t *change) {
    if (change->kind != DBT_LINK_REMOVED) return;
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i) {
        dbt_l2cap_partial_t *partial = &rx->partial[i];
        if (partial->data != 0 && partial->handle == change->handle &&
            partial->generation == change->generation) release_partial(rx, partial);
    }
}

unsigned dbt_l2cap_expire(dbt_l2cap_rx_t *rx, uint32_t now_ms) {
    unsigned expired = 0;
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i) {
        dbt_l2cap_partial_t *partial = &rx->partial[i];
        if (partial->data != 0 && (uint32_t)(now_ms - partial->deadline) < 0x80000000u) {
            release_partial(rx, partial);
            ++expired;
            ++rx->abandoned;
        }
    }
    return expired;
}

dbt_result_t dbt_l2cap_receive(dbt_l2cap_rx_t *rx, const dbt_acl_t *acl,
                              const dbt_hci_packet_t *packet, uint32_t now_ms) {
    if (rx == 0 || acl == 0 || packet == 0 || packet->type != DBT_H4_ACL ||
        packet->bc != 0 || (packet->size != 0 && packet->payload == 0) ||
        (packet->pb != 1 && packet->pb != 2)) return DBT_INVALID;
    const dbt_link_t *link = dbt_acl_find(acl, packet->handle);
    if (link == 0) return DBT_INVALID;
    dbt_l2cap_partial_t *current = 0;
    dbt_l2cap_partial_t *empty = 0;
    (void)dbt_l2cap_expire(rx, now_ms);
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i) {
        dbt_l2cap_partial_t *partial = &rx->partial[i];
        if (partial->data != 0 && partial->handle == link->handle) {
            if (partial->generation == link->generation) current = partial;
            else { release_partial(rx, partial); ++rx->abandoned; }
        }
        if (partial->data == 0) empty = partial;
    }
    if (packet->pb == 1) {
        if (current == 0 || packet->size == 0) return DBT_INVALID;
        if ((uint32_t)current->received + packet->size > current->expected) {
            release_partial(rx, current);
            ++rx->abandoned;
            return DBT_INVALID;
        }
        for (unsigned i = 0; i < packet->size; ++i) {
            current->data[current->received + i] = packet->payload[i];
        }
        current->received += packet->size;
        if (current->received == current->expected) {
            rx->hooks.pdu(rx->hooks.context, link->handle, link->generation,
                          current->cid, current->data, current->expected);
            ++rx->delivered;
            release_partial(rx, current);
        }
        return DBT_OK;
    }
    if (current != 0) {
        release_partial(rx, current);
        ++rx->abandoned;
        empty = current;
    }
    if (packet->size < 4) return DBT_INVALID;
    uint16_t expected = dbt_read_le16(packet->payload);
    uint16_t cid = dbt_read_le16(packet->payload + 2);
    uint16_t received = packet->size - 4u;
    if (cid == 0 || received > expected) return DBT_INVALID;
    if (expected > rx->max_pdu_payload) return DBT_NO_SPACE;
    if (received == expected) {
        rx->hooks.pdu(rx->hooks.context, link->handle, link->generation,
                      cid, packet->payload + 4, expected);
        ++rx->delivered;
        return DBT_OK;
    }
    if (empty == 0) return DBT_NO_SPACE;
    uint8_t *data = rx->hooks.allocate(rx->hooks.context, expected);
    if (data == 0) return DBT_NO_SPACE;
    empty->data = data;
    empty->handle = link->handle;
    empty->generation = link->generation;
    empty->cid = cid;
    empty->expected = expected;
    empty->received = received;
    empty->deadline = now_ms + rx->timeout_ms;
    for (unsigned i = 0; i < received; ++i) data[i] = packet->payload[4u + i];
    return DBT_OK;
}

dbt_result_t dbt_l2cap_tx_begin(dbt_l2cap_tx_t *tx, const dbt_link_t *link,
                               uint16_t cid, const uint8_t *data, uint16_t size) {
    if (tx == 0 || link == 0 || !link->used || cid == 0 || (size != 0 && data == 0)) return DBT_INVALID;
    tx->data = data;
    tx->generation = link->generation;
    tx->cursor = 0;
    tx->pending = 0;
    tx->handle = link->handle;
    tx->cid = cid;
    tx->size = size;
    tx->prefix_size=0;tx->prefix[0]=tx->prefix[1]=0;
    return DBT_OK;
}
dbt_result_t dbt_l2cap_tx_begin_credit(dbt_l2cap_tx_t *tx,const dbt_link_t *link,
    uint16_t cid,const uint8_t *data,uint16_t size,bool first,uint16_t sdu_length){
    if(first && size>UINT16_MAX-2u)return DBT_INVALID;
    dbt_result_t result=dbt_l2cap_tx_begin(tx,link,cid,data,size);if(result!=DBT_OK)return result;
    if(first){tx->prefix[0]=(uint8_t)sdu_length;tx->prefix[1]=(uint8_t)(sdu_length>>8);tx->prefix_size=2;tx->size=(uint16_t)(size+2u);}
    return DBT_OK;
}

dbt_result_t dbt_l2cap_tx_peek(dbt_l2cap_tx_t *tx, const dbt_acl_t *acl,
                              uint8_t *output, uint16_t capacity, uint16_t *written) {
    if (tx == 0 || acl == 0 || output == 0 || written == 0) return DBT_INVALID;
    tx->pending = 0;
    const dbt_link_t *link = dbt_acl_find(acl, tx->handle);
    if (link == 0 || link->generation != tx->generation) return DBT_INVALID;
    uint32_t total = 4u + tx->size;
    if (tx->cursor == total) return DBT_EMPTY;
    if (tx->cursor > total) return DBT_INVALID;
    uint32_t payload = total - tx->cursor;
    uint16_t mtu = acl->mtu[link->kind];
    /* The first HCI fragment must contain the complete L2CAP basic header. */
    if (mtu < 4) return DBT_UNSUPPORTED;
    if (payload > mtu) payload = mtu;
    if (payload + 5u > capacity || payload + 5u > DBT_MAX_H4_BYTES) return DBT_NO_SPACE;
    uint8_t pb = tx->cursor == 0 ? 0 : 1; /* Non-flushable start for both transports. */
    output[0] = DBT_H4_ACL;
    output[1] = (uint8_t)tx->handle;
    output[2] = (uint8_t)((tx->handle >> 8) | (pb << 4));
    output[3] = (uint8_t)payload;
    output[4] = (uint8_t)(payload >> 8);
    uint8_t header[4] = {(uint8_t)tx->size, (uint8_t)(tx->size >> 8), (uint8_t)tx->cid, (uint8_t)(tx->cid >> 8)};
    for (uint32_t i = 0; i < payload; ++i) {
        uint32_t position = tx->cursor + i;
        output[5u+i]=position<4u?header[position]:position<4u+tx->prefix_size?
            tx->prefix[position-4u]:tx->data[position-4u-tx->prefix_size];
    }
    *written = (uint16_t)(5u + payload);
    tx->pending = (uint16_t)payload;
    return DBT_OK;
}

dbt_result_t dbt_l2cap_tx_commit(dbt_l2cap_tx_t *tx) {
    if (tx == 0 || tx->pending == 0 || tx->cursor + tx->pending > 4u + tx->size) return DBT_INVALID;
    tx->cursor += tx->pending;
    tx->pending = 0;
    return DBT_OK;
}

dbt_result_t dbt_l2cap_send_next(dbt_l2cap_tx_t *tx, dbt_acl_t *acl,
                                dbt_transport_t transport, uint8_t *output, uint16_t capacity) {
    if (transport.can_send == 0 || transport.send == 0) return DBT_INVALID;
    /* Never overwrite a buffer still owned by an asynchronous transport. */
    if (!transport.can_send(transport.context)) return DBT_BUSY;
    uint16_t written = 0;
    dbt_result_t result = dbt_l2cap_tx_peek(tx, acl, output, capacity, &written);
    if (result != DBT_OK) return result;
    result = dbt_acl_submit(acl, transport, output, written);
    return result == DBT_OK ? dbt_l2cap_tx_commit(tx) : result;
}
