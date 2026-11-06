#include "dbt_l2cap.h"
#include <string.h>

static dbt_result_t reject_wire(dbt_l2cap_rx_t *rx, bool orphan) {
    ++rx->rejected;
    if (orphan) ++rx->orphaned;
    return DBT_INVALID;
}

static bool partial_active(const dbt_l2cap_rx_t *rx,const dbt_l2cap_partial_t *p){
    return p->data!=0 || p->expected>rx->max_pdu_payload;
}

static void release_partial(dbt_l2cap_rx_t *rx, dbt_l2cap_partial_t *partial) {
    if (partial->data != 0 && !rx->storage) rx->hooks.release(rx->hooks.context, partial->data);
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

dbt_result_t dbt_l2cap_rx_bind_storage(dbt_l2cap_rx_t *rx,uint8_t *storage,
                                     uint32_t bytes,uint16_t stride) {
    if (!rx || !storage || !rx->hooks.pdu || rx->storage ||
        ((uintptr_t)storage & 7u) || (stride & 7u) || stride < rx->max_pdu_payload ||
        !stride || bytes < (uint32_t)DBT_MAX_LINKS * stride) return DBT_INVALID;
    uintptr_t begin=(uintptr_t)storage,object=(uintptr_t)rx;
    if (begin<=object ? object-begin<bytes : begin-object<sizeof(*rx)) return DBT_INVALID;
    for (unsigned i=0;i<DBT_MAX_LINKS;++i) if (partial_active(rx,&rx->partial[i])) return DBT_BUSY;
    rx->storage=storage;rx->storage_stride=stride;
    return DBT_OK;
}

dbt_result_t dbt_l2cap_rx_set_oversize_handler(dbt_l2cap_rx_t *rx,dbt_l2cap_oversize_fn handler){
    if(!rx || !rx->hooks.pdu || !handler)return DBT_INVALID;
    for(unsigned i=0;i<DBT_MAX_LINKS;++i)if(partial_active(rx,&rx->partial[i]))return DBT_BUSY;
    rx->oversize=handler;return DBT_OK;
}

void dbt_l2cap_rx_clear(dbt_l2cap_rx_t *rx) {
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i) release_partial(rx, &rx->partial[i]);
}

void dbt_l2cap_link_change(dbt_l2cap_rx_t *rx, const dbt_link_change_t *change) {
    if (change->kind != DBT_LINK_REMOVED) return;
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i) {
        dbt_l2cap_partial_t *partial = &rx->partial[i];
        if (partial_active(rx,partial) && partial->handle == change->handle &&
            partial->generation == change->generation) release_partial(rx, partial);
    }
}

unsigned dbt_l2cap_expire(dbt_l2cap_rx_t *rx, uint32_t now_ms) {
    unsigned expired = 0;
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i) {
        dbt_l2cap_partial_t *partial = &rx->partial[i];
        if (partial_active(rx,partial) && (uint32_t)(now_ms - partial->deadline) < 0x80000000u) {
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
        (packet->size != 0 && packet->payload == 0)) return DBT_INVALID;
    if (packet->bc != 0 || (packet->pb != 1 && packet->pb != 2)) return reject_wire(rx, false);
    const dbt_link_t *link = dbt_acl_find(acl, packet->handle);
    if (link == 0) return reject_wire(rx, false);
    /* The ACL owner already identifies a stable link slot. Reassembly follows
     * that slot directly instead of searching a second table per fragment. */
    unsigned index=(unsigned)(link-acl->links);
    dbt_l2cap_partial_t *slot=&rx->partial[index];
    if (!rx->storage) (void)dbt_l2cap_expire(rx, now_ms);
    dbt_l2cap_partial_t *current=partial_active(rx,slot)?slot:0;
    if (current && (current->handle!=link->handle || current->generation!=link->generation ||
                   (uint32_t)(now_ms-current->deadline)<0x80000000u)) {
        release_partial(rx,current);++rx->abandoned;current=0;
    }
    if (packet->pb == 1) {
        if (current == 0) return reject_wire(rx, true);
        if (packet->size == 0) return reject_wire(rx, false);
        if ((uint32_t)current->received + packet->size > current->expected) {
            release_partial(rx, current);
            ++rx->abandoned;
            return reject_wire(rx, false);
        }
        uint16_t offset=current->received;
        current->received += packet->size;
        bool last=current->received==current->expected;
        dbt_result_t result=DBT_OK;
        if(current->expected>rx->max_pdu_payload){
            if(!rx->oversize)return DBT_INVALID;
            result=rx->oversize(rx->hooks.context,link->handle,link->generation,current->cid,
                packet->payload,packet->size,current->expected,offset,false,last);
        }else{
            memcpy(current->data+offset,packet->payload,packet->size);
            if(last){
                rx->hooks.pdu(rx->hooks.context,link->handle,link->generation,current->cid,current->data,current->expected);
                ++rx->delivered;
            }
        }
        if(last)release_partial(rx,current);
        return result;
    }
    if (current != 0) {
        release_partial(rx, current);
        ++rx->abandoned;
    }
    if (packet->size < 4) return reject_wire(rx, false);
    uint16_t expected = dbt_read_le16(packet->payload);
    uint16_t cid = dbt_read_le16(packet->payload + 2);
    uint16_t received = packet->size - 4u;
    if (cid == 0 || received > expected) return reject_wire(rx, false);
    if (expected > rx->max_pdu_payload){
        ++rx->oversized;
        if(!rx->oversize)return DBT_NO_SPACE;
        /* expected > max is the discard-state discriminator. No data buffer
         * or allocation belongs to this partial; preserve framing/generation. */
        slot->handle=link->handle;slot->generation=link->generation;slot->cid=cid;
        slot->expected=expected;slot->received=received;slot->deadline=now_ms+rx->timeout_ms;
        bool last=received==expected;
        dbt_result_t result=rx->oversize(rx->hooks.context,link->handle,link->generation,cid,
            packet->payload+4u,received,expected,0,true,last);
        /* Only an accepted containment policy is a local wire rejection. An
         * INVALID policy result must remain an API/invariant failure. */
        if(result==DBT_OK)++rx->rejected;
        if(last)release_partial(rx,slot);
        return result;
    }
    if (received == expected) {
        rx->hooks.pdu(rx->hooks.context, link->handle, link->generation,
                      cid, packet->payload + 4, expected);
        ++rx->delivered;
        return DBT_OK;
    }
    uint8_t *data = rx->storage?rx->storage+(uint32_t)index*rx->storage_stride:
                   rx->hooks.allocate(rx->hooks.context, expected);
    if (data == 0) return DBT_NO_SPACE;
    slot->data = data;
    slot->handle = link->handle;
    slot->generation = link->generation;
    slot->cid = cid;
    slot->expected = expected;
    slot->received = received;
    slot->deadline = now_ms + rx->timeout_ms;
    memcpy(data,packet->payload+4u,received);
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
    if(tx->prefix_size>sizeof(tx->prefix) || tx->size<tx->prefix_size ||
       (tx->size>tx->prefix_size && !tx->data))return DBT_INVALID;
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
    /* The logical PDU has three disjoint spans: basic header, optional copied
     * prefix and borrowed payload. Copy each intersected span once, including
     * fragments that end inside the header/prefix. No per-byte dispatch. */
    uint32_t position=tx->cursor,remaining=payload;uint8_t *destination=output+5u;
    if(position<4u){
        uint32_t span=4u-position;if(span>remaining)span=remaining;
        memcpy(destination,header+position,span);destination+=span;position+=span;remaining-=span;
    }
    if(remaining && position<4u+tx->prefix_size){
        uint32_t span=4u+tx->prefix_size-position;if(span>remaining)span=remaining;
        memcpy(destination,tx->prefix+(position-4u),span);destination+=span;position+=span;remaining-=span;
    }
    if(remaining)memcpy(destination,tx->data+(position-4u-tx->prefix_size),remaining);
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
