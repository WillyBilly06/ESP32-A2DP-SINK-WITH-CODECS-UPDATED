#include "dbt_acl.h"

static int index_for(const dbt_acl_t *acl, uint16_t handle) {
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i) {
        if (acl->links[i].used && acl->links[i].handle == handle) return (int)i;
    }
    return -1;
}
static unsigned pool_for(const dbt_acl_t *acl, dbt_link_kind_t kind) {
    return acl->shared_pool ? 0u : (unsigned)kind;
}

dbt_result_t dbt_acl_init(dbt_acl_t *acl, const dbt_controller_info_t *controller) {
    if (acl == 0 || controller == 0 || controller->classic_acl_bytes == 0 ||
        controller->classic_acl_packets == 0 || controller->le_acl_bytes == 0 ||
        controller->le_acl_packets == 0) return DBT_INVALID;
    uint8_t *bytes = (uint8_t *)acl;
    for (unsigned i = 0; i < sizeof(*acl); ++i) bytes[i] = 0;
    acl->shared_pool = controller->shared_acl_pool;
    acl->mtu[0] = controller->classic_acl_bytes;
    acl->mtu[1] = controller->le_acl_bytes;
    acl->pools[0].capacity = controller->classic_acl_packets;
    acl->pools[0].available = controller->classic_acl_packets;
    if (!acl->shared_pool) {
        acl->pools[1].capacity = controller->le_acl_packets;
        acl->pools[1].available = controller->le_acl_packets;
    }
    return DBT_OK;
}

const dbt_link_t *dbt_acl_find(const dbt_acl_t *acl, uint16_t handle) {
    int index = index_for(acl, handle);
    return index < 0 ? 0 : &acl->links[index];
}

static dbt_result_t connected(dbt_acl_t *acl, uint16_t handle, dbt_link_kind_t kind,
                               const uint8_t *peer, uint8_t address_type, uint8_t role,
                               bool encrypted, dbt_link_change_t *change) {
    if (handle > 0x0effu || index_for(acl, handle) >= 0) return DBT_INVALID;
    if (acl->generation == UINT32_MAX) return DBT_NO_SPACE;
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i) {
        dbt_link_t *link = &acl->links[i];
        if (link->used) continue;
        link->used = true;
        link->handle = handle;
        link->generation = ++acl->generation;
        link->outstanding = 0;
        link->kind = kind;
        link->role = role;
        link->peer_address_type = address_type;
        link->encrypted = encrypted;
        for (unsigned j = 0; j < 6; ++j) link->peer[j] = peer[j];
        change->kind = DBT_LINK_ADDED;
        change->handle = handle;
        change->generation = link->generation;
        return DBT_OK;
    }
    return DBT_NO_SPACE;
}

static dbt_result_t completed(dbt_acl_t *acl, const uint8_t *data, uint16_t size) {
    if (size == 0 || size != 1u + 4u * data[0]) return DBT_INVALID;
    uint32_t counts[DBT_MAX_LINKS] = {0};
    uint32_t unknown = 0;
    /* Validate and aggregate the whole event before changing any credit. This
     * includes duplicate handle records, avoiding partial updates on bad input. */
    for (unsigned i = 0; i < data[0]; ++i) {
        uint16_t handle = dbt_read_le16(data + 1u + 4u * i);
        uint16_t count = dbt_read_le16(data + 3u + 4u * i);
        if (handle > 0x0effu) return DBT_INVALID;
        int index = index_for(acl, handle);
        if (index < 0) unknown += count;
        else counts[index] += count;
    }
    uint32_t released[2] = {0, 0};
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i) {
        if (counts[i] > acl->links[i].outstanding) return DBT_INVALID;
        released[pool_for(acl, acl->links[i].kind)] += counts[i];
    }
    for (unsigned i = 0; i < 2; ++i) {
        if (released[i] + acl->pools[i].available > acl->pools[i].capacity) return DBT_INVALID;
    }
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i) acl->links[i].outstanding -= (uint16_t)counts[i];
    for (unsigned i = 0; i < 2; ++i) acl->pools[i].available += (uint16_t)released[i];
    acl->unknown_completions += unknown;
    return DBT_OK;
}

dbt_result_t dbt_acl_event(dbt_acl_t *acl, const dbt_hci_packet_t *packet,
                          dbt_link_change_t *change) {
    if (acl == 0 || packet == 0 || change == 0 || packet->type != DBT_H4_EVENT) return DBT_INVALID;
    change->kind = DBT_LINK_UNCHANGED;
    change->handle = 0;
    change->generation = 0;
    const uint8_t *data = packet->payload;
    switch (packet->event) {
    case 0x03: /* Connection Complete, ACL only; SCO has a separate future owner. */
        if (packet->size != 11) return DBT_INVALID;
        if (data[0] != 0) return DBT_OK;
        if (data[9] != 1) return DBT_UNSUPPORTED;
        return connected(acl, dbt_read_le16(data + 1), DBT_LINK_CLASSIC,
                         data + 3, 0, 0xff, data[10] != 0, change);
    case 0x3e: { /* LE Connection Complete and LE Enhanced Connection Complete. */
        if (packet->size == 0) return DBT_INVALID;
        bool enhanced = data[0] == 0x0a;
        if (data[0] != 1 && !enhanced) return DBT_UNSUPPORTED;
        if (packet->size != (enhanced ? 31 : 19)) return DBT_INVALID;
        if (data[1] != 0) return DBT_OK;
        if (data[4] > 1 || data[5] > (enhanced ? 3 : 1)) return DBT_INVALID;
        return connected(acl, dbt_read_le16(data + 2), DBT_LINK_LE,
                         data + 6, data[5], data[4], false, change);
    }
    case 0x05: { /* Disconnection Complete frees unacknowledged controller buffers. */
        if (packet->size != 4) return DBT_INVALID;
        if (data[0] != 0) return DBT_OK;
        int index = index_for(acl, dbt_read_le16(data + 1));
        if (index < 0) return DBT_UNSUPPORTED;
        dbt_link_t *link = &acl->links[index];
        dbt_acl_pool_t *pool = &acl->pools[pool_for(acl, link->kind)];
        if ((uint32_t)pool->available + link->outstanding > pool->capacity) return DBT_INVALID;
        pool->available += link->outstanding;
        change->kind = DBT_LINK_REMOVED;
        change->handle = link->handle;
        change->generation = link->generation;
        link->outstanding = 0;
        link->used = false;
        link->encrypted = false;
        return DBT_OK;
    }
    case 0x08: { /* Encryption Change; authentication/key-size policy remains separate. */
        if (packet->size != 4) return DBT_INVALID;
        int index = index_for(acl, dbt_read_le16(data + 1));
        if (index < 0) return DBT_UNSUPPORTED;
        if (data[0] == 0) acl->links[index].encrypted = data[3] != 0;
        return DBT_OK;
    }
    case 0x13: return completed(acl, data, packet->size);
    default: return DBT_UNSUPPORTED;
    }
}

dbt_result_t dbt_acl_submit(dbt_acl_t *acl, dbt_transport_t transport,
                           uint8_t *packet, uint16_t size) {
    if (acl == 0 || transport.can_send == 0 || transport.send == 0) return DBT_INVALID;
    dbt_hci_packet_t view;
    if (dbt_hci_decode(packet, size, &view) != DBT_OK || view.type != DBT_H4_ACL ||
        view.size == 0 || view.bc != 0 || view.pb == 3) return DBT_INVALID;
    int index = index_for(acl, view.handle);
    if (index < 0) return DBT_INVALID;
    dbt_link_t *link = &acl->links[index];
    if (link->kind == DBT_LINK_LE && view.pb == 2) return DBT_INVALID;
    if (view.size > acl->mtu[link->kind]) return DBT_NO_SPACE;
    dbt_acl_pool_t *pool = &acl->pools[pool_for(acl, link->kind)];
    if (pool->available == 0 || link->outstanding == UINT16_MAX ||
        !transport.can_send(transport.context)) return DBT_BUSY;
    if (!transport.send(transport.context, packet, size)) return DBT_BUSY;
    --pool->available;
    ++link->outstanding;
    return DBT_OK;
}
