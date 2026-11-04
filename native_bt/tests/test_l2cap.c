#include "dbt_l2cap.h"
#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif
#define CHECK(expression) do { if (!(expression)) return __LINE__; } while (0)

static dbt_acl_t acl;
static dbt_l2cap_rx_t rx;
static dbt_l2cap_tx_t tx;
static uint8_t pool[DBT_MAX_LINKS][128], output[128], message[100], delivered[128];
static bool used[DBT_MAX_LINKS], allocation_fails, transport_ready, transport_accepts;
static unsigned allocations, frees, callbacks, sent, bad_frees;
static uint16_t allocation_size, delivered_size, delivered_cid, delivered_handle, last_wire_size;
static uint8_t last_wire[128];
static uint8_t *allocate(void *context, uint16_t size) {
    (void)context;
    if (allocation_fails || size > 128) return 0;
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i) {
        if (!used[i]) { used[i] = true; ++allocations; allocation_size = size; return pool[i]; }
    }
    return 0;
}
static void release(void *context, uint8_t *data) {
    (void)context;
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i) {
        if (data == pool[i] && used[i]) { used[i] = false; ++frees; return; }
    }
    ++bad_frees;
}
static void pdu(void *context, uint16_t handle, uint32_t generation, uint16_t cid,
                 const uint8_t *data, uint16_t size) {
    (void)context; (void)generation;
    ++callbacks; delivered_handle = handle; delivered_size = size; delivered_cid = cid;
    for (unsigned i = 0; i < size && i < sizeof(delivered); ++i) delivered[i] = data[i];
}
static bool ready(void *context) { (void)context; return transport_ready; }
static bool send(void *context, uint8_t *data, uint16_t size) {
    (void)context;
    if (!transport_accepts) return false;
    ++sent; last_wire_size = size;
    for (unsigned i = 0; i < size; ++i) last_wire[i] = data[i];
    return true;
}
static const dbt_transport_t transport = {0, ready, send};
static dbt_result_t link_event(uint8_t code, const uint8_t *data, uint16_t size, dbt_link_change_t *change) {
    dbt_hci_packet_t packet = {0};
    packet.type = DBT_H4_EVENT; packet.event = code; packet.payload = data; packet.size = size;
    return dbt_acl_event(&acl, &packet, change);
}
static void fixture(void) {
    allocations = frees = callbacks = sent = bad_frees = 0;
    allocation_fails = false; transport_ready = transport_accepts = true;
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i) used[i] = false;
    for (unsigned i = 0; i < sizeof(message); ++i) message[i] = (uint8_t)i;
    dbt_controller_info_t info = {0};
    info.classic_acl_bytes = 27; info.classic_acl_packets = 3;
    info.le_acl_bytes = 27; info.le_acl_packets = 2;
    (void)dbt_acl_init(&acl, &info);
    dbt_link_change_t change;
    uint8_t event[19] = {1, 0, 1, 0, 1, 0};
    (void)link_event(0x3e, event, sizeof(event), &change);
    event[2] = 2;
    (void)link_event(0x3e, event, sizeof(event), &change);
    dbt_l2cap_hooks_t hooks = {0, allocate, release, pdu};
    (void)dbt_l2cap_rx_init(&rx, hooks, 100, 100);
}
static dbt_result_t receive(uint16_t handle, uint8_t pb, const uint8_t *data, uint16_t size, uint32_t now) {
    dbt_hci_packet_t packet = {0};
    packet.type = DBT_H4_ACL; packet.handle = handle; packet.pb = pb;
    packet.payload = data; packet.size = size;
    return dbt_l2cap_receive(&rx, &acl, &packet, now);
}

EXPORT int test_l2cap_complete_pdu_zero_allocation(void) {
    fixture();
    uint8_t p[] = {3, 0, 4, 0, 10, 11, 12};
    CHECK(receive(1, 2, p, sizeof(p), 0) == DBT_OK);
    CHECK(allocations == 0 && callbacks == 1 && delivered_size == 3 && delivered_cid == 4);
    CHECK(delivered[0] == 10 && delivered[2] == 12 && delivered_handle == 1);
    p[0] = 2;
    CHECK(receive(1, 2, p, sizeof(p), 0) == DBT_INVALID && callbacks == 1);
    CHECK(receive(1, 0, p, sizeof(p), 0) == DBT_INVALID);
    CHECK(receive(9, 2, p, sizeof(p), 0) == DBT_INVALID);
    return 0;
}

EXPORT int test_l2cap_fragmented_and_interleaved(void) {
    fixture();
    uint8_t first[] = {6, 0, 4, 0, 1, 2};
    CHECK(receive(1, 2, first, sizeof(first), 0) == DBT_OK && allocation_size == 6);
    first[4] = 9;
    CHECK(receive(2, 2, first, sizeof(first), 0) == DBT_OK && allocations == 2);
    uint8_t tail[] = {3, 4, 5, 6};
    CHECK(receive(1, 1, tail, sizeof(tail), 1) == DBT_OK);
    CHECK(callbacks == 1 && delivered_handle == 1 && delivered[0] == 1 && frees == 1);
    CHECK(receive(2, 1, tail, sizeof(tail), 1) == DBT_OK);
    CHECK(callbacks == 2 && delivered[0] == 9 && frees == 2 && bad_frees == 0);
    CHECK(receive(2, 1, tail, sizeof(tail), 1) == DBT_INVALID);
    return 0;
}

EXPORT int test_l2cap_reassembly_limits_and_oom(void) {
    fixture();
    uint8_t first[] = {101, 0, 4, 0, 1};
    CHECK(receive(1, 2, first, sizeof(first), 0) == DBT_NO_SPACE && allocations == 0);
    first[0] = 6; allocation_fails = true;
    CHECK(receive(1, 2, first, sizeof(first), 0) == DBT_NO_SPACE && callbacks == 0);
    allocation_fails = false;
    CHECK(receive(1, 2, first, sizeof(first), 0) == DBT_OK);
    uint8_t oversized[8] = {0};
    CHECK(receive(1, 1, oversized, sizeof(oversized), 1) == DBT_INVALID);
    CHECK(frees == 1 && callbacks == 0 && rx.abandoned == 1);
    CHECK(receive(1, 2, first, sizeof(first), 2) == DBT_OK);
    uint8_t restart[] = {0, 0, 4, 0};
    CHECK(receive(1, 2, restart, sizeof(restart), 3) == DBT_OK);
    CHECK(frees == 2 && callbacks == 1 && delivered_size == 0 && rx.abandoned == 2);
    return 0;
}

EXPORT int test_l2cap_disconnect_and_deadline(void) {
    fixture();
    uint8_t first[] = {6, 0, 4, 0, 1};
    CHECK(receive(1, 2, first, sizeof(first), UINT32_MAX - 49u) == DBT_OK);
    CHECK(dbt_l2cap_expire(&rx, 49) == 0);
    CHECK(dbt_l2cap_expire(&rx, 50) == 1 && frees == 1);
    CHECK(receive(1, 2, first, sizeof(first), 60) == DBT_OK);
    uint8_t disconnect[] = {0, 1, 0, 0x13};
    dbt_link_change_t change;
    CHECK(link_event(5, disconnect, sizeof(disconnect), &change) == DBT_OK);
    dbt_l2cap_link_change(&rx, &change);
    dbt_l2cap_link_change(&rx, &change);
    CHECK(frees == 2 && bad_frees == 0);
    dbt_l2cap_rx_clear(&rx);
    CHECK(frees == 2 && bad_frees == 0);
    return 0;
}

EXPORT int test_l2cap_tx_backpressure_and_roundtrip(void) {
    fixture();
    CHECK(dbt_l2cap_tx_begin(&tx, dbt_acl_find(&acl, 1), 4, message, 60) == DBT_OK);
    transport_ready = false; output[0] = 0xaa;
    CHECK(dbt_l2cap_send_next(&tx, &acl, transport, output, sizeof(output)) == DBT_BUSY);
    CHECK(output[0] == 0xaa && tx.cursor == 0);
    transport_ready = true;
    for (unsigned fragment = 0; fragment < 3; ++fragment) {
        CHECK(dbt_l2cap_send_next(&tx, &acl, transport, output, sizeof(output)) == DBT_OK);
        if (fragment == 0) last_wire[2] |= 0x20; /* Controller RX first-fragment flag. */
        dbt_hci_packet_t packet;
        CHECK(dbt_hci_decode(last_wire, last_wire_size, &packet) == DBT_OK);
        CHECK(dbt_l2cap_receive(&rx, &acl, &packet, fragment) == DBT_OK);
        uint8_t completed[] = {1, 1, 0, 1, 0};
        dbt_link_change_t change;
        CHECK(link_event(0x13, completed, sizeof(completed), &change) == DBT_OK);
    }
    CHECK(tx.cursor == 64 && sent == 3 && callbacks == 1 && delivered_size == 60);
    for (unsigned i = 0; i < 60; ++i) CHECK(delivered[i] == message[i]);
    CHECK(dbt_l2cap_send_next(&tx, &acl, transport, output, sizeof(output)) == DBT_EMPTY);
    CHECK(dbt_l2cap_tx_commit(&tx) == DBT_INVALID && allocations == frees && bad_frees == 0);
    return 0;
}

EXPORT int test_l2cap_tx_credit_stall_and_stale_generation(void) {
    fixture();
    CHECK(dbt_l2cap_tx_begin(&tx, dbt_acl_find(&acl, 1), 4, message, 80) == DBT_OK);
    CHECK(dbt_l2cap_send_next(&tx, &acl, transport, output, sizeof(output)) == DBT_OK);
    CHECK(dbt_l2cap_send_next(&tx, &acl, transport, output, sizeof(output)) == DBT_OK);
    uint32_t cursor = tx.cursor;
    CHECK(dbt_l2cap_send_next(&tx, &acl, transport, output, sizeof(output)) == DBT_BUSY);
    CHECK(tx.cursor == cursor && sent == 2);
    uint8_t disconnect[] = {0, 1, 0, 0x13};
    dbt_link_change_t change;
    CHECK(link_event(5, disconnect, sizeof(disconnect), &change) == DBT_OK);
    uint8_t connection[19] = {1, 0, 1, 0, 1, 0};
    CHECK(link_event(0x3e, connection, sizeof(connection), &change) == DBT_OK);
    uint16_t size = 0;
    CHECK(dbt_l2cap_tx_peek(&tx, &acl, output, sizeof(output), &size) == DBT_INVALID);
    CHECK(tx.cursor == cursor && tx.pending == 0);
    return 0;
}
