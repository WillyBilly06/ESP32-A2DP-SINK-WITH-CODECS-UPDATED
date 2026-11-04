#include "dbt_rx_flow.h"
#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif
#define CHECK(x)                                                                                   \
    do {                                                                                           \
        if (!(x))                                                                                  \
            return __LINE__;                                                                       \
    } while (0)
static dbt_host_t host;
static dbt_acl_t acl;
static dbt_rx_flow_t flow;
static bool ready, accept;
static uint8_t sent[259], *borrowed;
static uint16_t sent_size;
static unsigned sends;
static uint32_t now;
static void zero(void *data, unsigned size) {
    uint8_t *p = data;
    for (unsigned i = 0; i < size; ++i)
        p[i] = 0;
}
static bool can_send(void *context) {
    (void)context;
    return ready;
}
static bool send(void *context, uint8_t *bytes, uint16_t size) {
    (void)context;
    if (!accept)
        return false;
    for (unsigned i = 0; i < size; ++i)
        sent[i] = bytes[i];
    sent_size = size;
    borrowed = bytes;
    ready = false;
    ++sends;
    return true;
}
static int release_transport(void) {
    if (!ready)
        for (unsigned i = 0; i < sent_size; ++i)
            CHECK(sent[i] == borrowed[i]);
    ready = true;
    return 0;
}
static void fixture(void) {
    zero(&host, sizeof(host));
    zero(&acl, sizeof(acl));
    zero(&flow, sizeof(flow));
    ready = accept = true;
    sends = sent_size = 0;
    borrowed = 0;
    now = 100;
    dbt_transport_t transport = {0, can_send, send};
    (void)dbt_host_init(&host, transport, 0, 0, 100);
    host.state = DBT_HOST_IDENTIFIED;
    host.command_credits = 1;
    host.controller.supported_commands[10] = 0xe0;
    dbt_controller_info_t controller = {0};
    controller.classic_acl_bytes = 1021;
    controller.classic_acl_packets = 9;
    controller.le_acl_bytes = 251;
    controller.le_acl_packets = 10;
    (void)dbt_acl_init(&acl, &controller);
    (void)dbt_rx_flow_init(&flow, &host, &acl);
}
static void poll(void) {
    dbt_rx_flow_poll(&flow, ++now);
    dbt_host_poll(&host, now, 8);
    dbt_rx_flow_poll(&flow, now);
}
static int complete(uint8_t status) {
    int result = release_transport();
    if (result)
        return result;
    uint16_t opcode = host.pending_opcode;
    CHECK(opcode);
    uint8_t wire[] = {4, 0x0e, 4, 1, (uint8_t)opcode, (uint8_t)(opcode >> 8), status};
    CHECK(dbt_host_receive(&host, wire, sizeof(wire)) == DBT_OK);
    dbt_host_poll(&host, ++now, 8);
    return 0;
}
static int active(void) {
    CHECK(dbt_rx_flow_start(&flow, 1021, 3) == DBT_OK);
    poll();
    CHECK(dbt_read_le16(sent + 1) == 0x0c33 && sent_size == 11 && sent[4] == 0xfd && sent[5] == 3 &&
          sent[6] == 255 && sent[7] == 3 && sent[8] == 0 && sent[9] == 1 && sent[10] == 0);
    int r = complete(0);
    if (r)
        return r;
    CHECK(flow.state == DBT_RX_FLOW_ENABLE);
    poll();
    CHECK(dbt_read_le16(sent + 1) == 0x0c31 && sent_size == 5 && sent[4] == 1);
    r = complete(0);
    if (r)
        return r;
    CHECK(flow.state == DBT_RX_FLOW_ACTIVE && !flow.command_pending);
    return 0;
}
static int add_link(uint16_t handle, bool le) {
    uint8_t data[19] = {0};
    dbt_hci_packet_t packet = {0};
    dbt_link_change_t change;
    packet.type = 4;
    packet.payload = data;
    if (le) {
        packet.event = 0x3e;
        packet.size = 19;
        data[0] = 1;
        data[2] = (uint8_t)handle;
        data[3] = (uint8_t)(handle >> 8);
        data[4] = 1;
    } else {
        packet.event = 3;
        packet.size = 11;
        data[1] = (uint8_t)handle;
        data[2] = (uint8_t)(handle >> 8);
        data[9] = 1;
    }
    CHECK(dbt_acl_event(&acl, &packet, &change) == DBT_OK);
    return 0;
}
EXPORT int test_rx_flow_setup_limits_and_capabilities(void) {
    fixture();
    host.controller.supported_commands[10] = 0x60;
    CHECK(dbt_rx_flow_start(&flow, 1021, 3) == DBT_UNSUPPORTED &&
          flow.state == DBT_RX_FLOW_DISABLED);
    host.controller.supported_commands[10] = 0xe0;
    CHECK(dbt_rx_flow_start(&flow, DBT_MAX_H4_BYTES - 4u, 3) == DBT_INVALID);
    CHECK(dbt_rx_flow_start(&flow, 1021, 4) == DBT_NO_SPACE);
    CHECK(dbt_rx_flow_start(&flow, 1021, 0) == DBT_INVALID);
    CHECK(dbt_host_command(&host, 0x0c35, 0, 0, DBT_COMMAND_COMPLETE, 0, 0, now) ==
          DBT_UNSUPPORTED);
    int r = active();
    if (r)
        return r;
    CHECK(dbt_rx_flow_start(&flow, 1021, 3) == DBT_BUSY);
    CHECK(dbt_rx_flow_wait_ms(&flow) == UINT32_MAX && sends == 2);
    fixture();
    CHECK(dbt_rx_flow_start(&flow,1024,3)==DBT_OK);poll();
    CHECK(sent[4]==0 && sent[5]==4 && sent[6]==255 && sent[7]==3 && sent[9]==1);
    r=complete(0);if(r)return r;poll();r=complete(0);if(r)return r;
    CHECK(flow.state==DBT_RX_FLOW_ACTIVE);
    fixture();
    r = add_link(1, false);
    if (r)
        return r;
    CHECK(dbt_rx_flow_start(&flow, 1021, 3) == DBT_BUSY);
    fixture();
    CHECK(dbt_rx_flow_start(&flow, 1021, 3) == DBT_OK);
    poll();
    r = complete(0x11);
    if (r)
        return r;
    CHECK(flow.state == DBT_RX_FLOW_FAILED && host.state == DBT_HOST_FAILED &&
          host.controller_status == 0x11);
    fixture();
    CHECK(dbt_rx_flow_start(&flow, 1021, 3) == DBT_OK);
    ready = false;
    poll();
    dbt_host_poll(&host, now + 101, 8);
    CHECK(flow.state == DBT_RX_FLOW_FAILED && host.fault == DBT_FAULT_TIMEOUT);
    return 0;
}
EXPORT int test_rx_flow_acks_ignore_credits_and_retain_transport_bytes(void) {
    fixture();
    int r = active();
    if (r)
        return r;
    r = add_link(1, false);
    if (r)
        return r;
    r = add_link(2, true);
    if (r)
        return r;
    CHECK(dbt_host_command(&host, 0xfc01, 0, 0, DBT_COMMAND_COMPLETE, 0, 0, now) == DBT_OK);
    dbt_host_poll(&host, ++now, 8);
    CHECK(host.pending_opcode == 0xfc01 && host.command_credits == 0);
    r = release_transport();
    if (r)
        return r;
    acl.pools[0].available = acl.pools[1].available = 0;
    CHECK(dbt_rx_flow_completed(&flow, dbt_acl_find(&acl, 1)) == DBT_OK);
    CHECK(dbt_rx_flow_completed(&flow, dbt_acl_find(&acl, 1)) == DBT_OK);
    CHECK(dbt_rx_flow_completed(&flow, dbt_acl_find(&acl, 2)) == DBT_OK);
    CHECK(dbt_rx_flow_wait_ms(&flow) == 0);
    dbt_rx_flow_poll(&flow, ++now);
    const uint8_t expected[] = {1, 0x35, 0x0c, 9, 2, 1, 0, 2, 0, 2, 0, 1, 0};
    CHECK(sent_size == sizeof(expected));
    for (unsigned i = 0; i < sizeof(expected); ++i)
        CHECK(sent[i] == expected[i]);
    CHECK(flow.acknowledged == 3 && flow.commands_submitted == 1 && host.pending_opcode == 0xfc01 &&
          host.command_credits == 0);
    CHECK(dbt_rx_flow_completed(&flow, dbt_acl_find(&acl, 1)) == DBT_OK);
    dbt_rx_flow_poll(&flow, ++now);
    CHECK(flow.commands_submitted == 1 && dbt_rx_flow_wait_ms(&flow) == UINT32_MAX);
    dbt_rx_flow_abort(&flow);
    r = release_transport();
    if (r)
        return r;
    CHECK(flow.state == DBT_RX_FLOW_FAILED && flow.acknowledged == 3);
    dbt_host_abort(&host, DBT_FAULT_CONTROLLER);
    fixture();
    r = active();
    if (r)
        return r;
    r = add_link(1, false);
    if (r)
        return r;
    CHECK(dbt_rx_flow_completed(&flow, dbt_acl_find(&acl, 1)) == DBT_OK);
    accept = false;
    dbt_rx_flow_poll(&flow, ++now);
    CHECK(!flow.acknowledged && flow.pending[0].packets == 1);
    accept = true;
    dbt_rx_flow_poll(&flow, ++now);
    CHECK(flow.acknowledged == 1);
    r = release_transport();
    if (r)
        return r;
    return 0;
}
EXPORT int test_rx_flow_disconnect_generations_and_error_response(void) {
    fixture();
    int r = active();
    if (r)
        return r;
    r = add_link(1, false);
    if (r)
        return r;
    dbt_link_t old = *dbt_acl_find(&acl, 1);
    CHECK(dbt_rx_flow_completed(&flow, &old) == DBT_OK);
    uint8_t data[] = {0, 1, 0, 0x13};
    dbt_hci_packet_t event = {0};
    dbt_link_change_t change;
    event.type = 4;
    event.event = 5;
    event.payload = data;
    event.size = 4;
    CHECK(dbt_acl_event(&acl, &event, &change) == DBT_OK);
    dbt_rx_flow_link_change(&flow, &change);
    unsigned previous = sends;
    dbt_rx_flow_poll(&flow, ++now);
    CHECK(sends == previous && flow.retired_on_disconnect == 1);
    r = add_link(1, false);
    if (r)
        return r;
    CHECK(dbt_rx_flow_completed(&flow, &old) == DBT_INVALID);
    CHECK(dbt_rx_flow_completed(&flow, dbt_acl_find(&acl, 1)) == DBT_OK);
    dbt_rx_flow_poll(&flow, ++now);
    CHECK(sent_size == 9 && sent[7] == 1 && flow.acknowledged == 1);
    uint8_t rejection[] = {1, 0x35, 0x0c, 0x12};
    event.event = 0x0e;
    event.payload = rejection;
    event.size = 4;
    dbt_rx_flow_event(&flow, &event);
    CHECK(flow.state == DBT_RX_FLOW_FAILED && host.fault == DBT_FAULT_CONTROLLER &&
          host.controller_status == 0x12);
    r = release_transport();
    if (r)
        return r;
    return 0;
}
