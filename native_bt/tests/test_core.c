#include "dbt_host.h"

#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif
#define CHECK(expression) do { if (!(expression)) return __LINE__; } while (0)

/* Static fixtures also make test stack usage deterministic. */
static dbt_host_t host;
static dbt_queue_t queue;
static uint8_t input[DBT_MAX_H4_BYTES];
static uint8_t output[DBT_MAX_H4_BYTES];
static uint8_t last_tx[259];
static bool transport_ready;
static bool transport_accepts;
static unsigned sends;
static uint16_t tx_size;
static unsigned observed;
static uint8_t observation[32];

static bool can_send(void *context) { (void)context; return transport_ready; }
static bool send_packet(void *context, uint8_t *data, uint16_t size) {
    (void)context;
    if (!transport_accepts) return false;
    tx_size = size;
    for (uint16_t i = 0; i < size; ++i) last_tx[i] = data[i];
    ++sends;
    return true;
}
static void observe(void *context, const dbt_hci_packet_t *packet) {
    (void)context;
    if (observed < sizeof(observation)) {
        observation[observed] = packet->type == DBT_H4_EVENT ? packet->event : packet->type;
    }
    ++observed;
}
static void fixture(uint32_t now) {
    sends = 0;
    observed = 0;
    tx_size = 0;
    transport_ready = true;
    transport_accepts = true;
    dbt_transport_t transport = { 0, can_send, send_packet };
    (void)dbt_host_init(&host, transport, observe, 0, 100);
    (void)dbt_host_start(&host, now);
}
static dbt_result_t complete(uint16_t opcode, uint8_t credits, const uint8_t *data, uint8_t size) {
    uint8_t packet[80];
    packet[0] = DBT_H4_EVENT;
    packet[1] = DBT_EVT_COMMAND_COMPLETE;
    packet[2] = (uint8_t)(3u + size);
    packet[3] = credits;
    packet[4] = (uint8_t)opcode;
    packet[5] = (uint8_t)(opcode >> 8);
    for (uint16_t i = 0; i < size; ++i) packet[6u + i] = data[i];
    return dbt_host_receive(&host, packet, (uint16_t)(6u + size));
}
static dbt_result_t success(bool shared_pool) {
    uint8_t result[65];
    for (unsigned i = 0; i < sizeof(result); ++i) result[i] = 0;
    uint8_t size = 1;
    switch (host.pending_opcode) {
    case 0x0c03: size = 1; break;
    case 0x1001:
        size = 9; result[1] = 8; result[2] = 0x34; result[3] = 0x12;
        result[4] = 8; result[5] = 0xe5; result[6] = 2;
        result[7] = 0x78; result[8] = 0x56; break;
    case 0x1002:
        size = 65;
        for (unsigned i = 1; i < 65; ++i) result[i] = (uint8_t)i;
        break;
    case 0x1003: size = 9; result[5] = 0x40; break;
    case 0x1009: size = 7; result[1] = 1; result[6] = 6; break;
    case 0x1005:
        size = 8; result[1] = 0xfd; result[2] = 3; result[4] = 6; break;
    case 0x2003: size = 9; result[1] = 0x1f; break;
    case 0x2002:
        size = 4;
        if (!shared_pool) { result[1] = 0xfb; result[3] = 3; }
        break;
    default: return DBT_INVALID;
    }
    return complete(host.pending_opcode, 1, result, size);
}

EXPORT int test_hci_framing(void) {
    dbt_hci_packet_t packet;
    uint8_t event[] = { 4, 0xff, 2, 0x12, 0x34 };
    CHECK(dbt_hci_decode(event, sizeof(event), &packet) == DBT_OK);
    CHECK(packet.event == 0xff && packet.size == 2 && packet.payload[1] == 0x34);
    CHECK(dbt_hci_decode(event, sizeof(event) - 1, &packet) == DBT_INVALID);
    uint8_t acl[] = { 2, 0x23, 0x21, 2, 0, 0xaa, 0xbb };
    CHECK(dbt_hci_decode(acl, sizeof(acl), &packet) == DBT_OK);
    CHECK(packet.handle == 0x123 && packet.pb == 2 && packet.bc == 0 && packet.size == 2);
    acl[4] = 0xff;
    CHECK(dbt_hci_decode(acl, sizeof(acl), &packet) == DBT_INVALID);
    CHECK(dbt_hci_decode(0, 0, &packet) == DBT_INVALID);
    uint8_t sco[] = { 3, 0, 0, 0 };
    CHECK(dbt_hci_decode(sco, sizeof(sco), &packet) == DBT_OK && packet.type==DBT_H4_SCO && packet.size==0);
    return 0;
}

EXPORT int test_command_bounds(void) {
    uint8_t command[6] = { 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa };
    uint8_t parameters[] = { 0x12, 0x34 };
    uint16_t size = 0;
    CHECK(dbt_hci_command(0x1234, parameters, 2, command, 5, &size) == DBT_NO_SPACE);
    CHECK(command[0] == 0xaa && command[5] == 0xaa);
    CHECK(dbt_hci_command(0x1234, parameters, 2, command, 6, &size) == DBT_OK);
    CHECK(size == 6 && command[0] == 1 && command[1] == 0x34 && command[2] == 0x12);
    CHECK(command[3] == 2 && command[4] == 0x12 && command[5] == 0x34);
    CHECK(dbt_hci_command(1, 0, 1, command, 6, &size) == DBT_INVALID);
    return 0;
}

EXPORT int test_queue_wrap_and_copy(void) {
    dbt_queue_init(&queue);
    /* Force both the byte-ring wrap and 32-bit sequence-counter wrap. */
    atomic_store(&queue.written, UINT32_MAX - 8u);
    atomic_store(&queue.read, UINT32_MAX - 8u);
    for (unsigned i = 0; i < sizeof(input); ++i) input[i] = (uint8_t)i;
    CHECK(dbt_queue_push(&queue, input, sizeof(input), false) == DBT_OK);
    input[12] = 0;
    uint16_t size = 0;
    CHECK(dbt_queue_pop(&queue, output, 5, &size) == DBT_NO_SPACE);
    CHECK(size == sizeof(input) && dbt_queue_used(&queue) == sizeof(input) + 2u);
    CHECK(dbt_queue_pop(&queue, output, sizeof(output), &size) == DBT_OK);
    CHECK(output[12] == 12 && output[1025] == 1 && size == sizeof(input));
    CHECK(dbt_queue_used(&queue) == 0);
    CHECK(dbt_queue_pop(&queue, output, sizeof(output), &size) == DBT_EMPTY);
    CHECK(dbt_queue_push(&queue, input, 0, false) == DBT_INVALID);
    return 0;
}

EXPORT int test_control_reserve_preserves_order(void) {
    dbt_queue_init(&queue);
    for (unsigned i = 0; i < 3; ++i) {
        input[0] = (uint8_t)i;
        CHECK(dbt_queue_push(&queue, input, 1026, true) == DBT_OK);
    }
    CHECK(dbt_queue_push(&queue, input, 1026, true) == DBT_NO_SPACE);
    input[0] = 9;
    CHECK(dbt_queue_push(&queue, input, 258, false) == DBT_OK);
    CHECK(dbt_queue_push(&queue, input, 258, false) == DBT_OK);
    for (unsigned i = 0; i < 5; ++i) {
        uint16_t size = 0;
        CHECK(dbt_queue_pop(&queue, output, sizeof(output), &size) == DBT_OK);
        CHECK(output[0] == (i < 3 ? i : 9));
        CHECK(size == (i < 3 ? 1026 : 258));
    }
    CHECK(atomic_load(&queue.rejected) == 1);
    return 0;
}

EXPORT int test_bootstrap_separate_pools(void) {
    fixture(0);
    CHECK(dbt_host_start(&host, 0) == DBT_INVALID);
    dbt_host_poll(&host, 0, 8);
    CHECK(sends == 1 && tx_size == 4 && last_tx[1] == 3 && last_tx[2] == 12);
    for (uint32_t i = 1; i <= 8; ++i) {
        CHECK(success(false) == DBT_OK);
        dbt_host_poll(&host, i, 8);
    }
    CHECK(host.state == DBT_HOST_IDENTIFIED && host.commands_submitted == 8);
    CHECK(host.controller.hci_revision == 0x1234 && host.controller.manufacturer == 0x02e5);
    CHECK(host.controller.lmp_subversion == 0x5678);
    CHECK(host.controller.supported_commands[63] == 64);
    CHECK(host.controller.classic_acl_bytes == 1021 && host.controller.classic_acl_packets == 6);
    CHECK(!host.controller.shared_acl_pool && host.controller.le_acl_bytes == 251);
    CHECK(host.controller.le_acl_packets == 3);
    CHECK(dbt_host_wait_ms(&host, 10) == UINT32_MAX);
    return 0;
}

EXPORT int test_bootstrap_shared_pool(void) {
    fixture(0);
    dbt_host_poll(&host, 0, 8);
    for (uint32_t i = 1; i <= 8; ++i) {
        CHECK(success(true) == DBT_OK);
        dbt_host_poll(&host, i, 8);
    }
    CHECK(host.state == DBT_HOST_IDENTIFIED && host.controller.shared_acl_pool);
    CHECK(host.controller.le_acl_packets == 6 && host.controller.le_acl_bytes == 1021);
    /* This is one shared capacity, not two independent six-packet credit pools. */
    return 0;
}

EXPORT int test_transport_and_credit_gates(void) {
    fixture(0);
    transport_ready = false;
    dbt_host_poll(&host, 0, 8);
    CHECK(sends == 0 && host.command_credits == 1);
    transport_ready = true;
    transport_accepts = false;
    dbt_host_poll(&host, 1, 8);
    CHECK(sends == 0 && host.command_credits == 1 && host.pending_opcode == 0);
    transport_accepts = true;
    dbt_host_poll(&host, 2, 8);
    CHECK(sends == 1 && host.command_credits == 0);
    uint8_t ok[] = { 0 };
    CHECK(complete(0x0c03, 0, ok, 1) == DBT_OK);
    dbt_host_poll(&host, 3, 8);
    CHECK(sends == 1 && host.probe_index == 1 && host.pending_opcode == 0);
    /* Unrelated opcode 0 may replenish credits, but must not finish a probe. */
    CHECK(complete(0, 1, ok, 0) == DBT_OK);
    dbt_host_poll(&host, 4, 8);
    CHECK(sends == 2 && host.pending_opcode == 0x1001 && host.probe_index == 1);
    CHECK(host.unexpected_completions == 1);
    return 0;
}

EXPORT int test_status_is_not_completion(void) {
    fixture(0);
    dbt_host_poll(&host, 0, 8);
    uint8_t status[] = { 4, 15, 4, 0, 1, 3, 12 };
    CHECK(dbt_host_receive(&host, status, sizeof(status)) == DBT_OK);
    dbt_host_poll(&host, 1, 8);
    CHECK(host.pending_opcode == 0x0c03 && host.probe_index == 0 && sends == 1);
    dbt_host_poll(&host, 100, 8);
    CHECK(host.state == DBT_HOST_FAILED && host.fault == DBT_FAULT_TIMEOUT);
    return 0;
}

EXPORT int test_deadlines_wrap_and_credit_stall(void) {
    fixture(UINT32_MAX - 49u);
    transport_ready = false;
    dbt_host_poll(&host, 25, 8);
    CHECK(host.state == DBT_HOST_PROBING && dbt_host_wait_ms(&host, 25) == 25);
    dbt_host_poll(&host, 50, 8);
    CHECK(host.state == DBT_HOST_FAILED && host.fault == DBT_FAULT_TIMEOUT && sends == 0);
    fixture(0);
    dbt_host_poll(&host, 0, 8);
    uint8_t ok[] = { 0 };
    CHECK(complete(0x0c03, 0, ok, 1) == DBT_OK);
    dbt_host_poll(&host, 1, 8);
    dbt_host_poll(&host, 101, 8);
    CHECK(host.state == DBT_HOST_FAILED && host.fault == DBT_FAULT_TIMEOUT && sends == 1);
    return 0;
}

EXPORT int test_rejection_and_short_return(void) {
    fixture(0);
    dbt_host_poll(&host, 0, 8);
    uint8_t failure[] = { 0x0c };
    CHECK(complete(0x0c03, 1, failure, 1) == DBT_OK);
    dbt_host_poll(&host, 1, 8);
    CHECK(host.fault == DBT_FAULT_COMMAND_REJECTED && host.controller_status == 0x0c);
    fixture(0);
    dbt_host_poll(&host, 0, 8);
    CHECK(success(false) == DBT_OK);
    dbt_host_poll(&host, 1, 8);
    uint8_t short_result[] = { 0 };
    CHECK(complete(0x1001, 1, short_result, 1) == DBT_OK);
    dbt_host_poll(&host, 2, 8);
    CHECK(host.fault == DBT_FAULT_BAD_RESPONSE && host.probe_index == 1);
    fixture(0);
    dbt_host_poll(&host, 0, 8);
    uint8_t status[] = { 4, 15, 4, 0x0c, 1, 3, 12 };
    CHECK(dbt_host_receive(&host, status, sizeof(status)) == DBT_OK);
    dbt_host_poll(&host, 1, 8);
    CHECK(host.fault == DBT_FAULT_COMMAND_REJECTED);
    return 0;
}

EXPORT int test_requires_dual_mode_features(void) {
    fixture(0);
    dbt_host_poll(&host, 0, 8);
    for (uint32_t i = 1; i <= 3; ++i) {
        CHECK(success(false) == DBT_OK);
        dbt_host_poll(&host, i, 8);
    }
    CHECK(host.pending_opcode == 0x1003);
    uint8_t features[9] = { 0 };
    CHECK(complete(0x1003, 1, features, 9) == DBT_OK);
    dbt_host_poll(&host, 4, 8);
    CHECK(host.fault == DBT_FAULT_NOT_DUAL_MODE && sends == 4);
    return 0;
}

EXPORT int test_rx_overrun_fails_closed(void) {
    fixture(0);
    input[0] = 2; input[1] = 1; input[2] = 0x20; input[3] = 0xfd; input[4] = 3;
    for (unsigned i = 0; i < 3; ++i) CHECK(dbt_host_receive(&host, input, 1026) == DBT_OK);
    CHECK(dbt_host_receive(&host, input, 1026) == DBT_NO_SPACE);
    CHECK(dbt_host_wait_ms(&host, 0) == 0);
    dbt_host_poll(&host, 0, 8);
    CHECK(host.fault == DBT_FAULT_RX_OVERRUN && sends == 0 && observed == 0);
    return 0;
}

EXPORT int test_bounded_dispatch_and_order(void) {
    fixture(0);
    uint8_t acl[] = { 2, 1, 0x20, 0, 0 };
    uint8_t event[] = { 4, 0xff, 0 };
    CHECK(dbt_host_receive(&host, acl, sizeof(acl)) == DBT_OK);
    CHECK(dbt_host_receive(&host, event, sizeof(event)) == DBT_OK);
    CHECK(dbt_host_poll(&host, 0, 1) == 1 && observed == 1);
    CHECK(dbt_host_wait_ms(&host, 0) == 0);
    CHECK(dbt_host_poll(&host, 0, 1) == 1 && observed == 2);
    CHECK(observation[0] == DBT_H4_ACL && observation[1] == 0xff);
    CHECK(sends == 1);
    return 0;
}

EXPORT int test_bad_ingress_and_hardware_error(void) {
    fixture(0);
    uint8_t event[] = { 4, 0xff, 1 };
    CHECK(dbt_host_receive(&host, event, sizeof(event)) == DBT_INVALID);
    dbt_host_poll(&host, 0, 8);
    CHECK(host.fault == DBT_FAULT_BAD_PACKET && host.fault_stage == 1);
    fixture(0);
    uint8_t error[] = { 4, 16, 1, 0x42 };
    CHECK(dbt_host_receive(&host, error, sizeof(error)) == DBT_OK);
    dbt_host_poll(&host, 0, 8);
    CHECK(host.fault == DBT_FAULT_CONTROLLER && host.controller_status == 0x42);
    CHECK(host.fault_stage == 4 && host.fault_packet_size == sizeof(error) && host.fault_event == 16);
    fixture(0);
    uint8_t short_cc[] = { 4, 14, 2, 1, 3 };
    CHECK(dbt_host_receive(&host, short_cc, sizeof(short_cc)) == DBT_OK);
    dbt_host_poll(&host, 0, 8);
    CHECK(host.fault == DBT_FAULT_BAD_PACKET);
    CHECK(host.fault_stage == 4 && host.fault_packet_type == DBT_H4_EVENT && host.fault_event == 14);
    return 0;
}

EXPORT int test_decode_generated_lengths(void) {
    uint32_t random = 0x426c7565;
    dbt_hci_packet_t packet;
    for (unsigned trial = 0; trial < 12000; ++trial) {
        random = random * 1664525u + 1013904223u;
        uint16_t size = (uint16_t)(random % (DBT_MAX_H4_BYTES + 1u));
        for (unsigned i = 0; i < size; ++i) {
            random = random * 1664525u + 1013904223u;
            input[i] = (uint8_t)(random >> 24);
        }
        dbt_result_t result = dbt_hci_decode(input, size, &packet);
        if (result == DBT_OK) {
            CHECK(packet.type == 2 || packet.type == 3 || packet.type == 4);
            CHECK(packet.payload >= input && packet.payload + packet.size == input + size);
        }
    }
    return 0;
}

EXPORT uint32_t test_host_bytes(void) { return sizeof(dbt_host_t); }
EXPORT uint32_t test_queue_bytes(void) { return sizeof(dbt_queue_t); }

/* Called on two real threads by the Python harness. Each side owns its scratch. */
static dbt_queue_t stress_queue;
static uint8_t stress_tx[DBT_MAX_H4_BYTES];
static uint8_t stress_rx[DBT_MAX_H4_BYTES];
static uint32_t stress_expected;
EXPORT void stress_begin(void) {
    dbt_queue_init(&stress_queue);
    stress_expected = 0;
}
EXPORT int stress_produce(uint32_t sequence) {
    uint16_t size = (uint16_t)(8u + sequence % 1019u);
    for (uint16_t i = 0; i < size; ++i) stress_tx[i] = (uint8_t)(sequence + i);
    for (unsigned i = 0; i < 4; ++i) stress_tx[i] = (uint8_t)(sequence >> (8u * i));
    return (int)dbt_queue_push(&stress_queue, stress_tx, size, true);
}
EXPORT int stress_consume(void) {
    uint16_t size = 0;
    dbt_result_t result = dbt_queue_pop(&stress_queue, stress_rx, sizeof(stress_rx), &size);
    if (result == DBT_EMPTY) return 1;
    if (result != DBT_OK) return -1;
    uint32_t sequence = 0;
    for (unsigned i = 0; i < 4; ++i) sequence |= (uint32_t)stress_rx[i] << (8u * i);
    if (sequence != stress_expected || size != 8u + sequence % 1019u) return -2;
    for (uint16_t i = 4; i < size; ++i) {
        if (stress_rx[i] != (uint8_t)(sequence + i)) return -3;
    }
    ++stress_expected;
    return 0;
}
