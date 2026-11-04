#include "dbt_acl.h"
#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif
#define CHECK(expression) do { if (!(expression)) return __LINE__; } while (0)

static dbt_acl_t acl;
static bool ready, accepted;
static unsigned submissions;
static bool tx_ready(void *context) { (void)context; return ready; }
static bool tx_send(void *context, uint8_t *packet, uint16_t size) {
    (void)context; (void)packet; (void)size;
    if (!accepted) return false;
    ++submissions;
    return true;
}
static const dbt_transport_t transport = {0, tx_ready, tx_send};
static void fixture(bool shared) {
    dbt_controller_info_t info = {0};
    info.classic_acl_bytes = 1021;
    info.classic_acl_packets = 3;
    info.le_acl_bytes = shared ? 1021 : 27;
    info.le_acl_packets = shared ? 3 : 2;
    info.shared_acl_pool = shared;
    (void)dbt_acl_init(&acl, &info);
    ready = accepted = true;
    submissions = 0;
}
static dbt_result_t event(uint8_t code, const uint8_t *data, uint16_t size, dbt_link_change_t *change) {
    dbt_hci_packet_t packet = {0};
    packet.type = DBT_H4_EVENT;
    packet.event = code;
    packet.payload = data;
    packet.size = size;
    return dbt_acl_event(&acl, &packet, change);
}
static dbt_result_t connect_link(uint16_t handle, bool le, dbt_link_change_t *change) {
    uint8_t classic[11] = {0};
    uint8_t low_energy[19] = {0};
    if (le) {
        low_energy[0] = 1;
        low_energy[2] = (uint8_t)handle;
        low_energy[3] = (uint8_t)(handle >> 8);
        low_energy[4] = 1;
        low_energy[5] = 1;
        low_energy[6] = 0xaa;
        return event(0x3e, low_energy, 19, change);
    }
    classic[1] = (uint8_t)handle;
    classic[2] = (uint8_t)(handle >> 8);
    classic[3] = 0xbb;
    classic[9] = 1;
    return event(3, classic, 11, change);
}
static dbt_result_t submit(uint16_t handle, uint8_t pb) {
    uint8_t packet[] = {2, (uint8_t)handle, (uint8_t)((handle >> 8) | (pb << 4)), 1, 0, 0xaa};
    return dbt_acl_submit(&acl, transport, packet, sizeof(packet));
}

EXPORT int test_acl_shared_and_separate_credits(void) {
    dbt_link_change_t change;
    fixture(true);
    CHECK(connect_link(1, false, &change) == DBT_OK);
    CHECK(change.kind == DBT_LINK_ADDED && dbt_acl_find(&acl, 1)->peer[0] == 0xbb);
    CHECK(connect_link(2, true, &change) == DBT_OK);
    CHECK(dbt_acl_find(&acl, 2)->peer_address_type == 1);
    CHECK(submit(1, 0) == DBT_OK && submit(2, 0) == DBT_OK && submit(1, 1) == DBT_OK);
    CHECK(submit(2, 1) == DBT_BUSY && acl.pools[0].available == 0 && submissions == 3);
    uint8_t completed[] = {2, 1, 0, 2, 0, 2, 0, 1, 0};
    CHECK(event(0x13, completed, sizeof(completed), &change) == DBT_OK);
    CHECK(acl.pools[0].available == 3 && dbt_acl_find(&acl, 1)->outstanding == 0);
    fixture(false);
    CHECK(connect_link(1, false, &change) == DBT_OK && connect_link(2, true, &change) == DBT_OK);
    CHECK(submit(1, 0) == DBT_OK && submit(1, 1) == DBT_OK && submit(1, 1) == DBT_OK);
    CHECK(submit(1, 1) == DBT_BUSY);
    CHECK(submit(2, 0) == DBT_OK && acl.pools[1].available == 1);
    return 0;
}

EXPORT int test_acl_disconnect_and_handle_reuse(void) {
    fixture(true);
    dbt_link_change_t change;
    CHECK(connect_link(0x123, false, &change) == DBT_OK);
    uint32_t generation = change.generation;
    CHECK(submit(0x123, 0) == DBT_OK && submit(0x123, 1) == DBT_OK);
    uint8_t disconnect[] = {0, 0x23, 1, 0x13};
    CHECK(event(5, disconnect, sizeof(disconnect), &change) == DBT_OK);
    CHECK(change.kind == DBT_LINK_REMOVED && change.generation == generation);
    CHECK(acl.pools[0].available == 3 && dbt_acl_find(&acl, 0x123) == 0);
    CHECK(submit(0x123, 0) == DBT_INVALID);
    CHECK(connect_link(0x123, true, &change) == DBT_OK && change.generation != generation);
    CHECK(connect_link(0x123, false, &change) == DBT_INVALID);
    return 0;
}

EXPORT int test_acl_completed_event_transaction(void) {
    fixture(true);
    dbt_link_change_t change;
    CHECK(connect_link(1, false, &change) == DBT_OK);
    CHECK(submit(1, 0) == DBT_OK && submit(1, 1) == DBT_OK);
    uint8_t overclaim[] = {2, 1, 0, 1, 0, 1, 0, 2, 0};
    CHECK(event(0x13, overclaim, sizeof(overclaim), &change) == DBT_INVALID);
    CHECK(acl.pools[0].available == 1 && dbt_acl_find(&acl, 1)->outstanding == 2);
    overclaim[7] = 1;
    CHECK(event(0x13, overclaim, sizeof(overclaim), &change) == DBT_OK);
    CHECK(acl.pools[0].available == 3 && dbt_acl_find(&acl, 1)->outstanding == 0);
    CHECK(event(0x13, overclaim, sizeof(overclaim) - 1, &change) == DBT_INVALID);
    uint8_t unknown[] = {1, 9, 0, 2, 0};
    CHECK(event(0x13, unknown, sizeof(unknown), &change) == DBT_OK);
    CHECK(acl.unknown_completions == 2 && acl.pools[0].available == 3);
    return 0;
}

EXPORT int test_acl_admission_and_encryption(void) {
    fixture(false);
    dbt_link_change_t change;
    CHECK(connect_link(1, true, &change) == DBT_OK);
    CHECK(submit(1, 2) == DBT_INVALID); /* LE host->controller forbids flushable start. */
    ready = false;
    CHECK(submit(1, 0) == DBT_BUSY && acl.pools[1].available == 2);
    ready = true; accepted = false;
    CHECK(submit(1, 0) == DBT_BUSY && acl.pools[1].available == 2);
    accepted = true;
    uint8_t oversized[33] = {2, 1, 0, 28, 0};
    CHECK(dbt_acl_submit(&acl, transport, oversized, sizeof(oversized)) == DBT_NO_SPACE);
    uint8_t encryption[] = {0, 1, 0, 1};
    CHECK(event(8, encryption, sizeof(encryption), &change) == DBT_OK);
    CHECK(dbt_acl_find(&acl, 1)->encrypted);
    encryption[3] = 0;
    CHECK(event(8, encryption, sizeof(encryption), &change) == DBT_OK);
    CHECK(!dbt_acl_find(&acl, 1)->encrypted);
    return 0;
}

EXPORT int test_acl_link_limits_and_enhanced_le(void) {
    fixture(false);
    dbt_link_change_t change;
    uint8_t enhanced[31] = {0x0a, 0, 0x77, 0, 0, 3, 0xcc};
    CHECK(event(0x3e, enhanced, sizeof(enhanced), &change) == DBT_OK);
    CHECK(dbt_acl_find(&acl, 0x77)->peer_address_type == 3 && change.generation == 1);
    CHECK(connect_link(1, false, &change) == DBT_OK);
    CHECK(connect_link(2, true, &change) == DBT_OK);
    CHECK(connect_link(3, false, &change) == DBT_OK);
    CHECK(connect_link(4, false, &change) == DBT_NO_SPACE);
    CHECK(connect_link(0xf00, false, &change) == DBT_INVALID);
    return 0;
}
