#include "dbt_classic_security.h"
#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif
#define CHECK(expression)                                                                          \
    do {                                                                                           \
        if (!(expression))                                                                         \
            return __LINE__;                                                                       \
    } while (0)
static dbt_classic_security_t security;
static dbt_host_t host;
static dbt_acl_t acl;
static uint8_t address[6] = {1, 2, 3, 4, 5, 6};
static uint8_t sent[259];
static uint16_t sent_size;
static bool available, accept_send, allow_pair, allow_store, has_cached, fail_allocate;
static dbt_classic_key_t cached;
static uint32_t now, input_token, input_value;
static dbt_classic_input_t input_kind;
static unsigned sends, loads, input_calls, display_calls, ready_events, stored_events,
    removed_events, failed_events, lost_events;
static dbt_classic_security_reason_t reason;
static dbt_classic_security_info_t info;
typedef struct {
    uint32_t token;
    dbt_classic_key_t key;
    bool deleting;
} job_t;
static job_t jobs[16];
static unsigned job_count;
static _Alignas(16) uint8_t extended[80];
static unsigned allocation_size, allocations, releases;
static bool allocation_used, cleared_allocation;
static void wipe(void *data, unsigned count) {
    uint8_t *out = data;
    for (unsigned i = 0; i < count; ++i)
        out[i] = 0;
}
static void copy(void *out, const void *in, unsigned count) {
    uint8_t *a = out;
    const uint8_t *b = in;
    for (unsigned i = 0; i < count; ++i)
        a[i] = b[i];
}
static bool can_send(void *context) {
    (void)context;
    return available;
}
static bool send(void *context, uint8_t *data, uint16_t size) {
    (void)context;
    if (!accept_send)
        return false;
    copy(sent, data, size);
    sent_size = size;
    ++sends;
    return true;
}
static bool pairing(void *context, const uint8_t peer[6]) {
    (void)context;
    (void)peer;
    return allow_pair;
}
static dbt_result_t load_key(void *context, const uint8_t peer[6], dbt_classic_key_t *key) {
    (void)context;
    ++loads;
    bool same = true;
    for (unsigned i = 0; i < 6; ++i)
        if (peer[i] != address[i])
            same = false;
    if (!has_cached || !same)
        return DBT_EMPTY;
    copy(key, &cached, sizeof(*key));
    return DBT_OK;
}
static bool store_key(void *context, const uint8_t peer[6], const dbt_classic_key_t *key,
                      uint32_t token) {
    (void)context;
    (void)peer;
    if (!allow_store || job_count == 16)
        return false;
    job_t *job = &jobs[job_count++];
    job->token = token;
    job->deleting = key == 0;
    if (key)
        copy(&job->key, key, sizeof(*key));
    return true;
}
static uint8_t *allocate(void *context, uint16_t size) {
    (void)context;
    if (fail_allocate || allocation_used || size > sizeof(extended))
        return 0;
    allocation_used = true;
    allocation_size = size;
    ++allocations;
    return extended;
}
static void release(void *context, uint8_t *bytes) {
    (void)context;
    if (bytes != extended || !allocation_used) {
        cleared_allocation = false;
        return;
    }
    for (unsigned i = 0; i < allocation_size; ++i)
        if (bytes[i])
            cleared_allocation = false;
    allocation_used = false;
    ++releases;
}
static void input(void *context, const uint8_t peer[6], dbt_classic_input_t kind, uint32_t token,
                  uint32_t value) {
    (void)context;
    (void)peer;
    input_kind = kind;
    input_token = token;
    input_value = value;
    ++input_calls;
}
static void display(void *context, const uint8_t peer[6], uint32_t value) {
    (void)context;
    (void)peer;
    input_value = value;
    ++display_calls;
}
static void event(void *context, dbt_classic_security_event_t kind,
                  const dbt_classic_security_info_t *snapshot, dbt_classic_security_reason_t why,
                  uint8_t status) {
    (void)context;
    (void)status;
    copy(&info, snapshot, sizeof(info));
    reason = why;
    if (kind == DBT_CLASSIC_SECURITY_READY)
        ++ready_events;
    else if (kind == DBT_CLASSIC_BOND_STORED)
        ++stored_events;
    else if (kind == DBT_CLASSIC_BOND_REMOVED)
        ++removed_events;
    else if (kind == DBT_CLASSIC_SECURITY_FAILED)
        ++failed_events;
    else
        ++lost_events;
}
static dbt_classic_security_config_t configuration(void) {
    dbt_classic_security_config_t c = {0};
    c.io_capability = 3;
    c.authentication_requirements = 4;
    c.minimum_key_size = 16;
    c.procedure_timeout_ms = 30000;
    c.store_timeout_ms = 5000;
    c.allow_just_works = true;
    c.allow_legacy = true;
    c.require_durable_bond = true;
    return c;
}
static dbt_classic_security_hooks_t hooks(void) {
    dbt_classic_security_hooks_t h = {0};
    h.pairing_allowed = pairing;
    h.load = load_key;
    h.store = store_key;
    h.input = input;
    h.display_passkey = display;
    h.event = event;
    h.allocate = allocate;
    h.release = release;
    return h;
}
static dbt_result_t acl_event(uint8_t code, const uint8_t *data, uint8_t size,
                              dbt_link_change_t *change) {
    dbt_hci_packet_t packet = {0};
    packet.type = 4;
    packet.event = code;
    packet.payload = data;
    packet.size = size;
    return dbt_acl_event(&acl, &packet, change);
}
static dbt_result_t add_link(uint16_t handle) {
    uint8_t bytes[11] = {0};
    bytes[1] = (uint8_t)handle;
    bytes[2] = (uint8_t)(handle >> 8);
    copy(bytes + 3, address, 6);
    bytes[9] = 1;
    dbt_link_change_t change;
    dbt_result_t result = acl_event(3, bytes, 11, &change);
    if (result != DBT_OK)
        return result;
    return dbt_classic_security_link(&security, &change);
}
static dbt_result_t drop_link(uint16_t handle) {
    uint8_t bytes[4] = {0, (uint8_t)handle, (uint8_t)(handle >> 8), 0x13};
    dbt_link_change_t change;
    dbt_result_t result = acl_event(5, bytes, 4, &change);
    if (result != DBT_OK)
        return result;
    return dbt_classic_security_link(&security, &change);
}
static void fixture(void) {
    wipe(&security, sizeof(security));
    wipe(&host, sizeof(host));
    wipe(&acl, sizeof(acl));
    wipe(&cached, sizeof(cached));
    wipe(jobs, sizeof(jobs));
    available = accept_send = allow_pair = allow_store = cleared_allocation = true;
    has_cached = fail_allocate = allocation_used = false;
    sends = loads = input_calls = display_calls = ready_events = stored_events = removed_events =
        failed_events = lost_events = job_count = 0;
    allocations = releases = allocation_size = 0;
    now = 100;
    sent_size = 0;
    input_token = input_value = 0;
    input_kind = DBT_CLASSIC_INPUT_NONE;
    dbt_transport_t transport = {0, can_send, send};
    (void)dbt_host_init(&host, transport, 0, 0, 1000);
    host.state = DBT_HOST_IDENTIFIED;
    host.command_credits = 1;
    dbt_controller_info_t c = {0};
    c.classic_acl_bytes = 1021;
    c.classic_acl_packets = 9;
    c.le_acl_bytes = 251;
    c.le_acl_packets = 10;
    (void)dbt_acl_init(&acl, &c);
    (void)dbt_classic_security_init(&security, &host, &acl, configuration(), hooks());
    security.now_ms = now;
    (void)add_link(1);
}
static dbt_result_t receive(uint8_t code, const uint8_t *bytes, uint8_t size) {
    uint8_t wire[80];
    wire[0] = 4;
    wire[1] = code;
    wire[2] = size;
    copy(wire + 3, bytes, size);
    dbt_hci_packet_t packet;
    dbt_result_t result = dbt_hci_decode(wire, (uint16_t)(size + 3u), &packet);
    if (result != DBT_OK)
        return result;
    dbt_link_change_t change;
    if (code == 8)
        (void)dbt_acl_event(&acl, &packet, &change);
    return dbt_classic_security_receive(&security, &packet, ++now);
}
static dbt_result_t address_event(uint8_t code) { return receive(code, address, 6); }
static void poll(void) {
    dbt_classic_security_poll(&security, ++now);
    dbt_host_poll(&host, now, 8);
}
static int command(uint16_t opcode) {
    unsigned previous = sends;
    poll();
    CHECK(sends == previous + 1 && sent[0] == 1 && dbt_read_le16(sent + 1) == opcode &&
          host.pending_opcode == opcode);
    return 0;
}
static int complete(uint8_t status, uint8_t key_size) {
    uint16_t opcode = host.pending_opcode;
    CHECK(opcode != 0);
    uint8_t packet[18];
    uint8_t size;
    if (opcode == 0x0411 || opcode == 0x0413 || opcode == 0x0c53) {
        packet[0] = 4;
        packet[1] = 0x0f;
        packet[2] = 4;
        packet[3] = status;
        packet[4] = 1;
        packet[5] = (uint8_t)opcode;
        packet[6] = (uint8_t)(opcode >> 8);
        size = 7;
    } else {
        packet[0] = 4;
        packet[1] = 0x0e;
        packet[3] = 1;
        packet[4] = (uint8_t)opcode;
        packet[5] = (uint8_t)(opcode >> 8);
        packet[6] = status;
        if (opcode == 0x1408) {
            packet[2] = 7;
            packet[7] = (uint8_t)security.active.handle;
            packet[8] = (uint8_t)(security.active.handle >> 8);
            packet[9] = key_size;
            size = 10;
        } else {
            packet[2] = 10;
            copy(packet + 7, security.active.address, 6);
            size = 13;
        }
    }
    CHECK(dbt_host_receive(&host, packet, size) == DBT_OK);
    dbt_host_poll(&host, ++now, 8);
    return 0;
}
static int exchange(uint8_t remote_io, uint8_t remote_auth) {
    dbt_classic_security_pairable(&security, true);
    CHECK(address_event(0x31) == DBT_OK);
    int r = command(0x042b);
    if (r)
        return r;
    CHECK(sent_size == 13 && sent[10] == security.config.io_capability &&
          sent[12] == security.config.authentication_requirements);
    r = complete(0, 0);
    if (r)
        return r;
    uint8_t bytes[9];
    copy(bytes, address, 6);
    bytes[6] = remote_io;
    bytes[7] = 0;
    bytes[8] = remote_auth;
    CHECK(receive(0x32, bytes, 9) == DBT_OK);
    return 0;
}
static int confirmation(uint32_t value) {
    uint8_t bytes[10];
    copy(bytes, address, 6);
    for (unsigned i = 0; i < 4; ++i)
        bytes[6 + i] = (uint8_t)(value >> (8u * i));
    CHECK(receive(0x33, bytes, 10) == DBT_OK);
    return 0;
}
static int notify_key(uint8_t type, uint8_t seed) {
    uint8_t bytes[23];
    copy(bytes, address, 6);
    for (unsigned i = 0; i < 16; ++i)
        bytes[6 + i] = (uint8_t)(seed + i);
    bytes[22] = type;
    CHECK(receive(0x18, bytes, 23) == DBT_OK);
    return 0;
}
static int pairing_complete(void) {
    uint8_t bytes[7] = {0};
    copy(bytes + 1, address, 6);
    CHECK(receive(0x36, bytes, 7) == DBT_OK);
    return 0;
}
static int encrypted(uint8_t mode) {
    uint8_t bytes[4] = {0, 1, 0, mode};
    CHECK(receive(8, bytes, 4) == DBT_OK);
    return 0;
}
static dbt_result_t complete_job(unsigned index, bool success) {
    if (success) {
        has_cached = !jobs[index].deleting;
        if (has_cached)
            copy(&cached, &jobs[index].key, sizeof(cached));
    }
    return dbt_classic_security_complete_store(&security, jobs[index].token, success);
}
static int fresh_key(uint8_t key_type) {
    dbt_classic_security_pairable(&security,true);
    CHECK(dbt_classic_security_require(&security,dbt_acl_find(&acl,1),DBT_SECURITY_ENCRYPTED,++now)==DBT_BUSY);
    int r=command(0x0411);
    if(r)return r;
    r=complete(0,0);
    if(r)return r;
    r = exchange(1, 4);
    if (r)
        return r;
    r = notify_key(key_type, 0x20);
    if (r)
        return r;
    r=pairing_complete();
    if(r)return r;
    uint8_t authenticated[]={0,1,0};
    CHECK(receive(6,authenticated,3)==DBT_OK);
    return 0;
}

EXPORT int test_classic_key_classification_and_configuration(void) {
    fixture();
    dbt_classic_key_t key = {0};
    CHECK(dbt_classic_key_valid(&key) && !dbt_classic_key_authenticated(&key));
    key.pin_length = 16;
    CHECK(!dbt_classic_key_authenticated(&key));
    key.type = 4;
    CHECK(!dbt_classic_key_authenticated(&key));
    key.type = 5;
    CHECK(dbt_classic_key_authenticated(&key));
    key.type = 8;
    CHECK(dbt_classic_key_secure_connections(&key));
    for (unsigned i = 1; i <= 3; ++i) {
        key.type = (uint8_t)i;
        CHECK(!dbt_classic_key_valid(&key));
    }
    key.type = 6;
    CHECK(!dbt_classic_key_valid(&key));
    dbt_classic_security_config_t config = configuration();
    config.authentication_requirements = 5;
    CHECK(dbt_classic_security_init(&security, &host, &acl, config, hooks()) == DBT_INVALID);
    config = configuration();
    config.require_secure_connections = true;
    CHECK(dbt_classic_security_init(&security, &host, &acl, config, hooks()) == DBT_INVALID);
    config = configuration();
    config.minimum_key_size = 6;
    CHECK(dbt_classic_security_init(&security, &host, &acl, config, hooks()) == DBT_INVALID);
    return 0;
}
EXPORT int test_classic_pairing_policy_and_cached_key(void) {
    fixture();
    CHECK(address_event(0x31) == DBT_OK);
    int r = command(0x0434);
    if (r)
        return r;
    CHECK(sent_size == 11 && sent[10] == 0x18);
    r = complete(0, 0);
    if (r)
        return r;
    CHECK(address_event(0x17) == DBT_OK);
    r = command(0x040c);
    if (r)
        return r;
    r = complete(0, 0);
    if (r)
        return r;
    has_cached = true;
    cached.type = 5;
    for (unsigned i = 0; i < 16; ++i)
        cached.bytes[i] = (uint8_t)(0x50u + i);
    CHECK(address_event(0x17) == DBT_OK);
    r = command(0x040b);
    if (r)
        return r;
    CHECK(sent_size == 26);
    for (unsigned i = 0; i < 16; ++i)
        CHECK(sent[10 + i] == cached.bytes[i]);
    r = complete(0, 0);
    if (r)
        return r;
    CHECK(ready_events == 0 && !dbt_classic_security_authorize(&security, dbt_acl_find(&acl, 1),
                                                               DBT_SECURITY_ENCRYPTED));
    CHECK(!security.pairable && loads > 0);
    return 0;
}
EXPORT int test_classic_ssp_persistence_and_encryption_gates(void) {
    fixture();
    int r = exchange(1, 5);
    if (r)
        return r;
    r = confirmation(123456);
    if (r)
        return r;
    r = command(0x042c);
    if (r)
        return r;
    r = complete(0, 0);
    if (r)
        return r;
    CHECK(input_calls == 0);
    r = notify_key(4, 0x30);
    if (r)
        return r;
    CHECK(job_count == 1 && stored_events == 0 && !security.peers[0].bonded);
    r = pairing_complete();
    if (r)
        return r;
    /* Incoming SSP belongs to the peer. Pairing alone must not make us issue
     * a competing Set Connection Encryption command. */
    unsigned previous_sends=sends;
    poll();
    CHECK(sends==previous_sends && !security.peers[0].encryption_requested);
    CHECK(ready_events == 0);
    r = encrypted(1);
    if (r)
        return r;
    r = command(0x1408);
    if (r)
        return r;
    r = complete(0, 16);
    if (r)
        return r;
    CHECK(ready_events == 0);
    CHECK(complete_job(0, true) == DBT_OK && stored_events == 1 && ready_events == 1);
    CHECK(info.bonded && info.encrypted && info.key_size == 16 && !info.authenticated);
    CHECK(dbt_classic_security_authorize(&security, dbt_acl_find(&acl, 1), DBT_SECURITY_ENCRYPTED));
    CHECK(!dbt_classic_security_authorize(&security, dbt_acl_find(&acl, 1),
                                          DBT_SECURITY_AUTHENTICATED));
    for (unsigned i = 0; i < 16; ++i)
        CHECK(security.peers[0].key.bytes[i] == 0);
    CHECK(address_event(0x17) == DBT_OK);
    r = command(0x040b);
    if (r)
        return r;
    CHECK(sent[10] == 0x30 && sent[25] == 0x3f);
    return complete(0, 0);
}
EXPORT int test_classic_manual_confirmation_and_rejection(void) {
    fixture();
    security.config.io_capability = 1;
    security.config.authentication_requirements = 5;
    int r = exchange(1, 5);
    if (r)
        return r;
    r = confirmation(654321);
    if (r)
        return r;
    CHECK(input_calls == 1 && input_kind == DBT_CLASSIC_INPUT_CONFIRM && input_value == 654321);
    uint8_t invalid = 1;
    CHECK(dbt_classic_security_reply(&security, input_token, true, &invalid, 1) == DBT_INVALID);
    CHECK(dbt_classic_security_reply(&security, input_token, false, 0, 0) == DBT_OK &&
          failed_events == 1);
    r = command(0x042d);
    if (r)
        return r;
    r = complete(0, 0);
    if (r)
        return r;
    r = notify_key(5, 0x50);
    if (r)
        return r;
    CHECK(job_count == 0 && reason == DBT_CLASSIC_REASON_POLICY);
    CHECK(dbt_classic_security_reply(&security, input_token, true, 0, 0) == DBT_INVALID);
    return 0;
}
EXPORT int test_classic_pin_passkey_and_timeout(void) {
    fixture();
    dbt_classic_security_pairable(&security, true);
    CHECK(address_event(0x16) == DBT_OK && input_kind == DBT_CLASSIC_INPUT_PIN);
    uint8_t pin[16];
    for (unsigned i = 0; i < 16; ++i)
        pin[i] = (uint8_t)('0' + i % 10);
    CHECK(dbt_classic_security_reply(&security, input_token, true, pin, 0) == DBT_INVALID);
    CHECK(dbt_classic_security_reply(&security, input_token, true, pin, 16) == DBT_OK);
    int r = command(0x040d);
    if (r)
        return r;
    CHECK(sent_size == 27 && sent[10] == 16);
    for (unsigned i = 0; i < 16; ++i)
        CHECK(sent[11 + i] == pin[i]);
    r = complete(0, 0);
    if (r)
        return r;
    r = notify_key(0, 0x10);
    if (r)
        return r;
    CHECK(job_count == 1 && jobs[0].key.pin_length == 16);
    fixture();
    dbt_classic_security_pairable(&security, true);
    CHECK(address_event(0x34) == DBT_OK && input_kind == DBT_CLASSIC_INPUT_PASSKEY);
    uint8_t passkey[4] = {0x40, 0x42, 0x0f, 0};
    CHECK(dbt_classic_security_reply(&security, input_token, true, passkey, 4) == DBT_INVALID);
    passkey[0] = 0x3f;
    CHECK(dbt_classic_security_reply(&security, input_token, true, passkey, 4) == DBT_OK);
    r = command(0x042e);
    if (r)
        return r;
    CHECK(sent_size == 14 && sent[10] == 0x3f);
    r = complete(0, 0);
    if (r)
        return r;
    fixture();
    dbt_classic_security_pairable(&security, true);
    CHECK(address_event(0x34) == DBT_OK);
    uint32_t deadline = security.peers[0].deadline;
    dbt_classic_security_poll(&security, deadline);
    now = deadline;
    dbt_host_poll(&host, now, 8);
    CHECK(host.pending_opcode == 0x042f && failed_events == 1 &&
          reason == DBT_CLASSIC_REASON_TIMEOUT);
    return complete(0, 0);
}
EXPORT int test_classic_oob_lazy_storage_and_bounds(void) {
    fixture();
    security.config.oob_present = 1;
    int r = exchange(1, 4);
    if (r)
        return r;
    CHECK(address_event(0x35) == DBT_OK && input_kind == DBT_CLASSIC_INPUT_OOB);
    uint8_t data[64];
    for (unsigned i = 0; i < 64; ++i)
        data[i] = (uint8_t)(i + 1);
    CHECK(dbt_classic_security_reply(&security, input_token, true, data, 31) == DBT_INVALID);
    fail_allocate = true;
    CHECK(dbt_classic_security_reply(&security, input_token, true, data, 32) == DBT_NO_SPACE &&
          input_token == security.peers[0].input_token);
    fail_allocate = false;
    CHECK(dbt_classic_security_reply(&security, input_token, true, data, 32) == DBT_OK &&
          allocations == 1 && allocation_size == 38);
    r = command(0x0430);
    if (r)
        return r;
    CHECK(releases == 1 && cleared_allocation && !allocation_used && sent_size == 42);
    for (unsigned i = 0; i < 32; ++i)
        CHECK(sent[10 + i] == data[i]);
    r = complete(0, 0);
    if (r)
        return r;
    fixture();
    security.config.oob_present = 3;
    security.config.controller_secure_connections_enabled = true;
    r = exchange(1, 4);
    if (r)
        return r;
    CHECK(address_event(0x35) == DBT_OK);
    CHECK(dbt_classic_security_reply(&security, input_token, true, data, 64) == DBT_OK);
    r = command(0x0445);
    if (r)
        return r;
    CHECK(sent_size == 74 && allocation_size == 70 && releases == 1);
    for (unsigned i = 0; i < 64; ++i)
        CHECK(sent[10 + i] == data[i]);
    return complete(0, 0);
}
EXPORT int test_classic_key_rejection_and_short_key(void) {
    fixture();
    int r = exchange(1, 4);
    if (r)
        return r;
    r = notify_key(3, 0x20);
    if (r)
        return r;
    CHECK(failed_events == 1 && job_count == 0 && reason == DBT_CLASSIC_REASON_KEY_TYPE);
    fixture();
    r = fresh_key(4);
    if (r)
        return r;
    r = command(0x0413);
    if (r)
        return r;
    r = complete(0, 0);
    if (r)
        return r;
    r = encrypted(1);
    if (r)
        return r;
    r = command(0x1408);
    if (r)
        return r;
    r = complete(0, 7);
    if (r)
        return r;
    CHECK(failed_events == 1 && reason == DBT_CLASSIC_REASON_KEY_SIZE && ready_events == 0);
    return 0;
}
EXPORT int test_classic_cached_auth_and_verified_key_size(void) {
    fixture();
    has_cached = true;
    cached.type = 5;
    cached.bytes[0] = 0x44;
    CHECK(dbt_classic_security_require(&security, dbt_acl_find(&acl, 1), DBT_SECURITY_AUTHENTICATED,
                                       ++now) == DBT_BUSY);
    int r = command(0x0411);
    if (r)
        return r;
    r = complete(0, 0);
    if (r)
        return r;
    CHECK(ready_events == 0);
    uint8_t authenticated[] = {0, 1, 0};
    CHECK(receive(6, authenticated, 3) == DBT_OK);
    r = command(0x0413);
    if (r)
        return r;
    r = complete(0, 0);
    if (r)
        return r;
    r = encrypted(1);
    if (r)
        return r;
    r = command(0x1408);
    if (r)
        return r;
    CHECK(ready_events == 0);
    r = complete(0, 16);
    if (r)
        return r;
    CHECK(ready_events == 1 && info.authenticated &&
          dbt_classic_security_authorize(&security, dbt_acl_find(&acl, 1),
                                         DBT_SECURITY_AUTHENTICATED));
    dbt_link_t stale = *dbt_acl_find(&acl, 1);
    ++stale.generation;
    CHECK(!dbt_classic_security_authorize(&security, &stale, DBT_SECURITY_NONE));
    r = encrypted(0);
    if (r)
        return r;
    CHECK(failed_events == 1 && !dbt_classic_security_authorize(&security, dbt_acl_find(&acl, 1),
                                                                DBT_SECURITY_ENCRYPTED));
    return 0;
}
EXPORT int test_classic_store_failure_timeout_and_forget(void) {
    fixture();
    allow_store = false;
    int r = fresh_key(4);
    if (r)
        return r;
    CHECK(failed_events == 1 && reason == DBT_CLASSIC_REASON_STORAGE);
    fixture();
    r = fresh_key(4);
    if (r)
        return r;
    uint32_t token = jobs[0].token;
    dbt_classic_security_poll(&security, security.peers[0].store_deadline);
    CHECK(failed_events == 1 && !security.peers[0].bonded);
    CHECK(complete_job(0, true) == DBT_OK && ready_events == 0 && stored_events == 0);
    CHECK(dbt_classic_security_complete_store(&security, token, true) == DBT_EMPTY);
    fixture();
    has_cached = true;
    cached.type = 5;
    CHECK(dbt_classic_security_forget(&security, address, ++now) == DBT_OK && job_count == 1 &&
          jobs[0].deleting);
    CHECK(dbt_classic_security_clear(&security) == DBT_BUSY);
    CHECK(complete_job(0, true) == DBT_OK && removed_events == 1 && !has_cached &&
          failed_events == 1);
    return 0;
}
EXPORT int test_classic_disconnect_generation_and_store_completion(void) {
    fixture();
    int r = fresh_key(4);
    if (r)
        return r;
    uint32_t generation = acl.links[0].generation;
    CHECK(drop_link(1) == DBT_OK && lost_events == 1);
    CHECK(add_link(1) == DBT_BUSY);
    CHECK(complete_job(0, true) == DBT_OK && stored_events == 1 && !info.connected &&
          ready_events == 0);
    dbt_link_change_t change = {DBT_LINK_ADDED, 1, acl.links[0].generation};
    CHECK(dbt_classic_security_link(&security, &change) == DBT_OK);
    CHECK(acl.links[0].generation != generation &&
          security.peers[0].generation == acl.links[0].generation);
    CHECK(dbt_classic_security_complete_store(&security, jobs[0].token, true) == DBT_EMPTY);
    CHECK(address_event(0x17) == DBT_OK);
    r = command(0x040b);
    if (r)
        return r;
    CHECK(drop_link(1) == DBT_OK);
    r = complete(0, 0);
    if (r)
        return r;
    CHECK(security.stale_completions >= 2);
    CHECK(add_link(1) == DBT_OK);
    CHECK(
        !dbt_classic_security_authorize(&security, dbt_acl_find(&acl, 1), DBT_SECURITY_ENCRYPTED));
    return 0;
}
EXPORT int test_classic_rotation_waits_for_new_cipher_and_commit(void) {
    fixture();
    int r = fresh_key(4);
    if (r)
        return r;
    r = command(0x0413);
    if (r)
        return r;
    r = complete(0, 0);
    if (r)
        return r;
    r = encrypted(1);
    if (r)
        return r;
    r = command(0x1408);
    if (r)
        return r;
    r = complete(0, 16);
    if (r)
        return r;
    CHECK(complete_job(0, true) == DBT_OK && ready_events == 1);
    r = notify_key(6, 0x70);
    if (r)
        return r;
    CHECK(job_count == 2 && !security.peers[0].ready && security.peers[0].key_size == 0);
    CHECK(complete_job(1, true) == DBT_OK && stored_events == 2 && ready_events == 1);
    r = command(0x0c53);
    if (r)
        return r;
    r = complete(0, 0);
    if (r)
        return r;
    CHECK(ready_events == 1);
    uint8_t refreshed[] = {0, 1, 0};
    CHECK(receive(0x30, refreshed, 3) == DBT_OK);
    r = command(0x1408);
    if (r)
        return r;
    r = complete(0, 16);
    if (r)
        return r;
    CHECK(ready_events == 2 &&
          dbt_classic_security_authorize(&security, dbt_acl_find(&acl, 1), DBT_SECURITY_ENCRYPTED));
    return 0;
}
EXPORT int test_classic_stale_cipher_query_and_command_backpressure(void) {
    fixture();
    has_cached = true;
    cached.type = 5;
    int r = encrypted(1);
    if (r)
        return r;
    r = command(0x1408);
    if (r)
        return r;
    uint8_t refreshed[] = {0, 1, 0};
    CHECK(receive(0x30, refreshed, 3) == DBT_OK);
    r = complete(0, 16);
    if (r)
        return r;
    CHECK(ready_events == 0 && security.peers[0].key_size == 0);
    r = command(0x1408);
    if (r)
        return r;
    r = complete(0, 16);
    if (r)
        return r;
    CHECK(ready_events == 1);
    fixture();
    available = false;
    CHECK(address_event(0x17) == DBT_OK);
    poll();
    CHECK(security.command_active && host.pending_opcode == 0);
    CHECK(dbt_classic_security_clear(&security) == DBT_BUSY);
    dbt_host_abort(&host, DBT_FAULT_CONTROLLER);
    CHECK(!security.command_active && failed_events == 1);
    CHECK(dbt_classic_security_clear(&security) == DBT_OK);
    return 0;
}
EXPORT int test_sensitive_queue_and_hci_buffer_lifetime(void) {
    fixture();
    uint8_t bytes[26];
    bytes[0] = 4;
    bytes[1] = 0x18;
    bytes[2] = 23;
    for (unsigned i = 3; i < 26; ++i)
        bytes[i] = (uint8_t)(0x90 + i);
    uint32_t start = atomic_load(&host.ingress.written);
    CHECK(dbt_host_receive(&host, bytes, 26) == DBT_OK);
    dbt_host_poll(&host, ++now, 8);
    for (unsigned i = 0; i < 26; ++i) {
        CHECK(host.rx_packet[i] == 0);
        CHECK(host.ingress.bytes[(start + 2u + i) % DBT_QUEUE_BYTES] == 0);
    }
    uint8_t key_reply[22];
    for (unsigned i = 0; i < 22; ++i)
        key_reply[i] = (uint8_t)(i + 1);
    CHECK(dbt_host_command(&host, 0x040b, key_reply, 22, DBT_COMMAND_COMPLETE, 0, 0, now) ==
          DBT_OK);
    dbt_host_poll(&host, ++now, 8);
    CHECK(host.tx_retained && host.tx_command[25] == 22);
    for (unsigned i = 0; i < 255; ++i)
        CHECK(host.commands[host.command_head].parameters[i] == 0);
    available = false;
    dbt_host_poll(&host, ++now, 8);
    CHECK(host.tx_command[25] == 22);
    available = true;
    dbt_host_poll(&host, ++now, 8);
    for (unsigned i = 0; i < 259; ++i)
        CHECK(host.tx_command[i] == 0);
    dbt_host_abort(&host, DBT_FAULT_CONTROLLER);
    return 0;
}

EXPORT int test_classic_command_collision_waits_for_actual_events(void) {
    fixture();
    int r = fresh_key(4);
    if (r)
        return r;
    r = command(0x0413);
    if (r)
        return r;
    r = complete(0x0c, 0);
    if (r)
        return r;
    CHECK(failed_events == 0 && ready_events == 0 && security.rejected_commands == 1);
    CHECK(complete_job(0, true) == DBT_OK && ready_events == 0);
    r = encrypted(1);
    if (r)
        return r;
    r = command(0x1408);
    if (r)
        return r;
    r = complete(0, 16);
    if (r)
        return r;
    CHECK(ready_events == 1 &&
          dbt_classic_security_authorize(&security, dbt_acl_find(&acl, 1), DBT_SECURITY_ENCRYPTED));
    dbt_host_abort(&host, DBT_FAULT_CONTROLLER);
    CHECK(
        !dbt_classic_security_authorize(&security, dbt_acl_find(&acl, 1), DBT_SECURITY_ENCRYPTED));
    return 0;
}

EXPORT int test_classic_ssp_defers_local_security_demand(void) {
    fixture();
    int r=exchange(1,4);if(r)return r;
    CHECK(dbt_classic_security_require(&security,dbt_acl_find(&acl,1),DBT_SECURITY_ENCRYPTED,++now)==DBT_BUSY);
    unsigned previous_sends=sends;
    poll();
    CHECK(sends==previous_sends && security.peers[0].locally_required && !security.peers[0].auth_requested);
    r=notify_key(4,0x20);if(r)return r;
    r=pairing_complete();if(r)return r;
    r=command(0x0411);if(r)return r;
    r=complete(0,0);if(r)return r;
    previous_sends=sends;poll();
    CHECK(sends==previous_sends && !security.peers[0].auth_complete && !security.peers[0].encryption_requested);
    uint8_t authenticated[]={0,1,0};
    CHECK(receive(6,authenticated,3)==DBT_OK);
    r=command(0x0413);if(r)return r;
    r=complete(0,0);if(r)return r;
    CHECK(!dbt_classic_security_authorize(&security,dbt_acl_find(&acl,1),DBT_SECURITY_ENCRYPTED));
    r=encrypted(1);if(r)return r;
    r=command(0x1408);if(r)return r;
    r=complete(0,16);if(r)return r;
    CHECK(!ready_events && complete_job(0,true)==DBT_OK && ready_events==1 && !failed_events);
    return 0;
}
