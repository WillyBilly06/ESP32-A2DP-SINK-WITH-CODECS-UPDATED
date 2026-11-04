#include "dbt_att.h"
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
static dbt_att_server_t server;
static dbt_link_t links[2];
static _Alignas(16) uint8_t storage[32][600];
static bool used[32], fail_next, allow, bad_read;
static uint16_t allocated_sizes[32], live_bytes, peak_bytes;
static unsigned allocations, releases, writes_called, confirmations, failures, commits;
static uint16_t write_result, hook_error_handle;
static uint32_t now, last_token;
static uint8_t value[64], secure_value[64], long_value[100], saved_cccd[2];
static uint16_t value_size, secure_size;
static uint8_t output[517];
static uint16_t output_size;
static dbt_att_write_t copied_writes[8];
static uint8_t copied_data[8][512];
static dbt_att_cccd_t copied_cccds[16];
static uint8_t copied_count, copied_cccd_count;
static bool copied_prepared;
static const uint8_t primary[] = {0, 0x28}, secondary[] = {1, 0x28}, declaration[] = {3, 0x28},
                     cccd[] = {2, 0x29};
static const uint8_t name_type[] = {0, 0x2a}, service[] = {0, 0x18},
                     second_service[] = {0x12, 0x18};
static const uint8_t device_name[] = {'S', 'P', 'K'}, name_decl[] = {2, 3, 0, 0, 0x2a};
static const uint8_t custom_uuid[] = {0,    0x90, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12,
                                      0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12};
static const uint8_t custom_value_uuid[] = {1,    0x90, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12,
                                            0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12};
static const uint8_t custom_decl[] = {0x3a, 0x12, 0,    1,    0x90, 0x78, 0x56, 0x34, 0x12, 0x34,
                                      0x12, 0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12};
static const uint8_t secure_type[] = {2, 0xff}, notify_type[] = {3, 0xff}, long_type[] = {4, 0xff},
                     authorized_type[] = {5, 0xff};
static const uint8_t base_appearance[] = {0xfb, 0x34, 0x9b, 0x5f, 0x80, 0,    0, 0x80,
                                          0,    0x10, 0,    0,    1,    0x2a, 0, 0};
static const dbt_attribute_t attributes[] = {
    {1, DBT_ATT_READ, 2, 2, 0, 2, 0, primary, service},
    {2, DBT_ATT_READ, 5, 5, 0, 2, 0, declaration, name_decl},
    {3, DBT_ATT_READ, 3, 3, 0, 2, 0, name_type, device_name},
    {0x10, DBT_ATT_READ, 16, 16, 0, 2, 0, primary, custom_uuid},
    {0x11, DBT_ATT_READ, 19, 19, 0, 2, 0, declaration, custom_decl},
    {0x12, DBT_ATT_READ | DBT_ATT_WRITE | DBT_ATT_NOTIFY | DBT_ATT_INDICATE | DBT_ATT_DYNAMIC, 64,
     0, 0, 16, 0, custom_value_uuid, 0},
    {0x13, DBT_ATT_READ | DBT_ATT_WRITE, 2, 2, 0x12, 2, 0, cccd, 0},
    {0x14,
     DBT_ATT_READ | DBT_ATT_WRITE | DBT_ATT_READ_AUTHENTICATE | DBT_ATT_WRITE_AUTHENTICATE |
         DBT_ATT_DYNAMIC,
     64, 0, 0, 2, 16, secure_type, 0},
    {0x15, DBT_ATT_NOTIFY | DBT_ATT_READ_ENCRYPT, 64, 0, 0, 2, 16, notify_type, 0},
    {0x16, DBT_ATT_READ | DBT_ATT_WRITE, 2, 2, 0x15, 2, 0, cccd, 0},
    {0x17, DBT_ATT_READ, 100, 100, 0, 2, 0, long_type, long_value},
    {0x18, DBT_ATT_READ | DBT_ATT_DYNAMIC | DBT_ATT_READ_AUTHORIZE, 64, 0, 0, 2, 0, authorized_type,
     0},
    {0x20, DBT_ATT_READ, 2, 2, 0, 2, 0, secondary, second_service},
    {0x21, DBT_ATT_READ, 2, 2, 0, 16, 0, base_appearance, service}};
#define ATTRIBUTE_COUNT (sizeof(attributes) / sizeof(attributes[0]))
static void copy(uint8_t *out, const uint8_t *in, unsigned n) {
    for (unsigned i = 0; i < n; ++i)
        out[i] = in[i];
}
static void zero(void *data, unsigned n) {
    uint8_t *p = data;
    for (unsigned i = 0; i < n; ++i)
        p[i] = 0;
}
static uint8_t *allocate(void *context, uint16_t size) {
    (void)context;
    if (fail_next) {
        fail_next = false;
        return 0;
    }
    if (size > 600)
        return 0;
    for (unsigned i = 0; i < 32; ++i)
        if (!used[i]) {
            used[i] = true;
            allocated_sizes[i] = size;
            ++allocations;
            live_bytes += size;
            if (live_bytes > peak_bytes)
                peak_bytes = live_bytes;
            return storage[i];
        }
    return 0;
}
static void release(void *context, uint8_t *bytes) {
    (void)context;
    for (unsigned i = 0; i < 32; ++i)
        if (bytes == storage[i] && used[i]) {
            used[i] = false;
            live_bytes -= allocated_sizes[i];
            ++releases;
            return;
        }
    live_bytes = 0xffff;
}
static uint8_t authorize(void *context, const dbt_link_t *link, const dbt_attribute_t *attribute,
                         bool write) {
    (void)context;
    (void)link;
    (void)attribute;
    (void)write;
    return allow ? 0 : DBT_ATT_AUTHORIZATION;
}
static uint8_t read(void *context, const dbt_link_t *link, const dbt_attribute_t *attribute,
                    uint16_t offset, uint8_t *out, uint16_t capacity, uint16_t *written,
                    uint16_t *total) {
    (void)context;
    (void)link;
    uint8_t *source = attribute->handle == 0x14 ? secure_value : value;
    *total = attribute->handle == 0x14 ? secure_size : value_size;
    if (offset > *total)
        return DBT_ATT_INVALID_OFFSET;
    *written = (uint16_t)(*total - offset);
    if (*written > capacity)
        *written = capacity;
    copy(out, source + offset, *written);
    if (bad_read)
        ++*written;
    return 0;
}
static uint16_t apply_copied(void) {
    uint8_t next_value[64], next_secure[64];
    copy(next_value, value, 64);
    copy(next_secure, secure_value, 64);
    uint16_t size = value_size, secure = secure_size;
    for (unsigned i = 0; i < copied_count; ++i) {
        dbt_att_write_t *w = &copied_writes[i];
        if (w->handle == 0x13 || w->handle == 0x16)
            continue;
        uint8_t *target = w->handle == 0x14 ? next_secure : next_value;
        uint16_t *length = w->handle == 0x14 ? &secure : &size;
        if (w->offset > *length || (uint32_t)w->offset + w->size > 64)
            return DBT_ATT_INVALID_OFFSET;
        copy(target + w->offset, w->data, w->size);
        if (!copied_prepared)
            *length = w->size;
        else if (w->offset + w->size > *length)
            *length = (uint16_t)(w->offset + w->size);
    }
    copy(value, next_value, 64);
    copy(secure_value, next_secure, 64);
    value_size = size;
    secure_size = secure;
    for (unsigned i = 0; i < copied_cccd_count; ++i)
        saved_cccd[copied_cccds[i].handle == 0x13 ? 0 : 1] = (uint8_t)copied_cccds[i].value;
    ++commits;
    return 0;
}
static uint16_t write(void *context, const dbt_link_t *link, const dbt_att_batch_t *batch,
                      uint32_t token, uint16_t *bad_handle) {
    (void)context;
    (void)link;
    ++writes_called;
    last_token = token;
    *bad_handle = hook_error_handle;
    copied_count = batch->count;
    copied_cccd_count = batch->cccd_count;
    copied_prepared = batch->prepared;
    for (unsigned i = 0; i < batch->count; ++i) {
        copied_writes[i] = batch->writes[i];
        copy(copied_data[i], batch->writes[i].data, batch->writes[i].size);
        copied_writes[i].data = copied_data[i];
    }
    for (unsigned i = 0; i < batch->cccd_count; ++i)
        copied_cccds[i] = batch->cccds[i];
    if (write_result)
        return write_result;
    return apply_copied();
}
static dbt_result_t load_cccd(void *context, const dbt_link_t *link, uint16_t handle,
                              uint16_t *out) {
    (void)context;
    (void)link;
    *out = saved_cccd[handle == 0x13 ? 0 : 1];
    return *out ? DBT_OK : DBT_EMPTY;
}
static void confirmed(void *context, uint16_t handle, uint32_t generation, uint16_t attribute) {
    (void)context;
    (void)handle;
    (void)generation;
    (void)attribute;
    ++confirmations;
}
static void failed(void *context, uint16_t handle, uint32_t generation) {
    (void)context;
    (void)handle;
    (void)generation;
    ++failures;
}
static dbt_att_hooks_t hooks(void) {
    dbt_att_hooks_t h = {0};
    h.allocate = allocate;
    h.release = release;
    h.authorize = authorize;
    h.read = read;
    h.write = write;
    h.load_cccd = load_cccd;
    h.confirmed = confirmed;
    h.failed = failed;
    return h;
}
static void fixture(void) {
    dbt_att_clear(&server);
    zero(&server, sizeof(server));
    zero(used, sizeof(used));
    zero(links, sizeof(links));
    live_bytes = peak_bytes = 0;
    allocations = releases = writes_called = confirmations = failures = commits = 0;
    allow = true;
    fail_next = bad_read = false;
    write_result = hook_error_handle = 0;
    now = 100;
    last_token = 0;
    output_size = 0;
    saved_cccd[0] = saved_cccd[1] = 0;
    value_size = 8;
    secure_size = 4;
    for (unsigned i = 0; i < 64; ++i) {
        value[i] = (uint8_t)(0x40u + i);
        secure_value[i] = (uint8_t)(0x80u + i);
    }
    for (unsigned i = 0; i < 100; ++i)
        long_value[i] = (uint8_t)i;
    (void)dbt_att_init(&server, attributes, ATTRIBUTE_COUNT, hooks(), 517, 128);
    for (unsigned i = 0; i < 2; ++i) {
        links[i].used = true;
        links[i].kind = DBT_LINK_LE;
        links[i].handle = (uint16_t)(i + 1u);
        links[i].generation = i + 1u;
        (void)dbt_att_open(&server, &links[i]);
    }
}
static dbt_result_t rx(unsigned peer, const uint8_t *data, uint16_t size) {
    return dbt_att_receive(&server, &links[peer], data, size, ++now);
}
static int take(unsigned peer, uint8_t opcode) {
    dbt_att_view_t view;
    CHECK(dbt_att_tx_peek(&server, links[peer].handle, links[peer].generation, &view) == DBT_OK);
    CHECK(view.size <= sizeof(output) && view.data[0] == opcode);
    output_size = view.size;
    copy(output, view.data, view.size);
    CHECK(dbt_att_tx_admitted(&server, links[peer].handle, links[peer].generation, ++now) ==
          DBT_OK);
    return 0;
}
static int expect_error(unsigned peer, uint8_t request, uint16_t handle, uint8_t error) {
    int r = take(peer, 1);
    if (r)
        return r;
    CHECK(output_size == 5 && output[1] == request && dbt_read_le16(output + 2) == handle &&
          output[4] == error);
    return 0;
}
static int mtu(unsigned peer, uint16_t size) {
    uint8_t request[] = {2, (uint8_t)size, (uint8_t)(size >> 8)};
    CHECK(rx(peer, request, sizeof(request)) == DBT_OK);
    return take(peer, 3);
}
static int subscribe(unsigned peer, uint16_t handle, uint16_t flags) {
    uint8_t request[] = {0x12, (uint8_t)handle, (uint8_t)(handle >> 8), (uint8_t)flags,
                         (uint8_t)(flags >> 8)};
    CHECK(rx(peer, request, sizeof(request)) == DBT_OK);
    return take(peer, 0x13);
}
static int prepared(unsigned peer, uint16_t handle, uint16_t offset, const uint8_t *bytes,
                    uint16_t size) {
    uint8_t request[517];
    request[0] = 0x16;
    request[1] = (uint8_t)handle;
    request[2] = (uint8_t)(handle >> 8);
    request[3] = (uint8_t)offset;
    request[4] = (uint8_t)(offset >> 8);
    copy(request + 5, bytes, size);
    CHECK(rx(peer, request, (uint16_t)(size + 5u)) == DBT_OK);
    return take(peer, 0x17);
}
EXPORT int test_att_database_validation_and_link_lifecycle(void) {
    fixture();
    CHECK(server.cccd_count == 2 && live_bytes == 0 && allocations == 0);
    CHECK(dbt_att_open(&server, &links[0]) == DBT_BUSY);
    dbt_link_t classic = links[0];
    classic.kind = DBT_LINK_CLASSIC;
    CHECK(dbt_att_open(&server, &classic) == DBT_UNSUPPORTED);
    dbt_attribute_t invalid[ATTRIBUTE_COUNT];
    for (unsigned i = 0; i < ATTRIBUTE_COUNT; ++i)
        invalid[i] = attributes[i];
    invalid[1].handle = 1;
    dbt_att_server_t scratch;
    CHECK(dbt_att_init(&scratch, invalid, ATTRIBUTE_COUNT, hooks(), 517, 128) == DBT_INVALID);
    invalid[1] = attributes[1];
    invalid[6].cccd_for = 0xffff;
    CHECK(dbt_att_init(&scratch, invalid, ATTRIBUTE_COUNT, hooks(), 517, 128) == DBT_INVALID);
    CHECK(dbt_att_init(&scratch, attributes, ATTRIBUTE_COUNT, hooks(), 22, 128) == DBT_INVALID);
    dbt_att_close(&server, 1, 99);
    CHECK(server.connections[0].used);
    dbt_att_close(&server, 1, 1);
    CHECK(!server.connections[0].used);
    links[0].generation = 44;
    CHECK(dbt_att_open(&server, &links[0]) == DBT_OK && server.connections[0].mtu == 23);
    return 0;
}
EXPORT int test_att_discovery_mixed_uuid_and_groups(void) {
    fixture();
    uint8_t information[] = {4, 1, 0, 3, 0};
    CHECK(rx(0, information, 5) == DBT_OK);
    int r = take(0, 5);
    if (r)
        return r;
    CHECK(output_size == 14 && output[1] == 1 && output[12] == 0 && output[13] == 0x2a);
    information[1] = 0x10;
    information[3] = 0x18;
    CHECK(rx(0, information, 5) == DBT_OK);
    r = take(0, 5);
    if (r)
        return r;
    CHECK(output_size == 10 && output[1] == 1);
    information[1] = 0x12;
    CHECK(rx(0, information, 5) == DBT_OK);
    r = take(0, 5);
    if (r)
        return r;
    CHECK(output_size == 20 && output[1] == 2);
    uint8_t groups[] = {0x10, 1, 0, 0xff, 0xff, 0, 0x28};
    CHECK(rx(0, groups, 7) == DBT_OK);
    r = take(0, 0x11);
    if (r)
        return r;
    CHECK(output_size == 8 && output[1] == 6 && dbt_read_le16(output + 2) == 1 &&
          dbt_read_le16(output + 4) == 3);
    groups[1] = 0x10;
    CHECK(rx(0, groups, 7) == DBT_OK);
    r = take(0, 0x11);
    if (r)
        return r;
    CHECK(output_size == 22 && output[1] == 20 && dbt_read_le16(output + 4) == 0x18);
    uint8_t by_value[] = {6, 1, 0, 0xff, 0xff, 0, 0x28, 0, 0x18};
    CHECK(rx(0, by_value, 9) == DBT_OK);
    r = take(0, 7);
    if (r)
        return r;
    CHECK(output_size == 5 && dbt_read_le16(output + 1) == 1 && dbt_read_le16(output + 3) == 3);
    uint8_t declarations[] = {8, 1, 0, 0xff, 0xff, 3, 0x28};
    CHECK(rx(0, declarations, 7) == DBT_OK);
    r = take(0, 9);
    if (r)
        return r;
    CHECK(output_size == 9 && output[1] == 7 && output[4] == 2);
    return 0;
}
EXPORT int test_att_read_blob_multiple_and_base_uuid(void) {
    fixture();
    uint8_t request[] = {0x0c, 0x17, 0, 22, 0};
    CHECK(rx(0, request, 5) == DBT_OK);
    int r = take(0, 0x0d);
    if (r)
        return r;
    CHECK(output_size == 23 && output[1] == 22 && output[22] == 43);
    request[3] = 100;
    CHECK(rx(0, request, 5) == DBT_OK);
    r = take(0, 0x0d);
    if (r)
        return r;
    CHECK(output_size == 1);
    request[3] = 101;
    CHECK(rx(0, request, 5) == DBT_OK);
    r = expect_error(0, 0x0c, 0x17, DBT_ATT_INVALID_OFFSET);
    if (r)
        return r;
    request[1] = 3;
    request[3] = 0;
    CHECK(rx(0, request, 5) == DBT_OK);
    r = expect_error(0, 0x0c, 3, DBT_ATT_NOT_LONG);
    if (r)
        return r;
    uint8_t multiple[] = {0x0e, 3, 0, 0x17, 0};
    CHECK(rx(0, multiple, 5) == DBT_OK);
    r = take(0, 0x0f);
    if (r)
        return r;
    CHECK(output_size == 23 && output[1] == 'S' && output[4] == 0 && output[22] == 18);
    uint8_t invalid_late[] = {0x0e, 0x17, 0, 0xff, 0xff};
    CHECK(rx(0, invalid_late, 5) == DBT_OK);
    r = expect_error(0, 0x0e, 0xffff, DBT_ATT_INVALID_HANDLE);
    if (r)
        return r;
    uint8_t variable[] = {0x20, 3, 0, 0x17, 0};
    CHECK(rx(0, variable, 5) == DBT_OK);
    r = take(0, 0x21);
    if (r)
        return r;
    CHECK(output_size == 23 && dbt_read_le16(output + 1) == 3 && dbt_read_le16(output + 6) == 15);
    uint8_t by_type[] = {8, 1, 0, 0xff, 0xff, 1, 0x2a};
    CHECK(rx(0, by_type, 7) == DBT_OK);
    r = take(0, 9);
    if (r)
        return r;
    CHECK(output_size == 6 && dbt_read_le16(output + 2) == 0x21);
    return 0;
}
EXPORT int test_att_mtu_bounds_and_errors(void) {
    fixture();
    int r = mtu(0, 1000);
    if (r)
        return r;
    CHECK(server.connections[0].mtu == 517 && dbt_read_le16(output + 1) == 517);
    uint8_t again[] = {2, 23, 0};
    CHECK(rx(0, again, 3) == DBT_OK);
    r = expect_error(0, 2, 0, DBT_ATT_NOT_SUPPORTED);
    if (r)
        return r;
    r = mtu(1, 1);
    if (r)
        return r;
    CHECK(server.connections[1].mtu == 23);
    uint8_t bad[] = {0x0a, 3};
    CHECK(rx(0, bad, 2) == DBT_OK);
    r = expect_error(0, 0x0a, 0, DBT_ATT_INVALID_PDU);
    if (r)
        return r;
    uint8_t range[] = {4, 0, 0, 1, 0};
    CHECK(rx(0, range, 5) == DBT_OK);
    r = expect_error(0, 4, 0, DBT_ATT_INVALID_HANDLE);
    if (r)
        return r;
    uint8_t unsupported[] = {0x22};
    CHECK(rx(0, unsupported, 1) == DBT_OK);
    r = expect_error(0, 0x22, 0, DBT_ATT_NOT_SUPPORTED);
    if (r)
        return r;
    uint8_t group[] = {0x10, 1, 0, 0xff, 0xff, 3, 0x28};
    CHECK(rx(0, group, 7) == DBT_OK);
    return expect_error(0, 0x10, 1, DBT_ATT_GROUP_TYPE);
}
EXPORT int test_att_security_and_notify_only_protection(void) {
    fixture();
    uint8_t request[] = {0x0a, 0x14, 0};
    CHECK(rx(0, request, 3) == DBT_OK);
    int r = expect_error(0, 0x0a, 0x14, DBT_ATT_AUTHENTICATION);
    if (r)
        return r;
    links[0].encrypted = true;
    CHECK(dbt_att_security(&server, &links[0], true, 7, false) == DBT_OK);
    CHECK(rx(0, request, 3) == DBT_OK);
    r = expect_error(0, 0x0a, 0x14, DBT_ATT_KEY_SIZE);
    if (r)
        return r;
    CHECK(dbt_att_security(&server, &links[0], true, 16, false) == DBT_OK);
    CHECK(rx(0, request, 3) == DBT_OK);
    r = take(0, 0x0b);
    if (r)
        return r;
    CHECK(output_size == 5 && output[1] == 0x80);
    r = subscribe(0, 0x16, 1);
    if (r)
        return r;
    CHECK(dbt_att_notify(&server, &links[0], 0x15, value, 4, false) == DBT_OK);
    r = take(0, 0x1b);
    if (r)
        return r;
    links[0].encrypted = false;
    CHECK(dbt_att_notify(&server, &links[0], 0x15, value, 4, false) == DBT_INVALID);
    CHECK(dbt_att_security(&server, &links[0], true, 16, false) == DBT_INVALID);
    request[1] = 0x18;
    allow = false;
    CHECK(rx(0, request, 3) == DBT_OK);
    return expect_error(0, 0x0a, 0x18, DBT_ATT_AUTHORIZATION);
}
EXPORT int test_att_deferred_write_ack_and_stale_completion(void) {
    fixture();
    write_result = DBT_ATT_DEFERRED;
    uint8_t request[] = {0x12, 0x12, 0, 1, 2, 3};
    CHECK(rx(0, request, 6) == DBT_OK && writes_called == 1 && commits == 0 && value[0] == 0x40);
    dbt_att_view_t view;
    CHECK(dbt_att_tx_peek(&server, 1, 1, &view) == DBT_EMPTY);
    CHECK(rx(0, request, 6) == DBT_BUSY);
    uint8_t command[] = {0x52, 0x12, 0, 9};
    CHECK(rx(0, command, 4) == DBT_OK && server.ignored_commands == 1);
    CHECK(dbt_att_complete(&server, 1, 1, last_token + 1, 0, 0) == DBT_INVALID);
    CHECK(apply_copied() == 0);
    CHECK(dbt_att_complete(&server, 1, 1, last_token, 0, 0) == DBT_OK);
    int r = take(0, 0x13);
    if (r)
        return r;
    CHECK(value_size == 3 && value[2] == 3 && commits == 1);
    CHECK(rx(0, request, 6) == DBT_OK);
    uint32_t token = last_token;
    dbt_att_close(&server, 1, 1);
    links[0].generation = 33;
    CHECK(dbt_att_open(&server, &links[0]) == DBT_OK);
    CHECK(dbt_att_complete(&server, 1, 1, token, 0, 0) == DBT_INVALID);
    CHECK(dbt_att_complete(&server, 1, 33, token, 0, 0) == DBT_INVALID);
    return 0;
}
EXPORT int test_att_prepare_execute_atomic_and_cancel(void) {
    fixture();
    uint8_t a[] = {9, 8}, b[] = {7, 6};
    int r = prepared(0, 0x12, 0, a, 2);
    if (r)
        return r;
    r = prepared(0, 0x12, 2, b, 2);
    if (r)
        return r;
    CHECK(writes_called == 0 && server.connections[0].prepared_count == 2 && allocations == 2);
    uint8_t execute[] = {0x18, 1};
    CHECK(rx(0, execute, 2) == DBT_OK);
    r = take(0, 0x19);
    if (r)
        return r;
    CHECK(writes_called == 1 && commits == 1 && copied_count == 2 && value[0] == 9 &&
          value[3] == 6 && live_bytes == 0);
    r = prepared(0, 0x12, 0, a, 2);
    if (r)
        return r;
    r = prepared(1, 0x12, 0, b, 2);
    if (r)
        return r;
    execute[1] = 0;
    CHECK(rx(0, execute, 2) == DBT_OK);
    r = take(0, 0x19);
    if (r)
        return r;
    CHECK(server.connections[0].prepared_count == 0 && server.connections[1].prepared_count == 1 &&
          commits == 1);
    write_result = DBT_ATT_INVALID_LENGTH;
    hook_error_handle = 0x12;
    execute[1] = 1;
    CHECK(rx(1, execute, 2) == DBT_OK);
    r = expect_error(1, 0x18, 0x12, DBT_ATT_INVALID_LENGTH);
    if (r)
        return r;
    CHECK(value[0] == 9 && commits == 1 && live_bytes == 0);
    return 0;
}
EXPORT int test_att_prepare_limits_oom_and_deferred_execute(void) {
    fixture();
    fail_next = true;
    uint8_t request[] = {0x16, 0x12, 0, 0, 0, 0xaa};
    CHECK(rx(0, request, 6) == DBT_OK);
    int r = expect_error(0, 0x16, 0x12, DBT_ATT_PREPARE_FULL);
    if (r)
        return r;
    CHECK(server.connections[0].prepared_count == 0);
    for (unsigned i = 0; i < 8; ++i) {
        r = prepared(0, 0x12, 0, request + 5, 1);
        if (r)
            return r;
    }
    CHECK(rx(0, request, 6) == DBT_OK);
    r = expect_error(0, 0x16, 0x12, DBT_ATT_PREPARE_FULL);
    if (r)
        return r;
    write_result = DBT_ATT_DEFERRED;
    uint8_t execute[] = {0x18, 1};
    CHECK(rx(0, execute, 2) == DBT_OK && live_bytes == 0);
    CHECK(copied_count == 8 && commits == 0);
    CHECK(apply_copied() == 0);
    CHECK(dbt_att_complete(&server, 1, 1, last_token, 0, 0) == DBT_OK);
    r = take(0, 0x19);
    if (r)
        return r;
    CHECK(value[0] == 0xaa && commits == 1);
    return 0;
}
EXPORT int test_att_cccd_per_client_bonds_and_indications(void) {
    fixture();
    CHECK(dbt_att_notify(&server, &links[0], 0x12, value, 4, false) == DBT_UNSUPPORTED);
    int r = subscribe(0, 0x13, 3);
    if (r)
        return r;
    CHECK(dbt_att_notify(&server, &links[1], 0x12, value, 4, false) == DBT_UNSUPPORTED);
    CHECK(dbt_att_notify(&server, &links[0], 0x12, value, 4, true) == DBT_OK);
    r = take(0, 0x1d);
    if (r)
        return r;
    CHECK(dbt_att_notify(&server, &links[0], 0x12, value, 4, true) == DBT_BUSY);
    CHECK(dbt_att_notify(&server, &links[0], 0x12, value, 4, false) == DBT_OK);
    r = take(0, 0x1b);
    if (r)
        return r;
    uint8_t confirmation[] = {0x1e};
    CHECK(rx(0, confirmation, 1) == DBT_OK && confirmations == 1);
    CHECK(rx(0, confirmation, 1) == DBT_OK && confirmations == 1);
    dbt_att_close(&server, 1, 1);
    links[0].generation = 44;
    CHECK(dbt_att_open(&server, &links[0]) == DBT_OK);
    CHECK(dbt_att_notify(&server, &links[0], 0x12, value, 4, false) == DBT_UNSUPPORTED);
    CHECK(dbt_att_security(&server, &links[0], false, 0, true) == DBT_OK);
    CHECK(dbt_att_notify(&server, &links[0], 0x12, value, 4, false) == DBT_OK);
    r = take(0, 0x1b);
    if (r)
        return r;
    CHECK(dbt_att_security(&server, &links[0], false, 0, false) == DBT_OK);
    CHECK(dbt_att_notify(&server, &links[0], 0x12, value, 4, false) == DBT_UNSUPPORTED);
    return 0;
}
EXPORT int test_att_cccd_queued_transaction_and_error(void) {
    fixture();
    uint8_t low[] = {3}, high[] = {0};
    int r = prepared(0, 0x13, 0, low, 1);
    if (r)
        return r;
    r = prepared(0, 0x13, 1, high, 1);
    if (r)
        return r;
    write_result = DBT_ATT_DEFERRED;
    uint8_t execute[] = {0x18, 1};
    CHECK(rx(0, execute, 2) == DBT_OK);
    CHECK(copied_cccd_count == 1 && copied_cccds[0].value == 3 &&
          dbt_att_notify(&server, &links[0], 0x12, value, 4, false) == DBT_UNSUPPORTED);
    CHECK(dbt_att_complete(&server, 1, 1, last_token, DBT_ATT_UNLIKELY, 0x13) == DBT_OK);
    r = expect_error(0, 0x18, 0x13, DBT_ATT_UNLIKELY);
    if (r)
        return r;
    CHECK(dbt_att_notify(&server, &links[0], 0x12, value, 4, false) == DBT_UNSUPPORTED);
    write_result = 0;
    uint8_t invalid[] = {0x12, 0x13, 0, 4, 0};
    CHECK(rx(0, invalid, 5) == DBT_OK);
    return expect_error(0, 0x12, 0x13, DBT_ATT_INVALID_LENGTH);
}
EXPORT int test_att_buffer_pins_oom_and_timeouts(void) {
    fixture();
    int r = mtu(0, 517);
    if (r)
        return r;
    fail_next = true;
    uint8_t request[] = {0x0a, 0x17, 0};
    CHECK(rx(0, request, 3) == DBT_OK);
    r = expect_error(0, 0x0a, 0, DBT_ATT_RESOURCES);
    if (r)
        return r;
    CHECK(rx(0, request, 3) == DBT_OK);
    dbt_att_view_t first, second;
    CHECK(dbt_att_tx_peek(&server, 1, 1, &first) == DBT_OK);
    CHECK(dbt_att_tx_peek(&server, 1, 1, &second) == DBT_OK && first.data == second.data &&
          first.size == 101);
    CHECK(rx(0, request, 3) == DBT_BUSY && allocations == 1);
    CHECK(dbt_att_tx_admitted(&server, 1, 1, ++now) == DBT_OK && live_bytes == 0);
    write_result = DBT_ATT_DEFERRED;
    uint8_t write_request[] = {0x12, 0x12, 0, 1};
    CHECK(rx(0, write_request, 4) == DBT_OK);
    uint32_t start = now;
    CHECK(dbt_att_wait_ms(&server, now) == 30000);
    dbt_att_poll(&server, start + 29999);
    CHECK(failures == 0);
    dbt_att_poll(&server, start + 30000);
    CHECK(failures == 1);
    CHECK(dbt_att_complete(&server, 1, 1, last_token, 0, 0) == DBT_INVALID &&
          rx(0, request, 3) == DBT_BUSY);
    dbt_att_poll(&server, start + 40000);
    CHECK(failures == 1);
    return 0;
}
EXPORT int test_att_command_rules_and_bad_dynamic_read(void) {
    fixture();
    uint8_t signed_command[] = {0xd2, 0x12, 0, 1}, unknown[] = {0x42, 1, 2};
    CHECK(rx(0, signed_command, 4) == DBT_OK && rx(0, unknown, 3) == DBT_OK && writes_called == 0 &&
          server.ignored_commands == 2);
    uint8_t command[] = {0x52, 0x12, 0, 9, 8};
    CHECK(rx(0, command, 5) == DBT_OK && writes_called == 1 && value[0] == 9);
    dbt_att_view_t view;
    CHECK(dbt_att_tx_peek(&server, 1, 1, &view) == DBT_EMPTY);
    uint8_t invalid_handle[] = {0x12, 0, 0, 1};
    CHECK(rx(0, invalid_handle, 4) == DBT_OK);
    int r = expect_error(0, 0x12, 0, DBT_ATT_INVALID_HANDLE);
    if (r)
        return r;
    uint8_t read_request[] = {0x0a, 0x12, 0};
    bad_read = true;
    CHECK(rx(0, read_request, 3) == DBT_OK);
    return expect_error(0, 0x0a, 0x12, DBT_ATT_UNLIKELY);
}

EXPORT int test_att_generated_requests_and_resource_reclamation(void) {
    uint32_t random = 0x41545434u;
    for (unsigned opcode = 0; opcode < 256; ++opcode) {
        for (unsigned length = 1; length <= 40; ++length) {
            fixture();
            uint8_t packet[40];
            for (unsigned i = 0; i < length; ++i) {
                random = random * 1664525u + 1013904223u;
                packet[i] = (uint8_t)(random >> 24);
            }
            packet[0] = (uint8_t)opcode;
            dbt_result_t result = rx(0, packet, (uint16_t)length);
            CHECK(result == DBT_OK || result == DBT_INVALID || result == DBT_UNSUPPORTED);
            dbt_att_view_t view;
            result = dbt_att_tx_peek(&server, 1, 1, &view);
            if (result == DBT_OK) {
                CHECK(view.size > 0 && view.size <= 23 && view.data != 0);
                CHECK(dbt_att_tx_admitted(&server, 1, 1, ++now) == DBT_OK);
            } else CHECK(result == DBT_EMPTY);
            dbt_att_clear(&server);
            CHECK(live_bytes == 0 && releases == allocations);
        }
    }
    return 0;
}
