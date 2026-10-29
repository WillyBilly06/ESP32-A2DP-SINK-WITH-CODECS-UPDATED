#include "dbt_sdp.h"

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

static const uint8_t classes[] = {0x35, 3, 0x19, 0x11, 0x0b};
static const uint8_t protocol[] = {0x35, 8, 0x35, 6, 0x19, 1, 0, 9, 0, 0x19};
static const uint8_t browse[] = {0x35, 3, 0x19, 0x10, 2};
static const uint8_t name[] = {0x25, 4, 'T', 'e', 's', 't'};
static const uint8_t nil[] = {0};
static const uint8_t vendor[] = {0x1c, 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0,
                                 0x81, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef};
static const dbt_sdp_attribute_t attributes[] = {
    {1, sizeof(classes), classes}, {4, sizeof(protocol), protocol},
    {5, sizeof(browse), browse},   {0x100, sizeof(name), name},
    {0x101, sizeof(nil), nil},     {0x200, sizeof(vendor), vendor}};
static dbt_sdp_record_t records[DBT_SDP_RECORD_LIMIT];
static dbt_sdp_t server;
static uint8_t request[512], reply[1024], assembled[4096];
static uint16_t length, written;

static uint16_t read16(const uint8_t *p) { return (uint16_t)((uint16_t)p[0] << 8 | p[1]); }
static void set16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)v;
}
static bool equal(const uint8_t *a, const uint8_t *b, unsigned size) {
    for (unsigned i = 0; i < size; ++i)
        if (a[i] != b[i])
            return false;
    return true;
}
static bool fixture(unsigned count) {
    for (unsigned i = 0; i < DBT_SDP_RECORD_LIMIT; ++i) {
        records[i].handle = 0x10000u + i;
        records[i].attributes = attributes;
        records[i].count = sizeof(attributes) / sizeof(attributes[0]);
    }
    return dbt_sdp_init(&server, records, (uint16_t)count, 1) == DBT_OK;
}
static void begin(uint8_t kind) {
    request[0] = kind;
    request[1] = 0x12;
    request[2] = 0x34;
    length = 5;
}
static void byte(uint8_t value) { request[length++] = value; }
static void word(uint16_t value) {
    byte((uint8_t)(value >> 8));
    byte((uint8_t)value);
}
static void pattern(uint16_t uuid) {
    byte(0x35);
    byte(3);
    byte(0x19);
    word(uuid);
}
static void all_attributes(void) {
    byte(0x35);
    byte(5);
    byte(0x0a);
    word(0);
    word(0xffff);
}
static void finish(void) {
    byte(0);
    set16(request + 3, (uint16_t)(length - 5));
}
static void search(uint16_t uuid, uint16_t maximum) {
    begin(2);
    pattern(uuid);
    word(maximum);
    finish();
}
static void attribute(uint16_t maximum) {
    begin(4);
    word(1);
    word(0);
    word(maximum);
    all_attributes();
    finish();
}
static void combined(uint16_t uuid, uint16_t maximum) {
    begin(6);
    pattern(uuid);
    word(maximum);
    all_attributes();
    finish();
}
static bool respond(uint16_t capacity) {
    return dbt_sdp_respond(&server, 123, request, length, reply, capacity, &written) == DBT_OK;
}
static bool error(uint16_t code) {
    return respond(sizeof(reply)) && written == 7 && reply[0] == 1 && reply[1] == request[1] &&
           reply[2] == request[2] && read16(reply + 3) == 2 && read16(reply + 5) == code;
}

EXPORT int test_sdp_database_and_elements(void) {
    CHECK(fixture(1));
    dbt_sdp_t previous = server;
    CHECK(dbt_sdp_init(&server, 0, 1, 2) == DBT_INVALID);
    CHECK(server.records == previous.records && server.database_state == 1);
    records[0].handle = 1;
    CHECK(dbt_sdp_init(&server, records, 1, 2) == DBT_INVALID);
    CHECK(fixture(2));
    records[1].handle = records[0].handle;
    CHECK(dbt_sdp_init(&server, records, 2, 2) == DBT_INVALID);
    dbt_sdp_attribute_t attr = {1, 1, nil};
    records[0].attributes = &attr;
    records[0].count = 1;
    CHECK(dbt_sdp_init(&server, records, 1, 2) == DBT_INVALID);
    const uint8_t bad_uuid[] = {0x35, 9, 0x1b, 0, 0, 0, 0, 0, 0, 0, 0};
    attr.value = bad_uuid;
    attr.size = sizeof(bad_uuid);
    CHECK(dbt_sdp_init(&server, records, 1, 2) == DBT_INVALID);
    const uint8_t huge[] = {0x37, 0xff, 0xff, 0xff, 0xff};
    attr.value = huge;
    attr.size = sizeof(huge);
    CHECK(dbt_sdp_init(&server, records, 1, 2) == DBT_INVALID);
    CHECK(dbt_sdp_init(&server, 0, 0, 2) == DBT_OK);
    search(0x110b, 10);
    CHECK(respond(48));
    CHECK(written == 10 && read16(reply + 5) == 0 && read16(reply + 7) == 0 && reply[9] == 0);
    return 0;
}

EXPORT int test_sdp_uuid_search_and_network_order(void) {
    CHECK(fixture(3));
    search(0x110b, 2);
    CHECK(respond(48));
    const uint8_t expected[] = {3, 0x12, 0x34, 0, 13, 0, 2, 0, 2, 0, 1, 0, 0, 0, 1, 0, 1, 0};
    CHECK(written == sizeof(expected) && equal(reply, expected, written));
    begin(2);
    byte(0x35);
    byte(5);
    byte(0x1a);
    word(0);
    word(0x110b);
    word(10);
    finish();
    CHECK(respond(48) && read16(reply + 5) == 3);
    begin(2);
    byte(0x35);
    byte(17);
    byte(0x1c);
    word(0);
    word(0x110b);
    const uint8_t tail[] = {0, 0, 0x10, 0, 0x80, 0, 0, 0x80, 0x5f, 0x9b, 0x34, 0xfb};
    for (unsigned i = 0; i < sizeof(tail); ++i)
        byte(tail[i]);
    word(10);
    finish();
    CHECK(respond(48) && read16(reply + 5) == 3);
    request[12] ^= 1;
    CHECK(respond(48) && read16(reply + 5) == 0);
    begin(2);
    byte(0x35);
    byte(6);
    byte(0x19);
    word(0x110b);
    byte(0x19);
    word(0x100);
    word(10);
    finish();
    CHECK(respond(48) && read16(reply + 5) == 3);
    request[12] = 2;
    CHECK(respond(48) && read16(reply + 5) == 0);
    begin(2);
    byte(0x35);
    byte(sizeof(vendor));
    for (unsigned i = 0; i < sizeof(vendor); ++i)
        byte(vendor[i]);
    word(10);
    finish();
    CHECK(respond(48) && read16(reply + 5) == 3);
    return 0;
}

EXPORT int test_sdp_attributes_golden_and_nil_omission(void) {
    CHECK(fixture(1));
    /* Select handle, class, name and nil; nil must not appear in the answer. */
    begin(4);
    word(1);
    word(0);
    word(0xffff);
    byte(0x35);
    byte(10);
    byte(0x0a);
    word(0);
    word(1);
    byte(0x0a);
    word(0x100);
    word(0x101);
    finish();
    CHECK(respond(48));
    const uint8_t expected[] = {5,    0x12, 0x34, 0, 30,   0, 27,  0x35, 25,   9,   0,    0,
                                10,   0,    1,    0, 0,    9, 0,   1,    0x35, 3,   0x19, 0x11,
                                0x0b, 9,    1,    0, 0x25, 4, 'T', 'e',  's',  't', 0};
    CHECK(written == sizeof(expected) && equal(reply, expected, written));
    request[8] = 1;
    CHECK(error(2));
    begin(4);
    word(1);
    word(0);
    word(7);
    byte(0x35);
    byte(3);
    byte(9);
    word(0x9999);
    finish();
    CHECK(respond(48) && written == 10 && reply[7] == 0x35 && reply[8] == 0 && reply[9] == 0);
    return 0;
}

EXPORT int test_sdp_request_validation(void) {
    CHECK(fixture(1));
    search(0x110b, 0);
    CHECK(error(3));
    search(0x110b, 1);
    request[3] = 1;
    CHECK(error(4));
    search(0x110b, 1);
    request[0] = 3;
    CHECK(error(3));
    begin(2);
    byte(0x35);
    byte(0);
    word(1);
    finish();
    CHECK(error(3));
    begin(2);
    byte(0x35);
    byte(39);
    for (unsigned i = 0; i < 13; ++i) {
        byte(0x19);
        word(0x110b);
    }
    word(1);
    finish();
    CHECK(error(3));
    attribute(6);
    CHECK(error(3));
    begin(4);
    word(1);
    word(0);
    word(7);
    byte(0x35);
    byte(6);
    byte(9);
    word(1);
    byte(9);
    word(1);
    finish();
    CHECK(error(3));
    begin(4);
    word(1);
    word(0);
    word(7);
    byte(0x35);
    byte(5);
    byte(10);
    word(3);
    word(2);
    finish();
    CHECK(error(3));
    search(0x110b, 1);
    request[length - 1] = 1;
    CHECK(error(4));
    byte(0);
    set16(request + 3, (uint16_t)(length - 5));
    CHECK(error(5));
    CHECK(dbt_sdp_respond(&server, 123, request, 4, reply, 48, &written) == DBT_INVALID &&
          written == 0);
    CHECK(dbt_sdp_respond(&server, 123, request, length, reply, 6, &written) == DBT_NO_SPACE &&
          written == 0);
    CHECK(dbt_sdp_respond(&server, 123, request, length, request, 48, &written) == DBT_INVALID);
    return 0;
}

EXPORT int test_sdp_search_continuation_and_limits(void) {
    CHECK(fixture(12));
    search(0x110b, 12);
    unsigned count = 0, turns = 0;
    do {
        CHECK(respond(48) && reply[0] == 3 && read16(reply + 5) == 12);
        uint16_t got = read16(reply + 7);
        CHECK(got && count + got <= 12);
        for (unsigned i = 0; i < got; ++i) {
            const uint8_t *handle = reply + 9 + i * 4;
            CHECK(handle[0] == 0 && handle[1] == 1 && handle[2] == 0 && handle[3] == count + i);
        }
        count += got;
        unsigned token_at = 9 + got * 4;
        if (!reply[token_at])
            break;
        CHECK(reply[token_at] == 16);
        length = 12;
        for (unsigned i = 0; i < 17; ++i)
            byte(reply[token_at + i]);
        set16(request + 3, (uint16_t)(length - 5));
        request[2]++;
        CHECK(++turns < 10);
    } while (true);
    CHECK(count == 12 && turns == 1);
    search(0x110b, 1);
    CHECK(respond(48) && read16(reply + 5) == 1 && read16(reply + 7) == 1);
    return 0;
}

EXPORT int test_sdp_attribute_continuation_exact_allocations(void) {
    CHECK(fixture(1));
    attribute(0xffff);
    CHECK(respond(sizeof(reply)));
    uint16_t total = read16(reply + 5);
    for (unsigned i = 0; i < total; ++i)
        assembled[2048 + i] = reply[7 + i];
    attribute(7);
    uint16_t initial_length = (uint16_t)(length - 1), used = 0;
    unsigned turns = 0;
    do {
        uint16_t measured;
        CHECK(dbt_sdp_respond(&server, 123, request, length, 0, 48, &measured) == DBT_OK);
        CHECK(respond(measured) && written == measured && reply[0] == 5);
        uint16_t got = read16(reply + 5);
        CHECK(got >= 2 && got <= 7 && used + got <= total);
        for (unsigned i = 0; i < got; ++i)
            assembled[used + i] = reply[7 + i];
        used = (uint16_t)(used + got);
        if (!reply[7 + got])
            break;
        CHECK(reply[7 + got] == 16);
        length = initial_length;
        for (unsigned i = 0; i < 17; ++i)
            byte(reply[7 + got + i]);
        set16(request + 3, (uint16_t)(length - 5));
        ++request[2];
        CHECK(++turns < 100);
    } while (true);
    CHECK(used == total && equal(assembled, assembled + 2048, total));
    return 0;
}

EXPORT int test_sdp_continuation_binding_and_database(void) {
    CHECK(fixture(1));
    attribute(7);
    CHECK(respond(48));
    uint16_t got = read16(reply + 5);
    CHECK(reply[7 + got] == 16);
    length--;
    for (unsigned i = 0; i < 17; ++i)
        byte(reply[7 + got + i]);
    set16(request + 3, (uint16_t)(length - 5));
    CHECK(dbt_sdp_respond(&server, 124, request, length, reply, 48, &written) == DBT_OK &&
          read16(reply + 5) == 5);
    CHECK(dbt_sdp_init(&server, records, 1, 2) == DBT_OK);
    CHECK(error(5));
    CHECK(dbt_sdp_init(&server, records, 1, 1) == DBT_OK);
    request[10] = 8;
    CHECK(error(5));
    request[10] = 7;
    request[length - 1] = 0xff;
    CHECK(error(5));
    for (unsigned i = length - 4; i < length; ++i)
        request[i] = 0;
    CHECK(error(5));
    return 0;
}

EXPORT int test_sdp_combined_lists_and_empty_match(void) {
    CHECK(fixture(2));
    combined(0x1002, 0xffff);
    CHECK(respond(sizeof(reply)) && reply[0] == 7);
    uint16_t total = read16(reply + 5);
    CHECK(reply[7] == 0x35 && reply[8] == total - 2 && reply[9] == 0x35);
    uint16_t first = (uint16_t)(reply[10] + 2);
    CHECK(reply[9 + first] == 0x35 && reply[10 + first] == first - 2);
    /* The generated handles differ, while class/protocol/name bytes agree. */
    CHECK(reply[18] == 0 && reply[18 + first] == 1);
    combined(0x9999, 7);
    CHECK(respond(48) && written == 10 && read16(reply + 5) == 2 && reply[8] == 0);
    combined(0x1002, 7);
    uint16_t used = 0, initial = (uint16_t)(length - 1);
    for (unsigned turn = 0; turn < 100; ++turn) {
        CHECK(respond(48) && reply[0] == 7);
        uint16_t got = read16(reply + 5);
        CHECK(got >= 2 && got <= 7 && used + got <= sizeof(assembled));
        for (unsigned i = 0; i < got; ++i)
            assembled[used + i] = reply[7 + i];
        used = (uint16_t)(used + got);
        if (!reply[7 + got])
            break;
        length = initial;
        for (unsigned i = 0; i < 17; ++i)
            byte(reply[7 + got + i]);
        set16(request + 3, (uint16_t)(length - 5));
        ++request[2];
    }
    CHECK(used == total && assembled[0] == 0x35 && assembled[1] == total - 2);
    return 0;
}

EXPORT int test_sdp_generated_inputs_and_output_bounds(void) {
    CHECK(fixture(2));
    uint32_t state = 0x629da1u;
    for (unsigned trial = 0; trial < 8192; ++trial) {
        state = state * 1664525u + 1013904223u;
        length = (uint16_t)(5 + state % 127);
        for (unsigned i = 0; i < length; ++i) {
            state = state * 1664525u + 1013904223u;
            request[i] = (uint8_t)(state >> 24);
        }
        if (trial & 1)
            set16(request + 3, (uint16_t)(length - 5));
        for (unsigned i = 0; i < sizeof(reply); ++i)
            reply[i] = 0xa5;
        CHECK(respond(48) && written >= 7 && written <= 48);
        CHECK(read16(reply + 3) == written - 5 && reply[1] == request[1] && reply[2] == request[2]);
        for (unsigned i = written; i < sizeof(reply); ++i)
            CHECK(reply[i] == 0xa5);
    }
    return 0;
}
