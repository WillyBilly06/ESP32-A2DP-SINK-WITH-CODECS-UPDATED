#include "dbt_sdp.h"

/* Owned implementation of Core Vol 3 Part B. No reference-stack dependencies. */
typedef struct {
    const uint8_t *value;
    uint16_t size, total;
    uint8_t type;
} element_t;

typedef struct {
    element_t pattern, attributes;
    uint32_t handle, fingerprint, offset;
    uint16_t maximum;
    uint8_t kind;
} query_t;

typedef struct {
    uint8_t *output;
    uint32_t skip;
    uint16_t capacity, written;
} slice_t;

static uint16_t be16(const uint8_t *p) { return (uint16_t)((uint16_t)p[0] << 8 | p[1]); }
static uint32_t be32(const uint8_t *p) {
    return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3];
}
static void put16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)v;
}
static void put32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)v;
}

static bool element(const uint8_t *data, uint16_t available, element_t *out) {
    if (!available || !data)
        return false;
    uint8_t type = data[0] >> 3, index = data[0] & 7u;
    uint16_t header = 1;
    uint32_t bytes;
    if (type > 8 || (type == 0 && index != 0) || ((type == 1 || type == 2) && index > 4) ||
        (type == 3 && index != 1 && index != 2 && index != 4) || (type == 5 && index != 0) ||
        ((type == 4 || type == 6 || type == 7 || type == 8) && index < 5))
        return false;
    if (index < 5)
        bytes = type ? (uint32_t)1u << index : 0;
    else {
        header = (uint16_t)(1u + (1u << (index - 5u)));
        if (available < header)
            return false;
        bytes = index == 5 ? data[1] : index == 6 ? be16(data + 1) : be32(data + 1);
    }
    if (bytes > (uint32_t)available - header)
        return false;
    out->type = type;
    out->value = data + header;
    out->size = (uint16_t)bytes;
    out->total = (uint16_t)(header + bytes);
    return true;
}

static bool valid_tree(const uint8_t *data, uint16_t size, unsigned depth) {
    element_t item;
    if (!element(data, size, &item) || item.total != size)
        return false;
    if (item.type != 6 && item.type != 7)
        return true;
    if (depth >= DBT_SDP_DEPTH_LIMIT)
        return false;
    uint16_t at = 0;
    while (at < item.size) {
        element_t child;
        if (!element(item.value + at, (uint16_t)(item.size - at), &child) ||
            !valid_tree(item.value + at, child.total, depth + 1))
            return false;
        at = (uint16_t)(at + child.total);
    }
    return true;
}

static bool uuid_list(const element_t *list, bool bounded) {
    if (list->type != 6 || !list->size)
        return false;
    uint16_t at = 0;
    unsigned count = 0;
    while (at < list->size) {
        element_t uuid;
        if (!element(list->value + at, (uint16_t)(list->size - at), &uuid) || uuid.type != 3)
            return false;
        at = (uint16_t)(at + uuid.total);
        if (++count > 12 && bounded)
            return false;
    }
    return true;
}

dbt_result_t dbt_sdp_init(dbt_sdp_t *server, const dbt_sdp_record_t *records, uint16_t count,
                          uint32_t database_state) {
    if (!server || (count && !records) || count > DBT_SDP_RECORD_LIMIT || !database_state)
        return DBT_INVALID;
    for (unsigned i = 0; i < count; ++i) {
        const dbt_sdp_record_t *record = &records[i];
        if ((record->handle && record->handle < 0x10000u) ||
            (i && record->handle <= records[i - 1].handle) || !record->attributes ||
            !record->count || record->count > DBT_SDP_ATTRIBUTE_LIMIT ||
            record->attributes[0].id != 1)
            return DBT_INVALID;
        uint32_t bytes = 8; /* generated handle attribute */
        for (unsigned j = 0; j < record->count; ++j) {
            const dbt_sdp_attribute_t *attr = &record->attributes[j];
            if ((j && attr->id <= record->attributes[j - 1].id) || !attr->size ||
                !valid_tree(attr->value, attr->size, 0))
                return DBT_INVALID;
            bytes += 3u + attr->size;
            if (bytes > DBT_SDP_RECORD_BYTES)
                return DBT_NO_SPACE;
        }
        element_t classes;
        if (!element(record->attributes[0].value, record->attributes[0].size, &classes) ||
            !uuid_list(&classes, false))
            return DBT_INVALID;
    }
    server->records = records;
    server->count = count;
    server->database_state = database_state;
    return DBT_OK;
}

static bool uuid_equal(const element_t *a, const element_t *b) {
    static const uint8_t tail[12] = {0, 0, 0x10, 0, 0x80, 0, 0, 0x80, 0x5f, 0x9b, 0x34, 0xfb};
    if (a->size == b->size) {
        for (unsigned i = 0; i < a->size; ++i)
            if (a->value[i] != b->value[i])
                return false;
        return true;
    }
    const element_t *large = a->size > b->size ? a : b;
    const element_t *small = large == a ? b : a;
    if (large->size == 16)
        for (unsigned i = 0; i < 12; ++i)
            if (large->value[4 + i] != tail[i])
                return false;
    uint32_t short_value = small->size == 2 ? be16(small->value) : be32(small->value);
    return be32(large->value) == short_value;
}

static bool contains_uuid(const uint8_t *data, uint16_t size, const element_t *uuid,
                          unsigned depth) {
    element_t value;
    if (!element(data, size, &value))
        return false;
    if (value.type == 3)
        return uuid_equal(&value, uuid);
    if ((value.type != 6 && value.type != 7) || depth >= DBT_SDP_DEPTH_LIMIT)
        return false;
    uint16_t at = 0;
    while (at < value.size) {
        element_t child;
        if (!element(value.value + at, (uint16_t)(value.size - at), &child))
            return false;
        if (contains_uuid(value.value + at, child.total, uuid, depth + 1))
            return true;
        at = (uint16_t)(at + child.total);
    }
    return false;
}

static bool matches(const dbt_sdp_record_t *record, const element_t *pattern) {
    uint16_t at = 0;
    while (at < pattern->size) {
        element_t uuid;
        if (!element(pattern->value + at, (uint16_t)(pattern->size - at), &uuid))
            return false;
        bool found = false;
        for (unsigned i = 0; i < record->count && !found; ++i)
            found =
                contains_uuid(record->attributes[i].value, record->attributes[i].size, &uuid, 0);
        if (!found)
            return false;
        at = (uint16_t)(at + uuid.total);
    }
    return true;
}

static bool ranges_valid(const element_t *ranges) {
    if (ranges->type != 6 || !ranges->size)
        return false;
    uint16_t at = 0, previous = 0;
    bool first = true;
    while (at < ranges->size) {
        element_t range;
        if (!element(ranges->value + at, (uint16_t)(ranges->size - at), &range) ||
            range.type != 1 || (range.size != 2 && range.size != 4))
            return false;
        uint16_t start = be16(range.value), end = range.size == 2 ? start : be16(range.value + 2);
        if (start > end || (!first && start <= previous))
            return false;
        previous = end;
        first = false;
        at = (uint16_t)(at + range.total);
    }
    return true;
}

static bool selected(const element_t *ranges, uint16_t id) {
    uint16_t at = 0;
    while (at < ranges->size) {
        element_t range;
        if (!element(ranges->value + at, (uint16_t)(ranges->size - at), &range))
            return false;
        uint16_t start = be16(range.value), end = range.size == 2 ? start : be16(range.value + 2);
        if (id < start)
            return false;
        if (id <= end)
            return true;
        at = (uint16_t)(at + range.total);
    }
    return false;
}

static uint16_t parse(const dbt_sdp_t *server, uint32_t binding, const uint8_t *request,
                      uint16_t size, query_t *query) {
    if (be16(request + 3) != size - 5u)
        return 4;
    query->kind = request[0];
    if (query->kind != 2 && query->kind != 4 && query->kind != 6)
        return 3;
    uint16_t at = 5;
    if (query->kind == 4) {
        if (size - at < 4)
            return 4;
        query->handle = be32(request + at);
        at += 4;
    } else {
        if (!element(request + at, (uint16_t)(size - at), &query->pattern) ||
            !uuid_list(&query->pattern, true))
            return 3;
        at = (uint16_t)(at + query->pattern.total);
    }
    if (size - at < 2)
        return 4;
    query->maximum = be16(request + at);
    at += 2;
    if (query->maximum < (query->kind == 2 ? 1u : 7u))
        return 3;
    if (query->kind != 2) {
        if (!element(request + at, (uint16_t)(size - at), &query->attributes) ||
            !ranges_valid(&query->attributes))
            return 3;
        at = (uint16_t)(at + query->attributes.total);
    }
    if (at == size)
        return 4;
    uint8_t continuation = request[at];
    if ((uint32_t)at + 1u + continuation != size)
        return 4;
    if (continuation && continuation != 16)
        return 5;
    uint32_t hash = (2166136261u ^ query->kind) * 16777619u;
    for (unsigned i = 5; i < at; ++i)
        hash = (hash ^ request[i]) * 16777619u;
    query->fingerprint = hash;
    query->offset = 0;
    if (continuation) {
        const uint8_t *token = request + at + 1;
        if (be32(token) != server->database_state || be32(token + 4) != binding ||
            be32(token + 8) != hash || !(query->offset = be32(token + 12)))
            return 5;
    }
    return 0;
}

static uint8_t sequence(uint8_t *header, uint32_t size) {
    if (size <= 255) {
        header[0] = 0x35;
        header[1] = (uint8_t)size;
        return 2;
    }
    if (size <= 65535) {
        header[0] = 0x36;
        put16(header + 1, (uint16_t)size);
        return 3;
    }
    header[0] = 0x37;
    put32(header + 1, size);
    return 5;
}

static uint32_t attribute_bytes(const dbt_sdp_record_t *record, const element_t *ranges) {
    uint32_t bytes = selected(ranges, 0) ? 8u : 0u;
    for (unsigned i = 0; i < record->count; ++i)
        if (record->attributes[i].value[0] && selected(ranges, record->attributes[i].id))
            bytes += 3u + record->attributes[i].size;
    return bytes;
}

static void emit(slice_t *slice, const uint8_t *data, uint32_t size) {
    if (slice->skip >= size) {
        slice->skip -= size;
        return;
    }
    data += slice->skip;
    size -= slice->skip;
    slice->skip = 0;
    if (size > (uint32_t)slice->capacity - slice->written)
        size = slice->capacity - slice->written;
    for (uint32_t i = 0; i < size; ++i)
        slice->output[slice->written + i] = data[i];
    slice->written = (uint16_t)(slice->written + size);
}

static void emit_record(slice_t *slice, const dbt_sdp_record_t *record, const element_t *ranges) {
    uint8_t header[8];
    uint8_t count = sequence(header, attribute_bytes(record, ranges));
    emit(slice, header, count);
    if (selected(ranges, 0)) {
        header[0] = 9;
        header[1] = header[2] = 0;
        header[3] = 10;
        put32(header + 4, record->handle);
        emit(slice, header, 8);
    }
    for (unsigned i = 0; i < record->count; ++i) {
        const dbt_sdp_attribute_t *attr = &record->attributes[i];
        if (!attr->value[0] || !selected(ranges, attr->id))
            continue;
        header[0] = 9;
        put16(header + 1, attr->id);
        emit(slice, header, 3);
        emit(slice, attr->value, attr->size);
    }
}

static void response_header(uint8_t *output, uint8_t kind, const uint8_t *request, uint16_t size) {
    output[0] = kind;
    output[1] = request[1];
    output[2] = request[2];
    put16(output + 3, (uint16_t)(size - 5u));
}
static dbt_result_t error_response(const uint8_t *request, uint8_t *output, uint16_t error,
                                   uint16_t *written) {
    *written = 7;
    if (output) {
        response_header(output, 1, request, 7);
        put16(output + 5, error);
    }
    return DBT_OK;
}

dbt_result_t dbt_sdp_respond(const dbt_sdp_t *server, uint32_t binding, const uint8_t *request,
                             uint16_t size, uint8_t *output, uint16_t capacity, uint16_t *written) {
    if (written)
        *written = 0;
    if (!server || !server->database_state || !binding || !request || size < 5 || !written)
        return DBT_INVALID;
    if (capacity < 7)
        return DBT_NO_SPACE;
    if (output && ((uintptr_t)output - (uintptr_t)request < size ||
                   (uintptr_t)request - (uintptr_t)output < capacity))
        return DBT_INVALID;
    query_t query = {0};
    uint16_t error = parse(server, binding, request, size, &query);
    if (error)
        return error_response(request, output, error, written);
    const dbt_sdp_record_t *single = 0;
    uint32_t total = 0;
    uint16_t matches_count = 0;
    uint8_t header[5];
    for (unsigned i = 0; i < server->count; ++i) {
        const dbt_sdp_record_t *record = &server->records[i];
        if (query.kind == 4) {
            if (record->handle == query.handle)
                single = record;
        } else if (matches(record, &query.pattern)) {
            if (query.kind == 2) {
                if (matches_count < query.maximum)
                    ++matches_count;
            } else {
                uint32_t bytes = attribute_bytes(record, &query.attributes);
                total += bytes + sequence(header, bytes);
            }
        }
    }
    if (query.kind == 4) {
        if (!single)
            return error_response(request, output, 2, written);
        total = attribute_bytes(single, &query.attributes);
    }
    if (query.kind == 2)
        total = matches_count;
    else
        total += sequence(header, total);
    if (query.offset && query.offset >= total)
        return error_response(request, output, 5, written);
    uint32_t remaining = total - query.offset;
    if (query.kind != 2 && remaining < 2)
        return error_response(request, output, 5, written);
    uint16_t count;
    bool more;
    uint16_t prefix = query.kind == 2 ? 9 : 7;
    if (capacity <= prefix)
        return error_response(request, output, 6, written);
    uint32_t complete_capacity = (uint32_t)capacity - prefix - 1u;
    if (query.kind == 2)
        complete_capacity /= 4;
    else if (complete_capacity > query.maximum)
        complete_capacity = query.maximum;
    more = remaining > complete_capacity;
    if (more) {
        if (capacity <= prefix + 17u)
            return error_response(request, output, 6, written);
        uint32_t part_capacity = (uint32_t)capacity - prefix - 17u;
        if (query.kind == 2)
            part_capacity /= 4;
        else if (part_capacity > query.maximum)
            part_capacity = query.maximum;
        count = (uint16_t)part_capacity;
        /* AttributeListByteCount has a minimum of two, including the tail. */
        if (query.kind != 2 && remaining - count == 1)
            --count;
        if (count < (query.kind == 2 ? 1 : 2))
            return error_response(request, output, 6, written);
    } else
        count = (uint16_t)remaining;
    uint16_t payload = query.kind == 2 ? (uint16_t)(count * 4u) : count;
    *written = (uint16_t)(prefix + payload + (more ? 17u : 1u));
    if (!output)
        return DBT_OK;
    response_header(output, (uint8_t)(query.kind + 1), request, *written);
    if (query.kind == 2) {
        put16(output + 5, matches_count);
        put16(output + 7, count);
        unsigned matched = 0, emitted = 0;
        for (unsigned i = 0; i < server->count && emitted < count; ++i)
            if (matches(&server->records[i], &query.pattern) && matched++ >= query.offset) {
                put32(output + prefix + emitted * 4u, server->records[i].handle);
                ++emitted;
            }
    } else {
        put16(output + 5, count);
        slice_t slice = {output + prefix, query.offset, count, 0};
        if (query.kind == 4)
            emit_record(&slice, single, &query.attributes);
        else {
            uint32_t body = 0;
            for (unsigned i = 0; i < server->count; ++i)
                if (matches(&server->records[i], &query.pattern)) {
                    uint32_t bytes = attribute_bytes(&server->records[i], &query.attributes);
                    body += bytes + sequence(header, bytes);
                }
            uint8_t length = sequence(header, body);
            emit(&slice, header, length);
            for (unsigned i = 0; i < server->count; ++i)
                if (matches(&server->records[i], &query.pattern))
                    emit_record(&slice, &server->records[i], &query.attributes);
        }
    }
    uint8_t *token = output + prefix + payload;
    token[0] = more ? 16 : 0;
    if (more) {
        put32(token + 1, server->database_state);
        put32(token + 5, binding);
        put32(token + 9, query.fingerprint);
        put32(token + 13, query.offset + count);
    }
    return DBT_OK;
}
