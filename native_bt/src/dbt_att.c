#include "dbt_att.h"

static void zero(void *data, unsigned size) {
    uint8_t *p = data;
    for (unsigned i = 0; i < size; ++i)
        p[i] = 0;
}
static void copy(uint8_t *out, const uint8_t *in, unsigned size) {
    for (unsigned i = 0; i < size; ++i)
        out[i] = in[i];
}
static bool equal(const uint8_t *a, const uint8_t *b, unsigned size) {
    for (unsigned i = 0; i < size; ++i)
        if (a[i] != b[i])
            return false;
    return true;
}
static void put16(uint8_t *p, uint16_t value) {
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
}
static uint16_t minimum(uint16_t a, uint16_t b) { return a < b ? a : b; }
static bool due(uint32_t now, uint32_t deadline) {
    return (uint32_t)(now - deadline) < 0x80000000u;
}
static const uint8_t base_uuid[16] = {0xfb, 0x34, 0x9b, 0x5f, 0x80, 0, 0, 0x80,
                                      0,    0x10, 0,    0,    0,    0, 0, 0};
static bool uuid_equal(const uint8_t *a, uint8_t as, const uint8_t *b, uint8_t bs) {
    if (as == bs)
        return equal(a, b, as);
    const uint8_t *wide = as == 16 ? a : b, *small = as == 2 ? a : b;
    return equal(wide, base_uuid, 12) && !wide[14] && !wide[15] && wide[12] == small[0] &&
           wide[13] == small[1];
}
static bool type(const dbt_attribute_t *attribute, uint16_t value) {
    if (!attribute)
        return false;
    uint8_t short_uuid[2] = {(uint8_t)value, (uint8_t)(value >> 8)};
    return uuid_equal(attribute->uuid, attribute->uuid_size, short_uuid, 2);
}
static const dbt_attribute_t *find(const dbt_att_server_t *server, uint16_t handle) {
    unsigned low = 0, high = server->attribute_count;
    while (low < high) {
        unsigned middle = low + (high - low) / 2u;
        uint16_t candidate = server->attributes[middle].handle;
        if (candidate == handle)
            return &server->attributes[middle];
        if (candidate < handle)
            low = middle + 1u;
        else
            high = middle;
    }
    return 0;
}
static int cccd_index(const dbt_att_server_t *server, uint16_t handle) {
    unsigned index = 0;
    for (unsigned i = 0; i < server->attribute_count; ++i)
        if (type(&server->attributes[i], 0x2902)) {
            if (server->attributes[i].handle == handle)
                return (int)index;
            ++index;
        }
    return -1;
}
static uint16_t cccd_allowed(const dbt_att_server_t *server, const dbt_attribute_t *attribute) {
    const dbt_attribute_t *value = find(server, attribute->cccd_for);
    return value ? (uint16_t)(((value->flags & DBT_ATT_NOTIFY) ? 1u : 0u) |
                              ((value->flags & DBT_ATT_INDICATE) ? 2u : 0u))
                 : 0;
}
static dbt_att_connection_t *connection(dbt_att_server_t *server, uint16_t handle,
                                        uint32_t generation) {
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i) {
        dbt_att_connection_t *c = &server->connections[i];
        if (c->used && c->handle == handle && c->generation == generation)
            return c;
    }
    return 0;
}
static void free_buffer(dbt_att_server_t *server, dbt_att_connection_t *c, bool notice) {
    uint8_t **pointer = notice ? &c->notice : &c->response;
    uint8_t *small = notice ? c->short_notice : c->short_response;
    if (*pointer && *pointer != small)
        server->hooks.release(server->hooks.context, *pointer);
    *pointer = 0;
}
static void free_prepared(dbt_att_server_t *server, dbt_att_connection_t *c) {
    while (c->prepared) {
        dbt_att_prepared_t *next = c->prepared->next;
        server->hooks.release(server->hooks.context, (uint8_t *)c->prepared);
        c->prepared = next;
    }
    c->prepared_bytes = 0;
    c->prepared_count = 0;
}
static bool response_buffer(dbt_att_server_t *server, dbt_att_connection_t *c, uint16_t size) {
    c->response = size <= sizeof(c->short_response)
                      ? c->short_response
                      : server->hooks.allocate(server->hooks.context, size);
    return c->response != 0;
}
static void error_response(dbt_att_server_t *server, dbt_att_connection_t *c, uint8_t opcode,
                           uint16_t handle, uint8_t error) {
    free_buffer(server, c, false);
    c->response = c->short_response;
    c->response[0] = 1;
    c->response[1] = opcode;
    put16(c->response + 2, handle);
    c->response[4] = error;
    c->response_size = 5;
    c->response_ready = true;
    c->pending_mtu = 0;
}
static void ready_response(dbt_att_connection_t *c, uint16_t size) {
    c->response_size = size;
    c->response_ready = true;
}
static uint8_t secure_access(dbt_att_server_t *server, dbt_att_connection_t *c,
                             const dbt_link_t *link, const dbt_attribute_t *attribute, bool write) {
    bool authenticated =
        (attribute->flags & (write ? DBT_ATT_WRITE_AUTHENTICATE : DBT_ATT_READ_AUTHENTICATE)) != 0;
    bool encrypted =
        authenticated ||
        (attribute->flags & (write ? DBT_ATT_WRITE_ENCRYPT : DBT_ATT_READ_ENCRYPT)) != 0 ||
        attribute->minimum_key_size != 0;
    if (authenticated && (!link->encrypted || !c->authenticated))
        return DBT_ATT_AUTHENTICATION;
    if (encrypted && !link->encrypted)
        return DBT_ATT_ENCRYPTION;
    uint8_t required_key_size = attribute->minimum_key_size ? attribute->minimum_key_size : 7;
    if (encrypted && c->key_size < required_key_size)
        return DBT_ATT_KEY_SIZE;
    if (attribute->flags & (write ? DBT_ATT_WRITE_AUTHORIZE : DBT_ATT_READ_AUTHORIZE)) {
        if (!server->hooks.authorize)
            return DBT_ATT_AUTHORIZATION;
        server->in_callback = true;
        uint8_t result = server->hooks.authorize(server->hooks.context, link, attribute, write);
        server->in_callback = false;
        if (result)
            return result;
    }
    return 0;
}
static uint8_t permission(dbt_att_server_t *server, dbt_att_connection_t *c, const dbt_link_t *link,
                          const dbt_attribute_t *attribute, bool write) {
    if (!attribute)
        return DBT_ATT_INVALID_HANDLE;
    if (!(attribute->flags & (write ? DBT_ATT_WRITE : DBT_ATT_READ)))
        return write ? DBT_ATT_WRITE_DENIED : DBT_ATT_READ_DENIED;
    return secure_access(server, c, link, attribute, write);
}
static uint8_t read_value(dbt_att_server_t *server, dbt_att_connection_t *c, const dbt_link_t *link,
                          const dbt_attribute_t *attribute, uint16_t offset, uint8_t *output,
                          uint16_t capacity, uint16_t *written, uint16_t *total) {
    uint8_t error = permission(server, c, link, attribute, false);
    if (error)
        return error;
    int index = type(attribute, 0x2902) ? cccd_index(server, attribute->handle) : -1;
    if (index >= 0) {
        uint8_t value[2];
        put16(value, c->cccds[index]);
        *total = 2;
        if (offset > 2)
            return DBT_ATT_INVALID_OFFSET;
        *written = minimum(capacity, (uint16_t)(2u - offset));
        copy(output, value + offset, *written);
        return 0;
    }
    if (attribute->flags & DBT_ATT_DYNAMIC) {
        if (!server->hooks.read)
            return DBT_ATT_READ_DENIED;
        *written = *total = 0;
        server->in_callback = true;
        error = server->hooks.read(server->hooks.context, link, attribute, offset, output, capacity,
                                   written, total);
        server->in_callback = false;
        if (error)
            return error;
        if (*total > attribute->maximum_size || offset > *total ||
            *written != minimum(capacity, (uint16_t)(*total - offset)))
            return DBT_ATT_UNLIKELY;
        return 0;
    }
    *total = attribute->value_size;
    if (offset > *total)
        return DBT_ATT_INVALID_OFFSET;
    *written = minimum(capacity, (uint16_t)(*total - offset));
    if (*written)
        copy(output, attribute->value + offset, *written);
    return 0;
}
static uint16_t group_end(const dbt_att_server_t *server, unsigned index) {
    const dbt_attribute_t *a = &server->attributes[index];
    if (!type(a, 0x2800) && !type(a, 0x2801))
        return a->handle;
    for (unsigned i = index + 1; i < server->attribute_count; ++i)
        if (type(&server->attributes[i], 0x2800) || type(&server->attributes[i], 0x2801))
            return server->attributes[i - 1u].handle;
    return server->attributes[server->attribute_count - 1u].handle;
}

dbt_result_t dbt_att_init(dbt_att_server_t *server, const dbt_attribute_t *attributes,
                          uint16_t count, dbt_att_hooks_t hooks, uint16_t mtu,
                          uint16_t prepare_budget) {
    if (!server || !attributes || !count || !hooks.allocate || !hooks.release || mtu < 23 ||
        mtu > DBT_ATT_MAX_MTU)
        return DBT_INVALID;
    uint8_t cccds = 0;
    uint16_t previous = 0;
    for (unsigned i = 0; i < count; ++i) {
        const dbt_attribute_t *a = &attributes[i];
        if (!a->handle || a->handle <= previous || !a->uuid ||
            (a->uuid_size != 2 && a->uuid_size != 16) || a->maximum_size > DBT_ATT_MAX_VALUE ||
            a->value_size > a->maximum_size ||
            (a->value_size && !a->value && !(a->flags & DBT_ATT_DYNAMIC) && !type(a, 0x2902)) ||
            (a->minimum_key_size && (a->minimum_key_size < 7 || a->minimum_key_size > 16)))
            return DBT_INVALID;
        if (type(a, 0x2800) || type(a, 0x2801))
            if ((a->flags & DBT_ATT_DYNAMIC) || (a->value_size != 2 && a->value_size != 16))
                return DBT_INVALID;
        if (type(a, 0x2902)) {
            const dbt_attribute_t *value = 0;
            for (unsigned j = 0; j < i; ++j)
                if (attributes[j].handle == a->cccd_for)
                    value = &attributes[j];
            if (!value || !(value->flags & (DBT_ATT_NOTIFY | DBT_ATT_INDICATE)) ||
                a->maximum_size != 2 ||
                (a->flags & (DBT_ATT_READ | DBT_ATT_WRITE)) != (DBT_ATT_READ | DBT_ATT_WRITE) ||
                ++cccds > DBT_ATT_CCCD_LIMIT)
                return DBT_INVALID;
            for (unsigned j = 0; j < i; ++j)
                if (type(&attributes[j], 0x2902) && attributes[j].cccd_for == a->cccd_for)
                    return DBT_INVALID;
        } else if ((a->flags & DBT_ATT_WRITE) && !(a->flags & DBT_ATT_DYNAMIC))
            return DBT_INVALID;
        previous = a->handle;
    }
    zero(server, sizeof(*server));
    server->attributes = attributes;
    server->attribute_count = count;
    server->hooks = hooks;
    server->mtu = mtu;
    server->prepare_budget = prepare_budget;
    server->cccd_count = cccds;
    return DBT_OK;
}
dbt_result_t dbt_att_open(dbt_att_server_t *server, const dbt_link_t *link) {
    if (!server || server->in_callback || !link || !link->used || !link->generation)
        return DBT_INVALID;
    if (link->kind != DBT_LINK_LE)
        return DBT_UNSUPPORTED;
    if (connection(server, link->handle, link->generation))
        return DBT_BUSY;
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i)
        if (server->connections[i].used && server->connections[i].handle == link->handle)
            return DBT_BUSY;
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i)
        if (!server->connections[i].used) {
            dbt_att_connection_t *c = &server->connections[i];
            zero(c, sizeof(*c));
            c->used = true;
            c->handle = link->handle;
            c->generation = link->generation;
            c->mtu = 23;
            return DBT_OK;
        }
    return DBT_NO_SPACE;
}
void dbt_att_close(dbt_att_server_t *server, uint16_t handle, uint32_t generation) {
    if (!server || server->in_callback)
        return;
    dbt_att_connection_t *c = connection(server, handle, generation);
    if (!c)
        return;
    free_prepared(server, c);
    free_buffer(server, c, false);
    free_buffer(server, c, true);
    zero(c, sizeof(*c));
}
void dbt_att_clear(dbt_att_server_t *server) {
    if (!server || server->in_callback)
        return;
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i)
        if (server->connections[i].used)
            dbt_att_close(server, server->connections[i].handle, server->connections[i].generation);
}
dbt_result_t dbt_att_security(dbt_att_server_t *server, const dbt_link_t *link, bool authenticated,
                              uint8_t key_size, bool bonded) {
    if (!server || server->in_callback || !link || !link->used || key_size > 16 ||
        (link->encrypted && key_size < 7) || (!link->encrypted && (authenticated || key_size)))
        return DBT_INVALID;
    dbt_att_connection_t *c = connection(server, link->handle, link->generation);
    if (!c)
        return DBT_INVALID;
    if (c->deferred)
        return DBT_BUSY;
    uint16_t values[DBT_ATT_CCCD_LIMIT] = {0};
    if (bonded && !c->bonded) {
        unsigned index = 0;
        for (unsigned i = 0; i < server->attribute_count; ++i) {
            const dbt_attribute_t *a = &server->attributes[i];
            if (!type(a, 0x2902))
                continue;
            if (server->hooks.load_cccd) {
                server->in_callback = true;
                dbt_result_t result =
                    server->hooks.load_cccd(server->hooks.context, link, a->handle, &values[index]);
                server->in_callback = false;
                if (result != DBT_OK && result != DBT_EMPTY)
                    return result;
                if (result == DBT_EMPTY)
                    values[index] = 0;
                if (values[index] & ~cccd_allowed(server, a))
                    return DBT_INVALID;
            }
            ++index;
        }
        copy((uint8_t *)c->cccds, (const uint8_t *)values, sizeof(values));
    } else if (c->bonded && !bonded)
        zero(c->cccds, sizeof(c->cccds));
    c->authenticated = authenticated;
    c->key_size = key_size;
    c->bonded = bonded;
    return DBT_OK;
}
static uint8_t stage_cccd(dbt_att_server_t *server, dbt_att_connection_t *c,
                          const dbt_att_write_t *writes, uint8_t count,
                          dbt_att_cccd_t updates[DBT_ATT_CCCD_LIMIT], uint8_t *update_count,
                          uint16_t *bad_handle, bool queued) {
    copy((uint8_t *)c->staged_cccds, (const uint8_t *)c->cccds, sizeof(c->cccds));
    uint16_t changed = 0;
    for (unsigned i = 0; i < count; ++i) {
        const dbt_attribute_t *a = find(server, writes[i].handle);
        if (!type(a, 0x2902))
            continue;
        int index = cccd_index(server, a->handle);
        uint16_t offset = writes[i].offset, size = writes[i].size;
        if (offset > 2 || size > 2u - offset || (!queued && (offset || size != 2))) {
            *bad_handle = a->handle;
            return DBT_ATT_INVALID_LENGTH;
        }
        uint8_t value[2];
        put16(value, c->staged_cccds[index]);
        copy(value + offset, writes[i].data, size);
        c->staged_cccds[index] = dbt_read_le16(value);
        changed |= (uint16_t)(1u << index);
    }
    *update_count = 0;
    unsigned index = 0;
    for (unsigned i = 0; i < server->attribute_count; ++i) {
        const dbt_attribute_t *a = &server->attributes[i];
        if (!type(a, 0x2902))
            continue;
        if (changed & (1u << index)) {
            if (c->staged_cccds[index] & ~cccd_allowed(server, a)) {
                *bad_handle = a->handle;
                return DBT_ATT_INVALID_LENGTH;
            }
            updates[*update_count] = (dbt_att_cccd_t){a->handle, c->staged_cccds[index]};
            ++*update_count;
        }
        ++index;
    }
    return 0;
}
static void write_finished(dbt_att_server_t *server, dbt_att_connection_t *c, uint8_t error,
                           uint16_t handle) {
    c->deferred = false;
    if (error) {
        error_response(server, c, c->pending_opcode, handle, error);
        return;
    }
    copy((uint8_t *)c->cccds, (const uint8_t *)c->staged_cccds, sizeof(c->cccds));
    c->response = c->short_response;
    c->response[0] = c->pending_opcode == 0x18 ? 0x19 : 0x13;
    ready_response(c, 1);
}
static void write_batch(dbt_att_server_t *server, dbt_att_connection_t *c, const dbt_link_t *link,
                        const dbt_att_write_t *writes, uint8_t count, bool command, bool queued) {
    dbt_att_cccd_t updates[DBT_ATT_CCCD_LIMIT];
    uint8_t update_count = 0;
    uint16_t bad_handle = count ? writes[0].handle : 0;
    uint8_t error = 0;
    for (unsigned i = 0; i < count; ++i) {
        const dbt_attribute_t *a = find(server, writes[i].handle);
        error = permission(server, c, link, a, true);
        if (!error && ((uint32_t)writes[i].offset + writes[i].size > a->maximum_size))
            error = DBT_ATT_INVALID_LENGTH;
        if (error) {
            bad_handle = writes[i].handle;
            break;
        }
    }
    if (!error)
        error = stage_cccd(server, c, writes, count, updates, &update_count, &bad_handle, queued);
    if (!error && c->bonded && update_count && !server->hooks.write)
        error = DBT_ATT_UNLIKELY;
    bool external = false;
    for (unsigned i = 0; i < count; ++i)
        if (!type(find(server, writes[i].handle), 0x2902))
            external = true;
    if (!error && external && !server->hooks.write)
        error = DBT_ATT_WRITE_DENIED;
    if (!error && server->token == UINT32_MAX)
        error = DBT_ATT_RESOURCES;
    if (error) {
        if (command)
            ++server->ignored_commands;
        else
            error_response(server, c, c->pending_opcode, bad_handle, error);
        return;
    }
    c->token = ++server->token;
    dbt_att_batch_t batch = {writes, updates, count, update_count, command, queued};
    uint16_t result = 0;
    if (server->hooks.write && count) {
        server->in_callback = true;
        result = server->hooks.write(server->hooks.context, link, &batch, c->token, &bad_handle);
        server->in_callback = false;
    }
    if (command) {
        if (result) {
            ++server->ignored_commands;
            return;
        }
        copy((uint8_t *)c->cccds, (const uint8_t *)c->staged_cccds, sizeof(c->cccds));
        return;
    }
    if (result == DBT_ATT_DEFERRED) {
        c->deferred = true;
        c->error_handle = bad_handle;
        return;
    }
    write_finished(server, c, result <= 255 ? (uint8_t)result : DBT_ATT_UNLIKELY, bad_handle);
}
dbt_result_t dbt_att_complete(dbt_att_server_t *server, uint16_t handle, uint32_t generation,
                              uint32_t token, uint8_t error, uint16_t error_handle) {
    if (!server || server->in_callback)
        return DBT_INVALID;
    dbt_att_connection_t *c = connection(server, handle, generation);
    if (!c || !c->deferred || c->failed || c->token != token)
        return DBT_INVALID;
    write_finished(server, c, error, error ? error_handle : 0);
    return DBT_OK;
}
static void discovery(dbt_att_server_t *server, dbt_att_connection_t *c, const dbt_link_t *link,
                      const uint8_t *data, uint16_t size) {
    uint8_t opcode = data[0];
    uint16_t start = dbt_read_le16(data + 1), end = dbt_read_le16(data + 3);
    if (!start || start > end) {
        error_response(server, c, opcode, start, DBT_ATT_INVALID_HANDLE);
        return;
    }
    if (opcode == 0x10) {
        uint8_t primary[2] = {0, 0x28}, secondary[2] = {1, 0x28};
        if (!uuid_equal(data + 5, (uint8_t)(size - 5u), primary, 2) &&
            !uuid_equal(data + 5, (uint8_t)(size - 5u), secondary, 2)) {
            error_response(server, c, opcode, start, DBT_ATT_GROUP_TYPE);
            return;
        }
    }
    if (!response_buffer(server, c, c->mtu)) {
        error_response(server, c, opcode, 0, DBT_ATT_RESOURCES);
        return;
    }
    c->response[0] = (uint8_t)(opcode + 1u);
    uint16_t used = opcode == 6 ? 1 : 2, first_total = 0;
    uint8_t element = 0;
    for (unsigned i = 0; i < server->attribute_count; ++i) {
        const dbt_attribute_t *a = &server->attributes[i];
        if (a->handle < start)
            continue;
        if (a->handle > end)
            break;
        if (opcode == 4) {
            uint8_t length = (uint8_t)(2u + a->uuid_size);
            if (element && element != length)
                break;
            if (used + length > c->mtu)
                break;
            element = length;
            c->response[1] = a->uuid_size == 2 ? 1 : 2;
            put16(c->response + used, a->handle);
            copy(c->response + used + 2u, a->uuid, a->uuid_size);
            used += length;
            continue;
        }
        if (opcode == 6) {
            uint8_t wanted[2] = {data[5], data[6]};
            if (!uuid_equal(a->uuid, a->uuid_size, wanted, 2))
                continue;
            uint8_t value[DBT_ATT_MAX_VALUE];
            uint16_t written = 0, total = 0;
            uint8_t error =
                read_value(server, c, link, a, 0, value, sizeof(value), &written, &total);
            if (error) {
                if (used == 1)
                    error_response(server, c, opcode, a->handle, error);
                break;
            }
            bool same = total == size - 7u && equal(value, data + 7, total);
            if ((type(a, 0x2800) || type(a, 0x2801)) && (total == 2 || total == 16) &&
                (size - 7u == 2 || size - 7u == 16))
                same = uuid_equal(value, (uint8_t)total, data + 7, (uint8_t)(size - 7u));
            if (!same)
                continue;
            if (used + 4u > c->mtu)
                break;
            put16(c->response + used, a->handle);
            put16(c->response + used + 2u, group_end(server, i));
            used += 4;
            continue;
        }
        if (!uuid_equal(a->uuid, a->uuid_size, data + 5, (uint8_t)(size - 5u)))
            continue;
        uint16_t overhead = opcode == 0x10 ? 4 : 2,
                 cap = minimum((uint16_t)(c->mtu - 2u - overhead), (uint16_t)(255u - overhead));
        uint8_t value[253];
        uint16_t written = 0, total = 0;
        uint8_t error = read_value(server, c, link, a, 0, value, cap, &written, &total);
        if (error) {
            if (used == 2)
                error_response(server, c, opcode, a->handle, error);
            break;
        }
        uint8_t length = (uint8_t)(overhead + written);
        if (element && (element != length || first_total != total))
            break;
        if (used + length > c->mtu)
            break;
        element = length;
        first_total = total;
        c->response[1] = length;
        put16(c->response + used, a->handle);
        if (overhead == 4)
            put16(c->response + used + 2u, group_end(server, i));
        copy(c->response + used + overhead, value, written);
        used += length;
    }
    if (c->response_ready)
        return;
    if (used == (opcode == 6 ? 1 : 2)) {
        error_response(server, c, opcode, start, DBT_ATT_NOT_FOUND);
        return;
    }
    ready_response(c, used);
}
static void read_request(dbt_att_server_t *server, dbt_att_connection_t *c, const dbt_link_t *link,
                         const uint8_t *data, uint16_t size) {
    uint8_t opcode = data[0];
    if (opcode == 0x0a || opcode == 0x0c) {
        uint16_t handle = dbt_read_le16(data + 1),
                 offset = opcode == 0x0c ? dbt_read_le16(data + 3) : 0;
        const dbt_attribute_t *a = find(server, handle);
        if (!a) {
            error_response(server, c, opcode, handle, DBT_ATT_INVALID_HANDLE);
            return;
        }
        uint16_t bound = a->flags & DBT_ATT_DYNAMIC ? a->maximum_size : a->value_size;
        if (type(a, 0x2902))
            bound = 2;
        uint16_t capacity =
            offset > bound ? 0 : minimum((uint16_t)(c->mtu - 1u), (uint16_t)(bound - offset));
        if (!response_buffer(server, c, (uint16_t)(capacity + 1u))) {
            error_response(server, c, opcode, 0, DBT_ATT_RESOURCES);
            return;
        }
        c->response[0] = (uint8_t)(opcode + 1u);
        uint16_t written = 0, total = 0;
        uint8_t error =
            read_value(server, c, link, a, offset, c->response + 1, capacity, &written, &total);
        if (!error && opcode == 0x0c && total <= c->mtu - 1u)
            error = DBT_ATT_NOT_LONG;
        if (error) {
            error_response(server, c, opcode, handle, error);
            return;
        }
        ready_response(c, (uint16_t)(written + 1u));
        return;
    }
    uint32_t planned = 1;
    for (unsigned position = 1; position < size; position += 2) {
        const dbt_attribute_t *a = find(server, dbt_read_le16(data + position));
        if (a)
            planned += type(a, 0x2902)              ? 2u
                       : a->flags & DBT_ATT_DYNAMIC ? a->maximum_size
                                                    : a->value_size;
        if (opcode == 0x20)
            planned += 2;
    }
    uint16_t capacity_bound = planned > c->mtu ? c->mtu : (uint16_t)planned;
    if (!response_buffer(server, c, capacity_bound)) {
        error_response(server, c, opcode, 0, DBT_ATT_RESOURCES);
        return;
    }
    c->response[0] = (uint8_t)(opcode + 1u);
    uint16_t used = 1;
    for (unsigned position = 1; position < size; position += 2) {
        uint16_t handle = dbt_read_le16(data + position), written = 0, total = 0;
        const dbt_attribute_t *a = find(server, handle);
        uint16_t overhead = opcode == 0x20 ? 2 : 0,
                 capacity = used + overhead <= capacity_bound
                                ? (uint16_t)(capacity_bound - used - overhead)
                                : 0;
        uint8_t scratch[1];
        uint8_t *out = capacity ? c->response + used + overhead : scratch;
        uint8_t error = read_value(server, c, link, a, 0, out, capacity, &written, &total);
        if (error) {
            error_response(server, c, opcode, handle, error);
            return;
        }
        if (opcode == 0x20) {
            if (used + 2u > c->mtu)
                continue;
            put16(c->response + used, written);
            used += 2;
        }
        used += written;
    }
    ready_response(c, used);
}
static void prepare(dbt_att_server_t *server, dbt_att_connection_t *c, const dbt_link_t *link,
                    const uint8_t *data, uint16_t size) {
    uint16_t handle = dbt_read_le16(data + 1), offset = dbt_read_le16(data + 3),
             bytes = (uint16_t)(size - 5u);
    const dbt_attribute_t *a = find(server, handle);
    uint8_t error = permission(server, c, link, a, true);
    if (!error && offset > a->maximum_size)
        error = DBT_ATT_INVALID_OFFSET;
    if (!error && (uint32_t)offset + bytes > a->maximum_size)
        error = DBT_ATT_INVALID_LENGTH;
    if (!error && (c->prepared_count == DBT_ATT_PREPARE_LIMIT ||
                   (uint32_t)c->prepared_bytes + bytes > server->prepare_budget))
        error = DBT_ATT_PREPARE_FULL;
    if (error) {
        error_response(server, c, 0x16, handle, error);
        return;
    }
    if (!response_buffer(server, c, size)) {
        error_response(server, c, 0x16, 0, DBT_ATT_RESOURCES);
        return;
    }
    dbt_att_prepared_t *node = (dbt_att_prepared_t *)server->hooks.allocate(
        server->hooks.context, (uint16_t)(sizeof(*node) + bytes));
    if (!node) {
        error_response(server, c, 0x16, handle, DBT_ATT_PREPARE_FULL);
        return;
    }
    node->next = 0;
    node->handle = handle;
    node->offset = offset;
    node->size = bytes;
    copy(node->data, data + 5, bytes);
    dbt_att_prepared_t **tail = &c->prepared;
    while (*tail)
        tail = &(*tail)->next;
    *tail = node;
    ++c->prepared_count;
    c->prepared_bytes += bytes;
    copy(c->response, data, size);
    c->response[0] = 0x17;
    ready_response(c, size);
}
dbt_result_t dbt_att_receive(dbt_att_server_t *server, const dbt_link_t *link, const uint8_t *data,
                             uint16_t size, uint32_t now_ms) {
    if (!server || server->in_callback || !link || !link->used || !data || !size)
        return DBT_INVALID;
    dbt_att_connection_t *c = connection(server, link->handle, link->generation);
    if (!c)
        return DBT_INVALID;
    if (c->failed)
        return DBT_BUSY;
    uint8_t opcode = data[0];
    bool command = (opcode & 0x40u) != 0;
    if (command) {
        if (opcode != 0x52 || size < 3 || size > c->mtu || c->deferred) {
            ++server->ignored_commands;
            return DBT_OK;
        }
        dbt_att_write_t write = {dbt_read_le16(data + 1), 0, (uint16_t)(size - 3u), data + 3};
        write_batch(server, c, link, &write, 1, true, false);
        return DBT_OK;
    }
    if (opcode == 0x1e) {
        if (size != 1)
            return DBT_INVALID;
        if (c->indication_wait) {
            uint16_t attribute = c->indication_handle;
            c->indication_wait = false;
            c->indication_handle = 0;
            if (server->hooks.confirmed) {
                server->in_callback = true;
                server->hooks.confirmed(server->hooks.context, c->handle, c->generation, attribute);
                server->in_callback = false;
            }
        }
        return DBT_OK;
    }
    /* Odd methods are responses/server messages, never request/error loops. */
    if (opcode & 1u)
        return DBT_UNSUPPORTED;
    if (c->request_busy)
        return DBT_BUSY;
    c->request_busy = true;
    c->request_deadline = now_ms + 30000u;
    c->pending_opcode = opcode;
    bool valid = size <= c->mtu && !(opcode & 0x80u);
    switch (opcode) {
    case 2:
        valid = valid && size == 3;
        break;
    case 4:
        valid = valid && size == 5;
        break;
    case 6:
        valid = valid && size >= 7;
        break;
    case 8:
    case 0x10:
        valid = valid && (size == 7 || size == 21);
        break;
    case 0x0a:
        valid = valid && size == 3;
        break;
    case 0x0c:
        valid = valid && size == 5;
        break;
    case 0x0e:
    case 0x20:
        valid = valid && size >= 5 && (size & 1u);
        break;
    case 0x12:
        valid = valid && size >= 3;
        break;
    case 0x16:
        valid = valid && size >= 5;
        break;
    case 0x18:
        valid = valid && size == 2 && data[1] <= 1;
        break;
    default:
        error_response(server, c, opcode, 0, DBT_ATT_NOT_SUPPORTED);
        return DBT_OK;
    }
    if (!valid) {
        error_response(server, c, opcode, 0, DBT_ATT_INVALID_PDU);
        return DBT_OK;
    }
    switch (opcode) {
    case 2: {
        uint16_t client = dbt_read_le16(data + 1);
        if (c->mtu_exchanged) {
            error_response(server, c, opcode, 0, DBT_ATT_NOT_SUPPORTED);
            break;
        }
        c->pending_mtu = minimum(client < 23 ? 23 : client, server->mtu);
        c->response = c->short_response;
        c->response[0] = 3;
        put16(c->response + 1, server->mtu);
        ready_response(c, 3);
        break;
    }
    case 4:
    case 6:
    case 8:
    case 0x10:
        discovery(server, c, link, data, size);
        break;
    case 0x0a:
    case 0x0c:
    case 0x0e:
    case 0x20:
        read_request(server, c, link, data, size);
        break;
    case 0x12: {
        dbt_att_write_t write = {dbt_read_le16(data + 1), 0, (uint16_t)(size - 3u), data + 3};
        write_batch(server, c, link, &write, 1, false, false);
        break;
    }
    case 0x16:
        prepare(server, c, link, data, size);
        break;
    case 0x18:
        if (data[1] == 0) {
            free_prepared(server, c);
            c->response = c->short_response;
            c->response[0] = 0x19;
            ready_response(c, 1);
        } else {
            dbt_att_write_t writes[DBT_ATT_PREPARE_LIMIT];
            uint8_t count = 0;
            for (dbt_att_prepared_t *p = c->prepared; p; p = p->next)
                writes[count++] = (dbt_att_write_t){p->handle, p->offset, p->size, p->data};
            write_batch(server, c, link, writes, count, false, true);
            free_prepared(server, c);
        }
        break;
    default:
        break;
    }
    return DBT_OK;
}
dbt_result_t dbt_att_notify(dbt_att_server_t *server, const dbt_link_t *link, uint16_t handle,
                            const uint8_t *data, uint16_t size, bool indication) {
    if (!server || server->in_callback || !link || !link->used || (size && !data))
        return DBT_INVALID;
    dbt_att_connection_t *c = connection(server, link->handle, link->generation);
    if (!c || c->failed)
        return DBT_BUSY;
    const dbt_attribute_t *a = find(server, handle);
    if (!a || !(a->flags & (indication ? DBT_ATT_INDICATE : DBT_ATT_NOTIFY)))
        return DBT_UNSUPPORTED;
    int index = -1;
    for (unsigned i = 0; i < server->attribute_count; ++i)
        if (type(&server->attributes[i], 0x2902) && server->attributes[i].cccd_for == handle)
            index = cccd_index(server, server->attributes[i].handle);
    if (index < 0 || !(c->cccds[index] & (indication ? 2 : 1)))
        return DBT_UNSUPPORTED;
    if (c->notice_ready || (indication && c->indication_wait))
        return DBT_BUSY;
    if (size > c->mtu - 3u || size > a->maximum_size)
        return DBT_NO_SPACE;
    /* Characteristic read security also protects unsolicited value disclosure. */
    uint8_t error = secure_access(server, c, link, a, false);
    if (error)
        return DBT_INVALID;
    uint16_t bytes = (uint16_t)(size + 3u);
    c->notice = bytes <= sizeof(c->short_notice)
                    ? c->short_notice
                    : server->hooks.allocate(server->hooks.context, bytes);
    if (!c->notice)
        return DBT_NO_SPACE;
    c->notice[0] = indication ? 0x1d : 0x1b;
    put16(c->notice + 1, handle);
    copy(c->notice + 3, data, size);
    c->notice_size = bytes;
    c->notice_ready = true;
    return DBT_OK;
}
dbt_result_t dbt_att_tx_peek(dbt_att_server_t *server, uint16_t handle, uint32_t generation,
                             dbt_att_view_t *view) {
    if (!server || server->in_callback || !view)
        return DBT_INVALID;
    dbt_att_connection_t *c = connection(server, handle, generation);
    if (!c || c->failed)
        return DBT_BUSY;
    if (!c->locked)
        c->locked = c->response_ready ? 1 : c->notice_ready ? 2 : 0;
    if (!c->locked)
        return DBT_EMPTY;
    view->data = c->locked == 1 ? c->response : c->notice;
    view->size = c->locked == 1 ? c->response_size : c->notice_size;
    return DBT_OK;
}
dbt_result_t dbt_att_tx_admitted(dbt_att_server_t *server, uint16_t handle, uint32_t generation,
                                 uint32_t now_ms) {
    if (!server || server->in_callback)
        return DBT_INVALID;
    dbt_att_connection_t *c = connection(server, handle, generation);
    if (!c || !c->locked)
        return DBT_INVALID;
    if (c->locked == 1) {
        if (c->pending_mtu) {
            c->mtu = c->pending_mtu;
            c->pending_mtu = 0;
            c->mtu_exchanged = true;
        }
        free_buffer(server, c, false);
        c->response_ready = false;
        c->request_busy = false;
    } else {
        if (c->notice[0] == 0x1d && !c->failed) {
            c->indication_wait = true;
            c->indication_deadline = now_ms + 30000u;
            c->indication_handle = dbt_read_le16(c->notice + 1);
        }
        free_buffer(server, c, true);
        c->notice_ready = false;
    }
    c->locked = 0;
    return DBT_OK;
}
void dbt_att_poll(dbt_att_server_t *server, uint32_t now_ms) {
    if (!server || server->in_callback)
        return;
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i) {
        dbt_att_connection_t *c = &server->connections[i];
        if (!c->used || c->failed)
            continue;
        if ((c->request_busy && due(now_ms, c->request_deadline)) ||
            (c->indication_wait && due(now_ms, c->indication_deadline))) {
            c->failed = true;
            c->deferred = false;
            free_prepared(server, c);
            if (server->hooks.failed) {
                server->in_callback = true;
                server->hooks.failed(server->hooks.context, c->handle, c->generation);
                server->in_callback = false;
            }
        }
    }
}
uint32_t dbt_att_wait_ms(const dbt_att_server_t *server, uint32_t now_ms) {
    if (!server)
        return UINT32_MAX;
    uint32_t wait = UINT32_MAX;
    for (unsigned i = 0; i < DBT_MAX_LINKS; ++i) {
        const dbt_att_connection_t *c = &server->connections[i];
        if (!c->used || c->failed)
            continue;
        if (c->request_busy) {
            uint32_t delta = due(now_ms, c->request_deadline) ? 0 : c->request_deadline - now_ms;
            if (delta < wait)
                wait = delta;
        }
        if (c->indication_wait) {
            uint32_t delta =
                due(now_ms, c->indication_deadline) ? 0 : c->indication_deadline - now_ms;
            if (delta < wait)
                wait = delta;
        }
    }
    return wait;
}
