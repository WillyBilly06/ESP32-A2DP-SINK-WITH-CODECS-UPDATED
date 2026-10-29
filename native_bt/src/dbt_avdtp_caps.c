#include "dbt_avdtp_caps.h"

dbt_result_t dbt_avdtp_cap_next(const uint8_t *data, uint16_t size, uint16_t *cursor,
                                dbt_avdtp_cap_t *capability) {
    if ((!data && size) || !cursor || !capability || *cursor > size) return DBT_INVALID;
    if (*cursor == size) return DBT_EMPTY;
    uint16_t left = (uint16_t)(size - *cursor);
    if (left < 2 || data[*cursor + 1u] > left - 2u) return DBT_INVALID;
    dbt_avdtp_cap_t value = {data + *cursor + 2u, data[*cursor], data[*cursor + 1u]};
    *cursor = (uint16_t)(*cursor + 2u + value.size);
    *capability = value;
    return DBT_OK;
}

static uint8_t format_error(const dbt_avdtp_cap_t *cap) {
    switch (cap->category) {
    case DBT_AVDTP_MEDIA_TRANSPORT: return cap->size ? 0x23 : 0;
    case DBT_AVDTP_REPORTING: case DBT_AVDTP_DELAY_REPORTING: return cap->size ? 0x18 : 0;
    case DBT_AVDTP_RECOVERY:
        if (cap->size != 3) return 0x25;
        if (cap->data[0] != 1) return 0x22;
        return !cap->data[1] || cap->data[1] > 24 || !cap->data[2] || cap->data[2] > 24 ? 0x25 : 0;
    case DBT_AVDTP_CONTENT_PROTECTION: return cap->size < 2 ? 0x27 : 0;
    case DBT_AVDTP_HEADER_COMPRESSION: return cap->size != 1 ? 0x26 : 0;
    case DBT_AVDTP_MULTIPLEXING: return cap->size < 3 || cap->size > 7 || !(cap->size & 1u) ? 0x28 : 0;
    case DBT_AVDTP_MEDIA_CODEC: return cap->size < 2 ? 0x18 : 0;
    default: return 0;
    }
}

dbt_result_t dbt_avdtp_caps_validate(const uint8_t *data, uint16_t size,
    dbt_avdtp_cap_mode_t mode, dbt_avdtp_caps_t *capabilities, dbt_avdtp_cap_error_t *error) {
    if (!capabilities || !error || (!data && size) || mode > DBT_AVDTP_RECONFIGURATION ||
        mode < DBT_AVDTP_CAPABILITIES) return DBT_INVALID;
    dbt_avdtp_caps_t result = {0};
    dbt_avdtp_cap_error_t failure = {0};
    uint16_t cursor = 0;
    while (cursor < size) {
        failure.category = data[cursor];
        dbt_avdtp_cap_t cap;
        if (dbt_avdtp_cap_next(data, size, &cursor, &cap) != DBT_OK) {
            failure.code = 0x11;
            goto bad;
        }
        if (cap.category < 1 || cap.category > 8) {
            if (mode == DBT_AVDTP_CAPABILITIES) continue;
            failure.code = 0x17;
            goto bad;
        }
        if (mode == DBT_AVDTP_RECONFIGURATION && cap.category != DBT_AVDTP_MEDIA_CODEC &&
            cap.category != DBT_AVDTP_CONTENT_PROTECTION) {
            failure.code = 0x1a;
            goto bad;
        }
        uint16_t mask = (uint16_t)(1u << cap.category);
        if ((result.present & mask) &&
            !(mode == DBT_AVDTP_CAPABILITIES && cap.category == DBT_AVDTP_CONTENT_PROTECTION)) {
            failure.code = 0x18;
            goto bad;
        }
        failure.code = format_error(&cap);
        if (failure.code) goto bad;
        result.present |= mask;
        if (cap.category == DBT_AVDTP_MEDIA_CODEC) result.codec = cap;
    }
    if ((mode == DBT_AVDTP_CONFIGURATION &&
         (result.present & 0x82u) != 0x82u) ||
        (mode == DBT_AVDTP_RECONFIGURATION && !result.present)) {
        failure = (dbt_avdtp_cap_error_t){0, 0x1a};
        goto bad;
    }
    /* Multiplexed media is first, followed by each exposed optional session. */
    if (result.present & (1u << DBT_AVDTP_MULTIPLEXING)) {
        cursor = 0;
        dbt_avdtp_cap_t cap;
        while (dbt_avdtp_cap_next(data, size, &cursor, &cap) == DBT_OK) {
            if (cap.category != DBT_AVDTP_MULTIPLEXING) continue;
            uint8_t expected = (uint8_t)(3u + ((result.present & 4u) ? 2u : 0u) +
                                                   ((result.present & 8u) ? 2u : 0u));
            if (cap.size != expected) {
                failure = (dbt_avdtp_cap_error_t){DBT_AVDTP_MULTIPLEXING, 0x28};
                goto bad;
            }
        }
    }
    *capabilities = result;
    *error = (dbt_avdtp_cap_error_t){0};
    return DBT_OK;
bad:
    *error = failure;
    return DBT_INVALID;
}
