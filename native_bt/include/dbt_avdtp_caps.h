#ifndef DBT_AVDTP_CAPS_H
#define DBT_AVDTP_CAPS_H

#include "dbt_avdtp.h"

enum {
    DBT_AVDTP_MEDIA_TRANSPORT = 1, DBT_AVDTP_REPORTING = 2,
    DBT_AVDTP_RECOVERY = 3, DBT_AVDTP_CONTENT_PROTECTION = 4,
    DBT_AVDTP_HEADER_COMPRESSION = 5, DBT_AVDTP_MULTIPLEXING = 6,
    DBT_AVDTP_MEDIA_CODEC = 7, DBT_AVDTP_DELAY_REPORTING = 8
};
typedef enum { DBT_AVDTP_CAPABILITIES, DBT_AVDTP_CONFIGURATION,
               DBT_AVDTP_RECONFIGURATION } dbt_avdtp_cap_mode_t;
typedef struct {
    const uint8_t *data;
    uint8_t category, size;
} dbt_avdtp_cap_t;
typedef struct {
    uint16_t present;
    dbt_avdtp_cap_t codec;
} dbt_avdtp_caps_t;
typedef struct { uint8_t category, code; } dbt_avdtp_cap_error_t;

/* No allocation/copy. Views borrow the encoded message. next() preserves cursor
 * and output on a malformed element; returns EMPTY only at the exact end. */
dbt_result_t dbt_avdtp_cap_next(const uint8_t *data, uint16_t size, uint16_t *cursor,
                                dbt_avdtp_cap_t *capability);
/* Validate transport TLVs and command-specific category/duplicate rules. Unknown
 * categories in capabilities responses are skipped, but never accepted into a
 * configuration. Capabilities may repeat content protection; configurations may
 * not. The iterator preserves every alternative. Codec/CP-specific validation,
 * supported-service selection and allocation readiness belong to the profile.
 * Parsing a recovery/multiplexing entry does not enable those services. On wire
 * failure returns INVALID and the Set/Reconfigure rejection category/code. */
dbt_result_t dbt_avdtp_caps_validate(const uint8_t *data, uint16_t size,
    dbt_avdtp_cap_mode_t mode, dbt_avdtp_caps_t *capabilities, dbt_avdtp_cap_error_t *error);

#endif
