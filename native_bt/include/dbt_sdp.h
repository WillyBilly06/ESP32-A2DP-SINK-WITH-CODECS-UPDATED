#ifndef DBT_SDP_H
#define DBT_SDP_H

#include "dbt_hci.h"

#define DBT_SDP_RECORD_LIMIT 32u
#define DBT_SDP_ATTRIBUTE_LIMIT 64u
#define DBT_SDP_RECORD_BYTES 4096u
#define DBT_SDP_DEPTH_LIMIT 8u

/* Attribute values are complete SDP data elements in network byte order.
 * Immutable records/attributes/values may reside in flash. Attribute 0 is
 * generated from the record handle; supplied IDs must be strictly increasing
 * and begin with the mandatory ServiceClassIDList (1). No database copy. */
typedef struct {
    uint16_t id, size;
    const uint8_t *value;
} dbt_sdp_attribute_t;

typedef struct {
    uint32_t handle;
    const dbt_sdp_attribute_t *attributes;
    uint16_t count;
} dbt_sdp_record_t;

typedef struct {
    const dbt_sdp_record_t *records;
    uint32_t database_state;
    uint16_t count;
} dbt_sdp_t;

/* Single owner. Reinitialize with a different nonzero database_state after any
 * database change; keep each installed immutable database alive until replaced.
 * Handle 0 is reserved for the SDP server itself, 1..65535 are not registrable. */
dbt_result_t dbt_sdp_init(dbt_sdp_t *server, const dbt_sdp_record_t *records, uint16_t count,
                          uint32_t database_state);

/* Respond to exactly one complete request. binding identifies the L2CAP channel
 * lifetime and must change on reuse. The continuation contains no pointers and
 * is checked against binding, database version, query and offset. Its checksum
 * is a consistency check, not an authentication credential; records are public.
 *
 * capacity is bounded by the peer MTU. output and request must be disjoint.
 * With output=NULL this measures the next response without copying; an exact
 * allocation of *written bytes can then be used for the same request. Malformed
 * requests with a complete header get an SDP ErrorResponse (DBT_OK). A truncated
 * header returns DBT_INVALID; capacity below the error header returns NO_SPACE.
 * No allocation, mutable transaction state, or retained request pointers. */
dbt_result_t dbt_sdp_respond(const dbt_sdp_t *server, uint32_t binding, const uint8_t *request,
                             uint16_t request_size, uint8_t *output, uint16_t capacity,
                             uint16_t *written);

#endif
