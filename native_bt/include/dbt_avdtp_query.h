#ifndef DBT_AVDTP_QUERY_H
#define DBT_AVDTP_QUERY_H
#include "dbt_avdtp_caps.h"

typedef struct {uint8_t seid,media_type;bool sink,in_use;} dbt_avdtp_sep_t;
typedef enum {
    DBT_AVDTP_QUERY_NONE,DBT_AVDTP_QUERY_SUCCESS,DBT_AVDTP_QUERY_REJECTED,
    DBT_AVDTP_QUERY_GENERAL_REJECT,DBT_AVDTP_QUERY_PROTOCOL_ERROR,
    DBT_AVDTP_QUERY_NO_SPACE,DBT_AVDTP_QUERY_TIMEOUT,DBT_AVDTP_QUERY_CANCELLED
} dbt_avdtp_query_outcome_t;
typedef struct {
    uint32_t token,binding;
    dbt_avdtp_query_outcome_t outcome;
    dbt_avdtp_outcome_t transport_outcome;
    dbt_avdtp_type_t response_type;
    dbt_result_t payload_status;
    dbt_avdtp_cap_error_t capability_error;
    uint16_t bytes,required;
    uint8_t command,seid,label,peer_error;
    bool cancel_requested,response_seen;
} dbt_avdtp_query_result_t;
typedef struct {
    dbt_avdtp_completion_t completion;
    dbt_avdtp_t *signal;
    uint8_t *response;
    uint32_t serial,binding;
    uint16_t capacity;
    bool active,done;
    dbt_avdtp_query_result_t result;
} dbt_avdtp_query_t;

/* Owner-only. Fresh init once; begin reuses a completed context without
 * resetting its serial. Context/response/token/result must be disjoint and
 * caller-owned, independent of all signal-owned buffers. No default buffers,
 * heap, task or decoder work. Keep the normal signal/bearer owner polling.
 * Allowed requests: Discover (seid0), Get[All]Capabilities/GetConfiguration
 * (seid1..62). Capability acceptance is not codec compatibility or audio setup. */
void dbt_avdtp_query_init(dbt_avdtp_query_t *);
dbt_result_t dbt_avdtp_query_begin(dbt_avdtp_query_t *,dbt_avdtp_t *,uint8_t command,uint8_t seid,
    uint8_t *response,uint16_t capacity,uint32_t now_ms,uint32_t *token);
/* Cancel may wait for a borrowed/submitted request's response/timeout/clear.
 * A stale/error cancel is never a release fence. Only a matching terminal
 * result permits context/buffer release; never reinitialize/free it earlier. */
dbt_result_t dbt_avdtp_query_cancel(dbt_avdtp_query_t *,uint32_t token,uint32_t now_ms);
/* Terminal OK fences the callback borrow. Body bytes were copied in full into
 * the caller buffer; no-space never exposes a successful partial response.
 * Returned view remains caller-owned until the next accepted begin or free.
 * A timeout fences this context, not the signal's separately pinned TX bytes. */
dbt_result_t dbt_avdtp_query_result(const dbt_avdtp_query_t *,uint32_t token,
    dbt_avdtp_query_result_t *,const uint8_t **body);
/* Endpoint iterator preserves cursor/output on malformed data. Reserved field
 * bits are ignored. All up to 62 records are preserved; no SBC-only filtering. */
dbt_result_t dbt_avdtp_sep_next(const uint8_t *,uint16_t,uint16_t *,dbt_avdtp_sep_t *);
#endif
