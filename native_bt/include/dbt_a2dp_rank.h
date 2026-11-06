#ifndef DBT_A2DP_RANK_H
#define DBT_A2DP_RANK_H
#include "dbt_a2dp_generate.h"
#define DBT_A2DP_RANK_MAX_REQUESTS 16u
#define DBT_A2DP_RANK_CONFIGURATION_BYTES 32u

typedef enum {DBT_A2DP_RANK_NONE,DBT_A2DP_RANK_SUCCESS,DBT_A2DP_RANK_NO_MATCH,
    DBT_A2DP_RANK_DISCOVERY_ERROR,DBT_A2DP_RANK_CAPABILITIES_ERROR,DBT_A2DP_RANK_GENERATOR_ERROR,
    DBT_A2DP_RANK_CANCELLED,DBT_A2DP_RANK_SIGNAL_CHANGED,DBT_A2DP_RANK_CONFLICT,
    DBT_A2DP_RANK_TIMEOUT,DBT_A2DP_RANK_INTERNAL_ERROR} dbt_a2dp_rank_outcome_t;
typedef enum {DBT_A2DP_RANK_DISCOVER_WAIT,DBT_A2DP_RANK_NEXT,DBT_A2DP_RANK_CAPS_SUBMIT,
    DBT_A2DP_RANK_CAPS_WAIT,DBT_A2DP_RANK_EVALUATE,DBT_A2DP_RANK_CANCELLING,
    DBT_A2DP_RANK_FAULT_RETAIN,DBT_A2DP_RANK_DONE} dbt_a2dp_rank_phase_t;
typedef struct {
    const dbt_a2dp_generate_request_t *requests;uint8_t request_count;
    uint8_t *discovery,*capabilities;uint16_t discovery_capacity,capability_capacity;
    uint32_t timeout_ms;
} dbt_a2dp_rank_config_t;
typedef struct {
    uint32_t token,binding;
    dbt_a2dp_rank_outcome_t outcome;dbt_a2dp_rank_phase_t phase;
    dbt_avdtp_query_result_t discovery,last_capabilities,best_capabilities;
    dbt_a2dp_generate_result_t last_generated,best_generated;
    dbt_result_t submit_status,cancel_status;
    uint16_t submit_busy,evaluations;
    uint8_t records,visited,eligible,assessed,skipped,current_seid,current_request,best_request;
    uint16_t configuration_size;
    bool best_found,assessment_complete,cancel_requested,needs_signal_retirement;
} dbt_a2dp_rank_result_t;
typedef struct {
    dbt_avdtp_query_t query;
    dbt_avdtp_t *signal;const dbt_avdtp_endpoint_t *endpoints;
    dbt_a2dp_rank_config_t config;dbt_a2dp_rank_result_t result;
    dbt_avdtp_sep_t current;
    uint32_t serial,binding,query_token,deadline;
    uint16_t discovery_bytes,capability_bytes,cursor;
    uint8_t endpoint_count,request_index;
    bool active,has_query,cancel_attempted,submit_attempted;
    uint8_t best[DBT_A2DP_RANK_CONFIGURATION_BYTES];
} dbt_a2dp_rank_t;

/* Caller-owned bounded asynchronous ranking, not stream configuration.
 * Lower request index wins; equal priority keeps first discovery order, with
 * no claim of ranking unspecified defaults by acoustic/encoder quality. Every
 * eligible Source must be assessed; query failure prevents a successful lower
 * proposal. The explicit request list alone authorizes any format fallback.
 * No heap/task/default context. Poll alongside the existing signal/bearer.
 * One query transition, bounded SEP skip or format evaluation per iteration.
 * Config pointers, requests, endpoint descriptors and their bytes are borrowed
 * through matching terminal result; discovery/cap buffers must be disjoint and
 * remain unchanged by the caller. Capacities may deliberately bound memory:
 * truncation returns a query error, never partial successful ranking.
 * Cancellation/timeout waits for the accepted query callback fence. Stale/error
 * tokens do not release it. FAULT_RETAIN requires owner signal retirement and
 * retains storage rather than inventing a release fence. */
void dbt_a2dp_rank_init(dbt_a2dp_rank_t *);
dbt_result_t dbt_a2dp_rank_begin(dbt_a2dp_rank_t *,dbt_avdtp_t *,const dbt_avdtp_endpoint_t *,uint8_t,
    dbt_a2dp_rank_config_t,uint32_t now_ms,uint32_t *token);
void dbt_a2dp_rank_poll(dbt_a2dp_rank_t *,uint32_t now_ms);
uint32_t dbt_a2dp_rank_wait_ms(const dbt_a2dp_rank_t *,uint32_t now_ms);
dbt_result_t dbt_a2dp_rank_cancel(dbt_a2dp_rank_t *,uint32_t token);
/* Owner handoff/failure intent only; no callback-time mutation of transport. */
dbt_result_t dbt_a2dp_rank_interrupt(dbt_a2dp_rank_t *,uint32_t token,dbt_a2dp_rank_outcome_t);
/* OK fences input/response borrows. Proposal view exists only on complete
 * SUCCESS and stays inside caller context until next accepted begin/init/free.
 * Copy it before freeing context; no reservation or peer acceptance is implied. */
dbt_result_t dbt_a2dp_rank_result(const dbt_a2dp_rank_t *,uint32_t token,dbt_a2dp_rank_result_t *,const uint8_t **proposal);
bool dbt_a2dp_rank_done(const dbt_a2dp_rank_t *);
#endif
