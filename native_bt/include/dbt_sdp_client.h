#ifndef DBT_SDP_CLIENT_H
#define DBT_SDP_CLIENT_H
#include "dbt_sdp.h"

typedef struct {uint32_t binding;uint16_t query;} dbt_sdp_client_id_t;
typedef enum {
    DBT_SDP_CLIENT_NONE,DBT_SDP_CLIENT_SUCCESS,DBT_SDP_CLIENT_REMOTE_ERROR,
    DBT_SDP_CLIENT_NO_SPACE,DBT_SDP_CLIENT_PROTOCOL_ERROR,DBT_SDP_CLIENT_TIMEOUT,
    DBT_SDP_CLIENT_CANCELLED,DBT_SDP_CLIENT_TRANSPORT_ERROR
} dbt_sdp_client_outcome_t;
typedef enum {
    DBT_SDP_TX_UNSENT,DBT_SDP_TX_ADMITTED,DBT_SDP_TX_LINK_LOST,DBT_SDP_TX_ABORTED
} dbt_sdp_client_tx_outcome_t;
typedef struct {
    dbt_sdp_client_id_t id;
    dbt_sdp_client_outcome_t outcome;
    uint16_t bytes,records,peer_error,error_info_bytes,responses,admissions,stale; /* records: Search only. */
    bool must_close,cancel_requested,pinned,retirement_timed_out;
} dbt_sdp_client_result_t;
typedef struct {
    uint32_t binding,phase_ms,total_ms;
    uint16_t rx_mtu,tx_mtu,request_capacity,response_capacity;
    uint8_t *request,*response;
} dbt_sdp_client_config_t;
typedef struct {
    dbt_sdp_client_config_t config;
    dbt_sdp_client_result_t result;
    uint32_t phase_deadline,total_deadline;
    uint16_t next_tid,tid,prefix_size,tx_size,maximum,total_records;
    uint8_t kind,continuation_size,continuation[16];
    bool active,pinned,ready,waiting,response_seen,more,total_known,ever_admitted;
} dbt_sdp_client_t;

/* One owner and one query per channel lifetime. No heap/task/wait/callback.
 * Binding must identify the exact L2CAP channel lifetime; never reinitialize a
 * live object, reset IDs on the same channel, or reuse buffers while pinned.
 * Buffers and context must be disjoint and live through all transport loans.
 * Receive PDUs must be disjoint from that storage. MTUs/capacities are bounded
 * by DBT_SDP_RECORD_BYTES. No product instance or default large buffer exists. */
dbt_result_t dbt_sdp_client_init(dbt_sdp_client_t *,dbt_sdp_client_config_t);
/* Kinds 2/4/6 cover Search/Attribute/SearchAttribute. Prefix has no continuation
 * field. It is copied before return; output ID is unchanged on refusal.
 * Successful new begin invalidates the previous result view. */
dbt_result_t dbt_sdp_client_begin(dbt_sdp_client_t *,uint8_t,const uint8_t *,uint16_t,
                                 uint32_t now_ms,dbt_sdp_client_id_t *);
/* Peek pins immutable request storage. Every successful peek needs tx_done,
 * including when the runtime refuses a send. UNSENT is permitted only after
 * proof that no SDU was admitted and no transport still borrows it. LINK_LOST/
 * ABORTED also require actual transport retirement, not a cancel request. */
dbt_result_t dbt_sdp_client_tx_peek(dbt_sdp_client_t *,uint32_t now_ms,const uint8_t **,
                                   uint16_t *size,uint16_t *transaction);
dbt_result_t dbt_sdp_client_tx_done(dbt_sdp_client_t *,uint32_t binding,uint16_t transaction,
                                   dbt_sdp_client_tx_outcome_t,uint32_t now_ms);
dbt_result_t dbt_sdp_client_receive(dbt_sdp_client_t *,uint32_t binding,const uint8_t *,uint16_t,uint32_t now_ms);
dbt_result_t dbt_sdp_client_cancel(dbt_sdp_client_t *,dbt_sdp_client_id_t);
void dbt_sdp_client_abort(dbt_sdp_client_t *,uint32_t binding);
void dbt_sdp_client_poll(dbt_sdp_client_t *,uint32_t now_ms);
uint32_t dbt_sdp_client_wait_ms(const dbt_sdp_client_t *,uint32_t now_ms);
/* Readiness is separate from deadlines. Bearer scheduling must check actual
 * transport capacity; a refused send does not create an immediate retry wait. */
bool dbt_sdp_client_tx_ready(const dbt_sdp_client_t *);
/* Pending includes terminal results whose TX loan has not retired. Data is
 * exposed only for complete validated SUCCESS. Optional ErrorInfo is counted,
 * not retained. must_close requires channel teardown/new binding before begin.
 * Observe must_close even while PENDING: start teardown, then await tx_done.
 * IDs never wrap on a channel; exhaustion also requires a new binding. */
dbt_result_t dbt_sdp_client_result(const dbt_sdp_client_t *,dbt_sdp_client_id_t,
                                  dbt_sdp_client_result_t *,const uint8_t **data);
#endif
