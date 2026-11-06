#ifndef DBT_A2DP_CONNECT_H
#define DBT_A2DP_CONNECT_H
#include "dbt_a2dp_acceptor.h"
#include "dbt_gap.h"
#include "dbt_sdp_discovery.h"

typedef enum {
    DBT_A2DP_CONNECT_NONE,DBT_A2DP_CONNECT_SIGNALING_READY,
    DBT_A2DP_CONNECT_PAGE_ERROR,DBT_A2DP_CONNECT_DISCOVERY_ERROR,
    DBT_A2DP_CONNECT_SIGNAL_ERROR,DBT_A2DP_CONNECT_PROFILE_BUSY,
    DBT_A2DP_CONNECT_LINK_LOST,DBT_A2DP_CONNECT_TIMEOUT,
    DBT_A2DP_CONNECT_CANCELLED,DBT_A2DP_CONNECT_INTERNAL_ERROR
} dbt_a2dp_connect_outcome_t;
typedef enum {
    DBT_A2DP_CONNECT_IDLE,DBT_A2DP_CONNECT_PAGING,DBT_A2DP_CONNECT_DISCOVERY_BEGIN,
    DBT_A2DP_CONNECT_DISCOVERING,DBT_A2DP_CONNECT_SIGNAL_BEGIN,DBT_A2DP_CONNECT_SIGNALING,
    DBT_A2DP_CONNECT_RETIRING,DBT_A2DP_CONNECT_DONE
} dbt_a2dp_connect_phase_t;
typedef struct {
    dbt_page_config_t peer;
    dbt_sdp_discovery_config_t discovery;
    uint32_t page_ms,page_retire_ms,total_ms;
} dbt_a2dp_connect_config_t;
typedef struct {
    uint32_t token,generation;
    uint16_t handle;
    dbt_a2dp_connect_outcome_t outcome;
    dbt_a2dp_connect_phase_t phase;
    dbt_result_t last_error;
    dbt_gap_connect_result_t page;
    dbt_sdp_discovery_result_t discovery;
    dbt_a2dp_channel_t signaling;
    dbt_channel_event_t signaling_event;dbt_channel_reason_t signaling_reason;
    uint32_t signaling_event_ms;uint16_t signaling_result;bool signaling_event_seen;
    bool cancel_requested,used_existing_acl,owns_signaling,link_live,signaling_live,ownership_fault;
} dbt_a2dp_connect_result_t;
typedef struct {
    dbt_a2dp_acceptor_t *profile;
    dbt_gap_t *gap;
    dbt_page_t page;
    dbt_sdp_discovery_t discovery;
    dbt_a2dp_connect_config_t config;
    dbt_gap_connect_result_t page_result;
    dbt_a2dp_channel_t signaling;
    uint32_t serial,page_token,discovery_token,deadline,generation;
    uint16_t handle,signaling_result;
    dbt_channel_event_t signaling_event;dbt_channel_reason_t signaling_reason;
    uint32_t signaling_event_ms;bool signaling_event_seen;
    dbt_result_t last_error;
    uint8_t phase,outcome;
    bool cancel_requested,page_cancelled,discovery_cancelled,discovery_done;
    bool signal_cancelled,owns_signaling,used_existing_acl,expired,ownership_fault;
} dbt_a2dp_connect_t;

/* Fresh storage only. Completed instances are reused by begin without resetting
 * their serial. Context/request/response/token/result storage must be disjoint. The
 * caller explicitly selects the address; this grants no pairing or bond trust.
 * SDP PSM1 must be registered and events/data routed to discovery below. Keep
 * normal runtime, GAP and profile polling. This object polls its SDP operation.
 * No default buffers, task, heap, storage I/O or automatic retry policy. */
void dbt_a2dp_connect_init(dbt_a2dp_connect_t *);
dbt_result_t dbt_a2dp_connect_begin(dbt_a2dp_connect_t *,dbt_a2dp_acceptor_t *,dbt_gap_t *,
    dbt_a2dp_connect_config_t,uint32_t now_ms,uint32_t *token);
/* Metadata only, callback-safe. Forward exact signaling channel events before
 * observers can consume/release the operation. Timestamps are owner times. */
void dbt_a2dp_connect_event(dbt_a2dp_connect_t *,dbt_channel_event_t,const dbt_channel_t *,
    dbt_channel_reason_t,uint16_t result,uint32_t now_ms);
void dbt_a2dp_connect_poll(dbt_a2dp_connect_t *,uint32_t now_ms);
uint32_t dbt_a2dp_connect_wait_ms(const dbt_a2dp_connect_t *,uint32_t now_ms);
dbt_result_t dbt_a2dp_connect_cancel(dbt_a2dp_connect_t *,uint32_t token);
bool dbt_a2dp_connect_done(const dbt_a2dp_connect_t *);
/* Terminal OK fences page callbacks and SDP teardown. Failure/cancel also
 * waits for owned signaling retirement; success hands signaling to the profile.
 * The routing adapter must detach discovery before caller storage is freed.
 * Success means signaling only, not endpoint negotiation/audio or bond trust.
 * Existing/shared ACLs and signaling are never torn down by this operation.
 * A page may win cancellation: its exact live ACL is reported for the caller's
 * broader connection policy; this module never silently drops that identity. */
dbt_result_t dbt_a2dp_connect_result(const dbt_a2dp_connect_t *,uint32_t token,dbt_a2dp_connect_result_t *);
#endif
