#ifndef DBT_SDP_DISCOVERY_H
#define DBT_SDP_DISCOVERY_H
#include "dbt_sdp_client_bearer.h"

typedef enum {
    DBT_SDP_DISCOVERY_NONE,DBT_SDP_DISCOVERY_SUCCESS,DBT_SDP_DISCOVERY_NOT_FOUND,
    DBT_SDP_DISCOVERY_UNSUPPORTED,DBT_SDP_DISCOVERY_CHANNEL_ERROR,DBT_SDP_DISCOVERY_QUERY_ERROR,
    DBT_SDP_DISCOVERY_PROTOCOL_ERROR,DBT_SDP_DISCOVERY_TIMEOUT,DBT_SDP_DISCOVERY_CANCELLED
} dbt_sdp_discovery_outcome_t;
typedef struct {
    uint8_t *request,*response;
    uint16_t request_capacity,response_capacity,rx_mtu;
    uint32_t phase_ms,total_ms;
} dbt_sdp_discovery_config_t;
typedef struct {
    uint32_t token,generation;
    uint16_t handle,channel_result;
    dbt_sdp_discovery_outcome_t outcome;
    dbt_channel_reason_t channel_reason;
    dbt_sdp_client_result_t query;
    dbt_sdp_av_service_t service;
    bool cancel_requested;
} dbt_sdp_discovery_result_t;
typedef struct {
    dbt_runtime_t *runtime;
    dbt_sdp_discovery_config_t config;
    dbt_sdp_client_t client;
    dbt_sdp_client_bearer_t bearer;
    dbt_sdp_client_id_t query;
    dbt_sdp_av_service_t service;
    uint32_t deadline,serial,generation;
    uint16_t handle,cid,channel_result;
    dbt_channel_reason_t channel_reason;
    uint8_t state,outcome;
    bool client_ready,cancel_requested,timeout_seen;
} dbt_sdp_discovery_t;

/* One-owner Audio Source discovery on an existing exact Classic ACL link.
 * SDP PSM 1 must already be registered. Begin/poll run outside channel hooks;
 * event/receive are callback-safe. No task/heap/default buffers are created.
 * Context/buffers remain live through channel and TX retirement. Product
 * routing registrations must also detach before caller storage is released. */
void dbt_sdp_discovery_init(dbt_sdp_discovery_t *); /* Fresh/retired storage only. */
dbt_result_t dbt_sdp_discovery_begin(dbt_sdp_discovery_t *,dbt_runtime_t *,const dbt_link_t *,
    dbt_sdp_discovery_config_t,uint32_t now_ms,uint32_t *token);
bool dbt_sdp_discovery_matches(const dbt_sdp_discovery_t *,const dbt_channel_t *);
void dbt_sdp_discovery_event(dbt_sdp_discovery_t *,dbt_channel_event_t,const dbt_channel_t *,
    dbt_channel_reason_t,uint16_t result,uint32_t now_ms);
dbt_result_t dbt_sdp_discovery_receive(dbt_sdp_discovery_t *,const dbt_channel_t *,const uint8_t *,uint16_t,uint32_t);
void dbt_sdp_discovery_poll(dbt_sdp_discovery_t *,uint32_t now_ms);
uint32_t dbt_sdp_discovery_wait_ms(const dbt_sdp_discovery_t *,uint32_t now_ms);
dbt_result_t dbt_sdp_discovery_cancel(dbt_sdp_discovery_t *,uint32_t token);
dbt_result_t dbt_sdp_discovery_result(const dbt_sdp_discovery_t *,uint32_t token,dbt_sdp_discovery_result_t *);
#endif
