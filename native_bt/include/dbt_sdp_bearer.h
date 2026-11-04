#ifndef DBT_SDP_BEARER_H
#define DBT_SDP_BEARER_H

#include "dbt_runtime.h"
#include "dbt_sdp.h"

typedef struct {
    uint8_t *bytes;
    uint32_t serial, generation;
    uint16_t handle, cid, size;
    bool used, active;
    uint8_t error[7];
} dbt_sdp_reply_t;

typedef struct {
    dbt_runtime_t *runtime;
    const dbt_sdp_t *server;
    dbt_sdp_reply_t replies[DBT_CHANNEL_LIMIT];
    uint16_t maximum_pdu;
} dbt_sdp_bearer_t;

/* Owner only, after runtime.ready. Registers public Classic PSM 1. The immutable
 * database must describe only services actually implemented by the application.
 * Uses the runtime allocator for one exact-size response per busy SDP channel;
 * idle channels allocate nothing. OOM has an inline InsufficientResources reply.
 * maximum_pdu (48..runtime.maximum_mtu) limits both request and response storage. */
dbt_result_t dbt_sdp_bearer_init(dbt_sdp_bearer_t *bearer, dbt_runtime_t *runtime,
                                 const dbt_sdp_t *server, uint16_t maximum_pdu);
/* Forward dynamic data and channel events from runtime hooks. A second request
 * before the first response is admitted returns BUSY (SDP permits one outstanding
 * transaction per channel). Caller must explicitly close/refuse that violation. */
dbt_result_t dbt_sdp_bearer_receive(dbt_sdp_bearer_t *bearer, const dbt_channel_t *channel,
                                    const uint8_t *data, uint16_t size);
void dbt_sdp_bearer_event(dbt_sdp_bearer_t *bearer, dbt_channel_event_t event,
                          const dbt_channel_t *channel);
dbt_result_t dbt_sdp_bearer_poll(dbt_sdp_bearer_t *bearer);
uint32_t dbt_sdp_bearer_wait_ms(const dbt_sdp_bearer_t *bearer);
/* First quiesce/abort the runtime so it returns all borrowed responses. */
dbt_result_t dbt_sdp_bearer_clear(dbt_sdp_bearer_t *bearer);

#endif
