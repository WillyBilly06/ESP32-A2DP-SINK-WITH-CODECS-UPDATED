#ifndef DBT_A2DP_ACCEPTOR_H
#define DBT_A2DP_ACCEPTOR_H

#include "dbt_avdtp_bearer.h"
#include "dbt_avdtp_sink.h"

typedef struct {
    uint32_t serial,generation;
    uint16_t handle,cid,rx_mtu,tx_mtu;
} dbt_a2dp_channel_t;
typedef struct {
    dbt_runtime_t *runtime;
    dbt_avdtp_sink_t sink;
    dbt_avdtp_bearer_t bearer;
    dbt_avdtp_sink_hooks_t client;
    const dbt_avdtp_endpoint_t *endpoints;
    dbt_a2dp_channel_t signaling,media;
    uint32_t signal_ms,worker_ms,media_ms;
    uint16_t mtu,maximum_configuration;
    uint8_t count;
    bool enabled,active,draining,close_media;
} dbt_a2dp_acceptor_t;

/* Register encrypted Classic PSM 0x19, MTU >=335. Call after runtime.ready.
 * One sink reserves the speaker across peers until worker DROP is acknowledged.
 * Wire accept(), channel events and data into the runtime's composite hooks;
 * other PSMs remain the application's responsibility. No decoder task or SDP
 * record is installed here. init allocates nothing. Keep endpoints immutable. */
dbt_result_t dbt_a2dp_acceptor_init(dbt_a2dp_acceptor_t *profile,dbt_runtime_t *runtime,
    dbt_avdtp_sink_hooks_t worker,const dbt_avdtp_endpoint_t *endpoints,uint8_t count,
    uint16_t mtu,uint16_t maximum_configuration,uint32_t signal_ms,uint32_t worker_ms,uint32_t media_ms);
bool dbt_a2dp_acceptor_accept(const dbt_a2dp_acceptor_t *profile,const dbt_link_t *link);
/* Owner callbacks: no channel mutation/reentry; close operations are deferred
 * to poll. UNSUPPORTED means another PSM. INVALID means an unexpected channel;
 * a composite application must close that channel explicitly. */
dbt_result_t dbt_a2dp_acceptor_event(dbt_a2dp_acceptor_t *profile,dbt_channel_event_t event,
    const dbt_channel_t *channel);
dbt_result_t dbt_a2dp_acceptor_receive(dbt_a2dp_acceptor_t *profile,const dbt_channel_t *channel,
    const uint8_t *data,uint16_t size);
dbt_result_t dbt_a2dp_acceptor_poll(dbt_a2dp_acceptor_t *profile,uint32_t now_ms);
uint32_t dbt_a2dp_acceptor_wait_ms(const dbt_a2dp_acceptor_t *profile,uint32_t now_ms);
dbt_result_t dbt_a2dp_acceptor_complete(dbt_a2dp_acceptor_t *profile,uint32_t binding,uint32_t token,
    dbt_sink_work_result_t result,uint32_t now_ms);
/* Retire the current session without disabling future admission. Safe from a
 * channel callback: only requests draining; poll closes channels and waits for
 * worker DROP. BUSY means cleanup remains, and a new session is still gated. */
dbt_result_t dbt_a2dp_acceptor_retire(dbt_a2dp_acceptor_t *profile,uint32_t now_ms);
/* Stops admission; BUSY means channels/borrowed TX/worker cleanup remain. Keep
 * runtime and this profile polling and forwarding receipts until OK. */
dbt_result_t dbt_a2dp_acceptor_stop(dbt_a2dp_acceptor_t *profile,uint32_t now_ms);

#endif
