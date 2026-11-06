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
/* Outgoing signaling only, on an exact existing Classic ACL. The caller owns
 * peer/SDP selection; normal asynchronous encryption/authorization still runs.
 * PENDING copies a channel identity into key, BUSY accepts nothing. No caller
 * storage is borrowed. Route the usual channel events/data and keep polling.
 * This does not negotiate endpoints/configuration, start audio or select a
 * remembered peer. Initial MTUs are provisional until the OPEN event. */
dbt_result_t dbt_a2dp_acceptor_connect_begin(dbt_a2dp_acceptor_t *,const dbt_link_t *,
                                           uint32_t now_ms,dbt_a2dp_channel_t *key);
/* Current readiness, not a latched completion: OK is the exact live signaling
 * binding, PENDING is channel setup/retirement, EMPTY is stale/gone. Detailed
 * terminal reasons remain in the existing channel-event hook. */
dbt_result_t dbt_a2dp_acceptor_connect_status(const dbt_a2dp_acceptor_t *,const dbt_a2dp_channel_t *key);
/* Callback-safe close intent for that exact identity. Closing an established
 * signaling binding also retires its media/decoder through the normal owner. */
dbt_result_t dbt_a2dp_acceptor_connect_cancel(dbt_a2dp_acceptor_t *,const dbt_a2dp_channel_t *key,uint32_t now_ms);
/* After local Open Accept, initiate media on the exact current signaling key.
 * PENDING copies an identity and reserves it in the profile; only its OPEN
 * event binds media. No caller key storage is retained. No audio is started.
 * Poll/event/data routing and the existing media deadline remain mandatory.
 * BUSY accepts nothing. Call begin on the owner outside borrowed callbacks. */
dbt_result_t dbt_a2dp_acceptor_media_begin(dbt_a2dp_acceptor_t *,const dbt_a2dp_channel_t *signaling,
    uint32_t now_ms,dbt_a2dp_channel_t *media_key);
/* Readiness rather than a latched receipt; normal event reason/status retains
 * detailed failure information. Cancel retires this profile and decoder after
 * channel/worker ownership drains, preserving the shared ACL and other PSMs. */
dbt_result_t dbt_a2dp_acceptor_media_status(const dbt_a2dp_acceptor_t *,const dbt_a2dp_channel_t *);
dbt_result_t dbt_a2dp_acceptor_media_cancel(dbt_a2dp_acceptor_t *,const dbt_a2dp_channel_t *,uint32_t now_ms);
/* Abort the current endpoint on this exact signaling lifetime. Captures a
 * bound or staged media channel before accepting; even pending setup must
 * retire before terminal success. PENDING borrows context, not the input key. */
dbt_result_t dbt_a2dp_acceptor_abort_begin(dbt_a2dp_acceptor_t *,const dbt_a2dp_channel_t *,
    dbt_avdtp_local_t *,uint32_t now_ms,uint32_t *token);
/* Owner callbacks: no channel mutation/reentry; close operations are deferred
 * to poll. UNSUPPORTED means another PSM. INVALID means an unexpected channel;
 * a composite application must close that channel explicitly. */
dbt_result_t dbt_a2dp_acceptor_event(dbt_a2dp_acceptor_t *profile,dbt_channel_event_t event,
    const dbt_channel_t *channel);
/* Production routing must retain reason/status: CLOSED also covers failure of
 * a formerly open channel. The legacy wrapper assumes NORMAL/status0. */
dbt_result_t dbt_a2dp_acceptor_event_ex(dbt_a2dp_acceptor_t *,dbt_channel_event_t,
    const dbt_channel_t *,dbt_channel_reason_t reason,uint16_t wire_result);
/* Malformed signaling returns INVALID and increments sink.signal.discarded
 * without retiring live media. Transport/lifetime/resource failures still drain. */
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
