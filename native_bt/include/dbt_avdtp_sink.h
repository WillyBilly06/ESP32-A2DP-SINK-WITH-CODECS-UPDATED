#ifndef DBT_AVDTP_SINK_H
#define DBT_AVDTP_SINK_H

#include "dbt_avdtp_caps.h"

#define DBT_AVDTP_SINK_ENDPOINT_LIMIT 16u
typedef struct dbt_avdtp_endpoint dbt_avdtp_endpoint_t;
struct dbt_avdtp_endpoint {
    const uint8_t *capabilities;
    uint16_t size;
    uint8_t seid;
    /* Pure bounded metadata validation, no decoder construction. Returns an
     * AVDTP/A2DP rejection code, or zero for a supported complete selection. */
    uint8_t (*validate)(const dbt_avdtp_endpoint_t *endpoint, const dbt_avdtp_cap_t *codec);
};
typedef enum { DBT_STREAM_IDLE, DBT_STREAM_CONFIGURED, DBT_STREAM_OPEN,
    DBT_STREAM_STREAMING, DBT_STREAM_CLOSING, DBT_STREAM_ABORTING } dbt_stream_state_t;
typedef enum { DBT_SINK_CONFIGURE, DBT_SINK_RECONFIGURE, DBT_SINK_OPEN,
    DBT_SINK_START, DBT_SINK_SUSPEND, DBT_SINK_CLOSE, DBT_SINK_ABORT,
    DBT_SINK_DROP } dbt_sink_operation_t;
typedef enum { DBT_SINK_WORK_READY, DBT_SINK_WORK_REJECTED, DBT_SINK_WORK_FATAL } dbt_sink_work_result_t;
typedef enum { DBT_SINK_STATE, DBT_SINK_CLOSE_MEDIA, DBT_SINK_FAULT,
               DBT_SINK_DRAINED, DBT_SINK_DELAY_RESPONSE } dbt_sink_event_t;

typedef struct {
    uint32_t binding, token;
    dbt_sink_operation_t operation;
    uint8_t seid;
    const uint8_t *configuration;
    uint16_t size;
} dbt_sink_job_t;
typedef struct {
    void *context;
    uint8_t *(*allocate)(void *context, uint16_t size);
    void (*release)(void *context, uint8_t *data);
    /* Nonblocking enqueue. Must copy all job/configuration bytes before return.
     * OK requires a later owner-thread complete(); BUSY retries without acceptance.
     * Other results mean no job or resources were accepted. Never complete from
     * inside this call. The worker serializes jobs for this sink, releases old
     * large state before reconfiguration and never owns two decoder workspaces.
     * REJECTED completion guarantees the previous configuration remains usable;
     * if rollback cannot restore it, complete FATAL instead. DROP must release
     * all older jobs/resources before its READY acknowledgment. */
    dbt_result_t (*work)(void *context, const dbt_sink_job_t *job);
    /* Borrowed/nonblocking notification; inspect state, defer all mutations. */
    void (*event)(void *context, dbt_sink_event_t event, dbt_stream_state_t state);
    /* Borrowed RTP view, nonblocking copy to a bounded media worker queue. */
    dbt_result_t (*media)(void *context, const dbt_avdtp_media_t *media);
} dbt_avdtp_sink_hooks_t;

typedef struct {
    uint8_t *configuration;
    uint32_t token, deadline;
    uint16_t size;
    dbt_sink_operation_t operation;
    dbt_avdtp_type_t reply;
    uint8_t label, signal, endpoint, remote_seid, error_detail, error;
    bool used, submitted, attempted, done, queued, apply;
} dbt_sink_pending_t;

typedef struct {
    dbt_avdtp_t signal;
    dbt_avdtp_sink_hooks_t hooks;
    const dbt_avdtp_endpoint_t *endpoints;
    dbt_sink_pending_t pending;
    uint8_t *configuration;
    uint32_t binding, next_token, worker_ms, media_ms, media_deadline, media_binding;
    uint32_t media_packets, media_rejected;
    uint16_t configuration_size, maximum_configuration;
    dbt_stream_state_t state;
    uint8_t count, endpoint, remote_seid;
    bool worker_live, retiring, failed, in_work, drained, delay_enabled;
} dbt_avdtp_sink_t;

/* Acceptor side, one active audio stream/decoder per sink instance, selected
 * from immutable flash endpoints. No per-codec mutable arrays. The application
 * must reserve the speaker's sink instance across signaling connections; never
 * advertise the same endpoints through two uncoordinated instances. This first
 * sink supports media transport, codec and delay reporting; other services are
 * refused at init instead of being falsely advertised. Init allocates nothing.
 * Reinitialize only after shutdown returns OK and every worker receipt drains. */
dbt_result_t dbt_avdtp_sink_init(dbt_avdtp_sink_t *sink, dbt_avdtp_sink_hooks_t hooks,
    const dbt_avdtp_endpoint_t *endpoints, uint8_t count, uint32_t binding,
    uint16_t rx_mtu, uint16_t tx_mtu, uint16_t maximum_configuration,
    uint32_t signal_ms, uint32_t worker_ms, uint32_t media_ms);
dbt_result_t dbt_avdtp_sink_poll(dbt_avdtp_sink_t *sink, uint32_t now_ms);
dbt_result_t dbt_avdtp_sink_complete(dbt_avdtp_sink_t *sink, uint32_t binding,
    uint32_t token, dbt_sink_work_result_t result, uint32_t now_ms);
/* Profile manager validates media channel handle/generation/PSM before binding
 * its unique nonzero lifetime here. OPEN response admission precedes this call. */
dbt_result_t dbt_avdtp_sink_media_open(dbt_avdtp_sink_t *sink, uint32_t media_binding, uint32_t now_ms);
dbt_result_t dbt_avdtp_sink_media_closed(dbt_avdtp_sink_t *sink, uint32_t media_binding);
dbt_result_t dbt_avdtp_sink_media_receive(dbt_avdtp_sink_t *sink, uint32_t media_binding,
    const uint8_t *packet, uint16_t size);
dbt_result_t dbt_avdtp_sink_delay(dbt_avdtp_sink_t *sink, uint16_t delay_100us, uint32_t now_ms);
uint32_t dbt_avdtp_sink_wait_ms(const dbt_avdtp_sink_t *sink, uint32_t now_ms);
/* First close/quiesce the signaling runtime. BUSY preserves pinned TX and waits
 * for asynchronous DROP. Poll the retiring instance and deliver worker receipts
 * until OK. Never recycle it or accept a replacement decoder while BUSY. */
dbt_result_t dbt_avdtp_sink_shutdown(dbt_avdtp_sink_t *sink, uint32_t now_ms);

/* A2DP SBC Media Codec category validator (6-byte category body), no allocation.
 * Other codecs plug their own validators into the same endpoint interface. */
uint8_t dbt_a2dp_sbc_validate(const dbt_avdtp_endpoint_t *endpoint, const dbt_avdtp_cap_t *codec);

#endif
