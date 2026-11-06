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
    /* Both zero selects RTP. Otherwise a raw packet contains whole codec
     * codewords; framing comes from the selected endpoint, never sniffing
     * compressed bytes that can coincidentally resemble an RTP header. */
    uint16_t raw_codeword_bytes,raw_codeword_samples;
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


typedef struct dbt_avdtp_local dbt_avdtp_local_t;
typedef enum {
    DBT_AVDTP_LOCAL_NONE,DBT_AVDTP_LOCAL_SUCCESS,DBT_AVDTP_LOCAL_PEER_REJECTED,
    DBT_AVDTP_LOCAL_WORKER_REJECTED,DBT_AVDTP_LOCAL_WORKER_FATAL,
    DBT_AVDTP_LOCAL_WORKER_TIMEOUT,DBT_AVDTP_LOCAL_CANCELLED,
    DBT_AVDTP_LOCAL_REMOTE_ABORT,DBT_AVDTP_LOCAL_SIGNAL_ERROR,DBT_AVDTP_LOCAL_CLEANUP_ERROR,DBT_AVDTP_LOCAL_ROLLBACK_ERROR,DBT_AVDTP_LOCAL_MEDIA_ERROR,DBT_AVDTP_LOCAL_MEDIA_TIMEOUT
} dbt_avdtp_local_outcome_t;
typedef enum {DBT_AVDTP_LOCAL_PREPARING,DBT_AVDTP_LOCAL_WAIT_PEER,
    DBT_AVDTP_LOCAL_CLEANUP,DBT_AVDTP_LOCAL_RETIRING,DBT_AVDTP_LOCAL_DONE,DBT_AVDTP_LOCAL_ROLLBACK,DBT_AVDTP_LOCAL_WAIT_MEDIA_CLOSE,DBT_AVDTP_LOCAL_APPLYING} dbt_avdtp_local_phase_t;
typedef struct {
    uint32_t token,binding,worker_token,drop_token,rollback_token,media_binding,worker_deadline_ms,rollback_deadline_ms;
    dbt_avdtp_local_outcome_t outcome;dbt_avdtp_local_phase_t phase;
    dbt_sink_operation_t operation,rollback_operation;
    dbt_avdtp_outcome_t transport_outcome;dbt_avdtp_type_t response_type;
    dbt_sink_work_result_t worker_result,drop_result,rollback_result;
    dbt_result_t worker_submit_result,drop_submit_result,rollback_submit_result;
    uint16_t worker_busy,drop_busy,rollback_busy;
    uint16_t configuration_size,media_status;
    uint8_t local_seid,remote_seid,label,command,peer_category,peer_error,peer_seid,media_event,media_reason;
    bool transport_done,response_seen,worker_accepted,worker_result_valid;
    bool drop_accepted,drop_result_valid,cancel_requested,remote_abort;
    bool rollback_accepted,rollback_result_valid,rollback_timed_out,abort_response_admitted,media_closed,command_cancelled_unsent,media_close_requested,media_close_failed,media_timed_out,media_event_valid,worker_not_needed,media_was_pending;
    bool worker_timed_out,cleanup_timed_out,cleanup_complete,resources_transferred,profile_retired;
} dbt_avdtp_local_result_t;

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
    uint32_t raw_timestamp;
    uint16_t raw_sequence;
    uint16_t configuration_size, maximum_configuration;
    dbt_stream_state_t state;
    uint8_t count, endpoint, remote_seid;
    bool worker_live, retiring, failed, in_work, drained, delay_enabled, media_initiator;
    dbt_avdtp_local_t *local;
} dbt_avdtp_sink_t;

struct dbt_avdtp_local {
    dbt_avdtp_completion_t completion;dbt_avdtp_sink_t *sink;
    dbt_avdtp_local_result_t result;uint32_t serial;
    uint8_t endpoint,abort_label;
    bool active,done,notifying,wire_released,cleaning,drop_started,rollback_started,cancel_attempted,abort_queued;
};
/* Local Set Configuration: caller supplies an exact supported
 * selection, local Sink SEID and remote Source SEID. The command is copied
 * and blocked before worker preparation; only READY releases it to the peer.
 * No caller configuration bytes are borrowed. Keep context until matching
 * terminal result; rejection waits for acknowledged DROP before reuse.
 * Call all methods on the signaling owner, outside borrowed callbacks; init
 * only a fresh or terminal context. Begin returns PENDING only on acceptance.
 * Submitted/borrowed cancellation may retire signaling to avoid an uncertain
 * peer configuration. Local Open commits only signaling acceptance; the profile
 * must establish its media channel separately. */
void dbt_avdtp_local_init(dbt_avdtp_local_t *);
dbt_result_t dbt_avdtp_sink_local_configure_begin(dbt_avdtp_sink_t *,dbt_avdtp_local_t *,
    uint8_t local_seid,uint8_t remote_seid,const uint8_t *,uint16_t,uint32_t,uint32_t *token);
/* Open the current configured endpoint. Decoder preparation is asynchronous;
 * rejection/pre-wire cancel retain its configuration. Peer Abort or uncertain
 * post-wire cancellation retires resources before terminal completion. */
dbt_result_t dbt_avdtp_sink_local_open_begin(dbt_avdtp_sink_t *,dbt_avdtp_local_t *,
    uint32_t now_ms,uint32_t *token);
/* Start only the exact currently bound media lifetime. READY prepares the
 * renderer before command release; Accept commits streaming. Rejection or
 * pre-wire cancel after accepted preparation must acknowledge local SUSPEND
 * rollback before reusing the configured Open stream. Abort supersedes with
 * DROP; uncertain post-wire cancel/fatal rollback retires the profile. */
dbt_result_t dbt_avdtp_sink_local_start_begin(dbt_avdtp_sink_t *,dbt_avdtp_local_t *,
    uint32_t media_binding,uint32_t now_ms,uint32_t *token);
/* Suspend sends first and preserves current media admission until peer Accept.
 * Only then is local SUSPEND enqueued; READY commits Open. Rejection or exact
 * unsent cancel retains streaming. Failure after acceptance retires the profile.
 * The caller must supply the exact bound media lifetime, as for Start. */
dbt_result_t dbt_avdtp_sink_local_suspend_begin(dbt_avdtp_sink_t *,dbt_avdtp_local_t *,
    uint32_t media_binding,uint32_t now_ms,uint32_t *token);
/* Close from established Open/Streaming: keep the prior stream before peer
 * acceptance, then join asynchronous decoder release and exact media closure.
 * The caller context/configuration remain reserved until both are acknowledged.
 * Rejection or unsent cancel retains the prior state unless media was lost. */
dbt_result_t dbt_avdtp_sink_local_close_begin(dbt_avdtp_sink_t *,dbt_avdtp_local_t *,
    uint32_t media_binding,uint32_t now_ms,uint32_t *token);
/* Reconfigure the current Open endpoint using Reconfigure TLVs, requiring
 * an exact locally supported codec selection. Transport and negotiated Delay
 * remain committed. New normalized bytes are copied; only caller context is
 * borrowed. READY precedes wire release; Accept commits. Reject/pre-wire cancel
 * restores the old committed codec through a separately acknowledged worker
 * job when needed, without holding two decoder workspaces. */
dbt_result_t dbt_avdtp_sink_local_reconfigure_begin(dbt_avdtp_sink_t *,dbt_avdtp_local_t *,
    uint32_t media_binding,const uint8_t *,uint16_t,uint32_t now_ms,uint32_t *token);
/* Abort starts local release and wire submission independently. The owner
 * supplies zero, the bound media lifetime or an exactly reserved pending media
 * lifetime. It is irreversible once accepted: cancel/reject/failure retires
 * the profile. Context remains borrowed through worker/wire/media retirement.
 * Other active local/remote procedures return BUSY without accepting this call. */
dbt_result_t dbt_avdtp_sink_local_abort_begin(dbt_avdtp_sink_t *,dbt_avdtp_local_t *,
    uint32_t media_lifetime,uint32_t now_ms,uint32_t *token);
dbt_result_t dbt_avdtp_local_cancel(dbt_avdtp_local_t *,uint32_t token);
dbt_result_t dbt_avdtp_local_result(const dbt_avdtp_local_t *,uint32_t token,dbt_avdtp_local_result_t *);

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
/* Owner distinguishes failed transport retirement from an acknowledged close. */
dbt_result_t dbt_avdtp_sink_media_closed_ex(dbt_avdtp_sink_t *,uint32_t media_binding,bool transport_failed);
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
