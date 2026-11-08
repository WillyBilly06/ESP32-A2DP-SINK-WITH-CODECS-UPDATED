#ifndef DBT_CODEC_H
#define DBT_CODEC_H
#include "dbt_avdtp_sink.h"

#define DBT_CODEC_CIE_BYTES 26u
struct dbt_codec_ops;
typedef struct {
    const struct dbt_codec_ops *ops;
    uint32_t rate;
    uint16_t samples;
    uint8_t channels, sample_bytes, cie_size;
    uint8_t cie[DBT_CODEC_CIE_BYTES];
} dbt_codec_config_t;
typedef struct {
    void *context;
    /* Return an eight-byte aligned allocation. Codec state is worker-owned. */
    uint8_t *(*allocate)(void *context,uint32_t bytes);
    void (*release)(void *context,uint8_t *bytes);
    dbt_result_t (*pcm)(void *context,const void *samples,uint16_t frames,
        const dbt_codec_config_t *config,uint32_t timestamp);
    /* PCM returns OK after consuming/copying the entire view. PENDING accepts
     * a borrowed view: caller retains this buffer and the media input until
     * resume/reset/close, and must not overwrite it. BUSY/error does not accept
     * it and retains the legacy reset-on-error semantics. No callback waits. */
} dbt_codec_hooks_t;
typedef struct {
    dbt_codec_hooks_t hooks;
    dbt_codec_config_t config;
    void *state;
    uint32_t allocated_bytes,frames;
} dbt_codec_decoder_t;
typedef struct dbt_codec_ops {
    const char *name;
    /* Pure bounded CIE parsing. No allocation or decoder activation here. */
    dbt_result_t (*configure)(const uint8_t *cie,uint16_t size,dbt_codec_config_t *config);
    dbt_result_t (*open)(dbt_codec_decoder_t *decoder);
    void (*close)(dbt_codec_decoder_t *decoder);
    dbt_result_t (*reset)(dbt_codec_decoder_t *decoder);
    dbt_result_t (*media)(dbt_codec_decoder_t *decoder,const dbt_avdtp_media_t *packet,uint32_t now_ms);
    void (*expire)(dbt_codec_decoder_t *decoder,uint32_t now_ms);
    /* Continue after accepted PENDING PCM has been consumed. Emits at most one
     * further view when that callback again returns PENDING. No input replay.
     * A caller keeps the original media bytes live until OK/error/reset/close. */
    dbt_result_t (*resume)(dbt_codec_decoder_t *decoder,uint32_t now_ms);
    /* media/resume may return DBT_PROGRESS after a bounded internal unit that
     * emitted no PCM. Input remains borrowed and resume is mandatory. Unlike
     * PENDING, this owns no output view. The codec must advance its input or
     * finite internal state; waiting for another owner is not progress. */
    /* Decoder-owner query only. False means no fragment expiry is armed. */
    bool (*deadline)(const dbt_codec_decoder_t *decoder,uint32_t *when_ms);
} dbt_codec_ops_t;

dbt_result_t dbt_codec_init(dbt_codec_decoder_t *decoder,dbt_codec_hooks_t hooks);
/* Always releases the old codec before constructing a different working set. */
dbt_result_t dbt_codec_open(dbt_codec_decoder_t *decoder,const dbt_codec_config_t *config);
void dbt_codec_close(dbt_codec_decoder_t *decoder);
/* Accounted worker-only allocations; eight-byte headers are included in the
 * requested byte count. Immutable tables remain in the codec's flash rodata. */
uint8_t *dbt_codec_allocate(dbt_codec_decoder_t *decoder,uint32_t bytes);
void dbt_codec_release(dbt_codec_decoder_t *decoder,void *bytes);

#define DBT_CODEC_WORKER_JOBS 4u
#define DBT_CODEC_PCM_CHUNK_FRAMES 256u
#define DBT_CODEC_PCM_STALL_MS 200u
typedef struct {
    uint32_t binding,token,epoch;
    dbt_sink_operation_t operation;
    dbt_codec_config_t config;
} dbt_codec_job_t;
typedef struct {uint32_t binding,token;dbt_sink_work_result_t result;} dbt_codec_receipt_t;
typedef struct {
    /* Begin copies the descriptor. PENDING owns a nonzero ticket; BUSY owns
     * nothing. Poll/cancel are single nonblocking attempts. Cancellation must
     * retire renderer side effects before returning a terminal result. */
    dbt_result_t (*begin)(void *,bool,const dbt_codec_config_t *,uint32_t *);
    dbt_result_t (*poll)(void *,uint32_t);
    dbt_result_t (*cancel)(void *,uint32_t);
} dbt_codec_stream_async_t;
typedef struct {
    /* PENDING with a nonzero ticket accepts an immutable PCM loan. BUSY or a
     * rejected begin accepts nothing. Only OK/CANCELLED poll proves the view
     * retired; INVALID/stale is never ownership proof. Cancel requests bounded
     * retirement, then poll acknowledges it. Wake the worker on completion. */
    dbt_result_t (*begin)(void *,const void *,uint16_t,const dbt_codec_config_t *,uint32_t,uint32_t *);
    dbt_result_t (*poll)(void *,uint32_t);
    dbt_result_t (*cancel)(void *,uint32_t);
} dbt_codec_pcm_async_t;

typedef enum {
    DBT_CODEC_INPUT_SEQUENCE=1, DBT_CODEC_INPUT_SSRC, DBT_CODEC_QUEUE_FULL,
    DBT_CODEC_WORKER_SEQUENCE, DBT_CODEC_WORKER_SSRC, DBT_CODEC_DECODE_REJECT,
    DBT_CODEC_RESET_FAILED, DBT_CODEC_PCM_FAILED, DBT_CODEC_PCM_TIMEOUT, DBT_CODEC_PCM_BUSY,
    DBT_CODEC_DECODE_BEGIN, DBT_CODEC_DECODE_END, DBT_CODEC_RESET_BEGIN, DBT_CODEC_RESET_END,
    DBT_CODEC_PCM_BEGIN, DBT_CODEC_PCM_END,
    /* Host-producer event; appended to preserve existing timing kind values. */
    DBT_CODEC_INPUT_CLOCK_MAX
} dbt_codec_trace_kind_t;
typedef struct {
    uint32_t timestamp,ssrc,previous_ssrc;
    uint16_t sequence,expected_sequence,bytes,queued;
    dbt_codec_trace_kind_t kind;
    dbt_result_t result;
    bool marker;
    uint8_t prefix[2];
    uint8_t payload_type;
} dbt_codec_trace_t;

/* Return PCM sample-frames (one sample per channel), not codec-frame units. */
typedef uint32_t (*dbt_codec_media_frames_t)(const dbt_codec_config_t *,const dbt_avdtp_media_t *);
/* Policy depth bit7 opts into aligned variable-size records within the same
 * physical slabs; low7 bits still select their power-of-two count. */
#define DBT_CODEC_QUEUE_PACKED 0x80u
#define DBT_CODEC_QUEUE_DEPTH_MASK 0x7fu
typedef struct {uint8_t depth,low_water;} dbt_codec_queue_t;
typedef struct {
    dbt_codec_hooks_t decoder;
    const dbt_codec_ops_t *const *codecs;
    uint8_t codec_count;
    void (*wake_worker)(void *context);
    void (*wake_owner)(void *context);
    dbt_result_t (*stream)(void *context,bool start,const dbt_codec_config_t *config);
    /* Requires resumable codecs and nonblocking providers. The default copies
     * chunks <=256 frames; BUSY consumes nothing. async_pcm instead retains an
     * immutable loan until acknowledged. Stream hooks must also be
     * nonblocking/transactional before product integration. */
    bool deferred_pcm;
    dbt_codec_stream_async_t async_stream;
    /* Optional bounded, nonblocking diagnostics. Called by host producer or
     * decoder consumer; the receiver must handle these two writers safely.
     * Only the first two media octets are exposed for format diagnosis;
     * no security material is exposed and media bytes are never modified. */
    void (*trace)(void *context,const dbt_codec_trace_t *event);
    dbt_codec_pcm_async_t async_pcm;
    /* Pure selected-codec policy, called only while admission is closed.
     * Every slab retains the full init maximum_payload; no truncated packets.
     * low_water bounds fixed-slot capacity wakeups. Packed mode also wakes
     * when a sealed physical slab retires, independently of packet count. */
    dbt_codec_queue_t (*queue_policy)(void *context,const dbt_codec_config_t *config);
} dbt_codec_worker_hooks_t;
typedef struct {
    dbt_codec_worker_hooks_t hooks;
    dbt_codec_decoder_t decoder;
    dbt_codec_job_t jobs[DBT_CODEC_WORKER_JOBS];
    dbt_codec_receipt_t receipts[DBT_CODEC_WORKER_JOBS];
    _Atomic uint32_t job_written,job_read,receipt_written,receipt_read;
    _Atomic uint32_t gate_epoch,started_epoch,media_written,media_read;
    /* Owner polls faults and retires the sink on change; DROP reclaims state. */
    _Atomic uint32_t packets,media_drops,resets,faults,resident_bytes;
    _Atomic uint32_t pcm_backpressure,pcm_timeouts,pcm_delivered_frames;
    _Atomic uint32_t queue_full_drops,decode_rejects,reset_failures,pcm_failures;
    _Atomic uint32_t input_sequence_breaks,input_ssrc_changes,queue_high_water;
    _Atomic uint32_t input_packets,input_payload_bytes,input_first_timestamp,input_last_timestamp,input_stream_epoch;
    /* One host writer; maximum forward half-range RTP step for adjacent wire
     * sequences and the same SSRC. A raw clock observation, never a loss count. */
    _Atomic uint32_t input_clock_revision,input_clock_step,input_clock_sequence,input_clock_timestamp,input_clock_ssrc;
    /* Host-producer diagnostics, distinct from queue saturation/decode errors.
     * Maximum includes a size-rejected packet; no payload is exposed. */
    _Atomic uint32_t input_max_payload,input_oversize_rejects,input_last_oversize;
    /* Atomic diagnostics/credit geometry: physical depth in bits0..6, packed
     * flag bit7, low_water8..15,
     * slot stride16..31. Zero while the selected working set is absent. */
    _Atomic uint32_t queue_layout;
    /* Packed mode: page counts protect physical ownership; packet counters
     * above still count logical packets. Each offset has one runtime owner. */
    _Atomic uint32_t media_page_written,media_page_read;
    uint16_t media_write_offset,media_read_offset;
    /* Duration metadata only: no per-slot growth. Producer counts accepted
     * frame samples; copied-PCM delivery retires them using its existing count. */
    dbt_codec_media_frames_t media_frames;
    _Atomic uint32_t duration_input_frames,duration_pcm_base,duration_reject_base,duration_epoch;
    /* Host-producer only; compare before queue admission so locally rejected
     * packets do not masquerade as an upstream sequence discontinuity. */
    uint32_t input_epoch,input_ssrc;
    uint16_t input_sequence;
    bool input_clock_prev_wire;
    uint8_t *packet_storage;
    uint32_t binding,last_ssrc;
    uint16_t maximum_payload,stride,last_sequence;
    uint8_t depth,depth_limit,low_water;
    bool configured,streaming,sequence_valid,packed_media;
    /* Worker-owned borrow, not another PCM allocation. media_read pins the
     * input slot until all output and the codec's saved frame cursor retire. */
    const uint8_t *pending_pcm;
    uint32_t pending_timestamp,last_progress;
    uint16_t pending_frames,pending_offset;
    bool media_active,output_blocked;
    uint32_t stream_ticket,control_since;
    uint8_t control_phase;
    bool control_blocked,control_timedout,cancel_requested;
    uint32_t pcm_ticket;
    uint16_t pcm_ticket_frames;
    bool pcm_cancel_requested,pcm_timedout,pcm_fault_latched;
} dbt_codec_worker_t;

/* One host producer and a separate decode consumer, with copied jobs/receipts
 * and a lazy shared encoded queue. Start/stop gate epochs cover every codec. */
dbt_result_t dbt_codec_worker_init(dbt_codec_worker_t *worker,dbt_codec_worker_hooks_t hooks,
    uint16_t maximum_payload,uint8_t depth);
/* Startup-only, before jobs/worker creation. Host-producer-only bounded
 * duration callback; zero means unknown/inactive. It may inspect host-owned
 * negotiated features, never decoder state, allocation or another owner. */
dbt_result_t dbt_codec_worker_set_media_frames(dbt_codec_worker_t *,dbt_codec_media_frames_t);
/* Any observer: one attempt, no slots/pointers borrowed. Changed producer
 * counts invalidate the snapshot without retry. Available only
 * for copied deferred PCM in a stable epoch with no decode rejection. It
 * estimates encoded/in-flight samples not yet accepted by the PCM provider. */
bool dbt_codec_worker_buffered_frames(const dbt_codec_worker_t *,uint32_t *);
dbt_result_t dbt_codec_worker_submit(void *worker,const dbt_sink_job_t *job);
/* Host input remains valid for the call and is disjoint from packet_storage. */
dbt_result_t dbt_codec_worker_media(void *worker,const dbt_avdtp_media_t *packet);
/* Host receive-credit query: conservative count of full maximum-payload
 * admissions. Packed open-page tails never count as a full free slab. */
uint16_t dbt_codec_worker_free_slots(const dbt_codec_worker_t *worker);
dbt_result_t dbt_codec_worker_receipt(dbt_codec_worker_t *worker,dbt_codec_receipt_t *receipt);
/* First service at most one accepted PCM-loan completion/cancellation so codec
 * teardown cannot overtake it. Then at most four control receipts and the
 * requested media work budget. Legacy
 * mode counts packet operations (including a DBT_PROGRESS continuation);
 * deferred mode counts one codec emission/resume or one
 * <=256-frame PCM chunk. A BUSY provider is attempted once and then returns. */
uint16_t dbt_codec_worker_step(dbt_codec_worker_t *worker,uint32_t now_ms,uint16_t media_budget);
/* Worker-only scheduling queries. Arm one deadline/notification wait when not
 * runnable. The render provider wakes the worker when space becomes available. */
bool dbt_codec_worker_runnable(const dbt_codec_worker_t *worker);
bool dbt_codec_worker_deadline(const dbt_codec_worker_t *worker,uint32_t *when_ms);
typedef struct {
    uint32_t revision,epoch,step_ticks,timestamp,ssrc;
    uint16_t sequence;
    bool coherent,observed;
} dbt_codec_input_clock_t;
/* One sequence check; no retry, allocation, timer or worker-state access.
 * observed=false means no eligible pair in the current input epoch. */
void dbt_codec_worker_clock_snapshot(const dbt_codec_worker_t *,dbt_codec_input_clock_t *);
#endif
