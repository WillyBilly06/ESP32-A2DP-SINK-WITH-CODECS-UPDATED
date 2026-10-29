#ifndef DBT_CODEC_H
#define DBT_CODEC_H
#include "dbt_avdtp_sink.h"

#define DBT_CODEC_CIE_BYTES 16u
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
typedef struct {
    uint32_t binding,token,epoch;
    dbt_sink_operation_t operation;
    dbt_codec_config_t config;
} dbt_codec_job_t;
typedef struct {uint32_t binding,token;dbt_sink_work_result_t result;} dbt_codec_receipt_t;
typedef struct {
    dbt_codec_hooks_t decoder;
    const dbt_codec_ops_t *const *codecs;
    uint8_t codec_count;
    void (*wake_worker)(void *context);
    void (*wake_owner)(void *context);
    dbt_result_t (*stream)(void *context,bool start,const dbt_codec_config_t *config);
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
    uint8_t *packet_storage;
    uint32_t binding,last_ssrc;
    uint16_t maximum_payload,stride,last_sequence;
    uint8_t depth;
    bool configured,streaming,sequence_valid;
} dbt_codec_worker_t;

/* One host producer and a separate decode consumer, with copied jobs/receipts
 * and a lazy shared encoded queue. Start/stop gate epochs cover every codec. */
dbt_result_t dbt_codec_worker_init(dbt_codec_worker_t *worker,dbt_codec_worker_hooks_t hooks,
    uint16_t maximum_payload,uint8_t depth);
dbt_result_t dbt_codec_worker_submit(void *worker,const dbt_sink_job_t *job);
dbt_result_t dbt_codec_worker_media(void *worker,const dbt_avdtp_media_t *packet);
dbt_result_t dbt_codec_worker_receipt(dbt_codec_worker_t *worker,dbt_codec_receipt_t *receipt);
uint16_t dbt_codec_worker_step(dbt_codec_worker_t *worker,uint32_t now_ms,uint16_t media_budget);
#endif
