#ifndef DBT_SBC_WORKER_H
#define DBT_SBC_WORKER_H
#include "dbt_sbc_decoder.h"
#include "dbt_avdtp_sink.h"

#define DBT_SBC_WORKER_JOBS 4u
typedef struct {
    uint32_t binding,token,epoch;
    dbt_sink_operation_t operation;
    dbt_sbc_config_t configuration;
} dbt_sbc_work_t;
typedef struct {
    uint32_t binding,token;
    dbt_sink_work_result_t result;
} dbt_sbc_receipt_t;
typedef struct {
    dbt_sbc_decoder_hooks_t decoder;
    void (*wake_worker)(void *context);
    void (*wake_owner)(void *context);
    /* Decoder worker only. Stop must discard/drain old PCM copies before READY.
     * A missing callback is suitable only for a PCM consumer without playback. */
    dbt_result_t (*stream)(void *context,bool started);
} dbt_sbc_worker_hooks_t;
typedef struct {
    dbt_sbc_worker_hooks_t hooks;
    dbt_sbc_decoder_t decoder;
    dbt_sbc_media_t media;
    dbt_sbc_work_t jobs[DBT_SBC_WORKER_JOBS];
    dbt_sbc_receipt_t receipts[DBT_SBC_WORKER_JOBS];
    _Atomic uint32_t job_written,job_read,receipt_written,receipt_read;
    _Atomic uint32_t gate_epoch,started_epoch,media_written,media_read;
    _Atomic uint32_t packets,media_drops,resets,resident_bytes;
    uint8_t *packet_storage;
    uint32_t binding,last_ssrc;
    uint16_t maximum_payload,stride,last_sequence;
    uint8_t depth;
    bool configured,streaming,sequence_valid;
} dbt_sbc_worker_t;

/* Init before starting either owner. Allocates nothing. Exactly one host owner
 * submits jobs/media and drains receipts; a distinct decoder worker calls step.
 * Control jobs and receipts are copied bounded SPSC queues. Encoded media storage
 * is allocated only on configuration, and freed only after a host-submitted
 * control barrier. A gate epoch prevents an older Start from reopening ingress
 * after Stop/Abort/Drop, even if those operations overlap in time.
 * No task is started here: run step() in the application's dedicated task and
 * use the wake hooks. No direct reinitialization until DROP receipt is consumed. */
dbt_result_t dbt_sbc_worker_init(dbt_sbc_worker_t *worker,dbt_sbc_worker_hooks_t hooks,
    uint16_t maximum_payload,uint8_t queue_depth);
dbt_result_t dbt_sbc_worker_submit(void *worker,const dbt_sink_job_t *job);
dbt_result_t dbt_sbc_worker_media(void *worker,const dbt_avdtp_media_t *media);
dbt_result_t dbt_sbc_worker_receipt(dbt_sbc_worker_t *worker,dbt_sbc_receipt_t *receipt);
/* Bounded controls first, then media_budget packets. No controller/host-loop call. */
uint16_t dbt_sbc_worker_step(dbt_sbc_worker_t *worker,uint32_t now_ms,uint16_t media_budget);

#endif
