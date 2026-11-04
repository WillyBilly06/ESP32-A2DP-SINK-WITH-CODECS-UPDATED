#include "dbt_runtime.h"
#include "dbt_att.h"
#include "dbt_ertm_engine.h"
#include "dbt_audio.h"
#include "dbt_codec.h"
#include "dbt_smp_sc.h"
#include <stdint.h>

_Static_assert(sizeof(void *) == 4, "Measure ESP32 ABI, not desktop ABI");
const uint32_t dbt_audit_layout[] __attribute__((used)) = {
    sizeof(void *), sizeof(dbt_queue_t), sizeof(dbt_host_t), sizeof(dbt_acl_t),
    sizeof(dbt_runtime_t), sizeof(dbt_att_server_t), sizeof(dbt_att_connection_t),
    sizeof(dbt_ertm_engine_t), sizeof(dbt_codec_decoder_t), sizeof(dbt_audio_t),
    sizeof(dbt_smp_sc_t), DBT_QUEUE_BYTES, DBT_MAX_H4_BYTES,
    DBT_CONTROL_RESERVE_BYTES, DBT_MAX_LINKS, DBT_CODEC_WORKER_JOBS
};
