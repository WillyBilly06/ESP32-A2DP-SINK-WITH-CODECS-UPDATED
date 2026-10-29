#ifndef DBT_AAC_H
#define DBT_AAC_H
#include "dbt_codec.h"
#define DBT_AAC_MAX_RAW_BYTES 1536u
#define DBT_AAC_MAX_MUX_BYTES 8192u
#define DBT_AAC_MAX_SUBFRAMES 64u
typedef struct {uint32_t bit_offset;uint16_t size;} dbt_aac_unit_t;
typedef struct {bool configured;uint8_t subframes;} dbt_aac_mux_t;

/* AAC-LC, 44.1/48 kHz, mono/stereo; preserve output on invalid selection. */
dbt_result_t dbt_aac_configure(const uint8_t *cie,uint16_t size,dbt_codec_config_t *config);
uint8_t dbt_a2dp_aac_validate(const dbt_avdtp_endpoint_t *endpoint,const dbt_avdtp_cap_t *codec);
/* Strict LATM version-0 / single-program single-layer AAC-LC transport.
 * All units and padding are validated before mux state or outputs are published.
 * Optional other-data/CRC and 960-sample/SBR/PCE modes are refused explicitly. */
dbt_result_t dbt_aac_latm_parse(dbt_aac_mux_t *mux,const dbt_codec_config_t *config,
    const uint8_t *data,uint16_t size,dbt_aac_unit_t *units,uint8_t capacity,uint8_t *count);
dbt_result_t dbt_aac_latm_copy(const uint8_t *data,uint16_t size,dbt_aac_unit_t unit,
    uint8_t *output,uint16_t capacity);
extern const dbt_codec_ops_t dbt_aac_codec;
#endif
