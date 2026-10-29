#ifndef DBT_OPUS_H
#define DBT_OPUS_H
#include "dbt_codec.h"

#define DBT_OPUS_FRAGMENT_LIMIT 6144u
/* Android Opus and PipeWire Opus 05, one or two output channels at 48 kHz. */
dbt_result_t dbt_opus_configure(const uint8_t *cie,uint16_t size,dbt_codec_config_t *config);
uint8_t dbt_a2dp_opus_validate(const dbt_avdtp_endpoint_t *endpoint,const dbt_avdtp_cap_t *codec);
extern const dbt_codec_ops_t dbt_opus_codec;
#endif
