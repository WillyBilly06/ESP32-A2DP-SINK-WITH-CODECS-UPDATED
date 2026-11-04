#ifndef DBT_APTX_H
#define DBT_APTX_H
#include "dbt_codec.h"
dbt_result_t dbt_aptx_configure(const uint8_t *cie,uint16_t size,dbt_codec_config_t *config);
uint8_t dbt_a2dp_aptx_validate(const dbt_avdtp_endpoint_t *endpoint,const dbt_avdtp_cap_t *codec);
extern const dbt_codec_ops_t dbt_aptx_codec;
#endif
