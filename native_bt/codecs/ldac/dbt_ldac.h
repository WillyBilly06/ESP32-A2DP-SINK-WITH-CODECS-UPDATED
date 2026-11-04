#ifndef DBT_LDAC_H
#define DBT_LDAC_H
#include "dbt_codec.h"
#define DBT_LDAC_MAX_FRAMES 15u
typedef struct {uint16_t offset,size;} dbt_ldac_frame_t;
/* The descriptor is an offline candidate and requires the generated SDK port. */
dbt_result_t dbt_ldac_configure(const uint8_t *cie,uint16_t size,dbt_codec_config_t *config);
uint8_t dbt_a2dp_ldac_validate(const dbt_avdtp_endpoint_t *endpoint,const dbt_avdtp_cap_t *codec);
dbt_result_t dbt_ldac_packet_parse(const dbt_codec_config_t *config,const uint8_t *payload,uint16_t size,
    dbt_ldac_frame_t *frames,uint8_t capacity,uint8_t *count);
extern const dbt_codec_ops_t dbt_ldac_codec;
#endif
