#ifndef DBT_AVRCP_BEARER_H
#define DBT_AVRCP_BEARER_H
#include "dbt_avctp_bearer.h"
#include "dbt_avrcp.h"
/* Pair the already initialized profile with its already attached control
 * bearer. Both, and all callback state, remain live through quiescence. */
dbt_result_t dbt_avrcp_bearer_receive(dbt_avctp_bearer_t *,dbt_avrcp_t *,const dbt_channel_t *,const uint8_t *,uint16_t);
dbt_result_t dbt_avrcp_bearer_poll(dbt_avctp_bearer_t *,dbt_avrcp_t *,uint32_t);
uint32_t dbt_avrcp_bearer_wait_ms(const dbt_avctp_bearer_t *,const dbt_avrcp_t *,uint32_t);
#endif
