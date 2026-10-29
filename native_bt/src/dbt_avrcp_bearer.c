#include "dbt_avrcp_bearer.h"
static bool paired(const dbt_avctp_bearer_t *b,const dbt_avrcp_t *p) {
    return b && b->attached && p && p->transport==b->signal && !p->transport->browsing;
}
dbt_result_t dbt_avrcp_bearer_receive(dbt_avctp_bearer_t *b,dbt_avrcp_t *p,
    const dbt_channel_t *channel,const uint8_t *bytes,uint16_t size) {
    if (!paired(b,p) || p->in_callback) return DBT_INVALID;
    uint32_t now=b->runtime->now_ms;
    if (dbt_avrcp_poll(p,now)!=DBT_OK) return DBT_INVALID;
    p->now=now; p->input_status=DBT_OK;
    dbt_result_t result=dbt_avctp_bearer_receive(b,channel,bytes,size);
    return result==DBT_OK ? p->input_status : result;
}
dbt_result_t dbt_avrcp_bearer_poll(dbt_avctp_bearer_t *b,dbt_avrcp_t *p,uint32_t now) {
    if (!paired(b,p) || dbt_avrcp_poll(p,now)!=DBT_OK) return DBT_INVALID;
    return dbt_avctp_bearer_poll(b,now);
}
uint32_t dbt_avrcp_bearer_wait_ms(const dbt_avctp_bearer_t *b,const dbt_avrcp_t *p,uint32_t now) {
    if (!paired(b,p) || p->failed) return 0;
    uint32_t wait=dbt_avctp_bearer_wait_ms(b,now), profile_wait=dbt_avrcp_wait_ms(p,now);
    return profile_wait<wait ? profile_wait : wait;
}
