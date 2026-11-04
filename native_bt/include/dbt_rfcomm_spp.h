#ifndef DBT_RFCOMM_SPP_H
#define DBT_RFCOMM_SPP_H
#include "dbt_rfcomm_bearer.h"
#include "dbt_sdp.h"
/* Zero-initialize before first use and keep alive for the runtime lifetime.
 * Optional one-session SPP endpoint. The host application MUST forward its
 * accept/event/data hooks and call poll after runtime_poll; the SDP record is
 * borrowed and may be installed only while this endpoint and its hook routing
 * remain active. The service is not automatically inserted into any database. */
typedef struct {
    dbt_runtime_t *runtime;dbt_rfcomm_bearer_t bearer;dbt_rfcomm_dlc_hooks_t hooks;
    dbt_sdp_attribute_t attributes[4];dbt_sdp_record_t record;
    uint8_t protocols[14],early[4],scn,initial_credits;
    uint16_t max_frame,pending_handle,pending_cid;uint32_t pending_generation,pending_serial;
    struct {uint16_t handle,cid;uint32_t generation,serial;} rejected[DBT_CHANNEL_LIMIT];
    bool installed,pending,early_valid,bound,listen_set,failed;
} dbt_rfcomm_spp_t;
dbt_result_t dbt_rfcomm_spp_init(dbt_rfcomm_spp_t *,dbt_runtime_t *,uint8_t server_channel,
    uint16_t l2cap_mtu,uint16_t max_frame,uint8_t initial_credits,uint32_t record_handle,dbt_rfcomm_dlc_hooks_t);
bool dbt_rfcomm_spp_accept(const dbt_rfcomm_spp_t *,const dbt_link_t *,uint16_t psm);
void dbt_rfcomm_spp_event(dbt_rfcomm_spp_t *,dbt_channel_event_t,const dbt_channel_t *);
dbt_result_t dbt_rfcomm_spp_receive(dbt_rfcomm_spp_t *,const dbt_channel_t *,const uint8_t *,uint16_t);
void dbt_rfcomm_spp_poll(dbt_rfcomm_spp_t *,uint32_t now_ms);
const dbt_sdp_record_t *dbt_rfcomm_spp_record(const dbt_rfcomm_spp_t *);
#endif
