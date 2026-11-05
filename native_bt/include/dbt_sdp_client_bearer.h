#ifndef DBT_SDP_CLIENT_BEARER_H
#define DBT_SDP_CLIENT_BEARER_H
#include "dbt_sdp_client.h"
#include "dbt_runtime.h"

typedef struct {
    dbt_runtime_t *runtime;
    dbt_sdp_client_t *client;
    uint32_t serial,generation;
    uint16_t handle,cid,config_epoch,transaction;
    bool active,attached,closing,send_blocked;
} dbt_sdp_client_bearer_t;

/* Bind a fresh client to the caller-selected OPEN SDP client channel. This
 * does not register a duplicate PSM, open a channel or initiate a query.
 * Route matches here before the SDP server, including while closing. The
 * runtime/client/buffers/bearer stay alive until clear succeeds. The bearer
 * exclusively owns the client's transport peek/done calls after binding. */
dbt_result_t dbt_sdp_client_bearer_init(dbt_sdp_client_bearer_t *,dbt_runtime_t *,
                                      dbt_sdp_client_t *,const dbt_channel_t *);
bool dbt_sdp_client_bearer_matches(const dbt_sdp_client_bearer_t *,const dbt_channel_t *);
/* Receive/event are callback-safe: they mark state, never mutate channels.
 * A bound SDP protocol error closes this channel through poll, not the host. */
dbt_result_t dbt_sdp_client_bearer_receive(dbt_sdp_client_bearer_t *,const dbt_channel_t *,
                                         const uint8_t *,uint16_t,uint32_t now_ms);
void dbt_sdp_client_bearer_event(dbt_sdp_client_bearer_t *,dbt_channel_event_t,const dbt_channel_t *);
/* Poll outside runtime/channel callbacks; only this method mutates channels. */
dbt_result_t dbt_sdp_client_bearer_poll(dbt_sdp_client_bearer_t *,uint32_t now_ms);
uint32_t dbt_sdp_client_bearer_wait_ms(const dbt_sdp_client_bearer_t *,uint32_t now_ms);
/* Nonblocking request; poll drives actual channel closure. Clear remains BUSY
 * until channel retirement and the exact runtime TX callback release storage. */
dbt_result_t dbt_sdp_client_bearer_close(dbt_sdp_client_bearer_t *);
dbt_result_t dbt_sdp_client_bearer_clear(dbt_sdp_client_bearer_t *);
#endif
