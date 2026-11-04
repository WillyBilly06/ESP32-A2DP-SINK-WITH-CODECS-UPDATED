#ifndef DBT_RFCOMM_BEARER_H
#define DBT_RFCOMM_BEARER_H
#include "dbt_rfcomm_mux.h"
#include "dbt_rfcomm_dlc.h"
#include "dbt_runtime.h"
#define DBT_RFCOMM_BEARER_RX_SLOTS 8u
#define DBT_RFCOMM_BEARER_FRAME_MAX 256u
#define DBT_RFCOMM_BEARER_RX_CREDITS 4u
/* Zero-initialize before first bind. Do not rebind until closed and all runtime
 * completions have returned. Opt-in one-DLC adapter bound to an open PSM 3
 * Basic channel. The
 * owner routes channel data callbacks to enqueue and calls poll outside runtime
 * callbacks. Buffers are allocated only while this bearer is bound. */
typedef struct {
    dbt_rfcomm_mux_t mux;dbt_rfcomm_dlc_t dlc;dbt_runtime_t *runtime;
    uint32_t serial;uint8_t *rx[DBT_RFCOMM_BEARER_RX_SLOTS],*tx;
    uint16_t rx_size[DBT_RFCOMM_BEARER_RX_SLOTS],frame_capacity;
    uint8_t head,count;
    bool tx_active,tx_is_dlc,tx_is_app,app_pending,faulted,closed,in_callback;
    dbt_tx_done_t app_done;void *app_context;
} dbt_rfcomm_bearer_t;
dbt_result_t dbt_rfcomm_bearer_bind(dbt_rfcomm_bearer_t *,dbt_runtime_t *,uint16_t handle,uint16_t local_cid,bool initiator,uint32_t t1_ms);
dbt_result_t dbt_rfcomm_bearer_enqueue(dbt_rfcomm_bearer_t *,const dbt_channel_t *,const uint8_t *,uint16_t);
void dbt_rfcomm_bearer_poll(dbt_rfcomm_bearer_t *,uint32_t now_ms);
/* Hooks are borrowed and non-reentrant. Only one data DLC is supported. The
 * app must explicitly grant receive credits corresponding to available space. */
dbt_result_t dbt_rfcomm_bearer_listen(dbt_rfcomm_bearer_t *,uint8_t server_channel,uint16_t max_frame,uint8_t initial_credits,dbt_rfcomm_dlc_hooks_t);
dbt_result_t dbt_rfcomm_bearer_open(dbt_rfcomm_bearer_t *,uint8_t server_channel,uint16_t max_frame,uint8_t initial_credits,dbt_rfcomm_dlc_hooks_t);
/* Bytes are copied into the active bearer buffer; completion reports full
 * admission to HCI, not remote delivery. No overlap with another queued TX. */
dbt_result_t dbt_rfcomm_bearer_send(dbt_rfcomm_bearer_t *,const uint8_t *,uint16_t,dbt_tx_done_t,void *);
dbt_result_t dbt_rfcomm_bearer_grant(dbt_rfcomm_bearer_t *,uint8_t);
dbt_result_t dbt_rfcomm_bearer_close_dlc(dbt_rfcomm_bearer_t *);
#endif
