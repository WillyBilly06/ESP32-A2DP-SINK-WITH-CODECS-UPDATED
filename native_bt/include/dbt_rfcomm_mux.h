#ifndef DBT_RFCOMM_MUX_H
#define DBT_RFCOMM_MUX_H
#include "dbt_rfcomm.h"
/* Independent control-channel lifecycle, bound to one Basic L2CAP channel and
 * one physical-link generation. This is NOT an RFCOMM service or DLC engine. */
typedef enum { DBT_RFCOMM_MUX_DOWN, DBT_RFCOMM_MUX_START_QUEUED,
    DBT_RFCOMM_MUX_WAIT_UA, DBT_RFCOMM_MUX_ACCEPT_QUEUED, DBT_RFCOMM_MUX_OPEN,
    DBT_RFCOMM_MUX_DISC_QUEUED, DBT_RFCOMM_MUX_WAIT_DISC, DBT_RFCOMM_MUX_REPLY_QUEUED } dbt_rfcomm_mux_state_t;
typedef struct {
    uint32_t generation,deadline,t1_ms;
    uint16_t handle,local_cid;
    dbt_rfcomm_mux_state_t state;
    uint8_t tx[4],tx_effect;
    bool initiator,tx_ready,tx_pinned,timed_out,disc_crossed;
} dbt_rfcomm_mux_t;
/* Timer range corresponds to the control-channel T1 range. */
dbt_result_t dbt_rfcomm_mux_init(dbt_rfcomm_mux_t *mux,bool initiator,uint16_t handle,
    uint32_t generation,uint16_t local_cid,uint32_t t1_ms);
dbt_result_t dbt_rfcomm_mux_start(dbt_rfcomm_mux_t *mux);
dbt_result_t dbt_rfcomm_mux_close(dbt_rfcomm_mux_t *mux);
/* RX frame and its identity are borrowed for the duration of this call.
 * BUSY means no state was changed and the caller must retain/retry the frame. */
dbt_result_t dbt_rfcomm_mux_receive(dbt_rfcomm_mux_t *mux,uint16_t handle,uint32_t generation,
    uint16_t local_cid,const uint8_t *data,uint16_t size);
dbt_result_t dbt_rfcomm_mux_peek(dbt_rfcomm_mux_t *mux,const uint8_t **data,uint16_t *size);
dbt_result_t dbt_rfcomm_mux_commit(dbt_rfcomm_mux_t *mux,uint32_t now_ms);
dbt_result_t dbt_rfcomm_mux_cancel(dbt_rfcomm_mux_t *mux);
void dbt_rfcomm_mux_poll(dbt_rfcomm_mux_t *mux,uint32_t now_ms);
/* On a matching link loss, clear only after the bearer has returned a pinned
 * view. BUSY means the caller must cancel/complete that view first. */
dbt_result_t dbt_rfcomm_mux_link_lost(dbt_rfcomm_mux_t *mux,uint16_t handle,uint32_t generation);
#endif
