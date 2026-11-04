#ifndef DBT_RFCOMM_DLC_H
#define DBT_RFCOMM_DLC_H
#include "dbt_rfcomm_credit.h"
/* One active DLC on an already-open RFCOMM multiplexer. The owning bearer
 * supplies a fixed TX frame buffer and serializes all calls. */
typedef enum { DBT_RFCOMM_DLC_IDLE,DBT_RFCOMM_DLC_PN_QUEUED,DBT_RFCOMM_DLC_WAIT_PN,
    DBT_RFCOMM_DLC_PN_REPLY,DBT_RFCOMM_DLC_WAIT_SABM,DBT_RFCOMM_DLC_SABM_QUEUED,
    DBT_RFCOMM_DLC_WAIT_UA,DBT_RFCOMM_DLC_UA_QUEUED,DBT_RFCOMM_DLC_MSC,
    DBT_RFCOMM_DLC_OPEN,DBT_RFCOMM_DLC_DISC_QUEUED,DBT_RFCOMM_DLC_WAIT_DISC,
    DBT_RFCOMM_DLC_CLOSE_REPLY,DBT_RFCOMM_DLC_FAILED } dbt_rfcomm_dlc_state_t;
typedef enum { DBT_RFCOMM_DLC_EVENT_OPEN,DBT_RFCOMM_DLC_EVENT_CLOSED,DBT_RFCOMM_DLC_EVENT_FAILED } dbt_rfcomm_dlc_event_t;
typedef struct { void *context;void (*event)(void *,dbt_rfcomm_dlc_event_t,uint8_t);
    void (*data)(void *,uint8_t,const uint8_t *,uint16_t); } dbt_rfcomm_dlc_hooks_t;
typedef struct {
    dbt_rfcomm_dlc_hooks_t hooks;dbt_rfcomm_credit_t credits;
    uint8_t *tx;uint16_t capacity,tx_size,max_frame,proposed,listener_max;
    uint32_t deadline,t1_ms,t2_ms;
    uint8_t dlci,server_channel,initial_credits,tx_effect,line_status;
    uint8_t last_pn[8],last_pn_reply[8];
    dbt_rfcomm_dlc_state_t state;
    bool mux_initiator,server,credit_mode,tx_ready,tx_pinned,in_callback;
    bool msc_sent,msc_received,msc_ack,msc_reply,remote_flow_off,aggregate_flow_off;
} dbt_rfcomm_dlc_t;
dbt_result_t dbt_rfcomm_dlc_init(dbt_rfcomm_dlc_t *,bool mux_initiator,uint8_t *tx,uint16_t capacity,dbt_rfcomm_dlc_hooks_t);
dbt_result_t dbt_rfcomm_dlc_listen(dbt_rfcomm_dlc_t *,uint8_t server_channel,uint16_t max_frame,uint8_t initial_credits);
dbt_result_t dbt_rfcomm_dlc_open(dbt_rfcomm_dlc_t *,uint8_t server_channel,uint16_t max_frame,uint8_t initial_credits);
dbt_result_t dbt_rfcomm_dlc_receive(dbt_rfcomm_dlc_t *,const uint8_t *,uint16_t);
/* Copies one complete application frame to the owner-supplied TX buffer. The
 * bearer reports admission or abort; credit counters advance on commit only. */
dbt_result_t dbt_rfcomm_dlc_send(dbt_rfcomm_dlc_t *,const uint8_t *,uint16_t);
dbt_result_t dbt_rfcomm_dlc_grant(dbt_rfcomm_dlc_t *,uint8_t credits);
dbt_result_t dbt_rfcomm_dlc_close(dbt_rfcomm_dlc_t *);
dbt_result_t dbt_rfcomm_dlc_peek(dbt_rfcomm_dlc_t *,const uint8_t **,uint16_t *);
dbt_result_t dbt_rfcomm_dlc_commit(dbt_rfcomm_dlc_t *,uint32_t now_ms);
dbt_result_t dbt_rfcomm_dlc_cancel(dbt_rfcomm_dlc_t *);
/* The bearer may abandon a copied data frame that has not been submitted to
 * the runtime, e.g. to service a legacy flow-off command. */
bool dbt_rfcomm_dlc_queued_data(const dbt_rfcomm_dlc_t *);
dbt_result_t dbt_rfcomm_dlc_abort_queued_data(dbt_rfcomm_dlc_t *);
void dbt_rfcomm_dlc_poll(dbt_rfcomm_dlc_t *,uint32_t now_ms);
/* Owner calls after the transport has returned every pinned view. */
dbt_result_t dbt_rfcomm_dlc_shutdown(dbt_rfcomm_dlc_t *,bool link_lost);
#endif
