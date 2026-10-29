#ifndef DBT_ERTM_ENGINE_H
#define DBT_ERTM_ENGINE_H
#include "dbt_ertm.h"
typedef struct {
    uint16_t mtu,mps,retransmission_ms,monitor_ms;
    uint8_t window,max_transmit;
} dbt_ertm_parameters_t;
typedef struct {
    void *context;
    uint8_t *(*allocate)(void *context,uint16_t size);
    void (*release)(void *context,uint8_t *bytes);
    void (*sdu)(void *context,const uint8_t *data,uint16_t size);
} dbt_ertm_hooks_t;
struct dbt_ertm_slot;
typedef struct {
    dbt_ertm_hooks_t hooks;
    dbt_ertm_parameters_t local,peer;
    struct dbt_ertm_slot *slots;
    const uint8_t *transmit;
    uint8_t *partial;
    dbt_ertm_frame_t prepared;
    uint64_t retransmit;
    uint32_t retrans_at,monitor_at,final_epoch,prepared_final_epoch,final_served_epoch;
    uint32_t rx_absolute,ack_absolute,prepared_ack_absolute,rej_absolute,ambiguous_frames;
    uint32_t rx_frames,tx_frames,rx_sdus,tx_sdus,bad_fcs,ignored_frames;
    uint16_t tx_size,tx_cursor,rx_size,rx_cursor,poll_attempts;
    uint16_t local_cid,remote_cid;
    uint8_t capacity,head,count,next_tx,expected_ack,expected_rx,last_ack;
    uint8_t prepared_kind,prepared_seq,selective_seq,final_seq;
    bool active,failed,closing,in_callback,fcs,tx_active,generated;
    bool local_busy,remote_busy,rnr_sent,ack_needed,rej_needed,rej_sent;
    bool final_needed,final_selective,poll_needed,waiting_final,retrans_timer,monitor_timer;
    bool rej_actioned,srej_actioned,clear_busy_pending,force_ready_ack,prepared_busy,prepared_ready_ack;
} dbt_ertm_engine_t;
/* Single-owner, already-negotiated Enhanced-Control ERTM. Init allocates no
 * payloads. REJ receive recovery; peer SREJ honored. Local timers are actual
 * non-flushable sender values; peer MaxTransmit limits our attempts (0=infinite).
 * Negotiation/runtime are separate. A failed engine requires channel closure. */
dbt_result_t dbt_ertm_engine_init(dbt_ertm_engine_t *engine,dbt_ertm_hooks_t hooks,
    uint16_t local_cid,uint16_t remote_cid,dbt_ertm_parameters_t local,
    dbt_ertm_parameters_t peer,bool fcs);
/* BUSY retains a prepared pin and TX loan. Return/cancel the pin before
 * destroying the engine or reusing borrowed SDU storage. */
dbt_result_t dbt_ertm_engine_close(dbt_ertm_engine_t *engine);
/* Call only after the transport no longer borrows the prepared frame. */
dbt_result_t dbt_ertm_engine_cancel_frame(dbt_ertm_engine_t *engine);
/* Borrow one SDU through peer ACK and any pinned retransmission. Only window
 * metadata allocates; no TX payload copy. Reinitialize only after close. */
dbt_result_t dbt_ertm_engine_send(dbt_ertm_engine_t *engine,const uint8_t *data,uint16_t size);
dbt_result_t dbt_ertm_engine_view(dbt_ertm_engine_t *engine,dbt_ertm_frame_t *frame);
dbt_result_t dbt_ertm_engine_peek(dbt_ertm_engine_t *engine,uint8_t *output,uint16_t capacity,uint16_t *written);
/* Commit only after all ACL fragments of the prepared I/S frame are admitted.
 * Repeated views are stable; RX may run while a frame remains pinned. */
dbt_result_t dbt_ertm_engine_commit(dbt_ertm_engine_t *engine,uint32_t now_ms);
dbt_result_t dbt_ertm_engine_receive(dbt_ertm_engine_t *engine,const uint8_t *bytes,uint16_t size,uint32_t now_ms);
dbt_result_t dbt_ertm_engine_busy(dbt_ertm_engine_t *engine,bool busy);
void dbt_ertm_engine_poll(dbt_ertm_engine_t *engine,uint32_t now_ms);
uint32_t dbt_ertm_engine_wait_ms(const dbt_ertm_engine_t *engine,uint32_t now_ms);
#endif
