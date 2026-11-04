#ifndef DBT_AVCTP_BEARER_H
#define DBT_AVCTP_BEARER_H

#include "dbt_avctp.h"
#include "dbt_runtime.h"

typedef struct {
    dbt_runtime_t *runtime;
    dbt_avctp_t *signal;
    uint32_t serial, generation;
    uint16_t handle, cid;
    bool active, attached;
} dbt_avctp_bearer_t;

/* Attach an already-open Classic PSM 0x17 control channel. Browsing is deliberately rejected until an
 * ERTM-capable runtime bearer exists. This adapter does not register or
 * advertise a profile.
 * Signal binding must equal channel.serial, with matching negotiated MTUs. */
/* An MTU reconfiguration requires quiescing/clearing and a new binding. The
 * adapter fails closed on changed live MTUs rather than using stale limits. */
dbt_result_t dbt_avctp_bearer_init(dbt_avctp_bearer_t *bearer, dbt_runtime_t *runtime,
    dbt_avctp_t *signal, const dbt_channel_t *channel);
dbt_result_t dbt_avctp_bearer_receive(dbt_avctp_bearer_t *bearer,
    const dbt_channel_t *channel, const uint8_t *data, uint16_t size);
/* Forward CLOSED/FAILED after the runtime has returned any outgoing borrow. */
void dbt_avctp_bearer_event(dbt_avctp_bearer_t *bearer, dbt_channel_event_t event,
    const dbt_channel_t *channel);
dbt_result_t dbt_avctp_bearer_poll(dbt_avctp_bearer_t *bearer, uint32_t now_ms);
uint32_t dbt_avctp_bearer_wait_ms(const dbt_avctp_bearer_t *bearer, uint32_t now_ms);
/* Quiesce the runtime first. Returns BUSY until its outgoing borrow is returned. */
dbt_result_t dbt_avctp_bearer_clear(dbt_avctp_bearer_t *bearer);

#endif
