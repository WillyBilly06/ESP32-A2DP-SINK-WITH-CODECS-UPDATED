#ifndef DBT_SMP_BEARER_H
#define DBT_SMP_BEARER_H
#include "dbt_smp.h"
#include "dbt_smp_sc.h"
/* Owner-only adapter for the LE SMP fixed L2CAP channel. Input is consumed
 * synchronously after bounded controller ingress and L2CAP reassembly; no
 * controller callback may call this. The output pointer must be copied by the
 * transport before processing another owner event. */
#define DBT_SMP_LE_CID 0x0006u
typedef struct { dbt_smp_t engine; uint8_t pending[17],pending_size; } dbt_smp_bearer_t;
dbt_result_t dbt_smp_bearer_open(dbt_smp_bearer_t *b,uint16_t handle,uint32_t generation,
    uint8_t role,uint8_t local_type,const uint8_t local[6],uint8_t peer_type,
    const uint8_t peer[6],uint8_t min_key_size,bool allow_just_works);
void dbt_smp_bearer_close(dbt_smp_bearer_t *b);
dbt_result_t dbt_smp_bearer_receive(dbt_smp_bearer_t *b,uint16_t handle,uint32_t generation,
    uint16_t cid,const uint8_t *pdu,uint16_t size,uint32_t now);
dbt_result_t dbt_smp_bearer_tx(dbt_smp_bearer_t *b,uint16_t handle,uint32_t generation,
    const uint8_t **pdu,uint8_t *size);
dbt_result_t dbt_smp_bearer_tx_done(dbt_smp_bearer_t *b,uint16_t handle,uint32_t generation,bool admitted);
dbt_result_t dbt_smp_bearer_pump(dbt_smp_bearer_t *b,uint16_t handle,uint32_t generation,uint32_t now);
dbt_result_t dbt_smp_bearer_dist_tx(dbt_smp_bearer_t *b,uint16_t handle,uint32_t generation,
    const uint8_t **pdu,uint8_t *size,uint32_t now);
dbt_result_t dbt_smp_bearer_dist_token(const dbt_smp_bearer_t *b,uint16_t handle,uint32_t generation,uint32_t *serial);
dbt_result_t dbt_smp_bearer_dist_admitted(dbt_smp_bearer_t *b,uint16_t handle,uint32_t generation,
    uint32_t serial,bool admitted,uint32_t now);
dbt_result_t dbt_smp_bearer_dist_ack(dbt_smp_bearer_t *b,uint16_t handle,uint32_t generation,
    uint32_t serial,bool acknowledged,uint32_t now);

/* SC adapter owns an SC session; feature exchange remains the caller's job.
 * For phase 3, admission is never an acknowledgment. Intermediate PDUs advance
 * on ordered admission; a local final key requires a separate correlated ack. */
/* One retained inbound PDU per link. DBT_OK means consumed or copied for
 * retry; DBT_BUSY means the caller still owns an additional PDU and must apply
 * bounded backpressure or disconnect. */
typedef struct { dbt_smp_sc_t engine; uint8_t pending[65],pending_size; } dbt_smp_sc_bearer_t;
void dbt_smp_sc_bearer_close(dbt_smp_sc_bearer_t *b);
dbt_result_t dbt_smp_sc_bearer_pump(dbt_smp_sc_bearer_t *b,uint16_t handle,uint32_t generation,uint32_t now);
dbt_result_t dbt_smp_sc_bearer_receive(dbt_smp_sc_bearer_t *b,uint16_t handle,uint32_t generation,
    uint16_t cid,const uint8_t *pdu,uint16_t size,uint32_t now);
dbt_result_t dbt_smp_sc_bearer_tx(dbt_smp_sc_bearer_t *b,uint16_t handle,uint32_t generation,
    const uint8_t **pdu,uint8_t *size,uint32_t now);
dbt_result_t dbt_smp_sc_bearer_dist_token(const dbt_smp_sc_bearer_t *b,uint16_t handle,uint32_t generation,uint32_t *serial);
dbt_result_t dbt_smp_sc_bearer_tx_admitted(dbt_smp_sc_bearer_t *b,uint16_t handle,uint32_t generation,
    bool admitted,uint32_t now);
dbt_result_t dbt_smp_sc_bearer_dist_admitted(dbt_smp_sc_bearer_t *b,uint16_t handle,uint32_t generation,
    uint32_t serial,bool admitted,uint32_t now);
dbt_result_t dbt_smp_sc_bearer_dist_ack(dbt_smp_sc_bearer_t *b,uint16_t handle,uint32_t generation,
    uint32_t serial,bool acknowledged,uint32_t now);
#endif
