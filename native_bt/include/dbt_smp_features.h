#ifndef DBT_SMP_FEATURES_H
#define DBT_SMP_FEATURES_H
#include "dbt_queue.h"
#include "dbt_smp_sc.h"
#include "dbt_smp.h"
#include "dbt_smp_guard.h"
/* LE feature exchange, owner only. SC negotiates identity or CSRK masks;
 * legacy bonding requires both LTK masks. It rejects unsupported key masks,
 * unilateral OOB and security downgrades. */
typedef struct {
    uint8_t io,min_key_size,max_key_size,initiator_keys,responder_keys;
    bool support_sc,require_sc,bonding,require_mitm,oob;
} dbt_smp_feature_policy_t;
typedef enum { DBT_FEATURE_IDLE,DBT_FEATURE_RESPONSE,DBT_FEATURE_SEND_RESPONSE,
               DBT_FEATURE_READY,DBT_FEATURE_FAILED,DBT_FEATURE_TIMEOUT } dbt_smp_feature_state_t;
typedef struct {
    uint16_t handle;
    uint32_t generation,deadline;
    dbt_smp_feature_policy_t policy;
    dbt_smp_feature_state_t state;
    uint8_t preq[7],pres[7],tx[7],tx_size;
    bool initiator,used,secure_connections;
    bool claimed;
    dbt_smp_guard_t *guard;
    bool result_reported;
} dbt_smp_features_t;
dbt_result_t dbt_smp_features_open(dbt_smp_features_t *f,uint16_t handle,uint32_t generation,
    bool initiator,const dbt_smp_feature_policy_t *policy);
/* A guard is mandatory and must remain alive, owned by the host, and bound to
 * the claimed peer identity across links. */
dbt_result_t dbt_smp_features_set_guard(dbt_smp_features_t *f,dbt_smp_guard_t *guard,uint32_t now);
/* Report successful pairing only after the correlated encryption event (and,
 * when bonding, the required durable storage acknowledgment). Report abort or
 * failure otherwise. Exactly one result is accepted. */
dbt_result_t dbt_smp_features_report_result(dbt_smp_features_t *f,uint32_t generation,bool success,uint32_t now);
dbt_result_t dbt_smp_features_start(dbt_smp_features_t *f,uint32_t now);
dbt_result_t dbt_smp_features_receive(dbt_smp_features_t *f,const uint8_t *p,uint8_t n,uint32_t now);
void dbt_smp_features_poll(dbt_smp_features_t *f,uint32_t now);
dbt_result_t dbt_smp_features_tx(const dbt_smp_features_t *f,const uint8_t **p,uint8_t *n);
void dbt_smp_features_tx_done(dbt_smp_features_t *f,bool admitted,uint32_t now);
dbt_result_t dbt_smp_features_take(const dbt_smp_features_t *f,uint8_t preq[7],uint8_t pres[7],bool *sc);
/* After response admission, begin phase 2 on this exact transcript and link. */
dbt_result_t dbt_smp_features_open_sc(dbt_smp_features_t *f,dbt_smp_sc_t *s,
    uint8_t local_type,const uint8_t local[6],uint8_t peer_type,const uint8_t peer[6],uint32_t now);
dbt_result_t dbt_smp_features_resume_legacy(dbt_smp_features_t *f,dbt_smp_t *s,uint32_t now);
void dbt_smp_features_close(dbt_smp_features_t *f);
#endif
