#ifndef DBT_RFCOMM_CREDIT_H
#define DBT_RFCOMM_CREDIT_H
#include "dbt_rfcomm.h"
/* Initial PN values only, for a previously unopened DLC; no renegotiation or
 * service admission is implied by decoding a syntactically valid PN message. */
typedef struct { uint16_t max_frame;uint8_t dlci,priority,credits;bool command; } dbt_rfcomm_pn_t;
dbt_result_t dbt_rfcomm_pn_decode(const dbt_rfcomm_mcc_t *m,dbt_rfcomm_pn_t *pn);
dbt_result_t dbt_rfcomm_pn_encode(const dbt_rfcomm_pn_t *pn,uint8_t output[8],dbt_rfcomm_mcc_t *m);
/* One DLC's credit counters. A prepare is a reservation, not admission. The
 * owner must retain the actual payload separately through bearer completion. */
typedef struct { uint16_t tx_available,rx_outstanding;uint8_t grant;bool pending,payload; } dbt_rfcomm_credit_t;
dbt_result_t dbt_rfcomm_credit_init(dbt_rfcomm_credit_t *c,uint8_t peer_initial,uint8_t local_initial);
dbt_result_t dbt_rfcomm_credit_prepare(dbt_rfcomm_credit_t *c,bool payload,uint8_t grant);
dbt_result_t dbt_rfcomm_credit_commit(dbt_rfcomm_credit_t *c);
dbt_result_t dbt_rfcomm_credit_cancel(dbt_rfcomm_credit_t *c);
/* Applying an incoming complete validated UIH consumes one local credit only
 * for a nonempty payload; its credit octet adds to the peer's TX allowance. */
dbt_result_t dbt_rfcomm_credit_receive(dbt_rfcomm_credit_t *c,bool payload,uint8_t grant);
#endif
