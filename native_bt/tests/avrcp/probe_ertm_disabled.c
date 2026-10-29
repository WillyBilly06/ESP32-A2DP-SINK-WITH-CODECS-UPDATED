#include "dbt_ertm_engine.h"
/* This harness exercises Basic mode only. An accidental ERTM path fails closed. */
dbt_result_t dbt_ertm_engine_init(dbt_ertm_engine_t *e,dbt_ertm_hooks_t h,uint16_t l,uint16_t r,dbt_ertm_parameters_t a,dbt_ertm_parameters_t b,bool f) {
    (void)e;(void)h;(void)l;(void)r;(void)a;(void)b;(void)f;return DBT_UNSUPPORTED;
}
dbt_result_t dbt_ertm_engine_close(dbt_ertm_engine_t *e) {(void)e;return DBT_UNSUPPORTED;}
dbt_result_t dbt_ertm_engine_cancel_frame(dbt_ertm_engine_t *e) {(void)e;return DBT_UNSUPPORTED;}
dbt_result_t dbt_ertm_engine_send(dbt_ertm_engine_t *e,const uint8_t *p,uint16_t n) {(void)e;(void)p;(void)n;return DBT_UNSUPPORTED;}
dbt_result_t dbt_ertm_engine_view(dbt_ertm_engine_t *e,dbt_ertm_frame_t *f) {(void)e;(void)f;return DBT_UNSUPPORTED;}
dbt_result_t dbt_ertm_engine_peek(dbt_ertm_engine_t *e,uint8_t *p,uint16_t n,uint16_t *w) {(void)e;(void)p;(void)n;(void)w;return DBT_UNSUPPORTED;}
dbt_result_t dbt_ertm_engine_commit(dbt_ertm_engine_t *e,uint32_t n) {(void)e;(void)n;return DBT_UNSUPPORTED;}
dbt_result_t dbt_ertm_engine_receive(dbt_ertm_engine_t *e,const uint8_t *p,uint16_t n,uint32_t t) {(void)e;(void)p;(void)n;(void)t;return DBT_UNSUPPORTED;}
void dbt_ertm_engine_poll(dbt_ertm_engine_t *e,uint32_t n) {(void)e;(void)n;}
uint32_t dbt_ertm_engine_wait_ms(const dbt_ertm_engine_t *e,uint32_t n) {(void)e;(void)n;return UINT32_MAX;}
