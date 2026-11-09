/* Offline queue/provider shim exercising the real probe adapter and SMP manager. */
#ifdef NDEBUG
#error "SMP probe fixtures require active assertions"
#endif
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../../esp32_probe/main/probe_smp.c"

typedef struct { size_t size; int full; unsigned char *bytes; } queue_t;
static int worker_context, crypto_calls, store_calls, open_calls, close_calls, starts, fail_start, nvs_closes;
static BaseType_t requested_worker_core=DBT_ESP32_BACKGROUND_CORE;
QueueHandle_t xQueueCreate(UBaseType_t n,UBaseType_t size) {
    assert(n==1); queue_t *q=calloc(1,sizeof(*q)); assert(q); q->size=size; q->bytes=malloc(size); assert(q->bytes); return q;
}
BaseType_t xQueueSend(QueueHandle_t v,const void *item,TickType_t wait) {
    (void)wait; queue_t *q=v; if(q->full) return 0; memcpy(q->bytes,item,q->size); q->full=1; return pdTRUE;
}
BaseType_t xQueueReceive(QueueHandle_t v,void *item,TickType_t wait) {
    (void)wait; queue_t *q=v; if(!q->full) return 0; memcpy(item,q->bytes,q->size); memset(q->bytes,0,q->size); q->full=0; return pdTRUE;
}
UBaseType_t uxQueueMessagesWaiting(QueueHandle_t v) { return ((queue_t *)v)->full; }
void vQueueDelete(QueueHandle_t v) { queue_t *q=v; free(q->bytes); free(q); }
BaseType_t xTaskCreate(void (*fn)(void *),const char *name,uint32_t stack,void *p,UBaseType_t pri,TaskHandle_t *out) {
    (void)fn; (void)name; (void)p; (void)pri; (void)out; assert(stack==WORKER_STACK_BYTES);
    ++starts; return fail_start ? 0 : pdPASS;
}
BaseType_t xTaskCreatePinnedToCore(void (*fn)(void *),const char *name,uint32_t stack,void *p,UBaseType_t pri,TaskHandle_t *out,BaseType_t core) {
    assert(pri==1);
    assert(core==((!maintenance.active && slot.reconnect)?DBT_ESP32_RECONNECT_WORKER_CORE:DBT_ESP32_BACKGROUND_CORE));
    if(!maintenance.active && slot.reconnect)assert(core==0);else assert(core==1);
    requested_worker_core=core;return xTaskCreate(fn,name,stack,p,pri,out);
}
int xPortGetCoreID(void) { assert(worker_context);return requested_worker_core; }
void xTaskNotifyGive(TaskHandle_t h) { assert(h); }
void vTaskDelete(TaskHandle_t h) { (void)h; abort(); }
void esp_fill_random(void *p,size_t n) { assert(worker_context); memset(p,0x13,n); }
esp_err_t nvs_open(const char *name,int mode,nvs_handle_t *h) {
    assert(worker_context && (!strcmp(name,"native_le_bonds") || !strcmp(name,"native_cccd")) && mode==NVS_READWRITE); *h=1; return ESP_OK;
}
void nvs_close(nvs_handle_t h) { assert(worker_context && h==1); ++nvs_closes; }
#ifndef PROBE_REAL_NVS
bool probe_cccd_bond_tag(const dbt_smp_bond_record_t *r,uint8_t out[32]) {
    assert(worker_context && r);memset(out,0x23,32);return true;
}
void dbt_smp_nvs_init(dbt_smp_nvs_t *p,uint32_t h) { assert(worker_context); p->handle=h; p->poisoned=false; }
dbt_result_t dbt_smp_nvs_execute(dbt_smp_nvs_t *p,const dbt_smp_bond_job_t *j,dbt_smp_bond_record_t *out) {
    (void)p; (void)out; assert(worker_context); ++store_calls; return j->op==DBT_SMP_BOND_LOAD ? DBT_EMPTY : DBT_OK;
}
dbt_result_t dbt_smp_nvs_pending(dbt_smp_nvs_t *p,dbt_smp_bond_op_t *op,uint8_t *type,uint8_t id[6]) {
    (void)p; (void)op; (void)type; (void)id; assert(worker_context); return DBT_EMPTY;
}
dbt_result_t dbt_smp_nvs_reconcile(dbt_smp_nvs_t *p,dbt_smp_bond_op_t op,uint8_t type,const uint8_t id[6],bool *applied) {
    (void)p; (void)op; (void)type; (void)id; (void)applied; assert(worker_context); return DBT_EMPTY;
}
dbt_result_t dbt_smp_nvs_resolve_rpa(dbt_smp_nvs_t *p,const uint8_t rpa[6],dbt_smp_bond_record_t *out,bool *candidate) {
    (void)p; (void)rpa; (void)out; (void)candidate; assert(worker_context); return DBT_EMPTY;
}
bool dbt_smp_nvs_verify_rpa(const uint8_t rpa[6],const uint8_t irk[16],bool *matching) {
    assert(worker_context && rpa && irk && matching); *matching=rpa[0]==0xaa; return true;
}
#endif
bool dbt_smp_worker_mbedtls_open(dbt_smp_worker_mbedtls_t *w,uint32_t g,dbt_smp_worker_rng_fn rng,void *ctx) {
    assert(worker_context && g && rng); uint8_t v; assert(rng(ctx,&v,1)); w->internal=(void *)(uintptr_t)g; ++open_calls; return true;
}
void dbt_smp_worker_mbedtls_close(dbt_smp_worker_mbedtls_t *w) {
    assert(worker_context); if(w->internal) ++close_calls; w->internal=NULL;
}
bool dbt_smp_worker_mbedtls_sc(dbt_smp_worker_mbedtls_t *w,const dbt_smp_sc_job_t *j,dbt_smp_sc_result_t *o) {
    (void)o; assert(worker_context && (uintptr_t)w->internal==j->generation); ++crypto_calls; return false;
}
bool dbt_smp_worker_mbedtls_legacy(dbt_smp_worker_mbedtls_t *w,const dbt_smp_job_t *j,dbt_smp_job_result_t *o) {
    (void)o; assert(worker_context && (uintptr_t)w->internal==j->generation); ++crypto_calls; return false;
}
bool dbt_smp_worker_mbedtls_distribution(dbt_smp_worker_mbedtls_t *w,uint8_t m,uint8_t t,const uint8_t id[6],dbt_smp_dist_keys_t *o) {
    assert(worker_context && w->internal && m && t<2 && id); memset(o,0,sizeof(*o)); return true;
}
static worker_state_t fake_worker;
static void run_worker(void) {
    work_t w; completion_t c; uint8_t token;
    assert(xQueueReceive(work_queue,&token,0)==pdTRUE);
    w=work_mailbox; wipe(&work_mailbox,sizeof(work_mailbox));
    worker_context=1; worker_execute(&fake_worker,&w,&c); worker_context=0;
    if(w.kind==WORK_LOAD || w.kind==WORK_RESOLVE)assert(c.worker_core==(uint8_t)requested_worker_core);
    memset(&w,0,sizeof(w)); completion_mailbox=c; token=1;
    assert(xQueueSend(done_queue,&token,0)==pdTRUE); memset(&c,0,sizeof(c));
}
static uint8_t *allocate(void *ctx,uint16_t n) { (void)ctx; return malloc(n); }
static void release(void *ctx,uint8_t *p) { (void)ctx; free(p); }
static void fixed(void *ctx,const dbt_link_t *l,uint16_t cid,const uint8_t *p,uint16_t n) {
    (void)ctx; (void)l; (void)cid; (void)p; (void)n;
}
static dbt_runtime_t r;
int main(void) {
    r.ready=true; r.host.state=DBT_HOST_IDENTIFIED; r.host.timeout_ms=2000; r.maximum_mtu=128;
    r.hooks.allocate=allocate; r.hooks.release=release; r.channels.hooks.fixed_data=fixed;
    dbt_link_t *l=&r.acl.links[0]; l->used=true; l->kind=DBT_LINK_LE; l->handle=1; l->generation=1;
    l->role=1; l->peer_address_type=0; l->peer[0]=7;
    probe_smp_config_t cfg={0}; cfg.local_address[0]=1; cfg.pairing.allow_just_works=true;
    r.host.controller.address_le[0]=1;
    cfg.pairing.features.io=3; cfg.pairing.features.min_key_size=7; cfg.pairing.features.max_key_size=16;
    cfg.pairing.features.support_sc=true;
    assert(probe_smp_init(&r,(TaskHandle_t)1,&cfg)==DBT_OK);
    assert(probe_smp_quiescent());
    assert(starts==0 && !worker_live);
    assert(r.channels.fixed_mask[DBT_LINK_LE] & ((uint64_t)1<<6));
    uint8_t request[7]={1,3,0,8,16,0,0};
#ifdef DBT_SECURITY_REQUEST_TEST
    extern uint32_t test_prompt_window;
    extern bool test_admission_allowed,test_frozen;
    configuration.peripheral_security_requests=true;configuration.pairing.features.bonding=true;
    r.acl.mtu[DBT_LINK_LE]=27;test_prompt_window=1;r.now_ms=0;r.tx[0].active=true;
    probe_smp_poll(0);assert(request_for(1,1) && !request_for(1,1)->loan && !probe_smp_quiescent());
    r.tx[0].active=false;memset(security_requests,0,sizeof(security_requests));r.now_ms=10;
    probe_smp_poll(10);security_request_t *sr=request_for(1,1);
    assert(sr && sr->loan && r.tx[0].active && !probe_smp_quiescent());
    assert(r.tx[0].pdu.cid==6 && r.tx[0].pdu.size==2 && r.tx[0].pdu.data[0]==0x0b && r.tx[0].pdu.data[1]==9);
    assert(dbt_runtime_cancel_fixed(&r,1,2,6,security_request_done,sr)==DBT_EMPTY && r.tx[0].active);
    r.tx[0].pdu.cursor=1;assert(dbt_runtime_cancel_fixed(&r,1,1,6,security_request_done,sr)==DBT_BUSY && sr->loan);
    r.tx[0].pdu.cursor=0;test_prompt_window=0;r.now_ms=11;probe_smp_security_request_preflight(11);
    assert(!sr->loan && !r.tx[0].active && sr->outside_done && probe_smp_quiescent());
    probe_smp_poll(12);assert(!r.tx[0].active); /* expiration must not replay outside the window */
    test_prompt_window=2;probe_smp_poll(13);assert(sr->loan && r.tx[0].active);
    dbt_link_t *other=&r.acl.links[1];*other=(dbt_link_t){.used=true,.kind=DBT_LINK_LE,.role=1,.handle=2,.generation=9,.peer_address_type=0};
    probe_smp_poll(13);security_request_t *other_request=request_for(2,9);assert(other_request && other_request->loan && r.tx[1].active);
    test_admission_allowed=false;assert(probe_smp_receive(l,6,request,sizeof(request))==DBT_UNSUPPORTED);
    assert(!sr->loan && !r.tx[0].active && !slot.active); /* pairing cancelled prompt first */
    test_admission_allowed=true;assert(probe_smp_receive(l,6,request,sizeof(request))==DBT_OK && slot.active);
    assert(!other_request->loan && !r.tx[1].active && other->used); /* no second prompt during active SMP */
    memset(&slot,0,sizeof(slot));other->used=false;probe_smp_security_request_preflight(13);
    test_prompt_window=3;probe_smp_poll(14);assert(sr->loan);
    uint8_t ltk_request[13]={5,1,0};dbt_hci_packet_t ltk_event={.type=DBT_H4_EVENT,.event=0x3e,.size=13,.payload=ltk_request};
    assert(probe_smp_event(&ltk_event) && !sr->loan && !r.tx[0].active && slot.reconnect);
    memset(&slot,0,sizeof(slot));test_prompt_window=4;probe_smp_poll(15);assert(sr->loan);
    l->generation=2;probe_smp_security_request_preflight(16);
    assert(!r.tx[0].active && !request_for(1,1));
    probe_smp_poll(16);sr=request_for(1,2);assert(sr && sr->loan);
    test_frozen=true;probe_smp_security_request_preflight(17);assert(!sr->loan && !r.tx[0].active);
    test_frozen=false;test_prompt_window=5;r.now_ms=20;probe_smp_poll(20);assert(sr->loan);
    dbt_tx_done_t prompt_done=r.tx[0].done;void *prompt_context=r.tx[0].context;r.tx[0].active=false;
    prompt_done(prompt_context,DBT_TX_ADMITTED);
    assert(!sr->loan && sr->outside_done && sr->last_window==5 && !l->encrypted);
    dbt_smp_session_security_t unproven;assert(probe_smp_security(1,2,&unproven)!=DBT_OK);
    probe_smp_poll(21);assert(!r.tx[0].active);test_prompt_window=6;probe_smp_poll(2000);assert(!r.tx[0].active);
    assert(probe_smp_wait_ms(2000)==20);r.now_ms=2020;probe_smp_poll(2020);assert(sr->loan);
    prompt_done=r.tx[0].done;prompt_context=r.tx[0].context;r.tx[0].active=false;l->used=false;
    prompt_done(prompt_context,DBT_TX_LINK_LOST);assert(!request_for(1,2));
    l->used=true;l->generation=3;probe_smp_poll(2021);assert(request_for(1,3) && r.tx[0].active);
    l->generation=4;probe_smp_security_request_preflight(2022);assert(!r.tx[0].active && !request_for(1,3));
    test_prompt_window=0;probe_smp_poll(2100);sr=request_for(1,4);assert(sr && sr->loan);
    r.now_ms=7100;probe_smp_security_request_preflight(7100);assert(!sr->loan && sr->outside_done);
    probe_smp_poll(7101);assert(!r.tx[0].active); /* no repeated outside-window prompt */
    test_prompt_window=7;probe_smp_poll(7102);assert(sr->loan);
    test_frozen=true;probe_smp_security_request_preflight(7103);assert(!sr->loan);test_frozen=false;
    test_prompt_window=0;memset(security_requests,0,sizeof(security_requests));
    memset(&r.tx[0],0,sizeof(r.tx[0]));l->generation=1;r.now_ms=0;
    configuration.peripheral_security_requests=false;configuration.pairing.features.bonding=false;
#endif
#ifdef DBT_PROBE_PRODUCT_SECURITY
#ifndef DBT_SECURITY_REQUEST_TEST
    extern bool test_admission_allowed,test_frozen;
#endif
    test_admission_allowed=false;
    assert(probe_smp_receive(l,6,request,sizeof(request))==DBT_UNSUPPORTED && !slot.active && !worker_live);
    test_admission_allowed=true;
    test_frozen=true;assert(probe_smp_reconnect_start(l)==DBT_BUSY);
    uint8_t ltk[13]={5,1,0};dbt_hci_packet_t event={.type=DBT_H4_EVENT,.event=0x3e,.size=13,.payload=ltk};
    assert(!probe_smp_event(&event) && !slot.active);test_frozen=false;
#endif
    assert(probe_smp_receive(l,6,request,sizeof(request))==DBT_OK);
    assert(!probe_smp_quiescent());
    assert(!open_calls); probe_smp_poll(0); assert(starts==1 && worker_live && uxQueueMessagesWaiting(work_queue)==1 && !open_calls);
    run_worker(); assert(open_calls==1 && !session.runtime);
    probe_smp_poll(1); assert(session.runtime==&r && slot.prepared && session.tx_active);
    dbt_runtime_tx_t *tx=&r.tx[0]; assert(tx->active); tx->active=false; tx->done(tx->context,DBT_TX_ADMITTED);
    probe_smp_poll(2); assert(session.phase==DBT_SMP_SESSION_SC && uxQueueMessagesWaiting(work_queue));
    run_worker(); assert(crypto_calls==1); probe_smp_poll(3); probe_smp_poll(4);
    assert(slot.failed && uxQueueMessagesWaiting(work_queue));
    run_worker(); probe_smp_poll(5); assert(!slot.active && !worker_live && close_calls==1);
    assert(probe_smp_quiescent());
    uint16_t handle; uint32_t generation; assert(probe_smp_take_failure(&handle,&generation));
    assert(handle==1 && generation==1 && !probe_smp_take_failure(&handle,&generation));

    /* A disconnected generation's completed preparation is never attached. */
    r.now_ms=2000; l->generation=2;
    assert(probe_smp_receive(l,6,request,sizeof(request))==DBT_OK); probe_smp_poll(2000);
    dbt_link_change_t change={DBT_LINK_REMOVED,1,2}; probe_smp_link(&change); l->used=false;
    run_worker(); probe_smp_poll(2001); assert(!session.runtime && slot.active);
    run_worker(); probe_smp_poll(2002); assert(!slot.active && close_calls==2);

    /* Preflight has a bounded owner-side deadline; late worker completion is discarded. */
    r.now_ms=4000; l->used=true; l->generation=3; l->peer[0]=8;
    assert(probe_smp_receive(l,6,request,sizeof(request))==DBT_OK); probe_smp_poll(4000);
    probe_smp_poll(34001); assert(slot.failed && !session.runtime && work_inflight);
    assert(probe_smp_take_failure(&handle,&generation) && generation==3);
    probe_smp_poll(34001); /* Accepted worker job is still pending retirement. */
    if(probe_smp_take_failure(&handle,&generation)){
        fputs("FAIL duplicate SMP failure notification before worker retirement\n",stderr);return 1;
    }
    run_worker(); probe_smp_poll(34002); assert(!session.runtime && slot.active);
    run_worker(); probe_smp_poll(34003); assert(!slot.active);
    assert(!probe_smp_take_failure(&handle,&generation));

    /* Worker allocation failure never opens or accepts an SMP session. */
    r.now_ms=40000; l->generation=4; l->peer[0]=9; fail_start=1;
    assert(probe_smp_receive(l,6,request,sizeof(request))==DBT_OK);
    probe_smp_poll(40000); assert(slot.failed && !worker_live && !work_inflight);
    probe_smp_poll(40001); assert(!slot.active && !session.runtime);
    assert(probe_smp_take_failure(&handle,&generation) && generation==4);
    fail_start=0;

    /* Provider execution occurs only on the worker and retains durable result. */
    worker_state_t state={0}; work_t w={0}; completion_t c;
    w.kind=WORK_PREPARE; w.generation=8; w.payload.prepare.bonding=true;
    w.payload.prepare.mask=2; worker_context=1; worker_execute(&state,&w,&c); worker_context=0;
    assert(c.success && !c.existing && store_calls==1);
    memset(&w,0,sizeof(w)); w.kind=WORK_STORE; w.generation=8; w.payload.store.serial=4;
    w.payload.store.generation=8; w.payload.store.op=DBT_SMP_BOND_STORE;
    worker_context=1; worker_execute(&state,&w,&c); worker_context=0;
    /* Current store preserves prior identity proof through a load then store. */
    assert(c.success && c.serial==4 && store_calls==3);
    w.require_rpa_binding=true; w.payload.store.record.have_irk=true; w.payload.store.record.candidate_present=true;
    w.binding_rpa[0]=1; worker_context=1; worker_execute(&state,&w,&c); worker_context=0;
    assert(!c.success && store_calls==3);
    w.binding_rpa[0]=0xaa; worker_context=1; worker_execute(&state,&w,&c); worker_context=0;
    assert(c.success && store_calls==5);
    w.generation=9; worker_context=1; worker_execute(&state,&w,&c); worker_context=0;
    assert(!c.success && store_calls==5); w.generation=8;
    state.provider.poisoned=true;
    worker_context=1; worker_execute(&state,&w,&c); worker_context=0; assert(!c.success && storage_poisoned);
    w.kind=WORK_RETIRE; worker_context=1; worker_execute(&state,&w,&c); worker_context=0;
    assert(c.success && nvs_closes==1 && !state.storage_open);
    w.kind=WORK_PREPARE; w.generation=9; w.payload.prepare.bonding=true;
    worker_context=1; worker_execute(&state,&w,&c); worker_context=0; assert(!c.success);
    w.kind=WORK_RETIRE; worker_context=1; worker_execute(&state,&w,&c); worker_context=0; assert(c.success);
    /* Explicit maintenance dispatches on a lazy worker and retires before its
     * result is consumed. */
    assert(probe_smp_storage_inspect_start()==DBT_OK);
    assert(probe_smp_storage_inspect_start()==DBT_BUSY);
    probe_smp_poll(50000); assert(work_inflight); run_worker(); probe_smp_poll(50001);
    assert(maintenance.done && work_inflight); run_worker(); probe_smp_poll(50002);
    dbt_result_t status; dbt_smp_bond_op_t op; uint8_t type,id[6]; bool applied;
    assert(probe_smp_storage_result(&status,&op,&type,id,&applied)==DBT_OK && status==DBT_EMPTY);
    assert(!maintenance.active && !worker_live);
    fail_start=1; int before_starts=starts;
    assert(probe_smp_storage_inspect_start()==DBT_OK);
    probe_smp_poll(50010); probe_smp_poll(50011);
    assert(starts==before_starts+1 && !worker_live);
    assert(probe_smp_storage_result(&status,&op,&type,id,&applied)==DBT_OK && status==DBT_NO_SPACE);
    fail_start=0;
    /* Responder RPA first pairing requests peer IdKey only, with no local IRK. */
    storage_poisoned=false; /* independent simulated reboot */
    r.now_ms=60000; l->used=true; l->generation=5; l->role=1; l->peer_address_type=1;
    const uint8_t rpa[6]={0xaa,0xfb,0x0d,0x94,0x81,0x70}; memcpy(l->peer,rpa,6);
    configuration.peer_identity_only=true; configuration.pairing.features.bonding=true;
    configuration.pairing.features.require_sc=true;
    uint8_t private_request[7]={1,3,0,9,16,2,0};
    assert(probe_smp_receive(l,6,private_request,sizeof(private_request))==DBT_OK);
    probe_smp_poll(60000); assert(work_mailbox.payload.prepare.mask==0); run_worker();
    probe_smp_poll(60001);
    assert(session.runtime && session.feature.policy.initiator_keys==2 && session.feature.policy.responder_keys==0);
    assert(session.tx_active); dbt_runtime_tx_t *pdu=&r.tx[0]; pdu->active=false; pdu->done(pdu->context,DBT_TX_ABORTED);
    probe_smp_poll(60002); probe_smp_poll(60003); assert(work_inflight);
    run_worker(); probe_smp_poll(60004); assert(!slot.active);
    r.now_ms=70000; l->generation=6; l->role=0;
    assert(probe_smp_start(l)==DBT_OK); probe_smp_poll(70000); run_worker(); probe_smp_poll(70001);
    assert(session.runtime && session.feature.policy.initiator_keys==0 && session.feature.policy.responder_keys==2);
    assert(session.tx_active); pdu=&r.tx[0]; pdu->active=false; pdu->done(pdu->context,DBT_TX_ABORTED);
    probe_smp_poll(70002); probe_smp_poll(70003); assert(work_inflight);
    run_worker(); probe_smp_poll(70004); assert(!slot.active);
    return 0;
}
