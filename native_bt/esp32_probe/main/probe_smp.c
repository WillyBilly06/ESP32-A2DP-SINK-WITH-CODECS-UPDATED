#include "probe_smp.h"
#include "dbt_smp_nvs.h"
#include "dbt_smp_worker_mbedtls.h"
#include "probe_cccd_store.h"
#include "esp_random.h"
#include "freertos/queue.h"
#include <nvs.h>
#include <string.h>
#ifdef DBT_PROBE_RF_PROTOCOL_DIAGNOSTIC
#include "probe_security.h"
/* Owner-only, at most 24 metadata records. No key/nonce/random/PDU contents. */
static void trace_smp(uint16_t h,uint32_t g,uint8_t stage,int result,uint8_t code,uint16_t size,uint32_t window){
    static unsigned count;if(count>=24)return;++count;
    probe_security_smp_trace(h,g,stage,result,code,size,window);
}
#else
#define trace_smp(...) ((void)0)
#endif
#ifdef DBT_PROBE_PRODUCT_SECURITY
#include "probe_admission.h"
#endif

/* The probe supports one active procedure and a bounded, non-evicting guard
 * table. Worker inputs/results are copied through single-item queues; only the
 * host owner touches session or guard state. No controller callback waits. */
#define SMP_CID 6u
#define GUARDS 8u
#define QUARANTINES 16u
#define WORKER_STACK_BYTES 12288u
#define WAIT_MS 20u
#define SECURITY_REQUEST_TTL_MS 5000u
#define SECURITY_REQUEST_COOLDOWN_MS 2000u
typedef enum { WORK_PREPARE, WORK_CRYPTO, WORK_STORE, WORK_RETIRE, WORK_LOAD,
               WORK_INSPECT, WORK_RECONCILE, WORK_RESOLVE, WORK_PROMOTE } work_kind_t;
typedef struct {
    work_kind_t kind;
    uint32_t generation;
    bool require_rpa_binding;
    uint8_t binding_rpa[6];
    union {
        struct { uint8_t peer_type,peer[6],local_type,local[6],mask; bool bonding; } prepare;
        dbt_smp_session_crypto_job_t crypto;
        dbt_smp_bond_job_t store;
    } payload;
} work_t;
typedef struct {
    work_kind_t kind;
    uint32_t generation,serial;
    bool success,existing,resolved_candidate;
    dbt_smp_session_crypto_kind_t crypto_kind;
    dbt_result_t status;
    dbt_smp_bond_op_t operation;
    uint8_t identity_type,identity[6];
    bool tag_valid;
    uint8_t bond_tag[PROBE_CCCD_TAG_SIZE];
    bool applied;
    dbt_smp_bond_record_t previous;
    dbt_smp_dist_keys_t local_keys;
    union { dbt_smp_job_result_t legacy; dbt_smp_sc_result_t sc; } crypto;
} completion_t;
typedef struct { bool used; uint8_t type,address[6]; dbt_smp_guard_t guard; } guard_entry_t;
static dbt_runtime_t *runtime;
static TaskHandle_t owner;
static QueueHandle_t work_queue,done_queue;
static probe_smp_config_t configuration;
static guard_entry_t guards[GUARDS];
/* Unknown private addresses share one bounded retry bucket; rotating RPAs
 * never allocate a new identity-table entry before resolution. */
static guard_entry_t private_guard;
static uint16_t quarantined_handles[QUARANTINES];
static unsigned quarantine_count;
static bool quarantine_all;
static dbt_smp_session_t session;
static struct {
    uint16_t handle;
    uint32_t generation,prepare_deadline,negative_deadline,reply_token;
    uint8_t peer_type,peer[6],inbox[65],inbox_size;
    bool active,initiator,prepare_queued,prepared,disconnected,failed,retiring,retired,guard_reported,failure_reported;
    bool reconnect,reconnect_central,negative_needed,negative_started,reply_pending,reply_accepted,key_ready,verified;
    bool resolved,resolved_candidate,encrypted_verified,promotion_issued;
    uint8_t resolved_type,resolved_identity[6];
    /* The exact committed bond identity, not the current air address. */
    bool committed_identity_valid;
    uint8_t committed_identity_type,committed_identity[6];
    bool committed_tag_valid;
    uint8_t committed_tag[PROBE_CCCD_TAG_SIZE];
    bool reconnect_authenticated,reconnect_sc;
    uint16_t reply_opcode,ediv;
    uint8_t rand[8],key[16],reconnect_key_size;
    guard_entry_t *guard;
} slot;
static work_t pending;
static dbt_smp_reconnect_t *reconnect;
static uint8_t *reconnect_allocation;
/* FreeRTOS queue slots retain removed bytes. Keep only one-byte tokens in
 * queues; the consumer explicitly wipes each shared sensitive mailbox. */
static work_t work_mailbox;
static completion_t completion_mailbox;
static bool pending_valid,work_inflight,worker_live;
/* A failed/ambiguous mutation is latched across worker lifetimes. Reboot or
 * an explicit external reconciliation is needed before enabling bonding. */
static bool storage_poisoned;
static struct { bool pending; uint16_t handle; uint32_t generation; } failure;
static struct {
    bool active,issued,done,retiring,retired,applied;
    work_kind_t kind; uint32_t generation; dbt_result_t result;
    dbt_smp_bond_op_t operation; uint8_t type,identity[6];
} maintenance;
static uint32_t maintenance_serial;
static bool quarantined(uint16_t handle);
typedef struct {
    bool used,loan,outside_done,cancelling,cancel_expired;
    uint16_t handle;uint32_t generation,last_window,queued_window,deadline,next_allowed;
    uint8_t bytes[2];
} security_request_t;
static security_request_t security_requests[DBT_MAX_LINKS];

static void wipe(void *p,size_t n) { volatile uint8_t *v=p; while (n--) *v++=0; }
static void security_request_done(void *context,dbt_tx_outcome_t outcome){
    security_request_t *r=context;
    if(!r || !r->loan)return;
    trace_smp(r->handle,r->generation,2,(int)outcome,0x0b,2,r->queued_window);
    r->loan=false;
    if(outcome==DBT_TX_ADMITTED){
        r->outside_done=true;if(r->queued_window)r->last_window=r->queued_window;
        r->next_allowed=runtime->now_ms+SECURITY_REQUEST_COOLDOWN_MS;
    }else if(r->cancel_expired || !r->cancelling){r->outside_done=true;if(r->queued_window)r->last_window=r->queued_window;}
    r->cancelling=false;r->cancel_expired=false;r->queued_window=0;
    const dbt_link_t *l=dbt_acl_find(&runtime->acl,r->handle);
    if(!l || l->generation!=r->generation)wipe(r,sizeof(*r));
}
static security_request_t *request_for(uint16_t handle,uint32_t generation){
    for(unsigned i=0;i<DBT_MAX_LINKS;i++)if(security_requests[i].used && security_requests[i].handle==handle &&
        security_requests[i].generation==generation)return &security_requests[i];
    return NULL;
}
static dbt_result_t cancel_request(security_request_t *r,bool expired){
    if(!r || !r->loan)return DBT_OK;
    r->cancelling=true;r->cancel_expired=expired;
    dbt_result_t rc=dbt_runtime_cancel_fixed(runtime,r->handle,r->generation,SMP_CID,security_request_done,r);
    if(rc!=DBT_OK){r->cancelling=false;r->cancel_expired=false;}
    return rc;
}
static uint32_t request_window(uint32_t now){
#ifdef DBT_PROBE_PRODUCT_SECURITY
    return probe_admission_prompt_epoch(now);
#else
    (void)now;return 0;
#endif
}
static bool request_frozen(void){
#ifdef DBT_PROBE_PRODUCT_SECURITY
    return probe_admission_frozen();
#else
    return false;
#endif
}
void probe_smp_security_request_preflight(uint32_t now){
    if(!runtime || !configuration.peripheral_security_requests)return;
    uint32_t window=request_window(now);
    for(unsigned i=0;i<DBT_MAX_LINKS;i++){
        security_request_t *r=&security_requests[i];if(!r->used)continue;
        const dbt_link_t *l=dbt_acl_find(&runtime->acl,r->handle);
        bool stale=!l || l->generation!=r->generation || l->kind!=DBT_LINK_LE || l->role!=1;
        bool expired=(int32_t)(now-r->deadline)>=0 || (r->queued_window && r->queued_window!=window);
        if(r->loan && (stale || l->encrypted || request_frozen() || slot.active || expired))
            (void)cancel_request(r,expired);
        if(stale && !r->loan)wipe(r,sizeof(*r));
    }
}
static dbt_result_t stop_request(const dbt_link_t *l,uint32_t now){
    if(!l)return DBT_INVALID;
    security_request_t *r=request_for(l->handle,l->generation);if(!r)return DBT_OK;
    if(r->loan){dbt_result_t rc=cancel_request(r,false);if(rc!=DBT_OK)return rc;}
    r->outside_done=true;uint32_t window=request_window(now);if(window)r->last_window=window;
    return DBT_OK;
}
static void suspend_other_requests(const dbt_link_t *l){
    for(unsigned i=0;i<DBT_MAX_LINKS;i++){
        security_request_t *r=&security_requests[i];
        if(r->loan && (r->handle!=l->handle || r->generation!=l->generation))(void)cancel_request(r,false);
    }
}
static void security_request_poll(uint32_t now){
    if(!runtime || !runtime->ready || !configuration.peripheral_security_requests || request_frozen() ||
       slot.active || maintenance.active || runtime->acl.mtu[DBT_LINK_LE]<6)return;
    uint32_t window=request_window(now);
    for(unsigned i=0;i<DBT_MAX_LINKS;i++){
        const dbt_link_t *l=&runtime->acl.links[i];
        if(!l->used || l->kind!=DBT_LINK_LE || l->role!=1 || l->encrypted || quarantined(l->handle))continue;
        security_request_t *r=request_for(l->handle,l->generation);
        if(!r){for(unsigned j=0;j<DBT_MAX_LINKS;j++)if(!security_requests[j].used){r=&security_requests[j];break;}
            if(!r)continue;
            memset(r,0,sizeof(*r));r->used=true;r->handle=l->handle;r->generation=l->generation;r->next_allowed=now;}
        if(r->loan || (int32_t)(now-r->next_allowed)<0 || (window?r->last_window==window:r->outside_done))continue;
        r->bytes[0]=0x0b;r->bytes[1]=(configuration.pairing.features.bonding?1u:0u)|
            (configuration.pairing.features.require_mitm?4u:0u)|(configuration.pairing.features.support_sc?8u:0u);
        r->queued_window=window;r->deadline=now+SECURITY_REQUEST_TTL_MS;
        dbt_result_t result=dbt_runtime_send_fixed(runtime,l->handle,SMP_CID,r->bytes,2,security_request_done,r);
        trace_smp(l->handle,l->generation,1,(int)result,r->bytes[1],2,window);
        if(result==DBT_OK)r->loan=true;
        else {
            r->queued_window=0;r->next_allowed=now+WAIT_MS;
            if(result!=DBT_BUSY && result!=DBT_NO_SPACE){r->outside_done=true;if(window)r->last_window=window;}
        }
    }
}
static bool random_bytes(void *ctx,uint8_t *out,size_t n) {
    (void)ctx; esp_fill_random(out,n); return true;
}
static bool stable_identity(uint8_t type,const uint8_t address[6]) {
    return type==0 || (type==1 && (address[5]&0xc0u)==0xc0u);
}
static bool resolvable(uint8_t type,const uint8_t a[6]) {
    return type==1 && (a[5]&0xc0u)==0x40u &&
        !(a[3]==0 && a[4]==0 && (a[5]&0x3fu)==0) &&
        !(a[3]==0xff && a[4]==0xff && (a[5]&0x3fu)==0x3f);
}
static bool quarantined(uint16_t handle) {
    if (quarantine_all) return true;
    for (unsigned i=0;i<quarantine_count;i++) if (quarantined_handles[i]==handle) return true;
    return false;
}
static void quarantine(uint16_t handle) {
    if (quarantined(handle)) return;
    if (quarantine_count==QUARANTINES) { quarantine_all=true; return; }
    quarantined_handles[quarantine_count++]=handle;
}
static guard_entry_t *guard_for(uint8_t type,const uint8_t address[6],uint32_t now) {
    guard_entry_t *empty=NULL;
    for (unsigned i=0;i<GUARDS;i++) {
        if (guards[i].used && guards[i].type==type && !memcmp(guards[i].address,address,6))
            return &guards[i];
        if (!guards[i].used && !empty) empty=&guards[i];
    }
    if (!empty || dbt_smp_guard_init(&empty->guard,1000,60000,300000,now)!=DBT_OK) return NULL;
    empty->used=true; empty->type=type; memcpy(empty->address,address,6); return empty;
}
typedef struct {
    dbt_smp_worker_mbedtls_t crypto;
    uint32_t crypto_generation;
    nvs_handle_t storage;
    bool storage_open;
    uint32_t link_generation;
    dbt_smp_nvs_t provider;
} worker_state_t;
static void worker_execute(worker_state_t *state,work_t *w,completion_t *result) {
        dbt_smp_worker_mbedtls_t *crypto=&state->crypto;
        uint32_t *crypto_generation=&state->crypto_generation;
        dbt_smp_nvs_t *provider=&state->provider;
        work_t work=*w;
        completion_t done;
        memset(&done,0,sizeof(done)); done.kind=work.kind; done.generation=work.generation;
        if (work.kind==WORK_PREPARE) {
            dbt_smp_worker_mbedtls_close(crypto); *crypto_generation=0;
            done.success=dbt_smp_worker_mbedtls_open(crypto,work.generation,random_bytes,NULL);
            if (done.success) *crypto_generation=work.generation;
            if (done.success && work.payload.prepare.bonding) {
                if (!state->storage_open && !storage_poisoned) {
                    state->storage_open=nvs_open("native_le_bonds",NVS_READWRITE,&state->storage)==ESP_OK;
                    if (state->storage_open) dbt_smp_nvs_init(provider,state->storage);
                }
                if (!state->storage_open || provider->poisoned || storage_poisoned) done.success=false;
                else {
                    dbt_smp_bond_job_t load={0};
                    load.op=DBT_SMP_BOND_LOAD; load.generation=work.generation; load.serial=1;
                    load.identity_type=work.payload.prepare.peer_type;
                    memcpy(load.identity,work.payload.prepare.peer,6);
                    dbt_result_t rc=resolvable(load.identity_type,load.identity)
                        ? dbt_smp_nvs_resolve_rpa(provider,load.identity,&done.previous,&done.resolved_candidate)
                        : dbt_smp_nvs_execute(provider,&load,&done.previous);
                    done.existing=rc==DBT_OK;
                    if (rc!=DBT_OK && rc!=DBT_EMPTY) done.success=false;
                    wipe(&load,sizeof(load));
                }
                if (done.success && work.payload.prepare.mask)
                    done.success=dbt_smp_worker_mbedtls_distribution(crypto,work.payload.prepare.mask,
                        work.payload.prepare.local_type,work.payload.prepare.local,&done.local_keys);
            }
        } else if (work.kind==WORK_CRYPTO && *crypto_generation==work.generation) {
            done.crypto_kind=work.payload.crypto.kind;
            if (done.crypto_kind==DBT_SMP_SESSION_CRYPTO_SC) {
                done.serial=work.payload.crypto.input.sc.serial;
                done.success=dbt_smp_worker_mbedtls_sc(crypto,&work.payload.crypto.input.sc,&done.crypto.sc);
            } else if (done.crypto_kind==DBT_SMP_SESSION_CRYPTO_LEGACY) {
                done.serial=work.payload.crypto.input.legacy.serial;
                done.success=dbt_smp_worker_mbedtls_legacy(crypto,&work.payload.crypto.input.legacy,
                                                            &done.crypto.legacy);
            }
        } else if (work.kind==WORK_STORE) {
            done.serial=work.payload.store.serial;
            bool bound=true;
            if (work.require_rpa_binding) {
                const dbt_smp_bond_record_t *r=&work.payload.store.record;
                bound=r->have_irk && r->candidate_present && r->identity_type==r->candidate_type &&
                    !memcmp(r->identity,r->candidate_identity,6) && !memcmp(r->irk,r->candidate_irk,16);
                bool matching=false;
                if (bound) bound=dbt_smp_nvs_verify_rpa(work.binding_rpa,r->irk,&matching) && matching;
            }
            done.success=state->storage_open && !provider->poisoned &&
                *crypto_generation==work.generation && work.payload.store.generation==work.generation &&
                work.payload.store.op==DBT_SMP_BOND_STORE && bound &&
                dbt_smp_nvs_execute(provider,&work.payload.store,NULL)==DBT_OK;
            if (done.success) {
                done.identity_type=work.payload.store.identity_type;
                memcpy(done.identity,work.payload.store.identity,6);
                done.tag_valid=probe_cccd_bond_tag(&work.payload.store.record,done.bond_tag);
            }
            if (provider->poisoned) storage_poisoned=true;
        } else if (work.kind==WORK_LOAD || work.kind==WORK_RESOLVE) {
            done.serial=work.payload.store.serial;
            if (!state->storage_open && !storage_poisoned) {
                state->storage_open=nvs_open("native_le_bonds",NVS_READWRITE,&state->storage)==ESP_OK;
                if (state->storage_open) dbt_smp_nvs_init(provider,state->storage);
            }
            state->link_generation=work.generation;
            if (state->storage_open && !provider->poisoned && !storage_poisoned &&
                work.payload.store.generation==work.generation && work.payload.store.op==DBT_SMP_BOND_LOAD)
                done.success=(work.kind==WORK_RESOLVE
                    ? dbt_smp_nvs_resolve_rpa(provider,work.payload.store.identity,&done.previous,&done.resolved_candidate)
                    : dbt_smp_nvs_execute(provider,&work.payload.store,&done.previous))==DBT_OK;
            if(done.success)done.tag_valid=probe_cccd_bond_tag(&done.previous,done.bond_tag);
        } else if (work.kind==WORK_PROMOTE) {
            done.serial=work.payload.store.serial;
            done.success=state->storage_open && !provider->poisoned && !storage_poisoned &&
                state->link_generation==work.generation && work.payload.store.generation==work.generation &&
                work.payload.store.op==DBT_SMP_BOND_STORE &&
                dbt_smp_nvs_execute(provider,&work.payload.store,NULL)==DBT_OK;
            if (provider->poisoned) storage_poisoned=true;
        } else if (work.kind==WORK_INSPECT || work.kind==WORK_RECONCILE) {
            if (!state->storage_open) {
                state->storage_open=nvs_open("native_le_bonds",NVS_READWRITE,&state->storage)==ESP_OK;
                if (state->storage_open) dbt_smp_nvs_init(provider,state->storage);
            }
            done.status=DBT_INVALID;
            if (state->storage_open && !provider->poisoned) {
                if (work.kind==WORK_INSPECT)
                    done.status=dbt_smp_nvs_pending(provider,&done.operation,&done.identity_type,done.identity);
                else {
                    done.operation=work.payload.store.op;
                    done.identity_type=work.payload.store.identity_type;
                    memcpy(done.identity,work.payload.store.identity,6);
                    done.status=dbt_smp_nvs_reconcile(provider,done.operation,done.identity_type,
                                                       done.identity,&done.applied);
                    if (done.status==DBT_OK) storage_poisoned=false;
                }
                if (provider->poisoned) storage_poisoned=true;
            }
        } else if (work.kind==WORK_RETIRE &&
                   (*crypto_generation==work.generation || !*crypto_generation)) {
            dbt_smp_worker_mbedtls_close(crypto); *crypto_generation=0;
            state->link_generation=0;
            if (provider->poisoned) storage_poisoned=true;
            if (state->storage_open) { nvs_close(state->storage); state->storage_open=false; }
            wipe(provider,sizeof(*provider));
            done.success=true;
        }
        wipe(&work,sizeof(work));
        *result=done;
        wipe(&done,sizeof(done));
}
static void worker(void *unused) {
    (void)unused;
    worker_state_t state={0};
    for (;;) {
        work_t work; completion_t done; uint8_t token;
        if (xQueueReceive(work_queue,&token,portMAX_DELAY)!=pdTRUE) continue;
        work=work_mailbox; wipe(&work_mailbox,sizeof(work_mailbox));
        worker_execute(&state,&work,&done);
        bool retired=work.kind==WORK_RETIRE && done.success;
        wipe(&work,sizeof(work));
        completion_mailbox=done; token=retired ? 2 : 1;
        (void)xQueueSend(done_queue,&token,portMAX_DELAY);
        wipe(&done,sizeof(done));
        xTaskNotifyGive(owner);
        if (token==2) { vTaskDelete(NULL); return; }
    }
}

dbt_result_t probe_smp_init(dbt_runtime_t *r,TaskHandle_t host_owner,const probe_smp_config_t *config) {
    if (!r || !r->ready || !host_owner || !config || runtime || config->local_address_type>1)
        return DBT_INVALID;
    /* This probe's GAP advertises with the controller public address. Random
     * local addresses and enhanced privacy require explicit address capture. */
    if (config->local_address_type!=0 || memcmp(config->local_address,r->host.controller.address_le,6))
        return DBT_UNSUPPORTED;
    /* OOB requires a worker-private key bound to a prior secure OOB exchange;
     * this probe does not yet expose that pre-exchange. */
    if (config->pairing.features.oob || config->pairing.have_oob) return DBT_UNSUPPORTED;
    if (config->peer_identity_only && (!config->pairing.features.bonding || !config->pairing.features.require_sc ||
        config->pairing.features.initiator_keys || config->pairing.features.responder_keys)) return DBT_INVALID;
    if (config->pairing.features.bonding &&
        !stable_identity(config->local_address_type,config->local_address)) return DBT_INVALID;
    if (config->pairing.have_passkey && config->pairing.passkey>999999u) return DBT_INVALID;
    dbt_smp_features_t validation;
    dbt_result_t policy=dbt_smp_features_open(&validation,1,1,true,&config->pairing.features);
    if (policy!=DBT_OK) return policy;
    dbt_smp_features_close(&validation);
    if (dbt_smp_guard_init(&private_guard.guard,1000,60000,300000,r->now_ms)!=DBT_OK) return DBT_INVALID;
    private_guard.used=true;
    work_queue=xQueueCreate(1,sizeof(uint8_t)); done_queue=xQueueCreate(1,sizeof(uint8_t));
    if (!work_queue || !done_queue) goto no_space;
    if (dbt_channels_register_fixed(&r->channels,DBT_LINK_LE,SMP_CID)!=DBT_OK) goto no_space;
    owner=host_owner; configuration=*config; runtime=r;
    wipe(&configuration.pairing.local_keys,sizeof(configuration.pairing.local_keys));
    configuration.pairing.have_local_keys=false;
    return DBT_OK;
no_space:
    if (work_queue) vQueueDelete(work_queue);
    if (done_queue) vQueueDelete(done_queue);
    work_queue=done_queue=NULL; wipe(&configuration,sizeof(configuration)); owner=NULL;
    return DBT_NO_SPACE;
}
dbt_result_t probe_smp_set_passkey(uint32_t passkey) {
    if (!runtime || slot.active || passkey>999999u) return DBT_INVALID;
    configuration.pairing.passkey=passkey; configuration.pairing.have_passkey=true;
    return DBT_OK;
}
static dbt_result_t begin(const dbt_link_t *l,bool initiator) {
    if (!runtime || !l || !l->used || l->kind!=DBT_LINK_LE || l->encrypted || l->role>1 ||
        l->peer_address_type>1 || l->role!=(initiator?0:1)) return DBT_INVALID;
    const dbt_link_t *current=dbt_acl_find(&runtime->acl,l->handle);
    if (!current || current->generation!=l->generation) return DBT_INVALID;
    if (slot.active || maintenance.active || quarantined(l->handle)) return DBT_BUSY;
    if (configuration.pairing.features.bonding && !stable_identity(l->peer_address_type,l->peer) &&
        !(configuration.peer_identity_only && resolvable(l->peer_address_type,l->peer)))
        return DBT_UNSUPPORTED;
    guard_entry_t *g=resolvable(l->peer_address_type,l->peer) ? &private_guard :
                     guard_for(l->peer_address_type,l->peer,runtime->now_ms);
    if (!g || !dbt_smp_guard_allowed(&g->guard,runtime->now_ms)) return DBT_BUSY;
#ifdef DBT_PROBE_PRODUCT_SECURITY
    if(!probe_admission_le_begin(l,runtime->now_ms))return DBT_UNSUPPORTED;
#endif
    memset(&slot,0,sizeof(slot)); slot.active=true; slot.initiator=initiator;
    slot.handle=l->handle; slot.generation=l->generation; slot.peer_type=l->peer_address_type;
    slot.prepare_deadline=runtime->now_ms+30000u;
    memcpy(slot.peer,l->peer,6); slot.guard=g;suspend_other_requests(l);
    return DBT_OK;
}
static bool begin_reconnect(const dbt_link_t *l,const uint8_t *request,bool central) {
    if (!runtime || slot.active || maintenance.active || !configuration.pairing.features.bonding || !l || !l->used ||
        l->kind!=DBT_LINK_LE || l->role!=(central?0:1) || l->encrypted ||
        (!stable_identity(l->peer_address_type,l->peer) &&
         !(configuration.peer_identity_only && resolvable(l->peer_address_type,l->peer))) || quarantined(l->handle)) return false;
    const dbt_link_t *current=dbt_acl_find(&runtime->acl,l->handle);
    if (!current || current->generation!=l->generation) return false;
    memset(&slot,0,sizeof(slot)); slot.active=true; slot.reconnect=true;
    slot.reconnect_central=central;
    slot.handle=l->handle; slot.generation=l->generation; slot.peer_type=l->peer_address_type;
    memcpy(slot.peer,l->peer,6);
    if (!central) { memcpy(slot.rand,request+3,8); slot.ediv=dbt_read_le16(request+11); }
    slot.prepare_deadline=runtime->now_ms+runtime->host.timeout_ms;suspend_other_requests(l);
    return true;
}
dbt_result_t probe_smp_reconnect_start(const dbt_link_t *l) {
#ifdef DBT_PROBE_PRODUCT_SECURITY
    if(probe_admission_frozen())return DBT_BUSY;
#endif
    if (slot.active) return DBT_BUSY;
    return begin_reconnect(l,NULL,true) ? DBT_OK : DBT_INVALID;
}
static void cancel_reconnect_reply(void) {
    if (slot.reply_pending) {
        uint16_t opcode=slot.reply_opcode;
        dbt_command_cancel_result_t result=dbt_host_cancel_command(&runtime->host,slot.reply_token);
        if (result!=DBT_COMMAND_CANCELLED && opcode!=0x201b) quarantine(slot.handle);
    } else if (slot.reply_accepted && !slot.encrypted_verified) quarantine(slot.handle);
}
dbt_result_t probe_smp_start(const dbt_link_t *l) { return begin(l,true); }
dbt_result_t probe_smp_receive(const dbt_link_t *l,uint16_t cid,const uint8_t *p,uint16_t n) {
    if (!l || cid!=SMP_CID || !p || !n || n>65) return DBT_INVALID;
#ifdef DBT_PROBE_RF_PROTOCOL_DIAGNOSTIC
    if(n==7 && (p[0]==1 || p[0]==2)){
        static unsigned feature_records;if(feature_records<4){++feature_records;probe_security_pairing_features(l->handle,l->generation,p);}
    }
#endif
    trace_smp(l->handle,l->generation,3,0,p[0],n,request_window(runtime?runtime->now_ms:0));
    if(p[0]==1 && stop_request(l,runtime?runtime->now_ms:0)!=DBT_OK)return DBT_BUSY;
    if (!slot.active) {
        if (p[0]!=1) return DBT_INVALID;
        dbt_result_t rc=begin(l,false);
        trace_smp(l->handle,l->generation,4,(int)rc,p[0],n,request_window(runtime?runtime->now_ms:0));
        if (rc!=DBT_OK) return rc;
    }
    if (l->handle!=slot.handle || l->generation!=slot.generation || slot.failed || slot.disconnected)
        return DBT_BUSY;
    if (slot.reconnect) return DBT_BUSY;
    if (session.runtime) return dbt_smp_session_receive(&session,l,cid,p,n);
    if (slot.inbox_size) { slot.failed=true; return DBT_NO_SPACE; }
    memcpy(slot.inbox,p,n); slot.inbox_size=(uint8_t)n; return DBT_OK;
}
void probe_smp_link(const dbt_link_change_t *change) {
#ifdef DBT_PROBE_RF_PROTOCOL_DIAGNOSTIC
    if(change && change->kind==DBT_LINK_ADDED){
        const dbt_link_t *l=dbt_acl_find(&runtime->acl,change->handle);
        if(l && l->generation==change->generation && l->kind==DBT_LINK_LE){
            uint8_t flags=(configuration.peripheral_security_requests?1:0)|(runtime->ready?2:0)|(slot.active?4:0)|
                (maintenance.active?8:0)|(storage_poisoned?16:0)|(quarantined(l->handle)?32:0)|(request_frozen()?64:0);
            trace_smp(l->handle,l->generation,6,0,flags,0,request_window(runtime->now_ms));
        }
    }
#endif
    if(change && change->kind==DBT_LINK_REMOVED){
        security_request_t *r=request_for(change->handle,change->generation);
        if(r){if(r->loan)(void)cancel_request(r,false);if(!r->loan)wipe(r,sizeof(*r));}
    }
    if (!change || !slot.active || change->kind!=DBT_LINK_REMOVED ||
        change->handle!=slot.handle || change->generation!=slot.generation) return;
    slot.disconnected=true;
    if (session.runtime) dbt_smp_session_link(&session,change);
    if (session.uncertain_generation) quarantine(slot.handle);
    if (slot.reconnect) cancel_reconnect_reply();
}
bool probe_smp_event(const dbt_hci_packet_t *p) {
    if (!runtime || !p || p->type!=DBT_H4_EVENT) return false;
#ifdef DBT_PROBE_PRODUCT_SECURITY
    probe_admission_revalidate();
#endif
    if (p->event==0x3e && p->size==13 && p->payload && p->payload[0]==5) {
        uint16_t handle=dbt_read_le16(p->payload+1);
        const dbt_link_t *l=dbt_acl_find(&runtime->acl,handle);
        if(l && stop_request(l,runtime->now_ms)!=DBT_OK)return false;
        if (!slot.active) {
#ifdef DBT_PROBE_PRODUCT_SECURITY
            if(probe_admission_frozen())return false;
#endif
            bool accepted=begin_reconnect(l,p->payload,false);
            trace_smp(handle,l?l->generation:0,7,accepted?DBT_OK:DBT_INVALID,5,13,request_window(runtime->now_ms));
            return accepted;
        }
        if (slot.reconnect && handle==slot.handle && l && l->generation==slot.generation) {
            slot.failed=true;
            cancel_reconnect_reply();
            return false;
        }
    }
    if(p->event==0x08 && p->size==4 && p->payload){
        const dbt_link_t *l=dbt_acl_find(&runtime->acl,dbt_read_le16(p->payload+1));
        if(l && l->kind==DBT_LINK_LE && l->encrypted)(void)stop_request(l,runtime->now_ms);
    }
    if (slot.active && slot.reconnect && p->event==0x08 && p->size==4 && p->payload &&
        dbt_read_le16(p->payload+1)==slot.handle) {
        const dbt_link_t *l=dbt_acl_find(&runtime->acl,slot.handle);
        bool ok=l && l->generation==slot.generation && slot.reply_accepted && !slot.failed &&
                p->payload[0]==0 && p->payload[3]==1;
        if (!reconnect || dbt_smp_reconnect_encryption(reconnect,slot.generation,ok)!=DBT_OK || !ok) {
            slot.failed=true;
            cancel_reconnect_reply();
        } else {
            slot.encrypted_verified=true;
            if (reconnect->state==DBT_SMP_RECONNECT_SECURE) slot.verified=true;
            else if (reconnect->state==DBT_SMP_RECONNECT_PROMOTE)
                slot.prepare_deadline=runtime->now_ms+runtime->host.timeout_ms;
            else slot.failed=true;
        }
#ifdef DBT_PROBE_PRODUCT_SECURITY
        probe_admission_revalidate();
#endif
        return false;
    }
    if (!slot.active || !session.runtime) return false;
    dbt_result_t rc=dbt_smp_session_event(&session,p);
#ifdef DBT_PROBE_PRODUCT_SECURITY
    probe_admission_revalidate();
#endif
    return p->event==0x3e && p->size==13 && p->payload && p->payload[0]==5 && rc==DBT_OK;
}
static void enqueue(void) {
    if (pending_valid && !work_inflight) {
        uint8_t token=1; work_mailbox=pending;
        if (xQueueSend(work_queue,&token,0)==pdTRUE) {
            wipe(&pending,sizeof(pending)); pending_valid=false; work_inflight=true;
        } else wipe(&work_mailbox,sizeof(work_mailbox));
    }
}
static void prepare(void) {
    memset(&pending,0,sizeof(pending)); pending.kind=WORK_PREPARE; pending.generation=slot.generation;
    pending.payload.prepare.peer_type=slot.peer_type; memcpy(pending.payload.prepare.peer,slot.peer,6);
    pending.payload.prepare.local_type=configuration.local_address_type;
    memcpy(pending.payload.prepare.local,configuration.local_address,6);
    pending.payload.prepare.bonding=configuration.pairing.features.bonding;
    pending.payload.prepare.mask=configuration.peer_identity_only ? 0 :
        slot.initiator ? configuration.pairing.features.initiator_keys : configuration.pairing.features.responder_keys;
    pending_valid=true; slot.prepare_queued=true;
}
static bool reconnect_allocate(void) {
    size_t bytes=sizeof(*reconnect), alignment=_Alignof(dbt_smp_reconnect_t);
    if (bytes+alignment-1u>UINT16_MAX) return false;
    reconnect_allocation=runtime->hooks.allocate(runtime->hooks.context,(uint16_t)(bytes+alignment-1u));
    if (!reconnect_allocation) return false;
    size_t offset=(alignment-((uintptr_t)reconnect_allocation%alignment))%alignment;
    reconnect=(dbt_smp_reconnect_t *)(reconnect_allocation+offset);
    memset(reconnect,0,bytes);
    return true;
}
static bool reconnect_prepare(void) {
    if (resolvable(slot.peer_type,slot.peer)) {
        memset(&pending,0,sizeof(pending)); pending.kind=WORK_RESOLVE; pending.generation=slot.generation;
        pending.payload.store.op=DBT_SMP_BOND_LOAD; pending.payload.store.generation=slot.generation;
        pending.payload.store.serial=1; pending.payload.store.identity_type=slot.peer_type;
        memcpy(pending.payload.store.identity,slot.peer,6);
        pending_valid=true; slot.prepare_queued=true; return true;
    }
    if (!reconnect_allocate()) return false;
    if (dbt_smp_reconnect_open_role(reconnect,slot.generation,slot.peer_type,slot.peer,
          configuration.pairing.features.min_key_size,configuration.pairing.features.require_mitm,
          configuration.pairing.features.require_sc,!slot.reconnect_central)!=DBT_OK) return false;
    memset(&pending,0,sizeof(pending)); pending.kind=WORK_LOAD; pending.generation=slot.generation;
    if (dbt_smp_reconnect_job(reconnect,&pending.payload.store)!=DBT_OK) return false;
    pending_valid=true; slot.prepare_queued=true; return true;
}
static void reconnect_command_done(void *ctx,uint16_t opcode,dbt_command_outcome_t outcome,uint8_t status,
                                   const uint8_t *returns,uint8_t size) {
    (void)ctx;
    if (!slot.active || !slot.reconnect || !slot.reply_pending || opcode!=slot.reply_opcode) return;
    slot.reply_pending=false; slot.reply_token=0;
    if (slot.disconnected || slot.failed) return;
    const dbt_link_t *l=dbt_acl_find(&runtime->acl,slot.handle);
    bool good=l && l->generation==slot.generation && outcome==DBT_COMMAND_SUCCESS && !status && returns &&
              (opcode==0x2019 ? size==1 && returns[0]==0 : size==3 && dbt_read_le16(returns+1)==slot.handle);
    if (!good || opcode==0x201b) { slot.failed=true; return; }
    slot.reply_accepted=true;
    slot.prepare_deadline=runtime->now_ms+runtime->host.timeout_ms;
}
static void reconnect_reply(uint32_t now) {
    if (slot.reply_pending || slot.reply_accepted || (!slot.negative_needed && !slot.key_ready)) return;
    uint8_t params[28]={0}; params[0]=(uint8_t)slot.handle; params[1]=(uint8_t)(slot.handle>>8);
    uint16_t opcode=slot.negative_needed ? 0x201bu : slot.reconnect_central ? 0x2019u : 0x201au;
    uint8_t size=slot.negative_needed ? 2 : slot.reconnect_central ? 28 : 18;
    if (!slot.negative_needed && slot.reconnect_central) {
        memcpy(params+2,slot.rand,8); params[10]=(uint8_t)slot.ediv; params[11]=(uint8_t)(slot.ediv>>8);
        memcpy(params+12,slot.key,16);
    } else if (!slot.negative_needed) memcpy(params+2,slot.key,16);
    dbt_result_t rc=dbt_host_command_tracked(&runtime->host,opcode,params,size,
                                             slot.reconnect_central ? DBT_COMMAND_STATUS : DBT_COMMAND_COMPLETE,
                                             reconnect_command_done,NULL,now,&slot.reply_token);
    wipe(params,sizeof(params));
    if (rc==DBT_OK) {
        slot.reply_pending=true; slot.reply_opcode=opcode;
        slot.prepare_deadline=now+runtime->host.timeout_ms;
        wipe(slot.key,sizeof(slot.key)); slot.key_ready=false;
    } else if (rc!=DBT_BUSY && rc!=DBT_NO_SPACE) slot.failed=true;
}
static void complete(completion_t *done,uint32_t now) {
    if(slot.active && done->generation==slot.generation)
        trace_smp(slot.handle,slot.generation,9,(int)done->status,(uint8_t)done->kind,(uint16_t)((done->success?1:0)|(done->existing?2:0)),request_window(now));
    work_inflight=false;
    if (maintenance.active && done->generation==maintenance.generation) {
        if (done->kind==WORK_RETIRE) {
            maintenance.retired=done->success;
            if (done->success) worker_live=false;
        } else if (done->kind==maintenance.kind) {
            maintenance.done=true; maintenance.result=done->status; maintenance.applied=done->applied;
            if (done->kind==WORK_INSPECT && done->status==DBT_OK) {
                maintenance.operation=done->operation; maintenance.type=done->identity_type;
                memcpy(maintenance.identity,done->identity,6);
            }
        }
        return;
    }
    if (done->kind==WORK_RETIRE && slot.active && done->generation==slot.generation) {
        slot.retired=done->success;
        if (done->success) worker_live=false;
        return;
    }
    if (!slot.active || slot.disconnected || slot.failed || done->generation!=slot.generation) return;
    if ((done->kind==WORK_LOAD || done->kind==WORK_RESOLVE) && slot.reconnect) {
        if ((int32_t)(now-slot.prepare_deadline)>=0) {
            if (slot.reconnect_central) slot.failed=true; else slot.negative_needed=true;
        }
        if (slot.failed) return;
        if (slot.negative_needed) return;
        if (done->kind==WORK_RESOLVE && done->success) {
            if (!reconnect_allocate()) { if (slot.reconnect_central) slot.failed=true; else slot.negative_needed=true; return; }
            dbt_smp_bond_record_t *r=&done->previous;
            dbt_result_t opened=done->resolved_candidate
                ? dbt_smp_reconnect_open_candidate_role(reconnect,slot.generation,r->identity_type,r->identity,
                    r->candidate_type,r->candidate_identity,configuration.pairing.features.min_key_size,
                    configuration.pairing.features.require_mitm,configuration.pairing.features.require_sc,!slot.reconnect_central)
                : dbt_smp_reconnect_open_role(reconnect,slot.generation,r->identity_type,r->identity,
                    configuration.pairing.features.min_key_size,configuration.pairing.features.require_mitm,
                    configuration.pairing.features.require_sc,!slot.reconnect_central);
            if (opened!=DBT_OK) { if (slot.reconnect_central) slot.failed=true; else slot.negative_needed=true; return; }
        }
        if (!done->success || !reconnect || reconnect->state!=DBT_SMP_RECONNECT_LOAD) {
            if (slot.reconnect_central) slot.failed=true; else slot.negative_needed=true;
            return;
        }
        dbt_result_t rc=dbt_smp_reconnect_complete(reconnect,done->generation,done->serial,
                                                    done->success,done->success?&done->previous:NULL);
        if (rc==DBT_OK) {
            slot.reconnect_authenticated=reconnect->record.authenticated;
            slot.reconnect_sc=reconnect->record.secure_connections;
            slot.committed_identity_type=done->previous.identity_type;
            memcpy(slot.committed_identity,done->previous.identity,6);
            slot.committed_identity_valid=stable_identity(slot.committed_identity_type,slot.committed_identity);
            slot.committed_tag_valid=done->tag_valid;
            if(done->tag_valid)memcpy(slot.committed_tag,done->bond_tag,sizeof(slot.committed_tag));
            if (slot.reconnect_central) rc=dbt_smp_reconnect_ids(reconnect,&slot.ediv,slot.rand);
            if (rc==DBT_OK) rc=dbt_smp_reconnect_take(reconnect,slot.ediv,slot.rand,slot.key,&slot.reconnect_key_size);
        }
        if (rc!=DBT_OK) {
            wipe(slot.key,sizeof(slot.key));
            if (slot.reconnect_central) slot.failed=true; else slot.negative_needed=true;
        }
        else { slot.key_ready=true; slot.prepared=true; }
    } else if (done->kind==WORK_PROMOTE && slot.reconnect) {
        if ((int32_t)(now-slot.prepare_deadline)>=0 || !reconnect ||
            dbt_smp_reconnect_complete(reconnect,done->generation,done->serial,
                                                      done->success,NULL)!=DBT_OK)
            slot.failed=true;
        else slot.verified=true;
    } else if (done->kind==WORK_PREPARE && !slot.reconnect) {
        if (!done->success) { slot.failed=true; return; }
        dbt_smp_session_config_t cfg=configuration.pairing;
        if (configuration.peer_identity_only) {
            cfg.features.initiator_keys=slot.initiator ? 0 : 2;
            cfg.features.responder_keys=slot.initiator ? 2 : 0;
        }
        configuration.pairing.have_passkey=false; configuration.pairing.passkey=0;
        cfg.existing_bond=done->existing;
        if (done->existing) {
            cfg.old_sc=done->previous.secure_connections;
            cfg.old_authenticated=done->previous.authenticated;
            cfg.old_key_size=done->previous.key_size;
            if (resolvable(slot.peer_type,slot.peer)) {
                slot.resolved=true; slot.resolved_type=done->previous.identity_type;
                memcpy(slot.resolved_identity,done->previous.identity,6);
                guard_entry_t *g=guard_for(slot.resolved_type,slot.resolved_identity,now);
                if (!g || !dbt_smp_guard_allowed(&g->guard,now)) { wipe(&cfg,sizeof(cfg)); slot.failed=true; return; }
                slot.guard=g;
            }
        }
        cfg.local_keys=done->local_keys;
        cfg.have_local_keys=cfg.features.bonding &&
            (slot.initiator ? cfg.features.initiator_keys : cfg.features.responder_keys)!=0;
        dbt_result_t rc=dbt_smp_session_open(&session,runtime,slot.handle,slot.generation,
            configuration.local_address_type,configuration.local_address,slot.peer_type,slot.peer,
            &slot.guard->guard,&cfg,now);
        trace_smp(slot.handle,slot.generation,10,(int)rc,0,cfg.old_key_size,request_window(now));
        wipe(&cfg,sizeof(cfg));
        if (rc==DBT_OK && slot.initiator) rc=dbt_smp_session_start(&session,now);
        if (rc==DBT_OK && slot.inbox_size) {
            const dbt_link_t *l=dbt_acl_find(&runtime->acl,slot.handle);
            rc=dbt_smp_session_receive(&session,l,SMP_CID,slot.inbox,slot.inbox_size);
            trace_smp(slot.handle,slot.generation,11,(int)rc,(uint8_t)session.phase,slot.inbox_size,request_window(now));
        }
        wipe(slot.inbox,sizeof(slot.inbox)); slot.inbox_size=0;
        if (rc!=DBT_OK) slot.failed=true; else slot.prepared=true;
    } else if (done->kind==WORK_CRYPTO) {
        if (dbt_smp_session_crypto_complete(&session,done->generation,done->serial,done->success,
            done->crypto_kind==DBT_SMP_SESSION_CRYPTO_LEGACY ? &done->crypto.legacy : NULL,
            done->crypto_kind==DBT_SMP_SESSION_CRYPTO_SC ? &done->crypto.sc : NULL,now)!=DBT_OK)
            slot.failed=true;
    } else if (done->kind==WORK_STORE) {
        if (dbt_smp_session_store_complete(&session,done->generation,done->serial,done->success,now)!=DBT_OK)
            slot.failed=true;
        else if (done->success && stable_identity(done->identity_type,done->identity)) {
            slot.committed_identity_type=done->identity_type;
            memcpy(slot.committed_identity,done->identity,6);
            slot.committed_identity_valid=true;
            slot.committed_tag_valid=done->tag_valid;
            if(done->tag_valid)memcpy(slot.committed_tag,done->bond_tag,sizeof(slot.committed_tag));
        }
    }
}
void probe_smp_poll(uint32_t now) {
    if (!runtime) return;
    security_request_poll(now);
    completion_t done; uint8_t token;
    while (xQueueReceive(done_queue,&token,0)==pdTRUE) {
        done=completion_mailbox; wipe(&completion_mailbox,sizeof(completion_mailbox));
        complete(&done,now); wipe(&done,sizeof(done));
    }
    if (maintenance.active) {
        if (!maintenance.issued) {
            if (!worker_live && xTaskCreate(worker,"native_smp_worker",WORKER_STACK_BYTES,NULL,1,NULL)!=pdPASS) {
                maintenance.result=DBT_NO_SPACE; maintenance.issued=true;
                maintenance.done=true; maintenance.retired=true;
            } else {
                worker_live=true; memset(&pending,0,sizeof(pending)); pending.kind=maintenance.kind;
                pending.generation=maintenance.generation; pending.payload.store.op=maintenance.operation;
                pending.payload.store.identity_type=maintenance.type;
                memcpy(pending.payload.store.identity,maintenance.identity,6);
                pending_valid=true; maintenance.issued=true;
            }
        }
        if (maintenance.done && !maintenance.retiring && !maintenance.retired && !work_inflight && !pending_valid) {
            memset(&pending,0,sizeof(pending)); pending.kind=WORK_RETIRE; pending.generation=maintenance.generation;
            pending_valid=true; maintenance.retiring=true;
        }
        enqueue(); return;
    }
    if (!slot.active) return;
    const dbt_link_t *link=dbt_acl_find(&runtime->acl,slot.handle);
    if (!link || link->generation!=slot.generation) slot.disconnected=true;
    if (slot.reconnect && !slot.verified && (int32_t)(now-slot.prepare_deadline)>=0) {
        if (slot.reconnect_central || slot.reply_pending || slot.reply_accepted) slot.failed=true;
        else { slot.negative_needed=true; wipe(slot.key,sizeof(slot.key)); slot.key_ready=false; }
    } else if (!slot.reconnect && !slot.prepared && (int32_t)(now-slot.prepare_deadline)>=0)
        slot.failed=true;
    if (slot.failed || slot.disconnected) {
        wipe(slot.key,sizeof(slot.key)); slot.key_ready=false;
        if (slot.reconnect) cancel_reconnect_reply();
        if (session.uncertain_generation) quarantine(slot.handle);
        if (slot.failed && !slot.failure_reported && !failure.pending) {
            failure.pending=true; failure.handle=slot.handle; failure.generation=slot.generation;
            slot.failure_reported=true;
        }
        if (pending_valid && pending.kind!=WORK_RETIRE) { wipe(&pending,sizeof(pending)); pending_valid=false; }
        if (!slot.reconnect && !session.runtime && !slot.guard_reported) {
            dbt_smp_guard_failure(&slot.guard->guard,now); slot.guard_reported=true;
        }
        configuration.pairing.have_passkey=false; configuration.pairing.passkey=0;
        if (session.runtime && dbt_smp_session_close(&session)==DBT_BUSY) return;
        if (slot.reply_pending) return;
        if (reconnect) {
            dbt_smp_reconnect_close(reconnect);
            runtime->hooks.release(runtime->hooks.context,reconnect_allocation);
            reconnect=NULL; reconnect_allocation=NULL;
        }
        slot.guard_reported=true;
        if (!worker_live && !work_inflight) slot.retired=true;
        if (worker_live && !work_inflight && !slot.retiring && !slot.retired) {
            memset(&pending,0,sizeof(pending)); pending.kind=WORK_RETIRE;
            pending.generation=slot.generation; pending_valid=true; slot.retiring=true;
        }
        enqueue();
        if (work_inflight || pending_valid || !slot.retired) return;
        wipe(&slot,sizeof(slot)); return;
    }
    if (slot.reconnect && slot.negative_needed) {
        if (!slot.negative_started) {
            slot.negative_started=true; slot.negative_deadline=now+runtime->host.timeout_ms;
        } else if (!slot.reply_pending && (int32_t)(now-slot.negative_deadline)>=0) {
            slot.failed=true; return;
        }
        if (pending_valid && pending.kind==WORK_LOAD) { wipe(&pending,sizeof(pending)); pending_valid=false; }
        reconnect_reply(now); return;
    }
    if (!slot.prepare_queued) {
        if (!worker_live) {
            if (xTaskCreate(worker,"native_smp_worker",WORKER_STACK_BYTES,NULL,1,NULL)!=pdPASS) {
                if (slot.reconnect && !slot.reconnect_central) slot.negative_needed=true; else slot.failed=true;
                return;
            }
            worker_live=true;
        }
        if (slot.reconnect) {
            if (!reconnect_prepare()) {
                if (slot.reconnect_central) slot.failed=true; else slot.negative_needed=true;
                return;
            }
        } else prepare();
    }
    enqueue();
    if (slot.reconnect) {
        reconnect_reply(now);
        if (reconnect && reconnect->state==DBT_SMP_RECONNECT_PROMOTE && !slot.promotion_issued &&
            !pending_valid && !work_inflight) {
            memset(&pending,0,sizeof(pending)); pending.kind=WORK_PROMOTE; pending.generation=slot.generation;
            if (dbt_smp_reconnect_job(reconnect,&pending.payload.store)==DBT_OK) {
                pending_valid=true; slot.promotion_issued=true;
            } else slot.failed=true;
            enqueue();
        }
        if (slot.verified && !slot.retired && !slot.retiring && !work_inflight && !pending_valid) {
            memset(&pending,0,sizeof(pending)); pending.kind=WORK_RETIRE;
            pending.generation=slot.generation; pending_valid=true; slot.retiring=true;
            enqueue();
        }
        return;
    }
    if (!slot.prepared) return;
    dbt_smp_session_poll(&session,now);
    if (session.phase==DBT_SMP_SESSION_FAILED) { slot.failed=true; return; }
    if (session.result_reported && !slot.retired && !slot.retiring && !work_inflight && !pending_valid) {
        memset(&pending,0,sizeof(pending)); pending.kind=WORK_RETIRE;
        pending.generation=slot.generation; pending_valid=true; slot.retiring=true;
        enqueue();
    }
    if (!pending_valid && !work_inflight &&
        (session.phase==DBT_SMP_SESSION_LEGACY || session.phase==DBT_SMP_SESSION_SC)) {
        memset(&pending,0,sizeof(pending));
        pending.kind=WORK_CRYPTO; pending.generation=slot.generation;
        dbt_result_t rc=dbt_smp_session_crypto_job(&session,&pending.payload.crypto);
        if (rc==DBT_OK) pending_valid=true;
        else {
            wipe(&pending,sizeof(pending)); pending.kind=WORK_STORE; pending.generation=slot.generation;
            rc=dbt_smp_session_store_job(&session,&pending.payload.store);
            if (rc==DBT_OK && slot.resolved &&
                (pending.payload.store.identity_type!=slot.resolved_type ||
                 memcmp(pending.payload.store.identity,slot.resolved_identity,6))) {
                (void)dbt_smp_session_store_complete(&session,slot.generation,pending.payload.store.serial,false,now);
                slot.failed=true; wipe(&pending,sizeof(pending));
            } else if (rc==DBT_OK) pending_valid=true; else wipe(&pending,sizeof(pending));
            if (pending_valid && resolvable(slot.peer_type,slot.peer)) {
                pending.require_rpa_binding=true; memcpy(pending.binding_rpa,slot.peer,6);
            }
        }
    }
    enqueue();
}
uint32_t probe_smp_wait_ms(uint32_t now) {
    if (!runtime)return UINT32_MAX;
    uint32_t wait=UINT32_MAX;
    if(configuration.peripheral_security_requests){
        uint32_t window=request_window(now);
        for(unsigned i=0;i<DBT_MAX_LINKS;i++){
            const security_request_t *r=&security_requests[i];
            if(r->used && r->loan){uint32_t delay=(int32_t)(now-r->deadline)>=0?0:r->deadline-now;if(delay<wait)wait=delay;}
        }
        if(runtime->ready && runtime->acl.mtu[DBT_LINK_LE]>=6 && !request_frozen() && !slot.active && !maintenance.active){
            for(unsigned i=0;i<DBT_MAX_LINKS;i++){
                const dbt_link_t *l=&runtime->acl.links[i];
                if(!l->used || l->kind!=DBT_LINK_LE || l->role!=1 || l->encrypted || quarantined(l->handle))continue;
                security_request_t *r=request_for(l->handle,l->generation);
                if(!r){bool free_slot=false;for(unsigned j=0;j<DBT_MAX_LINKS;j++)if(!security_requests[j].used)free_slot=true;
                    uint32_t delay=free_slot?0:WAIT_MS;if(delay<wait)wait=delay;continue;}
                if(!r->loan && (window?r->last_window!=window:!r->outside_done)){
                    uint32_t delay=(int32_t)(now-r->next_allowed)>=0?0:r->next_allowed-now;if(delay<wait)wait=delay;
                }
            }
        }
    }
    if(slot.active || maintenance.active){
        if(uxQueueMessagesWaiting(done_queue) || (pending_valid && !work_inflight))return 0;
        if(WAIT_MS<wait)wait=WAIT_MS;
    }
    return wait;
}
bool probe_smp_quiescent(void){
    for(unsigned i=0;i<DBT_MAX_LINKS;i++)if(security_requests[i].loan)return false;
    if(!runtime || storage_poisoned || maintenance.active || pending_valid || work_inflight || worker_live ||
       !work_queue || !done_queue || uxQueueMessagesWaiting(work_queue) || uxQueueMessagesWaiting(done_queue))return false;
    if(configuration.peripheral_security_requests && runtime->ready && runtime->acl.mtu[DBT_LINK_LE]>=6 &&
       !request_frozen() && !slot.active){
        uint32_t window=request_window(runtime->now_ms);
        for(unsigned i=0;i<DBT_MAX_LINKS;i++){
            const dbt_link_t *l=&runtime->acl.links[i];
            if(!l->used || l->kind!=DBT_LINK_LE || l->role!=1 || l->encrypted || quarantined(l->handle))continue;
            security_request_t *r=request_for(l->handle,l->generation);
            if(!r || (window?r->last_window!=window:!r->outside_done))return false;
        }
    }
    if(!slot.active)return true;
    if(slot.failed || slot.disconnected || !slot.retired || slot.reply_pending)return false;
    return slot.reconnect?slot.verified:(session.runtime && session.result_reported && session.phase!=DBT_SMP_SESSION_FAILED);
}
bool probe_smp_take_failure(uint16_t *handle,uint32_t *generation) {
    if (!handle || !generation || !failure.pending) return false;
    *handle=failure.handle; *generation=failure.generation; failure.pending=false; return true;
}
dbt_result_t probe_smp_security(uint16_t handle,uint32_t generation,dbt_smp_session_security_t *out) {
    if (!slot.active || slot.handle!=handle || slot.generation!=generation || !out)
        return DBT_INVALID;
    if (slot.reconnect) {
        const dbt_link_t *l=dbt_acl_find(&runtime->acl,handle);
        if (slot.disconnected || !l || l->generation!=generation) return DBT_INVALID;
        memset(out,0,sizeof(*out));
        if (l->encrypted && slot.encrypted_verified && !slot.failed && reconnect &&
            (reconnect->state==DBT_SMP_RECONNECT_SECURE || reconnect->state==DBT_SMP_RECONNECT_PROMOTE)) {
            out->encrypted=true; out->authenticated=slot.reconnect_authenticated;
            out->key_size=slot.reconnect_key_size; out->bond_committed=true; out->peer_key_verified=true;
            out->identity_promotion_committed=reconnect->identity_promotion_committed;
        }
        return DBT_OK;
    }
    if (!session.runtime) return DBT_INVALID;
    return dbt_smp_session_security(&session,out);
}
dbt_result_t probe_smp_committed_identity(uint16_t handle,uint32_t generation,uint8_t *type,uint8_t identity[6]) {
    if (!type || !identity || !slot.active || slot.disconnected || slot.failed ||
        slot.handle!=handle || slot.generation!=generation || !slot.committed_identity_valid)
        return DBT_INVALID;
    dbt_smp_session_security_t security;
    if (probe_smp_security(handle,generation,&security)!=DBT_OK || !security.encrypted ||
        !security.bond_committed || (slot.reconnect && !slot.verified)) return DBT_BUSY;
    *type=slot.committed_identity_type; memcpy(identity,slot.committed_identity,6);
    return DBT_OK;
}
dbt_result_t probe_smp_committed_bond(uint16_t handle,uint32_t generation,uint8_t *type,
                                      uint8_t identity[6],uint8_t tag[PROBE_CCCD_TAG_SIZE]) {
    if(!tag)return DBT_INVALID;
    dbt_result_t rc=probe_smp_committed_identity(handle,generation,type,identity);
    if(rc!=DBT_OK)return rc;
    if(!slot.committed_tag_valid)return DBT_INVALID;
    memcpy(tag,slot.committed_tag,PROBE_CCCD_TAG_SIZE);return DBT_OK;
}
dbt_result_t probe_smp_numeric_value(uint32_t *value) {
    return dbt_smp_session_numeric_value(&session,value);
}
dbt_result_t probe_smp_numeric_confirm(bool accepted,uint32_t now) {
    return dbt_smp_session_numeric_confirm(&session,accepted,now);
}
static dbt_result_t maintenance_start(work_kind_t kind,dbt_smp_bond_op_t op,uint8_t type,const uint8_t *id) {
    if (!runtime) return DBT_INVALID;
    if (slot.active || maintenance.active || work_inflight || pending_valid) return DBT_BUSY;
    if (maintenance_serial==UINT32_MAX) return DBT_NO_SPACE;
    memset(&maintenance,0,sizeof(maintenance)); maintenance.active=true;
    maintenance.generation=++maintenance_serial; maintenance.kind=kind;
    maintenance.operation=op; maintenance.type=type; if (id) memcpy(maintenance.identity,id,6);
    return DBT_OK;
}
dbt_result_t probe_smp_storage_inspect_start(void) {
    return maintenance_start(WORK_INSPECT,DBT_SMP_BOND_NONE,0,NULL);
}
dbt_result_t probe_smp_storage_reconcile_start(dbt_smp_bond_op_t op,uint8_t type,const uint8_t id[6]) {
    if (!id || type>1 || (op!=DBT_SMP_BOND_STORE && op!=DBT_SMP_BOND_DELETE)) return DBT_INVALID;
    return maintenance_start(WORK_RECONCILE,op,type,id);
}
dbt_result_t probe_smp_storage_result(dbt_result_t *status,dbt_smp_bond_op_t *op,
                                     uint8_t *type,uint8_t id[6],bool *applied) {
    if (!status || !op || !type || !id || !applied || !maintenance.active) return DBT_INVALID;
    if (!maintenance.done || !maintenance.retired) return DBT_BUSY;
    *status=maintenance.result; *op=maintenance.operation; *type=maintenance.type;
    memcpy(id,maintenance.identity,6); *applied=maintenance.applied;
    wipe(&maintenance,sizeof(maintenance)); return DBT_OK;
}
