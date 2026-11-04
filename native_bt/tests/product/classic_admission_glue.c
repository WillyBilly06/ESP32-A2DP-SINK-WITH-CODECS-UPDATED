#include <assert.h>
#include <string.h>
#include "probe_security.h"
#include "freertos/queue.h"
#include "probe_admission.h"
#define BOND_LIMIT 8u
#ifndef DBT_PROBE_PRODUCT_SECURITY
static const uint8_t allowed_peer[6]={DBT_PROBE_PAIRING_PEER_BYTES};
#endif
typedef struct {uint8_t address[6];dbt_classic_key_t key;bool used;} cached_bond_t;
typedef struct {uint32_t token;uint8_t address[6];dbt_classic_key_t key;bool deleting,committed;} store_job_t;
static cached_bond_t bonds[BOND_LIMIT];static dbt_runtime_t *runtime;static QueueHandle_t work_queue,completion_queue;static unsigned store_inflight;static bool initialized;static dbt_classic_security_t security;
#include "classic_admission_extracted.inc"
static dbt_runtime_t test_runtime;
static bool window,lease,store_allowed,test_frozen;static unsigned queued,queue_messages,completion_messages;
bool probe_admission_window(uint32_t now){(void)now;return window;}
bool probe_admission_frozen(void){return test_frozen;}
bool probe_admission_classic_pair(const dbt_link_t *l,uint32_t now){(void)now;return lease && l && l->generation==9;}
bool probe_admission_classic_store(const uint8_t address[6],uint32_t now){(void)now;return store_allowed && address[0]==1;}
BaseType_t xQueueSend(QueueHandle_t q,const void *data,TickType_t wait){(void)data;(void)wait;assert(q);queued++;queue_messages++;return pdTRUE;}
UBaseType_t uxQueueMessagesWaiting(QueueHandle_t q){return q==work_queue?queue_messages:completion_messages;}
int main(void){
    runtime=&test_runtime;work_queue=(void *)1;completion_queue=(void *)2;initialized=true;assert(probe_security_quiescent());
    uint8_t allowed[6]={1,2,3,4,5,6},unknown[6]={7,2,3,4,5,6};dbt_classic_key_t key={0};
    dbt_link_t *l=&runtime->acl.links[0];l->used=true;l->kind=DBT_LINK_CLASSIC;l->generation=9;memcpy(l->peer,allowed,6);
#ifdef DBT_PROBE_PRODUCT_SECURITY
    assert(!probe_security_accept(NULL,allowed,NULL));window=true;assert(probe_security_accept(NULL,unknown,NULL));
    window=false;bonds[0].used=true;memcpy(bonds[0].address,allowed,6);assert(probe_security_accept(NULL,allowed,NULL));
    test_frozen=true;assert(!probe_security_accept(NULL,allowed,NULL));test_frozen=false;
    assert(!probe_security_accept(NULL,unknown,NULL));assert(!pairing(NULL,allowed));lease=true;assert(pairing(NULL,allowed));
    assert(!store(NULL,allowed,&key,1));store_allowed=true;assert(store(NULL,allowed,&key,1) && queued==1);
    l->generation=10;assert(!pairing(NULL,allowed));
#else
    assert(probe_security_accept(NULL,allowed,NULL) && !probe_security_accept(NULL,unknown,NULL));
    assert(pairing(NULL,allowed) && !pairing(NULL,unknown));
    assert(!store(NULL,unknown,&key,1));assert(store(NULL,allowed,&key,1) && queued==1);
#endif
    assert(!probe_security_quiescent());queue_messages=0;completion_messages=1;
    assert(!probe_security_quiescent());completion_messages=0;assert(!probe_security_quiescent());
    store_inflight=0;assert(probe_security_quiescent());
    security.peers[0].used=true;security.peers[0].connected=true;security.peers[0].accepted_pairing=true;
    assert(!probe_security_quiescent());security.peers[0].ready=true;assert(probe_security_quiescent());
    return 0;
}
