#ifndef DBT_ASYNC_TICKET_H
#define DBT_ASYNC_TICKET_H
#include "dbt_queue.h"

/* One bounded result slot. Tag identity and state share a lock-free 32-bit
 * atomic, so a stale poll/cancel cannot mutate a different current ticket.
 * Payload belongs to the caller/owner and remains live until terminal poll.
 * The next-id counter is protected by RESERVING, never an acquired spin lock. */
enum {
    DBT_TICKET_FREE=0,DBT_TICKET_WAITING=1,DBT_TICKET_CANCEL=2,
    DBT_TICKET_DONE=3,DBT_TICKET_COMPLETING=4,DBT_TICKET_CANCELLED=5,
    DBT_TICKET_RECLAIM=6,DBT_TICKET_RESERVING=7,DBT_TICKET_STALE=8
};
#define DBT_TICKET_ID_MAX (UINT32_MAX>>3)
typedef struct {_Atomic uint32_t tag,result;uint32_t next;} dbt_async_ticket_t;
static inline uint32_t dbt_ticket_tag(uint32_t id,unsigned state){return (id<<3)|state;}
static inline void dbt_ticket_init(dbt_async_ticket_t *t){atomic_init(&t->tag,0);atomic_init(&t->result,DBT_INVALID);t->next=0;}
static inline bool dbt_ticket_idle(const dbt_async_ticket_t *t){return !atomic_load_explicit(&t->tag,memory_order_acquire);}
static inline unsigned dbt_ticket_state(const dbt_async_ticket_t *t,uint32_t id){
    uint32_t tag=atomic_load_explicit(&t->tag,memory_order_acquire);
    return id && id<=DBT_TICKET_ID_MAX && (tag>>3)==id?(tag&7u):DBT_TICKET_STALE;
}
static inline dbt_result_t dbt_ticket_reserve(dbt_async_ticket_t *t,uint32_t avoid1,uint32_t avoid2,uint32_t *id){
    if(!t || !id)return DBT_INVALID;
    *id=0;uint32_t expected=0;
    if(!atomic_compare_exchange_strong_explicit(&t->tag,&expected,DBT_TICKET_RESERVING,memory_order_acq_rel,memory_order_acquire))return DBT_BUSY;
    for(unsigned i=0;i<4;++i){
        t->next=(t->next+1u)&DBT_TICKET_ID_MAX;
        if(t->next && t->next!=avoid1 && t->next!=avoid2){
            *id=t->next;atomic_store_explicit(&t->tag,dbt_ticket_tag(*id,DBT_TICKET_RESERVING),memory_order_release);return DBT_OK;
        }
    }
    atomic_store_explicit(&t->tag,0,memory_order_release);return DBT_INVALID;
}
static inline bool dbt_ticket_publish(dbt_async_ticket_t *t,uint32_t id){
    if(!id || id>DBT_TICKET_ID_MAX)return false;
    uint32_t expected=dbt_ticket_tag(id,DBT_TICKET_RESERVING);
    return atomic_compare_exchange_strong_explicit(&t->tag,&expected,dbt_ticket_tag(id,DBT_TICKET_WAITING),memory_order_acq_rel,memory_order_acquire);
}
/* Only a reservation whose queue publication failed may be aborted. */
static inline bool dbt_ticket_abort(dbt_async_ticket_t *t,uint32_t id){
    unsigned state=dbt_ticket_state(t,id);
    if(state!=DBT_TICKET_RESERVING && state!=DBT_TICKET_WAITING)return false;
    uint32_t expected=dbt_ticket_tag(id,state);
    return atomic_compare_exchange_strong_explicit(&t->tag,&expected,0,memory_order_acq_rel,memory_order_acquire);
}
static inline bool dbt_ticket_done_begin(dbt_async_ticket_t *t,uint32_t id){
    if(!id || id>DBT_TICKET_ID_MAX)return false;
    uint32_t expected=dbt_ticket_tag(id,DBT_TICKET_WAITING);
    return atomic_compare_exchange_strong_explicit(&t->tag,&expected,dbt_ticket_tag(id,DBT_TICKET_COMPLETING),memory_order_acq_rel,memory_order_acquire);
}
static inline bool dbt_ticket_done_end(dbt_async_ticket_t *t,uint32_t id,dbt_result_t result){
    if(dbt_ticket_state(t,id)!=DBT_TICKET_COMPLETING)return false;
    atomic_store_explicit(&t->result,(uint32_t)result,memory_order_relaxed);
    uint32_t expected=dbt_ticket_tag(id,DBT_TICKET_COMPLETING);
    return atomic_compare_exchange_strong_explicit(&t->tag,&expected,dbt_ticket_tag(id,DBT_TICKET_DONE),memory_order_acq_rel,memory_order_acquire);
}
/* Cancellation is accepted in WAITING/COMPLETING, or requests owner-side
 * reclamation after DONE. It never equates an invalid/stale id with retirement. */
static inline dbt_result_t dbt_ticket_cancel(dbt_async_ticket_t *t,uint32_t id){
    unsigned state=dbt_ticket_state(t,id);
    if(state==DBT_TICKET_CANCEL || state==DBT_TICKET_RECLAIM || state==DBT_TICKET_CANCELLED)return DBT_PENDING;
    if(state!=DBT_TICKET_WAITING && state!=DBT_TICKET_COMPLETING && state!=DBT_TICKET_DONE)return DBT_INVALID;
    uint32_t expected=dbt_ticket_tag(id,state);
    return atomic_compare_exchange_strong_explicit(&t->tag,&expected,dbt_ticket_tag(id,state==DBT_TICKET_DONE?DBT_TICKET_RECLAIM:DBT_TICKET_CANCEL),memory_order_acq_rel,memory_order_acquire)?DBT_PENDING:DBT_BUSY;
}
static inline bool dbt_ticket_reclaimed(dbt_async_ticket_t *t,uint32_t id,bool legacy_auto_release){
    unsigned state=dbt_ticket_state(t,id);
    if(state!=DBT_TICKET_CANCEL && state!=DBT_TICKET_RECLAIM)return false;
    uint32_t expected=dbt_ticket_tag(id,state);
    return atomic_compare_exchange_strong_explicit(&t->tag,&expected,legacy_auto_release?0:dbt_ticket_tag(id,DBT_TICKET_CANCELLED),memory_order_acq_rel,memory_order_acquire);
}
static inline dbt_result_t dbt_ticket_poll(dbt_async_ticket_t *t,uint32_t id){
    unsigned state=dbt_ticket_state(t,id);
    if(state==DBT_TICKET_WAITING || state==DBT_TICKET_COMPLETING || state==DBT_TICKET_CANCEL || state==DBT_TICKET_RECLAIM)return DBT_PENDING;
    if(state!=DBT_TICKET_DONE && state!=DBT_TICKET_CANCELLED)return DBT_INVALID;
    dbt_result_t result=state==DBT_TICKET_CANCELLED?DBT_CANCELLED:(dbt_result_t)atomic_load_explicit(&t->result,memory_order_relaxed);
    uint32_t expected=dbt_ticket_tag(id,state);
    return atomic_compare_exchange_strong_explicit(&t->tag,&expected,0,memory_order_acq_rel,memory_order_acquire)?result:DBT_PENDING;
}
#endif
