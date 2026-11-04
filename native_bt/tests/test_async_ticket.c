#include "dbt_async_ticket.h"
#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif
#define CHECK(x) do{if(!(x))return __LINE__;}while(0)
EXPORT int test_async_ticket_identity_and_completion(void){
    dbt_async_ticket_t t;dbt_ticket_init(&t);uint32_t id,other;
    CHECK(dbt_ticket_idle(&t) && dbt_ticket_reserve(&t,0,0,&id)==DBT_OK && id);
    CHECK(dbt_ticket_reserve(&t,0,0,&other)==DBT_BUSY && !other);
    CHECK(dbt_ticket_publish(&t,id) && dbt_ticket_poll(&t,id)==DBT_PENDING);
    CHECK(!dbt_ticket_done_begin(&t,id+1) && dbt_ticket_cancel(&t,id+1)==DBT_INVALID);
    CHECK(dbt_ticket_done_begin(&t,id) && dbt_ticket_done_end(&t,id,DBT_NO_SPACE));
    CHECK(dbt_ticket_poll(&t,id+1)==DBT_INVALID && dbt_ticket_poll(&t,id)==DBT_NO_SPACE && dbt_ticket_idle(&t));
    uint32_t old=id;CHECK(dbt_ticket_reserve(&t,0,0,&id)==DBT_OK && dbt_ticket_publish(&t,id));
    CHECK(dbt_ticket_cancel(&t,old)==DBT_INVALID && !dbt_ticket_abort(&t,old));
    CHECK(!dbt_ticket_done_begin(&t,id+(DBT_TICKET_ID_MAX+1u)));
    CHECK(dbt_ticket_done_begin(&t,id) && dbt_ticket_done_end(&t,id,DBT_OK));
    CHECK(!dbt_ticket_done_end(&t,old,DBT_NO_SPACE) && !dbt_ticket_done_end(&t,id,DBT_INVALID));
    CHECK(dbt_ticket_poll(&t,id)==DBT_OK);
    return 0;
}
EXPORT int test_async_ticket_cancel_phase_and_retirement(void){
    for(unsigned phase=0;phase<3;++phase){
        dbt_async_ticket_t t;dbt_ticket_init(&t);uint32_t id;
        CHECK(dbt_ticket_reserve(&t,0,0,&id)==DBT_OK && dbt_ticket_publish(&t,id));
        if(phase)CHECK(dbt_ticket_done_begin(&t,id));
        if(phase==2)CHECK(dbt_ticket_done_end(&t,id,DBT_OK));
        CHECK(dbt_ticket_cancel(&t,id)==DBT_PENDING && dbt_ticket_poll(&t,id)==DBT_PENDING);
        if(phase==1)CHECK(!dbt_ticket_done_end(&t,id,DBT_OK));
        CHECK(!dbt_ticket_idle(&t) && dbt_ticket_reclaimed(&t,id,false));
        CHECK(!dbt_ticket_idle(&t) && dbt_ticket_poll(&t,id)==DBT_CANCELLED && dbt_ticket_idle(&t));
        CHECK(!dbt_ticket_reclaimed(&t,id,false));
    }
    dbt_async_ticket_t t;dbt_ticket_init(&t);uint32_t id;
    CHECK(dbt_ticket_reserve(&t,0,0,&id)==DBT_OK && dbt_ticket_publish(&t,id));
    CHECK(dbt_ticket_cancel(&t,id)==DBT_PENDING && dbt_ticket_reclaimed(&t,id,true) && dbt_ticket_idle(&t));
    return 0;
}
EXPORT int test_async_ticket_wrap_and_reservation_abort(void){
    dbt_async_ticket_t t;dbt_ticket_init(&t);t.next=DBT_TICKET_ID_MAX-1u;uint32_t id;
    CHECK(dbt_ticket_reserve(&t,DBT_TICKET_ID_MAX,1,&id)==DBT_OK && id==2);
    CHECK(dbt_ticket_abort(&t,id) && dbt_ticket_idle(&t));
    CHECK(dbt_ticket_reserve(&t,0,0,&id)==DBT_OK && dbt_ticket_publish(&t,id));
    CHECK(dbt_ticket_abort(&t,id) && dbt_ticket_idle(&t));
    CHECK(dbt_ticket_poll(&t,0)==DBT_INVALID && dbt_ticket_cancel(&t,DBT_TICKET_ID_MAX+1u)==DBT_INVALID);
    atomic_store(&t.tag,DBT_TICKET_RESERVING);
    CHECK(!dbt_ticket_publish(&t,0) && !dbt_ticket_done_begin(&t,0));
    return 0;
}
