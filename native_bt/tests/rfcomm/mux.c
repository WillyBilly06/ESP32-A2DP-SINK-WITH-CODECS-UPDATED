#include "dbt_rfcomm_mux.h"
#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif
#define CHECK(x) do{if(!(x))return __LINE__;}while(0)
static dbt_rfcomm_mux_t a,b;
static int init(void){
    CHECK(dbt_rfcomm_mux_init(&a,true,1,7,0x40,10000)==DBT_OK);
    CHECK(dbt_rfcomm_mux_init(&b,false,1,7,0x41,10000)==DBT_OK);return 0;
}
static dbt_result_t move(dbt_rfcomm_mux_t *from,dbt_rfcomm_mux_t *to,uint32_t now){
    const uint8_t *p;uint16_t n;dbt_result_t r=dbt_rfcomm_mux_peek(from,&p,&n);if(r!=DBT_OK)return r;
    uint8_t copy[4];for(unsigned i=0;i<n;++i)copy[i]=p[i];
    r=dbt_rfcomm_mux_commit(from,now);if(r!=DBT_OK)return r;
    return dbt_rfcomm_mux_receive(to,to->handle,to->generation,to->local_cid,copy,n);
}
EXPORT int rfcomm_mux_pair_and_close(void){
    int r=init();if(r)return r;CHECK(dbt_rfcomm_mux_start(&a)==DBT_OK);
    CHECK(move(&a,&b,10)==DBT_OK && a.state==DBT_RFCOMM_MUX_WAIT_UA && b.state==DBT_RFCOMM_MUX_ACCEPT_QUEUED);
    CHECK(move(&b,&a,11)==DBT_OK && a.state==DBT_RFCOMM_MUX_OPEN && b.state==DBT_RFCOMM_MUX_OPEN);
    const uint8_t duplicate[]={3,0x3f,1,0x1c};
    CHECK(dbt_rfcomm_mux_receive(&b,1,7,0x41,duplicate,4)==DBT_OK && b.state==DBT_RFCOMM_MUX_OPEN);
    const uint8_t *p;uint16_t n;CHECK(dbt_rfcomm_mux_peek(&b,&p,&n)==DBT_OK && n==4);
    CHECK(dbt_rfcomm_mux_commit(&b,12)==DBT_OK && b.state==DBT_RFCOMM_MUX_OPEN);
    CHECK(dbt_rfcomm_mux_close(&a)==DBT_OK && move(&a,&b,13)==DBT_OK);
    CHECK(b.state==DBT_RFCOMM_MUX_REPLY_QUEUED && a.state==DBT_RFCOMM_MUX_WAIT_DISC);
    CHECK(move(&b,&a,14)==DBT_OK && a.state==DBT_RFCOMM_MUX_DOWN && b.state==DBT_RFCOMM_MUX_DOWN);
    return 0;
}
EXPORT int rfcomm_mux_identity_backpressure(void){
    int r=init();if(r)return r;const uint8_t sabm[]={3,0x3f,1,0x1c};
    CHECK(dbt_rfcomm_mux_receive(&b,1,8,0x41,sabm,4)==DBT_INVALID && b.state==DBT_RFCOMM_MUX_DOWN);
    CHECK(dbt_rfcomm_mux_receive(&b,1,7,0x40,sabm,4)==DBT_INVALID && b.state==DBT_RFCOMM_MUX_DOWN);
    CHECK(dbt_rfcomm_mux_receive(&b,1,7,0x41,sabm,4)==DBT_OK);
    const uint8_t *p,*p2;uint16_t n,n2;CHECK(dbt_rfcomm_mux_peek(&b,&p,&n)==DBT_OK);
    CHECK(dbt_rfcomm_mux_peek(&b,&p2,&n2)==DBT_OK && p==p2 && n==n2);
    CHECK(dbt_rfcomm_mux_link_lost(&b,1,7)==DBT_BUSY && b.state==DBT_RFCOMM_MUX_ACCEPT_QUEUED);
    CHECK(dbt_rfcomm_mux_cancel(&b)==DBT_OK && b.state==DBT_RFCOMM_MUX_DOWN);
    CHECK(dbt_rfcomm_mux_link_lost(&b,1,8)==DBT_INVALID && dbt_rfcomm_mux_link_lost(&b,1,7)==DBT_OK);
    return 0;
}
EXPORT int rfcomm_mux_admission_timeout_and_role(void){
    int r=init();if(r)return r;CHECK(dbt_rfcomm_mux_start(&a)==DBT_OK);
    const uint8_t ua[]={3,0x73,1,0xd7};
    CHECK(dbt_rfcomm_mux_receive(&a,1,7,0x40,ua,4)==DBT_INVALID && a.state==DBT_RFCOMM_MUX_START_QUEUED);
    dbt_rfcomm_mux_poll(&a,200000);CHECK(a.state==DBT_RFCOMM_MUX_START_QUEUED);
    const uint8_t *p;uint16_t n;CHECK(dbt_rfcomm_mux_peek(&a,&p,&n)==DBT_OK);
    CHECK(dbt_rfcomm_mux_commit(&a,0xfffffff0u)==DBT_OK);
    dbt_rfcomm_mux_poll(&a,9983);CHECK(a.state==DBT_RFCOMM_MUX_WAIT_UA);
    dbt_rfcomm_mux_poll(&a,9984);CHECK(a.state==DBT_RFCOMM_MUX_DOWN && a.timed_out);
    CHECK(dbt_rfcomm_mux_receive(&a,1,7,0x40,ua,4)==DBT_INVALID);
    const uint8_t badrole[]={1,0x3f,1,0x9a};
    CHECK(dbt_rfcomm_mux_receive(&b,1,7,0x41,badrole,4)!=DBT_OK && b.state==DBT_RFCOMM_MUX_DOWN);
    return 0;
}
EXPORT int rfcomm_mux_crossed_disconnect(void){
    int r=init();if(r)return r;CHECK(dbt_rfcomm_mux_start(&a)==DBT_OK);
    CHECK(move(&a,&b,1)==DBT_OK && move(&b,&a,2)==DBT_OK);
    CHECK(dbt_rfcomm_mux_close(&a)==DBT_OK && dbt_rfcomm_mux_close(&b)==DBT_OK);
    const uint8_t *p;uint16_t n;uint8_t from_a[4],from_b[4];
    CHECK(dbt_rfcomm_mux_peek(&a,&p,&n)==DBT_OK);for(unsigned i=0;i<4;++i)from_a[i]=p[i];
    CHECK(dbt_rfcomm_mux_commit(&a,3)==DBT_OK);
    CHECK(dbt_rfcomm_mux_receive(&b,1,7,0x41,from_a,4)==DBT_BUSY);
    CHECK(dbt_rfcomm_mux_peek(&b,&p,&n)==DBT_OK);for(unsigned i=0;i<4;++i)from_b[i]=p[i];
    CHECK(dbt_rfcomm_mux_commit(&b,4)==DBT_OK);
    CHECK(dbt_rfcomm_mux_receive(&a,1,7,0x40,from_b,4)==DBT_OK);
    CHECK(dbt_rfcomm_mux_receive(&b,1,7,0x41,from_a,4)==DBT_OK);
    CHECK(move(&a,&b,5)==DBT_OK && move(&b,&a,6)==DBT_OK);
    CHECK(a.state==DBT_RFCOMM_MUX_DOWN && b.state==DBT_RFCOMM_MUX_DOWN && !a.disc_crossed && !b.disc_crossed);
    return 0;
}
