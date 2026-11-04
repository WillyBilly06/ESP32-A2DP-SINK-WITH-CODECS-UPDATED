#include "dbt_rfcomm_dlc.h"
#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif
#define CHECK(x) do{if(!(x))return __LINE__;}while(0)
static uint8_t tx[256],wire[256];static dbt_rfcomm_dlc_t d;
static int inject(uint8_t type,const uint8_t *value,uint16_t len){
    uint8_t item[130];uint16_t item_size=0,wire_size=0;dbt_rfcomm_mcc_t m={value,len,type,true};
    if(dbt_rfcomm_mcc_encode(&m,item,sizeof(item),&item_size)!=DBT_OK)return -1;
    dbt_rfcomm_frame_t f={item,item_size,0,0,DBT_RFCOMM_UIH,dbt_rfcomm_cr(!d.mux_initiator,true),false,false};
    if(dbt_rfcomm_encode(&f,false,wire,sizeof(wire),&wire_size)!=DBT_OK)return -2;
    return dbt_rfcomm_dlc_receive(&d,wire,wire_size);
}
static int response(uint8_t type,const uint8_t *expected,unsigned n){
    const uint8_t *p;uint16_t size;dbt_rfcomm_frame_t f;dbt_rfcomm_mcc_t m;
    CHECK(dbt_rfcomm_dlc_peek(&d,&p,&size)==DBT_OK && dbt_rfcomm_decode(p,size,false,&f)==DBT_OK);
    CHECK(dbt_rfcomm_mcc_decode(f.data,f.size,&m)==DBT_OK && !m.command && m.type==type && m.size==n);
    for(unsigned i=0;i<n;++i)CHECK(m.data[i]==expected[i]);
    CHECK(dbt_rfcomm_dlc_commit(&d,1)==DBT_OK);return 0;
}
EXPORT int rfcomm_dlc_mcc_legacy_flow_and_nsc(void){
    CHECK(dbt_rfcomm_dlc_init(&d,true,tx,sizeof(tx),(dbt_rfcomm_dlc_hooks_t){0})==DBT_OK);
    d.state=DBT_RFCOMM_DLC_OPEN;d.dlci=6;d.max_frame=180;d.credit_mode=false;
    CHECK(inject(0x18,0,0)==DBT_OK && d.aggregate_flow_off);
    uint8_t data=42;CHECK(dbt_rfcomm_dlc_send(&d,&data,1)==DBT_BUSY);
    int r=response(0x18,0,0);if(r)return r;
    CHECK(inject(0x28,0,0)==DBT_OK && !d.aggregate_flow_off);r=response(0x28,0,0);if(r)return r;
    const uint8_t break_status[]={27,0x8c,3};CHECK(inject(0x38,break_status,3)==DBT_OK);
    r=response(0x38,break_status,3);if(r)return r;
    const uint8_t request[]={27},defaults[]={27,3,3,0,0x11,0x13,0,0};
    CHECK(inject(0x24,request,1)==DBT_OK);r=response(0x24,defaults,8);if(r)return r;
    const uint8_t changed[]={27,7,3,0,0x11,0x13,0x7f,0x3f};CHECK(inject(0x24,changed,8)==DBT_OK);
    r=response(0x24,defaults,8);if(r)return r;
    const uint8_t line[]={27,3};CHECK(inject(0x14,line,2)==DBT_OK && d.line_status==3);
    r=response(0x14,line,2);if(r)return r;
    const uint8_t unknown_type[]={0x5b};CHECK(inject(0x16,0,0)==DBT_OK);
    r=response(0x04,unknown_type,1);if(r)return r;
    CHECK(dbt_rfcomm_dlc_send(&d,&data,1)==DBT_OK);const uint8_t *p;uint16_t n;
    CHECK(dbt_rfcomm_dlc_peek(&d,&p,&n)==DBT_OK && dbt_rfcomm_dlc_queued_data(&d));
    CHECK(dbt_rfcomm_dlc_abort_queued_data(&d)==DBT_OK && d.state==DBT_RFCOMM_DLC_OPEN && !d.tx_ready);
    CHECK(dbt_rfcomm_dlc_send(&d,&data,1)==DBT_OK);
    CHECK(dbt_rfcomm_dlc_peek(&d,&p,&n)==DBT_OK && dbt_rfcomm_dlc_commit(&d,2)==DBT_OK);return 0;
}
EXPORT int rfcomm_dlc_pn_duplicate_and_deadline(void){
    CHECK(dbt_rfcomm_dlc_init(&d,false,tx,sizeof(tx),(dbt_rfcomm_dlc_hooks_t){0})==DBT_OK);
    CHECK(dbt_rfcomm_dlc_listen(&d,3,180,1)==DBT_OK);
    const uint8_t pn[]={6,0xf0,0,0,180,0,0,1},reply[]={6,0xe0,0,0,180,0,0,1};
    CHECK(inject(0x20,pn,8)==DBT_OK && d.state==DBT_RFCOMM_DLC_PN_REPLY);
    CHECK(inject(0x20,pn,8)==DBT_BUSY && d.credits.tx_available==1 && d.credits.rx_outstanding==1);
    int r=response(0x20,reply,8);if(r)return r;CHECK(d.state==DBT_RFCOMM_DLC_WAIT_SABM);
    CHECK(inject(0x20,pn,8)==DBT_OK && d.credits.tx_available==1 && d.credits.rx_outstanding==1);
    r=response(0x20,reply,8);if(r)return r;CHECK(d.state==DBT_RFCOMM_DLC_WAIT_SABM);
    dbt_rfcomm_dlc_poll(&d,60000);CHECK(d.state==DBT_RFCOMM_DLC_WAIT_SABM);
    dbt_rfcomm_dlc_poll(&d,60001);CHECK(d.state==DBT_RFCOMM_DLC_FAILED);
    CHECK(dbt_rfcomm_dlc_init(&d,true,tx,sizeof(tx),(dbt_rfcomm_dlc_hooks_t){0})==DBT_OK);
    CHECK(dbt_rfcomm_dlc_open(&d,3,180,1)==DBT_OK);const uint8_t *p;uint16_t n;
    CHECK(dbt_rfcomm_dlc_peek(&d,&p,&n)==DBT_OK && dbt_rfcomm_dlc_commit(&d,0xfffffff0u)==DBT_OK);
    dbt_rfcomm_dlc_poll(&d,19983);CHECK(d.state==DBT_RFCOMM_DLC_WAIT_PN);
    dbt_rfcomm_dlc_poll(&d,19984);CHECK(d.state==DBT_RFCOMM_DLC_FAILED);return 0;
}
EXPORT int rfcomm_dlc_reverse_direction(void){
    CHECK(dbt_rfcomm_dlc_init(&d,false,tx,sizeof(tx),(dbt_rfcomm_dlc_hooks_t){0})==DBT_OK);
    CHECK(dbt_rfcomm_dlc_open(&d,3,100,1)==DBT_OK && d.dlci==7);
    const uint8_t *p;uint16_t n;dbt_rfcomm_frame_t f;dbt_rfcomm_mcc_t m;
    CHECK(dbt_rfcomm_dlc_peek(&d,&p,&n)==DBT_OK && dbt_rfcomm_decode(p,n,false,&f)==DBT_OK && !f.cr);
    CHECK(dbt_rfcomm_mcc_decode(f.data,f.size,&m)==DBT_OK && m.command && m.type==0x20 && m.data[0]==7);
    CHECK(dbt_rfcomm_dlc_cancel(&d)==DBT_OK);
    CHECK(dbt_rfcomm_dlc_init(&d,true,tx,sizeof(tx),(dbt_rfcomm_dlc_hooks_t){0})==DBT_OK);
    CHECK(dbt_rfcomm_dlc_listen(&d,3,100,1)==DBT_OK);
    const uint8_t pn[]={7,0xf0,0,0,100,0,0,1};CHECK(inject(0x20,pn,8)==DBT_OK && d.dlci==7);
    const uint8_t reply[]={7,0xe0,0,0,100,0,0,1};return response(0x20,reply,8);
}
