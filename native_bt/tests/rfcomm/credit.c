#include "dbt_rfcomm_credit.h"
#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif
#define CHECK(x) do{if(!(x))return __LINE__;}while(0)
EXPORT int rfcomm_pn_and_credit(void){
    const uint8_t v[]={2,0xf0,7,0,122,0,0,7};
    dbt_rfcomm_mcc_t message={v,8,0x20,true};dbt_rfcomm_pn_t p;
    CHECK(dbt_rfcomm_pn_decode(&message,&p)==DBT_OK && p.dlci==2 && p.priority==7 && p.max_frame==122 && p.credits==7 && p.command);
    uint8_t bytes[8];dbt_rfcomm_mcc_t built;CHECK(dbt_rfcomm_pn_encode(&p,bytes,&built)==DBT_OK);
    for(unsigned i=0;i<8;++i)CHECK(bytes[i]==v[i]);
    p.command=false;CHECK(dbt_rfcomm_pn_encode(&p,bytes,&built)==DBT_OK && bytes[1]==0xe0 && !built.command);
    CHECK(dbt_rfcomm_pn_decode(&built,&p)==DBT_OK && !p.command);
    bytes[1]=0;CHECK(dbt_rfcomm_pn_decode(&built,&p)==DBT_UNSUPPORTED);
    bytes[1]=0xe0;bytes[7]=8;CHECK(dbt_rfcomm_pn_decode(&built,&p)==DBT_INVALID);
    dbt_rfcomm_credit_t c;CHECK(dbt_rfcomm_credit_init(&c,1,1)==DBT_OK);
    CHECK(dbt_rfcomm_credit_prepare(&c,true,2)==DBT_OK && c.tx_available==1 && c.rx_outstanding==1);
    CHECK(dbt_rfcomm_credit_prepare(&c,false,0)==DBT_BUSY);
    CHECK(dbt_rfcomm_credit_receive(&c,true,2)==DBT_OK && c.tx_available==3 && !c.rx_outstanding);
    CHECK(dbt_rfcomm_credit_commit(&c)==DBT_OK && c.tx_available==2 && c.rx_outstanding==2);
    CHECK(dbt_rfcomm_credit_prepare(&c,true,1)==DBT_OK && dbt_rfcomm_credit_cancel(&c)==DBT_OK && c.tx_available==2 && c.rx_outstanding==2);
    CHECK(dbt_rfcomm_credit_receive(&c,true,0)==DBT_OK && dbt_rfcomm_credit_receive(&c,true,0)==DBT_OK);
    CHECK(dbt_rfcomm_credit_receive(&c,true,1)==DBT_INVALID && c.tx_available==2 && c.rx_outstanding==0);
    c.tx_available=0;CHECK(dbt_rfcomm_credit_prepare(&c,true,0)==DBT_BUSY);
    CHECK(dbt_rfcomm_credit_prepare(&c,false,1)==DBT_OK && dbt_rfcomm_credit_commit(&c)==DBT_OK && c.rx_outstanding==1);
    c.tx_available=65535;CHECK(dbt_rfcomm_credit_receive(&c,true,1)==DBT_NO_SPACE && c.rx_outstanding==1 && c.tx_available==65535);
    c.rx_outstanding=65535;CHECK(dbt_rfcomm_credit_prepare(&c,false,1)==DBT_NO_SPACE && !c.pending);
    return 0;
}
