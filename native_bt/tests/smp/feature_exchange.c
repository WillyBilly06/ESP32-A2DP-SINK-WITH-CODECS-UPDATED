/* Real feature-exchange policy, with unsupported optional CTKD negotiated out. */
#include "dbt_smp_features.h"
#include <stdio.h>
#include <string.h>
#define CHECK(x) do {if(!(x))return __LINE__;}while(0)
static dbt_smp_feature_policy_t policy(void){
    dbt_smp_feature_policy_t p={0};p.io=3;p.min_key_size=p.max_key_size=16;
    p.initiator_keys=2;p.support_sc=p.require_sc=p.bonding=true;return p;
}
static int responder(const uint8_t request[7],dbt_smp_feature_policy_t p,uint8_t error){
    dbt_smp_features_t f;dbt_smp_guard_t g;
    CHECK(dbt_smp_guard_init(&g,10,1000,10000,0)==DBT_OK);
    CHECK(dbt_smp_features_open(&f,1,1,false,&p)==DBT_OK);
    CHECK(dbt_smp_features_set_guard(&f,&g,0)==DBT_OK);
    CHECK(dbt_smp_features_receive(&f,request,7,0)==DBT_OK);
    const uint8_t *wire;uint8_t size;
    CHECK(dbt_smp_features_tx(&f,&wire,&size)==DBT_OK);
    if(error){CHECK(f.state==DBT_FEATURE_FAILED && size==2 && wire[0]==5 && wire[1]==error);return 0;}
    CHECK(f.state==DBT_FEATURE_SEND_RESPONSE && size==7 && wire[0]==2);
    CHECK(wire[5]==2 && !wire[6] && !((wire[5]|wire[6])&8));
    CHECK(!memcmp(f.preq,request,7)); /* Exact transcript is retained for SC. */
    dbt_smp_features_tx_done(&f,true,1);
    uint8_t req[7],rsp[7];bool sc=false;
    CHECK(dbt_smp_features_take(&f,req,rsp,&sc)==DBT_OK && sc);
    CHECK(!memcmp(req,request,7) && rsp[5]==2 && !rsp[6]);
    return 0;
}
static int run(void){
    dbt_smp_feature_policy_t p=policy();uint8_t req[7]={1,3,0,9,16,15,15};int line;
    if((line=responder(req,p,0)))return line;
    req[3]=0x29;if((line=responder(req,p,0)))return line; /* CT2 can be offered without negotiation. */
    req[3]=9;req[5]=0x10;if((line=responder(req,p,10)))return line;
    req[5]=15;req[4]=15;if((line=responder(req,p,6)))return line;
    req[4]=16;req[3]=1;if((line=responder(req,p,5)))return line;
    req[3]=9;req[2]=1;if((line=responder(req,p,2)))return line;
    req[2]=0;p.require_mitm=true;if((line=responder(req,p,3)))return line;
    p=policy();p.initiator_keys|=8;dbt_smp_features_t f;
    CHECK(dbt_smp_features_open(&f,1,1,false,&p)==DBT_INVALID);
    p=policy();dbt_smp_guard_t g;
    CHECK(dbt_smp_guard_init(&g,10,1000,10000,0)==DBT_OK);
    CHECK(dbt_smp_features_open(&f,1,1,true,&p)==DBT_OK && dbt_smp_features_set_guard(&f,&g,0)==DBT_OK);
    CHECK(dbt_smp_features_start(&f,0)==DBT_OK);dbt_smp_features_tx_done(&f,true,1);
    uint8_t rsp[7]={2,3,0,9,16,10,0};
    CHECK(dbt_smp_features_receive(&f,rsp,7,2)==DBT_OK && f.state==DBT_FEATURE_FAILED);
    return 0;
}
int main(void){int line=run();if(line){printf("FAIL feature negotiation line %d\n",line);return 1;}
    puts("PASS peer LinkKey/CT2 offer handling, transcript preservation and unchanged SC/key-size/MITM/local-CTKD gates");return 0;}
