/* Real feature-exchange policy, with unsupported optional CTKD negotiated out. */
#include "dbt_smp_features.h"
#include "dbt_smp_association.h"
#include "dbt_smp_session.h"
#include <stdio.h>
#include <stdlib.h>
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
    dbt_smp_sc_t engine;uint8_t local[6]={1},peer[6]={2};
    CHECK(dbt_smp_features_open_sc(&f,&engine,0,local,0,peer,2)==DBT_OK);
    CHECK(!engine.numeric && !engine.passkey_mode && !engine.oob_mode && !engine.authenticated && engine.key_size==16);
    CHECK(!memcmp(engine.preq,request,7));
    CHECK(dbt_smp_sc_check_bond_policy(&engine,true,true,16)==DBT_UNSUPPORTED);
    dbt_smp_sc_close(&engine);
    return 0;
}
static unsigned allocations,releases;
static uint8_t *allocate(void *c,uint16_t n){(void)c;uint8_t *p=malloc(n);if(p)allocations++;return p;}
static void release(void *c,uint8_t *p){(void)c;if(p){releases++;free(p);}}
static void fixed(void *c,const dbt_link_t *l,uint16_t cid,const uint8_t *p,uint16_t n){(void)c;(void)l;(void)cid;(void)p;(void)n;}
static int session_policy(bool allow){
    dbt_runtime_t r={0};dbt_smp_session_t s={0};dbt_smp_guard_t g;uint8_t local[6]={1},peer[6]={2};
    r.ready=true;r.host.state=DBT_HOST_IDENTIFIED;r.host.timeout_ms=2000;r.maximum_mtu=128;
    r.hooks.allocate=allocate;r.hooks.release=release;r.channels.acl=&r.acl;r.channels.hooks.fixed_data=fixed;
    r.acl.mtu[DBT_LINK_LE]=251;
    dbt_link_t *l=&r.acl.links[0];l->used=true;l->kind=DBT_LINK_LE;l->handle=1;l->generation=1;l->role=1;memcpy(l->peer,peer,6);
    dbt_smp_session_config_t c={0};c.features=policy();c.features.bonding=false;c.features.initiator_keys=0;c.allow_just_works=allow;
    CHECK(dbt_smp_guard_init(&g,10,1000,10000,0)==DBT_OK);
    CHECK(dbt_smp_session_open(&s,&r,1,1,0,local,0,peer,&g,&c,0)==DBT_OK);
    uint8_t req[7]={1,4,0,12,16,0,0};CHECK(dbt_smp_session_receive(&s,l,6,req,7)==DBT_OK);
    for(unsigned t=1;t<8;t++){
        dbt_smp_session_poll(&s,t);
        for(unsigned i=0;i<sizeof(r.tx)/sizeof(r.tx[0]);i++)if(r.tx[i].active){
            dbt_runtime_tx_t *tx=&r.tx[i];tx->active=false;tx->done(tx->context,DBT_TX_ADMITTED);
        }
    }
    CHECK(s.phase==(allow?DBT_SMP_SESSION_SC:DBT_SMP_SESSION_FAILED));
    CHECK(dbt_smp_session_close(&s)==DBT_OK && allocations==releases);
    return 0;
}
static int legacy_policy(bool allow){
    dbt_smp_t s;uint8_t local[6]={1},peer[6]={2},req[7]={1,4,0,4,16,0,0};
    CHECK(dbt_smp_open(&s,1,1,1,0,local,0,peer,16,allow)==DBT_OK);
    CHECK(dbt_smp_receive(&s,req,7,0)==DBT_OK);
    const uint8_t *p;uint8_t n;CHECK(dbt_smp_tx(&s,&p,&n)==DBT_OK);
    CHECK(!s.authenticated);
    CHECK(allow?(n==7 && p[0]==2 && !(p[3]&4)):(n==2 && p[0]==5 && p[1]==3));
    dbt_smp_close(&s);return 0;
}
static int run(void){
    static const dbt_smp_association_t legacy[5][5]={
      {1,1,2,1,2},{1,1,2,1,2},{2,2,2,1,2},{1,1,1,1,1},{2,2,2,1,2}};
    static const dbt_smp_association_t secure[5][5]={
      {1,1,2,1,2},{1,3,2,1,3},{2,2,2,1,2},{1,1,1,1,1},{2,3,2,1,3}};
    for(unsigned sc=0;sc<2;sc++)for(unsigned i=0;i<5;i++)for(unsigned r=0;r<5;r++){
      CHECK(dbt_smp_select_association(sc,true,false,(uint8_t)i,(uint8_t)r)==(sc?secure[r][i]:legacy[r][i]));
      CHECK(dbt_smp_select_association(sc,false,false,(uint8_t)i,(uint8_t)r)==DBT_SMP_ASSOC_JUST_WORKS);
      CHECK(dbt_smp_select_association(sc,true,true,(uint8_t)i,(uint8_t)r)==DBT_SMP_ASSOC_OOB);
    }
    CHECK(dbt_smp_select_association(true,true,false,5,3)==DBT_SMP_ASSOC_INVALID);
    int guard_line;
    if((guard_line=session_policy(true)))return guard_line;
    if((guard_line=session_policy(false)))return guard_line;
    if((guard_line=legacy_policy(true)))return guard_line;
    if((guard_line=legacy_policy(false)))return guard_line;
    dbt_smp_feature_policy_t p=policy();uint8_t req[7]={1,3,0,9,16,15,15};int line;
    if((line=responder(req,p,0)))return line;
    req[1]=4;req[3]=13;if((line=responder(req,p,0)))return line; /* Captured phone request. */
    req[1]=3;
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
