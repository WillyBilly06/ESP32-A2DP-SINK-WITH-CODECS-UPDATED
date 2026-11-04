#include "dbt_credit.h"
#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif
#define CHECK(x) do{if(!(x))return __LINE__;}while(0)
typedef struct {
    dbt_credit_t channel;
    uint8_t storage[1002],result[1000];
    uint16_t size,allocated;
    unsigned allocations,releases,deliveries,errors;
    bool used,oom,reentrant;
} receiver_t;
static receiver_t r;
static void zero(void *p,unsigned n){uint8_t *v=p;for(unsigned i=0;i<n;++i)v[i]=0;}
static uint8_t *allocate(void *context,uint16_t bytes){
    receiver_t *p=context;++p->allocations;if(p->oom || p->used || bytes>1000)return 0;
    p->used=true;p->allocated=bytes;p->storage[0]=0x5a;p->storage[bytes+1u]=0xa5;return p->storage+1;
}
static void release(void *context,uint8_t *bytes){
    receiver_t *p=context;
    if(!p->used || bytes!=p->storage+1 || p->storage[0]!=0x5a || p->storage[p->allocated+1u]!=0xa5){++p->errors;return;}
    ++p->releases;p->used=false;
}
static void sdu(void *context,const uint8_t *bytes,uint16_t size){
    receiver_t *p=context;++p->deliveries;p->size=size;
    for(unsigned i=0;i<size && i<sizeof(p->result);++i)p->result[i]=bytes[i];
    if(p->reentrant){
        if(dbt_credit_add_peer_credits(&p->channel,1)!=DBT_INVALID)++p->errors;
        dbt_credit_close(&p->channel);if(!p->channel.active)++p->errors;
    }
}
static bool setup(uint16_t mtu,uint16_t mps,uint16_t credits){
    zero(&r,sizeof(r));dbt_credit_hooks_t hooks={&r,allocate,release,sdu};
    dbt_credit_parameters_t parameters={mtu,mps,credits};return dbt_credit_init(&r.channel,hooks,parameters,parameters)==DBT_OK;
}
EXPORT int test_credit_receive_lazy_sdu_and_bounds(void){
    CHECK(setup(1000,23,3) && !r.allocations);r.reentrant=true;
    const uint8_t whole[]={3,0,4,5,6};CHECK(dbt_credit_receive(&r.channel,whole,5)==DBT_OK);
    CHECK(r.deliveries==1 && r.size==3 && r.result[0]==4 && !r.allocations && r.channel.local.credits==2);
    uint8_t first[23]={30,0};for(unsigned i=2;i<23;++i)first[i]=(uint8_t)(i-2u);
    CHECK(dbt_credit_receive(&r.channel,first,23)==DBT_OK && r.used && r.allocated==30 && r.deliveries==1);
    uint8_t last[9];for(unsigned i=0;i<9;++i)last[i]=(uint8_t)(i+21u);
    CHECK(dbt_credit_receive(&r.channel,last,9)==DBT_OK && !r.used && r.releases==1 && r.deliveries==2 && r.size==30);
    for(unsigned i=0;i<30;++i)CHECK(r.result[i]==i);
    CHECK(dbt_credit_receive(&r.channel,whole,5)==DBT_INVALID && r.channel.failed && r.deliveries==2);
    dbt_credit_close(&r.channel);CHECK(!r.channel.active && !r.errors);
    CHECK(setup(23,23,1));uint8_t empty[]={0,0};CHECK(dbt_credit_receive(&r.channel,empty,2)==DBT_OK && r.deliveries==1 && !r.size && !r.allocations);
    dbt_credit_close(&r.channel);return 0;
}
EXPORT int test_credit_violations_oom_and_grant_admission(void){
    uint8_t frame[24]={30,0};
    CHECK(setup(23,23,3));CHECK(dbt_credit_receive(&r.channel,frame,2)==DBT_INVALID && r.channel.failed && !r.allocations);
    CHECK(setup(1000,23,3));CHECK(dbt_credit_receive(&r.channel,frame,24)==DBT_INVALID && r.channel.failed && !r.allocations);
    CHECK(setup(1000,23,3));CHECK(dbt_credit_receive(&r.channel,frame,23)==DBT_OK && r.used);
    CHECK(dbt_credit_receive(&r.channel,frame,10)==DBT_INVALID && r.channel.failed && !r.used && r.releases==1);
    CHECK(setup(1000,23,3));r.oom=true;CHECK(dbt_credit_receive(&r.channel,frame,2)==DBT_NO_SPACE && r.channel.failed && !r.used);
    CHECK(setup(1000,23,3));CHECK(dbt_credit_receive(&r.channel,frame,1)==DBT_INVALID && r.channel.failed);
    CHECK(setup(1000,23,3));CHECK(dbt_credit_receive(&r.channel,frame,23)==DBT_OK && r.used);
    dbt_credit_close(&r.channel);dbt_credit_close(&r.channel);CHECK(!r.used && r.releases==1 && !r.errors);
    CHECK(setup(1000,23,0));
    CHECK(dbt_credit_add_peer_credits(&r.channel,0)==DBT_OK && !r.channel.failed);
    CHECK(dbt_credit_grant_prepare(&r.channel,0)==DBT_INVALID);
    CHECK(dbt_credit_grant_prepare(&r.channel,5)==DBT_OK && !r.channel.local.credits);
    CHECK(dbt_credit_grant_prepare(&r.channel,1)==DBT_BUSY);
    dbt_credit_grant_cancel(&r.channel);CHECK(dbt_credit_grant_commit(&r.channel)==DBT_INVALID && !r.channel.local.credits);
    CHECK(dbt_credit_grant_prepare(&r.channel,5)==DBT_OK && dbt_credit_grant_commit(&r.channel)==DBT_OK && r.channel.local.credits==5);
    r.channel.local.credits=65535;CHECK(dbt_credit_grant_prepare(&r.channel,1)==DBT_NO_SPACE && !r.channel.failed);
    r.channel.peer.credits=65535;CHECK(dbt_credit_add_peer_credits(&r.channel,1)==DBT_INVALID && r.channel.failed);
    return 0;
}
EXPORT int test_credit_generated_roundtrip_and_tx_backpressure(void){
    uint8_t input[1000],output[96],again[96];for(unsigned i=0;i<1000;++i)input[i]=(uint8_t)(i*53u+19u);
    for(unsigned n=0;n<1001;++n){
        uint16_t mps=(uint16_t)(23u+n%74u);CHECK(setup(1000,mps,1));dbt_credit_t sender;
        dbt_credit_parameters_t p={1000,mps,1};dbt_credit_hooks_t hooks={&r,allocate,release,sdu};
        CHECK(dbt_credit_init(&sender,hooks,p,p)==DBT_OK && dbt_credit_tx_begin(&sender,input,(uint16_t)n)==DBT_OK);
        CHECK(dbt_credit_tx_begin(&sender,input,1)==DBT_BUSY);
        unsigned frames=0;
        while(sender.tx_active){
            uint16_t size=999,second=0;dbt_result_t result=dbt_credit_tx_peek(&sender,output,sizeof(output),&size);
            if(result==DBT_BUSY){
                CHECK(size==999 && !r.channel.local.credits && !sender.peer.credits);
                CHECK(dbt_credit_grant_prepare(&r.channel,1)==DBT_OK && dbt_credit_grant_commit(&r.channel)==DBT_OK);
                CHECK(dbt_credit_add_peer_credits(&sender,1)==DBT_OK);continue;
            }
            CHECK(result==DBT_OK && size<=mps && size);
            CHECK(dbt_credit_tx_peek(&sender,again,sizeof(again),&second)==DBT_OK && size==second && sender.peer.credits==1);
            for(unsigned i=0;i<size;++i)CHECK(output[i]==again[i]);
            CHECK(dbt_credit_receive(&r.channel,output,size)==DBT_OK);
            CHECK(dbt_credit_tx_commit(&sender)==DBT_OK && dbt_credit_tx_commit(&sender)==DBT_INVALID);CHECK(++frames<=45);
        }
        CHECK(r.deliveries==1 && r.size==n && !r.used && !r.errors && r.channel.frames_received==frames && sender.frames_sent==frames);
        CHECK(r.allocations==(n+2u>mps?1u:0u) && r.releases==r.allocations && !sender.transmit && sender.sdus_sent==1);
        for(unsigned i=0;i<n;++i)CHECK(input[i]==r.result[i]);
        uint16_t size=0;CHECK(dbt_credit_tx_peek(&sender,output,sizeof(output),&size)==DBT_EMPTY);
        dbt_credit_close(&sender);dbt_credit_close(&r.channel);
    }
    CHECK(setup(1000,23,1));CHECK(dbt_credit_tx_begin(&r.channel,input,50)==DBT_OK);
    uint16_t size=999;CHECK(dbt_credit_tx_peek(&r.channel,output,22,&size)==DBT_NO_SPACE && size==999 && !r.channel.pending_frame);
    CHECK(dbt_credit_tx_peek(&r.channel,output,23,&size)==DBT_OK && size==23 && output[0]==50 && output[1]==0 && output[2]==input[0]);
    dbt_credit_close(&r.channel);CHECK(!r.channel.transmit && !r.channel.tx_active);return 0;
}
