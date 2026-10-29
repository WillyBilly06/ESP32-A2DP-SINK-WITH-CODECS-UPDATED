#include "dbt_runtime.h"
#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif
#define CHECK(x) do{if(!(x))return __LINE__;}while(0)
typedef struct {
    dbt_runtime_t runtime;
    uint8_t wire[DBT_MAX_H4_BYTES],received[512],fixed[256],basic[512];
    _Alignas(8) uint8_t storage[12][1024];
    bool used[12],ready,stall,drop_next_data,dropped,fail_next,corrupt_mode,inject_no_fcs;
    uint16_t wire_size,cid,received_size,fixed_size,basic_size;
    unsigned sends,errors,allocated,released,opens,done,fixed_done;
    dbt_tx_outcome_t outcome;
    const uint8_t *borrowed;
} peer_t;
static peer_t peers[2];
static uint32_t now;
static void zero(void *p,unsigned n){uint8_t *b=p;for(unsigned i=0;i<n;++i)b[i]=0;}
static void copy(void *a,const void *b,unsigned n){uint8_t *x=a;const uint8_t *y=b;for(unsigned i=0;i<n;++i)x[i]=y[i];}
static void put16(uint8_t *p,uint16_t n){p[0]=(uint8_t)n;p[1]=(uint8_t)(n>>8);}
static bool ready(void *p){peer_t *v=p;return v->ready && !v->stall;}
static bool send(void *p,uint8_t *b,uint16_t n){
    peer_t *v=p;if(!ready(v) || n>sizeof(v->wire))return false;
    copy(v->wire,b,n);v->wire_size=n;v->borrowed=b;v->ready=false;++v->sends;return true;
}
static uint8_t *allocate(void *p,uint16_t n){
    peer_t *v=p;if(n>1024)return 0;
    if(v->fail_next){v->fail_next=false;return 0;}
    for(unsigned i=0;i<12;++i)if(!v->used[i]){v->used[i]=true;++v->allocated;return v->storage[i];}
    return 0;
}
static void release(void *p,uint8_t *b){
    peer_t *v=p;for(unsigned i=0;i<12;++i)if(v->storage[i]==b){
        if(!v->used[i])++v->errors;else{v->used[i]=false;++v->released;}return;
    }
    ++v->errors;
}
static void channel_event(void *p,dbt_channel_event_t event,const dbt_channel_t *c,dbt_channel_reason_t reason,uint16_t result){
    peer_t *v=p;(void)reason;(void)result;if(event==DBT_CH_EVENT_OPEN && c->psm==0x1b){v->cid=c->local_cid;++v->opens;}
}
static void data(void *p,const dbt_channel_t *c,const uint8_t *b,uint16_t n){
    peer_t *v=p;if(c->psm==0x19){if(n>sizeof(v->basic)){++v->errors;return;}copy(v->basic,b,n);v->basic_size=n;return;}
    if(n>sizeof(v->received)){++v->errors;return;}copy(v->received,b,n);v->received_size=n;
}
static void done(void *p,dbt_tx_outcome_t outcome){peer_t *v=p;++v->done;v->outcome=outcome;}
static void fixed_data(void *p,const dbt_link_t *link,uint16_t cid,const uint8_t *bytes,uint16_t n){
    peer_t *v=p;(void)link;(void)cid;if(n>sizeof(v->fixed)){++v->errors;return;}copy(v->fixed,bytes,n);v->fixed_size=n;
}
static void fixed_done(void *p,dbt_tx_outcome_t outcome){peer_t *v=p;++v->fixed_done;if(outcome!=DBT_TX_ADMITTED)++v->errors;}
static int event(peer_t *p,uint8_t code,const uint8_t *data,uint8_t n){
    uint8_t packet[260]={4,code,n};copy(packet+3,data,n);return dbt_runtime_receive(&p->runtime,packet,(uint16_t)(n+3));
}
static void check_borrow(peer_t *p){for(unsigned i=0;i<p->wire_size;++i)if(p->wire[i]!=p->borrowed[i]){++p->errors;break;}}
static void command_complete(peer_t *p){
    check_borrow(p);if(p->wire[0]!=1){++p->errors;p->ready=true;return;}
    uint16_t opcode=dbt_read_le16(p->wire+1);uint8_t data[68]={1,(uint8_t)opcode,(uint8_t)(opcode>>8),0};uint8_t n=4;
    switch(opcode){case 0x1001:n=12;data[4]=8;data[7]=8;break;case 0x1002:n=68;break;
    case 0x1003:n=12;data[8]=0x40;break;case 0x1009:n=10;data[4]=0x42;break;
    case 0x1005:n=11;data[4]=64;data[7]=2;break;case 0x2003:n=12;break;
    case 0x2002:n=7;data[4]=27;data[6]=2;break;default:break;}
    p->ready=true;if(event(p,0x0e,data,n)!=DBT_OK)++p->errors;
}
static int initialize(peer_t *p){
    zero(p,sizeof(*p));p->ready=true;dbt_runtime_hooks_t hooks={0};
    hooks.context=p;hooks.allocate=allocate;hooks.release=release;hooks.channels.context=p;
    hooks.channels.event=channel_event;hooks.channels.data=data;hooks.channels.fixed_data=fixed_data;
    dbt_transport_t transport={p,ready,send};
    CHECK(dbt_runtime_init(&p->runtime,transport,hooks,512,100)==DBT_OK);
    CHECK(dbt_runtime_start(&p->runtime,now)==DBT_OK);
    for(unsigned i=0;i<24 && !p->runtime.ready;++i){if(!p->ready)command_complete(p);dbt_runtime_poll(&p->runtime,++now,8,4);}
    CHECK(p->runtime.ready && p->ready && !p->errors);
    uint8_t link[11]={0};put16(link+1,1);link[9]=1;CHECK(event(p,3,link,11)==DBT_OK);
    dbt_runtime_poll(&p->runtime,++now,8,0);return 0;
}
static void transfer(peer_t *source,peer_t *target){
    if(source->ready)return;check_borrow(source);
    if(source->wire[0]!=2){++source->errors;source->ready=true;return;}
    uint8_t wire[DBT_MAX_H4_BYTES];copy(wire,source->wire,source->wire_size);uint16_t wire_size=source->wire_size;
    if(source->corrupt_mode && source->wire_size>=28 && wire[9]==4 && wire[21]==4 && wire[22]==9){
        wire[23]=0;source->corrupt_mode=false;
    }
    if(source->inject_no_fcs && wire_size>=28 && wire[9]==4 && wire[21]==4 && wire[22]==9){
        wire[wire_size++]=5;wire[wire_size++]=1;wire[wire_size++]=0;
        put16(wire+3,(uint16_t)(dbt_read_le16(wire+3)+3));
        put16(wire+5,(uint16_t)(dbt_read_le16(wire+5)+3));
        put16(wire+11,(uint16_t)(dbt_read_le16(wire+11)+3));source->inject_no_fcs=false;
    }
    if(((wire[2]>>4)&3u)==0)wire[2]=(uint8_t)((wire[2]&0xcfu)|0x20u);
    bool drop=source->drop_next_data && source->wire_size>=9 && ((source->wire[2]>>4)&3u)==0 &&
              dbt_read_le16(source->wire+7)>=0x40 && dbt_read_le16(source->wire+5)+9u==source->wire_size;
    if(drop){source->drop_next_data=false;source->dropped=true;}
    else if(dbt_runtime_receive(&target->runtime,wire,wire_size)!=DBT_OK)++target->errors;
    uint16_t handle=dbt_read_le16(source->wire+1)&0xfffu;
    uint8_t complete[]={1,(uint8_t)handle,(uint8_t)(handle>>8),1,0};if(event(source,0x13,complete,sizeof(complete))!=DBT_OK)++source->errors;
    source->ready=true;
}
static void step(void){
    dbt_runtime_poll(&peers[0].runtime,++now,8,4);dbt_runtime_poll(&peers[1].runtime,now,8,4);
    transfer(&peers[0],&peers[1]);transfer(&peers[1],&peers[0]);
}
static int setup_mode(bool secure,bool inject_no_fcs){
    now=0;int r=initialize(&peers[0]);if(r)return r;r=initialize(&peers[1]);if(r)return r;
    dbt_ertm_parameters_t params={256,64,2000,12000,4,3};
    if(secure){peers[0].runtime.acl.links[0].encrypted=true;peers[1].runtime.acl.links[0].encrypted=true;}
    dbt_security_t security=secure?DBT_SECURITY_ENCRYPTED:DBT_SECURITY_NONE;
    dbt_service_t service={0x1b,256,48,security};
    CHECK(dbt_channels_register(&peers[1].runtime.channels,service)==DBT_OK);
    CHECK(dbt_channels_require_ertm(&peers[1].runtime.channels,0x1b,params)==DBT_OK);
    uint16_t cid=0;peers[0].inject_no_fcs=inject_no_fcs;
    CHECK(dbt_channels_open_ertm(&peers[0].runtime.channels,1,0x1b,48,security,params,now,&cid)==DBT_OK);
    for(unsigned i=0;i<200 && (!peers[0].opens || !peers[1].opens);++i)step();
    CHECK(peers[0].opens==1 && peers[1].opens==1 && peers[0].cid==cid);
    return 0;
}
static int setup(void){return setup_mode(false,false);}
static int shutdown(void){
    CHECK(!peers[0].errors && !peers[1].errors);
    dbt_runtime_abort(&peers[0].runtime,DBT_FAULT_BAD_RESPONSE);dbt_runtime_abort(&peers[1].runtime,DBT_FAULT_BAD_RESPONSE);
    CHECK(peers[0].allocated==peers[0].released && peers[1].allocated==peers[1].released);
    return 0;
}
EXPORT int ertm_runtime_pair(void){
    int r=setup();if(r)return r;uint16_t cid=peers[0].cid;
    uint8_t payload[170];for(unsigned i=0;i<sizeof(payload);++i)payload[i]=(uint8_t)i;
    CHECK(dbt_runtime_send(&peers[0].runtime,1,cid,payload,sizeof(payload),done,&peers[0])==DBT_OK);
    for(unsigned i=0;i<200 && !peers[0].done;++i)step();
    CHECK(peers[0].done==1 && peers[0].outcome==DBT_TX_ADMITTED && peers[1].received_size==sizeof(payload));
    for(unsigned i=0;i<sizeof(payload);++i)CHECK(peers[1].received[i]==payload[i]);
    return shutdown();
}
EXPORT int ertm_runtime_ack_loan_and_recovery(void){
    int r=setup();if(r)return r;uint8_t payload[10]={1,2,3,4,5,6,7,8,9,10};
    peers[1].stall=true;
    CHECK(dbt_runtime_send(&peers[0].runtime,1,peers[0].cid,payload,sizeof(payload),done,&peers[0])==DBT_OK);
    for(unsigned i=0;i<30 && !peers[1].received_size;++i)step();
    CHECK(peers[1].received_size==sizeof(payload) && peers[0].done==0);
    peers[1].stall=false;for(unsigned i=0;i<40 && !peers[0].done;++i)step();
    CHECK(peers[0].done==1 && peers[0].outcome==DBT_TX_ADMITTED);
    peers[0].done=0;peers[1].received_size=0;peers[0].drop_next_data=true;
    CHECK(dbt_runtime_send(&peers[0].runtime,1,peers[0].cid,payload,sizeof(payload),done,&peers[0])==DBT_OK);
    for(unsigned i=0;i<30 && !peers[0].dropped;++i)step();
    CHECK(peers[0].dropped && !peers[0].done && !peers[1].received_size);
    now+=2001;for(unsigned i=0;i<100 && !peers[0].done;++i)step();
    CHECK(peers[0].done==1 && peers[0].outcome==DBT_TX_ADMITTED && peers[1].received_size==sizeof(payload));
    return shutdown();
}
EXPORT int ertm_runtime_link_loss_pinned(void){
    int r=setup();if(r)return r;uint8_t payload[170];zero(payload,sizeof(payload));
    CHECK(dbt_runtime_send(&peers[0].runtime,1,peers[0].cid,payload,sizeof(payload),done,&peers[0])==DBT_OK);
    dbt_runtime_poll(&peers[0].runtime,++now,8,1);
    CHECK(!peers[0].ready && peers[0].runtime.ertm_tx[0].active && peers[0].done==0);
    uint8_t disconnect[]={0,1,0,0x13};CHECK(event(&peers[0],5,disconnect,sizeof(disconnect))==DBT_OK);
    dbt_runtime_poll(&peers[0].runtime,++now,8,0);
    CHECK(peers[0].done==1 && peers[0].outcome==DBT_TX_LINK_LOST && !peers[0].runtime.ertm_tx[0].active);
    peers[0].ready=true;
    return shutdown();
}
EXPORT int ertm_runtime_bidirectional_oom(void){
    int r=setup();if(r)return r;uint8_t a[130],b[97];
    for(unsigned i=0;i<sizeof(a);++i)a[i]=(uint8_t)i;
    for(unsigned i=0;i<sizeof(b);++i)b[i]=(uint8_t)(255u-i);
    peers[0].fail_next=true;
    CHECK(dbt_runtime_send(&peers[0].runtime,1,peers[0].cid,a,sizeof(a),done,&peers[0])==DBT_NO_SPACE);
    CHECK(dbt_runtime_send(&peers[0].runtime,1,peers[0].cid,a,sizeof(a),done,&peers[0])==DBT_OK);
    CHECK(dbt_runtime_send(&peers[1].runtime,1,peers[1].cid,b,sizeof(b),done,&peers[1])==DBT_OK);
    for(unsigned i=0;i<200 && (!peers[0].done || !peers[1].done);++i)step();
    CHECK(peers[0].done==1 && peers[1].done==1 && peers[0].received_size==sizeof(b) && peers[1].received_size==sizeof(a));
    for(unsigned i=0;i<sizeof(a);++i)CHECK(peers[1].received[i]==a[i]);
    for(unsigned i=0;i<sizeof(b);++i)CHECK(peers[0].received[i]==b[i]);
    peers[0].done=0;
    CHECK(dbt_runtime_send(&peers[0].runtime,1,peers[0].cid,a,sizeof(a),done,&peers[0])==DBT_OK);
    peers[0].fail_next=true;
    for(unsigned i=0;i<50 && !peers[0].done;++i)step();
    CHECK(peers[0].done==1 && peers[0].outcome==DBT_TX_ABORTED);
    return shutdown();
}
EXPORT int ertm_runtime_required_rejects_basic(void){
    now=0;int r=initialize(&peers[0]);if(r)return r;r=initialize(&peers[1]);if(r)return r;
    dbt_ertm_parameters_t params={256,64,2000,12000,4,3};dbt_service_t service={0x1b,256,48,DBT_SECURITY_NONE};
    CHECK(dbt_channels_register(&peers[1].runtime.channels,service)==DBT_OK);
    CHECK(dbt_channels_require_ertm(&peers[1].runtime.channels,0x1b,params)==DBT_OK);
    uint16_t cid;peers[0].corrupt_mode=true;
    CHECK(dbt_channels_open_ertm(&peers[0].runtime.channels,1,0x1b,48,DBT_SECURITY_NONE,params,now,&cid)==DBT_OK);
    for(unsigned i=0;i<200 && dbt_channels_find(&peers[0].runtime.channels,1,cid);++i)step();
    CHECK(!peers[0].corrupt_mode && !peers[0].opens && !peers[1].opens);
    CHECK(!dbt_channels_find(&peers[0].runtime.channels,1,cid));
    return shutdown();
}
EXPORT int ertm_runtime_security_revocation(void){
    int r=setup_mode(true,false);if(r)return r;uint8_t payload[30]={0};
    CHECK(dbt_runtime_send(&peers[0].runtime,1,peers[0].cid,payload,sizeof(payload),done,&peers[0])==DBT_OK);
    peers[0].runtime.acl.links[0].encrypted=false;unsigned sends=peers[0].sends;
    dbt_runtime_poll(&peers[0].runtime,++now,8,0);
    CHECK(peers[0].done==1 && peers[0].outcome==DBT_TX_ABORTED && peers[0].sends==sends);
    return shutdown();
}
EXPORT int ertm_runtime_fcs_default_if_one_omits(void){
    int r=setup_mode(false,true);if(r)return r;uint8_t payload[]={10,20,30,40,50};
    CHECK(!peers[0].inject_no_fcs);
    CHECK(dbt_runtime_send(&peers[0].runtime,1,peers[0].cid,payload,sizeof(payload),done,&peers[0])==DBT_OK);
    for(unsigned i=0;i<100 && !peers[0].done;++i)step();
    CHECK(peers[0].done==1 && peers[1].received_size==sizeof(payload));
    for(unsigned i=0;i<sizeof(payload);++i)CHECK(peers[1].received[i]==payload[i]);
    return shutdown();
}
EXPORT int ertm_runtime_missing_peer_feature(void){
    now=0;int r=initialize(&peers[0]);if(r)return r;r=initialize(&peers[1]);if(r)return r;
    dbt_ertm_parameters_t params={256,64,2000,12000,4,3};dbt_service_t service={0x1b,256,48,DBT_SECURITY_NONE};
    CHECK(dbt_channels_register(&peers[1].runtime.channels,service)==DBT_OK);
    uint16_t cid;CHECK(dbt_channels_open_ertm(&peers[0].runtime.channels,1,0x1b,48,DBT_SECURITY_NONE,params,now,&cid)==DBT_OK);
    for(unsigned i=0;i<200 && dbt_channels_find(&peers[0].runtime.channels,1,cid);++i)step();
    CHECK(!peers[0].opens && !peers[1].opens && !dbt_channels_find(&peers[0].runtime.channels,1,cid));
    return shutdown();
}
EXPORT int ertm_runtime_le_coexistence(void){
    int r=setup();if(r)return r;
    for(unsigned p=0;p<2;++p){
        uint8_t le[19]={1,0,2,0};CHECK(event(&peers[p],0x3e,le,sizeof(le))==DBT_OK);
        dbt_runtime_poll(&peers[p].runtime,++now,8,0);
        CHECK(dbt_channels_register_fixed(&peers[p].runtime.channels,DBT_LINK_LE,4)==DBT_OK);
    }
    uint8_t a[170],b[150];for(unsigned i=0;i<sizeof(a);++i)a[i]=(uint8_t)i;
    for(unsigned i=0;i<sizeof(b);++i)b[i]=(uint8_t)(255u-i);
    CHECK(dbt_runtime_send(&peers[0].runtime,1,peers[0].cid,a,sizeof(a),done,&peers[0])==DBT_OK);
    CHECK(dbt_runtime_send_fixed(&peers[0].runtime,2,4,b,sizeof(b),fixed_done,&peers[0])==DBT_OK);
    for(unsigned i=0;i<300 && (!peers[0].done || !peers[0].fixed_done);++i)step();
    CHECK(peers[0].done==1 && peers[0].fixed_done==1 && peers[1].received_size==sizeof(a) && peers[1].fixed_size==sizeof(b));
    for(unsigned i=0;i<sizeof(a);++i)CHECK(peers[1].received[i]==a[i]);
    for(unsigned i=0;i<sizeof(b);++i)CHECK(peers[1].fixed[i]==b[i]);
    return shutdown();
}
EXPORT int ertm_runtime_basic_same_link(void){
    int r=setup();if(r)return r;
    dbt_service_t service={0x19,400,48,DBT_SECURITY_NONE};
    CHECK(dbt_channels_register(&peers[1].runtime.channels,service)==DBT_OK);
    uint16_t basic_cid;
    CHECK(dbt_channels_open(&peers[0].runtime.channels,1,0x19,400,48,DBT_SECURITY_NONE,now,&basic_cid)==DBT_OK);
    for(unsigned i=0;i<100;++i){
        const dbt_channel_t *c=dbt_channels_find(&peers[0].runtime.channels,1,basic_cid);
        if(c && c->state==DBT_CH_OPEN)break;step();
    }
    const dbt_channel_t *basic=dbt_channels_find(&peers[0].runtime.channels,1,basic_cid);
    CHECK(basic && basic->state==DBT_CH_OPEN);
    uint8_t a[170],b[300];for(unsigned i=0;i<sizeof(a);++i)a[i]=(uint8_t)i;
    for(unsigned i=0;i<sizeof(b);++i)b[i]=(uint8_t)(255u-i);
    CHECK(dbt_runtime_send(&peers[1].runtime,1,peers[1].cid,a,sizeof(a),done,&peers[1])==DBT_OK);
    CHECK(dbt_runtime_send(&peers[0].runtime,1,basic_cid,b,sizeof(b),fixed_done,&peers[0])==DBT_OK);
    for(unsigned i=0;i<300 && (!peers[1].done || !peers[0].fixed_done);++i)step();
    CHECK(peers[1].done==1 && peers[0].fixed_done==1 && peers[0].received_size==sizeof(a) && peers[1].basic_size==sizeof(b));
    for(unsigned i=0;i<sizeof(a);++i)CHECK(peers[0].received[i]==a[i]);
    for(unsigned i=0;i<sizeof(b);++i)CHECK(peers[1].basic[i]==b[i]);
    return shutdown();
}
EXPORT int ertm_runtime_wait_transport_gate(void){
    int r=setup();if(r)return r;uint8_t payload[]={1,2,3,4};
    peers[0].stall=true;
    CHECK(dbt_runtime_send(&peers[0].runtime,1,peers[0].cid,payload,sizeof(payload),done,&peers[0])==DBT_OK);
    CHECK(dbt_runtime_wait_ms(&peers[0].runtime,now)>0);
    for(unsigned i=0;i<8;++i)step();
    CHECK(!peers[0].done && !peers[1].received_size);
    peers[0].stall=false;
    for(unsigned i=0;i<80 && !peers[0].done;++i)step();
    CHECK(peers[0].done==1 && peers[1].received_size==sizeof(payload));
    return shutdown();
}
