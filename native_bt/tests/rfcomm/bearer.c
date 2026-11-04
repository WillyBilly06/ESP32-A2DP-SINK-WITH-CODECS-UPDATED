#include "dbt_rfcomm_bearer.h"
#include "dbt_rfcomm_spp.h"
#include "probe_rfcomm.h"
#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif
#define CHECK(x) do{if(!(x))return __LINE__;}while(0)
typedef struct {
    dbt_runtime_t runtime;dbt_rfcomm_bearer_t bearer;dbt_rfcomm_spp_t spp;
    _Alignas(8) uint8_t memory[12][1024];bool used[12],ready,bound,stall,legacy_pn,use_spp,use_probe;
    uint8_t wire[DBT_MAX_H4_BYTES],received[256];uint16_t wire_size,cid,received_size;unsigned errors,allocs,frees,fail_at,dlc_opens,dlc_closes,done;dbt_tx_outcome_t outcome;
} peer_t;
static peer_t p[2];static uint32_t now;
static void zero(void *v,unsigned n){uint8_t *d=v;for(unsigned i=0;i<n;++i)d[i]=0;}
static void copy(void *a,const void *b,unsigned n){uint8_t *x=a;const uint8_t *y=b;for(unsigned i=0;i<n;++i)x[i]=y[i];}
static void put16(uint8_t *x,uint16_t n){x[0]=(uint8_t)n;x[1]=(uint8_t)(n>>8);}
static bool ready(void *v){peer_t *s=v;return s->ready && !s->stall;}
static bool send(void *v,uint8_t *data,uint16_t n){peer_t *s=v;if(!ready(v) || n>sizeof(s->wire))return false;copy(s->wire,data,n);s->wire_size=n;s->ready=false;return true;}
static uint8_t *allocate(void *v,uint16_t n){peer_t *s=v;if(n>1024 || (s->fail_at && s->allocs+1==s->fail_at))return 0;for(unsigned i=0;i<12;++i)if(!s->used[i]){s->used[i]=true;++s->allocs;return s->memory[i];}return 0;}
static void release(void *v,uint8_t *bytes){peer_t *s=v;for(unsigned i=0;i<12;++i)if(s->memory[i]==bytes){if(!s->used[i])++s->errors;else{s->used[i]=false;++s->frees;}return;}++s->errors;}
static void channel_event(void *v,dbt_channel_event_t event,const dbt_channel_t *c,dbt_channel_reason_t reason,uint16_t result){
    peer_t *s=v;(void)reason;(void)result;if(event==DBT_CH_EVENT_OPEN && c->psm==3)s->cid=c->local_cid;
    if(s->use_spp)dbt_rfcomm_spp_event(&s->spp,event,c);
    if(s->use_probe)probe_rfcomm_event(event,c);
}
static void data(void *v,const dbt_channel_t *c,const uint8_t *bytes,uint16_t n){
    peer_t *s=v;if(s->use_probe){if(probe_rfcomm_data(c,bytes,n)!=DBT_OK)++s->errors;}
    else if(s->use_spp){if(dbt_rfcomm_spp_receive(&s->spp,c,bytes,n)!=DBT_OK)++s->errors;}
    else if(!s->bound || dbt_rfcomm_bearer_enqueue(&s->bearer,c,bytes,n)!=DBT_OK)++s->errors;
}
static bool channel_accept(void *v,const dbt_link_t *link,uint16_t psm){peer_t *s=v;if(s->use_probe)return probe_rfcomm_accept(link,psm);return !s->use_spp || psm!=3 || dbt_rfcomm_spp_accept(&s->spp,link,psm);}
static void dlc_event(void *v,dbt_rfcomm_dlc_event_t event,uint8_t dlci){peer_t *s=v;if(dlci!=6)++s->errors;if(event==DBT_RFCOMM_DLC_EVENT_OPEN)++s->dlc_opens;else if(event==DBT_RFCOMM_DLC_EVENT_CLOSED)++s->dlc_closes;else ++s->errors;}
static void dlc_data(void *v,uint8_t dlci,const uint8_t *bytes,uint16_t n){peer_t *s=v;if(dlci!=6 || n>sizeof(s->received)){++s->errors;return;}copy(s->received,bytes,n);s->received_size=n;}
static void app_done(void *v,dbt_tx_outcome_t outcome){peer_t *s=v;++s->done;s->outcome=outcome;}
static int event(peer_t *s,uint8_t code,const uint8_t *data,uint8_t n){uint8_t packet[260]={4,code,n};copy(packet+3,data,n);return dbt_runtime_receive(&s->runtime,packet,(uint16_t)(n+3));}
static void command_complete(peer_t *s){
    if(s->wire[0]!=1){++s->errors;s->ready=true;return;}uint16_t op=dbt_read_le16(s->wire+1);uint8_t d[68]={1,(uint8_t)op,(uint8_t)(op>>8),0};uint8_t n=4;
    switch(op){case 0x1001:n=12;d[4]=8;d[7]=8;break;case 0x1002:n=68;break;case 0x1003:n=12;d[8]=0x40;break;
    case 0x1009:n=10;d[4]=0x42;break;case 0x1005:n=11;d[4]=64;d[7]=2;break;case 0x2003:n=12;break;case 0x2002:n=7;d[4]=27;d[6]=2;break;default:break;}
    s->ready=true;if(event(s,0x0e,d,n)!=DBT_OK)++s->errors;
}
static int initialize(peer_t *s){
    zero(s,sizeof(*s));s->ready=true;dbt_runtime_hooks_t hooks={0};hooks.context=s;hooks.allocate=allocate;hooks.release=release;
    hooks.channels.context=s;hooks.channels.event=channel_event;hooks.channels.data=data;hooks.channels.accept=channel_accept;dbt_transport_t t={s,ready,send};
    CHECK(dbt_runtime_init(&s->runtime,t,hooks,512,100)==DBT_OK);CHECK(dbt_runtime_start(&s->runtime,now)==DBT_OK);
    for(unsigned i=0;i<24 && !s->runtime.ready;++i){if(!s->ready)command_complete(s);dbt_runtime_poll(&s->runtime,++now,8,4);}
    CHECK(s->runtime.ready && !s->errors);uint8_t link[11]={0};put16(link+1,1);link[9]=1;CHECK(event(s,3,link,sizeof(link))==DBT_OK);
    dbt_runtime_poll(&s->runtime,++now,8,0);return 0;
}
static void transfer(peer_t *src,peer_t *dst){
    if(src->ready)return;if(src->wire[0]!=2){++src->errors;src->ready=true;return;}
    uint8_t wire[DBT_MAX_H4_BYTES];copy(wire,src->wire,src->wire_size);
    if(src->legacy_pn && src->wire_size==23 && wire[10]==0xef && wire[12]==0x83 && wire[15]==0xf0){
        wire[15]=0;wire[21]=0;src->legacy_pn=false; /* simulate an older PN request */
    }
    if(((wire[2]>>4)&3u)==0)wire[2]=(uint8_t)((wire[2]&0xcfu)|0x20u);
    if(dbt_runtime_receive(&dst->runtime,wire,src->wire_size)!=DBT_OK)++dst->errors;
    uint8_t complete[]={1,1,0,1,0};if(event(src,0x13,complete,sizeof(complete))!=DBT_OK)++src->errors;src->ready=true;
}
static void step(void){
    for(unsigned i=0;i<2;++i){if(p[i].bound)dbt_rfcomm_bearer_poll(&p[i].bearer,now);if(p[i].use_spp)dbt_rfcomm_spp_poll(&p[i].spp,now);if(p[i].use_probe)probe_rfcomm_poll(now);}
    dbt_runtime_poll(&p[0].runtime,++now,8,4);dbt_runtime_poll(&p[1].runtime,now,8,4);
    transfer(&p[0],&p[1]);transfer(&p[1],&p[0]);
}
EXPORT int rfcomm_bearer_runtime_pair(void){
    now=0;int r=initialize(&p[0]);if(r)return r;r=initialize(&p[1]);if(r)return r;
    dbt_service_t service={3,256,48,DBT_SECURITY_NONE};CHECK(dbt_channels_register(&p[1].runtime.channels,service)==DBT_OK);
    uint16_t cid;CHECK(dbt_channels_open(&p[0].runtime.channels,1,3,256,48,DBT_SECURITY_NONE,now,&cid)==DBT_OK);
    for(unsigned i=0;i<100 && (!p[0].cid || !p[1].cid);++i)step();CHECK(p[0].cid==cid && p[1].cid);
    CHECK(dbt_rfcomm_bearer_bind(&p[1].bearer,&p[1].runtime,1,p[1].cid,false,10000)==DBT_OK);p[1].bound=true;
    CHECK(dbt_rfcomm_bearer_bind(&p[0].bearer,&p[0].runtime,1,p[0].cid,true,10000)==DBT_OK);p[0].bound=true;
    p[0].stall=true;for(unsigned i=0;i<8;++i)step();CHECK(p[0].bearer.mux.state==DBT_RFCOMM_MUX_START_QUEUED);
    p[0].stall=false;for(unsigned i=0;i<100 && (p[0].bearer.mux.state!=DBT_RFCOMM_MUX_OPEN || p[1].bearer.mux.state!=DBT_RFCOMM_MUX_OPEN);++i)step();
    CHECK(p[0].bearer.mux.state==DBT_RFCOMM_MUX_OPEN && p[1].bearer.mux.state==DBT_RFCOMM_MUX_OPEN);
    CHECK(dbt_rfcomm_mux_close(&p[0].bearer.mux)==DBT_OK);
    for(unsigned i=0;i<100 && (p[0].bearer.mux.state!=DBT_RFCOMM_MUX_DOWN || p[1].bearer.mux.state!=DBT_RFCOMM_MUX_DOWN);++i)step();
    CHECK(p[0].bearer.mux.state==DBT_RFCOMM_MUX_DOWN && p[1].bearer.mux.state==DBT_RFCOMM_MUX_DOWN && !p[0].errors && !p[1].errors);
    dbt_runtime_abort(&p[0].runtime,DBT_FAULT_BAD_RESPONSE);dbt_runtime_abort(&p[1].runtime,DBT_FAULT_BAD_RESPONSE);
    dbt_rfcomm_bearer_poll(&p[0].bearer,now);dbt_rfcomm_bearer_poll(&p[1].bearer,now);
    CHECK(p[0].allocs==p[0].frees && p[1].allocs==p[1].frees);return 0;
}
EXPORT int rfcomm_bearer_overflow_and_link_loss(void){
    now=0;int r=initialize(&p[0]);if(r)return r;r=initialize(&p[1]);if(r)return r;
    dbt_service_t service={3,256,48,DBT_SECURITY_NONE};CHECK(dbt_channels_register(&p[1].runtime.channels,service)==DBT_OK);
    uint16_t cid;CHECK(dbt_channels_open(&p[0].runtime.channels,1,3,256,48,DBT_SECURITY_NONE,now,&cid)==DBT_OK);
    for(unsigned i=0;i<100 && (!p[0].cid || !p[1].cid);++i)step();CHECK(p[0].cid==cid && p[1].cid);
    CHECK(dbt_rfcomm_bearer_bind(&p[1].bearer,&p[1].runtime,1,p[1].cid,false,10000)==DBT_OK);p[1].bound=true;
    CHECK(dbt_rfcomm_bearer_bind(&p[0].bearer,&p[0].runtime,1,p[0].cid,true,10000)==DBT_OK);p[0].bound=true;
    p[0].stall=true;dbt_rfcomm_bearer_poll(&p[0].bearer,now);
    CHECK(p[0].bearer.tx_active && p[0].bearer.mux.tx_pinned);
    const dbt_channel_t *c=dbt_channels_find(&p[1].runtime.channels,1,p[1].cid);CHECK(c);
    const uint8_t sabm[]={3,0x3f,1,0x1c};
    for(unsigned i=0;i<DBT_RFCOMM_BEARER_RX_SLOTS;++i)CHECK(dbt_rfcomm_bearer_enqueue(&p[1].bearer,c,sabm,4)==DBT_OK);
    CHECK(dbt_rfcomm_bearer_enqueue(&p[1].bearer,c,sabm,4)==DBT_NO_SPACE && p[1].bearer.faulted);
    uint8_t removed[]={0,1,0,0x13};CHECK(event(&p[0],5,removed,sizeof(removed))==DBT_OK);
    dbt_runtime_poll(&p[0].runtime,++now,8,0);dbt_rfcomm_bearer_poll(&p[0].bearer,now);
    CHECK(!p[0].bearer.tx_active && p[0].bearer.closed && !p[0].bearer.mux.generation);
    dbt_rfcomm_bearer_poll(&p[1].bearer,now);
    c=dbt_channels_find(&p[1].runtime.channels,1,p[1].cid);CHECK(c && c->state==DBT_CH_DISCONNECTING);
    dbt_runtime_abort(&p[0].runtime,DBT_FAULT_BAD_RESPONSE);dbt_runtime_abort(&p[1].runtime,DBT_FAULT_BAD_RESPONSE);
    dbt_rfcomm_bearer_poll(&p[0].bearer,now);dbt_rfcomm_bearer_poll(&p[1].bearer,now);
    CHECK(p[0].allocs==p[0].frees && p[1].allocs==p[1].frees);return 0;
}
EXPORT int rfcomm_bearer_encrypted_dlc_pair(void){
    now=0;int r=initialize(&p[0]);if(r)return r;r=initialize(&p[1]);if(r)return r;
    p[0].runtime.acl.links[0].encrypted=true;p[1].runtime.acl.links[0].encrypted=true;
    dbt_service_t service={3,256,48,DBT_SECURITY_ENCRYPTED};CHECK(dbt_channels_register(&p[1].runtime.channels,service)==DBT_OK);
    uint16_t cid;CHECK(dbt_channels_open(&p[0].runtime.channels,1,3,256,48,DBT_SECURITY_ENCRYPTED,now,&cid)==DBT_OK);
    for(unsigned i=0;i<100 && (!p[0].cid || !p[1].cid);++i)step();CHECK(p[0].cid==cid && p[1].cid);
    CHECK(dbt_rfcomm_bearer_bind(&p[1].bearer,&p[1].runtime,1,p[1].cid,false,10000)==DBT_OK);p[1].bound=true;
    CHECK(dbt_rfcomm_bearer_bind(&p[0].bearer,&p[0].runtime,1,p[0].cid,true,10000)==DBT_OK);p[0].bound=true;
    for(unsigned i=0;i<100 && (p[0].bearer.mux.state!=DBT_RFCOMM_MUX_OPEN || p[1].bearer.mux.state!=DBT_RFCOMM_MUX_OPEN);++i)step();
    CHECK(p[0].bearer.mux.state==DBT_RFCOMM_MUX_OPEN && p[1].bearer.mux.state==DBT_RFCOMM_MUX_OPEN);
    dbt_rfcomm_dlc_hooks_t h0={&p[0],dlc_event,dlc_data},h1={&p[1],dlc_event,dlc_data};
    CHECK(dbt_rfcomm_bearer_listen(&p[1].bearer,3,180,1,h1)==DBT_OK);
    CHECK(dbt_rfcomm_bearer_open(&p[0].bearer,3,200,1,h0)==DBT_OK);
    for(unsigned i=0;i<200 && (!p[0].dlc_opens || !p[1].dlc_opens) && !p[0].errors && !p[1].errors;++i)step();
    CHECK(p[0].dlc_opens==1 && p[1].dlc_opens==1 && !p[0].errors && !p[1].errors);
    CHECK(p[0].bearer.dlc.max_frame==180 && p[1].bearer.dlc.max_frame==180 && p[0].bearer.dlc.credit_mode);
    uint8_t a[140],b[90];for(unsigned i=0;i<sizeof(a);++i)a[i]=(uint8_t)i;for(unsigned i=0;i<sizeof(b);++i)b[i]=(uint8_t)(255-i);
    CHECK(dbt_rfcomm_bearer_send(&p[0].bearer,a,sizeof(a),app_done,&p[0])==DBT_OK);
    CHECK(dbt_rfcomm_bearer_send(&p[1].bearer,b,sizeof(b),app_done,&p[1])==DBT_OK);
    for(unsigned i=0;i<200 && (!p[0].done || !p[1].done || !p[0].received_size || !p[1].received_size);++i)step();
    CHECK(p[0].done==1 && p[1].done==1 && p[0].outcome==DBT_TX_ADMITTED && p[1].outcome==DBT_TX_ADMITTED);
    CHECK(p[0].received_size==sizeof(b) && p[1].received_size==sizeof(a));
    for(unsigned i=0;i<sizeof(a);++i)CHECK(p[1].received[i]==a[i]);for(unsigned i=0;i<sizeof(b);++i)CHECK(p[0].received[i]==b[i]);
    CHECK(dbt_rfcomm_bearer_send(&p[0].bearer,a,1,app_done,&p[0])==DBT_BUSY);
    CHECK(dbt_rfcomm_bearer_grant(&p[1].bearer,1)==DBT_OK);
    for(unsigned i=0;i<100 && !p[0].bearer.dlc.credits.tx_available;++i)step();
    CHECK(dbt_rfcomm_bearer_send(&p[0].bearer,a,1,app_done,&p[0])==DBT_OK);
    for(unsigned i=0;i<100 && p[0].done<2;++i)step();CHECK(p[0].done==2);
    CHECK(dbt_rfcomm_bearer_close_dlc(&p[0].bearer)==DBT_OK);
    for(unsigned i=0;i<100 && (!p[0].dlc_closes || !p[1].dlc_closes);++i)step();
    CHECK(p[0].dlc_closes==1 && p[1].dlc_closes==1 && !p[0].errors && !p[1].errors);
    CHECK(dbt_rfcomm_bearer_open(&p[0].bearer,3,200,1,h0)==DBT_OK);
    for(unsigned i=0;i<200 && (p[0].dlc_opens<2 || p[1].dlc_opens<2);++i)step();
    CHECK(p[0].dlc_opens==2 && p[1].dlc_opens==2 && p[0].bearer.dlc.credits.tx_available==1);
    CHECK(dbt_rfcomm_bearer_send(&p[0].bearer,a,sizeof(a),app_done,&p[0])==DBT_OK);
    dbt_rfcomm_bearer_poll(&p[0].bearer,now);CHECK(p[0].bearer.tx_active && p[0].bearer.dlc.tx_pinned);
    uint8_t removed[]={0,1,0,0x13};CHECK(event(&p[0],5,removed,sizeof(removed))==DBT_OK);
    dbt_runtime_poll(&p[0].runtime,++now,8,0);dbt_rfcomm_bearer_poll(&p[0].bearer,now);
    CHECK(p[0].done==3 && p[0].outcome==DBT_TX_LINK_LOST && p[0].bearer.closed);
    dbt_runtime_abort(&p[0].runtime,DBT_FAULT_BAD_RESPONSE);dbt_runtime_abort(&p[1].runtime,DBT_FAULT_BAD_RESPONSE);
    dbt_rfcomm_bearer_poll(&p[0].bearer,now);dbt_rfcomm_bearer_poll(&p[1].bearer,now);
    CHECK(p[0].allocs==p[0].frees && p[1].allocs==p[1].frees);return 0;
}
EXPORT int rfcomm_bearer_bind_oom(void){
    now=0;int r=initialize(&p[0]);if(r)return r;r=initialize(&p[1]);if(r)return r;
    dbt_service_t service={3,256,48,DBT_SECURITY_NONE};CHECK(dbt_channels_register(&p[1].runtime.channels,service)==DBT_OK);
    uint16_t cid;CHECK(dbt_channels_open(&p[0].runtime.channels,1,3,256,48,DBT_SECURITY_NONE,now,&cid)==DBT_OK);
    for(unsigned i=0;i<100 && (!p[0].cid || !p[1].cid);++i)step();CHECK(p[0].cid==cid && p[1].cid);
    unsigned allocated=p[0].allocs,released=p[0].frees;p[0].fail_at=allocated+3;
    CHECK(dbt_rfcomm_bearer_bind(&p[0].bearer,&p[0].runtime,1,p[0].cid,true,10000)==DBT_NO_SPACE);
    CHECK(p[0].allocs==allocated+2 && p[0].frees==released+2);
    p[0].fail_at=0;CHECK(dbt_rfcomm_bearer_bind(&p[0].bearer,&p[0].runtime,1,p[0].cid,true,10000)==DBT_OK);
    dbt_runtime_abort(&p[0].runtime,DBT_FAULT_BAD_RESPONSE);dbt_runtime_abort(&p[1].runtime,DBT_FAULT_BAD_RESPONSE);
    dbt_rfcomm_bearer_poll(&p[0].bearer,now);
    CHECK(p[0].allocs==p[0].frees && p[1].allocs==p[1].frees);return 0;
}
EXPORT int rfcomm_bearer_legacy_mode(void){
    now=0;int r=initialize(&p[0]);if(r)return r;r=initialize(&p[1]);if(r)return r;
    dbt_service_t service={3,256,48,DBT_SECURITY_NONE};CHECK(dbt_channels_register(&p[1].runtime.channels,service)==DBT_OK);
    uint16_t cid;CHECK(dbt_channels_open(&p[0].runtime.channels,1,3,256,48,DBT_SECURITY_NONE,now,&cid)==DBT_OK);
    for(unsigned i=0;i<100 && (!p[0].cid || !p[1].cid);++i)step();CHECK(p[0].cid==cid && p[1].cid);
    CHECK(dbt_rfcomm_bearer_bind(&p[1].bearer,&p[1].runtime,1,p[1].cid,false,10000)==DBT_OK);p[1].bound=true;
    CHECK(dbt_rfcomm_bearer_bind(&p[0].bearer,&p[0].runtime,1,p[0].cid,true,10000)==DBT_OK);p[0].bound=true;
    for(unsigned i=0;i<100 && (p[0].bearer.mux.state!=DBT_RFCOMM_MUX_OPEN || p[1].bearer.mux.state!=DBT_RFCOMM_MUX_OPEN);++i)step();
    dbt_rfcomm_dlc_hooks_t h0={&p[0],dlc_event,dlc_data},h1={&p[1],dlc_event,dlc_data};
    CHECK(dbt_rfcomm_bearer_listen(&p[1].bearer,3,180,1,h1)==DBT_OK);p[0].legacy_pn=true;
    CHECK(dbt_rfcomm_bearer_open(&p[0].bearer,3,180,1,h0)==DBT_OK);
    for(unsigned i=0;i<200 && (!p[0].dlc_opens || !p[1].dlc_opens) && !p[0].errors && !p[1].errors;++i)step();
    CHECK(!p[0].legacy_pn && p[0].dlc_opens==1 && p[1].dlc_opens==1 && !p[0].bearer.dlc.credit_mode && !p[1].bearer.dlc.credit_mode);
    uint8_t bytes[]={1,2,3,4,5};CHECK(dbt_rfcomm_bearer_send(&p[0].bearer,bytes,sizeof(bytes),app_done,&p[0])==DBT_OK);
    for(unsigned i=0;i<100 && (!p[0].done || !p[1].received_size);++i)step();CHECK(p[0].done==1 && p[1].received_size==sizeof(bytes));
    for(unsigned i=0;i<sizeof(bytes);++i)CHECK(p[1].received[i]==bytes[i]);
    dbt_runtime_abort(&p[0].runtime,DBT_FAULT_BAD_RESPONSE);dbt_runtime_abort(&p[1].runtime,DBT_FAULT_BAD_RESPONSE);
    dbt_rfcomm_bearer_poll(&p[0].bearer,now);dbt_rfcomm_bearer_poll(&p[1].bearer,now);
    CHECK(p[0].allocs==p[0].frees && p[1].allocs==p[1].frees);return 0;
}
EXPORT int rfcomm_spp_endpoint_and_sdp(void){
    now=0;int r=initialize(&p[0]);if(r)return r;r=initialize(&p[1]);if(r)return r;
    p[0].runtime.acl.links[0].encrypted=true;p[1].runtime.acl.links[0].encrypted=true;
    dbt_rfcomm_dlc_hooks_t h0={&p[0],dlc_event,dlc_data},h1={&p[1],dlc_event,dlc_data};
    CHECK(!dbt_rfcomm_spp_record(&p[1].spp));
    CHECK(dbt_rfcomm_spp_init(&p[1].spp,&p[1].runtime,3,256,180,1,0x10020,h1)==DBT_OK);p[1].use_spp=true;
    const dbt_sdp_record_t *record=dbt_rfcomm_spp_record(&p[1].spp);CHECK(record && record->handle==0x10020);
    dbt_sdp_t server;CHECK(dbt_sdp_init(&server,record,1,1)==DBT_OK);
    const uint8_t request[]={6,0,1,0,13,0x35,3,0x19,0x11,1,0,0xff,0x35,3,9,0,4,0};
    uint8_t answer[128];uint16_t written=0;CHECK(dbt_sdp_respond(&server,1,request,sizeof(request),answer,sizeof(answer),&written)==DBT_OK && answer[0]==7);
    bool protocol=false;for(unsigned i=0;i+2<written;++i)if(answer[i]==0x19 && answer[i+1]==0 && answer[i+2]==3)protocol=true;CHECK(protocol);
    uint16_t cid;CHECK(dbt_channels_open(&p[0].runtime.channels,1,3,256,48,DBT_SECURITY_ENCRYPTED,now,&cid)==DBT_OK);
    for(unsigned i=0;i<100 && (!p[0].cid || !p[1].cid);++i)step();CHECK(p[0].cid==cid && p[1].cid);
    CHECK(dbt_rfcomm_bearer_bind(&p[0].bearer,&p[0].runtime,1,p[0].cid,true,10000)==DBT_OK);p[0].bound=true;
    for(unsigned i=0;i<100 && (p[0].bearer.mux.state!=DBT_RFCOMM_MUX_OPEN || !p[1].spp.listen_set);++i)step();
    CHECK(p[0].bearer.mux.state==DBT_RFCOMM_MUX_OPEN && p[1].spp.listen_set);
    CHECK(dbt_rfcomm_bearer_open(&p[0].bearer,3,180,1,h0)==DBT_OK);
    for(unsigned i=0;i<200 && (!p[0].dlc_opens || !p[1].dlc_opens) && !p[0].errors && !p[1].errors;++i)step();
    CHECK(p[0].dlc_opens==1 && p[1].dlc_opens==1 && !p[0].errors && !p[1].errors);
    const uint8_t message[]={0x51,0x52,0x53};CHECK(dbt_rfcomm_bearer_send(&p[0].bearer,message,sizeof(message),app_done,&p[0])==DBT_OK);
    for(unsigned i=0;i<100 && (!p[0].done || !p[1].received_size);++i)step();CHECK(p[0].done==1 && p[1].received_size==3);
    for(unsigned i=0;i<3;++i)CHECK(p[1].received[i]==message[i]);
    dbt_runtime_abort(&p[0].runtime,DBT_FAULT_BAD_RESPONSE);dbt_runtime_abort(&p[1].runtime,DBT_FAULT_BAD_RESPONSE);
    dbt_rfcomm_bearer_poll(&p[0].bearer,now);dbt_rfcomm_spp_poll(&p[1].spp,now);
    CHECK(!dbt_rfcomm_spp_record(&p[1].spp));
    CHECK(p[0].allocs==p[0].frees && p[1].allocs==p[1].frees);return 0;
}
EXPORT int rfcomm_probe_echo_restart(void){
    now=0;int r=initialize(&p[0]);if(r)return r;r=initialize(&p[1]);if(r)return r;
    CHECK(!probe_rfcomm_record());p[0].runtime.acl.links[0].encrypted=true;p[1].runtime.acl.links[0].encrypted=true;
    CHECK(probe_rfcomm_init(&p[1].runtime,3,0x10030)==DBT_OK);p[1].use_probe=true;
    const dbt_sdp_record_t *record=probe_rfcomm_record();CHECK(record && record->handle==0x10030 && record->count==5);
    dbt_sdp_t server;CHECK(dbt_sdp_init(&server,record,1,1)==DBT_OK);
    uint16_t cid;CHECK(dbt_channels_open(&p[0].runtime.channels,1,3,256,48,DBT_SECURITY_ENCRYPTED,now,&cid)==DBT_OK);
    for(unsigned i=0;i<100 && (!p[0].cid || !p[1].cid);++i)step();CHECK(p[0].cid==cid && p[1].cid);
    CHECK(dbt_rfcomm_bearer_bind(&p[0].bearer,&p[0].runtime,1,p[0].cid,true,10000)==DBT_OK);p[0].bound=true;
    for(unsigned i=0;i<100 && p[0].bearer.mux.state!=DBT_RFCOMM_MUX_OPEN;++i)step();
    CHECK(p[0].bearer.mux.state==DBT_RFCOMM_MUX_OPEN);
    dbt_rfcomm_dlc_hooks_t h0={&p[0],dlc_event,dlc_data};CHECK(dbt_rfcomm_bearer_open(&p[0].bearer,3,180,1,h0)==DBT_OK);
    for(unsigned i=0;i<200 && !p[0].dlc_opens && !p[0].errors && !p[1].errors;++i)step();CHECK(p[0].dlc_opens==1 && !p[0].errors && !p[1].errors);
    uint8_t bytes[100];for(unsigned i=0;i<sizeof(bytes);++i)bytes[i]=(uint8_t)(i*3u);
    CHECK(dbt_rfcomm_bearer_send(&p[0].bearer,bytes,sizeof(bytes),app_done,&p[0])==DBT_OK);
    for(unsigned i=0;i<200 && (!p[0].done || p[0].received_size!=sizeof(bytes));++i)step();
    CHECK(p[0].done==1 && p[0].received_size==sizeof(bytes));for(unsigned i=0;i<sizeof(bytes);++i)CHECK(p[0].received[i]==bytes[i]);
    for(unsigned i=0;i<100 && !p[0].bearer.dlc.credits.tx_available;++i)step();CHECK(p[0].bearer.dlc.credits.tx_available);
    probe_rfcomm_restart_begin();CHECK(!probe_rfcomm_accept(dbt_acl_find(&p[1].runtime.acl,1),3));
    probe_rfcomm_restart_cancel();CHECK(p[0].bearer.dlc.state==DBT_RFCOMM_DLC_OPEN);
    p[0].done=0;p[0].received_size=0;p[1].stall=true;
    CHECK(dbt_rfcomm_bearer_send(&p[0].bearer,bytes,sizeof(bytes),app_done,&p[0])==DBT_OK);
    for(unsigned i=0;i<50 && !p[0].done;++i)step();CHECK(p[0].done==1);
    for(unsigned i=0;i<10;++i)step();probe_rfcomm_restart_begin();
    CHECK(!probe_rfcomm_restart_quiescent() && !probe_rfcomm_accept(dbt_acl_find(&p[1].runtime.acl,1),3));
    p[1].stall=false;for(unsigned i=0;i<400 && !probe_rfcomm_restart_quiescent();++i)step();
    CHECK(probe_rfcomm_restart_quiescent());probe_rfcomm_restart_cancel();
    CHECK(probe_rfcomm_accept(dbt_acl_find(&p[1].runtime.acl,1),3) && probe_rfcomm_record());
    dbt_rfcomm_bearer_poll(&p[0].bearer,now);p[0].bound=false;p[0].cid=p[1].cid=0;
    unsigned allocated=p[1].allocs,released=p[1].frees;p[1].fail_at=allocated+10;
    CHECK(dbt_channels_open(&p[0].runtime.channels,1,3,256,48,DBT_SECURITY_ENCRYPTED,now,&cid)==DBT_OK);
    for(unsigned i=0;i<200 && (!p[1].cid || p[1].allocs<allocated+9 || p[1].frees<released+9);++i)step();
    CHECK(p[1].cid && p[1].allocs==allocated+9 && p[1].frees==released+9);
    p[1].fail_at=0;probe_rfcomm_restart_begin();
    for(unsigned i=0;i<200 && !probe_rfcomm_restart_quiescent();++i)step();
    CHECK(probe_rfcomm_restart_quiescent());probe_rfcomm_restart_cancel();
    dbt_runtime_abort(&p[0].runtime,DBT_FAULT_BAD_RESPONSE);dbt_runtime_abort(&p[1].runtime,DBT_FAULT_BAD_RESPONSE);
    dbt_rfcomm_bearer_poll(&p[0].bearer,now);probe_rfcomm_poll(now);
    CHECK(p[0].allocs==p[0].frees && p[1].allocs==p[1].frees);return 0;
}
