#include "dbt_avrcp_bearer.h"
#include "dbt_att_bearer.h"
#include <string.h>
#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif
#define CHECK(x) do { if (!(x)) return __LINE__; } while(0)
#define POOL_COUNT 12
#define POOL_SIZE 2048

typedef struct {
    dbt_runtime_t runtime;
    dbt_avctp_t transport;
    dbt_avrcp_t profile;
    dbt_avctp_bearer_t bearer;
    uint8_t rx[1024], tx[1024], packet[64], continuation[1024], target[1024];
    uint8_t wire[DBT_MAX_H4_BYTES], fixed[256], metadata[700];
    _Alignas(8) uint8_t pool[POOL_COUNT][POOL_SIZE];
    bool pool_used[POOL_COUNT], ready, accept, profile_enabled;
    uint8_t *borrowed;
    uint16_t wire_size, cid, fixed_size, metadata_size;
    uint64_t notification_token, metadata_token, passthrough_token;
    unsigned errors, allocs, releases, data_calls, fixed_calls, notification_interim;
    unsigned notification_changed, metadata_complete, metadata_commands, pass_commands, pass_responses, pass_while_metadata;
    unsigned tx_fixed_done, tx_fixed_fail, foreign_done, sends, close_events, borrow_errors, classic_acl_sends, le_acl_sends;
    dbt_tx_outcome_t foreign_outcome;
} endpoint;
static endpoint peers[2];
static dbt_att_server_t att_server;
static dbt_att_bearer_t att_bearer;
static dbt_attribute_t att_attribute;
static const uint8_t att_uuid[]={0xed,0xff};
static uint8_t att_value[100];
static uint8_t metadata_value[600];
static uint32_t tick;
static bool att_ready;
static bool auto_metadata, auto_interim, auto_pass;
static unsigned att_errors;

static void put16(uint8_t *p,uint16_t x) {p[0]=(uint8_t)x; p[1]=(uint8_t)(x>>8);}
static uint16_t get16(const uint8_t *p) {return (uint16_t)(p[0]|(p[1]<<8));}
static bool can_send(void *ctx) {return ((endpoint *)ctx)->ready;}
static bool send_h4(void *ctx,uint8_t *p,uint16_t n) {
    endpoint *e=ctx; if(!e->ready || !e->accept || n>sizeof(e->wire)) return false;
    memcpy(e->wire,p,n); e->wire_size=n; e->borrowed=p; e->ready=false; e->sends++;
    if(n>=3 && p[0]==2) { uint16_t handle=get16(p+1)&0xfff;
        if(handle==1) e->classic_acl_sends++; else if(handle==2) e->le_acl_sends++; }
    return true;
}
static void check_loan(endpoint *e) {
    if(!e->ready && memcmp(e->borrowed,e->wire,e->wire_size)) e->borrow_errors++;
}
static uint8_t *allocate(void *ctx,uint16_t size) {
    endpoint *e=ctx; if(size>POOL_SIZE) return 0;
    for(unsigned i=0;i<POOL_COUNT;i++) if(!e->pool_used[i]) {e->pool_used[i]=true; e->allocs++; return e->pool[i];}
    return 0;
}
static void release(void *ctx,uint8_t *p) {
    endpoint *e=ctx; for(unsigned i=0;i<POOL_COUNT;i++) if(e->pool[i]==p) {
        if(!e->pool_used[i]) { e->errors++; return; }
        e->pool_used[i]=false; e->releases++; return;
    }
    e->errors++;
}
static void profile_message(void *ctx,const dbt_avrcp_message_t *m) {
    endpoint *e=ctx;
    if(m->aborted || m->stale || m->invalid_pid) {e->errors++; return;}
    if(m->response) {
        if(m->pdu==0x31 && m->code==15) e->notification_interim++;
        else if(m->pdu==0x31 && m->code==13) e->notification_changed++;
        else if(m->opcode==0x7c && m->code==9) {e->pass_responses++; if(!e->metadata_complete) e->pass_while_metadata++;}
        else if(m->pdu==0x20 && m->code==12) {
            if(m->size>sizeof(e->metadata)) {e->errors++; return;}
            memcpy(e->metadata,m->parameters,m->size); e->metadata_size=m->size; e->metadata_complete++;
        } else e->errors++;
    } else if(m->pdu==0x31) e->notification_token=m->token;
    else if(m->pdu==0x20) {e->metadata_token=m->token; e->metadata_commands++;}
    else if(m->opcode==0x7c) {e->passthrough_token=m->token; e->pass_commands++;}
    else e->errors++;
}
static void link_change(void *ctx,const dbt_link_change_t *change) {
    endpoint *e=ctx;
    if(e==&peers[1] && att_ready) {
        dbt_result_t r=dbt_att_bearer_link(&att_bearer,change);
        if(r!=DBT_OK) e->errors++;
    }
}
static void channel_event(void *ctx,dbt_channel_event_t event,const dbt_channel_t *c,dbt_channel_reason_t reason,uint16_t result) {
    endpoint *e=ctx; (void)reason; (void)result;
    if(event==DBT_CH_EVENT_OPEN && c->psm==0x17) e->cid=c->local_cid;
    if(event==DBT_CH_EVENT_CLOSED || event==DBT_CH_EVENT_FAILED) e->close_events++;
    if(e->profile_enabled) dbt_avctp_bearer_event(&e->bearer,event,c);
}
static void channel_data(void *ctx,const dbt_channel_t *c,const uint8_t *p,uint16_t n) {
    endpoint *e=ctx; e->data_calls++;
    if(!e->profile_enabled || dbt_avrcp_bearer_receive(&e->bearer,&e->profile,c,p,n)!=DBT_OK) e->errors++;
}
static void fixed_data(void *ctx,const dbt_link_t *link,uint16_t cid,const uint8_t *p,uint16_t n) {
    endpoint *e=ctx; e->fixed_calls++;
    if(e==&peers[1] && att_ready) {
        if(dbt_att_bearer_receive(&att_bearer,link,cid,p,n)!=DBT_OK) att_errors++;
    } else {
        if(n>sizeof(e->fixed)) {e->errors++; return;}
        memcpy(e->fixed,p,n); e->fixed_size=n;
    }
}
static void fixed_done(void *ctx,dbt_tx_outcome_t status) {
    endpoint *e=ctx; e->tx_fixed_done++; if(status!=DBT_TX_ADMITTED) e->tx_fixed_fail++;
}
static void foreign_done(void *ctx,dbt_tx_outcome_t status) {
    endpoint *e=ctx; e->foreign_done++; e->foreign_outcome=status;
}
static dbt_result_t event(endpoint *e,uint8_t code,const uint8_t *p,uint8_t n) {
    uint8_t packet[260]; packet[0]=4; packet[1]=code; packet[2]=n; if(n) memcpy(packet+3,p,n);
    return dbt_runtime_receive(&e->runtime,packet,(uint16_t)(n+3));
}
static void controller_init(endpoint *e) {
    if(e->ready) return;
    check_loan(e);
    if(e->wire[0]!=1) {e->errors++; e->ready=true; return;}
    uint16_t opcode=get16(e->wire+1); uint8_t p[68]={1,(uint8_t)opcode,(uint8_t)(opcode>>8),0}; uint8_t n=4;
    switch(opcode) {
    case 0x1001:n=12;p[4]=8;p[7]=8;break;
    case 0x1002:n=68;break;
    case 0x1003:n=12;p[8]=0x40;break;
    case 0x1009:n=10;p[4]=0x42;break;
    case 0x1005:n=11;p[4]=27;p[7]=2;break;
    case 0x2003:n=12;break;
    case 0x2002:n=7;p[4]=27;p[6]=2;break;
    default:break;
    }
    e->ready=true; if(event(e,0x0e,p,n)!=DBT_OK) e->errors++;
}
static int initialize(endpoint *e) {
    memset(e,0,sizeof(*e)); e->ready=true; e->accept=true;
    dbt_transport_t transport={e,can_send,send_h4}; dbt_runtime_hooks_t h={0};
    h.context=e; h.allocate=allocate; h.release=release; h.link=link_change;
    h.channels.context=e; h.channels.event=channel_event; h.channels.data=channel_data; h.channels.fixed_data=fixed_data;
    CHECK(dbt_runtime_init(&e->runtime,transport,h,1024,100)==DBT_OK);
    CHECK(dbt_runtime_start(&e->runtime,tick)==DBT_OK);
    for(unsigned i=0;i<24 && !e->runtime.ready;i++) {controller_init(e); dbt_runtime_poll(&e->runtime,++tick,8,4);}
    CHECK(e->runtime.ready && e->ready && e->sends==8); return 0;
}
static int link_add(endpoint *e,uint16_t handle,bool le) {
    uint8_t b[19]={0};
    if(le) {b[0]=1; put16(b+2,handle); b[4]=1; CHECK(event(e,0x3e,b,19)==DBT_OK);}
    else {put16(b+1,handle); b[9]=1; CHECK(event(e,3,b,11)==DBT_OK);}
    dbt_runtime_poll(&e->runtime,++tick,8,0); return 0;
}
static int disconnect(endpoint *e,uint16_t handle) {
    uint8_t p[]={0,(uint8_t)handle,(uint8_t)(handle>>8),0x13};
    CHECK(event(e,5,p,4)==DBT_OK); dbt_runtime_poll(&e->runtime,++tick,8,0); return 0;
}
static void deliver(endpoint *src,endpoint *dst) {
    if(src->ready) return;
    check_loan(src);
    if(src->wire[0]==1) {controller_init(src); return;}
    if(src->wire[0]!=2) {src->errors++; src->ready=true; return;}
    uint8_t packet[DBT_MAX_H4_BYTES]; memcpy(packet,src->wire,src->wire_size);
    uint16_t handle=get16(packet+1)&0xfff;
    if((packet[2]&0x30)==0) packet[2]|=0x20;
    if(dbt_runtime_receive(&dst->runtime,packet,src->wire_size)!=DBT_OK) dst->errors++;
    src->ready=true; uint8_t complete[]={1,(uint8_t)handle,(uint8_t)(handle>>8),1,0};
    if(event(src,0x13,complete,5)!=DBT_OK) src->errors++;
}
static void service_profiles(void) {
    endpoint *tg=&peers[1];
    if(tg->profile_enabled && auto_interim && tg->notification_token) {
        uint8_t value[]={13,55}; dbt_result_t r=dbt_avrcp_reply(&tg->profile,tg->notification_token,15,value,2,tick);
        if(r==DBT_OK) auto_interim=false; else if(r!=DBT_BUSY) tg->errors++;
    }
    if(tg->profile_enabled && auto_pass && tg->passthrough_token) {
        uint8_t ack[]={0x44,0}; dbt_result_t r=dbt_avrcp_reply(&tg->profile,tg->passthrough_token,9,ack,2,tick);
        if(r==DBT_OK) auto_pass=false; else if(r!=DBT_BUSY) tg->errors++;
    }
    if(tg->profile_enabled && auto_metadata && tg->metadata_token) {
        dbt_result_t r=dbt_avrcp_reply(&tg->profile,tg->metadata_token,12,metadata_value,sizeof(metadata_value),tick);
        if(r==DBT_OK) auto_metadata=false; else if(r!=DBT_BUSY) tg->errors++;
    }
    for(unsigned i=0;i<2;i++) if(peers[i].profile_enabled && peers[i].bearer.attached) {
        dbt_result_t r=dbt_avrcp_bearer_poll(&peers[i].bearer,&peers[i].profile,tick);
        if(r!=DBT_OK) peers[i].errors++;
    }
    if(att_ready) dbt_att_bearer_poll(&att_bearer,tick);
}
static void pump(unsigned count) {
    for(unsigned i=0;i<count;i++) {
        service_profiles(); deliver(&peers[0],&peers[1]); deliver(&peers[1],&peers[0]); tick++;
        dbt_runtime_poll(&peers[0].runtime,tick,8,4); dbt_runtime_poll(&peers[1].runtime,tick,8,4);
    }
}
static int attach(unsigned i) {
    endpoint *e=&peers[i]; const dbt_channel_t *c=dbt_channels_find(&e->runtime.channels,1,e->cid);
    CHECK(c && c->state==DBT_CH_OPEN && c->psm==0x17);
    CHECK(dbt_avrcp_init(&e->profile,&e->transport,profile_message,e,5000)==DBT_OK);
    CHECK(dbt_avctp_init(&e->transport,dbt_avrcp_hooks(&e->profile),c->serial,c->rx_mtu,c->tx_mtu,false,
        e->rx,sizeof(e->rx),e->tx,sizeof(e->tx),e->packet,3000,5000)==DBT_OK);
    CHECK(dbt_avrcp_set_continuation_buffer(&e->profile,e->continuation,sizeof(e->continuation))==DBT_OK);
    CHECK(dbt_avrcp_set_target_buffer(&e->profile,e->target,sizeof(e->target),10000)==DBT_OK);
    CHECK(dbt_avctp_bearer_init(&e->bearer,&e->runtime,&e->transport,c)==DBT_OK); e->profile_enabled=true; return 0;
}
static int setup(void) {
    tick=0; att_ready=false; auto_metadata=false; auto_interim=false; auto_pass=false; att_errors=0;
    int r=initialize(&peers[0]); if(r) return r; r=initialize(&peers[1]); if(r) return r;
    for(unsigned i=0;i<2;i++) {r=link_add(&peers[i],1,false); if(r)return r; r=link_add(&peers[i],2,true); if(r)return r;}
    CHECK(dbt_channels_register_fixed(&peers[0].runtime.channels,DBT_LINK_LE,4)==DBT_OK);
    for(unsigned i=0;i<100;i++) att_value[i]=(uint8_t)(i*7u);
    att_attribute=(dbt_attribute_t){0x30,DBT_ATT_READ,100,100,0,2,0,att_uuid,att_value};
    dbt_att_hooks_t h={0}; h.context=&peers[1]; h.allocate=allocate; h.release=release;
    CHECK(dbt_att_init(&att_server,&att_attribute,1,h,517,128)==DBT_OK);
    memset(&att_bearer,0,sizeof(att_bearer)); CHECK(dbt_att_bearer_init(&att_bearer,&peers[1].runtime,&att_server)==DBT_OK); att_ready=true;
    dbt_service_t svc={0x17,48,48,DBT_SECURITY_NONE}; CHECK(dbt_channels_register(&peers[1].runtime.channels,svc)==DBT_OK);
    uint16_t cid=0; CHECK(dbt_channels_open(&peers[0].runtime.channels,1,0x17,48,48,DBT_SECURITY_NONE,tick,&cid)==DBT_OK);
    pump(80); CHECK(peers[0].cid==cid && peers[1].cid);
    r=attach(0); if(r)return r; r=attach(1); if(r)return r;
    memset(metadata_value,'x',sizeof(metadata_value)); metadata_value[0]=1;
    metadata_value[1]=metadata_value[2]=metadata_value[3]=0; metadata_value[4]=1;
    metadata_value[5]=0; metadata_value[6]=0x6a; metadata_value[7]=2; metadata_value[8]=79; metadata_value[599]='z';
    return 0;
}
static int cleanup(void) {
    for(unsigned i=0;i<2;i++) dbt_runtime_abort(&peers[i].runtime,DBT_FAULT_BAD_RESPONSE);
    if(att_ready) {CHECK(dbt_att_bearer_clear(&att_bearer)==DBT_OK); att_ready=false;}
    for(unsigned i=0;i<2;i++) {
        if(peers[i].bearer.attached) CHECK(dbt_avctp_bearer_clear(&peers[i].bearer)==DBT_OK);
        CHECK(!peers[i].errors && !peers[i].borrow_errors && peers[i].allocs==peers[i].releases);
    }
    CHECK(!att_errors); return 0;
}
EXPORT int avrcp_runtime_interleave_credit(void) {
    int r=setup(); if(r)return r; endpoint *ct=&peers[0],*tg=&peers[1]; uint8_t label;
    uint8_t registration[]={13,0,0,0,0}; auto_interim=true;
    CHECK(dbt_avrcp_command(&ct->profile,0x31,registration,sizeof(registration),tick,&label)==DBT_OK);
    pump(100); CHECK(ct->notification_interim==1 && tg->notification_token && ct->transport.local_pending);
    static const uint8_t mtu_req[]={2,5,2};
    CHECK(dbt_runtime_send_fixed(&ct->runtime,2,4,mtu_req,sizeof(mtu_req),fixed_done,ct)==DBT_OK);
    pump(60); CHECK(ct->fixed_size==3 && ct->fixed[0]==3 && att_server.connections[0].mtu==517);
    uint8_t attrs[]={0,0,0,0,0,0,0,0,0}; static const uint8_t read_req[]={0x0a,0x30,0};
    unsigned acl_before=tg->classic_acl_sends, sdu_before=ct->data_calls, le_before=tg->le_acl_sends;
    tg->runtime.acl.pools[0].available=0; auto_metadata=true;
    CHECK(dbt_avrcp_command(&ct->profile,0x20,attrs,sizeof(attrs),tick,&label)==DBT_OK);
    CHECK(dbt_runtime_send_fixed(&ct->runtime,2,4,read_req,sizeof(read_req),fixed_done,ct)==DBT_OK);
    pump(120);
    CHECK(tg->metadata_commands==1 && !auto_metadata && !ct->metadata_complete);
    CHECK(tg->bearer.active && tg->transport.pinned && dbt_avctp_bearer_clear(&tg->bearer)==DBT_BUSY);
    CHECK(ct->fixed_size==101 && ct->fixed[0]==0x0b && ct->tx_fixed_done==2 && !ct->tx_fixed_fail);
    for(unsigned i=0;i<100;i++) CHECK(ct->fixed[i+1]==att_value[i]);
    static uint8_t pinned[64]; const uint16_t saved=tg->transport.packet_size;
    CHECK(saved && saved<=sizeof(pinned)); memcpy(pinned,tg->transport.packet,saved);
    pump(30); CHECK(tg->bearer.active && tg->transport.pinned && !memcmp(pinned,tg->transport.packet,saved));
    auto_pass=true; CHECK(dbt_avrcp_passthrough(&ct->profile,0x44,false,tick,&label)==DBT_OK);
    CHECK(dbt_runtime_send_fixed(&ct->runtime,2,4,read_req,sizeof(read_req),fixed_done,ct)==DBT_OK);
    tg->runtime.acl.pools[0].available=2; pump(10);
    CHECK(!ct->metadata_complete && tg->profile.target_active);
    pump(490);
    CHECK(ct->tx_fixed_done==3 && ct->fixed_calls==3 && ct->fixed_size==101 && !ct->tx_fixed_fail);
    for(unsigned i=0;i<100;i++) CHECK(ct->fixed[i+1]==att_value[i]);
    CHECK(ct->pass_responses==1 && ct->pass_while_metadata==1 && !auto_pass);
    CHECK(tg->classic_acl_sends-acl_before>=20 && ct->data_calls-sdu_before>=10 && tg->le_acl_sends-le_before>=4);
    CHECK(ct->metadata_complete==1 && ct->metadata_size==600 && !memcmp(ct->metadata,metadata_value,600));
    CHECK(!tg->bearer.active && !tg->transport.pinned && !ct->bearer.active && !ct->transport.pinned);
    uint8_t volume[]={13,56}; CHECK(dbt_avrcp_reply(&tg->profile,tg->notification_token,13,volume,2,tick)==DBT_OK);
    pump(80); CHECK(ct->notification_changed==1 && !ct->transport.local_pending);
    return cleanup();
}
EXPORT int avrcp_runtime_disconnect_reuse(void) {
    int r=setup(); if(r)return r; endpoint *ct=&peers[0],*tg=&peers[1]; uint8_t l;
    CHECK(dbt_avrcp_passthrough(&ct->profile,0x44,false,tick,&l)==DBT_OK); pump(50);
    CHECK(tg->pass_commands==1 && tg->passthrough_token); uint64_t old_token=tg->passthrough_token;
    uint8_t attrs[]={0,0,0,0,0,0,0,0,2,0,0,0,1,0,0,0,7};
    CHECK(dbt_avrcp_command(&ct->profile,0x20,attrs,sizeof(attrs),tick,&l)==DBT_OK);
    CHECK(dbt_avrcp_bearer_poll(&ct->bearer,&ct->profile,tick)==DBT_OK && ct->bearer.active && ct->transport.pinned);
    dbt_channel_t old=*dbt_channels_find(&ct->runtime.channels,1,ct->cid);
    dbt_runtime_poll(&ct->runtime,++tick,8,1); CHECK(!ct->ready && ct->wire[0]==2);
    CHECK(disconnect(ct,1)==0); check_loan(ct);
    CHECK(!ct->bearer.attached && !ct->bearer.active && !ct->transport.binding && !ct->transport.pinned && !ct->borrow_errors);
    for(unsigned i=0;i<DBT_MAX_LINKS;i++) CHECK(!ct->runtime.tx[i].active || ct->runtime.tx[i].pdu.handle!=1);
    ct->ready=true; /* release simulated controller borrow; do not deliver stale ACL */
    CHECK(disconnect(tg,1)==0); CHECK(!tg->bearer.attached && !tg->bearer.active && !tg->transport.binding && !tg->transport.pinned);
    ct->profile_enabled=false; tg->profile_enabled=false; ct->cid=0; tg->cid=0;
    CHECK(link_add(ct,1,false)==0 && link_add(tg,1,false)==0);
    uint16_t cid=0; CHECK(dbt_channels_open(&ct->runtime.channels,1,0x17,48,48,DBT_SECURITY_NONE,tick,&cid)==DBT_OK);
    pump(80); CHECK(ct->cid==cid && tg->cid);
    CHECK(attach(0)==0 && attach(1)==0);
    const dbt_channel_t *fresh=dbt_channels_find(&ct->runtime.channels,1,ct->cid);
    CHECK(fresh && fresh->link_generation!=old.link_generation && fresh->serial!=old.serial);
    const uint8_t stale[]={0,0x11,0x0e}; CHECK(dbt_avrcp_bearer_receive(&ct->bearer,&ct->profile,&old,stale,sizeof(stale))==DBT_INVALID);
    uint8_t ack[]={0x44,0}; CHECK(dbt_avrcp_reply(&tg->profile,old_token,9,ack,2,tick)==DBT_EMPTY);
    CHECK(!ct->bearer.active && !tg->bearer.active && !ct->transport.pinned && !tg->transport.pinned);
    return cleanup();
}
EXPORT int avrcp_runtime_competing_loan(void) {
    int r=setup(); if(r)return r; endpoint *ct=&peers[0]; static const uint8_t foreign[]={0,0x11,0x0e,0,0x48,0x7c,0x44,0}; uint8_t label;
    ct->runtime.acl.pools[0].available=0;
    CHECK(dbt_runtime_send(&ct->runtime,1,ct->cid,foreign,sizeof(foreign),foreign_done,ct)==DBT_OK);
    CHECK(dbt_avrcp_passthrough(&ct->profile,0x44,false,tick,&label)==DBT_OK);
    CHECK(dbt_avrcp_bearer_poll(&ct->bearer,&ct->profile,tick)==DBT_OK);
    CHECK(!ct->bearer.active && !ct->transport.pinned && ct->transport.tx_used && !ct->foreign_done);
    pump(30); CHECK(!ct->foreign_done && ct->transport.tx_used && !ct->transport.pinned);
    bool borrowed=false; for(unsigned i=0;i<DBT_MAX_LINKS;i++) if(ct->runtime.tx[i].active && ct->runtime.tx[i].pdu.data==foreign) borrowed=true;
    CHECK(borrowed); CHECK(disconnect(ct,1)==0);
    CHECK(ct->foreign_done==1 && ct->foreign_outcome==DBT_TX_LINK_LOST && !ct->bearer.attached && !ct->transport.binding);
    return cleanup();
}
