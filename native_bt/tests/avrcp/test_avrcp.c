#include "dbt_avrcp.h"
#include "dbt_avrcp_browse.h"
#include "dbt_avrcp_bearer.h"
#include <stddef.h>
#ifdef _WIN32
#define EXPORT __declspec(dllexport)
void *memcpy(void *d,const void *s,size_t n) { unsigned char *a=d; const unsigned char *b=s; for(size_t i=0;i<n;i++) a[i]=b[i]; return d; }
void *memmove(void *d,const void *s,size_t n) { unsigned char *a=d; const unsigned char *b=s; if(a<b) for(size_t i=0;i<n;i++) a[i]=b[i]; else for(size_t i=n;i;i--) a[i-1]=b[i-1]; return d; }
void *memset(void *d,int x,size_t n) { unsigned char *a=d; for(size_t i=0;i<n;i++) a[i]=(unsigned char)x; return d; }
#else
#define EXPORT __attribute__((visibility("default")))
#endif
#define CHECK(x) do { if(!(x)) return __LINE__; } while(0)
typedef struct { dbt_avctp_t t; dbt_avrcp_t p; uint8_t rx[1024],tx[1024],packet[64]; uint64_t token; unsigned calls; uint16_t last_size; uint8_t last_code, last_byte, last_pdu; bool last_stale, last_aborted, last_state_change; } peer;
static void callback(void *c,const dbt_avrcp_message_t *m) { peer *p=c; p->calls++; p->last_code=m->code; p->last_pdu=m->pdu; p->last_stale=m->stale; p->last_aborted=m->aborted; p->last_state_change=m->state_change; p->last_size=m->size; p->last_byte=m->size ? m->parameters[m->size-1] : 0; if(!m->response && !m->state_change) p->token=m->token; }
static int setup(peer *p,uint32_t id,uint16_t mtu) {
    *p=(peer){0};
    return dbt_avrcp_init(&p->p,&p->t,callback,p,100)==DBT_OK &&
        dbt_avctp_init(&p->t,dbt_avrcp_hooks(&p->p),id,mtu,mtu,false,p->rx,sizeof(p->rx),p->tx,sizeof(p->tx),p->packet,100,100)==DBT_OK;
}
static int transfer(peer *a,peer *b,uint32_t now) {
    const uint8_t *v; uint16_t n;
    while(dbt_avctp_tx_peek(&a->t,&v,&n)==DBT_OK) {
        if(dbt_avrcp_receive(&b->p,b->t.binding,v,n,now)!=DBT_OK) return 0;
        if(dbt_avctp_tx_commit(&a->t,now)!=DBT_OK) return 0;
    }
    return 1;
}
EXPORT int test_vector_and_fragments(void) {
    static peer a,b; CHECK(setup(&a,1,16)); CHECK(setup(&b,2,16));
    uint8_t label; CHECK(dbt_avrcp_passthrough(&a.p,0x44,false,0,&label)==DBT_OK);
    const uint8_t expected[]={0x00,0x11,0x0e,0x00,0x48,0x7c,0x44,0};
    const uint8_t *p; uint16_t n; CHECK(dbt_avctp_tx_peek(&a.t,&p,&n)==DBT_OK); CHECK(n==sizeof(expected));
    for(unsigned i=0;i<n;i++) CHECK(p[i]==expected[i]);
    CHECK(dbt_avctp_tx_cancel(&a.t)==DBT_OK); CHECK(transfer(&a,&b,1)); CHECK(b.calls==1);
    const uint8_t ok[]={0x44,0}; CHECK(dbt_avrcp_reply(&b.p,b.token,9,ok,2,2)==DBT_OK); CHECK(transfer(&b,&a,3)); CHECK(a.calls==1 && a.last_code==9);
    uint8_t attrs[]={0,0,0,0,0,0,0,0,2,0,0,0,1,0,0,0,7};
    CHECK(dbt_avrcp_command(&a.p,0x20,attrs,sizeof(attrs),4,&label)==DBT_OK);
    CHECK(dbt_avctp_tx_peek(&a.t,&p,&n)==DBT_OK); CHECK(n==16 && p[0]==0x14 && p[1]==2 && p[2]==0x11 && p[3]==0x0e);
    CHECK(dbt_avctp_tx_cancel(&a.t)==DBT_OK); CHECK(transfer(&a,&b,5)); CHECK(b.calls==2); return 0;
}
EXPORT int test_notify_and_parallel(void) {
    static peer a,b; CHECK(setup(&a,31,64)); CHECK(setup(&b,32,64));
    uint8_t l1,l2; uint8_t reg[]={13,0,0,0,0};
    CHECK(dbt_avrcp_command(&a.p,0x31,reg,5,0,&l1)==DBT_OK); CHECK(transfer(&a,&b,1));
    uint64_t token=b.token; uint8_t val[]={13,55}; CHECK(dbt_avrcp_reply(&b.p,token,15,val,2,2)==DBT_OK); CHECK(transfer(&b,&a,3));
    CHECK(a.calls==1 && (a.t.local_pending&(1u<<l1))); CHECK(dbt_avrcp_poll(&a.p,1000)==DBT_OK);
    CHECK(dbt_avrcp_passthrough(&a.p,0x44,false,1001,&l2)==DBT_OK && l1!=l2); CHECK(transfer(&a,&b,1002));
    uint8_t ack[]={0x44,0}; CHECK(dbt_avrcp_reply(&b.p,b.token,9,ack,2,1003)==DBT_OK); CHECK(transfer(&b,&a,1004));
    val[1]=56; CHECK(dbt_avrcp_reply(&b.p,token,13,val,2,1005)==DBT_OK); CHECK(transfer(&b,&a,1006)); CHECK(!a.t.local_pending); CHECK(a.calls==3);
    CHECK(dbt_avrcp_reply(&b.p,token,13,val,2,1007)==DBT_EMPTY); return 0;
}
EXPORT int test_malformed_stale_busy(void) {
    static peer a,b; CHECK(setup(&a,41,64)); CHECK(setup(&b,42,64)); uint8_t l;
    CHECK(dbt_avrcp_passthrough(&a.p,0x44,false,0,&l)==DBT_OK); CHECK(transfer(&a,&b,1));
    uint8_t bad[]={2,0x11,0x0e,9,0x48,0x7c,0x45,0};
    CHECK(dbt_avrcp_receive(&a.p,41,bad,sizeof(bad),2)==DBT_INVALID); CHECK(a.t.local_pending==1);
    CHECK(dbt_avrcp_receive(&a.p,999,bad,sizeof(bad),2)==DBT_EMPTY);
    const uint8_t *view; uint16_t size; uint8_t ack[]={0x44,0};
    CHECK(dbt_avrcp_reply(&b.p,b.token,9,ack,2,3)==DBT_OK); CHECK(dbt_avctp_tx_peek(&b.t,&view,&size)==DBT_OK);
    CHECK(dbt_avctp_clear(&b.t)==DBT_BUSY); CHECK(dbt_avctp_tx_cancel(&b.t)==DBT_OK); CHECK(transfer(&b,&a,4));
    CHECK(dbt_avrcp_receive(&a.p,41,bad,sizeof(bad),5)==DBT_EMPTY);
    CHECK(dbt_avrcp_passthrough(&a.p,0x45,false,6,&l)==DBT_OK); CHECK(dbt_avctp_tx_peek(&a.t,&view,&size)==DBT_OK);
    CHECK(dbt_avctp_poll(&a.t,106)==DBT_INVALID); CHECK(dbt_avctp_clear(&a.t)==DBT_BUSY); CHECK(dbt_avctp_tx_cancel(&a.t)==DBT_OK); CHECK(dbt_avctp_clear(&a.t)==DBT_OK); return 0;
}

EXPORT int test_labels_time_and_ipid(void) {
    static peer a,b; CHECK(setup(&a,90,64)); CHECK(setup(&b,91,64));
    uint8_t l; const uint8_t *p; uint16_t n;
    for (unsigned i=0;i<16;i++) {
        CHECK(dbt_avrcp_passthrough(&a.p,0x44,false,0,&l)==DBT_OK && l==i);
        CHECK(dbt_avctp_tx_peek(&a.t,&p,&n)==DBT_OK); CHECK(dbt_avctp_tx_commit(&a.t,1)==DBT_OK);
    }
    CHECK(dbt_avrcp_passthrough(&a.p,0x44,false,2,&l)==DBT_BUSY);
    uint8_t reply[]={2,0x11,0x0e,9,0x48,0x7c,0x44,0};
    CHECK(dbt_avrcp_receive(&a.p,90,reply,sizeof(reply),101)==DBT_INVALID); CHECK(a.t.failed);
    uint8_t wrong[]={0xa0,0x12,0x34,0};
    CHECK(dbt_avrcp_receive(&b.p,91,wrong,sizeof(wrong),0)==DBT_OK);
    CHECK(dbt_avrcp_poll(&b.p,0)==DBT_OK); CHECK(dbt_avctp_tx_peek(&b.t,&p,&n)==DBT_OK); CHECK(n==3 && p[0]==0xa3 && p[1]==0x12 && p[2]==0x34);
    CHECK(dbt_avctp_tx_cancel(&b.t)==DBT_OK); return 0;
}
EXPORT int test_aba_and_bad_notification(void) {
    static peer a,b; CHECK(setup(&a,200,64)); CHECK(setup(&b,201,64));
    uint8_t l, reg[]={13,0,0,0,0}; CHECK(dbt_avrcp_command(&a.p,0x31,reg,5,0,&l)==DBT_OK); CHECK(transfer(&a,&b,1));
    uint64_t old=b.token; uint8_t value[]={13,2};
    CHECK(dbt_avrcp_reply(&b.p,old,13,value,2,2)==DBT_INVALID);
    CHECK(dbt_avrcp_reply(&b.p,old,15,value,2,2)==DBT_OK); CHECK(transfer(&b,&a,3));
    uint8_t changed[]={2,0x11,0x0e,13,0x48,0,0,0x19,0x58,0x31,0,0,2,1,2};
    CHECK(dbt_avrcp_receive(&a.p,200,changed,sizeof(changed),4)==DBT_INVALID); CHECK(a.t.local_pending==1);
    CHECK(dbt_avctp_clear(&b.t)==DBT_OK); CHECK(setup(&b,202,64));
    CHECK(dbt_avrcp_reply(&b.p,old,13,value,2,4)==DBT_EMPTY);
    return 0;
}
EXPORT int test_unit_discovery(void) {
    static peer a; CHECK(setup(&a,300,64));
    uint8_t unit[]={0x40,0x11,0x0e,1,0xff,0x30,0xff,0xff,0xff,0xff,0xff};
    CHECK(dbt_avrcp_receive(&a.p,300,unit,sizeof(unit),0)==DBT_OK); CHECK(dbt_avrcp_poll(&a.p,1)==DBT_OK);
    const uint8_t *p; uint16_t n; CHECK(dbt_avctp_tx_peek(&a.t,&p,&n)==DBT_OK);
    const uint8_t expected[]={0x42,0x11,0x0e,0x0c,0xff,0x30,7,0x48,0xff,0xff,0xff};
    CHECK(n==sizeof(expected)); for(unsigned i=0;i<n;i++) CHECK(p[i]==expected[i]); CHECK(dbt_avctp_tx_commit(&a.t,1)==DBT_OK);
    uint8_t sub[]={0x50,0x11,0x0e,1,0xff,0x31,7,0xff,0xff,0xff,0xff};
    CHECK(dbt_avrcp_receive(&a.p,300,sub,sizeof(sub),2)==DBT_OK); CHECK(dbt_avrcp_poll(&a.p,3)==DBT_OK);
    CHECK(dbt_avctp_tx_peek(&a.t,&p,&n)==DBT_OK); CHECK(n==11 && p[0]==0x52 && p[5]==0x31 && p[6]==7 && p[7]==0x48);
    CHECK(dbt_avctp_tx_cancel(&a.t)==DBT_OK); return 0;
}
EXPORT int test_metadata_continuation(void) {
    static peer a; static uint8_t storage[100]; CHECK(setup(&a,401,64));
    CHECK(dbt_avrcp_set_continuation_buffer(&a.p,storage,sizeof(storage))==DBT_OK);
    uint8_t attrs[]={0,0,0,0,0,0,0,0,0}, l; CHECK(dbt_avrcp_command(&a.p,0x20,attrs,sizeof(attrs),0,&l)==DBT_OK && l==0);
    const uint8_t *p; uint16_t n; CHECK(dbt_avctp_tx_peek(&a.t,&p,&n)==DBT_OK); CHECK(dbt_avctp_tx_commit(&a.t,1)==DBT_OK);
    uint8_t start[]={2,0x11,0x0e,0x0c,0x48,0,0,0x19,0x58,0x20,1,0,11,1,0,0,0,1,0,0x6a,0,4,'A','B'};
    CHECK(dbt_avrcp_receive(&a.p,401,start,sizeof(start),2)==DBT_OK); CHECK(a.calls==0 && a.p.continuation_active);
    CHECK(dbt_avrcp_poll(&a.p,3)==DBT_OK); CHECK(dbt_avctp_tx_peek(&a.t,&p,&n)==DBT_OK);
    const uint8_t expected[]={0x10,0x11,0x0e,0,0x48,0,0,0x19,0x58,0x40,0,0,1,0x20};
    CHECK(n==sizeof(expected)); for(unsigned i=0;i<n;i++) CHECK(p[i]==expected[i]); CHECK(dbt_avctp_tx_commit(&a.t,3)==DBT_OK);
    uint8_t end[]={0x12,0x11,0x0e,0x0c,0x48,0,0,0x19,0x58,0x20,3,0,2,'C','D'};
    CHECK(dbt_avrcp_receive(&a.p,401,end,sizeof(end),4)==DBT_OK); CHECK(a.calls==1 && a.last_size==13 && a.last_byte=='D');
    CHECK(!a.p.continuation_active && !a.t.local_pending); return 0;
}
EXPORT int test_browse_wire(void) {
    static peer a,b; static dbt_avrcp_browse_t ba,bb; a=(peer){0}; b=(peer){0};
    CHECK(dbt_avrcp_browse_init(&ba,&a.t,callback,&a,100)==DBT_OK);
    CHECK(dbt_avrcp_browse_init(&bb,&b.t,callback,&b,100)==DBT_OK);
    CHECK(dbt_avctp_init(&a.t,dbt_avrcp_browse_hooks(&ba),51,64,64,true,a.rx,sizeof(a.rx),a.tx,sizeof(a.tx),a.packet,100,100)==DBT_OK);
    CHECK(dbt_avctp_init(&b.t,dbt_avrcp_browse_hooks(&bb),52,64,64,true,b.rx,sizeof(b.rx),b.tx,sizeof(b.tx),b.packet,100,100)==DBT_OK);
    uint8_t params[]={0,0,0,0,0,0,0,0,2,0},l;
    CHECK(dbt_avrcp_browse_command(&ba,0x71,params,sizeof(params),0,&l)==DBT_OK);
    const uint8_t *p; uint16_t n; CHECK(dbt_avctp_tx_peek(&a.t,&p,&n)==DBT_OK);
    const uint8_t vector[]={0,0x11,0x0e,0x71,0,10,0,0,0,0,0,0,0,0,2,0};
    CHECK(n==sizeof(vector)); for(unsigned i=0;i<n;i++) CHECK(p[i]==vector[i]);
    CHECK(dbt_avrcp_browse_receive(&bb,52,p,n,1)==DBT_OK); CHECK(dbt_avctp_tx_commit(&a.t,1)==DBT_OK);
    CHECK(b.token!=0); uint8_t bad[]={4,0,1,0,1,2,0,5,1};
    CHECK(dbt_avrcp_browse_reply(&bb,b.token,bad,sizeof(bad),2)==DBT_INVALID);
    uint8_t good[]={4,0,1,0,0}; CHECK(dbt_avrcp_browse_reply(&bb,b.token,good,sizeof(good),2)==DBT_OK);
    CHECK(dbt_avctp_tx_peek(&b.t,&p,&n)==DBT_OK); CHECK(dbt_avrcp_browse_receive(&ba,51,p,n,3)==DBT_OK); CHECK(dbt_avctp_tx_commit(&b.t,3)==DBT_OK);
    CHECK(a.calls==1 && !a.t.local_pending);
    uint8_t fragment[]={4,2,0x11,0x0e,0}; CHECK(dbt_avrcp_browse_receive(&ba,51,fragment,sizeof(fragment),4)==DBT_INVALID); return 0;
}
EXPORT int test_truncated_and_bounded(void) {
    static peer a; static uint8_t storage[4]; CHECK(setup(&a,500,64));
    CHECK(dbt_avrcp_set_continuation_buffer(&a.p,storage,sizeof(storage))==DBT_OK);
    uint8_t attrs[]={0,0,0,0,0,0,0,0,0}, l; CHECK(dbt_avrcp_command(&a.p,0x20,attrs,sizeof(attrs),0,&l)==DBT_OK);
    const uint8_t *v; uint16_t n; CHECK(dbt_avctp_tx_peek(&a.t,&v,&n)==DBT_OK); CHECK(dbt_avctp_tx_commit(&a.t,0)==DBT_OK);
    uint8_t start[]={2,0x11,0x0e,0x0c,0x48,0,0,0x19,0x58,0x20,1,0,5,1,0,0,0,1};
    CHECK(dbt_avrcp_receive(&a.p,500,start,sizeof(start),1)==DBT_NO_SPACE); CHECK(!a.p.failed && a.p.abort_needed);
    static peer b; uint32_t x=0x1f2e3d4c;
    for(unsigned i=0;i<20000;i++) {
        CHECK(setup(&b,501,64)); uint8_t bytes[64]; x=x*1664525u+1013904223u; unsigned length=x%65u;
        for(unsigned j=0;j<length;j++) {x=x*1664525u+1013904223u; bytes[j]=(uint8_t)(x>>24);}
        (void)dbt_avrcp_receive(&b.p,501,bytes,(uint16_t)length,1);
        CHECK(b.t.rx_size<=sizeof(b.rx)); CHECK(b.t.tx_size<=sizeof(b.tx));
    }
    return 0;
}
/* Controlled runtime stand-ins exercise adapter ownership, not the runtime. */
static dbt_channel_t mock_channel;
static dbt_link_t mock_link;
static dbt_tx_done_t mock_done;
static void *mock_context;
static const uint8_t *mock_bytes;
static uint16_t mock_size;
const dbt_channel_t *dbt_channels_find(const dbt_channels_t *c,uint16_t h,uint16_t cid) {
    (void)c; return h==mock_channel.handle && cid==mock_channel.local_cid ? &mock_channel : 0;
}
const dbt_link_t *dbt_acl_find(const dbt_acl_t *c,uint16_t h) {
    (void)c; return h==mock_link.handle ? &mock_link : 0;
}
dbt_result_t dbt_runtime_send(dbt_runtime_t *r,uint16_t h,uint16_t cid,const uint8_t *p,uint16_t n,dbt_tx_done_t cb,void *ctx) {
    (void)r; (void)h; (void)cid; if(mock_done) return DBT_BUSY;
    mock_done=cb; mock_context=ctx; mock_bytes=p; mock_size=n; return DBT_OK;
}
EXPORT int test_bearer_borrow(void) {
    static peer a; static dbt_runtime_t runtime; static dbt_avctp_bearer_t bearer;
    CHECK(setup(&a,606,64)); runtime=(dbt_runtime_t){0}; runtime.ready=true;
    mock_done=0; mock_channel=(dbt_channel_t){0}; mock_link=(dbt_link_t){0};
    mock_channel.serial=606; mock_channel.link_generation=10; mock_channel.handle=9;
    mock_channel.local_cid=0x40; mock_channel.psm=0x17; mock_channel.rx_mtu=64; mock_channel.tx_mtu=64; mock_channel.state=DBT_CH_OPEN;
    mock_link.handle=9; mock_link.generation=10; mock_link.kind=DBT_LINK_CLASSIC;
    CHECK(dbt_avctp_bearer_init(&bearer,&runtime,&a.t,&mock_channel)==DBT_OK);
    uint8_t l; CHECK(dbt_avrcp_passthrough(&a.p,0x44,false,0,&l)==DBT_OK);
    CHECK(dbt_avrcp_bearer_poll(&bearer,&a.p,0)==DBT_OK); CHECK(bearer.active && a.t.pinned && mock_size==8);
    CHECK(dbt_avctp_bearer_clear(&bearer)==DBT_BUSY); CHECK(mock_bytes[6]==0x44);
    dbt_channel_t stale=mock_channel; stale.serial++; CHECK(dbt_avrcp_bearer_receive(&bearer,&a.p,&stale,mock_bytes,mock_size)==DBT_INVALID);
    dbt_tx_done_t done=mock_done; mock_done=0; runtime.now_ms=1; done(mock_context,DBT_TX_ADMITTED);
    CHECK(!bearer.active && !a.t.pinned && a.t.local_sent==1);
    CHECK(dbt_avrcp_passthrough(&a.p,0x45,false,2,&l)==DBT_OK);
    CHECK(dbt_avrcp_bearer_poll(&bearer,&a.p,2)==DBT_OK); CHECK(bearer.active);
    dbt_avctp_bearer_event(&bearer,DBT_CH_EVENT_CLOSED,&mock_channel); CHECK(bearer.attached);
    done=mock_done; mock_done=0; done(mock_context,DBT_TX_ABORTED); CHECK(!bearer.attached && !a.t.binding);
    return 0;
}
EXPORT int test_notification_label_reserve(void) {
    static peer a; CHECK(setup(&a,700,64)); uint8_t reg[]={13,0,0,0,0}, label;
    const uint8_t *p; uint16_t n;
    for(unsigned i=0;i<15;i++) {
        CHECK(dbt_avrcp_command(&a.p,0x31,reg,sizeof(reg),0,&label)==DBT_OK);
        CHECK(dbt_avctp_tx_peek(&a.t,&p,&n)==DBT_OK); CHECK(dbt_avctp_tx_commit(&a.t,1)==DBT_OK);
    }
    CHECK(dbt_avrcp_command(&a.p,0x31,reg,sizeof(reg),2,&label)==DBT_BUSY);
    CHECK(dbt_avrcp_passthrough(&a.p,0x44,false,2,&label)==DBT_OK && label==15);
    return 0;
}
EXPORT int test_target_continuation(void) {
    static peer a,b; static uint8_t a_store[1024],b_store[1024],metadata_body[600];
    CHECK(setup(&a,801,64)); CHECK(setup(&b,802,64));
    CHECK(dbt_avrcp_set_continuation_buffer(&a.p,a_store,sizeof(a_store))==DBT_OK);
    CHECK(dbt_avrcp_set_target_buffer(&b.p,b_store,sizeof(b_store),1000)==DBT_OK);
    uint8_t attrs[]={0,0,0,0,0,0,0,0,0},l;
    CHECK(dbt_avrcp_command(&a.p,0x20,attrs,sizeof(attrs),0,&l)==DBT_OK); CHECK(transfer(&a,&b,1));
    metadata_body[0]=1; metadata_body[1]=0; metadata_body[2]=0; metadata_body[3]=0; metadata_body[4]=1;
    metadata_body[5]=0; metadata_body[6]=0x6a; metadata_body[7]=2; metadata_body[8]=79;
    for(unsigned i=9;i<600;i++) metadata_body[i]='x';
    metadata_body[599]='z';
    CHECK(dbt_avrcp_reply(&b.p,b.token,0x0c,metadata_body,sizeof(metadata_body),2)==DBT_OK);
    CHECK(b.p.target_active && transfer(&b,&a,3)); CHECK(a.p.continuation_needed && !a.calls);
    CHECK(dbt_avrcp_poll(&a.p,4)==DBT_OK); CHECK(transfer(&a,&b,5));
    CHECK(dbt_avrcp_poll(&b.p,6)==DBT_OK); CHECK(transfer(&b,&a,7));
    CHECK(!b.p.target_active && !a.p.continuation_active && a.calls==1 && a.last_size==600 && a.last_byte=='z');
    return 0;
}
EXPORT int test_target_abort_and_expiry(void) {
    static peer b; static uint8_t store[700],data[600]; CHECK(setup(&b,900,64));
    CHECK(dbt_avrcp_set_target_buffer(&b.p,store,sizeof(store),20)==DBT_OK);
    uint8_t cmd[]={0,0x11,0x0e,1,0x48,0,0,0x19,0x58,0x20,0,0,9,0,0,0,0,0,0,0,0,0};
    CHECK(dbt_avrcp_receive(&b.p,900,cmd,sizeof(cmd),0)==DBT_OK);
    data[0]=1; data[4]=1; data[6]=0x6a; data[7]=2; data[8]=79;
    CHECK(dbt_avrcp_reply(&b.p,b.token,0x0c,data,sizeof(data),1)==DBT_OK);
    const uint8_t *p; uint16_t n; while(dbt_avctp_tx_peek(&b.t,&p,&n)==DBT_OK) CHECK(dbt_avctp_tx_commit(&b.t,2)==DBT_OK);
    uint8_t abort[]={0x10,0x11,0x0e,0,0x48,0,0,0x19,0x58,0x41,0,0,1,0x20};
    CHECK(dbt_avrcp_receive(&b.p,900,abort,sizeof(abort),3)==DBT_OK);
    CHECK(dbt_avrcp_poll(&b.p,4)==DBT_OK && !b.p.target_active);
    CHECK(dbt_avctp_tx_peek(&b.t,&p,&n)==DBT_OK);
    const uint8_t expected[]={0x12,0x11,0x0e,9,0x48,0,0,0x19,0x58,0x41,0,0,0};
    CHECK(n==sizeof(expected)); for(unsigned i=0;i<n;i++) CHECK(p[i]==expected[i]); CHECK(dbt_avctp_tx_commit(&b.t,5)==DBT_OK);
    CHECK(dbt_avrcp_receive(&b.p,900,cmd,sizeof(cmd),6)==DBT_OK);
    CHECK(dbt_avrcp_reply(&b.p,b.token,0x0c,data,sizeof(data),7)==DBT_OK);
    while(dbt_avctp_tx_peek(&b.t,&p,&n)==DBT_OK) CHECK(dbt_avctp_tx_commit(&b.t,8)==DBT_OK);
    CHECK(dbt_avrcp_poll(&b.p,27)==DBT_OK && !b.p.target_active); return 0;
}
EXPORT int test_control_reject_backpressure(void) {
    static peer a; CHECK(setup(&a,1001,64)); uint8_t l;
    CHECK(dbt_avrcp_passthrough(&a.p,0x44,false,0,&l)==DBT_OK);
    uint8_t unknown[]={0x30,0x11,0x0e,1,0x48,0,0,0x19,0x58,0xee,0,0,0};
    CHECK(dbt_avrcp_receive(&a.p,1001,unknown,sizeof(unknown),1)==DBT_OK);
    CHECK(dbt_avrcp_poll(&a.p,2)==DBT_OK && (a.p.remote_auto&(1u<<3)));
    const uint8_t *p; uint16_t n; CHECK(dbt_avctp_tx_peek(&a.t,&p,&n)==DBT_OK); CHECK(dbt_avctp_tx_commit(&a.t,2)==DBT_OK);
    CHECK(dbt_avrcp_poll(&a.p,3)==DBT_OK); CHECK(dbt_avctp_tx_peek(&a.t,&p,&n)==DBT_OK);
    uint8_t expected[]={0x32,0x11,0x0e,0x0a,0x48,0,0,0x19,0x58,0xee,0,0,1,0};
    CHECK(n==sizeof(expected)); for(unsigned i=0;i<n;i++) CHECK(p[i]==expected[i]); CHECK(dbt_avctp_tx_commit(&a.t,3)==DBT_OK);
    uint8_t invalid[]={0x40,0x11,0x0e,1,0x48,0,0,0x19,0x58,0x10,0,0,1,9};
    CHECK(dbt_avrcp_receive(&a.p,1001,invalid,sizeof(invalid),4)==DBT_OK); CHECK(dbt_avrcp_poll(&a.p,4)==DBT_OK);
    CHECK(dbt_avctp_tx_peek(&a.t,&p,&n)==DBT_OK); CHECK(n==14 && p[0]==0x42 && p[9]==0x10 && p[13]==1); return 0;
}
EXPORT int test_controller_abort_quarantines(void) {
    static peer a; static uint8_t storage[100]; CHECK(setup(&a,1002,64)); CHECK(dbt_avrcp_set_continuation_buffer(&a.p,storage,sizeof(storage))==DBT_OK);
    uint8_t attrs[]={0,0,0,0,0,0,0,0,0},l; const uint8_t *p; uint16_t n;
    CHECK(dbt_avrcp_command(&a.p,0x20,attrs,9,0,&l)==DBT_OK); CHECK(dbt_avctp_tx_peek(&a.t,&p,&n)==DBT_OK); CHECK(dbt_avctp_tx_commit(&a.t,0)==DBT_OK);
    uint8_t start[]={2,0x11,0x0e,0x0c,0x48,0,0,0x19,0x58,0x20,1,0,1,1};
    CHECK(dbt_avrcp_receive(&a.p,1002,start,sizeof(start),1)==DBT_OK); CHECK(dbt_avrcp_poll(&a.p,2)==DBT_OK);
    CHECK(dbt_avctp_tx_peek(&a.t,&p,&n)==DBT_OK && p[0]==0x10 && p[9]==0x40); CHECK(dbt_avctp_tx_commit(&a.t,2)==DBT_OK);
    CHECK(dbt_avrcp_abort_continuation(&a.p)==DBT_OK && a.last_aborted && (a.p.local_abandoned&2));
    CHECK(dbt_avrcp_poll(&a.p,3)==DBT_OK); CHECK(dbt_avctp_tx_peek(&a.t,&p,&n)==DBT_OK && p[0]==0x20 && p[9]==0x41); CHECK(dbt_avctp_tx_commit(&a.t,3)==DBT_OK);
    unsigned calls=a.calls; uint8_t late[]={0x12,0x11,0x0e,0x0c,0x48,0,0,0x19,0x58,0x20,3,0,1,0};
    CHECK(dbt_avrcp_receive(&a.p,1002,late,sizeof(late),4)==DBT_OK && a.calls==calls && !(a.p.local_abandoned&2));
    uint8_t done[]={0x22,0x11,0x0e,9,0x48,0,0,0x19,0x58,0x41,0,0,0};
    CHECK(dbt_avrcp_receive(&a.p,1002,done,sizeof(done),5)==DBT_OK && !a.t.local_pending && a.last_pdu==0x41); return 0;
}
EXPORT int test_controller_interleave_aborts(void) {
    static peer a; static uint8_t storage[100]; CHECK(setup(&a,1003,64)); CHECK(dbt_avrcp_set_continuation_buffer(&a.p,storage,sizeof(storage))==DBT_OK);
    uint8_t reg[]={13,0,0,0,0},attrs[]={0,0,0,0,0,0,0,0,0},l; const uint8_t *p; uint16_t n;
    CHECK(dbt_avrcp_command(&a.p,0x31,reg,5,0,&l)==DBT_OK); CHECK(dbt_avctp_tx_peek(&a.t,&p,&n)==DBT_OK); CHECK(dbt_avctp_tx_commit(&a.t,0)==DBT_OK);
    uint8_t interim[]={2,0x11,0x0e,15,0x48,0,0,0x19,0x58,0x31,0,0,2,13,7};
    CHECK(dbt_avrcp_receive(&a.p,1003,interim,sizeof(interim),1)==DBT_OK);
    CHECK(dbt_avrcp_command(&a.p,0x20,attrs,9,2,&l)==DBT_OK); CHECK(dbt_avctp_tx_peek(&a.t,&p,&n)==DBT_OK); CHECK(dbt_avctp_tx_commit(&a.t,2)==DBT_OK);
    uint8_t start[]={0x12,0x11,0x0e,0x0c,0x48,0,0,0x19,0x58,0x20,1,0,1,1};
    CHECK(dbt_avrcp_receive(&a.p,1003,start,sizeof(start),3)==DBT_OK); CHECK(dbt_avrcp_poll(&a.p,4)==DBT_OK);
    CHECK(dbt_avctp_tx_peek(&a.t,&p,&n)==DBT_OK && p[0]==0x20); CHECK(dbt_avctp_tx_commit(&a.t,4)==DBT_OK);
    unsigned calls=a.calls; interim[3]=13; interim[14]=8;
    CHECK(dbt_avrcp_receive(&a.p,1003,interim,sizeof(interim),5)==DBT_OK);
    CHECK(a.calls==calls+2 && !a.p.continuation_active && !a.p.abort_needed && (a.p.local_abandoned&4));
    uint8_t late[]={0x22,0x11,0x0e,0x0c,0x48,0,0,0x19,0x58,0x20,3,0,1,0};
    CHECK(dbt_avrcp_receive(&a.p,1003,late,sizeof(late),6)==DBT_OK && !a.p.local_abandoned); return 0;
}
static int browse_transfer(peer *a,dbt_avrcp_browse_t *target,peer *b,uint32_t now) {
    const uint8_t *p; uint16_t n;
    while(dbt_avctp_tx_peek(&a->t,&p,&n)==DBT_OK) {
        if(dbt_avrcp_browse_receive(target,b->t.binding,p,n,now)!=DBT_OK) return 0;
        if(dbt_avctp_tx_commit(&a->t,now)!=DBT_OK) return 0;
    }
    return 1;
}
static int browse_setup(peer *p,dbt_avrcp_browse_t *b,uint32_t binding) {
    *p=(peer){0};
    return dbt_avrcp_browse_init(b,&p->t,callback,p,100)==DBT_OK &&
        dbt_avctp_init(&p->t,dbt_avrcp_browse_hooks(b),binding,64,64,true,p->rx,sizeof(p->rx),p->tx,sizeof(p->tx),p->packet,100,100)==DBT_OK;
}
EXPORT int test_browse_player_uid_lifecycle(void) {
    static peer a,b; static dbt_avrcp_browse_t ba,bb; CHECK(browse_setup(&a,&ba,1101)); CHECK(browse_setup(&b,&bb,1102));
    uint8_t l, folder[]={1,0,0,0,0,0,0,0,2,0}, player[]={0,0x42};
    CHECK(dbt_avrcp_browse_command(&ba,0x71,folder,sizeof(folder),0,&l)==DBT_INVALID);
    CHECK(dbt_avrcp_browse_command(&ba,0x70,player,2,0,&l)==DBT_OK); CHECK(browse_transfer(&a,&bb,&b,1));
    uint8_t selected[]={4,0x12,0x34,0,0,0,5,0,0x6a,0};
    CHECK(dbt_avrcp_browse_reply(&bb,b.token,selected,sizeof(selected),2)==DBT_OK);
    CHECK(bb.target_selected && bb.target_player==0x42 && bb.target_uid==0x1234 && !b.last_state_change);
    CHECK(browse_transfer(&b,&ba,&a,3)); CHECK(b.last_state_change); CHECK(ba.local_selected && ba.local_player==0x42 && ba.local_uid==0x1234);
    CHECK(dbt_avrcp_browse_command(&ba,0x71,folder,sizeof(folder),4,&l)==DBT_OK); CHECK(browse_transfer(&a,&bb,&b,5)); uint64_t old=b.token;
    CHECK(dbt_avrcp_browse_uids_changed(&bb,true,0x1235,false)==DBT_OK);
    uint8_t list[]={4,0x12,0x34,0,0}; CHECK(dbt_avrcp_browse_reply(&bb,old,list,sizeof(list),6)==DBT_EMPTY);
    CHECK(dbt_avrcp_browse_poll(&bb,6)==DBT_OK); CHECK(browse_transfer(&b,&ba,&a,7)); CHECK(a.last_pdu==0x71 && a.last_byte==5 && !ba.local_uid_valid && !ba.local_search_valid);
    CHECK(dbt_avrcp_browse_uids_changed(&ba,false,0x1235,false)==DBT_OK);
    uint8_t change[]={0x12,0x34,1,0,0,0,0,0,0,0,5}; CHECK(dbt_avrcp_browse_command(&ba,0x72,change,sizeof(change),8,&l)==DBT_INVALID);
    change[1]=0x35; CHECK(dbt_avrcp_browse_command(&ba,0x72,change,sizeof(change),8,&l)==DBT_OK); CHECK(browse_transfer(&a,&bb,&b,9));
    uint8_t count[]={4,0,0,0,4}; CHECK(dbt_avrcp_browse_reply(&bb,b.token,count,sizeof(count),10)==DBT_OK); CHECK(browse_transfer(&b,&ba,&a,11));
    CHECK(ba.local_depth==1 && bb.target_depth==1);
    uint8_t search[]={0,0x6a,0,1,'x'}; CHECK(dbt_avrcp_browse_command(&ba,0x80,search,sizeof(search),12,&l)==DBT_OK); CHECK(browse_transfer(&a,&bb,&b,13));
    uint8_t search_reply[]={4,0x12,0x35,0,0,0,2}; CHECK(dbt_avrcp_browse_reply(&bb,b.token,search_reply,sizeof(search_reply),14)==DBT_OK);
    CHECK(browse_transfer(&b,&ba,&a,15)); CHECK(ba.local_search_valid && bb.target_search_valid);
    CHECK(dbt_avrcp_browse_command(&ba,0x71,folder,sizeof(folder),16,&l)==DBT_OK); CHECK(browse_transfer(&a,&bb,&b,17));
    list[2]=0x35; CHECK(dbt_avrcp_browse_reply(&bb,b.token,list,sizeof(list),18)==DBT_OK);
    CHECK(dbt_avrcp_browse_uids_changed(&ba,false,0x1236,false)==DBT_OK); CHECK(browse_transfer(&b,&ba,&a,19)); CHECK(a.last_stale && ba.local_uid==0x1236 && !ba.local_search_valid);
    CHECK(dbt_avrcp_browse_uids_changed(&bb,true,0,true)==DBT_OK && !bb.target_selected); return 0;
}
EXPORT int test_browse_rejects_and_items(void) {
    static peer a; static dbt_avrcp_browse_t ba; CHECK(browse_setup(&a,&ba,1201));
    uint8_t unknown[]={0x40,0x11,0x0e,0xe2,0,0}; const uint8_t *p; uint16_t n;
    CHECK(dbt_avrcp_browse_receive(&ba,1201,unknown,sizeof(unknown),0)==DBT_OK); CHECK(dbt_avrcp_browse_poll(&ba,0)==DBT_OK);
    CHECK(dbt_avctp_tx_peek(&a.t,&p,&n)==DBT_OK); uint8_t expected[]={0x42,0x11,0x0e,0xa0,0,1,0};
    CHECK(n==sizeof(expected)); for(unsigned i=0;i<n;i++) CHECK(p[i]==expected[i]); CHECK(dbt_avctp_tx_commit(&a.t,0)==DBT_OK);
    uint8_t malformed[]={0x50,0x11,0x0e,0x70,0,1,0};
    CHECK(dbt_avrcp_browse_receive(&ba,1201,malformed,sizeof(malformed),1)==DBT_OK); CHECK(dbt_avrcp_browse_poll(&ba,1)==DBT_OK);
    CHECK(dbt_avctp_tx_peek(&a.t,&p,&n)==DBT_OK && n==7 && p[0]==0x52 && p[3]==0x70 && p[6]==2); CHECK(dbt_avctp_tx_commit(&a.t,1)==DBT_OK);
    uint8_t folder[]={0,0,0,0,0,0,0,0,2,0}, l;
    CHECK(dbt_avrcp_browse_command(&ba,0x71,folder,sizeof(folder),2,&l)==DBT_OK);
    CHECK(dbt_avctp_tx_peek(&a.t,&p,&n)==DBT_OK); CHECK(dbt_avctp_tx_commit(&a.t,2)==DBT_OK);
    uint8_t bad_item[]={2,0x11,0x0e,0x71,0,8,4,0,1,0,1,1,0,0};
    CHECK(dbt_avrcp_browse_receive(&ba,1201,bad_item,sizeof(bad_item),3)==DBT_INVALID);
    uint8_t general[]={2,0x11,0x0e,0xa0,0,1,0}; CHECK(dbt_avrcp_browse_receive(&ba,1201,general,sizeof(general),4)==DBT_OK);
    CHECK(a.last_pdu==0xa0 && !a.t.local_pending); return 0;
}
EXPORT int test_control_browse_ops_and_notifications(void) {
    static peer a,b; CHECK(setup(&a,1301,64)); CHECK(setup(&b,1302,64)); uint8_t l;
    uint8_t player[]={0,0x42}; CHECK(dbt_avrcp_command(&a.p,0x60,player,2,0,&l)==DBT_OK); CHECK(transfer(&a,&b,1));
    uint8_t ok[]={4}; CHECK(dbt_avrcp_reply(&b.p,b.token,9,ok,1,2)==DBT_OK); CHECK(transfer(&b,&a,3)); CHECK(a.last_pdu==0x60);
    uint8_t play[]={3,0,0,0,0,0,0,0,7,0x99,0x99};
    CHECK(dbt_avrcp_command(&a.p,0x74,play,sizeof(play),4,&l)==DBT_OK); CHECK(transfer(&a,&b,5));
    CHECK(dbt_avrcp_reply(&b.p,b.token,9,ok,1,6)==DBT_OK); CHECK(transfer(&b,&a,7)); CHECK(a.last_pdu==0x74);
    uint8_t reg[]={1,0,0,0,0}; CHECK(dbt_avrcp_command(&a.p,0x31,reg,5,8,&l)==DBT_OK); CHECK(transfer(&a,&b,9));
    uint8_t state[]={1,1}; CHECK(dbt_avrcp_reply(&b.p,b.token,15,state,2,10)==DBT_OK); CHECK(transfer(&b,&a,11));
    reg[0]=13; CHECK(dbt_avrcp_command(&a.p,0x31,reg,5,12,&l)==DBT_OK); CHECK(transfer(&a,&b,13)); uint64_t volume_token=b.token;
    uint8_t vol[]={13,55}; CHECK(dbt_avrcp_reply(&b.p,volume_token,15,vol,2,14)==DBT_OK); CHECK(transfer(&b,&a,15));
    CHECK(dbt_avrcp_addressed_player_changed(&b.p,16)==DBT_OK); CHECK(dbt_avrcp_poll(&b.p,16)==DBT_OK);
    CHECK(transfer(&b,&a,17)); CHECK(a.last_pdu==0x31 && a.last_code==0x0a && a.last_byte==0x16);
    CHECK(dbt_avrcp_reply(&b.p,volume_token,13,vol,2,18)==DBT_OK); CHECK(transfer(&b,&a,19)); CHECK(!a.t.local_pending); return 0;
}
EXPORT int test_browse_addressed_scope_uid(void) {
    static peer a,b; static dbt_avrcp_browse_t ba,bb; CHECK(browse_setup(&a,&ba,1401)); CHECK(browse_setup(&b,&bb,1402));
    uint8_t l, player[]={0,1}; CHECK(dbt_avrcp_browse_command(&ba,0x70,player,2,0,&l)==DBT_OK); CHECK(browse_transfer(&a,&bb,&b,1));
    uint8_t selected[]={4,0x12,0x34,0,0,0,0,0,0x6a,0}; CHECK(dbt_avrcp_browse_reply(&bb,b.token,selected,sizeof(selected),2)==DBT_OK); CHECK(browse_transfer(&b,&ba,&a,3));
    CHECK(dbt_avrcp_browse_addressed_uids_changed(&ba,false,0x9999,true)==DBT_OK);
    CHECK(dbt_avrcp_browse_addressed_uids_changed(&bb,true,0x9999,true)==DBT_OK);
    uint8_t get[]={3,0,0,0,0,0,0,0,7,0x12,0x34,0}; CHECK(dbt_avrcp_browse_command(&ba,0x73,get,sizeof(get),4,&l)==DBT_INVALID);
    get[9]=0x99; get[10]=0x99; CHECK(dbt_avrcp_browse_command(&ba,0x73,get,sizeof(get),4,&l)==DBT_OK); CHECK(browse_transfer(&a,&bb,&b,5));
    uint8_t result[]={4,1,0,0,0,1,0,0x6a,0,1,'x'}; CHECK(dbt_avrcp_browse_reply(&bb,b.token,result,sizeof(result),6)==DBT_OK); CHECK(browse_transfer(&b,&ba,&a,7));
    CHECK(ba.local_uid==0x1234 && ba.local_addressed_uid==0x9999 && bb.target_uid==0x1234 && bb.target_addressed_uid==0x9999); return 0;
}
EXPORT int test_not_implemented_echo(void) {
    static peer a; static uint8_t echo[512]; CHECK(setup(&a,1501,64)); CHECK(dbt_avrcp_set_echo_buffer(&a.p,echo,sizeof(echo))==DBT_OK);
    uint8_t l; CHECK(dbt_avrcp_passthrough(&a.p,0x44,false,0,&l)==DBT_OK);
    uint8_t unsupported[]={0x70,0x11,0x0e,1,0x48,0x71,0xa1,0xb2,0xc3};
    CHECK(dbt_avrcp_receive(&a.p,1501,unsupported,sizeof(unsupported),1)==DBT_OK);
    uint8_t other[]={0x80,0x11,0x0e,1,0x48,0x72,0x44}; CHECK(dbt_avrcp_receive(&a.p,1501,other,sizeof(other),1)==DBT_BUSY);
    const uint8_t *p; uint16_t n; CHECK(dbt_avctp_tx_peek(&a.t,&p,&n)==DBT_OK); CHECK(dbt_avctp_tx_commit(&a.t,1)==DBT_OK);
    CHECK(dbt_avrcp_poll(&a.p,2)==DBT_OK); CHECK(dbt_avctp_tx_peek(&a.t,&p,&n)==DBT_OK);
    const uint8_t expected[]={0x72,0x11,0x0e,8,0x48,0x71,0xa1,0xb2,0xc3}; CHECK(n==sizeof(expected));
    for(unsigned i=0;i<n;i++) CHECK(p[i]==expected[i]);
    CHECK(dbt_avctp_tx_commit(&a.t,2)==DBT_OK);
    uint8_t vol[]={12}; CHECK(dbt_avrcp_command(&a.p,0x50,vol,1,3,&l)==DBT_OK); CHECK(dbt_avctp_tx_peek(&a.t,&p,&n)==DBT_OK); CHECK(dbt_avctp_tx_commit(&a.t,3)==DBT_OK);
    uint8_t bad[]={0x12,0x11,0x0e,8,0x48,0,0,0x19,0x58,0x50,0,0,1,13};
    CHECK(dbt_avrcp_receive(&a.p,1501,bad,sizeof(bad),4)==DBT_INVALID); CHECK(a.t.local_pending&2);
    bad[13]=12; CHECK(dbt_avrcp_receive(&a.p,1501,bad,sizeof(bad),5)==DBT_OK && a.last_code==8 && !(a.t.local_pending&2)); return 0;
}
EXPORT int test_browse_large_bounded_sdu(void) {
    static peer a; static dbt_avctp_t transport; static dbt_avrcp_browse_t browse;
    static uint8_t rx[900],tx[900],packet[900],payload[600]; a=(peer){0};
    CHECK(dbt_avrcp_browse_init(&browse,&transport,callback,&a,100)==DBT_OK);
    CHECK(dbt_avctp_init(&transport,dbt_avrcp_browse_hooks(&browse),1601,900,900,true,rx,sizeof(rx),tx,sizeof(tx),packet,100,100)==DBT_OK);
    uint8_t select[]={0,0x11,0x0e,0x70,0,2,0,1}; CHECK(dbt_avrcp_browse_receive(&browse,1601,select,sizeof(select),0)==DBT_OK);
    uint8_t ok[]={4,0,1,0,0,0,0,0,0x6a,0}; CHECK(dbt_avrcp_browse_reply(&browse,a.token,ok,sizeof(ok),1)==DBT_OK);
    const uint8_t *p; uint16_t n; CHECK(dbt_avctp_tx_peek(&transport,&p,&n)==DBT_OK); CHECK(dbt_avctp_tx_commit(&transport,1)==DBT_OK);
    uint8_t req[]={0x10,0x11,0x0e,0x73,0,12,3,0,0,0,0,0,0,0,7,0,1,0};
    CHECK(dbt_avrcp_browse_receive(&browse,1601,req,sizeof(req),2)==DBT_OK);
    payload[0]=4; payload[1]=1; payload[5]=1; payload[7]=0x6a; payload[8]=2; payload[9]=78;
    for(unsigned i=10;i<600;i++) payload[i]='x';
    CHECK(dbt_avrcp_browse_reply(&browse,a.token,payload,sizeof(payload),3)==DBT_OK);
    CHECK(dbt_avctp_tx_peek(&transport,&p,&n)==DBT_OK && n==606 && (p[0]&0x0c)==0 && p[3]==0x73 && p[4]==2 && p[5]==0x58);
    CHECK(dbt_avctp_tx_commit(&transport,3)==DBT_OK); return 0;
}
EXPORT int test_continuation_rejected(void) {
    static peer a; static uint8_t storage[100]; CHECK(setup(&a,1701,64)); CHECK(dbt_avrcp_set_continuation_buffer(&a.p,storage,sizeof(storage))==DBT_OK);
    uint8_t attrs[]={0,0,0,0,0,0,0,0,0},l; const uint8_t *p; uint16_t n;
    CHECK(dbt_avrcp_command(&a.p,0x20,attrs,9,0,&l)==DBT_OK); CHECK(dbt_avctp_tx_peek(&a.t,&p,&n)==DBT_OK); CHECK(dbt_avctp_tx_commit(&a.t,0)==DBT_OK);
    uint8_t start[]={2,0x11,0x0e,0x0c,0x48,0,0,0x19,0x58,0x20,1,0,1,1};
    CHECK(dbt_avrcp_receive(&a.p,1701,start,sizeof(start),1)==DBT_OK); CHECK(dbt_avrcp_poll(&a.p,2)==DBT_OK);
    CHECK(dbt_avctp_tx_peek(&a.t,&p,&n)==DBT_OK); CHECK(dbt_avctp_tx_commit(&a.t,2)==DBT_OK);
    uint8_t empty[]={0x12,0x11,0x0e,8,0x48,0,0,0x19,0x58,0x40,0,0,0};
    CHECK(dbt_avrcp_receive(&a.p,1701,empty,sizeof(empty),3)==DBT_INVALID && a.p.continuation_active);
    uint8_t denied[]={0x12,0x11,0x0e,8,0x48,0,0,0x19,0x58,0x40,0,0,1,0x20}; unsigned calls=a.calls;
    CHECK(dbt_avrcp_receive(&a.p,1701,denied,sizeof(denied),4)==DBT_OK);
    CHECK(a.calls==calls+2 && !a.p.continuation_active && !a.p.local_abandoned && !a.t.local_pending); return 0;
}
EXPORT int test_browse_generated_and_wrap(void) {
    static peer a; static dbt_avrcp_browse_t ba; uint32_t x=0x8badf00d;
    for(unsigned i=0;i<10000;i++) {
        CHECK(browse_setup(&a,&ba,1801)); uint8_t bytes[64]; x=x*1664525u+1013904223u; unsigned n=3u+x%62u;
        bytes[0]=0; bytes[1]=0x11; bytes[2]=0x0e;
        for(unsigned j=3;j<n;j++) { x=x*1664525u+1013904223u; bytes[j]=(uint8_t)(x>>24); }
        if(n>=6) { static const uint8_t ids[]={0x70,0x71,0x72,0x73,0x75,0x80,0xff};
            bytes[3]=ids[x%7]; bytes[4]=0; bytes[5]=(uint8_t)(n-6); }
        (void)dbt_avrcp_browse_receive(&ba,1801,bytes,(uint16_t)n,1);
        CHECK(a.t.tx_size<=sizeof(a.tx) && a.t.rx_size<=sizeof(a.rx));
    }
    CHECK(browse_setup(&a,&ba,1802)); uint8_t select[]={0,0x11,0x0e,0x70,0,2,0,1};
    CHECK(dbt_avrcp_browse_receive(&ba,1802,select,sizeof(select),0xfffffff0u)==DBT_OK);
    CHECK(dbt_avrcp_browse_wait_ms(&ba,0xfffffff1u)==99);
    CHECK(dbt_avrcp_browse_poll(&ba,0x53)==DBT_OK); CHECK(dbt_avrcp_browse_poll(&ba,0x54)==DBT_INVALID); return 0;
}
