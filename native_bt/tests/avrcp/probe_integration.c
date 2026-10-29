#include "../../esp32_probe/main/probe_avrcp.c"
#include "../../esp32_probe/main/probe_sdp.c"
#include <string.h>
#ifdef _WIN32
#define EX __declspec(dllexport)
#else
#define EX __attribute__((visibility("default")))
#endif
#define CHECK(x) do {if(!(x)) return __LINE__;} while(0)
static dbt_runtime_t rt;
static dbt_channel_t channel;
static dbt_link_t link;
static uint8_t memory[8192],wire[1024];
static unsigned allocations,releases,volume_calls,volume_busy;
static uint16_t wire_n;
static const uint8_t *loan;
static dbt_tx_done_t done;
static void *done_context;
static dbt_service_t service;
static uint8_t *allocate(void *ctx,uint16_t n) { (void)ctx; if(n>sizeof(memory) || allocations!=releases)return 0; allocations++; return memory; }
static void release(void *ctx,uint8_t *p) { (void)ctx; if(p==memory) releases++; }
static dbt_result_t set_volume(void *ctx,uint8_t v) { (void)ctx;(void)v; volume_calls++; if(volume_busy) {volume_busy--;return DBT_BUSY;} return DBT_OK; }
dbt_result_t dbt_channels_register(dbt_channels_t *m,dbt_service_t x) {(void)m; service=x;return DBT_OK;}
const dbt_channel_t *dbt_channels_find(const dbt_channels_t *m,uint16_t h,uint16_t cid) {(void)m;return channel.handle==h && channel.local_cid==cid && channel.state!=DBT_CH_FREE?&channel:0;}
const dbt_link_t *dbt_acl_find(const dbt_acl_t *a,uint16_t h) {(void)a;return link.used && link.handle==h?&link:0;}
dbt_result_t dbt_runtime_send(dbt_runtime_t *r,uint16_t h,uint16_t cid,const uint8_t *p,uint16_t n,dbt_tx_done_t cb,void *ctx) {
    (void)r;(void)h;(void)cid; if(done) return DBT_BUSY; if(n>sizeof(wire))return DBT_INVALID;
    memcpy(wire,p,n);wire_n=n;loan=p;done=cb;done_context=ctx;return DBT_OK;
}
dbt_result_t dbt_channels_close(dbt_channels_t *m,uint16_t h,uint16_t cid,uint32_t now) {
    (void)now;
    for(unsigned i=0;i<DBT_CHANNEL_LIMIT;i++) if(m->channels[i].handle==h && m->channels[i].local_cid==cid && m->channels[i].state!=DBT_CH_FREE) {
        m->channels[i].state=DBT_CH_DISCONNECTING; return DBT_OK;
    }
    if(channel.handle==h && channel.local_cid==cid) {channel.state=DBT_CH_DISCONNECTING; return DBT_OK;}
    return DBT_INVALID;
}
dbt_result_t dbt_channels_open(dbt_channels_t *m,uint16_t h,uint16_t psm,uint16_t rx,uint16_t min,dbt_security_t sec,uint32_t now,uint16_t *cid) {(void)m;(void)h;(void)psm;(void)rx;(void)min;(void)sec;(void)now;(void)cid;return DBT_BUSY;}
dbt_result_t dbt_sdp_bearer_init(dbt_sdp_bearer_t *b,dbt_runtime_t *r,const dbt_sdp_t *d,uint16_t mtu) {(void)b;(void)r;(void)d;(void)mtu;return DBT_OK;}
dbt_result_t dbt_sdp_bearer_receive(dbt_sdp_bearer_t *b,const dbt_channel_t *c,const uint8_t *p,uint16_t n) {(void)b;(void)c;(void)p;(void)n;return DBT_OK;}
void dbt_sdp_bearer_event(dbt_sdp_bearer_t *b,dbt_channel_event_t e,const dbt_channel_t *c) {(void)b;(void)e;(void)c;}
dbt_result_t dbt_sdp_bearer_poll(dbt_sdp_bearer_t *b) {(void)b;return DBT_OK;}
uint32_t dbt_sdp_bearer_wait_ms(const dbt_sdp_bearer_t *b) {(void)b;return UINT32_MAX;}
static void step(unsigned now) {rt.now_ms=now;probe_avrcp_poll(now);}
static void finish(void) {dbt_tx_done_t f=done;void *ctx=done_context;done=0;done_context=0;if(f) f(ctx,DBT_TX_ADMITTED);}
static void feed(uint8_t label,bool response,uint8_t code,uint8_t pdu,const uint8_t *p,unsigned n) {
    uint8_t v[128]={(uint8_t)((label<<4)|(response?2:0)),0x11,0x0e,code,0x48,0,0,0x19,0x58,pdu,0,0,(uint8_t)n};
    if(n)memcpy(v+13,p,n);probe_avrcp_receive(&channel,v,(uint16_t)(13+n));
}
static unsigned label(void){return wire[0]>>4;}
static unsigned pdu(void){return wire[9];}
EX int probe_test_exchange(void) {
    rt.ready=true;rt.host.state=DBT_HOST_IDENTIFIED;rt.hooks.allocate=allocate;rt.hooks.release=release;
    link=(dbt_link_t){.used=true,.encrypted=true,.handle=1,.generation=7,.kind=DBT_LINK_CLASSIC};
    channel=(dbt_channel_t){.state=DBT_CH_OPEN,.serial=11,.link_generation=7,.handle=1,.local_cid=64,.remote_cid=65,.psm=0x17,.rx_mtu=48,.tx_mtu=48};
    CHECK(probe_avrcp_init(&rt,set_volume,0,55)==DBT_OK && service.psm==0x17 && service.security==DBT_SECURITY_ENCRYPTED);
    probe_avrcp_event(DBT_CH_EVENT_OPEN,&channel); CHECK(s.bearer.attached && allocations==1);
    step(1);CHECK(done && pdu()==0x10 && wire[13]==3);unsigned caps=label();finish();
    {uint8_t x[]={3,2,1,2};feed((uint8_t)caps,true,12,0x10,x,sizeof(x));}
    step(2);CHECK(done && pdu()==0x31 && wire[13]==1);unsigned reg1=label();finish();
    {uint8_t x[]={1,1};feed((uint8_t)reg1,true,15,0x31,x,sizeof(x));}
    step(3);CHECK(done && pdu()==0x31 && wire[13]==2);unsigned reg2=label();finish();
    {uint8_t x[]={2,0,0,0,0,0,0,0,1};feed((uint8_t)reg2,true,15,0x31,x,sizeof(x));}
    step(4);CHECK(done && pdu()==0x20);unsigned meta=label();finish();
    {uint8_t x[]={1,0,0,0,1,0,106,0,5,'h','e','l','l','o'};feed((uint8_t)meta,true,12,0x20,x,sizeof(x));}
    step(5);CHECK(done && pdu()==0x30);unsigned status=label();finish();
    {uint8_t x[]={0,0,0,200,0,0,0,9,1};feed((uint8_t)status,true,12,0x30,x,sizeof(x));}
    probe_avrcp_snapshot_t snap;CHECK(probe_avrcp_snapshot(&snap)==DBT_OK && snap.connected && snap.volume==55 &&
        snap.track_length_ms==200 && snap.track_position_ms==9 && !memcmp(snap.title,"hello",6) && snap.playback_status==1);
    /* Source controller registers for local volume. */
    {uint8_t x[]={13,0,0,0,0};feed(7,false,3,0x31,x,sizeof(x));}
    step(6);CHECK(done && label()==7 && pdu()==0x31 && wire[3]==15 && wire[14]==55);finish();
    /* A remote SetAbsoluteVolume must not echo a change notification. */
    {uint8_t x[]={80};feed(8,false,0,0x50,x,sizeof(x));}
    volume_busy=1;step(7);CHECK(!done && volume_calls==1);step(16);CHECK(!done && volume_calls==1);
    step(17);CHECK(done && pdu()==0x50 && wire[3]==9 && wire[13]==80 && volume_calls==2);finish();
    step(18);CHECK(!done && !s.volume_dirty);probe_avrcp_local_volume(81);step(19);
    CHECK(done && label()==7 && pdu()==0x31 && wire[3]==13 && wire[14]==81);finish();
    /* Category-2 target buttons change only on press; unsupported player
     * commands are explicitly NOT IMPLEMENTED. */
    {const uint8_t x[]={0xa0,0x11,0x0e,0,0x48,0x7c,0x41,0};probe_avrcp_receive(&channel,x,sizeof(x));}
    step(20);CHECK(done && label()==10 && wire[3]==9 && wire[6]==0x41 && s.view.volume==82);finish();
    {const uint8_t x[]={0xb0,0x11,0x0e,0,0x48,0x7c,0xc1,0};probe_avrcp_receive(&channel,x,sizeof(x));}
    step(21);CHECK(done && label()==11 && wire[3]==9 && wire[6]==0xc1 && s.view.volume==82);finish();
    {const uint8_t x[]={0xc0,0x11,0x0e,0,0x48,0x7c,0x44,0};probe_avrcp_receive(&channel,x,sizeof(x));}
    step(22);CHECK(done && label()==12 && wire[3]==8 && wire[6]==0x44);finish();
    /* Outstanding notifications do not consume the normal command path. */
    CHECK(probe_avrcp_key(0x44,false,23)==DBT_OK);step(23);CHECK(done && wire[5]==0x7c && wire[6]==0x44);finish();
    /* A stale old-binding callback cannot damage the current session. */
    {dbt_channel_t stale=channel;stale.serial++;const uint8_t x[]={0,0x11,0x0e};probe_avrcp_receive(&stale,x,sizeof(x));CHECK(!s.closing);}
    /* A pinned runtime loan survives closure until returned. */
    CHECK(probe_avrcp_key(0x45,false,24)==DBT_OK);step(24);CHECK(done && s.bearer.active);
    uint8_t saved[1024];memcpy(saved,loan,wire_n);probe_avrcp_event(DBT_CH_EVENT_CLOSED,&channel);
    CHECK(s.storage && releases==0 && !memcmp(saved,loan,wire_n));finish();
    channel.state=DBT_CH_FREE;step(25);CHECK(!s.storage && releases==1);
    /* A simultaneous pair is quiesced before a bounded retry delay. */
    rt.channels.channels[0]=(dbt_channel_t){.state=DBT_CH_CONFIGURING,.handle=1,.local_cid=64,.psm=0x17};
    rt.channels.channels[1]=(dbt_channel_t){.state=DBT_CH_CONFIGURING,.handle=1,.local_cid=65,.psm=0x17};
    step(26);CHECK(s.collision && rt.channels.channels[0].state==DBT_CH_DISCONNECTING &&
        rt.channels.channels[1].state==DBT_CH_DISCONNECTING && probe_avrcp_wait_ms(26)==UINT32_MAX);
    rt.channels.channels[0].state=rt.channels.channels[1].state=DBT_CH_FREE;step(27);
    CHECK(!s.collision && s.next_connect>=127 && s.next_connect<=1027);
    return 0;
}
EX int probe_test_sdp(void) {
    CHECK(probe_sdp_enable_avrcp(true,true)==DBT_OK && probe_sdp_init(&rt)==DBT_OK);
#ifdef DBT_PROBE_AUDIO_OUTPUT
    CHECK(probe_sdp_records()==5 && records[3].handle==0x10002 && records[4].handle==0x10003);
#else
    CHECK(probe_sdp_records()==4 && records[2].handle==0x10002 && records[3].handle==0x10003);
#endif
    CHECK(ct_features[2]==1 && tg_features[2]==2 && avrcp_protocol[9]==0x17 && avrcp_profile[9]==4);
    CHECK(probe_sdp_enable_avrcp(false,false)==DBT_INVALID);
    return 0;
}
