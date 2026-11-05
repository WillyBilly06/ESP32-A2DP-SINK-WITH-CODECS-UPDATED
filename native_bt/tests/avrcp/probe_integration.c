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
static bool close_busy;
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
    if(close_busy)return DBT_BUSY;
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
    /* Hold closes staged and concurrently-open controls, not unrelated PSMs.
     * A stale callback with a reused serial cannot clear the close binding. */
    CHECK(probe_avrcp_restart_hold(true,28)==DBT_OK);
    dbt_channel_t staged={.state=DBT_CH_OPEN,.serial=88,.link_generation=13,.handle=1,.local_cid=66,.psm=0x17};
    probe_avrcp_event(DBT_CH_EVENT_OPEN,&staged);CHECK(s.closing && !s.storage);
    dbt_channel_t stale=staged;stale.link_generation++;probe_avrcp_event(DBT_CH_EVENT_CLOSED,&stale);
    CHECK(s.closing && s.close_generation==13 && s.close_serial==88);
    rt.channels.channels[0]=staged;
    rt.channels.channels[1]=(dbt_channel_t){.state=DBT_CH_CONFIGURING,.serial=89,.link_generation=14,.handle=2,.local_cid=67,.psm=0x17};
    rt.channels.channels[2]=(dbt_channel_t){.state=DBT_CH_OPEN,.serial=90,.link_generation=15,.handle=3,.local_cid=68,.psm=0x19};
    close_busy=true;step(28);CHECK(!probe_avrcp_restart_quiescent() && probe_avrcp_wait_ms(28)==20);
    close_busy=false;step(48);CHECK(rt.channels.channels[0].state==DBT_CH_DISCONNECTING &&
        rt.channels.channels[1].state==DBT_CH_DISCONNECTING && rt.channels.channels[2].state==DBT_CH_OPEN);
    rt.channels.channels[0].state=rt.channels.channels[1].state=DBT_CH_FREE;step(49);
    CHECK(probe_avrcp_restart_quiescent() && rt.channels.channels[2].state==DBT_CH_OPEN);
    uint32_t errors=s.view.error_count;CHECK(probe_avrcp_restart_hold(false,50)==DBT_OK && s.view.error_count==errors);
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

EX int probe_test_wait_backpressure(void) {
    CHECK(!s.storage && !done);
    memset(&s,0,sizeof(s));memset(&rt,0,sizeof(rt));
    rt.ready=true;rt.host.state=DBT_HOST_IDENTIFIED;rt.hooks.allocate=allocate;rt.hooks.release=release;
    link=(dbt_link_t){.used=true,.encrypted=true,.handle=1,.generation=9,.kind=DBT_LINK_CLASSIC};
    channel=(dbt_channel_t){.state=DBT_CH_OPEN,.serial=13,.link_generation=9,.handle=1,.local_cid=64,.remote_cid=65,.psm=0x17,.rx_mtu=48,.tx_mtu=48};
    CHECK(probe_avrcp_init(&rt,set_volume,0,55)==DBT_OK);
    probe_avrcp_event(DBT_CH_EVENT_OPEN,&channel);CHECK(s.bearer.attached);
    s.cap_need=s.meta_need=s.status_need=false;s.reg_wanted=s.reg_sent=0;
    /* Fill real transaction slots and complete their runtime borrows. Each
     * transaction still awaits its peer response; no timer is expired. */
    for(unsigned i=0;i<16;i++){
        CHECK(probe_avrcp_key(0x44,false,100+i)==DBT_OK);step(100+i);CHECK(done);finish();
    }
    CHECK(s.transport.local_pending==0xffffu && !s.transport.tx_used && !s.bearer.active);
    s.cap_need=true;
    uint32_t remaining=dbt_avrcp_bearer_wait_ms(&s.bearer,&s.profile,116);
    CHECK(remaining>0 && remaining<UINT32_MAX);
    CHECK(probe_avrcp_wait_ms(116)==remaining);
    for(unsigned i=0;i<64;i++)step(116);
    CHECK(!done && s.cap_need && s.transport.local_pending==0xffffu);
    CHECK(probe_avrcp_wait_ms(116)==remaining);
    /* A real matching reply releases one label and immediately makes the
     * retained controller operation runnable again. */
    {const uint8_t reply[]={2,0x11,0x0e,9,0x48,0x7c,0x44,0};probe_avrcp_receive(&channel,reply,sizeof(reply));}
    CHECK(s.transport.local_pending==0xfffeu && probe_avrcp_wait_ms(116)==0);
    step(116);CHECK(done && pdu()==0x10 && !s.cap_need);finish();
    probe_avrcp_event(DBT_CH_EVENT_CLOSED,&channel);channel.state=DBT_CH_FREE;step(117);
    CHECK(!s.storage && allocations==releases);
    /* Fifteen real interim notifications hold their labels indefinitely.
     * The remaining label is reserved for ordinary controller commands. */
    channel.state=DBT_CH_OPEN;channel.serial++;probe_avrcp_event(DBT_CH_EVENT_OPEN,&channel);
    CHECK(s.bearer.attached);s.cap_need=s.meta_need=s.status_need=false;s.reg_wanted=s.reg_sent=0;
    for(unsigned i=0;i<15;i++){
        const uint8_t request[]={1,0,0,0,0};uint8_t transaction;
        CHECK(dbt_avrcp_command(&s.profile,0x31,request,sizeof(request),200+i,&transaction)==DBT_OK);
        step(200+i);CHECK(done);finish();
        {const uint8_t interim[]={1,1};feed(transaction,true,15,0x31,interim,sizeof(interim));}
    }
    CHECK(s.transport.local_pending==0x7fffu);
    s.reg_wanted=1;
    CHECK(probe_avrcp_wait_ms(215)==UINT32_MAX);
    s.status_need=true;CHECK(probe_avrcp_wait_ms(215)==0);
    step(215);CHECK(done && pdu()==0x30 && s.status_sent && s.reg_wanted==1);finish();
    CHECK(s.transport.local_pending==0xffffu);
    /* A pending target vendor reply waits for the continuation deadline,
     * while retaining its obligation even with all local labels occupied. */
    step(300);
    {const uint8_t value[]={80};feed(7,false,0,0x50,value,sizeof(value));}
    CHECK(s.work_count==1);s.profile.target_active=true;s.profile.target_deadline=550;
    CHECK(probe_avrcp_wait_ms(300)==250);
    unsigned calls=volume_calls;step(300);CHECK(!done && s.work_count==1 && volume_calls==calls+1);
    CHECK(probe_avrcp_wait_ms(300)==250);
    CHECK(probe_avrcp_wait_ms(550)==0);step(550);
    CHECK(done && label()==7 && pdu()==0x50 && s.work_count==0 && volume_calls==calls+1);finish();
    /* Pass-through replies use the peer label and remain runnable during
     * a target continuation; local-label exhaustion must not block them. */
    s.profile.target_active=true;s.profile.target_deadline=800;
    {const uint8_t request[]={0x90,0x11,0x0e,0,0x48,0x7c,0x41,0};probe_avrcp_receive(&channel,request,sizeof(request));}
    CHECK(s.work_count==1 && probe_avrcp_wait_ms(550)==0);step(550);
    CHECK(done && label()==9 && wire[5]==0x7c && s.profile.target_active);finish();
    probe_avrcp_event(DBT_CH_EVENT_CLOSED,&channel);channel.state=DBT_CH_FREE;step(551);
    CHECK(!s.storage && allocations==releases);
    return 0;
}
