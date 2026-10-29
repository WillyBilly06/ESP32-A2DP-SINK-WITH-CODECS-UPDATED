/* Builds on the existing simulated HCI pair while routing endpoint 0 through
 * the actual probe integration. Endpoint 1 is an independently scripted TG. */
#include "runtime_pair.c"
#include "probe_avrcp.h"
static uint8_t large_pool[8192],target_packet[1024],target_rx[1024],target_tx[1024],target_cont[2048],target_storage[2048];
static bool large_used;
static dbt_avctp_t target_transport;
static dbt_avrcp_t target_profile;
static dbt_avctp_bearer_t target_bearer;
static uint64_t tokens[16], registrations[2];
static uint8_t pdus[16],params[16],target_metadata[600];
static unsigned target_errors,volume_updates;
static uint8_t *probe_allocate(void *ctx,uint16_t n) {
    if(ctx==&peers[0] && n>POOL_SIZE) {if(large_used || n>sizeof(large_pool)) return 0;large_used=true;return large_pool;}
    return allocate(ctx,n);
}
static void probe_release(void *ctx,uint8_t *p) {
    if(p==large_pool) {if(!large_used)target_errors++;large_used=false;return;}release(ctx,p);
}
static dbt_result_t volume(void *ctx,uint8_t v) {(void)ctx;(void)v;volume_updates++;return DBT_OK;}
static void target_message(void *ctx,const dbt_avrcp_message_t *m) {
    (void)ctx;
    if(m->response || m->invalid_pid || m->aborted) {target_errors++;return;}
    tokens[m->label]=m->token;pdus[m->label]=m->pdu;params[m->label]=m->size?m->parameters[0]:0;
}
static void paired_event(void *ctx,dbt_channel_event_t ev,const dbt_channel_t *c,dbt_channel_reason_t reason,uint16_t result) {
    endpoint *e=ctx; (void)reason;(void)result;
    if(c->psm!=0x17)return;
    if(e==&peers[0]) {probe_avrcp_event(ev,c);return;}
    if(ev==DBT_CH_EVENT_OPEN) {
        dbt_result_t r=dbt_avrcp_init(&target_profile,&target_transport,target_message,0,5000);
        if(r==DBT_OK)r=dbt_avctp_init(&target_transport,dbt_avrcp_hooks(&target_profile),c->serial,c->rx_mtu,c->tx_mtu,false,
            target_rx,sizeof(target_rx),target_tx,sizeof(target_tx),target_packet,3000,5000);
        if(r==DBT_OK)r=dbt_avrcp_set_target_buffer(&target_profile,target_storage,sizeof(target_storage),10000);
        if(r==DBT_OK)r=dbt_avrcp_set_continuation_buffer(&target_profile,target_cont,sizeof(target_cont));
        if(r==DBT_OK)r=dbt_avctp_bearer_init(&target_bearer,&e->runtime,&target_transport,c);
        if(r!=DBT_OK)target_errors++;
    } else dbt_avctp_bearer_event(&target_bearer,ev,c);
}
static void paired_data(void *ctx,const dbt_channel_t *c,const uint8_t *p,uint16_t n) {
    endpoint *e=ctx;if(e==&peers[0])probe_avrcp_receive(c,p,n);
    else if(dbt_avrcp_bearer_receive(&target_bearer,&target_profile,c,p,n)!=DBT_OK)target_errors++;
}
static void target_poll(void) {
    if(!target_bearer.attached)return;
    for(unsigned i=0;i<16;i++) if(tokens[i]) {
        uint8_t b[9];const uint8_t *p=b;uint16_t n;uint8_t code=12;
        if(pdus[i]==0x10) {b[0]=3;b[1]=2;b[2]=1;b[3]=2;n=4;}
        else if(pdus[i]==0x31) {code=15;b[0]=params[i];if(params[i]==1) {b[1]=1;n=2;}else {memset(b+1,0,8);b[8]=1;n=9;}}
        else if(pdus[i]==0x20) {p=target_metadata;n=sizeof(target_metadata);}
        else if(pdus[i]==0x30) {memset(b,0,9);b[3]=100;b[7]=4;b[8]=1;n=9;}
        else {target_errors++;tokens[i]=0;break;}
        dbt_result_t r=dbt_avrcp_reply(&target_profile,tokens[i],code,p,n,tick);
        if(r==DBT_OK) {if(pdus[i]==0x31 && params[i]>=1 && params[i]<=2)registrations[params[i]-1]=tokens[i];tokens[i]=0;}
        else if(r!=DBT_BUSY)target_errors++;
        break;
    }
    if(dbt_avrcp_bearer_poll(&target_bearer,&target_profile,tick)!=DBT_OK)target_errors++;
}
static void paired_pump(unsigned count) {
    for(unsigned i=0;i<count;i++) {
        probe_avrcp_poll(tick);target_poll();deliver(&peers[0],&peers[1]);deliver(&peers[1],&peers[0]);tick++;
        dbt_runtime_poll(&peers[0].runtime,tick,8,4);dbt_runtime_poll(&peers[1].runtime,tick,8,4);
    }
}
EXPORT int probe_runtime_exchange(void) {
    tick=0;int r=initialize(&peers[0]);if(r)return r;r=initialize(&peers[1]);if(r)return r;
    for(unsigned i=0;i<2;i++) {
        peers[i].runtime.hooks.channels.event=paired_event;peers[i].runtime.hooks.channels.data=paired_data;
        r=link_add(&peers[i],1,false);if(r)return r;
        dbt_link_t *l=&peers[i].runtime.acl.links[0];CHECK(l->used && l->handle==1);l->encrypted=true;
    }
    peers[0].runtime.hooks.allocate=probe_allocate;peers[0].runtime.hooks.release=probe_release;
    CHECK(probe_avrcp_init(&peers[0].runtime,volume,0,55)==DBT_OK);
    CHECK(dbt_channels_register(&peers[1].runtime.channels,(dbt_service_t){0x17,48,48,DBT_SECURITY_ENCRYPTED})==DBT_OK);
    memset(target_metadata,'x',sizeof(target_metadata));target_metadata[0]=1;
    target_metadata[1]=target_metadata[2]=target_metadata[3]=0;target_metadata[4]=1;
    target_metadata[5]=0;target_metadata[6]=106;target_metadata[7]=2;target_metadata[8]=79;
    paired_pump(1600);
    probe_avrcp_snapshot_t snap;CHECK(probe_avrcp_snapshot(&snap)==DBT_OK && snap.connected && snap.metadata_updates==1);
    CHECK(registrations[0] && registrations[1] && snap.title[0]=='x' && snap.metadata_truncated && snap.track_length_ms==100);
    CHECK(!peers[0].errors && !peers[1].errors && !target_errors && large_used);
    /* An owned TX loan remains pinned under controller-credit stall and is
     * returned on physical-link removal before its memory is released. */
    peers[0].runtime.acl.pools[0].available=0;
    CHECK(probe_avrcp_key(0x44,false,tick)==DBT_OK);probe_avrcp_poll(tick);
    bool found=false;for(unsigned i=0;i<DBT_MAX_LINKS;i++)if(peers[0].runtime.tx[i].active)found=true;
    CHECK(found && large_used);CHECK(disconnect(&peers[0],1)==0);probe_avrcp_poll(tick);
    CHECK(!large_used && probe_avrcp_snapshot(&snap)==DBT_OK && !snap.connected);
    dbt_runtime_abort(&peers[0].runtime,DBT_FAULT_BAD_RESPONSE);dbt_runtime_abort(&peers[1].runtime,DBT_FAULT_BAD_RESPONSE);
    CHECK(!peers[0].borrow_errors && !peers[1].borrow_errors && !target_errors);
    return 0;
}
