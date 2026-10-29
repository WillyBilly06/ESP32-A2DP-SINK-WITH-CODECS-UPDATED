#include "dbt_gap.h"
#include "dbt_att_bearer.h"
#include "dbt_sdp_bearer.h"
#include "dbt_avdtp_bearer.h"
#include "dbt_avdtp_sink.h"
#include "dbt_a2dp_acceptor.h"
#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif
#define CHECK(expression) do { if (!(expression)) return __LINE__; } while (0)
typedef struct {
    dbt_runtime_t runtime;
    uint8_t wire[DBT_MAX_H4_BYTES], received[4096];
    _Alignas(8) uint8_t allocations[8][4096];
    bool used[8], ready, accept;
    uint8_t *borrowed;
    uint16_t wire_size, received_size, cid,last_data_cid;
    unsigned sends, opens, data_calls, fixed_calls, link_events, allocated, released, integrity_errors;
} peer_t;
static peer_t peers[2];
static dbt_gap_t gap;
static uint32_t clock_ms;
static unsigned done_calls;
static dbt_tx_outcome_t last_tx_outcome;
static dbt_command_outcome_t last_command_outcome;
static uint8_t last_status;
static dbt_att_server_t att_server;
static dbt_att_bearer_t att_bearer;
static unsigned att_errors;
static dbt_avdtp_t avdtp_signals[2];
static dbt_avdtp_bearer_t avdtp_bearers[2];
static bool avdtp_enabled;
static unsigned avdtp_errors, avdtp_commands[2], avdtp_responses[2];
static dbt_avdtp_sink_t runtime_sink;
static bool sink_runtime_enabled,sink_job_pending,sink_close_pending;
static uint32_t sink_job_token,sink_media_serial;
static uint16_t sink_media_cid;
static unsigned sink_media_calls;
static dbt_a2dp_acceptor_t acceptor;
static bool acceptor_enabled;
static void zero(void *data, unsigned size) { uint8_t *p=data; for(unsigned i=0;i<size;++i)p[i]=0; }
static void copy(uint8_t *out,const uint8_t *in,unsigned size){for(unsigned i=0;i<size;++i)out[i]=in[i];}
static void w16(uint8_t *p,uint16_t v){p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);}
static bool can_send(void *context){return ((peer_t *)context)->ready;}
static bool send(void *context,uint8_t *bytes,uint16_t size){
    peer_t *peer=context;
    if(!peer->ready || !peer->accept)return false;
    copy(peer->wire,bytes,size);peer->wire_size=size;peer->borrowed=bytes;peer->ready=false;++peer->sends;return true;
}
static uint8_t *allocate(void *context,uint16_t size){
    peer_t *peer=context;if(size>4096)return 0;
    for(unsigned i=0;i<8;++i)if(!peer->used[i]){peer->used[i]=true;++peer->allocated;return peer->allocations[i];}
    return 0;
}
static void release(void *context,uint8_t *bytes){
    peer_t *peer=context;
    for(unsigned i=0;i<8;++i)if(peer->allocations[i]==bytes){if(!peer->used[i])++peer->integrity_errors;peer->used[i]=false;++peer->released;return;}
    ++peer->integrity_errors;
}
static void link_change(void *context,const dbt_link_change_t *change){(void)change;++((peer_t *)context)->link_events;}
static void channel(void *context,dbt_channel_event_t kind,const dbt_channel_t *snapshot,dbt_channel_reason_t reason,uint16_t result){
    peer_t *peer=context;(void)reason;(void)result;
    if(kind==DBT_CH_EVENT_OPEN){++peer->opens;peer->cid=snapshot->local_cid;}
    if(acceptor_enabled && peer==&peers[1] && snapshot->psm==0x19){
        dbt_result_t status=dbt_a2dp_acceptor_event(&acceptor,kind,snapshot);
        if(status!=DBT_OK && status!=DBT_EMPTY){
            ++avdtp_errors;(void)dbt_a2dp_acceptor_retire(&acceptor,clock_ms);
        }
        return;
    }
    if(avdtp_enabled)dbt_avdtp_bearer_event(&avdtp_bearers[peer==&peers[1]],kind,snapshot);
    if(sink_runtime_enabled && peer==&peers[1] && snapshot->psm==0x19 && snapshot->local_cid!=avdtp_bearers[1].cid){
        if(kind==DBT_CH_EVENT_OPEN){
            sink_media_cid=snapshot->local_cid;sink_media_serial=snapshot->serial;
            if(dbt_avdtp_sink_media_open(&runtime_sink,sink_media_serial,clock_ms)!=DBT_OK)++avdtp_errors;
        }else if(kind==DBT_CH_EVENT_CLOSED || kind==DBT_CH_EVENT_FAILED){
            if(dbt_avdtp_sink_media_closed(&runtime_sink,snapshot->serial)!=DBT_OK)++avdtp_errors;
        }
    }
}
static void data(void *context,const dbt_channel_t *snapshot,const uint8_t *bytes,uint16_t size){
    if(acceptor_enabled && context==&peers[1] && snapshot->psm==0x19){
        dbt_result_t result=dbt_a2dp_acceptor_receive(&acceptor,snapshot,bytes,size);
        if(result!=DBT_OK && result!=DBT_EMPTY)++avdtp_errors;
        return;
    }
    if(sink_runtime_enabled && context==&peers[1] && snapshot->local_cid==sink_media_cid){
        if(dbt_avdtp_sink_media_receive(&runtime_sink,snapshot->serial,bytes,size)!=DBT_OK)++avdtp_errors;
        return;
    }
    if(avdtp_enabled){
        dbt_result_t result=dbt_avdtp_bearer_receive(&avdtp_bearers[context==&peers[1]],snapshot,bytes,size);
        if(result!=DBT_OK && result!=DBT_EMPTY)++avdtp_errors;
        return;
    }
    peer_t *peer=context;++peer->data_calls;peer->last_data_cid=snapshot->local_cid;peer->received_size=size;copy(peer->received,bytes,size);
}
static void fixed(void *context,const dbt_link_t *link,uint16_t cid,const uint8_t *bytes,uint16_t size){
    peer_t *peer=context;(void)link;(void)cid;++peer->fixed_calls;peer->received_size=size;copy(peer->received,bytes,size);
}
static void tx_done(void *context,dbt_tx_outcome_t outcome){(void)context;++done_calls;last_tx_outcome=outcome;}
static void command_done(void *context,uint16_t opcode,dbt_command_outcome_t outcome,uint8_t status,const uint8_t *returns,uint8_t size){
    (void)context;(void)opcode;(void)returns;(void)size;++done_calls;last_command_outcome=outcome;last_status=status;
}
static dbt_result_t event(peer_t *peer,uint8_t code,const uint8_t *bytes,uint8_t size){
    uint8_t packet[260];packet[0]=4;packet[1]=code;packet[2]=size;copy(packet+3,bytes,size);
    return dbt_runtime_receive(&peer->runtime,packet,(uint16_t)(3u+size));
}
static dbt_result_t complete(peer_t *peer,uint16_t opcode,uint8_t credits,uint8_t status){
    uint8_t bytes[4]={credits,(uint8_t)opcode,(uint8_t)(opcode>>8),status};return event(peer,0x0e,bytes,4);
}
static dbt_result_t command_status(peer_t *peer,uint16_t opcode,uint8_t credits,uint8_t status){
    uint8_t bytes[4]={status,credits,(uint8_t)opcode,(uint8_t)(opcode>>8)};return event(peer,0x0f,bytes,4);
}
static void check_borrow(peer_t *peer){
    if(peer->ready)return;
    for(unsigned i=0;i<peer->wire_size;++i)if(peer->borrowed[i]!=peer->wire[i]){++peer->integrity_errors;break;}
}
static void controller(peer_t *peer){
    if(peer->ready)return;
    check_borrow(peer);
    uint16_t opcode=dbt_read_le16(peer->wire+1);
    if(peer->wire[0]!=1){++peer->integrity_errors;peer->ready=true;return;}
    if(opcode==0x0c35){peer->ready=true;return;} /* No command credit or reply. */
    uint8_t bytes[68]={1,(uint8_t)opcode,(uint8_t)(opcode>>8),0};uint8_t size=4;
    switch(opcode){
    case 0x1001:size=12;bytes[4]=8;bytes[7]=8;break;
    case 0x1002:size=68;break;
    case 0x1003:size=12;bytes[8]=0x40;break;
    case 0x1009:size=10;bytes[4]=0x42;break;
    case 0x1005:size=11;bytes[4]=27;bytes[7]=2;break;
    case 0x2003:size=12;break;
    case 0x2002:size=7;bytes[4]=27;bytes[6]=2;break;
    default:break;
    }
    peer->ready=true;(void)event(peer,0x0e,bytes,size);
}
static int fixture(unsigned index){
    avdtp_enabled=false;
    sink_runtime_enabled=false;
    acceptor_enabled=false;
    peer_t *peer=&peers[index];zero(peer,sizeof(*peer));peer->ready=peer->accept=true;
    dbt_transport_t transport={peer,can_send,send};dbt_runtime_hooks_t hooks={0};
    hooks.context=peer;hooks.allocate=allocate;hooks.release=release;hooks.link=link_change;
    hooks.channels.context=peer;hooks.channels.event=channel;hooks.channels.data=data;hooks.channels.fixed_data=fixed;
    CHECK(dbt_runtime_init(&peer->runtime,transport,hooks,4096,100)==DBT_OK);
    CHECK(dbt_runtime_start(&peer->runtime,clock_ms)==DBT_OK);
    for(unsigned i=0;i<20 && !peer->runtime.ready;++i){controller(peer);dbt_runtime_poll(&peer->runtime,++clock_ms,8,4);}
    CHECK(peer->runtime.ready && peer->sends==8 && peer->ready);
    done_calls=0;return 0;
}
static void add_link(peer_t *peer,uint16_t handle,bool le){
    uint8_t bytes[19]={0};
    if(le){bytes[0]=1;w16(bytes+2,handle);bytes[4]=1;(void)event(peer,0x3e,bytes,19);}
    else{w16(bytes+1,handle);bytes[9]=1;(void)event(peer,3,bytes,11);}
    dbt_runtime_poll(&peer->runtime,++clock_ms,8,0);
}
static void completed_acl(peer_t *peer,uint16_t handle){uint8_t bytes[5]={1};w16(bytes+1,handle);w16(bytes+3,1);(void)event(peer,0x13,bytes,5);}
static void deliver(peer_t *source,peer_t *target){
    if(source->ready)return;
    check_borrow(source);
    if(source->wire[0]==1){controller(source);return;}
    uint8_t wire[DBT_MAX_H4_BYTES];copy(wire,source->wire,source->wire_size);
    uint16_t handle=dbt_read_le16(wire+1)&0xfff;
    if((wire[2]&0x30)==0)wire[2]|=0x20;
    (void)dbt_runtime_receive(&target->runtime,wire,source->wire_size);
    source->ready=true;completed_acl(source,handle);
}
static void pump(unsigned rounds){
    for(unsigned i=0;i<rounds;++i){
        deliver(&peers[0],&peers[1]);deliver(&peers[1],&peers[0]);++clock_ms;
        dbt_runtime_poll(&peers[0].runtime,clock_ms,8,4);dbt_runtime_poll(&peers[1].runtime,clock_ms,8,4);
    }
}
static int le_runtime_fixture(void){
    clock_ms=0;int result=fixture(0);if(result)return result;result=fixture(1);if(result)return result;
    for(unsigned i=0;i<2;++i){add_link(&peers[i],1,false);add_link(&peers[i],2,true);
        CHECK(dbt_channels_register_fixed(&peers[i].runtime.channels,DBT_LINK_LE,4)==DBT_OK);}
    return 0;
}
static int le_runtime_open(const dbt_le_service_t *local,const dbt_le_service_t *remote,uint16_t *cid,uint16_t *peer_cid){
    CHECK(dbt_channels_register_le(&peers[1].runtime.channels,remote,1)==DBT_OK);
    CHECK(dbt_channels_open_le(&peers[0].runtime.channels,2,local,clock_ms,cid)==DBT_OK);pump(50);
    const dbt_channel_t *c=dbt_channels_find(&peers[0].runtime.channels,2,*cid);
    CHECK(c && c->le && c->state==DBT_CH_OPEN);*peer_cid=c->remote_cid;
    c=dbt_channels_find(&peers[1].runtime.channels,2,*peer_cid);CHECK(c && c->le && c->state==DBT_CH_OPEN);return 0;
}
static dbt_result_t inject_le_credit(peer_t *peer,uint16_t source_cid,uint16_t credits,uint8_t id){
    uint8_t packet[17]={2,2,0x20,12,0,8,0,5,0,0x16,0,4,0,0,0,0,0};
    packet[10]=id;w16(packet+13,source_cid);w16(packet+15,credits);return dbt_runtime_receive(&peer->runtime,packet,sizeof(packet));
}
static int le_clean(void){
    for(unsigned i=0;i<2;++i){dbt_runtime_abort(&peers[i].runtime,DBT_FAULT_BAD_RESPONSE);
        CHECK(peers[i].allocated==peers[i].released && !peers[i].integrity_errors);}
    return 0;
}
EXPORT int test_runtime_le_coc_segmented_bidirectional_and_stalls(void){
    int result=le_runtime_fixture();if(result)return result;
    const dbt_le_service_t a={128,{1200,43,3},DBT_SECURITY_NONE,0},b={128,{1000,75,2},DBT_SECURITY_NONE,0};
    uint16_t cid=0,peer_cid=0;result=le_runtime_open(&a,&b,&cid,&peer_cid);if(result)return result;
    uint8_t audio[1000],reverse[900];for(unsigned i=0;i<1000;++i)audio[i]=(uint8_t)(i*71u);for(unsigned i=0;i<900;++i)reverse[i]=(uint8_t)(i*17u);
    CHECK(dbt_runtime_send(&peers[0].runtime,2,cid,audio,sizeof(audio),tx_done,0)==DBT_OK);pump(50);
    CHECK(!done_calls && !peers[1].data_calls && dbt_runtime_wait_ms(&peers[0].runtime,clock_ms)>0);
    CHECK(dbt_channels_grant_le(&peers[1].runtime.channels,2,peer_cid,20)==DBT_OK);pump(150);
    CHECK(done_calls==1 && last_tx_outcome==DBT_TX_ADMITTED && peers[1].data_calls==1 && peers[1].received_size==sizeof(audio));
    for(unsigned i=0;i<sizeof(audio);++i)CHECK(peers[1].received[i]==audio[i]);
    CHECK(dbt_runtime_send(&peers[1].runtime,2,peer_cid,reverse,sizeof(reverse),tx_done,0)==DBT_OK);pump(50);
    CHECK(done_calls==1 && !peers[0].data_calls);
    CHECK(dbt_channels_grant_le(&peers[0].runtime.channels,2,cid,30)==DBT_OK);pump(150);
    CHECK(done_calls==2 && peers[0].data_calls==1 && peers[0].received_size==sizeof(reverse));
    for(unsigned i=0;i<sizeof(reverse);++i)CHECK(peers[0].received[i]==reverse[i]);
    CHECK(dbt_runtime_send(&peers[0].runtime,2,cid,0,0,tx_done,0)==DBT_OK);pump(20);
    CHECK(done_calls==3 && peers[1].data_calls==2 && !peers[1].received_size);
    CHECK(dbt_channels_close(&peers[0].runtime.channels,2,cid,clock_ms)==DBT_OK);pump(30);
    CHECK(!dbt_channels_find(&peers[0].runtime.channels,2,cid) && !dbt_channels_find(&peers[1].runtime.channels,2,peer_cid));
    return le_clean();
}
EXPORT int test_runtime_le_coc_classic_fairness_and_fault_isolation(void){
    int result=le_runtime_fixture();if(result)return result;
    dbt_service_t classic={0x19,1200,48,DBT_SECURITY_NONE};uint16_t classic_cid=0;
    CHECK(dbt_channels_register(&peers[1].runtime.channels,classic)==DBT_OK);
    CHECK(dbt_channels_open(&peers[0].runtime.channels,1,0x19,1200,48,DBT_SECURITY_NONE,clock_ms,&classic_cid)==DBT_OK);pump(50);
    const dbt_le_service_t selection={128,{1000,23,1},DBT_SECURITY_NONE,0};uint16_t cid=0,peer_cid=0;
    result=le_runtime_open(&selection,&selection,&cid,&peer_cid);if(result)return result;
    uint8_t bytes[1000];for(unsigned i=0;i<1000;++i)bytes[i]=(uint8_t)i;
    CHECK(dbt_runtime_send(&peers[0].runtime,2,cid,bytes,200,tx_done,0)==DBT_OK);
    CHECK(dbt_runtime_send(&peers[0].runtime,1,classic_cid,bytes,1000,tx_done,0)==DBT_OK);pump(150);
    CHECK(done_calls==1 && peers[1].data_calls==1 && peers[1].received_size==1000);
    CHECK(inject_le_credit(&peers[0],peer_cid,65535,70)==DBT_OK && inject_le_credit(&peers[0],peer_cid,1,71)==DBT_OK);pump(50);
    CHECK(done_calls==2 && last_tx_outcome==DBT_TX_ABORTED && !dbt_channels_find(&peers[0].runtime.channels,2,cid));
    CHECK(!dbt_channels_find(&peers[1].runtime.channels,2,peer_cid));
    CHECK(dbt_channels_find(&peers[0].runtime.channels,1,classic_cid)->state==DBT_CH_OPEN);
    CHECK(dbt_runtime_send(&peers[0].runtime,1,classic_cid,bytes,40,tx_done,0)==DBT_OK);
    CHECK(dbt_runtime_send_fixed(&peers[0].runtime,2,4,bytes,40,tx_done,0)==DBT_OK);pump(50);
    CHECK(done_calls==4 && peers[1].data_calls==2 && peers[1].fixed_calls==1 && peers[0].runtime.ready && peers[1].runtime.ready);
    return le_clean();
}
EXPORT int test_runtime_le_coc_simultaneous_opens_and_cid_direction(void){
    int result=le_runtime_fixture();if(result)return result;
    const dbt_le_service_t s={128,{300,41,1},DBT_SECURITY_NONE,0};uint16_t cid[2]={0};
    for(unsigned i=0;i<2;++i){
        CHECK(dbt_channels_register_le(&peers[i].runtime.channels,&s,1)==DBT_OK);
        CHECK(dbt_channels_open_le(&peers[i].runtime.channels,2,&s,clock_ms,&cid[i])==DBT_OK);
    }
    pump(60);uint16_t receiver[2];
    for(unsigned i=0;i<2;++i){const dbt_channel_t *c=dbt_channels_find(&peers[i].runtime.channels,2,cid[i]);
        CHECK(c && c->state==DBT_CH_OPEN && c->remote_cid!=c->local_cid && peers[i].opens==2);receiver[i]=c->remote_cid;}
    uint8_t payload[200];for(unsigned i=0;i<sizeof(payload);++i)payload[i]=(uint8_t)(i*29u);
    for(unsigned i=0;i<2;++i)CHECK(dbt_runtime_send(&peers[i].runtime,2,cid[i],payload,sizeof(payload),tx_done,0)==DBT_OK);
    pump(50);CHECK(!done_calls);
    for(unsigned i=0;i<2;++i)CHECK(dbt_channels_grant_le(&peers[1u-i].runtime.channels,2,receiver[i],10)==DBT_OK);
    pump(100);CHECK(done_calls==2 && peers[0].data_calls==1 && peers[1].data_calls==1);
    for(unsigned i=0;i<2;++i){CHECK(peers[i].received_size==200);for(unsigned n=0;n<200;++n)CHECK(peers[i].received[n]==payload[n]);}
    return le_clean();
}
EXPORT int test_runtime_le_coc_abort_and_generation_cleanup(void){
    int result=le_runtime_fixture();if(result)return result;
    const dbt_le_service_t s={128,{1000,23,1},DBT_SECURITY_NONE,0};uint16_t cid=0,peer_cid=0;
    result=le_runtime_open(&s,&s,&cid,&peer_cid);if(result)return result;
    uint8_t payload[500];zero(payload,sizeof(payload));
    CHECK(dbt_runtime_send(&peers[0].runtime,2,cid,payload,sizeof(payload),tx_done,0)==DBT_OK);pump(40);
    CHECK(!done_calls && peers[1].allocated>peers[1].released);
    dbt_runtime_abort(&peers[1].runtime,DBT_FAULT_RX_OVERRUN);CHECK(peers[1].allocated==peers[1].released);
    uint8_t removed[]={0,2,0,0x13};CHECK(event(&peers[0],5,removed,sizeof(removed))==DBT_OK);
    dbt_runtime_poll(&peers[0].runtime,++clock_ms,8,0);
    CHECK(done_calls==1 && last_tx_outcome==DBT_TX_LINK_LOST && peers[0].allocated==peers[0].released);
    add_link(&peers[0],2,true);CHECK(!dbt_channels_find(&peers[0].runtime.channels,2,cid));
    uint8_t raw[11]={2,2,0x20,6,0,2,0,0,0,0,0};w16(raw+7,cid);
    CHECK(dbt_runtime_receive(&peers[0].runtime,raw,sizeof(raw))==DBT_OK);dbt_runtime_poll(&peers[0].runtime,++clock_ms,8,0);
    CHECK(peers[0].runtime.ready && !peers[0].data_calls);return le_clean();
}
static void avdtp_command_received(void *context,const dbt_avdtp_message_t *message){
    peer_t *peer=context;unsigned index=peer==&peers[1];++avdtp_commands[index];
    peer->received_size=message->size;copy(peer->received,message->data,message->size);
    if(dbt_avdtp_respond(&avdtp_signals[index],message->label,message->signal,DBT_AVDTP_ACCEPT,
                         message->data,message->size,clock_ms)!=DBT_OK)++avdtp_errors;
}
static void avdtp_response_received(void *context,dbt_avdtp_outcome_t outcome,
                                   const dbt_avdtp_message_t *message){
    peer_t *peer=context;unsigned index=peer==&peers[1];++avdtp_responses[index];
    if(outcome==DBT_AVDTP_RESPONSE && message){
        peer->received_size=message->size;copy(peer->received,message->data,message->size);
    }
}
static int avdtp_fixture_with_mtu(uint16_t mtu){
    clock_ms=0;int result=fixture(0);if(result)return result;result=fixture(1);if(result)return result;
    avdtp_errors=0;zero(avdtp_commands,sizeof(avdtp_commands));zero(avdtp_responses,sizeof(avdtp_responses));
    for(unsigned i=0;i<2;++i){add_link(&peers[i],1,false);add_link(&peers[i],2,true);
        CHECK(dbt_channels_register_fixed(&peers[i].runtime.channels,DBT_LINK_LE,4)==DBT_OK);}
    dbt_service_t service={0x19,mtu,48,DBT_SECURITY_NONE};
    CHECK(dbt_channels_register(&peers[1].runtime.channels,service)==DBT_OK);
    uint16_t cid=0;
    CHECK(dbt_channels_open(&peers[0].runtime.channels,1,0x19,mtu,48,DBT_SECURITY_NONE,clock_ms,&cid)==DBT_OK);
    pump(70);
    for(unsigned i=0;i<2;++i){
        peer_t *peer=&peers[i];
        const dbt_channel_t *current=dbt_channels_find(&peer->runtime.channels,1,peer->cid);
        CHECK(current && current->state==DBT_CH_OPEN);
        dbt_avdtp_hooks_t hooks={peer,allocate,release,avdtp_command_received,avdtp_response_received,0,0};
        CHECK(dbt_avdtp_init(&avdtp_signals[i],hooks,current->serial,current->rx_mtu,current->tx_mtu,1024,1000,2000)==DBT_OK);
        CHECK(dbt_avdtp_bearer_init(&avdtp_bearers[i],&peer->runtime,&avdtp_signals[i],current)==DBT_OK);
    }
    avdtp_enabled=true;return 0;
}
static int avdtp_fixture(void){return avdtp_fixture_with_mtu(48);}
EXPORT int test_avdtp_runtime_fragmentation_credit_stall_and_ble(void){
    int result=avdtp_fixture();if(result)return result;
    uint8_t body[193],label=0;for(unsigned i=0;i<sizeof(body);++i)body[i]=(uint8_t)(i*31u+7u);
    CHECK(dbt_avdtp_command(&avdtp_signals[0],DBT_AVDTP_SECURITY_CONTROL,body,sizeof(body),clock_ms,&label)==DBT_OK);
    CHECK(dbt_avdtp_bearer_poll(&avdtp_bearers[0],clock_ms)==DBT_OK && avdtp_bearers[0].active);
    CHECK(dbt_avdtp_bearer_clear(&avdtp_bearers[0])==DBT_BUSY);
    CHECK(dbt_avdtp_bearer_wait_ms(&avdtp_bearers[0],clock_ms)==1000);
    peers[0].runtime.acl.pools[0].available=0;
    CHECK(dbt_runtime_send_fixed(&peers[0].runtime,2,4,body,100,tx_done,0)==DBT_OK);
    pump(30);
    CHECK(peers[1].fixed_calls==1 && !avdtp_commands[1] && avdtp_signals[0].pinned);
    peers[0].runtime.acl.pools[0].available=2;
    for(unsigned i=0;i<100;++i){
        CHECK(dbt_avdtp_bearer_poll(&avdtp_bearers[0],clock_ms)==DBT_OK);
        CHECK(dbt_avdtp_bearer_poll(&avdtp_bearers[1],clock_ms)==DBT_OK);
        pump(1);
    }
    CHECK(avdtp_commands[1]==1 && avdtp_responses[0]==1 && !avdtp_errors);
    CHECK(peers[0].received_size==sizeof(body));
    for(unsigned i=0;i<sizeof(body);++i)CHECK(peers[0].received[i]==body[i]);
    CHECK(dbt_avdtp_bearer_clear(&avdtp_bearers[0])==DBT_OK);
    CHECK(dbt_avdtp_bearer_clear(&avdtp_bearers[1])==DBT_OK);
    for(unsigned i=0;i<2;++i)CHECK(peers[i].allocated==peers[i].released && !peers[i].integrity_errors);
    avdtp_enabled=false;return 0;
}
EXPORT int test_avdtp_runtime_disconnect_and_stale_channel(void){
    int result=avdtp_fixture();if(result)return result;
    uint8_t body[193]={0xa5},label=0;
    CHECK(dbt_avdtp_command(&avdtp_signals[0],DBT_AVDTP_SECURITY_CONTROL,body,sizeof(body),clock_ms,&label)==DBT_OK);
    CHECK(dbt_avdtp_bearer_poll(&avdtp_bearers[0],clock_ms)==DBT_OK);
    dbt_runtime_poll(&peers[0].runtime,++clock_ms,8,1);
    CHECK(!peers[0].ready && avdtp_signals[0].pinned && avdtp_bearers[0].active);
    dbt_channel_t old;
    copy((uint8_t *)&old,(const uint8_t *)dbt_channels_find(&peers[0].runtime.channels,1,peers[0].cid),sizeof(old));
    uint8_t disconnected[]={0,1,0,0x13};
    CHECK(event(&peers[0],5,disconnected,sizeof(disconnected))==DBT_OK);
    dbt_runtime_poll(&peers[0].runtime,++clock_ms,8,0);check_borrow(&peers[0]);
    CHECK(!avdtp_bearers[0].attached && !avdtp_signals[0].binding && avdtp_responses[0]==1);
    CHECK(peers[0].allocated==peers[0].released && !peers[0].integrity_errors);
    uint8_t stale[]={0,1};
    CHECK(dbt_avdtp_bearer_receive(&avdtp_bearers[0],&old,stale,sizeof(stale))==DBT_INVALID);
    CHECK(dbt_avdtp_bearer_clear(&avdtp_bearers[1])==DBT_OK);
    avdtp_enabled=false;return 0;
}
EXPORT int test_avdtp_runtime_reconfigured_mtu_fails_closed(void){
    int result=avdtp_fixture();if(result)return result;
    dbt_channel_t *live=0;
    for(unsigned i=0;i<DBT_CHANNEL_LIMIT;++i)
        if(peers[0].runtime.channels.channels[i].local_cid==peers[0].cid)
            live=&peers[0].runtime.channels.channels[i];
    CHECK(live);live->tx_mtu=64;
    CHECK(dbt_avdtp_bearer_poll(&avdtp_bearers[0],clock_ms)==DBT_INVALID);
    CHECK(!avdtp_bearers[0].attached && !avdtp_signals[0].binding);
    CHECK(dbt_avdtp_bearer_clear(&avdtp_bearers[1])==DBT_OK);
    avdtp_enabled=false;return 0;
}
static dbt_result_t sink_work(void *context,const dbt_sink_job_t *job){
    (void)context;if(sink_job_pending)return DBT_BUSY;
    sink_job_pending=true;sink_job_token=job->token;return DBT_OK;
}
static void sink_event(void *context,dbt_sink_event_t kind,dbt_stream_state_t state){
    (void)context;(void)state;
    if(kind==DBT_SINK_CLOSE_MEDIA)sink_close_pending=true;
    if(kind==DBT_SINK_FAULT)++avdtp_errors;
}
static dbt_result_t sink_media(void *context,const dbt_avdtp_media_t *media){
    (void)context;
    if(media->size!=181 || media->sequence!=0x1234 || media->payload[0]!=1)return DBT_INVALID;
    for(unsigned i=1;i<media->size;++i)if(media->payload[i]!=(uint8_t)(i*31u))return DBT_INVALID;
    ++sink_media_calls;return DBT_OK;
}
static bool sink_steps(unsigned rounds,bool worker_ready){
    for(unsigned i=0;i<rounds;++i){
        pump(1);
        if(dbt_avdtp_sink_poll(&runtime_sink,clock_ms)!=DBT_OK)return false;
        if(sink_job_pending && worker_ready){
            sink_job_pending=false;
            if(dbt_avdtp_sink_complete(&runtime_sink,runtime_sink.binding,sink_job_token,
                                       DBT_SINK_WORK_READY,clock_ms)!=DBT_OK)return false;
        }
        if(dbt_avdtp_bearer_poll(&avdtp_bearers[0],clock_ms)!=DBT_OK ||
           dbt_avdtp_bearer_poll(&avdtp_bearers[1],clock_ms)!=DBT_OK)return false;
        if(sink_close_pending){
            sink_close_pending=false;
            if(dbt_channels_close(&peers[1].runtime.channels,1,sink_media_cid,clock_ms)!=DBT_OK)return false;
        }
    }
    return true;
}
EXPORT int test_avdtp_sink_paired_runtime_media_channels_and_ble(void){
    int result=avdtp_fixture_with_mtu(335);if(result)return result;
    CHECK(dbt_avdtp_bearer_clear(&avdtp_bearers[1])==DBT_OK);
    static const uint8_t caps[]={1,0,7,6,0,0,0xff,0xff,2,53,8,0};
    static const dbt_avdtp_endpoint_t endpoint={caps,sizeof(caps),1,dbt_a2dp_sbc_validate};
    dbt_avdtp_sink_hooks_t hooks={&peers[1],allocate,release,sink_work,sink_event,sink_media};
    const dbt_channel_t *signaling=dbt_channels_find(&peers[1].runtime.channels,1,peers[1].cid);
    CHECK(signaling);
    CHECK(dbt_avdtp_sink_init(&runtime_sink,hooks,&endpoint,1,signaling->serial,signaling->rx_mtu,signaling->tx_mtu,128,5000,1000,2000)==DBT_OK);
    CHECK(dbt_avdtp_bearer_init(&avdtp_bearers[1],&peers[1].runtime,&runtime_sink.signal,signaling)==DBT_OK);
    sink_runtime_enabled=true;sink_job_pending=sink_close_pending=false;sink_media_calls=0;sink_media_cid=0;
    uint8_t configuration[]={4,8,1,0,7,6,0,0,0x21,0x15,2,53,8,0},label=0,seid=4;
    CHECK(dbt_avdtp_command(&avdtp_signals[0],3,configuration,sizeof(configuration),clock_ms,&label)==DBT_OK);
    CHECK(sink_steps(20,true) && runtime_sink.state==DBT_STREAM_CONFIGURED && avdtp_responses[0]==1);
    CHECK(dbt_avdtp_command(&avdtp_signals[0],6,&seid,1,clock_ms,&label)==DBT_OK);
    CHECK(sink_steps(20,true) && runtime_sink.state==DBT_STREAM_OPEN && avdtp_responses[0]==2);
    uint16_t remote_media_cid=0;
    CHECK(dbt_channels_open(&peers[0].runtime.channels,1,0x19,335,48,DBT_SECURITY_NONE,clock_ms,&remote_media_cid)==DBT_OK);
    CHECK(sink_steps(40,true) && runtime_sink.media_binding && sink_media_cid);
    CHECK(dbt_avdtp_command(&avdtp_signals[0],7,&seid,1,clock_ms,&label)==DBT_OK);
    CHECK(sink_steps(10,false) && sink_job_pending && runtime_sink.state==DBT_STREAM_OPEN);
    uint8_t ble[100];zero(ble,sizeof(ble));ble[0]=0x81;unsigned fixed_before=peers[1].fixed_calls;
    peers[1].runtime.acl.pools[0].available=0;
    CHECK(dbt_runtime_send_fixed(&peers[0].runtime,2,4,ble,sizeof(ble),tx_done,0)==DBT_OK);
    CHECK(sink_steps(20,true) && peers[1].fixed_calls==fixed_before+1 && runtime_sink.state==DBT_STREAM_OPEN);
    peers[1].runtime.acl.pools[0].available=2;
    CHECK(sink_steps(20,true) && runtime_sink.state==DBT_STREAM_STREAMING && avdtp_responses[0]==3);
    uint8_t rtp[193];zero(rtp,sizeof(rtp));rtp[0]=0x80;rtp[1]=96;rtp[2]=0x12;rtp[3]=0x34;rtp[7]=rtp[11]=rtp[12]=1;
    for(unsigned i=13;i<sizeof(rtp);++i)rtp[i]=(uint8_t)((i-12u)*31u);
    CHECK(dbt_runtime_send(&peers[0].runtime,1,remote_media_cid,rtp,sizeof(rtp),tx_done,0)==DBT_OK);
    CHECK(sink_steps(30,true) && sink_media_calls==1 && runtime_sink.media_packets==1);
    CHECK(dbt_avdtp_command(&avdtp_signals[0],9,&seid,1,clock_ms,&label)==DBT_OK);
    CHECK(sink_steps(20,true) && runtime_sink.state==DBT_STREAM_OPEN);
    CHECK(dbt_avdtp_command(&avdtp_signals[0],8,&seid,1,clock_ms,&label)==DBT_OK);
    CHECK(sink_steps(30,true) && runtime_sink.state==DBT_STREAM_IDLE && !avdtp_errors);
    CHECK(dbt_avdtp_bearer_clear(&avdtp_bearers[0])==DBT_OK);
    CHECK(dbt_avdtp_bearer_clear(&avdtp_bearers[1])==DBT_OK);
    CHECK(dbt_avdtp_sink_shutdown(&runtime_sink,clock_ms)==DBT_OK);
    for(unsigned i=0;i<2;++i)CHECK(peers[i].allocated==peers[i].released && !peers[i].integrity_errors);
    sink_runtime_enabled=avdtp_enabled=false;return 0;
}
static bool accept_profile(void *context,const dbt_link_t *link,uint16_t psm){
    return context!=&peers[1] || psm!=0x19 || dbt_a2dp_acceptor_accept(&acceptor,link);
}
static int acceptor_fixture(void){
    clock_ms=0;int result=fixture(0);if(result)return result;result=fixture(1);if(result)return result;
    avdtp_errors=0;zero(avdtp_commands,sizeof(avdtp_commands));zero(avdtp_responses,sizeof(avdtp_responses));
    for(unsigned i=0;i<2;++i){
        add_link(&peers[i],1,false);add_link(&peers[i],2,true);
        uint8_t encrypted[]={0,1,0,1};CHECK(event(&peers[i],8,encrypted,sizeof(encrypted))==DBT_OK);
        dbt_runtime_poll(&peers[i].runtime,++clock_ms,8,0);
        CHECK(dbt_channels_register_fixed(&peers[i].runtime.channels,DBT_LINK_LE,4)==DBT_OK);
    }
    static const uint8_t caps[]={1,0,7,6,0,0,0xff,0xff,2,53,8,0};
    static const dbt_avdtp_endpoint_t endpoint={caps,sizeof(caps),1,dbt_a2dp_sbc_validate};
    dbt_avdtp_sink_hooks_t hooks={&peers[1],allocate,release,sink_work,sink_event,sink_media};
    CHECK(dbt_a2dp_acceptor_init(&acceptor,&peers[1].runtime,hooks,&endpoint,1,672,128,5000,1000,2000)==DBT_OK);
    acceptor_enabled=true;sink_job_pending=false;sink_close_pending=false;sink_media_calls=0;
    peers[1].runtime.hooks.channels.accept=accept_profile;
    uint16_t cid=0;CHECK(dbt_channels_open(&peers[0].runtime.channels,1,0x19,672,335,DBT_SECURITY_ENCRYPTED,clock_ms,&cid)==DBT_OK);
    pump(50);CHECK(acceptor.active && acceptor.signaling.serial && !avdtp_errors);
    const dbt_channel_t *channel=dbt_channels_find(&peers[0].runtime.channels,1,cid);CHECK(channel);
    dbt_avdtp_hooks_t signal_hooks={&peers[0],allocate,release,avdtp_command_received,avdtp_response_received,0,0};
    CHECK(dbt_avdtp_init(&avdtp_signals[0],signal_hooks,channel->serial,channel->rx_mtu,channel->tx_mtu,1024,1000,2000)==DBT_OK);
    CHECK(dbt_avdtp_bearer_init(&avdtp_bearers[0],&peers[0].runtime,&avdtp_signals[0],channel)==DBT_OK);
    avdtp_enabled=true;return 0;
}
static bool acceptor_steps(unsigned rounds,bool worker_ready){
    for(unsigned i=0;i<rounds;++i){
        pump(1);
        if(dbt_a2dp_acceptor_poll(&acceptor,clock_ms)!=DBT_OK)return false;
        if(sink_job_pending && worker_ready){
            sink_job_pending=false;
            if(dbt_a2dp_acceptor_complete(&acceptor,acceptor.sink.binding,sink_job_token,DBT_SINK_WORK_READY,clock_ms)!=DBT_OK)return false;
        }
        if(avdtp_bearers[0].attached && dbt_avdtp_bearer_poll(&avdtp_bearers[0],clock_ms)!=DBT_OK)return false;
    }
    return true;
}
EXPORT int test_a2dp_acceptor_admission_routing_and_retirement(void){
    int result=acceptor_fixture();if(result)return result;uint16_t rejected=0;
    /* A media channel before Open is refused, without replacing signaling. */
    CHECK(dbt_channels_open(&peers[0].runtime.channels,1,0x19,672,335,DBT_SECURITY_ENCRYPTED,clock_ms,&rejected)==DBT_OK);
    CHECK(acceptor_steps(30,true) && !dbt_channels_find(&peers[0].runtime.channels,1,rejected));
    CHECK(acceptor.active && !acceptor.media.serial && acceptor.sink.state==DBT_STREAM_IDLE);
    uint8_t configuration[]={4,8,1,0,7,6,0,0,0x21,0x15,2,53,8,0},label=0,seid=4;
    CHECK(dbt_avdtp_command(&avdtp_signals[0],3,configuration,sizeof(configuration),clock_ms,&label)==DBT_OK);
    CHECK(acceptor_steps(20,true) && acceptor.sink.state==DBT_STREAM_CONFIGURED);
    CHECK(dbt_avdtp_command(&avdtp_signals[0],6,&seid,1,clock_ms,&label)==DBT_OK);
    CHECK(acceptor_steps(20,true) && acceptor.sink.state==DBT_STREAM_OPEN);
    uint16_t cid=0;CHECK(dbt_channels_open(&peers[0].runtime.channels,1,0x19,672,335,DBT_SECURITY_ENCRYPTED,clock_ms,&cid)==DBT_OK);
    CHECK(acceptor_steps(40,true) && acceptor.media.serial);
    CHECK(dbt_channels_open(&peers[0].runtime.channels,1,0x19,672,335,DBT_SECURITY_ENCRYPTED,clock_ms,&rejected)==DBT_OK);
    CHECK(acceptor_steps(30,true) && !dbt_channels_find(&peers[0].runtime.channels,1,rejected));
    CHECK(dbt_avdtp_command(&avdtp_signals[0],7,&seid,1,clock_ms,&label)==DBT_OK);
    CHECK(acceptor_steps(20,true) && acceptor.sink.state==DBT_STREAM_STREAMING);
    uint8_t rtp[193];zero(rtp,sizeof(rtp));rtp[0]=0x80;rtp[1]=96;rtp[2]=0x12;rtp[3]=0x34;rtp[7]=rtp[11]=rtp[12]=1;
    for(unsigned i=13;i<sizeof(rtp);++i)rtp[i]=(uint8_t)((i-12u)*31u);
    CHECK(dbt_runtime_send(&peers[0].runtime,1,cid,rtp,sizeof(rtp),tx_done,0)==DBT_OK);
    CHECK(acceptor_steps(30,true) && sink_media_calls==1 && !avdtp_errors);
    CHECK(dbt_a2dp_acceptor_stop(&acceptor,clock_ms)==DBT_BUSY);
    CHECK(acceptor_steps(60,false) && acceptor.active && acceptor.draining && sink_job_pending && acceptor.sink.retiring);
    CHECK(!dbt_a2dp_acceptor_accept(&acceptor,&peers[1].runtime.acl.links[0]));
    CHECK(acceptor_steps(3,true) && !acceptor.active && dbt_a2dp_acceptor_stop(&acceptor,clock_ms)==DBT_OK);
    for(unsigned i=0;i<2;++i)CHECK(peers[i].allocated==peers[i].released && !peers[i].integrity_errors);
    acceptor_enabled=avdtp_enabled=false;return 0;
}
EXPORT int test_a2dp_acceptor_abrupt_close_and_readmission(void){
    int result=acceptor_fixture();if(result)return result;
    uint8_t configuration[]={4,8,1,0,7,6,0,0,0x21,0x15,2,53,8,0},label=0,seid=4;
    CHECK(dbt_avdtp_command(&avdtp_signals[0],3,configuration,sizeof(configuration),clock_ms,&label)==DBT_OK);
    CHECK(acceptor_steps(20,true) && acceptor.sink.state==DBT_STREAM_CONFIGURED);
    CHECK(dbt_avdtp_command(&avdtp_signals[0],6,&seid,1,clock_ms,&label)==DBT_OK);
    CHECK(acceptor_steps(20,true) && acceptor.sink.state==DBT_STREAM_OPEN);
    uint16_t cid=0;CHECK(dbt_channels_open(&peers[0].runtime.channels,1,0x19,672,335,DBT_SECURITY_ENCRYPTED,clock_ms,&cid)==DBT_OK);
    CHECK(acceptor_steps(40,true) && acceptor.media.serial);
    uint32_t old=acceptor.signaling.serial;
    /* Ordinary device disconnect may close media without an AVDTP Close. The
     * error callback requests retirement without mutating channels recursively. */
    CHECK(dbt_channels_close(&peers[0].runtime.channels,1,cid,clock_ms)==DBT_OK);
    CHECK(acceptor_steps(60,false) && avdtp_errors==1 && acceptor.active && acceptor.draining);
    CHECK(acceptor.enabled && sink_job_pending && acceptor.sink.retiring);
    CHECK(!dbt_a2dp_acceptor_accept(&acceptor,&peers[1].runtime.acl.links[0]));
    CHECK(acceptor_steps(10,true) && !acceptor.active && !acceptor.draining && acceptor.enabled);
    CHECK(dbt_a2dp_acceptor_retire(&acceptor,clock_ms)==DBT_OK);
    CHECK(dbt_a2dp_acceptor_accept(&acceptor,&peers[1].runtime.acl.links[0]));
    CHECK(dbt_channels_open(&peers[0].runtime.channels,1,0x19,672,335,DBT_SECURITY_ENCRYPTED,clock_ms,&cid)==DBT_OK);
    CHECK(acceptor_steps(50,true) && acceptor.active && acceptor.signaling.serial!=old);
    CHECK(acceptor.sink.state==DBT_STREAM_IDLE && avdtp_errors==1);
    CHECK(dbt_a2dp_acceptor_retire(&acceptor,clock_ms)==DBT_BUSY);
    CHECK(acceptor_steps(50,true) && !acceptor.active && !acceptor.draining && acceptor.enabled);
    CHECK(dbt_a2dp_acceptor_stop(&acceptor,clock_ms)==DBT_OK && !acceptor.enabled);
    CHECK(!dbt_a2dp_acceptor_accept(&acceptor,&peers[1].runtime.acl.links[0]));
    for(unsigned i=0;i<2;++i)CHECK(peers[i].allocated==peers[i].released && !peers[i].integrity_errors);
    acceptor_enabled=avdtp_enabled=false;return 0;
}
EXPORT int test_a2dp_acceptor_configured_acl_loss(void){
    int result=acceptor_fixture();if(result)return result;
    uint8_t configuration[]={4,8,1,0,7,6,0,0,0x21,0x15,2,53,8,0},label=0;
    CHECK(dbt_avdtp_command(&avdtp_signals[0],3,configuration,sizeof(configuration),clock_ms,&label)==DBT_OK);
    CHECK(acceptor_steps(20,true) && acceptor.sink.state==DBT_STREAM_CONFIGURED);
    uint8_t removed[]={0,1,0,0x13};
    CHECK(event(&peers[0],5,removed,sizeof(removed))==DBT_OK);
    CHECK(event(&peers[1],5,removed,sizeof(removed))==DBT_OK);
    CHECK(acceptor_steps(50,false) && acceptor.active && acceptor.draining && sink_job_pending);
    CHECK(acceptor_steps(20,true) && !acceptor.active && !acceptor.draining && acceptor.enabled);
    CHECK(!avdtp_errors);
    for(unsigned i=0;i<2;++i)CHECK(peers[i].allocated==peers[i].released && !peers[i].integrity_errors);
    acceptor_enabled=avdtp_enabled=false;return 0;
}
EXPORT int test_runtime_first_parser_failure_metadata(void){
    for(unsigned kind=0;kind<3;++kind){
        clock_ms=0;int result=fixture(0);if(result)return result;
        peer_t *peer=&peers[0];add_link(peer,1,false);
        if(kind==0){
            uint8_t malformed[]={1};
            CHECK(event(peer,0x13,malformed,sizeof(malformed))==DBT_OK);
        }else if(kind==1){
            uint8_t bytes[]={2,1,0x20,5,0,0,0,0x40,0,0xa5};
            CHECK(dbt_runtime_receive(&peer->runtime,bytes,sizeof(bytes))==DBT_OK);
        }else{
            uint8_t bytes[]={2,1,0x20,8,0,4,0,1,0,8,0xa5,5,0};
            CHECK(dbt_runtime_receive(&peer->runtime,bytes,sizeof(bytes))==DBT_OK);
        }
        dbt_runtime_poll(&peer->runtime,++clock_ms,8,0);
        const dbt_runtime_failure_t *f=&peer->runtime.failure;
        CHECK(peer->runtime.host.fault==DBT_FAULT_BAD_PACKET && peer->runtime.host.fault_stage==5);
        CHECK(f->at_ms==clock_ms && f->result==DBT_INVALID);
        CHECK(f->origin==(kind==0?DBT_RUNTIME_FAULT_ACL_EVENT:kind==1?DBT_RUNTIME_FAULT_L2CAP:DBT_RUNTIME_FAULT_CHANNEL_PDU));
        if(kind){
            CHECK(f->handle==1 && f->cid==(kind==1?0x40:1) && f->generation && f->link_present);
            if(kind==1)CHECK(f->code==0 && f->identifier==0);
            else CHECK(f->code==8 && f->identifier==0xa5 && f->declared==5);
        }else CHECK(f->event==0x13 && f->bytes==1);
        uint32_t first_at=f->at_ms;uint8_t first_origin=f->origin;
        dbt_runtime_abort(&peer->runtime,DBT_FAULT_TIMEOUT);
        CHECK(f->at_ms==first_at && f->origin==first_origin && peer->runtime.host.fault==DBT_FAULT_BAD_PACKET);
        CHECK(peer->allocated==peer->released && !peer->integrity_errors);
    }
    return 0;
}
EXPORT int test_commands_copy_credit_status_and_rejection(void){
    clock_ms=0;int r=fixture(0);if(r)return r;peer_t *p=&peers[0];dbt_host_t *host=&p->runtime.host;
    uint8_t parameter=0x55;
    CHECK(dbt_host_command(host,0xfc01,&parameter,1,DBT_COMMAND_COMPLETE,command_done,0,clock_ms)==DBT_OK);
    parameter=0xaa;p->ready=false;CHECK(dbt_host_wait_ms(host,clock_ms)==100);
    dbt_runtime_poll(&p->runtime,++clock_ms,8,0);CHECK(host->pending_opcode==0);
    p->ready=true;dbt_runtime_poll(&p->runtime,++clock_ms,8,0);
    CHECK(p->wire[4]==0x55 && host->pending_opcode==0xfc01 && host->command_credits==0);
    CHECK(command_status(p,0xfc01,1,0)==DBT_OK);dbt_runtime_poll(&p->runtime,++clock_ms,8,0);
    CHECK(done_calls==0 && host->pending_opcode==0xfc01);
    CHECK(complete(p,0xfc01,0,0x0c)==DBT_OK);dbt_runtime_poll(&p->runtime,++clock_ms,8,0);
    CHECK(done_calls==1 && last_command_outcome==DBT_COMMAND_REJECTED && last_status==0x0c && host->state==DBT_HOST_IDENTIFIED);
    p->ready=true;
    CHECK(dbt_host_command(host,0x0409,0,0,DBT_COMMAND_STATUS,command_done,0,clock_ms)==DBT_OK);
    dbt_runtime_poll(&p->runtime,++clock_ms,8,0);CHECK(host->pending_opcode==0);
    CHECK(complete(p,0,1,0)==DBT_OK);dbt_runtime_poll(&p->runtime,++clock_ms,8,0);
    CHECK(host->pending_opcode==0x0409);p->ready=true;
    CHECK(command_status(p,0x0409,1,0)==DBT_OK);dbt_runtime_poll(&p->runtime,++clock_ms,8,0);
    CHECK(done_calls==2 && last_command_outcome==DBT_COMMAND_SUCCESS && host->pending_opcode==0);
    return 0;
}
EXPORT int test_commands_queue_timeout_abort_and_late_response(void){
    clock_ms=0;int r=fixture(0);if(r)return r;peer_t *p=&peers[0];dbt_host_t *host=&p->runtime.host;
    for(unsigned i=0;i<4;++i)CHECK(dbt_host_command(host,(uint16_t)(0xfc01+i),0,0,DBT_COMMAND_COMPLETE,command_done,0,clock_ms)==DBT_OK);
    CHECK(dbt_host_command(host,0xfc09,0,0,DBT_COMMAND_COMPLETE,0,0,clock_ms)==DBT_NO_SPACE);
    dbt_runtime_poll(&p->runtime,++clock_ms,8,0);CHECK(host->pending_opcode==0xfc01);
    dbt_runtime_poll(&p->runtime,clock_ms+100,8,0);
    CHECK(host->state==DBT_HOST_FAILED && host->fault==DBT_FAULT_TIMEOUT && done_calls==4 && host->command_count==0);
    CHECK(last_command_outcome==DBT_COMMAND_ABORTED && !p->runtime.ready);
    CHECK(complete(p,0xfc01,1,0)==DBT_OK);dbt_runtime_poll(&p->runtime,clock_ms+101,8,0);
    CHECK(done_calls==4 && host->state==DBT_HOST_FAILED);
    clock_ms=0;r=fixture(0);if(r)return r;p=&peers[0];host=&p->runtime.host;p->accept=false;
    CHECK(dbt_host_command(host,0xfc01,0,0,DBT_COMMAND_COMPLETE,command_done,0,clock_ms)==DBT_OK);
    dbt_runtime_poll(&p->runtime,clock_ms+100,8,0);
    CHECK(done_calls==1 && last_command_outcome==DBT_COMMAND_TIMEOUT && host->state==DBT_HOST_IDENTIFIED);
    return 0;
}
EXPORT int test_runtime_paired_channel_and_fragmented_data(void){
    clock_ms=0;int r=fixture(0);if(r)return r;r=fixture(1);if(r)return r;
    add_link(&peers[0],1,false);add_link(&peers[1],1,false);
    dbt_service_t service={0x19,256,48,DBT_SECURITY_NONE};
    CHECK(dbt_channels_register(&peers[1].runtime.channels,service)==DBT_OK);
    uint16_t cid=0;CHECK(dbt_channels_open(&peers[0].runtime.channels,1,0x19,256,48,DBT_SECURITY_NONE,clock_ms,&cid)==DBT_OK);
    pump(70);
    CHECK(peers[0].runtime.ready && peers[1].runtime.ready && peers[0].opens==1 && peers[1].opens==1);
    static uint8_t bytes[193];for(unsigned i=0;i<sizeof(bytes);++i)bytes[i]=(uint8_t)(i*37u);
    CHECK(dbt_runtime_send(&peers[0].runtime,1,cid,bytes,sizeof(bytes),tx_done,0)==DBT_OK);
    CHECK(dbt_runtime_send(&peers[0].runtime,1,cid,bytes,1,0,0)==DBT_BUSY);
    pump(40);
    CHECK(done_calls==1 && last_tx_outcome==DBT_TX_ADMITTED && peers[1].data_calls==1 && peers[1].received_size==sizeof(bytes));
    for(unsigned i=0;i<sizeof(bytes);++i)CHECK(peers[1].received[i]==bytes[i]);
    CHECK(peers[1].allocated==peers[1].released && peers[0].integrity_errors==0 && peers[1].integrity_errors==0);
    CHECK(dbt_channels_close(&peers[0].runtime.channels,1,cid,clock_ms)==DBT_OK);pump(30);
    CHECK(dbt_channels_find(&peers[0].runtime.channels,1,cid)==0);return 0;
}
EXPORT int test_runtime_dual_bearer_credit_fairness(void){
    clock_ms=0;int r=fixture(0);if(r)return r;r=fixture(1);if(r)return r;
    for(unsigned i=0;i<2;++i){add_link(&peers[i],1,false);add_link(&peers[i],2,true);
        CHECK(dbt_channels_register_fixed(&peers[i].runtime.channels,DBT_LINK_CLASSIC,7)==DBT_OK);
        CHECK(dbt_channels_register_fixed(&peers[i].runtime.channels,DBT_LINK_LE,4)==DBT_OK);}
    static uint8_t bytes[100];for(unsigned i=0;i<sizeof(bytes);++i)bytes[i]=(uint8_t)i;
    CHECK(dbt_runtime_send_fixed(&peers[0].runtime,1,7,bytes,100,tx_done,0)==DBT_OK);
    CHECK(dbt_runtime_send_fixed(&peers[0].runtime,2,4,bytes,100,tx_done,0)==DBT_OK);
    peers[0].runtime.acl.pools[0].available=0;pump(24);
    CHECK(done_calls==1 && peers[1].fixed_calls==1 && peers[0].runtime.tx[0].active && !peers[0].runtime.tx[1].active);
    CHECK(dbt_runtime_wait_ms(&peers[0].runtime,clock_ms)==UINT32_MAX);
    peers[0].runtime.acl.pools[0].available=2;pump(24);
    CHECK(done_calls==2 && peers[1].fixed_calls==2 && peers[0].integrity_errors==0 && peers[1].runtime.ready);return 0;
}
EXPORT int test_runtime_disconnect_pins_and_partial_expiry(void){
    clock_ms=0;int r=fixture(0);if(r)return r;peer_t *p=&peers[0];add_link(p,1,true);
    CHECK(dbt_channels_register_fixed(&p->runtime.channels,DBT_LINK_LE,4)==DBT_OK);
    static uint8_t bytes[100];CHECK(dbt_runtime_send_fixed(&p->runtime,1,4,bytes,100,tx_done,0)==DBT_OK);
    dbt_runtime_poll(&p->runtime,++clock_ms,8,1);CHECK(!p->ready);
    uint8_t disconnected[4]={0,1,0,0x13};CHECK(event(p,5,disconnected,4)==DBT_OK);
    dbt_runtime_poll(&p->runtime,++clock_ms,8,8);check_borrow(p);
    CHECK(done_calls==1 && last_tx_outcome==DBT_TX_LINK_LOST && p->integrity_errors==0 && p->runtime.acl.pools[1].available==2);
    p->ready=true;add_link(p,1,true);
    uint8_t partial[]={2,1,0x20,6,0,10,0,4,0,0xaa,0xbb};
    CHECK(dbt_runtime_receive(&p->runtime,partial,sizeof(partial))==DBT_OK);dbt_runtime_poll(&p->runtime,++clock_ms,8,0);
    CHECK(p->allocated==1 && p->released==0 && dbt_runtime_wait_ms(&p->runtime,clock_ms)==1000);
    dbt_runtime_poll(&p->runtime,clock_ms+1000,8,0);CHECK(p->released==1);
    clock_ms+=1001;
    CHECK(dbt_runtime_receive(&p->runtime,partial,sizeof(partial))==DBT_OK);dbt_runtime_poll(&p->runtime,++clock_ms,8,0);
    dbt_runtime_abort(&p->runtime,DBT_FAULT_CONTROLLER);CHECK(p->released==2 && !p->runtime.ready);return 0;
}
EXPORT int test_runtime_control_cross_link_scheduling(void){
    clock_ms=0;int r=fixture(0);if(r)return r;peer_t *p=&peers[0];add_link(p,1,false);add_link(p,2,true);
    uint8_t classic[]={8,1,2,0,0xaa,0xbb};uint8_t le[]={0x12,1,8,0,6,0,12,0,0,0,100,0};
    const dbt_link_t *a=dbt_acl_find(&p->runtime.acl,1),*b=dbt_acl_find(&p->runtime.acl,2);
    CHECK(dbt_channels_receive(&p->runtime.channels,1,a->generation,1,classic,sizeof(classic))==DBT_OK);
    CHECK(dbt_channels_receive(&p->runtime.channels,2,b->generation,5,le,sizeof(le))==DBT_OK);
    p->runtime.acl.pools[0].available=0;dbt_runtime_poll(&p->runtime,++clock_ms,8,1);
    CHECK(!p->ready && (dbt_read_le16(p->wire+1)&0xfff)==2 && dbt_read_le16(p->wire+7)==5 && p->wire[9]==1);
    p->ready=true;completed_acl(p,2);dbt_runtime_poll(&p->runtime,++clock_ms,8,1);
    CHECK(p->ready && dbt_runtime_wait_ms(&p->runtime,clock_ms)==UINT32_MAX);
    p->runtime.acl.pools[0].available=2;dbt_runtime_poll(&p->runtime,++clock_ms,8,1);
    CHECK(!p->ready && p->wire[9]==9 && (dbt_read_le16(p->wire+1)&0xfff)==1);return 0;
}
static int gap_fixture(void){
    clock_ms=0;int r=fixture(0);if(r)return r;
    dbt_gap_config_t config={0};config.name[0]='N';config.name[1]='B';
    config.advertising_minimum=0x80;config.advertising_maximum=0x100;
    config.advertising[0]=2;config.advertising[1]=1;config.advertising[2]=2;config.advertising_size=3;
    dbt_gap_hooks_t hooks={0};CHECK(dbt_gap_init(&gap,&peers[0].runtime.host,config,hooks)==DBT_OK);
    CHECK(dbt_gap_start(&gap)==DBT_OK);return 0;
}
static int gap_ready(void){
    int r=gap_fixture();if(r)return r;
    static const uint16_t expected[]={0x0c01,0x2001,0x0c6d,0x0c13,0x0c24,0x0c56,0x2006,0x2008,0x2009};
    for(unsigned i=0;i<9;++i){
        dbt_gap_poll(&gap,++clock_ms);dbt_runtime_poll(&peers[0].runtime,clock_ms,8,0);
        CHECK(dbt_read_le16(peers[0].wire+1)==expected[i] && !peers[0].ready);
        if(i==0)CHECK(peers[0].wire[11]&0x20);
        if(i==3)CHECK(peers[0].wire_size==252 && peers[0].wire[4]=='N' && peers[0].wire[6]==0 && peers[0].wire[251]==0);
        if(i==6)CHECK(peers[0].wire[17]==7 && peers[0].wire[18]==0);
        controller(&peers[0]);dbt_runtime_poll(&peers[0].runtime,++clock_ms,8,0);
    }
    CHECK(gap.state==DBT_GAP_READY && !gap.advertising && gap.classic_scan==0);return 0;
}
EXPORT int test_gap_setup_and_explicit_visibility(void){
    int r=gap_ready();if(r)return r;peer_t *p=&peers[0];
    CHECK(dbt_gap_set_classic_scan(&gap,true,true)==DBT_OK);
    CHECK(gap.classic_scan==0 && gap.classic_pending && dbt_gap_set_classic_scan(&gap,false,false)==DBT_BUSY);
    dbt_runtime_poll(&p->runtime,++clock_ms,8,0);CHECK(p->wire[4]==3);controller(p);
    dbt_runtime_poll(&p->runtime,++clock_ms,8,0);CHECK(gap.classic_scan==3 && !gap.classic_pending);
    CHECK(dbt_gap_set_advertising(&gap,true)==DBT_OK);dbt_runtime_poll(&p->runtime,++clock_ms,8,0);
    CHECK(!gap.advertising && gap.advertising_pending);controller(p);dbt_runtime_poll(&p->runtime,++clock_ms,8,0);CHECK(gap.advertising);
    uint8_t connected[19]={1,0,1,0,1};dbt_hci_packet_t packet={0};packet.type=4;packet.event=0x3e;packet.payload=connected;packet.size=19;
    CHECK(dbt_gap_event(&gap,&packet)==DBT_OK && !gap.advertising);
    CHECK(dbt_gap_set_advertising(&gap,true)==DBT_OK);dbt_runtime_poll(&p->runtime,++clock_ms,8,0);p->ready=true;
    CHECK(complete(p,0x200a,1,0x0c)==DBT_OK);dbt_runtime_poll(&p->runtime,++clock_ms,8,0);
    CHECK(!gap.advertising && !gap.advertising_pending && gap.rejected_commands==1);return 0;
}
EXPORT int test_gap_security_negative_replies_and_queue_bounds(void){
    int r=gap_ready();if(r)return r;peer_t *p=&peers[0];
    uint8_t body[13]={1,2,3,4,5,6};dbt_hci_packet_t packet={0};packet.type=4;packet.payload=body;packet.event=0x31;packet.size=6;
    for(unsigned i=0;i<4;++i)CHECK(dbt_gap_event(&gap,&packet)==DBT_OK);
    CHECK(dbt_gap_event(&gap,&packet)==DBT_NO_SPACE && gap.reply_count==4);
    dbt_gap_poll(&gap,++clock_ms);dbt_runtime_poll(&p->runtime,clock_ms,8,0);
    CHECK(dbt_read_le16(p->wire+1)==0x0434 && p->wire[3]==7 && p->wire[10]==0x18);
    for(unsigned i=0;i<4;++i){controller(p);dbt_runtime_poll(&p->runtime,++clock_ms,8,0);dbt_gap_poll(&gap,clock_ms);dbt_runtime_poll(&p->runtime,clock_ms,8,0);}
    CHECK(gap.reply_count==0 && p->runtime.host.command_count==0);
    packet.event=0x3e;packet.size=13;body[0]=5;body[1]=0x23;body[2]=0;
    CHECK(dbt_gap_event(&gap,&packet)==DBT_OK);dbt_gap_poll(&gap,++clock_ms);dbt_runtime_poll(&p->runtime,clock_ms,8,0);
    CHECK(dbt_read_le16(p->wire+1)==0x201b && p->wire[3]==2 && p->wire[4]==0x23);
    packet.size=12;CHECK(dbt_gap_event(&gap,&packet)==DBT_INVALID);return 0;
}
EXPORT int test_gap_connection_admission_is_not_establishment(void){
    int r=gap_ready();if(r)return r;peer_t *p=&peers[0];
    uint8_t body[10]={1,2,3,4,5,6,0,0,0,1};dbt_hci_packet_t packet={0};packet.type=4;packet.event=4;packet.payload=body;packet.size=10;
    CHECK(dbt_gap_event(&gap,&packet)==DBT_OK);dbt_gap_poll(&gap,++clock_ms);dbt_runtime_poll(&p->runtime,clock_ms,8,0);
    CHECK(dbt_read_le16(p->wire+1)==0x040a && p->wire[10]==0x0f);
    p->ready=true;CHECK(command_status(p,0x040a,1,0)==DBT_OK);dbt_runtime_poll(&p->runtime,++clock_ms,8,0);
    CHECK(p->runtime.ready && p->link_events==0);
    dbt_link_t link={0};link.used=true;link.handle=0x123;
    CHECK(dbt_gap_disconnect(&gap,&link,0)==DBT_INVALID);
    CHECK(dbt_gap_disconnect(&gap,&link,0x13)==DBT_OK);dbt_runtime_poll(&p->runtime,++clock_ms,8,0);
    CHECK(dbt_read_le16(p->wire+1)==0x0406 && p->wire[4]==0x23 && p->wire[6]==0x13);return 0;
}

static void att_fixed(void *context,const dbt_link_t *link,uint16_t cid,const uint8_t *bytes,uint16_t size){
    (void)context;if(dbt_att_bearer_receive(&att_bearer,link,cid,bytes,size)!=DBT_OK)++att_errors;
}
static void att_link(void *context,const dbt_link_change_t *change){
    link_change(context,change);if(dbt_att_bearer_link(&att_bearer,change)!=DBT_OK)++att_errors;
}
static void att_pump(unsigned rounds){for(unsigned i=0;i<rounds;++i){dbt_att_bearer_poll(&att_bearer,clock_ms);pump(1);}}
EXPORT int test_att_runtime_bearer_fragmentation_and_disconnect(void){
    clock_ms=0;int r=fixture(0);if(r)return r;r=fixture(1);if(r)return r;
    add_link(&peers[0],1,true);add_link(&peers[1],1,true);
    CHECK(dbt_channels_register_fixed(&peers[0].runtime.channels,DBT_LINK_LE,4)==DBT_OK);
    static uint8_t value[100];for(unsigned i=0;i<100;++i)value[i]=(uint8_t)(i*7u);
    static const uint8_t uuid[]={0xed,0xff};
    dbt_attribute_t attribute={0x30,DBT_ATT_READ,100,100,0,2,0,uuid,value};
    dbt_att_hooks_t hooks={0};hooks.context=&peers[1];hooks.allocate=allocate;hooks.release=release;
    CHECK(dbt_att_init(&att_server,&attribute,1,hooks,517,128)==DBT_OK);
    zero(&att_bearer,sizeof(att_bearer));att_errors=0;
    CHECK(dbt_att_bearer_init(&att_bearer,&peers[1].runtime,&att_server)==DBT_OK);
    peers[1].runtime.hooks.channels.fixed_data=att_fixed;peers[1].runtime.hooks.link=att_link;
    static const uint8_t mtu_request[]={2,5,2};
    CHECK(dbt_runtime_send_fixed(&peers[0].runtime,1,4,mtu_request,3,0,0)==DBT_OK);att_pump(20);
    CHECK(peers[0].received_size==3 && peers[0].received[0]==3 && att_server.connections[0].mtu==517);
    static const uint8_t read_request[]={0x0a,0x30,0};
    CHECK(dbt_runtime_send_fixed(&peers[0].runtime,1,4,read_request,3,0,0)==DBT_OK);att_pump(40);
    CHECK(peers[0].received_size==101 && peers[0].received[0]==0x0b && att_errors==0);
    for(unsigned i=0;i<100;++i)CHECK(peers[0].received[i+1]==value[i]);
    CHECK(peers[0].allocated==peers[0].released && peers[1].allocated==peers[1].released);
    CHECK(peers[0].integrity_errors==0 && peers[1].integrity_errors==0);
    peers[1].runtime.acl.pools[1].available=0;
    CHECK(dbt_runtime_send_fixed(&peers[0].runtime,1,4,read_request,3,0,0)==DBT_OK);att_pump(8);
    CHECK(att_bearer.bindings[0].active && att_server.connections[0].request_busy);
    CHECK(dbt_att_bearer_clear(&att_bearer)==DBT_BUSY);
    CHECK(dbt_att_bearer_wait_ms(&att_bearer,clock_ms)>29000);
    uint8_t disconnected[]={0,1,0,0x13};CHECK(event(&peers[1],5,disconnected,4)==DBT_OK);
    dbt_runtime_poll(&peers[1].runtime,++clock_ms,8,4);
    CHECK(!att_bearer.bindings[0].active && !att_server.connections[0].used && att_errors==0);
    CHECK(peers[1].allocated==peers[1].released && dbt_att_bearer_clear(&att_bearer)==DBT_OK);return 0;
}

EXPORT int test_runtime_channel_close_returns_borrowed_data(void){
    clock_ms=0;int r=fixture(0);if(r)return r;r=fixture(1);if(r)return r;
    add_link(&peers[0],1,false);add_link(&peers[1],1,false);
    dbt_service_t service={0x19,256,48,DBT_SECURITY_NONE};
    CHECK(dbt_channels_register(&peers[1].runtime.channels,service)==DBT_OK);
    uint16_t cid=0;CHECK(dbt_channels_open(&peers[0].runtime.channels,1,0x19,256,48,DBT_SECURITY_NONE,clock_ms,&cid)==DBT_OK);
    pump(70);CHECK(peers[0].opens==1 && peers[1].opens==1);
    static uint8_t payload[193];for(unsigned i=0;i<193;++i)payload[i]=(uint8_t)i;
    CHECK(dbt_runtime_send(&peers[0].runtime,1,cid,payload,193,tx_done,0)==DBT_OK);
    dbt_runtime_poll(&peers[0].runtime,++clock_ms,8,1);
    CHECK(!peers[0].ready && peers[0].runtime.tx[0].active && done_calls==0);
    CHECK(dbt_channels_close(&peers[1].runtime.channels,1,peers[1].cid,clock_ms)==DBT_OK);
    pump(30);
    CHECK(done_calls==1 && last_tx_outcome==DBT_TX_ABORTED && !peers[0].runtime.tx[0].active);
    CHECK(peers[1].data_calls==0 && peers[1].allocated==peers[1].released);
    CHECK(dbt_acl_find(&peers[0].runtime.acl,1)!=0 && peers[0].runtime.ready && peers[1].runtime.ready);
    CHECK(peers[0].integrity_errors==0 && peers[1].integrity_errors==0);return 0;
}

static dbt_sdp_t sdp_server;
static dbt_sdp_bearer_t sdp_bearer;
static unsigned sdp_errors;
static bool sdp_oom;
static uint8_t sdp_text[303];
static const uint8_t sdp_class[]={0x35,3,0x19,0xff,0x01};
static dbt_sdp_attribute_t sdp_attributes[2];
static dbt_sdp_record_t sdp_record;
static void sdp_data(void *context,const dbt_channel_t *channel,const uint8_t *bytes,uint16_t size){
    data(context,channel,bytes,size);
    if(dbt_sdp_bearer_receive(&sdp_bearer,channel,bytes,size)!=DBT_OK)++sdp_errors;
}
static void sdp_channel(void *context,dbt_channel_event_t kind,const dbt_channel_t *snapshot,dbt_channel_reason_t reason,uint16_t result){
    channel(context,kind,snapshot,reason,result);dbt_sdp_bearer_event(&sdp_bearer,kind,snapshot);
}
static uint8_t *sdp_allocate(void *context,uint16_t size){return sdp_oom?0:allocate(context,size);}
static int sdp_fixture(void){
    clock_ms=0;int r=fixture(0);if(r)return r;r=fixture(1);if(r)return r;
    add_link(&peers[0],1,false);add_link(&peers[1],1,false);
    sdp_text[0]=0x26;sdp_text[1]=1;sdp_text[2]=44;
    for(unsigned i=3;i<sizeof(sdp_text);++i)sdp_text[i]=(uint8_t)(i*17u);
    sdp_attributes[0]=(dbt_sdp_attribute_t){1,sizeof(sdp_class),sdp_class};
    sdp_attributes[1]=(dbt_sdp_attribute_t){0x100,sizeof(sdp_text),sdp_text};
    sdp_record=(dbt_sdp_record_t){0x10000,sdp_attributes,2};
    CHECK(dbt_sdp_init(&sdp_server,&sdp_record,1,1)==DBT_OK);
    zero(&sdp_bearer,sizeof(sdp_bearer));sdp_errors=0;sdp_oom=false;
    CHECK(dbt_sdp_bearer_init(&sdp_bearer,&peers[1].runtime,&sdp_server,672)==DBT_OK);
    peers[1].runtime.hooks.channels.data=sdp_data;peers[1].runtime.hooks.channels.event=sdp_channel;
    peers[1].runtime.hooks.allocate=sdp_allocate;
    uint16_t cid;
    CHECK(dbt_channels_open(&peers[0].runtime.channels,1,1,672,48,DBT_SECURITY_NONE,clock_ms,&cid)==DBT_OK);
    pump(70);CHECK(peers[0].opens==1 && peers[1].opens==1);
    CHECK(peers[1].allocated==peers[1].released && dbt_sdp_bearer_wait_ms(&sdp_bearer)==UINT32_MAX);
    return 0;
}
static void sdp_pump(unsigned rounds){
    for(unsigned i=0;i<rounds;++i){if(dbt_sdp_bearer_poll(&sdp_bearer)!=DBT_OK)++sdp_errors;pump(1);}
}
static const uint8_t sdp_attribute_request[]={4,0x12,0x34,0,14,0,1,0,0,0xff,0xff,0x35,5,10,0,0,0xff,0xff,0};
EXPORT int test_sdp_runtime_fragmentation_and_lazy_reply(void){
    int r=sdp_fixture();if(r)return r;
    CHECK(dbt_runtime_send(&peers[0].runtime,1,peers[0].cid,sdp_attribute_request,sizeof(sdp_attribute_request),0,0)==DBT_OK);
    sdp_pump(80);
    CHECK(!sdp_errors && peers[0].received_size==333 && peers[0].received[0]==5);
    CHECK(peers[0].received[1]==0x12 && peers[0].received[2]==0x34 && peers[0].received[7]==0x36);
    /* 3-byte outer sequence header + handle (8) + class (8) + text (306). */
    CHECK(peers[0].received[8]==1 && peers[0].received[9]==66);
    CHECK(peers[0].received[26]==9 && peers[0].received[27]==1 && peers[0].received[28]==0);
    CHECK(peers[0].received[29]==0x26 && peers[0].received[30]==1 && peers[0].received[31]==44);
    for(unsigned i=0;i<300;++i)CHECK(peers[0].received[32+i]==sdp_text[3+i]);
    CHECK(peers[1].allocated==peers[1].released && peers[0].allocated==peers[0].released);
    CHECK(!peers[0].integrity_errors && !peers[1].integrity_errors);
    CHECK(dbt_sdp_bearer_clear(&sdp_bearer)==DBT_OK);
    return 0;
}
EXPORT int test_sdp_runtime_oom_backpressure_and_disconnect(void){
    int r=sdp_fixture();if(r)return r;sdp_oom=true;
    CHECK(dbt_runtime_send(&peers[0].runtime,1,peers[0].cid,sdp_attribute_request,sizeof(sdp_attribute_request),0,0)==DBT_OK);
    sdp_pump(30);
    CHECK(peers[0].received_size==7 && peers[0].received[0]==1 && peers[0].received[6]==6 && !sdp_errors);
    CHECK(peers[1].allocated==0 && peers[1].released==0);
    sdp_oom=false;peers[1].runtime.acl.pools[0].available=0;
    CHECK(dbt_runtime_send(&peers[0].runtime,1,peers[0].cid,sdp_attribute_request,sizeof(sdp_attribute_request),0,0)==DBT_OK);
    sdp_pump(12);
    CHECK(sdp_bearer.replies[0].active && peers[1].runtime.tx[0].active);
    CHECK(dbt_sdp_bearer_clear(&sdp_bearer)==DBT_BUSY);
    const dbt_channel_t *current=dbt_channels_find(&peers[1].runtime.channels,1,peers[1].cid);CHECK(current);
    CHECK(dbt_sdp_bearer_receive(&sdp_bearer,current,sdp_attribute_request,sizeof(sdp_attribute_request))==DBT_BUSY);
    CHECK(dbt_sdp_bearer_wait_ms(&sdp_bearer)==UINT32_MAX);
    uint8_t disconnected[]={0,1,0,0x13};CHECK(event(&peers[1],5,disconnected,4)==DBT_OK);
    dbt_runtime_poll(&peers[1].runtime,++clock_ms,8,4);
    CHECK(!sdp_bearer.replies[0].active && !sdp_bearer.replies[0].bytes);
    CHECK(peers[1].allocated==peers[1].released && dbt_sdp_bearer_clear(&sdp_bearer)==DBT_OK);
    CHECK(!sdp_errors && !peers[1].integrity_errors);
    return 0;
}

static uint8_t large_classic_received[2048],large_le_received[515];
static unsigned large_classic_calls,large_le_calls;
static void large_classic_data(void *context,const dbt_channel_t *channel,const uint8_t *bytes,uint16_t size){
    (void)context;(void)channel;
    if(size!=sizeof(large_classic_received)){++peers[1].integrity_errors;return;}
    copy(large_classic_received,bytes,size);++large_classic_calls;
}
static void large_le_data(void *context,const dbt_link_t *link,uint16_t cid,const uint8_t *bytes,uint16_t size){
    (void)context;(void)link;(void)cid;
    if(size!=sizeof(large_le_received)){++peers[1].integrity_errors;return;}
    copy(large_le_received,bytes,size);++large_le_calls;
}
EXPORT int test_runtime_1021_byte_fragments_with_le_interleave(void){
    clock_ms=0;int r=fixture(0);if(r)return r;r=fixture(1);if(r)return r;
    for(unsigned i=0;i<2;++i){
        add_link(&peers[i],1,false);add_link(&peers[i],2,true);
        peers[i].runtime.acl.mtu[DBT_LINK_CLASSIC]=1021;
        peers[i].runtime.acl.mtu[DBT_LINK_LE]=251;
        CHECK(dbt_channels_register_fixed(&peers[i].runtime.channels,DBT_LINK_LE,4)==DBT_OK);
    }
    dbt_service_t service={0x1001,2048,48,DBT_SECURITY_NONE};
    CHECK(dbt_channels_register(&peers[1].runtime.channels,service)==DBT_OK);
    uint16_t cid=0;
    CHECK(dbt_channels_open(&peers[0].runtime.channels,1,0x1001,2048,48,DBT_SECURITY_NONE,clock_ms,&cid)==DBT_OK);
    pump(70);CHECK(peers[0].opens==1 && peers[1].opens==1);
    peers[1].runtime.hooks.channels.data=large_classic_data;
    peers[1].runtime.hooks.channels.fixed_data=large_le_data;
    large_classic_calls=large_le_calls=0;
    static uint8_t classic_bytes[2048],le_bytes[515];
    for(unsigned iteration=0;iteration<128;++iteration){
        for(unsigned i=0;i<sizeof(classic_bytes);++i)classic_bytes[i]=(uint8_t)(iteration+i*29u);
        for(unsigned i=0;i<sizeof(le_bytes);++i)le_bytes[i]=(uint8_t)(iteration+i*13u);
        CHECK(dbt_runtime_send(&peers[0].runtime,1,cid,classic_bytes,sizeof(classic_bytes),tx_done,0)==DBT_OK);
        CHECK(dbt_runtime_send_fixed(&peers[0].runtime,2,4,le_bytes,sizeof(le_bytes),tx_done,0)==DBT_OK);
        pump(40);
        CHECK(large_classic_calls==iteration+1 && large_le_calls==iteration+1);
        for(unsigned i=0;i<sizeof(classic_bytes);++i)CHECK(large_classic_received[i]==classic_bytes[i]);
        for(unsigned i=0;i<sizeof(le_bytes);++i)CHECK(large_le_received[i]==le_bytes[i]);
        CHECK(peers[1].allocated==peers[1].released && peers[0].integrity_errors==0 && peers[1].integrity_errors==0);
        CHECK(peers[0].runtime.ready && peers[1].runtime.ready && done_calls==2*(iteration+1));
    }
    return 0;
}

EXPORT int test_runtime_rx_flow_counts_consumed_classic_and_le_packets(void){
    clock_ms=0;int r=fixture(0);if(r)return r;r=fixture(1);if(r)return r;
    peers[1].runtime.host.controller.supported_commands[10]=0xe0;
    CHECK(dbt_runtime_enable_rx_flow(&peers[1].runtime,1021,3)==DBT_OK);pump(20);
    CHECK(peers[1].runtime.rx_flow.state==DBT_RX_FLOW_ACTIVE);
    for(unsigned i=0;i<2;++i){
        add_link(&peers[i],1,false);add_link(&peers[i],2,true);
        peers[i].runtime.acl.mtu[DBT_LINK_CLASSIC]=1021;peers[i].runtime.acl.mtu[DBT_LINK_LE]=251;
        CHECK(dbt_channels_register_fixed(&peers[i].runtime.channels,DBT_LINK_LE,4)==DBT_OK);
    }
    dbt_service_t service={0x1001,2048,48,DBT_SECURITY_NONE};
    CHECK(dbt_channels_register(&peers[1].runtime.channels,service)==DBT_OK);
    uint16_t cid;
    CHECK(dbt_channels_open(&peers[0].runtime.channels,1,0x1001,2048,48,DBT_SECURITY_NONE,clock_ms,&cid)==DBT_OK);
    pump(80);CHECK(peers[0].opens==1 && peers[1].opens==1);
    peers[1].runtime.hooks.channels.data=large_classic_data;peers[1].runtime.hooks.channels.fixed_data=large_le_data;
    large_classic_calls=large_le_calls=0;
    static uint8_t classic_bytes[2048],le_bytes[515];
    for(unsigned i=0;i<sizeof(classic_bytes);++i)classic_bytes[i]=(uint8_t)(i*29u+59);
    for(unsigned i=0;i<sizeof(le_bytes);++i)le_bytes[i]=(uint8_t)(i*13u+7);
    uint32_t before=peers[1].runtime.rx_flow.acknowledged;
    CHECK(dbt_runtime_send(&peers[0].runtime,1,cid,classic_bytes,sizeof(classic_bytes),0,0)==DBT_OK);
    CHECK(dbt_runtime_send_fixed(&peers[0].runtime,2,4,le_bytes,sizeof(le_bytes),0,0)==DBT_OK);
    pump(100);
    CHECK(large_classic_calls==1 && large_le_calls==1 && peers[1].runtime.rx_flow.acknowledged-before==6);
    for(unsigned i=0;i<sizeof(classic_bytes);++i)CHECK(large_classic_received[i]==classic_bytes[i]);
    for(unsigned i=0;i<sizeof(le_bytes);++i)CHECK(large_le_received[i]==le_bytes[i]);
    CHECK(!peers[0].integrity_errors && !peers[1].integrity_errors && peers[1].allocated==peers[1].released);
    CHECK(peers[1].runtime.ready && peers[1].runtime.host.unexpected_completions==0);
    return 0;
}

static unsigned sco_calls;
static uint8_t sco_status;
static void sco_data(void *context,const dbt_hci_packet_t *packet){
    peer_t *peer=context;++sco_calls;sco_status=packet->packet_status;
    peer->received_size=packet->size;copy(peer->received,packet->payload,packet->size);
    if(packet->type!=DBT_H4_SCO || packet->handle!=7)++peer->integrity_errors;
}
EXPORT int test_runtime_sco_framing_status_and_explicit_route(void){
    clock_ms=0;int r=fixture(0);if(r)return r;peer_t *peer=&peers[0];
    peer->runtime.hooks.sco=sco_data;sco_calls=0;
    static uint8_t wire[259];wire[0]=3;wire[1]=7;wire[3]=255;
    for(unsigned i=0;i<255;++i)wire[4+i]=(uint8_t)(i*17u+7);
    for(unsigned status=0;status<4;++status){
        wire[2]=(uint8_t)(status<<4);
        CHECK(dbt_runtime_receive(&peer->runtime,wire,sizeof(wire))==DBT_OK);
        dbt_runtime_poll(&peer->runtime,++clock_ms,8,0);
        CHECK(sco_calls==status+1 && sco_status==status && peer->received_size==255);
        for(unsigned i=0;i<255;++i)CHECK(peer->received[i]==wire[4+i]);
        CHECK(peer->runtime.ready && !peer->allocated && !peer->integrity_errors);
    }
    CHECK(dbt_runtime_receive(&peer->runtime,wire,sizeof(wire)-1)==DBT_INVALID);
    r=fixture(0);if(r)return r;peer=&peers[0];wire[2]=0;
    CHECK(dbt_runtime_receive(&peer->runtime,wire,sizeof(wire))==DBT_OK);
    dbt_runtime_poll(&peer->runtime,++clock_ms,8,0);
    CHECK(peer->runtime.host.fault==DBT_FAULT_UNSUPPORTED_PACKET && !peer->runtime.ready);
    r=fixture(0);if(r)return r;peer=&peers[0];wire[2]=0x40;
    CHECK(dbt_runtime_receive(&peer->runtime,wire,sizeof(wire))==DBT_INVALID);
    return 0;
}
