/* Actual flow core, existing setup/lifetime regressions and real app policy. */
#include "../test_rx_flow.c"
#include "dbt_avdtp_sink.h"
#include "dbt_codec.h"
#include <stdio.h>
static bool initialized,restart_hold;
static struct {
    bool active,draining;
    struct {bool failed,retiring;unsigned state;} sink;
    struct {uint32_t serial;uint16_t handle;uint32_t generation;} signaling;
} profile;
static dbt_codec_worker_t worker;
#include "actual_audio_limit.inc"
#include "actual_retirement.inc"
static uint16_t cap;
static bool gate_open;
static bool boolean_gate(void *context,const dbt_link_t *link){(void)context;(void)link;return gate_open;}
static uint16_t capped(void *context,const dbt_link_t *link,uint16_t pending,uint16_t window){
    (void)context;(void)pending;(void)window;return link->kind==DBT_LINK_LE?UINT16_MAX:cap;
}
static unsigned pending_total(void){unsigned n=0;for(unsigned i=0;i<DBT_MAX_LINKS;i++)if(flow.pending[i].used)n+=flow.pending[i].packets;return n;}
static unsigned pending_for(uint16_t handle){for(unsigned i=0;i<DBT_MAX_LINKS;i++)if(flow.pending[i].used&&flow.pending[i].handle==handle)return flow.pending[i].packets;return 0;}
static int setup_capped(void){fixture();cap=1;CHECK(dbt_rx_flow_set_limit(&flow,capped,0)==DBT_OK);int r=active();if(r)return r;r=add_link(1,false);if(r)return r;return add_link(2,true);}
static int partial_transport_and_legacy(void){
    int r=setup_capped();if(r)return r;
    for(unsigned i=0;i<3;i++)CHECK(dbt_rx_flow_completed(&flow,dbt_acl_find(&acl,1))==DBT_OK);
    CHECK(dbt_rx_flow_wait_ms(&flow)==0);dbt_rx_flow_poll(&flow,++now);
    CHECK(sent_size==9 && sent[7]==1 && flow.acknowledged==1 && flow.partial_commands==1 && pending_for(1)==2);
    CHECK(dbt_rx_flow_wait_ms(&flow)==UINT32_MAX);
    CHECK(dbt_rx_flow_completed(&flow,dbt_acl_find(&acl,1))==DBT_OK);
    dbt_rx_flow_poll(&flow,++now);CHECK(pending_for(1)==3 && flow.commands_submitted==1);
    r=release_transport();if(r)return r;
    accept=false;dbt_rx_flow_poll(&flow,++now);
    CHECK(pending_for(1)==3 && flow.acknowledged==1 && flow.partial_commands==1);
    accept=true;dbt_rx_flow_poll(&flow,++now);CHECK(pending_for(1)==2 && flow.acknowledged==2 && flow.partial_commands==2);
    r=release_transport();if(r)return r;cap=UINT16_MAX;dbt_rx_flow_poll(&flow,++now);
    CHECK(sent[7]==2 && flow.acknowledged==4 && !pending_total() && flow.partial_commands==2);
    r=release_transport();if(r)return r;CHECK(dbt_rx_flow_wait_ms(&flow)==UINT32_MAX);
    CHECK(dbt_rx_flow_set_gate(&flow,boolean_gate,0)==DBT_INVALID && dbt_rx_flow_set_limit(&flow,capped,0)==DBT_INVALID);
    fixture();gate_open=false;CHECK(dbt_rx_flow_set_gate(&flow,boolean_gate,0)==DBT_OK && !flow.completion_is_limit);
    CHECK(dbt_rx_flow_set_limit(&flow,0,0)==DBT_OK && flow.completion_is_limit);
    CHECK(dbt_rx_flow_set_gate(&flow,boolean_gate,0)==DBT_OK && !flow.completion_is_limit);
    r=active();if(r)return r;r=add_link(1,false);if(r)return r;
    CHECK(dbt_rx_flow_completed(&flow,dbt_acl_find(&acl,1))==DBT_OK);
    CHECK(dbt_rx_flow_wait_ms(&flow)==UINT32_MAX);unsigned n=sends;dbt_rx_flow_poll(&flow,++now);CHECK(sends==n);
    gate_open=true;CHECK(dbt_rx_flow_wait_ms(&flow)==0);dbt_rx_flow_poll(&flow,++now);
    CHECK(flow.acknowledged==1 && !flow.partial_commands);r=release_transport();if(r)return r;
    fixture();r=add_link(1,false);if(r)return r;
    CHECK(dbt_rx_flow_set_limit(&flow,capped,0)==DBT_BUSY);
    return 0;
}
static int partial_disconnect_and_ble(void){
    int r=setup_capped();if(r)return r;
    CHECK(dbt_rx_flow_completed(&flow,dbt_acl_find(&acl,1))==DBT_OK);
    CHECK(dbt_rx_flow_completed(&flow,dbt_acl_find(&acl,1))==DBT_OK);
    CHECK(dbt_rx_flow_completed(&flow,dbt_acl_find(&acl,2))==DBT_OK);
    host.command_credits=0;acl.pools[0].available=acl.pools[1].available=0;
    dbt_rx_flow_poll(&flow,++now);
    CHECK(sent_size==13 && sent[4]==2 && sent[7]==1 && sent[11]==1 && pending_for(1)==1 && !pending_for(2));
    CHECK(flow.acknowledged==2 && flow.partial_commands==1 && !host.command_credits);
    dbt_link_t old=*dbt_acl_find(&acl,1);uint8_t bytes[]={0,1,0,0x13};
    dbt_hci_packet_t event={.type=4,.event=5,.payload=bytes,.size=4};dbt_link_change_t change;
    CHECK(dbt_acl_event(&acl,&event,&change)==DBT_OK);dbt_rx_flow_link_change(&flow,&change);
    CHECK(flow.retired_on_disconnect==1 && !pending_for(1));
    r=add_link(1,false);if(r)return r;
    CHECK(dbt_rx_flow_completed(&flow,&old)==DBT_INVALID);
    CHECK(dbt_rx_flow_completed(&flow,dbt_acl_find(&acl,1))==DBT_OK);
    dbt_rx_flow_poll(&flow,++now);CHECK(flow.acknowledged==2);
    r=release_transport();if(r)return r;dbt_rx_flow_poll(&flow,++now);
    CHECK(sent_size==9 && sent[7]==1 && flow.acknowledged==3 && !pending_for(1));
    dbt_rx_flow_abort(&flow);r=release_transport();if(r)return r;
    return 0;
}
static void app_fixture(const dbt_link_t *link,unsigned depth){
    initialized=true;restart_hold=false;zero(&profile,sizeof profile);
    profile.active=true;profile.sink.state=DBT_STREAM_STREAMING;
    profile.signaling.serial=1;profile.signaling.handle=link->handle;profile.signaling.generation=link->generation;
    atomic_store(&worker.media_read,UINT32_MAX-1000u);atomic_store(&worker.media_written,UINT32_MAX-1000u);
    worker.low_water=(uint8_t)(depth>3?depth-3:1);
    atomic_store(&worker.queue_layout,depth|((depth>3?depth-3:1)<<8)|(1816u<<16));
}
static int policy_boundaries(void){
    fixture();int r=add_link(1,false);if(r)return r;r=add_link(2,true);if(r)return r;
    const dbt_link_t *link=dbt_acl_find(&acl,1),*le=dbt_acl_find(&acl,2);unsigned cases=0;
    for(unsigned depth=4;depth<=8;depth+=4)for(unsigned window=1;window<=3;window++)for(unsigned queued=0;queued<=depth;queued++)for(unsigned pending=0;pending<=window;pending++){
        app_fixture(link,depth);uint32_t read=UINT32_MAX-3u;atomic_store(&worker.media_read,read);atomic_store(&worker.media_written,read+queued);
        unsigned allowed=probe_audio_rx_credit_limit(0,link,pending,window),outside=window-pending;
        CHECK(allowed<=pending);
        if(queued+outside<=depth-1){
            CHECK(queued+outside+allowed<=depth-1);
            if(allowed<pending)CHECK(queued+outside+allowed+1>depth-1);
        }else CHECK(!allowed);
        CHECK(probe_audio_rx_credit_limit(0,le,pending,window)==pending);
        ++cases;
    }
    app_fixture(link,8);atomic_store(&worker.media_written,atomic_load(&worker.media_read)+6u);
    CHECK(!probe_audio_rx_credit_ready(0,link));
    CHECK(probe_audio_rx_credit_limit(0,link,3,3)==1);
    CHECK(probe_audio_rx_credit_limit(0,link,2,3)==0); /* Other-link permit may become audio. */
    atomic_store(&worker.media_written,atomic_load(&worker.media_read)+7u);CHECK(!probe_audio_rx_credit_limit(0,link,3,3));
    for(unsigned bit=0;bit<10;++bit){
        app_fixture(link,8);atomic_store(&worker.media_written,atomic_load(&worker.media_read)+7u);
        switch(bit){case 0:initialized=false;break;case 1:restart_hold=true;break;case 2:profile.active=false;break;
        case 3:profile.draining=true;break;case 4:profile.sink.failed=true;break;case 5:profile.sink.retiring=true;break;
        case 6:profile.sink.state=DBT_STREAM_IDLE;break;case 7:profile.signaling.serial=0;break;
        case 8:profile.signaling.handle=2;break;case 9:profile.signaling.generation++;break;}
        CHECK(probe_audio_rx_credit_limit(0,link,3,3)==3);
    }
    app_fixture(link,8);atomic_store(&worker.queue_layout,0);CHECK(!probe_audio_rx_credit_limit(0,link,3,3));
    atomic_store(&worker.queue_layout,8);atomic_store(&worker.media_written,atomic_load(&worker.media_read)+9u);CHECK(!probe_audio_rx_credit_limit(0,link,3,3));
    printf("PASS actual policy:%u capacity/maximality/wrap/LE cases plus lifecycle guards\n",cases);return 0;
}
static unsigned capacity_wakes,publication_errors;static uint32_t expected_read;
static void capacity_available(void *context){
    (void)context;++capacity_wakes;
    if(atomic_load_explicit(&worker.media_read,memory_order_acquire)!=expected_read)++publication_errors;
}
static int retirement_notifies_partial_capacity(void){
    fixture();CHECK(dbt_rx_flow_set_limit(&flow,probe_audio_rx_credit_limit,0)==DBT_OK);
    int r=active();if(r)return r;r=add_link(1,false);if(r)return r;
    const dbt_link_t *link=dbt_acl_find(&acl,1);app_fixture(link,8);
    worker.hooks.wake_owner=capacity_available;worker.hooks.decoder.context=0;
    uint32_t read=UINT32_MAX-2u;atomic_store(&worker.media_read,read);atomic_store(&worker.media_written,read+7u);
    for(unsigned i=0;i<3;++i)CHECK(dbt_rx_flow_completed(&flow,link)==DBT_OK);
    CHECK(dbt_rx_flow_wait_ms(&flow)==UINT32_MAX);
    capacity_wakes=publication_errors=0;expected_read=read+1u;retire_slot(&worker,read);
    CHECK(capacity_wakes==1 && !publication_errors && dbt_rx_flow_wait_ms(&flow)==0);
    dbt_rx_flow_poll(&flow,++now);CHECK(sent[7]==1 && pending_for(1)==2 && flow.partial_commands==1);
    r=release_transport();if(r)return r;
    for(unsigned queued=1;queued<=8;++queued){
        read=UINT32_MAX-3u;atomic_store(&worker.media_read,read);atomic_store(&worker.media_written,read+queued);
        capacity_wakes=0;expected_read=read+1u;retire_slot(&worker,read);
        CHECK(capacity_wakes==(queued>=5) && !publication_errors);
    }
    puts("PASS actual retirement publishes slot before capacity wake;7-to6 queues one partial credit, below-pressure retirements stay quiet");return 0;
}
static uint32_t rng_state=0x5317648u;
static uint32_t rng(void){rng_state=rng_state*1664525u+1013904223u;return rng_state;}
static int mixed_credit_model(unsigned depth){
    fixture();CHECK(dbt_rx_flow_set_limit(&flow,probe_audio_rx_credit_limit,0)==DBT_OK);
    int r=active();if(r)return r;r=add_link(1,false);if(r)return r;r=add_link(2,true);if(r)return r;
    const dbt_link_t *cl=dbt_acl_find(&acl,1),*le=dbt_acl_find(&acl,2);app_fixture(cl,depth);
    unsigned permits=3,q=0,deferred=0,received=0;
    for(unsigned step=0;step<50000;++step){
        uint32_t v=rng();
        if(deferred && (v&8u)){CHECK(q<depth);++q;deferred=0;atomic_fetch_add(&worker.media_written,1);}
        if(q && (v&3u)==0){--q;atomic_fetch_add(&worker.media_read,1);}
        if(permits && (v%3u)!=0){
            --permits;++received;
            if(v&4u){
                if((v%5u)==0 && !deferred)deferred=1;
                else if((v%7u)!=0){CHECK(q<depth);++q;atomic_fetch_add(&worker.media_written,1);}
                CHECK(dbt_rx_flow_completed(&flow,cl)==DBT_OK);
            }else CHECK(dbt_rx_flow_completed(&flow,le)==DBT_OK);
        }
        if(!ready && (v&16u)){r=release_transport();if(r)return r;}
        unsigned before=sends;dbt_rx_flow_poll(&flow,++now);
        if(sends!=before){
            CHECK(sends==before+1 && dbt_read_le16(sent+1)==0x0c35);
            for(unsigned i=0;i<sent[4];++i)permits+=dbt_read_le16(sent+7u+4u*i);
        }
        CHECK(permits+pending_total()==3 && received==flow.acknowledged+pending_total());
        CHECK(q+3u-pending_for(1)+deferred<=depth);
        CHECK(q<=depth && !host.fault && !flow.retired_on_disconnect);
        if(!ready)CHECK(dbt_rx_flow_wait_ms(&flow)==UINT32_MAX);
    }
    CHECK(flow.partial_commands && flow.acknowledged);
    r=release_transport();if(r)return r;
    printf("PASS50000 mixed Classic/LE permits, fragmentation/deferred PDU, held transport and queue-depth%u conservation; partial=%u\n",depth,flow.partial_commands);
    return 0;
}
int main(void){
    int line=test_rx_flow_setup_limits_and_capabilities();
    if(!line)line=test_rx_flow_acks_ignore_credits_and_retain_transport_bytes();
    if(!line)line=test_rx_flow_disconnect_generations_and_error_response();
    if(!line)line=partial_transport_and_legacy();if(!line)line=partial_disconnect_and_ble();
    if(!line)line=policy_boundaries();if(!line)line=retirement_notifies_partial_capacity();if(!line)line=mixed_credit_model(4);if(!line)line=mixed_credit_model(8);
    if(line){fprintf(stderr,"FAIL partial-credit source line%d\n",line);return 1;}
    puts("PASS actual partial HCI completion limits and lifetime; no hardware timing proof");return 0;
}
