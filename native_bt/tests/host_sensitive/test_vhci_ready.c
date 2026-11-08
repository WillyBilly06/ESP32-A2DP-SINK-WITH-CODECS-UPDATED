#include "probe_vhci_ready.h"
#include "dbt_hci.h"
#include <stdio.h>
#include <string.h>
#ifdef _WIN32
#include <windows.h>
static void yield_cpu(void){Sleep(0);}
#else
#include <pthread.h>
#include <sched.h>
static void yield_cpu(void){sched_yield();}
#endif
#define CHECK(x) do{if(!(x)){fprintf(stderr,"FAIL readiness line%d\n",__LINE__);return 1;}}while(0)
#define DBT_PROBE_SCHEDULER_TRACE 1
#define DBT_PROBE_AUDIO_OUTPUT 1
#define DBT_PROBE_AUDIO_TEST 1
#define HOST_CALL(phase,expression) do{expression;}while(0)
#define DBT_MAX_LINKS 4u
static _Atomic uint32_t vhci_tx_inflight;
static bool readiness_held_sampled,readiness_clear_sampled,sdk_ready,clear_during_check,complete_during_send;
static uint32_t sdk_checks,sdk_sends,wakes,flag_at_send;
static uint32_t fake_us;
static struct{struct{struct{bool used;uint16_t packets;}pending[DBT_MAX_LINKS];}rx_flow;}runtime;
static int64_t esp_timer_get_time(void){return fake_us;}
static void wake_owner(void){++wakes;}
static bool esp_vhci_host_check_send_available(void){
    ++sdk_checks;if(clear_during_check)atomic_store(&vhci_tx_inflight,0);return sdk_ready;
}
static void esp_vhci_host_send_packet(uint8_t *p,uint16_t n){
    (void)p;(void)n;++sdk_sends;flag_at_send=atomic_load(&vhci_tx_inflight);if(complete_during_send)atomic_store(&vhci_tx_inflight,0);
}
#include "readiness_under_test.h"
static int values(void){
    probe_vhci_ready_sample_t s;probe_vhci_ready_stats_t st;
    atomic_store(&vhci_tx_inflight,0);probe_vhci_ready_init(&vhci_tx_inflight);
    probe_vhci_ready_sample(&s);CHECK(s.coherent && !(s.flags&PROBE_VHCI_SAMPLE) && !s.owned_now);
    probe_vhci_ready_submitted(100,0x10c35u);atomic_store(&vhci_tx_inflight,1);
    probe_vhci_ready_sample(&s);CHECK(!(s.flags&PROBE_VHCI_SAMPLE) && (s.flags&PROBE_VHCI_SUBMITTED) && s.submitted_us==100 && s.packet==0x10c35u);
    probe_vhci_ready_observe(true,false,110,3);probe_vhci_ready_sample(&s);
    CHECK(s.coherent && s.pending_acks==3 && s.flags==(PROBE_VHCI_SAMPLE|PROBE_VHCI_HELD|PROBE_VHCI_SUBMITTED));
    probe_vhci_ready_observe(true,true,150,3);probe_vhci_ready_observe(true,true,650,3);
    probe_vhci_ready_stats(&st);CHECK(st.observations==3 && st.held==3 && st.ready_while_held==2 && st.max_ready_span_us==500 && st.max_slot_age_us==550 && st.bytes<=80);
    atomic_store(&vhci_tx_inflight,0);probe_vhci_ready_sample(&s);
    CHECK(s.coherent && !s.owned_now && (s.flags&PROBE_VHCI_HELD));
    probe_vhci_ready_observe(false,true,700,0);
    probe_vhci_ready_submitted(UINT32_MAX-100u,0x20080u);probe_vhci_ready_sample(&s);
    CHECK(!(s.flags&PROBE_VHCI_SAMPLE) && s.packet==0x20080u);
    probe_vhci_ready_observe(true,true,50,1);probe_vhci_ready_observe(true,true,800,0);
    probe_vhci_ready_stats(&st);CHECK(st.max_ready_span_us==750 && st.max_slot_age_us==901);
    probe_vhci_ready_init(NULL);probe_vhci_ready_sample(&s);CHECK(s.owned_now==UINT32_MAX);
    puts("PASS readiness samples, per-submission invalidation, independent current flag, age/span wrap and unknown state");return 0;
}
static int real_decisions(void){
    uint8_t packet[]={1,0x35,0x0c,5,1,0x80,0,1,0};
    for(unsigned bits=0;bits<32;++bits){
        bool held=(bits&1u)!=0;sdk_ready=(bits&2u)!=0;clear_during_check=(bits&4u)!=0;
        readiness_held_sampled=(bits&8u)!=0;readiness_clear_sampled=(bits&16u)!=0;
        atomic_store(&vhci_tx_inflight,held);probe_vhci_ready_init(&vhci_tx_inflight);
        memset(&runtime,0,sizeof runtime);runtime.rx_flow.pending[0].used=true;runtime.rx_flow.pending[0].packets=2;
        sdk_checks=sdk_sends=wakes=0;fake_us=200;
        CHECK(transport_ready(NULL)==(!held && sdk_ready));
        CHECK(sdk_checks==(held?((bits&8u)?0u:1u):1u));
        if(held && clear_during_check && !(bits&8u)){
            probe_vhci_ready_stats_t st;probe_vhci_ready_stats(&st);CHECK(!st.ready_while_held);
        }
    }
    for(unsigned sdk=0;sdk<2;++sdk)for(unsigned held=0;held<2;++held)for(unsigned cb=0;cb<2;++cb){
        sdk_ready=sdk!=0;clear_during_check=false;complete_during_send=cb!=0;
        readiness_held_sampled=readiness_clear_sampled=false;
        atomic_store(&vhci_tx_inflight,held);probe_vhci_ready_init(&vhci_tx_inflight);
        sdk_checks=sdk_sends=wakes=0;fake_us=500;
        bool accepted=transport_send(NULL,packet,sizeof packet);
        CHECK(accepted==(!held&&sdk) && sdk_sends==(accepted?1u:0u));
        if(accepted){CHECK(flag_at_send==1u);probe_vhci_ready_sample_t s;probe_vhci_ready_sample(&s);CHECK(s.packet==0x10c35u && !(s.flags&PROBE_VHCI_SAMPLE));CHECK(atomic_load(&vhci_tx_inflight)==(cb?0u:1u));}
        send_available();CHECK(!atomic_load(&vhci_tx_inflight) && wakes==1);
    }
    sdk_ready=false;clear_during_check=false;atomic_store(&vhci_tx_inflight,1);
    readiness_held_sampled=false;readiness_clear_sampled=false;sdk_checks=0;
    for(unsigned i=0;i<50;++i)CHECK(!transport_ready(NULL));
    CHECK(sdk_checks==1);readiness_held_sampled=false;CHECK(!transport_ready(NULL));CHECK(sdk_checks==2);
    puts("PASS actual main ready/send/callback decisions, fresh checks, callback races and one held-path observation per owner turn");return 0;
}
static _Atomic unsigned done,error;
#ifdef _WIN32
static DWORD WINAPI produce(LPVOID unused)
#else
static void *produce(void *unused)
#endif
{
    (void)unused;
    for(uint32_t i=0;i<50000u && !atomic_load(&error);++i){
        probe_vhci_ready_submitted(i*100u,0x10000u|i);atomic_store(&vhci_tx_inflight,1);
        probe_vhci_ready_observe(true,false,i*100u+10u,3);
        probe_vhci_ready_observe(true,true,i*100u+20u,3);atomic_store(&vhci_tx_inflight,0);
    }
    atomic_store(&done,1);
#ifdef _WIN32
    return 0;
#else
    return NULL;
#endif
}
int main(void){
    if(values() || real_decisions())return 1;
    atomic_store(&done,0);atomic_store(&error,0);probe_vhci_ready_init(&vhci_tx_inflight);
#ifdef _WIN32
    HANDLE thread=CreateThread(NULL,0,produce,NULL,0,NULL);CHECK(thread);
#else
    pthread_t thread;CHECK(!pthread_create(&thread,NULL,produce,NULL));
#endif
    unsigned snapshots=0;
    while(!atomic_load(&done)){
        probe_vhci_ready_sample_t s;probe_vhci_ready_sample(&s);
        if(s.coherent && (s.flags&PROBE_VHCI_SAMPLE)){
            uint32_t ordinal=s.packet&0xffffu,delta=(s.flags&PROBE_VHCI_SDK_READY)?20u:10u;
            if(s.submitted_us!=ordinal*100u || s.at_us-s.submitted_us!=delta || s.pending_acks!=3u)atomic_store(&error,1);
        }
        ++snapshots;yield_cpu();
    }
#ifdef _WIN32
    WaitForSingleObject(thread,INFINITE);CloseHandle(thread);
#else
    pthread_join(thread,NULL);
#endif
    CHECK(!atomic_load(&error));probe_vhci_ready_stats_t st;probe_vhci_ready_stats(&st);
    CHECK(st.observations==100000u && st.held==100000u && st.ready_while_held==50000u);
    printf("PASS50000 concurrent submissions and %u bounded snapshots; no hardware or timing claim\n",snapshots);return 0;
}
