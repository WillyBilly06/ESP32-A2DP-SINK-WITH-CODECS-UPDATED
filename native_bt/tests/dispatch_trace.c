#include <stdint.h>
#include <stdio.h>
#include <stdatomic.h>
#include <inttypes.h>
#include "probe_dispatch_trace.h"
static unsigned core,depth,max_depth,clocks,steps;
static uint32_t now_us=1000;
unsigned dispatch_test_core(void){return core;}
uint32_t dispatch_test_time(void){if(!depth)steps=99;++clocks;return now_us++;}
uint32_t dispatch_test_mask(void){unsigned old=depth++;if(depth>max_depth)max_depth=depth;return old;}
void dispatch_test_unmask(uint32_t old){depth=old;}
static unsigned inject;
static void snapshot_step(void);
#define DISPATCH_SNAPSHOT_STEP() snapshot_step()
#include DISPATCH_TRACE_IMPL
static void snapshot_step(void){
    if(inject==1)atomic_fetch_add(&rings[core].version,2u);
    if(inject==2)probe_dispatch_begin(99,now_us);
    inject=0;
}
#define CHECK(x) do{if(!(x)){fprintf(stderr,"FAIL dispatch line %u\n",(unsigned)__LINE__);return 1;}}while(0)
#define PROBE_SNAPSHOT_LOG(...) printf(__VA_ARGS__)
#include "formatter.inc"
int main(void){
    probe_dispatch_snapshot_t s,t;
    void *host=(void *)(uintptr_t)0x1230,*renderer=(void *)(uintptr_t)0x2340;
    probe_dispatch_bind(PROBE_DISPATCH_HOST,host);probe_dispatch_bind(PROBE_DISPATCH_RENDER,renderer);
    CHECK(!probe_dispatch_snapshot(0,&s) && !probe_dispatch_snapshot(2,&s));
    probe_dispatch_switch(1,host);probe_dispatch_mark(1,1,1);probe_dispatch_eof(0,host);
    CHECK(!clocks && !depth);core=1;
    probe_dispatch_begin(1,1000);probe_dispatch_switch(1,host);
    CHECK(atomic_load(&rings[1].head)==1 && clocks==1);
    now_us=1100;probe_dispatch_eof(1099,renderer);
    CHECK(atomic_load(&rings[1].events[1].at_us)==1099);
    CHECK((atomic_load(&rings[1].events[1].info)>>4)==PROBE_DISPATCH_RENDER);
    CHECK(!probe_dispatch_snapshot(1,&s));
    probe_dispatch_freeze(2,1200);CHECK(atomic_load(&trace.enabled));
    probe_dispatch_freeze(1,1200);CHECK(!atomic_load(&trace.enabled));
    CHECK(probe_dispatch_snapshot(1,&s) && s.serial==1 && s.gap_us==1200 && s.count==3);
    uint32_t head=s.head;probe_dispatch_switch(1,renderer);probe_dispatch_mark(3,2,100);
    CHECK(probe_dispatch_snapshot(1,&t) && t.head==head);
    CHECK(s.bindings==((1u<<PROBE_DISPATCH_HOST)|(1u<<PROBE_DISPATCH_RENDER)));
    /* Preserve monotonic heads across stream restarts, including word wrap. */
    atomic_store(&rings[0].head,UINT32_MAX-8u);atomic_store(&rings[1].head,UINT32_MAX-8u);
    probe_dispatch_begin(2,now_us);
    for(unsigned i=0;i<64;++i)for(core=0;core<2;++core)probe_dispatch_mark(PROBE_DISPATCH_DECODE_ENTER,3,i);
    core=1;probe_dispatch_freeze(2,now_us);
    CHECK(probe_dispatch_snapshot(0,&s) && s.count==16 && s.head-s.begin_head==64);
    CHECK((s.events[0].info>>8)==48 && (s.events[15].info>>8)==63);
    CHECK(probe_dispatch_snapshot(1,&s) && s.count==16 && s.head-s.begin_head==65);
    CHECK((s.events[0].info>>8)==49 && (s.events[15].info&15u)==PROBE_DISPATCH_FREEZE);
    CHECK(max_depth==1 && !depth && steps!=99);
    log_dispatch_trace();log_dispatch_trace();
    /* Readers never retry through active writers or a changed epoch. */
    core=1;uint32_t version=atomic_load(&rings[1].version);
    atomic_store(&rings[1].version,version|1u);CHECK(!probe_dispatch_snapshot(1,&s));
    atomic_store(&rings[1].version,version);inject=1;CHECK(!probe_dispatch_snapshot(1,&s));
    inject=2;CHECK(!probe_dispatch_snapshot(1,&s));
    probe_dispatch_freeze(99,now_us);CHECK(probe_dispatch_snapshot(0,&s) && !s.count);
    uint32_t old_gap=atomic_load(&trace.gap_us);
    atomic_store(&trace.gap_us,atomic_load(&trace.begin_us)+15000001u);CHECK(!probe_dispatch_snapshot(0,&s));
    atomic_store(&trace.gap_us,old_gap);
    CHECK(!probe_dispatch_snapshot(0,NULL));
    /* Unknown task identities are OTHER and never dereferenced. */
    core=0;probe_dispatch_begin(3,UINT32_MAX-2u);now_us=UINT32_MAX-2u;
    probe_dispatch_switch(0,(void *)(uintptr_t)0x3450);probe_dispatch_mark(PROBE_DISPATCH_HOST_YIELD,1,0xffffffu);
    probe_dispatch_mark(PROBE_DISPATCH_PCM_READY,4,0x12345678u);core=1;probe_dispatch_freeze(3,0);
    CHECK(probe_dispatch_snapshot(0,&s) && s.count==3 && !(s.events[0].info&0xf0u));
    CHECK(s.events[0].at_us==UINT32_MAX-2u && (s.events[2].info>>8)==0x345678u);
    probe_dispatch_begin(4,now_us);probe_dispatch_stop();probe_dispatch_mark(1,1,1);
    CHECK(!probe_dispatch_snapshot(0,&s));
    puts("PASS frozen dispatch: bounded16-slot tails, both cores, real identity classes, time/head wrap, freeze, restart, one-attempt interference, stopped/unknown states and actual JSON formatter");return 0;
}
