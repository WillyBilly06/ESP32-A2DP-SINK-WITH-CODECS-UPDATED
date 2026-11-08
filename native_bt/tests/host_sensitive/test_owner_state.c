/* Actual implementation with a deterministic preemption at one read boundary. */
#include <stdio.h>
static void owner_test_interleave(unsigned);
#define PROBE_OWNER_TEST_READ_PART(part) owner_test_interleave(part)
#ifndef OWNER_STATE_IMPLEMENTATION
#define OWNER_STATE_IMPLEMENTATION "../../esp32_probe/main/probe_owner_state.c"
#endif
#include OWNER_STATE_IMPLEMENTATION
static unsigned mutate,hook_calls;
static void owner_test_interleave(unsigned part){
    (void)part;++hook_calls;
    if(mutate){mutate=0;probe_owner_state_mark(PROBE_OWNER_PROFILES,77,0);probe_owner_state_credits(7,3,78);}
}
#define CHECK(x) do{if(!(x)){fprintf(stderr,"FAIL owner line%d\n",__LINE__);return 1;}}while(0)
int main(void){
    _Atomic uint32_t owned=1;probe_owner_sample_t s;
    probe_owner_state_init(NULL);probe_owner_state_sample(&s);
    CHECK(s.coherent && s.phase==PROBE_OWNER_UNKNOWN && !s.flags && s.tx_owned==UINT32_MAX);
    probe_owner_state_init(&owned);probe_owner_state_sample(&s);
    CHECK(s.coherent && s.flags==PROBE_OWNER_BOUND && s.tx_owned==1);
    CHECK(probe_owner_state_mark(PROBE_OWNER_PROFILES,UINT32_MAX-2u,0)==PROBE_OWNER_UNKNOWN);
    probe_owner_state_credits(2,1,0);probe_owner_state_sample(&s);
    CHECK(s.coherent && (s.flags&PROBE_OWNER_PENDING) && s.credits_at_us==0 && s.pending_counts==0x10002u);
    uint8_t ncp[]={1,0x35,0x0c,5,1,0x80,0,3,0};
    CHECK(probe_owner_state_completed(ncp,sizeof ncp,0));probe_owner_state_sample(&s);
    CHECK(s.coherent && (s.flags&PROBE_OWNER_NCP) && s.ncp_at_us==0 && s.ncp_packets==3);
    uint32_t version=atomic_load(&owner_state.version);
    for(unsigned n=0;n<sizeof ncp;++n)CHECK(!probe_owner_state_completed(ncp,n,9));
    uint8_t bad[sizeof ncp];for(unsigned i=0;i<sizeof ncp;++i)bad[i]=ncp[i];
    bad[0]=DBT_H4_ACL;CHECK(!probe_owner_state_completed(bad,sizeof bad,9));bad[0]=1;
    bad[1]=0x34;CHECK(!probe_owner_state_completed(bad,sizeof bad,9));bad[1]=0x35;
    bad[3]=4;CHECK(!probe_owner_state_completed(bad,sizeof bad,9));bad[3]=5;
    bad[4]=0;CHECK(!probe_owner_state_completed(bad,sizeof bad,9));bad[4]=DBT_MAX_LINKS+1u;
    CHECK(!probe_owner_state_completed(bad,sizeof bad,9));CHECK(!probe_owner_state_completed(NULL,9,9));
    CHECK(atomic_load(&owner_state.version)==version);
    uint8_t all[5u+4u*DBT_MAX_LINKS]={1,0x35,0x0c,1u+4u*DBT_MAX_LINKS,DBT_MAX_LINKS};
    for(unsigned i=0;i<DBT_MAX_LINKS;++i){all[7u+4u*i]=255;all[8u+4u*i]=255;}
    CHECK(probe_owner_state_completed(all,sizeof all,UINT32_MAX));probe_owner_state_sample(&s);
    CHECK(s.ncp_packets==(uint32_t)DBT_MAX_LINKS*UINT16_MAX && s.ncp_at_us==UINT32_MAX);
    probe_owner_state_mark(PROBE_OWNER_RUNTIME,1,0);probe_owner_state_credits(0,0,2);probe_owner_state_sample(&s);
    CHECK(s.coherent && (s.flags&PROBE_OWNER_IN_RUNTIME) && !(s.flags&PROBE_OWNER_PENDING));
    uint32_t prior=probe_owner_state_mark(PROBE_OWNER_RENDER_YIELD,3,0);CHECK(prior==PROBE_OWNER_RUNTIME);
    probe_owner_state_sample(&s);CHECK(s.phase==PROBE_OWNER_RENDER_YIELD && (s.flags&PROBE_OWNER_IN_RUNTIME));
    probe_owner_state_mark(prior,4,0);probe_owner_state_mark(PROBE_OWNER_PROFILES,5,0);
    probe_owner_state_credits(0,0,6);probe_owner_state_mark(PROBE_OWNER_SLEEP,7,UINT32_MAX);probe_owner_state_sample(&s);
    CHECK(s.coherent && !(s.flags&PROBE_OWNER_IN_RUNTIME) && (s.flags&PROBE_OWNER_PENDING) && !s.pending_counts);
    CHECK(s.wait_ms==UINT32_MAX && s.credits_at_us==6 && s.phase_at_us==7);
    atomic_store(&owned,0);probe_owner_state_sample(&s);CHECK(s.tx_owned==0);
    probe_owner_state_credits(65536u,0,8);probe_owner_state_sample(&s);CHECK(!(s.flags&PROBE_OWNER_PENDING));
    probe_owner_state_credits(0,65536u,8);probe_owner_state_sample(&s);CHECK(!(s.flags&PROBE_OWNER_PENDING));
    atomic_store(&owner_state.version,UINT32_MAX-1u);probe_owner_state_credits(1,2,9);
    CHECK(atomic_load(&owner_state.version)==0);probe_owner_state_sample(&s);CHECK(s.coherent);
    atomic_store(&owner_state.version,1);probe_owner_state_sample(&s);CHECK(!s.coherent && s.flags==PROBE_OWNER_BOUND);
    atomic_store(&owner_state.version,2);mutate=1;unsigned before=hook_calls;probe_owner_state_sample(&s);
    CHECK(hook_calls==before+1u && !s.coherent && s.flags==PROBE_OWNER_BOUND && s.tx_owned==0);
    probe_owner_state_sample(&s);CHECK(s.coherent && s.phase==PROBE_OWNER_PROFILES && s.pending_counts==0x30007u);
    probe_owner_state_sample(NULL);puts("PASS owner phases, runtime invalidation, exact NCP validation/counts, zero/wrap, independent ownership and bounded preemption snapshot");return 0;
}
