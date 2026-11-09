#include "probe_coex.h"
#include "esp_coexist.h"
#include <stdio.h>
#include <string.h>
#define CHECK(x) do {if(!(x))return __LINE__;}while(0)
static probe_coex_t s;
static unsigned calls,notes,failures,queued,completed,violation,op,bits;
static int sdk_result;static uint32_t clock_us;static bool reenter_owner;
int64_t esp_timer_get_time(void){uint32_t result=clock_us;clock_us+=17;return result;}
static int sdk(unsigned operation,esp_coex_status_type_t type,uint32_t status){
    ++calls;op=operation;bits=status;
    if(type!=ESP_COEX_ST_TYPE_BT || (atomic_load(&s.work_state)&255u)!=PROBE_COEX_WORK_RUNNING)++violation;
    if(probe_coex_worker_step(&s,0))++violation;
    if(reenter_owner){reenter_owner=false;probe_coex_poll(&s,true,false);
        if(!s.pending || probe_coex_wait_ms(&s,true,false)!=UINT32_MAX)++violation;}
    return sdk_result;
}
esp_err_t esp_coex_status_bit_set(esp_coex_status_type_t type,uint32_t status){return sdk(1,type,status);}
esp_err_t esp_coex_status_bit_clear(esp_coex_status_type_t type,uint32_t status){return sdk(0,type,status);}
static void note(const probe_coex_t *p,probe_coex_stage_t stage,int32_t result){
    ++notes;if(p!=&s)++violation;
    if(stage==PROBE_COEX_QUEUED){++queued;
        if(result!=INT32_MIN || !s.pending || atomic_load(&s.work_state)!=PROBE_COEX_WORK_IDLE ||
           probe_coex_worker_step(&s,0))++violation;
    }else if(stage==PROBE_COEX_COMPLETED){++completed;
        if(result || (atomic_load(&s.work_state)&255u)!=PROBE_COEX_WORK_DONE)++violation;
    }else {++failures;if(!result || !s.failed)++violation;}
}
static void reset(void){s=(probe_coex_t){.note=note};calls=notes=failures=queued=completed=violation=op=bits=0;
    sdk_result=0;clock_us=100;reenter_owner=false;}
static int settle(bool streaming){
    for(unsigned i=0;i<4;++i){probe_coex_poll(&s,true,streaming);
        if(s.pending){CHECK(probe_coex_worker_step(&s,1));CHECK(!probe_coex_worker_step(&s,0));}
        else break;}
    CHECK(!s.pending && !probe_coex_work_pending(&s) && !violation && !s.failed);
    CHECK(probe_coex_wait_ms(&s,true,streaming)==UINT32_MAX);return 0;
}
static int transitions(void){
    reset();CHECK(probe_coex_wait_ms(&s,true,false)==0);CHECK(settle(false)==0);
    CHECK(calls==2 && notes==4 && s.known==0x30 && !s.applied && op==0 && bits==0x20);
    CHECK(atomic_load(&s.state_snapshot)==0x30u);
    unsigned n=notes;probe_coex_poll(&s,true,false);CHECK(notes==n);
    CHECK(settle(true)==0 && calls==4 && s.known==0x30 && s.applied==0x10);
    CHECK(atomic_load(&s.state_snapshot)==0x1030u);
    CHECK(settle(false)==0 && calls==6 && !s.applied && completed==6 && queued==6);
    return 0;
}
static int asynchronous_latest(void){
    reset();probe_coex_poll(&s,false,true);CHECK(!s.pending && !calls && !notes);
    CHECK(probe_coex_wait_ms(&s,false,true)==UINT32_MAX);
    probe_coex_poll(&s,true,true);uint32_t token=s.token;
    CHECK(s.pending && !calls && queued==1 && probe_coex_work_pending(&s));
    CHECK(atomic_load(&s.state_snapshot)==0x10000u);
    for(unsigned i=0;i<20;++i)probe_coex_poll(&s,true,(i&1u)!=0);
    CHECK(!calls && s.token==token && queued==1 && probe_coex_wait_ms(&s,true,false)==UINT32_MAX);
    reenter_owner=true;CHECK(probe_coex_worker_step(&s,1));
    CHECK(calls==1 && op==1 && bits==0x10 && s.pending && !s.known && !s.applied);
    CHECK(atomic_load(&s.state_snapshot)==0x10000u); /* Worker cannot claim owner confirmation. */
    CHECK(s.started_us==100 && s.finished_us==117 && (atomic_load(&s.work_state)>>8)==1);
    CHECK(probe_coex_work_pending(&s) && probe_coex_wait_ms(&s,true,false)==0);
    probe_coex_poll(&s,true,false);
    CHECK(s.applied==0x10 && s.known==0x10 && s.pending && s.token!=token && calls==1);
    CHECK(probe_coex_worker_step(&s,0) && op==0 && bits==0x10);
    probe_coex_poll(&s,true,false);CHECK(probe_coex_worker_step(&s,1) && bits==0x20);
    probe_coex_poll(&s,true,false);CHECK(!s.pending && !s.applied && !violation);
    return 0;
}
static int errors_and_lifetime(void){
    reset();probe_coex_poll(&s,true,true);sdk_result=-1;
    CHECK(probe_coex_worker_step(&s,1));probe_coex_poll(&s,true,true);
    CHECK(s.failed && !s.pending && !s.known && !s.applied && failures==1 && !probe_coex_work_pending(&s));
    CHECK(atomic_load(&s.state_snapshot)==0x20000u);
    unsigned n=notes;probe_coex_poll(&s,true,false);CHECK(notes==n && calls==1 && probe_coex_wait_ms(&s,true,false)==UINT32_MAX);
    reset();s.token=UINT32_MAX;probe_coex_poll(&s,true,true);
    CHECK(s.failed && !s.pending && failures==1 && !calls && !violation && !probe_coex_work_pending(&s));
    reset();probe_coex_poll(&s,true,true);CHECK(probe_coex_worker_step(&s,1));
    /* Controller-ready admission may close, but a borrowed job still retires. */
    probe_coex_poll(&s,false,false);CHECK(!s.pending && s.applied==0x10 && s.known==0x10 && !violation);
    CHECK(settle(false)==0 && !s.applied);
    return 0;
}
static int time_wrap_and_empty(void){
    reset();clock_us=0xfffffff8u;probe_coex_poll(&s,true,true);
    CHECK(probe_coex_worker_step(&s,1));CHECK(s.started_us==0xfffffff8u && s.finished_us==9u);
    CHECK(s.finished_us-s.started_us==17u);probe_coex_poll(&s,true,true);
    CHECK(settle(true)==0 && !violation);
    CHECK(!probe_coex_worker_step(0,1) && !probe_coex_work_pending(0));
    CHECK(probe_coex_wait_ms(0,true,true)==UINT32_MAX);probe_coex_poll(0,true,true);
    return 0;
}
int main(void){
    int (*cases[])(void)={transitions,asynchronous_latest,errors_and_lifetime,time_wrap_and_empty};
    for(unsigned i=0;i<sizeof(cases)/sizeof(cases[0]);++i){int line=cases[i]();
        if(line){fprintf(stderr,"FAIL coex mailbox case%u line%d\n",i,line);return 1;}}
    puts("PASS actual coex mailbox: SDK dispatch on worker only, latest intent, immutable job, claim once, completion/error, lifetime, token exhaustion and timer wrap; mocked SDK, no RF or multithread proof");return 0;
}
