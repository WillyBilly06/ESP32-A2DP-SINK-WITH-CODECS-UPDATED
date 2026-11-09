/* Real PCM queue + reserve protocol; deterministic owner/event model only. */
#include <stdio.h>
#include <string.h>
#include "../test_audio.c"
#include "dbt_storage_reserve.h"
enum {WORKER=1,RENDER=2,MAIN_OWNER=3};
static dbt_audio_t a;
static dbt_storage_reserve_t reserve;
static unsigned role,role_errors,mask_depth,render_wakes,worker_wakes,io_calls,allocate_calls;
static uint32_t now_us,position,next_sample,cookies[2];
static bool cancel_during_allocation;
static uint8_t *r_allocate(void *ctx,uint32_t bytes){
    (void)ctx;++allocate_calls;if(role!=WORKER || mask_depth)++role_errors;
    uint8_t *p=page_allocate(0,bytes);
    if(cancel_during_allocation){cancel_during_allocation=false;unsigned old=role;role=MAIN_OWNER;dbt_storage_reserve_cancel(&reserve,&cookies[0]);role=old;}
    return p;
}
static void r_release(void *ctx,uint8_t *p){(void)ctx;if(role!=WORKER || mask_depth)++role_errors;page_release(0,p);}
static void r_wake_render(void *ctx){(void)ctx;++render_wakes;}
static void r_wake_worker(void *ctx,uintptr_t waiter){(void)ctx;if(role!=RENDER || waiter!=7u)++role_errors;++worker_wakes;}
static uint32_t r_clock(void *ctx){(void)ctx;return now_us;}
static uint32_t r_enter(void *ctx){(void)ctx;if(role!=RENDER || mask_depth)++role_errors;++mask_depth;return 42;}
static void r_exit(void *ctx,uint32_t mask){(void)ctx;if(role!=RENDER || mask_depth!=1 || mask!=42)++role_errors;--mask_depth;}
static const dbt_storage_reserve_ops_t ops={r_allocate,r_release,r_wake_render,r_wake_worker,r_clock,r_enter,r_exit};
static unsigned stage(void){return atomic_load(&reserve.state)&15u;}
static int setup(uint32_t rate,uint32_t cap,uint32_t target){
    page_reset(0);role=RENDER;role_errors=mask_depth=render_wakes=worker_wakes=io_calls=allocate_calls=0;
    now_us=UINT32_MAX-500u;position=next_sample=0;cancel_during_allocation=false;
    CHECK(dbt_audio_init(&a,(dbt_audio_memory_t){0,page_allocate,page_release})==DBT_OK);
    CHECK(dbt_audio_open(&a,(dbt_audio_config_t){rate,1024,cap,target,DBT_PCM_S32_LE,2,false})==DBT_OK);
    dbt_audio_set_mix(&a,page_mix,&position);
    CHECK(dbt_storage_reserve_init(&reserve,&ops,0)==DBT_OK);return 0;
}
static dbt_result_t prepare(void){role=WORKER;return dbt_storage_reserve_prepare(&reserve,&cookies[0],7);}
static dbt_result_t service(bool running,bool stopping){role=RENDER;return dbt_storage_reserve_render(&reserve,&a,11,running,stopping);}
static int fill(uint32_t frames){role=RENDER;CHECK(dbt_audio_render_silence(&a,frames)==frames);return 0;}
static int consume(uint32_t frames){
    int32_t pcm[192*2];CHECK(!mask_depth);
    while(frames){uint32_t n=frames>192u?192u:frames;CHECK(dbt_audio_take(&a,pcm,n)==n);
        for(unsigned i=0;i<n;++i){int32_t v=(int32_t)(++next_sample);CHECK(pcm[2*i]==v*17 && pcm[2*i+1u]==-v*31);}frames-=n;}
    return 0;
}
static int reach(unsigned wanted){
    CHECK(prepare()==DBT_PENDING);if(wanted==DBT_STORAGE_REQUEST)return 0;
    CHECK(service(true,false)==DBT_OK && stage()==DBT_STORAGE_ALLOCATE);if(wanted==DBT_STORAGE_ALLOCATE)return 0;
    CHECK(prepare()==DBT_PENDING && stage()==DBT_STORAGE_READY);if(wanted==DBT_STORAGE_READY)return 0;
    CHECK(service(true,false)==DBT_OK && stage()==DBT_STORAGE_PREFILL);if(wanted==DBT_STORAGE_PREFILL)return 0;
    CHECK(fill(reserve.target-dbt_audio_output_frames(&a))==0);CHECK(service(true,false)==DBT_OK && stage()==DBT_STORAGE_GRANTED);
    if(wanted==DBT_STORAGE_GRANTED)return 0;
    CHECK(prepare()==DBT_OK && stage()==DBT_STORAGE_RUNNING);++io_calls;
    if(wanted==DBT_STORAGE_RUNNING)return 0;
    role=WORKER;dbt_storage_reserve_release(&reserve);CHECK(stage()==DBT_STORAGE_RELEASE);return 0;
}
static int cleanup_running(void){
    role=WORKER;dbt_storage_reserve_release(&reserve);
    CHECK(consume(dbt_audio_output_frames(&a))==0);CHECK(service(true,false)==DBT_OK && stage()==DBT_STORAGE_DETACHED);
    role=WORKER;CHECK(dbt_storage_reserve_retire(&reserve)==DBT_OK && stage()==DBT_STORAGE_IDLE);
    CHECK(!dbt_storage_reserve_blocks_start(&reserve) && !atomic_load(&reserve.live_bytes));return 0;
}
static int normal_interval(void){
    CHECK(setup(192000,2560,2048)==0 && fill(1024)==0);unsigned before=page_calls;
    CHECK(reach(DBT_STORAGE_GRANTED)==0);CHECK(a.output.frames==4096 && reserve.target==3840 && page_calls==before+3 && !io_calls);
    now_us+=1500;CHECK(prepare()==DBT_OK);++io_calls;CHECK(stage()==DBT_STORAGE_RUNNING && atomic_load(&reserve.max_age_us)==1500);
    /* A 17.383ms cache-off slice consumes ceil(3337.536) actual PCM frames. */
    now_us+=17383;CHECK(consume(3338)==0 && !atomic_load(&a.zero_frames));
    role=WORKER;dbt_storage_reserve_release(&reserve);CHECK(service(true,false)==DBT_OK && stage()==DBT_STORAGE_RELEASE);
    role=WORKER;CHECK(dbt_storage_reserve_retire(&reserve)==DBT_PENDING && page_releases==0);
    CHECK(fill(2048-dbt_audio_output_frames(&a))==0);
    for(unsigned n=0;stage()!=DBT_STORAGE_DETACHED;++n){CHECK(n<4097 && consume(1)==0 && service(true,false)==DBT_OK);}
    role=WORKER;CHECK(dbt_storage_reserve_retire(&reserve)==DBT_OK && a.output.frames==2560 && !atomic_load(&reserve.live_bytes));
    CHECK(consume(dbt_audio_output_frames(&a))==0 && next_sample==position && io_calls==1);
    role=RENDER;dbt_audio_close(&a);dbt_storage_reserve_closed(&reserve);CHECK(page_calls==page_releases && !page_errors && !role_errors);
    return 0;
}
static int no_extra_and_age(void){
    CHECK(setup(48000,1536,1024)==0 && fill(1024)==0);unsigned before=page_calls;
    CHECK(prepare()==DBT_PENDING && service(true,false)==DBT_OK && stage()==DBT_STORAGE_PREFILL);
    CHECK(service(true,false)==DBT_OK && stage()==DBT_STORAGE_GRANTED && !allocate_calls);
    now_us+=2001;CHECK(prepare()==DBT_PENDING && stage()==DBT_STORAGE_PREFILL && atomic_load(&reserve.regrants)==1 && !io_calls);
    CHECK(service(true,false)==DBT_OK);now_us+=2000;CHECK(prepare()==DBT_OK);++io_calls;
    CHECK(page_calls==before && !atomic_load(&reserve.peak_bytes) && atomic_load(&reserve.max_age_us)==2000);
    CHECK(cleanup_running()==0);role=RENDER;dbt_audio_close(&a);dbt_storage_reserve_closed(&reserve);
    CHECK(page_calls==page_releases && !page_errors && !role_errors);return 0;
}
static int allocation_failures(void){
    for(unsigned fail=1;fail<=3;++fail){
        CHECK(setup(192000,2560,2048)==0 && fill(512)==0);unsigned before=page_calls;
        CHECK(prepare()==DBT_PENDING && service(true,false)==DBT_OK);page_fail=page_calls+fail;
        CHECK(prepare()==DBT_PENDING && allocate_calls==fail && page_calls==before+fail);
        CHECK(service(true,false)==DBT_OK && stage()==DBT_STORAGE_DETACHED);
        CHECK(prepare()==DBT_NO_SPACE && !io_calls && !atomic_load(&reserve.live_bytes) && a.output.frames==2560);
        CHECK(consume(512)==0 && position==next_sample);role=RENDER;dbt_audio_close(&a);dbt_storage_reserve_closed(&reserve);
        CHECK(page_calls-1u==page_releases && !page_errors && !role_errors);
    }
    return 0;
}
static int cancellation(void){
    const unsigned points[]={DBT_STORAGE_REQUEST,DBT_STORAGE_ALLOCATE,DBT_STORAGE_READY,DBT_STORAGE_PREFILL,DBT_STORAGE_GRANTED,DBT_STORAGE_RUNNING};
    for(unsigned c=0;c<sizeof points/sizeof points[0];++c){
        CHECK(setup(192000,2560,2048)==0 && fill(512)==0 && reach(points[c])==0);
        uintptr_t owner=atomic_load(&reserve.owner);role=MAIN_OWNER;dbt_storage_reserve_cancel(&reserve,&cookies[1]);CHECK(atomic_load(&reserve.owner)==owner);
        dbt_storage_reserve_cancel(&reserve,&cookies[0]);CHECK(atomic_load(&reserve.owner)&1u);
        if(points[c]==DBT_STORAGE_RUNNING){CHECK(cleanup_running()==0 && io_calls==1);}
        else {
            dbt_result_t result=DBT_PENDING;
            for(unsigned n=0;n<12 && result==DBT_PENDING;++n){
                if(stage()==DBT_STORAGE_RELEASE)CHECK(consume(dbt_audio_output_frames(&a))==0);
                CHECK(service(true,false)==DBT_OK);result=prepare();
            }
            CHECK(result==DBT_INVALID && !io_calls && stage()==DBT_STORAGE_IDLE && !atomic_load(&reserve.live_bytes));
        }
        CHECK(consume(dbt_audio_output_frames(&a))==0 && next_sample==position);role=RENDER;dbt_audio_close(&a);dbt_storage_reserve_closed(&reserve);
        CHECK(page_calls==page_releases && !page_errors && !role_errors);
    }
    CHECK(setup(192000,2560,2048)==0);CHECK(prepare()==DBT_PENDING && service(true,false)==DBT_OK);
    cancel_during_allocation=true;CHECK(prepare()==DBT_PENDING && service(true,false)==DBT_OK && prepare()==DBT_INVALID);
    CHECK(!io_calls && !atomic_load(&reserve.live_bytes));role=RENDER;dbt_audio_close(&a);CHECK(page_calls==page_releases && !page_errors && !role_errors);
    return 0;
}
static int close_each_phase(void){
    const unsigned points[]={DBT_STORAGE_REQUEST,DBT_STORAGE_ALLOCATE,DBT_STORAGE_READY,DBT_STORAGE_PREFILL,DBT_STORAGE_GRANTED,DBT_STORAGE_RUNNING,DBT_STORAGE_RELEASE};
    for(unsigned i=0;i<sizeof points/sizeof points[0];++i){
        CHECK(setup(192000,2560,2048)==0 && reach(points[i])==0);
        role=RENDER;dbt_audio_close(&a);dbt_storage_reserve_closed(&reserve);CHECK(!a.opened);
        if(points[i]==DBT_STORAGE_RUNNING || points[i]==DBT_STORAGE_RELEASE){
            role=WORKER;dbt_storage_reserve_release(&reserve);CHECK(service(false,false)==DBT_OK);role=WORKER;CHECK(dbt_storage_reserve_retire(&reserve)==DBT_OK);
        }else {
            dbt_result_t result=DBT_PENDING;
            for(unsigned n=0;n<10 && result==DBT_PENDING;++n){CHECK(service(false,false)==DBT_OK);result=prepare();}
            CHECK(result==DBT_OK);++io_calls;role=WORKER;dbt_storage_reserve_release(&reserve);CHECK(service(false,false)==DBT_OK);role=WORKER;CHECK(dbt_storage_reserve_retire(&reserve)==DBT_OK);
        }
        CHECK(io_calls==1 && stage()==DBT_STORAGE_IDLE && !atomic_load(&reserve.live_bytes) && page_calls==page_releases && !page_errors && !role_errors);
    }
    return 0;
}
static int borrower_identity_and_fault(void){
    CHECK(setup(192000,2560,2048)==0);CHECK(prepare()==DBT_PENDING);
    a.render_pending=1;CHECK(service(true,false)==DBT_OK && stage()==DBT_STORAGE_REQUEST && !allocate_calls);a.render_pending=0;
    CHECK(service(true,false)==DBT_OK && prepare()==DBT_PENDING);a.render_pending=1;
    CHECK(service(true,false)==DBT_OK && stage()==DBT_STORAGE_READY && a.output.frames==2560);
    role=MAIN_OWNER;dbt_storage_reserve_cancel(&reserve,&cookies[0]);CHECK(service(true,false)==DBT_OK && prepare()==DBT_INVALID);a.render_pending=0;
    role=RENDER;dbt_audio_close(&a);CHECK(page_calls==page_releases && !page_errors && !role_errors);
    CHECK(setup(192000,2560,2048)==0 && reach(DBT_STORAGE_RUNNING)==0);role=WORKER;dbt_storage_reserve_release(&reserve);
    reserve.attached++;CHECK(service(true,false)==DBT_INVALID);role=RENDER;dbt_audio_close(&a);dbt_storage_reserve_closed(&reserve);
    CHECK(service(false,false)==DBT_OK);role=WORKER;CHECK(dbt_storage_reserve_retire(&reserve)==DBT_OK && !atomic_load(&reserve.live_bytes));
    CHECK(page_calls==page_releases && !page_errors && !role_errors);
    atomic_store(&reserve.state,0xfffffff0u);CHECK(prepare()==DBT_NO_SPACE && !dbt_storage_reserve_blocks_start(&reserve));
    return 0;
}
static int delayed_publication(void){
    CHECK(setup(192000,2560,2048)==0 && reach(DBT_STORAGE_PREFILL)==0);
    CHECK(fill(reserve.target)==0);dbt_storage_reserve_published(&reserve,&a,12);
    CHECK(!atomic_load(&reserve.published_word) && consume(192)==0);
    CHECK(service(true,false)==DBT_OK && stage()==DBT_STORAGE_PREFILL);
    CHECK(fill(192)==0);dbt_storage_reserve_published(&reserve,&a,11);CHECK(consume(192)==0);
    now_us+=2001;CHECK(service(true,false)==DBT_OK && stage()==DBT_STORAGE_PREFILL);
    CHECK(fill(192)==0);dbt_storage_reserve_published(&reserve,&a,11);CHECK(consume(192)==0);
    now_us+=1000;CHECK(service(true,false)==DBT_OK && stage()==DBT_STORAGE_GRANTED);
    now_us+=500;CHECK(prepare()==DBT_OK && atomic_load(&reserve.max_age_us)==1500);
    uint32_t old=atomic_load(&reserve.published_word);CHECK(cleanup_running()==0);
    CHECK(reach(DBT_STORAGE_PREFILL)==0);atomic_store(&reserve.published_us,now_us);atomic_store(&reserve.published_word,old);
    CHECK(service(true,false)==DBT_OK && stage()==DBT_STORAGE_PREFILL); /* Ticket cannot replay. */
    role=RENDER;dbt_audio_close(&a);dbt_storage_reserve_closed(&reserve);
    CHECK(service(false,false)==DBT_OK && prepare()==DBT_OK);role=WORKER;dbt_storage_reserve_release(&reserve);
    CHECK(service(false,false)==DBT_OK);role=WORKER;CHECK(dbt_storage_reserve_retire(&reserve)==DBT_OK);
    CHECK(page_calls==page_releases && !role_errors && !page_errors);return 0;
}
int main(void){
    int a1=normal_interval(),a2=no_extra_and_age(),a3=allocation_failures(),a4=cancellation(),a5=close_each_phase(),a6=borrower_identity_and_fault(),a7=delayed_publication();
    printf("reserve real-PCM interval=%d existing-capacity/expiry/wrap=%d OOM=%d cancellation=%d close phases=%d borrower/fault/exhaustion=%d\n",a1,a2,a3,a4,a5,a6);
    printf("assisted publication/age/epoch/ticket=%d\n",a7);return a1||a2||a3||a4||a5||a6||a7;
}
