#define DBT_TASK_CYCLES_TEST 1
#define main prior_scheduler_main
#include "probe_scheduler_test.c"
#undef main
#include "../../esp32_probe/main/probe_task_cycles.c"
static unsigned fake_core,fake_isr,mask_level,mask_calls,mask_restores;
unsigned probe_task_test_core(void){return fake_core;}
unsigned probe_task_test_isr(void){return fake_isr;}
uint32_t probe_task_test_cycle(void){return cycles;}
uint32_t probe_task_test_mask(void){unsigned old=mask_level;mask_level=3;++mask_calls;return old;}
void probe_task_test_unmask(uint32_t previous){mask_level=previous;++mask_restores;}
static void cycle_fixture(void){
    fixture();fake_core=fake_isr=0;mask_level=mask_calls=mask_restores=0;
    atomic_store(&cycle_enabled,0);
    for(unsigned c=0;c<2;++c){
        for(unsigned s=0;s<2;++s){atomic_store(&cycle_cores[c].target[s],0);atomic_store(&cycle_cores[c].total[s],0);}
        atomic_store(&cycle_cores[c].current,0);
        atomic_store(&cycle_cores[c].began,0);atomic_store(&cycle_cores[c].faults,0);
    }
    memset(render_accounts,0,sizeof render_accounts);
    memset(&host_account,0,sizeof host_account);
}
int main(void){
    cycle_fixture();probe_task_cycle_mark_t a,b;
    probe_task_cycles_bind(1,task(3));probe_task_cycles_bind_host(task(4));fake_core=1;cycles=100;
    probe_task_cycles_in(1,task(3));probe_task_cycles_mark(1,&a);CHECK(!a.valid && !mask_calls);
    probe_task_cycles_enable();probe_task_cycles_out(1,task(4)); /* Unobserved pre-enable quantum. */
    probe_task_cycles_in(1,task(3));cycles=150;probe_task_cycles_mark(1,&a);
    CHECK(a.valid && a.charged_cycles==50 && a.wall_cycles==150 && a.owner==(uintptr_t)task(3));
    cycles=200;probe_task_cycles_out(1,task(3));cycles=210;probe_task_cycles_in(1,task(4));
    probe_task_cycle_mark_t host_mark;cycles=250;probe_task_cycles_mark_host(&host_mark);
    CHECK(host_mark.valid && host_mark.charged_cycles==40 && host_mark.owner==(uintptr_t)task(4));
    probe_task_cycles_mark(1,&b);CHECK(!b.valid && b.context_known && !b.owner);
    cycles=500;probe_task_cycles_out(1,task(4));cycles=520;probe_task_cycles_in(1,task(3));
    cycles=600;probe_task_cycles_mark(1,&b);
    CHECK(b.valid && b.charged_cycles-a.charged_cycles==130 && b.wall_cycles-a.wall_cycles==450);
    CHECK(mask_calls==4 && mask_restores==4 && !mask_level && !info_calls);
    CHECK(atomic_load(&cycle_cores[1].total[1])==290);
    probe_task_cycles_mark(0,&b);CHECK(!b.valid && mask_calls==4); /* Never read another core's CCOUNT tail. */
    probe_task_cycles_bind(0,task(14));fake_core=0;cycles=UINT32_MAX-20u;
    probe_task_cycles_in(0,task(14));cycles=UINT32_MAX-10u;probe_task_cycles_mark(0,&a);
    cycles=5;probe_task_cycles_out(0,task(14));cycles=6;probe_task_cycles_in(0,task(14));
    cycles=10;probe_task_cycles_mark(0,&b);
    CHECK(a.valid && b.valid && b.charged_cycles-a.charged_cycles==20 && b.wall_cycles-a.wall_cycles==21);
    atomic_store(&cycle_cores[0].total[0],UINT32_MAX-4u);cycles=20;probe_task_cycles_mark(0,&a);
    cycles=30;probe_task_cycles_mark(0,&b);CHECK(b.charged_cycles-a.charged_cycles==10);
    probe_task_cycles_in(0,task(14));probe_task_cycles_mark(0,&b);CHECK(!b.valid);
    uint32_t faults[2];probe_task_cycles_faults(faults);CHECK(faults[0]==1 && !faults[1]);
    probe_task_cycles_out(2,task(14));probe_task_cycles_bind(2,task(14));probe_task_cycles_mark(2,&b);CHECK(!b.valid);

    cycle_fixture();CHECK(probe_scheduler_start(task(1),task(2),task(4))==ESP_OK);fake_core=1;
    probe_scheduler_host_phase(PROBE_HOST_QUERY);probe_host_account_t host_stats;
    probe_scheduler_host_account(&host_stats);CHECK(!host_stats.revision && !host_stats.wrong_context); /* First IN not observed yet. */
    atomic_store(&host_phase,PROBE_HOST_OTHER);cycles=1000;probe_task_cycles_in(1,task(1));
    CHECK(probe_scheduler_host_phase(PROBE_HOST_QUERY)==PROBE_HOST_OTHER);
    cycles=1100;CHECK(probe_scheduler_host_phase(PROBE_HOST_SUBMIT)==PROBE_HOST_QUERY);
    cycles=1150;CHECK(probe_scheduler_host_phase(PROBE_HOST_QUERY)==PROBE_HOST_SUBMIT);
    cycles=1180;CHECK(probe_scheduler_host_phase(PROBE_HOST_OTHER)==PROBE_HOST_QUERY);
    cycles=1190;probe_task_cycles_out(1,task(1));cycles=1200;probe_task_cycles_in(1,task(5));
    cycles=1500;probe_task_cycles_out(1,task(5));cycles=1520;probe_task_cycles_in(1,task(1));
    cycles=1600;probe_scheduler_host_phase(PROBE_HOST_WAIT);probe_scheduler_host_account(&host_stats);
    CHECK(host_stats.coherent && !host_stats.invalid && !host_stats.wrong_context);
    CHECK(host_stats.charged_cycles[PROBE_HOST_QUERY]==130 && host_stats.charged_cycles[PROBE_HOST_SUBMIT]==50 && host_stats.charged_cycles[PROBE_HOST_OTHER]==90);
    CHECK(host_stats.intervals[PROBE_HOST_QUERY]==2 && host_stats.intervals[PROBE_HOST_SUBMIT]==1);
    fake_isr=1;probe_scheduler_host_phase(PROBE_HOST_AUDIO);fake_isr=0;
    probe_scheduler_host_account(&host_stats);CHECK(host_stats.wrong_context==1 && host_stats.charged_cycles[PROBE_HOST_OTHER]==90);
    cycles=1700;probe_scheduler_host_phase(PROBE_HOST_QUERY);probe_scheduler_host_account(&host_stats);
    CHECK(host_stats.charged_cycles[PROBE_HOST_WAIT]==0); /* Wrong-context diagnostic is sticky, no later charging. */

    cycle_fixture();CHECK(probe_scheduler_start(task(1),task(2),task(4))==ESP_OK);
    fake_core=1;cycles=1000;probe_task_cycles_in(1,task(3));
    probe_scheduler_mark_t begin,end;probe_scheduler_mark(1,&begin);
    cycles=begin.task.wall_cycles+500;probe_scheduler_mark(1,&end);
    CHECK(begin.task.valid && end.task.valid);
    probe_scheduler_interval_observe(PROBE_DELAY_LOCAL_RENDER,&begin,&end,10,13,128,7,192000);
    probe_render_account_t stats;probe_scheduler_render_account(PROBE_DELAY_LOCAL_RENDER,&stats);
    uint32_t wall=end.task.wall_cycles-begin.task.wall_cycles;
    CHECK(stats.coherent && stats.calls==1 && !stats.invalid && stats.frames==128 && stats.wall_cycles==wall && stats.charged_cycles==wall);
    probe_scheduler_mark(1,&begin);cycles+=100;probe_task_cycles_out(1,task(3));
    cycles+=10;probe_task_cycles_in(1,task(4));cycles+=200;probe_task_cycles_out(1,task(4));
    cycles+=10;probe_task_cycles_in(1,task(3));cycles+=100;probe_scheduler_mark(1,&end);
    probe_scheduler_interval_observe(PROBE_DELAY_LOCAL_RENDER,&begin,&end,20,22,128,7,192000);
    probe_scheduler_render_account(PROBE_DELAY_LOCAL_RENDER,&stats);
    CHECK(stats.calls==2 && stats.frames==256 && stats.wall_cycles-stats.charged_cycles==220);
    CHECK(stats.charged_at_max_wall<=stats.max_wall_cycles);
    end.task.owner=999;probe_scheduler_interval_observe(PROBE_DELAY_LOCAL_RENDER,&begin,&end,30,31,128,7,192000);
    probe_scheduler_render_account(PROBE_DELAY_LOCAL_RENDER,&stats);CHECK(stats.calls==2 && stats.invalid==1 && stats.frames==256);
    end.task.owner=begin.task.owner;
    probe_scheduler_interval_observe(PROBE_DELAY_LOCAL_RENDER,&begin,&end,40,41,64,8,96000);
    probe_scheduler_render_account(PROBE_DELAY_LOCAL_RENDER,&stats);CHECK(stats.calls==1 && !stats.invalid && stats.frames==64 && stats.stream_serial==8 && stats.rate==96000);
    atomic_store(&render_accounts[1].sequence,UINT32_MAX-1u);
    probe_scheduler_interval_observe(PROBE_DELAY_LOCAL_RENDER,&begin,&end,50,51,64,8,96000);
    probe_scheduler_render_account(PROBE_DELAY_LOCAL_RENDER,&stats);CHECK(stats.coherent && stats.revision==2 && stats.calls==2);
    probe_scheduler_render_account(PROBE_DELAY_ASSIST_RETIRE,&stats);CHECK(!stats.coherent && !stats.calls);
    probe_task_cycles_faults(faults);CHECK(!faults[0] && !faults[1] && mask_calls==mask_restores && !mask_level);
    puts("PASS task cycles: separate host/render identities, nested host phases, pre-enable/ISR/non-owner rejection, sticky context faults, running tail, preemption/selection exclusion, two cores, raw wrap, local-mask restoration, stream totals/maxima; no allocation or stack/task-list scans");
    return 0;
}
