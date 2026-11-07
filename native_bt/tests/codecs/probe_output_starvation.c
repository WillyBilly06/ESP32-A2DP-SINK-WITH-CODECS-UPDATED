/* Host-only actual output implementation; scheduler marks are metadata stubs.
 * No target timing is inferred from this deterministic virtual clock. */
#include "probe_output_format.c"
#include <stdio.h>
void probe_scheduler_mark(unsigned core,probe_scheduler_mark_t *m){memset(m,0,sizeof *m);m->core=core;}
void probe_scheduler_interval_observe(unsigned kind,const probe_scheduler_mark_t *a,const probe_scheduler_mark_t *b,
    uint32_t begin,uint32_t end,uint32_t frames,uint32_t serial,uint32_t rate){
    (void)kind;(void)a;(void)b;(void)begin;(void)end;(void)frames;(void)serial;(void)rate;
}
static int startup_observation(void){
    int32_t pcm[512*2]={0};
    test_clock_us=100000;CHECK(start_output(48000,2,DBT_PCM_S32_LE)==DBT_OK);
    atomic_store(&output.priming_state,40u);
    probe_output_stats_t s;probe_output_stats(&s);CHECK(!s.startup_coherent && !s.startup_enable_us);
    CHECK(probe_output_try_pcm_format(pcm,512,2,48000,DBT_PCM_S32_LE)==DBT_OK);
    CHECK(atomic_load(&output.startup_first_pcm_us)==100000u && !priming_ready(139u));
    CHECK(priming_ready(140u));
    CHECK(dbt_audio_render(&output.audio,512)==512);
    test_clock_us=140000;CHECK(enable_output()==DBT_OK);
    probe_output_stats(&s);CHECK(s.startup_coherent && s.startup_first_pcm_us==100000u && s.startup_enable_us==140000u);
    CHECK(s.startup_elapsed_us==40000u && s.startup_preload_frames==512u && !s.startup_input_frames);
    CHECK(s.startup_guard_state==(40u|PRIMING_ARMED|PRIMING_DONE));
    uint32_t v=atomic_load(&output.startup_version);atomic_store(&output.startup_version,v+1u);
    probe_output_stats(&s);CHECK(!s.startup_coherent);atomic_store(&output.startup_version,v+2u);
    close_output();CHECK(!live);
    test_clock_us=(int64_t)UINT32_MAX-10000u;CHECK(start_output(48000,2,DBT_PCM_S32_LE)==DBT_OK);
    probe_output_stats(&s);CHECK(!s.startup_coherent && !s.startup_enable_us);
    CHECK(probe_output_try_pcm_format(pcm,512,2,48000,DBT_PCM_S32_LE)==DBT_OK);
    CHECK(dbt_audio_render(&output.audio,512)==512);
    test_clock_us+=40000;CHECK(enable_output()==DBT_OK);probe_output_stats(&s);
    CHECK(s.startup_coherent && s.startup_elapsed_us==40000u && s.startup_enable_us==29999u);
    close_output();CHECK(!live);return 0;
}
int main(void){
    int line=probe_output_refill_windows();if(!line)line=startup_observation();
    if(line){fprintf(stderr,"FAIL finalized-refill/startup evidence line %d\n",line);return 1;}
    puts("PASS initial-versus-final shortage latches, recovered/canceled exclusion, closure-before-newer-consumption, reset, deadline, startup guard/enable, coherence and clock wrap; host only");
    return 0;
}
