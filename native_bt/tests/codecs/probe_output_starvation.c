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
static int partial_refill(void){
    static int32_t dma[DMA_DESCRIPTORS][DMA_FRAMES*2];
    int32_t input[512*2],expected[DMA_FRAMES*2];
    for(unsigned i=0;i<512*2;++i)input[i]=0x1000000+(int32_t)i*4000;
    CHECK(refill_test_start(dma,80000)==0);
    i2s_event_data_t e={.dma_buf=dma[0],.size=DMA_FRAMES*8u};(void)sent(0,&e,&output);
    CHECK(probe_output_try_pcm_format(input,512,2,96000,DBT_PCM_S32_LE)==DBT_OK);
    CHECK(refill_render_budget(128)==64 && refill_render_budget(32)==32);
    CHECK(dbt_audio_render(&output.audio,64)==64);
    memcpy(expected,output.audio.output_pages[0],64u*8u);test_clock_us=80100;
    CHECK(service_refill_window() && atomic_load(&output.descriptors[0].valid)==64);
    CHECK(atomic_load(&output.refill_pending_index)==0 && atomic_load(&output.refill_recovered_frames)==64);
    CHECK(!memcmp(dma[0],expected,64u*8u) && !service_refill_window());
    CHECK(dbt_audio_render(&output.audio,64)==64);
    memcpy(expected+128,output.audio.output_pages[0]+64u*8u,64u*8u);test_clock_us=80200;
    CHECK(service_refill_window() && atomic_load(&output.refill_pending_index)==UINT32_MAX);
    CHECK(atomic_load(&output.refill_recovered_frames)==128 && !atomic_load(&output.refill_unfilled_frames));
    CHECK(!atomic_load(&output.refill_contract_errors) && !memcmp(dma[0],expected,sizeof expected));
    close_output();CHECK(!live);
    CHECK(refill_test_start(dma,90000)==0);e.dma_buf=dma[0];(void)sent(0,&e,&output);
    CHECK(probe_output_try_pcm_format(input,512,2,96000,DBT_PCM_S32_LE)==DBT_OK);
    CHECK(dbt_audio_render(&output.audio,64)==64);test_clock_us=90100;CHECK(service_refill_window());
    CHECK(dbt_audio_render(&output.audio,64)==64);memcpy(expected,output.audio.output_pages[0]+64u*8u,64u*8u);
    test_clock_us=91333;e.dma_buf=dma[1];(void)sent(0,&e,&output);
    CHECK(atomic_load(&output.refill_unfilled_frames)==64 && !memcmp(dma[1],expected,64u*8u));
    for(unsigned i=128;i<256;++i)CHECK(!dma[0][i]);
    probe_output_starvation_t s;probe_output_first_unfilled(&s);
    CHECK(s.captured && s.coherent && s.taken_frames==64 && s.output_frames==64 && s.at_us==91333);
    close_output();CHECK(!live);
    CHECK(refill_test_start(dma,100000)==0);e.dma_buf=dma[0];(void)sent(0,&e,&output);
    CHECK(probe_output_try_pcm_format(input,512,2,96000,DBT_PCM_S32_LE)==DBT_OK);
    CHECK(dbt_audio_render(&output.audio,64)==64);test_clock_us=100100;CHECK(service_refill_window());
    atomic_store(&output.stop_requested,1);CHECK(!service_refill_window());close_output();
    CHECK(!live && atomic_load(&output.refill_canceled_frames)==64);probe_output_first_unfilled(&s);CHECK(!s.captured);
    return 0;
}
static int fragmented_dsp_equivalence(void){
    int32_t pcm[512*2],reference[256*2],candidate[256*2];
    for(unsigned i=0;i<512*2;++i)pcm[i]=(int32_t)((i*2891336453u+0x632be59bu)&0x3fffffffu)-0x20000000;
    const uint32_t rates[]={48000,192000};
    for(unsigned rate=0;rate<2;++rate)for(unsigned split=0;split<2;++split){
        CHECK(start_output(rates[rate],2,DBT_PCM_S32_LE)==DBT_OK);
        CHECK(dbt_audio_controls(&output.audio,11,8,12,73,1)==DBT_OK);
        CHECK(probe_output_try_pcm_format(pcm,512,2,rates[rate],DBT_PCM_S32_LE)==DBT_OK);
        for(unsigned block=0;block<2;++block){
            if(block)CHECK(dbt_audio_controls(&output.audio,-4,6,5,97,1)==DBT_OK);
            unsigned count=split?64:128;
            for(unsigned offset=0;offset<128;offset+=count){
                CHECK(dbt_audio_render(&output.audio,count)==count);
                CHECK(dbt_audio_take(&output.audio,(split?candidate:reference)+(block*128u+offset)*2u,count)==count);
            }
        }
        close_output();CHECK(!live);if(split)CHECK(!memcmp(reference,candidate,sizeof reference));
    }
    return 0;
}
int main(void){
    int line=probe_output_refill_windows();if(!line)line=startup_observation();
    if(!line)line=partial_refill();if(!line)line=fragmented_dsp_equivalence();
    if(line){fprintf(stderr,"FAIL finalized-refill/startup evidence line %d\n",line);return 1;}
    puts("PASS initial/final latches, partial prefix copies/order/expiry/cancel, urgent budget, exact split-versus-full DSP PCM with EQ/volume changes at48/192k, startup/coherence/wrap; host only");
    return 0;
}
