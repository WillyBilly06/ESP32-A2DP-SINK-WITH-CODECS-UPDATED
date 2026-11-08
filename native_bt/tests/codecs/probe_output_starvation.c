/* Host-only actual output implementation; scheduler marks are metadata stubs.
 * No target timing is inferred from this deterministic virtual clock. */
#include "probe_output_format.c"
#include <stdio.h>
#ifdef DBT_PROBE_SCHEDULER_TRACE
void probe_scheduler_mark(unsigned core,probe_scheduler_mark_t *m){memset(m,0,sizeof *m);m->core=core;}
void probe_scheduler_interval_observe(unsigned kind,const probe_scheduler_mark_t *a,const probe_scheduler_mark_t *b,
    uint32_t begin,uint32_t end,uint32_t frames,uint32_t serial,uint32_t rate){
    (void)kind;(void)a;(void)b;(void)begin;(void)end;(void)frames;(void)serial;(void)rate;
}
#endif
#ifdef PROBE_OWNER_STATE_TRACE
static _Atomic uint32_t fixture_tx_owned;
#endif
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
    test_clock_us=200000;CHECK(start_output(192000,2,DBT_PCM_S32_LE)==DBT_OK);
    atomic_store(&output.priming_state,50u);
    CHECK(probe_output_try_pcm_format(pcm,512,2,192000,DBT_PCM_S32_LE)==DBT_OK);
    CHECK(!priming_ready(249u) && priming_ready(250u));
    CHECK(dbt_audio_render(&output.audio,512)==512);
    test_clock_us=250000;CHECK(enable_output()==DBT_OK);probe_output_stats(&s);
    CHECK(s.startup_coherent && s.startup_elapsed_us==50000u && s.startup_encoded_slots==UINT32_MAX);
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
    const unsigned half=DMA_FRAMES/2u;
    int32_t input[512*2],expected[DMA_FRAMES*2];
    for(unsigned i=0;i<512*2;++i)input[i]=0x1000000+(int32_t)i*4000;
    CHECK(refill_test_start(dma,80000)==0);
    i2s_event_data_t e={.dma_buf=dma[0],.size=DMA_FRAMES*8u};(void)sent(0,&e,&output);
    CHECK(probe_output_try_pcm_format(input,512,2,96000,DBT_PCM_S32_LE)==DBT_OK);
    CHECK(refill_render_budget(128)==64u && refill_render_budget(32)==32);
    CHECK(dbt_audio_render(&output.audio,half)==half);
    memcpy(expected,output.audio.output_pages[0],half*8u);test_clock_us=80100;
    CHECK(service_refill_window() && atomic_load(&output.descriptors[0].valid)==half);
    CHECK(atomic_load(&output.refill_pending_index)==0 && atomic_load(&output.refill_recovered_frames)==half);
    CHECK(!memcmp(dma[0],expected,half*8u) && !service_refill_window());
    CHECK(dbt_audio_render(&output.audio,half)==half);
    memcpy(expected+half*2u,output.audio.output_pages[0]+half*8u,half*8u);test_clock_us=80200;
    CHECK(service_refill_window() && atomic_load(&output.refill_pending_index)==UINT32_MAX);
    CHECK(atomic_load(&output.refill_recovered_frames)==DMA_FRAMES && !atomic_load(&output.refill_unfilled_frames));
    CHECK(!atomic_load(&output.refill_contract_errors) && !memcmp(dma[0],expected,sizeof expected));
    close_output();CHECK(!live);
    CHECK(refill_test_start(dma,90000)==0);
#ifdef PROBE_OWNER_STATE_TRACE
    atomic_store(&fixture_tx_owned,1);probe_owner_state_init(&fixture_tx_owned);
    probe_owner_state_mark(PROBE_OWNER_PROFILES,88000,0);probe_owner_state_credits(2,1,88010);
    const uint8_t owner_ncp[]={1,0x35,0x0c,5,1,0x80,0,3,0};
    CHECK(probe_owner_state_completed(owner_ncp,sizeof owner_ncp,88020));
    probe_owner_state_mark(PROBE_OWNER_SLEEP,88030,UINT32_MAX);
#endif
    probe_output_trace_media_arrival(90000,90010,99,42,960,255,DBT_OK,true,89900,UINT32_MAX-7u,0x2001u);
    e.dma_buf=dma[0];(void)sent(0,&e,&output);
    CHECK(probe_output_try_pcm_format(input,512,2,96000,DBT_PCM_S32_LE)==DBT_OK);
    CHECK(dbt_audio_render(&output.audio,half)==half);test_clock_us=90100;CHECK(service_refill_window());
    CHECK(dbt_audio_render(&output.audio,half)==half);memcpy(expected,output.audio.output_pages[0]+half*8u,half*8u);
    test_clock_us=91333;e.dma_buf=dma[1];(void)sent(0,&e,&output);
    CHECK(atomic_load(&output.refill_unfilled_frames)==half && !memcmp(dma[1],expected,half*8u));
    for(unsigned i=half*2u;i<DMA_FRAMES*2u;++i)CHECK(!dma[0][i]);
    probe_output_starvation_t s;probe_output_first_unfilled(&s);
    CHECK(s.captured && s.coherent && s.taken_frames==half && s.output_frames==half && s.at_us==91333);
    CHECK(s.media_coherent && s.media_arrival_valid && s.media_arrival_us==89900u && s.media_arrival_cursor==UINT32_MAX-7u && s.media_arrival_flags==0x2001u);
#ifdef PROBE_OWNER_STATE_TRACE
    CHECK(s.owner.coherent && s.owner.phase==PROBE_OWNER_SLEEP && s.owner.phase_at_us==88030 && s.owner.wait_ms==UINT32_MAX);
    CHECK(s.owner.credits_at_us==88010 && s.owner.pending_counts==0x10002u && (s.owner.flags&PROBE_OWNER_PENDING));
    CHECK((s.owner.flags&PROBE_OWNER_NCP) && s.owner.ncp_at_us==88020 && s.owner.ncp_packets==3 && s.owner.tx_owned==1);
    atomic_store(&fixture_tx_owned,0);probe_owner_state_mark(PROBE_OWNER_LOOP,92000,0);probe_owner_state_credits(0,0,92001);
#endif
    probe_output_trace_media(92000,92001,100,43,1920,255,DBT_OK);
    probe_output_first_unfilled(&s);
    CHECK(s.media_epoch==99 && s.media_sequence==42 && s.media_arrival_valid && s.media_arrival_us==89900u);
#ifdef PROBE_OWNER_STATE_TRACE
    CHECK(s.owner.coherent && s.owner.phase==PROBE_OWNER_SLEEP && s.owner.pending_counts==0x10002u && s.owner.tx_owned==1);
#endif
    close_output();CHECK(!live);
    CHECK(refill_test_start(dma,100000)==0);e.dma_buf=dma[0];(void)sent(0,&e,&output);
    CHECK(probe_output_try_pcm_format(input,512,2,96000,DBT_PCM_S32_LE)==DBT_OK);
    CHECK(dbt_audio_render(&output.audio,half)==half);test_clock_us=100100;CHECK(service_refill_window());
    atomic_store(&output.stop_requested,1);CHECK(!service_refill_window());close_output();
    CHECK(!live && atomic_load(&output.refill_canceled_frames)==half);probe_output_first_unfilled(&s);CHECK(!s.captured && s.pending_acl_metadata==UINT32_MAX && s.vhci_owned_now==UINT32_MAX);
    return 0;
}
static int refill_before_diagnostics(void){
    static int32_t dma[DMA_DESCRIPTORS][DMA_FRAMES*2];int32_t pcm[512*2]={0};
    CHECK(refill_test_start(dma,120000)==0);
    i2s_event_data_t e={.dma_buf=dma[0],.size=DMA_FRAMES*8u};(void)sent(0,&e,&output);
    CHECK(probe_output_try_pcm_format(pcm,512,2,96000,DBT_PCM_S32_LE)==DBT_OK);
    CHECK(dbt_audio_render(&output.audio,64)==64);
    uint32_t deadline=atomic_load(&output.refill_deadline_us);
    /* One unrelated timer read consumes this last safe admission microsecond.
     * The actual masked copy still has its full unchanged 32us allowance. */
    test_clock_us=(int64_t)deadline-REFILL_COPY_BUDGET_US-2;test_clock_step=1;
    render_completed((uint32_t)test_clock_us-100u,true);test_clock_step=0;
    CHECK(atomic_load(&output.descriptors[0].valid)==64u && atomic_load(&output.refill_recovered_frames)==64u);
    CHECK(!atomic_load(&output.refill_contract_errors) && !atomic_load(&output.failed));
    close_output();CHECK(!live);return 0;
}
static int bounded_refill_burst(void){
    static int32_t dma[DMA_DESCRIPTORS][DMA_FRAMES*2];int32_t pcm[512*2]={0};
    CHECK(refill_test_start(dma,130000)==0);
    i2s_event_data_t e={.dma_buf=dma[0],.size=DMA_FRAMES*8u};(void)sent(0,&e,&output);
    CHECK(probe_output_try_pcm_format(pcm,512,2,96000,DBT_PCM_S32_LE)==DBT_OK);
    unsigned steps=0,consecutive=0;
    /* Even persistent work/new EOFs cannot remove the two-extra-step cap. */
    for(unsigned i=0;i<3000;++i){
      bool keep=refill_burst_continue(&steps);
      if(keep){++consecutive;CHECK(consecutive<=2u);}else{CHECK(consecutive==2u);consecutive=0;}
    }
    CHECK(!steps && !consecutive);
    CHECK(refill_burst_continue(&steps));atomic_store(&output.stop_requested,1);CHECK(!refill_burst_continue(&steps) && !steps);atomic_store(&output.stop_requested,0);
    CHECK(refill_burst_continue(&steps));atomic_store(&output.failed,1);CHECK(!refill_burst_continue(&steps) && !steps);atomic_store(&output.failed,0);
    CHECK(refill_burst_continue(&steps));output.stopping=true;CHECK(!refill_burst_continue(&steps) && !steps);output.stopping=false;
    uint32_t deadline=atomic_load(&output.refill_deadline_us);test_clock_us=deadline-REFILL_COPY_BUDGET_US;
    CHECK(!refill_burst_continue(&steps) && !steps);
    close_output();CHECK(!live);
    CHECK(refill_test_start(dma,140000)==0);(void)sent(0,&e,&output);
    CHECK(!refill_burst_continue(&steps)); /* Empty producer is an event wait. */
    atomic_store(&output.refill_pending_index,UINT32_MAX);CHECK(!refill_burst_continue(&steps));
    close_output();CHECK(!live);return 0;
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
    if(!line)line=refill_before_diagnostics();
    if(!line)line=bounded_refill_burst();
    if(!line)line=partial_refill();if(!line)line=fragmented_dsp_equivalence();
    if(line){fprintf(stderr,"FAIL finalized-refill/startup evidence line %d\n",line);return 1;}
    puts("PASS initial/final latches, partial prefix copies/order/expiry/cancel, urgent budget, exact split-versus-full DSP PCM with EQ/volume changes at48/192k, startup/coherence/wrap; host only");
    return 0;
}
