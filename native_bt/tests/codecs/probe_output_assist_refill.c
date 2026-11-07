/* The actual output owner must consume published PCM while the worker retains
 * its DSP/lifecycle borrow. A deterministic interleaving, not target timing. */
#include "probe_output_format.c"
#include <stdio.h>
static unsigned publication_checks,publication_error;
static int32_t *refill_destination;
bool probe_audio_dsp_assist_ready(void){return true;}
void dbt_output_test_step(unsigned stage){
    if(stage!=11 || publication_checks++)return;
    if(dbt_dsp_job_state(&assist.lane)!=DBT_DSP_JOB_BUSY ||
       dbt_audio_output_frames(&output.audio)!=DMA_FRAMES){publication_error=1;return;}
    int32_t expected[DMA_FRAMES*2];
    memcpy(expected,output.audio.output_pages[0],sizeof expected);
    fixed_dsp_t before=output.audio.dsp;
    unsigned allocations=live;
    fixture_core=1;
    fprintf(stderr,"refill interleave: owns=%u callback_core=%u pending=%u deadline=%u now=%u stop=%u failed=%u ready=%u\n",
        owns_output(),atomic_load(&output.callback_core),atomic_load(&output.refill_pending_index),atomic_load(&output.refill_deadline_us),
        (unsigned)test_clock_us,atomic_load(&output.stop_requested),atomic_load(&output.failed),dbt_audio_output_frames(&output.audio));
    bool again=render_step();fixture_core=0;
    if(again || dbt_dsp_job_state(&assist.lane)!=DBT_DSP_JOB_BUSY ||
       atomic_load(&output.refill_recovered_frames)!=DMA_FRAMES ||
       atomic_load(&output.refill_pending_index)!=UINT32_MAX ||
       memcmp(refill_destination,expected,sizeof expected) ||
       memcmp(&before,&output.audio.dsp,sizeof before) || live!=allocations)
        publication_error=2;
}
EX int probe_output_assist_refill(void){
    static int32_t dma[DMA_DESCRIPTORS][DMA_FRAMES*2];
    int32_t input[512*2];for(unsigned i=0;i<512*2;++i)input[i]=0x1000000+(int32_t)i*4000;
    fixture_core=1;CHECK(probe_output_init(0,0)==DBT_OK && ensure_auxiliary_render0());
    CHECK(refill_test_start(dma,10000)==0);
    atomic_store(&output.render_core,1);atomic_store(&output.callback_core,1);
    i2s_event_data_t event={.dma_buf=dma[0],.size=DMA_FRAMES*8u};
    (void)sent(0,&event,&output);
    /* The non-ESP callback fixture records CPU0; place the owner/IRQ context
     * explicitly after that callback, without changing its timing policy. */
    atomic_store(&output.callback_core,1);
    CHECK(atomic_load(&output.refill_pending_index)==0);
    CHECK(probe_output_try_pcm_format(input,512,2,96000,DBT_PCM_S32_LE)==DBT_OK);
    dbt_audio_render_job_t block;
    CHECK(dbt_audio_render_prepare(&output.audio,512,&block)==DBT_OK);
    assist.budget=512;assist.committed_frames=assist.completed_blocks=0;
    CHECK(dbt_dsp_job_submit(&assist.lane,&block)==DBT_OK);
    test_clock_us=10200;refill_destination=dma[0];fixture_core=0;
    CHECK(dsp_assist_work());
    fprintf(stderr,"publication_checks=%u error=%u recovered=%u owner_wakes_while_busy=%u state=%u\n",
        publication_checks,publication_error,atomic_load(&output.refill_recovered_frames),worker_refill_notifications,(unsigned)dbt_dsp_job_state(&assist.lane));
    CHECK(publication_checks==4 && !publication_error && worker_refill_notifications==1);
    CHECK(dbt_dsp_job_state(&assist.lane)==DBT_DSP_JOB_DONE);
    CHECK(dbt_audio_output_frames(&output.audio)==384 && assist.committed_frames==512);
    CHECK(!atomic_load(&output.refill_contract_errors) && !atomic_load(&output.failed));
    fixture_core=1;CHECK(dsp_assist_complete()==1 && dbt_dsp_job_state(&assist.lane)==DBT_DSP_JOB_IDLE);
    close_output();CHECK(!live);return 0;
}
#ifdef DBT_REFILL_STANDALONE
int main(void){int line=probe_output_assist_refill();
    if(line){fprintf(stderr,"FAIL busy-helper published PCM refill line %d\n",line);return 1;}
    puts("PASS actual owner refill during BUSY helper, early wake/publication before DONE, unchanged DSP/lifetime; model interleaving, no target timing");return 0;
}
#endif
