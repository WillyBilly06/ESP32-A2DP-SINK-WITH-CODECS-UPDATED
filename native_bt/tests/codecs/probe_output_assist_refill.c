/* The actual output owner must consume published PCM while the worker retains
 * its DSP/lifecycle borrow. A deterministic interleaving, not target timing. */
#include "probe_output_format.c"
#include <stdio.h>
static unsigned publication_checks,publication_error;
static bool storage_publication_mode;
static unsigned deferred_owner_eofs;
static int32_t *refill_destination;
probe_dsp_admission_t probe_audio_dsp_assist_admission(bool reserve){(void)reserve;return PROBE_DSP_IDLE_ASSIST;}
void dbt_output_test_step(unsigned stage){
    if(stage!=11)return;
    if(storage_publication_mode){
        if(dbt_audio_output_frames(&output.audio)>=3840u){
            int32_t consumed[192*2];
            if(dbt_audio_take(&output.audio,consumed,192)!=192)publication_error=4;
            ++deferred_owner_eofs; /* DMA runs before the owner retires DONE. */
        }
        return;
    }
    ++publication_checks;
    uint32_t ready=dbt_audio_output_frames(&output.audio);
    if(dbt_dsp_job_state(&assist.lane)!=DBT_DSP_JOB_BUSY || !ready){publication_error=1;return;}
    uint32_t valid=atomic_load(&output.descriptors[0].valid);
    uint32_t copy=atomic_load(&output.refill_pending_index)==0?DMA_FRAMES-valid:0;
    if(copy>ready)copy=ready;
    uint32_t read=atomic_load(&output.audio.output.read)%output.audio.config.output_frames;
    if(read+copy>DBT_AUDIO_OUTPUT_PAGE_FRAMES){publication_error=3;return;}
    int32_t expected[DMA_FRAMES*2];
    if(copy)memcpy(expected,output.audio.output_pages[0]+read*8u,copy*8u);
    fixed_dsp_t before=output.audio.dsp;
    unsigned allocations=live,recovered=atomic_load(&output.refill_recovered_frames);
    fixture_core=1;
    bool again=render_step();fixture_core=0;
    uint32_t expected_pending=valid+copy<DMA_FRAMES?0:UINT32_MAX;
    if(again || dbt_dsp_job_state(&assist.lane)!=DBT_DSP_JOB_BUSY ||
       atomic_load(&output.refill_recovered_frames)!=recovered+copy ||
       atomic_load(&output.refill_pending_index)!=expected_pending ||
       (copy && memcmp(refill_destination+valid*2u,expected,copy*8u)) ||
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
    CHECK(publication_checks==4 && !publication_error && worker_refill_notifications==(DMA_FRAMES+FIXED_DSP_BLOCK_FRAMES-1u)/FIXED_DSP_BLOCK_FRAMES);
    CHECK(dbt_dsp_job_state(&assist.lane)==DBT_DSP_JOB_DONE);
    CHECK(dbt_audio_output_frames(&output.audio)==512u-DMA_FRAMES && assist.committed_frames==512 && atomic_load(&output.refill_recovered_frames)==DMA_FRAMES);
    CHECK(!atomic_load(&output.refill_contract_errors) && !atomic_load(&output.failed));
    fixture_core=1;CHECK(dsp_assist_complete()==1 && dbt_dsp_job_state(&assist.lane)==DBT_DSP_JOB_IDLE);
    close_output();CHECK(!live);return 0;
}
static void assist_test_mix(void *p,int32_t *pcm,uint32_t frames){(void)p;(void)pcm;(void)frames;}

static int assisted_storage_publication(void){
    uint32_t cookie=0;static int32_t input[256*2];
    fixture_core=1;test_clock_us=200000;test_clock_step=0;
    CHECK(dbt_storage_reserve_init(&storage_reserve,&storage_ops,NULL)==DBT_OK);
    CHECK(start_output(192000,2,DBT_PCM_S32_LE)==DBT_OK);
    /* The existing test mixer permits deterministic startup silence. */
    dbt_audio_set_mix(&output.audio,assist_test_mix,NULL);
    CHECK(dbt_audio_render_silence(&output.audio,2432)==2432 && enable_output()==DBT_OK);
    atomic_store(&output.callback_calls,1);atomic_store(&output.callback_core,1);
    CHECK(probe_output_storage_prepare(&cookie,7)==DBT_PENDING);(void)render_step();
    CHECK((atomic_load(&storage_reserve.state)&15u)==DBT_STORAGE_ALLOCATE);
    CHECK(probe_output_storage_prepare(&cookie,7)==DBT_PENDING);(void)render_step();
    CHECK(storage_reserve.attached==3 && output.audio.output.frames==4096);
    storage_publication_mode=true;deferred_owner_eofs=0;
    for(unsigned step=0;step<64 && (atomic_load(&storage_reserve.state)&15u)!=DBT_STORAGE_GRANTED;++step){
        if(dbt_audio_input_frames(&output.audio)<=768)CHECK(probe_output_try_pcm_format(input,256,2,192000,DBT_PCM_S32_LE)==DBT_OK);
        fixture_core=1;(void)render_step();
        if(dbt_dsp_job_state(&assist.lane)==DBT_DSP_JOB_READY){fixture_core=0;CHECK(dsp_assist_work());fixture_core=1;(void)render_step();}
    }
    storage_publication_mode=false;
    CHECK(deferred_owner_eofs && !publication_error && (atomic_load(&storage_reserve.state)&15u)==DBT_STORAGE_GRANTED);
    CHECK(dbt_audio_output_frames(&output.audio)<3840 && probe_output_storage_prepare(&cookie,7)==DBT_OK);
    probe_output_storage_release();close_output();(void)render_step();CHECK(probe_output_storage_retire()==DBT_OK && !live);
    puts("PASS assisted reserve publication survives DMA consumption before DONE retirement; host interleaving only");return 0;
}
#ifdef DBT_REFILL_STANDALONE
int main(void){int line=probe_output_assist_refill();if(!line)line=assisted_storage_publication();
    if(line){fprintf(stderr,"FAIL busy-helper published PCM refill line %d\n",line);return 1;}
    puts("PASS actual owner refill during BUSY helper, early wake/publication before DONE, unchanged DSP/lifetime; model interleaving, no target timing");return 0;
}
#endif
