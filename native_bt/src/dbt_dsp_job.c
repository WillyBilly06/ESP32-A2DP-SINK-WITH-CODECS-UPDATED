#include "dbt_dsp_job.h"
_Static_assert(ATOMIC_INT_LOCK_FREE==2,"DSP job state requires lock-free words");
void dbt_dsp_job_init(dbt_dsp_job_lane_t *lane){
    if(!lane)return;
    lane->block=(dbt_audio_render_job_t){0};lane->cycles=lane->started_us=lane->finished_us=0;
    lane->result=DBT_OK;atomic_init(&lane->state,DBT_DSP_JOB_IDLE);
}
dbt_dsp_job_state_t dbt_dsp_job_state(const dbt_dsp_job_lane_t *lane){
    return lane?(dbt_dsp_job_state_t)atomic_load_explicit(&lane->state,memory_order_acquire):DBT_DSP_JOB_IDLE;
}
dbt_result_t dbt_dsp_job_submit(dbt_dsp_job_lane_t *lane,const dbt_audio_render_job_t *block){
    if(!lane || !block || !block->dsp || !block->pcm || !block->token || !block->frames ||
       block->frames>FIXED_DSP_BLOCK_FRAMES || (uintptr_t)block->pcm%4u || (uintptr_t)block->dsp%_Alignof(fixed_dsp_t))return DBT_INVALID;
    uintptr_t pcm=(uintptr_t)block->pcm,dsp=(uintptr_t)block->dsp;
    if(pcm<dsp?dsp-pcm<block->frames*8u:pcm-dsp<sizeof(fixed_dsp_t))return DBT_INVALID;
    if(dbt_dsp_job_state(lane)!=DBT_DSP_JOB_IDLE)return DBT_BUSY;
    /* Only this producer can leave IDLE. The worker never reads the metadata
     * until the release publication below, so no reservation spin is needed. */
    lane->block=*block;lane->cycles=lane->started_us=lane->finished_us=0;lane->result=DBT_PENDING;
    atomic_store_explicit(&lane->state,DBT_DSP_JOB_READY,memory_order_release);return DBT_OK;
}
bool dbt_dsp_job_work(dbt_dsp_job_lane_t *lane,dbt_dsp_job_clock_t clock){
    return dbt_dsp_job_work_commit(lane,clock,0,0);
}
bool dbt_dsp_job_work_commit(dbt_dsp_job_lane_t *lane,dbt_dsp_job_clock_t clock,
    dbt_result_t (*commit)(void *,const dbt_audio_render_job_t *,bool),void *context){
    if(!lane)return false;
    uint32_t expected=DBT_DSP_JOB_READY;
    if(!atomic_compare_exchange_strong_explicit(&lane->state,&expected,DBT_DSP_JOB_BUSY,memory_order_acq_rel,memory_order_relaxed))return false;
    uint32_t began=clock.cycles?clock.cycles(clock.context):0;
    lane->started_us=clock.time_us?clock.time_us(clock.context):0;
#ifdef FIXED_DSP_BLOCK_API
    lane->result=fixed_dsp_process_block(lane->block.dsp,lane->block.pcm,(uint16_t)lane->block.frames)?DBT_OK:DBT_INVALID;
#else
    lane->result=DBT_UNSUPPORTED;
#endif
    lane->cycles=clock.cycles?clock.cycles(clock.context)-began:0;
    lane->finished_us=clock.time_us?clock.time_us(clock.context):0;
    if(commit)lane->result=commit(context,&lane->block,lane->result==DBT_OK);
    /* No dereference of borrowed DSP/PCM or callback context follows this release. */
    atomic_store_explicit(&lane->state,DBT_DSP_JOB_DONE,memory_order_release);return true;
}
dbt_result_t dbt_dsp_job_poll(const dbt_dsp_job_lane_t *lane,const dbt_audio_render_job_t **block,
    uint32_t *cycles,uint32_t *started,uint32_t *finished){
    if(block)*block=0;
    if(!lane)return DBT_INVALID;
    dbt_dsp_job_state_t state=dbt_dsp_job_state(lane);
    if(state==DBT_DSP_JOB_IDLE)return DBT_EMPTY;
    if(state!=DBT_DSP_JOB_DONE)return DBT_PENDING;
    if(block)*block=&lane->block;
    if(cycles)*cycles=lane->cycles;
    if(started)*started=lane->started_us;
    if(finished)*finished=lane->finished_us;
    return lane->result;
}
dbt_result_t dbt_dsp_job_retire(dbt_dsp_job_lane_t *lane,uint32_t token){
    if(!lane || dbt_dsp_job_state(lane)!=DBT_DSP_JOB_DONE || !token || token!=lane->block.token)return DBT_INVALID;
    atomic_store_explicit(&lane->state,DBT_DSP_JOB_IDLE,memory_order_release);return DBT_OK;
}
