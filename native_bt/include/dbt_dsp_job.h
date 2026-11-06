#ifndef DBT_DSP_JOB_H
#define DBT_DSP_JOB_H
#include "dbt_audio.h"
typedef enum {DBT_DSP_JOB_IDLE=0,DBT_DSP_JOB_READY,DBT_DSP_JOB_BUSY,DBT_DSP_JOB_DONE} dbt_dsp_job_state_t;
typedef struct {
    _Atomic uint32_t state;
    dbt_audio_render_job_t block;
    uint32_t cycles,started_us,finished_us;
    dbt_result_t result;
} dbt_dsp_job_lane_t;
typedef struct {
    void *context;
    uint32_t (*cycles)(void *);
    uint32_t (*time_us)(void *);
} dbt_dsp_job_clock_t;
/* One producer/retirer, one task-context worker, one immutable borrowed block.
 * Initialize before either owner starts. No allocation, waits or retry loops.
 * Submit publishes metadata; work claims it once and publishes completion only
 * after its final DSP/PCM access. Finish the audio reservation before retire. */
void dbt_dsp_job_init(dbt_dsp_job_lane_t *);
dbt_result_t dbt_dsp_job_submit(dbt_dsp_job_lane_t *,const dbt_audio_render_job_t *);
bool dbt_dsp_job_work(dbt_dsp_job_lane_t *,dbt_dsp_job_clock_t);
/* Optional bounded commit runs on the borrower after DSP, while still BUSY.
 * It must retire the audio reservation even if math failed. DONE releases all
 * callback accesses as well as DSP/PCM accesses; the owner may then close. */
bool dbt_dsp_job_work_commit(dbt_dsp_job_lane_t *,dbt_dsp_job_clock_t,
    dbt_result_t (*commit)(void *,const dbt_audio_render_job_t *,bool),void *);
dbt_dsp_job_state_t dbt_dsp_job_state(const dbt_dsp_job_lane_t *);
dbt_result_t dbt_dsp_job_poll(const dbt_dsp_job_lane_t *,const dbt_audio_render_job_t **,
    uint32_t *cycles,uint32_t *started_us,uint32_t *finished_us);
dbt_result_t dbt_dsp_job_retire(dbt_dsp_job_lane_t *,uint32_t token);
#endif
