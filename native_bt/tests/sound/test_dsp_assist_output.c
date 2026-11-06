/* Actual output task/lifecycle with two mocked FreeRTOS cores. No hardware. */
#define main prior_sound_main
#define xTaskCreatePinnedToCore prior_task_create
#define xTaskNotifyGive prior_task_notify
#define vTaskNotifyGiveFromISR prior_isr_notify
#define ulTaskNotifyTake prior_task_take
#define dbt_output_test_step prior_output_hook
#include "test_probe_sound_output.c"
#undef main
#undef xTaskCreatePinnedToCore
#undef xTaskNotifyGive
#undef vTaskNotifyGiveFromISR
#undef ulTaskNotifyTake
#undef dbt_output_test_step

typedef struct {void (*fn)(void *);void *arg;int core;HANDLE thread,event;_Atomic unsigned notifications;} mock_task_t;
static mock_task_t mock_tasks[2];
static _Thread_local mock_task_t *current_task;
static _Atomic bool mock_shutdown,decoder_hint;
static _Atomic unsigned mock_errors,pending_timed_waits;
static HANDLE hook_release,commit_release;
extern uint32_t dbt_output_test_ready_pcm(void);
int xPortGetCoreID(void){return current_task?current_task->core:0;}
TaskHandle_t xTaskGetCurrentTaskHandle(void){return current_task;}
bool probe_audio_dsp_assist_ready(void){return atomic_load_explicit(&decoder_hint,memory_order_acquire);}
static DWORD WINAPI mock_entry(void *pointer){current_task=pointer;current_task->fn(current_task->arg);return 0;}
int xTaskCreatePinnedToCore(void (*fn)(void *),const char *name,unsigned stack,void *arg,unsigned priority,TaskHandle_t *task,int core){
    (void)name;(void)stack;(void)priority;
    if(core<0 || core>1 || mock_tasks[core].thread)return pdFALSE;
    mock_task_t *t=&mock_tasks[core];t->fn=fn;t->arg=arg;t->core=core;t->event=CreateEventA(NULL,FALSE,FALSE,NULL);
    if(!t->event)return pdFALSE;
    t->thread=CreateThread(NULL,0,mock_entry,t,0,NULL);if(!t->thread)return pdFALSE;*task=t;return pdPASS;
}
void xTaskNotifyGive(TaskHandle_t handle){
    if(handle!=&mock_tasks[0] && handle!=&mock_tasks[1]){atomic_fetch_add(&mock_errors,1);return;}
    mock_task_t *t=handle;atomic_fetch_add_explicit(&t->notifications,1,memory_order_release);SetEvent(t->event);
}
void vTaskNotifyGiveFromISR(TaskHandle_t task,BaseType_t *wake){xTaskNotifyGive(task);*wake=pdTRUE;}
unsigned ulTaskNotifyTake(int clear,unsigned ticks){
    if(!current_task){atomic_fetch_add(&main_waits,1);return 0;}
    if(atomic_load_explicit(&mock_shutdown,memory_order_acquire))ExitThread(0);
    if(current_task->core==1 && (atomic_load(&hold_hook)==4 || atomic_load(&hold_hook)==8) && atomic_load(&hook_reached)==atomic_load(&hold_hook) && ticks!=portMAX_DELAY)atomic_fetch_add(&pending_timed_waits,1);
    if(!atomic_load_explicit(&current_task->notifications,memory_order_acquire))
        WaitForSingleObject(current_task->event,ticks==portMAX_DELAY?INFINITE:ticks);
    if(atomic_load_explicit(&mock_shutdown,memory_order_acquire))ExitThread(0);
    unsigned count=atomic_exchange_explicit(&current_task->notifications,0,memory_order_acq_rel);
    (void)clear;return count;
}
void dbt_output_test_step(unsigned stage){
    if((stage==4 || stage==8) && atomic_load(&hold_hook)==stage){
        atomic_store_explicit(&hook_reached,stage,memory_order_release);
        if(WaitForSingleObject(stage==4?hook_release:commit_release,3000)!=WAIT_OBJECT_0)atomic_fetch_add(&mock_errors,2);
    }
}
static bool hook_ready(unsigned stage){for(unsigned i=0;i<2000;++i){if(atomic_load_explicit(&hook_reached,memory_order_acquire)==stage)return true;Sleep(1);}return false;}
static void release_helper(void){atomic_store(&hold_hook,0);SetEvent(hook_release);}
static dbt_result_t ticket_result(uint32_t token){for(unsigned i=0;i<2000;++i){dbt_result_t r=probe_output_request_poll(token);if(r!=DBT_PENDING)return r;Sleep(1);}return DBT_PENDING;}
static int start_stream(void){
    uint32_t token=0;CHECK(probe_output_stream_begin_primed(true,192000,2,DBT_PCM_S32_LE,false,0,&token)==DBT_PENDING);
    CHECK(ticket_result(token)==DBT_OK);return 0;
}
static int lend_block(unsigned frames){
    int32_t pcm[1024];for(unsigned i=0;i<1024;++i)pcm[i]=(int32_t)(i*65536u);
    atomic_store(&hook_reached,0);atomic_store(&hold_hook,4);ResetEvent(hook_release);
    atomic_store_explicit(&decoder_hint,true,memory_order_release);
    CHECK(probe_output_try_pcm_format(pcm,(uint16_t)frames,2,192000,DBT_PCM_S32_LE)==DBT_OK);
    CHECK(hook_ready(4));CHECK(probe_output_decode_budget()==0 && !dbt_output_test_ready_pcm());probe_dsp_assist_stats_t stats;probe_output_dsp_assist_snapshot(&stats);
    CHECK(stats.state==2 && stats.enabled);return 0;
}
static int stop_stream(void){uint32_t token=0;CHECK(probe_output_stream_begin(false,0,0,DBT_PCM_S16_LE,&token)==DBT_PENDING);CHECK(ticket_result(token)==DBT_OK);CHECK(!atomic_load(&live));return 0;}
int main(void){
    main_id=GetCurrentThreadId();hook_release=CreateEventA(NULL,TRUE,FALSE,NULL);commit_release=CreateEventA(NULL,TRUE,FALSE,NULL);CHECK(hook_release && commit_release);
    CHECK(probe_output_init(external_wake,&render_wakes)==DBT_OK);
    CHECK(!start_stream() && !lend_block(512));unsigned allocations=atomic_load(&live);CHECK(allocations>0);
    uint32_t stop=0;CHECK(probe_output_stream_begin(false,0,0,DBT_PCM_S16_LE,&stop)==DBT_PENDING);
    Sleep(20);CHECK(probe_output_request_poll(stop)==DBT_PENDING && atomic_load(&live)==allocations);
    probe_output_stats_t before;probe_output_stats(&before);CHECK(before.requested_bytes>0 && !before.rendered_frames);
    /* Hold after worker-side publication but before DONE. DMA may consume
     * the new PCM now; lifecycle must still retain every borrowed resource. */
    atomic_store(&hold_hook,8);SetEvent(hook_release);CHECK(hook_ready(8));
    CHECK(dbt_output_test_ready_pcm()==512 && probe_output_decode_budget()==0);
    CHECK(probe_output_request_poll(stop)==DBT_PENDING && atomic_load(&live)==allocations);
    atomic_store(&hold_hook,0);SetEvent(commit_release);
    CHECK(ticket_result(stop)==DBT_OK && !atomic_load(&live));
    probe_dsp_assist_stats_t stats;probe_output_dsp_assist_snapshot(&stats);CHECK(stats.jobs==1 && stats.blocks==4 && stats.frames==512 && stats.max_job_frames==512 && !stats.failures && !stats.state);
    /* Cancel a queued stop while the DSP owns its borrowed span. It must
     * retire the job before acknowledging cancellation, and retain the stream. */
    CHECK(!start_stream() && !lend_block(128));allocations=atomic_load(&live);
    CHECK(probe_output_stream_begin(false,0,0,DBT_PCM_S16_LE,&stop)==DBT_PENDING);
    CHECK(probe_output_request_cancel(stop)==DBT_PENDING);Sleep(20);
    CHECK(probe_output_request_poll(stop)==DBT_PENDING && atomic_load(&live)==allocations);
    release_helper();dbt_result_t cancelled=ticket_result(stop);
    CHECK(cancelled!=DBT_PENDING && cancelled!=DBT_INVALID && cancelled!=DBT_OK && atomic_load(&live)==allocations);
    CHECK(!stop_stream());probe_output_dsp_assist_snapshot(&stats);
    CHECK(stats.jobs==2 && stats.blocks==5 && stats.frames==640 && stats.max_job_frames==512 && !stats.failures && !stats.state);
    printf("main waits=%u blocking sends=%u mock errors=%u timed waits while held=%u\n",atomic_load(&main_waits),atomic_load(&blocking_sends),atomic_load(&mock_errors),atomic_load(&pending_timed_waits));
    CHECK(!atomic_load(&main_waits) && !atomic_load(&blocking_sends) && !atomic_load(&mock_errors) && !atomic_load(&pending_timed_waits));
    probe_output_restart_hold(true);CHECK(probe_output_restart_quiescent());
    atomic_store_explicit(&mock_shutdown,true,memory_order_release);
    for(unsigned i=0;i<2;++i){CHECK(mock_tasks[i].thread);SetEvent(mock_tasks[i].event);CHECK(WaitForSingleObject(mock_tasks[i].thread,2000)==WAIT_OBJECT_0);CloseHandle(mock_tasks[i].thread);CloseHandle(mock_tasks[i].event);}
    CloseHandle(hook_release);CloseHandle(commit_release);
    puts("PASS actual DSP-assisted output: held worker/queued stop/cancel, no early free, no timed pending polling, four-block/single-block leases, worker PCM published before DONE, decoder admission, single completion, both mocked cores, quiescent restart");return 0;
}
