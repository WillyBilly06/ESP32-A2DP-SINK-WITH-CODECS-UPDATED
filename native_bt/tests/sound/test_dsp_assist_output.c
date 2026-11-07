/* Actual output task/lifecycle with two mocked FreeRTOS cores. No hardware. */
#include "../../esp32_probe/main/probe_schedule_limits.h"
#define taskYIELD prior_task_yield
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
#undef taskYIELD

typedef struct {void (*fn)(void *);void *arg;int core;unsigned priority;HANDLE thread,event;_Atomic unsigned notifications;} mock_task_t;
static mock_task_t mock_tasks[2];
static _Thread_local mock_task_t *current_task;
static _Atomic bool mock_shutdown,decoder_hint;
static _Atomic unsigned mock_errors,pending_timed_waits,yields[2];
static _Atomic unsigned sound_progress_wakes;
static HANDLE producer_release;
static _Atomic bool observe_completion_order,hold_producer_wake;
static _Atomic unsigned owner_completion_signals,producer_hold_reached;
extern uint32_t dbt_output_test_ready_pcm(void);
static HANDLE yield_release;
static _Atomic unsigned yield_job_gate,yield_ready_frames;
void taskYIELD(void){
    if(current_task){
        atomic_fetch_add(&yields[current_task->core],1);
        unsigned gate=atomic_load_explicit(&yield_job_gate,memory_order_acquire);
        if(current_task->core==1 && gate){
            probe_dsp_assist_stats_t stats;probe_output_dsp_assist_snapshot(&stats);
            if(stats.jobs>=gate && atomic_exchange(&yield_job_gate,0)){
                atomic_store_explicit(&yield_ready_frames,dbt_output_test_ready_pcm(),memory_order_release);
                if(WaitForSingleObject(yield_release,3000)!=WAIT_OBJECT_0)atomic_fetch_add(&mock_errors,2);
            }
        }
    }
    SwitchToThread();
}
static HANDLE hook_release,commit_release,input_release;
extern uint32_t dbt_output_test_ready_pcm(void);
int xPortGetCoreID(void){return current_task?current_task->core:0;}
TaskHandle_t xTaskGetCurrentTaskHandle(void){return current_task;}
bool probe_audio_dsp_assist_ready(void){return atomic_load_explicit(&decoder_hint,memory_order_acquire);}
static DWORD WINAPI mock_entry(void *pointer){current_task=pointer;current_task->fn(current_task->arg);return 0;}
int xTaskCreatePinnedToCore(void (*fn)(void *),const char *name,unsigned stack,void *arg,unsigned priority,TaskHandle_t *task,int core){
    (void)name;(void)stack;
    if(core<0 || core>1 || mock_tasks[core].thread)return pdFALSE;
    mock_task_t *t=&mock_tasks[core];t->fn=fn;t->arg=arg;t->core=core;t->priority=priority;t->event=CreateEventA(NULL,FALSE,FALSE,NULL);
    if(!t->event)return pdFALSE;
    t->thread=CreateThread(NULL,0,mock_entry,t,0,NULL);if(!t->thread)return pdFALSE;*task=t;return pdPASS;
}
void xTaskNotifyGive(TaskHandle_t handle){
    if(handle!=&mock_tasks[0] && handle!=&mock_tasks[1]){atomic_fetch_add(&mock_errors,1);return;}
    mock_task_t *t=handle;
    if(current_task && current_task->core==0 && t==&mock_tasks[1] && atomic_load(&observe_completion_order))
        atomic_fetch_add(&owner_completion_signals,1);
    atomic_fetch_add_explicit(&t->notifications,1,memory_order_release);SetEvent(t->event);
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
    if((stage==4 || stage==8 || stage==10) && atomic_load(&hold_hook)==stage){
        atomic_store_explicit(&hook_reached,stage,memory_order_release);
        if(WaitForSingleObject(stage==4?hook_release:stage==8?commit_release:input_release,3000)!=WAIT_OBJECT_0)atomic_fetch_add(&mock_errors,2);
    }
}
static bool hook_ready(unsigned stage){for(unsigned i=0;i<2000;++i){if(atomic_load_explicit(&hook_reached,memory_order_acquire)==stage)return true;Sleep(1);}return false;}
static void release_helper(void){atomic_store(&hold_hook,0);SetEvent(hook_release);}
static void decoder_wake_with_preemption(void *context){
    external_wake(context);
    if(current_task && current_task->core==0 && atomic_exchange(&hold_producer_wake,false)){
        /* Model higher-priority decoder execution suspending this helper's
         * continuation at the notification. Core1 must already be signaled. */
        atomic_store_explicit(&producer_hold_reached,1,memory_order_release);
        if(WaitForSingleObject(producer_release,3000)!=WAIT_OBJECT_0)atomic_fetch_add(&mock_errors,2);
    }
}
static dbt_result_t ticket_result(uint32_t token){for(unsigned i=0;i<2000;++i){dbt_result_t r=probe_output_request_poll(token);if(r!=DBT_PENDING)return r;Sleep(1);}return DBT_PENDING;}
static int start_stream(void){
    uint32_t token=0;CHECK(probe_output_stream_begin_primed(true,192000,2,DBT_PCM_S32_LE,false,0,&token)==DBT_PENDING);
    CHECK(ticket_result(token)==DBT_OK);return 0;
}
static int lend_block(unsigned frames){
    int32_t pcm[2048];for(unsigned i=0;i<2048;++i)pcm[i]=(int32_t)(i*65536u);
    atomic_store(&hook_reached,0);atomic_store(&hold_hook,4);ResetEvent(hook_release);
    atomic_store_explicit(&decoder_hint,true,memory_order_release);
    CHECK(probe_output_try_pcm_format(pcm,(uint16_t)frames,2,192000,DBT_PCM_S32_LE)==DBT_OK);
    CHECK(hook_ready(4));CHECK(probe_output_decode_budget()==0 && !dbt_output_test_ready_pcm());probe_dsp_assist_stats_t stats;probe_output_dsp_assist_snapshot(&stats);
    CHECK(stats.state==2 && stats.enabled);return 0;
}
static int stop_stream(void){uint32_t token=0;CHECK(probe_output_stream_begin(false,0,0,DBT_PCM_S16_LE,&token)==DBT_PENDING);CHECK(ticket_result(token)==DBT_OK);CHECK(!atomic_load(&live));return 0;}
int main(void){
    main_id=GetCurrentThreadId();hook_release=CreateEventA(NULL,TRUE,FALSE,NULL);commit_release=CreateEventA(NULL,TRUE,FALSE,NULL);CHECK(hook_release && commit_release);
    CHECK(probe_output_init(decoder_wake_with_preemption,&render_wakes)==DBT_OK);
    probe_output_sound_wake(external_wake,&sound_progress_wakes);
#ifdef DBT_PROBE_EARLY_INPUT_RELEASE
    input_release=CreateEventA(NULL,TRUE,FALSE,NULL);CHECK(input_release);
    CHECK(!start_stream());
    unsigned music_only_wakes=atomic_load(&sound_progress_wakes);
    probe_output_stats_t geometry;probe_output_stats(&geometry);
#ifdef DBT_PROBE_HIGH_RATE_PCM_REBALANCE
    CHECK(geometry.input_capacity_frames==2048 && geometry.input_ring_bytes==16384 && geometry.pcm_target_frames==2048 && geometry.requested_bytes==39936);
#else
    CHECK(geometry.input_capacity_frames==1024 && geometry.input_ring_bytes==8192 && geometry.pcm_target_frames==3072 && geometry.requested_bytes==39936);
#endif
    uint32_t capacity=geometry.input_capacity_frames;
    int32_t first[256],replacement[4096];for(unsigned i=0;i<256;i++)first[i]=(int32_t)(i*65536u);
    for(unsigned i=0;i<4096;i++)replacement[i]=(int32_t)(0x12340000u+i);
    atomic_store(&decoder_hint,false);atomic_store(&hook_reached,0);atomic_store(&hold_hook,10);
    CHECK(probe_output_try_pcm_format(first,128,2,192000,DBT_PCM_S32_LE)==DBT_OK && hook_ready(10));
    CHECK(!dbt_output_test_ready_pcm()); /* copied output is still unpublished */
    CHECK(probe_output_try_pcm_format(replacement,(uint16_t)capacity,2,192000,DBT_PCM_S32_LE)==DBT_OK);
    atomic_store(&hold_hook,0);SetEvent(input_release);
    for(unsigned i=0;i<2000 && dbt_output_test_ready_pcm()!=capacity+128u;i++)Sleep(1);
    CHECK(dbt_output_test_ready_pcm()==capacity+128u);
    CHECK(atomic_load(&sound_progress_wakes)==music_only_wakes);
    CHECK(!stop_stream() && atomic_load(&sound_progress_wakes)>music_only_wakes);CloseHandle(input_release);
    puts("PASS early local input: full measured-capacity ring reused before DSP, output unpublished until processing");
#endif
#ifdef DBT_PROBE_HIGH_RATE_PCM_REBALANCE
    struct {uint32_t rate;uint8_t channels;dbt_pcm_format_t format;uint32_t input,target,bytes;} unaffected[]={
      {48000,2,DBT_PCM_S16_LE,512,1024,17408},{48000,2,DBT_PCM_S32_LE,512,1024,19456},
      {96000,2,DBT_PCM_S32_LE,512,1536,23552},{192000,1,DBT_PCM_S32_LE,1024,3072,35840},
      {192000,2,DBT_PCM_S16_LE,1024,3072,35840}};
    for(unsigned i=0;i<sizeof unaffected/sizeof unaffected[0];i++){
      uint32_t ticket=0;CHECK(probe_output_stream_begin_primed(true,unaffected[i].rate,unaffected[i].channels,unaffected[i].format,false,0,&ticket)==DBT_PENDING);
      CHECK(ticket_result(ticket)==DBT_OK);probe_output_stats_t g;probe_output_stats(&g);
      CHECK(g.input_capacity_frames==unaffected[i].input && g.pcm_target_frames==unaffected[i].target && g.requested_bytes==unaffected[i].bytes);
      CHECK(!stop_stream());
    }
    puts("PASS PCM rebalance: same39936-byte192k stereo payload allocation; five other formats/rates unchanged");
#endif
    /* Idle music skips sound-space signals, but a live reserve receives them.
     * Failed reserve and release receipts must still wake with no live token. */
    CHECK(!start_stream());uint32_t denied=0;
    unsigned before_sound_wake=atomic_load(&sound_progress_wakes);
    CHECK(probe_output_sound_reserve_begin(192000,true,&denied)==DBT_PENDING && ticket_result(denied)==DBT_BUSY);
    CHECK(atomic_load(&sound_progress_wakes)>before_sound_wake && !probe_output_sound_alive(denied));
    uint32_t lease=0;CHECK(probe_output_sound_reserve_begin(192000,false,&lease)==DBT_PENDING && ticket_result(lease)==DBT_OK);
    CHECK(probe_output_sound_alive(lease));before_sound_wake=atomic_load(&sound_progress_wakes);
    int32_t leased_pcm[256];for(unsigned i=0;i<256;i++)leased_pcm[i]=(int32_t)(i*65536u);
    CHECK(probe_output_try_pcm_format(leased_pcm,128,2,192000,DBT_PCM_S32_LE)==DBT_OK);
    for(unsigned i=0;i<2000 && dbt_output_test_ready_pcm()!=128;i++)Sleep(1);
    CHECK(dbt_output_test_ready_pcm()==128 && atomic_load(&sound_progress_wakes)>before_sound_wake);
    uint32_t released=0;before_sound_wake=atomic_load(&sound_progress_wakes);
    CHECK(probe_output_sound_release_begin(lease,&released)==DBT_PENDING && ticket_result(released)==DBT_OK);
    CHECK(!probe_output_sound_alive(lease) && atomic_load(&sound_progress_wakes)>before_sound_wake && !stop_stream());
    puts("PASS sound progress: idle music has no sound-space wake, live lease refills, failed reserve/release ACKs preserved");
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
#ifdef DBT_PROBE_RENDER_FAIR_SHARE
    CHECK(mock_tasks[1].priority==22 && mock_tasks[0].priority==20 && atomic_load(&yields[1]) && !atomic_load(&yields[0]));
    /* Refill input while the helper has published 512 frames but still owns
     * the lifecycle borrow. The first fair yield after DONE must include one
     * local 128-frame block, rather than yielding after bookkeeping alone. */
    yield_release=CreateEventA(NULL,TRUE,FALSE,NULL);CHECK(yield_release);ResetEvent(commit_release);
    CHECK(!start_stream() && !lend_block(512));
    atomic_store(&hold_hook,8);SetEvent(hook_release);CHECK(hook_ready(8));
    CHECK(dbt_output_test_ready_pcm()==512 && probe_output_decode_budget()==0);
    int32_t queued[512];for(unsigned i=0;i<512;i++)queued[i]=(int32_t)(0x12340000u+i);
    CHECK(probe_output_try_pcm_format(queued,256,2,192000,DBT_PCM_S32_LE)==DBT_OK);
    atomic_store_explicit(&yield_job_gate,3,memory_order_release);
    atomic_store(&hold_hook,0);SetEvent(commit_release);
    for(unsigned i=0;i<2000 && !atomic_load_explicit(&yield_ready_frames,memory_order_acquire);i++)Sleep(1);
    printf("first yield after DONE: ready=%u frames\n",atomic_load(&yield_ready_frames));
    CHECK(atomic_load(&yield_ready_frames)==640);
    atomic_store(&decoder_hint,false);SetEvent(yield_release);
    CHECK(!stop_stream());probe_output_dsp_assist_snapshot(&stats);
    CHECK(stats.jobs==3 && stats.frames==1152 && stats.blocks==9 && !stats.failures && !stats.state);
    CloseHandle(yield_release);
    puts("PASS retirement handoff: pending input receives one bounded local block before first fair yield");
#else
    CHECK(mock_tasks[1].priority==20 && mock_tasks[0].priority==20);
#endif
    /* No pending DMA window or EOF wakes exist in this fixture. Suspend the
     * helper at its producer wake after DONE; the signaled owner must retire
     * and close safely before this helper continuation resumes. */
    producer_release=CreateEventA(NULL,TRUE,FALSE,NULL);CHECK(producer_release);
    probe_output_dsp_assist_snapshot(&stats);unsigned jobs_before=stats.jobs;
    CHECK(!start_stream() && !lend_block(512));allocations=atomic_load(&live);
    atomic_store(&observe_completion_order,true);atomic_store(&hold_producer_wake,true);
    release_helper();
    for(unsigned i=0;i<2000 && !atomic_load_explicit(&producer_hold_reached,memory_order_acquire);++i)Sleep(1);
    CHECK(atomic_load(&producer_hold_reached));
    printf("owner completion signals before producer hold=%u\n",atomic_load(&owner_completion_signals));
    CHECK(atomic_load(&owner_completion_signals)==1);
    for(unsigned i=0;i<2000;++i){probe_output_dsp_assist_snapshot(&stats);if(stats.jobs==jobs_before+1u)break;Sleep(1);}
    CHECK(stats.jobs==jobs_before+1u && !stats.state && !stats.failures && atomic_load(&live)==allocations);
    CHECK(dbt_output_test_ready_pcm()==512 && !stop_stream());
    atomic_store(&observe_completion_order,false);SetEvent(producer_release);
    puts("PASS completion handoff: owner signaled before producer preemption, retires/closes during held helper continuation, no EOF needed");
    /* Supply one block beyond the lease cap. Publication is bounded by the
     * admitted cap; the owner retains its separate local block after DONE. */
    probe_output_dsp_assist_snapshot(&stats);
    unsigned cap_jobs=stats.jobs,cap_frames=stats.frames,cap_blocks=stats.blocks;
    const unsigned cap=DBT_PROBE_DSP_ASSIST_BLOCKS*FIXED_DSP_BLOCK_FRAMES;
    ResetEvent(commit_release);CHECK(!start_stream() && !lend_block(cap+128u));
    atomic_store(&hold_hook,8);SetEvent(hook_release);CHECK(hook_ready(8));
    CHECK(dbt_output_test_ready_pcm()==cap && probe_output_decode_budget()==0);
    atomic_store(&decoder_hint,false);atomic_store(&hold_hook,0);SetEvent(commit_release);
    for(unsigned i=0;i<2000;++i){probe_output_dsp_assist_snapshot(&stats);if(stats.jobs==cap_jobs+1u && dbt_output_test_ready_pcm()==cap+128u)break;Sleep(1);}
    CHECK(stats.jobs==cap_jobs+1u && stats.frames-cap_frames==cap && stats.blocks-cap_blocks==DBT_PROBE_DSP_ASSIST_BLOCKS && stats.max_job_frames==cap && !stats.failures && !stats.state);
    CHECK(dbt_output_test_ready_pcm()==cap+128u && !stop_stream());
    printf("PASS bounded helper cap: %u blocks / %u frames plus one local128, retained borrow until DONE\n",DBT_PROBE_DSP_ASSIST_BLOCKS,cap);
    printf("main waits=%u blocking sends=%u mock errors=%u timed waits while held=%u\n",atomic_load(&main_waits),atomic_load(&blocking_sends),atomic_load(&mock_errors),atomic_load(&pending_timed_waits));
    CHECK(!atomic_load(&main_waits) && !atomic_load(&blocking_sends) && !atomic_load(&mock_errors) && !atomic_load(&pending_timed_waits));
    probe_output_restart_hold(true);CHECK(probe_output_restart_quiescent());
    atomic_store_explicit(&mock_shutdown,true,memory_order_release);
    for(unsigned i=0;i<2;++i){CHECK(mock_tasks[i].thread);SetEvent(mock_tasks[i].event);CHECK(WaitForSingleObject(mock_tasks[i].thread,2000)==WAIT_OBJECT_0);CloseHandle(mock_tasks[i].thread);CloseHandle(mock_tasks[i].event);}
    CloseHandle(hook_release);CloseHandle(commit_release);CloseHandle(producer_release);
    puts("PASS actual DSP-assisted output: held worker/queued stop/cancel, no early free, no timed pending polling, four-block/single-block leases, worker PCM published before DONE, decoder admission, single completion, both mocked cores, quiescent restart");return 0;
}
