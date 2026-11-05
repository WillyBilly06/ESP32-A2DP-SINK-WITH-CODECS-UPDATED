/* Actual decoder/worker/DSP/I2S, synthetic loss-free 5 ms input. No controller,
 * filesystem or NVS. This is not RF/coexistence or analog-quality evidence. */
#include "probe_pipeline_check.h"
#ifdef DBT_PROBE_FIR_COMPILER_CHECK
#include "probe_fir_compare.h"
#elif defined(DBT_AUDIO_STAGE_PROFILE)
#include "probe_dsp_compare.h"
#endif
#include "probe_output.h"
#include "dbt_codec.h"
#include "dbt_lhdcv5_codec.h"
#include "probe_diagnostic_stats.h"
#include "esp_cpu.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>
typedef struct {uint32_t index;int32_t value;} probe_pcm_point_t;
#include "probe_lhdc_max_fixture.h"

typedef union {struct {uint32_t bytes;} h;uint64_t alignment;} allocation_t;
static struct {
    dbt_codec_worker_t worker;
    TaskHandle_t task,owner;
    _Atomic bool done;
    bool passed,control_unresolved;
    uint32_t live,peak,hash,frames,token,step_max,step_calls,queue_peak;
    uint64_t step_us,step_cycles,hash_us;
    uint32_t arrival[96],completion[96];
    dbt_result_t stream_result;
} pipeline;
static uint32_t now_ms(void){return (uint32_t)(esp_timer_get_time()/1000);}
static void wake(void *context){(void)context;if(pipeline.task)xTaskNotifyGive(pipeline.task);}
static uint8_t *allocate(void *context,uint32_t bytes){
    (void)context;
    if(!bytes || bytes>UINT32_MAX-sizeof(allocation_t))return NULL;
    uint32_t n=bytes+sizeof(allocation_t);
    allocation_t *p=heap_caps_aligned_alloc(8,n,MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
    if(!p)return NULL;
    p->h.bytes=n;pipeline.live+=n;
    if(pipeline.live>pipeline.peak)pipeline.peak=pipeline.live;
    return (uint8_t *)(p+1);
}
static void release(void *context,uint8_t *bytes){
    (void)context;if(!bytes)return;
    allocation_t *p=(allocation_t *)bytes-1;
    if(p->h.bytes>pipeline.live)pipeline.passed=false;
    else pipeline.live-=p->h.bytes;
    heap_caps_free(p);
}
static dbt_result_t pcm(void *context,const void *samples,uint16_t frames,const dbt_codec_config_t *config,uint32_t timestamp){
    (void)context;
    if(!config || config->rate!=192000 || config->channels!=2 || config->sample_bytes!=4 || timestamp!=pipeline.frames)return DBT_INVALID;
    dbt_result_t result=
#ifdef DBT_PROBE_PCM_LOANS
        DBT_OK; /* Accounting after the asynchronous begin accepted the view. */
#else
        probe_output_try_pcm_format(samples,frames,2,192000,DBT_PCM_S32_LE);
#endif
    if(result==DBT_OK){
        int64_t began=esp_timer_get_time();
        const uint8_t *p=samples;
        for(unsigned i=0;i<(unsigned)frames*8u;++i)pipeline.hash=(pipeline.hash^p[i])*16777619u;
        pipeline.frames+=frames;pipeline.hash_us+=(uint64_t)(esp_timer_get_time()-began);
    }
    return result;
}
static dbt_result_t stream_begin(void *context,bool start,const dbt_codec_config_t *config,uint32_t *ticket){
    (void)context;
    return probe_output_stream_begin(start,config->rate,config->channels,(dbt_pcm_format_t)config->sample_bytes,ticket);
}
static dbt_result_t stream_poll(void *context,uint32_t ticket){
    (void)context;dbt_result_t result=probe_output_request_poll(ticket);
    if(result!=DBT_PENDING)pipeline.stream_result=result;
    return result;
}
static dbt_result_t stream_cancel(void *context,uint32_t ticket){(void)context;return probe_output_request_cancel(ticket);}
#ifdef DBT_PROBE_PCM_LOANS
static dbt_result_t pcm_begin(void *context,const void *samples,uint16_t frames,const dbt_codec_config_t *config,uint32_t timestamp,uint32_t *ticket){
    if(!config || config->rate!=192000 || config->channels!=2 || config->sample_bytes!=4 || timestamp!=pipeline.frames)return DBT_INVALID;
    dbt_result_t result=probe_output_pcm_begin(samples,frames,2,192000,DBT_PCM_S32_LE,ticket);
    if(result==DBT_PENDING)(void)pcm(context,samples,frames,config,timestamp);
    return result;
}
static dbt_result_t pcm_poll(void *context,uint32_t ticket){(void)context;return probe_output_pcm_poll(ticket);}
static dbt_result_t pcm_cancel(void *context,uint32_t ticket){(void)context;return probe_output_pcm_cancel(ticket);}
#endif
/* Only the diagnostic task sleeps. Each iteration executes at most one codec
 * work unit. Renderer/PCM/stream callbacks never wait for another owner. */
static void sleep_until(int64_t deadline){
    if(dbt_codec_worker_runnable(&pipeline.worker))return;
    int64_t us=deadline-esp_timer_get_time();
    uint32_t when;
    if(dbt_codec_worker_deadline(&pipeline.worker,&when)){
        int32_t delta=(int32_t)(when-now_ms());
        int64_t control_us=delta>0?(int64_t)delta*1000:1;
        if(control_us<us)us=control_us;
    }
    if(us<=0)return;
    TickType_t ticks=pdMS_TO_TICKS((uint32_t)((us+999)/1000));
    ulTaskNotifyTake(pdTRUE,ticks?ticks:1);
}
static bool control(dbt_sink_operation_t operation,const uint8_t *configuration,uint16_t size){
    dbt_sink_job_t job={1,++pipeline.token,operation,1,configuration,size};
    if(dbt_codec_worker_submit(&pipeline.worker,&job)!=DBT_OK)return false;
    int64_t deadline=esp_timer_get_time()+2000000;
    do{
        dbt_codec_worker_step(&pipeline.worker,now_ms(),0);
        dbt_codec_receipt_t receipt;
        if(dbt_codec_worker_receipt(&pipeline.worker,&receipt)==DBT_OK)
            return receipt.token==job.token && receipt.result==DBT_SINK_WORK_READY;
        sleep_until(deadline);
    }while(esp_timer_get_time()<deadline);
    /* An unresolved renderer loan is retained until the core-0 return resets
     * this pending trial. Do not free it just because a local deadline expired. */
    pipeline.control_unresolved=true;return false;
}
static void run(void *unused){
    (void)unused;pipeline.task=xTaskGetCurrentTaskHandle();
    pipeline.hash=2166136261u;pipeline.passed=true;pipeline.stream_result=DBT_EMPTY;
    uint32_t before_free=heap_caps_get_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
    uint32_t before_largest=heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
    const dbt_codec_ops_t *codecs[]={&dbt_lhdcv5_codec};
    dbt_codec_worker_hooks_t hooks={0};
    hooks.decoder=(dbt_codec_hooks_t){NULL,allocate,release,pcm};
    hooks.codecs=codecs;hooks.codec_count=1;hooks.deferred_pcm=true;
    /* Producer, receipt owner and worker share this diagnostic task. Only the
     * external timer and renderer need notifications; no self-wake polling. */
    hooks.async_stream=(dbt_codec_stream_async_t){stream_begin,stream_poll,stream_cancel};
#ifdef DBT_PROBE_PCM_LOANS
    hooks.async_pcm=(dbt_codec_pcm_async_t){pcm_begin,pcm_poll,pcm_cancel};
#endif
    bool initialized=dbt_codec_worker_init(&pipeline.worker,hooks,628,2)==DBT_OK;
    bool output_initialized=initialized && probe_output_init(wake,NULL)==DBT_OK;
    bool configured=false,started=false;
    if(output_initialized){
        uint8_t configuration[4+sizeof(lhdc_max_cie)]={1,0,7,sizeof(lhdc_max_cie)};
        memcpy(configuration+4,lhdc_max_cie,sizeof(lhdc_max_cie));
        configured=control(DBT_SINK_CONFIGURE,configuration,sizeof(configuration));
        if(configured)started=control(DBT_SINK_START,NULL,0);
    }
    uint32_t opened_free=heap_caps_get_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
    uint32_t opened_largest=heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
    dbt_result_t start_result=pipeline.stream_result;
    pipeline.passed=output_initialized && configured && started && pipeline.passed;
    /* Nonzero EQ gains, integer processing and analysis; modest diagnostic
     * volume. This changes gain, never codec bitrate/depth or sample precision. */
    if(pipeline.passed)pipeline.passed=probe_output_controls(3,-2,2,50,9)==DBT_OK;
    esp_timer_handle_t timer=NULL;
    esp_timer_create_args_t timer_args={.callback=wake,.name="pipeline_source"};
    bool timer_started=false;
    int64_t began=esp_timer_get_time(),deadline=began+3000000;
    if(pipeline.passed){
        pipeline.passed=esp_timer_create(&timer_args,&timer)==ESP_OK;
        if(pipeline.passed){timer_started=esp_timer_start_periodic(timer,5000)==ESP_OK;pipeline.passed=timer_started;}
    }
    unsigned supplied=0,completed=0;
    while(pipeline.passed && completed<96 && esp_timer_get_time()<deadline){
        int64_t now=esp_timer_get_time();
        if(supplied<96 && now>=began+(int64_t)(supplied+1u)*5000){
            int64_t late=now-(began+(int64_t)(supplied+1u)*5000);
            pipeline.arrival[supplied]=(uint32_t)late;
            dbt_avdtp_media_t packet={lhdc_max_packets[supplied].data,supplied*960u,1,lhdc_max_packets[supplied].size,(uint16_t)supplied,96,false};
            pipeline.passed=dbt_codec_worker_media(&pipeline.worker,&packet)==DBT_OK;
            if(pipeline.passed)++supplied;
        }
        uint32_t queue=atomic_load(&pipeline.worker.media_written)-atomic_load(&pipeline.worker.media_read);
        if(queue>pipeline.queue_peak)pipeline.queue_peak=queue;
        int64_t step_at=esp_timer_get_time();uint32_t cycles=esp_cpu_get_cycle_count();
        dbt_codec_worker_step(&pipeline.worker,now_ms(),1);
        uint32_t elapsed_cycles=esp_cpu_get_cycle_count()-cycles;
        uint32_t elapsed=(uint32_t)(esp_timer_get_time()-step_at);
        pipeline.step_us+=elapsed;pipeline.step_cycles+=elapsed_cycles;++pipeline.step_calls;
        if(elapsed>pipeline.step_max)pipeline.step_max=elapsed;
        unsigned retired=atomic_load(&pipeline.worker.media_read);
        if(retired>completed){
            if(retired!=completed+1u || retired>96)pipeline.passed=false;
            else pipeline.completion[completed++]=(uint32_t)(esp_timer_get_time()-(began+(int64_t)retired*5000));
        }
        if(probe_output_failed() || atomic_load(&pipeline.worker.media_drops) || atomic_load(&pipeline.worker.faults))pipeline.passed=false;
        if(pipeline.passed && completed<96){
            int64_t next=supplied<96?began+(int64_t)(supplied+1u)*5000:deadline;
            if(next>deadline)next=deadline;
            sleep_until(next);
        }
    }
    int64_t ended=esp_timer_get_time();
    if(timer_started)esp_timer_stop(timer);
    if(timer)esp_timer_delete(timer);
    probe_output_stats_t active={0};probe_output_stats(&active);
    bool correctness=supplied==96 && completed==96 && pipeline.frames==92160 && pipeline.hash==4008775619u;
    bool continuity=active.running && active.dma_eofs && !active.failed && !active.underrun_frames && !active.descriptor_errors;
    pipeline.passed=pipeline.passed && correctness && continuity;
    bool dropped=false;
    if(initialized && !pipeline.control_unresolved)dropped=control(DBT_SINK_DROP,NULL,0);
    probe_output_stats_t closed={0};if(output_initialized)probe_output_stats(&closed);
    bool reclaimed=dropped && !pipeline.live && !pipeline.worker.decoder.allocated_bytes && !closed.requested_bytes && !closed.running;
    pipeline.passed=pipeline.passed && reclaimed && uxTaskGetStackHighWaterMark(NULL)>=1024u && active.render_stack_low_water_bytes>=1024u;
    probe_diagnostic_stats_t arrival={0},completion={0};
    if(supplied)probe_diagnostic_stats(pipeline.arrival,supplied,5000,&arrival);
    if(completed)probe_diagnostic_stats(pipeline.completion,completed,5000,&completion);
    printf("{\"probe\":\"pipeline_memory\",\"before_free\":%u,\"before_largest\":%u,\"opened_free\":%u,\"opened_largest\":%u,\"minimum_free\":%u,\"after_free\":%u,\"decoder_tracked_peak\":%u,\"renderer_requested\":%u,\"reclaimed\":%s,\"remaining_tracked\":%u,\"core1_stack_low\":%u,\"render_stack_low\":%u}\n",
        (unsigned)before_free,(unsigned)before_largest,(unsigned)opened_free,(unsigned)opened_largest,
        (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT),
        (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT),
        (unsigned)pipeline.peak,(unsigned)active.requested_bytes,reclaimed?"true":"false",(unsigned)pipeline.live,
        (unsigned)uxTaskGetStackHighWaterMark(NULL),(unsigned)active.render_stack_low_water_bytes);
    printf("{\"probe\":\"pipeline_timing\",\"elapsed_us\":%llu,\"worker_total_us\":%llu,\"worker_total_cycles\":%llu,\"worker_calls\":%u,\"worker_max_us\":%u,\"hash_us\":%llu,\"arrival_p99_us\":%u,\"arrival_max_us\":%u,\"completion_p99_us\":%u,\"completion_max_us\":%u,\"completion_over_5ms\":%u,\"queue_peak\":%u,\"render_total_us\":%u,\"render_calls\":%u,\"render_max_us\":%u}\n",
        (unsigned long long)(ended-began),(unsigned long long)pipeline.step_us,(unsigned long long)pipeline.step_cycles,
        (unsigned)pipeline.step_calls,(unsigned)pipeline.step_max,(unsigned long long)pipeline.hash_us,
        (unsigned)arrival.p99,(unsigned)arrival.maximum,(unsigned)completion.p99,(unsigned)completion.maximum,
        (unsigned)completion.at_or_above_budget,(unsigned)pipeline.queue_peak,
        (unsigned)active.render_total_us,(unsigned)active.render_calls,(unsigned)active.maximum_render_us);
    printf("{\"probe\":\"pipeline_i2s\",\"running\":%u,\"failed\":%u,\"accepted_frames\":%u,\"rendered_frames\":%u,\"dma_frames\":%u,\"dma_eofs\":%u,\"underrun_frames\":%u,\"descriptor_errors\":%u,\"max_eof_gap_us\":%u,\"ready_frames\":%u,\"input_ready_frames\":%u,\"stop_tail_excluded\":true}\n",
        (unsigned)active.running,(unsigned)active.failed,(unsigned)active.accepted_frames,(unsigned)active.rendered_frames,
        (unsigned)active.dma_source_frames,(unsigned)active.dma_eofs,(unsigned)active.underrun_frames,
        (unsigned)active.descriptor_errors,(unsigned)active.max_eof_gap_us,(unsigned)active.ready_frames,(unsigned)active.input_ready_frames);
    printf("{\"probe\":\"pipeline_allocation\",\"failed_bytes\":%u,\"failure_free\":%u,\"failure_largest\":%u,\"page_bytes\":%u}\n",
        (unsigned)active.allocation_failed_bytes,(unsigned)active.allocation_failure_free,(unsigned)active.allocation_failure_largest,
        (unsigned)(DBT_AUDIO_OUTPUT_PAGE_FRAMES*8u));
#ifdef DBT_PROBE_PCM_LOANS
    printf("{\"probe\":\"pipeline_pcm_loans\",\"mode\":%u,\"input_ring_bytes_avoided\":%u,\"renderer_begun\":%u,\"renderer_retired\":%u,\"renderer_cancelled\":%u,\"active\":%u,\"max_hold_us\":%u}\n",
        (unsigned)closed.pcm_loan_mode,(unsigned)closed.input_ring_bytes_avoided,(unsigned)closed.pcm_loans_begun,
        (unsigned)closed.pcm_loans_retired,(unsigned)closed.pcm_loans_cancelled,(unsigned)closed.pcm_loan_active,(unsigned)closed.pcm_loan_max_us);
#endif
    printf("{\"probe\":\"pipeline_render_stages\",\"quiescent\":%s,\"includes_stop_drain\":true,\"profile_frames\":%u,\"conversion_resampler_cycles\":%u,\"dsp_cycles\":%u,\"output_tail_cycles\":%u,\"controls_cycles\":%u,\"conversion_block_max_cycles\":%u,\"dsp_block_max_cycles\":%u,\"tail_block_max_cycles\":%u,\"media_drops\":%u,\"worker_faults\":%u,\"pcm_timeouts\":%u,\"cpu_mhz\":240,\"elapsed_core_cycles\":true}\n",
        reclaimed?"true":"false",(unsigned)closed.profile_frames,(unsigned)closed.profile_conversion,(unsigned)closed.profile_dsp,
        (unsigned)closed.profile_tail,(unsigned)closed.profile_controls,(unsigned)closed.profile_conversion_max,
        (unsigned)closed.profile_dsp_max,(unsigned)closed.profile_tail_max,(unsigned)atomic_load(&pipeline.worker.media_drops),
        (unsigned)atomic_load(&pipeline.worker.faults),(unsigned)atomic_load(&pipeline.worker.pcm_timeouts));
    printf("{\"probe\":\"target_pipeline_selftest\",\"passed\":%s,\"configured\":%s,\"started\":%s,\"stream_result\":%u,\"correctness\":%s,\"continuity\":%s,\"rate\":192000,\"sample_depth\":24,\"channels\":2,\"encoded_payload_bps\":1001600,\"packets\":%u,\"completed_packets\":%u,\"sample_frames\":%u,\"pcm_hash\":%u,\"synthetic_source\":true,\"integer_dsp\":true,\"i2s_test\":true,\"rf_test\":false,\"production_qualified\":false}\n",
        pipeline.passed?"true":"false",configured?"true":"false",started?"true":"false",(unsigned)start_result,
        correctness?"true":"false",continuity?"true":"false",supplied,completed,(unsigned)pipeline.frames,(unsigned)pipeline.hash);
#ifdef DBT_PROBE_FIR_COMPILER_CHECK
    if(reclaimed && !probe_fir_compare())pipeline.passed=false;
#elif defined(DBT_AUDIO_STAGE_PROFILE)
    if(reclaimed && !probe_dsp_compare())pipeline.passed=false;
#endif
    atomic_store_explicit(&pipeline.done,true,memory_order_release);xTaskNotifyGive(pipeline.owner);
    /* Renderer callbacks and a dispatched timer callback may still name this
     * task. Keep its notification target live until the admitted owner resets. */
    vTaskSuspend(NULL);
}
bool probe_pipeline_check(void){
    pipeline.owner=xTaskGetCurrentTaskHandle();
    atomic_init(&pipeline.done,false);
    if(xTaskCreatePinnedToCore(run,"pipeline_check",8192,NULL,5,NULL,1)!=pdPASS)return false;
    int64_t deadline=esp_timer_get_time()+
#ifdef DBT_PROBE_FIR_COMPILER_CHECK
        23000000;
#else
        10000000;
#endif
    while(!atomic_load_explicit(&pipeline.done,memory_order_acquire)){
        int64_t remaining=deadline-esp_timer_get_time();if(remaining<=0)return false;
        TickType_t ticks=pdMS_TO_TICKS((uint32_t)((remaining+999)/1000));
        ulTaskNotifyTake(pdTRUE,ticks?ticks:1);
    }
    return pipeline.passed;
}
