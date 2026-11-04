#include "dbt_codec.h"
#include "probe_codec_check.h"
#ifdef DBT_PROBE_SBC_BUILD
#include "dbt_sbc_codec.h"
#endif
#ifdef DBT_PROBE_OPUS_BUILD
#include "dbt_opus.h"
#include "probe_opus_vectors.h"
#endif
#ifdef DBT_PROBE_AAC_BUILD
#include "dbt_aac.h"
#endif
#ifdef DBT_PROBE_APTX_BUILD
#include "dbt_aptx.h"
#endif
#ifdef DBT_PROBE_LDAC_BUILD
#include "dbt_ldac.h"
#endif
#ifdef DBT_PROBE_LHDC_BUILD
#include "dbt_lhdcv5_codec.h"
#endif
#include "probe_codec_vectors.h"
#ifdef DBT_PROBE_LHDC_MAX_CHECK
#include "probe_lhdc_max_fixture.h"
#endif
#include "esp_heap_caps.h"
#include "esp_timer.h"
#ifdef ESP_PLATFORM
#include "esp_cpu.h"
#endif
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdatomic.h>
#include <stdio.h>
#include <stdint.h>

#define CHECK_STACK_BYTES 8192u
#define CHECK_STACK_FLOOR_BYTES 1024u
/* Host-derived sample checkpoints are used only for float-based codecs and
 * AAC's distinct target arithmetic path. The initial, explicit bound is 2%
 * of the expected magnitude plus 256 S16 or 2^24 S32 units. This is not a
 * fidelity benchmark or a substitute for a measured cross-target tolerance. */
typedef union { struct {uint32_t bytes;} h; uint64_t align; } allocation_t;
static struct {
    _Atomic bool done,running;
    bool passed;
    probe_codec_complete_fn complete;
    void *complete_context;
    dbt_codec_decoder_t decoder;
    const probe_codec_vector_t *vector;
#ifdef DBT_PROBE_OPUS_BUILD
    const opus_vector_t *opus;
#endif
    uint32_t hash,callbacks,sample_frames,word_index,point_index,peak,maximum_error;
    uint32_t live,peak_live,overall_peak,case_max_us,overall_max_us,min_free,overall_min_free;
    uint32_t case_max_cycles,case_deadline_misses,media_bytes;
    uint64_t case_cycles,case_us;
} check;

static uint8_t *allocate(void *context,uint32_t bytes){
    (void)context;
    if(!bytes || bytes>UINT32_MAX-sizeof(allocation_t))return 0;
    uint32_t requested=bytes+(uint32_t)sizeof(allocation_t);
    if(check.live>UINT32_MAX-requested)return 0;
    allocation_t *h=heap_caps_aligned_alloc(8,requested,MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
    if(!h)return 0;
    h->h.bytes=requested;check.live+=requested;if(check.live>check.peak_live)check.peak_live=check.live;
    if(check.live>check.overall_peak)check.overall_peak=check.live;
    uint32_t available=(uint32_t)heap_caps_get_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
    if(available<check.min_free)check.min_free=available;
    if(available<check.overall_min_free)check.overall_min_free=available;
    return (uint8_t *)(h+1);
}
static void release(void *context,uint8_t *p){
    (void)context;if(!p)return;allocation_t *h=(allocation_t *)p-1;
    if(h->h.bytes>check.live){check.passed=false;check.live=0;}else check.live-=h->h.bytes;
    heap_caps_free(h);
}
static uint32_t magnitude(int32_t value){return value<0?(uint32_t)(-(int64_t)value):(uint32_t)value;}
static dbt_result_t pcm(void *context,const void *samples,uint16_t frames,const dbt_codec_config_t *config,uint32_t timestamp){
    (void)context;(void)timestamp;
    if(!samples || !config || !frames)return DBT_INVALID;
#ifdef DBT_PROBE_OPUS_BUILD
    if(check.opus){
        if(config->sample_bytes!=2 || config->rate!=48000 || config->channels!=check.opus->channels || frames!=check.opus->samples)return DBT_INVALID;
        const uint8_t *p=samples;
        for(unsigned i=0;i<(unsigned)frames*config->channels*2u;++i)check.hash=(check.hash^p[i])*16777619u;
        ++check.callbacks;check.sample_frames+=frames;return DBT_OK;
    }
#endif
    const probe_codec_vector_t *v=check.vector;
    if(!v || config->sample_bytes!=v->width || config->rate!=v->rate || config->channels!=v->channels ||
       (v->callback_samples && frames!=v->callback_samples) || check.sample_frames>UINT32_MAX-frames)return DBT_INVALID;
    const uint8_t *bytes=samples;unsigned count=(unsigned)frames*v->channels;
    for(unsigned i=0;i<count*v->width;++i)check.hash=(check.hash^bytes[i])*16777619u;
    for(unsigned i=0;i<count;++i){
        int32_t value=v->width==2?((const int16_t *)samples)[i]:((const int32_t *)samples)[i];
        uint32_t power=magnitude(value);if(power>check.peak)check.peak=power;
        if(!v->exact && check.point_index<v->point_count && v->points[check.point_index].index==check.word_index){
            int32_t expected=v->points[check.point_index].value;
            int64_t delta=(int64_t)value-expected;if(delta<0)delta=-delta;
            uint64_t limit=(uint64_t)magnitude(expected)/50u+(v->width==2?256u:UINT64_C(16777216));
            if((uint64_t)delta>check.maximum_error)check.maximum_error=(uint32_t)delta;
            if((uint64_t)delta>limit)return DBT_INVALID;
            ++check.point_index;
        }
        ++check.word_index;
    }
    ++check.callbacks;check.sample_frames+=frames;return DBT_OK;
}
static void reset_case(void){
    check.vector=0;
#ifdef DBT_PROBE_OPUS_BUILD
    check.opus=0;
#endif
    check.hash=2166136261u;check.callbacks=check.sample_frames=check.word_index=check.point_index=0;
    check.peak=check.maximum_error=check.case_max_us=0;check.peak_live=check.live;
    check.case_max_cycles=check.case_deadline_misses=check.media_bytes=0;
    check.case_cycles=check.case_us=0;
    check.min_free=(uint32_t)heap_caps_get_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
}
static bool feed(const dbt_codec_ops_t *ops,const uint8_t *packet,uint16_t size,uint16_t sequence,uint32_t timestamp,bool marker){
    dbt_avdtp_media_t view={.payload=packet,.size=size,.sequence=sequence,.ssrc=1,.timestamp=timestamp,.payload_type=96,.marker=marker};
    uint32_t frames_before=check.sample_frames;
#ifdef ESP_PLATFORM
    uint32_t cycles_before=esp_cpu_get_cycle_count();
#endif
    int64_t started=esp_timer_get_time();dbt_result_t result=ops->media(&check.decoder,&view,0);
    int64_t delta=esp_timer_get_time()-started;uint32_t elapsed=delta<0?0:delta>UINT32_MAX?UINT32_MAX:(uint32_t)delta;
#ifdef ESP_PLATFORM
    uint32_t cycles=esp_cpu_get_cycle_count()-cycles_before;
    check.case_cycles+=cycles;if(cycles>check.case_max_cycles)check.case_max_cycles=cycles;
#endif
    check.case_us+=elapsed;check.media_bytes+=size;
    uint32_t frames=check.sample_frames-frames_before;
    uint32_t rate=check.vector?check.vector->rate:48000u;
    if(frames && (uint64_t)elapsed*rate>(uint64_t)frames*1000000u)++check.case_deadline_misses;
    if(elapsed>check.case_max_us)check.case_max_us=elapsed;
    if(elapsed>check.overall_max_us)check.overall_max_us=elapsed;
    return result==DBT_OK;
}
static bool run_vector(const probe_codec_vector_t *v){
    reset_case();check.vector=v;dbt_codec_config_t config;
    bool pass=v->ops && v->ops->configure(v->cie,v->cie_size,&config)==DBT_OK;
    if(pass){
        pass=config.rate==v->rate && config.channels==v->channels && config.sample_bytes==v->width;
        config.ops=v->ops;
        if(pass)pass=dbt_codec_open(&check.decoder,&config)==DBT_OK;
    }
    for(unsigned i=0;pass && i<v->packet_count;++i)
        pass=feed(v->ops,v->packet[i],v->bytes[i],(uint16_t)i,i*(v->callback_samples?v->callback_samples:128u),v->marker!=0);
    if(pass)pass=check.callbacks && check.sample_frames==v->sample_frames && check.peak>=v->minimum_peak &&
        (v->exact?check.hash==v->expected_hash:check.point_index==v->point_count && v->point_count>=4);
    dbt_codec_close(&check.decoder);
    if(check.decoder.allocated_bytes || check.live)pass=false;
    unsigned low=(unsigned)uxTaskGetStackHighWaterMark(NULL);if(low<CHECK_STACK_FLOOR_BYTES)pass=false;
    printf("{\"probe\":\"target_codec_case\",\"codec\":\"%s\",\"passed\":%s,\"rate\":%u,\"channels\":%u,\"sample_bytes\":%u,\"packets\":%u,\"callbacks\":%u,\"sample_frames\":%u,\"tracked_peak_bytes\":%u,\"min_free_after_alloc_bytes\":%u,\"stack_low_water_bytes\":%u,\"maximum_decode_us\":%u,\"comparison\":\"%s\",\"maximum_abs_error\":%u,\"media_total_us\":%llu,\"media_total_cycles\":%llu,\"media_max_cycles\":%u,\"media_deadline_misses\":%u,\"media_payload_bps\":%u,\"includes_pcm_validation\":true,\"isolated\":true}\n",
        v->name,pass?"true":"false",(unsigned)v->rate,v->channels,v->width,v->packet_count,
        (unsigned)check.callbacks,(unsigned)check.sample_frames,(unsigned)check.peak_live,(unsigned)check.min_free,low,(unsigned)check.case_max_us,
        v->exact?"exact":"bounded_samples",(unsigned)check.maximum_error,
        (unsigned long long)check.case_us,(unsigned long long)check.case_cycles,(unsigned)check.case_max_cycles,
        (unsigned)check.case_deadline_misses,check.sample_frames?(unsigned)((uint64_t)check.media_bytes*8u*v->rate/check.sample_frames):0);
    check.vector=0;return pass;
}
#ifdef DBT_PROBE_OPUS_BUILD
static bool run_opus(const opus_vector_t *v,unsigned index){
    reset_case();check.opus=v;dbt_codec_config_t config;
    bool pass=dbt_opus_configure(v->cie,v->cie_size,&config)==DBT_OK;
    if(pass){config.ops=&dbt_opus_codec;pass=dbt_codec_open(&check.decoder,&config)==DBT_OK;}
    for(unsigned i=0;pass && i<3;++i)pass=feed(&dbt_opus_codec,v->packet[i],v->bytes[i],(uint16_t)i,i*v->samples,false);
    if(pass)pass=check.hash==v->hash && check.callbacks==3 && check.sample_frames==3u*v->samples;
    dbt_codec_close(&check.decoder);
    if(check.decoder.allocated_bytes || check.live)pass=false;
    unsigned low=(unsigned)uxTaskGetStackHighWaterMark(NULL);if(low<CHECK_STACK_FLOOR_BYTES)pass=false;
    printf("{\"probe\":\"target_codec_case\",\"codec\":\"opus_%u\",\"passed\":%s,\"rate\":48000,\"channels\":%u,\"sample_bytes\":2,\"packets\":3,\"callbacks\":%u,\"sample_frames\":%u,\"tracked_peak_bytes\":%u,\"min_free_after_alloc_bytes\":%u,\"stack_low_water_bytes\":%u,\"maximum_decode_us\":%u,\"comparison\":\"exact\"}\n",
        index,pass?"true":"false",v->channels,(unsigned)check.callbacks,(unsigned)check.sample_frames,
        (unsigned)check.peak_live,(unsigned)check.min_free,low,(unsigned)check.case_max_us);
    check.opus=0;return pass;
}
#endif
#ifdef DBT_PROBE_LHDC_MAX_CHECK
static bool run_lhdc_maximum(void){
    const probe_codec_vector_t vector={.name="lhdcv5_192000_24_1000",.ops=&dbt_lhdcv5_codec,
        .cie=lhdc_max_cie,.cie_size=sizeof(lhdc_max_cie),.channels=2,.width=4,.rate=192000,
        .sample_frames=92160,.minimum_peak=100000000,.callback_samples=960,
        .points=lhdc_max_points,.point_count=sizeof(lhdc_max_points)/sizeof(lhdc_max_points[0])};
    reset_case();check.vector=&vector;dbt_codec_config_t config;
    bool pass=dbt_lhdcv5_configure(vector.cie,vector.cie_size,&config)==DBT_OK;
    if(pass){config.ops=vector.ops;pass=dbt_codec_open(&check.decoder,&config)==DBT_OK;}
    uint32_t measured[96],measured_count=0,late=0;uint64_t measured_sum=0;
    for(unsigned i=0;pass && i<sizeof(lhdc_max_packets)/sizeof(lhdc_max_packets[0]);++i){
        uint64_t before=check.case_us;
        pass=feed(vector.ops,lhdc_max_packets[i].data,lhdc_max_packets[i].size,(uint16_t)i,i*960u,false);
        uint32_t elapsed=(uint32_t)(check.case_us-before);
        if(i>=12u){measured[measured_count++]=elapsed;measured_sum+=elapsed;if(elapsed>=5000u)++late;}
        /* Finite diagnostic only: let idle/other tasks progress between input
         * fixtures. This delay is outside the measured media processing span. */
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    if(pass)pass=check.callbacks==96 && check.sample_frames==vector.sample_frames &&
        check.point_index==vector.point_count && check.peak>=vector.minimum_peak;
    dbt_codec_close(&check.decoder);
    if(check.live || check.decoder.allocated_bytes)pass=false;
    unsigned low=(unsigned)uxTaskGetStackHighWaterMark(NULL);if(low<CHECK_STACK_FLOOR_BYTES)pass=false;
    /* Bounded 84-element sort after measurement, no heap/statistics task. */
    for(unsigned i=1;i<measured_count;++i){uint32_t v=measured[i];unsigned j=i;
        while(j && measured[j-1]>v){measured[j]=measured[j-1];--j;}measured[j]=v;}
    uint32_t maximum=measured_count?measured[measured_count-1]:0;
    bool deadline=measured_count==84 && !late && maximum<5000u;
    printf("{\"probe\":\"lhdc_max_selftest\",\"correctness_passed\":%s,\"isolated_deadline_passed\":%s,\"rate\":192000,\"sample_depth\":24,\"channels\":2,\"requested_kbps\":1000,\"encoded_payload_bps\":1001600,\"packets\":%u,\"measured_packets\":%u,\"sample_frames\":%u,\"reference_points\":%u,\"maximum_abs_error\":%u,\"mean_us\":%u,\"p50_us\":%u,\"p95_us\":%u,\"p99_us\":%u,\"max_us\":%u,\"budget_us\":5000,\"deadline_misses\":%u,\"tracked_peak_bytes\":%u,\"stack_low_water_bytes\":%u,\"includes_pcm_validation\":true,\"dsp_i2s_test\":false,\"rf_test\":false}\n",
        pass?"true":"false",deadline?"true":"false",(unsigned)check.callbacks,(unsigned)measured_count,
        (unsigned)check.sample_frames,(unsigned)check.point_index,(unsigned)check.maximum_error,
        measured_count?(unsigned)(measured_sum/measured_count):0,measured_count?(unsigned)measured[(measured_count-1)*50u/100u]:0,
        measured_count?(unsigned)measured[((measured_count*95u+99u)/100u)-1u]:0,measured_count?(unsigned)measured[((measured_count*99u+99u)/100u)-1u]:0,
        (unsigned)maximum,(unsigned)late,(unsigned)check.peak_live,low);
    check.vector=0;return pass && deadline;
}
#endif
static void run(void *unused){
    (void)unused;check.passed=true;check.live=check.overall_peak=check.overall_max_us=0;
    check.overall_min_free=(uint32_t)heap_caps_get_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
    dbt_codec_hooks_t hooks={0,allocate,release,pcm};
    if(dbt_codec_init(&check.decoder,hooks)!=DBT_OK)check.passed=false;
    unsigned completed=0,selected=0,callbacks=0;
#ifdef DBT_PROBE_OPUS_BUILD
    unsigned opus_completed=0;bool opus_pass=check.passed;
    for(unsigned i=0;opus_pass && i<sizeof(opus_vectors)/sizeof(opus_vectors[0]);++i){
        if(run_opus(&opus_vectors[i],i)){++opus_completed;++completed;callbacks+=check.callbacks;}
        else opus_pass=false;
    }
    selected+=(unsigned)(sizeof(opus_vectors)/sizeof(opus_vectors[0]));
    unsigned opus_low=(unsigned)uxTaskGetStackHighWaterMark(NULL);
    if(opus_low<CHECK_STACK_FLOOR_BYTES)opus_pass=false;
    printf("{\"probe\":\"target_opus_selftest\",\"passed\":%s,\"configurations\":%u,\"frames\":%u,\"stack_bytes\":8192,\"stack_low_water_bytes\":%u,\"maximum_decode_us\":%u,\"rf_test\":false}\n",
        opus_pass?"true":"false",opus_completed,opus_completed*3u,opus_low,(unsigned)check.overall_max_us);
    if(!opus_pass)check.passed=false;
#endif
    for(unsigned i=0;i<sizeof(probe_codec_vectors)/sizeof(probe_codec_vectors[0]);++i){
        const probe_codec_vector_t *v=&probe_codec_vectors[i];if(!v->name)continue;++selected;
        if(check.live || check.decoder.allocated_bytes){check.passed=false;continue;}
        if(run_vector(v)){++completed;callbacks+=check.callbacks;}else check.passed=false;
    }
    dbt_codec_close(&check.decoder);
#ifdef DBT_PROBE_LHDC_MAX_CHECK
    if(!run_lhdc_maximum())check.passed=false;
#endif
    unsigned low=(unsigned)uxTaskGetStackHighWaterMark(NULL);
    if(!selected || low<CHECK_STACK_FLOOR_BYTES || check.live || check.decoder.allocated_bytes || completed!=selected)check.passed=false;
    printf("{\"probe\":\"target_codec_selftest\",\"passed\":%s,\"configurations\":%u,\"selected\":%u,\"callbacks\":%u,\"stack_bytes\":8192,\"stack_low_water_bytes\":%u,\"tracked_peak_bytes\":%u,\"min_free_after_alloc_bytes\":%u,\"maximum_decode_us\":%u,\"rf_test\":false}\n",
        check.passed?"true":"false",completed,selected,callbacks,low,(unsigned)check.overall_peak,(unsigned)check.overall_min_free,(unsigned)check.overall_max_us);
    /* Save completion before releasing the single-run admission flag. The
     * callback may admit the next run, so touch no shared state afterwards. */
    probe_codec_complete_fn complete=check.complete;
    void *context=check.complete_context;
    bool passed=check.passed;
    atomic_store_explicit(&check.done,true,memory_order_release);
    atomic_store_explicit(&check.running,false,memory_order_release);
    if(complete)complete(context,passed);
    vTaskDelete(NULL);
}
bool probe_codec_check_start(probe_codec_complete_fn complete,void *context){
    if(atomic_exchange_explicit(&check.running,true,memory_order_acq_rel))return false;
    atomic_store_explicit(&check.done,false,memory_order_release);
    check.complete=complete;check.complete_context=context;
    if(xTaskCreatePinnedToCore(run,"codec_check",CHECK_STACK_BYTES,NULL,5,NULL,1)!=pdPASS){
        atomic_store_explicit(&check.running,false,memory_order_release);return false;
    }
    return true;
}
static void notify_complete(void *context,bool passed){
    (void)passed;
    if(context)xTaskNotifyGive((TaskHandle_t)context);
}
bool probe_codec_check(TaskHandle_t owner){
    if(!owner || owner!=xTaskGetCurrentTaskHandle())return false;
    if(!probe_codec_check_start(notify_complete,owner))return false;
    int64_t deadline=esp_timer_get_time()+60000000;
    while(!atomic_load_explicit(&check.done,memory_order_acquire)){
        int64_t remaining=deadline-esp_timer_get_time();
        if(remaining<=0){
            printf("{\"probe\":\"target_codec_selftest\",\"passed\":false,\"timeout\":true,\"rf_test\":false}\n");
            return false;
        }
        TickType_t wait=pdMS_TO_TICKS((uint32_t)((remaining+999)/1000));
        ulTaskNotifyTake(pdTRUE,wait?wait:1);
    }
    return check.passed;
}
