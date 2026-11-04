#include "dbt_codec.h"
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
#include "esp_heap_caps.h"
#include "esp_timer.h"
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
    dbt_codec_decoder_t decoder;
    const probe_codec_vector_t *vector;
#ifdef DBT_PROBE_OPUS_BUILD
    const opus_vector_t *opus;
#endif
    uint32_t hash,callbacks,sample_frames,word_index,point_index,peak,maximum_error;
    uint32_t live,peak_live,overall_peak,case_max_us,overall_max_us,min_free,overall_min_free;
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
    check.min_free=(uint32_t)heap_caps_get_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
}
static bool feed(const dbt_codec_ops_t *ops,const uint8_t *packet,uint16_t size,uint16_t sequence,uint32_t timestamp,bool marker){
    dbt_avdtp_media_t view={.payload=packet,.size=size,.sequence=sequence,.ssrc=1,.timestamp=timestamp,.payload_type=96,.marker=marker};
    int64_t started=esp_timer_get_time();dbt_result_t result=ops->media(&check.decoder,&view,0);
    int64_t delta=esp_timer_get_time()-started;uint32_t elapsed=delta<0?0:delta>UINT32_MAX?UINT32_MAX:(uint32_t)delta;
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
    printf("{\"probe\":\"target_codec_case\",\"codec\":\"%s\",\"passed\":%s,\"rate\":%u,\"channels\":%u,\"sample_bytes\":%u,\"packets\":%u,\"callbacks\":%u,\"sample_frames\":%u,\"tracked_peak_bytes\":%u,\"min_free_after_alloc_bytes\":%u,\"stack_low_water_bytes\":%u,\"maximum_decode_us\":%u,\"comparison\":\"%s\",\"maximum_abs_error\":%u}\n",
        v->name,pass?"true":"false",(unsigned)v->rate,v->channels,v->width,v->packet_count,
        (unsigned)check.callbacks,(unsigned)check.sample_frames,(unsigned)check.peak_live,(unsigned)check.min_free,low,(unsigned)check.case_max_us,
        v->exact?"exact":"bounded_samples",(unsigned)check.maximum_error);
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
    unsigned low=(unsigned)uxTaskGetStackHighWaterMark(NULL);
    if(!selected || low<CHECK_STACK_FLOOR_BYTES || check.live || check.decoder.allocated_bytes || completed!=selected)check.passed=false;
    printf("{\"probe\":\"target_codec_selftest\",\"passed\":%s,\"configurations\":%u,\"selected\":%u,\"callbacks\":%u,\"stack_bytes\":8192,\"stack_low_water_bytes\":%u,\"tracked_peak_bytes\":%u,\"min_free_after_alloc_bytes\":%u,\"maximum_decode_us\":%u,\"rf_test\":false}\n",
        check.passed?"true":"false",completed,selected,callbacks,low,(unsigned)check.overall_peak,(unsigned)check.overall_min_free,(unsigned)check.overall_max_us);
    atomic_store_explicit(&check.done,true,memory_order_release);
    atomic_store_explicit(&check.running,false,memory_order_release);
    vTaskDelete(NULL);
}
bool probe_codec_check(TaskHandle_t owner){
    (void)owner;
    if(atomic_exchange_explicit(&check.running,true,memory_order_acq_rel))return false;
    atomic_store_explicit(&check.done,false,memory_order_release);
    if(xTaskCreatePinnedToCore(run,"codec_check",CHECK_STACK_BYTES,NULL,5,NULL,1)!=pdPASS){
        atomic_store_explicit(&check.running,false,memory_order_release);return false;
    }
    int64_t deadline=esp_timer_get_time()+60000000;
    while(!atomic_load_explicit(&check.done,memory_order_acquire)){
        if(esp_timer_get_time()>=deadline){
            printf("{\"probe\":\"target_codec_selftest\",\"passed\":false,\"timeout\":true,\"rf_test\":false}\n");
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    return check.passed;
}
