#include "dbt_esp32_affinity.h"
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
#include "probe_diagnostic_stats.h"
_Static_assert(sizeof(lhdc_max_packets)/sizeof(lhdc_max_packets[0])==PROBE_DIAGNOSTIC_SAMPLES_MAX,"Maximum LHDC fixture must match bounded diagnostic storage");
#endif
#ifdef DBT_PROBE_LHDC_STAGE_PROFILE
#include "probe_lhdc_profile.h"
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
    uint32_t timestamp_hash,deferred_calls,deferred_busy;
#ifdef DBT_PROBE_LHDC_MAX_CHECK
    uint32_t prefix_hash,prefix_frames;
#endif
    bool deferred;
    uint32_t live,peak_live,overall_peak,case_max_us,overall_max_us,min_free,overall_min_free;
    uint32_t case_max_cycles,case_deadline_misses,media_bytes;
    uint64_t case_cycles,case_us,callback_us,callback_cycles;
    uint32_t last_media_us,last_callback_us,last_media_cycles,last_callback_cycles;
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
static dbt_result_t validate_pcm(void *context,const void *samples,uint16_t frames,const dbt_codec_config_t *config,uint32_t timestamp){
    (void)context;(void)timestamp;
    if(!samples || !config || !frames)return DBT_INVALID;
    for(unsigned i=0;i<frames;++i)check.timestamp_hash=(check.timestamp_hash^(timestamp+i))*16777619u;
#ifdef DBT_PROBE_OPUS_BUILD
    if(check.opus){
        if(config->sample_bytes!=2 || config->rate!=48000 || config->channels!=check.opus->channels || (!check.deferred && frames!=check.opus->samples))return DBT_INVALID;
        const uint8_t *p=samples;
        for(unsigned i=0;i<(unsigned)frames*config->channels*2u;++i)check.hash=(check.hash^p[i])*16777619u;
        ++check.callbacks;check.sample_frames+=frames;return DBT_OK;
    }
#endif
    const probe_codec_vector_t *v=check.vector;
    if(!v || config->sample_bytes!=v->width || config->rate!=v->rate || config->channels!=v->channels ||
       (!check.deferred && v->callback_samples && frames!=v->callback_samples) || check.sample_frames>UINT32_MAX-frames)return DBT_INVALID;
    const uint8_t *bytes=samples;unsigned count=(unsigned)frames*v->channels;
#ifdef DBT_PROBE_LHDC_MAX_CHECK
    unsigned prefix=check.sample_frames<512u?512u-check.sample_frames:0;
    if(prefix>frames)prefix=frames;
    unsigned prefix_bytes=prefix*v->channels*v->width;
    for(unsigned i=0;i<prefix_bytes;++i)check.hash=(check.hash^bytes[i])*16777619u;
    if(prefix){check.prefix_hash=check.hash;check.prefix_frames=check.sample_frames+prefix;}
    for(unsigned i=prefix_bytes;i<count*v->width;++i)check.hash=(check.hash^bytes[i])*16777619u;
#else
    for(unsigned i=0;i<count*v->width;++i)check.hash=(check.hash^bytes[i])*16777619u;
#endif
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
static inline uint32_t read_cycles(void){
#ifdef ESP_PLATFORM
    return esp_cpu_get_cycle_count();
#else
    return 0;
#endif
}
static dbt_result_t pcm(void *context,const void *samples,uint16_t frames,const dbt_codec_config_t *config,uint32_t timestamp){
    if(check.deferred && ++check.deferred_calls%3u==0){++check.deferred_busy;return DBT_BUSY;}
    int64_t started=esp_timer_get_time();uint32_t cycles=read_cycles();
    dbt_result_t result=validate_pcm(context,samples,frames,config,timestamp);
    uint32_t duration_cycles=read_cycles()-cycles;
    int64_t duration=esp_timer_get_time()-started;
    if(duration<0 || duration>UINT32_MAX)return DBT_INVALID;
    check.callback_us+=(uint32_t)duration;check.callback_cycles+=duration_cycles;
    return result;
}
static void reset_case(void){
    check.vector=0;
#ifdef DBT_PROBE_OPUS_BUILD
    check.opus=0;
#endif
    check.hash=2166136261u;check.callbacks=check.sample_frames=check.word_index=check.point_index=0;
    check.timestamp_hash=2166136261u;check.deferred_calls=check.deferred_busy=0;check.deferred=false;
#ifdef DBT_PROBE_LHDC_MAX_CHECK
    check.prefix_hash=2166136261u;check.prefix_frames=0;
#endif
    check.peak=check.maximum_error=check.case_max_us=0;check.peak_live=check.live;
    check.case_max_cycles=check.case_deadline_misses=check.media_bytes=0;
    check.case_cycles=check.case_us=0;
    check.callback_us=check.callback_cycles=0;
    check.last_media_us=check.last_callback_us=check.last_media_cycles=check.last_callback_cycles=0;
    check.min_free=(uint32_t)heap_caps_get_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
}
static bool feed(const dbt_codec_ops_t *ops,const uint8_t *packet,uint16_t size,uint16_t sequence,uint32_t timestamp,bool marker){
    dbt_avdtp_media_t view={.payload=packet,.size=size,.sequence=sequence,.ssrc=1,.timestamp=timestamp,.payload_type=96,.marker=marker};
    uint32_t frames_before=check.sample_frames;
    uint64_t callback_before=check.callback_us,callback_cycles_before=check.callback_cycles;
    uint32_t cycles_before=read_cycles();
    int64_t started=esp_timer_get_time();dbt_result_t result=ops->media(&check.decoder,&view,0);
    int64_t delta=esp_timer_get_time()-started;uint32_t elapsed=delta<0?0:delta>UINT32_MAX?UINT32_MAX:(uint32_t)delta;
    uint32_t cycles=read_cycles()-cycles_before;
    check.case_cycles+=cycles;if(cycles>check.case_max_cycles)check.case_max_cycles=cycles;
    check.last_media_us=elapsed;check.last_callback_us=(uint32_t)(check.callback_us-callback_before);
    check.last_media_cycles=cycles;check.last_callback_cycles=(uint32_t)(check.callback_cycles-callback_cycles_before);
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
static bool run_lhdc_maximum(unsigned trial){
    const probe_codec_vector_t vector={.name="lhdcv5_192000_24_1000",.ops=&dbt_lhdcv5_codec,
        .cie=lhdc_max_cie,.cie_size=sizeof(lhdc_max_cie),.channels=2,.width=4,.rate=192000,
        .sample_frames=92160,.minimum_peak=100000000,.callback_samples=960,
        .points=lhdc_max_points,.point_count=sizeof(lhdc_max_points)/sizeof(lhdc_max_points[0])};
    reset_case();check.vector=&vector;dbt_codec_config_t config;
    bool pass=dbt_lhdcv5_configure(vector.cie,vector.cie_size,&config)==DBT_OK;
    if(pass){config.ops=vector.ops;pass=dbt_codec_open(&check.decoder,&config)==DBT_OK;}
    uint32_t measured[96],estimated[96],measured_count=0,callback_max=0;
    uint64_t callback_sum=0,estimated_cycles=0;
#ifdef DBT_PROBE_LHDC_STAGE_PROFILE
    probe_lhdc_profile_reset(false);
#endif
    for(unsigned i=0;pass && i<sizeof(lhdc_max_packets)/sizeof(lhdc_max_packets[0]);++i){
#ifdef DBT_PROBE_LHDC_STAGE_PROFILE
        if(i==12u)probe_lhdc_profile_reset(true);
#endif
        pass=feed(vector.ops,lhdc_max_packets[i].data,lhdc_max_packets[i].size,(uint16_t)i,i*960u,false);
        if(check.last_callback_us>check.last_media_us || check.last_callback_cycles>check.last_media_cycles)pass=false;
        if(pass && i>=12u){
            measured[measured_count]=check.last_media_us;
            estimated[measured_count++]=check.last_media_us-check.last_callback_us;
            callback_sum+=check.last_callback_us;
            if(check.last_callback_us>callback_max)callback_max=check.last_callback_us;
            estimated_cycles+=check.last_media_cycles-check.last_callback_cycles;
        }
        /* Finite diagnostic only: let idle/other tasks progress between input
         * fixtures. This delay is outside the measured media processing span. */
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    if(pass)pass=check.callbacks==96 && check.sample_frames==vector.sample_frames &&
        check.point_index==vector.point_count && check.peak>=vector.minimum_peak;
    dbt_codec_close(&check.decoder);
    if(check.live || check.decoder.allocated_bytes)pass=false;
    unsigned low=(unsigned)uxTaskGetStackHighWaterMark(NULL);if(low<CHECK_STACK_FLOOR_BYTES)pass=false;
    probe_diagnostic_stats_t total={0},estimate={0};
    if(!probe_diagnostic_stats(measured,measured_count,5000,&total) ||
       !probe_diagnostic_stats(estimated,measured_count,5000,&estimate))pass=false;
#ifdef DBT_PROBE_LHDC_STAGE_PROFILE
    probe_lhdc_profile_t stages;
    if(!probe_lhdc_profile_finish(&stages) || stages.channel_calls!=measured_count*2u)pass=false;
    static const char *names[6]={"pre","entropy","mantissa","dequant_sns","imdct","overlap"};
    for(unsigned i=0;i<6;++i)
        printf("{\"probe\":\"lhdc_stage_cycles\",\"trial\":%u,\"stage\":\"%s\",\"channel_calls\":%u,\"sum_cycles\":%llu,\"maximum_cycles\":%u,\"elapsed_core_cycles\":true}\n",
            trial,names[i],(unsigned)stages.channel_calls,(unsigned long long)stages.sum_cycles[i],(unsigned)stages.max_cycles[i]);
#endif
    bool deadline=measured_count==84 && !total.at_or_above_budget && total.maximum<5000u;
    printf("{\"probe\":\"lhdc_max_selftest\",\"trial\":%u,\"correctness_passed\":%s,\"isolated_deadline_passed\":%s,\"rate\":192000,\"sample_depth\":24,\"channels\":2,\"requested_kbps\":1000,\"encoded_payload_bps\":1001600,\"packets\":%u,\"measured_packets\":%u,\"sample_frames\":%u,\"reference_points\":%u,\"maximum_abs_error\":%u,\"mean_us\":%u,\"p50_us\":%u,\"p95_us\":%u,\"p99_us\":%u,\"max_us\":%u,\"budget_us\":5000,\"deadline_misses\":%u,\"tracked_peak_bytes\":%u,\"stack_low_water_bytes\":%u,\"includes_pcm_validation\":true,\"dsp_i2s_test\":false,\"rf_test\":false}\n",
        trial,pass?"true":"false",deadline?"true":"false",(unsigned)check.callbacks,(unsigned)measured_count,
        (unsigned)check.sample_frames,(unsigned)check.point_index,(unsigned)check.maximum_error,
        (unsigned)total.mean,(unsigned)total.p50,(unsigned)total.p95,(unsigned)total.p99,
        (unsigned)total.maximum,(unsigned)total.at_or_above_budget,(unsigned)check.peak_live,low);
    printf("{\"probe\":\"lhdc_decoder_estimate\",\"trial\":%u,\"mean_us\":%u,\"p50_us\":%u,\"p95_us\":%u,\"p99_us\":%u,\"max_us\":%u,\"deadline_misses\":%u,\"mean_cycles\":%u,\"validation_mean_us\":%u,\"validation_max_us\":%u,\"callback_time_subtracted\":true,\"includes_media_wrapper\":true,\"dsp_i2s_test\":false,\"rf_test\":false}\n",
        trial,(unsigned)estimate.mean,(unsigned)estimate.p50,(unsigned)estimate.p95,(unsigned)estimate.p99,(unsigned)estimate.maximum,
        (unsigned)estimate.at_or_above_budget,measured_count?(unsigned)(estimated_cycles/measured_count):0,
        measured_count?(unsigned)(callback_sum/measured_count):0,(unsigned)callback_max);
    printf("{\"probe\":\"lhdc_pcm_hash\",\"trial\":%u,\"sample_frames\":%u,\"fnv1a32\":%u,\"prefix_frames\":%u,\"prefix_fnv1a32\":%u}\n",
        trial,(unsigned)check.sample_frames,(unsigned)check.hash,(unsigned)check.prefix_frames,(unsigned)check.prefix_hash);
    check.vector=0;return pass && deadline;
}
#endif
#ifdef DBT_PROBE_DEFERRED_CODEC_CHECK
static dbt_codec_worker_t deferred_worker;
static bool deferred_control(dbt_sink_operation_t operation,const uint8_t *configuration,uint16_t size){
    dbt_sink_job_t job={1,1,operation,1,configuration,size};
    if(dbt_codec_worker_submit(&deferred_worker,&job)!=DBT_OK)return false;
    dbt_codec_worker_step(&deferred_worker,(uint32_t)(esp_timer_get_time()/1000),0);
    dbt_codec_receipt_t receipt;
    return dbt_codec_worker_receipt(&deferred_worker,&receipt)==DBT_OK && receipt.result==DBT_SINK_WORK_READY;
}
static bool deferred_packet(const probe_codec_vector_t *v,unsigned opus_index,bool maximum,unsigned i,
                            const uint8_t **data,uint16_t *size,uint32_t *timestamp,bool *marker){
    (void)opus_index;
    if(v){*data=v->packet[i];*size=v->bytes[i];*timestamp=i*(v->callback_samples?v->callback_samples:128u);*marker=v->marker!=0;return true;}
#ifdef DBT_PROBE_LHDC_MAX_CHECK
    if(maximum){*data=lhdc_max_packets[i].data;*size=lhdc_max_packets[i].size;*timestamp=i*960u;*marker=false;return true;}
#else
    (void)maximum;
#endif
#ifdef DBT_PROBE_OPUS_BUILD
    if(opus_index<sizeof(opus_vectors)/sizeof(opus_vectors[0])){
        const opus_vector_t *o=&opus_vectors[opus_index];*data=o->packet[i];*size=o->bytes[i];*timestamp=i*o->samples;*marker=false;return true;
    }
#endif
    return false;
}
static bool run_deferred(const probe_codec_vector_t *v,unsigned opus_index,bool maximum,uint32_t expected_hash,uint32_t expected_timestamps){
    if(check.live || check.decoder.allocated_bytes)return false;
    reset_case();check.deferred=true;check.vector=v;
    const dbt_codec_ops_t *ops=v?v->ops:NULL;const uint8_t *cie=v?v->cie:NULL;
    uint16_t cie_size=v?v->cie_size:0,packets=v?v->packet_count:0;
    uint32_t expected_frames=v?v->sample_frames:0;
#ifdef DBT_PROBE_LHDC_MAX_CHECK
    const probe_codec_vector_t max_vector={.name="lhdcv5_192000_24_1000",.ops=&dbt_lhdcv5_codec,
        .cie=lhdc_max_cie,.cie_size=sizeof(lhdc_max_cie),.channels=2,.width=4,.rate=192000,
        .sample_frames=92160,.minimum_peak=100000000,.callback_samples=960,
        .points=lhdc_max_points,.point_count=sizeof(lhdc_max_points)/sizeof(lhdc_max_points[0])};
    if(maximum){check.vector=&max_vector;ops=max_vector.ops;cie=max_vector.cie;cie_size=max_vector.cie_size;packets=96;expected_frames=92160;}
#else
    if(maximum)return false;
#endif
#ifdef DBT_PROBE_OPUS_BUILD
    if(!v && !maximum && opus_index<sizeof(opus_vectors)/sizeof(opus_vectors[0])){
        check.opus=&opus_vectors[opus_index];ops=&dbt_opus_codec;cie=check.opus->cie;cie_size=check.opus->cie_size;packets=3;expected_frames=3u*check.opus->samples;
    }
#endif
    bool pass=ops && cie && packets && cie_size<=DBT_CODEC_CIE_BYTES;uint16_t payload=5;
    for(unsigned i=0;pass && i<packets;++i){const uint8_t *data;uint16_t n;uint32_t timestamp;bool marker;
        pass=deferred_packet(v,opus_index,maximum,i,&data,&n,&timestamp,&marker);
        if(pass && n>payload)payload=n;
    }
    const dbt_codec_ops_t *codecs[]={ops};dbt_codec_worker_hooks_t hooks={0};
    hooks.decoder=(dbt_codec_hooks_t){0,allocate,release,pcm};hooks.codecs=codecs;hooks.codec_count=1;hooks.deferred_pcm=true;
    bool initialized=pass && dbt_codec_worker_init(&deferred_worker,hooks,payload,2)==DBT_OK;
    pass=initialized;uint32_t maximum_step=0;
    if(pass){uint8_t configuration[4+DBT_CODEC_CIE_BYTES]={1,0,7,0};configuration[3]=(uint8_t)cie_size;
        for(unsigned i=0;i<cie_size;++i)configuration[4+i]=cie[i];
        pass=deferred_control(DBT_SINK_CONFIGURE,configuration,(uint16_t)(4u+cie_size)) && deferred_control(DBT_SINK_START,NULL,0);
    }
    for(unsigned i=0;pass && i<packets;++i){const uint8_t *data;uint16_t n;uint32_t timestamp;bool marker;
        pass=deferred_packet(v,opus_index,maximum,i,&data,&n,&timestamp,&marker);
        if(!pass)break;
        dbt_avdtp_media_t packet={data,timestamp,1,n,(uint16_t)i,96,marker};
        pass=dbt_codec_worker_media(&deferred_worker,&packet)==DBT_OK;
        for(unsigned work=0;pass && work<16384 && atomic_load(&deferred_worker.media_read)!=atomic_load(&deferred_worker.media_written);++work){
            int64_t began=esp_timer_get_time();uint16_t made=dbt_codec_worker_step(&deferred_worker,(uint32_t)(began/1000),1);
            int64_t duration=esp_timer_get_time()-began;
            if(duration<0 || duration>UINT32_MAX || made>1)pass=false;
            else if((uint32_t)duration>maximum_step)maximum_step=(uint32_t)duration;
            if(atomic_load(&deferred_worker.media_drops) || atomic_load(&deferred_worker.faults))pass=false;
            if(!made || deferred_worker.output_blocked)vTaskDelay(pdMS_TO_TICKS(1));else taskYIELD();
        }
        if(atomic_load(&deferred_worker.media_read)!=atomic_load(&deferred_worker.media_written))pass=false;
    }
    if(pass)pass=check.sample_frames==expected_frames && check.hash==expected_hash && check.timestamp_hash==expected_timestamps && check.deferred_busy;
    if(initialized && !deferred_control(DBT_SINK_DROP,NULL,0))pass=false;
    if(check.live || (initialized && deferred_worker.decoder.allocated_bytes))pass=false;
    unsigned low=(unsigned)uxTaskGetStackHighWaterMark(NULL);if(low<CHECK_STACK_FLOOR_BYTES)pass=false;
    printf("{\"probe\":\"target_deferred_codec\",\"codec\":\"%s\",\"index\":%u,\"passed\":%s,\"sample_frames\":%u,\"pcm_hash\":%u,\"timestamp_hash\":%u,\"busy_attempts\":%u,\"maximum_step_us\":%u,\"cold_initialization_included\":true,\"tracked_peak_bytes\":%u,\"stack_low_water_bytes\":%u,\"simulated_output_backpressure\":true,\"i2s_test\":false,\"rf_test\":false}\n",
        ops?ops->name:"invalid",opus_index,pass?"true":"false",(unsigned)check.sample_frames,(unsigned)check.hash,(unsigned)check.timestamp_hash,
        (unsigned)check.deferred_busy,(unsigned)maximum_step,(unsigned)check.peak_live,low);
    check.deferred=false;check.vector=NULL;
#ifdef DBT_PROBE_OPUS_BUILD
    check.opus=NULL;
#endif
    return pass;
}
#endif
static void run(void *unused){
    (void)unused;check.passed=true;check.live=check.overall_peak=check.overall_max_us=0;
    check.overall_min_free=(uint32_t)heap_caps_get_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
    dbt_codec_hooks_t hooks={0,allocate,release,pcm};
    if(dbt_codec_init(&check.decoder,hooks)!=DBT_OK)check.passed=false;
    unsigned completed=0,selected=0,callbacks=0;
#ifdef DBT_PROBE_DEFERRED_CODEC_CHECK
    uint32_t vector_hashes[sizeof(probe_codec_vectors)/sizeof(probe_codec_vectors[0])]={0},vector_timestamps[sizeof(probe_codec_vectors)/sizeof(probe_codec_vectors[0])]={0};
#ifdef DBT_PROBE_OPUS_BUILD
    uint32_t opus_hashes[sizeof(opus_vectors)/sizeof(opus_vectors[0])],opus_timestamps[sizeof(opus_vectors)/sizeof(opus_vectors[0])];
    for(unsigned i=0;i<sizeof(opus_hashes)/sizeof(opus_hashes[0]);++i)opus_hashes[i]=opus_timestamps[i]=0;
#endif
#endif
#ifdef DBT_PROBE_OPUS_BUILD
    unsigned opus_completed=0;bool opus_pass=check.passed;
    for(unsigned i=0;opus_pass && i<sizeof(opus_vectors)/sizeof(opus_vectors[0]);++i){
        if(run_opus(&opus_vectors[i],i)){++opus_completed;++completed;callbacks+=check.callbacks;}
        else opus_pass=false;
#ifdef DBT_PROBE_DEFERRED_CODEC_CHECK
        opus_hashes[i]=check.hash;opus_timestamps[i]=check.timestamp_hash;
#endif
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
#ifdef DBT_PROBE_DEFERRED_CODEC_CHECK
        vector_hashes[i]=check.hash;vector_timestamps[i]=check.timestamp_hash;
#endif
    }
    dbt_codec_close(&check.decoder);
#ifdef DBT_PROBE_LHDC_MAX_CHECK
    for(unsigned trial=0;trial<3;++trial)if(!run_lhdc_maximum(trial))check.passed=false;
#endif
#ifdef DBT_PROBE_DEFERRED_CODEC_CHECK
#ifdef DBT_PROBE_LHDC_MAX_CHECK
    uint32_t maximum_hash=check.hash,maximum_timestamps=check.timestamp_hash;
#endif
    for(unsigned i=0;i<sizeof(probe_codec_vectors)/sizeof(probe_codec_vectors[0]);++i)if(probe_codec_vectors[i].name)
        if(!run_deferred(&probe_codec_vectors[i],i,false,vector_hashes[i],vector_timestamps[i]))check.passed=false;
#ifdef DBT_PROBE_OPUS_BUILD
    for(unsigned i=0;i<sizeof(opus_vectors)/sizeof(opus_vectors[0]);++i)
        if(!run_deferred(NULL,i,false,opus_hashes[i],opus_timestamps[i]))check.passed=false;
#endif
#ifdef DBT_PROBE_LHDC_MAX_CHECK
    if(!run_deferred(NULL,0,true,maximum_hash,maximum_timestamps))check.passed=false;
#endif
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
    if(xTaskCreatePinnedToCore(run,"codec_check",CHECK_STACK_BYTES,NULL,5,NULL,DBT_ESP32_DECODER_CORE)!=pdPASS){
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
