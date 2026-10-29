#include "probe_audio.h"
#include "dbt_codec.h"
#include "dbt_sbc_codec.h"
#ifdef DBT_PROBE_OPUS_BUILD
#include "dbt_opus.h"
#define AUDIO_OPUS_JSON ",\"Android Opus\",\"Opus05\""
#else
#define AUDIO_OPUS_JSON ""
#endif
#ifdef DBT_PROBE_AAC_BUILD
#include "dbt_aac.h"
#define AUDIO_AAC_JSON ",\"AAC-LC\""
#else
#define AUDIO_AAC_JSON ""
#endif
#ifdef DBT_PROBE_APTX_BUILD
#include "dbt_aptx.h"
#define AUDIO_APTX_JSON ",\"aptX\",\"aptX HD\",\"aptX LL\""
#else
#define AUDIO_APTX_JSON ""
#endif
#ifdef DBT_PROBE_LDAC_BUILD
#include "dbt_ldac.h"
#define AUDIO_LDAC_JSON ",\"LDAC\""
#else
#define AUDIO_LDAC_JSON ""
#endif
#ifdef DBT_PROBE_LHDC_BUILD
#include "dbt_lhdcv5_codec.h"
#define AUDIO_LHDC_JSON ",\"LHDC V5\""
#else
#define AUDIO_LHDC_JSON ""
#endif
#define AUDIO_CODECS_JSON "[\"SBC\"" AUDIO_OPUS_JSON AUDIO_AAC_JSON AUDIO_APTX_JSON AUDIO_LDAC_JSON AUDIO_LHDC_JSON "]"
#ifdef DBT_PROBE_AVRCP
#include "probe_avrcp.h"
#endif
#ifdef DBT_PROBE_AUDIO_OUTPUT
#include "probe_output.h"
#define AUDIO_OUTPUT_JSON "true"
#else
#define AUDIO_OUTPUT_JSON "false"
#endif
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include <inttypes.h>
#include <stdio.h>

static dbt_a2dp_acceptor_t profile;
static dbt_codec_worker_t worker;
static TaskHandle_t owner,decoder_task,logger_task;
static bool initialized,failed;
static _Atomic uint32_t pcm_generation,pcm_frames,pcm_bytes,pcm_hash,pcm_rate,pcm_channels,pcm_codec,pcm_sample_bytes;
static _Atomic uint32_t profile_snapshot,event_errors,last_channel_event,last_channel_result;
static _Atomic uint32_t ignored_dynamic,oversized_dynamic;
/* Each endpoint is validated independently before an asynchronous worker job.
 * Opus 05 offers forward mono/stereo only; reverse audio is not supported. */
static const uint8_t sbc_capabilities[]={1,0,7,6,0,0,0x3f,0xff,2,53,8,0};
#ifdef DBT_PROBE_OPUS_BUILD
static const uint8_t android_capabilities[]={1,0,7,9,0,0xff,0xe0,0,0,0,1,0,0x9b,8,0};
static const uint8_t opus05_capabilities[]={1,0,7,26,0,0xff,0xf1,5,0,0,5,0x10,2,1,3,0,0,0,
                                             0x1f,0,0,0,0,0,0,0,0,0,0,0,8,0};
#endif
#ifdef DBT_PROBE_AAC_BUILD
/* MPEG-2/4 AAC-LC, 44.1/48 kHz mono/stereo, VBR, up to 320 kbit/s. */
static const uint8_t aac_capabilities[]={1,0,7,8,0,2,0xc0,1,0x8c,0x84,0xe2,0,8,0};
#endif
#ifdef DBT_PROBE_APTX_BUILD
/* Only stereo, 44.1/48 kHz and no aptX LL reverse channel/extensions. */
static const uint8_t aptx_capabilities[]={1,0,7,9,0,0xff,0x4f,0,0,0,1,0,0x32,8,0};
static const uint8_t aptx_hd_capabilities[]={1,0,7,13,0,0xff,0xd7,0,0,0,0x24,0,0x32,0,0,0,0,8,0};
static const uint8_t aptx_ll_capabilities[]={1,0,7,10,0,0xff,0x0a,0,0,0,2,0,0x32,0,8,0};
#endif
#ifdef DBT_PROBE_LDAC_BUILD
static const uint8_t ldac_capabilities[]={1,0,7,10,0,0xff,0x2d,1,0,0,0xaa,0,0x3c,7,8,0};
#endif
#ifdef DBT_PROBE_LHDC_BUILD
/* Positive encoded-frame coverage currently covers 16/24-bit stereo. */
static const uint8_t lhdc_capabilities[]={1,0,7,13,0,0xff,0x3a,5,0,0,0x35,0x4c,0x35,6,0x11,0x40,0,8,0};
#endif
static const dbt_avdtp_endpoint_t endpoints[]={
    {sbc_capabilities,sizeof(sbc_capabilities),1,dbt_a2dp_sbc_validate},
#ifdef DBT_PROBE_OPUS_BUILD
    {android_capabilities,sizeof(android_capabilities),2,dbt_a2dp_opus_validate},
    {opus05_capabilities,sizeof(opus05_capabilities),3,dbt_a2dp_opus_validate},
#endif
#ifdef DBT_PROBE_AAC_BUILD
    {aac_capabilities,sizeof(aac_capabilities),4,dbt_a2dp_aac_validate},
#endif
#ifdef DBT_PROBE_APTX_BUILD
    {aptx_capabilities,sizeof(aptx_capabilities),5,dbt_a2dp_aptx_validate},
    {aptx_hd_capabilities,sizeof(aptx_hd_capabilities),6,dbt_a2dp_aptx_validate},
    {aptx_ll_capabilities,sizeof(aptx_ll_capabilities),7,dbt_a2dp_aptx_validate},
#endif
#ifdef DBT_PROBE_LDAC_BUILD
    {ldac_capabilities,sizeof(ldac_capabilities),8,dbt_a2dp_ldac_validate},
#endif
#ifdef DBT_PROBE_LHDC_BUILD
    {lhdc_capabilities,sizeof(lhdc_capabilities),9,dbt_a2dp_lhdcv5_validate},
#endif
};
static const dbt_codec_ops_t *const codecs[]={&dbt_sbc_codec,
#ifdef DBT_PROBE_OPUS_BUILD
    &dbt_opus_codec,
#endif
#ifdef DBT_PROBE_AAC_BUILD
    &dbt_aac_codec,
#endif
#ifdef DBT_PROBE_APTX_BUILD
    &dbt_aptx_codec,
#endif
#ifdef DBT_PROBE_LDAC_BUILD
    &dbt_ldac_codec,
#endif
#ifdef DBT_PROBE_LHDC_BUILD
    &dbt_lhdcv5_codec,
#endif
};
static uint32_t now_ms(void){return (uint32_t)(esp_timer_get_time()/1000);}
static uint8_t *allocate(void *context,uint16_t bytes){(void)context;return heap_caps_malloc(bytes,MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);}
static uint8_t *codec_allocate(void *context,uint32_t bytes){(void)context;return heap_caps_aligned_alloc(8,bytes,MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);}
static void release(void *context,uint8_t *bytes){(void)context;heap_caps_free(bytes);}
static void wake_worker(void *context){(void)context;if(decoder_task)xTaskNotifyGive(decoder_task);}
static void wake_owner(void *context){(void)context;xTaskNotifyGive(owner);}
static dbt_result_t pcm(void *context,const void *samples,uint16_t count,const dbt_codec_config_t *config,uint32_t timestamp){
    (void)context;(void)timestamp;
    if(!config || !samples || !count || config->sample_bytes<2 || config->sample_bytes>4 ||
       config->channels<1 || config->channels>2 || !config->rate)return DBT_INVALID;
    uint8_t channels=config->channels;uint32_t rate=config->rate,bytes=(uint32_t)config->sample_bytes*count*channels;
#ifdef DBT_PROBE_AUDIO_OUTPUT
    dbt_result_t result=probe_output_pcm_format(samples,count,channels,rate,(dbt_pcm_format_t)config->sample_bytes);
    if(result!=DBT_OK)return result;
#endif
    atomic_fetch_add_explicit(&pcm_generation,1,memory_order_acq_rel);
    uint32_t hash=atomic_load_explicit(&pcm_hash,memory_order_relaxed);const uint8_t *p=samples;
    for(unsigned i=0;i<bytes;++i)hash=(hash^p[i])*16777619u;
    atomic_fetch_add_explicit(&pcm_frames,1,memory_order_relaxed);
    atomic_fetch_add_explicit(&pcm_bytes,bytes,memory_order_relaxed);
    atomic_store_explicit(&pcm_hash,hash,memory_order_relaxed);
    atomic_store_explicit(&pcm_rate,rate,memory_order_relaxed);
    atomic_store_explicit(&pcm_channels,channels,memory_order_relaxed);
    atomic_store_explicit(&pcm_sample_bytes,config->sample_bytes,memory_order_relaxed);
    atomic_fetch_add_explicit(&pcm_generation,1,memory_order_release);
    return DBT_OK;
}
static dbt_result_t stream(void *context,bool started,const dbt_codec_config_t *config){
    (void)context;
    if(!config)return DBT_INVALID;
#ifdef DBT_PROBE_AUDIO_OUTPUT
    dbt_result_t result=probe_output_stream_format(started,config->rate,config->channels,(dbt_pcm_format_t)config->sample_bytes);
    if(result!=DBT_OK)return result;
#endif
    if(started){
        uint32_t codec=config->ops==&dbt_sbc_codec?1u:0u;
#ifdef DBT_PROBE_OPUS_BUILD
        if(config->ops==&dbt_opus_codec)codec=config->cie_size==9?2u:3u;
#endif
#ifdef DBT_PROBE_AAC_BUILD
        if(config->ops==&dbt_aac_codec)codec=4;
#endif
#ifdef DBT_PROBE_APTX_BUILD
        if(config->ops==&dbt_aptx_codec)codec=config->cie_size==9?5u:config->cie_size==13?6u:7u;
#endif
#ifdef DBT_PROBE_LDAC_BUILD
        if(config->ops==&dbt_ldac_codec)codec=8;
#endif
#ifdef DBT_PROBE_LHDC_BUILD
        if(config->ops==&dbt_lhdcv5_codec)codec=9;
#endif
        atomic_fetch_add_explicit(&pcm_generation,1,memory_order_acq_rel);
        atomic_store_explicit(&pcm_frames,0,memory_order_relaxed);
        atomic_store_explicit(&pcm_bytes,0,memory_order_relaxed);
        atomic_store_explicit(&pcm_hash,2166136261u,memory_order_relaxed);
        atomic_store_explicit(&pcm_codec,codec,memory_order_relaxed);
        atomic_store_explicit(&pcm_rate,config->rate,memory_order_relaxed);
        atomic_store_explicit(&pcm_channels,config->channels,memory_order_relaxed);
        atomic_store_explicit(&pcm_sample_bytes,config->sample_bytes,memory_order_relaxed);
        atomic_fetch_add_explicit(&pcm_generation,1,memory_order_release);
    }
    return DBT_OK;
}
static void decode_loop(void *unused){
    (void)unused;
    uint32_t seen_faults=0;
    for(;;){
        uint16_t done=dbt_codec_worker_step(&worker,now_ms(),2);
        uint32_t faults=atomic_load_explicit(&worker.faults,memory_order_acquire);
        if(faults!=seen_faults){seen_faults=faults;wake_owner(0);}
        if(done==2){taskYIELD();continue;}
        ulTaskNotifyTake(pdTRUE,pdMS_TO_TICKS(20));
    }
}
#ifdef DBT_PROBE_AVRCP
/* Bounded UTF-8 strings are escaped for JSON on the low-priority logger.
 * The output buffer is allocated only when a new revision needs reporting. */
static void metadata_escape(char *out,const char *in){
    static const char hex[]="0123456789abcdef";
    unsigned at=0;
    for(unsigned i=0;i<PROBE_AVRCP_TEXT_BYTES && in[i];++i){
        unsigned char c=(unsigned char)in[i];
        if(c=='"' || c=='\\'){out[at++]='\\';out[at++]=(char)c;}
        else if(c<0x20){out[at++]='\\';out[at++]='u';out[at++]='0';out[at++]='0';out[at++]=hex[c>>4];out[at++]=hex[c&15];}
        else out[at++]=(char)c;
    }
    out[at]=0;
}
static bool log_metadata(void){
    static probe_avrcp_snapshot_t metadata;
    static uint32_t previous_metadata;
    if(probe_avrcp_snapshot(&metadata)!=DBT_OK || metadata.metadata_updates==previous_metadata)return false;
    const size_t stride=PROBE_AVRCP_TEXT_BYTES*6u+1u;
    char *escaped=heap_caps_malloc(3u*stride,MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
    if(!escaped)return false;
    metadata_escape(escaped,metadata.title);metadata_escape(escaped+stride,metadata.artist);metadata_escape(escaped+2u*stride,metadata.album);
    printf("{\"probe\":\"avrcp_metadata\",\"revision\":%" PRIu32 ",\"generation\":%" PRIu32
           ",\"truncated\":%u,\"title\":\"%s\",\"artist\":\"%s\",\"album\":\"%s\"}\n",
           metadata.metadata_updates,metadata.generation,(unsigned)metadata.metadata_truncated,escaped,escaped+stride,escaped+2u*stride);
    previous_metadata=metadata.metadata_updates;
    heap_caps_free(escaped);return true;
}
#endif
static void log_loop(void *unused){
    (void)unused;
    uint32_t previous=UINT32_MAX,previous_errors=UINT32_MAX,previous_ignored=UINT32_MAX,previous_oversized=UINT32_MAX;
    for(;;){
        vTaskDelay(pdMS_TO_TICKS(1000));
        uint32_t generation=atomic_load_explicit(&pcm_generation,memory_order_acquire);
        if(generation&1u)continue;
        uint32_t frames=atomic_load_explicit(&pcm_frames,memory_order_relaxed),bytes=atomic_load_explicit(&pcm_bytes,memory_order_relaxed);
        uint32_t hash=atomic_load_explicit(&pcm_hash,memory_order_relaxed),rate=atomic_load_explicit(&pcm_rate,memory_order_relaxed);
        uint32_t channels=atomic_load_explicit(&pcm_channels,memory_order_relaxed),codec=atomic_load_explicit(&pcm_codec,memory_order_relaxed);
        uint32_t sample_bytes=atomic_load_explicit(&pcm_sample_bytes,memory_order_relaxed);
        if(generation!=atomic_load_explicit(&pcm_generation,memory_order_acquire))continue;
        printf("{\"probe\":\"sbc_pcm\",\"frames\":%" PRIu32 ",\"bytes\":%" PRIu32 ",\"fnv1a32\":%" PRIu32
               ",\"rate\":%" PRIu32 ",\"channels\":%" PRIu32 ",\"media_packets\":%" PRIu32 ",\"drops\":%" PRIu32
               ",\"resets\":%" PRIu32 ",\"resident\":%" PRIu32 ",\"codec\":%" PRIu32 ",\"sample_bytes\":%" PRIu32
               ",\"faults\":%" PRIu32 ",\"decoder_stack_low_water_bytes\":%u,\"internal_heap_free\":%u,\"internal_heap_min_free\":%u,\"internal_heap_largest\":%u,\"i2s_output\":" AUDIO_OUTPUT_JSON "}\n",frames,bytes,hash,rate,channels,
               atomic_load(&worker.packets),atomic_load(&worker.media_drops),atomic_load(&worker.resets),atomic_load(&worker.resident_bytes),codec,sample_bytes,
               atomic_load(&worker.faults),(unsigned)uxTaskGetStackHighWaterMark(decoder_task),
               (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT),
               (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT),
               (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT));
        uint32_t snapshot=atomic_load(&profile_snapshot),errors=atomic_load(&event_errors);
        uint32_t ignored=atomic_load(&ignored_dynamic),oversized=atomic_load(&oversized_dynamic);
        if(snapshot!=previous || errors!=previous_errors || ignored!=previous_ignored || oversized!=previous_oversized){
            previous=snapshot;previous_errors=errors;
            previous_ignored=ignored;previous_oversized=oversized;
            printf("{\"probe\":\"native_profile\",\"state\":%u,\"active\":%u,\"enabled\":%u,\"draining\":%u,"
                   "\"event_errors\":%" PRIu32 ",\"last_event\":%" PRIu32 ",\"last_result\":%" PRIu32 ",\"detail\":%" PRIu32
                   ",\"ignored_dynamic\":%" PRIu32 ",\"oversized_dynamic\":%" PRIu32 "}\n",
                   (unsigned)(snapshot&255u),(unsigned)((snapshot>>8)&1u),(unsigned)((snapshot>>9)&1u),
                   (unsigned)((snapshot>>10)&1u),errors,atomic_load(&last_channel_event),atomic_load(&last_channel_result),snapshot>>11,ignored,oversized);
        }
#ifdef DBT_PROBE_AVRCP
        (void)log_metadata();
#endif
#ifdef DBT_PROBE_AUDIO_OUTPUT
        probe_output_stats_t stats;probe_output_stats(&stats);
        printf("{\"probe\":\"native_i2s\",\"running\":%" PRIu32 ",\"failed\":%" PRIu32 ",\"rate\":%" PRIu32
               ",\"requested_bytes\":%" PRIu32 ",\"accepted_frames\":%" PRIu32 ",\"rendered_frames\":%" PRIu32 ",\"ready_frames\":%" PRIu32
               ",\"dma_eofs\":%" PRIu32 ",\"dma_source_frames\":%" PRIu32 ",\"underrun_frames\":%" PRIu32
               ",\"descriptor_errors\":%" PRIu32 ",\"max_eof_gap_us\":%" PRIu32 ",\"maximum_render_us\":%" PRIu32
               ",\"first_underrun_accepted_frames\":%" PRIu32 ",\"first_underrun_dma_frames\":%" PRIu32 "}\n",
               stats.running,stats.failed,stats.rate,stats.requested_bytes,stats.accepted_frames,stats.rendered_frames,stats.ready_frames,
               stats.dma_eofs,stats.dma_source_frames,stats.underrun_frames,stats.descriptor_errors,stats.max_eof_gap_us,stats.maximum_render_us,
               stats.first_underrun_accepted_frames,stats.first_underrun_dma_frames);
        printf("{\"probe\":\"audio_clock\",\"ppm\":%" PRId32 ",\"observed_ppm\":%" PRId32 ",\"error_frames\":%" PRId32
               ",\"state\":%" PRIu32 ",\"target_frames\":%" PRIu32 ",\"updates\":%" PRIu32 ",\"disturbances\":%" PRIu32
               ",\"observed_valid\":%" PRIu32 ",\"snapshot_valid\":%" PRIu32 "}\n",stats.clock_ppm,stats.clock_observed_ppm,stats.clock_error_frames,
               stats.clock_state,stats.clock_target_frames,stats.clock_updates,stats.clock_disturbances,stats.clock_observed_valid,stats.clock_snapshot_valid);
#endif
    }
}
static void sink_event(void *context,dbt_sink_event_t event,dbt_stream_state_t state){(void)context;(void)event;(void)state;}
dbt_result_t probe_audio_init(dbt_runtime_t *runtime,TaskHandle_t host_owner){
    owner=host_owner;
    dbt_codec_worker_hooks_t hooks={0};hooks.decoder=(dbt_codec_hooks_t){0,codec_allocate,release,pcm};
    hooks.codecs=codecs;hooks.codec_count=(uint8_t)(sizeof(codecs)/sizeof(codecs[0]));
    hooks.wake_worker=wake_worker;hooks.wake_owner=wake_owner;hooks.stream=stream;
    dbt_result_t result=dbt_codec_worker_init(&worker,hooks,672,8);
    if(result!=DBT_OK)return result;
    dbt_avdtp_sink_hooks_t client={&worker,allocate,release,dbt_codec_worker_submit,sink_event,dbt_codec_worker_media};
    result=dbt_a2dp_acceptor_init(&profile,runtime,client,endpoints,(uint8_t)(sizeof(endpoints)/sizeof(endpoints[0])),672,64,5000,2000,5000);
    if(result!=DBT_OK)return result;
    if(xTaskCreatePinnedToCore(decode_loop,"native_decoder",8192,0,5,&decoder_task,1)!=pdPASS)return DBT_NO_SPACE;
#ifdef DBT_PROBE_AUDIO_OUTPUT
    result=probe_output_init(wake_worker,0);
    if(result!=DBT_OK){vTaskDelete(decoder_task);decoder_task=0;return result;}
#endif
    if(xTaskCreatePinnedToCore(log_loop,"native_pcm_log",4096,0,1,&logger_task,0)!=pdPASS){vTaskDelete(decoder_task);decoder_task=0;return DBT_NO_SPACE;}
    initialized=true;
    printf("{\"probe\":\"sbc_probe_ready\",\"psm\":25,\"mtu\":672,\"worker_bytes\":%u,\"profile_bytes\":%u,\"decoder_stack\":8192,"
           "\"logger_stack\":4096,\"codecs\":" AUDIO_CODECS_JSON ",\"diagnostic_only\":true,\"audio_sink_sdp_advertised\":" AUDIO_OUTPUT_JSON ",\"i2s_output\":" AUDIO_OUTPUT_JSON "}\n",
           (unsigned)sizeof(worker),(unsigned)sizeof(profile));
    return DBT_OK;
}
bool probe_audio_accept(void *context,const dbt_link_t *link,uint16_t psm){(void)context;return psm!=0x19 || (initialized && dbt_a2dp_acceptor_accept(&profile,link));}
void probe_audio_event(dbt_channel_event_t event,const dbt_channel_t *channel){
    if(!initialized || channel->psm!=0x19)return;
    dbt_result_t result=dbt_a2dp_acceptor_event(&profile,event,channel);
    if(result!=DBT_OK && result!=DBT_EMPTY){
        atomic_store(&last_channel_event,(uint32_t)event);atomic_store(&last_channel_result,(uint32_t)result);
        atomic_fetch_add(&event_errors,1);
        (void)dbt_a2dp_acceptor_retire(&profile,now_ms());
    }
}
void probe_audio_receive(const dbt_channel_t *channel,const uint8_t *data,uint16_t size){
    if(initialized)(void)dbt_a2dp_acceptor_receive(&profile,channel,data,size);
}
void probe_audio_poll(uint32_t now){
    if(!initialized)return;
    dbt_codec_receipt_t receipt;
    for(unsigned i=0;i<DBT_CODEC_WORKER_JOBS && dbt_codec_worker_receipt(&worker,&receipt)==DBT_OK;++i)
        (void)dbt_a2dp_acceptor_complete(&profile,receipt.binding,receipt.token,receipt.result,now);
    if(atomic_load_explicit(&worker.faults,memory_order_acquire))failed=true;
    if(failed)(void)dbt_a2dp_acceptor_stop(&profile,now);
    (void)dbt_a2dp_acceptor_poll(&profile,now);
    atomic_store(&ignored_dynamic,profile.runtime->channels.ignored_dynamic_pdus);
    atomic_store(&oversized_dynamic,profile.runtime->channels.oversized_dynamic_pdus);
    uint32_t staged=0;
    for(unsigned i=0;i<DBT_CHANNEL_LIMIT;++i){
        const dbt_channel_t *channel=&profile.runtime->channels.channels[i];
        if(channel->psm==0x19 && channel->state!=DBT_CH_FREE)++staged;
    }
    /* One owner publishes a compact diagnostic snapshot; logger never reads
     * mutable profile state. Detail bits: retiring, drained, worker, borrowed,
     * attached, pinned, pending, submitted, sink/signal failed, signal/media key;
     * remaining upper bits count staged PSM channels. */
    atomic_store(&profile_snapshot,(uint32_t)profile.sink.state|((uint32_t)profile.active<<8)|
        ((uint32_t)profile.enabled<<9)|((uint32_t)profile.draining<<10)|
        ((uint32_t)profile.sink.retiring<<11)|((uint32_t)profile.sink.drained<<12)|
        ((uint32_t)profile.sink.worker_live<<13)|((uint32_t)profile.bearer.active<<14)|
        ((uint32_t)profile.bearer.attached<<15)|((uint32_t)profile.sink.signal.pinned<<16)|
        ((uint32_t)profile.sink.pending.used<<17)|((uint32_t)profile.sink.pending.submitted<<18)|
        ((uint32_t)profile.sink.failed<<19)|((uint32_t)profile.sink.signal.failed<<20)|
        ((uint32_t)(profile.signaling.serial!=0)<<21)|((uint32_t)(profile.media.serial!=0)<<22)|(staged<<23));
#ifdef DBT_PROBE_AUDIO_OUTPUT
    probe_output_stats_t stats;probe_output_stats(&stats);
    if(stats.failed)failed=true;
    static uint32_t delay_binding,delay_at;
    if(profile.active && !profile.draining && profile.sink.delay_enabled &&
       profile.sink.state>=DBT_STREAM_CONFIGURED && profile.sink.state<=DBT_STREAM_STREAMING &&
       !profile.sink.pending.used && !profile.sink.signal.tx[0].used &&
       (delay_binding!=profile.sink.binding || now-delay_at>=1000)){
        dbt_avdtp_caps_t caps;dbt_avdtp_cap_error_t error;dbt_codec_config_t config;bool selected=false;
        if(dbt_avdtp_caps_validate(profile.sink.configuration,profile.sink.configuration_size,DBT_AVDTP_CONFIGURATION,&caps,&error)==DBT_OK){
            for(unsigned i=0;i<sizeof(codecs)/sizeof(codecs[0]);++i)
                if(codecs[i]->configure(caps.codec.data,caps.codec.size,&config)==DBT_OK){selected=true;break;}
        }
        if(selected && dbt_avdtp_sink_delay(&profile.sink,probe_output_delay_100us(config.rate),now)==DBT_OK){delay_binding=profile.sink.binding;delay_at=now;}
    }
#endif
}
uint32_t probe_audio_wait_ms(uint32_t now){return initialized?dbt_a2dp_acceptor_wait_ms(&profile,now):UINT32_MAX;}
