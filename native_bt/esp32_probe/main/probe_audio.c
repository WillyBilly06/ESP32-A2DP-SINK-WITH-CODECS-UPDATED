#include "probe_audio.h"
#include "dbt_sbc_worker.h"
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
static dbt_sbc_worker_t worker;
static TaskHandle_t owner,decoder_task,logger_task;
static bool initialized,failed;
static _Atomic uint32_t pcm_generation,pcm_frames,pcm_bytes,pcm_hash,pcm_rate,pcm_channels;
static _Atomic uint32_t profile_snapshot,event_errors,last_channel_event,last_channel_result;
static _Atomic uint32_t ignored_dynamic,oversized_dynamic;
static const uint8_t capabilities[]={1,0,7,6,0,0,0x3f,0xff,2,53,8,0};
static const dbt_avdtp_endpoint_t endpoint={capabilities,sizeof(capabilities),1,dbt_a2dp_sbc_validate};
static uint32_t now_ms(void){return (uint32_t)(esp_timer_get_time()/1000);}
static uint8_t *allocate(void *context,uint16_t bytes){(void)context;return heap_caps_malloc(bytes,MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);}
static void release(void *context,uint8_t *bytes){(void)context;heap_caps_free(bytes);}
static void wake_worker(void *context){(void)context;if(decoder_task)xTaskNotifyGive(decoder_task);}
static void wake_owner(void *context){(void)context;xTaskNotifyGive(owner);}
static dbt_result_t pcm(void *context,const int16_t *samples,uint16_t count,uint8_t channels,uint32_t rate,uint32_t timestamp){
    (void)context;(void)timestamp;uint32_t bytes=2u*count*channels;
#ifdef DBT_PROBE_AUDIO_OUTPUT
    dbt_result_t result=probe_output_pcm(samples,count,channels,rate);
    if(result!=DBT_OK)return result;
#endif
    atomic_fetch_add_explicit(&pcm_generation,1,memory_order_acq_rel);
    uint32_t hash=atomic_load_explicit(&pcm_hash,memory_order_relaxed);const uint8_t *p=(const uint8_t *)samples;
    for(unsigned i=0;i<bytes;++i)hash=(hash^p[i])*16777619u;
    atomic_fetch_add_explicit(&pcm_frames,1,memory_order_relaxed);
    atomic_fetch_add_explicit(&pcm_bytes,bytes,memory_order_relaxed);
    atomic_store_explicit(&pcm_hash,hash,memory_order_relaxed);
    atomic_store_explicit(&pcm_rate,rate,memory_order_relaxed);
    atomic_store_explicit(&pcm_channels,channels,memory_order_relaxed);
    atomic_fetch_add_explicit(&pcm_generation,1,memory_order_release);
    return DBT_OK; /* Diagnostic counter consumer: no I2S or acoustic claim. */
}
static dbt_result_t stream(void *context,bool started){
    (void)context;
#ifdef DBT_PROBE_AUDIO_OUTPUT
    dbt_result_t result=probe_output_stream(started,worker.decoder.config.rate,worker.decoder.config.channels);
    if(result!=DBT_OK)return result;
#endif
    if(started){
        atomic_fetch_add_explicit(&pcm_generation,1,memory_order_acq_rel);
        atomic_store_explicit(&pcm_frames,0,memory_order_relaxed);
        atomic_store_explicit(&pcm_bytes,0,memory_order_relaxed);
        atomic_store_explicit(&pcm_hash,2166136261u,memory_order_relaxed);
        atomic_fetch_add_explicit(&pcm_generation,1,memory_order_release);
    }
    return DBT_OK;
}
static void decode_loop(void *unused){
    (void)unused;
    for(;;){
        uint16_t done=dbt_sbc_worker_step(&worker,now_ms(),2);
        if(done==2){taskYIELD();continue;}
        ulTaskNotifyTake(pdTRUE,pdMS_TO_TICKS(20));
    }
}
static void log_loop(void *unused){
    (void)unused;
    uint32_t previous=UINT32_MAX,previous_errors=UINT32_MAX,previous_ignored=UINT32_MAX,previous_oversized=UINT32_MAX;
    for(;;){
        vTaskDelay(pdMS_TO_TICKS(1000));
        uint32_t generation=atomic_load_explicit(&pcm_generation,memory_order_acquire);
        if(generation&1u)continue;
        uint32_t frames=atomic_load_explicit(&pcm_frames,memory_order_relaxed),bytes=atomic_load_explicit(&pcm_bytes,memory_order_relaxed);
        uint32_t hash=atomic_load_explicit(&pcm_hash,memory_order_relaxed),rate=atomic_load_explicit(&pcm_rate,memory_order_relaxed);
        uint32_t channels=atomic_load_explicit(&pcm_channels,memory_order_relaxed);
        if(generation!=atomic_load_explicit(&pcm_generation,memory_order_acquire))continue;
        printf("{\"probe\":\"sbc_pcm\",\"frames\":%" PRIu32 ",\"bytes\":%" PRIu32 ",\"fnv1a32\":%" PRIu32
               ",\"rate\":%" PRIu32 ",\"channels\":%" PRIu32 ",\"media_packets\":%" PRIu32 ",\"drops\":%" PRIu32
               ",\"resets\":%" PRIu32 ",\"resident\":%" PRIu32 ",\"i2s_output\":" AUDIO_OUTPUT_JSON "}\n",frames,bytes,hash,rate,channels,
               atomic_load(&worker.packets),atomic_load(&worker.media_drops),atomic_load(&worker.resets),atomic_load(&worker.resident_bytes));
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
#endif
    }
}
static void sink_event(void *context,dbt_sink_event_t event,dbt_stream_state_t state){(void)context;(void)event;(void)state;}
dbt_result_t probe_audio_init(dbt_runtime_t *runtime,TaskHandle_t host_owner){
    owner=host_owner;
    dbt_sbc_worker_hooks_t hooks={0};hooks.decoder=(dbt_sbc_decoder_hooks_t){0,allocate,release,pcm};
    hooks.wake_worker=wake_worker;hooks.wake_owner=wake_owner;hooks.stream=stream;
    dbt_result_t result=dbt_sbc_worker_init(&worker,hooks,672,8);
    if(result!=DBT_OK)return result;
    dbt_avdtp_sink_hooks_t client={&worker,allocate,release,dbt_sbc_worker_submit,sink_event,dbt_sbc_worker_media};
    result=dbt_a2dp_acceptor_init(&profile,runtime,client,&endpoint,1,672,64,5000,2000,5000);
    if(result!=DBT_OK)return result;
    if(xTaskCreatePinnedToCore(decode_loop,"native_sbc",4096,0,5,&decoder_task,1)!=pdPASS)return DBT_NO_SPACE;
#ifdef DBT_PROBE_AUDIO_OUTPUT
    result=probe_output_init(wake_worker,0);
    if(result!=DBT_OK){vTaskDelete(decoder_task);decoder_task=0;return result;}
#endif
    if(xTaskCreatePinnedToCore(log_loop,"native_pcm_log",2048,0,1,&logger_task,0)!=pdPASS){vTaskDelete(decoder_task);decoder_task=0;return DBT_NO_SPACE;}
    initialized=true;
    printf("{\"probe\":\"sbc_probe_ready\",\"psm\":25,\"mtu\":672,\"worker_bytes\":%u,\"profile_bytes\":%u,\"decoder_stack\":4096,"
           "\"logger_stack\":2048,\"diagnostic_only\":true,\"audio_sink_sdp_advertised\":" AUDIO_OUTPUT_JSON ",\"i2s_output\":" AUDIO_OUTPUT_JSON "}\n",
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
    dbt_sbc_receipt_t receipt;
    for(unsigned i=0;i<DBT_SBC_WORKER_JOBS && dbt_sbc_worker_receipt(&worker,&receipt)==DBT_OK;++i)
        (void)dbt_a2dp_acceptor_complete(&profile,receipt.binding,receipt.token,receipt.result,now);
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
        dbt_avdtp_caps_t caps;dbt_avdtp_cap_error_t error;dbt_sbc_config_t config;
        if(dbt_avdtp_caps_validate(profile.sink.configuration,profile.sink.configuration_size,DBT_AVDTP_CONFIGURATION,&caps,&error)==DBT_OK &&
           dbt_sbc_configure(caps.codec.data,caps.codec.size,&config)==DBT_OK &&
           dbt_avdtp_sink_delay(&profile.sink,probe_output_delay_100us(config.rate),now)==DBT_OK){delay_binding=profile.sink.binding;delay_at=now;}
    }
#endif
}
uint32_t probe_audio_wait_ms(uint32_t now){return initialized?dbt_a2dp_acceptor_wait_ms(&profile,now):UINT32_MAX;}
