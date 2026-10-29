#include "probe_audio.h"
#include "dbt_sbc_worker.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include <inttypes.h>
#include <stdio.h>

static dbt_a2dp_acceptor_t profile;
static dbt_sbc_worker_t worker;
static TaskHandle_t owner,decoder_task,logger_task;
static bool initialized,failed;
static _Atomic uint32_t pcm_generation,pcm_frames,pcm_bytes,pcm_hash,pcm_rate,pcm_channels;
static const uint8_t capabilities[]={1,0,7,6,0,0,0x3f,0xff,2,53,8,0};
static const dbt_avdtp_endpoint_t endpoint={capabilities,sizeof(capabilities),1,dbt_a2dp_sbc_validate};
static uint32_t now_ms(void){return (uint32_t)(esp_timer_get_time()/1000);}
static uint8_t *allocate(void *context,uint16_t bytes){(void)context;return heap_caps_malloc(bytes,MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);}
static void release(void *context,uint8_t *bytes){(void)context;heap_caps_free(bytes);}
static void wake_worker(void *context){(void)context;if(decoder_task)xTaskNotifyGive(decoder_task);}
static void wake_owner(void *context){(void)context;xTaskNotifyGive(owner);}
static dbt_result_t pcm(void *context,const int16_t *samples,uint16_t count,uint8_t channels,uint32_t rate,uint32_t timestamp){
    (void)context;(void)timestamp;uint32_t bytes=2u*count*channels;
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
               ",\"resets\":%" PRIu32 ",\"resident\":%" PRIu32 ",\"i2s_output\":false}\n",frames,bytes,hash,rate,channels,
               atomic_load(&worker.packets),atomic_load(&worker.media_drops),atomic_load(&worker.resets),atomic_load(&worker.resident_bytes));
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
    if(xTaskCreatePinnedToCore(log_loop,"native_pcm_log",2048,0,1,&logger_task,0)!=pdPASS){vTaskDelete(decoder_task);decoder_task=0;return DBT_NO_SPACE;}
    initialized=true;
    printf("{\"probe\":\"sbc_probe_ready\",\"psm\":25,\"mtu\":672,\"worker_bytes\":%u,\"profile_bytes\":%u,\"decoder_stack\":4096,"
           "\"logger_stack\":2048,\"diagnostic_only\":true,\"audio_sink_sdp_advertised\":false,\"i2s_output\":false}\n",
           (unsigned)sizeof(worker),(unsigned)sizeof(profile));
    return DBT_OK;
}
bool probe_audio_accept(void *context,const dbt_link_t *link,uint16_t psm){(void)context;return psm!=0x19 || (initialized && dbt_a2dp_acceptor_accept(&profile,link));}
void probe_audio_event(dbt_channel_event_t event,const dbt_channel_t *channel){
    if(!initialized || channel->psm!=0x19)return;
    dbt_result_t result=dbt_a2dp_acceptor_event(&profile,event,channel);
    if(result!=DBT_OK && result!=DBT_EMPTY)failed=true;
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
}
uint32_t probe_audio_wait_ms(uint32_t now){return initialized?dbt_a2dp_acceptor_wait_ms(&profile,now):UINT32_MAX;}
