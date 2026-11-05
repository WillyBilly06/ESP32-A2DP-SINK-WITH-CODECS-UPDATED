#include "probe_schedule_limits.h"
#include "probe_audio.h"
#include "probe_audio_limits.h"
#ifdef DBT_PROBE_SCHEDULER_TRACE
#include "probe_scheduler.h"
#endif
#include "probe_security.h"
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
#define AUDIO_SDP_JSON "true"
#else
#define AUDIO_SDP_JSON "false"
#endif
#if defined(DBT_PROBE_AUDIO_OUTPUT) && !defined(DBT_PROBE_RF_PROTOCOL_DIAGNOSTIC)
#define AUDIO_OUTPUT_JSON "true"
#else
#define AUDIO_OUTPUT_JSON "false"
#endif
#if defined(DBT_PROBE_PCM_HASH) || !defined(DBT_PROBE_STAY_NATIVE)
#define AUDIO_PCM_HASH_ENABLED 1
#define AUDIO_PCM_HASH_JSON "true"
#else
#define AUDIO_PCM_HASH_ENABLED 0
#define AUDIO_PCM_HASH_JSON "false"
#endif
#ifdef DBT_PROBE_RF_PROTOCOL_DIAGNOSTIC
#define AUDIO_RF_DIAG_JSON "true"
#else
#define AUDIO_RF_DIAG_JSON "false"
#endif
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_attr.h"
#ifdef ESP_PLATFORM
#include "esp_task_wdt.h"
#endif
#include <inttypes.h>
#include <stdio.h>
#include <stdarg.h>
#ifndef DBT_PROBE_DECODER_CORE
#define DBT_PROBE_DECODER_CORE DBT_ESP32_DECODER_CORE
#endif
#if DBT_PROBE_DECODER_CORE != 0 && DBT_PROBE_DECODER_CORE != 1
#error "Decoder core must be exactly 0 or 1"
#endif
#ifndef DBT_PROBE_LOGGER_CORE
#define DBT_PROBE_LOGGER_CORE (1-DBT_PROBE_DECODER_CORE)
#endif
#if DBT_PROBE_LOGGER_CORE != 0 && DBT_PROBE_LOGGER_CORE != 1
#error "Logger core must be 0 or 1"
#endif
#define AUDIO_LOGGER_CORE DBT_PROBE_LOGGER_CORE
#if defined(DBT_PROBE_STAY_NATIVE) && (DBT_PROBE_DECODER_CORE != DBT_ESP32_DECODER_CORE || DBT_PROBE_LOGGER_CORE != DBT_ESP32_BACKGROUND_CORE)
#error Stay-native product requires decoder CPU0 and logger CPU1
#endif
/* Priority policy is centralized in probe_schedule_limits.h. */

/* Test-only isolation, never an alternative product audio path. Decoder/RTP,
 * endpoints and product security are unchanged. Product IO/sounds remain;
 * this mode omits only A2DP PCM forwarding and its output-failure coupling. */
#if defined(DBT_PROBE_RF_PROTOCOL_DIAGNOSTIC) && (!defined(DBT_PROBE_PRODUCT_IO) || !defined(DBT_PROBE_PRODUCT_SECURITY) || !defined(DBT_PROBE_AUDIO_OUTPUT) || defined(DBT_PROBE_ASYNC_AUDIO) || defined(DBT_PROBE_KEEP_OWNED_ACTIVE))
#error "RF protocol diagnostic requires secured product IO and finite synchronous checksum mode"
#endif

static dbt_a2dp_acceptor_t profile;
static dbt_codec_worker_t worker;
static TaskHandle_t owner,decoder_task,logger_task;
static _Atomic uint32_t decoder_run_core;
static _Atomic uint32_t decoder_wait_calls[4],decoder_wait_elapsed[4],decoder_wait_maximum[4];
static _Atomic uint32_t decoder_wait_kind,decoder_wait_began;
static bool initialized,failed,restart_hold,restart_drained;
/* Owner-only cursor; cumulative worker faults are diagnostics, not a permanent
 * service-disable flag. Session retirement still fences all borrowed PCM. */
static uint32_t worker_faults_seen;
#ifdef DBT_PROBE_AUDIO_OUTPUT
static bool output_initialized;
/* The existing one-second report cadence is an owner deadline, not a poll. */
static uint32_t delay_binding,delay_at;
static uint32_t delay_report_wait_ms(uint32_t now){
    const dbt_avdtp_sink_t *sink=&profile.sink;
    if(failed || restart_hold || !profile.enabled || !profile.active || profile.draining ||
       sink->failed || sink->retiring || sink->signal.failed || !sink->delay_enabled ||
       sink->state<DBT_STREAM_CONFIGURED || sink->state>DBT_STREAM_STREAMING ||
       sink->pending.used || sink->signal.tx[0].used)return UINT32_MAX;
    if(delay_binding!=sink->binding)return 0;
    uint32_t elapsed=now-delay_at;
    return elapsed>=1000u?0:1000u-elapsed;
}
#endif
static probe_audio_product_event_fn product_observer;
static void *product_context;
static _Atomic uint32_t pcm_generation,pcm_frames,pcm_bytes,pcm_hash,pcm_rate,pcm_channels,pcm_codec,pcm_sample_bytes;
static _Atomic uint32_t codec_revision,codec_cie_size,codec_cie_words[(DBT_CODEC_CIE_BYTES+3u)/4u];
static _Atomic uint32_t profile_snapshot,event_errors,last_channel_event,last_channel_result;
static _Atomic uint32_t ignored_dynamic,oversized_dynamic;
static _Atomic uint32_t l2cap_delivered,l2cap_abandoned,l2cap_rejected,l2cap_orphaned;
static _Atomic uint32_t l2cap_malformed_signaling,l2cap_unknown_responses,l2cap_retained_bytes;
static _Atomic uint32_t media_sdu_max,last_oversized_size,last_oversized_mtu,last_oversized_psm;
/* Separate single-writer rings for host admission and decoder events. Logging
 * never waits on the producer; record loss is itself exposed every second. */
#define TRACE_SLOTS 16u
typedef struct {dbt_codec_trace_t event;uint32_t at_us,ordinal,codec;} media_trace_t;
typedef struct {
    media_trace_t items[TRACE_SLOTS];
    _Atomic uint32_t written,read,lost;
    uint32_t ordinal;
} media_trace_ring_t;
static media_trace_ring_t media_trace_rings[2];
static _Atomic uint32_t rx_packets,rx_acl,rx_events,rx_bytes,rx_isr,rx_rejected,rx_pending_us;
static _Atomic uint32_t rx_callback_max_us,first_rx_to_poll_max_us,host_poll_max_us;
static _Atomic uint32_t worker_steps,worker_elapsed_us,worker_step_max_us,worker_hist[8];
static _Atomic uint32_t owner_iterations,owner_immediate[15];
static _Atomic uint32_t owner_ready_yields;
static _Atomic uint32_t snapshot_format_lost;
static DRAM_ATTR _Atomic uint32_t watchdog_events,watchdog_hook_us,watchdog_isr_core;
static void IRAM_ATTR record_watchdog(unsigned core){
    atomic_store(&watchdog_hook_us,(uint32_t)esp_timer_get_time());
    atomic_store(&watchdog_isr_core,core);
    atomic_store(&watchdog_events,atomic_load(&watchdog_events)+1u);
}
#ifdef ESP_PLATFORM
/* Public SDK hook: called after its task-list diagnostics and before backtrace
 * handling. Only record metadata here; watchdog policy and output stay intact. */
void IRAM_ATTR esp_task_wdt_isr_user_handler(void){record_watchdog((unsigned)xPortGetCoreID());}
#endif
typedef struct {
    uint32_t began;
    _Atomic uint32_t calls,elapsed_us,maximum_us,hist[8];
} phase_timing_t;
static phase_timing_t phase_timing[3];
static const char *const trace_names[]={"invalid","input_sequence","input_ssrc","queue_full",
    "worker_sequence","worker_ssrc","decode_reject","reset_failed","pcm_failed","pcm_timeout","pcm_busy"};
static const char *codec_name(uint32_t codec){
    static const char *const names[]={"none","SBC","Android Opus","Opus05","AAC-LC","aptX","aptX HD","aptX LL","LDAC","LHDC V5"};
    return codec<sizeof(names)/sizeof(names[0])?names[codec]:"unknown";
}
static void maximum(_Atomic uint32_t *value,uint32_t candidate){
    /* Each maximum has one writer; the logger only reads it. */
    if(candidate>atomic_load_explicit(value,memory_order_relaxed))atomic_store_explicit(value,candidate,memory_order_relaxed);
}
/* Logger-only. Arguments have been captured before entry; the time range
 * includes preemption during their collection, never the UART transmission.
 * Ordinary task loggers emit one complete JSON write. SDK early/ISR output
 * can still interleave independently on the UART. */
/* IEEE CRC32 of the exact bytes preceding the checksum member. Logger-only,
 * bounded by its 1024-byte buffer; detects capture corruption, not authenticity. */
static uint32_t snapshot_crc32(const char *bytes,size_t length){
    uint32_t crc=UINT32_MAX;
    for(size_t i=0;i<length;++i){
        crc^=(uint8_t)bytes[i];
        for(unsigned bit=0;bit<8;++bit)crc=(crc>>1)^(0xedb88320u&(0u-(crc&1u)));
    }
    return crc^UINT32_MAX;
}
static bool snapshot_vformat(char *line,size_t capacity,uint32_t began,const char *format,va_list arguments){
    uint32_t ended=(uint32_t)esp_timer_get_time();
    if(capacity<160u)goto lost;
    int n=vsnprintf(line,capacity-128u,format,arguments);
    if(n<2 || (size_t)n>=capacity-128u || line[n-2]!='}' || line[n-1]!='\n')goto lost;
    int tail=snprintf(line+n-2,capacity-(size_t)n+2u,
        ",\"snapshot_begin_us\":%" PRIu32 ",\"at_us\":%" PRIu32 ",\"snapshot_span_us\":%" PRIu32,
        began,ended,ended-began);
    if(tail<0 || (size_t)tail>=capacity-(size_t)n+2u)goto lost;
    size_t prefix=(size_t)n-2u+(size_t)tail;
    uint32_t crc=snapshot_crc32(line,prefix);
    tail=snprintf(line+prefix,capacity-prefix,",\"record_crc32\":\"%08" PRIx32 "\"}\n",crc);
    if(tail<0 || (size_t)tail>=capacity-prefix)goto lost;
    return true;
lost:
    atomic_fetch_add_explicit(&snapshot_format_lost,1,memory_order_relaxed);return false;
}
static void __attribute__((format(printf,2,3))) snapshot_log(uint32_t began,const char *format,...){
    char line[1024];va_list arguments;va_start(arguments,format);
    bool ready=snapshot_vformat(line,sizeof(line),began,format,arguments);va_end(arguments);
    if(ready)printf("%s",line);
}
#define PROBE_SNAPSHOT_LOG(...) do {uint32_t snapshot_began=(uint32_t)esp_timer_get_time();snapshot_log(snapshot_began,__VA_ARGS__);}while(0)
static void timing_event(unsigned kind){
    unsigned index=(kind-DBT_CODEC_DECODE_BEGIN)/2u;
    if(index>=3)return;
    phase_timing_t *phase=&phase_timing[index];uint32_t at=(uint32_t)esp_timer_get_time();
    if((kind-DBT_CODEC_DECODE_BEGIN)%2u==0){phase->began=at;return;}
    uint32_t elapsed=at-phase->began;
    atomic_fetch_add(&phase->calls,1);atomic_fetch_add(&phase->elapsed_us,elapsed);maximum(&phase->maximum_us,elapsed);
    unsigned bucket=0;uint32_t bound=125;
    while(bucket<7 && elapsed>bound){++bucket;bound*=2;}
    atomic_fetch_add(&phase->hist[bucket],1);
}
static void media_trace(void *context,const dbt_codec_trace_t *event){
    (void)context;
    if(event->kind>=DBT_CODEC_DECODE_BEGIN){
        timing_event((unsigned)event->kind);return;
    }
    media_trace_ring_t *ring=&media_trace_rings[event->kind<=DBT_CODEC_QUEUE_FULL?0:1];
    uint32_t ordinal=++ring->ordinal;
    uint32_t written=atomic_load_explicit(&ring->written,memory_order_relaxed);
    if(written-atomic_load_explicit(&ring->read,memory_order_acquire)>=TRACE_SLOTS){atomic_fetch_add(&ring->lost,1);return;}
    ring->items[written%TRACE_SLOTS]=(media_trace_t){*event,(uint32_t)esp_timer_get_time(),ordinal,atomic_load_explicit(&pcm_codec,memory_order_relaxed)};
    atomic_store_explicit(&ring->written,written+1u,memory_order_release);
}
static void log_media_trace(void){
    for(unsigned i=0;i<2;++i)for(unsigned budget=0;budget<2;++budget){
        media_trace_ring_t *ring=&media_trace_rings[i];uint32_t read=atomic_load(&ring->read);
        if(read==atomic_load_explicit(&ring->written,memory_order_acquire))break;
        media_trace_t record=ring->items[read%TRACE_SLOTS];
        atomic_store_explicit(&ring->read,read+1u,memory_order_release);
        const dbt_codec_trace_t *e=&record.event;
        printf("{\"probe\":\"media_trace\",\"at_us\":%" PRIu32 ",\"stage\":\"%s\",\"ordinal\":%" PRIu32
               ",\"sequence\":%u,\"expected\":%u,\"rtp_timestamp\":%" PRIu32 ",\"ssrc\":%" PRIu32
               ",\"previous_ssrc\":%" PRIu32 ",\"bytes\":%u,\"queued\":%u,\"result\":%u,\"codec\":%" PRIu32 ",\"codec_name\":\"%s\",\"rtp_marker\":%s,\"payload_prefix\":[%u,%u]}\n",
               record.at_us,(unsigned)e->kind<sizeof(trace_names)/sizeof(trace_names[0])?trace_names[e->kind]:"invalid",record.ordinal,e->sequence,e->expected_sequence,e->timestamp,
               e->ssrc,e->previous_ssrc,e->bytes,e->queued,(unsigned)e->result,record.codec,codec_name(record.codec),e->marker?"true":"false",e->prefix[0],e->prefix[1]);
    }
}
void probe_audio_transport_received(uint32_t began_us,uint16_t bytes,uint8_t type,dbt_result_t result,bool isr){
    atomic_fetch_add(&rx_packets,1);atomic_fetch_add(&rx_bytes,bytes);
    if(type==DBT_H4_ACL)atomic_fetch_add(&rx_acl,1);
    if(type==DBT_H4_EVENT)atomic_fetch_add(&rx_events,1);
    if(isr)atomic_fetch_add(&rx_isr,1);
    if(result!=DBT_OK)atomic_fetch_add(&rx_rejected,1);
    uint32_t empty=0;uint32_t stamp=began_us?began_us:1u;
    atomic_compare_exchange_strong_explicit(&rx_pending_us,&empty,stamp,memory_order_release,memory_order_relaxed);
    maximum(&rx_callback_max_us,(uint32_t)esp_timer_get_time()-began_us);
}
void probe_audio_transport_poll_begin(uint32_t began_us){
    (void)began_us;
    uint32_t pending=atomic_exchange_explicit(&rx_pending_us,0,memory_order_acq_rel);
    /* A controller callback may preempt the owner after its caller sampled
     * began_us. Sample after taking the pending stamp to avoid a negative
     * duration masquerading as a near-UINT32_MAX scheduling delay. */
    if(pending)maximum(&first_rx_to_poll_max_us,(uint32_t)esp_timer_get_time()-pending);
}
void probe_audio_transport_poll_end(uint32_t began_us){maximum(&host_poll_max_us,(uint32_t)esp_timer_get_time()-began_us);}
void probe_audio_owner_wait(uint32_t immediate_mask){
    atomic_fetch_add_explicit(&owner_iterations,1,memory_order_relaxed);
    for(unsigned i=0;i<15;++i)if(immediate_mask&(1u<<i))atomic_fetch_add_explicit(&owner_immediate[i],1,memory_order_relaxed);
}
void probe_audio_owner_ready_yield(void){atomic_fetch_add_explicit(&owner_ready_yields,1,memory_order_relaxed);}
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
bool probe_audio_sound_memory_ok(void){return heap_caps_get_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT)>=24u*1024u;}
dbt_result_t probe_audio_output_bootstrap(void){
#ifdef DBT_PROBE_AUDIO_OUTPUT
    if(output_initialized)return DBT_OK;
    dbt_result_t r=probe_output_init(wake_worker,0);if(r==DBT_OK)output_initialized=true;return r;
#else
    return DBT_UNSUPPORTED;
#endif
}
static dbt_result_t pcm(void *context,const void *samples,uint16_t count,const dbt_codec_config_t *config,uint32_t timestamp){
#ifdef DBT_PROBE_PCM_COPIED
    _Static_assert(PROBE_PCM_TRANSFER_FRAMES>=DBT_CODEC_PCM_CHUNK_FRAMES,"Compact ring must accept one codec PCM quantum");
#endif
    (void)context;(void)timestamp;
    if(!config || !samples || !count || config->sample_bytes<2 || config->sample_bytes>4 ||
       config->channels<1 || config->channels>2 || !config->rate)return DBT_INVALID;
    uint8_t channels=config->channels;uint32_t rate=config->rate,bytes=(uint32_t)config->sample_bytes*count*channels;
#ifndef DBT_PROBE_ASYNC_AUDIO
    timing_event(DBT_CODEC_PCM_BEGIN);
#endif
#if defined(DBT_PROBE_AUDIO_OUTPUT) && !defined(DBT_PROBE_RF_PROTOCOL_DIAGNOSTIC) && !defined(DBT_PROBE_PCM_LOANS)
#ifdef DBT_PROBE_ASYNC_AUDIO
    dbt_result_t result=probe_output_try_pcm_format(samples,count,channels,rate,(dbt_pcm_format_t)config->sample_bytes);
#else
    dbt_result_t result=probe_output_pcm_format(samples,count,channels,rate,(dbt_pcm_format_t)config->sample_bytes);
#endif
    if(result!=DBT_OK){
#ifndef DBT_PROBE_ASYNC_AUDIO
        timing_event(DBT_CODEC_PCM_END);
#endif
        return result;
    }
#endif
    atomic_fetch_add_explicit(&pcm_generation,1,memory_order_acq_rel);
#if AUDIO_PCM_HASH_ENABLED
    uint32_t hash=atomic_load_explicit(&pcm_hash,memory_order_relaxed);const uint8_t *p=samples;
    for(unsigned i=0;i<bytes;++i)hash=(hash^p[i])*16777619u;
    atomic_store_explicit(&pcm_hash,hash,memory_order_relaxed);
#endif
    atomic_fetch_add_explicit(&pcm_frames,1,memory_order_relaxed);
    atomic_fetch_add_explicit(&pcm_bytes,bytes,memory_order_relaxed);
    atomic_store_explicit(&pcm_rate,rate,memory_order_relaxed);
    atomic_store_explicit(&pcm_channels,channels,memory_order_relaxed);
    atomic_store_explicit(&pcm_sample_bytes,config->sample_bytes,memory_order_relaxed);
    atomic_fetch_add_explicit(&pcm_generation,1,memory_order_release);
#ifndef DBT_PROBE_ASYNC_AUDIO
    timing_event(DBT_CODEC_PCM_END);
#endif
    return DBT_OK;
}
#ifdef DBT_PROBE_PCM_LOANS
_Static_assert(PROBE_PCM_LOAN_FRAMES==DBT_CODEC_PCM_CHUNK_FRAMES,"PCM loan chunk contracts must match");
static dbt_result_t pcm_begin(void *context,const void *samples,uint16_t frames,const dbt_codec_config_t *config,uint32_t timestamp,uint32_t *ticket){
    if(!config)return DBT_INVALID;
    dbt_result_t result=probe_output_pcm_begin(samples,frames,config->channels,config->rate,(dbt_pcm_format_t)config->sample_bytes,ticket);
    if(result==DBT_PENDING)(void)pcm(context,samples,frames,config,timestamp);
    return result;
}
static dbt_result_t pcm_poll(void *context,uint32_t ticket){(void)context;return probe_output_pcm_poll(ticket);}
static dbt_result_t pcm_cancel(void *context,uint32_t ticket){(void)context;return probe_output_pcm_cancel(ticket);}
#endif
static dbt_result_t publish_stream(bool started,const dbt_codec_config_t *config){
    if(!config)return DBT_INVALID;
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
        atomic_store_explicit(&pcm_hash,AUDIO_PCM_HASH_ENABLED?2166136261u:0u,memory_order_relaxed);
        atomic_store_explicit(&pcm_codec,codec,memory_order_relaxed);
        atomic_store_explicit(&pcm_rate,config->rate,memory_order_relaxed);
        atomic_store_explicit(&pcm_channels,config->channels,memory_order_relaxed);
        atomic_store_explicit(&pcm_sample_bytes,config->sample_bytes,memory_order_relaxed);
        for(unsigned word=0;word<(DBT_CODEC_CIE_BYTES+3u)/4u;++word){
            uint32_t value=0;
            for(unsigned byte=0;byte<4;++byte){unsigned at=4u*word+byte;if(at<config->cie_size && at<DBT_CODEC_CIE_BYTES)value|=(uint32_t)config->cie[at]<<(8u*byte);}
            atomic_store_explicit(&codec_cie_words[word],value,memory_order_relaxed);
        }
        atomic_store_explicit(&codec_cie_size,config->cie_size,memory_order_relaxed);
        atomic_fetch_add_explicit(&codec_revision,1,memory_order_release);
        atomic_fetch_add_explicit(&pcm_generation,1,memory_order_release);
    }
    return DBT_OK;
}
#ifdef DBT_PROBE_ASYNC_AUDIO
static bool async_stream_start;
static dbt_result_t stream_begin(void *context,bool start,const dbt_codec_config_t *config,uint32_t *ticket){
    (void)context;
    if(!config)return DBT_INVALID;
    dbt_result_t result=probe_output_stream_begin(start,config->rate,config->channels,(dbt_pcm_format_t)config->sample_bytes,ticket);
    if(result==DBT_PENDING)async_stream_start=start;
    return result;
}
static dbt_result_t stream_poll(void *context,uint32_t ticket){
    (void)context;dbt_result_t result=probe_output_request_poll(ticket);
    if(result==DBT_OK)return publish_stream(async_stream_start,&worker.decoder.config);
    return result;
}
static dbt_result_t stream_cancel(void *context,uint32_t ticket){(void)context;return probe_output_request_cancel(ticket);}
#else
static dbt_result_t stream(void *context,bool started,const dbt_codec_config_t *config){
    (void)context;
    if(!config)return DBT_INVALID;
#if defined(DBT_PROBE_AUDIO_OUTPUT) && !defined(DBT_PROBE_RF_PROTOCOL_DIAGNOSTIC)
    dbt_result_t result=probe_output_stream_format(started,config->rate,config->channels,(dbt_pcm_format_t)config->sample_bytes);
    if(result!=DBT_OK)return result;
#endif
    return publish_stream(started,config);
}
#endif
static TickType_t decoder_wait_ticks(uint32_t now){
    uint32_t when;
    if(!dbt_codec_worker_deadline(&worker,&when))return portMAX_DELAY;
    int32_t remaining=(int32_t)(when-now);
    if(remaining<=0)return 0; /* service an expired deadline on the next step */
    TickType_t ticks=pdMS_TO_TICKS((uint32_t)remaining);
    return ticks?ticks:1;
}
static void decode_loop(void *unused){
    (void)unused;
    uint32_t seen_faults=0;
    atomic_store(&decoder_run_core,(uint32_t)xPortGetCoreID());
    atomic_store(&decoder_wait_kind,UINT32_MAX);
    for(;;){
        uint32_t began=(uint32_t)esp_timer_get_time();
        (void)dbt_codec_worker_step(&worker,now_ms(),2);
        uint32_t elapsed=(uint32_t)esp_timer_get_time()-began;
        atomic_fetch_add(&worker_steps,1);atomic_fetch_add(&worker_elapsed_us,elapsed);maximum(&worker_step_max_us,elapsed);
        unsigned bucket=0;uint32_t bound=125;
        while(bucket<7 && elapsed>bound){++bucket;bound*=2;}
        atomic_fetch_add(&worker_hist[bucket],1);
        uint32_t faults=atomic_load_explicit(&worker.faults,memory_order_acquire);
        if(faults!=seen_faults){seen_faults=faults;wake_owner(0);}
        if(dbt_codec_worker_runnable(&worker)){taskYIELD();continue;}
        TickType_t wait=decoder_wait_ticks(now_ms());
        /* Notifications queued before this call remain pending. No fixed
         * media polling period and no wait while useful work is runnable. */
        if(!wait){(void)ulTaskNotifyTake(pdTRUE,0);continue;}
        unsigned reason=worker.pcm_ticket?1u:worker.pending_pcm&&worker.output_blocked?0u:
            worker.control_phase||atomic_load(&worker.job_read)!=atomic_load(&worker.job_written)?2u:3u;
        uint32_t before=(uint32_t)esp_timer_get_time();
        atomic_store_explicit(&decoder_wait_began,before,memory_order_relaxed);
        atomic_store_explicit(&decoder_wait_kind,reason,memory_order_release);
        (void)ulTaskNotifyTake(pdTRUE,wait);
        uint32_t waited=(uint32_t)esp_timer_get_time()-before;
        atomic_store_explicit(&decoder_wait_kind,UINT32_MAX,memory_order_release);
        atomic_fetch_add(&decoder_wait_calls[reason],1);
        atomic_fetch_add(&decoder_wait_elapsed[reason],waited);
        maximum(&decoder_wait_maximum[reason],waited);
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
    unsigned ticks=0;
    uint32_t previous_codec_revision=0;
    for(;;){
        vTaskDelay(pdMS_TO_TICKS(100));log_media_trace();
        if(++ticks<10)continue;
        ticks=0;
        uint32_t at=(uint32_t)esp_timer_get_time();
#ifdef DBT_PROBE_SCHEDULER_TRACE
        for(unsigned core=0;core<2;++core){
            probe_scheduler_snapshot_t sample;uint32_t began=(uint32_t)esp_timer_get_time();
            probe_scheduler_snapshot(core,&sample);
            snapshot_log(began,"{\"probe\":\"scheduler_samples\",\"core\":%u,\"sample_hz\":%u,\"coherent\":%s,\"known_task_mask\":%" PRIu32
                ",\"host\":%" PRIu32 ",\"decoder\":%" PRIu32 ",\"renderer\":%" PRIu32 ",\"logger\":%" PRIu32 ",\"controller\":%" PRIu32
                ",\"idle\":%" PRIu32 ",\"other\":%" PRIu32 ",\"last_tick_us\":%" PRIu32 ",\"max_tick_gap_us\":%" PRIu32
                ",\"max_gap_begin_us\":%" PRIu32 ",\"max_gap_end_us\":%" PRIu32 ",\"max_gap_previous_role\":%" PRIu32 ",\"max_gap_current_role\":%" PRIu32
                ",\"max_callback_body_cycles\":%" PRIu32 "}\n",core,(unsigned)configTICK_RATE_HZ,sample.coherent?"true":"false",sample.known_task_mask,
                sample.counts[0],sample.counts[1],sample.counts[2],sample.counts[3],sample.counts[4],sample.counts[5],sample.counts[6],
                sample.at_us,sample.max_gap_us,sample.max_gap_begin_us,sample.max_gap_end_us,sample.max_gap_previous_role,sample.max_gap_current_role,sample.max_callback_cycles);
            snapshot_log(began,"{\"probe\":\"host_phase_samples\",\"core\":%u,\"coherent\":%s,\"last_tick_us\":%" PRIu32
                ",\"other\":%" PRIu32 ",\"controller_query\":%" PRIu32 ",\"controller_submit\":%" PRIu32 ",\"runtime_poll\":%" PRIu32
                ",\"wait_calculation\":%" PRIu32 ",\"gatt_poll\":%" PRIu32 ",\"security_poll\":%" PRIu32 ",\"product_poll\":%" PRIu32
                ",\"io_poll\":%" PRIu32 ",\"audio_poll\":%" PRIu32 ",\"avrcp_poll\":%" PRIu32 ",\"console_log\":%" PRIu32 "}\n",
                core,sample.coherent?"true":"false",sample.at_us,
                sample.host_phases[0],sample.host_phases[1],sample.host_phases[2],sample.host_phases[3],sample.host_phases[4],sample.host_phases[5],
                sample.host_phases[6],sample.host_phases[7],sample.host_phases[8],sample.host_phases[9],sample.host_phases[10],sample.host_phases[11]);
        }
#endif
        PROBE_SNAPSHOT_LOG("{\"probe\":\"l2cap_media_sizes\",\"collection_cycle_us\":%" PRIu32
               ",\"advertised_media_mtu\":%u,\"max_delivered_media_sdu_bytes\":%" PRIu32
               ",\"oversize_count_all_dynamic_channels\":%" PRIu32 ",\"last_oversize_bytes\":%" PRIu32
               ",\"last_oversize_channel_mtu\":%" PRIu32 ",\"last_oversize_psm\":%" PRIu32 "}\n",
               at,(unsigned)PROBE_A2DP_RX_MTU,atomic_load(&media_sdu_max),atomic_load(&oversized_dynamic),
               atomic_load(&last_oversized_size),atomic_load(&last_oversized_mtu),atomic_load(&last_oversized_psm));
        PROBE_SNAPSHOT_LOG("{\"probe\":\"decoder_input_queue\",\"collection_cycle_us\":%" PRIu32
               ",\"packet_capacity_bytes\":%u,\"packet_slots\":%u,\"slot_stride_bytes\":%u"
               ",\"reserved_packet_bytes\":%u,\"max_seen_payload_bytes\":%" PRIu32
               ",\"oversize_rejects\":%" PRIu32 ",\"last_oversize_bytes\":%" PRIu32 "}\n",
               at,(unsigned)worker.maximum_payload,(unsigned)worker.depth,(unsigned)worker.stride,
               (unsigned)worker.stride*worker.depth,atomic_load(&worker.input_max_payload),
               atomic_load(&worker.input_oversize_rejects),atomic_load(&worker.input_last_oversize));
        PROBE_SNAPSHOT_LOG("{\"probe\":\"owner_wait_reasons\",\"collection_cycle_us\":%" PRIu32 ",\"iterations\":%" PRIu32
               ",\"runtime\":%" PRIu32 ",\"admission\":%" PRIu32 ",\"smp\":%" PRIu32 ",\"avrcp\":%" PRIu32
               ",\"restart\":%" PRIu32 ",\"pair_button\":%" PRIu32 ",\"rfcomm\":%" PRIu32 ",\"gap\":%" PRIu32
               ",\"product\":%" PRIu32 ",\"io\":%" PRIu32 ",\"att\":%" PRIu32 ",\"security\":%" PRIu32
               ",\"echo\":%" PRIu32 ",\"sdp\":%" PRIu32 ",\"a2dp\":%" PRIu32 ",\"runnable_yields\":%" PRIu32 "}\n",at,atomic_load(&owner_iterations),
               atomic_load(&owner_immediate[0]),atomic_load(&owner_immediate[1]),atomic_load(&owner_immediate[2]),
               atomic_load(&owner_immediate[3]),atomic_load(&owner_immediate[4]),atomic_load(&owner_immediate[5]),
               atomic_load(&owner_immediate[6]),atomic_load(&owner_immediate[7]),atomic_load(&owner_immediate[8]),
               atomic_load(&owner_immediate[9]),atomic_load(&owner_immediate[10]),atomic_load(&owner_immediate[11]),
               atomic_load(&owner_immediate[12]),atomic_load(&owner_immediate[13]),atomic_load(&owner_immediate[14]),atomic_load(&owner_ready_yields));
        PROBE_SNAPSHOT_LOG("{\"probe\":\"vhci_timing\",\"collection_cycle_us\":%" PRIu32 ",\"packets\":%" PRIu32 ",\"acl\":%" PRIu32
               ",\"events\":%" PRIu32 ",\"bytes\":%" PRIu32 ",\"isr_callbacks\":%" PRIu32 ",\"rejected\":%" PRIu32
               ",\"callback_max_us\":%" PRIu32 ",\"first_rx_to_poll_max_us\":%" PRIu32 ",\"host_poll_max_us\":%" PRIu32
               ",\"ingress_high_water_bytes\":%" PRIu32 ",\"ingress_rejected\":%" PRIu32 ",\"logger_stack_low_water_bytes\":%u"
               ",\"watchdog_events\":%" PRIu32 ",\"watchdog_hook_at_us\":%" PRIu32 ",\"watchdog_isr_core\":%" PRIu32 "}\n",
               at,atomic_load(&rx_packets),atomic_load(&rx_acl),atomic_load(&rx_events),atomic_load(&rx_bytes),atomic_load(&rx_isr),
               atomic_load(&rx_rejected),atomic_load(&rx_callback_max_us),atomic_load(&first_rx_to_poll_max_us),atomic_load(&host_poll_max_us),
               atomic_load(&profile.runtime->host.ingress.high_water),atomic_load(&profile.runtime->host.ingress.rejected),
               (unsigned)uxTaskGetStackHighWaterMark(logger_task),atomic_load(&watchdog_events),atomic_load(&watchdog_hook_us),atomic_load(&watchdog_isr_core));
        PROBE_SNAPSHOT_LOG("{\"probe\":\"media_drop_reasons\",\"collection_cycle_us\":%" PRIu32 ",\"queue_full\":%" PRIu32 ",\"decode_reject\":%" PRIu32
               ",\"reset_failed\":%" PRIu32 ",\"pcm_failed\":%" PRIu32 ",\"pcm_timeout\":%" PRIu32 ",\"pcm_busy\":%" PRIu32
               ",\"input_sequence_breaks\":%" PRIu32 ",\"input_ssrc_changes\":%" PRIu32 ",\"queue_high_water\":%" PRIu32
               ",\"host_trace_lost\":%" PRIu32 ",\"worker_trace_lost\":%" PRIu32 ",\"snapshot_records_lost\":%" PRIu32 "}\n",at,
               atomic_load(&worker.queue_full_drops),atomic_load(&worker.decode_rejects),atomic_load(&worker.reset_failures),
               atomic_load(&worker.pcm_failures),atomic_load(&worker.pcm_timeouts),atomic_load(&worker.pcm_backpressure),
               atomic_load(&worker.input_sequence_breaks),atomic_load(&worker.input_ssrc_changes),atomic_load(&worker.queue_high_water),
               atomic_load(&media_trace_rings[0].lost),atomic_load(&media_trace_rings[1].lost),atomic_load(&snapshot_format_lost));
        PROBE_SNAPSHOT_LOG("{\"probe\":\"l2cap_reassembly\",\"collection_cycle_us\":%" PRIu32 ",\"delivered\":%" PRIu32
            ",\"abandoned\":%" PRIu32 ",\"rejected_wire_packets\":%" PRIu32 ",\"orphan_continuations\":%" PRIu32 ",\"malformed_signaling\":%" PRIu32 ",\"unknown_responses\":%" PRIu32 ",\"retained_storage_bytes\":%" PRIu32 "}\n",
            at,atomic_load(&l2cap_delivered),atomic_load(&l2cap_abandoned),atomic_load(&l2cap_rejected),atomic_load(&l2cap_orphaned),atomic_load(&l2cap_malformed_signaling),atomic_load(&l2cap_unknown_responses),atomic_load(&l2cap_retained_bytes));
        PROBE_SNAPSHOT_LOG("{\"probe\":\"worker_timing\",\"collection_cycle_us\":%" PRIu32 ",\"steps\":%" PRIu32 ",\"elapsed_us\":%" PRIu32
               ",\"max_step_us\":%" PRIu32 ",\"wall_time_includes_preemption\":true,\"step_budget\":2,\"histogram_us_le\":[125,250,500,1000,2000,4000,8000],\"histogram_counts\":[%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 "]}\n",
               at,atomic_load(&worker_steps),atomic_load(&worker_elapsed_us),atomic_load(&worker_step_max_us),
               atomic_load(&worker_hist[0]),atomic_load(&worker_hist[1]),atomic_load(&worker_hist[2]),atomic_load(&worker_hist[3]),
               atomic_load(&worker_hist[4]),atomic_load(&worker_hist[5]),atomic_load(&worker_hist[6]),atomic_load(&worker_hist[7]));
        static const char *const wait_names[]={"pcm_space","pcm_loan_retirement","control","encoded_input"};
        uint32_t waiting=atomic_load_explicit(&decoder_wait_kind,memory_order_acquire);
        uint32_t wait_began=atomic_load_explicit(&decoder_wait_began,memory_order_relaxed);
        for(unsigned reason=0;reason<4;++reason){
            uint32_t stamp=(uint32_t)esp_timer_get_time();
            snapshot_log(stamp,"{\"probe\":\"decoder_wait\",\"reason\":\"%s\",\"core\":%" PRIu32
                ",\"calls\":%" PRIu32 ",\"elapsed_us\":%" PRIu32 ",\"maximum_us\":%" PRIu32
                ",\"currently_waiting\":%s,\"current_wait_us\":%" PRIu32 ",\"wall_time_includes_preemption\":true}\n",
                wait_names[reason],atomic_load(&decoder_run_core),atomic_load(&decoder_wait_calls[reason]),
                atomic_load(&decoder_wait_elapsed[reason]),atomic_load(&decoder_wait_maximum[reason]),
                waiting==reason?"true":"false",waiting==reason?stamp-wait_began:0u);
        }
        static const char *const phases[]={"decoder_execution","decoder_reset","pcm_delivery"};
        for(unsigned i=0;i<3;++i){
            phase_timing_t *p=&phase_timing[i];
            PROBE_SNAPSHOT_LOG("{\"probe\":\"%s\",\"collection_cycle_us\":%" PRIu32 ",\"calls\":%" PRIu32 ",\"elapsed_us\":%" PRIu32
                   ",\"maximum_us\":%" PRIu32 ",\"wall_time_includes_preemption\":true,\"includes_inline_pcm\":%s,\"histogram_us_le\":[125,250,500,1000,2000,4000,8000],\"histogram_counts\":[%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 "]}\n",
                   phases[i],at,atomic_load(&p->calls),atomic_load(&p->elapsed_us),atomic_load(&p->maximum_us),
                   i==0 && !worker.hooks.deferred_pcm?"true":"false",
                   atomic_load(&p->hist[0]),atomic_load(&p->hist[1]),atomic_load(&p->hist[2]),atomic_load(&p->hist[3]),
                   atomic_load(&p->hist[4]),atomic_load(&p->hist[5]),atomic_load(&p->hist[6]),atomic_load(&p->hist[7]));
        }
        uint32_t pcm_snapshot_began=(uint32_t)esp_timer_get_time();
        uint32_t generation=atomic_load_explicit(&pcm_generation,memory_order_acquire);
        if(generation&1u)continue;
        uint32_t frames=atomic_load_explicit(&pcm_frames,memory_order_relaxed),bytes=atomic_load_explicit(&pcm_bytes,memory_order_relaxed);
        uint32_t hash=atomic_load_explicit(&pcm_hash,memory_order_relaxed),rate=atomic_load_explicit(&pcm_rate,memory_order_relaxed);
        uint32_t channels=atomic_load_explicit(&pcm_channels,memory_order_relaxed),codec=atomic_load_explicit(&pcm_codec,memory_order_relaxed);
        uint32_t sample_bytes=atomic_load_explicit(&pcm_sample_bytes,memory_order_relaxed);
        uint32_t revision=atomic_load_explicit(&codec_revision,memory_order_acquire),cie_size=atomic_load(&codec_cie_size),cie_words[(DBT_CODEC_CIE_BYTES+3u)/4u];
        for(unsigned i=0;i<(DBT_CODEC_CIE_BYTES+3u)/4u;++i)cie_words[i]=atomic_load(&codec_cie_words[i]);
        if(generation!=atomic_load_explicit(&pcm_generation,memory_order_acquire))continue;
        if(revision!=previous_codec_revision){
            static const char hex[]="0123456789abcdef";char cie[2u*DBT_CODEC_CIE_BYTES+1u];unsigned n=cie_size>DBT_CODEC_CIE_BYTES?DBT_CODEC_CIE_BYTES:cie_size;
            for(unsigned i=0;i<n;++i){uint8_t b=(uint8_t)(cie_words[i/4u]>>(8u*(i%4u)));cie[2*i]=hex[b>>4];cie[2*i+1]=hex[b&15u];}
            cie[2*n]=0;
            snapshot_log(pcm_snapshot_began,"{\"probe\":\"codec_configuration\",\"collection_cycle_us\":%" PRIu32 ",\"revision\":%" PRIu32 ",\"codec\":%" PRIu32
                   ",\"rate\":%" PRIu32 ",\"channels\":%" PRIu32 ",\"pcm_container_bytes\":%" PRIu32 ",\"cie_size\":%" PRIu32 ",\"cie_hex\":\"%s\"}\n",
                   at,revision,codec,rate,channels,sample_bytes,cie_size,cie);previous_codec_revision=revision;
        }
        snapshot_log(pcm_snapshot_began,"{\"probe\":\"media_input\",\"collection_cycle_us\":%" PRIu32 ",\"stream_epoch\":%" PRIu32 ",\"packets\":%" PRIu32
               ",\"payload_bytes\":%" PRIu32 ",\"rtp_first_timestamp\":%" PRIu32 ",\"rtp_last_timestamp\":%" PRIu32 ",\"rate\":%" PRIu32 "}\n",
               at,atomic_load(&worker.input_stream_epoch),atomic_load(&worker.input_packets),atomic_load(&worker.input_payload_bytes),
               atomic_load(&worker.input_first_timestamp),atomic_load(&worker.input_last_timestamp),rate);
        snapshot_log(pcm_snapshot_began,"{\"probe\":\"sbc_pcm\",\"frames\":%" PRIu32 ",\"bytes\":%" PRIu32 ",\"fnv1a32\":%" PRIu32
               ",\"fnv1a32_enabled\":" AUDIO_PCM_HASH_JSON ",\"frames_unit\":\"pcm_views\""
               ",\"rate\":%" PRIu32 ",\"channels\":%" PRIu32 ",\"media_packets\":%" PRIu32 ",\"drops\":%" PRIu32
               ",\"resets\":%" PRIu32 ",\"resident\":%" PRIu32 ",\"codec\":%" PRIu32 ",\"codec_name\":\"%s\",\"diagnostic_scope\":\"all_codecs\",\"sample_bytes\":%" PRIu32
               ",\"faults\":%" PRIu32 ",\"decoder_stack_low_water_bytes\":%u,\"internal_heap_free\":%u,\"internal_heap_min_free\":%u,\"internal_heap_largest\":%u,\"i2s_output\":" AUDIO_OUTPUT_JSON "}\n",frames,bytes,hash,rate,channels,
               atomic_load(&worker.packets),atomic_load(&worker.media_drops),atomic_load(&worker.resets),atomic_load(&worker.resident_bytes),codec,codec_name(codec),sample_bytes,
               atomic_load(&worker.faults),(unsigned)uxTaskGetStackHighWaterMark(decoder_task),
               (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT),
               (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT),
               (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT));
        uint32_t profile_snapshot_began=(uint32_t)esp_timer_get_time();
        uint32_t snapshot=atomic_load(&profile_snapshot),errors=atomic_load(&event_errors);
        uint32_t ignored=atomic_load(&ignored_dynamic),oversized=atomic_load(&oversized_dynamic);
        if(snapshot!=previous || errors!=previous_errors || ignored!=previous_ignored || oversized!=previous_oversized){
            previous=snapshot;previous_errors=errors;
            previous_ignored=ignored;previous_oversized=oversized;
            snapshot_log(profile_snapshot_began,"{\"probe\":\"native_profile\",\"state\":%u,\"active\":%u,\"enabled\":%u,\"draining\":%u,"
                   "\"event_errors\":%" PRIu32 ",\"last_event\":%" PRIu32 ",\"last_result\":%" PRIu32 ",\"detail\":%" PRIu32
                   ",\"ignored_dynamic\":%" PRIu32 ",\"oversized_dynamic\":%" PRIu32 "}\n",
                   (unsigned)(snapshot&255u),(unsigned)((snapshot>>8)&1u),(unsigned)((snapshot>>9)&1u),
                   (unsigned)((snapshot>>10)&1u),errors,atomic_load(&last_channel_event),atomic_load(&last_channel_result),snapshot>>11,ignored,oversized);
        }
#ifdef DBT_PROBE_AVRCP
        (void)log_metadata();
#endif
#ifdef DBT_PROBE_AUDIO_OUTPUT
        uint32_t output_snapshot_began=(uint32_t)esp_timer_get_time();
        probe_output_stats_t stats;probe_output_stats(&stats);
#ifdef DBT_PROBE_PCM_LOANS
        snapshot_log(output_snapshot_began,"{\"probe\":\"i2s_pcm_loans\",\"mode\":%" PRIu32 ",\"input_ring_bytes_avoided\":%" PRIu32
               ",\"renderer_begun\":%" PRIu32 ",\"renderer_retired\":%" PRIu32 ",\"renderer_cancelled\":%" PRIu32
               ",\"active\":%" PRIu32 ",\"max_hold_us\":%" PRIu32 "}\n",stats.pcm_loan_mode,stats.input_ring_bytes_avoided,
               stats.pcm_loans_begun,stats.pcm_loans_retired,stats.pcm_loans_cancelled,stats.pcm_loan_active,stats.pcm_loan_max_us);
#endif
        snapshot_log(output_snapshot_began,"{\"probe\":\"i2s_allocation_failure\",\"failed_bytes\":%" PRIu32
               ",\"free_bytes_at_failure\":%" PRIu32 ",\"largest_block_at_failure\":%" PRIu32
               ",\"requested_rate\":%" PRIu32 ",\"requested_channels\":%" PRIu32 ",\"requested_sample_bytes\":%" PRIu32
               ",\"failure_at_us\":%" PRIu32 ",\"audio_bytes_requested\":%" PRIu32 ",\"audio_bytes_allocated\":%" PRIu32
               ",\"output_pages_allocated\":%" PRIu32 ",\"free_before_audio\":%" PRIu32 ",\"free_after_audio_cleanup\":%" PRIu32
               ",\"free_32bit_at_failure\":%" PRIu32 ",\"largest_32bit_at_failure\":%" PRIu32 "}\n",stats.allocation_failed_bytes,stats.allocation_failure_free,
               stats.allocation_failure_largest,stats.allocation_failure_rate,stats.allocation_failure_channels,
               stats.allocation_failure_sample_bytes,stats.allocation_failure_at_us,stats.allocation_failure_audio_requested,
               stats.allocation_failure_audio_allocated,stats.allocation_failure_pages,stats.allocation_failure_free_before,
               stats.allocation_failure_free_after_cleanup,stats.allocation_failure_free_32bit,stats.allocation_failure_largest_32bit);
        snapshot_log(output_snapshot_began,"{\"probe\":\"i2s_callback_work\",\"configured_irq_priority\":%" PRIu32
               ",\"calls\":%" PRIu32 ",\"elapsed_us\":%" PRIu32 ",\"maximum_us\":%" PRIu32 ",\"core\":%" PRIu32
               ",\"application_callback_only\":true,\"elapsed_includes_preemption\":true}\n",
               stats.configured_irq_priority,stats.callback_calls,stats.callback_elapsed_us,stats.callback_max_us,stats.callback_core);
        snapshot_log(output_snapshot_began,"{\"probe\":\"render_work\",\"core\":%" PRIu32 ",\"collection_cycle_us\":%" PRIu32 ",\"calls\":%" PRIu32 ",\"elapsed_us\":%" PRIu32
               ",\"maximum_us\":%" PRIu32 ",\"wall_time_includes_preemption\":true,\"input_ready_frames\":%" PRIu32 ",\"render_stack_low_water_bytes\":%" PRIu32
               ",\"allocation_failed_bytes\":%" PRIu32 ",\"allocation_failure_free\":%" PRIu32 ",\"allocation_failure_largest\":%" PRIu32
               ",\"request_fault\":%" PRIu32 ",\"stale_receipts\":%" PRIu32 "}\n",stats.render_core,at,stats.render_calls,stats.render_total_us,
               stats.maximum_render_us,stats.input_ready_frames,stats.render_stack_low_water_bytes,stats.allocation_failed_bytes,
               stats.allocation_failure_free,stats.allocation_failure_largest,stats.request_fault,stats.stale_receipts);
        snapshot_log(output_snapshot_began,"{\"probe\":\"i2s_descriptor_faults\",\"collection_cycle_us\":%" PRIu32 ",\"shape\":%" PRIu32 ",\"pool\":%" PRIu32
               ",\"order\":%" PRIu32 ",\"expected_index\":%" PRIu32 ",\"observed_index\":%" PRIu32 ",\"observed_size\":%" PRIu32
               ",\"fault_at_us\":%" PRIu32 ",\"fault_gap_us\":%" PRIu32 ",\"fault_stopping\":%" PRIu32 "}\n",
               at,stats.descriptor_shape_errors,stats.descriptor_pool_errors,stats.descriptor_order_errors,
               stats.descriptor_expected_index,stats.descriptor_observed_index,stats.descriptor_observed_size,
               stats.descriptor_fault_at_us,stats.descriptor_fault_gap_us,stats.descriptor_fault_stopping);
        snapshot_log(output_snapshot_began,"{\"probe\":\"native_i2s\",\"running\":%" PRIu32 ",\"failed\":%" PRIu32 ",\"rate\":%" PRIu32
               ",\"requested_bytes\":%" PRIu32 ",\"accepted_frames\":%" PRIu32 ",\"rendered_frames\":%" PRIu32 ",\"ready_frames\":%" PRIu32
               ",\"dma_eofs\":%" PRIu32 ",\"dma_source_frames\":%" PRIu32 ",\"underrun_frames\":%" PRIu32
               ",\"descriptor_errors\":%" PRIu32 ",\"max_eof_gap_us\":%" PRIu32 ",\"maximum_render_us\":%" PRIu32
               ",\"first_underrun_accepted_frames\":%" PRIu32 ",\"first_underrun_dma_frames\":%" PRIu32 ",\"pcm_target_ms\":%" PRIu32 ",\"block_dsp\":%" PRIu32 ",\"word_dsp\":%" PRIu32
               ",\"pcm_input_mode\":%" PRIu32 ",\"input_capacity_frames\":%" PRIu32 ",\"input_ring_bytes\":%" PRIu32 "}\n",
               stats.running,stats.failed,stats.rate,stats.requested_bytes,stats.accepted_frames,stats.rendered_frames,stats.ready_frames,
               stats.dma_eofs,stats.dma_source_frames,stats.underrun_frames,stats.descriptor_errors,stats.max_eof_gap_us,stats.maximum_render_us,
               stats.first_underrun_accepted_frames,stats.first_underrun_dma_frames,stats.pcm_target_ms,stats.block_dsp,stats.word_dsp,
               stats.pcm_input_mode,stats.input_capacity_frames,stats.input_ring_bytes);
        snapshot_log(output_snapshot_began,"{\"probe\":\"audio_clock\",\"ppm\":%" PRId32 ",\"observed_ppm\":%" PRId32 ",\"error_frames\":%" PRId32
               ",\"state\":%" PRIu32 ",\"target_frames\":%" PRIu32 ",\"updates\":%" PRIu32 ",\"disturbances\":%" PRIu32
               ",\"observed_valid\":%" PRIu32 ",\"snapshot_valid\":%" PRIu32 "}\n",stats.clock_ppm,stats.clock_observed_ppm,stats.clock_error_frames,
               stats.clock_state,stats.clock_target_frames,stats.clock_updates,stats.clock_disturbances,stats.clock_observed_valid,stats.clock_snapshot_valid);
#endif
    }
}
void probe_audio_product_observer(probe_audio_product_event_fn fn,void *context){product_observer=fn;product_context=context;}
static void sink_event(void *context,dbt_sink_event_t event,dbt_stream_state_t state){
    (void)context;
    dbt_sink_operation_t operation=profile.sink.pending.operation;
#ifdef DBT_PROBE_AUDIO_OUTPUT
    /* Reconfiguration keeps the signaling binding but may change the rate and
     * output latency. Its first report must not wait for the prior cadence. */
    if(event==DBT_SINK_STATE && profile.sink.pending.used &&
       (operation==DBT_SINK_CONFIGURE || operation==DBT_SINK_RECONFIGURE))delay_binding=0;
#endif
    if(!product_observer || event!=DBT_SINK_STATE || !profile.signaling.serial)return;
    if(profile.sink.pending.used && (operation==DBT_SINK_CONFIGURE || operation==DBT_SINK_RECONFIGURE)){
        uint32_t rate=48000;dbt_avdtp_caps_t caps;dbt_avdtp_cap_error_t error;dbt_codec_config_t config;
        if(dbt_avdtp_caps_validate(profile.sink.configuration,profile.sink.configuration_size,DBT_AVDTP_CONFIGURATION,&caps,&error)==DBT_OK)
            for(unsigned i=0;i<sizeof(codecs)/sizeof(codecs[0]);i++)if(codecs[i]->configure(caps.codec.data,caps.codec.size,&config)==DBT_OK){rate=config.rate;break;}
        product_observer(product_context,PROBE_AUDIO_CONFIGURED,profile.signaling.handle,profile.signaling.generation,rate,now_ms());
    }else if(state==DBT_STREAM_STREAMING)
        product_observer(product_context,PROBE_AUDIO_STREAMING,profile.signaling.handle,profile.signaling.generation,0,now_ms());
}
dbt_result_t probe_audio_init(dbt_runtime_t *runtime,TaskHandle_t host_owner){
    if(restart_hold)return DBT_BUSY;
    owner=host_owner;
    dbt_codec_worker_hooks_t hooks={0};hooks.decoder=(dbt_codec_hooks_t){0,codec_allocate,release,pcm};
    hooks.codecs=codecs;hooks.codec_count=(uint8_t)(sizeof(codecs)/sizeof(codecs[0]));
    hooks.wake_worker=wake_worker;hooks.wake_owner=wake_owner;
    hooks.trace=media_trace;
#ifdef DBT_PROBE_ASYNC_AUDIO
    hooks.deferred_pcm=true;hooks.async_stream=(dbt_codec_stream_async_t){stream_begin,stream_poll,stream_cancel};
#ifdef DBT_PROBE_PCM_LOANS
    hooks.async_pcm=(dbt_codec_pcm_async_t){pcm_begin,pcm_poll,pcm_cancel};
#endif
#else
    hooks.stream=stream;
#endif
    dbt_result_t result=dbt_codec_worker_init(&worker,hooks,PROBE_A2DP_RX_MTU,PROBE_A2DP_MEDIA_SLOTS);
    if(result!=DBT_OK)return result;
    dbt_avdtp_sink_hooks_t client={&worker,allocate,release,dbt_codec_worker_submit,sink_event,dbt_codec_worker_media};
    result=dbt_a2dp_acceptor_init(&profile,runtime,client,endpoints,(uint8_t)(sizeof(endpoints)/sizeof(endpoints[0])),PROBE_A2DP_RX_MTU,64,5000,2000,5000);
    if(result!=DBT_OK)return result;
    if(xTaskCreatePinnedToCore(decode_loop,"native_decoder",8192,0,DBT_PROBE_DECODER_PRIORITY,&decoder_task,DBT_PROBE_DECODER_CORE)!=pdPASS)return DBT_NO_SPACE;
#ifdef DBT_PROBE_AUDIO_OUTPUT
    result=probe_audio_output_bootstrap();
    if(result!=DBT_OK){vTaskDelete(decoder_task);decoder_task=0;return result;}
#endif
    if(xTaskCreatePinnedToCore(log_loop,"native_pcm_log",4096,0,1,&logger_task,AUDIO_LOGGER_CORE)!=pdPASS){vTaskDelete(decoder_task);decoder_task=0;return DBT_NO_SPACE;}
#ifdef DBT_PROBE_SCHEDULER_TRACE
    esp_err_t sampled=probe_scheduler_start(owner,decoder_task,logger_task);
    printf("{\"probe\":\"scheduler_sampler\",\"active\":%s,\"result\":%u,\"sample_hz\":%u,\"task_count_added\":0,\"cpu_runtime_exact\":false}\n",
        sampled==ESP_OK?"true":"false",(unsigned)sampled,(unsigned)configTICK_RATE_HZ);
#endif
    initialized=true;
    printf("{\"probe\":\"scheduler_layout\",\"host_core\":%u,\"controller_core\":%u,\"host_priority\":%u,\"decoder_core\":%u,\"decoder_priority\":%u,\"owner_ready_yield\":%u,\"renderer_core\":%u,\"renderer_priority\":%u,\"logger_core\":%u,\"background_core\":%u,\"logger_priority\":1}\n",(unsigned)xPortGetCoreID(),(unsigned)DBT_ESP32_BLUETOOTH_CORE,(unsigned)DBT_PROBE_HOST_PRIORITY,(unsigned)DBT_PROBE_DECODER_CORE,(unsigned)DBT_PROBE_DECODER_PRIORITY,(unsigned)DBT_PROBE_OWNER_READY_YIELD,(unsigned)DBT_ESP32_RENDER_CORE,(unsigned)DBT_PROBE_RENDER_PRIORITY,(unsigned)AUDIO_LOGGER_CORE,(unsigned)DBT_ESP32_BACKGROUND_CORE);
    printf("{\"probe\":\"sbc_probe_ready\",\"psm\":25,\"mtu\":%u,\"worker_bytes\":%u,\"profile_bytes\":%u,\"decoder_stack\":8192,"
           "\"logger_stack\":4096,\"codecs\":" AUDIO_CODECS_JSON ",\"diagnostic_only\":true,\"audio_sink_sdp_advertised\":" AUDIO_SDP_JSON ",\"i2s_output\":" AUDIO_OUTPUT_JSON ",\"rf_protocol_diagnostic\":" AUDIO_RF_DIAG_JSON "}\n",
           (unsigned)PROBE_A2DP_RX_MTU,(unsigned)sizeof(worker),(unsigned)sizeof(profile));
    return DBT_OK;
}
bool probe_audio_accept(void *context,const dbt_link_t *link,uint16_t psm){(void)context;return psm!=0x19 || (initialized && !restart_hold && dbt_a2dp_acceptor_accept(&profile,link));}
void probe_audio_event(dbt_channel_event_t event,const dbt_channel_t *channel){
    if(!initialized || channel->psm!=0x19)return;
    bool was_active=profile.active;
    bool signal=profile.signaling.serial && channel->serial==profile.signaling.serial &&
        channel->handle==profile.signaling.handle && channel->link_generation==profile.signaling.generation;
    uint16_t handle=channel->handle;uint32_t generation=channel->link_generation;
    dbt_result_t result=dbt_a2dp_acceptor_event(&profile,event,channel);
    if(result==DBT_OK && product_observer){
        if(event==DBT_CH_EVENT_OPEN && !was_active && profile.active && profile.signaling.serial==channel->serial)
            product_observer(product_context,PROBE_AUDIO_CONNECTED,handle,generation,0,now_ms());
        else if(signal && (event==DBT_CH_EVENT_CLOSED || event==DBT_CH_EVENT_FAILED))
            product_observer(product_context,PROBE_AUDIO_DISCONNECTED,handle,generation,0,now_ms());
    }
    if(result!=DBT_OK && result!=DBT_EMPTY){
        atomic_store(&last_channel_event,(uint32_t)event);atomic_store(&last_channel_result,(uint32_t)result);
        atomic_fetch_add(&event_errors,1);
        (void)dbt_a2dp_acceptor_retire(&profile,now_ms());
    }
}
void probe_audio_receive(const dbt_channel_t *channel,const uint8_t *data,uint16_t size){
    if(initialized){
        if(channel && profile.media.serial && channel->serial==profile.media.serial)maximum(&media_sdu_max,size);
        (void)dbt_a2dp_acceptor_receive(&profile,channel,data,size);
    }
}
void probe_audio_poll(uint32_t now){
    if(!initialized)return;
    dbt_codec_receipt_t receipt;
    for(unsigned i=0;i<DBT_CODEC_WORKER_JOBS && dbt_codec_worker_receipt(&worker,&receipt)==DBT_OK;++i)
        (void)dbt_a2dp_acceptor_complete(&profile,receipt.binding,receipt.token,receipt.result,now);
    uint32_t faults=atomic_load_explicit(&worker.faults,memory_order_acquire);
    if(faults!=worker_faults_seen){
        worker_faults_seen=faults;
        /* Retire this binding through channel closure and acknowledged DROP.
         * Do not disable the service forever because its lifetime counter is
         * nonzero. Admission stays closed while draining; restart holds and
         * renderer/ownership failures retain their explicit stop policy. */
        (void)dbt_a2dp_acceptor_retire(&profile,now);
    }
    if(failed || restart_hold)restart_drained=dbt_a2dp_acceptor_stop(&profile,now)==DBT_OK;
    (void)dbt_a2dp_acceptor_poll(&profile,now);
    atomic_store(&l2cap_delivered,profile.runtime->rx.delivered);
    atomic_store(&l2cap_abandoned,profile.runtime->rx.abandoned);
    atomic_store(&l2cap_rejected,profile.runtime->rx.rejected);
    atomic_store(&l2cap_orphaned,profile.runtime->rx.orphaned);
    atomic_store(&l2cap_malformed_signaling,profile.runtime->channels.malformed_pdus);
    atomic_store(&l2cap_unknown_responses,profile.runtime->channels.unknown_responses);
    atomic_store(&l2cap_retained_bytes,profile.runtime->rx.storage?(uint32_t)DBT_MAX_LINKS*profile.runtime->rx.storage_stride:0u);
    atomic_store(&ignored_dynamic,profile.runtime->channels.ignored_dynamic_pdus);
    atomic_store(&oversized_dynamic,profile.runtime->channels.oversized_dynamic_pdus);
    const dbt_channel_oversize_t *oversize=&profile.runtime->channels.last_oversized;
    atomic_store(&last_oversized_size,oversize->size);atomic_store(&last_oversized_mtu,oversize->rx_mtu);atomic_store(&last_oversized_psm,oversize->psm);
#ifdef DBT_PROBE_RF_PROTOCOL_DIAGNOSTIC
    static uint32_t oversize_seen;static unsigned oversize_logged;
    uint32_t oversized=profile.runtime->channels.oversized_dynamic_pdus;
    if(oversized!=oversize_seen){
        oversize_seen=oversized;
        if(oversize_logged<4){++oversize_logged;probe_security_oversized(&profile.runtime->channels.last_oversized);}
    }
#endif
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
#ifndef DBT_PROBE_RF_PROTOCOL_DIAGNOSTIC
    if(probe_output_failed())failed=true;
#endif
    if(delay_report_wait_ms(now)==0){
        /* Rate-limit attempts as well as successes: an unavailable/invalid
         * provider must not create an immediately runnable owner loop. */
        delay_binding=profile.sink.binding;delay_at=now;
        dbt_avdtp_caps_t caps;dbt_avdtp_cap_error_t error;dbt_codec_config_t config;bool selected=false;
        if(dbt_avdtp_caps_validate(profile.sink.configuration,profile.sink.configuration_size,DBT_AVDTP_CONFIGURATION,&caps,&error)==DBT_OK){
            for(unsigned i=0;i<sizeof(codecs)/sizeof(codecs[0]);++i)
                if(codecs[i]->configure(caps.codec.data,caps.codec.size,&config)==DBT_OK){selected=true;break;}
        }
        if(selected)(void)dbt_avdtp_sink_delay(&profile.sink,probe_output_delay_100us(config.rate),now);
        /* A request can observe a signaling expiry after acceptor_poll. Do not
         * go idle with a failed signal and no remaining owner deadline. */
        if(profile.sink.failed || profile.sink.signal.failed)(void)dbt_a2dp_acceptor_retire(&profile,now);
    }
#endif
}
uint32_t probe_audio_wait_ms(uint32_t now){
    if(!initialized)return UINT32_MAX;
    uint32_t wait=dbt_a2dp_acceptor_wait_ms(&profile,now);
#ifdef DBT_PROBE_AUDIO_OUTPUT
    uint32_t report=delay_report_wait_ms(now);if(report<wait)wait=report;
#endif
    return wait;
}
const dbt_link_t *probe_audio_peer_link(uint16_t handle,uint32_t generation){
    if(!initialized || !profile.runtime || !generation)return 0;
    const dbt_link_t *link=dbt_acl_find(&profile.runtime->acl,handle);
    return link && link->kind==DBT_LINK_CLASSIC && link->generation==generation?link:0;
}
dbt_result_t probe_audio_peer(uint16_t *handle,uint32_t *generation){
    if(!handle || !generation)return DBT_INVALID;
    if(!initialized || !profile.active || !profile.signaling.serial ||
       !probe_audio_peer_link(profile.signaling.handle,profile.signaling.generation))return DBT_EMPTY;
    *handle=profile.signaling.handle;*generation=profile.signaling.generation;return DBT_OK;
}

dbt_result_t probe_audio_restart_hold(bool hold,uint32_t now){
    restart_hold=hold;restart_drained=!initialized;
    if(!initialized)return DBT_OK;
    if(!hold){if(!failed)profile.enabled=true;return failed?DBT_INVALID:DBT_OK;}
    dbt_result_t r=dbt_a2dp_acceptor_stop(&profile,now);restart_drained=r==DBT_OK;
    return failed?DBT_INVALID:r;
}
bool probe_audio_restart_quiescent(void){return restart_hold && restart_drained && !failed;}
