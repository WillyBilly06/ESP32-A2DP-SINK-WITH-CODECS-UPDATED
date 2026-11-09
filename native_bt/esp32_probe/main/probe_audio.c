#include "probe_heap_alloc_trace.h"
#include "probe_schedule_limits.h"
#include "probe_audio.h"
#include "probe_audio_trace.h"
#if (defined(DBT_PROBE_SCHEDULER_TRACE) || defined(DBT_PROBE_STAY_NATIVE)) && defined(DBT_PROBE_AUDIO_OUTPUT)
#include "probe_arrival.h"
#include "probe_owner_state.h"
#endif
#if defined(DBT_PROBE_SCHEDULER_TRACE) && defined(DBT_PROBE_AUDIO_OUTPUT)
#include "probe_vhci_ready.h"
#endif
#include <string.h>
#include "probe_sdp.h"
#ifdef DBT_PROBE_CONSOLE_IRQ
#include "probe_console.h"
#endif
#include "probe_audio_limits.h"
#if defined(DBT_PROBE_SCHEDULER_TRACE) || defined(DBT_PROBE_TASK_RUNTIME_STATS)
#include "probe_scheduler.h"
#endif
#ifdef DBT_PROBE_SCHEDULER_TRACE
#include "probe_pcm_level.h"
#endif
#include "probe_security.h"
#include "dbt_codec.h"
#include "dbt_pcm_snapshot.h"
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
#ifdef DBT_PROBE_LHDC_STAGE_PROFILE
#include "probe_lhdc_profile.h"
#endif
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
#include "probe_dispatch_trace.h"
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
static dbt_a2dp_connect_t *outgoing_connect;
static dbt_a2dp_rank_t *outgoing_rank;
static dbt_codec_worker_t worker;
static TaskHandle_t owner,decoder_task,logger_task;
static _Atomic uint32_t decoder_run_core;
static _Atomic uint32_t decoder_wait_calls[5],decoder_wait_elapsed[5],decoder_wait_maximum[5];
/* 0..4 are event waits;5 is only the codec media/resume call itself. */
enum { DECODER_COMPUTING=5u };
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
#ifdef DBT_PROBE_SCHEDULER_TRACE
static probe_pcm_level_t pcm_level;
static _Atomic uint32_t level_sequence,level_epoch,level_codec,level_rate,level_channels,level_width,level_frames,level_count,level_peak,level_lo,level_hi;
static uint32_t level_serial;
static void pcm_level_begin(uint32_t codec,uint32_t rate,uint32_t channels,uint32_t width){
    probe_pcm_level_start(&pcm_level,rate,channels,width);
    uint32_t seq=atomic_load_explicit(&level_sequence,memory_order_relaxed);
    atomic_store_explicit(&level_sequence,seq+1u,memory_order_seq_cst);
    atomic_store_explicit(&level_epoch,0,memory_order_relaxed);
    atomic_store_explicit(&level_codec,codec,memory_order_relaxed);
    if(!++level_serial)++level_serial;
    atomic_store_explicit(&level_sequence,seq+2u,memory_order_release);
}
static void pcm_level_observe(const void *samples,uint32_t frames){
    if(!probe_pcm_level_observe(&pcm_level,samples,frames))return;
    uint32_t seq=atomic_load_explicit(&level_sequence,memory_order_relaxed);
    atomic_store_explicit(&level_sequence,seq+1u,memory_order_seq_cst);
    atomic_store_explicit(&level_rate,pcm_level.rate,memory_order_relaxed);
    atomic_store_explicit(&level_channels,pcm_level.channels,memory_order_relaxed);
    atomic_store_explicit(&level_width,pcm_level.width,memory_order_relaxed);
    atomic_store_explicit(&level_frames,pcm_level.frames,memory_order_relaxed);
    atomic_store_explicit(&level_count,pcm_level.count,memory_order_relaxed);
    atomic_store_explicit(&level_peak,pcm_level.peak,memory_order_relaxed);
    atomic_store_explicit(&level_lo,(uint32_t)pcm_level.squares,memory_order_relaxed);
    atomic_store_explicit(&level_hi,(uint32_t)(pcm_level.squares>>32),memory_order_relaxed);
    atomic_store_explicit(&level_epoch,level_serial,memory_order_relaxed);
    atomic_store_explicit(&level_sequence,seq+2u,memory_order_release);
}
static void log_pcm_level(void);
#endif
static _Atomic uint32_t pcm_generation,pcm_frames,pcm_bytes,pcm_hash,pcm_rate,pcm_channels,pcm_codec,pcm_sample_bytes;
static _Atomic uint32_t codec_revision,codec_cie_size,codec_cie_words[(DBT_CODEC_CIE_BYTES+3u)/4u];
static _Atomic uint32_t profile_snapshot,event_errors,last_channel_event,last_channel_result;
static _Atomic uint32_t ignored_dynamic,oversized_dynamic;
static _Atomic uint32_t l2cap_delivered,l2cap_abandoned,l2cap_rejected,l2cap_orphaned,l2cap_oversized;
static _Atomic uint32_t l2cap_malformed_signaling,l2cap_unknown_responses,l2cap_retained_bytes;
static _Atomic uint32_t media_sdu_max,last_oversized_size,last_oversized_mtu,last_oversized_psm;
/* Separate single-writer rings for host admission and decoder events. Logging
 * never waits on the producer; record loss is itself exposed every second. */
typedef probe_audio_trace_record_t media_trace_t;
typedef probe_audio_trace_ring_t media_trace_ring_t;
static media_trace_ring_t media_trace_rings[2];
static _Atomic uint32_t rx_packets,rx_acl,rx_events,rx_bytes,rx_isr,rx_rejected,rx_pending_us;
static _Atomic uint32_t rx_callback_max_us,first_rx_to_poll_max_us,host_poll_max_us;
static _Atomic uint32_t worker_steps,worker_elapsed_us,worker_step_max_us,worker_hist[8];
static _Atomic uint32_t credit_window_status,credit_pending_classic,credit_pending_le,credit_acknowledged,credit_commands,credit_partial_commands,credit_peak_blocked,credit_media_queued;
static _Atomic uint32_t owner_iterations,owner_immediate[15];
static _Atomic uint32_t owner_ready_yields,owner_service_yields;
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
#ifdef DBT_PROBE_CONSOLE_IRQ
static _Atomic uint32_t snapshot_uart_lost;
#endif
static uint32_t snapshot_uart_failures(void){
#ifdef DBT_PROBE_CONSOLE_IRQ
    return atomic_load_explicit(&snapshot_uart_lost,memory_order_relaxed);
#else
    return 0;
#endif
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
    if(ready){
#ifdef DBT_PROBE_CONSOLE_IRQ
        if(!probe_console_write_record(line,strlen(line),sizeof(line)))
            atomic_store(&snapshot_uart_lost,atomic_load(&snapshot_uart_lost)+1u);
#else
        printf("%s",line);
#endif
    }
}
#ifdef DBT_PROBE_OWNER_PACER
#include "probe_owner_pacer.h"
#endif
#define PROBE_SNAPSHOT_LOG(...) do {uint32_t snapshot_began=(uint32_t)esp_timer_get_time();snapshot_log(snapshot_began,__VA_ARGS__);}while(0)
#if defined(PROBE_OUTPUT_GAP_TRACE) && defined(DBT_PROBE_AUDIO_OUTPUT)
static void decoder_timeline(uint32_t phase,uint32_t at){
    uint32_t read=atomic_load(&worker.media_read),written=atomic_load(&worker.media_written);
    probe_output_trace_decoder(phase,at,written-read);
}
static dbt_result_t timed_worker_media(void *context,const dbt_avdtp_media_t *packet){
    uint32_t began=(uint32_t)esp_timer_get_time();
    bool arrival_valid=false;uint32_t arrival_us=0,arrival_cursor=0,arrival_flags=0;
#if (defined(DBT_PROBE_SCHEDULER_TRACE) || defined(DBT_PROBE_STAY_NATIVE)) && defined(DBT_PROBE_AUDIO_OUTPUT)
    probe_arrival_sample_t arrival={0};
    dbt_runtime_t *runtime=profile.bearer.runtime;
    if(runtime && runtime->host.in_poll && runtime->host.in_callback){
        const uint8_t *raw=runtime->host.rx_packet;
        if(raw[0]==DBT_H4_ACL){
            uint32_t wire_bytes=5u+dbt_read_le16(raw+3);
            if(wire_bytes<=DBT_MAX_H4_BYTES)arrival_valid=probe_arrival_media(
                atomic_load_explicit(&runtime->host.ingress.read,memory_order_relaxed),
                raw,(uint16_t)wire_bytes,packet,&arrival);
        }
    }
    arrival_us=arrival.arrival_us;arrival_cursor=arrival.queue_cursor;arrival_flags=arrival.acl_flags;
#endif
    if(context && packet && packet->payload && packet->size){
        dbt_codec_worker_t *w=context;
        probe_output_trace_media_arrival(began,0,atomic_load(&w->started_epoch),packet->sequence,packet->timestamp,packet->ssrc,DBT_PENDING,
            arrival_valid,arrival_us,arrival_cursor,arrival_flags);
    }
    dbt_result_t result=dbt_codec_worker_media(context,packet);
    if(context && packet && packet->payload && packet->size){
        dbt_codec_worker_t *w=context;
        probe_output_trace_media_arrival(began,(uint32_t)esp_timer_get_time(),atomic_load(&w->input_stream_epoch),
            packet->sequence,packet->timestamp,packet->ssrc,result,arrival_valid,arrival_us,arrival_cursor,arrival_flags);
    }
    return result;
}
#else
#define timed_worker_media dbt_codec_worker_media
#endif
static void timing_event(unsigned kind){
#ifdef DBT_PROBE_DISPATCH_TRACE
    if(kind==DBT_CODEC_DECODE_BEGIN)probe_dispatch_mark(PROBE_DISPATCH_DECODE_ENTER,PROBE_DISPATCH_DECODE,0);
    else if(kind==DBT_CODEC_DECODE_END)probe_dispatch_mark(PROBE_DISPATCH_DECODE_LEAVE,PROBE_DISPATCH_DECODE,0);
#endif
    unsigned index=(kind-DBT_CODEC_DECODE_BEGIN)/2u;
    if(index>=3)return;
    /* Functional producer state, independent of optional timeline/profilers.
     * PCM/reset/end transitions revoke speculative helper admission. */
    atomic_store_explicit(&decoder_wait_kind,kind==DBT_CODEC_DECODE_BEGIN?
        DECODER_COMPUTING:UINT32_MAX,memory_order_release);
    phase_timing_t *phase=&phase_timing[index];uint32_t at=(uint32_t)esp_timer_get_time();
#if defined(PROBE_OUTPUT_GAP_TRACE) && defined(DBT_PROBE_AUDIO_OUTPUT)
    decoder_timeline((kind-DBT_CODEC_DECODE_BEGIN)%2u?0u:index+1u,at);
#endif
    if((kind-DBT_CODEC_DECODE_BEGIN)%2u==0){phase->began=at;return;}
    uint32_t elapsed=at-phase->began;
    atomic_fetch_add(&phase->calls,1);atomic_fetch_add(&phase->elapsed_us,elapsed);maximum(&phase->maximum_us,elapsed);
    unsigned bucket=0;uint32_t bound=125;
    while(bucket<7 && elapsed>bound){++bucket;bound*=2;}
    atomic_fetch_add(&phase->hist[bucket],1);
}
static uint32_t trace_clock(void *context){(void)context;return (uint32_t)esp_timer_get_time();}
/* Host-owner only; this copies a diagnostic snapshot and never touches stdio.
 * Full diagnostic storage is counted by host_trace_lost, not a protocol wait. */
dbt_result_t probe_audio_host_record(const probe_host_record_t *record){
    if(!initialized || !logger_task || xTaskGetCurrentTaskHandle()!=owner)return DBT_UNSUPPORTED;
    return probe_audio_trace_host(&media_trace_rings[0],record,trace_clock,NULL);
}
/* Host-owner submission observation. Acceptance closes the media gate inside
 * [begin_us,event_us]; a controller preemption can widen this interval. */
static dbt_result_t sink_work(void *context,const dbt_sink_job_t *job){
    uint32_t began=(uint32_t)esp_timer_get_time();
    dbt_result_t result=dbt_codec_worker_submit(context,job);
    if(result==DBT_OK && job){
        probe_host_record_t record={.kind=PROBE_HOST_A2DP_JOB,.value={job->binding,job->token,
            (uint32_t)job->operation,atomic_load(&worker.gate_epoch),began,atomic_load(&worker.started_epoch),0}};
        (void)probe_audio_host_record(&record);
    }
    return result;
}
dbt_result_t probe_audio_host_failure(const probe_host_failure_t *failure){
    if(!initialized || !logger_task || xTaskGetCurrentTaskHandle()!=owner)return DBT_UNSUPPORTED;
    return probe_audio_trace_failure(&media_trace_rings[0],failure,trace_clock,NULL);
}
dbt_result_t probe_audio_host_controller(const probe_host_controller_t *controller){
    if(!initialized || !logger_task || xTaskGetCurrentTaskHandle()!=owner)return DBT_UNSUPPORTED;
    return probe_audio_trace_controller(&media_trace_rings[0],controller,trace_clock,NULL);
}
#if defined(DBT_PROBE_SCHEDULER_TRACE) && !defined(DBT_PROBE_TASK_RUNTIME_STATS)
#define PCM_BUSY_TRACE_JSON "true"
#else
#define PCM_BUSY_TRACE_JSON "false"
#endif
static void media_trace(void *context,const dbt_codec_trace_t *event){
    (void)context;
#if !defined(DBT_PROBE_SCHEDULER_TRACE) || defined(DBT_PROBE_TASK_RUNTIME_STATS)
    /* Expected PCM backpressure is already counted by the worker. Runtime
     * observation preserves normal event load; detailed packet-only mode
     * keeps individual episodes. Both preserve faults,
     * sequence changes and the immutable first-gap snapshots. */
    if(event->kind==DBT_CODEC_PCM_BUSY)return;
#endif
    if(event->kind>=DBT_CODEC_DECODE_BEGIN && event->kind!=DBT_CODEC_INPUT_CLOCK_MAX){
        timing_event((unsigned)event->kind);return;
    }
    media_trace_ring_t *ring=&media_trace_rings[(event->kind<=DBT_CODEC_QUEUE_FULL || event->kind==DBT_CODEC_INPUT_CLOCK_MAX)?0:1];
    (void)probe_audio_trace_media(ring,event,atomic_load_explicit(&pcm_codec,memory_order_relaxed),trace_clock,NULL);
}
#if defined(PROBE_OUTPUT_GAP_TRACE) && defined(DBT_PROBE_AUDIO_OUTPUT)
static void log_first_starvation(void){
    static uint32_t prior_serial[2]={UINT32_MAX,UINT32_MAX},prior_captured[2];
    static const char *const names[2][5]={{"first_starvation","starvation_media","starvation_decoder","starvation_renderer","starvation_transport"},
                                         {"first_unfilled","unfilled_media","unfilled_decoder","unfilled_renderer","unfilled_transport"}};
    static const char *const phases[]={"between_operations","decode","reset","pcm_delivery","wait_pcm_space","wait_pcm_loan","wait_control","wait_encoded_input","wait_shared_renderer"};
    /* Two immutable latches, one read attempt each. UART remains logger-only. */
    for(unsigned kind=0;kind<2;++kind){
        probe_output_starvation_t s;
        if(kind)probe_output_first_unfilled(&s);else probe_output_first_starvation(&s);
        if(!s.coherent || (s.stream_serial==prior_serial[kind] && s.captured==prior_captured[kind]))continue;
        prior_serial[kind]=s.stream_serial;prior_captured[kind]=s.captured;
        PROBE_SNAPSHOT_LOG("{\"probe\":\"%s\",\"stream_serial\":%" PRIu32 ",\"captured\":%s,\"coherent\":true"
            ",\"event_us\":%" PRIu32 ",\"taken_frames\":%" PRIu32 ",\"input_frames\":%" PRIu32 ",\"output_frames\":%" PRIu32
            ",\"accepted_frames\":%" PRIu32 ",\"rendered_frames\":%" PRIu32 ",\"dma_source_frames\":%" PRIu32
            ",\"deadline_us\":%" PRIu32 ",\"pending_acl_metadata\":%" PRIu32 ",\"pending_acl_metadata_valid\":%s,\"finalized_shortage\":%s}\n",
            names[kind][0],s.stream_serial,s.captured?"true":"false",s.at_us,s.taken_frames,s.input_frames,s.output_frames,
            s.accepted_frames,s.rendered_frames,s.dma_source_frames,s.deadline_us,s.pending_acl_metadata,s.pending_acl_metadata!=UINT32_MAX?"true":"false",kind?"true":"false");
        if(!s.captured)continue;
        PROBE_SNAPSHOT_LOG("{\"probe\":\"%s\",\"stream_serial\":%" PRIu32 ",\"event_us\":%" PRIu32
            ",\"coherent\":%s,\"last_acl_any_link_us\":%" PRIu32 ",\"last_host_poll_us\":%" PRIu32
            ",\"media_begin_us\":%" PRIu32 ",\"media_admitted_us\":%" PRIu32 ",\"media_epoch\":%" PRIu32
            ",\"media_sequence\":%" PRIu32 ",\"media_timestamp\":%" PRIu32 ",\"media_ssrc\":%" PRIu32 ",\"media_result\":%" PRIu32
            ",\"last_media_gap_us\":%" PRIu32 ",\"max_media_gap_us\":%" PRIu32
            ",\"media_arrival_valid\":%s,\"media_current_acl_arrival_us\":%" PRIu32 ",\"media_hci_queue_cursor\":%" PRIu32 ",\"media_acl_flags\":%" PRIu32 ",\"media_hci_gap_us\":%" PRIu32 ",\"media_hci_gap_valid\":%s}\n",names[kind][1],s.stream_serial,s.at_us,s.media_coherent?"true":"false",
            s.acl_us,s.host_poll_us,s.media_us,s.media_admit_us,s.media_epoch,s.media_sequence,s.media_timestamp,s.media_ssrc,s.media_result,s.media_gap_us,s.media_max_gap_us,
            s.media_arrival_valid?"true":"false",s.media_arrival_us,s.media_arrival_cursor,s.media_arrival_flags,s.media_hci_gap_us,s.media_hci_gap_us!=UINT32_MAX?"true":"false");
        PROBE_SNAPSHOT_LOG("{\"probe\":\"%s\",\"stream_serial\":%" PRIu32 ",\"event_us\":%" PRIu32
            ",\"coherent\":%s,\"phase\":%" PRIu32 ",\"phase_name\":\"%s\",\"phase_since_us\":%" PRIu32 ",\"encoded_queued_at_phase_change\":%" PRIu32 "}\n",
            names[kind][2],s.stream_serial,s.at_us,s.decoder_coherent?"true":"false",s.decoder_phase,s.decoder_phase<9?phases[s.decoder_phase]:"unknown",s.decoder_at_us,s.decoder_queued);
        PROBE_SNAPSHOT_LOG("{\"probe\":\"%s\",\"stream_serial\":%" PRIu32 ",\"event_us\":%" PRIu32
            ",\"coherent\":%s,\"inside_audio_render\":%s,\"phase_since_us\":%" PRIu32 ",\"dsp_assist_state_at_gap\":%" PRIu32 "}\n",
            names[kind][3],s.stream_serial,s.at_us,s.render_coherent?"true":"false",s.render_phase?"true":"false",s.render_at_us,s.dsp_assist_state);
#ifdef PROBE_OWNER_STATE_TRACE
        const probe_owner_sample_t *o=&s.owner;
        static const char *const owner_names[]={"unknown","loop","runtime","profiles","deadlines","sleep","render_yield","ready_yield"};
        PROBE_SNAPSHOT_LOG("{\"probe\":\"%s_owner\",\"stream_serial\":%" PRIu32 ",\"event_us\":%" PRIu32
            ",\"coherent\":%s,\"phase\":%" PRIu32 ",\"phase_name\":\"%s\",\"phase_since_us\":%" PRIu32
            ",\"phase_is_intent_marker\":true,\"wait_ms\":%" PRIu32 ",\"inside_runtime_window\":%s,\"tx_owned_independent\":%" PRIu32 ",\"tx_owned_known\":%s}\n",
            kind?"unfilled":"starvation",s.stream_serial,s.at_us,o->coherent?"true":"false",o->phase,
            o->phase<sizeof(owner_names)/sizeof(owner_names[0])?owner_names[o->phase]:"invalid",o->phase_at_us,o->wait_ms,
            (o->flags&PROBE_OWNER_IN_RUNTIME)?"true":"false",o->tx_owned,(o->flags&PROBE_OWNER_BOUND)?"true":"false");
        PROBE_SNAPSHOT_LOG("{\"probe\":\"%s_credits\",\"stream_serial\":%" PRIu32 ",\"event_us\":%" PRIu32
            ",\"coherent\":%s,\"pending_known\":%s,\"counts_sampled_us\":%" PRIu32
            ",\"pending_classic\":%" PRIu32 ",\"pending_le\":%" PRIu32
            ",\"completion_submission_known\":%s,\"completion_submitted_us\":%" PRIu32 ",\"completion_packets\":%" PRIu32
            ",\"submission_is_sdk_return_not_controller_ack\":true}\n",
            kind?"unfilled":"starvation",s.stream_serial,s.at_us,o->coherent?"true":"false",
            o->coherent && (o->flags&PROBE_OWNER_PENDING)?"true":"false",o->credits_at_us,
            o->pending_counts&UINT16_MAX,o->pending_counts>>16,
            o->coherent && (o->flags&PROBE_OWNER_NCP)?"true":"false",o->ncp_at_us,o->ncp_packets);
#endif
#ifdef DBT_PROBE_SCHEDULER_TRACE
        PROBE_SNAPSHOT_LOG("{\"probe\":\"%s\",\"stream_serial\":%" PRIu32 ",\"event_us\":%" PRIu32
            ",\"sample_coherent\":%s,\"sample_valid\":%s,\"sample_us\":%" PRIu32
            ",\"owned_at_sample\":%s,\"sdk_ready_at_sample\":%s,\"submission_known\":%s"
            ",\"submitted_us\":%" PRIu32 ",\"last_hci_type\":%" PRIu32 ",\"last_hci_code\":%" PRIu32
            ",\"pending_acks_at_sample\":%" PRIu32 ",\"owned_at_gap\":%" PRIu32 ",\"owned_at_gap_valid\":%s,\"owned_read_independent\":true}\n",
            names[kind][4],s.stream_serial,s.at_us,s.vhci_coherent?"true":"false",
            s.vhci_coherent && (s.vhci_flags&PROBE_VHCI_SAMPLE)?"true":"false",s.vhci_at_us,
            (s.vhci_flags&PROBE_VHCI_HELD)?"true":"false",(s.vhci_flags&PROBE_VHCI_SDK_READY)?"true":"false",
            (s.vhci_flags&PROBE_VHCI_SUBMITTED)?"true":"false",s.vhci_submitted_us,s.vhci_packet>>16,s.vhci_packet&0xffffu,
            s.vhci_pending_acks,s.vhci_owned_now,s.vhci_owned_now!=UINT32_MAX?"true":"false");
#endif

    }
}
#endif
static void log_media_trace(void){
    for(unsigned i=0;i<2;++i)for(unsigned budget=0;budget<2;++budget){
        media_trace_t record;if(!probe_audio_trace_pop(&media_trace_rings[i],&record))break;
        if(record.codec==PROBE_AUDIO_TRACE_CONTROLLER){
            const probe_host_controller_t *c=&record.controller;
            PROBE_SNAPSHOT_LOG("{\"probe\":\"controller_identified\",\"profiles_ready\":false,\"hci_version\":%" PRIu32 ",\"manufacturer\":%" PRIu32
                ",\"host_bytes\":%" PRIu32 ",\"runtime_bytes\":%" PRIu32 ",\"classic_acl_bytes\":%" PRIu32 ",\"classic_acl_packets\":%" PRIu32
                ",\"le_acl_bytes\":%" PRIu32 ",\"le_acl_packets\":%" PRIu32 ",\"shared_acl_pool\":%s,\"internal_free\":%u,\"internal_largest\":%u"
                ",\"address\":\"%02X:%02X:%02X:%02X:%02X:%02X\",\"collection_context\":\"background_logger\",\"event_us\":%" PRIu32 ",\"host_log_ordinal\":%" PRIu32 "}\n",
                c->hci_version,c->manufacturer,c->host_bytes,c->runtime_bytes,c->classic_acl_bytes,c->classic_acl_packets,c->le_acl_bytes,c->le_acl_packets,c->shared_acl_pool?"true":"false",
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT),(unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT),
                c->address[0],c->address[1],c->address[2],c->address[3],c->address[4],c->address[5],record.at_us,record.ordinal);
            continue;
        }
        if(record.codec==PROBE_AUDIO_TRACE_FAILURE){
            const probe_host_failure_t *f=&record.failure;
            PROBE_SNAPSHOT_LOG("{\"probe\":\"failed\",\"fault\":%" PRIu32 ",\"controller_status\":%" PRIu32
                ",\"pending_opcode\":%" PRIu32 ",\"rx_rejected\":%" PRIu32 ",\"rx_flow_failed_opcode\":%" PRIu32
                ",\"event_us\":%" PRIu32 ",\"host_log_ordinal\":%" PRIu32 "}\n",
                f->fault,f->controller_status,f->pending_opcode,f->rx_rejected,f->rx_flow_failed_opcode,record.at_us,record.ordinal);
            PROBE_SNAPSHOT_LOG("{\"probe\":\"parser_failure\",\"at_ms\":%" PRIu32 ",\"origin\":%" PRIu32 ",\"result\":%" PRIu32
                ",\"handle\":%" PRIu32 ",\"generation\":%" PRIu32 ",\"link_present\":%" PRIu32 ",\"cid\":%" PRIu32 ",\"bytes\":%" PRIu32 ",\"declared\":%" PRIu32
                ",\"type\":%" PRIu32 ",\"event\":%" PRIu32 ",\"pb\":%" PRIu32 ",\"bc\":%" PRIu32 ",\"code\":%" PRIu32 ",\"identifier\":%" PRIu32
                ",\"host_stage\":%" PRIu32 ",\"host_packet_bytes\":%" PRIu32 ",\"host_type\":%" PRIu32 ",\"host_event\":%" PRIu32
                ",\"event_us\":%" PRIu32 ",\"host_log_ordinal\":%" PRIu32 "}\n",
                f->at_ms,f->origin,f->result,f->handle,f->generation,f->link_present,f->cid,f->bytes,f->declared,
                f->packet_type,f->event,f->pb,f->bc,f->code,f->identifier,f->host_stage,f->host_packet_bytes,f->host_type,f->host_event,record.at_us,record.ordinal);
            continue;
        }
        if(record.codec==PROBE_AUDIO_TRACE_CORRUPT){atomic_fetch_add_explicit(&snapshot_format_lost,1,memory_order_relaxed);continue;}
        if(record.codec==PROBE_AUDIO_TRACE_HOST){
            const probe_host_record_t *state=&record.host;
            if(state->kind==PROBE_HOST_CLASSIC_VISIBILITY){
                PROBE_SNAPSHOT_LOG("{\"probe\":\"classic_visibility\",\"completed_scan\":%" PRIu32 ",\"requested_scan\":%" PRIu32 ",\"command_pending\":%s,\"pairing_open\":%s,\"pairing_origin\":%" PRIu32 ",\"event_us\":%" PRIu32 ",\"host_log_ordinal\":%" PRIu32 "}\n",
                    state->value[0],state->value[1],state->value[2]?"true":"false",state->value[3]?"true":"false",state->value[4],record.at_us,record.ordinal);
            }else if(state->kind==PROBE_HOST_GAP_STATE){
                PROBE_SNAPSHOT_LOG("{\"probe\":\"gap_state\",\"state\":%" PRIu32 ",\"controller_status\":%" PRIu32 ",\"profiles_ready\":false,\"advertising\":%s,\"classic_scan\":%" PRIu32 ",\"event_us\":%" PRIu32 ",\"host_log_ordinal\":%" PRIu32 "}\n",
                    state->value[0],state->value[1],state->value[2]?"true":"false",state->value[3],record.at_us,record.ordinal);
            }else if(state->kind==PROBE_HOST_A2DP_DELAY){
                PROBE_SNAPSHOT_LOG("{\"probe\":\"avdtp_delay_estimate\",\"encoded_frames\":%" PRIu32 ",\"encoded_known\":%s,\"rate\":%" PRIu32
                    ",\"delay_100us\":%" PRIu32 ",\"result\":%" PRIu32 ",\"binding\":%" PRIu32 ",\"event_us\":%" PRIu32
                    ",\"adjacent_buffer_estimate\":true,\"host_log_ordinal\":%" PRIu32 "}\n",
                    state->value[0],(state->value[2]>>16)?"true":"false",state->value[1],state->value[2]&65535u,state->value[3],state->value[4],record.at_us,record.ordinal);
            }else if(state->kind==PROBE_HOST_A2DP_JOB){
                PROBE_SNAPSHOT_LOG("{\"probe\":\"a2dp_worker_submit\",\"binding\":%" PRIu32 ",\"token\":%" PRIu32 ",\"operation\":%" PRIu32 ",\"gate_epoch\":%" PRIu32 ",\"begin_us\":%" PRIu32 ",\"started_epoch_at_return\":%" PRIu32 ",\"accepted\":true,\"event_us\":%" PRIu32 ",\"host_log_ordinal\":%" PRIu32 "}\n",
                    state->value[0],state->value[1],state->value[2],state->value[3],state->value[4],state->value[5],record.at_us,record.ordinal);
            }else if(state->kind==PROBE_HOST_A2DP_STATE){
                PROBE_SNAPSHOT_LOG("{\"probe\":\"a2dp_lifecycle\",\"binding\":%" PRIu32 ",\"pending_token\":%" PRIu32 ",\"event\":%" PRIu32 ",\"state\":%" PRIu32 ",\"media_binding\":%" PRIu32 ",\"gate_epoch\":%" PRIu32 ",\"started_epoch\":%" PRIu32 ",\"event_us\":%" PRIu32 ",\"host_log_ordinal\":%" PRIu32 "}\n",
                    state->value[0],state->value[1],state->value[2],state->value[3],state->value[4],state->value[5],state->value[6],record.at_us,record.ordinal);
            }else if(state->kind==PROBE_HOST_A2DP_HANDOFF){
                PROBE_SNAPSHOT_LOG("{\"probe\":\"a2dp_stream_handoff\",\"binding\":%" PRIu32 ",\"token\":%" PRIu32 ",\"job_epoch\":%" PRIu32 ",\"phase\":%" PRIu32 ",\"start\":%s,\"renderer_ticket\":%" PRIu32 ",\"result\":%" PRIu32 ",\"event_us\":%" PRIu32 ",\"decoder_log_ordinal\":%" PRIu32 "}\n",
                    state->value[0],state->value[1],state->value[2],state->value[3],state->value[4]?"true":"false",state->value[5],state->value[6],record.at_us,record.ordinal);
            }else if(state->kind==PROBE_HOST_SMP){
                PROBE_SNAPSHOT_LOG("{\"probe\":\"smp_trace\",\"handle\":%" PRIu32 ",\"generation\":%" PRIu32 ",\"stage\":%" PRIu32 ",\"result\":%" PRId32 ",\"code\":%" PRIu32 ",\"size\":%" PRIu32 ",\"window\":%" PRIu32 ",\"event_us\":%" PRIu32 ",\"trace_records_lost\":%" PRIu32 ",\"host_log_ordinal\":%" PRIu32 "}\n",
                    state->value[0],state->value[1],state->value[2],(int32_t)state->value[3],state->value[4]&255u,state->value[4]>>8,state->value[5],record.at_us,state->value[6],record.ordinal);
            }else if(state->kind==PROBE_HOST_CCCD){
                PROBE_SNAPSHOT_LOG("{\"probe\":\"gatt_cccd\",\"handle\":%" PRIu32 ",\"generation\":%" PRIu32 ",\"token\":%" PRIu32 ",\"stage\":%" PRIu32 ",\"phase\":%" PRIu32 ",\"result\":%" PRIu32 ",\"job_kind\":%" PRIu32 ",\"event_us\":%" PRIu32 ",\"host_log_ordinal\":%" PRIu32 "}\n",
                    state->value[0],state->value[1],state->value[2],state->value[3],state->value[4],state->value[5],state->value[6],record.at_us,record.ordinal);
            }else if(state->kind==PROBE_HOST_CCCD_WORK){
                bool executed=(state->value[3]&PROBE_CCCD_WORK_EXECUTED)!=0;
                PROBE_SNAPSHOT_LOG("{\"probe\":\"gatt_cccd_work\",\"handle\":%" PRIu32 ",\"generation\":%" PRIu32 ",\"token\":%" PRIu32 ",\"job_kind\":%" PRIu32
                    ",\"executed\":%s,\"worker_core_valid\":%s,\"worker_core\":%" PRIu32 ",\"queued_us\":%" PRIu32 ",\"started_us\":%" PRIu32 ",\"finished_us\":%" PRIu32
                    ",\"queue_wait_us\":%" PRIu32 ",\"work_wall_us\":%" PRIu32 ",\"post_work_to_owner_us\":%" PRIu32 ",\"wall_time_includes_preemption\":true,\"event_us\":%" PRIu32 ",\"host_log_ordinal\":%" PRIu32 "}\n",
                    state->value[0],state->value[1],state->value[2],state->value[3]&255u,executed?"true":"false",executed?"true":"false",
                    (state->value[3]>>PROBE_CCCD_WORK_CORE_SHIFT)&1u,state->value[4],state->value[5],state->value[6],
                    executed?state->value[5]-state->value[4]:0u,executed?state->value[6]-state->value[5]:0u,
                    executed?record.at_us-state->value[6]:0u,record.at_us,record.ordinal);
            }else if(state->kind==PROBE_HOST_MEMORY_SERVICES){
                PROBE_SNAPSHOT_LOG("{\"probe\":\"boot_memory\",\"host\":\"owned\",\"phase\":\"services_ready\",\"internal_free\":%u,\"internal_largest\":%u,\"internal_minimum\":%u,\"internal_total\":%u,\"owner_stack_low\":%u"
                    ",\"collection_context\":\"background_logger\",\"event_us\":%" PRIu32 ",\"host_log_ordinal\":%" PRIu32 "}\n",
                    (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT),(unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT),
                    (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT),(unsigned)heap_caps_get_total_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT),
                    (unsigned)uxTaskGetStackHighWaterMark(owner),record.at_us,record.ordinal);
            }else atomic_fetch_add_explicit(&snapshot_format_lost,1,memory_order_relaxed);
            continue;
        }
        const dbt_codec_trace_t *e=&record.event;
        printf("{\"probe\":\"media_trace\",\"at_us\":%" PRIu32 ",\"stage\":\"%s\",\"ordinal\":%" PRIu32
               ",\"sequence\":%u,\"expected\":%u,\"rtp_timestamp\":%" PRIu32 ",\"ssrc\":%" PRIu32
               ",\"previous_ssrc\":%" PRIu32 ",\"bytes\":%u,\"queued\":%u,\"result\":%u,\"codec\":%" PRIu32 ",\"codec_name\":\"%s\",\"rtp_marker\":%s,\"payload_prefix\":[%u,%u],\"framing\":\"%s\",\"sequence_origin\":\"%s\",\"timestamp_origin\":\"%s\"}\n",
               record.at_us,e->kind==DBT_CODEC_INPUT_CLOCK_MAX?"input_clock_max":(unsigned)e->kind<sizeof(trace_names)/sizeof(trace_names[0])?trace_names[e->kind]:"invalid",record.ordinal,e->sequence,e->expected_sequence,e->timestamp,
               e->ssrc,e->previous_ssrc,e->bytes,e->queued,(unsigned)e->result,record.codec,codec_name(record.codec),e->marker?"true":"false",e->prefix[0],e->prefix[1],
               e->payload_type==DBT_AVDTP_RAW_PAYLOAD_TYPE?"raw":"rtp",
               e->payload_type==DBT_AVDTP_RAW_PAYLOAD_TYPE?"host_ordinal":"wire_rtp",
               e->payload_type==DBT_AVDTP_RAW_PAYLOAD_TYPE?"host_samples":"wire_rtp");
    }
}
#ifdef DBT_PROBE_LHDC_BUILD
static void log_lhdc_quality(uint32_t cycle){
    dbt_lhdcv5_quality_t q;if(!dbt_lhdcv5_quality_snapshot(&q) || !q.attempts)return;
    PROBE_SNAPSHOT_LOG("{\"probe\":\"lhdc_decode_result\",\"collection_cycle_us\":%" PRIu32 ",\"revision\":%" PRIu32
        ",\"scope\":\"decoder_owner_after_cold_checks\",\"attempts\":%" PRIu32 ",\"rejected\":%" PRIu32
        ",\"last_event_valid\":%s,\"last_event_attempt\":%" PRIu32 ",\"last_event_generation\":%" PRIu32
        ",\"last_event_frame_index\":%" PRIu32 ",\"last_event_sequence\":%" PRIu32 ",\"last_event_rtp_timestamp\":%" PRIu32
        ",\"last_event_ssrc\":%" PRIu32 ",\"last_event_frame_bytes\":%" PRIu32 ",\"last_event_result\":%" PRId32 ",\"last_event_flags\":%" PRIu32 "}\n",
        cycle,q.revision,q.attempts,q.rejected,q.flags?"true":"false",q.event_attempt,q.generation,q.frame_index,q.sequence,q.rtp_timestamp,q.ssrc,q.frame_bytes,q.result,q.flags);
    PROBE_SNAPSHOT_LOG("{\"probe\":\"lhdc_channel_recovery\",\"collection_cycle_us\":%" PRIu32 ",\"revision\":%" PRIu32
        ",\"channel0_errors\":%" PRIu32 ",\"channel1_silenced\":%" PRIu32 ",\"last_event_attempt\":%" PRIu32
        ",\"last_event_channel0_result\":%" PRId32 ",\"last_event_channel1_result\":%" PRId32 ",\"last_event_flags\":%" PRIu32 "}\n",
        cycle,q.revision,q.channel0_errors,q.channel1_silenced,q.event_attempt,q.channel0_result,q.channel1_result,q.flags);
    PROBE_SNAPSHOT_LOG("{\"probe\":\"lhdc_frame_quality\",\"collection_cycle_us\":%" PRIu32 ",\"revision\":%" PRIu32
        ",\"peak_concealed\":%" PRIu32 ",\"nonfinite_frames\":%" PRIu32 ",\"clipped_frames\":%" PRIu32 ",\"clipped_samples\":%" PRIu32
        ",\"last_event_attempt\":%" PRIu32 ",\"last_event_peak_f32_bits\":\"%08" PRIx32 "\",\"last_event_flags\":%" PRIu32 "}\n",
        cycle,q.revision,q.peak_concealed,q.nonfinite_frames,q.clipped_frames,q.clipped_samples,q.event_attempt,q.peak_f32_bits,q.flags);
}
#endif

#ifdef DBT_PROBE_LHDC_STAGE_PROFILE
static void log_lhdc_live(uint32_t cycle){
    probe_lhdc_live_t s;if(!probe_lhdc_profile_snapshot(&s) || !s.channel_calls)return;
    PROBE_SNAPSHOT_LOG("{\"probe\":\"lhdc_live_stages\",\"collection_cycle_us\":%" PRIu32 ",\"revision\":%" PRIu32 ",\"generation\":%" PRIu32 ",\"rate\":%" PRIu32 ",\"pcm_container_bytes\":%" PRIu32 ",\"collecting\":%" PRIu32 ",\"channel_calls\":%" PRIu32
        ",\"sum_cycles\":[%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 "],\"max_cycles\":[%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 "]"
        ",\"frame_calls\":%" PRIu32 ",\"frame_sum_cycles\":[%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 "],\"frame_max_cycles\":[%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 "]"
        ",\"stages\":[\"preamble\",\"entropy\",\"mantissa\",\"dequant_sns\",\"imdct\",\"overlap\"],\"frame_stages\":[\"prepare\",\"channel_envelope\",\"scale_scan\",\"output_tail\"],\"cycles_wrap_uint32\":true,\"cycles_include_preemption\":true,\"instrumented\":true,\"publication_period_frames\":16}\n",
        cycle,s.revision,s.generation,s.rate,s.sample_bytes,s.collecting,s.channel_calls,
        s.sum_cycles[0],s.sum_cycles[1],s.sum_cycles[2],s.sum_cycles[3],s.sum_cycles[4],s.sum_cycles[5],
        s.max_cycles[0],s.max_cycles[1],s.max_cycles[2],s.max_cycles[3],s.max_cycles[4],s.max_cycles[5],s.frame_calls,
        s.frame_sum_cycles[0],s.frame_sum_cycles[1],s.frame_sum_cycles[2],s.frame_sum_cycles[3],
        s.frame_max_cycles[0],s.frame_max_cycles[1],s.frame_max_cycles[2],s.frame_max_cycles[3]);
}
#endif
void probe_audio_transport_received(uint32_t began_us,uint16_t bytes,uint8_t type,dbt_result_t result,bool isr){
    atomic_fetch_add(&rx_packets,1);atomic_fetch_add(&rx_bytes,bytes);
    if(type==DBT_H4_ACL){
        atomic_fetch_add(&rx_acl,1);
#if defined(PROBE_OUTPUT_GAP_TRACE) && defined(DBT_PROBE_AUDIO_OUTPUT)
        probe_output_trace_acl(began_us);
#endif
    }
    if(type==DBT_H4_EVENT)atomic_fetch_add(&rx_events,1);
    if(isr)atomic_fetch_add(&rx_isr,1);
    if(result!=DBT_OK)atomic_fetch_add(&rx_rejected,1);
    uint32_t empty=0;uint32_t stamp=began_us?began_us:1u;
    atomic_compare_exchange_strong_explicit(&rx_pending_us,&empty,stamp,memory_order_release,memory_order_relaxed);
    maximum(&rx_callback_max_us,(uint32_t)esp_timer_get_time()-began_us);
}
void probe_audio_transport_poll_begin(uint32_t began_us){
    (void)began_us;
#if defined(PROBE_OUTPUT_GAP_TRACE) && defined(DBT_PROBE_AUDIO_OUTPUT)
    probe_output_trace_host_poll(began_us);
#endif
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
void probe_audio_owner_service_yield(void){atomic_fetch_add_explicit(&owner_service_yields,1,memory_order_relaxed);}
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
    {sbc_capabilities,sizeof(sbc_capabilities),1,dbt_a2dp_sbc_validate,0,0},
#ifdef DBT_PROBE_OPUS_BUILD
    {android_capabilities,sizeof(android_capabilities),2,dbt_a2dp_opus_validate,0,0},
    {opus05_capabilities,sizeof(opus05_capabilities),3,dbt_a2dp_opus_validate,0,0},
#endif
#ifdef DBT_PROBE_AAC_BUILD
    {aac_capabilities,sizeof(aac_capabilities),4,dbt_a2dp_aac_validate,0,0},
#endif
#ifdef DBT_PROBE_APTX_BUILD
    {aptx_capabilities,sizeof(aptx_capabilities),5,dbt_a2dp_aptx_validate,4,4},
    {aptx_hd_capabilities,sizeof(aptx_hd_capabilities),6,dbt_a2dp_aptx_validate,0,0},
    {aptx_ll_capabilities,sizeof(aptx_ll_capabilities),7,dbt_a2dp_aptx_validate,4,4},
#endif
#ifdef DBT_PROBE_LDAC_BUILD
    {ldac_capabilities,sizeof(ldac_capabilities),8,dbt_a2dp_ldac_validate,0,0},
#endif
#ifdef DBT_PROBE_LHDC_BUILD
    {lhdc_capabilities,sizeof(lhdc_capabilities),9,dbt_a2dp_lhdcv5_validate,0,0},
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
#ifdef DBT_PROBE_SCHEDULER_TRACE
    pcm_level_observe(samples,count);
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
#ifdef DBT_PROBE_SCHEDULER_TRACE
        pcm_level_begin(codec,config->rate,config->channels,config->sample_bytes);
#endif
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
#ifdef DBT_PROBE_SCHEDULER_TRACE
    probe_scheduler_owner_capture(started);
#endif
    return DBT_OK;
}
#ifdef DBT_PROBE_ASYNC_AUDIO
static bool async_stream_start;
/* Decoder owner alone writes ring1. The current immutable control job pins its
 * token/epoch until the hook returns; never inspect another owner's live job. */
static void stream_handoff_trace(unsigned phase,bool start,uint32_t ticket,dbt_result_t result){
    if(!initialized || !decoder_task || xTaskGetCurrentTaskHandle()!=decoder_task)return;
    uint32_t read=atomic_load_explicit(&worker.job_read,memory_order_relaxed);
    if(read==atomic_load_explicit(&worker.job_written,memory_order_acquire))return;
    const dbt_codec_job_t *job=&worker.jobs[read%DBT_CODEC_WORKER_JOBS];
    probe_host_record_t record={.kind=PROBE_HOST_A2DP_HANDOFF,.value={job->binding,job->token,job->epoch,
        phase,start?1u:0u,ticket,(uint32_t)result}};
    (void)probe_audio_trace_host(&media_trace_rings[1],&record,trace_clock,NULL);
}
static dbt_result_t stream_begin(void *context,bool start,const dbt_codec_config_t *config,uint32_t *ticket){
    (void)context;
    if(!config || !config->rate)return DBT_INVALID;
    uint16_t priming=0;
#ifdef DBT_PROBE_AAC_BUILD
    if(config->ops==&dbt_aac_codec){
        uint32_t two_units=(2u*config->samples*1000u+config->rate-1u)/config->rate;
        priming=(uint16_t)(two_units>40u?two_units:40u);
    }
#endif
#ifdef DBT_PROBE_LDAC_BUILD
    if(config->ops==&dbt_ldac_codec)priming=40;
#endif
#ifdef DBT_PROBE_LHDC_BUILD
    /* High-rate startup trace showed about48ms of host/RTP cadence deficit,
     * exhausting the prior50ms reserve before decode/refill completed. Trial
     * an80ms event/deadline hold using the existing queues; no added buffers. */
    if(config->ops==&dbt_lhdcv5_codec)priming=config->rate>96000u?80u:40u;
#endif
#ifdef DBT_PROBE_CODEC_AWARE_RENDER
    bool share=config->ops==&dbt_sbc_codec;
#ifdef DBT_PROBE_AAC_BUILD
    share=share || config->ops==&dbt_aac_codec;
#endif
#ifdef DBT_PROBE_APTX_BUILD
    share=share || config->ops==&dbt_aptx_codec;
#endif
#ifdef DBT_PROBE_LDAC_BUILD
    share=share || config->ops==&dbt_ldac_codec;
#endif
#if defined(DBT_PROBE_LHDC_CORE0_RENDER) && defined(DBT_PROBE_LHDC_BUILD)
    /* Opt-in measurement: reuse the same closed-channel handoff and the
     * decoder-aware notification/backpressure policy, with no new task type. */
    share=share || config->ops==&dbt_lhdcv5_codec;
#endif
    dbt_result_t result=probe_output_stream_begin_primed(start,config->rate,config->channels,
        (dbt_pcm_format_t)config->sample_bytes,share,priming,ticket);
#else
    dbt_result_t result=probe_output_stream_begin_primed(start,config->rate,config->channels,(dbt_pcm_format_t)config->sample_bytes,false,priming,ticket);
#endif
    if(result==DBT_PENDING)async_stream_start=start;
    if(result!=DBT_BUSY)stream_handoff_trace(0u,start,ticket?*ticket:0u,result);
    return result;
}
static dbt_result_t stream_poll(void *context,uint32_t ticket){
    (void)context;dbt_result_t result=probe_output_request_poll(ticket);
    if(result!=DBT_PENDING && result!=DBT_BUSY)stream_handoff_trace(1u,async_stream_start,ticket,result);
    if(result==DBT_OK)return publish_stream(async_stream_start,&worker.decoder.config);
    return result;
}
static dbt_result_t stream_cancel(void *context,uint32_t ticket){
    (void)context;dbt_result_t result=probe_output_request_cancel(ticket);
    if(result!=DBT_BUSY)stream_handoff_trace(2u,async_stream_start,ticket,result);
    return result;
}
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
bool probe_audio_owner_pressure(void){
#if defined(DBT_PROBE_LHDC_BUILD) && defined(DBT_PROBE_AUDIO_OUTPUT)
    return atomic_load_explicit(&pcm_codec,memory_order_acquire)==9u && probe_output_owner_needs_time();
#else
    return false;
#endif
}
uint32_t probe_audio_startup_encoded_slots(void){
    uint32_t read=atomic_load_explicit(&worker.media_read,memory_order_acquire);
    uint32_t written=atomic_load_explicit(&worker.media_written,memory_order_acquire);
    uint32_t after=atomic_load_explicit(&worker.media_read,memory_order_acquire);
    uint32_t layout=atomic_load_explicit(&worker.queue_layout,memory_order_acquire);
    uint32_t depth=layout&DBT_CODEC_QUEUE_DEPTH_MASK;
    uint32_t capacity=(layout&DBT_CODEC_QUEUE_PACKED)?depth*((layout>>16)/24u):depth;
    return read==after && depth && written-read<=capacity?written-read:UINT32_MAX;
}
probe_dsp_admission_t probe_audio_dsp_assist_admission(bool has_decode_reserve){
#ifdef DBT_PROBE_LHDC_BUILD
    if(atomic_load_explicit(&pcm_codec,memory_order_relaxed)!=9u)return PROBE_DSP_NO_ASSIST;
    unsigned reason=atomic_load_explicit(&decoder_wait_kind,memory_order_acquire);
    if(reason==0u || reason==3u)return PROBE_DSP_IDLE_ASSIST;
    return has_decode_reserve && reason==DECODER_COMPUTING?
        PROBE_DSP_RESERVED_ASSIST:PROBE_DSP_NO_ASSIST;
#else
    (void)has_decode_reserve;return PROBE_DSP_NO_ASSIST;
#endif
}

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
        uint16_t budget=2;
#if defined(DBT_PROBE_AUDIO_OUTPUT) && !defined(DBT_PROBE_RF_PROTOCOL_DIAGNOSTIC)
        budget=probe_output_decode_budget();
#endif
        (void)dbt_codec_worker_step(&worker,now_ms(),budget);
        uint32_t elapsed=(uint32_t)esp_timer_get_time()-began;
        atomic_fetch_add(&worker_steps,1);atomic_fetch_add(&worker_elapsed_us,elapsed);maximum(&worker_step_max_us,elapsed);
        unsigned bucket=0;uint32_t bound=125;
        while(bucket<7 && elapsed>bound){++bucket;bound*=2;}
        atomic_fetch_add(&worker_hist[bucket],1);
        uint32_t faults=atomic_load_explicit(&worker.faults,memory_order_acquire);
        if(faults!=seen_faults){seen_faults=faults;wake_owner(0);}
        bool render_handoff=false;
#if defined(DBT_PROBE_AUDIO_OUTPUT) && !defined(DBT_PROBE_RF_PROTOCOL_DIAGNOSTIC)
        render_handoff=budget==0 && probe_output_decode_budget()==0;
#endif
        if(dbt_codec_worker_runnable(&worker) && !render_handoff){taskYIELD();continue;}
        TickType_t wait=decoder_wait_ticks(now_ms());
        /* Notifications queued before this call remain pending. No fixed
         * media polling period and no wait while useful work is runnable. */
        if(!wait){(void)ulTaskNotifyTake(pdTRUE,0);continue;}
        unsigned reason=render_handoff?4u:worker.pcm_ticket?1u:worker.pending_pcm&&worker.output_blocked?0u:
            worker.control_phase||atomic_load(&worker.job_read)!=atomic_load(&worker.job_written)?2u:3u;
        uint32_t before=(uint32_t)esp_timer_get_time();
        atomic_store_explicit(&decoder_wait_began,before,memory_order_relaxed);
        atomic_store_explicit(&decoder_wait_kind,reason,memory_order_release);
#if defined(PROBE_OUTPUT_GAP_TRACE) && defined(DBT_PROBE_AUDIO_OUTPUT)
        decoder_timeline(4u+reason,before);
#endif
        (void)ulTaskNotifyTake(pdTRUE,wait);
#if defined(PROBE_OUTPUT_GAP_TRACE) && defined(DBT_PROBE_AUDIO_OUTPUT)
        decoder_timeline(0,(uint32_t)esp_timer_get_time());
#endif
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
    static uint32_t previous_playback_generation,previous_playback_us;
    static uint32_t previous_playback_state=UINT32_MAX;
    if(probe_avrcp_snapshot(&metadata)!=DBT_OK)return false;
    uint32_t state=(uint32_t)metadata.playback_status|((uint32_t)metadata.connected<<8);
    if(previous_playback_state!=state || previous_playback_generation!=metadata.generation ||
       previous_playback_us!=metadata.playback_status_changed_us){
        PROBE_SNAPSHOT_LOG("{\"probe\":\"avrcp_playback\",\"generation\":%" PRIu32 ",\"connected\":%s,\"status\":%u,\"status_changed_us\":%" PRIu32
            ",\"clock\":\"esp_timer_boot_epoch_us32\",\"timestamp_is_peer_status_callback_observation\":true}\n",
            metadata.generation,metadata.connected?"true":"false",(unsigned)metadata.playback_status,metadata.playback_status_changed_us);
        previous_playback_state=state;previous_playback_generation=metadata.generation;
        previous_playback_us=metadata.playback_status_changed_us;
    }
    if(metadata.metadata_updates==previous_metadata)return false;
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
#ifdef DBT_PROBE_SCHEDULER_TRACE
static void log_render_delays(void){
    static uint32_t seen[PROBE_DELAY_KINDS];
    for(unsigned kind=0;kind<PROBE_DELAY_KINDS;++kind){
        probe_scheduler_interval_t span;probe_scheduler_interval_snapshot(kind,&span);
        if(!span.coherent || span.revision==seen[kind])continue;
        seen[kind]=span.revision;
        PROBE_SNAPSHOT_LOG("{\"probe\":\"%s\",\"core\":%u,\"scope\":\"stream_max_wall\",\"stream_serial\":%" PRIu32 ",\"rate\":%" PRIu32 ",\"begin_us\":%" PRIu32
            ",\"end_us\":%" PRIu32 ",\"wall_us\":%" PRIu32 ",\"frames\":%" PRIu32
            ",\"begin_tick_us\":%" PRIu32 ",\"end_tick_us\":%" PRIu32 ",\"markers_valid\":%s,\"sample_hz\":%u"
            ",\"ticks_not_cpu_time\":true,\"known_task_mask\":%" PRIu32
            ",\"begin_marker_cycles\":%" PRIu32 ",\"end_marker_cycles\":%" PRIu32
            ",\"host\":%" PRIu32 ",\"decoder\":%" PRIu32 ",\"renderer\":%" PRIu32 ",\"logger\":%" PRIu32
            ",\"controller\":%" PRIu32 ",\"idle\":%" PRIu32 ",\"other\":%" PRIu32 ",\"ipc\":%" PRIu32 ",\"timer\":%" PRIu32
            ",\"host_other\":%" PRIu32 ",\"host_query\":%" PRIu32 ",\"host_submit\":%" PRIu32
            ",\"host_runtime\":%" PRIu32 ",\"host_wait\":%" PRIu32 ",\"host_gatt\":%" PRIu32
            ",\"host_security\":%" PRIu32 ",\"host_product\":%" PRIu32 ",\"host_io\":%" PRIu32
            ",\"host_audio\":%" PRIu32 ",\"host_avrcp\":%" PRIu32 ",\"host_console\":%" PRIu32 "}\n",
            kind==PROBE_DELAY_LOCAL_RENDER?"render_local_delay":kind==PROBE_DELAY_LOCAL_RENDER0?"render_local_delay_core0":"render_retirement_delay",
            kind==PROBE_DELAY_LOCAL_RENDER0?0u:1u,
            span.stream_serial,span.rate,span.begin_us,span.end_us,span.wall_us,span.frames,span.begin_tick_us,span.end_tick_us,
            span.markers_valid?"true":"false",(unsigned)configTICK_RATE_HZ,span.known_task_mask,
            span.begin_read_cycles,span.end_read_cycles,
            span.counts[0],span.counts[1],span.counts[2],span.counts[3],span.counts[4],span.counts[5],span.counts[6],span.counts[7],span.counts[8],
            span.host_phases[0],span.host_phases[1],span.host_phases[2],span.host_phases[3],span.host_phases[4],span.host_phases[5],
            span.host_phases[6],span.host_phases[7],span.host_phases[8],span.host_phases[9],span.host_phases[10],span.host_phases[11]);
    }
}
#endif
#ifdef DBT_PROBE_SCHEDULER_TRACE
static void log_pcm_level(void){
    static uint32_t reported;
    uint32_t seq=atomic_load_explicit(&level_sequence,memory_order_acquire);
    if(seq&1u)return;
    uint32_t epoch=atomic_load_explicit(&level_epoch,memory_order_acquire);if(!epoch || epoch==reported)return;
    uint32_t codec=atomic_load(&level_codec),rate=atomic_load(&level_rate),channels=atomic_load(&level_channels),width=atomic_load(&level_width);
    uint32_t frames=atomic_load(&level_frames),count=atomic_load(&level_count),peak=atomic_load(&level_peak),lo=atomic_load(&level_lo),hi=atomic_load(&level_hi);
    atomic_thread_fence(memory_order_seq_cst);
    if(seq!=atomic_load_explicit(&level_sequence,memory_order_acquire) || epoch!=atomic_load_explicit(&level_epoch,memory_order_acquire))return;
    PROBE_SNAPSHOT_LOG("{\"probe\":\"decoder_pcm_level\",\"epoch\":%" PRIu32
        ",\"codec\":%" PRIu32
        ",\"rate\":%" PRIu32
        ",\"channels\":%" PRIu32
        ",\"pcm_container_bytes\":%" PRIu32
        ",\"frames\":%" PRIu32
        ",\"sampled_channels\":%" PRIu32
        ",\"peak_q15\":%" PRIu32
        ",\"squares_lo\":%" PRIu32
        ",\"squares_hi\":%" PRIu32
        ",\"stride_frames\":1,\"skip_seconds\":1,\"window_seconds\":4,\"domain\":\"decoder_before_dsp_top16\",\"decimated\":false,\"coherent\":true}\n",
        epoch,codec,rate,channels,width,frames,count,peak,lo,hi);reported=epoch;
}
static void log_owner_costs(void){
    static uint32_t reported_epoch;
    static const char *const names[PROBE_COST_SLOTS]={
        "runtime_wait","parameters_wait","admission_wait","smp_wait","avrcp_wait",
        "button_wait","rfcomm_wait","gap_wait","product_wait","io_wait","att_wait",
        "security_wait","echo_wait","sdp_wait","audio_wait","clock_floor","wait_total",
        "poll_other","controller_query","controller_submit","runtime_poll","poll_wait",
        "gatt_poll","security_poll","product_poll","io_poll","audio_poll","avrcp_poll","console_poll"};
    uint32_t seen=0,epoch=probe_scheduler_owner_cost_finished_epoch(&seen);bool incomplete=false;
    if(!epoch || epoch==reported_epoch)return;
    for(unsigned slot=0;slot<PROBE_COST_SLOTS;++slot){
        if(!(seen&(1u<<slot)))continue;
        probe_owner_cost_t s;
        if(!probe_scheduler_owner_cost_snapshot(slot,&s) || s.epoch!=epoch){incomplete=true;continue;}
        PROBE_SNAPSHOT_LOG("{\"probe\":\"owner_cost_%u\",\"slot\":%u,\"name\":\"%s\",\"epoch\":%" PRIu32
            ",\"calls\":%" PRIu32 ",\"cycles\":%" PRIu32 ",\"min_cycles\":%" PRIu32 ",\"max_cycles\":%" PRIu32
            ",\"cpu_mhz\":240,\"coherent\":true,\"cycles_include_preemption\":true,\"irq_time_included\":true"
            ",\"nested_categories_inclusive\":true,\"counters_wrap_u32\":true,\"reported_after_pause\":true}\n",
            slot,slot,names[slot],s.epoch,s.calls,s.cycles,s.min_cycles,s.max_cycles);
    }
    if(!incomplete && probe_scheduler_owner_cost_finished_epoch(NULL)==epoch)reported_epoch=epoch;
}
#endif
#ifdef DBT_PROBE_TASK_RUNTIME_STATS
static void log_task_runtime(void){
    static const char *const roles[PROBE_RUNTIME_TASKS]={"host","decoder","renderer1","logger","controller","idle0","idle1","ipc0","ipc1","timer","renderer0"};
    probe_task_runtime_snapshot_t s;probe_scheduler_task_runtime(&s);
    for(unsigned i=0;i<PROBE_RUNTIME_TASKS;++i){
        if(!(s.present_mask&(1u<<i)))continue;
        const probe_task_runtime_t *t=&s.tasks[i];
        PROBE_SNAPSHOT_LOG("{\"probe\":\"task_runtime\",\"role\":\"%s\",\"core\":%" PRId32
            ",\"priority\":%" PRIu32 ",\"task_number\":%" PRIu32 ",\"runtime_us\":%" PRIu32
            ",\"begin_us\":%" PRIu32 ",\"end_us\":%" PRIu32 ",\"read_span_us\":%" PRIu32 ",\"max_query_us\":%" PRIu32
            ",\"present_mask\":%" PRIu32 ",\"clock\":\"esp_timer_u32\",\"clock_wrap_during_read\":%s,\"stack_scanned\":false,\"query_wall_includes_preemption\":true,\"interrupt_time_included\":true,\"running_tail_excluded\":true}\n",
            roles[i],t->core,t->priority,t->task_number,t->counter_us,s.begin_us,s.end_us,s.end_us-s.begin_us,s.max_query_us,
            s.present_mask,s.end_us<s.begin_us?"true":"false");
    }
}
#endif
#ifdef DBT_PROBE_TASK_CYCLE_ACCOUNTING
static void log_render_accounting(void){
    uint32_t faults[2];probe_task_cycles_faults(faults);
    for(unsigned core=0;core<2u;++core){
        probe_render_account_t s;
        probe_scheduler_render_account(core?PROBE_DELAY_LOCAL_RENDER:PROBE_DELAY_LOCAL_RENDER0,&s);
        if(!s.calls && !s.invalid && !faults[core])continue;
        PROBE_SNAPSHOT_LOG("{\"probe\":\"render_task_cycles\",\"core\":%u,\"stream_serial\":%" PRIu32
            ",\"rate\":%" PRIu32 ",\"coherent\":%s,\"calls\":%" PRIu32 ",\"invalid\":%" PRIu32 ",\"frames\":%" PRIu32
            ",\"wall_cycles\":%" PRIu32 ",\"charged_cycles\":%" PRIu32
            ",\"max_wall_cycles\":%" PRIu32 ",\"charged_at_max_wall\":%" PRIu32 ",\"max_charged_cycles\":%" PRIu32
            ",\"hook_faults\":%" PRIu32 ",\"cpu_mhz\":240,\"scope\":\"local_audio_render_markers\",\"irq_time_included\":true"
            ",\"running_tail_included\":true,\"other_tasks_and_selection_gap_excluded\":true,\"counters_wrap_u32\":true}\n",
            core,s.stream_serial,s.rate,s.coherent?"true":"false",s.calls,s.invalid,s.frames,s.wall_cycles,s.charged_cycles,
            s.max_wall_cycles,s.charged_at_max_wall,s.max_charged_cycles,faults[core]);
    }
}
static void log_host_accounting(void){
    probe_host_account_t s;probe_scheduler_host_account(&s);
    if(!s.revision && !s.wrong_context)return;
    uint32_t faults[2];probe_task_cycles_faults(faults);
    PROBE_SNAPSHOT_LOG("{\"probe\":\"host_task_cycles\",\"core\":1,\"coherent\":%s,\"invalid\":%" PRIu32
        ",\"wrong_context\":%" PRIu32 ",\"hook_faults\":%" PRIu32
        ",\"wrong_pc\":%" PRIu32 ",\"wrong_phase\":%" PRIu32
        ",\"charged_cycles\":[%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 "]"
        ",\"intervals\":[%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 "]"
        ",\"order\":\"other,query,submit,runtime,wait,gatt,security,product,io,audio,avrcp,console\",\"cpu_mhz\":240"
        ",\"scope\":\"up_to_last_host_phase_transition\",\"irq_time_included\":true,\"switched_away_excluded\":true,\"counters_wrap_u32\":true}\n",
        s.coherent?"true":"false",s.invalid,s.wrong_context,faults[1],s.wrong_pc,s.wrong_phase,
        s.charged_cycles[0],s.charged_cycles[1],s.charged_cycles[2],s.charged_cycles[3],s.charged_cycles[4],s.charged_cycles[5],
        s.charged_cycles[6],s.charged_cycles[7],s.charged_cycles[8],s.charged_cycles[9],s.charged_cycles[10],s.charged_cycles[11],
        s.intervals[0],s.intervals[1],s.intervals[2],s.intervals[3],s.intervals[4],s.intervals[5],
        s.intervals[6],s.intervals[7],s.intervals[8],s.intervals[9],s.intervals[10],s.intervals[11]);
}
#endif
#ifdef DBT_PROBE_OWNER_PACER
static void log_owner_pacer(void){
    static uint32_t seen=UINT32_MAX,seen_failed=UINT32_MAX;
    probe_owner_pacer_stats_t s;probe_owner_pacer_snapshot(&s);
    if(seen==s.pauses && seen_failed==s.failed)return;
    seen=s.pauses;seen_failed=s.failed;
    PROBE_SNAPSHOT_LOG("{\"probe\":\"owner_budget\",\"enabled\":%" PRIu32 ",\"waiting\":%" PRIu32 ",\"failed\":%" PRIu32
        ",\"slice_us\":%u,\"release_us\":%u,\"pauses\":%" PRIu32 ",\"releases\":%" PRIu32
        ",\"early_wait_checks\":%" PRIu32 ",\"max_slice_wall_us\":%" PRIu32 ",\"max_pause_wall_us\":%" PRIu32
        ",\"arm_failures\":%" PRIu32 ",\"counter_scope\":\"since_boot\",\"independent_counters\":true,\"timer_is_on_demand\":true}\n",
        s.enabled,s.waiting,s.failed,(unsigned)PROBE_OWNER_SLICE_US,(unsigned)PROBE_OWNER_RELEASE_US,
        s.pauses,s.releases,s.wait_checks,s.max_slice_us,s.max_pause_us,s.arm_failures);
}
#endif
#if defined(DBT_PROBE_VOLUME_RAMP_COMPARE) || defined(DBT_PROBE_MATCHED_FEEDBACK_COMPARE)
#ifdef DBT_PROBE_MATCHED_FEEDBACK_COMPARE
#define DSP_KERNEL_PROBE "dsp_matched_feedback_kernel"
#define DSP_KERNEL_REFERENCE "five_products"
#define DSP_KERNEL_CANDIDATE "same_sign_four_products"
#define DSP_KERNEL_SCOPE "full_steady_blocks_latest_policy"
#else
#define DSP_KERNEL_PROBE "dsp_volume_kernel"
#define DSP_KERNEL_REFERENCE "canonical_prefix"
#define DSP_KERNEL_CANDIDATE "stable_other_filters"
#define DSP_KERNEL_SCOPE "full_volume_only_ramps_latest_policy"
#endif
static void log_volume_kernels(void){
    static uint32_t seen[2][2];
    if(probe_output_running())return; /* No extra UART load during timing. */
    for(unsigned core=0;core<2;++core)for(unsigned mode=0;mode<2;++mode){
        fixed_dsp_volume_stats_t s;fixed_dsp_volume_stats(core,mode,&s);
        if(!s.coherent || s.revision==seen[core][mode])continue;
        seen[core][mode]=s.revision;
        PROBE_SNAPSHOT_LOG("{\"probe\":\"%s\",\"core\":%u,\"mode\":\"%s\",\"order\":\"ABBA\",\"rate\":%" PRIu32
            ",\"gains_db\":[%d,%d,%d],\"flags\":%u,\"timed_calls\":%" PRIu32 ",\"frames_per_call\":128,\"cycles\":%" PRIu32
            ",\"minimum_cycles\":%" PRIu32 ",\"maximum_cycles\":%" PRIu32 ",\"clipped_calls_untimed\":%" PRIu32
            ",\"cycles_include_preemption\":true,\"counts_wrap_uint32\":true,\"scope\":\"%s\"}\n",
            DSP_KERNEL_PROBE,core,mode?DSP_KERNEL_CANDIDATE:DSP_KERNEL_REFERENCE,s.rate,(int)(int8_t)s.policy,(int)(int8_t)(s.policy>>8),
            (int)(int8_t)(s.policy>>16),(unsigned)(s.policy>>24),s.calls,s.cycles,s.minimum,s.maximum,s.clipped,DSP_KERNEL_SCOPE);
    }
}
#endif
#ifdef DBT_PROBE_DISPATCH_TRACE
static void log_dispatch_trace(void){
    static uint32_t previous[2];
    for(unsigned core=0;core<2;++core){
        probe_dispatch_snapshot_t s;if(!probe_dispatch_snapshot(core,&s) || previous[core]==s.serial)continue;
        PROBE_SNAPSHOT_LOG("{\"probe\":\"dispatch_window\",\"stream_serial\":%" PRIu32 ",\"core\":%u,\"begin_us\":%" PRIu32 ",\"gap_us\":%" PRIu32 ",\"head\":%" PRIu32 ",\"begin_head\":%" PRIu32 ",\"count\":%" PRIu32 ",\"bindings\":%" PRIu32 ",\"prefix_overwritten\":%s,\"task_switch_is_selection\":true,\"interrupt_time_included\":true,\"one_attempt\":true}\n",
            s.serial,core,s.begin_us,s.gap_us,s.head,s.begin_head,s.count,s.bindings,s.head-s.begin_head>s.count?"true":"false");
        for(unsigned i=0;i<s.count;++i){probe_dispatch_event_t *e=&s.events[i];
            PROBE_SNAPSHOT_LOG("{\"probe\":\"dispatch_event\",\"stream_serial\":%" PRIu32 ",\"core\":%u,\"ordinal\":%" PRIu32 ",\"event_us\":%" PRIu32 ",\"kind\":%u,\"task_role\":%u,\"value\":%" PRIu32 ",\"after_gap\":%s}\n",
                s.serial,core,s.head-s.count+i,e->at_us,(unsigned)(e->info&15u),(unsigned)((e->info>>4)&15u),e->info>>8,(int32_t)(e->at_us-s.gap_us)>0?"true":"false");
        }
        previous[core]=s.serial;
    }
}
#endif
#ifdef DBT_PROBE_HEAP_ALLOC_TRACE
static void log_heap_alloc_capture(void){
    static bool printed;
    probe_heap_alloc_poll();
    if(printed || probe_output_running())return;
    probe_heap_capture_t s;if(!probe_heap_alloc_snapshot(&s))return;
    printed=true;
    PROBE_SNAPSHOT_LOG("{\"probe\":\"heap_alloc_capture\",\"handle\":%" PRIu32 ",\"generation\":%" PRIu32 ",\"records\":%" PRIu32
        ",\"capacity\":48,\"overflow\":%" PRIu32 ",\"collision_observed\":%" PRIu32 ",\"begin_us\":%" PRIu32 ",\"stop_requested_us\":%" PRIu32
        ",\"frozen_us\":%" PRIu32 ",\"reason\":%" PRIu32 ",\"row_storage_bytes\":%" PRIu32 ",\"minimum_request_bytes\":2048,\"creator_all_sizes\":true,\"exhaustive\":false,\"timestamps_are_callback_observations\":true,\"owner_task_id\":%" PRIu32 ",\"smp_task_id\":%" PRIu32 ",\"persist_task_id\":%" PRIu32 ",\"general_task_id\":%" PRIu32 "}\n",
        s.handle,s.generation,s.count,s.overflow,s.collision,s.begin_us,s.stop_requested_us,s.frozen_us,s.reason,s.bytes,s.owner_task_id,s.smp_task_id,s.persist_task_id,s.general_task_id);
    for(unsigned i=0;i<s.count && i<PROBE_HEAP_ALLOC_ROWS;i++){
        probe_heap_row_t r;if(!probe_heap_alloc_row(i,&r))continue;
        if(r.kind==1u)PROBE_SNAPSHOT_LOG("{\"probe\":\"heap_alloc_observation\",\"ordinal\":%u,\"address\":%" PRIu32 ",\"bytes\":%" PRIu32 ",\"caps\":%" PRIu32
            ",\"callback_us\":%" PRIu32 ",\"task_id\":%" PRIu32 ",\"role\":%" PRIu32 ",\"entry_core\":%" PRIu32 ",\"isr\":%s}\n",
            i,r.a,r.b,r.c,r.begin_us,r.actor,r.flags&255u,(r.flags>>8)&1u,(r.flags&512u)?"true":"false");
        else if(r.kind==3u)PROBE_SNAPSHOT_LOG("{\"probe\":\"heap_free_observation\",\"ordinal\":%u,\"address\":%" PRIu32 ",\"callback_us\":%" PRIu32 ",\"entry_core\":%" PRIu32 "}\n",
            i,r.a,r.begin_us,(r.flags>>8)&1u);
        else if(r.kind==4u)PROBE_SNAPSHOT_LOG("{\"probe\":\"heap_task_retire\",\"ordinal\":%u,\"task_id\":%" PRIu32 ",\"role\":%" PRIu32
            ",\"before_delete_us\":%" PRIu32 ",\"free_bytes\":%" PRIu32 ",\"minimum_bytes\":%" PRIu32 ",\"stack_low_water_bytes\":%" PRIu32 ",\"entry_core\":%" PRIu32 "}\n",
            i,r.a,r.b,r.begin_us,r.c,r.actor,r.end_us,(r.flags>>8)&1u);
        else PROBE_SNAPSHOT_LOG("{\"probe\":\"heap_phase_memory\",\"ordinal\":%u,\"job_kind\":%" PRIu32 ",\"entry_core\":%" PRIu32
            ",\"free_before\":%" PRIu32 ",\"minimum_before\":%" PRIu32 ",\"free_after\":%" PRIu32 ",\"minimum_after\":%" PRIu32 ",\"begin_us\":%" PRIu32 ",\"end_us\":%" PRIu32 "}\n",
            i,r.flags&255u,(r.flags>>8)&1u,r.a,r.b,r.c,r.actor,r.begin_us,r.end_us);
    }
}
#endif

static void log_loop(void *unused){
    (void)unused;
    uint32_t previous=UINT32_MAX,previous_errors=UINT32_MAX,previous_ignored=UINT32_MAX,previous_oversized=UINT32_MAX;
    unsigned ticks=0;
    uint32_t previous_codec_revision=0;
    uint32_t previous_encoded_epoch=UINT32_MAX,previous_encoded_known=UINT32_MAX;
#if defined(DBT_PROBE_STAY_NATIVE) && defined(DBT_PROBE_AUDIO_OUTPUT)
    uint32_t previous_activity[8]={0};unsigned quiet_ticks=0;bool activity_seen=false;
#endif
    for(;;){
        vTaskDelay(pdMS_TO_TICKS(100));log_media_trace();
#ifdef DBT_PROBE_HEAP_ALLOC_TRACE
        log_heap_alloc_capture();
#endif
#ifdef DBT_PROBE_DISPATCH_TRACE
        log_dispatch_trace();
#endif
#ifdef DBT_APTX_SNAPSHOT
        dbt_pcm_snapshot_log(!probe_output_running());
#endif
#if defined(PROBE_OUTPUT_GAP_TRACE) && defined(DBT_PROBE_AUDIO_OUTPUT)
        log_first_starvation();
#endif
        if(++ticks<10)continue;
        ticks=0;
        uint32_t at=(uint32_t)esp_timer_get_time();
#ifdef DBT_PROBE_AVRCP
        (void)log_metadata();
#endif
#if defined(DBT_PROBE_STAY_NATIVE) && defined(DBT_PROBE_AUDIO_OUTPUT)
        /* Event/fault rings and first-gap latches drain above this gate. Keep
         * playback detail at its existing cadence; while stopped, avoid
         * formatting unchanged cumulative counters four cycles out of five.
         * Lifecycle, PCM progress and error changes reopen detail next cycle.
         * A small liveness record keeps passive UART readers current. */
        uint32_t activity[8]={atomic_load(&profile_snapshot),atomic_load(&event_errors),
            atomic_load(&codec_revision),atomic_load(&pcm_generation),
            atomic_load(&worker.faults),atomic_load(&worker.media_drops),
            atomic_load(&ignored_dynamic),atomic_load(&oversized_dynamic)};
        if(!probe_output_running() && activity_seen &&
           !memcmp(previous_activity,activity,sizeof activity) && ++quiet_ticks<5u){
            PROBE_SNAPSHOT_LOG("{\"probe\":\"logger_idle\",\"detail_period_ms\":5000,\"event_drain_period_ms\":100,\"playback_detail_period_ms\":1000}\n");
            continue;
        }
        memcpy(previous_activity,activity,sizeof activity);activity_seen=true;quiet_ticks=0;
#endif
#ifdef DBT_PROBE_DISPATCH_TRACE
        probe_media_timing_t media_time;(void)probe_output_media_timing(&media_time);
        PROBE_SNAPSHOT_LOG("{\"probe\":\"media_admission_clock\",\"collection_cycle_us\":%" PRIu32 ",\"coherent\":%s,\"media_epoch\":%" PRIu32 ",\"sequence\":%" PRIu32 ",\"rtp_timestamp\":%" PRIu32 ",\"ssrc\":%" PRIu32 ",\"begin_us\":%" PRIu32 ",\"admitted_us\":%" PRIu32 ",\"result\":%" PRIu32 ",\"hci_arrival_valid\":%s,\"hci_arrival_us\":%" PRIu32 ",\"rf_arrival\":false}\n",
            at,media_time.coherent?"true":"false",media_time.epoch,media_time.sequence,media_time.timestamp,media_time.ssrc,media_time.begin_us,media_time.admitted_us,media_time.result,media_time.arrival_valid?"true":"false",media_time.arrival_us);
#endif

#if (defined(DBT_PROBE_SCHEDULER_TRACE) || defined(DBT_PROBE_STAY_NATIVE)) && defined(DBT_PROBE_AUDIO_OUTPUT)
        /* Playback counters use the one-second detail cadence. Idle detail
         * is reduced above; trace and first-gap draining remain independent. */
        probe_arrival_stats_t arrival_stats;probe_arrival_stats(&arrival_stats);
        PROBE_SNAPSHOT_LOG("{\"probe\":\"media_arrival_fifo\",\"published\":%" PRIu32 ",\"consumed\":%" PRIu32 ",\"lost\":%" PRIu32
            ",\"status\":%" PRIu32 ",\"metadata_bytes\":%" PRIu32 ",\"metadata_slots\":8,\"diagnostic_only\":true,\"association_scope\":\"complete_current_acl_l2cap_rtp\",\"counts_independent\":true}\n",
            arrival_stats.published,arrival_stats.consumed,arrival_stats.lost,arrival_stats.status,arrival_stats.bytes);
#endif
#if defined(DBT_PROBE_SCHEDULER_TRACE) && defined(DBT_PROBE_AUDIO_OUTPUT)
        probe_vhci_ready_stats_t readiness;probe_vhci_ready_stats(&readiness);
        PROBE_SNAPSHOT_LOG("{\"probe\":\"vhci_ready_probe\",\"observations\":%" PRIu32 ",\"held_observations\":%" PRIu32
            ",\"sdk_ready_while_held\":%" PRIu32 ",\"max_ready_observation_span_us\":%" PRIu32
            ",\"max_slot_age_when_sdk_ready_us\":%" PRIu32 ",\"metadata_bytes\":%" PRIu32
            ",\"independent_counters\":true,\"sends_unchanged\":true,\"held_queries_limited_per_owner_turn\":true}\n",
            readiness.observations,readiness.held,readiness.ready_while_held,readiness.max_ready_span_us,readiness.max_slot_age_us,readiness.bytes);
#endif
#if defined(DBT_PROBE_VOLUME_RAMP_COMPARE) || defined(DBT_PROBE_MATCHED_FEEDBACK_COMPARE)
        log_volume_kernels();
#endif
#ifdef DBT_PROBE_OWNER_PACER
        log_owner_pacer();
#endif
#ifdef DBT_PROBE_SCHEDULER_TRACE
        log_render_delays();
        log_owner_costs();
        log_pcm_level();
#endif
#ifdef DBT_PROBE_TASK_RUNTIME_STATS
        log_task_runtime();
#endif
#ifdef DBT_PROBE_TASK_CYCLE_ACCOUNTING
        log_render_accounting();
        log_host_accounting();
#endif
#ifdef DBT_PROBE_DSP_ASSIST
        probe_dsp_assist_stats_t assisted;probe_output_dsp_assist_snapshot(&assisted);
        PROBE_SNAPSHOT_LOG("{\"probe\":\"dsp_assist\",\"collection_cycle_us\":%" PRIu32 ",\"counter_scope\":\"since_boot\",\"independent_counters\":true,\"enabled\":%" PRIu32 ",\"state\":%" PRIu32 ",\"worker_core\":0,\"render_owner_core\":1,\"jobs\":%" PRIu32 ",\"decode_reserve_submitted\":%" PRIu32 ",\"blocks\":%" PRIu32 ",\"max_blocks_per_job\":%u,\"max_job_frames\":%" PRIu32 ",\"frames\":%" PRIu32 ",\"helper_cycles\":%" PRIu32 ",\"helper_max_cycles\":%" PRIu32 ",\"queue_max_us\":%" PRIu32 ",\"finish_to_commit_max_us\":%" PRIu32 ",\"commit_to_retire_max_us\":%" PRIu32 ",\"publication_core\":0,\"wall_max_us\":%" PRIu32 ",\"failures\":%" PRIu32 ",\"cycles_include_preemption\":true,\"cycles_wrap_uint32\":true,\"wall_includes_async_wait\":true}\n",
            at,assisted.enabled,assisted.state,assisted.jobs,assisted.reserve_submitted,assisted.blocks,(unsigned)DBT_PROBE_DSP_ASSIST_BLOCKS,assisted.max_job_frames,assisted.frames,assisted.cycles,assisted.max_cycles,
            assisted.queue_max_us,assisted.finish_wait_max_us,assisted.retire_wait_max_us,assisted.wall_max_us,assisted.failures);
#endif
#ifdef DBT_PROBE_LHDC_BUILD
        log_lhdc_quality(at);
#ifdef DBT_PROBE_LHDC_STAGE_PROFILE
        log_lhdc_live(at);
#endif
#endif
#ifdef DBT_PROBE_SCHEDULER_TRACE
        for(unsigned core=0;core<2;++core){
            probe_scheduler_snapshot_t sample;uint32_t began=(uint32_t)esp_timer_get_time();
            probe_scheduler_snapshot(core,&sample);
            snapshot_log(began,"{\"probe\":\"scheduler_samples\",\"core\":%u,\"sample_hz\":%u,\"coherent\":%s,\"known_task_mask\":%" PRIu32
                ",\"host\":%" PRIu32 ",\"decoder\":%" PRIu32 ",\"renderer\":%" PRIu32 ",\"logger\":%" PRIu32 ",\"controller\":%" PRIu32
                ",\"idle\":%" PRIu32 ",\"other\":%" PRIu32 ",\"ipc\":%" PRIu32 ",\"timer\":%" PRIu32 ",\"last_tick_us\":%" PRIu32 ",\"max_tick_gap_us\":%" PRIu32
                ",\"max_gap_begin_us\":%" PRIu32 ",\"max_gap_end_us\":%" PRIu32 ",\"max_gap_previous_role\":%" PRIu32 ",\"max_gap_current_role\":%" PRIu32
                ",\"max_callback_body_cycles\":%" PRIu32 "}\n",core,(unsigned)PROBE_SCHEDULER_SAMPLE_HZ,sample.coherent?"true":"false",sample.known_task_mask,
                sample.counts[0],sample.counts[1],sample.counts[2],sample.counts[3],sample.counts[4],sample.counts[5],sample.counts[6],sample.counts[7],sample.counts[8],
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
        uint32_t credit_status=atomic_load(&credit_window_status);
        PROBE_SNAPSHOT_LOG("{\"probe\":\"rx_credit_backpressure\",\"collection_cycle_us\":%" PRIu32
            " ,\"host_packet_window\":%u,\"classic_gate_open\":%s,\"classic_releasable_packets\":%u,\"credit_policy\":\"partial_slots\",\"pending_completed_classic\":%" PRIu32
            " ,\"pending_completed_le\":%" PRIu32
            " ,\"peak_blocked_classic\":%" PRIu32
            " ,\"acknowledged_acl\":%" PRIu32
            " ,\"completion_commands\":%" PRIu32
            " ,\"partial_completion_commands\":%" PRIu32
            " ,\"media_queued\":%" PRIu32 "}\n",
            at,(unsigned)(credit_status&0xffffu),(credit_status&0x10000u)?"true":"false",(unsigned)(credit_status>>17),atomic_load(&credit_pending_classic),
            atomic_load(&credit_pending_le),atomic_load(&credit_peak_blocked),atomic_load(&credit_acknowledged),
            atomic_load(&credit_commands),atomic_load(&credit_partial_commands),atomic_load(&credit_media_queued));
        uint32_t encoded_frames=0,encoded_epoch=atomic_load(&worker.duration_epoch);
        bool encoded_known=dbt_codec_worker_buffered_frames(&worker,&encoded_frames);
        if(encoded_epoch!=atomic_load(&worker.duration_epoch)){encoded_known=false;encoded_frames=0;}
        if(encoded_known || encoded_epoch!=previous_encoded_epoch || (uint32_t)encoded_known!=previous_encoded_known){
        PROBE_SNAPSHOT_LOG("{\"probe\":\"encoded_media_buffer\",\"collection_cycle_us\":%" PRIu32
            ",\"encoded_epoch\":%" PRIu32 ",\"encoded_pcm_frames\":%" PRIu32 ",\"known\":%s,\"single_attempt\":true,\"copied_deferred_only\":true}\n",
            at,encoded_epoch,encoded_frames,encoded_known?"true":"false");
            previous_encoded_epoch=encoded_epoch;previous_encoded_known=encoded_known;
        }
        uint32_t packet_geometry=atomic_load_explicit(&worker.queue_layout,memory_order_acquire);
        uint32_t packet_slabs=packet_geometry&DBT_CODEC_QUEUE_DEPTH_MASK,packet_stride=packet_geometry>>16;
        PROBE_SNAPSHOT_LOG("{\"probe\":\"decoder_input_queue\",\"collection_cycle_us\":%" PRIu32
               ",\"packet_capacity_bytes\":%u,\"packet_slots\":%u,\"slot_stride_bytes\":%u"
               ",\"reserved_packet_bytes\":%u,\"max_seen_payload_bytes\":%" PRIu32
               ",\"oversize_rejects\":%" PRIu32 ",\"last_oversize_bytes\":%" PRIu32 "}\n",
               at,(unsigned)worker.maximum_payload,(unsigned)packet_slabs,(unsigned)packet_stride,
               (unsigned)(packet_stride*packet_slabs),atomic_load(&worker.input_max_payload),
               atomic_load(&worker.input_oversize_rejects),atomic_load(&worker.input_last_oversize));
        PROBE_SNAPSHOT_LOG("{\"probe\":\"owner_wait_reasons\",\"collection_cycle_us\":%" PRIu32 ",\"nonruntime_queries_skipped_on_runtime_ready\":true,\"iterations\":%" PRIu32
               ",\"runtime\":%" PRIu32 ",\"admission\":%" PRIu32 ",\"smp\":%" PRIu32 ",\"avrcp\":%" PRIu32
               ",\"restart\":%" PRIu32 ",\"pair_button\":%" PRIu32 ",\"rfcomm\":%" PRIu32 ",\"gap\":%" PRIu32
               ",\"product\":%" PRIu32 ",\"io\":%" PRIu32 ",\"att\":%" PRIu32 ",\"security\":%" PRIu32
               ",\"echo\":%" PRIu32 ",\"sdp\":%" PRIu32 ",\"a2dp\":%" PRIu32 ",\"runnable_yields\":%" PRIu32 "}\n",at,atomic_load(&owner_iterations),
               atomic_load(&owner_immediate[0]),atomic_load(&owner_immediate[1]),atomic_load(&owner_immediate[2]),
               atomic_load(&owner_immediate[3]),atomic_load(&owner_immediate[4]),atomic_load(&owner_immediate[5]),
               atomic_load(&owner_immediate[6]),atomic_load(&owner_immediate[7]),atomic_load(&owner_immediate[8]),
               atomic_load(&owner_immediate[9]),atomic_load(&owner_immediate[10]),atomic_load(&owner_immediate[11]),
               atomic_load(&owner_immediate[12]),atomic_load(&owner_immediate[13]),atomic_load(&owner_immediate[14]),atomic_load(&owner_ready_yields));
        PROBE_SNAPSHOT_LOG("{\"probe\":\"owner_render_service\",\"yield_count\":%" PRIu32 "}\n",atomic_load(&owner_service_yields));
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
               ",\"host_trace_scope\":\"media_and_host_state\",\"host_trace_lost\":%" PRIu32 ",\"worker_pcm_busy_events\":" PCM_BUSY_TRACE_JSON ",\"worker_trace_lost\":%" PRIu32 ",\"snapshot_records_lost\":%" PRIu32 ",\"snapshot_uart_lost\":%" PRIu32 "}\n",at,
               atomic_load(&worker.queue_full_drops),atomic_load(&worker.decode_rejects),atomic_load(&worker.reset_failures),
               atomic_load(&worker.pcm_failures),atomic_load(&worker.pcm_timeouts),atomic_load(&worker.pcm_backpressure),
               atomic_load(&worker.input_sequence_breaks),atomic_load(&worker.input_ssrc_changes),atomic_load(&worker.queue_high_water),
               atomic_load(&media_trace_rings[0].lost),atomic_load(&media_trace_rings[1].lost),atomic_load(&snapshot_format_lost),snapshot_uart_failures());
        PROBE_SNAPSHOT_LOG("{\"probe\":\"l2cap_reassembly\",\"collection_cycle_us\":%" PRIu32 ",\"delivered\":%" PRIu32
            ",\"abandoned\":%" PRIu32 ",\"rejected_wire_packets\":%" PRIu32 ",\"orphan_continuations\":%" PRIu32 ",\"oversized_pdus\":%" PRIu32 ",\"malformed_signaling\":%" PRIu32 ",\"unknown_responses\":%" PRIu32 ",\"retained_storage_bytes\":%" PRIu32 "}\n",
            at,atomic_load(&l2cap_delivered),atomic_load(&l2cap_abandoned),atomic_load(&l2cap_rejected),atomic_load(&l2cap_orphaned),atomic_load(&l2cap_oversized),atomic_load(&l2cap_malformed_signaling),atomic_load(&l2cap_unknown_responses),atomic_load(&l2cap_retained_bytes));
        PROBE_SNAPSHOT_LOG("{\"probe\":\"worker_timing\",\"collection_cycle_us\":%" PRIu32 ",\"steps\":%" PRIu32 ",\"elapsed_us\":%" PRIu32
               ",\"max_step_us\":%" PRIu32 ",\"wall_time_includes_preemption\":true,\"step_budget\":2,\"budget_is_ceiling\":true,\"histogram_us_le\":[125,250,500,1000,2000,4000,8000],\"histogram_counts\":[%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 "]}\n",
               at,atomic_load(&worker_steps),atomic_load(&worker_elapsed_us),atomic_load(&worker_step_max_us),
               atomic_load(&worker_hist[0]),atomic_load(&worker_hist[1]),atomic_load(&worker_hist[2]),atomic_load(&worker_hist[3]),
               atomic_load(&worker_hist[4]),atomic_load(&worker_hist[5]),atomic_load(&worker_hist[6]),atomic_load(&worker_hist[7]));
        static const char *const wait_names[]={"pcm_space","pcm_loan_retirement","control","encoded_input","shared_renderer"};
        uint32_t waiting=atomic_load_explicit(&decoder_wait_kind,memory_order_acquire);
        uint32_t wait_began=atomic_load_explicit(&decoder_wait_began,memory_order_relaxed);
        for(unsigned reason=0;reason<5;++reason){
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
               ",\"payload_bytes\":%" PRIu32 ",\"rtp_first_timestamp\":%" PRIu32 ",\"rtp_last_timestamp\":%" PRIu32 ",\"rate\":%" PRIu32 ",\"timestamp_origin\":\"%s\"}\n",
               at,atomic_load(&worker.input_stream_epoch),atomic_load(&worker.input_packets),atomic_load(&worker.input_payload_bytes),
               atomic_load(&worker.input_first_timestamp),atomic_load(&worker.input_last_timestamp),rate,
               (codec==5u || codec==7u)?"host_samples":"wire_rtp");
#if defined(DBT_PROBE_AUDIO_OUTPUT) && defined(PROBE_OUTPUT_GAP_TRACE)
        probe_media_timing_t media_timing;
        if(probe_output_media_timing(&media_timing)){
            PROBE_SNAPSHOT_LOG("{\"probe\":\"media_ingress_timing\",\"epoch\":%u,\"sequence\":%u,\"media_timestamp\":%u,\"ssrc\":%u,\"host_begin_us\":%u,\"host_admitted_us\":%u,\"result\":%u,\"hci_arrival_valid\":%s,\"hci_arrival_us\":%u,\"host_gap_us\":%u,\"hci_gap_us\":%u,\"hci_gap_valid\":%s,\"scope\":\"latest_synchronous_media_tuple\"}\n",
                (unsigned)media_timing.epoch,(unsigned)media_timing.sequence,(unsigned)media_timing.timestamp,(unsigned)media_timing.ssrc,
                (unsigned)media_timing.begin_us,(unsigned)media_timing.admitted_us,(unsigned)media_timing.result,
                media_timing.arrival_valid?"true":"false",(unsigned)media_timing.arrival_us,(unsigned)media_timing.host_gap_us,
                (unsigned)media_timing.hci_gap_us,media_timing.hci_gap_us!=UINT32_MAX?"true":"false");
        }
#endif
        dbt_codec_input_clock_t input_clock_snapshot;dbt_codec_worker_clock_snapshot(&worker,&input_clock_snapshot);
        PROBE_SNAPSHOT_LOG("{\"probe\":\"rtp_input_clock\",\"collection_cycle_us\":%" PRIu32 ",\"stream_epoch\":%" PRIu32
            ",\"coherent\":%s,\"observed\":%s,\"revision\":%" PRIu32 ",\"max_forward_step_ticks\":%" PRIu32
            ",\"previous_sequence\":%u,\"sequence\":%u,\"previous_rtp_timestamp\":%" PRIu32 ",\"rtp_timestamp\":%" PRIu32
            ",\"ssrc\":%" PRIu32 ",\"scope\":\"input_epoch_adjacent_wire_sequence_same_ssrc_forward_half_range\",\"before_queue_and_decode\":true,\"loss_count\":false}\n",
            at,input_clock_snapshot.epoch,input_clock_snapshot.coherent?"true":"false",input_clock_snapshot.observed?"true":"false",
            input_clock_snapshot.revision,input_clock_snapshot.step_ticks,input_clock_snapshot.observed?(unsigned)(uint16_t)(input_clock_snapshot.sequence-1u):0u,(unsigned)input_clock_snapshot.sequence,
            input_clock_snapshot.timestamp-input_clock_snapshot.step_ticks,input_clock_snapshot.timestamp,input_clock_snapshot.ssrc);
#ifdef DBT_PROBE_APTX_BUILD
        dbt_aptx_quality_t aptx_quality;dbt_aptx_quality_snapshot(&aptx_quality);
        snapshot_log(pcm_snapshot_began,"{\"probe\":\"aptx_resynchronization\",\"counter_scope\":\"decoder_lifetime_since_boot\",\"active\":%s"
            ",\"bounded_probe_calls\":%" PRIu32 ",\"consumed_encoded_bytes\":%" PRIu32 ",\"resync_discarded_bytes\":%" PRIu32
            ",\"completed\":%" PRIu32 ",\"parity_rejects\":%" PRIu32 ",\"max_new_codewords_per_probe\":1,\"radio_loss_counter\":false}\n",
            aptx_quality.active?"true":"false",aptx_quality.calls,aptx_quality.consumed_bytes,aptx_quality.discarded_bytes,
            aptx_quality.completed,aptx_quality.parity_rejects);
#endif
        uint32_t queue_layout=atomic_load_explicit(&worker.queue_layout,memory_order_acquire);
        uint32_t queue_depth=queue_layout&DBT_CODEC_QUEUE_DEPTH_MASK,queue_stride=queue_layout>>16;
        snapshot_log(pcm_snapshot_began,"{\"probe\":\"codec_queue_memory\",\"collection_cycle_us\":%" PRIu32 ",\"last_started_codec\":%" PRIu32
            ",\"depth\":%" PRIu32 ",\"credit_low_water\":%" PRIu32 ",\"slot_payload_capacity\":%u,\"slot_stride\":%" PRIu32
            ",\"accounted_queue_allocation_bytes\":%" PRIu32 ",\"packed_records\":%s,\"includes_allocation_header\":true}\n",
            at,codec,queue_depth,(queue_layout>>8)&255u,(unsigned)PROBE_A2DP_RX_MTU,queue_stride,
            queue_depth?queue_stride*queue_depth+8u:0u,(queue_layout&DBT_CODEC_QUEUE_PACKED)?"true":"false");
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
                   ",\"delay_reporting\":%s,\"ignored_dynamic\":%" PRIu32 ",\"oversized_dynamic\":%" PRIu32 "}\n",
                   (unsigned)(snapshot&255u),(unsigned)((snapshot>>8)&1u),(unsigned)((snapshot>>9)&1u),
                   (unsigned)((snapshot>>10)&1u),errors,atomic_load(&last_channel_event),atomic_load(&last_channel_result),(snapshot&0x7fffffffu)>>11,(snapshot>>31)?"true":"false",ignored,oversized);
        }
#ifdef DBT_PROBE_AUDIO_OUTPUT
        uint32_t output_snapshot_began=(uint32_t)esp_timer_get_time();
        probe_output_stats_t stats;probe_output_stats(&stats);
        snapshot_log(output_snapshot_began,"{\"probe\":\"dsp_signal_quality\",\"valid\":%s,\"owner_snapshot_us\":%" PRIu32
            ",\"rate\":%" PRIu32 ",\"gains_db\":[%d,%d,%d],\"volume\":%u,\"flags\":%u,\"rendered_frames\":%" PRIu32
            ",\"peak_q27\":%" PRIu32 ",\"limited_channel_samples\":%" PRIu32 ",\"over_full_scale_channel_samples\":%" PRIu32 "}\n",
            stats.dsp_valid?"true":"false",stats.dsp_at_us,stats.dsp_rate,
            (int)(int8_t)stats.dsp_gains,(int)(int8_t)(stats.dsp_gains>>8),(int)(int8_t)(stats.dsp_gains>>16),
            (unsigned)((stats.dsp_options>>8)&255u),(unsigned)(stats.dsp_options&255u),stats.dsp_frames,
            stats.dsp_peak_q27,stats.dsp_limited,stats.dsp_over_full_scale);
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
        uint32_t dma_bytes=PROBE_OUTPUT_DMA_DESCRIPTORS*PROBE_OUTPUT_DMA_FRAMES*8u;
        uint32_t output_capacity=stats.input_capacity_frames && stats.requested_bytes>=stats.input_ring_bytes+dma_bytes?
            (stats.requested_bytes-stats.input_ring_bytes-dma_bytes)/8u:0;
#ifdef DBT_PROBE_EARLY_INPUT_RELEASE
        const char *early_input="true";
#else
        const char *early_input="false";
#endif
        /* Logger-local configuration identity: avoid formatting/transmitting
         * unchanged policies every cycle. A ten-cycle heartbeat lets a reattached
         * reader recover configuration. Fault/counter/event records below
         * remain independent and periodic; every emitted UART line is retained. */
        static uint32_t previous_config[13],config_ticks;static bool config_seen;
        uint32_t config_key[13]={stats.rate,stats.input_capacity_frames,stats.input_ring_bytes,
            stats.requested_bytes,stats.pcm_target_frames,stats.render_core,stats.render_owner_transfers,
            stats.auxiliary_render_stack_bytes,stats.auxiliary_render_allocation_failures,
            stats.priming_ms,stats.priming_remaining_ms,stats.startup_enable_us,stats.startup_coherent};
        if(!config_seen || memcmp(previous_config,config_key,sizeof config_key) || ++config_ticks>=10u){
            memcpy(previous_config,config_key,sizeof config_key);config_seen=true;config_ticks=0;
        snapshot_log(output_snapshot_began,"{\"probe\":\"pcm_buffer_geometry\",\"configured_target_ms\":%u,\"rate\":%" PRIu32
            ",\"decoder_transfer_limit_frames\":%u,\"input_capacity_frames\":%" PRIu32 ",\"input_bytes\":%" PRIu32
            ",\"output_capacity_frames\":%" PRIu32 ",\"output_target_frames\":%" PRIu32
            ",\"output_frame_bytes\":8,\"render_block_limit_frames\":%u,\"dma_descriptors\":%u,\"dma_frames_per_descriptor\":%u"
            ",\"dma_bytes\":%" PRIu32 ",\"configured_early_input_release\":%s,\"output_capacity_from_allocation_receipt\":true}\n",
            (unsigned)DBT_PROBE_PCM_TARGET_MS,stats.rate,(unsigned)DBT_CODEC_PCM_CHUNK_FRAMES,stats.input_capacity_frames,stats.input_ring_bytes,
            output_capacity,stats.pcm_target_frames,(unsigned)FIXED_DSP_BLOCK_FRAMES,(unsigned)PROBE_OUTPUT_DMA_DESCRIPTORS,
            (unsigned)PROBE_OUTPUT_DMA_FRAMES,dma_bytes,early_input);
        snapshot_log(output_snapshot_began,"{\"probe\":\"pcm_startup_guard\",\"collection_cycle_us\":%" PRIu32
               ",\"minimum_ms\":%" PRIu32 ",\"remaining_ms\":%" PRIu32 ",\"buffers_added\":0}\n",
               at,stats.priming_ms,stats.priming_remaining_ms);

#ifdef PROBE_OUTPUT_GAP_TRACE
        snapshot_log(output_snapshot_began,"{\"probe\":\"pcm_startup_actual\",\"first_pcm_us\":%" PRIu32
            ",\"enable_us\":%" PRIu32 ",\"elapsed_us\":%" PRIu32 ",\"input_frames\":%" PRIu32
            ",\"preload_frames\":%" PRIu32 ",\"guard_state\":%" PRIu32 ",\"encoded_slots\":%" PRIu32 ",\"encoded_slots_known\":%s,\"coherent\":%s}\n",
            stats.startup_first_pcm_us,stats.startup_enable_us,stats.startup_elapsed_us,
            stats.startup_input_frames,stats.startup_preload_frames,stats.startup_guard_state,stats.startup_encoded_slots,
            stats.startup_coherent && stats.startup_encoded_slots!=UINT32_MAX?"true":"false",stats.startup_coherent?"true":"false");
#endif
        snapshot_log(output_snapshot_began,"{\"probe\":\"pcm_clock_policy\",\"adaptive_clock\":%" PRIu32
               ",\"one_to_one_pcm\":%s,\"target_frames\":%" PRIu32 "}\n",stats.adaptive_clock,stats.adaptive_clock?"false":"true",stats.pcm_target_frames);
#ifdef DBT_PROBE_CODEC_AWARE_RENDER
        snapshot_log(output_snapshot_began,"{\"probe\":\"render_owner_policy\",\"collection_cycle_us\":%" PRIu32
               ",\"core\":%" PRIu32 ",\"transfers\":%" PRIu32 ",\"auxiliary_stack_bytes\":%" PRIu32
               ",\"allocation_failures\":%" PRIu32 ",\"main_priority\":%" PRIu32 ",\"auxiliary_priority\":%" PRIu32 ",\"single_owner\":true}\n",at,stats.render_core,
               stats.render_owner_transfers,stats.auxiliary_render_stack_bytes,stats.auxiliary_render_allocation_failures,
               stats.main_render_priority,stats.auxiliary_render_priority);
#endif
        }
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
#ifdef DBT_AUDIO_STAGE_PROFILE
        snapshot_log(output_snapshot_began,"{\"probe\":\"render_stage_cycles\",\"collection_cycle_us\":%" PRIu32
               ",\"frames\":%" PRIu32 ",\"conversion_fir\":%" PRIu32 ",\"dsp\":%" PRIu32 ",\"dsp_counter_excludes_assist\":true,\"publication_mix\":%" PRIu32
               ",\"controls\":%" PRIu32 ",\"conversion_fir_max\":%" PRIu32 ",\"dsp_max\":%" PRIu32 ",\"publication_mix_max\":%" PRIu32
               ",\"core\":%" PRIu32 ",\"core_field_is_render_owner\":true,\"cycles_measured_on_executing_core\":true,\"core_cycles_include_preemption\":true,\"counts_wrap_uint32\":true}\n",at,
               stats.profile_frames,stats.profile_conversion,stats.profile_dsp,stats.profile_tail,stats.profile_controls,
               stats.profile_conversion_max,stats.profile_dsp_max,stats.profile_tail_max,stats.render_core);
#endif
        snapshot_log(output_snapshot_began,"{\"probe\":\"i2s_refill_window\",\"collection_cycle_us\":%" PRIu32
            " ,\"period_us\":%" PRIu32
            " ,\"initial_shortage_frames\":%" PRIu32
            " ,\"recovered_frames\":%" PRIu32
            " ,\"recovered_copies\":%" PRIu32
            " ,\"unfilled_frames\":%" PRIu32
            " ,\"canceled_frames\":%" PRIu32
            " ,\"copy_max_us\":%" PRIu32
            " ,\"mask_max_us\":%" PRIu32
            " ,\"contract_errors\":%" PRIu32
            " ,\"dispatch_requests\":%" PRIu32
            " ,\"pending_index\":%" PRIu32 ",\"guard_us\":64,\"copy_budget_us\":32,\"closes_at_next_eof\":true}\n",
            at,stats.refill_period_us,stats.underrun_frames,stats.refill_recovered_frames,stats.refill_recovered_calls,
            stats.refill_unfilled_frames,stats.refill_canceled_frames,stats.refill_copy_max_us,stats.refill_mask_max_us,stats.refill_contract_errors,stats.refill_dispatch_requests,stats.refill_pending_index);
        snapshot_log(output_snapshot_began,"{\"probe\":\"i2s_order_recovery\",\"collection_cycle_us\":%" PRIu32
               ",\"recoveries\":%" PRIu32 ",\"missed_callbacks_min\":%" PRIu32 ",\"source_frames_recovered\":%" PRIu32
               ",\"additional_complete_laps_unknown\":true}\n",at,stats.descriptor_order_recoveries,
               stats.descriptor_missed_eofs_min,stats.descriptor_recovered_source_frames);
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
    probe_host_record_t record={.kind=PROBE_HOST_A2DP_STATE,.value={profile.sink.binding,
        profile.sink.pending.used?profile.sink.pending.token:0u,(uint32_t)event,(uint32_t)state,
        profile.sink.media_binding,atomic_load(&worker.gate_epoch),atomic_load(&worker.started_epoch)}};
    (void)probe_audio_host_record(&record);
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
_Static_assert(PROBE_A2DP_MEDIA_SLOTS>=8,"Selected-codec policy needs up to eight full-MTU slots");
static dbt_codec_queue_t codec_queue_policy(void *context,const dbt_codec_config_t *config){
    (void)context;(void)config;
    /* Keep burst/startup capacity for AAC, LDAC and LHDC. Lighter codecs use
     * four slots. Each holds a whole negotiated 1800-byte SDU, not PCM scratch.
     * The controller may still deliver three previously credited ACL packets. */
    uint8_t depth=4;
#ifdef DBT_PROBE_AAC_BUILD
    if(config->ops==&dbt_aac_codec)depth=8;
#endif
#ifdef DBT_PROBE_LDAC_BUILD
    if(config->ops==&dbt_ldac_codec)depth=8;
#endif
#ifdef DBT_PROBE_LHDC_BUILD
    if(config->ops==&dbt_lhdcv5_codec)depth=8;
#endif
    uint8_t low=(uint8_t)(depth/2u);
#ifdef DBT_PROBE_LHDC_BUILD
    /* At192k pack small LHDC records within the existing physical slabs.
     * Admission/credits still reserve worst-case full-MTU slabs for permits
     * already outside the host; an open slab tail never grants such credit. */
    if(config->ops==&dbt_lhdcv5_codec && config->rate==192000u)low=(uint8_t)(depth-3u);
#endif
    if(low>depth-3u)low=(uint8_t)(depth-3u);
    uint8_t storage=depth;
#ifdef DBT_PROBE_LHDC_BUILD
    if(config->ops==&dbt_lhdcv5_codec && config->rate==192000u)storage|=DBT_CODEC_QUEUE_PACKED;
#endif
    return (dbt_codec_queue_t){storage,low};
}
uint16_t probe_audio_rx_credit_window(void){return 3u;}
bool probe_audio_rx_credit_ready(void *context,const dbt_link_t *link){
    (void)context;
    if(!initialized || !link || link->kind!=DBT_LINK_CLASSIC || restart_hold ||
       !profile.active || profile.draining || profile.sink.failed || profile.sink.retiring ||
       profile.sink.state!=DBT_STREAM_STREAMING || !profile.signaling.serial ||
       profile.signaling.handle!=link->handle || profile.signaling.generation!=link->generation)return true;
    uint32_t layout=atomic_load_explicit(&worker.queue_layout,memory_order_acquire);
    uint32_t depth=layout&DBT_CODEC_QUEUE_DEPTH_MASK;
    uint8_t low=(uint8_t)(layout>>8);
    return low && low<=depth && dbt_codec_worker_free_slots(&worker)>depth-low;
}
uint16_t probe_audio_rx_credit_limit(void *context,const dbt_link_t *link,uint16_t pending,uint16_t window){
    (void)context;
    if(!pending)return 0;
    if(!initialized || !link || link->kind!=DBT_LINK_CLASSIC || restart_hold ||
       !profile.active || profile.draining || profile.sink.failed || profile.sink.retiring ||
       profile.sink.state!=DBT_STREAM_STREAMING || !profile.signaling.serial ||
       profile.signaling.handle!=link->handle || profile.signaling.generation!=link->generation)return pending;
    uint32_t free_slots=dbt_codec_worker_free_slots(&worker);
    if(!free_slots)return 0;
    /* Every permit not held in THIS link's pending count could still become
     * an audio packet, including credits released for another (e.g.LE) link.
     * Keep the existing one-slot allowance for a deferred/partial media PDU. */
    uint32_t outside=window>pending?window-pending:0;
    uint32_t free=free_slots-1u;
    if(free<=outside)return 0;
    uint32_t allowed=free-outside;
    return allowed<pending?(uint16_t)allowed:pending;
}

static uint32_t media_frame_count(const dbt_codec_config_t *config,const dbt_avdtp_media_t *packet){
#ifndef DBT_PROBE_SCHEDULER_TRACE
    /* No hot-path duration work for a feature this peer did not negotiate. */
    if(!profile.sink.delay_enabled)return 0;
#endif
#ifdef DBT_PROBE_LHDC_BUILD
    if(config->ops==&dbt_lhdcv5_codec)return dbt_lhdcv5_packet_frames(config,packet);
#else
    (void)config;(void)packet;
#endif
    return 0; /* Other codecs retain the existing PCM-only estimate. */
}
dbt_result_t probe_audio_init(dbt_runtime_t *runtime,TaskHandle_t host_owner){
    if(restart_hold)return DBT_BUSY;
    owner=host_owner;
    dbt_codec_worker_hooks_t hooks={0};hooks.decoder=(dbt_codec_hooks_t){0,codec_allocate,release,pcm};
    hooks.codecs=codecs;hooks.codec_count=(uint8_t)(sizeof(codecs)/sizeof(codecs[0]));
    hooks.wake_worker=wake_worker;hooks.wake_owner=wake_owner;
    hooks.queue_policy=codec_queue_policy;
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
    result=dbt_codec_worker_set_media_frames(&worker,media_frame_count);if(result!=DBT_OK)return result;
    dbt_avdtp_sink_hooks_t client={&worker,allocate,release,sink_work,sink_event,timed_worker_media};
    result=dbt_a2dp_acceptor_init(&profile,runtime,client,endpoints,(uint8_t)(sizeof(endpoints)/sizeof(endpoints[0])),PROBE_A2DP_RX_MTU,64,5000,2000,5000);
    if(result!=DBT_OK)return result;
    /* Cold codec checks finished before this init. Enable live quality counts
     * before creating the single decoder owner; never read its codec pointers. */
#ifdef DBT_PROBE_LHDC_BUILD
    dbt_lhdcv5_quality_start();
#endif
    #ifdef DBT_APTX_SNAPSHOT
    dbt_pcm_snapshot_arm();
    #endif
    if(xTaskCreatePinnedToCore(decode_loop,"native_decoder",8192,0,DBT_PROBE_DECODER_PRIORITY,&decoder_task,DBT_PROBE_DECODER_CORE)!=pdPASS)return DBT_NO_SPACE;
#ifdef DBT_PROBE_AUDIO_OUTPUT
    result=probe_audio_output_bootstrap();
    if(result!=DBT_OK){vTaskDelete(decoder_task);decoder_task=0;return result;}
#endif
    if(xTaskCreatePinnedToCore(log_loop,"native_pcm_log",4096,0,1,&logger_task,AUDIO_LOGGER_CORE)!=pdPASS){vTaskDelete(decoder_task);decoder_task=0;return DBT_NO_SPACE;}
#if defined(DBT_PROBE_SCHEDULER_TRACE) || defined(DBT_PROBE_TASK_RUNTIME_STATS)
    esp_err_t sampled=probe_scheduler_start(owner,decoder_task,logger_task);
    printf("{\"probe\":\"scheduler_sampler\",\"active\":%s,\"result\":%u,\"sample_hz\":%u,\"tick_sampling\":%s,\"runtime_only\":%s,\"task_count_added\":0,\"cpu_runtime_exact\":false}\n",
        sampled==ESP_OK?"true":"false",(unsigned)sampled,(unsigned)PROBE_SCHEDULER_SAMPLE_HZ,PROBE_TICK_SAMPLING_JSON,PROBE_RUNTIME_ONLY_JSON);
#endif
    initialized=true;
#ifdef DBT_PROBE_DISPATCH_TRACE
    probe_dispatch_bind(PROBE_DISPATCH_HOST,owner);probe_dispatch_bind(PROBE_DISPATCH_DECODE,decoder_task);
    probe_dispatch_bind(PROBE_DISPATCH_LOGGER,logger_task);
    probe_dispatch_bind(PROBE_DISPATCH_CONTROLLER,xTaskGetHandle("btController"));
    probe_dispatch_bind(PROBE_DISPATCH_IPC1,xTaskGetHandle("ipc1"));
    probe_dispatch_bind(PROBE_DISPATCH_IDLE0,xTaskGetHandle("IDLE0"));
    probe_dispatch_bind(PROBE_DISPATCH_IDLE1,xTaskGetHandle("IDLE1"));
#endif
#ifdef DBT_PROBE_LHDC_STAGE_PROFILE
    probe_lhdc_profile_live_enable();
#endif
    printf("{\"probe\":\"scheduler_layout\",\"host_core\":%u,\"controller_core\":%u,\"host_priority\":%u,\"decoder_core\":%u,\"decoder_priority\":%u,\"owner_ready_yield\":%u,\"renderer_core\":%u,\"renderer_priority\":%u,\"logger_core\":%u,\"background_core\":%u,\"logger_priority\":1}\n",(unsigned)xPortGetCoreID(),(unsigned)DBT_ESP32_BLUETOOTH_CORE,(unsigned)DBT_PROBE_HOST_PRIORITY,(unsigned)DBT_PROBE_DECODER_CORE,(unsigned)DBT_PROBE_DECODER_PRIORITY,(unsigned)DBT_PROBE_OWNER_READY_YIELD,(unsigned)DBT_ESP32_RENDER_CORE,(unsigned)DBT_PROBE_RENDER_PRIORITY,(unsigned)AUDIO_LOGGER_CORE,(unsigned)DBT_ESP32_BACKGROUND_CORE);
    printf("{\"probe\":\"sbc_probe_ready\",\"psm\":25,\"mtu\":%u,\"worker_bytes\":%u,\"profile_bytes\":%u,\"decoder_stack\":8192,"
           "\"logger_stack\":4096,\"codecs\":" AUDIO_CODECS_JSON ",\"diagnostic_only\":true,\"audio_sink_sdp_advertised\":" AUDIO_SDP_JSON ",\"i2s_output\":" AUDIO_OUTPUT_JSON ",\"rf_protocol_diagnostic\":" AUDIO_RF_DIAG_JSON "}\n",
           (unsigned)PROBE_A2DP_RX_MTU,(unsigned)sizeof(worker),(unsigned)sizeof(profile));
    return DBT_OK;
}
bool probe_audio_accept(void *context,const dbt_link_t *link,uint16_t psm){(void)context;return psm!=0x19 || (initialized && !restart_hold && dbt_a2dp_acceptor_accept(&profile,link));}
dbt_result_t probe_audio_connect_begin(const dbt_link_t *link,dbt_a2dp_channel_t *key){
    if(!initialized || restart_hold)return DBT_BUSY;
    return dbt_a2dp_acceptor_connect_begin(&profile,link,now_ms(),key);
}
dbt_result_t probe_audio_connect_status(const dbt_a2dp_channel_t *key){
    return initialized?dbt_a2dp_acceptor_connect_status(&profile,key):DBT_INVALID;
}
dbt_result_t probe_audio_connect_cancel(const dbt_a2dp_channel_t *key){
    return initialized?dbt_a2dp_acceptor_connect_cancel(&profile,key,now_ms()):DBT_INVALID;
}
dbt_result_t probe_audio_configure_begin(const dbt_a2dp_channel_t *key,dbt_avdtp_local_t *context,
    uint8_t local_seid,uint8_t remote_seid,const uint8_t *configuration,uint16_t size,uint32_t *token){
    if(!initialized || restart_hold)return DBT_BUSY;
    dbt_result_t ready=dbt_a2dp_acceptor_connect_status(&profile,key);
    if(ready!=DBT_OK)return ready==DBT_PENDING?DBT_BUSY:ready;
    return dbt_avdtp_sink_local_configure_begin(&profile.sink,context,local_seid,remote_seid,configuration,size,now_ms(),token);
}
dbt_result_t probe_audio_configure_cancel(dbt_avdtp_local_t *context,uint32_t token){
    return dbt_avdtp_local_cancel(context,token);
}
dbt_result_t probe_audio_configure_result(const dbt_avdtp_local_t *context,uint32_t token,dbt_avdtp_local_result_t *result){
    return dbt_avdtp_local_result(context,token,result);
}
dbt_result_t probe_audio_open_begin(const dbt_a2dp_channel_t *key,dbt_avdtp_local_t *context,uint32_t *token){
    if(!initialized || restart_hold)return DBT_BUSY;
    dbt_result_t ready=dbt_a2dp_acceptor_connect_status(&profile,key);
    if(ready!=DBT_OK)return ready==DBT_PENDING?DBT_BUSY:ready;
    return dbt_avdtp_sink_local_open_begin(&profile.sink,context,now_ms(),token);
}
static dbt_result_t local_stream_begin(const dbt_a2dp_channel_t *signaling,const dbt_a2dp_channel_t *media,
    dbt_avdtp_local_t *context,uint32_t *token,dbt_sink_operation_t operation){
    if(!initialized || restart_hold)return DBT_BUSY;
    dbt_result_t ready=dbt_a2dp_acceptor_connect_status(&profile,signaling);
    if(ready!=DBT_OK)return ready==DBT_PENDING?DBT_BUSY:ready;
    ready=dbt_a2dp_acceptor_media_status(&profile,media);
    if(ready!=DBT_OK)return ready==DBT_PENDING?DBT_BUSY:ready;
    if(operation==DBT_SINK_START)return dbt_avdtp_sink_local_start_begin(&profile.sink,context,media->serial,now_ms(),token);
    if(operation==DBT_SINK_SUSPEND)return dbt_avdtp_sink_local_suspend_begin(&profile.sink,context,media->serial,now_ms(),token);
    return operation==DBT_SINK_CLOSE?dbt_avdtp_sink_local_close_begin(&profile.sink,context,media->serial,now_ms(),token):DBT_UNSUPPORTED;
}
dbt_result_t probe_audio_start_begin(const dbt_a2dp_channel_t *signaling,const dbt_a2dp_channel_t *media,
    dbt_avdtp_local_t *context,uint32_t *token){return local_stream_begin(signaling,media,context,token,DBT_SINK_START);}
dbt_result_t probe_audio_suspend_begin(const dbt_a2dp_channel_t *signaling,const dbt_a2dp_channel_t *media,
    dbt_avdtp_local_t *context,uint32_t *token){return local_stream_begin(signaling,media,context,token,DBT_SINK_SUSPEND);}
dbt_result_t probe_audio_close_begin(const dbt_a2dp_channel_t *signaling,const dbt_a2dp_channel_t *media,
    dbt_avdtp_local_t *context,uint32_t *token){return local_stream_begin(signaling,media,context,token,DBT_SINK_CLOSE);}
dbt_result_t probe_audio_reconfigure_begin(const dbt_a2dp_channel_t *signaling,const dbt_a2dp_channel_t *media,
    dbt_avdtp_local_t *context,const uint8_t *configuration,uint16_t size,uint32_t *token){
    if(!initialized || restart_hold)return DBT_BUSY;
    dbt_result_t ready=dbt_a2dp_acceptor_connect_status(&profile,signaling);
    if(ready!=DBT_OK)return ready==DBT_PENDING?DBT_BUSY:ready;
    ready=dbt_a2dp_acceptor_media_status(&profile,media);
    if(ready!=DBT_OK)return ready==DBT_PENDING?DBT_BUSY:ready;
    return dbt_avdtp_sink_local_reconfigure_begin(&profile.sink,context,media->serial,configuration,size,now_ms(),token);
}
dbt_result_t probe_audio_abort_begin(const dbt_a2dp_channel_t *signaling,dbt_avdtp_local_t *context,uint32_t *token){
    return initialized && !restart_hold?dbt_a2dp_acceptor_abort_begin(&profile,signaling,context,now_ms(),token):DBT_BUSY;
}
dbt_result_t probe_audio_local_cancel(dbt_avdtp_local_t *context,uint32_t token){return dbt_avdtp_local_cancel(context,token);}
dbt_result_t probe_audio_local_result(const dbt_avdtp_local_t *context,uint32_t token,dbt_avdtp_local_result_t *result){
    return dbt_avdtp_local_result(context,token,result);
}
dbt_result_t probe_audio_media_begin(const dbt_a2dp_channel_t *signaling,dbt_a2dp_channel_t *key){
    return initialized && !restart_hold?dbt_a2dp_acceptor_media_begin(&profile,signaling,now_ms(),key):DBT_BUSY;
}
dbt_result_t probe_audio_media_status(const dbt_a2dp_channel_t *key){
    return initialized?dbt_a2dp_acceptor_media_status(&profile,key):DBT_INVALID;
}
dbt_result_t probe_audio_media_cancel(const dbt_a2dp_channel_t *key){
    return initialized?dbt_a2dp_acceptor_media_cancel(&profile,key,now_ms()):DBT_INVALID;
}
dbt_result_t probe_audio_query_begin(const dbt_a2dp_channel_t *key,dbt_avdtp_query_t *query,
    uint8_t command,uint8_t seid,uint8_t *response,uint16_t capacity,uint32_t *token){
    if(!initialized || restart_hold)return DBT_BUSY;
    dbt_result_t ready=dbt_a2dp_acceptor_connect_status(&profile,key);
    if(ready!=DBT_OK)return ready==DBT_PENDING?DBT_BUSY:ready;
    return dbt_avdtp_query_begin(query,&profile.sink.signal,command,seid,response,capacity,now_ms(),token);
}
dbt_result_t probe_audio_query_cancel(dbt_avdtp_query_t *query,uint32_t token){
    return dbt_avdtp_query_cancel(query,token,now_ms());
}
dbt_result_t probe_audio_query_result(const dbt_avdtp_query_t *query,uint32_t token,
    dbt_avdtp_query_result_t *result,const uint8_t **body){
    return dbt_avdtp_query_result(query,token,result,body);
}
static bool match_overlap(const void *a,unsigned an,const void *b,unsigned bn){
    uintptr_t x=(uintptr_t)a,y=(uintptr_t)b;return an && bn && (x<=y?y-x<an:x-y<bn);
}
static bool match_owned(const void *p,unsigned n){
    const dbt_avdtp_sink_t *sink=&profile.sink;const dbt_avdtp_t *signal=&sink->signal;
    return match_overlap(p,n,&profile,sizeof(profile)) ||
        (profile.runtime && match_overlap(p,n,profile.runtime,sizeof(*profile.runtime))) ||
        match_overlap(p,n,sink->configuration,sink->configuration_size) ||
        match_overlap(p,n,sink->pending.configuration,sink->pending.size) ||
        match_overlap(p,n,signal->rx,signal->rx_capacity) || match_overlap(p,n,signal->packet,signal->packet_size) ||
        match_overlap(p,n,signal->tx[0].data,signal->tx[0].size) || match_overlap(p,n,signal->tx[1].data,signal->tx[1].size);
}
static bool match_output_alias(uint8_t *configuration,uint16_t capacity,void *result,unsigned result_size,const void *p,unsigned bytes){
    return match_overlap(configuration,capacity,p,bytes) || match_overlap(result,result_size,p,bytes);
}
static bool match_buffers(uint8_t *configuration,uint16_t capacity,void *result,unsigned result_size,
    const dbt_a2dp_channel_t *key,const dbt_avdtp_query_t *discover,const dbt_avdtp_query_t *capabilities,const void *request,unsigned request_size){
    return key && discover && capabilities && request && result && (configuration || !capacity) &&
        !match_owned(configuration,capacity) && !match_owned(result,result_size) && !match_overlap(configuration,capacity,result,result_size) &&
        !match_output_alias(configuration,capacity,result,result_size,key,sizeof(*key)) &&
        !match_output_alias(configuration,capacity,result,result_size,request,request_size) &&
        !match_output_alias(configuration,capacity,result,result_size,discover,sizeof(*discover)) &&
        !match_output_alias(configuration,capacity,result,result_size,capabilities,sizeof(*capabilities)) &&
        !match_output_alias(configuration,capacity,result,result_size,discover->response,discover->capacity) &&
        !match_output_alias(configuration,capacity,result,result_size,capabilities->response,capabilities->capacity);
}
static bool match_receipt(const dbt_avdtp_query_result_t *r,uint32_t binding){
    return r->binding==binding && r->outcome==DBT_AVDTP_QUERY_SUCCESS && r->transport_outcome==DBT_AVDTP_RESPONSE &&
        r->response_seen && r->response_type==DBT_AVDTP_ACCEPT && r->payload_status==DBT_OK &&
        !r->cancel_requested && r->bytes==r->required;
}
static dbt_result_t match_view(const dbt_a2dp_channel_t *key,const dbt_avdtp_query_t *discover,uint32_t discover_token,
    const dbt_avdtp_query_t *capabilities,uint32_t capabilities_token,dbt_avdtp_sep_t *peer,const uint8_t **body,uint16_t *bytes){
    if(!initialized || restart_hold)return DBT_BUSY;
    dbt_result_t ready=dbt_a2dp_acceptor_connect_status(&profile,key);
    if(ready!=DBT_OK)return ready==DBT_PENDING?DBT_BUSY:ready;
    if(discover->signal!=&profile.sink.signal || capabilities->signal!=&profile.sink.signal)return DBT_EMPTY;
    dbt_avdtp_query_result_t d,c;const uint8_t *records;
    ready=dbt_avdtp_query_result(discover,discover_token,&d,&records);
    if(ready!=DBT_OK)return ready==DBT_PENDING?DBT_BUSY:ready;
    ready=dbt_avdtp_query_result(capabilities,capabilities_token,&c,body);
    if(ready!=DBT_OK)return ready==DBT_PENDING?DBT_BUSY:ready;
    if(!match_receipt(&d,key->serial) || !match_receipt(&c,key->serial) || d.command!=DBT_AVDTP_DISCOVER ||
       (c.command!=DBT_AVDTP_GET_CAPABILITIES && c.command!=DBT_AVDTP_GET_ALL_CAPABILITIES))return DBT_UNSUPPORTED;
    uint16_t cursor=0;
    for(unsigned i=0;i<62;++i){
        ready=dbt_avdtp_sep_next(records,d.bytes,&cursor,peer);
        if(ready!=DBT_OK)return ready;
        if(peer->seid==c.seid){*bytes=c.bytes;return DBT_OK;}
    }
    return DBT_EMPTY;
}
dbt_result_t probe_audio_match(const dbt_a2dp_channel_t *key,const dbt_avdtp_query_t *discover,uint32_t discover_token,
    const dbt_avdtp_query_t *capabilities,uint32_t capabilities_token,const dbt_a2dp_select_request_t *request,
    uint8_t *configuration,uint16_t capacity,dbt_a2dp_select_result_t *result){
    if(!match_buffers(configuration,capacity,result,sizeof(*result),key,discover,capabilities,request,sizeof(*request)))return DBT_INVALID;
    dbt_avdtp_sep_t peer;const uint8_t *body;uint16_t bytes;
    dbt_result_t ready=match_view(key,discover,discover_token,capabilities,capabilities_token,&peer,&body,&bytes);
    if(ready!=DBT_OK)return ready;
    return dbt_a2dp_select_exact(endpoints,(uint8_t)(sizeof(endpoints)/sizeof(endpoints[0])),&peer,body,bytes,request,configuration,capacity,result);
}
dbt_result_t probe_audio_generate(const dbt_a2dp_channel_t *key,const dbt_avdtp_query_t *discover,uint32_t discover_token,
    const dbt_avdtp_query_t *capabilities,uint32_t capabilities_token,const dbt_a2dp_generate_request_t *request,
    uint8_t *configuration,uint16_t capacity,dbt_a2dp_generate_result_t *result){
    if(!match_buffers(configuration,capacity,result,sizeof(*result),key,discover,capabilities,request,sizeof(*request)))return DBT_INVALID;
    dbt_avdtp_sep_t peer;const uint8_t *body;uint16_t bytes;
    dbt_result_t ready=match_view(key,discover,discover_token,capabilities,capabilities_token,&peer,&body,&bytes);
    if(ready!=DBT_OK)return ready;
    return dbt_a2dp_generate(endpoints,(uint8_t)(sizeof(endpoints)/sizeof(endpoints[0])),&peer,body,bytes,request,configuration,capacity,result);
}
dbt_result_t probe_audio_rank_begin(const dbt_a2dp_channel_t *key,dbt_a2dp_rank_t *context,dbt_a2dp_rank_config_t config,uint32_t *token){
    if(!context || !token || match_owned(context,sizeof(*context)) || match_owned(token,sizeof(*token)) ||
       match_owned(config.requests,(unsigned)config.request_count*sizeof(*config.requests)) ||
       match_owned(config.discovery,config.discovery_capacity) || match_owned(config.capabilities,config.capability_capacity))return DBT_INVALID;
    if(!initialized || restart_hold || failed || outgoing_rank || outgoing_connect)return DBT_BUSY;
    dbt_result_t ready=dbt_a2dp_acceptor_connect_status(&profile,key);
    if(ready!=DBT_OK)return ready==DBT_PENDING?DBT_BUSY:ready;
    if(profile.sink.state!=DBT_STREAM_IDLE || profile.sink.pending.used || profile.sink.local || profile.media.serial)return DBT_BUSY;
    ready=dbt_a2dp_rank_begin(context,&profile.sink.signal,endpoints,(uint8_t)(sizeof(endpoints)/sizeof(endpoints[0])),config,now_ms(),token);
    if(ready==DBT_PENDING)outgoing_rank=context;
    return ready;
}
dbt_result_t probe_audio_rank_cancel(dbt_a2dp_rank_t *context,uint32_t token){
    if(!context || !token)return DBT_INVALID;
    return outgoing_rank==context?dbt_a2dp_rank_cancel(context,token):DBT_EMPTY;
}
dbt_result_t probe_audio_rank_result(dbt_a2dp_rank_t *context,uint32_t token,dbt_a2dp_rank_result_t *result,const uint8_t **proposal){
    if(!context || !token || !result || !proposal || match_owned(result,sizeof(*result)) || match_owned(proposal,sizeof(*proposal)))return DBT_INVALID;
    if(outgoing_rank!=context)return DBT_EMPTY;
    dbt_result_t rc=dbt_a2dp_rank_result(context,token,result,proposal);
    if(rc==DBT_OK)outgoing_rank=0;
    return rc;
}
dbt_result_t probe_audio_dial_begin(dbt_a2dp_connect_t *context,dbt_gap_t *gap,
                                    dbt_a2dp_connect_config_t config,uint32_t *token){
    if(!context || !token)return DBT_INVALID;
    if(!initialized || restart_hold || failed || outgoing_connect || outgoing_rank)return DBT_BUSY;
    dbt_result_t result=probe_sdp_discovery_bind(&context->discovery);
    if(result!=DBT_OK)return result;
    result=dbt_a2dp_connect_begin(context,&profile,gap,config,now_ms(),token);
    if(result==DBT_PENDING){outgoing_connect=context;return result;}
    if(probe_sdp_discovery_unbind(&context->discovery)!=DBT_OK)return DBT_INVALID;
    return result;
}
dbt_result_t probe_audio_dial_cancel(dbt_a2dp_connect_t *context,uint32_t token){
    if(!context || !token)return DBT_INVALID;
    return outgoing_connect==context?dbt_a2dp_connect_cancel(context,token):DBT_EMPTY;
}
dbt_result_t probe_audio_dial_result(dbt_a2dp_connect_t *context,uint32_t token,dbt_a2dp_connect_result_t *result){
    if(!context || !token || !result)return DBT_INVALID;
    if(outgoing_connect!=context)return DBT_EMPTY;
    dbt_result_t status=dbt_a2dp_connect_result(outgoing_connect,token,result);
    if(status==DBT_OK){
        dbt_result_t detached=probe_sdp_discovery_unbind(&outgoing_connect->discovery);
        if(detached!=DBT_OK)return detached==DBT_BUSY?DBT_PENDING:DBT_INVALID;
        outgoing_connect=0;
    }
    return status;
}
void probe_audio_event(dbt_channel_event_t event,const dbt_channel_t *channel){
    probe_audio_event_ex(event,channel,DBT_CH_REASON_NORMAL,0);
}
void probe_audio_event_ex(dbt_channel_event_t event,const dbt_channel_t *channel,dbt_channel_reason_t reason,uint16_t wire_result){
    if(!initialized || !channel || channel->psm!=0x19)return;
    bool was_active=profile.active;
    bool signal=profile.signaling.serial && channel->serial==profile.signaling.serial &&
        channel->handle==profile.signaling.handle && channel->link_generation==profile.signaling.generation;
    uint16_t handle=channel->handle;uint32_t generation=channel->link_generation;
    dbt_result_t result=dbt_a2dp_acceptor_event_ex(&profile,event,channel,reason,wire_result);
    if(outgoing_connect)dbt_a2dp_connect_event(outgoing_connect,event,channel,reason,wire_result,now_ms());
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
    const dbt_rx_flow_t *flow=&profile.runtime->rx_flow;
    uint32_t classic_pending=0,le_pending=0,blocked=0;
    for(unsigned i=0;i<DBT_MAX_LINKS;++i){
        const dbt_rx_completed_t *p=&flow->pending[i];if(!p->used)continue;
        const dbt_link_t *link=dbt_acl_find(&profile.runtime->acl,p->handle);
        if(!link || link->generation!=p->generation)continue;
        if(link->kind==DBT_LINK_LE)le_pending+=p->packets;
        else{classic_pending+=p->packets;blocked+=p->packets-probe_audio_rx_credit_limit(NULL,link,p->packets,flow->packet_count);}
    }
    uint32_t releasable=classic_pending-blocked;
    bool gate=!classic_pending || releasable!=0;
    /* Valid shared-window counts fit the unused15 bits (currently three). */
    atomic_store(&credit_window_status,(uint32_t)flow->packet_count|(gate?0x10000u:0)|
        ((releasable>0x7fffu?0x7fffu:releasable)<<17));
    atomic_store(&credit_pending_classic,classic_pending);atomic_store(&credit_pending_le,le_pending);
#ifdef PROBE_OWNER_STATE_TRACE
    probe_owner_state_credits(classic_pending,le_pending,(uint32_t)esp_timer_get_time());
#endif
    atomic_store(&credit_acknowledged,flow->acknowledged);atomic_store(&credit_commands,flow->commands_submitted);atomic_store(&credit_partial_commands,flow->partial_commands);
    if(blocked>atomic_load(&credit_peak_blocked))atomic_store(&credit_peak_blocked,blocked);
    atomic_store(&credit_media_queued,atomic_load(&worker.media_written)-atomic_load(&worker.media_read));
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
    if(outgoing_connect){
        if(failed || restart_hold)(void)dbt_a2dp_connect_cancel(outgoing_connect,outgoing_connect->serial);
        dbt_a2dp_connect_poll(outgoing_connect,now);
    }
    if(outgoing_rank){
        if(failed || restart_hold)(void)dbt_a2dp_rank_cancel(outgoing_rank,outgoing_rank->serial);
        else if(!profile.active || profile.draining || profile.signaling.serial!=outgoing_rank->binding)
            (void)dbt_a2dp_rank_interrupt(outgoing_rank,outgoing_rank->serial,DBT_A2DP_RANK_SIGNAL_CHANGED);
        else if(profile.sink.state!=DBT_STREAM_IDLE || profile.sink.pending.used || profile.sink.local || profile.media.serial)
            (void)dbt_a2dp_rank_interrupt(outgoing_rank,outgoing_rank->serial,DBT_A2DP_RANK_CONFLICT);
        dbt_a2dp_rank_poll(outgoing_rank,now);
        if(outgoing_rank->result.needs_signal_retirement && profile.active && !profile.draining &&
           profile.signaling.serial==outgoing_rank->binding)(void)dbt_a2dp_acceptor_retire(&profile,now);
    }
    atomic_store(&l2cap_delivered,profile.runtime->rx.delivered);
    atomic_store(&l2cap_abandoned,profile.runtime->rx.abandoned);
    atomic_store(&l2cap_rejected,profile.runtime->rx.rejected);
    atomic_store(&l2cap_orphaned,profile.runtime->rx.orphaned);
    atomic_store(&l2cap_oversized,profile.runtime->rx.oversized);
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
    _Static_assert(sizeof(profile.runtime->channels.channels)/sizeof(profile.runtime->channels.channels[0])<256u,"Profile staged count must leave bit31 free");
    atomic_store(&profile_snapshot,(uint32_t)profile.sink.state|((uint32_t)profile.active<<8)|
        ((uint32_t)profile.enabled<<9)|((uint32_t)profile.draining<<10)|
        ((uint32_t)profile.sink.retiring<<11)|((uint32_t)profile.sink.drained<<12)|
        ((uint32_t)profile.sink.worker_live<<13)|((uint32_t)profile.bearer.active<<14)|
        ((uint32_t)profile.bearer.attached<<15)|((uint32_t)profile.sink.signal.pinned<<16)|
        ((uint32_t)profile.sink.pending.used<<17)|((uint32_t)profile.sink.pending.submitted<<18)|
        ((uint32_t)profile.sink.failed<<19)|((uint32_t)profile.sink.signal.failed<<20)|
        ((uint32_t)(profile.signaling.serial!=0)<<21)|((uint32_t)(profile.media.serial!=0)<<22)|(staged<<23)|((uint32_t)profile.sink.delay_enabled<<31));
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
        if(selected){
            uint32_t encoded_frames=0;bool known=dbt_codec_worker_buffered_frames(&worker,&encoded_frames);
            uint16_t delay=probe_output_buffered_delay_100us(config.rate,encoded_frames);
            dbt_result_t reported=dbt_avdtp_sink_delay(&profile.sink,delay,now);
            const probe_host_record_t record={.kind=PROBE_HOST_A2DP_DELAY,.value={encoded_frames,config.rate,(uint32_t)delay|((uint32_t)known<<16),(uint32_t)reported,profile.sink.binding}};
            (void)probe_audio_host_record(&record);
        }
        /* A request can observe a signaling expiry after acceptor_poll. Do not
         * go idle with a failed signal and no remaining owner deadline. */
        if(profile.sink.failed || profile.sink.signal.failed)(void)dbt_a2dp_acceptor_retire(&profile,now);
    }
#endif
}
uint32_t probe_audio_wait_ms(uint32_t now){
    if(!initialized)return UINT32_MAX;
    uint32_t wait=dbt_a2dp_acceptor_wait_ms(&profile,now);
    if(outgoing_connect){uint32_t next=dbt_a2dp_connect_wait_ms(outgoing_connect,now);if(next<wait)wait=next;}
    if(outgoing_rank){uint32_t next=dbt_a2dp_rank_wait_ms(outgoing_rank,now);if(next<wait)wait=next;}
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
    if(outgoing_connect)(void)dbt_a2dp_connect_cancel(outgoing_connect,outgoing_connect->serial);
    if(outgoing_rank)(void)dbt_a2dp_rank_cancel(outgoing_rank,outgoing_rank->serial);
    if(profile.sink.local)(void)dbt_avdtp_local_cancel(profile.sink.local,profile.sink.local->serial);
    dbt_result_t r=dbt_a2dp_acceptor_stop(&profile,now);restart_drained=r==DBT_OK;
    if(outgoing_connect && !dbt_a2dp_connect_done(outgoing_connect) && r==DBT_OK)r=DBT_BUSY;
    if(outgoing_rank && !dbt_a2dp_rank_done(outgoing_rank) && r==DBT_OK)r=DBT_BUSY;
    return failed?DBT_INVALID:r;
}
bool probe_audio_restart_quiescent(void){return restart_hold && restart_drained && !failed &&
    (!outgoing_connect || dbt_a2dp_connect_done(outgoing_connect)) &&
    (!outgoing_rank || dbt_a2dp_rank_done(outgoing_rank));}
