#include "probe_audio_trace.h"
#include <windows.h>
#include <assert.h>
#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
static probe_audio_trace_ring_t media_trace_rings[2];
static _Atomic uint32_t snapshot_format_lost;
static uint32_t clock_calls,clock_value;
static uint32_t clock_now(void *context){assert(context==&clock_calls);++clock_calls;return clock_value++;}
static char lines[128][1024];static unsigned line_count;
static _Atomic bool hold_writer;
static HANDLE writer_entered,writer_release;
static int capture(const char *format,va_list args){
    assert(line_count<128);int count=vsnprintf(lines[line_count],sizeof(lines[0]),format,args);
    assert(count>0 && (unsigned)count<sizeof(lines[0]));++line_count;
    if(atomic_load_explicit(&hold_writer,memory_order_acquire)){
        SetEvent(writer_entered);assert(WaitForSingleObject(writer_release,3000)==WAIT_OBJECT_0);
    }
    return count;
}
static void capture_snapshot(const char *format,...){va_list args;va_start(args,format);(void)capture(format,args);va_end(args);}
static int capture_printf(const char *format,...){va_list args;va_start(args,format);int count=capture(format,args);va_end(args);return count;}
static bool initialized;static void *owner,*logger_task,*current_task;
static _Atomic uint32_t pcm_codec;static unsigned timing_calls,timing_kind;
static void timing_event(unsigned kind){++timing_calls;timing_kind=kind;}
static int64_t esp_timer_get_time(void){++clock_calls;return clock_value++;}
static void *xTaskGetCurrentTaskHandle(void){return current_task;}
#define PROBE_SNAPSHOT_LOG(...) capture_snapshot(__VA_ARGS__)
#define printf capture_printf
#include "actual_trace_logger.inc"
#undef printf
typedef struct {uint32_t state,last_status;uint8_t classic_scan;bool classic_pending,advertising;} test_gap_t;
static unsigned startup_lines;
static int capture_startup(const char *format,...){(void)format;++startup_lines;return 0;}
#define HOST_PRINTF capture_startup
#define DBT_PROBE_AUDIO_TEST 1
#include "actual_main_records.inc"
static DWORD WINAPI logger_thread(void *context){(void)context;log_media_trace();return 0;}
int main(void){
    probe_audio_trace_ring_t ring;probe_audio_trace_init(&ring);probe_audio_trace_t out;
    probe_host_record_t state={.kind=PROBE_HOST_CLASSIC_VISIBILITY,.value={2,3,1,1,3}};
    dbt_codec_trace_t event={0};event.kind=DBT_CODEC_INPUT_CLOCK_MAX;event.sequence=65535;
    event.expected_sequence=65534;event.timestamp=UINT32_MAX;event.ssrc=0x12345678u;
    event.previous_ssrc=0x87654321u;event.bytes=1800;event.queued=8;event.result=DBT_OK;
    event.marker=true;event.payload_type=96;event.prefix[0]=4;event.prefix[1]=233;
    assert(!probe_audio_trace_pop(&ring,&out));
    assert(!probe_audio_trace_media(NULL,&event,9,clock_now,&clock_calls));
    assert(!probe_audio_trace_media(&ring,NULL,9,clock_now,&clock_calls));
    assert(!probe_audio_trace_media(&ring,&event,PROBE_AUDIO_TRACE_HOST,clock_now,&clock_calls));
    assert(probe_audio_trace_host(&ring,NULL,clock_now,&clock_calls)==DBT_INVALID);
    probe_host_record_t bad={.kind=99};assert(probe_audio_trace_host(&ring,&bad,clock_now,&clock_calls)==DBT_INVALID);
    assert(!clock_calls && !ring.ordinal && !atomic_load(&ring.lost));
    fixture_visibility((test_gap_t){.classic_scan=2},3,true,3);fixture_gap((test_gap_t){.state=2});
    assert(startup_lines==2 && !clock_calls);
    for(unsigned i=0;i<16;++i)assert(probe_audio_trace_host(&ring,&state,clock_now,&clock_calls)==DBT_OK);
    for(unsigned i=0;i<8;++i)assert(probe_audio_trace_host(&ring,&state,clock_now,&clock_calls)==DBT_NO_SPACE);
    assert(clock_calls==16 && ring.ordinal==24 && atomic_load(&ring.lost)==8);
    for(unsigned i=0;i<16;++i){assert(probe_audio_trace_pop(&ring,&out));assert(out.ordinal==i+1 && out.codec==PROBE_AUDIO_TRACE_HOST && out.host.value[4]==3);}
    assert(!probe_audio_trace_pop(&ring,&out));
    atomic_store(&ring.read,UINT32_MAX-7u);atomic_store(&ring.written,UINT32_MAX-7u);ring.ordinal=UINT32_MAX-4u;
    uint32_t ordinal=ring.ordinal;
    for(unsigned i=0;i<16;++i){
        if(i&1u)assert(probe_audio_trace_media(&ring,&event,9,clock_now,&clock_calls));
        else assert(probe_audio_trace_host(&ring,&state,clock_now,&clock_calls)==DBT_OK);
    }
    state.value[4]=0;event.prefix[1]=0;event.timestamp=0;
    for(unsigned i=0;i<16;++i){
        assert(probe_audio_trace_pop(&ring,&out));assert(out.ordinal==++ordinal);
        if(i&1u){assert(out.codec==9 && out.event.kind==DBT_CODEC_INPUT_CLOCK_MAX && out.event.sequence==65535);
            assert(out.event.expected_sequence==65534 && out.event.timestamp==UINT32_MAX && out.event.ssrc==0x12345678u);
            assert(out.event.previous_ssrc==0x87654321u && out.event.bytes==1800 && out.event.queued==8);
            assert(out.event.marker && out.event.payload_type==96 && out.event.prefix[0]==4 && out.event.prefix[1]==233);
        }else assert(out.codec==PROBE_AUDIO_TRACE_HOST && out.host.value[4]==3);
    }
    probe_audio_trace_init(&media_trace_rings[0]);probe_audio_trace_init(&media_trace_rings[1]);
    atomic_store(&pcm_codec,9);event.kind=DBT_CODEC_INPUT_CLOCK_MAX;media_trace(NULL,&event);
    assert(atomic_load(&media_trace_rings[0].written)==1 && !atomic_load(&media_trace_rings[1].written));
    assert(probe_audio_trace_pop(&media_trace_rings[0],&out) && out.codec==9 && out.event.kind==DBT_CODEC_INPUT_CLOCK_MAX);
    event.kind=DBT_CODEC_PCM_BUSY;media_trace(NULL,&event);
    assert(probe_audio_trace_pop(&media_trace_rings[1],&out) && out.event.kind==DBT_CODEC_PCM_BUSY);
    event.kind=DBT_CODEC_DECODE_BEGIN;media_trace(NULL,&event);
    assert(timing_calls==1 && timing_kind==DBT_CODEC_DECODE_BEGIN);
    assert(!probe_audio_trace_pop(&media_trace_rings[0],&out) && !probe_audio_trace_pop(&media_trace_rings[1],&out));
    probe_audio_trace_init(&media_trace_rings[0]);probe_audio_trace_init(&media_trace_rings[1]);clock_value=123456;
    int owner_id,logger_id,foreign_id;owner=&owner_id;logger_task=&logger_id;current_task=owner;
    assert(probe_audio_host_record(&state)==DBT_UNSUPPORTED);initialized=true;current_task=&foreign_id;
    assert(probe_audio_host_record(&state)==DBT_UNSUPPORTED);current_task=owner;
    assert(probe_audio_host_record(NULL)==DBT_INVALID);
    state.value[4]=3;fixture_visibility((test_gap_t){.classic_scan=2,.classic_pending=true},3,true,3);
    probe_host_record_t gap={.kind=PROBE_HOST_GAP_STATE,.value={2,17,1,3,0}};
    fixture_gap((test_gap_t){.state=2,.last_status=17,.advertising=true,.classic_scan=3});log_media_trace();
    assert(startup_lines==2);
    assert(line_count==2 && strstr(lines[0],"\"probe\":\"classic_visibility\"") && strstr(lines[0],"\"event_us\":123456"));
    assert(strstr(lines[0],"\"completed_scan\":2") && strstr(lines[0],"\"requested_scan\":3"));
    assert(strstr(lines[0],"\"command_pending\":true") && strstr(lines[0],"\"pairing_open\":true") && strstr(lines[0],"\"pairing_origin\":3"));
    assert(strstr(lines[1],"\"probe\":\"gap_state\"") && strstr(lines[1],"\"controller_status\":17"));
    assert(strstr(lines[1],"\"profiles_ready\":false") && strstr(lines[1],"\"advertising\":true") && strstr(lines[1],"\"classic_scan\":3"));
    writer_entered=CreateEventA(NULL,TRUE,FALSE,NULL);writer_release=CreateEventA(NULL,TRUE,FALSE,NULL);assert(writer_entered && writer_release);
    assert(probe_audio_trace_host(&media_trace_rings[0],&state,clock_now,&clock_calls)==DBT_OK);
    atomic_store(&hold_writer,true);HANDLE thread=CreateThread(NULL,0,logger_thread,NULL,0,NULL);assert(thread);
    assert(WaitForSingleObject(writer_entered,3000)==WAIT_OBJECT_0);
    /* The actual logger has copied/released the first slot before UART work. */
    unsigned calls_before=clock_calls;
    for(unsigned i=0;i<16;++i){gap.value[1]=100u+i;assert(probe_audio_trace_host(&media_trace_rings[0],&gap,clock_now,&clock_calls)==DBT_OK);}
    fixture_gap((test_gap_t){.state=2,.last_status=17,.advertising=true,.classic_scan=3});
    assert(startup_lines==2);
    assert(clock_calls==calls_before+16 && atomic_load(&media_trace_rings[0].lost)==1);
    assert(strstr(lines[2],"\"pairing_origin\":3"));
    atomic_store_explicit(&hold_writer,false,memory_order_release);SetEvent(writer_release);
    assert(WaitForSingleObject(thread,3000)==WAIT_OBJECT_0);CloseHandle(thread);
    for(unsigned i=0;i<8;++i)log_media_trace();
    assert(atomic_load(&media_trace_rings[0].read)==atomic_load(&media_trace_rings[0].written));
    assert(line_count==19 && !atomic_load(&snapshot_format_lost));
    CloseHandle(writer_entered);CloseHandle(writer_release);
    printf("PASS bounded shared trace: full/drop/wrap/copy, actual JSON logger, blocked UART consumer, unchanged slot size=%zu ring=%zu\n",sizeof(probe_audio_trace_t),sizeof(probe_audio_trace_ring_t));
    return 0;
}
