#include "probe_audio_trace.h"
#include "dbt_runtime.h"
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
static HANDLE writer_entered,writer_release,part_entered,part_release;
static _Atomic bool hold_part;static unsigned part_calls;
void probe_audio_trace_test_part(unsigned part){
    ++part_calls;
    if(!part && atomic_load_explicit(&hold_part,memory_order_acquire)){SetEvent(part_entered);assert(WaitForSingleObject(part_release,3000)==WAIT_OBJECT_0);}
}
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
static dbt_codec_worker_t worker;
static dbt_result_t submit_result=DBT_OK;
dbt_result_t dbt_codec_worker_submit(void *context,const dbt_sink_job_t *job){
    assert(context==&worker && job);return submit_result;
}
static void timing_event(unsigned kind){++timing_calls;timing_kind=kind;}
static int64_t esp_timer_get_time(void){++clock_calls;return clock_value++;}
static void *xTaskGetCurrentTaskHandle(void){return current_task;}
#define MALLOC_CAP_INTERNAL 1u
#define MALLOC_CAP_8BIT 2u
static uint32_t memory_values[4]={123456,54321,111111,217248};
static _Atomic uint32_t memory_reads[4],stack_reads;static void *last_stack_task;
static _Atomic bool hold_heap;static HANDLE heap_entered,heap_release;
static size_t heap_caps_get_free_size(unsigned caps){assert(caps==3);atomic_fetch_add(&memory_reads[0],1);if(atomic_load(&hold_heap)){SetEvent(heap_entered);assert(WaitForSingleObject(heap_release,3000)==WAIT_OBJECT_0);}return memory_values[0];}
static size_t heap_caps_get_largest_free_block(unsigned caps){assert(caps==3);atomic_fetch_add(&memory_reads[1],1);return memory_values[1];}
static size_t heap_caps_get_minimum_free_size(unsigned caps){assert(caps==3);atomic_fetch_add(&memory_reads[2],1);return memory_values[2];}
static size_t heap_caps_get_total_size(unsigned caps){assert(caps==3);atomic_fetch_add(&memory_reads[3],1);return memory_values[3];}
static unsigned uxTaskGetStackHighWaterMark(void *task){last_stack_task=task;atomic_fetch_add(&stack_reads,1);assert(!task || task==owner);return task?444u:333u;}
#define PROBE_SNAPSHOT_LOG(...) capture_snapshot(__VA_ARGS__)
#define printf capture_printf
#include "actual_trace_logger.inc"
#undef printf
typedef struct {uint32_t state,last_status;uint8_t classic_scan;bool classic_pending,advertising;} test_gap_t;
typedef struct {uint32_t fault,controller_status,pending_opcode;struct {_Atomic uint32_t rejected;} ingress;uint32_t fault_stage,fault_packet_size,fault_packet_type,fault_event;} test_host_t;
typedef struct {struct {uint32_t failed_opcode;} rx_flow;dbt_runtime_failure_t failure;} test_runtime_t;
static unsigned startup_lines;
static char startup_text[16][1024];
static int capture_startup(const char *format,...){assert(startup_lines<16);va_list args;va_start(args,format);int count=vsnprintf(startup_text[startup_lines],sizeof(startup_text[0]),format,args);va_end(args);assert(count>0 && count<1024);++startup_lines;return count;}
#define HOST_PRINTF capture_startup
#define DBT_PROBE_AUDIO_TEST 1
#include "actual_main_records.inc"
static DWORD WINAPI logger_thread(void *context){(void)context;log_media_trace();return 0;}
static probe_host_controller_t partial_controller;
static probe_host_failure_t partial_failure;static dbt_result_t partial_result;
static DWORD WINAPI failure_thread(void *context){(void)context;partial_result=probe_audio_host_failure(&partial_failure);return 0;}
static DWORD WINAPI controller_thread(void *context){(void)context;partial_result=probe_audio_host_controller(&partial_controller);return 0;}
int main(void){
    probe_audio_trace_ring_t ring;probe_audio_trace_init(&ring);probe_audio_trace_record_t out;
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
#ifdef DBT_PROBE_SCHEDULER_TRACE
    assert(probe_audio_trace_pop(&media_trace_rings[1],&out) && out.event.kind==DBT_CODEC_PCM_BUSY);
#else
    assert(!probe_audio_trace_pop(&media_trace_rings[1],&out) && !atomic_load(&media_trace_rings[1].lost));
#endif
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
    /* Three-slot admission is all-or-nothing for every free-space boundary,
     * including physical and uint32 cursor wrap. Caller storage is reusable. */
    probe_host_failure_t failure={0},saved_failure;
    uint32_t failure_words[24];for(unsigned i=0;i<24;++i)failure_words[i]=0xabc00000u+i;
    memcpy(&failure,failure_words,sizeof(failure));saved_failure=failure;
    for(unsigned used=0;used<=16;++used){
        probe_audio_trace_init(&ring);atomic_store(&ring.read,UINT32_MAX-1u);atomic_store(&ring.written,UINT32_MAX-1u);
        ring.ordinal=UINT32_MAX-5u;
        for(unsigned i=0;i<used;++i)assert(probe_audio_trace_host(&ring,&state,clock_now,&clock_calls)==DBT_OK);
        unsigned before_clock=clock_calls,before_parts=part_calls;uint32_t before_write=atomic_load(&ring.written);
        assert(probe_audio_trace_failure(&ring,&failure,clock_now,&clock_calls)==(used<=13?DBT_OK:DBT_NO_SPACE));
        assert(atomic_load(&ring.written)==before_write+(used<=13?3u:0u));
        assert(clock_calls==before_clock+(used<=13?1u:0u) && part_calls==before_parts+(used<=13?3u:0u));
        for(unsigned i=0;i<used;++i)assert(probe_audio_trace_pop(&ring,&out) && out.codec==PROBE_AUDIO_TRACE_HOST);
        if(used<=13){assert(probe_audio_trace_pop(&ring,&out) && out.codec==PROBE_AUDIO_TRACE_FAILURE);assert(!memcmp(&out.failure,&saved_failure,sizeof(saved_failure)));}
        assert(!probe_audio_trace_pop(&ring,&out) && atomic_load(&ring.lost)==(used<=13?0u:1u));
    }
    for(uint32_t marker=PROBE_AUDIO_TRACE_CORRUPT;;++marker){assert(!probe_audio_trace_media(&ring,&event,marker,clock_now,&clock_calls));if(marker==UINT32_MAX)break;}
    /* No failure prefix is visible before the final cursor publication. */
    probe_audio_trace_init(&media_trace_rings[0]);partial_failure=saved_failure;
    part_entered=CreateEventA(NULL,TRUE,FALSE,NULL);part_release=CreateEventA(NULL,TRUE,FALSE,NULL);assert(part_entered && part_release);
    atomic_store(&hold_part,true);HANDLE partial=CreateThread(NULL,0,failure_thread,NULL,0,NULL);assert(partial);
    assert(WaitForSingleObject(part_entered,3000)==WAIT_OBJECT_0);
    assert(!atomic_load(&media_trace_rings[0].written) && !probe_audio_trace_pop(&media_trace_rings[0],&out));
    atomic_store(&hold_part,false);SetEvent(part_release);assert(WaitForSingleObject(partial,3000)==WAIT_OBJECT_0);
    assert(partial_result==DBT_OK);memset(&partial_failure,0,sizeof(partial_failure));
    assert(probe_audio_trace_pop(&media_trace_rings[0],&out) && out.codec==PROBE_AUDIO_TRACE_FAILURE && !memcmp(&out.failure,&saved_failure,sizeof(saved_failure)));
    CloseHandle(partial);CloseHandle(part_entered);CloseHandle(part_release);
    /* Incomplete/mismatched private groups never read unpublished slots. */
    probe_audio_trace_init(&ring);memset(ring.items,0,sizeof(ring.items));ring.items[0].codec=PROBE_AUDIO_TRACE_FAILURE;
    atomic_store(&ring.written,1);assert(probe_audio_trace_pop(&ring,&out) && out.codec==PROBE_AUDIO_TRACE_CORRUPT && atomic_load(&ring.read)==1);
    probe_audio_trace_init(&ring);assert(probe_audio_trace_failure(&ring,&saved_failure,clock_now,&clock_calls)==DBT_OK);
    ring.items[2].at_us^=1u;assert(probe_audio_trace_pop(&ring,&out) && out.codec==PROBE_AUDIO_TRACE_CORRUPT);
    assert(probe_audio_trace_pop(&ring,&out) && out.codec==PROBE_AUDIO_TRACE_CORRUPT);
    assert(probe_audio_trace_pop(&ring,&out) && out.codec==PROBE_AUDIO_TRACE_CORRUPT && !probe_audio_trace_pop(&ring,&out));
    test_host_t host={.fault=5,.controller_status=17,.pending_opcode=4660,.fault_stage=7,.fault_packet_size=1029,.fault_packet_type=2,.fault_event=19};
    atomic_init(&host.ingress.rejected,UINT32_C(0xfedcba98));
    test_runtime_t runtime={.rx_flow={64513},.failure={.at_ms=UINT32_MAX-13u,.origin=6,.result=3,.handle=291,.generation=UINT32_C(0xabcdef01),.link_present=true,.cid=17767,.bytes=1800,.declared=4096,.packet_type=2,.event=62,.pb=2,.bc=1,.code=11,.identifier=239}};
    unsigned boot_before=startup_lines;initialized=false;fixture_failure(&host,runtime);assert(startup_lines==boot_before+2);initialized=true;
    probe_audio_trace_init(&media_trace_rings[0]);probe_audio_trace_init(&media_trace_rings[1]);clock_value=UINT32_MAX-1u;
    unsigned failure_lines=line_count;fixture_failure(&host,runtime);host.controller_status=99;runtime.failure.handle=0;
    assert(line_count==failure_lines);log_media_trace();assert(line_count==failure_lines+2);
    printf("FAILURE_JSON %s",lines[failure_lines]);printf("FAILURE_JSON %s",lines[failure_lines+1]);
    /* The logger releases all three slots before either UART line can wait. */
    writer_entered=CreateEventA(NULL,TRUE,FALSE,NULL);writer_release=CreateEventA(NULL,TRUE,FALSE,NULL);assert(writer_entered && writer_release);
    unsigned stalled_failure_lines=line_count;fixture_failure(&host,runtime);atomic_store(&hold_writer,true);thread=CreateThread(NULL,0,logger_thread,NULL,0,NULL);assert(thread);
    assert(WaitForSingleObject(writer_entered,3000)==WAIT_OBJECT_0);calls_before=clock_calls;
    for(unsigned i=0;i<16;++i)assert(probe_audio_host_record(&state)==DBT_OK);
    unsigned direct_before=startup_lines;fixture_failure(&host,runtime);
    assert(startup_lines==direct_before && clock_calls==calls_before+16 && atomic_load(&media_trace_rings[0].lost)==1);
    atomic_store(&hold_writer,false);SetEvent(writer_release);assert(WaitForSingleObject(thread,3000)==WAIT_OBJECT_0);CloseHandle(thread);
    for(unsigned i=0;i<8;++i)log_media_trace();assert(atomic_load(&media_trace_rings[0].written)==atomic_load(&media_trace_rings[0].read));
    assert(strstr(lines[stalled_failure_lines],"\"controller_status\":99") && strstr(lines[stalled_failure_lines+1],"\"handle\":0"));
    printf("FAILURE_STALLED_JSON %s",lines[stalled_failure_lines]);printf("FAILURE_STALLED_JSON %s",lines[stalled_failure_lines+1]);
    CloseHandle(writer_entered);CloseHandle(writer_release);
    probe_host_controller_t controller_copy={.hci_version=8,.manufacturer=96,.host_bytes=6696,.runtime_bytes=11168,
        .classic_acl_bytes=1021,.classic_acl_packets=9,.le_acl_bytes=251,.le_acl_packets=10,.shared_acl_pool=0,.address={0x6c,0xc8,0x40,0x42,0xbf,0x66}};
    for(unsigned used=0;used<=16;++used){
        probe_audio_trace_init(&ring);atomic_store(&ring.read,UINT32_MAX-1u);atomic_store(&ring.written,UINT32_MAX-1u);
        for(unsigned i=0;i<used;++i)assert(probe_audio_trace_host(&ring,&state,clock_now,&clock_calls)==DBT_OK);
        unsigned before_clock=clock_calls,before_parts=part_calls;uint32_t before_write=atomic_load(&ring.written);
        assert(probe_audio_trace_controller(&ring,&controller_copy,clock_now,&clock_calls)==(used<=14?DBT_OK:DBT_NO_SPACE));
        assert(clock_calls==before_clock+(used<=14?1u:0u) && part_calls==before_parts+(used<=14?2u:0u));
        assert(atomic_load(&ring.written)==before_write+(used<=14?2u:0u));
        for(unsigned i=0;i<used;++i)assert(probe_audio_trace_pop(&ring,&out) && out.codec==PROBE_AUDIO_TRACE_HOST);
        if(used<=14){assert(probe_audio_trace_pop(&ring,&out) && out.codec==PROBE_AUDIO_TRACE_CONTROLLER);assert(!memcmp(&out.controller,&controller_copy,sizeof(controller_copy)));}
        assert(!probe_audio_trace_pop(&ring,&out) && atomic_load(&ring.lost)==(used<=14?0u:1u));
    }
    assert(!probe_audio_trace_media(&ring,&event,PROBE_AUDIO_TRACE_CONTROLLER,clock_now,&clock_calls));
    probe_audio_trace_init(&media_trace_rings[0]);partial_controller=controller_copy;
    part_entered=CreateEventA(NULL,TRUE,FALSE,NULL);part_release=CreateEventA(NULL,TRUE,FALSE,NULL);assert(part_entered && part_release);
    atomic_store(&hold_part,true);partial=CreateThread(NULL,0,controller_thread,NULL,0,NULL);assert(partial);
    assert(WaitForSingleObject(part_entered,3000)==WAIT_OBJECT_0);assert(!atomic_load(&media_trace_rings[0].written) && !probe_audio_trace_pop(&media_trace_rings[0],&out));
    atomic_store(&hold_part,false);SetEvent(part_release);assert(WaitForSingleObject(partial,3000)==WAIT_OBJECT_0);assert(partial_result==DBT_OK);
    memset(&partial_controller,0,sizeof(partial_controller));assert(probe_audio_trace_pop(&media_trace_rings[0],&out) && !memcmp(&out.controller,&controller_copy,sizeof(controller_copy)));
    CloseHandle(partial);CloseHandle(part_entered);CloseHandle(part_release);
    static dbt_host_t real_host;static dbt_runtime_t real_runtime;
    real_host.controller.hci_version=8;real_host.controller.manufacturer=96;real_host.controller.classic_acl_bytes=1021;real_host.controller.classic_acl_packets=9;
    real_host.controller.le_acl_bytes=251;real_host.controller.le_acl_packets=10;real_host.controller.shared_acl_pool=false;
    const uint8_t controller_le[]={0x66,0xbf,0x42,0x40,0xc8,0x6c};memcpy(real_host.controller.address_le,controller_le,6);
    unsigned memory_boot=startup_lines;initialized=false;fixture_memory();fixture_controller(&real_host,real_runtime);initialized=true;
    printf("MEMORY_LEGACY_JSON %s",startup_text[memory_boot]);printf("CONTROLLER_LEGACY_JSON %s",startup_text[memory_boot+1]);
    for(unsigned i=0;i<4;++i)atomic_store(&memory_reads[i],0);atomic_store(&stack_reads,0);
    probe_audio_trace_init(&media_trace_rings[0]);probe_audio_trace_init(&media_trace_rings[1]);clock_value=1420000;unsigned memory_line=line_count;
    fixture_memory();fixture_controller(&real_host,real_runtime);
    for(unsigned i=0;i<4;++i)assert(!atomic_load(&memory_reads[i]));assert(!atomic_load(&stack_reads));
    real_host.controller.manufacturer=7;memset(real_host.controller.address_le,0,6);
    memory_values[0]=222222;memory_values[1]=22222;memory_values[2]=33333;memory_values[3]=44444;
    log_media_trace();assert(line_count==memory_line+2 && last_stack_task==owner);
    printf("MEMORY_DEFERRED_JSON %s",lines[memory_line]);printf("CONTROLLER_DEFERRED_JSON %s",lines[memory_line+1]);
    /* A blocked background heap read cannot make owner publication query it. */
    heap_entered=CreateEventA(NULL,TRUE,FALSE,NULL);heap_release=CreateEventA(NULL,TRUE,FALSE,NULL);assert(heap_entered && heap_release);
    fixture_memory();atomic_store(&hold_heap,true);thread=CreateThread(NULL,0,logger_thread,NULL,0,NULL);assert(thread);
    assert(WaitForSingleObject(heap_entered,3000)==WAIT_OBJECT_0);uint32_t memory_before=atomic_load(&memory_reads[0]);
    for(unsigned i=0;i<16;++i)assert(probe_audio_host_record(&state)==DBT_OK);
    direct_before=startup_lines;fixture_memory();fixture_controller(&real_host,real_runtime);
    assert(startup_lines==direct_before && atomic_load(&memory_reads[0])==memory_before && atomic_load(&media_trace_rings[0].lost)==2);
    atomic_store(&hold_heap,false);SetEvent(heap_release);assert(WaitForSingleObject(thread,3000)==WAIT_OBJECT_0);CloseHandle(thread);
    for(unsigned i=0;i<8;++i)log_media_trace();CloseHandle(heap_entered);CloseHandle(heap_release);
    probe_audio_trace_init(&media_trace_rings[0]);probe_audio_trace_init(&media_trace_rings[1]);
    probe_host_record_t delay_value={.kind=PROBE_HOST_A2DP_DELAY,.value={13440,192000,881u|(1u<<16),DBT_OK,7}};
    unsigned delay_line=line_count;assert(probe_audio_host_record(&delay_value)==DBT_OK);log_media_trace();
    assert(line_count==delay_line+1);printf("DELAY_ESTIMATE_JSON %s",lines[delay_line]);
    probe_host_record_t cccd_value={.kind=PROBE_HOST_CCCD,.value={0,6,0xfedcba98u,3,1,14,0}};
    unsigned cccd_line=line_count;assert(probe_audio_host_record(&cccd_value)==DBT_OK);
    cccd_value.value[4]=5;log_media_trace();
    assert(line_count==cccd_line+1);printf("CCCD_JSON %s",lines[cccd_line]);
    atomic_store(&worker.gate_epoch,7);atomic_store(&worker.started_epoch,4);
    dbt_sink_job_t job={.binding=10,.token=20,.operation=DBT_SINK_START};
    unsigned job_line=line_count;assert(sink_work(&worker,&job)==DBT_OK);log_media_trace();
    assert(line_count==job_line+1);printf("A2DP_JOB_JSON %s",lines[job_line]);
    probe_host_record_t smp_value={.kind=PROBE_HOST_SMP,.value={0,6,7,(uint32_t)-2,5u|(13u<<8),0xfedcba98u,9}};
    unsigned smp_line=line_count;assert(probe_audio_host_record(&smp_value)==DBT_OK);
    smp_value.value[4]=0;log_media_trace();assert(line_count==smp_line+1);
    printf("SMP_JSON %s",lines[smp_line]);
    submit_result=DBT_BUSY;job_line=line_count;
    assert(sink_work(&worker,&job)==DBT_BUSY);log_media_trace();assert(line_count==job_line);
    printf("PASS deferred heap collection: owner/full never query allocator, background uses fixed owner handle, controller properties immutable\n");
    printf("PASS atomic failure group:24 fields,3 existing slots,all space/wrap boundaries,partial publication,malformed groups,startup/full policy,blocked UART\n");
    printf("PASS bounded shared trace: full/drop/wrap/copy, actual JSON logger, blocked UART consumer, unchanged slot size=%zu ring=%zu\n",sizeof(probe_audio_trace_t),sizeof(probe_audio_trace_ring_t));
    return 0;
}
