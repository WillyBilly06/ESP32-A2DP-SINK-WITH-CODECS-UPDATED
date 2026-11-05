#include <stdlib.h>
#include <string.h>
#include "../../esp32_probe/main/probe_audio.c"
#ifdef DBT_PROBE_APTX_BUILD
#include "freeaptx.h"
#endif
#ifdef _WIN32
#define EX __declspec(dllexport)
#else
#define EX __attribute__((visibility("default")))
#endif
#define CHECK(x) do {if(!(x))return __LINE__;}while(0)
static unsigned live_allocs,fail_alloc,owner_wakes;
static int created_decoder_core=-1;
static dbt_result_t stop_result=DBT_OK;
static uint32_t now;
static bool snapshot_for_test(char *line,size_t capacity,uint32_t began,const char *format,...){
    va_list arguments;va_start(arguments,format);bool ok=snapshot_vformat(line,capacity,began,format,arguments);va_end(arguments);return ok;
}
EX int probe_audio_format_record(char *line,unsigned capacity){
    return snapshot_for_test(line,capacity,1000,"{\"probe\":\"snapshot_test\",\"value\":%u}\n",7u);
}
EX int probe_audio_snapshot_records(void){
    char line[1024],large[1000];now=30;
    CHECK(snapshot_crc32("123456789",9)==0xcbf43926u && snapshot_crc32("",0)==0);
    CHECK(snapshot_for_test(line,sizeof(line),1000,"{\"probe\":\"snapshot_test\",\"value\":%u}\n",7u));
    CHECK(strstr(line,"\"snapshot_begin_us\":1000") && strstr(line,"\"at_us\":30000") && strstr(line,"\"snapshot_span_us\":29000"));
    char *checksum=strstr(line,",\"record_crc32\":\"");unsigned stored=0;
    CHECK(checksum && sscanf(checksum,",\"record_crc32\":\"%x",&stored)==1);
    CHECK(stored==snapshot_crc32(line,(size_t)(checksum-line)));
    line[2]^=1;CHECK(stored!=snapshot_crc32(line,(size_t)(checksum-line)));line[2]^=1;
    now=1;CHECK(snapshot_for_test(line,sizeof(line),UINT32_MAX-1999u,"{\"probe\":\"wrap\"}\n"));
    CHECK(strstr(line,"\"at_us\":1000") && strstr(line,"\"snapshot_span_us\":3000"));
    memset(large,'x',sizeof(large)-1);large[sizeof(large)-1]=0;
    CHECK(!snapshot_for_test(line,sizeof(line),0,"{\"probe\":\"large\",\"value\":\"%s\"}\n",large));
    CHECK(!snapshot_for_test(line,64,0,"{\"probe\":\"small\"}\n"));
    CHECK(!snapshot_for_test(line,sizeof(line),0,"not json\n"));
    CHECK(atomic_load(&snapshot_format_lost)==3);atomic_store(&snapshot_format_lost,0);
    now=30;record_watchdog(1);CHECK(atomic_load(&watchdog_events)==1 && atomic_load(&watchdog_hook_us)==30000 && atomic_load(&watchdog_isr_core)==1);
    now=31;record_watchdog(0);CHECK(atomic_load(&watchdog_events)==2 && atomic_load(&watchdog_hook_us)==31000 && !atomic_load(&watchdog_isr_core));
    atomic_store(&watchdog_events,0);atomic_store(&watchdog_hook_us,0);atomic_store(&watchdog_isr_core,0);now=0;return 0;
}
static unsigned product_events;static probe_audio_product_event_t last_product_event;
static uint16_t product_handle;static uint32_t product_generation,product_rate;
static void product_event(void *ctx,probe_audio_product_event_t e,uint16_t h,uint32_t g,uint32_t r,uint32_t t){
    (void)ctx;(void)t;product_events++;last_product_event=e;product_handle=h;product_generation=g;product_rate=r;
}
const dbt_link_t *dbt_acl_find(const dbt_acl_t *acl,uint16_t handle){
    for(unsigned i=0;i<DBT_MAX_LINKS;i++)if(acl->links[i].used && acl->links[i].handle==handle)return &acl->links[i];
    return 0;
}
#ifdef DBT_PROBE_AVRCP
static probe_avrcp_snapshot_t fake_metadata;
static bool snapshot_busy;
dbt_result_t probe_avrcp_snapshot(probe_avrcp_snapshot_t *out){if(snapshot_busy)return DBT_BUSY;*out=fake_metadata;return DBT_OK;}
EX int probe_audio_metadata(void){
    char dst[PROBE_AVRCP_TEXT_BYTES*6u+1u];
    const char sample[]={'A','"','\\','\n',1,(char)0xc3,(char)0xa9,0};
    metadata_escape(dst,sample);
    CHECK(!strcmp(dst,"A\\\"\\\\\\u000a\\u0001\xc3\xa9"));
    memset(dst,0,sizeof(dst));char full[PROBE_AVRCP_TEXT_BYTES];memset(full,1,sizeof(full));
    metadata_escape(dst,full);CHECK(strlen(dst)==PROBE_AVRCP_TEXT_BYTES*6u && dst[sizeof(dst)-1]==0);
    memcpy(fake_metadata.title,sample,sizeof(sample));fake_metadata.metadata_updates=1;fake_metadata.generation=4;
    CHECK(log_metadata() && !log_metadata());snapshot_busy=true;fake_metadata.metadata_updates=2;
    CHECK(!log_metadata());snapshot_busy=false;fail_alloc=1;CHECK(!log_metadata() && !live_allocs);
    CHECK(log_metadata() && !log_metadata() && !live_allocs);return 0;
}
#endif
EX int probe_audio_diagnostics(void){
    memset(media_trace_rings,0,sizeof(media_trace_rings));
    dbt_codec_trace_t e={1234,9,8,10,9,77,8,DBT_CODEC_QUEUE_FULL,DBT_NO_SPACE};
    now=10;
    for(unsigned i=0;i<TRACE_SLOTS+1u;++i){e.sequence=(uint16_t)i;media_trace(0,&e);}
    CHECK(atomic_load(&media_trace_rings[0].written)==TRACE_SLOTS && atomic_load(&media_trace_rings[0].lost)==1);
    CHECK(media_trace_rings[0].items[0].event.timestamp==1234 && media_trace_rings[0].items[0].at_us==10000);
    CHECK(media_trace_rings[0].items[TRACE_SLOTS-1].ordinal==TRACE_SLOTS);
    e.kind=DBT_CODEC_DECODE_REJECT;media_trace(0,&e);
    CHECK(atomic_load(&media_trace_rings[1].written)==1 && !atomic_load(&media_trace_rings[1].lost));
    log_media_trace();
    CHECK(atomic_load(&media_trace_rings[0].read)==2 && atomic_load(&media_trace_rings[1].read)==1);
    e.kind=DBT_CODEC_INPUT_SEQUENCE;media_trace(0,&e);
    CHECK(media_trace_rings[0].items[0].ordinal==TRACE_SLOTS+2u && atomic_load(&media_trace_rings[0].lost)==1);
    probe_audio_transport_received(9000,100,DBT_H4_ACL,DBT_OK,false);
    CHECK(atomic_load(&rx_packets)==1 && atomic_load(&rx_acl)==1 && atomic_load(&rx_callback_max_us)==1000);
    now=12;probe_audio_transport_poll_begin(12000);now=14;probe_audio_transport_poll_end(12000);
    CHECK(atomic_load(&first_rx_to_poll_max_us)==3000 && atomic_load(&host_poll_max_us)==2000 && !atomic_load(&rx_pending_us));
    probe_audio_transport_received(13000,20,DBT_H4_EVENT,DBT_NO_SPACE,true);
    CHECK(atomic_load(&rx_packets)==2 && atomic_load(&rx_events)==1 && atomic_load(&rx_isr)==1 && atomic_load(&rx_rejected)==1);
    probe_audio_transport_poll_begin(12000); /* stale caller timestamp after preemption */
    CHECK(atomic_load(&first_rx_to_poll_max_us)==3000);
    e.kind=DBT_CODEC_DECODE_BEGIN;now=20;media_trace(0,&e);
    e.kind=DBT_CODEC_PCM_BEGIN;now=21;media_trace(0,&e);
    e.kind=DBT_CODEC_PCM_END;now=22;media_trace(0,&e);
    e.kind=DBT_CODEC_DECODE_END;now=24;media_trace(0,&e);
    CHECK(atomic_load(&phase_timing[0].calls)==1 && atomic_load(&phase_timing[0].elapsed_us)==4000 && atomic_load(&phase_timing[0].hist[5])==1);
    CHECK(atomic_load(&phase_timing[2].calls)==1 && atomic_load(&phase_timing[2].elapsed_us)==1000 && atomic_load(&phase_timing[2].hist[3])==1);
    probe_audio_owner_wait((1u<<0)|(1u<<14));probe_audio_owner_wait(1u<<2);
    CHECK(atomic_load(&owner_iterations)==2 && atomic_load(&owner_immediate[0])==1 && atomic_load(&owner_immediate[2])==1);
    CHECK(atomic_load(&owner_immediate[14])==1 && !atomic_load(&owner_immediate[1]));
    dbt_codec_config_t config={0};config.rate=192000;config.channels=2;config.sample_bytes=4;config.ops=&dbt_sbc_codec;config.cie_size=13;
    for(unsigned i=0;i<13;++i)config.cie[i]=(uint8_t)i;
    CHECK(publish_stream(true,&config)==DBT_OK && atomic_load(&codec_revision)==1 && atomic_load(&codec_cie_size)==13);
    CHECK(atomic_load(&codec_cie_words[0])==0x03020100u && atomic_load(&codec_cie_words[2])==0x0b0a0908u && atomic_load(&codec_cie_words[3])==12);
    config.cie_size=26;for(unsigned i=0;i<26;++i)config.cie[i]=(uint8_t)i;
    CHECK(publish_stream(true,&config)==DBT_OK && atomic_load(&codec_revision)==2 && atomic_load(&codec_cie_words[6])==0x1918u);
    config.cie_size=6;CHECK(publish_stream(true,&config)==DBT_OK && atomic_load(&codec_revision)==3);
    CHECK(atomic_load(&codec_cie_words[1])==0x00000504u && !atomic_load(&codec_cie_words[2]) && !atomic_load(&codec_cie_words[3]));
    CHECK(!atomic_load(&codec_cie_words[6]));
    memset(media_trace_rings,0,sizeof(media_trace_rings));now=0;return 0;
}
EX int probe_audio_wait_policy(void){
    dbt_codec_worker_t saved=worker;
    memset(&worker,0,sizeof(worker));
    CHECK(decoder_wait_ticks(100)==portMAX_DELAY);
    worker.control_phase=1;worker.control_since=100;
    CHECK(decoder_wait_ticks(1500)==pdMS_TO_TICKS(100));
    CHECK(decoder_wait_ticks(1600)==0 && decoder_wait_ticks(1601)==0);
    worker.control_since=0xfffffff0u;
    CHECK(decoder_wait_ticks(0xfffffff5u)==pdMS_TO_TICKS(1495));
    CHECK(decoder_wait_ticks((uint32_t)(0xfffffff0u+1500u))==0);
    worker.control_timedout=true;CHECK(decoder_wait_ticks(0)==portMAX_DELAY);
    worker=saved;return 0;
}
#ifdef DBT_PROBE_AUDIO_OUTPUT
static bool output_started;
static uint32_t output_rate,output_frames;
static uint8_t output_channels;
static dbt_pcm_format_t output_format;
static dbt_audio_t output_audio;
static uint8_t *output_allocate(void *ctx,uint32_t n){(void)ctx;return heap_caps_malloc(n,MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);}
static void output_release(void *ctx,uint8_t *p){(void)ctx;heap_caps_free(p);}
dbt_result_t probe_output_init(void (*wake)(void *),void *ctx){(void)wake;(void)ctx;return dbt_audio_init(&output_audio,(dbt_audio_memory_t){0,output_allocate,output_release});}
dbt_result_t probe_output_stream_format(bool start,uint32_t rate,uint8_t channels,dbt_pcm_format_t format){
    if(!start){dbt_audio_close(&output_audio);output_started=false;return DBT_OK;}
    dbt_audio_config_t c={rate,2048,8192,3072,format,channels,false};
    dbt_result_t r=dbt_audio_open(&output_audio,c);if(r==DBT_OK){output_started=true;output_rate=rate;output_channels=channels;output_format=format;}
    return r;
}
dbt_result_t probe_output_pcm_format(const void *samples,uint16_t frames,uint8_t channels,uint32_t rate,dbt_pcm_format_t format){
    if(!samples || !output_started || rate!=output_rate || channels!=output_channels || format!=output_format)return DBT_INVALID;
    dbt_result_t r=dbt_audio_submit(&output_audio,samples,frames);if(r==DBT_OK)output_frames+=frames;return r;
}
#ifdef DBT_PROBE_ASYNC_AUDIO
static uint32_t pending_ticket,next_ticket;static bool pending_start,cancelled;
static dbt_codec_config_t pending_config;
dbt_result_t probe_output_stream_begin(bool start,uint32_t rate,uint8_t channels,dbt_pcm_format_t format,uint32_t *ticket){
    *ticket=0;if(pending_ticket)return DBT_BUSY;
    pending_ticket=++next_ticket;pending_start=start;cancelled=false;
    pending_config=(dbt_codec_config_t){0};pending_config.rate=rate;pending_config.channels=channels;pending_config.sample_bytes=(uint8_t)format;
    *ticket=pending_ticket;return DBT_PENDING;
}
dbt_result_t probe_output_request_poll(uint32_t ticket){
    if(!ticket || ticket!=pending_ticket)return DBT_INVALID;
    pending_ticket=0;
    if(cancelled)return DBT_CANCELLED;
    return probe_output_stream_format(pending_start,pending_config.rate,pending_config.channels,(dbt_pcm_format_t)pending_config.sample_bytes);
}
dbt_result_t probe_output_request_cancel(uint32_t ticket){if(!ticket || ticket!=pending_ticket)return DBT_INVALID;cancelled=true;return DBT_PENDING;}
dbt_result_t probe_output_try_pcm_format(const void *samples,uint16_t frames,uint8_t channels,uint32_t rate,dbt_pcm_format_t format){return probe_output_pcm_format(samples,frames,channels,rate,format);}
#endif
EX int probe_audio_formats(void){
    const uint8_t packed24[]={0,0,0x80,0xff,0xff,0x7f,1,2,3,0xfd,0xfe,0xff};
    const int32_t pcm32[]={INT32_MIN,INT32_MAX,123456,-123456};
    dbt_codec_config_t c={0};c.rate=48000;c.channels=2;c.samples=2;c.sample_bytes=3;
    CHECK(probe_output_stream_format(true,48000,2,DBT_PCM_S24_LE)==DBT_OK);
    CHECK(pcm(0,packed24,2,&c,0)==DBT_OK && output_audio.input.stride==6 && !memcmp(output_audio.input.bytes,packed24,sizeof(packed24)));
    CHECK(probe_output_pcm_format(packed24,2,2,44100,DBT_PCM_S24_LE)==DBT_INVALID);
    CHECK(probe_output_pcm_format(packed24,2,1,48000,DBT_PCM_S24_LE)==DBT_INVALID);
    CHECK(probe_output_pcm_format(packed24,2,2,48000,DBT_PCM_S32_LE)==DBT_INVALID);
    CHECK(probe_output_stream_format(false,0,0,0)==DBT_OK);
    c.sample_bytes=4;CHECK(probe_output_stream_format(true,48000,2,DBT_PCM_S32_LE)==DBT_OK);
    CHECK(pcm(0,pcm32,2,&c,0)==DBT_OK && output_audio.input.stride==8 && !memcmp(output_audio.input.bytes,pcm32,sizeof(pcm32)));
    CHECK(probe_output_stream_format(false,0,0,0)==DBT_OK && !live_allocs);return 0;
}
void probe_output_stats(probe_output_stats_t *stats){memset(stats,0,sizeof(*stats));}
uint16_t probe_output_delay_100us(uint32_t rate){return rate?640:0;}
#endif
void *heap_caps_malloc(size_t n,unsigned flags) {
    (void)flags;if(fail_alloc){fail_alloc--;return 0;}
#ifdef TEST_EIGHT_MOD16
    /* Exercise the codec allocator contract with an alternate 8-byte-aligned
     * base address, rather than relying on the platform malloc alignment. */
    void *base=malloc(n+32);if(!base)return 0;
    uintptr_t address=(((uintptr_t)base+sizeof(void *)+15u)&~(uintptr_t)15u)+8u;
    ((void **)address)[-1]=base;void *p=(void *)address;
#else
    void *p=malloc(n);
#endif
    if(p)live_allocs++;return p;
}
void *heap_caps_aligned_alloc(size_t alignment,size_t n,unsigned flags){
    if(alignment>8)return 0;return heap_caps_malloc(n,flags);
}
void heap_caps_free(void *p) {if(p){live_allocs--;
#ifdef TEST_EIGHT_MOD16
    free(((void **)p)[-1]);
#else
    free(p);
#endif
}}
size_t heap_caps_get_free_size(unsigned f){(void)f;return 123456;}
size_t heap_caps_get_minimum_free_size(unsigned f){(void)f;return 123000;}
size_t heap_caps_get_largest_free_block(unsigned f){(void)f;return 120000;}
int64_t esp_timer_get_time(void){return (int64_t)now*1000;}
void xTaskNotifyGive(TaskHandle_t task){(void)task;owner_wakes++;}
void taskYIELD(void){} unsigned ulTaskNotifyTake(int c,unsigned t){(void)c;(void)t;return 0;}
void vTaskDelay(unsigned t){(void)t;}void vTaskDelete(TaskHandle_t t){(void)t;}
unsigned uxTaskGetStackHighWaterMark(TaskHandle_t t){(void)t;return 4096;}
int xTaskCreatePinnedToCore(void (*f)(void *),const char *n,unsigned bytes,void *a,unsigned p,TaskHandle_t *task,int core){(void)f;(void)bytes;(void)a;(void)p;if(!strcmp(n,"native_decoder"))created_decoder_core=core;*task=(void *)1;return pdPASS;}
dbt_result_t dbt_a2dp_acceptor_init(dbt_a2dp_acceptor_t *p,dbt_runtime_t *r,dbt_avdtp_sink_hooks_t h,const dbt_avdtp_endpoint_t *e,uint8_t n,uint16_t mtu,uint16_t c,uint32_t a,uint32_t b,uint32_t d){(void)h;(void)mtu;(void)c;(void)a;(void)b;(void)d;p->runtime=r;return e==endpoints&&n==sizeof(endpoints)/sizeof(endpoints[0])?DBT_OK:DBT_INVALID;}
bool dbt_a2dp_acceptor_accept(const dbt_a2dp_acceptor_t *p,const dbt_link_t *l){(void)p;(void)l;return true;}
dbt_result_t dbt_a2dp_acceptor_event(dbt_a2dp_acceptor_t *p,dbt_channel_event_t e,const dbt_channel_t *c){
    if(e==DBT_CH_EVENT_OPEN && !p->active){p->active=true;p->signaling=(dbt_a2dp_channel_t){.serial=c->serial,.generation=c->link_generation,.handle=c->handle};}
    else if((e==DBT_CH_EVENT_CLOSED || e==DBT_CH_EVENT_FAILED) && p->signaling.serial==c->serial)p->signaling.serial=0;
    return DBT_OK;
}
dbt_result_t dbt_a2dp_acceptor_receive(dbt_a2dp_acceptor_t *p,const dbt_channel_t *c,const uint8_t *d,uint16_t n){(void)p;(void)c;(void)d;(void)n;return DBT_OK;}
dbt_result_t dbt_a2dp_acceptor_poll(dbt_a2dp_acceptor_t *p,uint32_t t){(void)p;(void)t;return DBT_OK;}
uint32_t dbt_a2dp_acceptor_wait_ms(const dbt_a2dp_acceptor_t *p,uint32_t t){(void)p;(void)t;return UINT32_MAX;}
dbt_result_t dbt_a2dp_acceptor_complete(dbt_a2dp_acceptor_t *p,uint32_t b,uint32_t t,dbt_sink_work_result_t r,uint32_t n){(void)p;(void)b;(void)t;(void)r;(void)n;return DBT_OK;}
dbt_result_t dbt_a2dp_acceptor_retire(dbt_a2dp_acceptor_t *p,uint32_t t){(void)p;(void)t;return DBT_OK;}
dbt_result_t dbt_a2dp_acceptor_stop(dbt_a2dp_acceptor_t *p,uint32_t t){(void)t;p->enabled=false;return stop_result;}
static dbt_avdtp_sink_t sink;
static uint8_t label,wire[128];static uint16_t wire_n;
static dbt_result_t request(uint8_t id,const uint8_t *body,uint16_t size){
    uint8_t bytes[128];bytes[0]=(uint8_t)((label++&15u)<<4);bytes[1]=id;if(size)memcpy(bytes+2,body,size);
    return dbt_avdtp_receive(&sink.signal,123,bytes,(uint16_t)(size+2),++now);
}
static dbt_result_t output(void){
    const uint8_t *p;uint16_t n;dbt_result_t r=dbt_avdtp_tx_peek(&sink.signal,&p,&n);
    if(r!=DBT_OK)return r;if(n>sizeof(wire))return DBT_NO_SPACE;memcpy(wire,p,n);wire_n=n;return dbt_avdtp_tx_commit(&sink.signal,++now);
}
static int complete(void){
    CHECK(dbt_avdtp_sink_poll(&sink,++now)==DBT_OK);
    dbt_codec_receipt_t receipt;dbt_result_t ready=DBT_EMPTY;
    for(unsigned i=0;i<8 && ready==DBT_EMPTY;++i){(void)dbt_codec_worker_step(&worker,++now,0);ready=dbt_codec_worker_receipt(&worker,&receipt);}
    CHECK(ready==DBT_OK && receipt.result==DBT_SINK_WORK_READY);
    CHECK(dbt_avdtp_sink_complete(&sink,receipt.binding,receipt.token,receipt.result,++now)==DBT_OK);
    CHECK(output()==DBT_OK && (wire[0]&3)==2);return 0;
}
#if defined(DBT_PROBE_AAC_BUILD) || defined(DBT_PROBE_APTX_BUILD)
static int reject_codec(uint8_t seid,const uint8_t *cie,uint8_t size,bool allocation_failure){
    uint8_t config[64]={0};config[0]=(uint8_t)(seid<<2);config[1]=8;config[2]=1;config[3]=0;config[4]=7;config[5]=size;
    memcpy(config+6,cie,size);config[6+size]=8;
    CHECK(request(3,config,(uint16_t)(8+size))==DBT_OK);
    if(allocation_failure){
        CHECK(dbt_avdtp_sink_poll(&sink,++now)==DBT_OK);fail_alloc=1;(void)dbt_codec_worker_step(&worker,++now,0);
        dbt_codec_receipt_t receipt;CHECK(dbt_codec_worker_receipt(&worker,&receipt)==DBT_OK && receipt.result==DBT_SINK_WORK_REJECTED);
        CHECK(dbt_avdtp_sink_complete(&sink,receipt.binding,receipt.token,receipt.result,++now)==DBT_OK);
    }
    CHECK(output()==DBT_OK && (wire[0]&3)==3 && sink.state==DBT_STREAM_IDLE && !atomic_load(&worker.resident_bytes) && !live_allocs);
    return 0;
}
#endif
static uint32_t hash_bytes(const uint8_t *data,unsigned n){uint32_t h=2166136261u;for(unsigned i=0;i<n;i++)h=(h^data[i])*16777619u;return h;}
static int run_codec(uint8_t seid,const uint8_t *cie,uint8_t cie_size,const uint8_t *frame,uint16_t n,uint16_t expected_samples,uint32_t codec_id,
                     bool prefix,uint8_t sample_bytes,const uint8_t *expected,uint16_t expected_bytes,uint32_t rate){
    uint8_t config[64]={0};config[0]=(uint8_t)(seid<<2);config[1]=8;config[2]=1;config[3]=0;config[4]=7;config[5]=cie_size;
    memcpy(config+6,cie,cie_size);config[6+cie_size]=8;config[7+cie_size]=0;
    CHECK(request(3,config,(uint16_t)(8+cie_size))==DBT_OK && complete()==0 && sink.state==DBT_STREAM_CONFIGURED);
    CHECK(atomic_load(&worker.resident_bytes)>0 && worker.decoder.config.ops && worker.decoder.config.samples==expected_samples);
    uint8_t ep=(uint8_t)(seid<<2);CHECK(request(6,&ep,1)==DBT_OK && complete()==0);
    CHECK(dbt_avdtp_sink_media_open(&sink,456,++now)==DBT_OK);
    const uint8_t premature[]={0x80,96,0,1,0,0,0,1,0,0,0,1,1,0};
    CHECK(dbt_avdtp_sink_media_receive(&sink,456,premature,sizeof(premature))==DBT_BUSY);
    CHECK(request(7,&ep,1)==DBT_OK && complete()==0 && sink.state==DBT_STREAM_STREAMING);
    CHECK(atomic_load(&pcm_codec)==codec_id && atomic_load(&pcm_rate)==rate && atomic_load(&pcm_sample_bytes)==sample_bytes);
    uint8_t packet[700]={0x80,96,0,1,0,0,0,1,0,0,0,1};if(codec_id==4)packet[1]|=0x80;uint16_t offset=12;
    if(prefix)packet[offset++]=1;CHECK((unsigned)n+offset<=sizeof(packet));memcpy(packet+offset,frame,n);
    CHECK(dbt_avdtp_sink_media_receive(&sink,456,packet,(uint16_t)(n+offset))==DBT_OK);
    CHECK(dbt_codec_worker_step(&worker,++now,1)==1);
    for(unsigned i=0;i<128 && atomic_load(&worker.media_read)!=atomic_load(&worker.media_written);++i)(void)dbt_codec_worker_step(&worker,++now,1);
    CHECK(atomic_load(&worker.media_read)==atomic_load(&worker.media_written) && atomic_load(&pcm_frames)>0 && atomic_load(&pcm_channels)==2);
    uint32_t callbacks_before=atomic_load(&pcm_frames);
    if(expected){CHECK(atomic_load(&pcm_bytes)==expected_bytes && atomic_load(&pcm_hash)==hash_bytes(expected,expected_bytes));}
#ifdef DBT_PROBE_AUDIO_OUTPUT
    if(expected){CHECK(output_audio.input.stride==2u*sample_bytes && !memcmp(output_audio.input.bytes,expected,expected_bytes));}
#endif
#ifdef DBT_PROBE_AUDIO_OUTPUT
    CHECK(output_started && output_rate==rate && output_channels==2 && output_format==sample_bytes && output_frames>=expected_samples);
#endif
    packet[3]=2;CHECK(dbt_avdtp_sink_media_receive(&sink,456,packet,(uint16_t)(n+offset))==DBT_OK);
    CHECK(request(9,&ep,1)==DBT_OK && complete()==0 && sink.state==DBT_STREAM_OPEN);
    CHECK(dbt_codec_worker_step(&worker,++now,1)==0 && atomic_load(&pcm_frames)==callbacks_before);
    CHECK(request(8,&ep,1)==DBT_OK && complete()==0 && dbt_avdtp_sink_media_closed(&sink,456)==DBT_OK);
    CHECK(!atomic_load(&worker.resident_bytes) && !worker.decoder.state && !worker.packet_storage);
#ifdef DBT_PROBE_AUDIO_OUTPUT
    CHECK(!output_started);
#endif
    return 0;
}
EX int probe_audio_profiles(const uint8_t *sbc,uint16_t n,const uint8_t *sbc_cie,const uint8_t *aac,uint16_t aac_n,const uint8_t *aac44,uint16_t aac44_n){
    static dbt_runtime_t runtime;runtime.ready=true;

    CHECK(probe_audio_init(&runtime,(void *)2)==DBT_OK && initialized && !live_allocs);
    CHECK(created_decoder_core==DBT_PROBE_DECODER_CORE);
    dbt_link_t restart_peer={0};stop_result=DBT_BUSY;
    CHECK(probe_audio_restart_hold(true,++now)==DBT_BUSY && !probe_audio_restart_quiescent());
    CHECK(!probe_audio_accept(0,&restart_peer,0x19));
    CHECK(probe_audio_init(&runtime,(void *)2)==DBT_BUSY);
    stop_result=DBT_OK;probe_audio_poll(++now);CHECK(probe_audio_restart_quiescent());
    CHECK(probe_audio_restart_hold(false,++now)==DBT_OK && !probe_audio_restart_quiescent());
    CHECK(probe_audio_accept(0,&restart_peer,0x19));
    uint16_t handle=0;uint32_t generation=0;
    CHECK(probe_audio_peer(&handle,&generation)==DBT_EMPTY);
    runtime.acl.links[0]=(dbt_link_t){.used=true,.kind=DBT_LINK_CLASSIC,.handle=12,.generation=8};
    profile.active=true;profile.signaling=(dbt_a2dp_channel_t){.serial=3,.generation=8,.handle=12};
    CHECK(probe_audio_peer(&handle,&generation)==DBT_OK && handle==12 && generation==8);
    CHECK(probe_audio_peer_link(handle,generation)==&runtime.acl.links[0]);
    runtime.acl.links[0].generation=9;CHECK(probe_audio_peer(&handle,&generation)==DBT_EMPTY && !probe_audio_peer_link(12,8));
    runtime.acl.links[0].kind=DBT_LINK_LE;CHECK(!probe_audio_peer_link(12,9));
    profile.active=false;memset(&profile.signaling,0,sizeof(profile.signaling));memset(&runtime.acl,0,sizeof(runtime.acl));
    probe_audio_product_observer(product_event,0);
    dbt_channel_t ch={.psm=0x19,.serial=4,.handle=42,.link_generation=30};
    probe_audio_event(DBT_CH_EVENT_OPEN,&ch);
    CHECK(product_events==1 && last_product_event==PROBE_AUDIO_CONNECTED && product_handle==42 && product_generation==30);
    dbt_channel_t media=ch;media.serial=5;probe_audio_event(DBT_CH_EVENT_OPEN,&media);probe_audio_event(DBT_CH_EVENT_CLOSED,&media);CHECK(product_events==1);
    profile.sink.pending.used=true;profile.sink.pending.operation=DBT_SINK_CONFIGURE;
    sink_event(0,DBT_SINK_STATE,DBT_STREAM_CONFIGURED);
    CHECK(product_events==2 && last_product_event==PROBE_AUDIO_CONFIGURED && product_rate==48000);
    profile.sink.pending.used=false;sink_event(0,DBT_SINK_STATE,DBT_STREAM_STREAMING);
    CHECK(product_events==3 && last_product_event==PROBE_AUDIO_STREAMING);
    probe_audio_event(DBT_CH_EVENT_CLOSED,&ch);CHECK(product_events==4 && last_product_event==PROBE_AUDIO_DISCONNECTED);
    profile.active=false;memset(&profile.signaling,0,sizeof(profile.signaling));memset(&profile.sink,0,sizeof(profile.sink));probe_audio_product_observer(0,0);
    dbt_avdtp_sink_hooks_t hooks={&worker,allocate,release,dbt_codec_worker_submit,sink_event,dbt_codec_worker_media};
    CHECK(dbt_avdtp_sink_init(&sink,hooks,endpoints,(uint8_t)(sizeof(endpoints)/sizeof(endpoints[0])),123,672,672,64,5000,2000,5000)==DBT_OK);
#ifdef DBT_PROBE_OPUS_BUILD
    CHECK(request(1,0,0)==DBT_OK && output()==DBT_OK && wire_n==2u+2u*(sizeof(endpoints)/sizeof(endpoints[0])) && wire[2]==4 && wire[4]==8 && wire[6]==12);
    uint8_t ep=8;CHECK(request(12,&ep,1)==DBT_OK && output()==DBT_OK && wire_n==sizeof(android_capabilities)+2 && !memcmp(wire+2,android_capabilities,sizeof(android_capabilities)));
    /* Vendor mismatch is rejected at the selected endpoint before worker allocation. */
    uint8_t wrong[]={8,8,1,0,7,9,0,0xff,0xe1,0,0,0,1,0,0x92,8,0};
    CHECK(request(3,wrong,sizeof(wrong))==DBT_OK && output()==DBT_OK && (wire[0]&3)==3 && !atomic_load(&worker.resident_bytes));
    /* Codec construction OOM returns a rejected response and releases staging. */
    uint8_t oom[]={8,8,1,0,7,9,0,0xff,0xe0,0,0,0,1,0,0x92,8,0};
    CHECK(request(3,oom,sizeof(oom))==DBT_OK && dbt_avdtp_sink_poll(&sink,++now)==DBT_OK);
    fail_alloc=1;(void)dbt_codec_worker_step(&worker,++now,0);dbt_codec_receipt_t reject;
    CHECK(dbt_codec_worker_receipt(&worker,&reject)==DBT_OK && reject.result==DBT_SINK_WORK_REJECTED);
    CHECK(dbt_avdtp_sink_complete(&sink,reject.binding+1,reject.token,reject.result,++now)==DBT_EMPTY);
    CHECK(dbt_avdtp_sink_complete(&sink,reject.binding,reject.token,reject.result,++now)==DBT_OK);
    CHECK(output()==DBT_OK && (wire[0]&3)==3 && !atomic_load(&worker.resident_bytes) && !live_allocs);
    static const uint8_t opus_silence[]={0xf8,0xff,0xfe};
    static const uint8_t android[]={0,0xff,0xe0,0,0,0,1,0,0x92};
    static const uint8_t opus05[]={0,0xff,0xf1,5,0,0,5,0x10,2,1,3,0,0,0,8,0,0,0,0,0,0,0,0,0,0,0};
#else
    CHECK(request(1,0,0)==DBT_OK && output()==DBT_OK && wire_n==4 && wire[2]==4);
#endif
    int r=run_codec(1,sbc_cie,6,sbc,n,128,1,true,2,0,0,48000);if(r)return r;
#ifdef DBT_PROBE_OPUS_BUILD
    r=run_codec(2,android,sizeof(android),opus_silence,sizeof(opus_silence),960,2,true,2,0,0,48000);if(r)return r;
    r=run_codec(3,opus05,sizeof(opus05),opus_silence,sizeof(opus_silence),960,3,true,2,0,0,48000);if(r)return r;
#endif
#ifdef DBT_PROBE_AAC_BUILD
    static const uint8_t aac_cie[]={0,2,0x40,0,0x84,0,0,0};
    static const uint8_t aac_unsupported[]={0,2,0x20,0,0x84,0,0,0};
    r=reject_codec(4,aac_unsupported,sizeof(aac_unsupported),false);if(r)return r;
    r=reject_codec(4,aac_cie,sizeof(aac_cie),true);if(r)return r;
    CHECK(aac && aac_n);r=run_codec(4,aac_cie,sizeof(aac_cie),aac,aac_n,1024,4,false,2,0,0,48000);if(r)return r;
    static const uint8_t aac44_cie[]={0,2,0x40,1,4,0,0,0};
    CHECK(aac44 && aac44_n);r=run_codec(4,aac44_cie,sizeof(aac44_cie),aac44,aac44_n,1024,4,false,2,0,0,44100);if(r)return r;
#else
    (void)aac;(void)aac_n;(void)aac44;(void)aac44_n;
#endif
#ifdef DBT_PROBE_APTX_BUILD
    static const uint8_t caps[3][13]={{0,0xff,0x4f,0,0,0,1,0,0x12},
                                     {0,0xff,0xd7,0,0,0,0x24,0,0x12,0,0,0,0},
                                     {0,0xff,0x0a,0,0,0,2,0,0x12,0}};
    static const uint8_t sizes[]={9,13,10};
    static const uint8_t aptx_ll_reverse[]={0,0xff,0x0a,0,0,0,2,0,0x12,1};
    r=reject_codec(5,caps[1],sizes[1],false);if(r)return r;
    r=reject_codec(7,aptx_ll_reverse,sizeof(aptx_ll_reverse),false);if(r)return r;
    r=reject_codec(6,caps[1],sizes[1],true);if(r)return r;
    for(unsigned i=0;i<3;i++){
        bool hd=i==1;struct aptx_context *enc=aptx_init(hd),*ref=aptx_init(hd);CHECK(enc && ref);
        uint8_t src[768],encoded[192],expected[1024];for(unsigned j=0;j<sizeof(src);j++)src[j]=(uint8_t)(j*37u+19u);
        size_t encoded_n=0,expected_n=0;CHECK(aptx_encode(enc,src,sizeof(src),encoded,sizeof(encoded),&encoded_n)==sizeof(src));
        CHECK(encoded_n && aptx_decode32(ref,encoded,encoded_n,expected,sizeof(expected),&expected_n)==encoded_n && expected_n);
        aptx_finish(enc);aptx_finish(ref);
        r=run_codec((uint8_t)(5+i),caps[i],sizes[i],encoded,(uint16_t)encoded_n,4,5u+i,false,4,expected,(uint16_t)expected_n,48000);if(r)return r;
        uint8_t at44[13];memcpy(at44,caps[i],sizes[i]);at44[8]=0x22;
        r=run_codec((uint8_t)(5+i),at44,sizes[i],encoded,(uint16_t)encoded_n,4,5u+i,false,4,expected,(uint16_t)expected_n,44100);if(r)return r;
    }
#endif
    CHECK(dbt_avdtp_sink_shutdown(&sink,++now)==DBT_OK && !live_allocs);return 0;
}
