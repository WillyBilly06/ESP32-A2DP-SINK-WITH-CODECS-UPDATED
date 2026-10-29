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
static uint32_t now;
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
int xTaskCreatePinnedToCore(void (*f)(void *),const char *n,unsigned bytes,void *a,unsigned p,TaskHandle_t *task,int core){(void)f;(void)n;(void)bytes;(void)a;(void)p;(void)core;*task=(void *)1;return pdPASS;}
dbt_result_t dbt_a2dp_acceptor_init(dbt_a2dp_acceptor_t *p,dbt_runtime_t *r,dbt_avdtp_sink_hooks_t h,const dbt_avdtp_endpoint_t *e,uint8_t n,uint16_t mtu,uint16_t c,uint32_t a,uint32_t b,uint32_t d){(void)h;(void)mtu;(void)c;(void)a;(void)b;(void)d;p->runtime=r;return e==endpoints&&n==sizeof(endpoints)/sizeof(endpoints[0])?DBT_OK:DBT_INVALID;}
bool dbt_a2dp_acceptor_accept(const dbt_a2dp_acceptor_t *p,const dbt_link_t *l){(void)p;(void)l;return true;}
dbt_result_t dbt_a2dp_acceptor_event(dbt_a2dp_acceptor_t *p,dbt_channel_event_t e,const dbt_channel_t *c){(void)p;(void)e;(void)c;return DBT_OK;}
dbt_result_t dbt_a2dp_acceptor_receive(dbt_a2dp_acceptor_t *p,const dbt_channel_t *c,const uint8_t *d,uint16_t n){(void)p;(void)c;(void)d;(void)n;return DBT_OK;}
dbt_result_t dbt_a2dp_acceptor_poll(dbt_a2dp_acceptor_t *p,uint32_t t){(void)p;(void)t;return DBT_OK;}
uint32_t dbt_a2dp_acceptor_wait_ms(const dbt_a2dp_acceptor_t *p,uint32_t t){(void)p;(void)t;return UINT32_MAX;}
dbt_result_t dbt_a2dp_acceptor_complete(dbt_a2dp_acceptor_t *p,uint32_t b,uint32_t t,dbt_sink_work_result_t r,uint32_t n){(void)p;(void)b;(void)t;(void)r;(void)n;return DBT_OK;}
dbt_result_t dbt_a2dp_acceptor_retire(dbt_a2dp_acceptor_t *p,uint32_t t){(void)p;(void)t;return DBT_OK;}
dbt_result_t dbt_a2dp_acceptor_stop(dbt_a2dp_acceptor_t *p,uint32_t t){(void)p;(void)t;return DBT_OK;}
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
    (void)dbt_codec_worker_step(&worker,++now,0);dbt_codec_receipt_t receipt;
    CHECK(dbt_codec_worker_receipt(&worker,&receipt)==DBT_OK && receipt.result==DBT_SINK_WORK_READY);
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
    CHECK(dbt_codec_worker_step(&worker,++now,1)==1 && atomic_load(&pcm_frames)==1 && atomic_load(&pcm_channels)==2);
    if(expected){CHECK(atomic_load(&pcm_bytes)==expected_bytes && atomic_load(&pcm_hash)==hash_bytes(expected,expected_bytes));}
#ifdef DBT_PROBE_AUDIO_OUTPUT
    if(expected){CHECK(output_audio.input.stride==2u*sample_bytes && !memcmp(output_audio.input.bytes,expected,expected_bytes));}
#endif
#ifdef DBT_PROBE_AUDIO_OUTPUT
    CHECK(output_started && output_rate==rate && output_channels==2 && output_format==sample_bytes && output_frames>=expected_samples);
#endif
    packet[3]=2;CHECK(dbt_avdtp_sink_media_receive(&sink,456,packet,(uint16_t)(n+offset))==DBT_OK);
    CHECK(request(9,&ep,1)==DBT_OK && complete()==0 && sink.state==DBT_STREAM_OPEN);
    CHECK(dbt_codec_worker_step(&worker,++now,1)==0 && atomic_load(&pcm_frames)==1);
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
