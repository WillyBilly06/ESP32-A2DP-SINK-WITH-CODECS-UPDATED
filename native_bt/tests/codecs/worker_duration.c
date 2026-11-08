#include "../test_codec_deferred.c"
#include <stdio.h>
#include "probe_audio_trace.h"
#include "dbt_lhdcv5_codec.h"
const dbt_codec_ops_t dbt_lhdcv5_codec={0};
uint32_t test_app_duration_normal(bool,const dbt_codec_config_t *,const dbt_avdtp_media_t *);
uint32_t test_app_duration_detailed(bool,const dbt_codec_config_t *,const dbt_avdtp_media_t *);
static bool unknown_duration,overcount_reject,change_duration_count;
static unsigned duration_hook_calls;
void codec_duration_test_step(const void *pointer){
    if(change_duration_count){
        change_duration_count=false;++duration_hook_calls;
        dbt_codec_worker_t *w=(dbt_codec_worker_t *)pointer;
        atomic_store(&w->duration_input_frames,atomic_load(&w->duration_input_frames)+600u);
    }
}
static uint32_t duration(const dbt_codec_config_t *config,const dbt_avdtp_media_t *p){
    if(unknown_duration || p->size!=2)return 0;
    if(overcount_reject)return config->samples;
    return p->payload[1]&&p->payload[1]<=2?config->samples*p->payload[1]:0;
}
static int begin(void){
    CHECK(setup());unknown_duration=overcount_reject=change_duration_count=false;duration_hook_calls=0;
    CHECK(dbt_codec_worker_set_media_frames(NULL,duration)==DBT_INVALID);
    CHECK(dbt_codec_worker_set_media_frames(&f.worker,NULL)==DBT_INVALID);
    CHECK(dbt_codec_worker_set_media_frames(&f.worker,duration)==DBT_OK);
    CHECK(dbt_codec_worker_set_media_frames(&f.worker,duration)==DBT_BUSY);
    CHECK(control(DBT_SINK_CONFIGURE)&&control(DBT_SINK_START));return 0;
}
static int accounting(void){
    CHECK(!begin());uint32_t n=99;CHECK(dbt_codec_worker_buffered_frames(&f.worker,&n)&&!n);
    CHECK(dbt_codec_worker_set_media_frames(&f.worker,duration)==DBT_BUSY);
    uint8_t p[]={5,2},q[]={6,1};CHECK(packet(p,1,0)==DBT_OK);
    CHECK(dbt_codec_worker_buffered_frames(&f.worker,&n)&&n==1200);
    CHECK(dbt_codec_worker_step(&f.worker,0,1)==1&&f.decodes==1);
    CHECK(dbt_codec_worker_buffered_frames(&f.worker,&n)&&n==1200);
    CHECK(dbt_codec_worker_step(&f.worker,1,1)==1&&f.pcm_frames==256);
    CHECK(dbt_codec_worker_buffered_frames(&f.worker,&n)&&n==944);
    f.busy=true;CHECK(!dbt_codec_worker_step(&f.worker,2,1));
    CHECK(dbt_codec_worker_buffered_frames(&f.worker,&n)&&n==944);
    CHECK(packet(q,2,1200)==DBT_OK&&packet(q,3,1800)==DBT_NO_SPACE);
    CHECK(dbt_codec_worker_buffered_frames(&f.worker,&n)&&n==1544);
    f.busy=false;
    for(unsigned i=0;i<40&&atomic_load(&f.worker.media_read)!=atomic_load(&f.worker.media_written);++i){
        CHECK(dbt_codec_worker_step(&f.worker,3+i,1)<=1);
        CHECK(dbt_codec_worker_buffered_frames(&f.worker,&n)&&n==1800u-f.pcm_frames);
    }
    CHECK(f.pcm_frames==1800&&dbt_codec_worker_buffered_frames(&f.worker,&n)&&!n);
    CHECK(control(DBT_SINK_SUSPEND)&&!dbt_codec_worker_buffered_frames(&f.worker,&n)&&!n);
    CHECK(control(DBT_SINK_START)&&dbt_codec_worker_buffered_frames(&f.worker,&n)&&!n);
    CHECK(packet(q,4,0)==DBT_OK&&dbt_codec_worker_buffered_frames(&f.worker,&n)&&n==600);
    for(unsigned i=0;i<12&&atomic_load(&f.worker.media_read)!=atomic_load(&f.worker.media_written);++i)dbt_codec_worker_step(&f.worker,50+i,1);
    CHECK(f.pcm_frames==2400&&dbt_codec_worker_buffered_frames(&f.worker,&n)&&!n);
    CHECK(control(DBT_SINK_DROP)&&!f.live&&!f.errors&&f.allocations==f.releases);return 0;
}
static int unknown_and_rejected(void){
    CHECK(!begin());uint8_t p[]={1,1};uint32_t n=99;unknown_duration=true;
    CHECK(packet(p,1,0)==DBT_OK&&!dbt_codec_worker_buffered_frames(&f.worker,&n)&&!n);
    CHECK(control(DBT_SINK_SUSPEND)&&control(DBT_SINK_START));unknown_duration=false;overcount_reject=true;p[1]=3;
    CHECK(packet(p,2,0)==DBT_OK&&dbt_codec_worker_buffered_frames(&f.worker,&n)&&n==600);
    dbt_codec_worker_step(&f.worker,0,1);
    CHECK(atomic_load(&f.worker.decode_rejects)==1&&!dbt_codec_worker_buffered_frames(&f.worker,&n)&&!n);
    CHECK(control(DBT_SINK_SUSPEND)&&control(DBT_SINK_START));overcount_reject=false;
    CHECK(dbt_codec_worker_buffered_frames(&f.worker,&n)&&!n);
    CHECK(control(DBT_SINK_DROP)&&!f.live&&!f.errors);return 0;
}
static int wrap_and_fences(void){
    CHECK(!begin());uint32_t n=99,epoch=atomic_load(&f.worker.started_epoch);
    atomic_store(&f.worker.duration_pcm_base,UINT32_MAX-5u);atomic_store(&f.worker.pcm_delivered_frames,UINT32_MAX-1u);atomic_store(&f.worker.duration_input_frames,10);
    CHECK(dbt_codec_worker_buffered_frames(&f.worker,&n)&&n==6);
    atomic_store(&f.worker.pcm_delivered_frames,2);CHECK(dbt_codec_worker_buffered_frames(&f.worker,&n)&&n==2);
    atomic_store(&f.worker.duration_input_frames,7);CHECK(!dbt_codec_worker_buffered_frames(&f.worker,&n)&&!n);
    atomic_store(&f.worker.duration_input_frames,10);atomic_store(&f.worker.gate_epoch,epoch+1u);CHECK(!dbt_codec_worker_buffered_frames(&f.worker,&n));
    atomic_store(&f.worker.gate_epoch,epoch);atomic_store(&f.worker.duration_epoch,0);CHECK(!dbt_codec_worker_buffered_frames(&f.worker,&n));
    atomic_store(&f.worker.duration_epoch,epoch);CHECK(dbt_codec_worker_buffered_frames(&f.worker,&n)&&n==2);
    change_duration_count=true;uint32_t before=atomic_load(&f.worker.duration_input_frames);
    CHECK(!dbt_codec_worker_buffered_frames(&f.worker,&n)&&!n&&duration_hook_calls==1&&!change_duration_count);
    atomic_store(&f.worker.duration_input_frames,before);
    atomic_store(&f.worker.duration_pcm_base,0);atomic_store(&f.worker.pcm_delivered_frames,UINT32_MAX-2u);atomic_store(&f.worker.duration_input_frames,3);
    CHECK(dbt_codec_worker_buffered_frames(&f.worker,&n)&&n==6);
    CHECK(!dbt_codec_worker_buffered_frames(NULL,&n)&&!dbt_codec_worker_buffered_frames(&f.worker,NULL));
    CHECK(control(DBT_SINK_DROP)&&!f.live&&!f.errors);
    CHECK(start()&&!dbt_codec_worker_buffered_frames(&f.worker,&n));CHECK(control(DBT_SINK_DROP)&&!f.live);
    return 0;
}
uint32_t dbt_lhdcv5_packet_frames(const dbt_codec_config_t *,const dbt_avdtp_media_t *);
static int lhdc_framing(void){
    uint8_t p[2050]={0};dbt_codec_config_t c={0};c.samples=960;
    dbt_avdtp_media_t packet_view={.payload=p,.payload_type=96};
    unsigned cases=0;
    for(unsigned count=1;count<=63;++count)for(unsigned low=0;low<4;++low){
        p[0]=(uint8_t)((count<<2)|low);p[1]=255;
        for(unsigned i=0;i<count;++i){p[2+4*i]=1;p[3+4*i]=0;p[4+4*i]=p[5+4*i]=0;}
        packet_view.size=(uint16_t)(2+4*count);CHECK(dbt_lhdcv5_packet_frames(&c,&packet_view)==count*960u);++cases;
        --packet_view.size;CHECK(!dbt_lhdcv5_packet_frames(&c,&packet_view));++cases;
    }
    for(unsigned word=0;word<1024;++word){
        p[0]=4;p[2]=(uint8_t)word;p[3]=(uint8_t)(word>>8);packet_view.size=(uint16_t)(4+2*word);
        CHECK(dbt_lhdcv5_packet_frames(&c,&packet_view)==(word?960u:0u));++cases;
    }
    p[0]=4;p[2]=1;p[3]=0;packet_view.size=6;packet_view.payload_type=96;c.ops=&dbt_lhdcv5_codec;
    CHECK(!test_app_duration_normal(false,&c,&packet_view));
    CHECK(test_app_duration_normal(true,&c,&packet_view)==960);
    CHECK(test_app_duration_detailed(false,&c,&packet_view)==960 && test_app_duration_detailed(true,&c,&packet_view)==960);
    c.ops=&codec;CHECK(!test_app_duration_normal(true,&c,&packet_view));
    packet_view.payload_type=DBT_AVDTP_RAW_PAYLOAD_TYPE;CHECK(!dbt_lhdcv5_packet_frames(&c,&packet_view));
    packet_view.payload_type=96;c.samples=0;CHECK(!dbt_lhdcv5_packet_frames(&c,&packet_view));
    printf("PASS actual LHDC duration framing: %u grouped/low-bit/truncation/ten-bit-length cases; raw/empty config rejected\n",cases);return 0;
}
static uint32_t trace_calls;
static uint32_t clock_read(void *unused){(void)unused;++trace_calls;return 197;}
static int delay_trace(void){
    probe_audio_trace_ring_t ring;probe_audio_trace_init(&ring);trace_calls=0;
    probe_host_record_t value={.kind=PROBE_HOST_A2DP_DELAY,.value={13440,192000,881u|(1u<<16),DBT_OK,7}};
    for(unsigned i=0;i<PROBE_AUDIO_TRACE_SLOTS;++i)CHECK(probe_audio_trace_host(&ring,&value,clock_read,NULL)==DBT_OK);
    CHECK(probe_audio_trace_host(&ring,&value,clock_read,NULL)==DBT_NO_SPACE&&trace_calls==PROBE_AUDIO_TRACE_SLOTS&&atomic_load(&ring.lost)==1);
    probe_audio_trace_record_t record;CHECK(probe_audio_trace_pop(&ring,&record)&&record.codec==PROBE_AUDIO_TRACE_HOST&&record.host.kind==PROBE_HOST_A2DP_DELAY&&record.at_us==197);
    for(unsigned i=0;i<5;++i)CHECK(record.host.value[i]==value.value[i]);
    return 0;
}
int test_delay_math(void);
int main(void){
    int r=accounting();if(!r)r=unknown_and_rejected();if(!r)r=wrap_and_fences();if(!r)r=lhdc_framing();if(!r)r=delay_trace();if(!r)r=test_delay_math();
    if(r){fprintf(stderr,"FAIL duration fixture line %d\n",r);return 1;}
    puts("PASS actual duration accounting: grouped packet samples, partial256 PCM acceptance, BUSY/full queues, unknown/reject fences, restart, wrap, default-off and allocation symmetry");return 0;
}
