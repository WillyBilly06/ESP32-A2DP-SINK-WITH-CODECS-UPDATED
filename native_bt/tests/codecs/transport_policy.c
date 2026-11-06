/* Focused regression for raw aptX admission and selected-queue ownership.
 * Production parsers/worker, fake codec/I/O: no RF or PCM-quality evidence. */
#include "dbt_codec.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do{if(!(x)){fprintf(stderr,"failed line %u\n",__LINE__);return 1;}}while(0)
static unsigned live,peak,calls,owner_wakes,fail_after;
static uint8_t *alloc32(void *c,uint32_t n){
    (void)c;if(fail_after && !--fail_after)return NULL;
    uint8_t *p=malloc(n+8);if(!p)return NULL;memcpy(p,&n,4);live+=n;
    if(live>peak)peak=live;return p+8;
}
static uint8_t *alloc16(void *c,uint16_t n){return alloc32(c,n);}
static void release(void *c,uint8_t *p){(void)c;unsigned n;memcpy(&n,p-8,4);live-=n;free(p-8);}
static dbt_result_t pcm(void *c,const void *p,uint16_t n,const dbt_codec_config_t *f,uint32_t t){(void)c;(void)p;(void)n;(void)f;(void)t;return DBT_OK;}
static dbt_result_t configure(const uint8_t *p,uint16_t n,dbt_codec_config_t *c){
    if(n!=6 || p[0] || p[1])return DBT_UNSUPPORTED;
    *c=(dbt_codec_config_t){0};c->rate=p[2]==0x21?44100:48000;c->samples=128;c->channels=2;c->sample_bytes=2;c->cie_size=6;memcpy(c->cie,p,6);return DBT_OK;
}
static dbt_result_t open_codec(dbt_codec_decoder_t *d){d->state=dbt_codec_allocate(d,128);return d->state?DBT_OK:DBT_NO_SPACE;}
static void close_codec(dbt_codec_decoder_t *d){if(d->state)dbt_codec_release(d,d->state);d->state=NULL;}
static dbt_result_t reset_codec(dbt_codec_decoder_t *d){(void)d;return DBT_OK;}
static dbt_result_t decode(dbt_codec_decoder_t *d,const dbt_avdtp_media_t *p,uint32_t now){
    (void)d;(void)now;++calls;return p->size==1800 && p->payload[0]==0x80 && p->payload[1799]==0x55?DBT_OK:DBT_INVALID;
}
static const dbt_codec_ops_t codec={"ownership fixture",configure,open_codec,close_codec,reset_codec,decode,0,0,0};
static const dbt_codec_ops_t *const codecs[]={&codec};
static void wake(void *c){(void)c;++owner_wakes;}
static dbt_codec_queue_t policy(void *c,const dbt_codec_config_t *f){(void)c;return f->rate==44100?(dbt_codec_queue_t){4,1}:(dbt_codec_queue_t){8,4};}
static int control(dbt_codec_worker_t *w,dbt_sink_operation_t op,bool large){
    uint8_t caps[]={1,0,7,6,0,0,0x21,0x15,2,53};if(large)caps[6]=0x11;
    dbt_sink_job_t job={1,1,op,1,caps,sizeof(caps)};
    if(dbt_codec_worker_submit(w,&job)!=DBT_OK)return -1;
    dbt_codec_worker_step(w,1,0);dbt_codec_receipt_t r;
    return dbt_codec_worker_receipt(w,&r)==DBT_OK?(int)r.result:-1;
}
static dbt_result_t work(void *c,const dbt_sink_job_t *j){(void)c;(void)j;return DBT_OK;}
static dbt_avdtp_media_t observed;
static dbt_result_t view(void *c,const dbt_avdtp_media_t *p){(void)c;observed=*p;++calls;return DBT_OK;}
int main(void){
    uint8_t caps[]={1,0,7,6,0,0,0xff,0xff,2,53};
    dbt_avdtp_endpoint_t ep={caps,sizeof(caps),1,dbt_a2dp_sbc_validate,4,4};
    dbt_avdtp_sink_t sink;dbt_avdtp_sink_hooks_t sh={0,alloc16,release,work,0,view};
    CHECK(dbt_avdtp_sink_init(&sink,sh,&ep,1,1,1800,64,128,5000,2000,5000)==DBT_OK);
    sink.state=DBT_STREAM_STREAMING;sink.media_binding=2;
    uint8_t raw[1800]={0x80,96,0,0};raw[1799]=0x55;
    /* Even RTP-looking compressed bytes are borrowed whole, without sniffing. */
    CHECK(dbt_avdtp_sink_media_receive(&sink,2,raw,1800)==DBT_OK);
    CHECK(observed.payload==raw && observed.size==1800 && observed.sequence==0 && observed.timestamp==0 && observed.payload_type==DBT_AVDTP_RAW_PAYLOAD_TYPE);
    CHECK(dbt_avdtp_sink_media_receive(&sink,2,raw,1799)==DBT_INVALID);
    CHECK(dbt_avdtp_sink_media_receive(&sink,2,raw,4)==DBT_OK && observed.sequence==1 && observed.timestamp==1800);
    ep.raw_codeword_bytes=ep.raw_codeword_samples=0;
    CHECK(dbt_avdtp_sink_media_receive(&sink,2,raw,16)==DBT_OK && observed.payload==raw+12 && observed.size==4 && observed.payload_type==96);
    CHECK(dbt_avdtp_sink_media_receive(&sink,2,raw,4)==DBT_INVALID);
    dbt_codec_worker_t w;dbt_codec_worker_hooks_t h={0};
    h.decoder=(dbt_codec_hooks_t){0,alloc32,release,pcm};h.codecs=codecs;h.codec_count=1;h.wake_owner=wake;h.queue_policy=policy;
    CHECK(dbt_codec_worker_init(&w,h,1800,8)==DBT_OK && !live && !atomic_load(&w.queue_layout));
    CHECK(control(&w,DBT_SINK_CONFIGURE,false)==DBT_SINK_WORK_READY && w.depth==4 && w.low_water==1);
    unsigned small=live;CHECK(control(&w,DBT_SINK_START,false)==DBT_SINK_WORK_READY);calls=owner_wakes=0;
    dbt_avdtp_media_t p={raw,0,0,1800,0,DBT_AVDTP_RAW_PAYLOAD_TYPE,false};
    for(unsigned i=0;i<4;++i){p.sequence=(uint16_t)(i*2);CHECK(dbt_codec_worker_media(&w,&p)==DBT_OK);}
    CHECK(!atomic_load(&w.input_sequence_breaks) && dbt_codec_worker_media(&w,&p)==DBT_NO_SPACE);
    CHECK(dbt_codec_worker_step(&w,2,3)==3 && !owner_wakes && calls==3);
    CHECK(dbt_codec_worker_step(&w,3,1)==1 && owner_wakes==1 && calls==4);
    /* Reconfiguration releases the old queue before constructing the next. */
    CHECK(control(&w,DBT_SINK_RECONFIGURE,true)==DBT_SINK_WORK_READY && w.depth==8 && w.low_water==4);
    CHECK(live-small==4u*w.stride && peak==live);
    CHECK(control(&w,DBT_SINK_RECONFIGURE,false)==DBT_SINK_WORK_READY && live==small);
    fail_after=2;CHECK(control(&w,DBT_SINK_RECONFIGURE,true)==DBT_SINK_WORK_REJECTED && live==small && w.depth==4);
    CHECK(control(&w,DBT_SINK_DROP,false)==DBT_SINK_WORK_READY && !live && !atomic_load(&w.queue_layout));
    puts("PASS raw/RTP framing, full-MTU ownership, per-codec queue, credit wake, OOM rollback");return 0;
}
