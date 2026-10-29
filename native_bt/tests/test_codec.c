#include "dbt_aac.h"
#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif
#define CHECK(x) do{if(!(x))return __LINE__;}while(0)
static void zero(void *v,unsigned n){uint8_t *p=v;for(unsigned i=0;i<n;++i)p[i]=0;}
static bool equal(const void *a,const void *b,unsigned n){const uint8_t *x=a,*y=b;for(unsigned i=0;i<n;++i)if(x[i]!=y[i])return false;return true;}
static const uint8_t aac_cie[]={0,2,0x40,0,0x84,0x84,0xe2,0};
static const uint8_t aac_caps[]={1,0,7,8,0,2,0xc0,1,0x8c,0x84,0xe2,0};
EXPORT int test_aac_selection_and_endpoint_subset(void){
    dbt_codec_config_t c,old;zero(&c,sizeof(c));
    CHECK(dbt_aac_configure(aac_cie,8,&c)==DBT_OK && c.rate==48000 && c.channels==2 && c.samples==1024 && c.sample_bytes==2);
    CHECK(c.cie_size==8 && equal(c.cie,aac_cie,8));old=c;
    uint8_t selected[8];for(unsigned i=0;i<8;++i)selected[i]=aac_cie[i];
    selected[2]=0xc0;CHECK(dbt_aac_configure(selected,8,&c)==DBT_INVALID && equal(&c,&old,sizeof(c)));
    selected[2]=0x40;selected[4]=0x8c;CHECK(dbt_aac_configure(selected,8,&c)==DBT_INVALID);
    selected[4]=0x84;selected[3]=1;CHECK(dbt_aac_configure(selected,8,&c)==DBT_INVALID);
    selected[3]=0;selected[4]=0x44;CHECK(dbt_aac_configure(selected,8,&c)==DBT_UNSUPPORTED);
    selected[3]=1;selected[4]=8;CHECK(dbt_aac_configure(selected,8,&c)==DBT_OK && c.rate==44100 && c.channels==1);
    dbt_avdtp_endpoint_t endpoint={aac_caps,sizeof(aac_caps),2,dbt_a2dp_aac_validate};
    dbt_avdtp_cap_t cap={aac_cie,7,8};
    CHECK(!dbt_a2dp_aac_validate(&endpoint,&cap));
    uint8_t local[sizeof(aac_caps)];for(unsigned i=0;i<sizeof(local);++i)local[i]=aac_caps[i];endpoint.capabilities=local;
    local[9]&=0x7f;CHECK(dbt_a2dp_aac_validate(&endpoint,&cap)==0x29);
    local[9]=0x84;local[11]=0;local[10]=1;CHECK(dbt_a2dp_aac_validate(&endpoint,&cap)==0x29);
    cap.category=1;CHECK(dbt_a2dp_aac_validate(&endpoint,&cap)==0x18);return 0;
}
typedef struct {uint8_t bytes[1800];unsigned bits;} writer_t;
static void put(writer_t *b,unsigned value,unsigned n){for(unsigned i=n;i;--i){unsigned p=b->bits++;if((value>>(i-1u))&1u)b->bytes[p>>3]|=(uint8_t)(1u<<(7u-(p&7u)));}}
static unsigned mux_write(writer_t *b,bool same,unsigned frames,unsigned length){
    zero(b,sizeof(*b));put(b,same,1);
    if(!same){put(b,0,1);put(b,1,1);put(b,frames-1u,6);put(b,0,4);put(b,0,3);
        put(b,2,5);put(b,3,4);put(b,2,4);put(b,0,3);put(b,0,3);put(b,255,8);put(b,0,1);put(b,0,1);}
    for(unsigned frame=0;frame<frames;++frame){unsigned left=length;while(left>=255){put(b,255,8);left-=255;}put(b,left,8);
        for(unsigned i=0;i<length;++i)put(b,(i+frame)&255u,8);}
    return (b->bits+7u)/8u;
}
EXPORT int test_aac_latm_boundaries_and_transaction(void){
    writer_t b;dbt_codec_config_t c;CHECK(dbt_aac_configure(aac_cie,8,&c)==DBT_OK);
    dbt_aac_mux_t mux={false,0};dbt_aac_unit_t units[4];uint8_t count=99,out[301];zero(units,sizeof(units));
    unsigned size=mux_write(&b,true,1,4);
    CHECK(dbt_aac_latm_parse(&mux,&c,b.bytes,(uint16_t)size,units,4,&count)==DBT_INVALID && count==99 && !mux.configured);
    size=mux_write(&b,false,2,300);
    CHECK(dbt_aac_latm_parse(&mux,&c,b.bytes,(uint16_t)size,units,1,&count)==DBT_NO_SPACE && !mux.configured && count==99);
    CHECK(dbt_aac_latm_parse(&mux,&c,b.bytes,(uint16_t)size,units,4,&count)==DBT_OK && mux.configured && mux.subframes==2 && count==2);
    CHECK(units[0].size==300 && (units[0].bit_offset&7u));
    for(unsigned frame=0;frame<2;++frame){out[300]=0xa5;CHECK(dbt_aac_latm_copy(b.bytes,(uint16_t)size,units[frame],out,300)==DBT_OK);
        for(unsigned i=0;i<300;++i){CHECK(out[i]==(uint8_t)(i+frame));}
        CHECK(out[300]==0xa5);}
    dbt_aac_unit_t saved[4];for(unsigned i=0;i<4;++i)saved[i]=units[i];count=99;
    CHECK(dbt_aac_latm_parse(&mux,&c,b.bytes,(uint16_t)(size-1u),units,4,&count)==DBT_INVALID && count==99 && equal(saved,units,sizeof(units)) && mux.subframes==2);
    b.bytes[size-1u]|=1;CHECK(dbt_aac_latm_parse(&mux,&c,b.bytes,(uint16_t)size,units,4,&count)==DBT_INVALID);
    size=mux_write(&b,true,2,10);CHECK(dbt_aac_latm_parse(&mux,&c,b.bytes,(uint16_t)size,units,4,&count)==DBT_OK && count==2);
    CHECK(dbt_aac_latm_copy(b.bytes,(uint16_t)size,units[0],out,9)==DBT_INVALID);
    mux.subframes=255;CHECK(dbt_aac_latm_parse(&mux,&c,b.bytes,(uint16_t)size,units,255,&count)==DBT_INVALID);
    size=mux_write(&b,false,1,4);b.bytes[0]|=0x40;
    CHECK(dbt_aac_latm_parse(&mux,&c,b.bytes,(uint16_t)size,units,4,&count)==DBT_UNSUPPORTED);return 0;
}
/* Fake decoders test ownership/scheduling only; real AAC PCM has a separate
 * standalone Helix harness. Neither fake is advertised by the ESP32 probe. */
typedef struct {
    dbt_codec_worker_t worker;
    _Alignas(8) uint8_t storage[8][2048];
    uint32_t sizes[8],live,peak,allocations,releases,fail_at,opens,closes,resets,media,errors;
    bool used[8],reset_bad,stream_bad;uint8_t received;uint32_t timestamp;
} fixture_t;
static fixture_t f;
static uint8_t *allocate(void *context,uint32_t size){
    fixture_t *p=context;++p->allocations;if(p->fail_at==p->allocations || size>2048)return 0;
    for(unsigned i=0;i<8;++i)if(!p->used[i]){p->used[i]=true;p->sizes[i]=size;p->live+=size;if(p->live>p->peak)p->peak=p->live;return p->storage[i];}
    return 0;
}
static void release(void *context,uint8_t *bytes){
    fixture_t *p=context;
    for(unsigned i=0;i<8;++i){if(bytes==p->storage[i]){
        if(!p->used[i]){++p->errors;return;}p->used[i]=false;p->live-=p->sizes[i];++p->releases;return;
    }}
    ++p->errors;
}
static dbt_result_t pcm(void *context,const void *samples,uint16_t frames,const dbt_codec_config_t *config,uint32_t timestamp){
    (void)context;(void)samples;(void)frames;(void)config;(void)timestamp;return DBT_OK;}
static dbt_result_t fake_configure(const uint8_t *cie,uint16_t size,dbt_codec_config_t *c){
    if(!cie || size!=6 || cie[0] || cie[1])return DBT_UNSUPPORTED;
    zero(c,sizeof(*c));c->rate=48000;c->channels=2;c->samples=128;c->sample_bytes=2;c->cie_size=6;
    for(unsigned i=0;i<6;++i)c->cie[i]=cie[i];
    return DBT_OK;
}
static dbt_result_t fake_open(dbt_codec_decoder_t *d){fixture_t *p=d->hooks.context;++p->opens;
    d->state=dbt_codec_allocate(d,d->config.cie[1]==2?256:128);return d->state?DBT_OK:DBT_NO_SPACE;}
static void fake_close(dbt_codec_decoder_t *d){fixture_t *p=d->hooks.context;if(d->state){++p->closes;dbt_codec_release(d,d->state);d->state=0;}}
static dbt_result_t fake_reset(dbt_codec_decoder_t *d){fixture_t *p=d->hooks.context;++p->resets;return p->reset_bad?DBT_INVALID:DBT_OK;}
static dbt_result_t fake_media(dbt_codec_decoder_t *d,const dbt_avdtp_media_t *packet,uint32_t now){
    (void)now;fixture_t *p=d->hooks.context;++p->media;p->received=packet->payload[0];p->timestamp=packet->timestamp;return DBT_OK;}
static dbt_result_t stream(void *context,bool start,const dbt_codec_config_t *config){(void)start;(void)config;return ((fixture_t *)context)->stream_bad?DBT_BUSY:DBT_OK;}
static const dbt_codec_ops_t fake_sbc={"ownership SBC",fake_configure,fake_open,fake_close,fake_reset,fake_media,0};
static const dbt_codec_ops_t fake_aac={"ownership AAC",dbt_aac_configure,fake_open,fake_close,fake_reset,fake_media,0};
static const dbt_codec_ops_t *const codecs[]={&fake_sbc,&fake_aac};
static bool setup(void){zero(&f,sizeof(f));dbt_codec_worker_hooks_t h={{&f,allocate,release,pcm},codecs,2,0,0,stream};return dbt_codec_worker_init(&f.worker,h,128,2)==DBT_OK;}
static dbt_result_t submit(dbt_sink_operation_t operation,uint32_t binding,uint32_t token,bool aac){
    uint8_t sbc[]={1,0,7,6,0,0,0x21,0x15,2,53},aac_config[12]={1,0,7,8};
    for(unsigned i=0;i<8;++i)aac_config[4+i]=aac_cie[i];
    dbt_sink_job_t j={binding,token,operation,1,aac?aac_config:sbc,(uint16_t)(aac?12:10)};return dbt_codec_worker_submit(&f.worker,&j);
}
static int complete(dbt_sink_operation_t operation,uint32_t binding,bool aac){
    if(submit(operation,binding,1,aac)!=DBT_OK)return -1;
    dbt_codec_worker_step(&f.worker,1,0);dbt_codec_receipt_t r;
    return dbt_codec_worker_receipt(&f.worker,&r)==DBT_OK?(int)r.result:-1;
}
EXPORT int test_codec_lazy_lifecycle_and_rollback(void){
    CHECK(setup() && !f.live && !f.allocations && !f.worker.decoder.state);
    CHECK(complete(DBT_SINK_CONFIGURE,1,false)==DBT_SINK_WORK_READY);uint32_t sbc=f.live;
    CHECK(sbc && atomic_load(&f.worker.resident_bytes)==sbc);
    f.fail_at=f.allocations+1;CHECK(complete(DBT_SINK_RECONFIGURE,1,true)==DBT_SINK_WORK_REJECTED);
    CHECK(f.live==sbc && f.worker.decoder.config.ops==&fake_sbc && f.worker.binding==1 && f.peak==sbc);
    f.fail_at=0;CHECK(complete(DBT_SINK_RECONFIGURE,1,true)==DBT_SINK_WORK_READY);
    CHECK(f.worker.decoder.config.ops==&fake_aac && f.live>sbc && f.peak==f.live);
    CHECK(complete(DBT_SINK_CLOSE,1,true)==DBT_SINK_WORK_READY && !f.live && !f.worker.binding);
    f.fail_at=f.allocations+2;CHECK(complete(DBT_SINK_CONFIGURE,2,false)==DBT_SINK_WORK_REJECTED && !f.live && !f.worker.binding);
    f.fail_at=0;CHECK(complete(DBT_SINK_CONFIGURE,3,false)==DBT_SINK_WORK_READY);
    CHECK(complete(DBT_SINK_ABORT,3,false)==DBT_SINK_WORK_READY && !f.live && !f.errors);
    return 0;
}
EXPORT int test_codec_queue_gates_and_copy(void){
    CHECK(setup());CHECK(complete(DBT_SINK_CONFIGURE,1,false)==DBT_SINK_WORK_READY);
    CHECK(complete(DBT_SINK_START,1,false)==DBT_SINK_WORK_READY);
    uint8_t bytes[]={0x55,2};dbt_avdtp_media_t p={bytes,123,9,2,65535,96,true};
    CHECK(dbt_codec_worker_media(&f.worker,&p)==DBT_OK);bytes[0]=0xaa;
    CHECK(dbt_codec_worker_step(&f.worker,2,1)==1 && f.received==0x55 && f.timestamp==123);
    p.sequence=0;CHECK(dbt_codec_worker_media(&f.worker,&p)==DBT_OK);CHECK(dbt_codec_worker_step(&f.worker,2,1)==1 && !atomic_load(&f.worker.resets));
    for(unsigned i=0;i<4;++i)CHECK(submit(DBT_SINK_START,1,i+1,false)==DBT_OK);
    dbt_codec_worker_step(&f.worker,3,0); /* Fill receipts without consuming them. */
    for(unsigned i=0;i<4;++i)CHECK(submit(DBT_SINK_START,1,i+5,false)==DBT_OK);
    uint32_t epoch=atomic_load(&f.worker.gate_epoch);CHECK(submit(DBT_SINK_CLOSE,1,99,false)==DBT_BUSY && atomic_load(&f.worker.gate_epoch)==epoch);
    CHECK(dbt_codec_worker_step(&f.worker,4,0)==0 && f.worker.binding==1);
    dbt_codec_receipt_t r;for(unsigned i=0;i<4;++i)CHECK(dbt_codec_worker_receipt(&f.worker,&r)==DBT_OK && r.token==i+1);
    dbt_codec_worker_step(&f.worker,5,0);for(unsigned i=0;i<4;++i)CHECK(dbt_codec_worker_receipt(&f.worker,&r)==DBT_OK && r.token==i+5);
    CHECK(dbt_codec_worker_media(&f.worker,&p)==DBT_OK);
    CHECK(submit(DBT_SINK_CLOSE,1,10,false)==DBT_OK && dbt_codec_worker_media(&f.worker,&p)==DBT_BUSY);
    CHECK(dbt_codec_worker_step(&f.worker,6,8)==0 && !f.live && !f.errors);return 0;
}
EXPORT int test_codec_reset_failure_stops_stream(void){
    CHECK(setup());CHECK(complete(DBT_SINK_CONFIGURE,1,false)==DBT_SINK_WORK_READY);
    CHECK(complete(DBT_SINK_START,1,false)==DBT_SINK_WORK_READY);
    uint8_t bytes[]={1};dbt_avdtp_media_t p={bytes,0,1,1,2,96,true};
    CHECK(dbt_codec_worker_media(&f.worker,&p)==DBT_OK && dbt_codec_worker_step(&f.worker,1,1)==1);
    f.reset_bad=true;p.sequence=4;CHECK(dbt_codec_worker_media(&f.worker,&p)==DBT_OK);
    CHECK(dbt_codec_worker_step(&f.worker,2,1)==1 && atomic_load(&f.worker.faults)==1 && f.media==1);
    CHECK(dbt_codec_worker_media(&f.worker,&p)==DBT_BUSY && !f.worker.streaming);
    CHECK(complete(DBT_SINK_DROP,1,false)==DBT_SINK_WORK_READY && !f.live && !f.errors);return 0;
}
