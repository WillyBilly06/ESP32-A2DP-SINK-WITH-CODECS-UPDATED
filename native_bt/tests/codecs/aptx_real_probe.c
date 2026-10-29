#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "dbt_aptx.h"
#include "dbt_sbc_codec.h"
#include "freeaptx.h"
#if defined(_WIN32)
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif
static dbt_codec_worker_t worker;
static struct aptx_context *encoder,*reference;
static const dbt_codec_ops_t *const codecs[]={&dbt_sbc_codec,&dbt_aptx_codec};
static unsigned live,peak,calls,fail_at,pcm_bytes;
static uint8_t *output;
typedef union {struct {unsigned size;} h;uint64_t align;} header_t;
static uint8_t *allocate(void *ctx,uint32_t size){
    (void)ctx;++calls;if(calls==fail_at)return 0;header_t *h=malloc(sizeof(*h)+size);if(!h)return 0;
    h->h.size=size;live+=size;if(live>peak)peak=live;return (uint8_t *)(h+1);
}
static void release(void *ctx,uint8_t *p){(void)ctx;header_t *h=(header_t *)p-1;live-=h->h.size;free(h);}
static dbt_result_t pcm(void *ctx,const void *samples,uint16_t frames,const dbt_codec_config_t *c,uint32_t ts){
    (void)ctx;(void)ts;unsigned n=(unsigned)frames*c->channels*c->sample_bytes;
    if(!output || pcm_bytes+n>8192)return DBT_NO_SPACE;memcpy(output+pcm_bytes,samples,n);pcm_bytes+=n;return DBT_OK;
}
EXPORT unsigned probe_live(void){return live;}
EXPORT unsigned probe_peak(void){return peak;}
EXPORT unsigned worker_drops(void){return atomic_load(&worker.media_drops);}
EXPORT void probe_peak_reset(void){peak=live;}
EXPORT void probe_fail_in(unsigned n){fail_at=calls+n;}
EXPORT int probe_configure(const uint8_t *cie,unsigned size,unsigned *rate,unsigned *channels,unsigned *sample_bytes){
    dbt_codec_config_t c;int r=dbt_aptx_configure(cie,(uint16_t)size,&c);
    if(r==DBT_OK){*rate=c.rate;*channels=c.channels;*sample_bytes=c.sample_bytes;}return r;
}
EXPORT int probe_validate(const uint8_t *caps,unsigned caps_size,const uint8_t *cie,unsigned size){
    dbt_avdtp_endpoint_t endpoint={caps,(uint16_t)caps_size,1,dbt_a2dp_aptx_validate};
    dbt_avdtp_cap_t codec={cie,7,(uint8_t)size};return dbt_a2dp_aptx_validate(&endpoint,&codec);
}
EXPORT int worker_init(void){dbt_codec_worker_hooks_t h={{0,allocate,release,pcm},codecs,2,0,0,0};return dbt_codec_worker_init(&worker,h,2048,2);}
EXPORT int worker_control(unsigned op,const uint8_t *cie,unsigned size){
    uint8_t caps[DBT_CODEC_CIE_BYTES+4]={1,0,7,0};if(size>DBT_CODEC_CIE_BYTES)return -1;
    caps[3]=(uint8_t)size;for(unsigned i=0;i<size;++i)caps[4+i]=cie[i];
    dbt_sink_job_t j={1,1,(dbt_sink_operation_t)op,1,caps,(uint16_t)(size+4)};
    if(dbt_codec_worker_submit(&worker,&j)!=DBT_OK)return -2;
    dbt_codec_worker_step(&worker,1,0);dbt_codec_receipt_t r;
    return dbt_codec_worker_receipt(&worker,&r)==DBT_OK?(int)r.result:-3;
}
EXPORT int worker_packet(const uint8_t *data,unsigned size,unsigned seq,uint8_t *pcm_out,unsigned *written){
    dbt_avdtp_media_t packet={data,100+seq*128u,1,(uint16_t)size,(uint16_t)seq,96,false};output=pcm_out;pcm_bytes=0;
    int r=dbt_codec_worker_media(&worker,&packet);if(r==DBT_OK)dbt_codec_worker_step(&worker,2,1);
    *written=pcm_bytes;output=0;return r;
}
EXPORT int reference_open(int is_hd){
    encoder=aptx_init(is_hd);reference=aptx_init(is_hd);return encoder && reference?0:-1;
}
EXPORT void reference_close(void){if(encoder)aptx_finish(encoder);if(reference)aptx_finish(reference);encoder=reference=0;}
EXPORT unsigned reference_encode(const uint8_t *pcm,unsigned size,uint8_t *encoded,unsigned capacity,unsigned *written){
    size_t n=0;size_t used=aptx_encode(encoder,pcm,size,encoded,capacity,&n);*written=(unsigned)n;return (unsigned)used;
}
EXPORT unsigned reference_decode(const uint8_t *encoded,unsigned size,uint8_t *pcm_out,unsigned capacity,unsigned *written){
    size_t n=0;size_t used=aptx_decode32(reference,encoded,size,pcm_out,capacity,&n);*written=(unsigned)n;return (unsigned)used;
}
