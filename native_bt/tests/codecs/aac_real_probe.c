#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "dbt_aac.h"
#include "dbt_sbc_codec.h"
#if defined(_WIN32)
#include <malloc.h>
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif

static dbt_codec_decoder_t native;
static dbt_codec_worker_t worker;
static const dbt_codec_ops_t *const codecs[]={&dbt_sbc_codec,&dbt_aac_codec};
static uint32_t live, peak, fail_at, calls, pcm_bytes;
static uint8_t *pcm_out;
typedef union { struct { uint32_t size; } h; uint64_t align; } header_t;
static uint8_t *allocate(void *ctx,uint32_t size){
    (void)ctx; ++calls; if(fail_at==calls)return 0;
    header_t *h=malloc(sizeof(*h)+size);if(!h)return 0;h->h.size=size;
    live+=size;if(live>peak)peak=live;return (uint8_t *)(h+1);
}
static void release(void *ctx,uint8_t *p){(void)ctx;header_t *h=(header_t *)p-1;live-=h->h.size;free(h);}
static dbt_result_t pcm(void *ctx,const void *samples,uint16_t frames,const dbt_codec_config_t *c,uint32_t timestamp){
    (void)ctx;(void)timestamp;uint32_t n=(uint32_t)frames*c->channels*2u;
    if(!pcm_out || pcm_bytes+n>8192)return DBT_NO_SPACE;
    memcpy(pcm_out+pcm_bytes,samples,n);pcm_bytes+=n;return DBT_OK;
}
EXPORT int probe_open(unsigned rate,unsigned channels){
    uint8_t cie[]={0,2,0x40,0,0,0,0,0};
    if(rate==44100)cie[3]=1;else if(rate!=48000)return -1;
    cie[4]=(uint8_t)((rate==48000?0x80:0)|(channels==1?8:4));
    dbt_codec_config_t c;if(dbt_aac_configure(cie,8,&c)!=DBT_OK)return -2;
    c.ops=&dbt_aac_codec;
    if(!native.hooks.allocate){dbt_codec_hooks_t h={0,allocate,release,pcm};if(dbt_codec_init(&native,h)!=DBT_OK)return -3;}
    return dbt_codec_open(&native,&c);
}
EXPORT int probe_sbc_open(const uint8_t *cie){
    dbt_codec_config_t c;if(dbt_sbc_codec.configure(cie,6,&c)!=DBT_OK)return -1;
    c.ops=&dbt_sbc_codec;
    if(!native.hooks.allocate){dbt_codec_hooks_t h={0,allocate,release,pcm};if(dbt_codec_init(&native,h)!=DBT_OK)return -2;}
    return dbt_codec_open(&native,&c);
}
EXPORT void probe_close(void){dbt_codec_close(&native);}
EXPORT unsigned probe_live(void){return live;}
EXPORT unsigned probe_peak(void){return peak;}
EXPORT void probe_peak_reset(void){peak=live;}
EXPORT void probe_fail_next(void){fail_at=calls+1;}
EXPORT void probe_fail_in(unsigned n){fail_at=calls+n;}
EXPORT int probe_reset(void){return native.config.ops->reset(&native);}
EXPORT void probe_expire(unsigned now){native.config.ops->expire(&native,now);}
EXPORT int probe_media(const uint8_t *p,unsigned size,unsigned timestamp,unsigned seq,unsigned marker,unsigned now,uint8_t *out,unsigned *written){
    dbt_avdtp_media_t m={p,timestamp,1,(uint16_t)size,(uint16_t)seq,96,marker!=0};
    pcm_out=out;pcm_bytes=0;int result=native.config.ops->media(&native,&m,now);*written=pcm_bytes;pcm_out=0;return result;
}
EXPORT int worker_init(void){
    dbt_codec_worker_hooks_t h={{0,allocate,release,pcm},codecs,2,0,0,0};
    return dbt_codec_worker_init(&worker,h,2048,2);
}
EXPORT int worker_control(unsigned operation,const uint8_t *cie,unsigned size){
    uint8_t caps[20]={1,0,7,0};
    if(size>16)return -1;
    caps[3]=(uint8_t)size;for(unsigned i=0;i<size;++i)caps[4+i]=cie[i];
    dbt_sink_job_t j={1,1,(dbt_sink_operation_t)operation,1,caps,(uint16_t)(size+4)};
    int r=dbt_codec_worker_submit(&worker,&j);if(r!=DBT_OK)return -2;
    dbt_codec_worker_step(&worker,1,0);dbt_codec_receipt_t receipt;
    return dbt_codec_worker_receipt(&worker,&receipt)==DBT_OK?(int)receipt.result:-3;
}
EXPORT int worker_packet(const uint8_t *p,unsigned size,unsigned sequence,unsigned marker,uint8_t *out,unsigned *written){
    dbt_avdtp_media_t packet={p,100,1,(uint16_t)size,(uint16_t)sequence,96,marker!=0};
    pcm_out=out;pcm_bytes=0;
    int r=dbt_codec_worker_media(&worker,&packet);
    if(r==DBT_OK)dbt_codec_worker_step(&worker,2,1);
    *written=pcm_bytes;pcm_out=0;return r;
}
