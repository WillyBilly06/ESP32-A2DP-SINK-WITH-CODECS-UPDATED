#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "dbt_opus.h"
#include "dbt_sbc_codec.h"
#include "opus.h"
#if defined(_WIN32)
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif

static dbt_codec_worker_t worker;
static dbt_codec_decoder_t direct;
static const dbt_codec_ops_t *const codecs[]={&dbt_sbc_codec,&dbt_opus_codec};
static unsigned live,peak,calls,fail_at,pcm_bytes;
static uint8_t *pcm_output;
typedef union {struct {unsigned size;} h;uint64_t align;} header_t;
static uint8_t *allocate(void *context,uint32_t size){
    (void)context;++calls;if(fail_at==calls)return 0;
    header_t *h=malloc(sizeof(*h)+size);if(!h)return 0;h->h.size=size;live+=size;if(live>peak)peak=live;return (uint8_t *)(h+1);
}
static void release(void *context,uint8_t *bytes){(void)context;header_t *h=(header_t *)bytes-1;live-=h->h.size;free(h);}
static dbt_result_t pcm(void *context,const void *samples,uint16_t frames,const dbt_codec_config_t *config,uint32_t timestamp){
    (void)context;(void)timestamp;unsigned bytes=(unsigned)frames*config->channels*2u;
    if(!pcm_output || pcm_bytes+bytes>8192)return DBT_NO_SPACE;
    memcpy(pcm_output+pcm_bytes,samples,bytes);pcm_bytes+=bytes;return DBT_OK;
}
EXPORT unsigned probe_live(void){return live;}
EXPORT unsigned probe_peak(void){return peak;}
EXPORT void probe_peak_reset(void){peak=live;}
EXPORT void probe_fail_in(unsigned n){fail_at=calls+n;}
EXPORT int probe_configure(const uint8_t *cie,unsigned size,unsigned *rate,unsigned *channels,unsigned *samples){
    dbt_codec_config_t c;int r=dbt_opus_configure(cie,(uint16_t)size,&c);
    if(r==DBT_OK){*rate=c.rate;*channels=c.channels;*samples=c.samples;}return r;
}
EXPORT int probe_validate(const uint8_t *caps,unsigned caps_size,const uint8_t *cie,unsigned size){
    dbt_avdtp_endpoint_t endpoint={caps,(uint16_t)caps_size,1,dbt_a2dp_opus_validate};
    dbt_avdtp_cap_t codec={cie,7,(uint8_t)size};return dbt_a2dp_opus_validate(&endpoint,&codec);
}
EXPORT int probe_open(const uint8_t *cie,unsigned size){
    dbt_codec_config_t c;if(dbt_opus_configure(cie,(uint16_t)size,&c)!=DBT_OK)return -1;c.ops=&dbt_opus_codec;
    if(!direct.hooks.allocate){dbt_codec_hooks_t h={0,allocate,release,pcm};if(dbt_codec_init(&direct,h)!=DBT_OK)return -2;}
    return dbt_codec_open(&direct,&c);
}
EXPORT void probe_close(void){dbt_codec_close(&direct);}
EXPORT void probe_expire(unsigned now){direct.config.ops->expire(&direct,now);}
EXPORT int probe_media(const uint8_t *data,unsigned size,unsigned seq,unsigned timestamp,unsigned now,uint8_t *out,unsigned *written){
    dbt_avdtp_media_t p={data,timestamp,1,(uint16_t)size,(uint16_t)seq,96,false};
    pcm_output=out;pcm_bytes=0;int r=direct.config.ops->media(&direct,&p,now);*written=pcm_bytes;pcm_output=0;return r;
}
EXPORT int worker_init(void){
    dbt_codec_worker_hooks_t h={{0,allocate,release,pcm},codecs,2,0,0,0};
    return dbt_codec_worker_init(&worker,h,2048,2);
}
EXPORT int worker_control(unsigned operation,const uint8_t *cie,unsigned size){
    uint8_t caps[DBT_CODEC_CIE_BYTES+4]={1,0,7,0};if(size>DBT_CODEC_CIE_BYTES)return -1;
    caps[3]=(uint8_t)size;for(unsigned i=0;i<size;++i)caps[4+i]=cie[i];
    dbt_sink_job_t j={1,1,(dbt_sink_operation_t)operation,1,caps,(uint16_t)(size+4)};
    if(dbt_codec_worker_submit(&worker,&j)!=DBT_OK)return -2;
    dbt_codec_worker_step(&worker,1,0);dbt_codec_receipt_t receipt;
    return dbt_codec_worker_receipt(&worker,&receipt)==DBT_OK?(int)receipt.result:-3;
}
EXPORT int worker_packet(const uint8_t *data,unsigned size,unsigned seq,unsigned timestamp,uint8_t *out,unsigned *written){
    dbt_avdtp_media_t p={data,timestamp,1,(uint16_t)size,(uint16_t)seq,96,false};pcm_output=out;pcm_bytes=0;
    int r=dbt_codec_worker_media(&worker,&p);if(r==DBT_OK)dbt_codec_worker_step(&worker,2,1);
    *written=pcm_bytes;pcm_output=0;return r;
}
