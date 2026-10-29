#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "dbt_ldac.h"
#include "dbt_sbc_codec.h"
#if defined(_WIN32)
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif
static dbt_codec_worker_t worker;
static const dbt_codec_ops_t *const codecs[]={&dbt_sbc_codec,&dbt_ldac_codec};
static unsigned live,peak,calls,fail_at,pcm_bytes;
static uint8_t *output;
static uint32_t pcm_timestamp[DBT_LDAC_MAX_FRAMES];
static uint16_t pcm_samples[DBT_LDAC_MAX_FRAMES];
static uint8_t pcm_channels[DBT_LDAC_MAX_FRAMES],pcm_width[DBT_LDAC_MAX_FRAMES];
static unsigned pcm_calls;
extern int dbt_ldac_shift_table_test(void);
EXPORT int probe_shift_table(void){return dbt_ldac_shift_table_test();}
typedef union {struct {unsigned size;} h;uint64_t align;} header_t;
static uint8_t *allocate(void *ctx,uint32_t size){
    (void)ctx;++calls;if(calls==fail_at)return 0;header_t *h=malloc(sizeof(*h)+size);if(!h)return 0;
    h->h.size=size;live+=size;if(live>peak)peak=live;return (uint8_t *)(h+1);
}
static void release(void *ctx,uint8_t *p){(void)ctx;header_t *h=(header_t *)p-1;live-=h->h.size;free(h);}
static dbt_result_t pcm(void *ctx,const void *samples,uint16_t frames,const dbt_codec_config_t *c,uint32_t ts){
    (void)ctx;unsigned n=(unsigned)frames*c->channels*c->sample_bytes;
    if(pcm_calls>=DBT_LDAC_MAX_FRAMES)return DBT_NO_SPACE;
    pcm_timestamp[pcm_calls]=ts;pcm_samples[pcm_calls]=frames;pcm_channels[pcm_calls]=c->channels;pcm_width[pcm_calls]=c->sample_bytes;++pcm_calls;
    if(!output || pcm_bytes+n>8192)return DBT_NO_SPACE;memcpy(output+pcm_bytes,samples,n);pcm_bytes+=n;return DBT_OK;
}
EXPORT unsigned probe_live(void){return live;}
EXPORT unsigned probe_peak(void){return peak;}
EXPORT unsigned probe_calls(void){return calls;}
EXPORT unsigned worker_drops(void){return atomic_load(&worker.media_drops);}
EXPORT unsigned worker_faults(void){return atomic_load(&worker.faults);}
EXPORT unsigned worker_pcm_calls(void){return pcm_calls;}
EXPORT uint32_t worker_pcm_timestamp(unsigned i){return i<pcm_calls?pcm_timestamp[i]:0;}
EXPORT unsigned worker_pcm_samples(unsigned i){return i<pcm_calls?pcm_samples[i]:0;}
EXPORT unsigned worker_pcm_channels(unsigned i){return i<pcm_calls?pcm_channels[i]:0;}
EXPORT unsigned worker_pcm_width(unsigned i){return i<pcm_calls?pcm_width[i]:0;}
EXPORT void probe_peak_reset(void){peak=live;}
EXPORT void probe_fail_in(unsigned n){fail_at=calls+n;}
EXPORT int worker_init(void){dbt_codec_worker_hooks_t h={{0,allocate,release,pcm},codecs,2,0,0,0};return dbt_codec_worker_init(&worker,h,2048,2);}
EXPORT int worker_control(unsigned op,const uint8_t *cie,unsigned size){
    uint8_t caps[DBT_CODEC_CIE_BYTES+4]={1,0,7,0};if(size>DBT_CODEC_CIE_BYTES)return -1;
    caps[3]=(uint8_t)size;for(unsigned i=0;i<size;++i)caps[4+i]=cie[i];
    dbt_sink_job_t j={1,1,(dbt_sink_operation_t)op,1,caps,(uint16_t)(size+4)};
    if(dbt_codec_worker_submit(&worker,&j)!=DBT_OK)return -2;
    dbt_codec_worker_step(&worker,1,0);dbt_codec_receipt_t r;
    return dbt_codec_worker_receipt(&worker,&r)==DBT_OK?(int)r.result:-3;
}
EXPORT int worker_packet_at(const uint8_t *data,unsigned size,unsigned seq,uint32_t timestamp,uint8_t *pcm_out,unsigned *written){
    dbt_avdtp_media_t p={data,timestamp,1,(uint16_t)size,(uint16_t)seq,96,false};output=pcm_out;pcm_bytes=0;pcm_calls=0;
    int r=dbt_codec_worker_media(&worker,&p);if(r==DBT_OK)dbt_codec_worker_step(&worker,2,1);
    *written=pcm_bytes;output=0;return r;
}
EXPORT int worker_packet(const uint8_t *data,unsigned size,unsigned seq,uint8_t *pcm_out,unsigned *written){
    return worker_packet_at(data,size,seq,100+seq*256u,pcm_out,written);
}
