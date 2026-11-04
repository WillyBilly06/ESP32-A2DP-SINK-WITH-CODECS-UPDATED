#include <stdlib.h>
#include <stdint.h>
#include "dbt_lhdcv5_codec.h"
#include "dbt_sbc_codec.h"
#if defined(_WIN32)
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif
int g_nzc_formula=0;
int g_pkframe=0;
extern int dbt_lhdc_recip_port_test(void);
EXPORT int probe_reciprocal_table(void){return dbt_lhdc_recip_port_test();}
static dbt_codec_worker_t worker;
static const dbt_codec_ops_t *const codecs[]={&dbt_sbc_codec,&dbt_lhdcv5_codec};
static unsigned live,peak,calls,fail_at,pcm_frames;
static uint8_t *pcm_output;
static unsigned pcm_capacity,pcm_size;
static uint32_t pcm_timestamps[15];
static unsigned pcm_calls;
typedef union {struct {unsigned size;} h;uint64_t align;} header_t;
static uint8_t *allocate(void *ctx,uint32_t size){
    (void)ctx;++calls;if(calls==fail_at)return 0;header_t *h=malloc(sizeof(*h)+size);if(!h)return 0;
    h->h.size=size;live+=size;if(live>peak)peak=live;return (uint8_t *)(h+1);
}
static void release(void *ctx,uint8_t *p){(void)ctx;header_t *h=(header_t *)p-1;live-=h->h.size;free(h);}
static dbt_result_t pcm(void *ctx,const void *samples,uint16_t frames,const dbt_codec_config_t *c,uint32_t ts){
    (void)ctx;unsigned bytes=(unsigned)frames*c->channels*c->sample_bytes;
    if(pcm_calls>=15 || (pcm_output && bytes>pcm_capacity-pcm_size))return DBT_NO_SPACE;
    pcm_timestamps[pcm_calls++]=ts;
    if(pcm_output){const uint8_t *p=samples;for(unsigned i=0;i<bytes;++i)pcm_output[pcm_size+i]=p[i];pcm_size+=bytes;}
    pcm_frames+=frames;return DBT_OK;
}
EXPORT unsigned probe_live(void){return live;}
EXPORT unsigned probe_peak(void){return peak;}
EXPORT unsigned probe_calls(void){return calls;}
EXPORT unsigned probe_pcm_frames(void){return pcm_frames;}
EXPORT void probe_fail_in(unsigned n){fail_at=calls+n;}
EXPORT void probe_peak_reset(void){peak=live;}
EXPORT int probe_configure(const uint8_t *cie,unsigned size,unsigned *rate,unsigned *frames,unsigned *sample_bytes){
    dbt_codec_config_t c;int r=dbt_lhdcv5_configure(cie,(uint16_t)size,&c);
    if(r==DBT_OK){*rate=c.rate;*frames=c.samples;*sample_bytes=c.sample_bytes;}return r;
}
EXPORT int probe_validate(const uint8_t *caps,unsigned caps_size,const uint8_t *cie,unsigned size){
    dbt_avdtp_endpoint_t endpoint={caps,(uint16_t)caps_size,1,dbt_a2dp_lhdcv5_validate};
    dbt_avdtp_cap_t codec={cie,7,(uint8_t)size};return dbt_a2dp_lhdcv5_validate(&endpoint,&codec);
}
EXPORT int worker_init(void){dbt_codec_worker_hooks_t h={{0,allocate,release,pcm},codecs,2,0,0,0};pcm_frames=0;return dbt_codec_worker_init(&worker,h,2048,2);}
EXPORT int worker_control(unsigned op,const uint8_t *cie,unsigned size){
    uint8_t caps[DBT_CODEC_CIE_BYTES+4]={1,0,7,0};if(size>DBT_CODEC_CIE_BYTES)return -1;
    caps[3]=(uint8_t)size;for(unsigned i=0;i<size;++i)caps[4+i]=cie[i];
    dbt_sink_job_t j={1,1,(dbt_sink_operation_t)op,1,caps,(uint16_t)(size+4)};
    if(dbt_codec_worker_submit(&worker,&j)!=DBT_OK)return -2;
    dbt_codec_worker_step(&worker,1,0);dbt_codec_receipt_t r;
    return dbt_codec_worker_receipt(&worker,&r)==DBT_OK?(int)r.result:-3;
}
EXPORT int worker_packet(const uint8_t *data,unsigned size,unsigned seq,unsigned now){
    pcm_output=0;pcm_size=pcm_calls=0;
    dbt_avdtp_media_t p={data,100,1,(uint16_t)size,(uint16_t)seq,96,false};
    int r=dbt_codec_worker_media(&worker,&p);if(r==DBT_OK)dbt_codec_worker_step(&worker,now,1);return r;
}
EXPORT void worker_expire(unsigned now){dbt_codec_worker_step(&worker,now,0);}
EXPORT unsigned worker_drops(void){return atomic_load(&worker.media_drops);}
EXPORT unsigned worker_pcm_calls(void){return pcm_calls;}
EXPORT uint32_t worker_pcm_timestamp(unsigned i){return i<pcm_calls?pcm_timestamps[i]:0;}
EXPORT int worker_packet_pcm(const uint8_t *data,unsigned size,unsigned seq,unsigned now,uint32_t timestamp,
                             uint8_t *pcm_out,unsigned capacity,unsigned *written){
    dbt_avdtp_media_t p={data,timestamp,1,(uint16_t)size,(uint16_t)seq,96,false};
    pcm_output=pcm_out;pcm_capacity=capacity;pcm_size=pcm_calls=0;
    int r=dbt_codec_worker_media(&worker,&p);if(r==DBT_OK)dbt_codec_worker_step(&worker,now,1);
    *written=pcm_size;pcm_output=0;return r;
}
