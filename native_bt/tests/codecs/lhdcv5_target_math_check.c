#include "dbt_lhdcv5_codec.h"
#include <stdlib.h>
#include <string.h>
#ifdef _WIN32
#define EX __declspec(dllexport)
#else
#define EX __attribute__((visibility("default")))
#endif
int g_nzc_formula,g_pkframe;
static dbt_codec_decoder_t decoder;static uint8_t *destination;static uint32_t capacity,used,live;
static uint8_t *allocate(void *ctx,uint32_t n){(void)ctx;uint8_t *p=malloc(n);if(p)live++;return p;}
static void release(void *ctx,uint8_t *p){(void)ctx;if(p){live--;free(p);}}
static dbt_result_t pcm(void *ctx,const void *p,uint16_t frames,const dbt_codec_config_t *cfg,uint32_t timestamp){
    (void)ctx;(void)timestamp;uint32_t n=(uint32_t)frames*cfg->channels*cfg->sample_bytes;
    if(!destination || n>capacity-used)return DBT_NO_SPACE;memcpy(destination+used,p,n);used+=n;return DBT_OK;
}
EX int math_open(const uint8_t *cie,uint8_t n){
    dbt_codec_config_t cfg;dbt_result_t r=dbt_lhdcv5_configure(cie,n,&cfg);if(r!=DBT_OK)return r;
    r=dbt_codec_init(&decoder,(dbt_codec_hooks_t){0,allocate,release,pcm});if(r!=DBT_OK)return r;cfg.ops=&dbt_lhdcv5_codec;return dbt_codec_open(&decoder,&cfg);
}
EX int math_packet(const uint8_t *data,uint16_t size,uint16_t sequence,uint8_t *out,uint32_t out_size,uint32_t *written){
    dbt_avdtp_media_t p={.payload=data,.size=size,.sequence=sequence,.ssrc=1,.timestamp=(uint32_t)sequence*decoder.config.samples*4,.payload_type=96};
    destination=out;capacity=out_size;used=0;dbt_result_t r=decoder.config.ops->media(&decoder,&p,sequence);
    *written=used;destination=0;return r;
}
EX int math_close(void){dbt_codec_close(&decoder);return live?1:0;}
