#include "common/bt_target.h"
#include "oi_codec_sbc.h"
#include "dbt_sbc_decoder.h"

typedef struct {
    OI_CODEC_SBC_DECODER_CONTEXT codec;
    uint16_t data_bytes,pcm_bytes,frame_bytes;
    /* uint32 data immediately follows this aligned struct; then PCM and input
     * padding. One active allocation, no permanent decoder or PCM BSS. */
    OI_UINT32 data[];
} decoder_state_t;
_Static_assert(sizeof(OI_INT32)==4 && sizeof(OI_UINT32)==4,"SBC arithmetic requires 32-bit types");
_Static_assert(sizeof(decoder_state_t)%4==0,"Aligned SBC working data");

dbt_result_t dbt_sbc_decoder_init(dbt_sbc_decoder_t *d,dbt_sbc_decoder_hooks_t hooks){
    if(!d || !hooks.allocate || !hooks.release || !hooks.pcm)return DBT_INVALID;
    uint8_t *p=(uint8_t *)d;for(unsigned i=0;i<sizeof(*d);++i)p[i]=0;
    d->hooks=hooks;return DBT_OK;
}
void dbt_sbc_decoder_close(dbt_sbc_decoder_t *d){
    if(!d)return;
    if(d->state)d->hooks.release(d->hooks.context,d->state);
    d->state=0;d->allocated_bytes=0;
}
dbt_result_t dbt_sbc_decoder_reset(dbt_sbc_decoder_t *d){
    if(!d || !d->state)return DBT_INVALID;
    decoder_state_t *s=d->state;
    /* The SDK reset clears its context, but its external synthesis history was
     * historically zero-initialized BSS. Lazy heap state must establish that
     * same cold-stream history explicitly, also after discontinuities. */
    uint8_t *history=(uint8_t *)s->data;
    for(unsigned i=0;i<s->data_bytes;++i)history[i]=0;
    OI_STATUS result=OI_CODEC_SBC_DecoderReset(&s->codec,s->data,s->data_bytes,d->config.channels,d->config.channels,FALSE,FALSE);
    if(result!=OI_OK)return DBT_INVALID;
    return OI_CODEC_SBC_DecoderLimit(&s->codec,FALSE,(d->config.format&1u)?1:0)==OI_OK?DBT_OK:DBT_INVALID;
}
dbt_result_t dbt_sbc_decoder_open(dbt_sbc_decoder_t *d,dbt_sbc_config_t config){
    if(!d || !d->hooks.allocate)return DBT_INVALID;
    uint16_t maximum=0;
    for(unsigned pool=config.maximum_bitpool;pool>=config.minimum_bitpool && pool>0;--pool)
        if(dbt_sbc_frame_bytes(&config,(uint8_t)pool,&maximum)==DBT_OK)break;
    if(!maximum)return DBT_INVALID;
    dbt_sbc_decoder_close(d);
    /* Twenty-seven synthesis buffers per negotiated channel (not two channels
     * unconditionally). Immutable synthesis tables remain in codec .rodata. */
    uint16_t data=(uint16_t)(4u*16u*config.channels*8u+2u*config.channels*8u*27u);
    uint16_t pcm=(uint16_t)(2u*config.channels*config.samples);
    uint16_t bytes=(uint16_t)(sizeof(decoder_state_t)+data+pcm+maximum+4u);
    decoder_state_t *s=(decoder_state_t *)d->hooks.allocate(d->hooks.context,bytes);
    if(!s)return DBT_NO_SPACE;
    if((uintptr_t)s%4u){d->hooks.release(d->hooks.context,(uint8_t *)s);return DBT_INVALID;}
    s->data_bytes=data;s->pcm_bytes=pcm;s->frame_bytes=maximum;d->state=s;d->config=config;d->allocated_bytes=bytes;
    dbt_result_t result=dbt_sbc_decoder_reset(d);
    if(result!=DBT_OK)dbt_sbc_decoder_close(d);
    return result;
}
dbt_result_t dbt_sbc_decoder_frame(void *context,const dbt_sbc_frame_t *frame,uint32_t timestamp){
    dbt_sbc_decoder_t *d=context;
    if(!d || !d->state || !frame)return DBT_INVALID;
    decoder_state_t *s=d->state;dbt_sbc_frame_t checked;
    if(dbt_sbc_frame_decode(&d->config,frame->data,frame->size,&checked)!=DBT_OK ||
       checked.size!=frame->size || frame->size>s->frame_bytes){++d->failures;return DBT_INVALID;}
    int16_t *pcm=(int16_t *)((uint8_t *)s->data+s->data_bytes);
    uint8_t *input=(uint8_t *)pcm+s->pcm_bytes;
    for(unsigned i=0;i<frame->size;++i)input[i]=frame->data[i];
    for(unsigned i=0;i<4;++i)input[frame->size+i]=0;
    const OI_BYTE *cursor=input;OI_UINT32 left=frame->size,written=s->pcm_bytes;
    OI_STATUS status=OI_CODEC_SBC_DecodeFrame(&s->codec,&cursor,&left,pcm,&written);
    if(status!=OI_OK || left || cursor!=input+frame->size || written!=s->pcm_bytes){++d->failures;return DBT_INVALID;}
    dbt_result_t result=d->hooks.pcm(d->hooks.context,pcm,d->config.samples,d->config.channels,d->config.rate,timestamp);
    if(result==DBT_OK)++d->frames;else ++d->failures;
    return result;
}
