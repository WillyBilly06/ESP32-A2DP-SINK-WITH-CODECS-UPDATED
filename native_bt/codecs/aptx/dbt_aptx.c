#include "dbt_aptx.h"
#include "freeaptx.h"

extern size_t dbt_aptx_backend_size(void);
extern struct aptx_context *dbt_aptx_backend_init(void *memory,int hd);
typedef struct {struct aptx_context *codec;uint32_t workspace_bytes,next_timestamp;bool clock_valid;} aptx_state_t;
static uint32_t align8(uint32_t n){return (n+7u)&~7u;}
static uint32_t le32(const uint8_t *p){return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);}
static unsigned le16(const uint8_t *p){return (unsigned)p[0]|((unsigned)p[1]<<8);}
static bool hd(const uint8_t *p){return le32(p+2)==0xd7u && le16(p+6)==0x24u;}
dbt_result_t dbt_aptx_configure(const uint8_t *p,uint16_t size,dbt_codec_config_t *out){
    if(!p || !out || size<9 || p[0] || p[1]!=0xff)return DBT_UNSUPPORTED;
    uint32_t vendor=le32(p+2);unsigned codec=le16(p+6);
    if(vendor==0x4fu && codec==1u){if(size!=9)return DBT_INVALID;}
    else if(hd(p)){if(size!=13)return DBT_INVALID;for(unsigned i=9;i<13;++i)if(p[i])return DBT_UNSUPPORTED;}
    else if(vendor==0xau && codec==2u){if(size!=10)return DBT_INVALID;if(p[9])return DBT_UNSUPPORTED;}
    else return DBT_UNSUPPORTED;
    unsigned rate=p[8]&0xf0u;
    if((rate!=0x10u && rate!=0x20u) || (p[8]&15u)!=2u)return DBT_UNSUPPORTED;
    dbt_codec_config_t c={0};c.rate=rate==0x10u?48000:44100;c.channels=2;c.samples=4;c.sample_bytes=4;c.cie_size=(uint8_t)size;
    for(unsigned i=0;i<size;++i)c.cie[i]=p[i];*out=c;return DBT_OK;
}
uint8_t dbt_a2dp_aptx_validate(const dbt_avdtp_endpoint_t *endpoint,const dbt_avdtp_cap_t *codec){
    if(!endpoint || !codec || codec->category!=7 || !codec->data)return 0x18;
    dbt_codec_config_t checked;dbt_avdtp_caps_t local;dbt_avdtp_cap_error_t error;
    if(dbt_aptx_configure(codec->data,codec->size,&checked)!=DBT_OK ||
       dbt_avdtp_caps_validate(endpoint->capabilities,endpoint->size,DBT_AVDTP_CAPABILITIES,&local,&error)!=DBT_OK ||
       local.codec.size!=codec->size)return 0x29;
    const uint8_t *p=codec->data,*c=local.codec.data;
    if(c[0] || c[1]!=0xff || le32(p+2)!=le32(c+2) || le16(p+6)!=le16(c+6) || (p[8]&c[8])!=p[8])return 0x29;
    return 0;
}
static uint8_t *workspace(aptx_state_t *s){return (uint8_t *)s+align8(sizeof(*s));}
static uint8_t *pcm(aptx_state_t *s){return workspace(s)+s->workspace_bytes;}
static void close_decoder(dbt_codec_decoder_t *d){if(d->state){dbt_codec_release(d,d->state);d->state=0;}}
static dbt_result_t reset_decoder(dbt_codec_decoder_t *d){aptx_state_t *s=d->state;if(!s)return DBT_INVALID;aptx_reset(s->codec);s->clock_valid=false;return DBT_OK;}
static dbt_result_t open_decoder(dbt_codec_decoder_t *d){
    dbt_codec_config_t c;
    if(dbt_aptx_configure(d->config.cie,d->config.cie_size,&c)!=DBT_OK || c.rate!=d->config.rate || c.channels!=d->config.channels ||
       c.samples!=d->config.samples || c.sample_bytes!=d->config.sample_bytes)return DBT_INVALID;
    size_t required=dbt_aptx_backend_size();if(!required || required>UINT32_MAX-2048u)return DBT_INVALID;
    uint32_t work=align8((uint32_t)required),bytes=align8(sizeof(aptx_state_t))+work+1024u;
    aptx_state_t *s=(aptx_state_t *)dbt_codec_allocate(d,bytes);if(!s)return DBT_NO_SPACE;
    *s=(aptx_state_t){0};d->state=s;s->workspace_bytes=work;s->codec=dbt_aptx_backend_init(workspace(s),hd(c.cie));
    return s->codec?DBT_OK:DBT_INVALID;
}
static dbt_result_t media(dbt_codec_decoder_t *d,const dbt_avdtp_media_t *p,uint32_t now){
    (void)now;aptx_state_t *s=d->state;
    if(!s || !p || !p->payload || !p->size || p->payload_type<96)return DBT_INVALID;
    unsigned quantum=hd(d->config.cie)?6u:4u;if(p->size%quantum)return DBT_INVALID;
    if(!s->clock_valid){s->next_timestamp=p->timestamp;s->clock_valid=true;}
    size_t offset=0;
    while(offset<p->size){
        size_t written=0,processed=aptx_decode32(s->codec,p->payload+offset,p->size-offset,pcm(s),1024,&written);
        if(!processed || processed>p->size-offset || processed%quantum || written>1024 || written%8)return DBT_INVALID;
        offset+=processed;
        if(written){uint16_t frames=(uint16_t)(written/8u);dbt_result_t r=d->hooks.pcm(d->hooks.context,pcm(s),frames,&d->config,s->next_timestamp);
            if(r!=DBT_OK)return r;s->next_timestamp+=frames;d->frames+=frames;}
    }
    return DBT_OK;
}
const dbt_codec_ops_t dbt_aptx_codec={"aptX",dbt_aptx_configure,open_decoder,close_decoder,reset_decoder,media,0};
