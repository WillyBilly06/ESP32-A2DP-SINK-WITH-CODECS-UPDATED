#include "dbt_aac.h"
#include "aacdec.h"
#include "aaccommon.h"
#include "coder.h"

#ifdef AAC_ENABLE_SBR
#error "The native AAC-LC adapter requires SBR to be disabled"
#endif
_Static_assert(AAC_MAX_NCHANS==2,"The native PCM workspace requires exactly two-channel Helix bounds");
/* Provided only by the reviewed, generated channel-guarded SDK unit. */
extern int dbt_helix_channel_guard(void);

typedef struct {
    HAACDecoder codec;
    dbt_aac_mux_t mux;
    uint8_t *fragment;
    uint32_t fragment_at,fragment_timestamp,fragment_ssrc;
    uint16_t fragment_size,fragment_capacity;
    uint32_t workspace_bytes;
    uint32_t last_ssrc,discard_timestamp,discard_ssrc;
    uint16_t last_sequence;
    bool sequence_valid,discard_active;
    /* Aligned workspace, PCM and padded raw input follow this header. */
} aac_state_t;
static uint32_t align8(uint32_t n){return (n+7u)&~7u;}
static void zero(void *p,unsigned n){uint8_t *v=p;for(unsigned i=0;i<n;++i)v[i]=0;}
static uint8_t *workspace(aac_state_t *s){return (uint8_t *)s+align8(sizeof(*s));}
static int16_t *pcm(aac_state_t *s){return (int16_t *)(workspace(s)+s->workspace_bytes);}
static uint8_t *raw(aac_state_t *s){return (uint8_t *)(pcm(s)+2048u);}
static void fragment_clear(dbt_codec_decoder_t *d){
    aac_state_t *s=d->state;if(!s)return;
    if(s->fragment)dbt_codec_release(d,s->fragment);
    s->fragment=0;s->fragment_size=s->fragment_capacity=0;
}
static void abandon_fragment(dbt_codec_decoder_t *d){
    aac_state_t *s=d->state;
    if(s && s->fragment){
        s->discard_active=true;s->discard_timestamp=s->fragment_timestamp;s->discard_ssrc=s->fragment_ssrc;
        fragment_clear(d);
    }
}
static void close_decoder(dbt_codec_decoder_t *d){
    if(!d->state)return;
    fragment_clear(d);dbt_codec_release(d,d->state);d->state=0;
}
static dbt_result_t reset_decoder(dbt_codec_decoder_t *d){
    aac_state_t *s=d->state;if(!s)return DBT_INVALID;
    abandon_fragment(d);s->sequence_valid=false;
    if(AACFlushCodec(s->codec)!=ERR_AAC_NONE)return DBT_INVALID;
    AACFrameInfo info;zero(&info,sizeof(info));info.nChans=d->config.channels;info.sampRateCore=(int)d->config.rate;info.profile=AAC_PROFILE_LC;
    return AACSetRawBlockParams(s->codec,0,&info)==ERR_AAC_NONE?DBT_OK:DBT_INVALID;
}
static dbt_result_t open_decoder(dbt_codec_decoder_t *d){
    dbt_codec_config_t checked;
    if(dbt_helix_channel_guard()!=1 || dbt_aac_configure(d->config.cie,d->config.cie_size,&checked)!=DBT_OK ||
       checked.rate!=d->config.rate || checked.channels!=d->config.channels ||
       d->config.samples!=1024 || d->config.sample_bytes!=2)return DBT_INVALID;
    uint32_t work=align8(sizeof(AACDecInfo))+align8(sizeof(PSInfoBase));
    /* The codec can discover a mismatched channel layout before returning an
     * error. Its documented output bound is two channels even for mono CIE. */
    uint32_t bytes=align8(sizeof(aac_state_t))+work+4096u+768u*d->config.channels+4u;
    aac_state_t *s=(aac_state_t *)dbt_codec_allocate(d,bytes);if(!s)return DBT_NO_SPACE;
    zero(s,bytes);d->state=s;s->workspace_bytes=work;s->codec=AACInitDecoderPre(workspace(s),(int)work);
    if(!s->codec)return DBT_INVALID;
    return reset_decoder(d);
}
static dbt_result_t decode_mux(dbt_codec_decoder_t *d,const uint8_t *data,uint16_t size,uint32_t timestamp){
    aac_state_t *s=d->state;dbt_aac_unit_t units[DBT_AAC_MAX_SUBFRAMES];uint8_t count=0;
    dbt_result_t result=dbt_aac_latm_parse(&s->mux,&d->config,data,size,units,DBT_AAC_MAX_SUBFRAMES,&count);
    if(result!=DBT_OK)return result;
    for(unsigned i=0;i<count;++i){
        uint8_t *input=raw(s);
        result=dbt_aac_latm_copy(data,size,units[i],input,(uint16_t)(768u*d->config.channels));
        if(result!=DBT_OK)return result;
        for(unsigned n=0;n<4;++n)input[units[i].size+n]=0;
        unsigned char *cursor=input;int left=units[i].size;
        if(AACDecode(s->codec,&cursor,&left,pcm(s))!=ERR_AAC_NONE || left)return DBT_INVALID;
        AACFrameInfo info;AACGetLastFrameInfo(s->codec,&info);
        if(info.nChans!=d->config.channels || info.sampRateOut!=(int)d->config.rate ||
           info.outputSamps!=1024*d->config.channels || info.bitsPerSample!=16 || info.profile!=AAC_PROFILE_LC)return DBT_INVALID;
        result=d->hooks.pcm(d->hooks.context,pcm(s),1024,&d->config,timestamp+i*1024u);
        if(result!=DBT_OK)return result;
        ++d->frames;
    }
    return DBT_OK;
}
static void expire(dbt_codec_decoder_t *d,uint32_t now){
    aac_state_t *s=d->state;if(s && s->fragment && now-s->fragment_at>=100u)abandon_fragment(d);
}
static dbt_result_t media(dbt_codec_decoder_t *d,const dbt_avdtp_media_t *p,uint32_t now){
    aac_state_t *s=d->state;
    if(!s || !p || !p->payload || !p->size || p->payload_type<96)return DBT_INVALID;
    expire(d,now);
    if(s->sequence_valid && (p->ssrc!=s->last_ssrc || p->sequence!=(uint16_t)(s->last_sequence+1u))){
        if(reset_decoder(d)!=DBT_OK)return DBT_INVALID;
    }
    s->sequence_valid=true;s->last_sequence=p->sequence;s->last_ssrc=p->ssrc;
    if(s->fragment && (p->timestamp!=s->fragment_timestamp || p->ssrc!=s->fragment_ssrc))abandon_fragment(d);
    if(s->discard_active){
        if(p->timestamp==s->discard_timestamp && p->ssrc==s->discard_ssrc){
            if(p->marker)s->discard_active=false;
            return DBT_INVALID;
        }
        s->discard_active=false;
    }
    if(!s->fragment && p->marker)return decode_mux(d,p->payload,p->size,p->timestamp);
    uint32_t needed=(uint32_t)s->fragment_size+p->size;
    if(needed>DBT_AAC_MAX_MUX_BYTES){abandon_fragment(d);s->discard_active=true;s->discard_timestamp=p->timestamp;s->discard_ssrc=p->ssrc;return DBT_INVALID;}
    if(needed>s->fragment_capacity){
        uint32_t capacity=256;while(capacity<needed)capacity<<=1;
        uint8_t *next=dbt_codec_allocate(d,capacity);if(!next){abandon_fragment(d);s->discard_active=true;s->discard_timestamp=p->timestamp;s->discard_ssrc=p->ssrc;return DBT_NO_SPACE;}
        for(unsigned i=0;i<s->fragment_size;++i)next[i]=s->fragment[i];
        if(s->fragment)dbt_codec_release(d,s->fragment);
        s->fragment=next;s->fragment_capacity=(uint16_t)capacity;
    }
    if(!s->fragment_size){s->fragment_at=now;s->fragment_timestamp=p->timestamp;s->fragment_ssrc=p->ssrc;}
    for(unsigned i=0;i<p->size;++i)s->fragment[s->fragment_size+i]=p->payload[i];
    s->fragment_size=(uint16_t)needed;
    if(!p->marker)return DBT_OK;
    dbt_result_t result=decode_mux(d,s->fragment,s->fragment_size,s->fragment_timestamp);fragment_clear(d);return result;
}
const dbt_codec_ops_t dbt_aac_codec={"AAC-LC",dbt_aac_configure,open_decoder,close_decoder,reset_decoder,media,expire};
