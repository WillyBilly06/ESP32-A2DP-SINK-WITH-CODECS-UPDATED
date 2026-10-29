#include "dbt_lhdcv5_codec.h"
#include "dbt_lhdc_alloc.h"
#include "lhdc_dec.h"
#include "lhdc_entropy_dec.h"
#include "lhdc_imdct.h"
#include "lhdc_dec_slot.h"

_Static_assert(LHDC_NSLOTS==1,"The native LHDC worker requires the single-slot sibling decoder");

#define LHDC_FRAGMENT_MAX 3072u
#define LHDC_FRAME_MAX 1536u
typedef struct {
    lhdc_decoder_t *codec;
    uint8_t *fragment;
    uint32_t workspace_bytes,fragment_at,timestamp,ssrc,last_ssrc;
    uint16_t fragment_size,fragment_capacity,next_sequence,last_sequence;
    uint8_t remaining;
    bool sequence_valid;
} lhdc_state_t;
/* Decoder caches are process-global. The audio product has one worker and
 * this binding prohibits two active owners. All cache allocations are charged
 * to that owner's hooks by the generated decoder units. */
static dbt_codec_decoder_t *active;
static bool allocation_failed;
void *dbt_lhdc_mem_alloc(size_t size){
    if(!active || !size || size>UINT32_MAX){allocation_failed=true;return 0;}
    void *p=dbt_codec_allocate(active,(uint32_t)size);if(!p)allocation_failed=true;return p;
}
void dbt_lhdc_mem_free(void *p){if(active && p)dbt_codec_release(active,p);}
static uint32_t align8(uint32_t n){return (n+7u)&~7u;}
static void zero(void *p,unsigned n){uint8_t *b=p;for(unsigned i=0;i<n;++i)b[i]=0;}
static void copy(void *to,const void *from,unsigned n){uint8_t *d=to;const uint8_t *s=from;for(unsigned i=0;i<n;++i)d[i]=s[i];}
static bool one(unsigned v){return v && !(v&(v-1u));}
dbt_result_t dbt_lhdcv5_configure(const uint8_t *p,uint16_t size,dbt_codec_config_t *out){
    if(!p || !out || size!=13 || p[0] || p[1]!=0xff || p[2]!=0x3a || p[3]!=5 || p[4] || p[5] || p[6]!=0x35 || p[7]!=0x4c)return DBT_UNSUPPORTED;
    unsigned rate=p[8],depth=p[9]&7u;
    if(!one(rate) || !(rate&0x35u) || !one(depth) || (p[9]&8u) || p[10]!=0x11 || (p[11]&~0x40u) || p[12])return DBT_UNSUPPORTED;
    static const uint16_t minimum[]={64,160,256,400},maximum[]={1000,400,500,900};
    if(minimum[(p[9]>>6)&3u]>maximum[(p[9]>>4)&3u])return DBT_INVALID;
    dbt_codec_config_t c={0};
    c.rate=rate==1?192000:rate==4?96000:rate==16?48000:44100;
    c.samples=(uint16_t)(c.rate==44100?240:c.rate/200u);c.channels=2;c.sample_bytes=depth==4?2:4;c.cie_size=13;
    copy(c.cie,p,13);*out=c;return DBT_OK;
}
uint8_t dbt_a2dp_lhdcv5_validate(const dbt_avdtp_endpoint_t *endpoint,const dbt_avdtp_cap_t *codec){
    if(!endpoint || !codec || codec->category!=7 || !codec->data)return 0x18;
    dbt_codec_config_t c;dbt_avdtp_caps_t local;dbt_avdtp_cap_error_t error;
    if(dbt_lhdcv5_configure(codec->data,codec->size,&c)!=DBT_OK ||
       dbt_avdtp_caps_validate(endpoint->capabilities,endpoint->size,DBT_AVDTP_CAPABILITIES,&local,&error)!=DBT_OK || local.codec.size!=13)return 0x29;
    const uint8_t *p=codec->data,*cap=local.codec.data;
    for(unsigned i=0;i<8;++i)if(p[i]!=cap[i])return 0x29;
    if(!(p[8]&cap[8]) || !(p[9]&cap[9]&7u) || (p[10]&cap[10])!=p[10] || (p[11]&cap[11])!=p[11])return 0x29;
    static const uint16_t minimum[]={64,160,256,400},maximum[]={1000,400,500,900};
    return minimum[(p[9]>>6)&3u]<minimum[(cap[9]>>6)&3u] || maximum[(p[9]>>4)&3u]>maximum[(cap[9]>>4)&3u]?0x29:0;
}
static uint8_t *workspace(lhdc_state_t *s){return (uint8_t *)s+align8(sizeof(*s));}
static uint8_t *pcm(lhdc_state_t *s){return workspace(s)+s->workspace_bytes;}
static void clear_fragment(dbt_codec_decoder_t *d){
    lhdc_state_t *s=d->state;if(!s)return;if(s->fragment)dbt_codec_release(d,s->fragment);
    s->fragment=0;s->fragment_size=s->fragment_capacity=s->remaining=0;
}
static void close_decoder(dbt_codec_decoder_t *d){
    lhdc_state_t *s=d->state;if(!s)return;
    clear_fragment(d);lhdc_dec_deinit((lhdc_decoder_t *)workspace(s));
    lhdc_dec_free_slot_scratch();lhdc_entropy_free();lhdc_dec_free_window();
    lhdc_imdct_free_480();lhdc_imdct_free_960();lhdc_imdct_free_1920();lhdc_imdct_free_cos();
    dbt_codec_release(d,s);d->state=0;if(active==d)active=0;
}
static dbt_result_t reset_decoder(dbt_codec_decoder_t *d){
    lhdc_state_t *s=d->state;if(!s || !s->codec)return DBT_INVALID;
    clear_fragment(d);s->sequence_valid=false;lhdc_dec_reset(s->codec);return DBT_OK;
}
static dbt_result_t open_decoder(dbt_codec_decoder_t *d){
    dbt_codec_config_t checked;
    if(dbt_lhdcv5_configure(d->config.cie,d->config.cie_size,&checked)!=DBT_OK || checked.rate!=d->config.rate ||
       checked.channels!=d->config.channels || checked.samples!=d->config.samples || checked.sample_bytes!=d->config.sample_bytes)return DBT_INVALID;
    if(active)return DBT_BUSY;
    size_t needed=lhdc_dec_get_workspace_size(checked.rate,5);if(!needed || needed>UINT32_MAX-8192u)return DBT_INVALID;
    uint32_t work=align8((uint32_t)needed),bytes=align8(sizeof(lhdc_state_t))+work+(uint32_t)checked.samples*checked.channels*checked.sample_bytes;
    lhdc_state_t *s=(lhdc_state_t *)dbt_codec_allocate(d,bytes);if(!s)return DBT_NO_SPACE;
    zero(s,bytes);s->workspace_bytes=work;d->state=s;active=d;allocation_failed=false;
    lhdc_dec_config_t c={(lhdc_dec_sample_rate_t)checked.rate,
        (lhdc_dec_bitdepth_t)((checked.cie[9]&7u)==4?16:(checked.cie[9]&7u)==2?24:32),
        LHDC_DEC_FRAME_5MS,2,LHDC_FRAME_MAX,0};
    s->codec=lhdc_dec_init(workspace(s),&c);
    return s->codec && dbt_lhdc_entropy_ready() && !allocation_failed?DBT_OK:DBT_NO_SPACE;
}
static dbt_result_t decode_payload(dbt_codec_decoder_t *d,const uint8_t *p,uint16_t size,uint32_t timestamp){
    lhdc_state_t *s=d->state;if(size<4)return DBT_INVALID;
    unsigned pos=1,frames=0;
    while(pos<size){
        if(size-pos<2)return DBT_INVALID;
        unsigned n=((unsigned)p[pos]|((unsigned)p[pos+1]<<8))&1023u;n*=2u;
        if(!n || n>LHDC_FRAME_MAX || n+2u>size-pos)return DBT_INVALID;
        pos+=n+2u;++frames;
    }
    if(!frames)return DBT_INVALID;
    pos=1;
    for(unsigned i=0;i<frames;++i){
        unsigned expected=2u+2u*(((unsigned)p[pos]|((unsigned)p[pos+1]<<8))&1023u);
        size_t consumed=0;uint32_t generated=0;lhdc_dec_frame_info_t info={0};allocation_failed=false;
        lhdc_dec_ret_t r=lhdc_dec_decode_frame(s->codec,p+pos,expected,pcm(s),d->config.samples,&consumed,&generated,&info);
        if(r!=LHDC_DEC_OK || allocation_failed || consumed!=expected || generated!=d->config.samples || info.samples_per_channel!=generated ||
           info.channels!=d->config.channels || info.sample_rate!=d->config.rate || info.bit_depth!=8u*d->config.sample_bytes || info.frame_duration_ms!=5)return DBT_INVALID;
        dbt_result_t result=d->hooks.pcm(d->hooks.context,pcm(s),(uint16_t)generated,&d->config,timestamp+i*generated);
        if(result!=DBT_OK)return result;
        ++d->frames;pos+=expected;
    }
    return DBT_OK;
}
static void expire(dbt_codec_decoder_t *d,uint32_t now){lhdc_state_t *s=d->state;if(s && s->fragment && now-s->fragment_at>=100u)(void)reset_decoder(d);}
static dbt_result_t media(dbt_codec_decoder_t *d,const dbt_avdtp_media_t *p,uint32_t now){
    lhdc_state_t *s=d->state;if(!s || !p || !p->payload || p->size<2 || p->payload_type<96)return DBT_INVALID;
    expire(d,now);
    if(s->sequence_valid && (p->ssrc!=s->last_ssrc || p->sequence!=(uint16_t)(s->last_sequence+1u)))if(reset_decoder(d)!=DBT_OK)return DBT_INVALID;
    s->sequence_valid=true;s->last_ssrc=p->ssrc;s->last_sequence=p->sequence;
    unsigned h=p->payload[0],count=h&15u;bool fragmented=(h&128u)!=0,first=(h&64u)!=0,last=(h&32u)!=0;
    const uint8_t *data=p->payload+1;uint16_t size=(uint16_t)(p->size-1u);
    if(!fragmented){if(s->fragment && reset_decoder(d)!=DBT_OK)return DBT_INVALID;return decode_payload(d,data,size,p->timestamp);}
    if(!count || (first && (last || count<2)) || (last && count!=1) || (!last && count==1)){clear_fragment(d);return DBT_INVALID;}
    if(first){if(s->fragment && reset_decoder(d)!=DBT_OK)return DBT_INVALID;s->timestamp=p->timestamp;s->ssrc=p->ssrc;s->fragment_at=now;s->remaining=(uint8_t)count;s->next_sequence=p->sequence;}
    if(!s->remaining || count!=s->remaining || p->sequence!=s->next_sequence || p->timestamp!=s->timestamp || p->ssrc!=s->ssrc){clear_fragment(d);return DBT_INVALID;}
    uint32_t needed=(uint32_t)s->fragment_size+size;if(needed>LHDC_FRAGMENT_MAX){clear_fragment(d);return DBT_INVALID;}
    if(needed>s->fragment_capacity){
        uint32_t cap=256;while(cap<needed && cap<2048)cap<<=1;if(cap<needed)cap=LHDC_FRAGMENT_MAX;
        uint8_t *next=dbt_codec_allocate(d,cap);if(!next){clear_fragment(d);return DBT_NO_SPACE;}
        copy(next,s->fragment,s->fragment_size);if(s->fragment)dbt_codec_release(d,s->fragment);s->fragment=next;s->fragment_capacity=(uint16_t)cap;
    }
    copy(s->fragment+s->fragment_size,data,size);s->fragment_size=(uint16_t)needed;--s->remaining;++s->next_sequence;
    if(!last)return DBT_OK;
    dbt_result_t result=decode_payload(d,s->fragment,s->fragment_size,s->timestamp);clear_fragment(d);return result;
}
const dbt_codec_ops_t dbt_lhdcv5_codec={"LHDC V5",dbt_lhdcv5_configure,open_decoder,close_decoder,reset_decoder,media,expire};
