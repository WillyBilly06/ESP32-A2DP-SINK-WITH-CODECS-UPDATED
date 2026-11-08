#include "dbt_lhdcv5_codec.h"
#ifdef DBT_LHDC_STAGE_PROFILE
extern void dbt_lhdc_stage_live_begin(uint32_t,uint32_t);
extern void dbt_lhdc_stage_live_end(void);
#endif
#include "dbt_lhdc_alloc.h"
#include "lhdc_dec.h"
#include "lhdc_dec_internal.h"
#include "lhdc_entropy_dec.h"
#include "lhdc_imdct.h"
#include "lhdc_dec_slot.h"

_Static_assert(LHDC_NSLOTS==1,"The native LHDC worker requires the single-slot sibling decoder");

/* The encoded header carries a ten-bit count of 16-bit payload words.
 * Packet MTU and fragment assembly are separate limits from one frame. */
#define LHDC_FRAME_MAX (1023u * 2u)
_Static_assert(LHDC_DEC_MAX_FRAME_BYTES>=LHDC_FRAME_MAX,"LHDC decoder payload buffer is smaller than a valid encoded frame");
typedef struct {
#ifdef DBT_LHDC_QUALITY
    struct {uint32_t flags,peak_f32_bits,clipped_samples,generation;int32_t channel_result[2];} quality;
#endif
    lhdc_decoder_t *codec;
    uint32_t workspace_bytes,last_ssrc;
    uint16_t last_sequence;
    bool sequence_valid;
    const uint8_t *batch;
    uint32_t batch_timestamp;
    uint16_t batch_size,batch_pos,batch_index;
} lhdc_state_t;
/* Decoder caches are process-global. The audio product has one worker and
 * this binding prohibits two active owners. All cache allocations are charged
 * to that owner's hooks by the generated decoder units. */
static dbt_codec_decoder_t *active;
static bool allocation_failed;

#ifdef DBT_LHDC_QUALITY
#define QUALITY_FIELDS(X) \
    X(uint32_t,attempts) X(uint32_t,rejected) X(uint32_t,channel0_errors) X(uint32_t,channel1_silenced) \
    X(uint32_t,peak_concealed) X(uint32_t,nonfinite_frames) X(uint32_t,clipped_frames) X(uint32_t,clipped_samples) \
    X(uint32_t,event_attempt) X(uint32_t,generation) X(uint32_t,frame_index) X(uint32_t,rtp_timestamp) X(uint32_t,ssrc) \
    X(uint32_t,sequence) X(uint32_t,frame_bytes) X(uint32_t,flags) X(uint32_t,peak_f32_bits) \
    X(int32_t,result) X(int32_t,channel0_result) X(int32_t,channel1_result)
static struct {
    _Atomic uint32_t revision,enabled;
#define QUALITY_ATOMIC(type,field) _Atomic type field;
    QUALITY_FIELDS(QUALITY_ATOMIC)
#undef QUALITY_ATOMIC
} quality;
static uint32_t quality_serial;
_Static_assert(ATOMIC_INT_LOCK_FREE==2 && sizeof(float)==4,"LHDC quality metadata requires lock-free words and binary32 storage");
void dbt_lhdcv5_quality_start(void){atomic_store_explicit(&quality.enabled,true,memory_order_seq_cst);}
bool dbt_lhdcv5_quality_snapshot(dbt_lhdcv5_quality_t *out){
    if(!out || !atomic_load_explicit(&quality.enabled,memory_order_seq_cst))return false;
    for(unsigned attempt=0;attempt<3;++attempt){
        uint32_t revision=atomic_load_explicit(&quality.revision,memory_order_seq_cst);if(revision&1u)continue;
        dbt_lhdcv5_quality_t value;value.revision=revision;
#define QUALITY_LOAD(type,field) value.field=atomic_load_explicit(&quality.field,memory_order_seq_cst);
        QUALITY_FIELDS(QUALITY_LOAD)
#undef QUALITY_LOAD
        if(revision==atomic_load_explicit(&quality.revision,memory_order_seq_cst)){*out=value;return true;}
    }
    return false;
}
void dbt_lhdc_channel_failure(unsigned channel,int result){
    if(!active || !active->state || channel>1)return;
    lhdc_state_t *s=active->state;s->quality.flags|=channel?DBT_LHDC_CHANNEL1_SILENCED:DBT_LHDC_CHANNEL0_ERROR;
    s->quality.channel_result[channel]=result;
}
void dbt_lhdc_frame_quality(float peak,unsigned clipped_samples,int concealed,int nonfinite){
    if(!active || !active->state)return;
    lhdc_state_t *s=active->state;s->quality.flags|=DBT_LHDC_QUALITY_SCANNED;
    if(concealed)s->quality.flags|=DBT_LHDC_PEAK_CONCEALED;
    if(nonfinite)s->quality.flags|=DBT_LHDC_NONFINITE;
    if(clipped_samples)s->quality.flags|=DBT_LHDC_CLIPPED;
    s->quality.clipped_samples=clipped_samples;
    __builtin_memcpy(&s->quality.peak_f32_bits,&peak,sizeof(peak));
}
static void quality_add(_Atomic uint32_t *value,uint32_t add){
    if(add)atomic_store_explicit(value,atomic_load_explicit(value,memory_order_seq_cst)+add,memory_order_seq_cst);
}
static void quality_commit(const lhdc_state_t *s,uint32_t frame,uint32_t timestamp,uint16_t bytes,int result,bool accepted){
    if(!atomic_load_explicit(&quality.enabled,memory_order_seq_cst))return;
    uint32_t revision=atomic_load_explicit(&quality.revision,memory_order_seq_cst),flags=s->quality.flags;
    if(!accepted)flags|=DBT_LHDC_FRAME_REJECTED;
    atomic_store_explicit(&quality.revision,revision+1u,memory_order_seq_cst);
    quality_add(&quality.attempts,1);quality_add(&quality.rejected,!accepted);
    quality_add(&quality.channel0_errors,(flags&DBT_LHDC_CHANNEL0_ERROR)!=0);
    quality_add(&quality.channel1_silenced,(flags&DBT_LHDC_CHANNEL1_SILENCED)!=0);
    quality_add(&quality.peak_concealed,(flags&DBT_LHDC_PEAK_CONCEALED)!=0);
    quality_add(&quality.nonfinite_frames,(flags&DBT_LHDC_NONFINITE)!=0);
    quality_add(&quality.clipped_frames,(flags&DBT_LHDC_CLIPPED)!=0);quality_add(&quality.clipped_samples,s->quality.clipped_samples);
#define QUALITY_STORE(field,value) atomic_store_explicit(&quality.field,(value),memory_order_seq_cst)
    if(flags&~(uint32_t)DBT_LHDC_QUALITY_SCANNED){
        QUALITY_STORE(event_attempt,atomic_load_explicit(&quality.attempts,memory_order_seq_cst));
        QUALITY_STORE(generation,s->quality.generation);QUALITY_STORE(frame_index,frame);QUALITY_STORE(rtp_timestamp,timestamp);
        QUALITY_STORE(ssrc,s->last_ssrc);QUALITY_STORE(sequence,s->last_sequence);QUALITY_STORE(frame_bytes,bytes);
        QUALITY_STORE(flags,flags);QUALITY_STORE(peak_f32_bits,s->quality.peak_f32_bits);QUALITY_STORE(result,result);
        QUALITY_STORE(channel0_result,s->quality.channel_result[0]);QUALITY_STORE(channel1_result,s->quality.channel_result[1]);
    }
#undef QUALITY_STORE
    atomic_store_explicit(&quality.revision,revision+2u,memory_order_seq_cst);
}
#undef QUALITY_FIELDS
#endif

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
static uint8_t *pcm(lhdc_state_t *s){
#ifdef DBT_LHDC_KEEP_DEDICATED_PCM
    return workspace(s)+s->workspace_bytes;
#else
    /* The single-slot decoder finishes both transforms and overlap-adds
     * before emitting interleaved PCM. Its float sources are pcm_mid and
     * ch_pcm (mdct_in), so mdct_out is dead during that final conversion.
     * The worker fences every PCM loan before decoding/reset/close/reconfigure;
     * the next transform may reuse this storage only after retirement. */
    return (uint8_t *)s->codec->slot[0].mdct_out;
#endif
}
static void close_decoder(dbt_codec_decoder_t *d){
    lhdc_state_t *s=d->state;if(!s)return;

#ifdef DBT_LHDC_STAGE_PROFILE
    dbt_lhdc_stage_live_end();
#endif
    lhdc_dec_deinit((lhdc_decoder_t *)workspace(s));
    lhdc_dec_free_slot_scratch();lhdc_entropy_free();lhdc_dec_free_window();
    lhdc_imdct_free_480();lhdc_imdct_free_960();lhdc_imdct_free_1920();lhdc_imdct_free_cos();
    dbt_codec_release(d,s);d->state=0;if(active==d)active=0;
}
static dbt_result_t reset_decoder(dbt_codec_decoder_t *d){
    lhdc_state_t *s=d->state;if(!s || !s->codec)return DBT_INVALID;
    s->batch=0;s->batch_size=s->batch_pos=s->batch_index=0;
    s->sequence_valid=false;lhdc_dec_reset(s->codec);return DBT_OK;
}
static dbt_result_t open_decoder(dbt_codec_decoder_t *d){
    dbt_codec_config_t checked;
    if(dbt_lhdcv5_configure(d->config.cie,d->config.cie_size,&checked)!=DBT_OK || checked.rate!=d->config.rate ||
       checked.channels!=d->config.channels || checked.samples!=d->config.samples || checked.sample_bytes!=d->config.sample_bytes)return DBT_INVALID;
    if(active)return DBT_BUSY;
    size_t needed=lhdc_dec_get_workspace_size(checked.rate,5);if(!needed || needed>UINT32_MAX-8192u)return DBT_INVALID;
    uint32_t work=align8((uint32_t)needed),bytes=align8(sizeof(lhdc_state_t))+work;
#ifdef DBT_LHDC_KEEP_DEDICATED_PCM
    bytes+=(uint32_t)checked.samples*checked.channels*checked.sample_bytes;
#endif
    lhdc_state_t *s=(lhdc_state_t *)dbt_codec_allocate(d,bytes);if(!s)return DBT_NO_SPACE;
    zero(s,bytes);s->workspace_bytes=work;d->state=s;active=d;allocation_failed=false;
    lhdc_dec_config_t c={(lhdc_dec_sample_rate_t)checked.rate,
        (lhdc_dec_bitdepth_t)((checked.cie[9]&7u)==4?16:(checked.cie[9]&7u)==2?24:32),
        LHDC_DEC_FRAME_5MS,2,LHDC_FRAME_MAX,0};
    s->codec=lhdc_dec_init(workspace(s),&c);
    if(!s->codec || !dbt_lhdc_entropy_ready() || allocation_failed)return DBT_NO_SPACE;
#ifndef DBT_LHDC_KEEP_DEDICATED_PCM
    /* Parameters come from the configured CIE, never packet-controlled rate
     * changes. Allocation is fixed until a worker-fenced reconfiguration.
     * Both S16 and S32 stereo containers fit the M-float transform buffer. */
    if(!pcm(s) || s->codec->alloc_mdct_size!=2*(int)checked.samples ||
       (uint32_t)checked.samples*checked.channels*checked.sample_bytes>
       (uint32_t)s->codec->alloc_mdct_size*sizeof(float))return DBT_INVALID;
#endif
#ifdef DBT_LHDC_QUALITY
    if(++quality_serial==0)++quality_serial;
    s->quality.generation=quality_serial;
#endif
#ifdef DBT_LHDC_STAGE_PROFILE
    dbt_lhdc_stage_live_begin(checked.rate,checked.sample_bytes);
#endif
    return DBT_OK;
}
static dbt_result_t batch_finish(dbt_codec_decoder_t *d,dbt_result_t result){
    lhdc_state_t *s=d->state;
    s->batch=0;s->batch_size=s->batch_pos=s->batch_index=0;
    return result;
}
static dbt_result_t resume(dbt_codec_decoder_t *d,uint32_t now){
    (void)now;lhdc_state_t *s=d->state;if(!s)return DBT_INVALID;
    if(!s->batch)return DBT_OK;
    while(s->batch_pos<s->batch_size){
        const uint8_t *p=s->batch;unsigned pos=s->batch_pos;
        unsigned expected=2u+2u*(((unsigned)p[pos]|((unsigned)p[pos+1]<<8))&1023u);
        size_t consumed=0;uint32_t generated=0;lhdc_dec_frame_info_t info={0};allocation_failed=false;
#ifdef DBT_LHDC_QUALITY
        s->quality.flags=s->quality.peak_f32_bits=s->quality.clipped_samples=0;
        s->quality.channel_result[0]=s->quality.channel_result[1]=0;
        uint32_t frame_index=s->codec->frame_index;
#endif
        lhdc_dec_ret_t r=lhdc_dec_decode_frame(s->codec,p+pos,expected,pcm(s),d->config.samples,&consumed,&generated,&info);
        bool valid=r==LHDC_DEC_OK && !allocation_failed && consumed==expected && generated==d->config.samples && info.samples_per_channel==generated &&
            info.channels==d->config.channels && info.sample_rate==d->config.rate && info.bit_depth==8u*d->config.sample_bytes && info.frame_duration_ms==5;
#ifdef DBT_LHDC_QUALITY
        if(allocation_failed)s->quality.flags|=DBT_LHDC_ALLOCATION_FAILED;
        if(r==LHDC_DEC_OK && !allocation_failed && !valid)s->quality.flags|=DBT_LHDC_FORMAT_MISMATCH;
        quality_commit(s,frame_index,s->batch_timestamp+(uint32_t)s->batch_index*d->config.samples,(uint16_t)expected,(int)r,valid);
#endif
        if(!valid)return batch_finish(d,DBT_INVALID);
        dbt_result_t result=d->hooks.pcm(d->hooks.context,pcm(s),(uint16_t)generated,&d->config,
            s->batch_timestamp+(uint32_t)s->batch_index*generated);
        if(result!=DBT_OK && result!=DBT_PENDING)return batch_finish(d,result);
        ++d->frames;s->batch_pos=(uint16_t)(pos+expected);++s->batch_index;
        if(result==DBT_PENDING)return result;
    }
    return batch_finish(d,DBT_OK);
}
static unsigned payload_frame_count(const uint8_t *p,uint16_t size,uint8_t declared_frames){
    if(!p || size<4 || !declared_frames)return 0;
    unsigned pos=0,frames=0;
    while(pos<size){
        if(size-pos<2)return 0;
        unsigned n=((unsigned)p[pos]|((unsigned)p[pos+1]<<8))&1023u;n*=2u;
        if(!n || n>LHDC_FRAME_MAX || n+2u>size-pos)return 0;
        pos+=n+2u;if(++frames>declared_frames)return 0;
    }
    return frames==declared_frames?frames:0;
}
uint32_t dbt_lhdcv5_packet_frames(const dbt_codec_config_t *config,const dbt_avdtp_media_t *p){
    if(!config || !config->samples || !p || !p->payload || p->size<6 || p->payload_type<96 ||
       p->payload_type==DBT_AVDTP_RAW_PAYLOAD_TYPE)return 0;
    uint8_t count=(uint8_t)(p->payload[0]>>2);
    return payload_frame_count(p->payload+2,(uint16_t)(p->size-2u),count)*(uint32_t)config->samples;
}
static dbt_result_t decode_payload(dbt_codec_decoder_t *d,const uint8_t *p,uint16_t size,uint8_t declared_frames,uint32_t timestamp){
    lhdc_state_t *s=d->state;
    if(!payload_frame_count(p,size,declared_frames))return DBT_INVALID;
    s->batch=p;s->batch_size=size;s->batch_pos=0;s->batch_index=0;
    s->batch_timestamp=timestamp;
    return resume(d,0);
}
static dbt_result_t media(dbt_codec_decoder_t *d,const dbt_avdtp_media_t *p,uint32_t now){
    (void)now;
    lhdc_state_t *s=d->state;if(!s || !p || !p->payload || p->size<6 || p->payload_type<96)return DBT_INVALID;
    if(s->batch)return DBT_BUSY;
    if(s->sequence_valid && (p->ssrc!=s->last_ssrc || p->sequence!=(uint16_t)(s->last_sequence+1u)))if(reset_decoder(d)!=DBT_OK)return DBT_INVALID;
    s->sequence_valid=true;s->last_ssrc=p->ssrc;s->last_sequence=p->sequence;
    /* LHDC V5 has a two-octet media header. The encoder places its frame
     * count in bits 7:2 and its packet sequence in the second octet. Those
     * bits are not SBC F/S/L fragmentation flags. L2CAP reassembly has
     * already completed before this immutable payload reaches the worker. */
    uint8_t count=(uint8_t)(p->payload[0]>>2);
    if(!count || (uint32_t)count*4u>p->size-2u)return DBT_INVALID;
    return decode_payload(d,p->payload+2,(uint16_t)(p->size-2u),count,p->timestamp);
}
const dbt_codec_ops_t dbt_lhdcv5_codec={"LHDC V5",dbt_lhdcv5_configure,open_decoder,close_decoder,reset_decoder,media,0,resume,0};
