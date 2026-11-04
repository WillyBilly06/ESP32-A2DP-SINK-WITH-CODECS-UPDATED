#include "dbt_ldac.h"
#include "dbt_ldac_port.h"

/* The SDK codec has process-global tables; the product owns one codec worker. */
static dbt_codec_decoder_t *active;
static bool allocation_failed;
uint32_t dbt_ldac_port_bit_limit;
int dbt_ldac_port_overread;
typedef struct {HANDLE_LDAC_BT codec;uint32_t pcm_bytes;} ldac_state_t;
void *dbt_ldac_mem_alloc(size_t size){
    if(!active || !size || size>UINT32_MAX){allocation_failed=true;return 0;}
    void *p=dbt_codec_allocate(active,(uint32_t)size);if(!p)allocation_failed=true;return p;
}
void *dbt_ldac_mem_calloc(size_t count,size_t size){
    if(!count || !size || count>SIZE_MAX/size){allocation_failed=true;return 0;}
    size_t bytes=count*size;uint8_t *p=dbt_ldac_mem_alloc(bytes);
    if(p)for(size_t i=0;i<bytes;++i)p[i]=0;
    return p;
}
void dbt_ldac_mem_free(void *p){if(active && p)dbt_codec_release(active,p);}
static uint32_t align8(uint32_t n){return (n+7u)&~7u;}
static uint8_t *planar(ldac_state_t *s){return (uint8_t *)s+align8(sizeof(*s));}
static uint8_t *pcm(ldac_state_t *s){return planar(s)+s->pcm_bytes;}
static uint8_t *encoded(ldac_state_t *s){return pcm(s)+s->pcm_bytes;}
static void close_decoder(dbt_codec_decoder_t *d){
    ldac_state_t *s=d->state;if(!s)return;
    if(s->codec){ldacBT_close_handle(s->codec);ldacBT_free_handle(s->codec);}
    dbt_codec_release(d,s);d->state=0;if(active==d)active=0;
}
static dbt_result_t reset_decoder(dbt_codec_decoder_t *d){
    ldac_state_t *s=d->state;if(!s || !s->codec)return DBT_INVALID;
    allocation_failed=false;
    int result=ldacBT_init_handle_decode(s->codec,d->config.cie[9],(int)d->config.rate,0,0,0);
    if(result || allocation_failed || !dbt_ldac_bind_pcm(s->codec,planar(s),d->config.samples,d->config.channels)){
        bool no_memory=allocation_failed;
        /* Reset first releases the previous codec buffers. A failed replacement
         * must not retain a partially constructed working set while the worker
         * waits for its close/abort job. The worker's packet queue is separate. */
        close_decoder(d);
        return no_memory?DBT_NO_SPACE:DBT_INVALID;
    }
    return DBT_OK;
}
static dbt_result_t open_decoder(dbt_codec_decoder_t *d){
    dbt_codec_config_t checked;
    if(dbt_ldac_configure(d->config.cie,d->config.cie_size,&checked)!=DBT_OK || checked.rate!=d->config.rate ||
       checked.channels!=d->config.channels || checked.samples!=d->config.samples || checked.sample_bytes!=d->config.sample_bytes)return DBT_INVALID;
    if(active)return DBT_BUSY;
    uint32_t pcm_bytes=(uint32_t)checked.samples*checked.channels*4u;
    uint32_t bytes=align8(sizeof(ldac_state_t))+2u*pcm_bytes+517u;
    ldac_state_t *s=(ldac_state_t *)dbt_codec_allocate(d,bytes);if(!s)return DBT_NO_SPACE;
    for(uint32_t i=0;i<bytes;++i)((uint8_t *)s)[i]=0;
    s->pcm_bytes=pcm_bytes;d->state=s;active=d;allocation_failed=false;
    s->codec=ldacBT_get_handle();if(!s->codec || allocation_failed)return DBT_NO_SPACE;
    return reset_decoder(d);
}
static dbt_result_t media(dbt_codec_decoder_t *d,const dbt_avdtp_media_t *p,uint32_t now){
    (void)now;ldac_state_t *s=d->state;
    if(!s || !p || !p->payload || p->payload_type<96)return DBT_INVALID;
    dbt_ldac_frame_t frames[DBT_LDAC_MAX_FRAMES];uint8_t count=0;
    dbt_result_t result=dbt_ldac_packet_parse(&d->config,p->payload,p->size,frames,DBT_LDAC_MAX_FRAMES,&count);
    if(result!=DBT_OK)return result;
    for(unsigned i=0;i<count;++i){
        uint16_t n=frames[i].size;uint8_t *input=encoded(s);
        if(n>515u)return DBT_INVALID;
        for(unsigned j=0;j<n;++j)input[j]=p->payload[frames[i].offset+j];
        input[n]=input[n+1]=0;
        dbt_ldac_port_bit_limit=(uint32_t)(n-3u)*8u;dbt_ldac_port_overread=0;allocation_failed=false;
        int used=0,written=0;
        int r=ldacBT_decode(s->codec,input,pcm(s),LDACBT_SMPL_FMT_S32,n,&used,&written);
        if(r || allocation_failed || dbt_ldac_port_overread || used!=n || written!=(int)s->pcm_bytes)return DBT_INVALID;
        result=d->hooks.pcm(d->hooks.context,pcm(s),d->config.samples,&d->config,p->timestamp+i*d->config.samples);
        if(result!=DBT_OK)return result;
        ++d->frames;
    }
    return DBT_OK;
}
const dbt_codec_ops_t dbt_ldac_codec={"LDAC",dbt_ldac_configure,open_decoder,close_decoder,reset_decoder,media,0};
