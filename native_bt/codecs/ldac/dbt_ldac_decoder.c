#include "dbt_ldac.h"
#include "dbt_ldac_port.h"
#include <string.h>

/* The SDK codec has process-global tables; the product owns one codec worker. */
static dbt_codec_decoder_t *active;
static bool allocation_failed;
uint32_t dbt_ldac_port_bit_limit;
int dbt_ldac_port_overread;
typedef struct {
    HANDLE_LDAC_BT codec;uint32_t pcm_bytes,work_bytes;
    const uint8_t *batch;
    dbt_ldac_frame_t batch_frames[DBT_LDAC_MAX_FRAMES];
    uint32_t batch_timestamp;
    uint8_t batch_count,batch_index;
    bool heap_setup;
} ldac_state_t;
static uint32_t align8(uint32_t n){return (n+7u)&~7u;}
static uint8_t *planar(ldac_state_t *s){return (uint8_t *)s+align8(sizeof(*s));}
static uint8_t *pcm(ldac_state_t *s){return planar(s)+s->pcm_bytes;}
static uint8_t *encoded(ldac_state_t *s){return pcm(s)+s->pcm_bytes;}
static uint8_t *work(ldac_state_t *s){return encoded(s)+520u;}
void *dbt_ldac_mem_alloc(size_t size){
    ldac_state_t *s=active?active->state:0;
    /* Only initial SDK handle construction may reach the external allocator. */
    if(!s || !s->heap_setup || !size || size>UINT32_MAX){allocation_failed=true;return 0;}
    void *p=dbt_codec_allocate(active,(uint32_t)size);if(!p)allocation_failed=true;return p;
}
void *dbt_ldac_mem_calloc(size_t count,size_t size){
    if(!count || !size || count>SIZE_MAX/size){allocation_failed=true;return 0;}
    size_t bytes=count*size;void *p=dbt_ldac_mem_alloc(bytes);
    if(p)memset(p,0,bytes);
    return p;
}
void *dbt_ldac_work_calloc(char **cursor,size_t count,size_t size){
    ldac_state_t *s=active?active->state:0;
    if(!s || !cursor || !*cursor || !count || !size || size>SIZE_MAX-7u){allocation_failed=true;return 0;}
    size_t unit=(size+7u)&~(size_t)7u;
    if(count>SIZE_MAX/unit){allocation_failed=true;return 0;}
    size_t bytes=count*unit;uintptr_t base=(uintptr_t)work(s),at=(uintptr_t)*cursor;
    if(s->work_bytes>UINTPTR_MAX-base){allocation_failed=true;return 0;}
    uintptr_t end=base+s->work_bytes;
    if(at<base || at>end || (at&7u) || bytes>end-at){allocation_failed=true;return 0;}
    void *p=*cursor;memset(p,0,bytes);*cursor+=bytes;return p;
}
void dbt_ldac_mem_free(void *p){
    if(!active || !p)return;
    ldac_state_t *s=active->state;
    if(s){
        uintptr_t at=(uintptr_t)p,base=(uintptr_t)work(s);
        /* SDK partial-initialization cleanup may visit arena blocks directly. */
        if(at>=base && at-base<s->work_bytes)return;
    }
    dbt_codec_release(active,p);
}
static void close_decoder(dbt_codec_decoder_t *d){
    ldac_state_t *s=d->state;if(!s)return;
    if(s->codec){
        ldacBT_close_handle(s->codec);
        /* Close skips caller-owned blocks. Detach those pointers before freeing
         * the separately owned SDK library handle, whose free otherwise skips. */
        dbt_ldac_work_unbind(s->codec);ldacBT_free_handle(s->codec);
    }
    dbt_codec_release(d,s);d->state=0;if(active==d)active=0;
}
static dbt_result_t reset_decoder(dbt_codec_decoder_t *d){
    ldac_state_t *s=d->state;if(!s || !s->codec || active!=d)return DBT_INVALID;
    s->batch=0;s->batch_count=s->batch_index=0;allocation_failed=false;
    if(!dbt_ldac_work_bind(s->codec,work(s)))return DBT_INVALID;
    int result=ldacBT_init_handle_decode(s->codec,d->config.cie[9],(int)d->config.rate,0,0,0);
    if(result || allocation_failed || !dbt_ldac_bind_pcm(s->codec,planar(s),d->config.samples,d->config.channels)){
        /* All working blocks remain in the retained arena. A failed reset
         * closes worker admission; its explicit DROP/close reclaims handles.
         * Open failure is also closed by the generic codec-open contract. */
        return allocation_failed?DBT_NO_SPACE:DBT_INVALID;
    }
    return DBT_OK;
}
static dbt_result_t open_decoder(dbt_codec_decoder_t *d){
    dbt_codec_config_t checked;
    if(dbt_ldac_configure(d->config.cie,d->config.cie_size,&checked)!=DBT_OK || checked.rate!=d->config.rate ||
       checked.channels!=d->config.channels || checked.samples!=d->config.samples || checked.sample_bytes!=d->config.sample_bytes)return DBT_INVALID;
    if(active)return DBT_BUSY;
    uint32_t work_bytes=dbt_ldac_work_bytes(checked.samples,checked.cie[9]);
    if(!work_bytes)return DBT_INVALID;
    uint32_t pcm_bytes=(uint32_t)checked.samples*checked.channels*4u;
    uint64_t bytes=(uint64_t)align8(sizeof(ldac_state_t))+2u*pcm_bytes+520u+work_bytes;
    if(bytes>UINT32_MAX)return DBT_INVALID;
    ldac_state_t *s=(ldac_state_t *)dbt_codec_allocate(d,(uint32_t)bytes);if(!s)return DBT_NO_SPACE;
    memset(s,0,(size_t)bytes);s->pcm_bytes=pcm_bytes;s->work_bytes=work_bytes;
    d->state=s;active=d;allocation_failed=false;s->heap_setup=true;
    s->codec=ldacBT_get_handle();s->heap_setup=false;
    if(!s->codec || allocation_failed)return DBT_NO_SPACE;
    return reset_decoder(d);
}
static dbt_result_t batch_finish(ldac_state_t *s,dbt_result_t result){
    s->batch=0;s->batch_count=s->batch_index=0;return result;
}
static dbt_result_t resume(dbt_codec_decoder_t *d,uint32_t now){
    (void)now;ldac_state_t *s=d->state;
    if(!s)return DBT_INVALID;
    if(!s->batch)return DBT_OK;
    while(s->batch_index<s->batch_count){
        unsigned i=s->batch_index;
        uint16_t n=s->batch_frames[i].size;uint8_t *input=encoded(s);
        if(n>515u)return batch_finish(s,DBT_INVALID);
        memcpy(input,s->batch+s->batch_frames[i].offset,n);
        input[n]=input[n+1]=0;
        dbt_ldac_port_bit_limit=(uint32_t)(n-3u)*8u;dbt_ldac_port_overread=0;allocation_failed=false;
        int used=0,written=0;
        int r=ldacBT_decode(s->codec,input,pcm(s),LDACBT_SMPL_FMT_S32,n,&used,&written);
        if(r || allocation_failed || dbt_ldac_port_overread || used!=n || written!=(int)s->pcm_bytes)return batch_finish(s,DBT_INVALID);
        dbt_result_t result=d->hooks.pcm(d->hooks.context,pcm(s),d->config.samples,&d->config,s->batch_timestamp+i*d->config.samples);
        if(result!=DBT_OK && result!=DBT_PENDING)return batch_finish(s,result);
        ++d->frames;++s->batch_index;
        if(result==DBT_PENDING)return result;
    }
    return batch_finish(s,DBT_OK);
}
static dbt_result_t media(dbt_codec_decoder_t *d,const dbt_avdtp_media_t *p,uint32_t now){
    ldac_state_t *s=d->state;
    if(!s || !p || !p->payload || p->payload_type<96)return DBT_INVALID;
    if(s->batch)return DBT_BUSY;
    uint8_t count=0;
    dbt_result_t result=dbt_ldac_packet_parse(&d->config,p->payload,p->size,s->batch_frames,DBT_LDAC_MAX_FRAMES,&count);
    if(result!=DBT_OK)return result;
    s->batch=p->payload;s->batch_timestamp=p->timestamp;s->batch_count=count;s->batch_index=0;
    return resume(d,now);
}
const dbt_codec_ops_t dbt_ldac_codec={"LDAC",dbt_ldac_configure,open_decoder,close_decoder,reset_decoder,media,0,resume,0};
