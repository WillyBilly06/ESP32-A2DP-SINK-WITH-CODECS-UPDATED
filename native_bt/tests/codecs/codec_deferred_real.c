/* Real algorithms, host allocator and simulated renderer backpressure.
 * Compare every PCM byte and per-sample timestamp with synchronous decoding. */
#include "dbt_codec.h"
#include "dbt_sbc_codec.h"
#include "dbt_opus.h"
#include "dbt_aac.h"
#include "dbt_aptx.h"
#include "dbt_ldac.h"
#include "dbt_lhdcv5_codec.h"
#include "probe_codec_vectors.h"
#include "probe_opus_vectors.h"
#include "probe_lhdc_max_fixture.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
int g_nzc_formula=0,g_pkframe=0;
typedef struct {const probe_codec_vector_t *v;const opus_vector_t *opus;unsigned mode,index;} case_t;
typedef struct {
    void *allocation[64];uint32_t sizes[64],live,peak,allocs,frees;
    uint8_t *reference;uint32_t *timestamps,capacity,reference_frames,compared_frames;
    dbt_codec_config_t config;dbt_aac_mux_t builder_mux;
    bool deferred,want_pause,pause_request,busy_retry;
    unsigned calls,busy_calls;int error;
} context_t;
static uint8_t *allocate(void *context,uint32_t size){
    context_t *c=context;if(!size)return NULL;
    for(unsigned i=0;i<64;++i)if(!c->allocation[i]){
        void *p=malloc(size);if(!p)return NULL;
        if((uintptr_t)p&7u){free(p);return NULL;}
        c->allocation[i]=p;c->sizes[i]=size;c->live+=size;++c->allocs;
        if(c->live>c->peak)c->peak=c->live;return p;
    }
    return NULL;
}
static void release(void *context,uint8_t *p){
    context_t *c=context;
    for(unsigned i=0;i<64;++i)if(c->allocation[i]==p){
        c->allocation[i]=NULL;c->live-=c->sizes[i];++c->frees;free(p);return;
    }
    c->error=__LINE__;
}
static bool live_span(const context_t *c,const void *samples,uint32_t bytes){
    uintptr_t p=(uintptr_t)samples;
    for(unsigned i=0;i<64;++i)if(c->allocation[i] && bytes<=c->sizes[i]){
        uintptr_t base=(uintptr_t)c->allocation[i];
        if(p>=base && p-base<=c->sizes[i]-bytes)return true;
    }
    return false;
}
static dbt_result_t pcm(void *context,const void *samples,uint16_t frames,const dbt_codec_config_t *config,uint32_t timestamp){
    context_t *c=context;++c->calls;
    if(!config || config->rate!=c->config.rate || config->channels!=c->config.channels || config->sample_bytes!=c->config.sample_bytes ||
       !frames || !live_span(c,samples,(uint32_t)frames*config->channels*config->sample_bytes)){
        c->error=__LINE__;return DBT_INVALID;
    }
    uint32_t stride=(uint32_t)config->channels*config->sample_bytes;
    if(!c->deferred){
        if(c->reference_frames+frames>c->capacity){c->error=__LINE__;return DBT_INVALID;}
        memcpy(c->reference+c->reference_frames*stride,samples,frames*stride);
        for(unsigned i=0;i<frames;++i)c->timestamps[c->reference_frames+i]=timestamp+i;
        c->reference_frames+=frames;return DBT_OK;
    }
    if(frames>DBT_CODEC_PCM_CHUNK_FRAMES){c->error=__LINE__;return DBT_INVALID;}
    if(c->want_pause){c->want_pause=false;c->pause_request=true;++c->busy_calls;return DBT_BUSY;}
    if(c->calls%3u==0 && !c->busy_retry){c->busy_retry=true;++c->busy_calls;return DBT_BUSY;}
    c->busy_retry=false;
    if(c->compared_frames+frames>c->reference_frames ||
       memcmp(c->reference+c->compared_frames*stride,samples,frames*stride)){
        c->error=__LINE__;return DBT_INVALID;
    }
    for(unsigned i=0;i<frames;++i)if(c->timestamps[c->compared_frames+i]!=timestamp+i){c->error=__LINE__;return DBT_INVALID;}
    c->compared_frames+=frames;return DBT_OK;
}
static const dbt_codec_ops_t *ops_for(case_t c){return c.v?c.v->ops:c.opus?&dbt_opus_codec:&dbt_lhdcv5_codec;}
static unsigned packet_count(case_t c){return c.v?c.v->packet_count:c.opus?3u:96u;}
static uint32_t expected_frames(case_t c){return c.v?c.v->sample_frames:c.opus?3u*c.opus->samples:92160u;}
static const uint8_t *cie_for(case_t c){return c.v?c.v->cie:c.opus?c.opus->cie:lhdc_max_cie;}
static unsigned cie_size(case_t c){return c.v?c.v->cie_size:c.opus?c.opus->cie_size:sizeof(lhdc_max_cie);}
static void original_packet(case_t c,unsigned i,const uint8_t **data,uint16_t *bytes,bool *marker){
    if(c.v){*data=c.v->packet[i];*bytes=c.v->bytes[i];*marker=c.v->marker!=0;}
    else if(c.opus){*data=c.opus->packet[i];*bytes=c.opus->bytes[i];*marker=false;}
    else {*data=lhdc_max_packets[i].data;*bytes=lhdc_max_packets[i].size;*marker=false;}
}
typedef struct {uint8_t data[8192];unsigned bits;bool failed;} writer_t;
static void put(writer_t *w,unsigned value,unsigned bits){
    for(unsigned i=bits;i;--i){unsigned at=w->bits++;if(at>=sizeof(w->data)*8u){w->failed=true;return;}
        if((value>>(i-1u))&1u)w->data[at/8u]|=(uint8_t)(1u<<(7u-at%8u));}
}
static bool duplicate_packet(case_t c,context_t *ctx,const uint8_t *source,uint16_t size,uint8_t *out,uint16_t *written){
    const dbt_codec_ops_t *ops=ops_for(c);
    if(ops==&dbt_aac_codec){
        dbt_aac_unit_t units[DBT_AAC_MAX_SUBFRAMES];uint8_t count=0;
        if(dbt_aac_latm_parse(&ctx->builder_mux,&ctx->config,source,size,units,DBT_AAC_MAX_SUBFRAMES,&count)!=DBT_OK || count!=1)return false;
        uint8_t raw[DBT_AAC_MAX_RAW_BYTES];if(dbt_aac_latm_copy(source,size,units[0],raw,sizeof(raw))!=DBT_OK)return false;
        writer_t w={0};put(&w,0,1);put(&w,0,1);put(&w,1,1);put(&w,1,6);put(&w,0,4);put(&w,0,3);
        put(&w,2,5);put(&w,ctx->config.rate==48000?3:4,4);put(&w,ctx->config.channels,4);put(&w,0,3);
        put(&w,0,3);put(&w,255,8);put(&w,0,1);put(&w,0,1);
        for(unsigned frame=0;frame<2;++frame){unsigned left=units[0].size;
            while(left>=255){put(&w,255,8);left-=255;}put(&w,left,8);
            for(unsigned i=0;i<units[0].size;++i)put(&w,raw[i],8);}
        if(w.failed)return false;*written=(uint16_t)((w.bits+7u)/8u);memcpy(out,w.data,*written);return true;
    }
    unsigned repeats=ops==&dbt_aptx_codec?3u:2u;
    unsigned prefix=ops==&dbt_aptx_codec?0u:1u;
    if(size<prefix || prefix+repeats*(size-prefix)>8192)return false;
    if(prefix){if((source[0]&15u)!=1u || (source[0]&0xf0u))return false;out[0]=(uint8_t)repeats;}
    for(unsigned i=0;i<repeats;++i)memcpy(out+prefix+i*(size-prefix),source+prefix,size-prefix);
    *written=(uint16_t)(prefix+repeats*(size-prefix));return true;
}
static bool control(dbt_codec_worker_t *w,dbt_sink_operation_t op,const uint8_t *configuration,uint16_t size){
    dbt_sink_job_t job={1,1,op,1,configuration,size};
    if(dbt_codec_worker_submit(w,&job)!=DBT_OK)return false;
    dbt_codec_worker_step(w,0,0);dbt_codec_receipt_t r;
    return dbt_codec_worker_receipt(w,&r)==DBT_OK && r.result==DBT_SINK_WORK_READY;
}
static bool feed(case_t c,context_t *ctx,dbt_codec_decoder_t *direct,dbt_codec_worker_t *worker,unsigned index,uint32_t *now,bool stop_at_pending){
    const uint8_t *data;uint16_t size;bool marker;original_packet(c,index,&data,&size,&marker);
    uint8_t changed[8192];
    if(c.mode==2){if(!duplicate_packet(c,ctx,data,size,changed,&size))return false;data=changed;}
    unsigned parts=c.mode==1?2u:1u;
    for(unsigned part=0;part<parts;++part){
        const uint8_t *p=data;uint16_t n=size;bool end=marker;uint8_t fragment[8192];
        if(parts==2){
            bool aac=ops_for(c)==&dbt_aac_codec;unsigned prefix=aac?0u:1u;
            if(size<=prefix+1u)return false;
            unsigned body=size-prefix,first=(body+1u)/2u,offset=part?first:0u;
            n=(uint16_t)((part?body-first:first)+prefix);
            if(prefix)fragment[0]=part?0xa1:0xc2;
            memcpy(fragment+prefix,data+prefix+offset,n-prefix);p=fragment;end=aac?part!=0:false;
        }
        dbt_avdtp_media_t media={p,0xffff0000u+index*12345u,1,n,(uint16_t)(65535u+index*parts+part),96,end};
        if(!worker){if(direct->config.ops->media(direct,&media,*now)!=DBT_OK)return false;}
        else {
            if(dbt_codec_worker_media(worker,&media)!=DBT_OK)return false;
            /* Producer storage may immediately change; the worker owns its copy. */
            if(p==fragment)memset(fragment,0xcd,n);
            unsigned attempts=0;
            while(atomic_load(&worker->media_read)!=atomic_load(&worker->media_written)){
                if(++attempts>10000 || dbt_codec_worker_step(worker,(*now)++,1)>1 || atomic_load(&worker->faults) || atomic_load(&worker->media_drops))return false;
                if(stop_at_pending && worker->pending_pcm)return true;
                if(ctx->pause_request){*now+=150u;ctx->pause_request=false;}
            }
        }
        ++*now;
    }
    return true;
}
#define TRY(x) do {if(!(x)){line=__LINE__;goto finish;}}while(0)
static int run_case(case_t c){
    context_t ctx={0};dbt_codec_decoder_t direct={0};dbt_codec_worker_t worker={0};
    bool worker_initialized=false;int line=0;uint32_t now=0;unsigned count=packet_count(c);
    const dbt_codec_ops_t *ops=ops_for(c);
    TRY(ops->configure(cie_for(c),(uint16_t)cie_size(c),&ctx.config)==DBT_OK);
    ctx.config.ops=ops;ctx.capacity=expected_frames(c)*(c.mode==2?3u:1u)+8192u;
    uint32_t stride=(uint32_t)ctx.config.channels*ctx.config.sample_bytes;
    ctx.reference=malloc(ctx.capacity*stride);ctx.timestamps=malloc(ctx.capacity*sizeof(uint32_t));
    TRY(ctx.reference && ctx.timestamps);
    dbt_codec_hooks_t hooks={&ctx,allocate,release,pcm};
    TRY(dbt_codec_init(&direct,hooks)==DBT_OK && dbt_codec_open(&direct,&ctx.config)==DBT_OK && ops->reset(&direct)==DBT_OK);
    for(unsigned i=0;i<count;++i)TRY(feed(c,&ctx,&direct,NULL,i,&now,false));
    if(c.mode!=2)TRY(ctx.reference_frames==expected_frames(c));
    else if(ops!=&dbt_aptx_codec)TRY(ctx.reference_frames==2u*expected_frames(c));
    TRY(!ctx.error && ctx.reference_frames);
    dbt_codec_close(&direct);TRY(!ctx.live && ctx.allocs==ctx.frees);
    ctx.deferred=true;ctx.want_pause=true;ctx.calls=0;ctx.builder_mux=(dbt_aac_mux_t){0};now=0;
    const dbt_codec_ops_t *codecs[]={ops};
    dbt_codec_worker_hooks_t wh={hooks,codecs,1,0,0,0,true};
    TRY(dbt_codec_worker_init(&worker,wh,4096,2)==DBT_OK);worker_initialized=true;
    uint8_t configuration[4+DBT_CODEC_CIE_BYTES]={1,0,7,0};configuration[3]=(uint8_t)cie_size(c);
    memcpy(configuration+4,cie_for(c),cie_size(c));
    TRY(control(&worker,DBT_SINK_CONFIGURE,configuration,(uint16_t)(4u+cie_size(c))) && control(&worker,DBT_SINK_START,NULL,0));
    for(unsigned i=0;i<count;++i)TRY(feed(c,&ctx,NULL,&worker,i,&now,false));
    TRY(ctx.compared_frames==ctx.reference_frames && !ctx.error && ctx.busy_calls && !worker.pending_pcm && !worker.media_active);
    TRY(control(&worker,DBT_SINK_DROP,NULL,0) && !ctx.live && ctx.allocs==ctx.frees);
    /* Cancel live borrows on the actual decoder paths, including completed
     * fragment storage. A later START must not replay the old saved cursor. */
    TRY(control(&worker,DBT_SINK_CONFIGURE,configuration,(uint16_t)(4u+cie_size(c))) && control(&worker,DBT_SINK_START,NULL,0));
    ctx.builder_mux=(dbt_aac_mux_t){0};unsigned calls_before=ctx.calls;
    for(unsigned i=0;i<count && !worker.pending_pcm;++i)TRY(feed(c,&ctx,NULL,&worker,i,&now,true));
    TRY(worker.pending_pcm && ctx.calls==calls_before);
    TRY(control(&worker,DBT_SINK_SUSPEND,NULL,0) && !worker.pending_pcm && !worker.media_active);
    TRY(control(&worker,DBT_SINK_START,NULL,0));
    for(unsigned i=1;i<count && !worker.pending_pcm;++i)TRY(feed(c,&ctx,NULL,&worker,i,&now,true));
    TRY(worker.pending_pcm && ctx.calls==calls_before);
    TRY(control(&worker,DBT_SINK_SUSPEND,NULL,0) && control(&worker,DBT_SINK_RECONFIGURE,configuration,(uint16_t)(4u+cie_size(c))) && control(&worker,DBT_SINK_START,NULL,0));
    ctx.builder_mux=(dbt_aac_mux_t){0};
    for(unsigned i=0;i<count && !worker.pending_pcm;++i)TRY(feed(c,&ctx,NULL,&worker,i,&now,true));
    TRY(worker.pending_pcm && control(&worker,DBT_SINK_DROP,NULL,0));
    TRY(!ctx.live && ctx.allocs==ctx.frees && ctx.calls==calls_before && !ctx.error);
finish:
    if(worker_initialized && worker.binding)control(&worker,DBT_SINK_DROP,NULL,0);
    dbt_codec_close(&direct);
    printf("{\"probe\":\"deferred_codec_case\",\"codec\":\"%s\",\"index\":%u,\"mode\":%u,\"passed\":%s,\"line\":%d,\"consumer_line\":%d,\"sample_frames\":%u,\"busy_attempts\":%u,\"tracked_peak_bytes\":%u,\"live_bytes\":%u}\n",
        c.v?c.v->name:c.opus?"Opus":"LHDC 192k/S24/1000",c.index,c.mode,line?"false":"true",line,ctx.error,
        (unsigned)ctx.compared_frames,ctx.busy_calls,(unsigned)ctx.peak,(unsigned)ctx.live);
    free(ctx.reference);free(ctx.timestamps);return line;
}
int main(void){
    unsigned cases=0;
    for(unsigned i=0;i<sizeof(probe_codec_vectors)/sizeof(probe_codec_vectors[0]);++i){
        const probe_codec_vector_t *v=&probe_codec_vectors[i];if(!v->name)continue;
        case_t c={v,NULL,0,i};if(run_case(c))return 1;++cases;
        if(v->ops==&dbt_sbc_codec || v->ops==&dbt_aac_codec || v->ops==&dbt_lhdcv5_codec){c.mode=1;if(run_case(c))return 1;++cases;}
        if(v->ops==&dbt_sbc_codec || v->ops==&dbt_aac_codec || v->ops==&dbt_ldac_codec || v->ops==&dbt_aptx_codec){c.mode=2;if(run_case(c))return 1;++cases;}
    }
    for(unsigned i=0;i<sizeof(opus_vectors)/sizeof(opus_vectors[0]);++i)for(unsigned mode=0;mode<2;++mode){
        case_t c={NULL,&opus_vectors[i],mode,i};if(run_case(c))return 1;++cases;
    }
    case_t maximum={0};if(run_case(maximum))return 1;++cases;
    printf("{\"probe\":\"deferred_codec_summary\",\"passed\":true,\"cases\":%u,\"maximum_rate_tested\":true,\"rf_test\":false}\n",cases);
    return 0;
}
