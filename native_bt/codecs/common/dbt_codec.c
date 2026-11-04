#include "dbt_codec.h"

typedef struct {uint32_t bytes,pad;} allocation_t;
typedef struct {
    uint32_t timestamp,ssrc;
    uint16_t sequence,size;
    uint8_t payload_type;
    bool marker;
    uint8_t bytes[];
} media_t;
static void zero(void *p,unsigned n){uint8_t *v=p;for(unsigned i=0;i<n;++i)v[i]=0;}
static void copy(void *to,const void *from,unsigned n){uint8_t *d=to;const uint8_t *s=from;for(unsigned i=0;i<n;++i)d[i]=s[i];}

dbt_result_t dbt_codec_init(dbt_codec_decoder_t *d,dbt_codec_hooks_t hooks){
    if(!d || !hooks.allocate || !hooks.release || !hooks.pcm)return DBT_INVALID;
    zero(d,sizeof(*d));d->hooks=hooks;return DBT_OK;
}
uint8_t *dbt_codec_allocate(dbt_codec_decoder_t *d,uint32_t bytes){
    if(!d || !bytes || bytes>UINT32_MAX-sizeof(allocation_t) ||
       d->allocated_bytes>UINT32_MAX-bytes-sizeof(allocation_t))return 0;
    uint32_t total=bytes+sizeof(allocation_t);uint8_t *raw=d->hooks.allocate(d->hooks.context,total);
    if(!raw)return 0;
    if((uintptr_t)raw&7u){d->hooks.release(d->hooks.context,raw);return 0;}
    allocation_t *a=(allocation_t *)raw;a->bytes=total;a->pad=0;
    d->allocated_bytes+=total;return raw+sizeof(*a);
}
void dbt_codec_release(dbt_codec_decoder_t *d,void *bytes){
    if(!d || !bytes)return;
    allocation_t *a=(allocation_t *)((uint8_t *)bytes-sizeof(allocation_t));
    d->allocated_bytes-=a->bytes;d->hooks.release(d->hooks.context,(uint8_t *)a);
}
void dbt_codec_close(dbt_codec_decoder_t *d){
    if(!d)return;
    if(d->config.ops)d->config.ops->close(d);
    d->state=0;zero(&d->config,sizeof(d->config));
}
dbt_result_t dbt_codec_open(dbt_codec_decoder_t *d,const dbt_codec_config_t *config){
    if(!d || !d->hooks.allocate || !d->hooks.release || !d->hooks.pcm || !config || !config->ops ||
       !config->ops->open || !config->ops->close || !config->ops->reset || !config->ops->media || !config->rate || !config->samples ||
       config->channels<1 || config->channels>2 || config->sample_bytes<2 || config->sample_bytes>4 ||
       config->cie_size>DBT_CODEC_CIE_BYTES)return DBT_INVALID;
    dbt_codec_config_t next;copy(&next,config,sizeof(next));
    dbt_codec_close(d);copy(&d->config,&next,sizeof(next));d->frames=0;
    dbt_result_t result=next.ops->open(d);
    if(result!=DBT_OK)dbt_codec_close(d);
    return result;
}
static bool enabled(const dbt_codec_worker_t *w){uint32_t start=atomic_load_explicit(&w->started_epoch,memory_order_acquire);return start && start==atomic_load_explicit(&w->gate_epoch,memory_order_acquire);}
static void wake_worker(dbt_codec_worker_t *w){if(w->hooks.wake_worker)w->hooks.wake_worker(w->hooks.decoder.context);}
static void wake_owner(dbt_codec_worker_t *w){if(w->hooks.wake_owner)w->hooks.wake_owner(w->hooks.decoder.context);}
dbt_result_t dbt_codec_worker_init(dbt_codec_worker_t *w,dbt_codec_worker_hooks_t hooks,uint16_t payload,uint8_t depth){
    uint32_t stride=(sizeof(media_t)+payload+3u)&~3u;
    if(!w || !hooks.codecs || !hooks.codec_count || hooks.codec_count>8 || payload<5 || payload>4096 ||
       !depth || depth>16 || (depth&(depth-1u)) || stride*depth>UINT16_MAX)return DBT_INVALID;
    for(unsigned i=0;i<hooks.codec_count;++i){const dbt_codec_ops_t *o=hooks.codecs[i];if(!o || !o->configure || !o->open || !o->close || !o->reset || !o->media)return DBT_INVALID;}
    zero(w,sizeof(*w));w->hooks=hooks;w->maximum_payload=payload;w->stride=(uint16_t)stride;w->depth=depth;
    atomic_init(&w->job_written,0);atomic_init(&w->job_read,0);atomic_init(&w->receipt_written,0);atomic_init(&w->receipt_read,0);
    atomic_init(&w->gate_epoch,0);atomic_init(&w->started_epoch,0);atomic_init(&w->media_written,0);atomic_init(&w->media_read,0);
    atomic_init(&w->packets,0);atomic_init(&w->media_drops,0);atomic_init(&w->resets,0);atomic_init(&w->faults,0);atomic_init(&w->resident_bytes,0);
    return dbt_codec_init(&w->decoder,hooks.decoder);
}
dbt_result_t dbt_codec_worker_submit(void *context,const dbt_sink_job_t *request){
    dbt_codec_worker_t *w=context;
    if(!w || !request || !request->binding || !request->token || request->operation<DBT_SINK_CONFIGURE || request->operation>DBT_SINK_DROP)return DBT_INVALID;
    dbt_codec_config_t config;zero(&config,sizeof(config));
    if(request->operation==DBT_SINK_CONFIGURE || request->operation==DBT_SINK_RECONFIGURE){
        dbt_avdtp_caps_t caps;dbt_avdtp_cap_error_t error;
        if(dbt_avdtp_caps_validate(request->configuration,request->size,DBT_AVDTP_CONFIGURATION,&caps,&error)!=DBT_OK)return DBT_UNSUPPORTED;
        for(unsigned i=0;i<w->hooks.codec_count;++i){
            if(w->hooks.codecs[i]->configure(caps.codec.data,caps.codec.size,&config)==DBT_OK){config.ops=w->hooks.codecs[i];break;}
        }
        if(!config.ops)return DBT_UNSUPPORTED;
    }
    uint32_t written=atomic_load_explicit(&w->job_written,memory_order_relaxed);
    if(written-atomic_load_explicit(&w->job_read,memory_order_acquire)>=DBT_CODEC_WORKER_JOBS)return DBT_BUSY;
    uint32_t epoch=atomic_fetch_add_explicit(&w->gate_epoch,1,memory_order_acq_rel)+1u;
    if(!epoch)epoch=atomic_fetch_add_explicit(&w->gate_epoch,1,memory_order_acq_rel)+1u;
    dbt_codec_job_t *job=&w->jobs[written%DBT_CODEC_WORKER_JOBS];
    job->binding=request->binding;job->token=request->token;job->epoch=epoch;job->operation=request->operation;copy(&job->config,&config,sizeof(config));
    atomic_store_explicit(&w->job_written,written+1u,memory_order_release);wake_worker(w);return DBT_OK;
}
dbt_result_t dbt_codec_worker_media(void *context,const dbt_avdtp_media_t *packet){
    dbt_codec_worker_t *w=context;
    if(!w || !packet || !packet->payload || !packet->size || packet->size>w->maximum_payload)return DBT_INVALID;
    if(!enabled(w))return DBT_BUSY;
    uint32_t written=atomic_load_explicit(&w->media_written,memory_order_relaxed);
    if(written-atomic_load_explicit(&w->media_read,memory_order_acquire)>=w->depth){atomic_fetch_add(&w->media_drops,1);return DBT_NO_SPACE;}
    media_t *slot=(media_t *)(w->packet_storage+(written%w->depth)*w->stride);
    slot->timestamp=packet->timestamp;slot->ssrc=packet->ssrc;slot->sequence=packet->sequence;slot->size=packet->size;
    slot->payload_type=packet->payload_type;slot->marker=packet->marker;copy(slot->bytes,packet->payload,packet->size);
    atomic_store_explicit(&w->media_written,written+1u,memory_order_release);wake_worker(w);return DBT_OK;
}
dbt_result_t dbt_codec_worker_receipt(dbt_codec_worker_t *w,dbt_codec_receipt_t *receipt){
    if(!w || !receipt)return DBT_INVALID;
    uint32_t read=atomic_load_explicit(&w->receipt_read,memory_order_relaxed);
    if(read==atomic_load_explicit(&w->receipt_written,memory_order_acquire))return DBT_EMPTY;
    *receipt=w->receipts[read%DBT_CODEC_WORKER_JOBS];atomic_store_explicit(&w->receipt_read,read+1u,memory_order_release);wake_worker(w);return DBT_OK;
}
static void clear(dbt_codec_worker_t *w){
    atomic_store_explicit(&w->started_epoch,0,memory_order_release);
    if(w->packet_storage)dbt_codec_release(&w->decoder,w->packet_storage);
    w->packet_storage=0;dbt_codec_close(&w->decoder);w->configured=w->streaming=w->sequence_valid=false;
    atomic_store(&w->resident_bytes,w->decoder.allocated_bytes);
}
static dbt_result_t construct(dbt_codec_worker_t *w,const dbt_codec_config_t *config){
    dbt_result_t result=dbt_codec_open(&w->decoder,config);if(result!=DBT_OK)return result;
    w->packet_storage=dbt_codec_allocate(&w->decoder,w->stride*w->depth);
    if(!w->packet_storage){dbt_codec_close(&w->decoder);return DBT_NO_SPACE;}
    w->configured=true;w->sequence_valid=false;atomic_store(&w->media_written,0);atomic_store(&w->media_read,0);return DBT_OK;
}
static void fault(dbt_codec_worker_t *w){
    atomic_store_explicit(&w->started_epoch,0,memory_order_release);
    w->streaming=w->sequence_valid=false;
    if(w->hooks.stream)(void)w->hooks.stream(w->hooks.decoder.context,false,&w->decoder.config);
    atomic_fetch_add(&w->faults,1);
}
static dbt_sink_work_result_t control(dbt_codec_worker_t *w,const dbt_codec_job_t *job){
    if(w->binding && w->binding!=job->binding)return DBT_SINK_WORK_FATAL;
    if(job->operation==DBT_SINK_CLOSE || job->operation==DBT_SINK_ABORT || job->operation==DBT_SINK_DROP){
        dbt_result_t stopped=w->hooks.stream?w->hooks.stream(w->hooks.decoder.context,false,&w->decoder.config):DBT_OK;
        clear(w);w->binding=0;return stopped==DBT_OK?DBT_SINK_WORK_READY:DBT_SINK_WORK_FATAL;
    }
    if(job->operation==DBT_SINK_CONFIGURE || job->operation==DBT_SINK_RECONFIGURE){
        dbt_codec_config_t old;copy(&old,&w->decoder.config,sizeof(old));
        bool restore=w->configured && job->operation==DBT_SINK_RECONFIGURE;
        clear(w);w->binding=job->binding;dbt_result_t result=construct(w,&job->config);
        if(result!=DBT_OK){
            if(!restore){w->binding=0;return DBT_SINK_WORK_REJECTED;}
            if(construct(w,&old)!=DBT_OK)return DBT_SINK_WORK_FATAL;
        }
        return result==DBT_OK?DBT_SINK_WORK_READY:DBT_SINK_WORK_REJECTED;
    }
    if(!w->configured)return DBT_SINK_WORK_REJECTED;
    if(job->operation==DBT_SINK_OPEN)return DBT_SINK_WORK_READY;
    if(job->operation==DBT_SINK_START || job->operation==DBT_SINK_SUSPEND){
        bool start=job->operation==DBT_SINK_START;
        dbt_result_t result=w->hooks.stream?w->hooks.stream(w->hooks.decoder.context,start,&w->decoder.config):DBT_OK;
        if(result!=DBT_OK){if(w->streaming)atomic_store(&w->started_epoch,job->epoch);return DBT_SINK_WORK_REJECTED;}
        w->sequence_valid=false;atomic_store(&w->media_read,atomic_load(&w->media_written));
        if(w->decoder.config.ops->reset(&w->decoder)!=DBT_OK){fault(w);return DBT_SINK_WORK_FATAL;}
        w->streaming=start;if(start)atomic_store_explicit(&w->started_epoch,job->epoch,memory_order_release);
        return DBT_SINK_WORK_READY;
    }
    return DBT_SINK_WORK_REJECTED;
}
uint16_t dbt_codec_worker_step(dbt_codec_worker_t *w,uint32_t now,uint16_t budget){
    if(!w)return 0;
    for(unsigned i=0;i<DBT_CODEC_WORKER_JOBS;++i){
        uint32_t read=atomic_load_explicit(&w->job_read,memory_order_relaxed);if(read==atomic_load_explicit(&w->job_written,memory_order_acquire))break;
        uint32_t written=atomic_load_explicit(&w->receipt_written,memory_order_relaxed);
        if(written-atomic_load_explicit(&w->receipt_read,memory_order_acquire)>=DBT_CODEC_WORKER_JOBS)break;
        const dbt_codec_job_t *job=&w->jobs[read%DBT_CODEC_WORKER_JOBS];dbt_sink_work_result_t result=control(w,job);
        w->receipts[written%DBT_CODEC_WORKER_JOBS]=(dbt_codec_receipt_t){job->binding,job->token,result};
        atomic_store(&w->resident_bytes,w->decoder.allocated_bytes);atomic_store_explicit(&w->job_read,read+1u,memory_order_release);
        atomic_store_explicit(&w->receipt_written,written+1u,memory_order_release);wake_owner(w);
    }
    uint16_t count=0;
    while(count<budget && enabled(w)){
        uint32_t read=atomic_load_explicit(&w->media_read,memory_order_relaxed);if(read==atomic_load_explicit(&w->media_written,memory_order_acquire))break;
        media_t *slot=(media_t *)(w->packet_storage+(read%w->depth)*w->stride);
        if(w->sequence_valid && (slot->sequence!=(uint16_t)(w->last_sequence+1u) || slot->ssrc!=w->last_ssrc)){
            atomic_fetch_add(&w->resets,1);
            if(w->decoder.config.ops->reset(&w->decoder)!=DBT_OK){
                atomic_fetch_add(&w->media_drops,1);atomic_store_explicit(&w->media_read,read+1u,memory_order_release);fault(w);++count;break;
            }
        }
        w->sequence_valid=true;w->last_sequence=slot->sequence;w->last_ssrc=slot->ssrc;
        dbt_avdtp_media_t packet={slot->bytes,slot->timestamp,slot->ssrc,slot->size,slot->sequence,slot->payload_type,slot->marker};
        if(w->decoder.config.ops->media(&w->decoder,&packet,now)!=DBT_OK){
            atomic_fetch_add(&w->media_drops,1);
            if(w->decoder.config.ops->reset(&w->decoder)!=DBT_OK)fault(w);
        }
        atomic_fetch_add(&w->packets,1);atomic_store_explicit(&w->media_read,read+1u,memory_order_release);++count;
    }
    if(w->configured && w->decoder.config.ops->expire)w->decoder.config.ops->expire(&w->decoder,now);
    atomic_store(&w->resident_bytes,w->decoder.allocated_bytes);return count;
}
