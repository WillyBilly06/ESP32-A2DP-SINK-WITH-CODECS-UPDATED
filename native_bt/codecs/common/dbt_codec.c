#include "dbt_codec.h"
#include <stddef.h>
#include <string.h>
#ifndef DBT_CODEC_DURATION_TEST_STEP
#define DBT_CODEC_DURATION_TEST_STEP(worker) ((void)(worker))
#endif

typedef struct {uint32_t bytes,pad;} allocation_t;
typedef struct {
    uint32_t timestamp,ssrc;
    uint16_t sequence,size;
    uint8_t payload_type;
    bool marker;
    uint8_t prefix[2];
    uint8_t bytes[];
} media_t;
_Static_assert(offsetof(media_t,bytes)%8u==0,"Encoded media payload must begin on an eight-byte boundary");
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
static void pending_clear(dbt_codec_worker_t *w){
    w->pending_pcm=0;w->pending_frames=w->pending_offset=0;w->output_blocked=false;
}
static void trace(dbt_codec_worker_t *w,dbt_codec_trace_kind_t kind,const media_t *slot,
                  uint16_t expected,uint32_t previous_ssrc,dbt_result_t result){
    if(!w->hooks.trace)return;
    dbt_codec_trace_t event={slot->timestamp,slot->ssrc,previous_ssrc,slot->sequence,
        expected,slot->size,(uint16_t)(atomic_load(&w->media_written)-atomic_load(&w->media_read)),kind,result,
        slot->marker,{slot->prefix[0],slot->prefix[1]},slot->payload_type};
    w->hooks.trace(w->hooks.decoder.context,&event);
}
static bool reset(dbt_codec_worker_t *w,const media_t *slot){
    trace(w,DBT_CODEC_RESET_BEGIN,slot,slot->sequence,slot->ssrc,DBT_OK);
    dbt_result_t result=w->decoder.config.ops->reset(&w->decoder);
    trace(w,DBT_CODEC_RESET_END,slot,slot->sequence,slot->ssrc,result);
    if(result==DBT_OK)return true;
    atomic_fetch_add(&w->reset_failures,1);
    trace(w,DBT_CODEC_RESET_FAILED,slot,(uint16_t)(w->last_sequence+1u),w->last_ssrc,result);return false;
}
static dbt_result_t decode(dbt_codec_worker_t *w,const media_t *slot,uint32_t now,bool resume){
    trace(w,DBT_CODEC_DECODE_BEGIN,slot,slot->sequence,slot->ssrc,DBT_OK);
    dbt_avdtp_media_t packet={slot->bytes,slot->timestamp,slot->ssrc,slot->size,slot->sequence,slot->payload_type,slot->marker};
    dbt_result_t result=resume?w->decoder.config.ops->resume(&w->decoder,now):w->decoder.config.ops->media(&w->decoder,&packet,now);
    trace(w,DBT_CODEC_DECODE_END,slot,slot->sequence,slot->ssrc,result);return result;
}
static void worker_sequence(dbt_codec_worker_t *w,const media_t *slot){
    if(slot->sequence!=(uint16_t)(w->last_sequence+1u))
        trace(w,DBT_CODEC_WORKER_SEQUENCE,slot,(uint16_t)(w->last_sequence+1u),w->last_ssrc,DBT_OK);
    if(slot->ssrc!=w->last_ssrc)
        trace(w,DBT_CODEC_WORKER_SSRC,slot,(uint16_t)(w->last_sequence+1u),w->last_ssrc,DBT_OK);
}
static uint8_t *deferred_allocate(void *context,uint32_t bytes){
    dbt_codec_worker_t *w=context;return w->hooks.decoder.allocate(w->hooks.decoder.context,bytes);
}
static void deferred_release(void *context,uint8_t *bytes){
    dbt_codec_worker_t *w=context;w->hooks.decoder.release(w->hooks.decoder.context,bytes);
}
static dbt_result_t deferred_pcm(void *context,const void *samples,uint16_t frames,
                                const dbt_codec_config_t *config,uint32_t timestamp){
    dbt_codec_worker_t *w=context;
    if(!samples || !frames || !config || !w->configured || !w->streaming || w->pending_pcm ||
       config->rate!=w->decoder.config.rate || config->channels!=w->decoder.config.channels ||
       config->sample_bytes!=w->decoder.config.sample_bytes)return DBT_INVALID;
    w->pending_pcm=samples;w->pending_frames=frames;w->pending_offset=0;
    w->pending_timestamp=timestamp;w->output_blocked=false;return DBT_PENDING;
}
dbt_result_t dbt_codec_worker_init(dbt_codec_worker_t *w,dbt_codec_worker_hooks_t hooks,uint16_t payload,uint8_t depth){
    uint32_t stride=(sizeof(media_t)+payload+7u)&~7u;
    if(!w || !hooks.codecs || !hooks.codec_count || hooks.codec_count>8 || payload<5 || payload>4096 ||
       !depth || depth>16 || (depth&(depth-1u)) || stride*depth>UINT16_MAX)return DBT_INVALID;
    for(unsigned i=0;i<hooks.codec_count;++i){const dbt_codec_ops_t *o=hooks.codecs[i];if(!o || !o->configure || !o->open || !o->close || !o->reset || !o->media || (hooks.deferred_pcm && !o->resume))return DBT_INVALID;}
    if(hooks.deferred_pcm && (!hooks.decoder.allocate || !hooks.decoder.release || !hooks.decoder.pcm))return DBT_INVALID;
    if((hooks.async_stream.begin || hooks.async_stream.poll || hooks.async_stream.cancel) &&
       (!hooks.async_stream.begin || !hooks.async_stream.poll || !hooks.async_stream.cancel || hooks.stream))return DBT_INVALID;
    if((hooks.async_pcm.begin || hooks.async_pcm.poll || hooks.async_pcm.cancel) &&
       (!hooks.async_pcm.begin || !hooks.async_pcm.poll || !hooks.async_pcm.cancel || !hooks.deferred_pcm || !hooks.async_stream.begin))return DBT_INVALID;
    zero(w,sizeof(*w));w->hooks=hooks;w->maximum_payload=payload;w->stride=(uint16_t)stride;
    w->depth=w->depth_limit=depth;w->low_water=(uint8_t)((depth+1u)/2u);
    atomic_init(&w->queue_layout,0);
    atomic_init(&w->media_page_written,0);atomic_init(&w->media_page_read,0);
    atomic_init(&w->duration_input_frames,0);atomic_init(&w->duration_pcm_base,0);
    atomic_init(&w->duration_reject_base,0);atomic_init(&w->duration_epoch,0);
    atomic_init(&w->job_written,0);atomic_init(&w->job_read,0);atomic_init(&w->receipt_written,0);atomic_init(&w->receipt_read,0);
    atomic_init(&w->gate_epoch,0);atomic_init(&w->started_epoch,0);atomic_init(&w->media_written,0);atomic_init(&w->media_read,0);
    atomic_init(&w->packets,0);atomic_init(&w->media_drops,0);atomic_init(&w->resets,0);atomic_init(&w->faults,0);atomic_init(&w->resident_bytes,0);
    atomic_init(&w->pcm_backpressure,0);atomic_init(&w->pcm_timeouts,0);atomic_init(&w->pcm_delivered_frames,0);
    atomic_init(&w->queue_full_drops,0);atomic_init(&w->decode_rejects,0);atomic_init(&w->reset_failures,0);atomic_init(&w->pcm_failures,0);
    atomic_init(&w->input_sequence_breaks,0);atomic_init(&w->input_ssrc_changes,0);atomic_init(&w->queue_high_water,0);
    atomic_init(&w->input_packets,0);atomic_init(&w->input_payload_bytes,0);atomic_init(&w->input_first_timestamp,0);
    atomic_init(&w->input_last_timestamp,0);atomic_init(&w->input_stream_epoch,0);
    atomic_init(&w->input_clock_revision,0);atomic_init(&w->input_clock_step,0);
    atomic_init(&w->input_clock_sequence,0);atomic_init(&w->input_clock_timestamp,0);atomic_init(&w->input_clock_ssrc,0);
    atomic_init(&w->input_max_payload,0);atomic_init(&w->input_oversize_rejects,0);atomic_init(&w->input_last_oversize,0);
    dbt_codec_hooks_t decoder=hooks.decoder;
    if(hooks.deferred_pcm)decoder=(dbt_codec_hooks_t){w,deferred_allocate,deferred_release,deferred_pcm};
    return dbt_codec_init(&w->decoder,decoder);
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
static void input_clock(dbt_codec_worker_t *w,const media_t *packet,uint32_t epoch){
    bool reset=w->input_epoch!=epoch,wire=packet->payload_type!=DBT_AVDTP_RAW_PAYLOAD_TYPE;
    bool adjacent=!reset && wire && w->input_clock_prev_wire && packet->ssrc==w->input_ssrc &&
        packet->sequence==(uint16_t)(w->input_sequence+1u);
    uint32_t step=adjacent?packet->timestamp-atomic_load_explicit(&w->input_last_timestamp,memory_order_relaxed):0;
    uint32_t tag=atomic_load_explicit(&w->input_clock_sequence,memory_order_relaxed);
    bool update=adjacent && step<UINT32_C(0x80000000) &&
        (!(tag&UINT32_C(0x10000)) || step>atomic_load_explicit(&w->input_clock_step,memory_order_relaxed));
    if(reset || update){
        uint32_t revision=atomic_load_explicit(&w->input_clock_revision,memory_order_relaxed);
        atomic_store_explicit(&w->input_clock_revision,revision+1u,memory_order_seq_cst);
        atomic_thread_fence(memory_order_seq_cst);
        if(reset){
            atomic_store_explicit(&w->input_clock_step,0,memory_order_relaxed);
            atomic_store_explicit(&w->input_clock_sequence,0,memory_order_relaxed);
            atomic_store_explicit(&w->input_clock_timestamp,0,memory_order_relaxed);
            atomic_store_explicit(&w->input_clock_ssrc,0,memory_order_relaxed);
            atomic_store_explicit(&w->input_stream_epoch,epoch,memory_order_relaxed);
        }
        if(update){
            atomic_store_explicit(&w->input_clock_step,step,memory_order_relaxed);
            atomic_store_explicit(&w->input_clock_sequence,UINT32_C(0x10000)|packet->sequence,memory_order_relaxed);
            atomic_store_explicit(&w->input_clock_timestamp,packet->timestamp,memory_order_relaxed);
            atomic_store_explicit(&w->input_clock_ssrc,packet->ssrc,memory_order_relaxed);
        }
        uint32_t next=revision+2u;
        atomic_store_explicit(&w->input_clock_revision,next?next:2u,memory_order_release);
        if(update)trace(w,DBT_CODEC_INPUT_CLOCK_MAX,packet,packet->sequence,packet->ssrc,DBT_OK);
    }
    w->input_clock_prev_wire=wire;
}
void dbt_codec_worker_clock_snapshot(const dbt_codec_worker_t *w,dbt_codec_input_clock_t *out){
    if(!out)return;
    *out=(dbt_codec_input_clock_t){0};if(!w)return;
    uint32_t before=atomic_load_explicit(&w->input_clock_revision,memory_order_acquire);
    out->epoch=atomic_load_explicit(&w->input_stream_epoch,memory_order_relaxed);
    out->step_ticks=atomic_load_explicit(&w->input_clock_step,memory_order_relaxed);
    uint32_t tag=atomic_load_explicit(&w->input_clock_sequence,memory_order_relaxed);
    out->sequence=(uint16_t)tag;out->observed=(tag&UINT32_C(0x10000))!=0;
    out->timestamp=atomic_load_explicit(&w->input_clock_timestamp,memory_order_relaxed);
    out->ssrc=atomic_load_explicit(&w->input_clock_ssrc,memory_order_relaxed);
    atomic_thread_fence(memory_order_seq_cst);out->revision=before;
    out->coherent=before && !(before&1u) && before==atomic_load_explicit(&w->input_clock_revision,memory_order_acquire);
}
dbt_result_t dbt_codec_worker_set_media_frames(dbt_codec_worker_t *w,dbt_codec_media_frames_t frames){
    if(!w || !frames || !w->hooks.codecs)return DBT_INVALID;
    if(w->media_frames || atomic_load(&w->job_written) || atomic_load(&w->job_read) ||
       atomic_load(&w->gate_epoch) || atomic_load(&w->started_epoch))return DBT_BUSY;
    w->media_frames=frames;return DBT_OK;
}
bool dbt_codec_worker_buffered_frames(const dbt_codec_worker_t *w,uint32_t *frames){
    if(!w || !frames)return false;
    *frames=0;
    uint32_t epoch=atomic_load_explicit(&w->started_epoch,memory_order_acquire);
    if(!epoch || epoch!=atomic_load_explicit(&w->gate_epoch,memory_order_acquire) ||
       epoch!=atomic_load_explicit(&w->duration_epoch,memory_order_acquire))return false;
    uint32_t base=atomic_load_explicit(&w->duration_pcm_base,memory_order_relaxed);
    uint32_t written=atomic_load_explicit(&w->duration_input_frames,memory_order_acquire);
    uint32_t delivered=atomic_load_explicit(&w->pcm_delivered_frames,memory_order_acquire)-base;
    uint32_t buffered=written-delivered;
    DBT_CODEC_DURATION_TEST_STEP(w);
    if(buffered>=0x80000000u || written!=atomic_load_explicit(&w->duration_input_frames,memory_order_acquire) ||
       atomic_load(&w->decode_rejects)!=atomic_load(&w->duration_reject_base) ||
       epoch!=atomic_load_explicit(&w->duration_epoch,memory_order_acquire) ||
       epoch!=atomic_load_explicit(&w->started_epoch,memory_order_acquire) ||
       epoch!=atomic_load_explicit(&w->gate_epoch,memory_order_acquire))return false;
    *frames=buffered;return true;
}
static uint16_t record_stride(uint16_t size){return (uint16_t)((sizeof(media_t)+size+7u)&~7u);}
static void release_page(dbt_codec_worker_t *w){
    w->media_read_offset=0;
    atomic_store_explicit(&w->media_page_read,atomic_load_explicit(&w->media_page_read,memory_order_relaxed)+1u,memory_order_release);
    wake_owner(w);
}
/* Consumer only, after acquiring a nonempty logical queue or while holding its
 * current decoder borrow. One record can cross at most one sealed page tail. */
static media_t *read_slot(dbt_codec_worker_t *w,uint32_t read){
    if(!w->packed_media)return (media_t *)(w->packet_storage+(read&(w->depth-1u))*w->stride);
    uint32_t page=atomic_load_explicit(&w->media_page_read,memory_order_relaxed);
    media_t *slot=(media_t *)(w->packet_storage+(page&(w->depth-1u))*w->stride+w->media_read_offset);
    if((uint32_t)w->stride-w->media_read_offset<sizeof(media_t) || slot->size==UINT16_MAX){
        release_page(w);++page;
        slot=(media_t *)(w->packet_storage+(page&(w->depth-1u))*w->stride);
    }
    return slot;
}
/* Producer only. Do not seal the previous page unless the next physical slab
 * is available. Metadata/payload publish together through media_written. */
static media_t *write_slot(dbt_codec_worker_t *w,uint32_t written,uint16_t size){
    if(!w->packed_media){
        if(written-atomic_load_explicit(&w->media_read,memory_order_acquire)>=w->depth)return NULL;
        return (media_t *)(w->packet_storage+(written&(w->depth-1u))*w->stride);
    }
    uint32_t page=atomic_load_explicit(&w->media_page_written,memory_order_relaxed);
    uint16_t offset=w->media_write_offset,bytes=record_stride(size);
    if(!offset || bytes>w->stride-offset){
        uint32_t read=atomic_load_explicit(&w->media_page_read,memory_order_acquire);
        if(page-read>=w->depth)return NULL;
        if(offset && (uint32_t)w->stride-offset>=sizeof(media_t)){
            media_t *tail=(media_t *)(w->packet_storage+((page-1u)&(w->depth-1u))*w->stride+offset);
            tail->size=UINT16_MAX;
        }
        ++page;offset=0;
        atomic_store_explicit(&w->media_page_written,page,memory_order_relaxed);
    }
    media_t *slot=(media_t *)(w->packet_storage+((page-1u)&(w->depth-1u))*w->stride+offset);
    w->media_write_offset=(uint16_t)(offset+bytes);return slot;
}
uint16_t dbt_codec_worker_free_slots(const dbt_codec_worker_t *w){
    if(!w)return 0;
    uint32_t layout=atomic_load_explicit(&w->queue_layout,memory_order_acquire);
    uint32_t depth=layout&DBT_CODEC_QUEUE_DEPTH_MASK;
    if(!depth || depth>16u)return 0;
    uint32_t read=atomic_load_explicit((layout&DBT_CODEC_QUEUE_PACKED)?&w->media_page_read:&w->media_read,memory_order_acquire);
    uint32_t written=atomic_load_explicit((layout&DBT_CODEC_QUEUE_PACKED)?&w->media_page_written:&w->media_written,memory_order_acquire);
    uint32_t used=written-read;
    return used<depth?(uint16_t)(depth-used):0;
}
dbt_result_t dbt_codec_worker_media(void *context,const dbt_avdtp_media_t *packet){
    dbt_codec_worker_t *w=context;
    if(!w || !packet || !packet->payload || !packet->size)return DBT_INVALID;
    if(!enabled(w))return DBT_BUSY;
    if(packet->size>atomic_load_explicit(&w->input_max_payload,memory_order_relaxed))
        atomic_store_explicit(&w->input_max_payload,packet->size,memory_order_relaxed);
    if(packet->size>w->maximum_payload){
        atomic_fetch_add_explicit(&w->input_oversize_rejects,1,memory_order_relaxed);
        atomic_store_explicit(&w->input_last_oversize,packet->size,memory_order_relaxed);
        return DBT_INVALID;
    }
    uint32_t epoch=atomic_load_explicit(&w->started_epoch,memory_order_acquire);
    if(w->input_epoch!=epoch){
        atomic_store(&w->input_packets,0);atomic_store(&w->input_payload_bytes,0);
        atomic_store(&w->input_first_timestamp,packet->timestamp);
    }
    atomic_fetch_add(&w->input_packets,1);atomic_fetch_add(&w->input_payload_bytes,packet->size);
    media_t input={packet->timestamp,packet->ssrc,packet->sequence,packet->size,packet->payload_type,packet->marker,
        {packet->payload[0],packet->size>1u?packet->payload[1]:0}};
    input_clock(w,&input,epoch);
    atomic_store(&w->input_last_timestamp,packet->timestamp);
    if(w->input_epoch==epoch && packet->payload_type!=DBT_AVDTP_RAW_PAYLOAD_TYPE){
        if(packet->sequence!=(uint16_t)(w->input_sequence+1u)){
            atomic_fetch_add(&w->input_sequence_breaks,1);
            trace(w,DBT_CODEC_INPUT_SEQUENCE,&input,(uint16_t)(w->input_sequence+1u),w->input_ssrc,DBT_OK);
        }
        if(packet->ssrc!=w->input_ssrc){
            atomic_fetch_add(&w->input_ssrc_changes,1);
            trace(w,DBT_CODEC_INPUT_SSRC,&input,(uint16_t)(w->input_sequence+1u),w->input_ssrc,DBT_OK);
        }
    }
    w->input_epoch=epoch;w->input_sequence=packet->sequence;w->input_ssrc=packet->ssrc;
    uint32_t written=atomic_load_explicit(&w->media_written,memory_order_relaxed);
    uint32_t queued=written-atomic_load_explicit(&w->media_read,memory_order_acquire);
    media_t *slot=write_slot(w,written,packet->size);
    if(!slot){
        atomic_fetch_add(&w->queue_full_drops,1);atomic_fetch_add(&w->media_drops,1);
        trace(w,DBT_CODEC_QUEUE_FULL,&input,packet->sequence,packet->ssrc,DBT_NO_SPACE);return DBT_NO_SPACE;
    }
    if(queued+1u>atomic_load(&w->queue_high_water))atomic_store(&w->queue_high_water,queued+1u);
    slot->timestamp=packet->timestamp;slot->ssrc=packet->ssrc;slot->sequence=packet->sequence;slot->size=packet->size;
    slot->payload_type=packet->payload_type;slot->marker=packet->marker;
    slot->prefix[0]=input.prefix[0];slot->prefix[1]=input.prefix[1];memcpy(slot->bytes,packet->payload,packet->size);
    if(atomic_load_explicit(&w->duration_epoch,memory_order_acquire)==epoch){
        uint32_t frames=w->media_frames(&w->decoder.config,packet);
        if(!frames || frames>=0x80000000u)atomic_store_explicit(&w->duration_epoch,0,memory_order_release);
        else atomic_store_explicit(&w->duration_input_frames,atomic_load(&w->duration_input_frames)+frames,memory_order_release);
    }
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
    pending_clear(w);w->media_active=false;
    if(w->packet_storage)dbt_codec_release(&w->decoder,w->packet_storage);
    w->packet_storage=0;dbt_codec_close(&w->decoder);w->configured=w->streaming=w->sequence_valid=false;
    atomic_store_explicit(&w->queue_layout,0,memory_order_release);
    atomic_store(&w->resident_bytes,w->decoder.allocated_bytes);
}
static dbt_result_t construct(dbt_codec_worker_t *w,const dbt_codec_config_t *config){
    dbt_codec_queue_t queue={w->depth_limit,(uint8_t)((w->depth_limit+1u)/2u)};
    if(w->hooks.queue_policy)queue=w->hooks.queue_policy(w->hooks.decoder.context,config);
    uint8_t depth=(uint8_t)(queue.depth&DBT_CODEC_QUEUE_DEPTH_MASK);
    bool packed=(queue.depth&DBT_CODEC_QUEUE_PACKED)!=0;
    if(!depth || depth>w->depth_limit || (depth&(depth-1u)) ||
       !queue.low_water || queue.low_water>depth || (packed && depth<2u))return DBT_INVALID;
    w->depth=depth;w->low_water=queue.low_water;w->packed_media=packed;
    dbt_result_t result=dbt_codec_open(&w->decoder,config);if(result!=DBT_OK)return result;
    w->packet_storage=dbt_codec_allocate(&w->decoder,w->stride*w->depth);
    if(!w->packet_storage){dbt_codec_close(&w->decoder);return DBT_NO_SPACE;}
    w->configured=true;w->sequence_valid=false;atomic_store(&w->media_written,0);atomic_store(&w->media_read,0);
    atomic_store(&w->media_page_written,0);atomic_store(&w->media_page_read,0);
    w->media_write_offset=w->media_read_offset=0;
    atomic_store_explicit(&w->queue_layout,(uint32_t)queue.depth|((uint32_t)w->low_water<<8)|((uint32_t)w->stride<<16),memory_order_release);
    return DBT_OK;
}
static void fault(dbt_codec_worker_t *w){
    atomic_store_explicit(&w->started_epoch,0,memory_order_release);
    pending_clear(w);w->media_active=false;
    w->streaming=w->sequence_valid=false;
    if(!w->hooks.async_stream.begin && w->hooks.stream)(void)w->hooks.stream(w->hooks.decoder.context,false,&w->decoder.config);
    atomic_fetch_add(&w->faults,1);
}
static dbt_sink_work_result_t control(dbt_codec_worker_t *w,const dbt_codec_job_t *job,bool supplied,dbt_result_t completion,bool accepted){
    if(w->binding && w->binding!=job->binding)return DBT_SINK_WORK_FATAL;
    if(job->operation==DBT_SINK_CLOSE || job->operation==DBT_SINK_ABORT || job->operation==DBT_SINK_DROP){
        dbt_result_t stopped=supplied?completion:w->hooks.stream?w->hooks.stream(w->hooks.decoder.context,false,&w->decoder.config):DBT_OK;
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
        dbt_result_t result=supplied?completion:w->hooks.stream?w->hooks.stream(w->hooks.decoder.context,start,&w->decoder.config):DBT_OK;
        if(result!=DBT_OK){
            if(accepted){pending_clear(w);w->media_active=false;w->streaming=w->sequence_valid=false;atomic_store(&w->started_epoch,0);}
            else if(w->streaming)atomic_store(&w->started_epoch,job->epoch);
            return accepted && result!=DBT_CANCELLED?DBT_SINK_WORK_FATAL:DBT_SINK_WORK_REJECTED;
        }
        pending_clear(w);w->media_active=false;
        w->sequence_valid=false;atomic_store(&w->media_read,atomic_load(&w->media_written));
        atomic_store(&w->media_page_written,0);atomic_store(&w->media_page_read,0);
        w->media_write_offset=w->media_read_offset=0;
        if(w->decoder.config.ops->reset(&w->decoder)!=DBT_OK){fault(w);return DBT_SINK_WORK_FATAL;}
        if(start){
            atomic_store(&w->duration_input_frames,0);
            atomic_store(&w->duration_pcm_base,atomic_load(&w->pcm_delivered_frames));
            atomic_store(&w->duration_reject_base,atomic_load(&w->decode_rejects));
            bool supported=w->media_frames && w->hooks.deferred_pcm && !w->hooks.async_pcm.begin;
            atomic_store_explicit(&w->duration_epoch,supported?job->epoch:0,memory_order_release);
        }
        w->streaming=start;if(start)atomic_store_explicit(&w->started_epoch,job->epoch,memory_order_release);
        return DBT_SINK_WORK_READY;
    }
    return DBT_SINK_WORK_REJECTED;
}
/* One control job pins its input and its reserved receipt capacity until its
 * renderer ticket retires. A newer epoch cancels it before starting the next
 * job; neither a pending operation nor a timeout restores producer admission. */
static bool control_async(dbt_codec_worker_t *w,const dbt_codec_job_t *job,uint32_t now,dbt_sink_work_result_t *result){
    bool stream_job=job->operation==DBT_SINK_START || job->operation==DBT_SINK_SUSPEND ||
        job->operation==DBT_SINK_CLOSE || job->operation==DBT_SINK_ABORT || job->operation==DBT_SINK_DROP;
    if(!w->hooks.async_stream.begin || !stream_job || (w->binding && w->binding!=job->binding) ||
       ((job->operation==DBT_SINK_START || job->operation==DBT_SINK_SUSPEND) && !w->configured)){
        *result=control(w,job,false,DBT_OK,false);return true;
    }
    if(!w->control_phase){w->control_phase=1;w->control_since=now;w->control_timedout=false;w->cancel_requested=false;w->stream_ticket=0;}
    w->control_blocked=false;
    bool superseded=atomic_load_explicit(&w->gate_epoch,memory_order_acquire)!=job->epoch;
    if(!w->control_timedout && now-w->control_since>=1500u){
        w->control_timedout=true;atomic_store(&w->started_epoch,0);atomic_fetch_add(&w->faults,1);wake_owner(w);
    }
    if(w->control_phase==1 && superseded){
        *result=DBT_SINK_WORK_REJECTED;w->control_phase=0;return true;
    }
    if(w->control_phase==1 && w->control_timedout){
        /* Nothing was accepted: do not perform a late start when a busy
         * renderer eventually becomes available. Retain the working set for
         * the owner's explicit DROP, with media admission already closed. */
        *result=DBT_SINK_WORK_FATAL;w->control_phase=0;return true;
    }
    if(w->control_phase==1){
        uint32_t ticket=0;dbt_result_t begun=w->hooks.async_stream.begin(w->hooks.decoder.context,job->operation==DBT_SINK_START,&w->decoder.config,&ticket);
        if(begun==DBT_BUSY){w->control_blocked=true;return false;}
        if(begun==DBT_PENDING && ticket){w->stream_ticket=ticket;w->control_phase=2;w->control_blocked=true;return false;}
        if(begun==DBT_PENDING)begun=DBT_INVALID;
        *result=control(w,job,true,begun,false);w->control_phase=0;return true;
    }
    if((superseded || w->control_timedout) && !w->cancel_requested){
        dbt_result_t cancelled=w->hooks.async_stream.cancel(w->hooks.decoder.context,w->stream_ticket);
        if(cancelled==DBT_PENDING)w->cancel_requested=true;
        else if(cancelled==DBT_BUSY){w->control_blocked=true;return false;}
        else {
            *result=control(w,job,true,DBT_INVALID,true);w->control_phase=0;w->stream_ticket=0;return true;
        }
    }
    dbt_result_t polled=w->hooks.async_stream.poll(w->hooks.decoder.context,w->stream_ticket);
    if(polled==DBT_PENDING){w->control_blocked=true;return false;}
    *result=control(w,job,true,polled,true);w->control_phase=0;w->stream_ticket=0;w->cancel_requested=false;return true;
}
static void retire_slot(dbt_codec_worker_t *w,uint32_t read){
    if(w->packed_media){
        const media_t *slot=read_slot(w,read);
        w->media_read_offset=(uint16_t)(w->media_read_offset+record_stride(slot->size));
        if(w->media_read_offset==w->stride)release_page(w);
    }
    uint32_t queued=atomic_load_explicit(&w->media_written,memory_order_acquire)-read;
    atomic_store_explicit(&w->media_read,read+1u,memory_order_release);
    /* A partial receive-credit grant can become possible before the low-water
     * crossing. Signal each reclaimed slot in this pressure region; the host
     * may coalesce notifications. Below it all controller permits already fit. */
    if(w->packed_media || queued>=w->low_water)wake_owner(w);
}
static void retire_media(dbt_codec_worker_t *w,uint32_t read,dbt_result_t result){
    if(result!=DBT_OK){
        const media_t *slot=read_slot(w,read);
        atomic_fetch_add(&w->decode_rejects,1);trace(w,DBT_CODEC_DECODE_REJECT,slot,slot->sequence,slot->ssrc,result);
        pending_clear(w);atomic_fetch_add(&w->media_drops,1);
        if(!reset(w,slot))fault(w);
    }
    w->media_active=false;atomic_fetch_add(&w->packets,1);
    retire_slot(w,read);
}
static void fail_media(dbt_codec_worker_t *w,uint32_t read){
    /* Fatal output/decoder failure needs no replacement codec allocation.
     * Disable admission, retire the lease, and let the owner's DROP reclaim. */
    atomic_fetch_add(&w->media_drops,1);fault(w);retire_media(w,read,DBT_OK);
}
/* Service at most one accepted loan before any job can reset/free its codec.
 * A provider contract failure closes admission but retains the pinned decoder
 * and packet until a real retirement acknowledgement arrives. */
static bool pcm_retire(dbt_codec_worker_t *w,uint32_t now,bool cancel){
    if(!w->pcm_ticket)return true;
    w->output_blocked=true;w->control_blocked=true;
    if(!w->pcm_timedout && now-w->last_progress>=DBT_CODEC_PCM_STALL_MS){
        w->pcm_timedout=true;atomic_fetch_add(&w->pcm_timeouts,1);
        uint32_t read=atomic_load_explicit(&w->media_read,memory_order_relaxed);
        const media_t *slot=read_slot(w,read);
        trace(w,DBT_CODEC_PCM_TIMEOUT,slot,slot->sequence,slot->ssrc,DBT_BUSY);
        if(!w->pcm_fault_latched){
            w->pcm_fault_latched=true;atomic_store(&w->started_epoch,0);
            atomic_fetch_add(&w->media_drops,1);atomic_fetch_add(&w->faults,1);wake_owner(w);
        }
    }
    if((cancel || w->pcm_timedout || w->pcm_fault_latched) && !w->pcm_cancel_requested){
        dbt_result_t result=w->hooks.async_pcm.cancel(w->hooks.decoder.context,w->pcm_ticket);
        if(result==DBT_PENDING)w->pcm_cancel_requested=true;
        else if(result!=DBT_BUSY && !w->pcm_fault_latched){
            w->pcm_fault_latched=true;atomic_store(&w->started_epoch,0);
            atomic_fetch_add(&w->pcm_failures,1);atomic_fetch_add(&w->media_drops,1);atomic_fetch_add(&w->faults,1);wake_owner(w);
        }
        return false;
    }
    dbt_result_t result=w->hooks.async_pcm.poll(w->hooks.decoder.context,w->pcm_ticket);
    if(result==DBT_PENDING)return false;
    if(result!=DBT_OK && result!=DBT_CANCELLED){
        if(!w->pcm_fault_latched){
            w->pcm_fault_latched=true;atomic_store(&w->started_epoch,0);
            atomic_fetch_add(&w->pcm_failures,1);atomic_fetch_add(&w->media_drops,1);atomic_fetch_add(&w->faults,1);wake_owner(w);
        }
        return false;
    }
    bool aborted=w->pcm_cancel_requested || w->pcm_timedout || w->pcm_fault_latched || result==DBT_CANCELLED;
    uint16_t take=w->pcm_ticket_frames;w->pcm_ticket=0;w->pcm_ticket_frames=0;
    w->pcm_cancel_requested=w->pcm_timedout=w->pcm_fault_latched=false;
    w->output_blocked=w->control_blocked=false;
    if(!cancel && aborted){
        uint32_t read=atomic_load_explicit(&w->media_read,memory_order_relaxed);
        atomic_fetch_add(&w->pcm_failures,1);fail_media(w,read);return true;
    }
    if(!aborted){
        w->pending_offset=(uint16_t)(w->pending_offset+take);w->last_progress=now;
        atomic_fetch_add(&w->pcm_delivered_frames,take);
        if(w->pending_offset==w->pending_frames)pending_clear(w);
    }
    return true;
}
static uint16_t deferred_step(dbt_codec_worker_t *w,uint32_t now,uint16_t budget){
    uint16_t done=0;
    while(done<budget && enabled(w)){
        uint32_t read=atomic_load_explicit(&w->media_read,memory_order_relaxed);
        if(w->pending_pcm){
            uint16_t remaining=(uint16_t)(w->pending_frames-w->pending_offset);
            uint16_t take=remaining>DBT_CODEC_PCM_CHUNK_FRAMES?DBT_CODEC_PCM_CHUNK_FRAMES:remaining;
            uint32_t stride=(uint32_t)w->decoder.config.channels*w->decoder.config.sample_bytes;
            const media_t *slot=read_slot(w,read);
            trace(w,DBT_CODEC_PCM_BEGIN,slot,slot->sequence,slot->ssrc,DBT_OK);
            uint32_t ticket=0;
            dbt_result_t result=w->hooks.async_pcm.begin?
                w->hooks.async_pcm.begin(w->hooks.decoder.context,w->pending_pcm+(uint32_t)w->pending_offset*stride,
                    take,&w->decoder.config,w->pending_timestamp+w->pending_offset,&ticket):
                w->hooks.decoder.pcm(w->hooks.decoder.context,w->pending_pcm+(uint32_t)w->pending_offset*stride,
                    take,&w->decoder.config,w->pending_timestamp+w->pending_offset);
            trace(w,DBT_CODEC_PCM_END,slot,slot->sequence,slot->ssrc,result);
            if(w->hooks.async_pcm.begin && result==DBT_PENDING && ticket){
                w->pcm_ticket=ticket;w->pcm_ticket_frames=take;w->output_blocked=true;
                ++done;break;
            }
            if(result==DBT_BUSY){
                if(!w->output_blocked){
                    const media_t *slot=read_slot(w,read);
                    trace(w,DBT_CODEC_PCM_BUSY,slot,slot->sequence,slot->ssrc,result);
                }
                w->output_blocked=true;atomic_fetch_add(&w->pcm_backpressure,1);
                if(now-w->last_progress>=DBT_CODEC_PCM_STALL_MS){
                    const media_t *slot=read_slot(w,read);
                    trace(w,DBT_CODEC_PCM_TIMEOUT,slot,slot->sequence,slot->ssrc,DBT_BUSY);
                    atomic_fetch_add(&w->pcm_timeouts,1);fail_media(w,read);
                }
                break;
            }
            ++done;
            if(result!=DBT_OK || w->hooks.async_pcm.begin){
                const media_t *slot=read_slot(w,read);
                atomic_fetch_add(&w->pcm_failures,1);trace(w,DBT_CODEC_PCM_FAILED,slot,slot->sequence,slot->ssrc,result);
                fail_media(w,read);break;
            }
            w->pending_offset=(uint16_t)(w->pending_offset+take);w->last_progress=now;w->output_blocked=false;
            atomic_fetch_add(&w->pcm_delivered_frames,take);
            if(w->pending_offset==w->pending_frames)pending_clear(w);
            continue;
        }
        dbt_result_t result;
        if(w->media_active)result=decode(w,read_slot(w,read),now,true);
        else {
            if(read==atomic_load_explicit(&w->media_written,memory_order_acquire))break;
            media_t *slot=read_slot(w,read);
            if(w->sequence_valid && (slot->sequence!=(uint16_t)(w->last_sequence+1u) || slot->ssrc!=w->last_ssrc)){
                atomic_fetch_add(&w->resets,1);
                worker_sequence(w,slot);
                if(!reset(w,slot)){fail_media(w,read);++done;break;}
            }
            w->sequence_valid=true;w->last_sequence=slot->sequence;w->last_ssrc=slot->ssrc;
            w->last_progress=now;result=decode(w,slot,now,false);
        }
        ++done;
        if(result==DBT_PENDING && w->pending_pcm){w->media_active=true;w->last_progress=now;continue;}
        if(result==DBT_PROGRESS && !w->pending_pcm && w->decoder.config.ops->resume){
            w->media_active=true;w->last_progress=now;continue;
        }
        /* PENDING without an accepted view, or OK after ignoring a view's
         * PENDING return, is a codec contract violation, never a retry spin. */
        if(result==DBT_PENDING || result==DBT_PROGRESS || w->pending_pcm)result=DBT_INVALID;
        retire_media(w,read,result);
    }
    return done;
}
bool dbt_codec_worker_runnable(const dbt_codec_worker_t *w){
    if(!w)return false;
    if(w->pcm_ticket)return false;
    if(atomic_load(&w->job_read)!=atomic_load_explicit(&w->job_written,memory_order_acquire) &&
       atomic_load(&w->receipt_written)-atomic_load_explicit(&w->receipt_read,memory_order_acquire)<DBT_CODEC_WORKER_JOBS)return !w->control_blocked;
    return enabled(w) && !w->output_blocked && (w->media_active || w->pending_pcm ||
        atomic_load(&w->media_read)!=atomic_load_explicit(&w->media_written,memory_order_acquire));
}
bool dbt_codec_worker_deadline(const dbt_codec_worker_t *w,uint32_t *when){
    if(!w || !when)return false;
    if(w->pcm_ticket){
        if(w->pcm_timedout || w->pcm_fault_latched || w->pcm_cancel_requested)return false;
        *when=w->last_progress+DBT_CODEC_PCM_STALL_MS;return true;
    }
    if(w->control_phase){if(w->control_timedout)return false;*when=w->control_since+1500u;return true;}
    if(enabled(w) && w->pending_pcm && w->output_blocked){*when=w->last_progress+DBT_CODEC_PCM_STALL_MS;return true;}
    if(w->configured && !w->media_active && !w->pending_pcm && w->decoder.config.ops->deadline)
        return w->decoder.config.ops->deadline(&w->decoder,when);
    return false;
}
uint16_t dbt_codec_worker_step(dbt_codec_worker_t *w,uint32_t now,uint16_t budget){
    if(!w)return 0;
    bool jobs=atomic_load(&w->job_read)!=atomic_load_explicit(&w->job_written,memory_order_acquire);
    if(!pcm_retire(w,now,jobs || !enabled(w)))return 0;
    for(unsigned i=0;i<DBT_CODEC_WORKER_JOBS;++i){
        uint32_t read=atomic_load_explicit(&w->job_read,memory_order_relaxed);if(read==atomic_load_explicit(&w->job_written,memory_order_acquire))break;
        uint32_t written=atomic_load_explicit(&w->receipt_written,memory_order_relaxed);
        if(written-atomic_load_explicit(&w->receipt_read,memory_order_acquire)>=DBT_CODEC_WORKER_JOBS)break;
        const dbt_codec_job_t *job=&w->jobs[read%DBT_CODEC_WORKER_JOBS];dbt_sink_work_result_t result;
        if(!control_async(w,job,now,&result))break;
        w->receipts[written%DBT_CODEC_WORKER_JOBS]=(dbt_codec_receipt_t){job->binding,job->token,result};
        atomic_store(&w->resident_bytes,w->decoder.allocated_bytes);atomic_store_explicit(&w->job_read,read+1u,memory_order_release);
        atomic_store_explicit(&w->receipt_written,written+1u,memory_order_release);wake_owner(w);
    }
    uint16_t count=0;
    if(w->hooks.deferred_pcm)count=deferred_step(w,now,budget);
    while(!w->hooks.deferred_pcm && count<budget && enabled(w)){
        uint32_t read=atomic_load_explicit(&w->media_read,memory_order_relaxed);if(read==atomic_load_explicit(&w->media_written,memory_order_acquire))break;
        media_t *slot=read_slot(w,read);
        if(!w->media_active && w->sequence_valid && (slot->sequence!=(uint16_t)(w->last_sequence+1u) || slot->ssrc!=w->last_ssrc)){
            atomic_fetch_add(&w->resets,1);
            worker_sequence(w,slot);
            if(!reset(w,slot)){
                atomic_fetch_add(&w->media_drops,1);retire_slot(w,read);fault(w);++count;break;
            }
        }
        if(!w->media_active){w->sequence_valid=true;w->last_sequence=slot->sequence;w->last_ssrc=slot->ssrc;}
        dbt_result_t result=decode(w,slot,now,w->media_active);
        if(result==DBT_PROGRESS && w->decoder.config.ops->resume){w->media_active=true;++count;continue;}
        w->media_active=false;
        if(result!=DBT_OK){
            atomic_fetch_add(&w->decode_rejects,1);trace(w,DBT_CODEC_DECODE_REJECT,slot,slot->sequence,slot->ssrc,result);
            atomic_fetch_add(&w->media_drops,1);
            if(!reset(w,slot))fault(w);
        }
        atomic_fetch_add(&w->packets,1);retire_slot(w,read);++count;
    }
    if(w->configured && !w->media_active && !w->pending_pcm && w->decoder.config.ops->expire)w->decoder.config.ops->expire(&w->decoder,now);
    atomic_store(&w->resident_bytes,w->decoder.allocated_bytes);return count;
}
