#include "dbt_sbc_worker.h"

typedef struct {
    uint32_t timestamp,ssrc;
    uint16_t sequence,size;
    uint8_t payload_type;
    bool marker;
    uint8_t bytes[];
} media_slot_t;
static void zero(void *p,unsigned n){uint8_t *bytes=p;for(unsigned i=0;i<n;++i)bytes[i]=0;}
static void copy(uint8_t *to,const uint8_t *from,uint16_t n){for(unsigned i=0;i<n;++i)to[i]=from[i];}
static bool enabled(const dbt_sbc_worker_t *w){
    uint32_t started=atomic_load_explicit(&w->started_epoch,memory_order_acquire);
    return started && started==atomic_load_explicit(&w->gate_epoch,memory_order_acquire);
}
static void wake_worker(dbt_sbc_worker_t *w){if(w->hooks.wake_worker)w->hooks.wake_worker(w->hooks.decoder.context);}
static void wake_owner(dbt_sbc_worker_t *w){if(w->hooks.wake_owner)w->hooks.wake_owner(w->hooks.decoder.context);}
dbt_result_t dbt_sbc_worker_init(dbt_sbc_worker_t *w,dbt_sbc_worker_hooks_t hooks,uint16_t payload,uint8_t depth){
    uint32_t stride=(sizeof(media_slot_t)+payload+3u)&~3u;
    if(!w || !hooks.decoder.allocate || !hooks.decoder.release || !hooks.decoder.pcm ||
       payload<5 || payload>4096 || !depth || depth>16 || (depth&(depth-1u)) || stride*depth>UINT16_MAX)return DBT_INVALID;
    zero(w,sizeof(*w));w->hooks=hooks;w->maximum_payload=payload;w->depth=depth;w->stride=(uint16_t)stride;
    atomic_init(&w->job_written,0);atomic_init(&w->job_read,0);atomic_init(&w->receipt_written,0);atomic_init(&w->receipt_read,0);
    atomic_init(&w->gate_epoch,0);atomic_init(&w->started_epoch,0);atomic_init(&w->media_written,0);atomic_init(&w->media_read,0);
    atomic_init(&w->packets,0);atomic_init(&w->media_drops,0);atomic_init(&w->resets,0);atomic_init(&w->resident_bytes,0);
    return dbt_sbc_decoder_init(&w->decoder,hooks.decoder);
}
dbt_result_t dbt_sbc_worker_submit(void *context,const dbt_sink_job_t *request){
    dbt_sbc_worker_t *w=context;
    if(!w || !request || !request->binding || !request->token || request->operation>DBT_SINK_DROP || request->operation<DBT_SINK_CONFIGURE)return DBT_INVALID;
    dbt_sbc_config_t config={0};
    if(request->operation==DBT_SINK_CONFIGURE || request->operation==DBT_SINK_RECONFIGURE){
        dbt_avdtp_caps_t caps;dbt_avdtp_cap_error_t error;
        if(dbt_avdtp_caps_validate(request->configuration,request->size,DBT_AVDTP_CONFIGURATION,&caps,&error)!=DBT_OK ||
           dbt_sbc_configure(caps.codec.data,caps.codec.size,&config)!=DBT_OK)return DBT_UNSUPPORTED;
    }
    /* Gate first, even when a full control queue requires retry. Host submission
     * and media publication have the same producer, so the control release also
     * orders every earlier payload copy before worker-side destruction. */
    uint32_t epoch=atomic_fetch_add_explicit(&w->gate_epoch,1,memory_order_acq_rel)+1u;
    if(!epoch)epoch=atomic_fetch_add_explicit(&w->gate_epoch,1,memory_order_acq_rel)+1u;
    uint32_t written=atomic_load_explicit(&w->job_written,memory_order_relaxed);
    uint32_t read=atomic_load_explicit(&w->job_read,memory_order_acquire);
    if(written-read>=DBT_SBC_WORKER_JOBS)return DBT_BUSY;
    dbt_sbc_work_t *job=&w->jobs[written%DBT_SBC_WORKER_JOBS];
    job->binding=request->binding;job->token=request->token;job->operation=request->operation;job->epoch=epoch;
    job->configuration=config;
    atomic_store_explicit(&w->job_written,written+1u,memory_order_release);wake_worker(w);return DBT_OK;
}
dbt_result_t dbt_sbc_worker_media(void *context,const dbt_avdtp_media_t *media){
    dbt_sbc_worker_t *w=context;
    if(!w || !media || !media->payload || !media->size || media->size>w->maximum_payload)return DBT_INVALID;
    if(!enabled(w))return DBT_BUSY;
    uint32_t written=atomic_load_explicit(&w->media_written,memory_order_relaxed);
    uint32_t read=atomic_load_explicit(&w->media_read,memory_order_acquire);
    if(written-read>=w->depth){atomic_fetch_add_explicit(&w->media_drops,1,memory_order_relaxed);return DBT_NO_SPACE;}
    media_slot_t *slot=(media_slot_t *)(w->packet_storage+(written%w->depth)*w->stride);
    slot->timestamp=media->timestamp;slot->ssrc=media->ssrc;slot->sequence=media->sequence;
    slot->size=media->size;slot->payload_type=media->payload_type;slot->marker=media->marker;
    copy(slot->bytes,media->payload,media->size);
    atomic_store_explicit(&w->media_written,written+1u,memory_order_release);wake_worker(w);return DBT_OK;
}
dbt_result_t dbt_sbc_worker_receipt(dbt_sbc_worker_t *w,dbt_sbc_receipt_t *receipt){
    if(!w || !receipt)return DBT_INVALID;
    uint32_t read=atomic_load_explicit(&w->receipt_read,memory_order_relaxed);
    if(read==atomic_load_explicit(&w->receipt_written,memory_order_acquire))return DBT_EMPTY;
    *receipt=w->receipts[read%DBT_SBC_WORKER_JOBS];
    atomic_store_explicit(&w->receipt_read,read+1u,memory_order_release);wake_worker(w);return DBT_OK;
}
static void clear(dbt_sbc_worker_t *w){
    atomic_store_explicit(&w->started_epoch,0,memory_order_release);
    dbt_sbc_media_reset(&w->media);dbt_sbc_decoder_close(&w->decoder);
    if(w->packet_storage)w->hooks.decoder.release(w->hooks.decoder.context,w->packet_storage);
    w->packet_storage=0;w->configured=w->streaming=w->sequence_valid=false;
    atomic_store_explicit(&w->resident_bytes,0,memory_order_relaxed);
}
static dbt_result_t construct(dbt_sbc_worker_t *w,dbt_sbc_config_t config){
    dbt_result_t result=dbt_sbc_decoder_open(&w->decoder,config);if(result!=DBT_OK)return result;
    w->packet_storage=w->hooks.decoder.allocate(w->hooks.decoder.context,(uint16_t)(w->stride*w->depth));
    if(!w->packet_storage){dbt_sbc_decoder_close(&w->decoder);return DBT_NO_SPACE;}
    w->configured=true;w->sequence_valid=false;
    atomic_store_explicit(&w->media_written,0,memory_order_relaxed);atomic_store_explicit(&w->media_read,0,memory_order_relaxed);
    atomic_store_explicit(&w->resident_bytes,w->decoder.allocated_bytes+w->stride*w->depth,memory_order_relaxed);
    return DBT_OK;
}
static uint8_t *fragment_allocate(void *context,uint16_t bytes){
    dbt_sbc_worker_t *w=context;uint8_t *p=w->hooks.decoder.allocate(w->hooks.decoder.context,bytes);
    if(p)atomic_fetch_add_explicit(&w->resident_bytes,bytes,memory_order_relaxed);
    return p;
}
static void fragment_release(void *context,uint8_t *bytes){
    dbt_sbc_worker_t *w=context;atomic_fetch_sub_explicit(&w->resident_bytes,w->media.capacity,memory_order_relaxed);
    w->hooks.decoder.release(w->hooks.decoder.context,bytes);
}
static dbt_result_t decode_frame(void *context,const dbt_sbc_frame_t *frame,uint32_t timestamp){dbt_sbc_worker_t *w=context;return dbt_sbc_decoder_frame(&w->decoder,frame,timestamp);}
static dbt_sink_work_result_t control(dbt_sbc_worker_t *w,const dbt_sbc_work_t *job){
    if(w->binding && w->binding!=job->binding)return DBT_SINK_WORK_FATAL;
    if(job->operation==DBT_SINK_DROP || job->operation==DBT_SINK_CLOSE || job->operation==DBT_SINK_ABORT){
        dbt_result_t stopped=w->hooks.stream?w->hooks.stream(w->hooks.decoder.context,false):DBT_OK;
        clear(w);if(job->operation==DBT_SINK_DROP)w->binding=0;
        return stopped==DBT_OK?DBT_SINK_WORK_READY:DBT_SINK_WORK_FATAL;
    }
    if(job->operation==DBT_SINK_CONFIGURE || job->operation==DBT_SINK_RECONFIGURE){
        dbt_sbc_config_t old=w->decoder.config;bool restore=w->configured && job->operation==DBT_SINK_RECONFIGURE;
        clear(w);w->binding=job->binding;
        dbt_result_t result=construct(w,job->configuration);
        if(result!=DBT_OK){
            if(!restore)return DBT_SINK_WORK_REJECTED;
            if(construct(w,old)!=DBT_OK)return DBT_SINK_WORK_FATAL;
        }
        dbt_sbc_media_hooks_t hooks={w,fragment_allocate,fragment_release,decode_frame};
        (void)dbt_sbc_media_init(&w->media,hooks,w->decoder.config,100);
        return result==DBT_OK?DBT_SINK_WORK_READY:DBT_SINK_WORK_REJECTED;
    }
    if(!w->configured)return DBT_SINK_WORK_REJECTED;
    if(job->operation==DBT_SINK_OPEN)return DBT_SINK_WORK_READY;
    if(job->operation==DBT_SINK_START || job->operation==DBT_SINK_SUSPEND){
        bool start=job->operation==DBT_SINK_START;
        dbt_result_t result=w->hooks.stream?w->hooks.stream(w->hooks.decoder.context,start):DBT_OK;
        if(result!=DBT_OK){
            if(w->streaming)atomic_store_explicit(&w->started_epoch,job->epoch,memory_order_release);
            return DBT_SINK_WORK_REJECTED;
        }
        dbt_sbc_media_reset(&w->media);w->sequence_valid=false;
        uint32_t written=atomic_load_explicit(&w->media_written,memory_order_acquire);
        atomic_store_explicit(&w->media_read,written,memory_order_release);
        if(dbt_sbc_decoder_reset(&w->decoder)!=DBT_OK)return DBT_SINK_WORK_FATAL;
        w->streaming=start;
        if(start)atomic_store_explicit(&w->started_epoch,job->epoch,memory_order_release);
        return DBT_SINK_WORK_READY;
    }
    return DBT_SINK_WORK_REJECTED;
}
uint16_t dbt_sbc_worker_step(dbt_sbc_worker_t *w,uint32_t now,uint16_t budget){
    if(!w)return 0;
    for(unsigned i=0;i<DBT_SBC_WORKER_JOBS;++i){
        uint32_t read=atomic_load_explicit(&w->job_read,memory_order_relaxed);
        if(read==atomic_load_explicit(&w->job_written,memory_order_acquire))break;
        uint32_t written=atomic_load_explicit(&w->receipt_written,memory_order_relaxed);
        if(written-atomic_load_explicit(&w->receipt_read,memory_order_acquire)>=DBT_SBC_WORKER_JOBS)break;
        const dbt_sbc_work_t *job=&w->jobs[read%DBT_SBC_WORKER_JOBS];
        dbt_sink_work_result_t result=control(w,job);
        w->receipts[written%DBT_SBC_WORKER_JOBS]=(dbt_sbc_receipt_t){job->binding,job->token,result};
        atomic_store_explicit(&w->job_read,read+1u,memory_order_release);
        atomic_store_explicit(&w->receipt_written,written+1u,memory_order_release);wake_owner(w);
    }
    uint16_t packets=0;
    while(packets<budget && enabled(w)){
        uint32_t read=atomic_load_explicit(&w->media_read,memory_order_relaxed);
        if(read==atomic_load_explicit(&w->media_written,memory_order_acquire))break;
        media_slot_t *slot=(media_slot_t *)(w->packet_storage+(read%w->depth)*w->stride);
        if(w->sequence_valid && (slot->sequence!=(uint16_t)(w->last_sequence+1u) || slot->ssrc!=w->last_ssrc)){
            dbt_sbc_media_reset(&w->media);(void)dbt_sbc_decoder_reset(&w->decoder);
            atomic_fetch_add_explicit(&w->resets,1,memory_order_relaxed);
        }
        w->sequence_valid=true;w->last_sequence=slot->sequence;w->last_ssrc=slot->ssrc;
        dbt_avdtp_media_t packet={slot->bytes,slot->timestamp,slot->ssrc,slot->size,slot->sequence,slot->payload_type,slot->marker};
        if(dbt_sbc_media_receive(&w->media,&packet,now)!=DBT_OK){
            atomic_fetch_add_explicit(&w->media_drops,1,memory_order_relaxed);(void)dbt_sbc_decoder_reset(&w->decoder);
        }
        atomic_fetch_add_explicit(&w->packets,1,memory_order_relaxed);
        atomic_store_explicit(&w->media_read,read+1u,memory_order_release);++packets;
    }
    dbt_sbc_media_expire(&w->media,now);return packets;
}
