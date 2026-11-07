#include "probe_audio_trace.h"
#include <stddef.h>
void probe_audio_trace_init(probe_audio_trace_ring_t *ring){
    if(!ring)return;
    atomic_init(&ring->written,0);atomic_init(&ring->read,0);atomic_init(&ring->lost,0);ring->ordinal=0;
}
static probe_audio_trace_t *reserve(probe_audio_trace_ring_t *ring,uint32_t *written,uint32_t *ordinal){
    *ordinal=++ring->ordinal;
    *written=atomic_load_explicit(&ring->written,memory_order_relaxed);
    if(*written-atomic_load_explicit(&ring->read,memory_order_acquire)>=PROBE_AUDIO_TRACE_SLOTS){
        /* One producer owns this counter; the consumer only observes it. */
        atomic_store_explicit(&ring->lost,atomic_load_explicit(&ring->lost,memory_order_relaxed)+1u,memory_order_relaxed);
        return NULL;
    }
    return &ring->items[*written&(PROBE_AUDIO_TRACE_SLOTS-1u)];
}
bool probe_audio_trace_media(probe_audio_trace_ring_t *ring,const dbt_codec_trace_t *event,
    uint32_t codec,probe_audio_trace_clock_t clock,void *context){
    if(!ring||!event||!clock||codec==PROBE_AUDIO_TRACE_HOST)return false;
    uint32_t written,ordinal;probe_audio_trace_t *slot=reserve(ring,&written,&ordinal);if(!slot)return false;
    *slot=(probe_audio_trace_t){.event=*event,.at_us=clock(context),.ordinal=ordinal,.codec=codec};
    atomic_store_explicit(&ring->written,written+1u,memory_order_release);return true;
}
dbt_result_t probe_audio_trace_host(probe_audio_trace_ring_t *ring,const probe_host_record_t *record,
    probe_audio_trace_clock_t clock,void *context){
    if(!ring||!record||!clock||(record->kind!=PROBE_HOST_CLASSIC_VISIBILITY&&record->kind!=PROBE_HOST_GAP_STATE))return DBT_INVALID;
    uint32_t written,ordinal;probe_audio_trace_t *slot=reserve(ring,&written,&ordinal);if(!slot)return DBT_NO_SPACE;
    *slot=(probe_audio_trace_t){.host=*record,.at_us=clock(context),.ordinal=ordinal,.codec=PROBE_AUDIO_TRACE_HOST};
    atomic_store_explicit(&ring->written,written+1u,memory_order_release);return DBT_OK;
}
bool probe_audio_trace_pop(probe_audio_trace_ring_t *ring,probe_audio_trace_t *record){
    if(!ring||!record)return false;
    uint32_t read=atomic_load_explicit(&ring->read,memory_order_relaxed);
    if(read==atomic_load_explicit(&ring->written,memory_order_acquire))return false;
    *record=ring->items[read&(PROBE_AUDIO_TRACE_SLOTS-1u)];
    atomic_store_explicit(&ring->read,read+1u,memory_order_release);return true;
}
