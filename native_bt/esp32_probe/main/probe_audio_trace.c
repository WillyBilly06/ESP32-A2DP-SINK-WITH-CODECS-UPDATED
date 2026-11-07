#include "probe_audio_trace.h"
#include <stddef.h>
#include <string.h>
#ifndef PROBE_AUDIO_TRACE_TEST_PART
#define PROBE_AUDIO_TRACE_TEST_PART(part) ((void)(part))
#endif
void probe_audio_trace_init(probe_audio_trace_ring_t *ring){
    if(!ring)return;
    atomic_init(&ring->written,0);atomic_init(&ring->read,0);atomic_init(&ring->lost,0);ring->ordinal=0;
}
static bool reserve(probe_audio_trace_ring_t *ring,uint32_t count,uint32_t *written,uint32_t *ordinal){
    *ordinal=++ring->ordinal;
    *written=atomic_load_explicit(&ring->written,memory_order_relaxed);
    uint32_t used=*written-atomic_load_explicit(&ring->read,memory_order_acquire);
    if(used>PROBE_AUDIO_TRACE_SLOTS || count>PROBE_AUDIO_TRACE_SLOTS-used){
        /* One producer owns this counter; the consumer only observes it. */
        atomic_store_explicit(&ring->lost,atomic_load_explicit(&ring->lost,memory_order_relaxed)+1u,memory_order_relaxed);
        return false;
    }
    return true;
}
bool probe_audio_trace_media(probe_audio_trace_ring_t *ring,const dbt_codec_trace_t *event,
    uint32_t codec,probe_audio_trace_clock_t clock,void *context){
    if(!ring||!event||!clock||codec>=PROBE_AUDIO_TRACE_CONTROLLER)return false;
    uint32_t written,ordinal;if(!reserve(ring,1,&written,&ordinal))return false;
    ring->items[written&(PROBE_AUDIO_TRACE_SLOTS-1u)]=(probe_audio_trace_t){.event=*event,.at_us=clock(context),.ordinal=ordinal,.codec=codec};
    atomic_store_explicit(&ring->written,written+1u,memory_order_release);return true;
}
dbt_result_t probe_audio_trace_host(probe_audio_trace_ring_t *ring,const probe_host_record_t *record,
    probe_audio_trace_clock_t clock,void *context){
    if(!ring||!record||!clock||(record->kind!=PROBE_HOST_CLASSIC_VISIBILITY&&record->kind!=PROBE_HOST_GAP_STATE&&record->kind!=PROBE_HOST_MEMORY_SERVICES))return DBT_INVALID;
    uint32_t written,ordinal;if(!reserve(ring,1,&written,&ordinal))return DBT_NO_SPACE;
    ring->items[written&(PROBE_AUDIO_TRACE_SLOTS-1u)]=(probe_audio_trace_t){.host=*record,.at_us=clock(context),.ordinal=ordinal,.codec=PROBE_AUDIO_TRACE_HOST};
    atomic_store_explicit(&ring->written,written+1u,memory_order_release);return DBT_OK;
}
static dbt_result_t push_group(probe_audio_trace_ring_t *ring,const void *snapshot,uint32_t parts,uint32_t kind,
    probe_audio_trace_clock_t clock,void *context){
    if(!ring||!snapshot||!clock)return DBT_INVALID;
    uint32_t written,ordinal;if(!reserve(ring,parts,&written,&ordinal))return DBT_NO_SPACE;
    uint32_t at=clock(context);
    for(unsigned part=0;part<parts;++part){
        probe_audio_trace_t *slot=&ring->items[(written+part)&(PROBE_AUDIO_TRACE_SLOTS-1u)];
        memcpy(slot->failure_words,(const unsigned char *)snapshot+part*sizeof(slot->failure_words),sizeof(slot->failure_words));
        slot->at_us=at;slot->ordinal=ordinal;slot->codec=part?PROBE_AUDIO_TRACE_CONTINUATION:kind;
        PROBE_AUDIO_TRACE_TEST_PART(part);
    }
    atomic_store_explicit(&ring->written,written+parts,memory_order_release);return DBT_OK;
}
dbt_result_t probe_audio_trace_failure(probe_audio_trace_ring_t *ring,const probe_host_failure_t *failure,
    probe_audio_trace_clock_t clock,void *context){
    return push_group(ring,failure,PROBE_AUDIO_TRACE_FAILURE_PARTS,PROBE_AUDIO_TRACE_FAILURE,clock,context);
}
dbt_result_t probe_audio_trace_controller(probe_audio_trace_ring_t *ring,const probe_host_controller_t *controller,
    probe_audio_trace_clock_t clock,void *context){
    return push_group(ring,controller,PROBE_AUDIO_TRACE_CONTROLLER_PARTS,PROBE_AUDIO_TRACE_CONTROLLER,clock,context);
}
bool probe_audio_trace_pop(probe_audio_trace_ring_t *ring,probe_audio_trace_record_t *record){
    if(!ring||!record)return false;
    uint32_t read=atomic_load_explicit(&ring->read,memory_order_relaxed);
    uint32_t available=atomic_load_explicit(&ring->written,memory_order_acquire)-read;if(!available)return false;
    const probe_audio_trace_t *first=&ring->items[read&(PROBE_AUDIO_TRACE_SLOTS-1u)];
    record->at_us=first->at_us;record->ordinal=first->ordinal;record->codec=first->codec;
    uint32_t count=1;
    if(first->codec==PROBE_AUDIO_TRACE_FAILURE || first->codec==PROBE_AUDIO_TRACE_CONTROLLER){
        uint32_t parts=first->codec==PROBE_AUDIO_TRACE_FAILURE?PROBE_AUDIO_TRACE_FAILURE_PARTS:PROBE_AUDIO_TRACE_CONTROLLER_PARTS;
        bool valid=available>=parts && available<=PROBE_AUDIO_TRACE_SLOTS;
        for(unsigned part=1;valid && part<parts;++part){
            const probe_audio_trace_t *slot=&ring->items[(read+part)&(PROBE_AUDIO_TRACE_SLOTS-1u)];
            valid=slot->codec==PROBE_AUDIO_TRACE_CONTINUATION && slot->ordinal==first->ordinal && slot->at_us==first->at_us;
        }
        if(valid){
            unsigned char *destination=first->codec==PROBE_AUDIO_TRACE_FAILURE?(unsigned char *)&record->failure:(unsigned char *)&record->controller;
            for(unsigned part=0;part<parts;++part){
                const probe_audio_trace_t *slot=&ring->items[(read+part)&(PROBE_AUDIO_TRACE_SLOTS-1u)];
                memcpy(destination+part*sizeof(slot->failure_words),slot->failure_words,sizeof(slot->failure_words));
            }
            count=parts;
        }else record->codec=PROBE_AUDIO_TRACE_CORRUPT;
    }else if(first->codec==PROBE_AUDIO_TRACE_CONTINUATION || first->codec==PROBE_AUDIO_TRACE_CORRUPT)record->codec=PROBE_AUDIO_TRACE_CORRUPT;
    else if(first->codec==PROBE_AUDIO_TRACE_HOST)record->host=first->host;
    else record->event=first->event;
    atomic_store_explicit(&ring->read,read+count,memory_order_release);return true;
}
