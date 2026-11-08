#include "probe_arrival.h"
#include <stdatomic.h>
#ifdef ESP_PLATFORM
#include "esp_attr.h"
#define ARRIVAL_IRAM IRAM_ATTR
#else
#define ARRIVAL_IRAM
#endif
_Static_assert(ATOMIC_INT_LOCK_FREE==2,"Arrival metadata needs lock-free words");
_Static_assert(!(PROBE_ARRIVAL_SLOTS&(PROBE_ARRIVAL_SLOTS-1u)),"Power-of-two metadata capacity");
static struct {
    probe_arrival_sample_t slots[PROBE_ARRIVAL_SLOTS];
    _Atomic uint32_t published,consumed,lost,status;
} arrival;
static void invalidate(uint32_t reason){
    uint32_t empty=0;
    (void)atomic_compare_exchange_strong_explicit(&arrival.status,&empty,reason,memory_order_release,memory_order_relaxed);
}
void probe_arrival_init(void){
    atomic_init(&arrival.published,0);atomic_init(&arrival.consumed,0);
    atomic_init(&arrival.lost,0);atomic_init(&arrival.status,0);
}
bool probe_arrival_begin(uint32_t cursor,uint32_t at,const uint8_t *data,uint16_t size){
    if(!data || !size || data[0]!=DBT_H4_ACL)return false;
    if(size<5u || size>DBT_MAX_H4_BYTES || dbt_read_le16(data+3)!=(uint16_t)(size-5u)){
        invalidate(1);return false;
    }
    if(atomic_load_explicit(&arrival.status,memory_order_acquire))return false;
    uint32_t written=atomic_load_explicit(&arrival.published,memory_order_relaxed);
    uint32_t read=atomic_load_explicit(&arrival.consumed,memory_order_acquire);
    if(written-read>=PROBE_ARRIVAL_SLOTS){
        atomic_fetch_add_explicit(&arrival.lost,1,memory_order_relaxed);
        invalidate(2);return false;
    }
    arrival.slots[written&(PROBE_ARRIVAL_SLOTS-1u)]=(probe_arrival_sample_t){cursor,at,size,dbt_read_le16(data+1)};
    atomic_store_explicit(&arrival.published,written+1u,memory_order_release);
    return true;
}
void probe_arrival_end(uint32_t cursor,uint16_t bytes,dbt_result_t result,uint32_t written){
    if(result!=DBT_OK || written!=cursor+(uint32_t)bytes+2u)invalidate(3);
}
bool probe_arrival_peek(uint32_t end,uint16_t flags,uint16_t bytes,probe_arrival_sample_t *out){
    if(!out)return false;
    *out=(probe_arrival_sample_t){0};
    if(atomic_load_explicit(&arrival.status,memory_order_acquire))return false;
    uint32_t read=atomic_load_explicit(&arrival.consumed,memory_order_relaxed);
    uint32_t written=atomic_load_explicit(&arrival.published,memory_order_acquire);
    uint32_t count=written-read;
    if(!count || count>PROBE_ARRIVAL_SLOTS)return false;
    const probe_arrival_sample_t *s=&arrival.slots[read&(PROBE_ARRIVAL_SLOTS-1u)];
    if(s->queue_cursor+(uint32_t)s->wire_bytes+2u!=end || s->wire_bytes!=bytes || s->acl_flags!=flags)return false;
    *out=*s;return true;
}
static uint32_t be32(const uint8_t *p){
    return ((uint32_t)p[0]<<24)|((uint32_t)p[1]<<16)|((uint32_t)p[2]<<8)|p[3];
}
bool probe_arrival_media(uint32_t end,const uint8_t *h4,uint16_t bytes,const dbt_avdtp_media_t *media,probe_arrival_sample_t *out){
    if(out)*out=(probe_arrival_sample_t){0};
    if(!h4 || !media || !out || !media->payload || !media->size || media->payload_type>127u ||
       bytes<21u || bytes>DBT_MAX_H4_BYTES || h4[0]!=DBT_H4_ACL || dbt_read_le16(h4+3)!=(uint16_t)(bytes-5u))return false;
    uint16_t flags=dbt_read_le16(h4+1);unsigned pb=(flags>>12)&3u;
    if((pb!=0u && pb!=2u) || dbt_read_le16(h4+5)!=(uint16_t)(bytes-9u) || (h4[9]>>6)!=2u ||
       (h4[10]&127u)!=media->payload_type || ((h4[10]&128u)!=0)!=media->marker ||
       (((uint16_t)h4[11]<<8)|h4[12])!=media->sequence || be32(h4+13)!=media->timestamp || be32(h4+17)!=media->ssrc)return false;
    uintptr_t first=(uintptr_t)(h4+21),last=(uintptr_t)(h4+bytes),payload=(uintptr_t)media->payload;
    if(payload<first || payload>last || media->size>last-payload)return false;
    return probe_arrival_peek(end,flags,bytes,out);
}
void probe_arrival_retire(uint32_t end,uint16_t flags,uint16_t bytes){
    if(atomic_load_explicit(&arrival.status,memory_order_acquire))return;
    probe_arrival_sample_t sample;
    if(!probe_arrival_peek(end,flags,bytes,&sample)){invalidate(4);return;}
    uint32_t read=atomic_load_explicit(&arrival.consumed,memory_order_relaxed);
    atomic_store_explicit(&arrival.consumed,read+1u,memory_order_release);
}
void probe_arrival_stats(probe_arrival_stats_t *out){
    if(!out)return;
    out->published=atomic_load_explicit(&arrival.published,memory_order_acquire);
    out->consumed=atomic_load_explicit(&arrival.consumed,memory_order_acquire);
    out->lost=atomic_load_explicit(&arrival.lost,memory_order_relaxed);
    out->status=atomic_load_explicit(&arrival.status,memory_order_acquire);
    out->bytes=(uint32_t)sizeof(arrival);
}

uint32_t ARRIVAL_IRAM probe_arrival_pending(void){
    if(atomic_load_explicit(&arrival.status,memory_order_acquire))return UINT32_MAX;
    uint32_t read=atomic_load_explicit(&arrival.consumed,memory_order_acquire);
    uint32_t written=atomic_load_explicit(&arrival.published,memory_order_acquire);
    uint32_t after=atomic_load_explicit(&arrival.consumed,memory_order_acquire);
    uint32_t count=written-read;
    if(read!=after || count>PROBE_ARRIVAL_SLOTS || atomic_load_explicit(&arrival.status,memory_order_acquire))return UINT32_MAX;
    return count;
}
