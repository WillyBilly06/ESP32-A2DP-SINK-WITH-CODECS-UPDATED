#ifndef PROBE_AUDIO_TRACE_H
#define PROBE_AUDIO_TRACE_H
#include "dbt_codec.h"
#include "probe_host_record.h"
#include <stdatomic.h>
#define PROBE_AUDIO_TRACE_SLOTS 16u
#define PROBE_AUDIO_TRACE_HOST UINT32_MAX
_Static_assert(PROBE_AUDIO_TRACE_SLOTS && !(PROBE_AUDIO_TRACE_SLOTS&(PROBE_AUDIO_TRACE_SLOTS-1u)),"Trace capacity must be a power of two");
typedef struct {
    union {dbt_codec_trace_t event;probe_host_record_t host;};
    uint32_t at_us,ordinal,codec;
} probe_audio_trace_t;
typedef struct {
    probe_audio_trace_t items[PROBE_AUDIO_TRACE_SLOTS];
    _Atomic uint32_t written,read,lost;
    uint32_t ordinal;
} probe_audio_trace_ring_t;
typedef uint32_t (*probe_audio_trace_clock_t)(void *);
typedef struct {dbt_codec_trace_t event;uint32_t at_us,ordinal,codec;} probe_audio_trace_legacy_t;
_Static_assert(sizeof(probe_host_record_t)<=sizeof(dbt_codec_trace_t),"Host record must reuse existing trace storage");
_Static_assert(sizeof(probe_audio_trace_t)==sizeof(probe_audio_trace_legacy_t),"Trace slot size must not grow");
/* Before concurrent use only. Each ring has one producer and one consumer. */
void probe_audio_trace_init(probe_audio_trace_ring_t *);
/* Bounded copies with release publication. Full queues count one lost record;
 * they do not call the clock, wait, allocate or consume the caller's object. */
bool probe_audio_trace_media(probe_audio_trace_ring_t *,const dbt_codec_trace_t *,uint32_t codec,probe_audio_trace_clock_t,void *);
dbt_result_t probe_audio_trace_host(probe_audio_trace_ring_t *,const probe_host_record_t *,probe_audio_trace_clock_t,void *);
/* Copy before release of the slot; UART work belongs after this returns. */
bool probe_audio_trace_pop(probe_audio_trace_ring_t *,probe_audio_trace_t *);
#endif
