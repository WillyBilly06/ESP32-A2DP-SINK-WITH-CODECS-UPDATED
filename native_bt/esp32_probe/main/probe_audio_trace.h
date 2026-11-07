#ifndef PROBE_AUDIO_TRACE_H
#define PROBE_AUDIO_TRACE_H
#include "dbt_codec.h"
#include "probe_host_record.h"
#include <stdatomic.h>
#define PROBE_AUDIO_TRACE_SLOTS 16u
#define PROBE_AUDIO_TRACE_HOST UINT32_MAX
#define PROBE_AUDIO_TRACE_FAILURE (UINT32_MAX-1u)
#define PROBE_AUDIO_TRACE_CONTINUATION (UINT32_MAX-2u)
#define PROBE_AUDIO_TRACE_CORRUPT (UINT32_MAX-3u)
#define PROBE_AUDIO_TRACE_FAILURE_PARTS 3u
#define PROBE_AUDIO_TRACE_PART_WORDS 8u
_Static_assert(PROBE_AUDIO_TRACE_SLOTS && !(PROBE_AUDIO_TRACE_SLOTS&(PROBE_AUDIO_TRACE_SLOTS-1u)),"Trace capacity must be a power of two");
typedef struct {
    union {dbt_codec_trace_t event;probe_host_record_t host;uint32_t failure_words[PROBE_AUDIO_TRACE_PART_WORDS];};
    uint32_t at_us,ordinal,codec;
} probe_audio_trace_t;
/* Consumer-local copy only. No new queue/global storage is allocated. */
typedef struct {
    union {dbt_codec_trace_t event;probe_host_record_t host;probe_host_failure_t failure;};
    uint32_t at_us,ordinal,codec;
} probe_audio_trace_record_t;
typedef struct {
    probe_audio_trace_t items[PROBE_AUDIO_TRACE_SLOTS];
    _Atomic uint32_t written,read,lost;
    uint32_t ordinal;
} probe_audio_trace_ring_t;
typedef uint32_t (*probe_audio_trace_clock_t)(void *);
typedef struct {dbt_codec_trace_t event;uint32_t at_us,ordinal,codec;} probe_audio_trace_legacy_t;
_Static_assert(sizeof(probe_host_record_t)<=sizeof(dbt_codec_trace_t),"Host record must reuse existing trace storage");
_Static_assert(sizeof(probe_audio_trace_t)==sizeof(probe_audio_trace_legacy_t),"Trace slot size must not grow");
_Static_assert(PROBE_AUDIO_TRACE_FAILURE_PARTS && PROBE_AUDIO_TRACE_FAILURE_PARTS<=PROBE_AUDIO_TRACE_SLOTS,"Failure group must fit the ring");
_Static_assert(sizeof(probe_host_failure_t)==PROBE_AUDIO_TRACE_FAILURE_PARTS*PROBE_AUDIO_TRACE_PART_WORDS*sizeof(uint32_t),"Failure snapshot must fit three existing slots");
/* Before concurrent use only. Each ring has one producer and one consumer. */
void probe_audio_trace_init(probe_audio_trace_ring_t *);
/* Bounded copies with release publication. Full queues count one lost logical
 * record, call no clock, and neither wait, allocate nor borrow caller storage.
 * The clock hook must be a bounded read and must not reenter this producer. */
bool probe_audio_trace_media(probe_audio_trace_ring_t *,const dbt_codec_trace_t *,uint32_t codec,probe_audio_trace_clock_t,void *);
dbt_result_t probe_audio_trace_host(probe_audio_trace_ring_t *,const probe_host_record_t *,probe_audio_trace_clock_t,void *);
dbt_result_t probe_audio_trace_failure(probe_audio_trace_ring_t *,const probe_host_failure_t *,probe_audio_trace_clock_t,void *);
/* Complete local copy precedes slot release. Incomplete/malformed private
 * groups produce CORRUPT; the background logger owns that diagnostic count. */
bool probe_audio_trace_pop(probe_audio_trace_ring_t *,probe_audio_trace_record_t *);
#endif
