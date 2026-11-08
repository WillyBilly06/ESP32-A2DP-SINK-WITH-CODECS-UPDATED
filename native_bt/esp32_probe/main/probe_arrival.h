#ifndef PROBE_ARRIVAL_H
#define PROBE_ARRIVAL_H
#include "dbt_hci.h"
#include "dbt_avdtp.h"
#include <stdbool.h>
#include <stdint.h>
#define PROBE_ARRIVAL_SLOTS 8u
typedef struct {
    uint32_t queue_cursor,arrival_us;
    uint16_t wire_bytes,acl_flags;
} probe_arrival_sample_t;
typedef struct {uint32_t published,consumed,lost,status,bytes;} probe_arrival_stats_t;
/* Diagnostic SPSC metadata only: controller producer, host consumer. Initialize
 * before callbacks. No payload copies, allocation, clock reads or waits. */
void probe_arrival_init(void);
/* Publish BEFORE H4 queue publication. Cursor is its actual producer cursor. */
bool probe_arrival_begin(uint32_t,uint32_t,const uint8_t *,uint16_t);
/* After actual receive: rejection/non-enqueue invalidates timing only. */
void probe_arrival_end(uint32_t,uint16_t,dbt_result_t,uint32_t);
/* Host only during current popped ACL dispatch. Nonmutating lookup; deferred
 * delivery can have no current match. end is the actual post-pop read cursor. */
bool probe_arrival_peek(uint32_t,uint16_t,uint16_t,probe_arrival_sample_t *);
/* Host after synchronous ACL callbacks return. No resynchronization guesses. */
void probe_arrival_retire(uint32_t,uint16_t,uint16_t);
/* Correlate only a complete current Basic L2CAP/RTP packet whose actual media
 * view is inside this H4 buffer. Fragmented/deferred/raw media is unavailable. */
bool probe_arrival_media(uint32_t,const uint8_t *,uint16_t,const dbt_avdtp_media_t *,probe_arrival_sample_t *);
/* Independent atomic counts, not a simultaneous snapshot. */
void probe_arrival_stats(probe_arrival_stats_t *);
/* Bounded IRAM-safe count of published, unretired ACL metadata. UINT32_MAX
 * means diagnostic failure or a concurrent retirement, never guessed zero. */
uint32_t probe_arrival_pending(void);
#endif
