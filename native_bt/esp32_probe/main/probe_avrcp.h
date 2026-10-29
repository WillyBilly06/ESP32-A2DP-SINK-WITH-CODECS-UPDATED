#ifndef PROBE_AVRCP_H
#define PROBE_AVRCP_H
#include "dbt_avrcp_bearer.h"

#define PROBE_AVRCP_TEXT_BYTES 96u
/* The adapter is called only by the host owner and must return promptly. It
 * publishes a volume-only change to the audio worker without changing EQ. */
typedef dbt_result_t (*probe_avrcp_set_volume_fn)(void *, uint8_t);
typedef struct {
    uint32_t generation, track_length_ms, track_position_ms, metadata_updates;
    uint32_t connection_count, error_count;
    uint8_t connected, volume, playback_status, metadata_truncated;
    char title[PROBE_AVRCP_TEXT_BYTES];
    char artist[PROBE_AVRCP_TEXT_BYTES];
    char album[PROBE_AVRCP_TEXT_BYTES];
} probe_avrcp_snapshot_t;

/* init registers encrypted Classic PSM 0x17. The volume hook and the runtime
 * allocator are required. All functions except snapshot run on the host owner. */
dbt_result_t probe_avrcp_init(dbt_runtime_t *, probe_avrcp_set_volume_fn, void *, uint8_t initial_volume);
bool probe_avrcp_accept(void *, const dbt_link_t *, uint16_t psm);
void probe_avrcp_event(dbt_channel_event_t, const dbt_channel_t *);
void probe_avrcp_receive(const dbt_channel_t *, const uint8_t *, uint16_t);
void probe_avrcp_poll(uint32_t now_ms);
uint32_t probe_avrcp_wait_ms(uint32_t now_ms);
/* Called after an independently-originated volume change has been published to
 * audio. An AVRCP SetAbsoluteVolume is already handled internally and must not
 * be echoed back as a local change. */
void probe_avrcp_local_volume(uint8_t volume);
/* Send a single AV/C button press or release. Caller preserves button timing. */
dbt_result_t probe_avrcp_key(uint8_t operation, bool release, uint32_t now_ms);
/* Lock-free bounded reader for the logger. Strings are UTF-8 byte strings and
 * must be escaped by any formatter. DBT_BUSY asks the reader to try later. */
dbt_result_t probe_avrcp_snapshot(probe_avrcp_snapshot_t *);
bool probe_avrcp_ready(void);
#endif
