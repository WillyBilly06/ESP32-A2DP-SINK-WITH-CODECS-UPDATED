#ifndef PROBE_SDP_H
#define PROBE_SDP_H
#include "dbt_sdp_bearer.h"
/* Enable only after the corresponding owned profile has initialized. */
dbt_result_t probe_sdp_enable_avrcp(bool controller, bool target);
dbt_result_t probe_sdp_init(dbt_runtime_t *runtime);
dbt_result_t probe_sdp_receive(const dbt_channel_t *channel, const uint8_t *data, uint16_t size);
void probe_sdp_event(dbt_channel_event_t event, const dbt_channel_t *channel);
dbt_result_t probe_sdp_poll(void);
uint32_t probe_sdp_wait_ms(void);
unsigned probe_sdp_bytes(void);
unsigned probe_sdp_records(void);
bool probe_sdp_audio_sink(void);
#endif
