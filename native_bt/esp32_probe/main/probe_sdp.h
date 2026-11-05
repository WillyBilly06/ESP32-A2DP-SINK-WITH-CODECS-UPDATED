#ifndef PROBE_SDP_H
#define PROBE_SDP_H
#include "dbt_sdp_bearer.h"
#include "dbt_sdp_discovery.h"
/* Enable only after the corresponding owned profile has initialized. */
dbt_result_t probe_sdp_enable_avrcp(bool controller, bool target);
/* Before init, one explicitly initialized profile may supply an immutable
 * record borrowed for the full SDP server lifetime. */
dbt_result_t probe_sdp_enable_extra_record(const dbt_sdp_record_t *);

dbt_result_t probe_sdp_init(dbt_runtime_t *runtime);
dbt_result_t probe_sdp_receive(const dbt_channel_t *channel, const uint8_t *data, uint16_t size);
void probe_sdp_event(dbt_channel_event_t event, const dbt_channel_t *channel);
void probe_sdp_event_ex(dbt_channel_event_t,const dbt_channel_t *,dbt_channel_reason_t,uint16_t);
/* Initialize caller storage first. A registered operation owns routing until
 * result returns terminal OK and detaches it; only then release/reuse it. */
dbt_result_t probe_sdp_discover_audio_source(dbt_sdp_discovery_t *,const dbt_link_t *,
    dbt_sdp_discovery_config_t,uint32_t now_ms,uint32_t *token);
dbt_result_t probe_sdp_discover_cancel(uint32_t token);
dbt_result_t probe_sdp_discover_result(uint32_t token,dbt_sdp_discovery_result_t *);
dbt_result_t probe_sdp_poll(void);
uint32_t probe_sdp_wait_ms(void);
unsigned probe_sdp_bytes(void);
unsigned probe_sdp_records(void);
bool probe_sdp_audio_sink(void);
#endif
