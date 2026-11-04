#ifndef PROBE_RFCOMM_H
#define PROBE_RFCOMM_H
#include "dbt_rfcomm_spp.h"
/* Optional encrypted single-session serial echo. No PSM/record exists before
 * explicit init. The owner forwards composite accept, event and data callbacks,
 * calls poll outside runtime callbacks, and keeps the wrapper alive throughout
 * runtime lifetime. No RFCOMM payload is logged. */
dbt_result_t probe_rfcomm_init(dbt_runtime_t *runtime,uint8_t server_channel,uint32_t record_handle);
bool probe_rfcomm_accept(const dbt_link_t *link,uint16_t psm);
void probe_rfcomm_event(dbt_channel_event_t event,const dbt_channel_t *channel);
dbt_result_t probe_rfcomm_data(const dbt_channel_t *channel,const uint8_t *data,uint16_t size);
void probe_rfcomm_poll(uint32_t now_ms);
uint32_t probe_rfcomm_wait_ms(uint32_t now_ms);
/* Borrowed record includes the label Native RFCOMM Echo; add it to the existing
 * SDP database only after init and composite hook routing are installed. Keep
 * that database/record alive together; this function does not publish it. */
const dbt_sdp_record_t *probe_rfcomm_record(void);
/* Restart barrier: reject new RFCOMM sessions and echo submissions; retain
 * callbacks, drain an admitted TX, close the exact RFCOMM channels, then report
 * quiescence. Cancel reopens admission; a close already sent still completes. */
void probe_rfcomm_restart_begin(void);
bool probe_rfcomm_restart_quiescent(void);
void probe_rfcomm_restart_cancel(void);
#endif
