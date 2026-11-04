#ifndef PROBE_AUDIO_H
#define PROBE_AUDIO_H
#include "dbt_a2dp_acceptor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
/* Host-owner control/data hooks; decoder and PCM work run on the separate
 * generic codec worker. Optional Opus, AAC and aptX endpoints are present only
 * when their corresponding DBT_PROBE_*_BUILD components are enabled. */
dbt_result_t probe_audio_init(dbt_runtime_t *runtime,TaskHandle_t owner);
bool probe_audio_accept(void *context,const dbt_link_t *link,uint16_t psm);
void probe_audio_event(dbt_channel_event_t event,const dbt_channel_t *channel);
void probe_audio_receive(const dbt_channel_t *channel,const uint8_t *data,uint16_t size);
void probe_audio_poll(uint32_t now_ms);
uint32_t probe_audio_wait_ms(uint32_t now_ms);
/* Host owner only. Snapshot the Classic ACL underlying the active AVDTP
 * signaling session, even when playback is paused. EMPTY means no live peer. */
dbt_result_t probe_audio_peer(uint16_t *handle,uint32_t *generation);
/* Resolve the captured identity again immediately before enqueueing work. */
const dbt_link_t *probe_audio_peer_link(uint16_t handle,uint32_t generation);
typedef enum { PROBE_AUDIO_CONNECTED,PROBE_AUDIO_DISCONNECTED,
               PROBE_AUDIO_CONFIGURED,PROBE_AUDIO_STREAMING } probe_audio_product_event_t;
/* Owner-only synchronous, copied event; no storage or playback in callback. */
typedef void (*probe_audio_product_event_fn)(void *,probe_audio_product_event_t,
                                             uint16_t,uint32_t,uint32_t,uint32_t);
void probe_audio_product_observer(probe_audio_product_event_fn,void *);
bool probe_audio_sound_memory_ok(void);
/* Bounded scalar diagnostics only; receive may run in controller/ISR context.
 * first_rx_to_poll_us is a wake/service indicator, not per-packet latency. */
void probe_audio_transport_received(uint32_t began_us,uint16_t bytes,uint8_t type,dbt_result_t result,bool isr);
void probe_audio_transport_poll_begin(uint32_t began_us);
void probe_audio_transport_poll_end(uint32_t began_us);
/* Before controller startup, initialize the shared output exactly once. */
dbt_result_t probe_audio_output_bootstrap(void);
/* Owner: stop new sessions and drain existing channels/worker loans. Keep
 * normal event/receipt/poll forwarding until quiescent; cancel reopens admission. */
dbt_result_t probe_audio_restart_hold(bool,uint32_t now_ms);
bool probe_audio_restart_quiescent(void);
#endif
