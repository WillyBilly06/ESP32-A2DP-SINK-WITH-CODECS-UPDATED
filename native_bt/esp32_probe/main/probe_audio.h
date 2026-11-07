#ifndef PROBE_AUDIO_H
#define PROBE_AUDIO_H
#include "probe_host_record.h"
#include "dbt_a2dp_acceptor.h"
#include "dbt_a2dp_connect.h"
#include "dbt_avdtp_query.h"
#include "dbt_a2dp_select.h"
#include "dbt_a2dp_generate.h"
#include "dbt_a2dp_rank.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
/* Host-owner control/data hooks; decoder and PCM work run on the separate
 * generic codec worker. Optional Opus, AAC and aptX endpoints are present only
 * when their corresponding DBT_PROBE_*_BUILD components are enabled. */
/* Host-owner RX-credit policy: window reserves enough encoded slots for all
 * already authorized ACL packets. BLE completions are always eligible. */
/* Host-owner only. Bounded copy into existing diagnostics; never waits for UART.
 * NO_SPACE is an explicitly counted diagnostic loss, not protocol backpressure. */
dbt_result_t probe_audio_host_record(const probe_host_record_t *);
/* All-or-nothing immutable failure snapshot in three existing trace slots. */
dbt_result_t probe_audio_host_failure(const probe_host_failure_t *);
/* Copied controller properties; background logger collects heap values later. */
dbt_result_t probe_audio_host_controller(const probe_host_controller_t *);
uint16_t probe_audio_rx_credit_window(void);
bool probe_audio_rx_credit_ready(void *,const dbt_link_t *);
dbt_result_t probe_audio_init(dbt_runtime_t *runtime,TaskHandle_t owner);
bool probe_audio_accept(void *context,const dbt_link_t *link,uint16_t psm);
/* Host-owner outgoing signaling entry points. No peer is automatically
 * selected/paged and no codec procedure is initiated by these APIs. */
dbt_result_t probe_audio_connect_begin(const dbt_link_t *,dbt_a2dp_channel_t *);
dbt_result_t probe_audio_connect_status(const dbt_a2dp_channel_t *);
dbt_result_t probe_audio_connect_cancel(const dbt_a2dp_channel_t *);
/* Explicit local configuration and stream controls on exact channel bindings. Shared
 * caller context remains borrowed through matching terminal local_result. */
dbt_result_t probe_audio_configure_begin(const dbt_a2dp_channel_t *,dbt_avdtp_local_t *,
    uint8_t local_seid,uint8_t remote_seid,const uint8_t *,uint16_t,uint32_t *);
dbt_result_t probe_audio_configure_cancel(dbt_avdtp_local_t *,uint32_t);
dbt_result_t probe_audio_configure_result(const dbt_avdtp_local_t *,uint32_t,dbt_avdtp_local_result_t *);
dbt_result_t probe_audio_open_begin(const dbt_a2dp_channel_t *,dbt_avdtp_local_t *,uint32_t *);
dbt_result_t probe_audio_start_begin(const dbt_a2dp_channel_t *signaling,const dbt_a2dp_channel_t *media,
    dbt_avdtp_local_t *,uint32_t *);
dbt_result_t probe_audio_suspend_begin(const dbt_a2dp_channel_t *signaling,const dbt_a2dp_channel_t *media,
    dbt_avdtp_local_t *,uint32_t *);
dbt_result_t probe_audio_close_begin(const dbt_a2dp_channel_t *signaling,const dbt_a2dp_channel_t *media,
    dbt_avdtp_local_t *,uint32_t *);
dbt_result_t probe_audio_reconfigure_begin(const dbt_a2dp_channel_t *signaling,const dbt_a2dp_channel_t *media,
    dbt_avdtp_local_t *,const uint8_t *,uint16_t,uint32_t *);
dbt_result_t probe_audio_abort_begin(const dbt_a2dp_channel_t *signaling,dbt_avdtp_local_t *,uint32_t *);
dbt_result_t probe_audio_local_cancel(dbt_avdtp_local_t *,uint32_t);
dbt_result_t probe_audio_local_result(const dbt_avdtp_local_t *,uint32_t,dbt_avdtp_local_result_t *);
/* Media follows local Open acceptance. Keys are copied; no audio start/default caller. */
dbt_result_t probe_audio_media_begin(const dbt_a2dp_channel_t *signaling,dbt_a2dp_channel_t *media_key);
dbt_result_t probe_audio_media_status(const dbt_a2dp_channel_t *);
dbt_result_t probe_audio_media_cancel(const dbt_a2dp_channel_t *);
/* Read-only AVDTP queries. Context/response stay caller-owned through terminal result. */
dbt_result_t probe_audio_query_begin(const dbt_a2dp_channel_t *,dbt_avdtp_query_t *,uint8_t,uint8_t,
    uint8_t *,uint16_t,uint32_t *);
dbt_result_t probe_audio_query_cancel(dbt_avdtp_query_t *,uint32_t);
dbt_result_t probe_audio_query_result(const dbt_avdtp_query_t *,uint32_t,dbt_avdtp_query_result_t *,const uint8_t **);
/* Owner coordinator: init caller storage once; PENDING borrows context/buffers
 * through terminal dial_result, which also detaches SDP routing. No automatic
 * peer selection, endpoint negotiation or audio start. */
/* Exact-format proposal from terminal Discover + Get[All]Capabilities query
 * objects on this signaling lifetime. Inputs stay disjoint/caller-owned; no
 * reservation, decoder activation, Android preference write or RF command. */
dbt_result_t probe_audio_match(const dbt_a2dp_channel_t *,const dbt_avdtp_query_t *discover,uint32_t discover_token,
    const dbt_avdtp_query_t *capabilities,uint32_t capabilities_token,const dbt_a2dp_select_request_t *,
    uint8_t *configuration,uint16_t capacity,dbt_a2dp_select_result_t *);
/* Generated strict family/format proposal using the same current query fences. */
dbt_result_t probe_audio_generate(const dbt_a2dp_channel_t *,const dbt_avdtp_query_t *,uint32_t,
    const dbt_avdtp_query_t *,uint32_t,const dbt_a2dp_generate_request_t *,uint8_t *,uint16_t,dbt_a2dp_generate_result_t *);
/* Explicit asynchronous ranking on an idle exact signaling session. PENDING
 * borrows context/inputs through terminal rank_result, which detaches owner
 * routing. The copied proposal does not configure or start a stream. */
dbt_result_t probe_audio_rank_begin(const dbt_a2dp_channel_t *,dbt_a2dp_rank_t *,dbt_a2dp_rank_config_t,uint32_t *);
dbt_result_t probe_audio_rank_cancel(dbt_a2dp_rank_t *,uint32_t);
dbt_result_t probe_audio_rank_result(dbt_a2dp_rank_t *,uint32_t,dbt_a2dp_rank_result_t *,const uint8_t **);
dbt_result_t probe_audio_dial_begin(dbt_a2dp_connect_t *,dbt_gap_t *,dbt_a2dp_connect_config_t,uint32_t *);
dbt_result_t probe_audio_dial_cancel(dbt_a2dp_connect_t *,uint32_t);
dbt_result_t probe_audio_dial_result(dbt_a2dp_connect_t *,uint32_t,dbt_a2dp_connect_result_t *);
void probe_audio_event(dbt_channel_event_t event,const dbt_channel_t *channel);
void probe_audio_event_ex(dbt_channel_event_t,const dbt_channel_t *,dbt_channel_reason_t,uint16_t);
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
/* Atomic scheduling hint only; no borrowed decoder or codec state. */
bool probe_audio_dsp_assist_ready(void);
bool probe_audio_owner_pressure(void);
/* Bounded scalar diagnostics only; receive may run in controller/ISR context.
 * first_rx_to_poll_us is a wake/service indicator, not per-packet latency. */
void probe_audio_transport_received(uint32_t began_us,uint16_t bytes,uint8_t type,dbt_result_t result,bool isr);
void probe_audio_transport_poll_begin(uint32_t began_us);
void probe_audio_transport_poll_end(uint32_t began_us);
void probe_audio_owner_wait(uint32_t immediate_mask);
void probe_audio_owner_ready_yield(void);
void probe_audio_owner_service_yield(void);
/* Before controller startup, initialize the shared output exactly once. */
dbt_result_t probe_audio_output_bootstrap(void);
/* Owner: stop new sessions and drain existing channels/worker loans. Keep
 * normal event/receipt/poll forwarding until quiescent; cancel reopens admission. */
dbt_result_t probe_audio_restart_hold(bool,uint32_t now_ms);
bool probe_audio_restart_quiescent(void);
#endif
