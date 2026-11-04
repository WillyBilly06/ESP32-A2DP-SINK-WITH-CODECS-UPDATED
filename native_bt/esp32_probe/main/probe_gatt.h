#ifndef PROBE_GATT_H
#define PROBE_GATT_H
#include "dbt_gap.h"
#include "dbt_ble_control.h"
#include "dbt_smp_session.h"

/* Configure both before init when SMP is present. Committed-bond writes are
 * rejected until the worker-loaded, incarnation-bound state is attached. */
dbt_result_t probe_gatt_security_query_configure(
    dbt_result_t (*query)(uint16_t,uint32_t,dbt_smp_session_security_t *));
dbt_result_t probe_gatt_bond_query_configure(
    dbt_result_t (*query)(uint16_t,uint32_t,uint8_t *,uint8_t[6],uint8_t[32]));

/* Configure before init. Worker callbacks own persistence/provider work;
 * publication callbacks must be nonblocking and preserve independent volume. */
dbt_result_t probe_gatt_control_configure(const dbt_ble_control_state_t *,const dbt_ble_control_hooks_t *);
/* Host owner only; best-effort 3-byte baseline meter notification. */
void probe_gatt_meter(uint8_t,uint8_t,uint8_t);
/* Task context only (including render task); nonblocking persistent worker wake. */
void probe_gatt_control_wake(void);
/* Owner: external provider demand can start a worker without any BLE command. */
dbt_result_t probe_gatt_control_worker_required(bool);
/* Owner: admit one encoder detent through the same immediate-commit EQ lane.
 * BUSY means retain the event; reaching +/-12 is an accepted no-op. */
dbt_result_t probe_gatt_control_eq_delta(uint8_t band,int8_t delta);
/* Owner: one persisted toggle of control bit 1/2/4, sharing remote-write order. */
dbt_result_t probe_gatt_control_toggle(uint8_t flag);
/* Owner only; completed response copied into bounded, generation-bound status queue. */
dbt_result_t probe_gatt_control_notify(const dbt_ble_control_peer_t *,const uint8_t *,uint8_t);
dbt_result_t probe_gatt_control_state(dbt_ble_control_state_t *);
/* Owner only, after durable/provider receipt; LED bytes use the BLE wire order. */
dbt_result_t probe_gatt_control_led_applied(const uint8_t[10]);
dbt_result_t probe_gatt_control_sound_applied(uint8_t);
/* Owner only, after correlated successful GAP name/scan-response completion. */
dbt_result_t probe_gatt_control_name_applied(const uint8_t *,uint8_t);
/* Owner only; retries publication of the latest committed EQ without writing
 * NVS again. DBT_EMPTY means no retry is pending. */
dbt_result_t probe_gatt_retry_eq_publication(void);
/* Same publish-only recovery for a committed control-flags change. */
dbt_result_t probe_gatt_retry_control_publication(void);

dbt_result_t probe_gatt_init(dbt_runtime_t *runtime, dbt_gap_t *gap);
void probe_gatt_advertisement(dbt_gap_config_t *configuration);
void probe_gatt_link(void *context, const dbt_link_change_t *change);
void probe_gatt_receive(void *context, const dbt_link_t *link, uint16_t cid, const uint8_t *data,
                        uint16_t size);
void probe_gatt_poll(uint32_t now_ms);
uint32_t probe_gatt_wait_ms(uint32_t now_ms);
bool probe_gatt_connected(void);
unsigned probe_gatt_bytes(void);
#endif
