#ifndef PROBE_GATT_H
#define PROBE_GATT_H
#include "dbt_gap.h"
#include "dbt_ble_control.h"

/* Configure before init. Worker callbacks own persistence/provider work;
 * publication callbacks must be nonblocking and preserve independent volume. */
dbt_result_t probe_gatt_control_configure(const dbt_ble_control_state_t *,const dbt_ble_control_hooks_t *);
/* Host owner only; best-effort 3-byte baseline meter notification. */
void probe_gatt_meter(uint8_t,uint8_t,uint8_t);
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
