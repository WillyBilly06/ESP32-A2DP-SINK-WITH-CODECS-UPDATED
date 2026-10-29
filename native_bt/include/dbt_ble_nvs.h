#ifndef DBT_BLE_NVS_H
#define DBT_BLE_NVS_H
#include <stdbool.h>
#include <stdint.h>
#include "dbt_ble_control.h"
/* Worker only. Uses existing baseline namespace/keys; no erase or migration. */
bool dbt_ble_nvs_commit_eq(void *,int8_t,int8_t,int8_t);
bool dbt_ble_nvs_commit_control(void *,uint8_t);
/* Worker only, read-only. Caller supplies defaults, firmware and actual sound
 * presence bits; success atomically updates the state and sets valid. Missing
 * namespaces/keys retain caller defaults. No erase, write or migration. */
dbt_result_t dbt_ble_nvs_load_state(dbt_ble_control_state_t *);
#endif
