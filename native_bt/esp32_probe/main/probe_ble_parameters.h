#ifndef PROBE_BLE_PARAMETERS_H
#define PROBE_BLE_PARAMETERS_H
#include "dbt_runtime.h"

/* Same product policy as the released Bluedroid GATT server. */
#define PROBE_BLE_INTERVAL_MIN 18u
#define PROBE_BLE_INTERVAL_MAX 24u
#define PROBE_BLE_SUPERVISION 400u
typedef struct {
    uint32_t generation,at_ms;
    uint16_t handle,interval,latency,timeout,opcode,l2cap_result;
    uint8_t stage,status,outcome,role,local_feature,peer_feature,retries,local_result;
} probe_ble_parameters_record_t;
/* All APIs run on the one host owner. No task, allocation, spin or blocking
 * command submission. Controller admission and actual update are distinct. */
void probe_ble_parameters_link(dbt_runtime_t *,const dbt_link_change_t *);
void probe_ble_parameters_event(dbt_runtime_t *,const dbt_hci_packet_t *);
void probe_ble_parameters_poll(dbt_runtime_t *,uint32_t now_ms,bool hold);
uint32_t probe_ble_parameters_wait_ms(uint32_t now_ms);
/* Nonblocking metadata sink, implemented by the existing protocol logger. */
void probe_ble_parameters_log(const probe_ble_parameters_record_t *);
#endif
