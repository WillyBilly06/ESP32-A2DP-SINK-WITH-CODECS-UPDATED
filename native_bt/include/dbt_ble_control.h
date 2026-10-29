#ifndef DBT_BLE_CONTROL_H
#define DBT_BLE_CONTROL_H
#include "dbt_acl.h"
#define DBT_BLE_CONTROL_MAX_COMMAND 512u
#define DBT_BLE_CONTROL_MAX_STATUS 82u
typedef struct {
    int8_t eq[3]; uint8_t control,led[10],sound;
    uint8_t name[32],name_size,firmware[32],firmware_size;
    bool valid;
} dbt_ble_control_state_t;
typedef struct { uint16_t handle; uint32_t generation; } dbt_ble_control_peer_t;
typedef struct {
    void *context;
    /* Host owner only: bounded and nonblocking policy for mutating commands. */
    bool (*authorize)(void *,const dbt_link_t *,uint8_t);
    /* Worker only. Success means this exact tuple was committed immediately. */
    bool (*commit_eq)(void *,int8_t,int8_t,int8_t);
    bool (*commit_control)(void *,uint8_t);
    /* Bounded/nonblocking; normally on worker, and on host owner for explicit
     * publish-only retries. Preserve independent volume and control state. */
    bool (*publish_eq)(void *,int8_t,int8_t,int8_t);
    bool (*publish_control)(void *,uint8_t);
    /* Worker only. The provider owns name/LED/sound/OTA semantics. A success
     * response must reflect a completed operation, never queue admission.
     * Providers may run concurrently with EQ and must synchronize shared state. */
    bool (*command)(void *,const dbt_ble_control_peer_t *,uint8_t,const uint8_t *,uint16_t,uint8_t *,uint8_t *);
} dbt_ble_control_hooks_t;
typedef struct { uint8_t data[DBT_BLE_CONTROL_MAX_STATUS],size; bool durable_eq,durable_control; } dbt_ble_control_result_t;
/* Worker-owned state; process exactly one command, never coalesce EQ commits. */
dbt_result_t dbt_ble_control_execute(dbt_ble_control_state_t *,const dbt_ble_control_hooks_t *,
                                     const dbt_ble_control_peer_t *,const uint8_t *,uint16_t,dbt_ble_control_result_t *);
dbt_result_t dbt_ble_control_status(const dbt_ble_control_state_t *,uint8_t *,uint8_t *);
#endif
