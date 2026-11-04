#ifndef PROBE_PRODUCT_H
#define PROBE_PRODUCT_H
#include "dbt_product_peer.h"
#include "dbt_ota_esp.h"

typedef struct {
    void *context;
    uint32_t (*now_ms)(void *);
    void (*wake_owner)(void *);
    void (*wake_worker)(void *);
    /* Optional synchronous worker provider for LED and sound. */
    bool (*other_command)(void *,const dbt_ble_control_peer_t *,uint8_t,const uint8_t *,uint16_t,uint8_t *,uint8_t *);
    uint32_t ota_timeout_ms,name_timeout_ms;
} probe_product_hooks_t;
typedef struct {
    dbt_product_peer_t peer;dbt_ota_t ota;dbt_ota_esp_t esp;probe_product_hooks_t hooks;
    _Atomic uint32_t name_phase,name_token,name_handle,name_generation,name_length,name_words[8];
    _Atomic uint32_t action_phase,action_handle,action_generation;
    uint32_t next_name_token;bool ready;
} probe_product_t;
typedef struct {uint32_t token,generation;uint16_t handle;uint8_t length;char name[33];} probe_product_name_t;
typedef enum {PROBE_PRODUCT_NO_RESULT,PROBE_PRODUCT_SELECTED,PROBE_PRODUCT_REJECTED,PROBE_PRODUCT_SELECTION_AMBIGUOUS} probe_product_action_result_t;
/* Startup worker only: read existing NVS and select the other explicit owned
 * OTA slot. No writes, GAP calls, erase or selector change. */
bool probe_product_startup_worker(probe_product_t *,dbt_ble_control_state_t *,const probe_product_hooks_t *);
/* Host owner only: publish verified security/grant state; generation 0 revokes. */
bool probe_product_publish_peer(probe_product_t *,const dbt_product_peer_value_t *);
/* Generic control worker callback, or route to it from an application mux. */
bool probe_product_command(void *,const dbt_ble_control_peer_t *,uint8_t,const uint8_t *,uint16_t,uint8_t *,uint8_t *);
/* The same worker calls tick on wake and periodically, including while idle. */
void probe_product_worker_tick(probe_product_t *);
/* Suitable for the GATT general-worker poll hook; false means do not retire. */
bool probe_product_worker_poll(void *,bool retiring);
/* As above, but defer explicit activation when another owner-managed audio
 * operation cannot tolerate blocking flash work. */
bool probe_product_worker_poll_guarded(void *,bool retiring,bool allow_activation);
/* Suitable for the GATT owner-side peer-lost hook; bounded and nonblocking. */
void probe_product_peer_lost(void *,const dbt_ble_control_peer_t *);
/* Worker only. */
bool probe_product_worker_quiescent(const probe_product_t *);
/* Worker only; retry cleanup of a faulted backend before retiring. */
bool probe_product_worker_retire(probe_product_t *);
/* Owner takes the durable name and applies GAP/advertising asynchronously.
 * Completion is true only after the actual owner/controller operation succeeds. */
bool probe_product_take_name(probe_product_t *,probe_product_name_t *);
bool probe_product_complete_name(probe_product_t *,uint32_t token,bool applied);
/* Explicit user/owner action only; scheduling does not select or restart. */
bool probe_product_request_activation(probe_product_t *,const dbt_ble_control_peer_t *);
probe_product_action_result_t probe_product_take_activation_result(probe_product_t *);
#endif
