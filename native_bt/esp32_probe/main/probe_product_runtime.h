#ifndef PROBE_PRODUCT_RUNTIME_H
#define PROBE_PRODUCT_RUNTIME_H
#include "probe_product.h"
#include "dbt_gap.h"
#include "dbt_smp_session.h"
typedef struct {
    probe_product_t *product;dbt_runtime_t *runtime;dbt_gap_t *gap;
    dbt_result_t (*security)(uint16_t,uint32_t,dbt_smp_session_security_t *);
    dbt_product_peer_value_t current;dbt_ble_control_peer_t granted_peer;
    dbt_ota_manifest_t grant;bool have_current,ota_granted;
    probe_product_name_t name;uint32_t rename_token,retry_at;bool have_name,retry_pending;
} probe_product_runtime_t;
bool probe_product_runtime_init(probe_product_runtime_t *,probe_product_t *,dbt_runtime_t *,dbt_gap_t *,
                                dbt_result_t (*security)(uint16_t,uint32_t,dbt_smp_session_security_t *));
/* Owner-only, called after SMP and GAP polls. */
void probe_product_runtime_poll(probe_product_runtime_t *);
/* Owner-only bounded refresh, safe before dispatching a fixed-CID command. */
void probe_product_runtime_refresh_security(probe_product_runtime_t *);
uint32_t probe_product_runtime_wait_ms(const probe_product_runtime_t *,uint32_t now_ms);
/* Owner's explicit, generation-bound authorization; NULL means baseline
 * grant with no independently trusted digest. No selector write. */
dbt_result_t probe_product_runtime_grant_ota(probe_product_runtime_t *,const dbt_ble_control_peer_t *,const dbt_ota_manifest_t *);
dbt_result_t probe_product_runtime_request_activation(probe_product_runtime_t *,const dbt_ble_control_peer_t *);
probe_product_action_result_t probe_product_runtime_take_activation(probe_product_runtime_t *);
/* Main-owned entry points: caller must be the host owner and use a genuine
 * explicit local authorization decision bound to the current peer. */
dbt_result_t probe_application_grant_ota(uint16_t,uint32_t,const dbt_ota_manifest_t *);
dbt_result_t probe_application_activate_ota(uint16_t,uint32_t);
probe_product_action_result_t probe_application_take_ota_result(void);
typedef bool (*probe_application_other_command_t)(void *,const dbt_ble_control_peer_t *,uint8_t,const uint8_t *,uint16_t,uint8_t *,uint8_t *);
typedef bool (*probe_application_audio_busy_t)(void *);
/* Owner-only and before the startup worker begins. */
bool probe_application_set_other_provider(probe_application_other_command_t,probe_application_audio_busy_t,void *);
#endif
