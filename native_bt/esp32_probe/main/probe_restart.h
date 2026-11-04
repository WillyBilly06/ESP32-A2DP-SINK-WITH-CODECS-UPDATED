#ifndef PROBE_RESTART_H
#define PROBE_RESTART_H
#include "dbt_runtime.h"
#include "probe_product_io_owner.h"
/* Owner-only coordinator. It never selects a boot partition or resets hardware.
 * Keep all profiles/runtime/workers polling while active. */
typedef struct {
    dbt_runtime_t *runtime;probe_product_io_owner_t *input;
    uint32_t epoch,deadline;bool active,failed,prepared,avrcp_held;
} probe_restart_t;
bool probe_restart_init(probe_restart_t *,dbt_runtime_t *,probe_product_io_owner_t *);
dbt_result_t probe_restart_prepare(probe_restart_t *,uint32_t epoch,uint32_t now_ms);
bool probe_restart_quiescent(probe_restart_t *,uint32_t now_ms);
void probe_restart_cancel(probe_restart_t *,uint32_t now_ms);
bool probe_restart_active(const probe_restart_t *);
#endif
