#ifndef DBT_SMP_NVS_H
#define DBT_SMP_NVS_H
#include "dbt_smp_bond.h"
/* SDK adapter, callable only by a serialized storage worker. The application
 * opens a dedicated NVS namespace, owns its lifecycle, configures encryption
 * if required, and serializes all writers/readers to that namespace including
 * cross-transport writers. Success means nvs_commit plus readback verification.
 * It does not initialize or erase any partition or namespace. After an
 * ambiguous mutation failure the provider is poisoned; reopen and explicitly
 * inspect/reconcile the persistent intent before permitting further access. */
/* ESP-IDF's nvs_handle_t is uint32_t; keep SDK headers out of this API. */
typedef struct { uint32_t handle; bool poisoned; } dbt_smp_nvs_t;
void dbt_smp_nvs_init(dbt_smp_nvs_t *provider,uint32_t handle);
dbt_result_t dbt_smp_nvs_execute(dbt_smp_nvs_t *provider,const dbt_smp_bond_job_t *job,
    dbt_smp_bond_record_t *loaded);
/* Worker only. Read the versioned intent's non-secret identity/operation.
 * DBT_EMPTY means no intent; invalid/unreadable markers fail closed. */
dbt_result_t dbt_smp_nvs_pending(dbt_smp_nvs_t *provider,dbt_smp_bond_op_t *operation,
    uint8_t *identity_type,uint8_t identity[6]);
/* Explicit worker-only recovery: clear the marker only when identity and
 * operation match and the durable state matches its before or after digest.
 * Set applied only on success: true means target state, false means before
 * state. Never rewrites a bond record or establishes peer possession. */
dbt_result_t dbt_smp_nvs_reconcile(dbt_smp_nvs_t *provider,dbt_smp_bond_op_t operation,
    uint8_t identity_type,const uint8_t identity[6],bool *applied);
/* Worker only, at most 32 namespace entries; exactly one IRK-backed match.
 * A candidate match is provisional until a correlated LTK encryption and
 * durable promotion. Ambiguous, malformed, over-limit and pending-intent
 * stores fail closed; no record is modified. */
dbt_result_t dbt_smp_nvs_resolve_rpa(dbt_smp_nvs_t *,const uint8_t rpa_le[6],
                                      dbt_smp_bond_record_t *,bool *candidate);
/* Worker only: independently bind distributed SMP-order IRK to the actual
 * connection RPA before committing a first private-address pairing. */
bool dbt_smp_nvs_verify_rpa(const uint8_t rpa_le[6],const uint8_t irk_le[16],bool *matching);
#endif
