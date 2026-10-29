#ifndef DBT_SMP_BOND_H
#define DBT_SMP_BOND_H
#include "dbt_queue.h"

/* Storage broker, independent of the incomplete pairing engine. A storage
 * worker performs the requested operation and acknowledges durable commit.
 * Identity is a stable claimed peer identity, never a transient RPA. A
 * distributed candidate remains provisional until correlated LTK proof and
 * the durable promotion receipt; Just Works is not MITM authentication.
 * Owner assigns a fresh nonzero generation on each lifetime. */
typedef struct {
    uint8_t identity_type, identity[6];
    uint8_t ltk[16], rand[8], irk[16];
    uint16_t ediv;
    uint8_t key_size;
    bool authenticated, secure_connections, have_irk;
    uint8_t local_csrk[16],peer_csrk[16];
    bool have_local_csrk,have_peer_csrk;
    uint8_t local_irk[16],candidate_irk[16],candidate_identity[6],candidate_type;
    bool have_local_irk,candidate_present,candidate_verified;
    uint8_t local_ltk[16],local_rand[8];
    uint16_t local_ediv;
    bool have_local_ltk,have_peer_ltk;
} dbt_smp_bond_record_t;
typedef enum { DBT_SMP_BOND_NONE, DBT_SMP_BOND_STORE, DBT_SMP_BOND_LOAD,
               DBT_SMP_BOND_DELETE } dbt_smp_bond_op_t;
typedef struct {
    uint32_t generation, serial;
    dbt_smp_bond_op_t op;
    uint8_t identity_type, identity[6];
    dbt_smp_bond_record_t record;
} dbt_smp_bond_job_t;
typedef struct {
    uint32_t generation, serial;
    dbt_smp_bond_job_t job;
    dbt_smp_bond_record_t loaded;
    bool pending, loaded_valid;
} dbt_smp_bond_t;
void dbt_smp_bond_init(dbt_smp_bond_t *b,uint32_t generation);
void dbt_smp_bond_close(dbt_smp_bond_t *b);
dbt_result_t dbt_smp_bond_begin(dbt_smp_bond_t *b,dbt_smp_bond_op_t op,
    uint8_t identity_type,const uint8_t identity[6],const dbt_smp_bond_record_t *record);
dbt_result_t dbt_smp_bond_job(const dbt_smp_bond_t *b,dbt_smp_bond_job_t *job);
/* success for STORE/DELETE means durable completion, not queue admission.
 * LOAD success must supply a validated storage result matching the identity. */
dbt_result_t dbt_smp_bond_complete(dbt_smp_bond_t *b,uint32_t generation,
    uint32_t serial,bool success,const dbt_smp_bond_record_t *loaded);
/* On an owner-managed timeout, invalidate the outstanding transaction. A late
 * worker receipt is rejected; it cannot undo storage effects already issued. */
dbt_result_t dbt_smp_bond_cancel(dbt_smp_bond_t *b,uint32_t generation,uint32_t serial);
/* Consume a loaded record once, only if it meets all requested properties. */
dbt_result_t dbt_smp_bond_take(dbt_smp_bond_t *b,uint8_t min_key_size,
    bool require_authenticated,bool require_sc,dbt_smp_bond_record_t *record);

typedef enum { DBT_SMP_RECONNECT_UNUSED, DBT_SMP_RECONNECT_LOAD, DBT_SMP_RECONNECT_KEY,
               DBT_SMP_RECONNECT_ENCRYPTION, DBT_SMP_RECONNECT_SECURE,
               DBT_SMP_RECONNECT_FAILED, DBT_SMP_RECONNECT_PROMOTE } dbt_smp_reconnect_state_t;
typedef struct {
    uint32_t generation;
    dbt_smp_bond_t broker;
    dbt_smp_bond_record_t record;
    dbt_smp_reconnect_state_t state;
    uint8_t min_key_size;
    bool require_authenticated,require_sc;
    bool candidate_mode;
    uint8_t candidate_type,candidate_identity[6];
    bool local_role_peripheral;
    bool peer_key_verified,identity_promotion_committed;
} dbt_smp_reconnect_t;
/* Identity must be verified externally (public or static identity address).
 * Load is asynchronous; no key is released before a validated load result. */
dbt_result_t dbt_smp_reconnect_open(dbt_smp_reconnect_t *r,uint32_t generation,
    uint8_t identity_type,const uint8_t identity[6],uint8_t min_key_size,
    bool require_authenticated,bool require_sc);
dbt_result_t dbt_smp_reconnect_open_role(dbt_smp_reconnect_t *r,uint32_t generation,
    uint8_t identity_type,const uint8_t identity[6],uint8_t min_key_size,
    bool require_authenticated,bool require_sc,bool local_role_peripheral);
/* Load using an established identity but attempt encryption with the supplied
 * provisional identity. Promotion must be durably committed after encryption. */
dbt_result_t dbt_smp_reconnect_open_candidate(dbt_smp_reconnect_t *r,uint32_t generation,
    uint8_t identity_type,const uint8_t identity[6],uint8_t candidate_type,const uint8_t candidate[6],
    uint8_t min_key_size,bool require_authenticated,bool require_sc);
dbt_result_t dbt_smp_reconnect_open_candidate_role(dbt_smp_reconnect_t *,uint32_t,
    uint8_t,const uint8_t[6],uint8_t,const uint8_t[6],uint8_t,bool,bool,bool local_role_peripheral);
void dbt_smp_reconnect_close(dbt_smp_reconnect_t *r);
dbt_result_t dbt_smp_reconnect_job(const dbt_smp_reconnect_t *r,dbt_smp_bond_job_t *job);
dbt_result_t dbt_smp_reconnect_complete(dbt_smp_reconnect_t *r,uint32_t generation,uint32_t serial,
    bool success,const dbt_smp_bond_record_t *record);
dbt_result_t dbt_smp_reconnect_cancel(dbt_smp_reconnect_t *r,uint32_t generation,uint32_t serial);
dbt_result_t dbt_smp_reconnect_ids(const dbt_smp_reconnect_t *r,uint16_t *ediv,uint8_t rand[8]);
/* Supplied identifiers must match, including SC's zero EDIV/Rand. */
dbt_result_t dbt_smp_reconnect_take(dbt_smp_reconnect_t *r,uint16_t ediv,const uint8_t rand[8],
    uint8_t ltk[16],uint8_t *key_size);
/* Only a correlated encryption event after using that key proves encryption. */
dbt_result_t dbt_smp_reconnect_encryption(dbt_smp_reconnect_t *r,uint32_t generation,bool encrypted);
#endif
