#ifndef DBT_SMP_SC_H
#define DBT_SMP_SC_H
#include "dbt_queue.h"
#include "dbt_smp_bond.h"
#include "dbt_smp_dist.h"

/* Experimental LE Secure Connections phase 2: Just Works, Numeric Comparison,
 * Passkey Entry, or bilateral Out-of-Band. Feature
 * exchange and phase 3 are separate; callers must select this engine only
 * after validating the full policy. All byte arrays are little-endian, as on
 * SMP/HCI. A single owner handles state; worker jobs are copied and returned
 * to that owner. Crypto/private keys never run on the owner or callback. */
typedef enum {
    DBT_SC_UNUSED, DBT_SC_KEYGEN, DBT_SC_WAIT_PUBLIC, DBT_SC_COMMIT,
    DBT_SC_WAIT_CONFIRM, DBT_SC_WAIT_RANDOM, DBT_SC_DERIVE, DBT_SC_WAIT_CHECK,
    DBT_SC_ENCRYPTION, DBT_SC_SECURE, DBT_SC_FAILED, DBT_SC_TIMED_OUT, DBT_SC_BOND_STORE,
    DBT_SC_DISTRIBUTION, DBT_SC_WAIT_USER, DBT_SC_ROUND_LOCAL, DBT_SC_ROUND_VERIFY,
    DBT_SC_OOB_VERIFY
} dbt_smp_sc_state_t;
typedef enum { DBT_SC_JOB_NONE, DBT_SC_JOB_KEYGEN, DBT_SC_JOB_COMMIT, DBT_SC_JOB_DERIVE,
               DBT_SC_JOB_ROUND_LOCAL, DBT_SC_JOB_ROUND_VERIFY, DBT_SC_JOB_OOB_VERIFY } dbt_smp_sc_job_kind_t;
typedef struct {
    dbt_smp_sc_job_kind_t kind;
    uint32_t generation,serial;
    bool initiator;
    uint8_t key_size,preq[7],pres[7],iat,rat,ia[6],ra[6];
    uint8_t local_public[64],peer_public[64],local_nonce[16],peer_nonce[16],peer_confirm[16];
    uint32_t passkey;
    uint8_t round;
    uint8_t local_oob_random[16],peer_oob_random[16],peer_oob_confirm[16];
} dbt_smp_sc_job_t;
typedef struct {
    uint8_t public_key[64],nonce[16],confirm[16],ltk[16],local_check[16],peer_check[16];
    bool peer_commitment_valid;
    uint32_t numeric_value;
} dbt_smp_sc_result_t;
typedef struct {
    uint16_t handle;
    uint32_t generation,serial,deadline;
    dbt_smp_sc_state_t state;
    dbt_smp_sc_job_t job;
    uint8_t preq[7],pres[7],iat,rat,ia[6],ra[6],key_size;
    uint8_t local_public[64],peer_public[64],local_nonce[16],peer_nonce[16];
    uint8_t peer_confirm[16],ltk[16],local_check[16],peer_check[16],tx[65],tx_size;
    bool initiator,job_pending,key_taken,used;
    bool bonding,bond_committed;
    uint8_t bond_ltk[16];
    dbt_smp_bond_t bond;
    uint8_t local_mask,peer_mask,local_csrk[16],peer_csrk[16];
    bool distribution_ready;
    dbt_smp_dist_t dist;
    bool numeric,authenticated;
    uint32_t numeric_value;
    uint32_t passkey;
    uint8_t round;
    bool passkey_mode,passkey_ready;
    bool oob_mode,oob_ready,oob_authenticated;
    uint8_t local_oob_public[64],local_oob_random[16],peer_oob_random[16],peer_oob_confirm[16];
    bool bond_policy_checked;
    dbt_smp_dist_keys_t local_distribution,peer_distribution;
    bool peer_final_received,final_ll_acked,encryption_verified;
    uint32_t hci_buffers_completed,local_pdus_admitted;
} dbt_smp_sc_t;
/* Both features must advertise SC; MITM requires compatible numeric
 * comparison or passkey I/O capabilities; bilateral OOB overrides I/O. Phase 3 accepts
 * identity and signing-key distribution; other negotiated masks fail closed. Bonding
 * is accepted only when both request it and the peer used a public/static
 * identity address, or an RPA with peer identity-key distribution negotiated.
 * The distributed identity remains provisional until subsequent LTK proof.
 * No implicit downgrade. A fresh generation identifies all
 * link and worker context. */
dbt_result_t dbt_smp_sc_open(dbt_smp_sc_t *s,uint16_t handle,uint32_t generation,
    uint8_t role,uint8_t local_type,const uint8_t local[6],uint8_t peer_type,const uint8_t peer[6],
    const uint8_t preq[7],const uint8_t pres[7],uint8_t min_key_size,uint32_t now);
void dbt_smp_sc_close(dbt_smp_sc_t *s);
dbt_result_t dbt_smp_sc_receive(dbt_smp_sc_t *s,const uint8_t *p,uint16_t n,uint32_t now);
void dbt_smp_sc_poll(dbt_smp_sc_t *s,uint32_t now);
dbt_result_t dbt_smp_sc_tx(const dbt_smp_sc_t *s,const uint8_t **p,uint8_t *n);
void dbt_smp_sc_tx_done(dbt_smp_sc_t *s,bool admitted,uint32_t now);
dbt_result_t dbt_smp_sc_job(const dbt_smp_sc_t *s,dbt_smp_sc_job_t *out);
/* KEYGEN: fresh worker-held P-256 private key and CSPRNG nonce. COMMIT:
 * f4(PKbx,PKax,Nb,0). DERIVE: validate peer P-256 point, reject debug keys,
 * bind private key to local public key, perform ECDH, verify peer commitment
 * on the initiator, derive LTK and both f6 checks, and calculate numeric_value
 * with g2 mod 1,000,000 for Numeric Comparison. ROUND_LOCAL generates a fresh
 * nonce and computes f4(local_x,peer_x,nonce,0x80|passkey_bit); ROUND_VERIFY
 * validates the peer's round commitment. DERIVE uses the passkey in f6 for
 * Passkey Entry, or the two OOB random values for OOB. OOB_VERIFY checks
 * f4(peer_x,peer_x,peer_oob_random,0) against peer_oob_confirm. The OOB
 * KEYGEN worker must retrieve the same private/public key used to create its
 * pre-exchanged OOB data. Wipe all worker copies. */
dbt_result_t dbt_smp_sc_complete(dbt_smp_sc_t *s,uint32_t generation,uint32_t serial,
    bool success,const dbt_smp_sc_result_t *result,uint32_t now);
dbt_result_t dbt_smp_sc_take_ltk(dbt_smp_sc_t *s,uint8_t key[16],uint8_t *key_size);
dbt_result_t dbt_smp_sc_encryption(dbt_smp_sc_t *s,uint32_t generation,bool encrypted);
dbt_result_t dbt_smp_sc_numeric_value(const dbt_smp_sc_t *s,uint32_t generation,uint32_t *value);
dbt_result_t dbt_smp_sc_numeric_confirm(dbt_smp_sc_t *s,uint32_t generation,bool accepted,uint32_t now);
/* The UI must use a fresh random six-digit passkey for every pairing, with the
 * input/display roles dictated by the exchanged IO capabilities. */
dbt_result_t dbt_smp_sc_set_passkey(dbt_smp_sc_t *s,uint32_t passkey);
/* Both OOB flags must be set. Peer data must already be securely bound to the
 * connection identity. The caller supplies its pre-exchanged public key and
 * random plus the peer random and confirm, all in SMP little-endian order.
 * The channel authentication claim is an application assertion. */
dbt_result_t dbt_smp_sc_set_oob(dbt_smp_sc_t *s,const uint8_t local_public[64],
    const uint8_t local_random[16],const uint8_t peer_random[16],const uint8_t peer_confirm[16],
    bool channel_authenticated);
/* Before bonding, compare the intended new security properties with the
 * trusted existing record (if any). The storage provider must serialize this
 * lookup and final commit to prevent a concurrent bond replacement race. */
dbt_result_t dbt_smp_sc_check_bond_policy(dbt_smp_sc_t *s,bool existing,
    bool existing_authenticated,uint8_t existing_key_size);
/* Provide fresh local identity/signing keys before KEYGEN completes. If this
 * device sends the final key, completion waits for genuine link acknowledgment. */
dbt_result_t dbt_smp_sc_set_distribution(dbt_smp_sc_t *s,const dbt_smp_dist_keys_t *keys);
dbt_result_t dbt_smp_sc_dist_tx(dbt_smp_sc_t *s,const uint8_t **p,uint8_t *n,uint32_t now);
dbt_result_t dbt_smp_sc_dist_tx_admitted(dbt_smp_sc_t *s,uint32_t generation,uint32_t serial,bool admitted,uint32_t now);
dbt_result_t dbt_smp_sc_dist_final_ack(dbt_smp_sc_t *s,uint32_t generation,uint32_t serial,bool acknowledged,uint32_t now);
dbt_result_t dbt_smp_sc_dist_receive(dbt_smp_sc_t *s,const uint8_t *p,uint8_t n,uint32_t now);
/* Present only after verified encryption for a negotiated bond. Storage must
 * atomically and durably commit before reporting success. Until then the bond
 * is not committed and state is DBT_SC_BOND_STORE. */
dbt_result_t dbt_smp_sc_bond_job(const dbt_smp_sc_t *s,dbt_smp_bond_job_t *job);
dbt_result_t dbt_smp_sc_bond_complete(dbt_smp_sc_t *s,uint32_t generation,uint32_t serial,bool committed);
dbt_result_t dbt_smp_sc_bond_cancel(dbt_smp_sc_t *s,uint32_t generation,uint32_t serial);
#endif
