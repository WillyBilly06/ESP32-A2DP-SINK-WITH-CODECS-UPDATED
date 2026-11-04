#ifndef DBT_SMP_H
#define DBT_SMP_H
#include "dbt_queue.h"
#include "dbt_smp_bond.h"
#include "dbt_smp_dist.h"

/* Experimental legacy Just Works, Passkey Entry, or OOB.
 * One instance per LE link;
 * all state APIs are host-owner only. Not a production security provider. */
typedef enum { DBT_SMP_IDLE, DBT_SMP_FEATURES, DBT_SMP_LOCAL_CRYPTO,
 DBT_SMP_CONFIRM, DBT_SMP_RANDOM, DBT_SMP_VERIFY, DBT_SMP_ENCRYPTION,
 DBT_SMP_SECURE, DBT_SMP_FAILED, DBT_SMP_TIMED_OUT, DBT_SMP_DISTRIBUTION,
 DBT_SMP_BOND_PENDING } dbt_smp_state_t;
typedef enum { DBT_SMP_JOB_NONE, DBT_SMP_JOB_LOCAL, DBT_SMP_JOB_VERIFY } dbt_smp_job_kind_t;
typedef struct {
    dbt_smp_job_kind_t kind;
    uint32_t generation, serial;
    uint8_t preq[7], pres[7], ia[6], ra[6], iat, rat;
    uint8_t local_random[16], peer_random[16], tk[16];
    bool initiator;
} dbt_smp_job_t;
typedef struct { uint8_t random[16], confirm[16], stk[16]; } dbt_smp_job_result_t;
typedef struct {
    uint16_t handle; uint32_t generation, serial, deadline;
    dbt_smp_state_t state;
    dbt_smp_job_t job;
    uint8_t preq[7],pres[7],ia[6],ra[6],iat,rat;
    uint8_t random[16],peer_confirm[16],peer_random[16],stk[16],tx[17],tk[16];
    uint8_t tx_size,key_size,min_key_size,max_key_size,reason;
    bool initiator, allow_just_works, peer_confirmed, job_pending, key_taken, used;
    bool have_oob, oob_authenticated, authenticated, have_passkey;
    uint8_t io_capability;
    bool bonding,bond_committed;
    uint8_t local_mask,peer_mask;
    dbt_smp_bond_t bond;
    dbt_smp_dist_t dist;
    dbt_smp_dist_keys_t local_distribution,peer_distribution;
    bool peer_final_received,final_ll_acked,encryption_verified;
    uint32_t hci_buffers_completed,local_pdus_admitted;
} dbt_smp_t;
/* Addresses are exact connection-setup addresses, in HCI little-endian order.
 * role is 0 for central/initiator or 1 for peripheral/responder. */
dbt_result_t dbt_smp_open(dbt_smp_t *s, uint16_t handle, uint32_t generation,
 uint8_t role, uint8_t local_type, const uint8_t local[6], uint8_t peer_type,
 const uint8_t peer[6], uint8_t min_key_size, bool allow_just_works);
void dbt_smp_close(dbt_smp_t *s);
/* Configure a fresh peer-bound legacy OOB TK before pairing. authenticated is
 * an application assertion that the OOB channel provides the required MITM
 * protection; false never satisfies MITM policy. Both peers must advertise
 * OOB or pairing fails without fallback. The caller clears its own TK copy. */
dbt_result_t dbt_smp_set_oob(dbt_smp_t *s,const uint8_t tk[16],bool authenticated);
/* Fresh six-digit passkey supplied by the application UI. io_capability must
 * truthfully describe the local input/display hardware. No fallback to Just
 * Works is permitted when the peer's I/O cannot support Passkey Entry. */
dbt_result_t dbt_smp_set_passkey(dbt_smp_t *s,uint32_t passkey,uint8_t io_capability);
dbt_result_t dbt_smp_set_max_key_size(dbt_smp_t *s,uint8_t max_key_size);
/* Configure bonding before feature exchange, after selecting OOB/passkey. Both
 * local and peer masks must include legacy EncKey; keys come from trustworthy
 * workers. When this device sends the global final key, completion requires a
 * genuine final-key link acknowledgment. Existing bond properties must be obtained
 * from serialized trusted storage to prevent a downgrade. */
dbt_result_t dbt_smp_set_bonding(dbt_smp_t *s,uint8_t local_mask,uint8_t peer_mask,
    const dbt_smp_dist_keys_t *local,bool existing,bool old_sc,
    bool old_authenticated,uint8_t old_key_size);
/* Begin phase 2 from an already admitted feature transcript. The local
 * configuration (I/O, OOB, keys and bonding policy) must match it exactly. */
dbt_result_t dbt_smp_resume_features(dbt_smp_t *s,const uint8_t preq[7],const uint8_t pres[7],uint32_t now);
dbt_result_t dbt_smp_start(dbt_smp_t *s, uint32_t now);
dbt_result_t dbt_smp_receive(dbt_smp_t *s,const uint8_t *pdu,uint16_t size,uint32_t now);
void dbt_smp_poll(dbt_smp_t *s,uint32_t now);
dbt_result_t dbt_smp_tx(const dbt_smp_t *s,const uint8_t **data,uint8_t *size);
void dbt_smp_tx_done(dbt_smp_t *s,bool admitted);
/* Copy the job to a worker and publish its result back to the owner. The worker
 * MUST use a CSPRNG and the c1/s1 helpers including job.tk for exactly these job inputs. Every
 * outstanding job needs its own copy; clear that copy after completion. */
dbt_result_t dbt_smp_job(const dbt_smp_t *s,dbt_smp_job_t *out);
dbt_result_t dbt_smp_complete(dbt_smp_t *s,uint32_t generation,uint32_t serial,
 bool success,const dbt_smp_job_result_t *result,uint32_t now);
/* Consume once. Controller command acceptance is not encryption. Peripheral
 * returns this for a matching controller STK request; central starts encryption.
 * Caller must clear all copies and correlate subsequent encryption event. */
dbt_result_t dbt_smp_take_stk(dbt_smp_t *s,uint8_t out[16],uint8_t *size);
/* Call only on a correlated successful controller Encryption Change for this
 * generation after the submitted STK was actually used. Lost/disabled events
 * revoke security. A command status or an already encrypted link is insufficient. */
dbt_result_t dbt_smp_encryption(dbt_smp_t *s,uint32_t generation,bool encrypted);
dbt_result_t dbt_smp_dist_tx_legacy(dbt_smp_t *s,const uint8_t **p,uint8_t *n,uint32_t now);
dbt_result_t dbt_smp_dist_admitted_legacy(dbt_smp_t *s,uint32_t generation,uint32_t serial,bool admitted,uint32_t now);
dbt_result_t dbt_smp_dist_final_ack_legacy(dbt_smp_t *s,uint32_t generation,uint32_t serial,bool acknowledged,uint32_t now);
dbt_result_t dbt_smp_dist_receive_legacy(dbt_smp_t *s,const uint8_t *p,uint8_t n,uint32_t now);
dbt_result_t dbt_smp_bond_job_legacy(const dbt_smp_t *s,dbt_smp_bond_job_t *job);
dbt_result_t dbt_smp_bond_complete_legacy(dbt_smp_t *s,uint32_t generation,uint32_t serial,bool committed);
dbt_result_t dbt_smp_bond_cancel_legacy(dbt_smp_t *s,uint32_t generation,uint32_t serial);
#endif
