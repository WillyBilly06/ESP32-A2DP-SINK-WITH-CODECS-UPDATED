#ifndef DBT_SMP_SESSION_H
#define DBT_SMP_SESSION_H
#include "dbt_runtime.h"
#include "dbt_smp_bearer.h"
#include "dbt_smp_features.h"

/* One owner-thread session per LE link. The application forwards the existing
 * runtime fixed-data, packet and link callbacks; none performs crypto, storage,
 * allocation or HCI submission. poll() runs on the same owner outside runtime
 * callbacks. Feature exchange is resident; the selected phase-2 bearer is
 * allocated only after negotiation. The exact connection addresses are supplied
 * by the application and must be those used when the connection was created. */
typedef enum { DBT_SMP_SESSION_IDLE, DBT_SMP_SESSION_FEATURES, DBT_SMP_SESSION_LEGACY,
    DBT_SMP_SESSION_SC, DBT_SMP_SESSION_FAILED, DBT_SMP_SESSION_CLOSED } dbt_smp_session_phase_t;
typedef enum { DBT_SMP_SESSION_CRYPTO_NONE, DBT_SMP_SESSION_CRYPTO_LEGACY,
    DBT_SMP_SESSION_CRYPTO_SC } dbt_smp_session_crypto_kind_t;
typedef struct {
    dbt_smp_session_crypto_kind_t kind;
    union { dbt_smp_job_t legacy; dbt_smp_sc_job_t sc; } input;
} dbt_smp_session_crypto_job_t;
typedef struct {
    dbt_smp_feature_policy_t features;
    bool allow_just_works,have_passkey,have_oob,have_local_keys;
    uint32_t passkey;
    uint8_t legacy_oob_tk[16],sc_oob_public[64],sc_local_oob_random[16],sc_peer_oob_random[16],sc_peer_oob_confirm[16];
    bool oob_authenticated,existing_bond,old_sc,old_authenticated;
    uint8_t old_key_size;
    dbt_smp_dist_keys_t local_keys;
} dbt_smp_session_config_t;
typedef struct {
    bool encrypted,authenticated,bond_committed,peer_key_verified;
    bool peer_final_received,final_ll_acked;
    uint8_t key_size,negotiated_local_keys,negotiated_peer_keys;
    bool phase3_complete,identity_promotion_committed;
    uint8_t admitted_local_keys,received_peer_keys;
} dbt_smp_session_security_t;
#define DBT_SMP_SESSION_RX_DEPTH 8u
typedef struct { uint8_t data[65],size; } dbt_smp_session_rx_t;
typedef struct dbt_smp_session {
    dbt_runtime_t *runtime;
    dbt_smp_guard_t *guard;
    dbt_smp_features_t feature;
    dbt_smp_session_config_t config;
    void *bearer;
    uint8_t *allocation;
    dbt_smp_session_rx_t *rx_queue;
    uint32_t generation,now,crypto_serial,store_serial,store_deadline,tx_serial,command_generation,command_token;
    uint16_t handle,command_opcode,bearer_size;
    uint8_t role,local_type,peer_type,local[6],peer[6];
    uint8_t rx_head,rx_count,tx[65],tx_size,pending_key[16],key_size;
    uint8_t ltk_rand[8]; uint16_t ltk_ediv;
    dbt_smp_session_phase_t phase;
    dbt_smp_session_phase_t bearer_kind;
    dbt_smp_session_crypto_kind_t crypto_kind;
    bool tx_active,tx_distribution,crypto_issued,store_issued,command_pending,command_accepted;
    bool key_ready,key_consumed,ltk_request,ltk_invalid,encrypted_event_pending;
    bool encryption_event_seen,uncertain_generation;
    bool result_reported,closing,store_wait_started;
} dbt_smp_session_t;
/* Caller zero-initializes the session. runtime must be ready with its fixed_data
 * hook installed, link must be a current unencrypted LE ACL. The feature
 * guard belongs to the claimed identity and remains alive across links. A
 * bounded receive queue is allocated once here and released on close; receive
 * copies without allocating. Exhaustion fails the procedure explicitly. */
dbt_result_t dbt_smp_session_open(dbt_smp_session_t *s,dbt_runtime_t *runtime,
    uint16_t handle,uint32_t generation,uint8_t local_type,const uint8_t local[6],
    uint8_t peer_type,const uint8_t peer[6],dbt_smp_guard_t *guard,
    const dbt_smp_session_config_t *config,uint32_t now);
dbt_result_t dbt_smp_session_start(dbt_smp_session_t *s,uint32_t now);
dbt_result_t dbt_smp_session_receive(dbt_smp_session_t *s,const dbt_link_t *link,uint16_t cid,
    const uint8_t *pdu,uint16_t size);
/* Returns OK when an LE LTK Request for this session was claimed; the GAP
 * security hook must not also send a negative reply for a claimed request. */
dbt_result_t dbt_smp_session_event(dbt_smp_session_t *s,const dbt_hci_packet_t *packet);
void dbt_smp_session_link(dbt_smp_session_t *s,const dbt_link_change_t *change);
void dbt_smp_session_poll(dbt_smp_session_t *s,uint32_t now);
dbt_result_t dbt_smp_session_crypto_job(dbt_smp_session_t *s,dbt_smp_session_crypto_job_t *job);
dbt_result_t dbt_smp_session_crypto_complete(dbt_smp_session_t *s,uint32_t generation,uint32_t serial,
    bool success,const dbt_smp_job_result_t *legacy,const dbt_smp_sc_result_t *sc,uint32_t now);
dbt_result_t dbt_smp_session_store_job(dbt_smp_session_t *s,dbt_smp_bond_job_t *job);
dbt_result_t dbt_smp_session_store_complete(dbt_smp_session_t *s,uint32_t generation,uint32_t serial,
    bool durable_commit,uint32_t now);
dbt_result_t dbt_smp_session_final_ack(dbt_smp_session_t *s,uint32_t generation,uint32_t serial,
    bool acknowledged,uint32_t now);
dbt_result_t dbt_smp_session_distribution_token(const dbt_smp_session_t *s,uint32_t *serial);
dbt_result_t dbt_smp_session_numeric_value(const dbt_smp_session_t *s,uint32_t *value);
dbt_result_t dbt_smp_session_numeric_confirm(dbt_smp_session_t *s,bool accepted,uint32_t now);
dbt_result_t dbt_smp_session_security(const dbt_smp_session_t *s,dbt_smp_session_security_t *status);
/* Refuses to invalidate a borrowed runtime PDU or outstanding HCI command. */
dbt_result_t dbt_smp_session_close(dbt_smp_session_t *s);
#endif
