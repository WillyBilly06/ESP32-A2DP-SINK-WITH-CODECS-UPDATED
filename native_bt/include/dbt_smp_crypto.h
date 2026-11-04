#ifndef DBT_SMP_CRYPTO_H
#define DBT_SMP_CRYPTO_H
#include <stdbool.h>
#include <stdint.h>

/* SMP integers, addresses, and PDUs use least-significant octet first, including
 * the opcode as byte 0 of preq/pres. The AES hook uses FIPS-197 byte order. Run
 * these functions on a crypto worker, never on a controller callback/host loop. */
typedef bool (*dbt_smp_aes128_fn)(void *context, const uint8_t key_be[16],
                                  const uint8_t block_be[16], uint8_t result_be[16]);
bool dbt_smp_c1(dbt_smp_aes128_fn aes, void *context, const uint8_t tk_le[16],
    const uint8_t random_le[16], const uint8_t preq[7], const uint8_t pres[7],
    uint8_t iat, const uint8_t ia_le[6], uint8_t rat, const uint8_t ra_le[6],
    uint8_t confirm_le[16]);
bool dbt_smp_s1(dbt_smp_aes128_fn aes, void *context, const uint8_t tk_le[16],
    const uint8_t responder_random_le[16], const uint8_t initiator_random_le[16],
    uint8_t stk_le[16]);
/* Resolve one HCI-order RPA with one SMP-order IRK. Returns false on invalid
 * address or AES failure; matching is written only on successful execution. */
bool dbt_smp_ah_resolve(dbt_smp_aes128_fn aes,void *context,const uint8_t irk_le[16],
                        const uint8_t rpa_le[6],bool *matching);

/* Secure Connections primitives only, not a pairing implementation. All
 * multi-octet values here are in specification display / big-endian order,
 * including each 7-byte address (type followed by address). The worker must
 * supply a real AES-CMAC implementation. No controller or storage calls. */
typedef bool (*dbt_smp_cmac_fn)(void *context, const uint8_t key[16],
    const uint8_t *message, unsigned length, uint8_t result[16]);
bool dbt_smp_f4(dbt_smp_cmac_fn cmac, void *context, const uint8_t u[32],
    const uint8_t v[32], const uint8_t x[16], uint8_t z, uint8_t out[16]);
bool dbt_smp_f5(dbt_smp_cmac_fn cmac, void *context, const uint8_t w[32],
    const uint8_t n1[16], const uint8_t n2[16], const uint8_t a1[7],
    const uint8_t a2[7], uint8_t mackey[16], uint8_t ltk[16]);
bool dbt_smp_f6(dbt_smp_cmac_fn cmac, void *context, const uint8_t w[16],
    const uint8_t n1[16], const uint8_t n2[16], const uint8_t r[16],
    const uint8_t io[3], const uint8_t a1[7], const uint8_t a2[7], uint8_t out[16]);
bool dbt_smp_g2(dbt_smp_cmac_fn cmac, void *context, const uint8_t u[32],
    const uint8_t v[32], const uint8_t x[16], const uint8_t y[16], uint32_t *out);

/* Secure Connections stage-2 worker helper. The ECDH hook must validate both
 * P-256 points, bind local_public to the correct private key held by the
 * worker, reject debug keys and return the 32-byte big-endian shared secret.
 * It must run off the host thread. This computes candidate values only; a
 * caller must verify the peer's f4 commitment and DHKey check, enforce its
 * association policy, and correlate the later encryption event. */
typedef bool (*dbt_smp_ecdh_fn)(void *context,const uint8_t local_public[64],
    const uint8_t peer_public[64],uint8_t shared[32]);
bool dbt_smp_sc_derive(dbt_smp_ecdh_fn ecdh,dbt_smp_cmac_fn cmac,void *context,
    const uint8_t local_public[64],const uint8_t peer_public[64],
    const uint8_t na[16],const uint8_t nb[16],const uint8_t a[7],const uint8_t b[7],
    const uint8_t io_a[3],const uint8_t io_b[3],const uint8_t ra[16],const uint8_t rb[16],
    uint8_t key_size,uint8_t ltk[16],uint8_t check_a[16],uint8_t check_b[16]);
bool dbt_smp_sc_confirm(dbt_smp_cmac_fn cmac,void *context,const uint8_t u[32],
    const uint8_t v[32],const uint8_t nonce[16],uint8_t z,const uint8_t expected[16]);
#endif
