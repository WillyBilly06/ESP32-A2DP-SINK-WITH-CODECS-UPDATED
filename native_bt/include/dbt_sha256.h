#ifndef DBT_SHA256_H
#define DBT_SHA256_H
#include <stddef.h>
#include <stdint.h>
#include "mbedtls/version.h"
#if MBEDTLS_VERSION_MAJOR >= 4
#include "psa/crypto.h"
typedef psa_hash_operation_t dbt_sha256_context;

static inline void dbt_sha256_init(dbt_sha256_context *h) {
    *h = (psa_hash_operation_t)PSA_HASH_OPERATION_INIT;
}
static inline void dbt_sha256_free(dbt_sha256_context *h) {
    (void)psa_hash_abort(h);
    dbt_sha256_init(h);
}
static inline int dbt_sha256_starts(dbt_sha256_context *h, int is224) {
    if (is224 || psa_crypto_init() != PSA_SUCCESS) return -1;
    return (int)psa_hash_setup(h, PSA_ALG_SHA_256);
}
static inline int dbt_sha256_update(dbt_sha256_context *h, const uint8_t *p, size_t n) {
    return (int)psa_hash_update(h, p, n);
}
static inline int dbt_sha256_finish(dbt_sha256_context *h, uint8_t out[32]) {
    size_t n = 0;
    psa_status_t status = psa_hash_finish(h, out, 32, &n);
    return status == PSA_SUCCESS && n == 32 ? 0 : -1;
}
static inline int dbt_sha256(const uint8_t *p, size_t n, uint8_t out[32], int is224) {
    size_t written = 0;
    if (is224 || psa_crypto_init() != PSA_SUCCESS) return -1;
    psa_status_t status = psa_hash_compute(PSA_ALG_SHA_256, p, n, out, 32, &written);
    return status == PSA_SUCCESS && written == 32 ? 0 : -1;
}
#else
#include "mbedtls/sha256.h"
typedef mbedtls_sha256_context dbt_sha256_context;
#define dbt_sha256_init mbedtls_sha256_init
#define dbt_sha256_free mbedtls_sha256_free
#define dbt_sha256_starts mbedtls_sha256_starts
#define dbt_sha256_update mbedtls_sha256_update
#define dbt_sha256_finish mbedtls_sha256_finish
#define dbt_sha256 mbedtls_sha256
#endif
#endif
