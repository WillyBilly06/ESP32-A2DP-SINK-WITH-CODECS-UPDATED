#ifndef DBT_CRYPTO_H
#define DBT_CRYPTO_H

/* Cryptographic primitives for storage/crypto workers, never VHCI callbacks.
 * Protocol state and byte-order conversions remain in the owned SMP modules.
 * The legacy branch keeps the existing host reference tests usable; ESP-IDF
 * 6.1 uses only public PSA APIs, not Mbed TLS private declarations.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "dbt_sha256.h"

#if MBEDTLS_VERSION_MAJOR >= 4
#include "psa/crypto.h"
static inline bool dbt_crypto_aes128(const uint8_t key[16], const uint8_t in[16], uint8_t out[16]) {
    if (psa_crypto_init() != PSA_SUCCESS) return false;
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    psa_set_key_type(&attributes, PSA_KEY_TYPE_AES);
    psa_set_key_bits(&attributes, 128);
    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_ENCRYPT);
    psa_set_key_algorithm(&attributes, PSA_ALG_ECB_NO_PADDING);
    psa_key_id_t id = 0;
    psa_status_t status = psa_import_key(&attributes, key, 16, &id);
    psa_reset_key_attributes(&attributes);
    if (status != PSA_SUCCESS) return false;
    size_t n = 0;
    status = psa_cipher_encrypt(id, PSA_ALG_ECB_NO_PADDING, in, 16, out, 16, &n);
    psa_status_t cleanup = psa_destroy_key(id);
    return status == PSA_SUCCESS && cleanup == PSA_SUCCESS && n == 16;
}
static inline bool dbt_crypto_cmac128(const uint8_t key[16], const uint8_t *p, size_t n, uint8_t out[16]) {
    if (psa_crypto_init() != PSA_SUCCESS) return false;
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    psa_set_key_type(&attributes, PSA_KEY_TYPE_AES);
    psa_set_key_bits(&attributes, 128);
    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_SIGN_MESSAGE);
    psa_set_key_algorithm(&attributes, PSA_ALG_CMAC);
    psa_key_id_t id = 0;
    psa_status_t status = psa_import_key(&attributes, key, 16, &id);
    psa_reset_key_attributes(&attributes);
    if (status != PSA_SUCCESS) return false;
    size_t written = 0;
    status = psa_mac_compute(id, PSA_ALG_CMAC, p, n, out, 16, &written);
    psa_status_t cleanup = psa_destroy_key(id);
    return status == PSA_SUCCESS && cleanup == PSA_SUCCESS && written == 16;
}
#else
#include "mbedtls/aes.h"
#include "mbedtls/cipher.h"
#include "mbedtls/cmac.h"
static inline bool dbt_crypto_aes128(const uint8_t key[16], const uint8_t in[16], uint8_t out[16]) {
    mbedtls_aes_context context;
    mbedtls_aes_init(&context);
    int status = mbedtls_aes_setkey_enc(&context, key, 128);
    if (!status) status = mbedtls_aes_crypt_ecb(&context, MBEDTLS_AES_ENCRYPT, in, out);
    mbedtls_aes_free(&context);
    return status == 0;
}
static inline bool dbt_crypto_cmac128(const uint8_t key[16], const uint8_t *p, size_t n, uint8_t out[16]) {
    const mbedtls_cipher_info_t *info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_128_ECB);
    return info && mbedtls_cipher_cmac(info, key, 128, p, n, out) == 0;
}
#endif
#endif
