/* Host-only software PSA provider for the exact primitives used by SMP/OTA.
 * This does not substitute for tests of Espressif's target hardware drivers.
 */
#ifndef DBT_PSA_TEST_CONFIG_H
#define DBT_PSA_TEST_CONFIG_H
#define MBEDTLS_PSA_CRYPTO_C
#define MBEDTLS_PSA_BUILTIN_GET_ENTROPY
#define MBEDTLS_CTR_DRBG_C
#define MBEDTLS_PSA_P256M_DRIVER_ENABLED
#define PSA_WANT_KEY_TYPE_AES 1
#define PSA_WANT_ALG_ECB_NO_PADDING 1
#define PSA_WANT_ALG_CMAC 1
#define PSA_WANT_ALG_SHA_256 1
#define PSA_WANT_ALG_ECDH 1
#define PSA_WANT_ECC_SECP_R1_256 1
#define PSA_WANT_KEY_TYPE_ECC_PUBLIC_KEY 1
#define PSA_WANT_KEY_TYPE_ECC_KEY_PAIR_BASIC 1
#define PSA_WANT_KEY_TYPE_ECC_KEY_PAIR_IMPORT 1
#define PSA_WANT_KEY_TYPE_ECC_KEY_PAIR_EXPORT 1
#define PSA_WANT_KEY_TYPE_ECC_KEY_PAIR_GENERATE 1
#endif
