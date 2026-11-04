#ifndef DBT_SMP_WORKER_MBEDTLS_H
#define DBT_SMP_WORKER_MBEDTLS_H
#include <stddef.h>
#include "dbt_smp.h"
#include "dbt_smp_sc.h"
/* One zero-initialized worker per active link. Allocate, invoke and close only on a crypto
 * worker thread. rng must supply cryptographically secure random bytes and may
 * block. Never use a controller callback or host loop to run these functions. */
typedef bool (*dbt_smp_worker_rng_fn)(void *context,uint8_t *out,size_t n);
typedef struct { void *internal; } dbt_smp_worker_mbedtls_t;
bool dbt_smp_worker_mbedtls_open(dbt_smp_worker_mbedtls_t *w,uint32_t generation,
    dbt_smp_worker_rng_fn rng,void *rng_context);
void dbt_smp_worker_mbedtls_close(dbt_smp_worker_mbedtls_t *w);
/* Generate a fresh private key and OOB data before feature exchange. The
 * caller securely transfers public-key commitment/random/identity out of band
 * and uses this same worker for the ensuing generation-bound SC jobs. */
bool dbt_smp_worker_mbedtls_oob(dbt_smp_worker_mbedtls_t *w,uint8_t public_key[64],
    uint8_t random[16],uint8_t confirm[16]);
bool dbt_smp_worker_mbedtls_legacy(dbt_smp_worker_mbedtls_t *w,const dbt_smp_job_t *job,
    dbt_smp_job_result_t *result);
bool dbt_smp_worker_mbedtls_sc(dbt_smp_worker_mbedtls_t *w,const dbt_smp_sc_job_t *job,
    dbt_smp_sc_result_t *result);
/* Generate local phase-3 material once per worker/generation. Identity must be
 * an application-verified public or static address. Legacy LTK is generated
 * at full width and masked later to the negotiated size. */
bool dbt_smp_worker_mbedtls_distribution(dbt_smp_worker_mbedtls_t *w,uint8_t mask,
    uint8_t identity_type,const uint8_t identity[6],dbt_smp_dist_keys_t *out);
#endif
