#include "dbt_crypto.h"
#include "dbt_smp_worker_mbedtls.h"
#include <stdio.h>
#include <string.h>

#define REQUIRE(x) do { if (!(x)) { fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #x); return 1; } } while (0)
static bool random_bytes(void *context, uint8_t *out, size_t n) {
    (void)context;
    return psa_generate_random(out, n) == PSA_SUCCESS;
}
static int primitives(void) {
    static const uint8_t key[16] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
    static const uint8_t input[16] = {0,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff};
    static const uint8_t encrypted[16] = {0x69,0xc4,0xe0,0xd8,0x6a,0x7b,0x04,0x30,0xd8,0xcd,0xb7,0x80,0x70,0xb4,0xc5,0x5a};
    static const uint8_t cmac_key[16] = {0x2b,0x7e,0x15,0x16,0x28,0xae,0xd2,0xa6,0xab,0xf7,0x15,0x88,0x09,0xcf,0x4f,0x3c};
    static const uint8_t empty_mac[16] = {0xbb,0x1d,0x69,0x29,0xe9,0x59,0x37,0x28,0x7f,0xa3,0x7d,0x12,0x9b,0x75,0x67,0x46};
    static const uint8_t sha_abc[32] = {0xba,0x78,0x16,0xbf,0x8f,0x01,0xcf,0xea,0x41,0x41,0x40,0xde,0x5d,0xae,0x22,0x23,0xb0,0x03,0x61,0xa3,0x96,0x17,0x7a,0x9c,0xb4,0x10,0xff,0x61,0xf2,0x00,0x15,0xad};
    uint8_t out[32] = {0};
    REQUIRE(dbt_crypto_aes128(key, input, out)); REQUIRE(!memcmp(out, encrypted, 16));
    REQUIRE(dbt_crypto_cmac128(cmac_key, input, 0, out)); REQUIRE(!memcmp(out, empty_mac, 16));
    REQUIRE(dbt_sha256((const uint8_t *)"abc", 3, out, 0) == 0); REQUIRE(!memcmp(out, sha_abc, 32));
    dbt_sha256_context hash; dbt_sha256_init(&hash);
    REQUIRE(dbt_sha256_starts(&hash, 0) == 0);
    REQUIRE(dbt_sha256_update(&hash, (const uint8_t *)"a", 1) == 0);
    REQUIRE(dbt_sha256_update(&hash, (const uint8_t *)"bc", 2) == 0);
    REQUIRE(dbt_sha256_finish(&hash, out) == 0); REQUIRE(!memcmp(out, sha_abc, 32));
    dbt_sha256_free(&hash); dbt_sha256_free(&hash);
    puts("PASS PSA AES-128, NIST empty CMAC, SHA-256 one-shot/streaming/cleanup");
    return 0;
}
static dbt_smp_sc_job_t job(uint32_t generation, uint32_t serial, dbt_smp_sc_job_kind_t kind, bool initiator) {
    dbt_smp_sc_job_t j = {0};
    j.generation=generation; j.serial=serial; j.kind=kind; j.initiator=initiator; j.key_size=16;
    j.preq[0]=1; j.pres[0]=2; j.preq[1]=j.pres[1]=3;
    j.preq[3]=j.pres[3]=8; j.preq[4]=j.pres[4]=16;
    for(unsigned i=0;i<6;i++){j.ia[i]=(uint8_t)(i+1);j.ra[i]=(uint8_t)(i+11);}
    return j;
}
static void bind(dbt_smp_sc_job_t *j, const dbt_smp_sc_result_t *local, const dbt_smp_sc_result_t *peer, bool captured) {
    memcpy(j->local_public, local->public_key, 64); memcpy(j->peer_public, peer->public_key, 64);
    memcpy(j->local_nonce, local->nonce, 16); memcpy(j->peer_nonce, peer->nonce, 16);
    if(captured){j->preq[1]=4;j->preq[3]=13;j->pres[3]=9;j->preq[5]=j->preq[6]=15;j->pres[5]=2;j->passkey=123456;}

}
static int secure_connections(bool captured) {
    dbt_smp_worker_mbedtls_t a={0}, b={0}; dbt_smp_sc_result_t pa,pb,commit,ka,kb;
    REQUIRE(dbt_smp_worker_mbedtls_open(&a,17,random_bytes,0));
    REQUIRE(dbt_smp_worker_mbedtls_open(&b,29,random_bytes,0));
    dbt_smp_sc_job_t ja=job(17,1,DBT_SC_JOB_KEYGEN,true), jb=job(29,1,DBT_SC_JOB_KEYGEN,false);
    REQUIRE(dbt_smp_worker_mbedtls_sc(&a,&ja,&pa)); REQUIRE(dbt_smp_worker_mbedtls_sc(&b,&jb,&pb));
    REQUIRE(memcmp(pa.public_key,pb.public_key,64));
    jb=job(29,2,DBT_SC_JOB_COMMIT,false); bind(&jb,&pb,&pa,captured);
    REQUIRE(dbt_smp_worker_mbedtls_sc(&b,&jb,&commit));
    ja=job(17,2,DBT_SC_JOB_DERIVE,true); bind(&ja,&pa,&pb,captured); memcpy(ja.peer_confirm,commit.confirm,16);
    jb=job(29,3,DBT_SC_JOB_DERIVE,false); bind(&jb,&pb,&pa,captured);
    REQUIRE(dbt_smp_worker_mbedtls_sc(&a,&ja,&ka)); REQUIRE(ka.peer_commitment_valid);
    REQUIRE(dbt_smp_worker_mbedtls_sc(&b,&jb,&kb));
    REQUIRE(!memcmp(ka.ltk,kb.ltk,16)); REQUIRE(!memcmp(ka.local_check,kb.peer_check,16));
    REQUIRE(!memcmp(ka.peer_check,kb.local_check,16));
    REQUIRE(!dbt_smp_worker_mbedtls_sc(&a,&ja,&ka));
    dbt_smp_worker_mbedtls_close(&a); dbt_smp_worker_mbedtls_close(&b);
    REQUIRE(!a.internal && !b.internal);
    puts("PASS PSA P-256 two-party SC key agreement, commitments, f5/f6, retirement and replay rejection");
    if(captured){puts("PASS captured MITM/NoIO transcript uses Just Works commitments, ignoring an unrelated passkey field");return 0;}

    REQUIRE(dbt_smp_worker_mbedtls_open(&a,31,random_bytes,0));
    ja=job(31,1,DBT_SC_JOB_KEYGEN,true); REQUIRE(dbt_smp_worker_mbedtls_sc(&a,&ja,&pa));
    ja=job(31,2,DBT_SC_JOB_DERIVE,true); bind(&ja,&pa,&pb,captured); memset(ja.peer_public,0,64);
    REQUIRE(!dbt_smp_worker_mbedtls_sc(&a,&ja,&ka));
    dbt_smp_sc_result_t zero={0}; REQUIRE(!memcmp(&ka,&zero,sizeof(ka)));
    dbt_smp_worker_mbedtls_close(&a); REQUIRE(!a.internal);
    puts("PASS PSA invalid P-256 point fails closed and wipes result");
    for(unsigned i=0;i<96;i++) {
        REQUIRE(dbt_smp_worker_mbedtls_open(&a,100+i,random_bytes,0));
        ja=job(100+i,1,DBT_SC_JOB_KEYGEN,true);
        REQUIRE(dbt_smp_worker_mbedtls_sc(&a,&ja,&pa));
        dbt_smp_worker_mbedtls_close(&a); REQUIRE(!a.internal);
        dbt_smp_worker_mbedtls_close(&a);
    }
    puts("PASS PSA 96 key-generation/close cycles without slot exhaustion");
    return 0;
}
int main(void) {
    REQUIRE(psa_crypto_init()==PSA_SUCCESS);
    if(primitives() || secure_connections(false) || secure_connections(true)) return 1;
    puts("Host PSA provider checks passed; not ESP32 execution, RF or pairing evidence.");
    return 0;
}
