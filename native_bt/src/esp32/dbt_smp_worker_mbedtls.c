#include "dbt_smp_worker_mbedtls.h"
#include "dbt_smp_crypto.h"
#include "mbedtls/aes.h"
#include "mbedtls/cipher.h"
#include "mbedtls/cmac.h"
#include "mbedtls/ecdh.h"
#include "mbedtls/ecp.h"
#include <stdlib.h>
#include <string.h>

typedef struct {
  uint32_t generation, last_serial;
  dbt_smp_worker_rng_fn rng;
  void *rng_context;
  mbedtls_ecp_keypair key;
  bool key_ready, oob_prepared, retired, keys_generated;
  uint8_t mode;
} worker_t;
static void retire(worker_t *p);
static void wipe(void *v, size_t n) {
  volatile uint8_t *p = v;
  while (n--)
    *p++ = 0;
}
static void reverse(uint8_t *out, const uint8_t *in, unsigned n) {
  for (unsigned i = 0; i < n; i++)
    out[i] = in[n - 1 - i];
}
static int random_cb(void *v, unsigned char *out, size_t n) {
  worker_t *w = v;
  return w->rng(w->rng_context, out, n) ? 0 : -1;
}
static bool aes(void *ctx, const uint8_t k[16], const uint8_t in[16], uint8_t out[16]) {
  (void)ctx;
  mbedtls_aes_context a;
  mbedtls_aes_init(&a);
  int rc = mbedtls_aes_setkey_enc(&a, k, 128);
  if (!rc)
    rc = mbedtls_aes_crypt_ecb(&a, MBEDTLS_AES_ENCRYPT, in, out);
  mbedtls_aes_free(&a);
  return rc == 0;
}
static bool cmac(void *ctx, const uint8_t k[16], const uint8_t *m, unsigned n, uint8_t out[16]) {
  (void)ctx;
  const mbedtls_cipher_info_t *ci = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_128_ECB);
  return ci && mbedtls_cipher_cmac(ci, k, 128, m, n, out) == 0;
}
static bool public_be(worker_t *w, uint8_t out[64]) {
  uint8_t p[65];
  size_t n = 0;
  int rc = mbedtls_ecp_point_write_binary(&w->key.MBEDTLS_PRIVATE(grp), &w->key.MBEDTLS_PRIVATE(Q),
                                          MBEDTLS_ECP_PF_UNCOMPRESSED, &n, p, sizeof(p));
  if (!rc && n == 65)
    memcpy(out, p + 1, 64);
  else
    rc = -1;
  wipe(p, sizeof(p));
  return rc == 0;
}
static bool generate(worker_t *w) {
  if (w->retired || w->key_ready)
    return false;
  if (mbedtls_ecp_gen_key(MBEDTLS_ECP_DP_SECP256R1, &w->key, random_cb, w))
    return false;
  w->key_ready = true;
  return true;
}
static bool ecdh(void *ctx, const uint8_t local[64], const uint8_t peer[64], uint8_t shared[32]) {
  static const uint8_t debug_public[64] = {
      0x20, 0xb0, 0x03, 0xd2, 0xf2, 0x97, 0xbe, 0x2c, 0x5e, 0x2c, 0x83, 0xa7, 0xe9, 0xf9, 0xa5, 0xb9,
      0xef, 0xf4, 0x91, 0x11, 0xac, 0xf4, 0xfd, 0xdb, 0xcc, 0x03, 0x01, 0x48, 0x0e, 0x35, 0x9d, 0xe6,
      0xdc, 0x80, 0x9c, 0x49, 0x65, 0x2a, 0xeb, 0x6d, 0x63, 0x32, 0x9a, 0xbf, 0x5a, 0x52, 0x15, 0x5c,
      0x76, 0x63, 0x45, 0xc2, 0x8f, 0xed, 0x30, 0x24, 0x74, 0x1c, 0x8e, 0xd0, 0x15, 0x89, 0xd2, 0x8b};
  worker_t *w = ctx;
  uint8_t actual[64], encoded[65];
  mbedtls_ecp_point q;
  mbedtls_mpi z;
  if (!w->key_ready || !public_be(w, actual) || memcmp(actual, local, 64) ||
      !memcmp(peer, debug_public, 64) || !memcmp(local, debug_public, 64)) {
    wipe(actual, sizeof(actual));
    return false;
  }
  encoded[0] = 4;
  memcpy(encoded + 1, peer, 64);
  mbedtls_ecp_point_init(&q);
  mbedtls_mpi_init(&z);
  int rc = mbedtls_ecp_point_read_binary(&w->key.MBEDTLS_PRIVATE(grp), &q, encoded, sizeof(encoded));
  if (!rc)
    rc = mbedtls_ecp_check_pubkey(&w->key.MBEDTLS_PRIVATE(grp), &q);
  if (!rc)
    rc = mbedtls_ecdh_compute_shared(&w->key.MBEDTLS_PRIVATE(grp), &z, &q, &w->key.MBEDTLS_PRIVATE(d),
                                     random_cb, w);
  if (!rc)
    rc = mbedtls_mpi_write_binary(&z, shared, 32);
  mbedtls_mpi_free(&z);
  mbedtls_ecp_point_free(&q);
  wipe(actual, sizeof(actual));
  wipe(encoded, sizeof(encoded));
  if (rc)
    wipe(shared, 32);
  return rc == 0;
}
bool dbt_smp_worker_mbedtls_open(dbt_smp_worker_mbedtls_t *w, uint32_t gen, dbt_smp_worker_rng_fn rng,
                                 void *ctx) {
  if (!w || !gen || !rng || w->internal)
    return false;
  worker_t *p = calloc(1, sizeof(*p));
  if (!p)
    return false;
  p->generation = gen;
  p->rng = rng;
  p->rng_context = ctx;
  mbedtls_ecp_keypair_init(&p->key);
  w->internal = p;
  return true;
}
void dbt_smp_worker_mbedtls_close(dbt_smp_worker_mbedtls_t *w) {
  if (!w || !w->internal)
    return;
  worker_t *p = w->internal;
  mbedtls_ecp_keypair_free(&p->key);
  wipe(p, sizeof(*p));
  free(p);
  w->internal = 0;
}
bool dbt_smp_worker_mbedtls_oob(dbt_smp_worker_mbedtls_t *w, uint8_t public_key[64], uint8_t random[16],
                                uint8_t confirm[16]) {
  if (!w || !w->internal || !public_key || !random || !confirm)
    return false;
  worker_t *p = w->internal;
  if (p->retired || p->mode || p->last_serial)
    return false;
  uint8_t pub[64], rbe[16], cbe[16];
  bool ok = generate(p) && public_be(p, pub) && p->rng(p->rng_context, random, 16);
  if (ok) {
    reverse(rbe, random, 16);
    ok = dbt_smp_f4(cmac, p, pub, pub, rbe, 0, cbe);
  }
  if (ok) {
    reverse(public_key, pub, 32);
    reverse(public_key + 32, pub + 32, 32);
    reverse(confirm, cbe, 16);
    p->oob_prepared = true;
    p->mode = 2;
  } else {
    wipe(public_key, 64);
    wipe(random, 16);
    wipe(confirm, 16);
    retire(p);
  }
  wipe(pub, sizeof(pub));
  wipe(rbe, 16);
  wipe(cbe, 16);
  return ok;
}
static bool claim(worker_t *p, uint32_t gen, uint32_t serial, uint8_t mode) {
  if (p->retired || (p->mode && p->mode != mode) || !gen || gen != p->generation || !serial ||
      serial <= p->last_serial)
    return false;
  p->mode = mode;
  p->last_serial = serial;
  return true;
}
static void retire(worker_t *p) {
  mbedtls_ecp_keypair_free(&p->key);
  mbedtls_ecp_keypair_init(&p->key);
  p->key_ready = false;
  p->oob_prepared = false;
  p->retired = true;
}
bool dbt_smp_worker_mbedtls_legacy(dbt_smp_worker_mbedtls_t *w, const dbt_smp_job_t *j,
                                   dbt_smp_job_result_t *r) {
  if (!w || !w->internal || !j || !r)
    return false;
  worker_t *p = w->internal;
  memset(r, 0, sizeof(*r));
  if (!claim(p, j->generation, j->serial, 1))
    return false;
  bool ok = false;
  if (j->kind == DBT_SMP_JOB_LOCAL) {
    ok = p->rng(p->rng_context, r->random, 16) &&
         dbt_smp_c1(aes, p, j->tk, r->random, j->preq, j->pres, j->iat, j->ia, j->rat, j->ra, r->confirm);
  } else if (j->kind == DBT_SMP_JOB_VERIFY) {
    ok =
        dbt_smp_c1(aes, p, j->tk, j->peer_random, j->preq, j->pres, j->iat, j->ia, j->rat, j->ra, r->confirm);
    if (ok)
      ok = dbt_smp_s1(aes, p, j->tk, j->initiator ? j->peer_random : j->local_random,
                      j->initiator ? j->local_random : j->peer_random, r->stk);
  }
  if (!ok) {
    wipe(r, sizeof(*r));
    retire(p);
  } else if (j->kind == DBT_SMP_JOB_VERIFY)
    retire(p);
  return ok;
}
bool dbt_smp_worker_mbedtls_sc(dbt_smp_worker_mbedtls_t *w, const dbt_smp_sc_job_t *j,
                               dbt_smp_sc_result_t *r) {
  if (!w || !w->internal || !j || !r)
    return false;
  worker_t *p = w->internal;
  memset(r, 0, sizeof(*r));
  if (!claim(p, j->generation, j->serial, 2))
    return false;
  uint8_t lp[64], pp[64], ln[16], pn[16], conf[16], ra[16] = {0}, rb[16] = {0};
  uint8_t a[7], b[7], ioa[3], iob[3], ltk[16], ea[16], eb[16];
  reverse(lp, j->local_public, 32);
  reverse(lp + 32, j->local_public + 32, 32);
  reverse(pp, j->peer_public, 32);
  reverse(pp + 32, j->peer_public + 32, 32);
  reverse(ln, j->local_nonce, 16);
  reverse(pn, j->peer_nonce, 16);
  reverse(conf, j->peer_confirm, 16);
  bool ok = false;
  if (j->kind == DBT_SC_JOB_KEYGEN) {
    bool oob = j->preq[2] && j->pres[2];
    ok = (oob ? p->oob_prepared : (!p->oob_prepared && generate(p))) && public_be(p, lp) &&
         p->rng(p->rng_context, r->nonce, 16);
    if (ok) {
      reverse(r->public_key, lp, 32);
      reverse(r->public_key + 32, lp + 32, 32);
    }
  } else if (j->kind == DBT_SC_JOB_COMMIT) {
    ok = dbt_smp_f4(cmac, p, lp, pp, ln, 0, conf);
    if (ok)
      reverse(r->confirm, conf, 16);
  } else if (j->kind == DBT_SC_JOB_ROUND_LOCAL) {
    uint8_t z = j->round < 20 ? (uint8_t)(0x80u | ((j->passkey >> j->round) & 1u)) : 0;
    ok = j->round < 20 && p->rng(p->rng_context, r->nonce, 16);
    if (ok) {
      reverse(ln, r->nonce, 16);
      ok = dbt_smp_f4(cmac, p, lp, pp, ln, z, conf);
    }
    if (ok)
      reverse(r->confirm, conf, 16);
  } else if (j->kind == DBT_SC_JOB_ROUND_VERIFY) {
    uint8_t z = j->round < 20 ? (uint8_t)(0x80u | ((j->passkey >> j->round) & 1u)) : 0;
    ok = j->round < 20;
    if (ok)
      r->peer_commitment_valid = dbt_smp_sc_confirm(cmac, p, pp, lp, pn, z, conf);
  } else if (j->kind == DBT_SC_JOB_OOB_VERIFY) {
    reverse(pn, j->peer_oob_random, 16);
    reverse(conf, j->peer_oob_confirm, 16);
    ok = true;
    r->peer_commitment_valid = dbt_smp_sc_confirm(cmac, p, pp, pp, pn, 0, conf);
  } else if (j->kind == DBT_SC_JOB_DERIVE) {
    bool oob = j->preq[2] && j->pres[2];
    bool mitm = ((j->preq[3] | j->pres[3]) & 4) != 0;
    bool numeric =
        mitm && !oob && (j->preq[1] == 1 || j->preq[1] == 4) && (j->pres[1] == 1 || j->pres[1] == 4);
    bool passkey = mitm && !oob && !numeric;
    if (oob) {
      reverse(ra, j->initiator ? j->local_oob_random : j->peer_oob_random, 16);
      reverse(rb, j->initiator ? j->peer_oob_random : j->local_oob_random, 16);
    } else if (passkey) {
      ra[13] = rb[13] = (uint8_t)(j->passkey >> 16);
      ra[14] = rb[14] = (uint8_t)(j->passkey >> 8);
      ra[15] = rb[15] = (uint8_t)j->passkey;
    }
    a[0] = j->iat;
    b[0] = j->rat;
    reverse(a + 1, j->ia, 6);
    reverse(b + 1, j->ra, 6);
    ioa[0] = j->preq[3];
    ioa[1] = j->preq[2];
    ioa[2] = j->preq[1];
    iob[0] = j->pres[3];
    iob[1] = j->pres[2];
    iob[2] = j->pres[1];
    const uint8_t *na = j->initiator ? ln : pn, *nb = j->initiator ? pn : ln;
    ok = dbt_smp_sc_derive(ecdh, cmac, p, lp, pp, na, nb, a, b, ioa, iob, ra, rb, j->key_size, ltk, ea, eb);
    if (ok && j->initiator && !oob && !passkey)
      r->peer_commitment_valid = dbt_smp_sc_confirm(cmac, p, pp, lp, pn, 0, conf);
    if (ok && numeric) {
      uint32_t val = 0;
      ok = dbt_smp_g2(cmac, p, j->initiator ? lp : pp, j->initiator ? pp : lp, na, nb, &val);
      r->numeric_value = val % 1000000u;
    }
    if (ok) {
      reverse(r->ltk, ltk, 16);
      reverse(r->local_check, j->initiator ? ea : eb, 16);
      reverse(r->peer_check, j->initiator ? eb : ea, 16);
    }
  }
  wipe(lp, sizeof(lp));
  wipe(pp, sizeof(pp));
  wipe(ln, 16);
  wipe(pn, 16);
  wipe(conf, 16);
  wipe(ra, 16);
  wipe(rb, 16);
  wipe(a, 7);
  wipe(b, 7);
  wipe(ioa, 3);
  wipe(iob, 3);
  wipe(ltk, 16);
  wipe(ea, 16);
  wipe(eb, 16);
  if (!ok) {
    wipe(r, sizeof(*r));
    retire(p);
  } else if (j->kind == DBT_SC_JOB_DERIVE)
    retire(p);
  return ok;
}
bool dbt_smp_worker_mbedtls_distribution(dbt_smp_worker_mbedtls_t *w, uint8_t mask, uint8_t type,
                                         const uint8_t identity[6], dbt_smp_dist_keys_t *out) {
  if (!w || !w->internal || !out || !mask || (mask & ~7) || type > 1 || !identity ||
      (type == 1 && (identity[5] & 0xc0) != 0xc0))
    return false;
  worker_t *p = w->internal;
  if (p->retired || p->keys_generated)
    return false;
  memset(out, 0, sizeof(*out));
  uint8_t ediv[2] = {0};
  bool ok = true;
  if (mask & 1) {
    ok = p->rng(p->rng_context, out->ltk, 16) && p->rng(p->rng_context, ediv, 2) &&
         p->rng(p->rng_context, out->rand, 8);
    if (ok)
      out->ediv = (uint16_t)(ediv[0] | ((uint16_t)ediv[1] << 8));
  }
  if (ok && (mask & 2)) {
    ok = p->rng(p->rng_context, out->irk, 16);
    out->identity_type = type;
    memcpy(out->identity, identity, 6);
  }
  if (ok && (mask & 4))
    ok = p->rng(p->rng_context, out->csrk, 16);
  wipe(ediv, sizeof(ediv));
  if (!ok) {
    wipe(out, sizeof(*out));
    retire(p);
    return false;
  }
  p->keys_generated = true;
  return true;
}
