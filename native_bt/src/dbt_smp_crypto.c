#include "dbt_smp_crypto.h"
#include <string.h>

static void wipe(void *v, unsigned n) {
  volatile uint8_t *p = v;
  while (n--)
    *p++ = 0;
}
static bool e(dbt_smp_aes128_fn aes, void *ctx, const uint8_t key[16], const uint8_t input[16],
              uint8_t out[16]) {
  uint8_t k[16], b[16], r[16];
  bool ok;
  for (unsigned i = 0; i < 16; i++) {
    k[i] = key[15 - i];
    b[i] = input[15 - i];
  }
  ok = aes(ctx, k, b, r);
  if (ok)
    for (unsigned i = 0; i < 16; i++)
      out[i] = r[15 - i];
  else
    memset(out, 0, 16);
  wipe(k, 16);
  wipe(b, 16);
  wipe(r, 16);
  return ok;
}
bool dbt_smp_ah_resolve(dbt_smp_aes128_fn aes,void *ctx,const uint8_t irk[16],
                        const uint8_t rpa[6],bool *matching) {
  if (!aes || !irk || !rpa || !matching || (rpa[5]&0xc0u)!=0x40u ||
      (rpa[3]==0 && rpa[4]==0 && (rpa[5]&0x3fu)==0) ||
      (rpa[3]==0xff && rpa[4]==0xff && (rpa[5]&0x3fu)==0x3f)) return false;
  uint8_t block[16]={0},result[16]; memcpy(block,rpa+3,3);
  bool ok=e(aes,ctx,irk,block,result);
  if (ok) *matching=((result[0]^rpa[0]) | (result[1]^rpa[1]) | (result[2]^rpa[2]))==0;
  wipe(block,sizeof(block)); wipe(result,sizeof(result)); return ok;
}
bool dbt_smp_c1(dbt_smp_aes128_fn aes, void *ctx, const uint8_t tk[16], const uint8_t r[16],
                const uint8_t preq[7], const uint8_t pres[7], uint8_t iat, const uint8_t ia[6], uint8_t rat,
                const uint8_t ra[6], uint8_t out[16]) {
  if (!aes || !tk || !r || !preq || !pres || !ia || !ra || !out || iat > 1 || rat > 1)
    return false;
  uint8_t p1[16], p2[16] = {0}, v[16];
  bool ok;
  p1[0] = iat;
  p1[1] = rat;
  memcpy(p1 + 2, preq, 7);
  memcpy(p1 + 9, pres, 7);
  memcpy(p2, ra, 6);
  memcpy(p2 + 6, ia, 6);
  for (unsigned i = 0; i < 16; i++)
    v[i] = r[i] ^ p1[i];
  ok = e(aes, ctx, tk, v, v);
  if (ok) {
    for (unsigned i = 0; i < 16; i++)
      v[i] ^= p2[i];
    ok = e(aes, ctx, tk, v, out);
  }
  if (!ok)
    memset(out, 0, 16);
  wipe(v, 16);
  wipe(p1, 16);
  wipe(p2, 16);
  return ok;
}
bool dbt_smp_s1(dbt_smp_aes128_fn aes, void *ctx, const uint8_t tk[16], const uint8_t rr[16],
                const uint8_t ir[16], uint8_t out[16]) {
  if (!aes || !tk || !rr || !ir || !out)
    return false;
  uint8_t v[16];
  memcpy(v, ir, 8);
  memcpy(v + 8, rr, 8);
  bool ok = e(aes, ctx, tk, v, out);
  wipe(v, 16);
  return ok;
}

bool dbt_smp_f4(dbt_smp_cmac_fn mac, void *ctx, const uint8_t u[32], const uint8_t v[32], const uint8_t x[16],
                uint8_t z, uint8_t out[16]) {
  if (!mac || !u || !v || !x || !out)
    return false;
  uint8_t m[65];
  memcpy(m, u, 32);
  memcpy(m + 32, v, 32);
  m[64] = z;
  bool ok = mac(ctx, x, m, sizeof(m), out);
  if (!ok)
    wipe(out, 16);
  wipe(m, sizeof(m));
  return ok;
}
bool dbt_smp_f5(dbt_smp_cmac_fn mac, void *ctx, const uint8_t w[32], const uint8_t n1[16],
                const uint8_t n2[16], const uint8_t a1[7], const uint8_t a2[7], uint8_t mk[16],
                uint8_t ltk[16]) {
  static const uint8_t salt[16] = {0x6c, 0x88, 0x83, 0x91, 0xaa, 0xf5, 0xa5, 0x38,
                                   0x60, 0x37, 0x0b, 0xdb, 0x5a, 0x60, 0x83, 0xbe};
  if (!mac || !w || !n1 || !n2 || !a1 || !a2 || !mk || !ltk || mk == ltk)
    return false;
  uint8_t t[16], m[53];
  m[0] = 0;
  memcpy(m + 1, "btle", 4);
  memcpy(m + 5, n1, 16);
  memcpy(m + 21, n2, 16);
  memcpy(m + 37, a1, 7);
  memcpy(m + 44, a2, 7);
  m[51] = 1;
  m[52] = 0;
  bool ok = mac(ctx, salt, w, 32, t);
  if (ok)
    ok = mac(ctx, t, m, sizeof(m), mk);
  if (ok) {
    m[0] = 1;
    ok = mac(ctx, t, m, sizeof(m), ltk);
  }
  if (!ok) {
    wipe(mk, 16);
    wipe(ltk, 16);
  }
  wipe(t, sizeof(t));
  wipe(m, sizeof(m));
  return ok;
}
bool dbt_smp_f6(dbt_smp_cmac_fn mac, void *ctx, const uint8_t w[16], const uint8_t n1[16],
                const uint8_t n2[16], const uint8_t r[16], const uint8_t io[3], const uint8_t a1[7],
                const uint8_t a2[7], uint8_t out[16]) {
  if (!mac || !w || !n1 || !n2 || !r || !io || !a1 || !a2 || !out)
    return false;
  uint8_t m[65];
  memcpy(m, n1, 16);
  memcpy(m + 16, n2, 16);
  memcpy(m + 32, r, 16);
  memcpy(m + 48, io, 3);
  memcpy(m + 51, a1, 7);
  memcpy(m + 58, a2, 7);
  bool ok = mac(ctx, w, m, sizeof(m), out);
  if (!ok)
    wipe(out, 16);
  wipe(m, sizeof(m));
  return ok;
}
bool dbt_smp_g2(dbt_smp_cmac_fn mac, void *ctx, const uint8_t u[32], const uint8_t v[32], const uint8_t x[16],
                const uint8_t y[16], uint32_t *out) {
  if (!mac || !u || !v || !x || !y || !out)
    return false;
  uint8_t m[80], tag[16];
  memcpy(m, u, 32);
  memcpy(m + 32, v, 32);
  memcpy(m + 64, y, 16);
  bool ok = mac(ctx, x, m, sizeof(m), tag);
  *out = ok ? ((uint32_t)tag[12] << 24) | ((uint32_t)tag[13] << 16) | ((uint32_t)tag[14] << 8) | tag[15] : 0;
  wipe(m, sizeof(m));
  wipe(tag, sizeof(tag));
  return ok;
}
bool dbt_smp_sc_derive(dbt_smp_ecdh_fn ecdh, dbt_smp_cmac_fn mac, void *ctx, const uint8_t local[64],
                       const uint8_t peer[64], const uint8_t na[16], const uint8_t nb[16], const uint8_t a[7],
                       const uint8_t b[7], const uint8_t ioa[3], const uint8_t iob[3], const uint8_t ra[16],
                       const uint8_t rb[16], uint8_t size, uint8_t ltk[16], uint8_t ea[16], uint8_t eb[16]) {
  if (!ecdh || !mac || !local || !peer || !na || !nb || !a || !b || !ioa || !iob || !ra || !rb || !ltk ||
      !ea || !eb || size < 7 || size > 16 || ltk == ea || ltk == eb || ea == eb || a[0] > 1 || b[0] > 1)
    return false;
  /* Equal X values include reflection and opposite points; reject both. */
  if (!memcmp(local, peer, 32)) {
    wipe(ltk, 16);
    wipe(ea, 16);
    wipe(eb, 16);
    return false;
  }
  uint8_t dh[32] = {0}, mk[16] = {0};
  bool ok = ecdh(ctx, local, peer, dh);
  if (ok)
    ok = dbt_smp_f5(mac, ctx, dh, na, nb, a, b, mk, ltk);
  if (ok)
    ok = dbt_smp_f6(mac, ctx, mk, na, nb, rb, ioa, a, b, ea);
  if (ok)
    ok = dbt_smp_f6(mac, ctx, mk, nb, na, ra, iob, b, a, eb);
  if (ok)
    wipe(ltk, 16 - size); /* most-significant bytes in BE representation */
  else {
    wipe(ltk, 16);
    wipe(ea, 16);
    wipe(eb, 16);
  }
  wipe(dh, sizeof(dh));
  wipe(mk, sizeof(mk));
  return ok;
}
bool dbt_smp_sc_confirm(dbt_smp_cmac_fn mac, void *ctx, const uint8_t u[32], const uint8_t v[32],
                        const uint8_t nonce[16], uint8_t z, const uint8_t expected[16]) {
  if (!mac || !u || !v || !nonce || !expected)
    return false;
  uint8_t actual[16];
  bool ok = dbt_smp_f4(mac, ctx, u, v, nonce, z, actual);
  unsigned difference = 0;
  for (unsigned i = 0; i < 16; i++)
    difference |= actual[i] ^ expected[i];
  wipe(actual, sizeof(actual));
  return ok && difference == 0;
}
