#include "dbt_smp_features.h"
#include <string.h>
#define TIMEOUT_MS 30000u
static void wipe(void *v, unsigned n) {
  volatile uint8_t *p = v;
  while (n--)
    *p++ = 0;
}
static void queue(dbt_smp_features_t *f, const uint8_t *p, uint8_t n, uint32_t now) {
  memcpy(f->tx, p, n);
  f->tx_size = n;
  f->deadline = now + TIMEOUT_MS;
}
static void fail(dbt_smp_features_t *f, uint8_t reason, bool send, uint32_t now) {
  wipe(f->tx, sizeof(f->tx));
  f->tx_size = 0;
  f->state = DBT_FEATURE_FAILED;
  if (f->guard && !f->result_reported) {
    dbt_smp_guard_failure(f->guard, now);
    f->result_reported = true;
  }
  if (send) {
    uint8_t p[2] = {5, reason};
    queue(f, p, 2, now);
  }
}
static bool io_works(uint8_t a, uint8_t b, bool sc) {
  if (sc && (a == 1 || a == 4) && (b == 1 || b == 4))
    return true;
  bool ak = a == 2 || a == 4, bk = b == 2 || b == 4;
  bool ad = a == 0 || a == 1 || a == 4, bd = b == 0 || b == 1 || b == 4;
  return (ak && bd) || (bk && ad) || (a == 2 && b == 2);
}
static uint8_t validate(const dbt_smp_features_t *f, const uint8_t req[7], const uint8_t rsp[7]) {
  if (req[0] != 1 || rsp[0] != 2 || req[1] > 4 || rsp[1] > 4 || req[2] > 1 || rsp[2] > 1 ||
      (req[3] & 3) > 1 || (rsp[3] & 3) > 1 || req[4] < 7 || rsp[4] < 7 || req[4] > 16 || rsp[4] > 16 ||
      ((req[5] | req[6] | rsp[5] | rsp[6]) & ~7) || (rsp[5] & ~req[5]) || (rsp[6] & ~req[6]))
    return 10;
  bool sc = (req[3] & 8) && (rsp[3] & 8), bond = (req[3] & 1) && (rsp[3] & 1);
  if (f->policy.require_sc && !sc)
    return 5;
  if (f->policy.bonding && !bond)
    return 3;
  if ((req[3] & 1) != (rsp[3] & 1))
    return 3;
  if ((req[5] || req[6] || rsp[5] || rsp[6]) && !bond)
    return 10;
  if (sc && ((rsp[5] | rsp[6]) & 1))
    return 10;
  if (!sc && bond && (!(rsp[5] & 1) || !(rsp[6] & 1)))
    return 3;
  if (req[2] != rsp[2])
    return 2;
  if ((req[3] | rsp[3]) & 4)
    if (!req[2] && !io_works(req[1], rsp[1], sc))
      return 3;
  if ((req[4] < rsp[4] ? req[4] : rsp[4]) < f->policy.min_key_size)
    return 6;
  return 0;
}
dbt_result_t dbt_smp_features_open(dbt_smp_features_t *f, uint16_t h, uint32_t gen, bool initiator,
                                   const dbt_smp_feature_policy_t *p) {
  if (!f || !gen || !p || p->io > 4 || p->min_key_size < 7 || p->max_key_size > 16 ||
      p->max_key_size < p->min_key_size || (p->require_sc && !p->support_sc) ||
      ((p->initiator_keys | p->responder_keys) & ~7) ||
      (!p->bonding && (p->initiator_keys || p->responder_keys)))
    return DBT_INVALID;
  memset(f, 0, sizeof(*f));
  f->used = true;
  f->handle = h;
  f->generation = gen;
  f->initiator = initiator;
  f->policy = *p;
  return DBT_OK;
}
void dbt_smp_features_close(dbt_smp_features_t *f) {
  if (f)
    wipe(f, sizeof(*f));
}
dbt_result_t dbt_smp_features_set_guard(dbt_smp_features_t *f, dbt_smp_guard_t *guard, uint32_t now) {
  if (!f || !f->used || !guard || !guard->base_ms)
    return DBT_INVALID;
  if (f->state != DBT_FEATURE_IDLE || f->guard)
    return DBT_BUSY;
  if (!dbt_smp_guard_allowed(guard, now))
    return DBT_BUSY;
  f->guard = guard;
  return DBT_OK;
}
dbt_result_t dbt_smp_features_report_result(dbt_smp_features_t *f, uint32_t gen, bool success, uint32_t now) {
  if (!f || !f->used || f->generation != gen || !f->guard || f->result_reported)
    return DBT_INVALID;
  if (success && (!f->claimed || f->state != DBT_FEATURE_READY))
    return DBT_INVALID;
  if (success)
    dbt_smp_guard_success(f->guard, now);
  else
    dbt_smp_guard_failure(f->guard, now);
  f->result_reported = true;
  return DBT_OK;
}
dbt_result_t dbt_smp_features_start(dbt_smp_features_t *f, uint32_t now) {
  if (!f || !f->used || !f->initiator || !f->guard)
    return DBT_INVALID;
  if (f->state != DBT_FEATURE_IDLE)
    return DBT_BUSY;
  uint8_t req[7] = {1,
                    f->policy.io,
                    f->policy.oob,
                    0,
                    f->policy.max_key_size,
                    f->policy.initiator_keys,
                    f->policy.responder_keys};
  req[3] = (f->policy.bonding ? 1 : 0) | (f->policy.require_mitm ? 4 : 0) | (f->policy.support_sc ? 8 : 0);
  memcpy(f->preq, req, 7);
  queue(f, req, 7, now);
  f->state = DBT_FEATURE_RESPONSE;
  return DBT_OK;
}
void dbt_smp_features_poll(dbt_smp_features_t *f, uint32_t now) {
  if (!f || !f->used || f->state == DBT_FEATURE_IDLE || f->state == DBT_FEATURE_READY ||
      f->state == DBT_FEATURE_FAILED || f->state == DBT_FEATURE_TIMEOUT)
    return;
  if ((int32_t)(now - f->deadline) >= 0) {
    wipe(f->tx, sizeof(f->tx));
    f->tx_size = 0;
    f->state = DBT_FEATURE_TIMEOUT;
    if (f->guard && !f->result_reported) {
      dbt_smp_guard_failure(f->guard, now);
      f->result_reported = true;
    }
  }
}
dbt_result_t dbt_smp_features_receive(dbt_smp_features_t *f, const uint8_t *p, uint8_t n, uint32_t now) {
  if (!f || !f->used || !p || !n)
    return DBT_INVALID;
  dbt_smp_features_poll(f, now);
  if (f->state == DBT_FEATURE_FAILED || f->state == DBT_FEATURE_TIMEOUT || f->state == DBT_FEATURE_READY)
    return DBT_BUSY;
  if (p[0] == 5 && n == 2) {
    fail(f, p[1], false, now);
    return DBT_OK;
  }
  if (f->tx_size)
    return DBT_BUSY;
  if (!f->guard)
    return DBT_INVALID;
  if (!f->initiator && f->state == DBT_FEATURE_IDLE && p[0] == 1 && n == 7) {
    memcpy(f->preq, p, 7);
    uint8_t rsp[7] = {2,
                      f->policy.io,
                      f->policy.oob,
                      0,
                      f->policy.max_key_size,
                      (uint8_t)(p[5] & f->policy.initiator_keys),
                      (uint8_t)(p[6] & f->policy.responder_keys)};
    rsp[3] = (f->policy.bonding ? 1 : 0) | (f->policy.require_mitm ? 4 : 0) | (f->policy.support_sc ? 8 : 0);
    if ((p[3] & 8) && (rsp[3] & 8)) {
      rsp[5] &= (uint8_t)~1;
      rsp[6] &= (uint8_t)~1;
    }
    uint8_t reason = validate(f, p, rsp);
    if (reason) {
      fail(f, reason, true, now);
      return DBT_OK;
    }
    memcpy(f->pres, rsp, 7);
    f->secure_connections = (p[3] & 8) && (rsp[3] & 8);
    queue(f, rsp, 7, now);
    f->state = DBT_FEATURE_SEND_RESPONSE;
    return DBT_OK;
  }
  if (f->initiator && f->state == DBT_FEATURE_RESPONSE && p[0] == 2 && n == 7) {
    uint8_t reason = validate(f, f->preq, p);
    if (reason) {
      fail(f, reason, true, now);
      return DBT_OK;
    }
    memcpy(f->pres, p, 7);
    f->secure_connections = (f->preq[3] & 8) && (p[3] & 8);
    f->state = DBT_FEATURE_READY;
    return DBT_OK;
  }
  fail(f, 10, true, now);
  return DBT_OK;
}
dbt_result_t dbt_smp_features_tx(const dbt_smp_features_t *f, const uint8_t **p, uint8_t *n) {
  if (!f || !f->used || !p || !n)
    return DBT_INVALID;
  if (!f->tx_size)
    return DBT_EMPTY;
  *p = f->tx;
  *n = f->tx_size;
  return DBT_OK;
}
void dbt_smp_features_tx_done(dbt_smp_features_t *f, bool admitted, uint32_t now) {
  if (!f || !f->used)
    return;
  dbt_smp_features_poll(f, now);
  if (!f->tx_size)
    return;
  wipe(f->tx, sizeof(f->tx));
  f->tx_size = 0;
  if (!admitted) {
    fail(f, 8, false, now);
    return;
  }
  if (f->state == DBT_FEATURE_SEND_RESPONSE)
    f->state = DBT_FEATURE_READY;
}
dbt_result_t dbt_smp_features_take(const dbt_smp_features_t *f, uint8_t req[7], uint8_t rsp[7], bool *sc) {
  if (!f || !f->used || !req || !rsp || !sc)
    return DBT_INVALID;
  if (f->state != DBT_FEATURE_READY)
    return DBT_BUSY;
  memcpy(req, f->preq, 7);
  memcpy(rsp, f->pres, 7);
  *sc = f->secure_connections;
  return DBT_OK;
}
dbt_result_t dbt_smp_features_open_sc(dbt_smp_features_t *f, dbt_smp_sc_t *s, uint8_t lt,
                                      const uint8_t local[6], uint8_t pt, const uint8_t peer[6],
                                      uint32_t now) {
  if (!f || !f->used || !s || f->state != DBT_FEATURE_READY || f->claimed)
    return DBT_INVALID;
  if (!f->secure_connections)
    return DBT_UNSUPPORTED;
  dbt_result_t rc = dbt_smp_sc_open(s, f->handle, f->generation, f->initiator ? 0 : 1, lt, local, pt, peer,
                                    f->preq, f->pres, f->policy.min_key_size, now);
  if (rc == DBT_OK)
    f->claimed = true;
  return rc;
}
dbt_result_t dbt_smp_features_resume_legacy(dbt_smp_features_t *f, dbt_smp_t *s, uint32_t now) {
  if (!f || !f->used || !s || !s->used || f->state != DBT_FEATURE_READY || f->claimed ||
      f->handle != s->handle || f->generation != s->generation || f->initiator != s->initiator)
    return DBT_INVALID;
  if (f->secure_connections)
    return DBT_UNSUPPORTED;
  dbt_result_t rc = dbt_smp_resume_features(s, f->preq, f->pres, now);
  if (rc == DBT_OK)
    f->claimed = true;
  return rc;
}
