#include "dbt_smp.h"
#include <string.h>
#define SMP_MS 30000u
static void wipe(void *v, unsigned n) {
  volatile uint8_t *p = v;
  while (n--)
    *p++ = 0;
}
static bool equal(const uint8_t *a, const uint8_t *b, unsigned n) {
  unsigned d = 0;
  for (unsigned i = 0; i < n; i++)
    d |= a[i] ^ b[i];
  return d == 0;
}
static bool expired(uint32_t now, uint32_t deadline) { return (int32_t)(now - deadline) >= 0; }
static void secrets(dbt_smp_t *s) {
  wipe(s->random, 16);
  wipe(s->peer_random, 16);
  wipe(s->peer_confirm, 16);
  wipe(s->stk, 16);
  wipe(s->tk, 16);
  wipe(&s->job, sizeof(s->job));
  s->job_pending = false;
  s->authenticated = false;
  s->encryption_verified = false;
  dbt_smp_bond_close(&s->bond);
  dbt_smp_dist_close(&s->dist);
  s->bond_committed = false;
  wipe(&s->local_distribution, sizeof(s->local_distribution));
  wipe(&s->peer_distribution, sizeof(s->peer_distribution));
}
static void queue(dbt_smp_t *s, const uint8_t *data, uint8_t len, uint32_t now) {
  memcpy(s->tx, data, len);
  s->tx_size = len;
  s->deadline = now + SMP_MS;
}
static void failure(dbt_smp_t *s, uint8_t reason, uint32_t now, bool send) {
  secrets(s);
  s->reason = reason;
  s->state = DBT_SMP_FAILED;
  s->key_taken = false;
  wipe(s->tx, sizeof(s->tx));
  s->tx_size = 0;
  if (send) {
    uint8_t p[2] = {5, reason};
    queue(s, p, 2, now);
  }
}
static void job(dbt_smp_t *s, dbt_smp_job_kind_t kind) {
  wipe(&s->job, sizeof(s->job));
  s->job.kind = kind;
  s->job.generation = s->generation;
  s->job.serial = ++s->serial;
  s->job.initiator = s->initiator;
  memcpy(s->job.preq, s->preq, 7);
  memcpy(s->job.pres, s->pres, 7);
  memcpy(s->job.ia, s->ia, 6);
  memcpy(s->job.ra, s->ra, 6);
  s->job.iat = s->iat;
  s->job.rat = s->rat;
  memcpy(s->job.local_random, s->random, 16);
  memcpy(s->job.peer_random, s->peer_random, 16);
  memcpy(s->job.tk, s->tk, 16);
  s->job_pending = true;
}
dbt_result_t dbt_smp_open(dbt_smp_t *s, uint16_t handle, uint32_t gen, uint8_t role, uint8_t lt,
                          const uint8_t local[6], uint8_t pt, const uint8_t peer[6], uint8_t min,
                          bool allow) {
  if (!s || !local || !peer || !gen || role > 1 || lt > 1 || pt > 1 || min < 7 || min > 16)
    return DBT_INVALID;
  memset(s, 0, sizeof(*s));
  s->used = true;
  s->handle = handle;
  s->generation = gen;
  s->initiator = role == 0;
  s->min_key_size = min;
  s->max_key_size = 16;
  s->allow_just_works = allow;
  s->iat = s->initiator ? lt : pt;
  s->rat = s->initiator ? pt : lt;
  memcpy(s->ia, s->initiator ? local : peer, 6);
  memcpy(s->ra, s->initiator ? peer : local, 6);
  return DBT_OK;
}
void dbt_smp_close(dbt_smp_t *s) {
  if (s)
    wipe(s, sizeof(*s));
}
dbt_result_t dbt_smp_set_oob(dbt_smp_t *s, const uint8_t tk[16], bool authenticated) {
  if (!s || !s->used || !tk)
    return DBT_INVALID;
  if (s->state != DBT_SMP_IDLE || s->have_passkey || s->bonding)
    return DBT_BUSY;
  memcpy(s->tk, tk, 16);
  s->have_oob = true;
  s->oob_authenticated = authenticated;
  return DBT_OK;
}
dbt_result_t dbt_smp_set_passkey(dbt_smp_t *s, uint32_t passkey, uint8_t io) {
  if (!s || !s->used || passkey > 999999u || io > 4 || io == 3)
    return DBT_INVALID;
  if (s->state != DBT_SMP_IDLE || s->have_oob || s->bonding)
    return DBT_BUSY;
  wipe(s->tk, 16);
  s->tk[0] = (uint8_t)passkey;
  s->tk[1] = (uint8_t)(passkey >> 8);
  s->tk[2] = (uint8_t)(passkey >> 16);
  s->have_passkey = true;
  s->io_capability = io;
  return DBT_OK;
}
dbt_result_t dbt_smp_set_max_key_size(dbt_smp_t *s, uint8_t max) {
  if (!s || !s->used || max < 7 || max > 16 || max < s->min_key_size)
    return DBT_INVALID;
  if (s->state != DBT_SMP_IDLE || s->bonding)
    return DBT_BUSY;
  s->max_key_size = max;
  return DBT_OK;
}
dbt_result_t dbt_smp_set_bonding(dbt_smp_t *s, uint8_t lm, uint8_t pm, const dbt_smp_dist_keys_t *local,
                                 bool existing, bool old_sc, bool old_auth, uint8_t old_size) {
  if (!s || !s->used || !local || (lm & ~7) || (pm & ~7) || !(lm & 1) || !(pm & 1) ||
      (existing && (old_size < 7 || old_size > 16)))
    return DBT_INVALID;
  if (s->state != DBT_SMP_IDLE || s->bonding)
    return DBT_BUSY;
  uint8_t peer_type = s->initiator ? s->rat : s->iat;
  const uint8_t *peer = s->initiator ? s->ra : s->ia;
  if (peer_type == 1 && (peer[5] & 0xc0) != 0xc0)
    return DBT_UNSUPPORTED;
  if ((lm & 2) &&
      (local->identity_type > 1 || (local->identity_type == 1 && (local->identity[5] & 0xc0) != 0xc0)))
    return DBT_INVALID;
  bool auth = s->have_passkey || (s->have_oob && s->oob_authenticated);
  if (existing && (old_sc || (old_auth && !auth) || old_size > s->min_key_size))
    return DBT_UNSUPPORTED;
  s->bonding = true;
  s->local_mask = lm;
  s->peer_mask = pm;
  s->local_distribution = *local;
  dbt_smp_bond_init(&s->bond, s->generation);
  return DBT_OK;
}
dbt_result_t dbt_smp_start(dbt_smp_t *s, uint32_t now) {
  if (!s || !s->used || !s->initiator)
    return DBT_INVALID;
  if (s->state != DBT_SMP_IDLE)
    return DBT_BUSY;
  if (!s->allow_just_works && !s->have_oob && !s->have_passkey)
    return DBT_UNSUPPORTED;
  uint8_t p[7] = {1, 3, 0, 0, 16, 0, 0};
  p[1] = s->have_passkey ? s->io_capability : 3;
  p[2] = s->have_oob;
  p[3] = (s->oob_authenticated || s->have_passkey) ? 4 : 0;
  p[4] = s->max_key_size;
  if (s->bonding) {
    p[3] |= 1;
    p[5] = s->local_mask;
    p[6] = s->peer_mask;
  }
  memcpy(s->preq, p, 7);
  s->state = DBT_SMP_FEATURES;
  queue(s, p, 7, now);
  return DBT_OK;
}
static uint8_t features(dbt_smp_t *s, const uint8_t *p) {
  if (p[1] > 4 || p[2] > 1 || p[4] < 7 || p[4] > 16 || (p[3] & 3) > 1 || (p[5] & 0xf0) || (p[6] & 0xf0))
    return 10;
  if (s->have_oob && !p[2])
    return 2;
  if (s->have_passkey) {
    uint8_t l = s->io_capability, r = p[1];
    bool lk = l == 2 || l == 4, rk = r == 2 || r == 4;
    bool ld = l == 0 || l == 1 || l == 4, rd = r == 0 || r == 1 || r == 4;
    if (!((lk && rd) || (rk && ld) || (l == 2 && r == 2)))
      return 3;
  } else if ((p[3] & 4) && !(s->have_oob && s->oob_authenticated))
    return 3;
  if (!s->have_oob && !s->have_passkey && !s->allow_just_works)
    return 3;
  if (p[4] < s->min_key_size)
    return 6;
  /* No bonding, key distribution, OOB, MITM, SC, CTKD, or signing. */
  if (s->bonding) {
    if ((p[3] & 3) != 1)
      return 3;
    if (s->initiator) {
      if ((p[5] & ~s->preq[5]) || (p[6] & ~s->preq[6]))
        return 10;
      s->local_mask = p[5];
      s->peer_mask = p[6];
    } else {
      s->peer_mask &= p[5];
      s->local_mask &= p[6];
    }
    if (!(s->local_mask & 1) || !(s->peer_mask & 1))
      return 3;
  } else if (s->initiator && ((p[3] & 3) || p[5] || p[6]))
    return 10;
  s->key_size = p[4] < s->max_key_size ? p[4] : s->max_key_size;
  return 0;
}
dbt_result_t dbt_smp_resume_features(dbt_smp_t *s, const uint8_t req[7], const uint8_t rsp[7], uint32_t now) {
  if (!s || !s->used || !req || !rsp)
    return DBT_INVALID;
  if (s->state != DBT_SMP_IDLE)
    return DBT_BUSY;
  const uint8_t *mine = s->initiator ? req : rsp, *peer = s->initiator ? rsp : req;
  uint8_t expected_io = s->have_passkey ? s->io_capability : 3;
  bool auth = s->have_passkey || (s->have_oob && s->oob_authenticated);
  if (req[0] != 1 || rsp[0] != 2 || ((req[3] & 8) && (rsp[3] & 8)) || mine[1] != expected_io ||
      mine[2] != (uint8_t)s->have_oob || ((mine[3] & 4) != 0) != auth || ((mine[3] & 1) != 0) != s->bonding ||
      mine[4] != s->max_key_size || (rsp[5] & ~req[5]) || (rsp[6] & ~req[6]))
    return DBT_INVALID;
  if ((!s->bonding && (req[5] || req[6] || rsp[5] || rsp[6])) ||
      (s->bonding && s->initiator && ((req[5] & ~s->local_mask) || (req[6] & ~s->peer_mask))))
    return DBT_INVALID;
  uint8_t lm = s->local_mask, pm = s->peer_mask;
  memcpy(s->preq, req, 7);
  uint8_t why = features(s, peer);
  if (why || (s->bonding && (rsp[5] != (s->initiator ? s->local_mask : s->peer_mask) ||
                             rsp[6] != (s->initiator ? s->peer_mask : s->local_mask)))) {
    s->local_mask = lm;
    s->peer_mask = pm;
    wipe(s->preq, 7);
    return DBT_UNSUPPORTED;
  }
  memcpy(s->pres, rsp, 7);
  s->authenticated = auth;
  s->deadline = now + SMP_MS;
  s->state = DBT_SMP_LOCAL_CRYPTO;
  job(s, DBT_SMP_JOB_LOCAL);
  return DBT_OK;
}
dbt_result_t dbt_smp_receive(dbt_smp_t *s, const uint8_t *p, uint16_t n, uint32_t now) {
  if (!s || !s->used || !p || !n)
    return DBT_INVALID;
  dbt_smp_poll(s, now);
  if (s->state == DBT_SMP_TIMED_OUT || s->state == DBT_SMP_FAILED)
    return DBT_BUSY;
  if (p[0] == 5 && n == 2) {
    failure(s, p[1], now, false);
    return DBT_OK;
  }
  if (s->tx_size)
    return DBT_BUSY;
  if (!s->initiator && s->state == DBT_SMP_IDLE && p[0] == 1 && n == 7) {
    uint8_t why = features(s, p);
    if (why) {
      failure(s, why, now, true);
      return DBT_OK;
    }
    memcpy(s->preq, p, 7);
    uint8_t resp[7] = {2, 3, 0, 0, 16, 0, 0};
    resp[1] = s->have_passkey ? s->io_capability : 3;
    resp[2] = s->have_oob;
    resp[3] = (s->oob_authenticated || s->have_passkey) ? 4 : 0;
    resp[4] = s->max_key_size;
    if (s->bonding) {
      resp[3] |= 1;
      resp[5] = s->peer_mask;
      resp[6] = s->local_mask;
    }
    s->authenticated = s->have_passkey || (s->have_oob && s->oob_authenticated);
    memcpy(s->pres, resp, 7);
    queue(s, resp, 7, now);
    s->state = DBT_SMP_LOCAL_CRYPTO;
    job(s, DBT_SMP_JOB_LOCAL);
    return DBT_OK;
  }
  if (s->initiator && s->state == DBT_SMP_FEATURES && p[0] == 2 && n == 7) {
    uint8_t why = features(s, p);
    if (why) {
      failure(s, why, now, true);
      return DBT_OK;
    }
    s->authenticated = s->have_passkey || (s->have_oob && s->oob_authenticated);
    memcpy(s->pres, p, 7);
    s->state = DBT_SMP_LOCAL_CRYPTO;
    job(s, DBT_SMP_JOB_LOCAL);
    return DBT_OK;
  }
  if (!s->initiator && s->state == DBT_SMP_LOCAL_CRYPTO && !s->peer_confirmed && p[0] == 3 && n == 17) {
    memcpy(s->peer_confirm, p + 1, 16);
    s->peer_confirmed = true;
    return DBT_OK;
  }
  if (s->state == DBT_SMP_CONFIRM && p[0] == 3 && n == 17) {
    memcpy(s->peer_confirm, p + 1, 16);
    s->peer_confirmed = true;
    if (s->initiator) {
      uint8_t b[17] = {4};
      memcpy(b + 1, s->random, 16);
      queue(s, b, 17, now);
    } else { /* local confirm is already computed and held in job result buffer */
      uint8_t b[17] = {3};
      memcpy(b + 1, s->job.local_random, 16);
      queue(s, b, 17, now);
      wipe(s->job.local_random, 16);
    }
    s->state = DBT_SMP_RANDOM;
    return DBT_OK;
  }
  if (s->state == DBT_SMP_RANDOM && p[0] == 4 && n == 17) {
    if (equal(p + 1, s->random, 16)) {
      failure(s, 4, now, true);
      return DBT_OK;
    }
    memcpy(s->peer_random, p + 1, 16);
    s->state = DBT_SMP_VERIFY;
    job(s, DBT_SMP_JOB_VERIFY);
    return DBT_OK;
  }
  failure(s, (p[0] >= 1 && p[0] <= 14) ? 10 : 7, now, true);
  return DBT_OK;
}
void dbt_smp_poll(dbt_smp_t *s, uint32_t now) {
  if (!s || !s->used)
    return;
  if (s->state != DBT_SMP_IDLE && s->state != DBT_SMP_FAILED && s->state != DBT_SMP_TIMED_OUT &&
      s->state != DBT_SMP_SECURE && s->state != DBT_SMP_BOND_PENDING && expired(now, s->deadline)) {
    secrets(s);
    s->tx_size = 0;
    s->key_taken = false;
    s->state = DBT_SMP_TIMED_OUT;
  }
}
dbt_result_t dbt_smp_tx(const dbt_smp_t *s, const uint8_t **p, uint8_t *n) {
  if (!s || !p || !n || !s->used)
    return DBT_INVALID;
  if (!s->tx_size)
    return DBT_EMPTY;
  *p = s->tx;
  *n = s->tx_size;
  return DBT_OK;
}
void dbt_smp_tx_done(dbt_smp_t *s, bool ok) {
  if (!s || !s->tx_size)
    return;
  wipe(s->tx, sizeof(s->tx));
  s->tx_size = 0;
  if (!ok) {
    secrets(s);
    s->state = DBT_SMP_FAILED;
    s->key_taken = false;
  }
}
dbt_result_t dbt_smp_job(const dbt_smp_t *s, dbt_smp_job_t *out) {
  if (!s || !out || !s->used)
    return DBT_INVALID;
  if (!s->job_pending)
    return DBT_EMPTY;
  *out = s->job;
  return DBT_OK;
}
dbt_result_t dbt_smp_complete(dbt_smp_t *s, uint32_t gen, uint32_t serial, bool ok,
                              const dbt_smp_job_result_t *r, uint32_t now) {
  if (!s || !s->used || (ok && !r))
    return DBT_INVALID;
  dbt_smp_poll(s, now);
  if (!s->job_pending || s->generation != gen || s->job.serial != serial)
    return DBT_INVALID;
  if (!ok) {
    failure(s, 8, now, true);
    return DBT_OK;
  }
  dbt_smp_job_kind_t kind = s->job.kind;
  s->job_pending = false;
  wipe(&s->job, sizeof(s->job));
  if (kind == DBT_SMP_JOB_LOCAL) {
    memcpy(s->random, r->random, 16);
    if (s->initiator || s->peer_confirmed) {
      if (s->tx_size) {
        failure(s, 8, now, false);
        return DBT_BUSY;
      }
      uint8_t b[17] = {3};
      memcpy(b + 1, r->confirm, 16);
      queue(s, b, 17, now);
      s->state = s->initiator ? DBT_SMP_CONFIRM : DBT_SMP_RANDOM;
    } else {
      memcpy(s->job.local_random, r->confirm, 16);
      s->state = DBT_SMP_CONFIRM;
    }
  } else if (kind == DBT_SMP_JOB_VERIFY) {
    if (!equal(r->confirm, s->peer_confirm, 16)) {
      failure(s, 4, now, true);
      return DBT_OK;
    }
    memcpy(s->stk, r->stk, 16);
    for (unsigned i = s->key_size; i < 16; i++)
      s->stk[i] = 0;
    if (!s->initiator) {
      uint8_t b[17] = {4};
      memcpy(b + 1, s->random, 16);
      queue(s, b, 17, now);
    }
    wipe(s->random, 16);
    wipe(s->peer_random, 16);
    wipe(s->peer_confirm, 16);
    wipe(s->tk, 16);
    s->state = DBT_SMP_ENCRYPTION;
  } else
    return DBT_INVALID;
  return DBT_OK;
}
dbt_result_t dbt_smp_take_stk(dbt_smp_t *s, uint8_t out[16], uint8_t *size) {
  if (!s || !out || !size || !s->used)
    return DBT_INVALID;
  if (s->state != DBT_SMP_ENCRYPTION || s->key_taken || s->tx_size)
    return DBT_BUSY;
  memcpy(out, s->stk, 16);
  *size = s->key_size;
  wipe(s->stk, 16);
  s->key_taken = true;
  return DBT_OK;
}
dbt_result_t dbt_smp_encryption(dbt_smp_t *s, uint32_t gen, bool encrypted) {
  if (!s || !s->used || s->generation != gen)
    return DBT_INVALID;
  if (!encrypted) {
    secrets(s);
    s->key_taken = false;
    s->state = DBT_SMP_FAILED;
    return DBT_OK;
  }
  if (s->state != DBT_SMP_ENCRYPTION || !s->key_taken)
    return DBT_INVALID;
  s->encryption_verified = true;
  if (s->bonding) {
    for (unsigned i = s->key_size; i < 16; i++)
      s->local_distribution.ltk[i] = 0;
    dbt_result_t rc = dbt_smp_dist_open(&s->dist, gen, s->initiator, false, s->key_size, s->local_mask,
                                        s->peer_mask, true, &s->local_distribution);
    if (rc != DBT_OK) {
      failure(s, 8, 0, false);
      return rc;
    }
    s->state = DBT_SMP_DISTRIBUTION;
  } else
    s->state = DBT_SMP_SECURE;
  return DBT_OK;
}
static void finish_distribution(dbt_smp_t *s) {
  dbt_smp_dist_keys_t peer;
  memset(&peer, 0, sizeof(peer));
  if (dbt_smp_dist_take(&s->dist, &peer) != DBT_OK)
    return;
  s->peer_final_received = s->dist.peer_final_received;
  s->final_ll_acked = s->dist.final_ll_acked;
  s->hci_buffers_completed = s->dist.hci_buffers_completed;
  s->local_pdus_admitted = s->dist.local_pdus_admitted;
  s->peer_distribution = peer;
  wipe(&peer, sizeof(peer));
  dbt_smp_dist_close(&s->dist);
  dbt_smp_bond_record_t record;
  memset(&record, 0, sizeof(record));
  record.identity_type = s->initiator ? s->rat : s->iat;
  memcpy(record.identity, s->initiator ? s->ra : s->ia, 6);
  record.key_size = s->key_size;
  record.authenticated = s->authenticated;
  record.have_peer_ltk = true;
  memcpy(record.ltk, s->peer_distribution.ltk, 16);
  memcpy(record.rand, s->peer_distribution.rand, 8);
  record.ediv = s->peer_distribution.ediv;
  record.have_local_ltk = true;
  memcpy(record.local_ltk, s->local_distribution.ltk, 16);
  memcpy(record.local_rand, s->local_distribution.rand, 8);
  record.local_ediv = s->local_distribution.ediv;
  if (s->local_mask & 2) {
    record.have_local_irk = true;
    memcpy(record.local_irk, s->local_distribution.irk, 16);
  }
  if (s->peer_mask & 2) {
    record.candidate_present = true;
    record.candidate_type = s->peer_distribution.identity_type;
    memcpy(record.candidate_identity, s->peer_distribution.identity, 6);
    memcpy(record.candidate_irk, s->peer_distribution.irk, 16);
  }
  if (s->local_mask & 4) {
    record.have_local_csrk = true;
    memcpy(record.local_csrk, s->local_distribution.csrk, 16);
  }
  if (s->peer_mask & 4) {
    record.have_peer_csrk = true;
    memcpy(record.peer_csrk, s->peer_distribution.csrk, 16);
  }
  dbt_result_t rc =
      dbt_smp_bond_begin(&s->bond, DBT_SMP_BOND_STORE, record.identity_type, record.identity, &record);
  wipe(&record, sizeof(record));
  wipe(&s->local_distribution, sizeof(s->local_distribution));
  wipe(&s->peer_distribution, sizeof(s->peer_distribution));
  if (rc != DBT_OK) {
    failure(s, 8, 0, false);
    return;
  }
  s->state = DBT_SMP_BOND_PENDING;
}
dbt_result_t dbt_smp_dist_tx_legacy(dbt_smp_t *s, const uint8_t **p, uint8_t *n, uint32_t now) {
  if (!s || !s->used)
    return DBT_INVALID;
  dbt_smp_poll(s, now);
  if (s->state != DBT_SMP_DISTRIBUTION)
    return DBT_BUSY;
  bool fresh = s->dist.tx_size == 0;
  dbt_result_t rc = dbt_smp_dist_tx(&s->dist, p, n);
  if (rc == DBT_OK && fresh)
    s->deadline = now + SMP_MS;
  return rc;
}
dbt_result_t dbt_smp_dist_admitted_legacy(dbt_smp_t *s, uint32_t gen, uint32_t serial, bool admitted,
                                          uint32_t now) {
  if (!s || !s->used)
    return DBT_INVALID;
  dbt_smp_poll(s, now);
  if (s->state != DBT_SMP_DISTRIBUTION)
    return DBT_BUSY;
  dbt_result_t rc = dbt_smp_dist_tx_admitted(&s->dist, gen, serial, admitted);
  if (s->dist.failed) {
    failure(s, 8, now, false);
    return rc;
  }
  finish_distribution(s);
  return rc;
}
dbt_result_t dbt_smp_dist_final_ack_legacy(dbt_smp_t *s, uint32_t generation, uint32_t serial,
                                           bool acknowledged, uint32_t now) {
  if (!s || !s->used)
    return DBT_INVALID;
  dbt_smp_poll(s, now);
  if (s->state != DBT_SMP_DISTRIBUTION)
    return DBT_BUSY;
  dbt_result_t rc = dbt_smp_dist_final_ack(&s->dist, generation, serial, acknowledged);
  if (s->dist.failed) {
    failure(s, 8, now, false);
    return rc;
  }
  finish_distribution(s);
  return rc;
}
dbt_result_t dbt_smp_dist_receive_legacy(dbt_smp_t *s, const uint8_t *p, uint8_t n, uint32_t now) {
  if (!s || !s->used)
    return DBT_INVALID;
  dbt_smp_poll(s, now);
  if (s->state != DBT_SMP_DISTRIBUTION)
    return DBT_BUSY;
  dbt_result_t rc = dbt_smp_dist_receive(&s->dist, p, n);
  if (s->dist.failed) {
    failure(s, 10, now, true);
    return rc;
  }
  finish_distribution(s);
  return rc;
}
dbt_result_t dbt_smp_bond_job_legacy(const dbt_smp_t *s, dbt_smp_bond_job_t *out) {
  if (!s || !s->used || s->state != DBT_SMP_BOND_PENDING)
    return DBT_INVALID;
  return dbt_smp_bond_job(&s->bond, out);
}
dbt_result_t dbt_smp_bond_complete_legacy(dbt_smp_t *s, uint32_t gen, uint32_t serial, bool committed) {
  if (!s || !s->used || s->state != DBT_SMP_BOND_PENDING)
    return DBT_INVALID;
  dbt_result_t rc = dbt_smp_bond_complete(&s->bond, gen, serial, committed, 0);
  if (rc != DBT_OK) {
    if (gen == s->generation && !s->bond.pending)
      failure(s, 8, 0, false);
    return rc;
  }
  s->bond_committed = true;
  s->state = DBT_SMP_SECURE;
  return DBT_OK;
}
dbt_result_t dbt_smp_bond_cancel_legacy(dbt_smp_t *s, uint32_t gen, uint32_t serial) {
  if (!s || !s->used || s->state != DBT_SMP_BOND_PENDING)
    return DBT_INVALID;
  dbt_result_t rc = dbt_smp_bond_cancel(&s->bond, gen, serial);
  if (rc == DBT_OK)
    failure(s, 8, 0, false);
  return rc;
}
