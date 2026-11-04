#include "dbt_smp_sc.h"
#include <string.h>
#define TIMEOUT_MS 30000u
static void wipe(void *v, unsigned n) {
  volatile uint8_t *p = v;
  while (n--)
    *p++ = 0;
}
static bool same(const uint8_t *a, const uint8_t *b, unsigned n) {
  unsigned d = 0;
  for (unsigned i = 0; i < n; i++)
    d |= a[i] ^ b[i];
  return d == 0;
}
static void clear_secrets(dbt_smp_sc_t *s) {
  wipe(s->local_nonce, 16);
  wipe(s->peer_nonce, 16);
  wipe(s->peer_confirm, 16);
  wipe(s->ltk, 16);
  wipe(s->local_check, 16);
  wipe(s->peer_check, 16);
  wipe(s->bond_ltk, 16);
  dbt_smp_bond_close(&s->bond);
  s->bond_committed = false;
  wipe(s->local_csrk, 16);
  wipe(s->peer_csrk, 16);
  dbt_smp_dist_close(&s->dist);
  wipe(&s->local_distribution, sizeof(s->local_distribution));
  wipe(&s->peer_distribution, sizeof(s->peer_distribution));
  s->authenticated = false;
  s->encryption_verified = false;
  s->numeric_value = 0;
  wipe(&s->passkey, sizeof(s->passkey));
  s->passkey_ready = false;
  wipe(s->local_oob_public, 64);
  wipe(s->local_oob_random, 16);
  wipe(s->peer_oob_random, 16);
  wipe(s->peer_oob_confirm, 16);
  s->oob_ready = false;
  wipe(&s->job, sizeof(s->job));
  s->job_pending = false;
  s->key_taken = false;
}
static void queue(dbt_smp_sc_t *s, uint8_t code, const uint8_t *value, uint8_t n, uint32_t now) {
  s->tx[0] = code;
  if (n)
    memcpy(s->tx + 1, value, n);
  s->tx_size = n + 1;
  s->deadline = now + TIMEOUT_MS;
}
static void fail(dbt_smp_sc_t *s, uint8_t reason, bool send, uint32_t now) {
  clear_secrets(s);
  wipe(s->tx, sizeof(s->tx));
  s->tx_size = 0;
  s->state = DBT_SC_FAILED;
  if (send)
    queue(s, 5, &reason, 1, now);
}
static void job(dbt_smp_sc_t *s, dbt_smp_sc_job_kind_t kind) {
  dbt_smp_sc_job_t *j = &s->job;
  wipe(j, sizeof(*j));
  j->kind = kind;
  j->generation = s->generation;
  j->serial = ++s->serial;
  if (!j->serial)
    j->serial = ++s->serial;
  j->initiator = s->initiator;
  j->key_size = s->key_size;
  memcpy(j->preq, s->preq, 7);
  memcpy(j->pres, s->pres, 7);
  j->iat = s->iat;
  j->rat = s->rat;
  memcpy(j->ia, s->ia, 6);
  memcpy(j->ra, s->ra, 6);
  memcpy(j->local_public, s->local_public, 64);
  memcpy(j->peer_public, s->peer_public, 64);
  memcpy(j->local_nonce, s->local_nonce, 16);
  memcpy(j->peer_nonce, s->peer_nonce, 16);
  memcpy(j->peer_confirm, s->peer_confirm, 16);
  s->job_pending = true;
  j->passkey = s->passkey;
  j->round = s->round;
  memcpy(j->local_oob_random, s->local_oob_random, 16);
  memcpy(j->peer_oob_random, s->peer_oob_random, 16);
  memcpy(j->peer_oob_confirm, s->peer_oob_confirm, 16);
}
dbt_result_t dbt_smp_sc_open(dbt_smp_sc_t *s, uint16_t h, uint32_t gen, uint8_t role, uint8_t lt,
                             const uint8_t local[6], uint8_t pt, const uint8_t peer[6], const uint8_t req[7],
                             const uint8_t rsp[7], uint8_t min, uint32_t now) {
  if (!s || !gen || role > 1 || lt > 1 || pt > 1 || !local || !peer || !req || !rsp || min < 7 || min > 16)
    return DBT_INVALID;
  if (req[0] != 1 || rsp[0] != 2 || req[1] > 4 || rsp[1] > 4 || req[2] > 1 || rsp[2] > 1 ||
      req[2] != rsp[2] || !(req[3] & 8) || !(rsp[3] & 8) || (req[3] & 3) > 1 || (rsp[3] & 3) > 1 ||
      ((req[3] & 3) != (rsp[3] & 3)) || ((req[5] | req[6]) & ~7) || ((rsp[5] | rsp[6]) & ~6) ||
      (rsp[5] & ~req[5]) || (rsp[6] & ~req[6]) || (!(req[3] & 1) && (req[5] || req[6] || rsp[5] || rsp[6])) ||
      req[4] < min || rsp[4] < min || req[4] > 16 || rsp[4] > 16)
    return DBT_UNSUPPORTED;
  bool mitm = ((req[3] | rsp[3]) & 4) != 0;
  bool oob = req[2] != 0;
  bool numeric = mitm && !oob && (req[1] == 1 || req[1] == 4) && (rsp[1] == 1 || rsp[1] == 4);
  bool ki = req[1] == 2 || req[1] == 4, kr = rsp[1] == 2 || rsp[1] == 4;
  bool di = req[1] == 0 || req[1] == 1 || req[1] == 4, dr = rsp[1] == 0 || rsp[1] == 1 || rsp[1] == 4;
  bool passkey = mitm && !numeric && !oob && ((ki && dr) || (kr && di) || (req[1] == 2 && rsp[1] == 2));
  if (mitm && !numeric && !passkey && !oob)
    return DBT_UNSUPPORTED;
  if ((req[3] & 1) && pt == 1 && (peer[5] & 0xc0) != 0xc0) {
    uint8_t peer_mask=role==0 ? rsp[6] : rsp[5];
    bool rpa=(peer[5]&0xc0u)==0x40u &&
        !(peer[3]==0 && peer[4]==0 && (peer[5]&0x3fu)==0) &&
        !(peer[3]==0xff && peer[4]==0xff && (peer[5]&0x3fu)==0x3f);
    if (!rpa || !(peer_mask&2)) return DBT_UNSUPPORTED;
  }
  memset(s, 0, sizeof(*s));
  s->used = true;
  s->handle = h;
  s->generation = gen;
  s->initiator = role == 0;
  s->iat = s->initiator ? lt : pt;
  s->rat = s->initiator ? pt : lt;
  memcpy(s->ia, s->initiator ? local : peer, 6);
  memcpy(s->ra, s->initiator ? peer : local, 6);
  memcpy(s->preq, req, 7);
  memcpy(s->pres, rsp, 7);
  s->key_size = req[4] < rsp[4] ? req[4] : rsp[4];
  s->bonding = (req[3] & 1) != 0;
  dbt_smp_bond_init(&s->bond, gen);
  s->local_mask = s->initiator ? rsp[5] : rsp[6];
  s->peer_mask = s->initiator ? rsp[6] : rsp[5];
  s->distribution_ready = s->local_mask == 0;
  s->numeric = numeric;
  s->passkey_mode = passkey;
  s->passkey_ready = !passkey;
  s->oob_mode = oob;
  s->oob_ready = !oob;
  s->bond_policy_checked = !s->bonding;
  s->state = DBT_SC_KEYGEN;
  s->deadline = now + TIMEOUT_MS;
  job(s, DBT_SC_JOB_KEYGEN);
  return DBT_OK;
}
void dbt_smp_sc_close(dbt_smp_sc_t *s) {
  if (s)
    wipe(s, sizeof(*s));
}
void dbt_smp_sc_poll(dbt_smp_sc_t *s, uint32_t now) {
  if (!s || !s->used || s->state == DBT_SC_FAILED || s->state == DBT_SC_TIMED_OUT ||
      s->state == DBT_SC_SECURE || s->state == DBT_SC_BOND_STORE)
    return;
  if ((int32_t)(now - s->deadline) >= 0) {
    clear_secrets(s);
    wipe(s->tx, sizeof(s->tx));
    s->tx_size = 0;
    s->state = DBT_SC_TIMED_OUT;
  }
}
dbt_result_t dbt_smp_sc_receive(dbt_smp_sc_t *s, const uint8_t *p, uint16_t n, uint32_t now) {
  if (!s || !s->used || !p || !n)
    return DBT_INVALID;
  dbt_smp_sc_poll(s, now);
  if (s->state == DBT_SC_FAILED || s->state == DBT_SC_TIMED_OUT)
    return DBT_BUSY;
  if (p[0] == 5 && n == 2) {
    fail(s, p[1], false, now);
    return DBT_OK;
  }
  /* Caller must retain and retry a PDU blocked by a local worker/transport. */
  if (s->tx_size || s->job_pending || s->state == DBT_SC_WAIT_USER)
    return DBT_BUSY;
  if (s->state == DBT_SC_WAIT_PUBLIC && p[0] == 12 && n == 65) {
    if (same(s->local_public, p + 1, 32)) {
      fail(s, 10, true, now);
      return DBT_OK;
    }
    memcpy(s->peer_public, p + 1, 64);
    if (s->initiator) {
      if (s->passkey_mode) {
        s->state = DBT_SC_ROUND_LOCAL;
        job(s, DBT_SC_JOB_ROUND_LOCAL);
      } else if (s->oob_mode) {
        s->state = DBT_SC_OOB_VERIFY;
        job(s, DBT_SC_JOB_OOB_VERIFY);
      } else
        s->state = DBT_SC_WAIT_CONFIRM;
    } else {
      queue(s, 12, s->local_public, 64, now);
      s->state = DBT_SC_COMMIT;
    }
    return DBT_OK;
  }
  if (s->state == DBT_SC_WAIT_CONFIRM && p[0] == 3 && n == 17 && (s->initiator || s->passkey_mode)) {
    memcpy(s->peer_confirm, p + 1, 16);
    if (s->initiator)
      queue(s, 4, s->local_nonce, 16, now);
    else
      queue(s, 3, s->local_check, 16, now);
    s->state = DBT_SC_WAIT_RANDOM;
    return DBT_OK;
  }
  if (s->state == DBT_SC_WAIT_RANDOM && p[0] == 4 && n == 17) {
    memcpy(s->peer_nonce, p + 1, 16);
    if (s->passkey_mode) {
      s->state = DBT_SC_ROUND_VERIFY;
      job(s, DBT_SC_JOB_ROUND_VERIFY);
      return DBT_OK;
    }
    s->state = DBT_SC_DERIVE;
    if (s->initiator)
      job(s, DBT_SC_JOB_DERIVE);
    else
      queue(s, 4, s->local_nonce, 16, now);
    return DBT_OK;
  }
  if (s->state == DBT_SC_WAIT_CHECK && p[0] == 13 && n == 17) {
    if (!same(s->peer_check, p + 1, 16)) {
      fail(s, 11, true, now);
      return DBT_OK;
    }
    if (!s->initiator)
      queue(s, 13, s->local_check, 16, now);
    s->state = DBT_SC_ENCRYPTION;
    return DBT_OK;
  }
  fail(s, 10, true, now);
  return DBT_OK;
}
dbt_result_t dbt_smp_sc_tx(const dbt_smp_sc_t *s, const uint8_t **p, uint8_t *n) {
  if (!s || !s->used || !p || !n)
    return DBT_INVALID;
  if (!s->tx_size)
    return DBT_EMPTY;
  *p = s->tx;
  *n = s->tx_size;
  return DBT_OK;
}
void dbt_smp_sc_tx_done(dbt_smp_sc_t *s, bool admitted, uint32_t now) {
  if (!s || !s->used)
    return;
  dbt_smp_sc_poll(s, now);
  if (!s->tx_size)
    return;
  uint8_t code = s->tx[0];
  wipe(s->tx, sizeof(s->tx));
  s->tx_size = 0;
  if (!admitted) {
    fail(s, 8, false, now);
    return;
  }
  if (!s->initiator && code == 12 && s->state == DBT_SC_COMMIT) {
    if (s->passkey_mode) {
      s->state = DBT_SC_ROUND_LOCAL;
      job(s, DBT_SC_JOB_ROUND_LOCAL);
    } else if (s->oob_mode) {
      s->state = DBT_SC_OOB_VERIFY;
      job(s, DBT_SC_JOB_OOB_VERIFY);
    } else
      job(s, DBT_SC_JOB_COMMIT);
  }
  if (!s->initiator && code == 4 && s->state == DBT_SC_DERIVE)
    job(s, DBT_SC_JOB_DERIVE);
  if (!s->initiator && code == 4 && s->state == DBT_SC_ROUND_LOCAL) {
    if (s->round == 19) {
      s->authenticated = true;
      s->state = DBT_SC_DERIVE;
      job(s, DBT_SC_JOB_DERIVE);
    } else {
      s->round++;
      job(s, DBT_SC_JOB_ROUND_LOCAL);
    }
  }
}
dbt_result_t dbt_smp_sc_job(const dbt_smp_sc_t *s, dbt_smp_sc_job_t *out) {
  if (!s || !s->used || !out)
    return DBT_INVALID;
  if (!s->job_pending)
    return DBT_EMPTY;
  *out = s->job;
  return DBT_OK;
}
dbt_result_t dbt_smp_sc_complete(dbt_smp_sc_t *s, uint32_t gen, uint32_t serial, bool success,
                                 const dbt_smp_sc_result_t *r, uint32_t now) {
  if (!s || !s->used || (success && !r))
    return DBT_INVALID;
  dbt_smp_sc_poll(s, now);
  if (!s->job_pending || s->generation != gen || s->job.serial != serial)
    return DBT_INVALID;
  if (success && s->job.kind == DBT_SC_JOB_KEYGEN &&
      (!s->distribution_ready || !s->passkey_ready || !s->oob_ready || !s->bond_policy_checked))
    return DBT_BUSY;
  if (!success) {
    fail(s, 8, true, now);
    return DBT_OK;
  }
  dbt_smp_sc_job_kind_t kind = s->job.kind;
  wipe(&s->job, sizeof(s->job));
  s->job_pending = false;
  if (kind == DBT_SC_JOB_KEYGEN) {
    if (s->oob_mode && !same(s->local_oob_public, r->public_key, 64)) {
      fail(s, 10, true, now);
      return DBT_OK;
    }
    memcpy(s->local_public, r->public_key, 64);
    memcpy(s->local_nonce, r->nonce, 16);
    s->state = DBT_SC_WAIT_PUBLIC;
    if (s->initiator)
      queue(s, 12, s->local_public, 64, now);
  } else if (kind == DBT_SC_JOB_COMMIT) {
    queue(s, 3, r->confirm, 16, now);
    s->state = DBT_SC_WAIT_RANDOM;
  } else if (kind == DBT_SC_JOB_ROUND_LOCAL) {
    memcpy(s->local_nonce, r->nonce, 16);
    memcpy(s->local_check, r->confirm, 16);
    s->state = DBT_SC_WAIT_CONFIRM;
    if (s->initiator)
      queue(s, 3, s->local_check, 16, now);
  } else if (kind == DBT_SC_JOB_ROUND_VERIFY) {
    if (!r->peer_commitment_valid) {
      fail(s, 4, true, now);
      return DBT_OK;
    }
    if (!s->initiator) {
      queue(s, 4, s->local_nonce, 16, now);
      s->state = DBT_SC_ROUND_LOCAL;
    } else if (s->round == 19) {
      s->authenticated = true;
      s->state = DBT_SC_DERIVE;
      job(s, DBT_SC_JOB_DERIVE);
    } else {
      s->round++;
      s->state = DBT_SC_ROUND_LOCAL;
      job(s, DBT_SC_JOB_ROUND_LOCAL);
    }
  } else if (kind == DBT_SC_JOB_OOB_VERIFY) {
    if (!r->peer_commitment_valid) {
      fail(s, 4, true, now);
      return DBT_OK;
    }
    s->authenticated = s->oob_authenticated;
    s->state = DBT_SC_WAIT_RANDOM;
    if (s->initiator)
      queue(s, 4, s->local_nonce, 16, now);
  } else if (kind == DBT_SC_JOB_DERIVE) {
    if (s->initiator && !s->passkey_mode && !s->oob_mode && !r->peer_commitment_valid) {
      fail(s, 4, true, now);
      return DBT_OK;
    }
    memcpy(s->ltk, r->ltk, 16);
    for (unsigned i = s->key_size; i < 16; i++)
      s->ltk[i] = 0;
    if (s->bonding)
      memcpy(s->bond_ltk, s->ltk, 16);
    memcpy(s->local_check, r->local_check, 16);
    memcpy(s->peer_check, r->peer_check, 16);
    wipe(s->local_nonce, 16);
    wipe(s->peer_nonce, 16);
    wipe(s->peer_confirm, 16);
    wipe(&s->passkey, sizeof(s->passkey));
    wipe(s->local_oob_random, 16);
    wipe(s->peer_oob_random, 16);
    wipe(s->peer_oob_confirm, 16);
    if (s->numeric) {
      if (r->numeric_value >= 1000000u) {
        fail(s, 8, true, now);
        return DBT_OK;
      }
      s->numeric_value = r->numeric_value;
      s->state = DBT_SC_WAIT_USER;
    } else {
      s->state = DBT_SC_WAIT_CHECK;
      if (s->initiator)
        queue(s, 13, s->local_check, 16, now);
    }
  } else
    return DBT_INVALID;
  return DBT_OK;
}
dbt_result_t dbt_smp_sc_take_ltk(dbt_smp_sc_t *s, uint8_t key[16], uint8_t *size) {
  if (!s || !s->used || !key || !size)
    return DBT_INVALID;
  if (s->state != DBT_SC_ENCRYPTION || s->key_taken || s->tx_size)
    return DBT_BUSY;
  memcpy(key, s->ltk, 16);
  *size = s->key_size;
  wipe(s->ltk, 16);
  s->key_taken = true;
  return DBT_OK;
}
static dbt_result_t start_store(dbt_smp_sc_t *s) {
  dbt_smp_bond_record_t record;
  memset(&record, 0, sizeof(record));
  record.identity_type = s->initiator ? s->rat : s->iat;
  memcpy(record.identity, s->initiator ? s->ra : s->ia, 6);
  memcpy(record.ltk, s->bond_ltk, 16);
  record.key_size = s->key_size;
  record.secure_connections = true;
  record.authenticated = s->authenticated;
  record.have_local_csrk = (s->local_mask & 4) != 0;
  record.have_peer_csrk = (s->peer_mask & 4) != 0;
  memcpy(record.local_csrk, s->local_csrk, 16);
  memcpy(record.peer_csrk, s->peer_csrk, 16);
  if (s->local_mask & 2) {
    record.have_local_irk = true;
    memcpy(record.local_irk, s->local_distribution.irk, 16);
  }
  if (s->peer_mask & 2) {
    record.candidate_present = true;
    record.candidate_type = s->peer_distribution.identity_type;
    memcpy(record.candidate_identity, s->peer_distribution.identity, 6);
    memcpy(record.candidate_irk, s->peer_distribution.irk, 16);
    /* Pairing used an RPA for the transcript. Only the encrypted, completed
     * identity distribution supplies the stable storage key. */
    if (record.identity_type==1 && (record.identity[5]&0xc0u)==0x40u) {
      record.identity_type=record.candidate_type;
      memcpy(record.identity,record.candidate_identity,6);
      record.have_irk=true;
      memcpy(record.irk,record.candidate_irk,16);
    }
  }
  dbt_result_t rc =
      dbt_smp_bond_begin(&s->bond, DBT_SMP_BOND_STORE, record.identity_type, record.identity, &record);
  wipe(&record, sizeof(record));
  wipe(s->bond_ltk, 16);
  wipe(s->local_csrk, 16);
  wipe(s->peer_csrk, 16);
  wipe(&s->local_distribution, sizeof(s->local_distribution));
  wipe(&s->peer_distribution, sizeof(s->peer_distribution));
  if (rc != DBT_OK) {
    fail(s, 8, false, 0);
    return rc;
  }
  s->state = DBT_SC_BOND_STORE;
  return DBT_OK;
}
static void finish_dist(dbt_smp_sc_t *s) {
  dbt_smp_dist_keys_t peer;
  memset(&peer, 0, sizeof(peer));
  if (dbt_smp_dist_take(&s->dist, &peer) != DBT_OK)
    return;
  s->peer_final_received = s->dist.peer_final_received;
  s->final_ll_acked = s->dist.final_ll_acked;
  s->hci_buffers_completed = s->dist.hci_buffers_completed;
  s->local_pdus_admitted = s->dist.local_pdus_admitted;
  s->peer_distribution = peer;
  if (s->peer_mask & 4)
    memcpy(s->peer_csrk, peer.csrk, 16);
  wipe(&peer, sizeof(peer));
  dbt_smp_dist_close(&s->dist);
  (void)start_store(s);
}
dbt_result_t dbt_smp_sc_encryption(dbt_smp_sc_t *s, uint32_t gen, bool encrypted) {
  if (!s || !s->used || gen != s->generation)
    return DBT_INVALID;
  if (!encrypted) {
    fail(s, 0, false, 0);
    return DBT_OK;
  }
  if (s->state != DBT_SC_ENCRYPTION || !s->key_taken)
    return DBT_INVALID;
  s->encryption_verified = true;
  wipe(s->local_check, 16);
  wipe(s->peer_check, 16);
  if (s->bonding && (s->local_mask || s->peer_mask)) {
    dbt_result_t rc = dbt_smp_dist_open(&s->dist, gen, s->initiator, true, s->key_size, s->local_mask,
                                        s->peer_mask, true, &s->local_distribution);
    if (rc != DBT_OK) {
      fail(s, 8, false, 0);
      return rc;
    }
    s->state = DBT_SC_DISTRIBUTION;
  } else if (s->bonding)
    return start_store(s);
  else
    s->state = DBT_SC_SECURE;
  return DBT_OK;
}
dbt_result_t dbt_smp_sc_numeric_value(const dbt_smp_sc_t *s, uint32_t gen, uint32_t *value) {
  if (!s || !s->used || s->generation != gen || !value)
    return DBT_INVALID;
  if (s->state != DBT_SC_WAIT_USER)
    return DBT_BUSY;
  *value = s->numeric_value;
  return DBT_OK;
}
dbt_result_t dbt_smp_sc_numeric_confirm(dbt_smp_sc_t *s, uint32_t gen, bool accepted, uint32_t now) {
  if (!s || !s->used || s->generation != gen)
    return DBT_INVALID;
  dbt_smp_sc_poll(s, now);
  if (s->state != DBT_SC_WAIT_USER)
    return DBT_BUSY;
  s->numeric_value = 0;
  if (!accepted) {
    fail(s, 12, true, now);
    return DBT_OK;
  }
  s->authenticated = true;
  s->state = DBT_SC_WAIT_CHECK;
  if (s->initiator)
    queue(s, 13, s->local_check, 16, now);
  return DBT_OK;
}
dbt_result_t dbt_smp_sc_set_passkey(dbt_smp_sc_t *s, uint32_t passkey) {
  if (!s || !s->used || passkey > 999999u)
    return DBT_INVALID;
  if (s->state != DBT_SC_KEYGEN || !s->passkey_mode)
    return DBT_BUSY;
  s->passkey = passkey;
  s->passkey_ready = true;
  return DBT_OK;
}
dbt_result_t dbt_smp_sc_set_oob(dbt_smp_sc_t *s, const uint8_t local_public[64],
                                const uint8_t local_random[16], const uint8_t peer_random[16],
                                const uint8_t peer_confirm[16], bool authenticated) {
  if (!s || !s->used || !local_public || !local_random || !peer_random || !peer_confirm)
    return DBT_INVALID;
  if (s->state != DBT_SC_KEYGEN || !s->oob_mode)
    return DBT_BUSY;
  if (((s->preq[3] | s->pres[3]) & 4) && !authenticated)
    return DBT_UNSUPPORTED;
  memcpy(s->local_oob_public, local_public, 64);
  memcpy(s->local_oob_random, local_random, 16);
  memcpy(s->peer_oob_random, peer_random, 16);
  memcpy(s->peer_oob_confirm, peer_confirm, 16);
  s->oob_authenticated = authenticated;
  s->oob_ready = true;
  if (s->bonding)
    s->bond_policy_checked = false;
  return DBT_OK;
}
dbt_result_t dbt_smp_sc_check_bond_policy(dbt_smp_sc_t *s, bool existing, bool old_auth, uint8_t old_size) {
  if (!s || !s->used || (existing && (old_size < 7 || old_size > 16)))
    return DBT_INVALID;
  if (s->state != DBT_SC_KEYGEN || !s->bonding || !s->oob_ready)
    return DBT_BUSY;
  bool new_auth = s->numeric || s->passkey_mode || (s->oob_mode && s->oob_authenticated);
  if (existing && (old_size > s->key_size || (old_auth && !new_auth)))
    return DBT_UNSUPPORTED;
  s->bond_policy_checked = true;
  return DBT_OK;
}
dbt_result_t dbt_smp_sc_set_distribution(dbt_smp_sc_t *s, const dbt_smp_dist_keys_t *keys) {
  if (!s || !s->used || !keys)
    return DBT_INVALID;
  if (s->state != DBT_SC_KEYGEN || !s->local_mask)
    return DBT_BUSY;
  if ((s->local_mask & 2) &&
      (keys->identity_type > 1 || (keys->identity_type == 1 && (keys->identity[5] & 0xc0) != 0xc0)))
    return DBT_INVALID;
  s->local_distribution = *keys;
  if (s->local_mask & 4)
    memcpy(s->local_csrk, keys->csrk, 16);
  s->distribution_ready = true;
  return DBT_OK;
}
dbt_result_t dbt_smp_sc_dist_tx(dbt_smp_sc_t *s, const uint8_t **p, uint8_t *n, uint32_t now) {
  if (!s || !s->used)
    return DBT_INVALID;
  dbt_smp_sc_poll(s, now);
  if (s->state != DBT_SC_DISTRIBUTION)
    return DBT_BUSY;
  bool new_packet = s->dist.tx_size == 0;
  dbt_result_t rc = dbt_smp_dist_tx(&s->dist, p, n);
  if (rc == DBT_OK && new_packet)
    s->deadline = now + TIMEOUT_MS;
  return rc;
}
dbt_result_t dbt_smp_sc_dist_tx_admitted(dbt_smp_sc_t *s, uint32_t gen, uint32_t serial, bool admitted,
                                         uint32_t now) {
  if (!s || !s->used)
    return DBT_INVALID;
  dbt_smp_sc_poll(s, now);
  if (s->state != DBT_SC_DISTRIBUTION)
    return DBT_BUSY;
  dbt_result_t rc = dbt_smp_dist_tx_admitted(&s->dist, gen, serial, admitted);
  if (s->dist.failed) {
    fail(s, 8, false, now);
    return rc;
  }
  finish_dist(s);
  return rc;
}
dbt_result_t dbt_smp_sc_dist_final_ack(dbt_smp_sc_t *s, uint32_t generation, uint32_t serial,
                                       bool acknowledged, uint32_t now) {
  if (!s || !s->used)
    return DBT_INVALID;
  dbt_smp_sc_poll(s, now);
  if (s->state != DBT_SC_DISTRIBUTION)
    return DBT_BUSY;
  dbt_result_t rc = dbt_smp_dist_final_ack(&s->dist, generation, serial, acknowledged);
  if (s->dist.failed) {
    fail(s, 8, false, now);
    return rc;
  }
  finish_dist(s);
  return rc;
}
dbt_result_t dbt_smp_sc_dist_receive(dbt_smp_sc_t *s, const uint8_t *p, uint8_t n, uint32_t now) {
  if (!s || !s->used)
    return DBT_INVALID;
  dbt_smp_sc_poll(s, now);
  if (s->state != DBT_SC_DISTRIBUTION)
    return DBT_BUSY;
  dbt_result_t rc = dbt_smp_dist_receive(&s->dist, p, n);
  if (s->dist.failed) {
    fail(s, 10, true, now);
    return rc;
  }
  finish_dist(s);
  return rc;
}
dbt_result_t dbt_smp_sc_bond_job(const dbt_smp_sc_t *s, dbt_smp_bond_job_t *job_out) {
  if (!s || !s->used || s->state != DBT_SC_BOND_STORE)
    return DBT_INVALID;
  return dbt_smp_bond_job(&s->bond, job_out);
}
dbt_result_t dbt_smp_sc_bond_complete(dbt_smp_sc_t *s, uint32_t gen, uint32_t serial, bool committed) {
  if (!s || !s->used || s->state != DBT_SC_BOND_STORE)
    return DBT_INVALID;
  dbt_result_t rc = dbt_smp_bond_complete(&s->bond, gen, serial, committed, 0);
  if (rc != DBT_OK) {
    if (gen == s->generation && !s->bond.pending)
      fail(s, 8, false, 0);
    return rc;
  }
  s->bond_committed = true;
  s->state = DBT_SC_SECURE;
  return DBT_OK;
}
dbt_result_t dbt_smp_sc_bond_cancel(dbt_smp_sc_t *s, uint32_t gen, uint32_t serial) {
  if (!s || !s->used || s->state != DBT_SC_BOND_STORE)
    return DBT_INVALID;
  dbt_result_t rc = dbt_smp_bond_cancel(&s->bond, gen, serial);
  if (rc == DBT_OK)
    fail(s, 8, false, 0);
  return rc;
}
