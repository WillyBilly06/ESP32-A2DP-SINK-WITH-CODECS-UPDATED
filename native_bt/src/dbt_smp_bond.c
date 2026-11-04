#include "dbt_smp_bond.h"
#include <string.h>
static void wipe(void *v, unsigned n) {
  volatile uint8_t *p = v;
  while (n--)
    *p++ = 0;
}
static bool valid(const dbt_smp_bond_record_t *r, uint8_t type, const uint8_t id[6]) {
  if (!r || r->identity_type != type || memcmp(r->identity, id, 6) || r->key_size < 7 || r->key_size > 16)
    return false;
  for (unsigned i = r->key_size; i < 16; i++)
    if (r->ltk[i])
      return false;
  for (unsigned i = r->key_size; i < 16; i++)
    if (r->local_ltk[i])
      return false;
  if (r->secure_connections && (r->have_local_ltk || r->have_peer_ltk))
    return false;
  if (r->candidate_verified && !r->candidate_present)
    return false;
  if (r->candidate_present &&
      (r->candidate_type > 1 || (r->candidate_type == 1 && (r->candidate_identity[5] & 0xc0) != 0xc0)))
    return false;
  if (r->have_irk && r->candidate_present && r->identity_type==r->candidate_type &&
      !memcmp(r->identity,r->candidate_identity,6) && memcmp(r->irk,r->candidate_irk,16))
    return false;
  if (r->secure_connections) {
    if (r->ediv)
      return false;
    for (unsigned i = 0; i < 8; i++)
      if (r->rand[i])
        return false;
  }
  return true;
}
void dbt_smp_bond_init(dbt_smp_bond_t *b, uint32_t gen) {
  if (b) {
    memset(b, 0, sizeof(*b));
    b->generation = gen;
  }
}
void dbt_smp_bond_close(dbt_smp_bond_t *b) {
  if (b)
    wipe(b, sizeof(*b));
}
dbt_result_t dbt_smp_bond_begin(dbt_smp_bond_t *b, dbt_smp_bond_op_t op, uint8_t type, const uint8_t id[6],
                                const dbt_smp_bond_record_t *record) {
  if (!b || !b->generation || !id || type > 1 || op < DBT_SMP_BOND_STORE || op > DBT_SMP_BOND_DELETE)
    return DBT_INVALID;
  if (b->pending)
    return DBT_BUSY;
  if (op == DBT_SMP_BOND_STORE && !valid(record, type, id))
    return DBT_INVALID;
  wipe(&b->loaded, sizeof(b->loaded));
  b->loaded_valid = false;
  wipe(&b->job, sizeof(b->job));
  b->serial++;
  if (!b->serial)
    b->serial++;
  b->job.generation = b->generation;
  b->job.serial = b->serial;
  b->job.op = op;
  b->job.identity_type = type;
  memcpy(b->job.identity, id, 6);
  if (op == DBT_SMP_BOND_STORE)
    b->job.record = *record;
  b->pending = true;
  return DBT_OK;
}
dbt_result_t dbt_smp_bond_job(const dbt_smp_bond_t *b, dbt_smp_bond_job_t *out) {
  if (!b || !out || !b->generation)
    return DBT_INVALID;
  if (!b->pending)
    return DBT_EMPTY;
  *out = b->job;
  return DBT_OK;
}
dbt_result_t dbt_smp_bond_complete(dbt_smp_bond_t *b, uint32_t gen, uint32_t serial, bool success,
                                   const dbt_smp_bond_record_t *loaded) {
  if (!b || !b->pending || gen != b->generation || serial != b->job.serial)
    return DBT_INVALID;
  bool ok = success;
  if (b->job.op == DBT_SMP_BOND_LOAD && success) {
    ok = valid(loaded, b->job.identity_type, b->job.identity);
    if (ok) {
      b->loaded = *loaded;
      b->loaded_valid = true;
    }
  }
  wipe(&b->job, sizeof(b->job));
  b->pending = false;
  return ok ? DBT_OK : DBT_INVALID;
}
dbt_result_t dbt_smp_bond_cancel(dbt_smp_bond_t *b, uint32_t gen, uint32_t serial) {
  if (!b || !b->pending || gen != b->generation || serial != b->job.serial)
    return DBT_INVALID;
  wipe(&b->job, sizeof(b->job));
  wipe(&b->loaded, sizeof(b->loaded));
  b->pending = false;
  b->loaded_valid = false;
  return DBT_OK;
}
dbt_result_t dbt_smp_bond_take(dbt_smp_bond_t *b, uint8_t min, bool auth, bool sc,
                               dbt_smp_bond_record_t *out) {
  if (!b || !out || min < 7 || min > 16)
    return DBT_INVALID;
  if (b->pending)
    return DBT_BUSY;
  if (!b->loaded_valid)
    return DBT_EMPTY;
  if (b->loaded.key_size < min || (auth && !b->loaded.authenticated) ||
      (sc && !b->loaded.secure_connections)) {
    wipe(&b->loaded, sizeof(b->loaded));
    b->loaded_valid = false;
    return DBT_UNSUPPORTED;
  }
  *out = b->loaded;
  wipe(&b->loaded, sizeof(b->loaded));
  b->loaded_valid = false;
  return DBT_OK;
}
dbt_result_t dbt_smp_reconnect_open(dbt_smp_reconnect_t *r, uint32_t gen, uint8_t type, const uint8_t id[6],
                                    uint8_t min, bool auth, bool sc) {
  if (!r || !gen || !id || type > 1 || min < 7 || min > 16 || (type == 1 && (id[5] & 0xc0) != 0xc0))
    return DBT_INVALID;
  memset(r, 0, sizeof(*r));
  r->generation = gen;
  r->min_key_size = min;
  r->require_authenticated = auth;
  r->require_sc = sc;
  dbt_smp_bond_init(&r->broker, gen);
  dbt_result_t rc = dbt_smp_bond_begin(&r->broker, DBT_SMP_BOND_LOAD, type, id, 0);
  if (rc == DBT_OK)
    r->state = DBT_SMP_RECONNECT_LOAD;
  return rc;
}
dbt_result_t dbt_smp_reconnect_open_candidate(dbt_smp_reconnect_t *r, uint32_t gen, uint8_t type,
                                              const uint8_t id[6], uint8_t candidate_type,
                                              const uint8_t candidate[6], uint8_t min, bool auth, bool sc) {
  if (!candidate || candidate_type > 1 || (candidate_type == 1 && (candidate[5] & 0xc0) != 0xc0))
    return DBT_INVALID;
  dbt_result_t rc = dbt_smp_reconnect_open(r, gen, type, id, min, auth, sc);
  if (rc == DBT_OK) {
    r->candidate_mode = true;
    r->candidate_type = candidate_type;
    memcpy(r->candidate_identity, candidate, 6);
  }
  return rc;
}
dbt_result_t dbt_smp_reconnect_open_role(dbt_smp_reconnect_t *r, uint32_t gen, uint8_t type,
                                         const uint8_t id[6], uint8_t min, bool auth, bool sc,
                                         bool peripheral) {
  dbt_result_t rc = dbt_smp_reconnect_open(r, gen, type, id, min, auth, sc);
  if (rc == DBT_OK)
    r->local_role_peripheral = peripheral;
  return rc;
}
dbt_result_t dbt_smp_reconnect_open_candidate_role(dbt_smp_reconnect_t *r,uint32_t gen,uint8_t type,
    const uint8_t id[6],uint8_t candidate_type,const uint8_t candidate[6],uint8_t min,bool auth,bool sc,
    bool peripheral) {
  dbt_result_t rc=dbt_smp_reconnect_open_candidate(r,gen,type,id,candidate_type,candidate,min,auth,sc);
  if (rc==DBT_OK) r->local_role_peripheral=peripheral;
  return rc;
}
void dbt_smp_reconnect_close(dbt_smp_reconnect_t *r) {
  if (r)
    wipe(r, sizeof(*r));
}
dbt_result_t dbt_smp_reconnect_job(const dbt_smp_reconnect_t *r, dbt_smp_bond_job_t *job_out) {
  if (!r || (r->state != DBT_SMP_RECONNECT_LOAD && r->state != DBT_SMP_RECONNECT_PROMOTE))
    return DBT_INVALID;
  return dbt_smp_bond_job(&r->broker, job_out);
}
dbt_result_t dbt_smp_reconnect_complete(dbt_smp_reconnect_t *r, uint32_t gen, uint32_t serial, bool success,
                                        const dbt_smp_bond_record_t *record) {
  if (!r || (r->state != DBT_SMP_RECONNECT_LOAD && r->state != DBT_SMP_RECONNECT_PROMOTE))
    return DBT_INVALID;
  if (r->state == DBT_SMP_RECONNECT_PROMOTE) {
    dbt_result_t promoted = dbt_smp_bond_complete(&r->broker, gen, serial, success, 0);
    if (promoted == DBT_OK) {
      r->identity_promotion_committed = true;
      r->state = DBT_SMP_RECONNECT_SECURE;
    } else if (gen == r->generation && !r->broker.pending)
      r->state = DBT_SMP_RECONNECT_FAILED;
    return promoted;
  }
  dbt_result_t rc = dbt_smp_bond_complete(&r->broker, gen, serial, success, record);
  if (rc != DBT_OK) {
    if (gen == r->generation && !r->broker.pending)
      r->state = DBT_SMP_RECONNECT_FAILED;
    return rc;
  }
  rc = dbt_smp_bond_take(&r->broker, r->min_key_size, r->require_authenticated, r->require_sc, &r->record);
  if (rc == DBT_OK && r->candidate_mode &&
      (!r->record.candidate_present || r->record.candidate_type != r->candidate_type ||
       memcmp(r->record.candidate_identity, r->candidate_identity, 6))) {
    wipe(&r->record, sizeof(r->record));
    rc = DBT_INVALID;
  }
  if (rc == DBT_OK && !r->record.secure_connections &&
      (r->local_role_peripheral ? !r->record.have_local_ltk : !r->record.have_peer_ltk)) {
    wipe(&r->record, sizeof(r->record));
    rc = DBT_UNSUPPORTED;
  }
  r->state = rc == DBT_OK ? DBT_SMP_RECONNECT_KEY : DBT_SMP_RECONNECT_FAILED;
  return rc;
}
dbt_result_t dbt_smp_reconnect_cancel(dbt_smp_reconnect_t *r, uint32_t gen, uint32_t serial) {
  if (!r || (r->state != DBT_SMP_RECONNECT_LOAD && r->state != DBT_SMP_RECONNECT_PROMOTE))
    return DBT_INVALID;
  dbt_result_t rc = dbt_smp_bond_cancel(&r->broker, gen, serial);
  if (rc == DBT_OK)
    r->state = DBT_SMP_RECONNECT_FAILED;
  return rc;
}
dbt_result_t dbt_smp_reconnect_ids(const dbt_smp_reconnect_t *r, uint16_t *ediv, uint8_t rand[8]) {
  if (!r || !ediv || !rand)
    return DBT_INVALID;
  if (r->state != DBT_SMP_RECONNECT_KEY)
    return DBT_BUSY;
  if (r->local_role_peripheral && !r->record.secure_connections) {
    *ediv = r->record.local_ediv;
    memcpy(rand, r->record.local_rand, 8);
  } else {
    *ediv = r->record.ediv;
    memcpy(rand, r->record.rand, 8);
  }
  return DBT_OK;
}
dbt_result_t dbt_smp_reconnect_take(dbt_smp_reconnect_t *r, uint16_t ediv, const uint8_t rand[8],
                                    uint8_t ltk[16], uint8_t *size) {
  if (!r || !rand || !ltk || !size)
    return DBT_INVALID;
  if (r->state != DBT_SMP_RECONNECT_KEY)
    return DBT_BUSY;
  bool local = r->local_role_peripheral && !r->record.secure_connections;
  if (ediv != (local ? r->record.local_ediv : r->record.ediv) ||
      memcmp(rand, local ? r->record.local_rand : r->record.rand, 8))
    return DBT_INVALID;
  memcpy(ltk, local ? r->record.local_ltk : r->record.ltk, 16);
  *size = r->record.key_size;
  if (!r->candidate_mode)
    wipe(&r->record, sizeof(r->record));
  r->state = DBT_SMP_RECONNECT_ENCRYPTION;
  return DBT_OK;
}
dbt_result_t dbt_smp_reconnect_encryption(dbt_smp_reconnect_t *r, uint32_t gen, bool encrypted) {
  if (!r || !r->generation || gen != r->generation)
    return DBT_INVALID;
  if (!encrypted) {
    wipe(&r->record, sizeof(r->record));
    dbt_smp_bond_close(&r->broker);
    r->state = DBT_SMP_RECONNECT_FAILED;
    r->peer_key_verified = false;
    return DBT_OK;
  }
  if (r->state != DBT_SMP_RECONNECT_ENCRYPTION)
    return DBT_INVALID;
  r->peer_key_verified = true;
  if (r->candidate_mode && !r->record.candidate_verified) {
    r->record.candidate_verified = true;
    dbt_result_t rc = dbt_smp_bond_begin(&r->broker, DBT_SMP_BOND_STORE, r->record.identity_type,
                                         r->record.identity, &r->record);
    wipe(&r->record, sizeof(r->record));
    if (rc != DBT_OK) {
      r->state = DBT_SMP_RECONNECT_FAILED;
      return rc;
    }
    r->state = DBT_SMP_RECONNECT_PROMOTE;
    return DBT_OK;
  }
  if (r->candidate_mode && r->record.candidate_verified)
    r->identity_promotion_committed = true;
  wipe(&r->record, sizeof(r->record));
  r->state = DBT_SMP_RECONNECT_SECURE;
  return DBT_OK;
}
