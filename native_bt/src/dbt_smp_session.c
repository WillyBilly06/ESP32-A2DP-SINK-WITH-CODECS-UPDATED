#include "dbt_smp_session.h"
#include <string.h>
#define SMP_CID 6u
#define LE_ENABLE_ENCRYPTION 0x2019u
#define LE_LTK_REPLY 0x201au
#define LE_LTK_NEGATIVE 0x201bu
typedef struct {
  dbt_smp_bearer_t *legacy;
  dbt_smp_sc_bearer_t *sc;
} selected_t;
_Static_assert(sizeof(dbt_smp_sc_bearer_t) + _Alignof(dbt_smp_sc_bearer_t) - 1u <= UINT16_MAX,
               "SC bearer allocation fits allocator");
_Static_assert(sizeof(dbt_smp_bearer_t) + _Alignof(dbt_smp_bearer_t) - 1u <= UINT16_MAX,
               "legacy bearer allocation fits allocator");
static selected_t selected(const dbt_smp_session_t *s) {
  selected_t out = {(dbt_smp_bearer_t *)s->bearer, (dbt_smp_sc_bearer_t *)s->bearer};
  return out;
}
static void wipe(void *v, unsigned n) {
  volatile uint8_t *p = v;
  while (n--)
    *p++ = 0;
}
static bool current(const dbt_smp_session_t *s) {
  if (!s || !s->runtime || !s->runtime->ready || s->runtime->host.state != DBT_HOST_IDENTIFIED)
    return false;
  const dbt_link_t *l = dbt_acl_find(&s->runtime->acl, s->handle);
  return l && l->kind == DBT_LINK_LE && l->generation == s->generation;
}
static void fail(dbt_smp_session_t *s) {
  if (!s || s->phase == DBT_SMP_SESSION_FAILED || s->phase == DBT_SMP_SESSION_CLOSED)
    return;
  s->phase = DBT_SMP_SESSION_FAILED;
  s->crypto_issued = false;
  s->store_issued = false;
  if (s->rx_queue) wipe(s->rx_queue,DBT_SMP_SESSION_RX_DEPTH*sizeof(*s->rx_queue));
  s->rx_head = s->rx_count = 0;
  wipe(s->pending_key, 16);
  s->key_ready = false;
  wipe(&s->config, sizeof(s->config));
  if (s->bearer)
    wipe(s->bearer, s->bearer_size);
  if (!s->result_reported && s->feature.guard) {
    (void)dbt_smp_features_report_result(&s->feature, s->generation, false, s->now);
    s->result_reported = true;
  }
  if (s->command_pending && s->runtime) {
    dbt_command_cancel_result_t result=dbt_host_cancel_command(&s->runtime->host,s->command_token);
    if (result!=DBT_COMMAND_CANCELLED &&
        (s->command_opcode==LE_ENABLE_ENCRYPTION || s->command_opcode==LE_LTK_REPLY) &&
        !s->encryption_event_seen) s->uncertain_generation=true;
  }
  if (s->command_accepted && !s->encryption_event_seen &&
      (s->command_opcode==LE_ENABLE_ENCRYPTION || s->command_opcode==LE_LTK_REPLY))
    s->uncertain_generation=true;
}
static dbt_smp_dist_t *distribution(dbt_smp_session_t *s) {
  if (!s || !s->bearer)
    return 0;
  selected_t references = selected(s);
  selected_t *b = &references;
  if (s->bearer_kind == DBT_SMP_SESSION_SC && b->sc->engine.state == DBT_SC_DISTRIBUTION)
    return &b->sc->engine.dist;
  if (s->bearer_kind == DBT_SMP_SESSION_LEGACY && b->legacy->engine.state == DBT_SMP_DISTRIBUTION)
    return &b->legacy->engine.dist;
  return 0;
}
dbt_result_t dbt_smp_session_open(dbt_smp_session_t *s, dbt_runtime_t *r, uint16_t handle, uint32_t gen,
                                  uint8_t lt, const uint8_t local[6], uint8_t pt, const uint8_t peer[6],
                                  dbt_smp_guard_t *guard, const dbt_smp_session_config_t *cfg, uint32_t now) {
  if (!s || !r || !local || !peer || !guard || !cfg || !gen || lt > 1 || pt > 1 || s->runtime)
    return DBT_INVALID;
  if (!r->ready)
    return DBT_BUSY;
  if (cfg->features.support_sc && r->maximum_mtu < 65)
    return DBT_NO_SPACE;
  const dbt_link_t *link = dbt_acl_find(&r->acl, handle);
  if (!link || link->kind != DBT_LINK_LE || link->generation != gen || link->role > 1 || link->encrypted)
    return DBT_INVALID;
  if (link->peer_address_type <= 1 && (link->peer_address_type != pt || memcmp(link->peer, peer, 6)))
    return DBT_INVALID;
  dbt_result_t rc = dbt_channels_register_fixed(&r->channels, DBT_LINK_LE, SMP_CID);
  if (rc != DBT_OK)
    return rc;
  memset(s, 0, sizeof(*s));
  s->runtime = r;
  s->guard = guard;
  s->handle = handle;
  s->generation = gen;
  s->role = link->role;
  s->local_type = lt;
  s->peer_type = pt;
  memcpy(s->local, local, 6);
  memcpy(s->peer, peer, 6);
  s->config = *cfg;
  s->now = now;
  rc = dbt_smp_features_open(&s->feature, handle, gen, s->role == 0, &cfg->features);
  if (rc == DBT_OK)
    rc = dbt_smp_features_set_guard(&s->feature, guard, now);
  if (rc != DBT_OK) {
    wipe(s, sizeof(*s));
    return rc;
  }
  s->rx_queue=(dbt_smp_session_rx_t *)r->hooks.allocate(r->hooks.context,
      (uint16_t)(DBT_SMP_SESSION_RX_DEPTH*sizeof(*s->rx_queue)));
  if (!s->rx_queue) { dbt_smp_features_close(&s->feature); wipe(s,sizeof(*s)); return DBT_NO_SPACE; }
  memset(s->rx_queue,0,DBT_SMP_SESSION_RX_DEPTH*sizeof(*s->rx_queue));
  s->phase = DBT_SMP_SESSION_FEATURES;
  return DBT_OK;
}
dbt_result_t dbt_smp_session_start(dbt_smp_session_t *s, uint32_t now) {
  if (!s || s->phase != DBT_SMP_SESSION_FEATURES || s->role != 0 || !current(s))
    return DBT_INVALID;
  s->now = now;
  return dbt_smp_features_start(&s->feature, now);
}
dbt_result_t dbt_smp_session_receive(dbt_smp_session_t *s, const dbt_link_t *link, uint16_t cid,
                                     const uint8_t *pdu, uint16_t n) {
  if (!s || !link || !pdu || !n || n > sizeof(s->rx_queue[0].data) || cid != SMP_CID || link->kind != DBT_LINK_LE ||
      link->handle != s->handle || link->generation != s->generation || !current(s))
    return DBT_INVALID;
  s->now = s->runtime->now_ms;
  if (s->closing || s->phase == DBT_SMP_SESSION_FAILED || s->phase == DBT_SMP_SESSION_CLOSED)
    return DBT_BUSY;
  if (!s->rx_queue) return DBT_INVALID;
  if (s->rx_count==DBT_SMP_SESSION_RX_DEPTH) {
    fail(s);
    return DBT_NO_SPACE;
  }
  dbt_smp_session_rx_t *item=&s->rx_queue[(s->rx_head+s->rx_count)%DBT_SMP_SESSION_RX_DEPTH];
  memcpy(item->data,pdu,n); item->size=(uint8_t)n; ++s->rx_count;
  return DBT_OK;
}
dbt_result_t dbt_smp_session_event(dbt_smp_session_t *s, const dbt_hci_packet_t *p) {
  if (!s || !p || p->type != DBT_H4_EVENT || !current(s))
    return DBT_INVALID;
  s->now = s->runtime->now_ms;
  if (s->phase == DBT_SMP_SESSION_FAILED || s->closing)
    return DBT_BUSY;
  if (p->event == 0x3e && p->size == 13 && p->payload && p->payload[0] == 5) {
    if (dbt_read_le16(p->payload + 1) != s->handle)
      return DBT_UNSUPPORTED;
    if (s->role != 1) {
      fail(s);
      return DBT_INVALID;
    }
    if (s->ltk_request) {
      fail(s);
      return DBT_INVALID;
    }
    if (s->command_pending || s->key_consumed) {
      fail(s);
      return DBT_INVALID;
    }
    memcpy(s->ltk_rand, p->payload + 3, 8);
    s->ltk_ediv = dbt_read_le16(p->payload + 11);
    s->ltk_invalid =
        s->ltk_ediv != 0 || (s->phase != DBT_SMP_SESSION_SC && s->phase != DBT_SMP_SESSION_LEGACY);
    for (unsigned i = 0; i < 8; i++)
      if (s->ltk_rand[i])
        s->ltk_invalid = true;
    s->ltk_request = true;
    return DBT_OK;
  }
  if (p->event == 0x08 && p->size == 4 && p->payload && dbt_read_le16(p->payload + 1) == s->handle) {
    if (p->payload[0] || p->payload[3] != 1) {
      if (s->command_accepted) s->encryption_event_seen = true;
      fail(s);
      return DBT_OK;
    }
    /* A queued or merely submitted command cannot bind an older encryption
     * event to this key. Accept only after the matching command result. */
    if (!s->key_consumed || !s->command_accepted) {
      fail(s);
      return DBT_INVALID;
    }
    s->encryption_event_seen = true;
    s->encrypted_event_pending = true;
    return DBT_OK;
  }
  return DBT_UNSUPPORTED;
}
void dbt_smp_session_link(dbt_smp_session_t *s, const dbt_link_change_t *change) {
  if (s && change && change->kind == DBT_LINK_REMOVED && change->handle == s->handle &&
      change->generation == s->generation) {
    if (s->runtime)
      s->now = s->runtime->now_ms;
    fail(s);
  }
}
static dbt_result_t setup_selected(dbt_smp_session_t *s) {
  bool sc = false;
  uint8_t req[7], rsp[7];
  dbt_result_t rc = dbt_smp_features_take(&s->feature, req, rsp, &sc);
  if (rc != DBT_OK)
    return rc;
  size_t bytes = sc ? sizeof(dbt_smp_sc_bearer_t) : sizeof(dbt_smp_bearer_t);
  size_t alignment = sc ? _Alignof(dbt_smp_sc_bearer_t) : _Alignof(dbt_smp_bearer_t);
  uint8_t *raw = s->runtime->hooks.allocate(s->runtime->hooks.context, (uint16_t)(bytes + alignment - 1u));
  if (!raw)
    return DBT_NO_SPACE;
  size_t offset = (alignment - ((uintptr_t)raw % alignment)) % alignment;
  s->bearer = raw + offset;
  s->bearer_size = (uint16_t)bytes;
  memset(s->bearer, 0, bytes);
  s->allocation = raw;
  selected_t references = selected(s);
  selected_t *b = &references;
  bool oob = req[2] && rsp[2];
  bool mitm = ((req[3] | rsp[3]) & 4) != 0;
  bool numeric = sc && mitm && !oob && (req[1] == 1 || req[1] == 4) && (rsp[1] == 1 || rsp[1] == 4);
  bool passkey = mitm && !oob && !numeric;
  if (sc) {
    s->bearer_kind = DBT_SMP_SESSION_SC;
    rc = dbt_smp_features_open_sc(&s->feature, &b->sc->engine, s->local_type, s->local, s->peer_type, s->peer,
                                  s->now);
    if (rc == DBT_OK && passkey)
      rc = s->config.have_passkey ? dbt_smp_sc_set_passkey(&b->sc->engine, s->config.passkey)
                                  : DBT_UNSUPPORTED;
    if (rc == DBT_OK && oob)
      rc = s->config.have_oob
               ? dbt_smp_sc_set_oob(&b->sc->engine, s->config.sc_oob_public, s->config.sc_local_oob_random,
                                    s->config.sc_peer_oob_random, s->config.sc_peer_oob_confirm,
                                    s->config.oob_authenticated)
               : DBT_UNSUPPORTED;
    if (rc == DBT_OK && b->sc->engine.bonding)
      rc = dbt_smp_sc_check_bond_policy(&b->sc->engine, s->config.existing_bond, s->config.old_authenticated,
                                        s->config.old_key_size);
    if (rc == DBT_OK && b->sc->engine.local_mask)
      rc = s->config.have_local_keys ? dbt_smp_sc_set_distribution(&b->sc->engine, &s->config.local_keys)
                                     : DBT_UNSUPPORTED;
  } else {
    s->bearer_kind = DBT_SMP_SESSION_LEGACY;
    rc = dbt_smp_bearer_open(b->legacy, s->handle, s->generation, s->role, s->local_type, s->local,
                             s->peer_type, s->peer, s->config.features.min_key_size,
                             s->config.allow_just_works);
    if (rc == DBT_OK)
      rc = dbt_smp_set_max_key_size(&b->legacy->engine, s->config.features.max_key_size);
    if (rc == DBT_OK && passkey)
      rc = s->config.have_passkey
               ? dbt_smp_set_passkey(&b->legacy->engine, s->config.passkey, s->config.features.io)
               : DBT_UNSUPPORTED;
    if (rc == DBT_OK && oob)
      rc = s->config.have_oob
               ? dbt_smp_set_oob(&b->legacy->engine, s->config.legacy_oob_tk, s->config.oob_authenticated)
               : DBT_UNSUPPORTED;
    if (rc == DBT_OK && (req[3] & 1)) {
      uint8_t lm = s->role == 0 ? rsp[5] : rsp[6], pm = s->role == 0 ? rsp[6] : rsp[5];
      rc = s->config.have_local_keys
               ? dbt_smp_set_bonding(&b->legacy->engine, lm, pm, &s->config.local_keys,
                                     s->config.existing_bond, s->config.old_sc, s->config.old_authenticated,
                                     s->config.old_key_size)
               : DBT_UNSUPPORTED;
    }
    if (rc == DBT_OK)
      rc = dbt_smp_features_resume_legacy(&s->feature, &b->legacy->engine, s->now);
  }
  wipe(&s->config, sizeof(s->config));
  if (rc != DBT_OK) {
    wipe(s->bearer, s->bearer_size);
    s->runtime->hooks.release(s->runtime->hooks.context, raw);
    s->bearer = 0;
    s->allocation = 0;
    return rc;
  }
  s->phase = s->bearer_kind;
  return DBT_OK;
}
static void tx_done(void *context, dbt_tx_outcome_t outcome) {
  dbt_smp_session_t *s = context;
  if (!s || !s->tx_active)
    return;
  s->now = s->runtime->now_ms;
  s->tx_active = false;
  wipe(s->tx, sizeof(s->tx));
  s->tx_size = 0;
  if (s->phase == DBT_SMP_SESSION_FAILED || s->closing)
    return;
  bool admitted = outcome == DBT_TX_ADMITTED;
  selected_t references = selected(s);
  selected_t *b = &references;
  if (s->phase == DBT_SMP_SESSION_FEATURES)
    dbt_smp_features_tx_done(&s->feature, admitted, s->runtime->now_ms);
  else if (s->bearer_kind == DBT_SMP_SESSION_SC) {
    if (s->tx_distribution)
      (void)dbt_smp_sc_bearer_dist_admitted(b->sc, s->handle, s->generation, s->tx_serial, admitted,
                                            s->runtime->now_ms);
    else
      (void)dbt_smp_sc_bearer_tx_admitted(b->sc, s->handle, s->generation, admitted, s->runtime->now_ms);
  } else if (s->bearer_kind == DBT_SMP_SESSION_LEGACY) {
    if (s->tx_distribution)
      (void)dbt_smp_bearer_dist_admitted(b->legacy, s->handle, s->generation, s->tx_serial, admitted,
                                         s->runtime->now_ms);
    else
      (void)dbt_smp_bearer_tx_done(b->legacy, s->handle, s->generation, admitted);
  }
  s->tx_distribution = false;
  s->tx_serial = 0;
  if (!admitted)
    fail(s);
}
static void command_done(void *context, uint16_t opcode, dbt_command_outcome_t outcome, uint8_t status,
                         const uint8_t *returns, uint8_t n) {
  dbt_smp_session_t *s = context;
  if (!s || !s->command_pending || opcode != s->command_opcode)
    return;
  s->command_pending = false;
  s->command_token = 0;
  if (s->closing || s->phase == DBT_SMP_SESSION_FAILED || s->command_generation != s->generation ||
      !current(s))
    return;
  bool good = outcome == DBT_COMMAND_SUCCESS && status == 0;
  if (opcode == LE_LTK_REPLY || opcode == LE_LTK_NEGATIVE)
    good = good && n == 3 && returns && dbt_read_le16(returns + 1) == s->handle;
  if (!good || opcode == LE_LTK_NEGATIVE) {
    fail(s);
    return;
  }
  s->command_accepted = true;
}
static void send_pending(dbt_smp_session_t *s) {
  if (s->tx_active || s->phase == DBT_SMP_SESSION_FAILED)
    return;
  const uint8_t *p = 0;
  uint8_t n = 0;
  uint32_t token = 0;
  dbt_result_t rc = DBT_EMPTY;
  selected_t references = selected(s);
  selected_t *b = &references;
  s->tx_distribution = false;
  if (s->phase == DBT_SMP_SESSION_FEATURES)
    rc = dbt_smp_features_tx(&s->feature, &p, &n);
  else if (s->phase == DBT_SMP_SESSION_SC) {
    rc = dbt_smp_sc_bearer_tx(b->sc, s->handle, s->generation, &p, &n, s->now);
    if (rc == DBT_OK && b->sc->engine.state == DBT_SC_DISTRIBUTION) {
      s->tx_distribution = true;
      rc = dbt_smp_sc_bearer_dist_token(b->sc, s->handle, s->generation, &token);
    }
  } else if (s->phase == DBT_SMP_SESSION_LEGACY) {
    if (b->legacy->engine.state == DBT_SMP_DISTRIBUTION) {
      s->tx_distribution = true;
      rc = dbt_smp_bearer_dist_tx(b->legacy, s->handle, s->generation, &p, &n, s->now);
      if (rc == DBT_OK)
        rc = dbt_smp_bearer_dist_token(b->legacy, s->handle, s->generation, &token);
    } else
      rc = dbt_smp_bearer_tx(b->legacy, s->handle, s->generation, &p, &n);
  }
  if (rc == DBT_EMPTY || rc == DBT_BUSY)
    return;
  if (rc != DBT_OK || !p || !n || n > sizeof(s->tx)) {
    fail(s);
    return;
  }
  memcpy(s->tx, p, n);
  s->tx_size = n;
  s->tx_serial = token;
  rc = dbt_runtime_send_fixed(s->runtime, s->handle, SMP_CID, s->tx, n, tx_done, s);
  if (rc == DBT_OK)
    s->tx_active = true;
  else {
    wipe(s->tx, sizeof(s->tx));
    s->tx_size = 0;
    if (rc != DBT_BUSY && rc != DBT_NO_SPACE)
      fail(s);
  }
}
static void process_input(dbt_smp_session_t *s) {
  if (!s->rx_count || !s->rx_queue)
    return;
  dbt_smp_session_rx_t *item=&s->rx_queue[s->rx_head];
  dbt_result_t rc;
  selected_t references = selected(s);
  selected_t *b = &references;
  if (s->phase == DBT_SMP_SESSION_FEATURES)
    rc = dbt_smp_features_receive(&s->feature, item->data, item->size, s->now);
  else if (s->phase == DBT_SMP_SESSION_SC)
    rc = dbt_smp_sc_bearer_receive(b->sc, s->handle, s->generation, SMP_CID, item->data, item->size, s->now);
  else if (s->phase == DBT_SMP_SESSION_LEGACY)
    rc =
        dbt_smp_bearer_receive(b->legacy, s->handle, s->generation, SMP_CID, item->data, item->size, s->now);
  else
    return;
  if (rc == DBT_BUSY)
    return;
  wipe(item,sizeof(*item));
  s->rx_head=(uint8_t)((s->rx_head+1)%DBT_SMP_SESSION_RX_DEPTH); --s->rx_count;
  if (rc != DBT_OK)
    fail(s);
}
static void encryption_command(dbt_smp_session_t *s) {
  if (s->command_pending || s->command_accepted || s->phase == DBT_SMP_SESSION_FAILED)
    return;
  selected_t references = selected(s);
  selected_t *b = &references;
  if (s->role == 1 && !s->ltk_request)
    return;
  if (s->ltk_invalid) {
    uint8_t params[2] = {(uint8_t)s->handle, (uint8_t)(s->handle >> 8)};
    dbt_result_t rc = dbt_host_command_tracked(&s->runtime->host, LE_LTK_NEGATIVE, params, 2, DBT_COMMAND_COMPLETE,
                                               command_done, s, s->now, &s->command_token);
    if (rc == DBT_OK) {
      s->command_pending = true;
      s->command_opcode = LE_LTK_NEGATIVE;
      s->command_generation = s->generation;
    } else if (rc != DBT_BUSY && rc != DBT_NO_SPACE)
      fail(s);
    return;
  }
  if (!b)
    return;
  if (!s->key_ready && !s->key_consumed) {
    dbt_result_t rc = s->bearer_kind == DBT_SMP_SESSION_SC
                          ? dbt_smp_sc_take_ltk(&b->sc->engine, s->pending_key, &s->key_size)
                          : dbt_smp_take_stk(&b->legacy->engine, s->pending_key, &s->key_size);
    if (rc == DBT_BUSY)
      return;
    if (rc != DBT_OK) {
      fail(s);
      return;
    }
    s->key_ready = true;
  }
  if (!s->key_ready)
    return;
  uint8_t params[28] = {0};
  params[0] = (uint8_t)s->handle;
  params[1] = (uint8_t)(s->handle >> 8);
  uint16_t opcode = s->role == 0 ? LE_ENABLE_ENCRYPTION : LE_LTK_REPLY;
  uint8_t size = s->role == 0 ? 28 : 18;
  memcpy(params + (s->role == 0 ? 12 : 2), s->pending_key, 16);
  dbt_result_t rc =
      dbt_host_command_tracked(&s->runtime->host, opcode, params, size,
                               s->role == 0 ? DBT_COMMAND_STATUS : DBT_COMMAND_COMPLETE,
                               command_done, s, s->now, &s->command_token);
  wipe(params, sizeof(params));
  if (rc == DBT_OK) {
    s->command_pending = true;
    s->command_opcode = opcode;
    s->command_generation = s->generation;
    s->key_ready = false;
    s->key_consumed = true;
    wipe(s->pending_key, 16);
  } else if (rc != DBT_BUSY && rc != DBT_NO_SPACE)
    fail(s);
}
static void apply_encryption(dbt_smp_session_t *s) {
  if (!s->encrypted_event_pending || !s->command_accepted || !current(s))
    return;
  selected_t references = selected(s);
  selected_t *b = &references;
  s->encrypted_event_pending = false;
  dbt_result_t rc = s->bearer_kind == DBT_SMP_SESSION_SC
                        ? dbt_smp_sc_encryption(&b->sc->engine, s->generation, true)
                        : dbt_smp_encryption(&b->legacy->engine, s->generation, true);
  if (rc != DBT_OK)
    fail(s);
}
static bool store_pending(const dbt_smp_session_t *s) {
  if (!s->bearer)
    return false;
  selected_t references = selected(s);
  selected_t *b = &references;
  return (s->phase == DBT_SMP_SESSION_SC && b->sc->engine.state == DBT_SC_BOND_STORE) ||
         (s->phase == DBT_SMP_SESSION_LEGACY && b->legacy->engine.state == DBT_SMP_BOND_PENDING);
}
static void arm_store_deadline(dbt_smp_session_t *s, uint32_t now) {
  if (store_pending(s) && !s->store_wait_started) {
    s->store_wait_started = true;
    s->store_deadline = now + s->runtime->host.timeout_ms;
  }
}
void dbt_smp_session_poll(dbt_smp_session_t *s, uint32_t now) {
  if (!s || !s->runtime || s->closing || s->phase == DBT_SMP_SESSION_CLOSED ||
      s->phase == DBT_SMP_SESSION_FAILED)
    return;
  s->now = now;
  if (!current(s)) {
    fail(s);
    return;
  }
  /* Encryption Change and phase-3 ACL PDUs can be delivered in the same
   * runtime poll. Apply the correlated event before dispatching those PDUs. */
  apply_encryption(s);
  if (s->phase==DBT_SMP_SESSION_FAILED) return;
  arm_store_deadline(s, now);
  if (s->store_wait_started && (int32_t)(now - s->store_deadline) >= 0) {
    selected_t references = selected(s);
    selected_t *b = &references;
    dbt_smp_bond_job_t job;
    dbt_result_t rc = s->phase == DBT_SMP_SESSION_SC ? dbt_smp_sc_bond_job(&b->sc->engine, &job)
                                                     : dbt_smp_bond_job_legacy(&b->legacy->engine, &job);
    if (rc == DBT_OK) {
      if (s->phase == DBT_SMP_SESSION_SC)
        (void)dbt_smp_sc_bond_cancel(&b->sc->engine, s->generation, job.serial);
      else if (s->phase == DBT_SMP_SESSION_LEGACY)
        (void)dbt_smp_bond_cancel_legacy(&b->legacy->engine, s->generation, job.serial);
      wipe(&job, sizeof(job));
    }
    s->store_issued = false;
    fail(s);
    return;
  }
  if (s->ltk_invalid) {
    encryption_command(s);
    return;
  }
  if (s->phase == DBT_SMP_SESSION_FEATURES) {
    dbt_smp_features_poll(&s->feature, now);
    process_input(s);
    if (s->feature.state == DBT_FEATURE_FAILED || s->feature.state == DBT_FEATURE_TIMEOUT) {
      fail(s);
      return;
    }
    if (s->feature.state == DBT_FEATURE_READY && !s->tx_active) {
      if (setup_selected(s) != DBT_OK) {
        fail(s);
        return;
      }
    }
  }
  selected_t references = selected(s);
  selected_t *b = &references;
  if (s->phase == DBT_SMP_SESSION_SC) {
    dbt_smp_sc_poll(&b->sc->engine, now);
    if (b->sc->pending_size)
      (void)dbt_smp_sc_bearer_pump(b->sc, s->handle, s->generation, now);
    if (b->sc->engine.state == DBT_SC_FAILED || b->sc->engine.state == DBT_SC_TIMED_OUT) {
      fail(s);
      return;
    }
    process_input(s);
    for (unsigned i=1;i<DBT_SMP_SESSION_RX_DEPTH && s->rx_count && s->phase==DBT_SMP_SESSION_SC &&
         b->sc->engine.state==DBT_SC_DISTRIBUTION;i++) {
      uint8_t before=s->rx_count;
      process_input(s);
      if (s->rx_count==before) break;
    }
  } else if (s->phase == DBT_SMP_SESSION_LEGACY) {
    dbt_smp_poll(&b->legacy->engine, now);
    if (b->legacy->pending_size)
      (void)dbt_smp_bearer_pump(b->legacy, s->handle, s->generation, now);
    if (b->legacy->engine.state == DBT_SMP_FAILED || b->legacy->engine.state == DBT_SMP_TIMED_OUT) {
      fail(s);
      return;
    }
    process_input(s);
    for (unsigned i=1;i<DBT_SMP_SESSION_RX_DEPTH && s->rx_count && s->phase==DBT_SMP_SESSION_LEGACY &&
         b->legacy->engine.state==DBT_SMP_DISTRIBUTION;i++) {
      uint8_t before=s->rx_count;
      process_input(s);
      if (s->rx_count==before) break;
    }
  }
  if (s->phase != DBT_SMP_SESSION_SC && s->phase != DBT_SMP_SESSION_LEGACY) {
    send_pending(s);
    return;
  }
  bool needs_key = s->bearer_kind == DBT_SMP_SESSION_SC ? b->sc->engine.state == DBT_SC_ENCRYPTION
                                                        : b->legacy->engine.state == DBT_SMP_ENCRYPTION;
  if (needs_key || s->ltk_invalid)
    encryption_command(s);
  apply_encryption(s);
  if (s->phase == DBT_SMP_SESSION_FAILED)
    return;
  arm_store_deadline(s, now);
  bool secure = s->bearer_kind == DBT_SMP_SESSION_SC ? b->sc->engine.state == DBT_SC_SECURE
                                                     : b->legacy->engine.state == DBT_SMP_SECURE;
  if (secure && !s->result_reported) {
    if (dbt_smp_features_report_result(&s->feature, s->generation, true, now) != DBT_OK) {
      fail(s);
      return;
    }
    s->result_reported = true;
  }
  send_pending(s);
}
dbt_result_t dbt_smp_session_crypto_job(dbt_smp_session_t *s, dbt_smp_session_crypto_job_t *out) {
  if (!s || !out || !s->bearer || !current(s))
    return DBT_INVALID;
  if (s->crypto_issued)
    return DBT_BUSY;
  memset(out, 0, sizeof(*out));
  selected_t references = selected(s);
  selected_t *b = &references;
  dbt_result_t rc;
  if (s->phase == DBT_SMP_SESSION_SC) {
    out->kind = DBT_SMP_SESSION_CRYPTO_SC;
    rc = dbt_smp_sc_job(&b->sc->engine, &out->input.sc);
    if (rc == DBT_OK)
      s->crypto_serial = out->input.sc.serial;
  } else if (s->phase == DBT_SMP_SESSION_LEGACY) {
    out->kind = DBT_SMP_SESSION_CRYPTO_LEGACY;
    rc = dbt_smp_job(&b->legacy->engine, &out->input.legacy);
    if (rc == DBT_OK)
      s->crypto_serial = out->input.legacy.serial;
  } else
    return DBT_EMPTY;
  if (rc == DBT_OK) {
    s->crypto_kind = out->kind;
    s->crypto_issued = true;
  } else
    wipe(out, sizeof(*out));
  return rc;
}
dbt_result_t dbt_smp_session_crypto_complete(dbt_smp_session_t *s, uint32_t gen, uint32_t serial,
                                             bool success, const dbt_smp_job_result_t *legacy,
                                             const dbt_smp_sc_result_t *sc, uint32_t now) {
  if (!s || !s->crypto_issued || !s->bearer || !current(s) || gen != s->generation ||
      serial != s->crypto_serial)
    return DBT_INVALID;
  s->now = now;
  selected_t references = selected(s);
  selected_t *b = &references;
  dbt_result_t rc;
  if (s->crypto_kind == DBT_SMP_SESSION_CRYPTO_SC && s->phase == DBT_SMP_SESSION_SC)
    rc = dbt_smp_sc_complete(&b->sc->engine, gen, serial, success, sc, now);
  else if (s->crypto_kind == DBT_SMP_SESSION_CRYPTO_LEGACY && s->phase == DBT_SMP_SESSION_LEGACY)
    rc = dbt_smp_complete(&b->legacy->engine, gen, serial, success, legacy, now);
  else
    return DBT_INVALID;
  if (rc == DBT_OK) {
    s->crypto_issued = false;
    s->crypto_kind = DBT_SMP_SESSION_CRYPTO_NONE;
  }
  return rc;
}
dbt_result_t dbt_smp_session_store_job(dbt_smp_session_t *s, dbt_smp_bond_job_t *out) {
  if (!s || !out || !s->bearer || !current(s))
    return DBT_INVALID;
  if (s->store_issued)
    return DBT_BUSY;
  selected_t references = selected(s);
  selected_t *b = &references;
  dbt_result_t rc;
  if (s->phase == DBT_SMP_SESSION_SC && b->sc->engine.state == DBT_SC_BOND_STORE)
    rc = dbt_smp_sc_bond_job(&b->sc->engine, out);
  else if (s->phase == DBT_SMP_SESSION_LEGACY && b->legacy->engine.state == DBT_SMP_BOND_PENDING)
    rc = dbt_smp_bond_job_legacy(&b->legacy->engine, out);
  else
    return DBT_EMPTY;
  if (rc == DBT_OK) {
    s->store_issued = true;
    s->store_serial = out->serial;
    arm_store_deadline(s, s->now);
  }
  return rc;
}
dbt_result_t dbt_smp_session_store_complete(dbt_smp_session_t *s, uint32_t gen, uint32_t serial, bool durable,
                                            uint32_t now) {
  if (!s || !s->store_issued || !s->bearer || !current(s) || gen != s->generation ||
      serial != s->store_serial)
    return DBT_INVALID;
  s->now = now;
  if ((int32_t)(now - s->store_deadline) >= 0) {
    dbt_smp_session_poll(s, now);
    return DBT_INVALID;
  }
  selected_t references = selected(s);
  selected_t *b = &references;
  dbt_result_t rc = s->phase == DBT_SMP_SESSION_SC
                        ? dbt_smp_sc_bond_complete(&b->sc->engine, gen, serial, durable)
                        : dbt_smp_bond_complete_legacy(&b->legacy->engine, gen, serial, durable);
  if (rc == DBT_OK || !durable)
    s->store_issued = false;
  if (rc == DBT_OK)
    s->store_wait_started = false;
  if (!durable)
    fail(s);
  return rc;
}
dbt_result_t dbt_smp_session_distribution_token(const dbt_smp_session_t *s, uint32_t *serial) {
  if (!s || !serial)
    return DBT_INVALID;
  if (s->tx_active && s->tx_distribution && s->tx_serial) {
    *serial = s->tx_serial;
    return DBT_OK;
  }
  dbt_smp_dist_t *d = distribution((dbt_smp_session_t *)s);
  return d ? dbt_smp_dist_final_pending(d, serial) : DBT_EMPTY;
}
dbt_result_t dbt_smp_session_final_ack(dbt_smp_session_t *s, uint32_t gen, uint32_t serial, bool ack,
                                       uint32_t now) {
  if (!s || !s->bearer || !current(s) || gen != s->generation)
    return DBT_INVALID;
  s->now = now;
  selected_t references = selected(s);
  selected_t *b = &references;
  if (s->phase == DBT_SMP_SESSION_SC)
    return dbt_smp_sc_bearer_dist_ack(b->sc, s->handle, gen, serial, ack, now);
  if (s->phase == DBT_SMP_SESSION_LEGACY)
    return dbt_smp_bearer_dist_ack(b->legacy, s->handle, gen, serial, ack, now);
  return DBT_INVALID;
}
dbt_result_t dbt_smp_session_numeric_value(const dbt_smp_session_t *s, uint32_t *value) {
  if (!s || !s->bearer || s->phase != DBT_SMP_SESSION_SC || !current(s))
    return DBT_INVALID;
  return dbt_smp_sc_numeric_value(&((dbt_smp_sc_bearer_t *)s->bearer)->engine, s->generation, value);
}
dbt_result_t dbt_smp_session_numeric_confirm(dbt_smp_session_t *s, bool accepted, uint32_t now) {
  if (!s || !s->bearer || s->phase != DBT_SMP_SESSION_SC || !current(s))
    return DBT_INVALID;
  return dbt_smp_sc_numeric_confirm(&((dbt_smp_sc_bearer_t *)s->bearer)->engine, s->generation, accepted,
                                    now);
}
dbt_result_t dbt_smp_session_security(const dbt_smp_session_t *s, dbt_smp_session_security_t *out) {
  if (!s || !out || !s->bearer || !current(s))
    return DBT_INVALID;
  memset(out, 0, sizeof(*out));
  selected_t references = selected(s);
  selected_t *b = &references;
  if (s->phase == DBT_SMP_SESSION_SC) {
    const dbt_smp_sc_t *e = &b->sc->engine;
    out->encrypted = e->encryption_verified;
    out->authenticated = out->encrypted && e->authenticated;
    out->bond_committed = e->bond_committed;
    out->peer_final_received = e->peer_final_received;
    out->final_ll_acked = e->final_ll_acked;
    out->key_size = out->encrypted ? e->key_size : 0;
    out->negotiated_local_keys = e->local_mask;
    out->negotiated_peer_keys = e->peer_mask;
    out->phase3_complete = out->encrypted && (e->state == DBT_SC_BOND_STORE || e->state == DBT_SC_SECURE);
  } else if (s->phase == DBT_SMP_SESSION_LEGACY) {
    const dbt_smp_t *e = &b->legacy->engine;
    out->encrypted = e->encryption_verified;
    out->authenticated = out->encrypted && e->authenticated;
    out->bond_committed = e->bond_committed;
    out->peer_final_received = e->peer_final_received;
    out->final_ll_acked = e->final_ll_acked;
    out->key_size = out->encrypted ? e->key_size : 0;
    out->negotiated_local_keys = e->local_mask;
    out->negotiated_peer_keys = e->peer_mask;
    out->phase3_complete = out->encrypted && (e->state == DBT_SMP_BOND_PENDING || e->state == DBT_SMP_SECURE);
  } else
    return DBT_BUSY;
  if (out->phase3_complete) {
    out->admitted_local_keys = out->negotiated_local_keys;
    out->received_peer_keys = out->negotiated_peer_keys;
  }
  /* Bonded reconnect is a separate dbt_smp_reconnect_t lifecycle. */
  out->peer_key_verified = false;
  return DBT_OK;
}
dbt_result_t dbt_smp_session_close(dbt_smp_session_t *s) {
  if (!s || !s->runtime)
    return DBT_INVALID;
  s->closing = true;
  if (s->command_pending)
    (void)dbt_host_cancel_command(&s->runtime->host,s->command_token);
  if (s->tx_active || s->command_pending)
    return DBT_BUSY;
  if (!s->result_reported && s->feature.guard) {
    (void)dbt_smp_features_report_result(&s->feature, s->generation, false, s->now);
    s->result_reported = true;
  }
  if (s->bearer) {
    wipe(s->bearer, s->bearer_size);
    s->runtime->hooks.release(s->runtime->hooks.context, s->allocation);
  }
  if (s->rx_queue) {
    wipe(s->rx_queue,DBT_SMP_SESSION_RX_DEPTH*sizeof(*s->rx_queue));
    s->runtime->hooks.release(s->runtime->hooks.context,(uint8_t *)s->rx_queue);
  }
  wipe(s, sizeof(*s));
  return DBT_OK;
}
