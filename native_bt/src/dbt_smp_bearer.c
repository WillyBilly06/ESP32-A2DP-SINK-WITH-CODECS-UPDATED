#include "dbt_smp_bearer.h"
#include <string.h>
static void wipe(void *v, unsigned n) {
  volatile uint8_t *p = v;
  while (n--)
    *p++ = 0;
}
static bool matches(const dbt_smp_bearer_t *b, uint16_t handle, uint32_t gen) {
  return b && b->engine.used && b->engine.handle == handle && b->engine.generation == gen;
}
dbt_result_t dbt_smp_bearer_open(dbt_smp_bearer_t *b, uint16_t h, uint32_t g, uint8_t role, uint8_t lt,
                                 const uint8_t local[6], uint8_t pt, const uint8_t peer[6], uint8_t min,
                                 bool allow) {
  if (!b)
    return DBT_INVALID;
  memset(b, 0, sizeof(*b));
  return dbt_smp_open(&b->engine, h, g, role, lt, local, pt, peer, min, allow);
}
void dbt_smp_bearer_close(dbt_smp_bearer_t *b) {
  if (b)
    wipe(b, sizeof(*b));
}
static dbt_result_t legacy_deliver(dbt_smp_bearer_t *b, const uint8_t *p, uint8_t n, uint32_t now) {
  if (b->engine.state == DBT_SMP_DISTRIBUTION && p[0] != 5)
    return dbt_smp_dist_receive_legacy(&b->engine, p, n, now);
  return dbt_smp_receive(&b->engine, p, n, now);
}
dbt_result_t dbt_smp_bearer_receive(dbt_smp_bearer_t *b, uint16_t h, uint32_t g, uint16_t cid,
                                    const uint8_t *p, uint16_t n, uint32_t now) {
  if (!matches(b, h, g) || cid != DBT_SMP_LE_CID || !p || !n || n > 17)
    return DBT_INVALID;
  if (b->pending_size)
    return DBT_BUSY;
  dbt_result_t rc = legacy_deliver(b, p, (uint8_t)n, now);
  if (rc == DBT_BUSY && b->engine.state != DBT_SMP_FAILED && b->engine.state != DBT_SMP_TIMED_OUT) {
    memcpy(b->pending, p, n);
    b->pending_size = (uint8_t)n;
    return DBT_OK;
  }
  return rc;
}
dbt_result_t dbt_smp_bearer_tx(dbt_smp_bearer_t *b, uint16_t h, uint32_t g, const uint8_t **p, uint8_t *n) {
  if (!matches(b, h, g))
    return DBT_INVALID;
  if (b->engine.state == DBT_SMP_DISTRIBUTION)
    return DBT_UNSUPPORTED;
  return dbt_smp_tx(&b->engine, p, n);
}
dbt_result_t dbt_smp_bearer_tx_done(dbt_smp_bearer_t *b, uint16_t h, uint32_t g, bool admitted) {
  if (!matches(b, h, g))
    return DBT_INVALID;
  if (b->engine.state == DBT_SMP_DISTRIBUTION)
    return DBT_UNSUPPORTED;
  if (!b->engine.tx_size)
    return DBT_INVALID;
  dbt_smp_tx_done(&b->engine, admitted);
  return DBT_OK;
}
dbt_result_t dbt_smp_bearer_pump(dbt_smp_bearer_t *b, uint16_t h, uint32_t g, uint32_t now) {
  if (!matches(b, h, g))
    return DBT_INVALID;
  dbt_smp_poll(&b->engine, now);
  if (b->engine.state == DBT_SMP_FAILED || b->engine.state == DBT_SMP_TIMED_OUT) {
    wipe(b->pending, sizeof(b->pending));
    b->pending_size = 0;
    return DBT_INVALID;
  }
  if (!b->pending_size)
    return DBT_EMPTY;
  dbt_result_t rc = legacy_deliver(b, b->pending, b->pending_size, now);
  if (rc != DBT_BUSY) {
    wipe(b->pending, sizeof(b->pending));
    b->pending_size = 0;
  }
  return rc;
}
dbt_result_t dbt_smp_bearer_dist_tx(dbt_smp_bearer_t *b, uint16_t h, uint32_t g, const uint8_t **p,
                                    uint8_t *n, uint32_t now) {
  if (!matches(b, h, g))
    return DBT_INVALID;
  return dbt_smp_dist_tx_legacy(&b->engine, p, n, now);
}
dbt_result_t dbt_smp_bearer_dist_token(const dbt_smp_bearer_t *b, uint16_t h, uint32_t g, uint32_t *serial) {
  if (!matches(b, h, g))
    return DBT_INVALID;
  return dbt_smp_dist_tx_token(&b->engine.dist, serial);
}
dbt_result_t dbt_smp_bearer_dist_admitted(dbt_smp_bearer_t *b, uint16_t h, uint32_t g, uint32_t serial,
                                          bool admitted, uint32_t now) {
  if (!matches(b, h, g))
    return DBT_INVALID;
  if (b->engine.state != DBT_SMP_DISTRIBUTION || !b->engine.dist.tx_size)
    return DBT_EMPTY;
  return dbt_smp_dist_admitted_legacy(&b->engine, g, serial, admitted, now);
}
dbt_result_t dbt_smp_bearer_dist_ack(dbt_smp_bearer_t *b, uint16_t h, uint32_t g, uint32_t serial, bool ack,
                                     uint32_t now) {
  if (!matches(b, h, g))
    return DBT_INVALID;
  return dbt_smp_dist_final_ack_legacy(&b->engine, g, serial, ack, now);
}
static bool sc_matches(const dbt_smp_sc_bearer_t *b, uint16_t h, uint32_t g) {
  return b && b->engine.used && b->engine.handle == h && b->engine.generation == g;
}
void dbt_smp_sc_bearer_close(dbt_smp_sc_bearer_t *b) {
  if (b)
    wipe(b, sizeof(*b));
}
static dbt_result_t sc_deliver(dbt_smp_sc_bearer_t *b, const uint8_t *p, uint8_t n, uint32_t now) {
  if (b->engine.state == DBT_SC_DISTRIBUTION && p[0] != 5)
    return dbt_smp_sc_dist_receive(&b->engine, p, n, now);
  return dbt_smp_sc_receive(&b->engine, p, n, now);
}
dbt_result_t dbt_smp_sc_bearer_pump(dbt_smp_sc_bearer_t *b, uint16_t h, uint32_t g, uint32_t now) {
  if (!sc_matches(b, h, g))
    return DBT_INVALID;
  dbt_smp_sc_poll(&b->engine, now);
  if (b->engine.state == DBT_SC_FAILED || b->engine.state == DBT_SC_TIMED_OUT) {
    wipe(b->pending, sizeof(b->pending));
    b->pending_size = 0;
    return DBT_INVALID;
  }
  if (!b->pending_size)
    return DBT_EMPTY;
  dbt_result_t rc = sc_deliver(b, b->pending, b->pending_size, now);
  if (rc != DBT_BUSY) {
    wipe(b->pending, sizeof(b->pending));
    b->pending_size = 0;
  }
  return rc;
}
dbt_result_t dbt_smp_sc_bearer_receive(dbt_smp_sc_bearer_t *b, uint16_t h, uint32_t g, uint16_t cid,
                                       const uint8_t *p, uint16_t n, uint32_t now) {
  if (!sc_matches(b, h, g) || cid != DBT_SMP_LE_CID || !p || !n || n > 65)
    return DBT_INVALID;
  if (b->pending_size)
    return DBT_BUSY;
  dbt_result_t rc = sc_deliver(b, p, (uint8_t)n, now);
  if (rc == DBT_BUSY && b->engine.state != DBT_SC_FAILED && b->engine.state != DBT_SC_TIMED_OUT) {
    memcpy(b->pending, p, n);
    b->pending_size = (uint8_t)n;
    return DBT_OK;
  }
  return rc;
}
dbt_result_t dbt_smp_sc_bearer_tx(dbt_smp_sc_bearer_t *b, uint16_t h, uint32_t g, const uint8_t **p,
                                  uint8_t *n, uint32_t now) {
  if (!sc_matches(b, h, g))
    return DBT_INVALID;
  dbt_smp_sc_poll(&b->engine, now);
  if (b->engine.state == DBT_SC_DISTRIBUTION)
    return dbt_smp_sc_dist_tx(&b->engine, p, n, now);
  return dbt_smp_sc_tx(&b->engine, p, n);
}
dbt_result_t dbt_smp_sc_bearer_dist_token(const dbt_smp_sc_bearer_t *b, uint16_t h, uint32_t g,
                                          uint32_t *serial) {
  if (!sc_matches(b, h, g))
    return DBT_INVALID;
  return dbt_smp_dist_tx_token(&b->engine.dist, serial);
}
dbt_result_t dbt_smp_sc_bearer_tx_admitted(dbt_smp_sc_bearer_t *b, uint16_t h, uint32_t g, bool admitted,
                                           uint32_t now) {
  if (!sc_matches(b, h, g))
    return DBT_INVALID;
  if (b->engine.state == DBT_SC_DISTRIBUTION)
    return DBT_UNSUPPORTED;
  if (!b->engine.tx_size)
    return DBT_EMPTY;
  dbt_smp_sc_tx_done(&b->engine, admitted, now);
  return DBT_OK;
}
dbt_result_t dbt_smp_sc_bearer_dist_admitted(dbt_smp_sc_bearer_t *b, uint16_t h, uint32_t g, uint32_t serial,
                                             bool admitted, uint32_t now) {
  if (!sc_matches(b, h, g))
    return DBT_INVALID;
  return dbt_smp_sc_dist_tx_admitted(&b->engine, g, serial, admitted, now);
}
dbt_result_t dbt_smp_sc_bearer_dist_ack(dbt_smp_sc_bearer_t *b, uint16_t h, uint32_t g, uint32_t serial,
                                        bool ack, uint32_t now) {
  if (!sc_matches(b, h, g))
    return DBT_INVALID;
  return dbt_smp_sc_dist_final_ack(&b->engine, g, serial, ack, now);
}
