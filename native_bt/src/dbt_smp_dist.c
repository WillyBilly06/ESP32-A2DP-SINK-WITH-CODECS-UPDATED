#include "dbt_smp_dist.h"
#include <string.h>
static void wipe(void *v, unsigned n) {
  volatile uint8_t *p = v;
  while (n--)
    *p++ = 0;
}
static uint8_t next(uint8_t mask, uint8_t step) {
  static const uint8_t codes[5] = {6, 7, 8, 9, 10};
  static const uint8_t bits[5] = {1, 1, 2, 2, 4};
  for (unsigned i = step; i < 5; i++)
    if (mask & bits[i])
      return codes[i];
  return 0;
}
/* phase 0 is peer first for central, local first for peripheral. */
static void start_phase(dbt_smp_dist_t *s) {
  if (s->phase != 0)
    return;
  s->phase = s->initiator ? 2 : 1;
  s->step = 0;
  uint8_t mask = s->phase == 1 ? s->local_mask : s->peer_mask;
  if (!next(mask, 0)) {
    s->phase = s->phase == 1 ? 2 : 1;
  }
  mask = s->phase == 1 ? s->local_mask : s->peer_mask;
  if (!next(mask, 0))
    s->phase = 3;
}
static void step_done(dbt_smp_dist_t *s, uint8_t code) {
  s->step = (uint8_t)(code - 6 + 1);
  uint8_t mask = s->phase == 1 ? s->local_mask : s->peer_mask;
  if (next(mask, s->step))
    return;
  if (s->phase == 2 && (!s->initiator || !s->local_mask))
    s->peer_final_received = true;
  if (s->phase == 1)
    wipe(&s->local, sizeof(s->local));
  /* phase 3 indicates first phase complete; 4 is all complete. */
  if ((s->initiator && s->phase == 2) || (!s->initiator && s->phase == 1)) {
    s->phase = s->phase == 1 ? 2 : 1;
    s->step = 0;
    mask = s->phase == 1 ? s->local_mask : s->peer_mask;
    if (!next(mask, 0))
      s->phase = 4;
  } else
    s->phase = 4;
}
static void fail(dbt_smp_dist_t *s) {
  wipe(&s->local, sizeof(s->local));
  wipe(&s->peer, sizeof(s->peer));
  wipe(s->tx, sizeof(s->tx));
  s->tx_size = 0;
  s->final_wait_ack = false;
  s->early_final_acked = false;
  s->final_ll_acked = false;
  s->failed = true;
}
dbt_result_t dbt_smp_dist_open(dbt_smp_dist_t *s, uint32_t gen, bool initiator, bool sc, uint8_t size,
                               uint8_t lm, uint8_t pm, bool encrypted, const dbt_smp_dist_keys_t *local) {
  if (!s || !gen || size < 7 || size > 16 || (lm & ~7) || (pm & ~7) || (sc && ((lm | pm) & 1)) ||
      !encrypted || (lm && !local))
    return DBT_INVALID;
  if ((lm & 2) &&
      (local->identity_type > 1 || (local->identity_type == 1 && (local->identity[5] & 0xc0) != 0xc0)))
    return DBT_INVALID;
  if (lm & 1)
    for (unsigned i = size; i < 16; i++)
      if (local->ltk[i])
        return DBT_INVALID;
  memset(s, 0, sizeof(*s));
  s->generation = gen;
  s->initiator = initiator;
  s->secure_connections = sc;
  s->key_size = size;
  s->local_mask = lm;
  s->peer_mask = pm;
  if (lm)
    s->local = *local;
  start_phase(s);
  if (s->phase == 3)
    s->phase = 4;
  return DBT_OK;
}
void dbt_smp_dist_close(dbt_smp_dist_t *s) {
  if (s)
    wipe(s, sizeof(*s));
}
dbt_result_t dbt_smp_dist_tx(dbt_smp_dist_t *s, const uint8_t **p, uint8_t *n) {
  if (!s || !p || !n || !s->generation)
    return DBT_INVALID;
  if (s->failed)
    return DBT_INVALID;
  if (s->phase != 1)
    return DBT_EMPTY;
  if (!s->tx_size) {
    if (s->tx_serial == UINT32_MAX) {
      fail(s);
      return DBT_INVALID;
    }
    ++s->tx_serial;
    uint8_t code = next(s->local_mask, s->step);
    s->tx[0] = code;
    if (code == 6) {
      memcpy(s->tx + 1, s->local.ltk, 16);
      s->tx_size = 17;
    } else if (code == 7) {
      s->tx[1] = (uint8_t)s->local.ediv;
      s->tx[2] = (uint8_t)(s->local.ediv >> 8);
      memcpy(s->tx + 3, s->local.rand, 8);
      s->tx_size = 11;
    } else if (code == 8) {
      memcpy(s->tx + 1, s->local.irk, 16);
      s->tx_size = 17;
    } else if (code == 9) {
      s->tx[1] = s->local.identity_type;
      memcpy(s->tx + 2, s->local.identity, 6);
      s->tx_size = 8;
    } else if (code == 10) {
      memcpy(s->tx + 1, s->local.csrk, 16);
      s->tx_size = 17;
    } else
      return DBT_INVALID;
  }
  *p = s->tx;
  *n = s->tx_size;
  return DBT_OK;
}
dbt_result_t dbt_smp_dist_tx_token(const dbt_smp_dist_t *s, uint32_t *serial) {
  if (!s || !s->generation || !serial)
    return DBT_INVALID;
  if (!s->tx_size || s->failed || s->phase != 1)
    return DBT_EMPTY;
  *serial = s->tx_serial;
  return DBT_OK;
}
dbt_result_t dbt_smp_dist_tx_admitted(dbt_smp_dist_t *s, uint32_t gen, uint32_t serial, bool admitted) {
  if (!s || !s->generation || s->generation != gen || serial != s->tx_serial || !s->tx_size || s->failed ||
      s->phase != 1)
    return DBT_INVALID;
  uint8_t code = s->tx[0];
  wipe(s->tx, sizeof(s->tx));
  s->tx_size = 0;
  if (!admitted) {
    fail(s);
    return DBT_OK;
  }
  ++s->local_pdus_admitted;
  /* The central sends the final key if it has any local keys; otherwise the
   * peripheral's last local key is the global final key. */
  if (!next(s->local_mask, (uint8_t)(code - 5)) && (s->initiator || !s->peer_mask)) {
    s->final_serial = s->tx_serial;
    if (s->early_final_acked) {
      wipe(&s->local, sizeof(s->local));
      s->final_ll_acked = true;
      s->early_final_acked = false;
      s->phase = 4;
    } else {
      s->final_wait_ack = true;
      s->phase = 5;
    }
  } else {
    step_done(s, code);
  }
  return DBT_OK;
}
dbt_result_t dbt_smp_dist_final_pending(const dbt_smp_dist_t *s, uint32_t *serial) {
  if (!s || !s->generation || !serial)
    return DBT_INVALID;
  if (s->failed || !s->final_wait_ack || s->phase != 5)
    return DBT_EMPTY;
  *serial = s->final_serial;
  return DBT_OK;
}
dbt_result_t dbt_smp_dist_final_ack(dbt_smp_dist_t *s, uint32_t gen, uint32_t serial, bool acknowledged) {
  if (!s || !s->generation || s->generation != gen || s->failed)
    return DBT_INVALID;
  if (s->phase == 1 && s->tx_size && serial == s->tx_serial &&
      !next(s->local_mask, (uint8_t)(s->tx[0] - 5)) && (s->initiator || !s->peer_mask)) {
    if (s->early_final_acked)
      return DBT_INVALID;
    if (!acknowledged) {
      fail(s);
      return DBT_OK;
    }
    s->early_final_acked = true;
    return DBT_OK;
  }
  if (!s->final_wait_ack || s->phase != 5 || s->final_serial != serial)
    return DBT_INVALID;
  s->final_wait_ack = false;
  if (!acknowledged) {
    fail(s);
    return DBT_OK;
  }
  wipe(&s->local, sizeof(s->local));
  s->final_ll_acked = true;
  s->phase = 4;
  return DBT_OK;
}
dbt_result_t dbt_smp_dist_hci_buffers_completed(dbt_smp_dist_t *s, uint32_t gen, uint16_t count) {
  if (!s || !s->generation || s->generation != gen || !count)
    return DBT_INVALID;
  if (s->failed)
    return DBT_BUSY;
  if (UINT32_MAX - s->hci_buffers_completed < count)
    s->hci_buffers_completed = UINT32_MAX;
  else
    s->hci_buffers_completed += count;
  return DBT_OK;
}
dbt_result_t dbt_smp_dist_receive(dbt_smp_dist_t *s, const uint8_t *p, uint8_t n) {
  if (!s || !p || !n || !s->generation)
    return DBT_INVALID;
  if (s->failed || s->phase != 2)
    return DBT_BUSY;
  uint8_t code = next(s->peer_mask, s->step);
  uint8_t size = code == 7 ? 11 : code == 9 ? 8 : 17;
  if (p[0] != code || n != size) {
    fail(s);
    return DBT_INVALID;
  }
  if (code == 6) {
    for (unsigned i = s->key_size; i < 16; i++)
      if (p[1 + i]) {
        fail(s);
        return DBT_INVALID;
      }
    memcpy(s->peer.ltk, p + 1, 16);
  } else if (code == 7) {
    s->peer.ediv = (uint16_t)(p[1] | ((uint16_t)p[2] << 8));
    memcpy(s->peer.rand, p + 3, 8);
  } else if (code == 8)
    memcpy(s->peer.irk, p + 1, 16);
  else if (code == 9) {
    if (p[1] > 1 || (p[1] == 1 && (p[7] & 0xc0) != 0xc0)) {
      fail(s);
      return DBT_INVALID;
    }
    s->peer.identity_type = p[1];
    memcpy(s->peer.identity, p + 2, 6);
  } else if (code == 10)
    memcpy(s->peer.csrk, p + 1, 16);
  step_done(s, code);
  return DBT_OK;
}
dbt_result_t dbt_smp_dist_encryption_lost(dbt_smp_dist_t *s, uint32_t gen) {
  if (!s || !s->generation || s->generation != gen)
    return DBT_INVALID;
  fail(s);
  return DBT_OK;
}
dbt_result_t dbt_smp_dist_take(dbt_smp_dist_t *s, dbt_smp_dist_keys_t *peer) {
  if (!s || !peer || !s->generation)
    return DBT_INVALID;
  if (s->failed || s->phase != 4 || s->taken)
    return DBT_BUSY;
  *peer = s->peer;
  wipe(&s->peer, sizeof(s->peer));
  s->taken = true;
  return DBT_OK;
}
