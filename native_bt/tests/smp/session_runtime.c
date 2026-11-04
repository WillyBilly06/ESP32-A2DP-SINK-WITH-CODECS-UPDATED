/* Paired in-process runtimes with an emulated HCI controller, no radio. */
#include "dbt_smp_session.h"
#include <stdlib.h>
#include <string.h>
#ifdef _WIN32
#define API __declspec(dllexport)
#else
#define API __attribute__((visibility("default")))
#endif
typedef struct peer {
  dbt_runtime_t runtime;
  dbt_smp_session_t session;
  dbt_smp_guard_t guard;
  uint8_t wire[DBT_MAX_H4_BYTES], *borrowed;
  uint16_t wire_size;
  unsigned allocated, released, errors, tx_commands, tx_acl;
  bool ready, fail_large, fail_rx;
} peer_t;
static peer_t peers[2];
static uint32_t now_ms;
static uint8_t central_key[16];
static bool have_central_key;
static bool invalid_ltk_request;
static bool burst_on_encrypt,burst_injected;
static unsigned negative_replies;
static const uint8_t addr_a[6] = {1, 2, 3, 4, 5, 6}, addr_b[6] = {7, 8, 9, 10, 11, 12};
static const uint8_t addr_rpa[6]={0xaa,0xfb,0x0d,0x94,0x81,0x70};
static const uint8_t identity_irk[16]={0x9b,0x7d,0x39,0x0a,0xa6,0x10,0x10,0x34,
                                       0x05,0xad,0xc8,0x57,0xa3,0x34,0x02,0xec};
static void w16(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
}
static uint8_t *allocate(void *ctx, uint16_t n) {
  peer_t *p = ctx;
  if ((p->fail_large && n > 1000) ||
      (p->fail_rx && n==DBT_SMP_SESSION_RX_DEPTH*sizeof(dbt_smp_session_rx_t)))
    return 0;
  uint8_t *mem = malloc((size_t)n + 1u);
  if (!mem)
    return 0;
  ++p->allocated;
  return mem + 1;
}
static void release(void *ctx, uint8_t *mem) {
  if (mem) {
    ++((peer_t *)ctx)->released;
    free(mem - 1);
  }
}
static bool can_send(void *ctx) { return ((peer_t *)ctx)->ready; }
static bool send(void *ctx, uint8_t *bytes, uint16_t size) {
  peer_t *p = ctx;
  if (!p->ready || size > sizeof(p->wire))
    return false;
  memcpy(p->wire, bytes, size);
  p->wire_size = size;
  p->borrowed = bytes;
  p->ready = false;
  return true;
}
static dbt_result_t event(peer_t *p, uint8_t code, const uint8_t *payload, uint8_t n) {
  uint8_t frame[260];
  frame[0] = 4;
  frame[1] = code;
  frame[2] = n;
  memcpy(frame + 3, payload, n);
  return dbt_runtime_receive(&p->runtime, frame, (uint16_t)(n + 3u));
}
static void fixed(void *ctx, const dbt_link_t *link, uint16_t cid, const uint8_t *data, uint16_t n) {
  peer_t *p = ctx;
  if (cid == 6 && dbt_smp_session_receive(&p->session, link, cid, data, n) != DBT_OK)
    ++p->errors;
}
static void packet(void *ctx, const dbt_hci_packet_t *view) {
  peer_t *p = ctx;
  if (view->type != DBT_H4_EVENT)
    return;
  if (view->event == 0x08 || (view->event == 0x3e && view->size && view->payload[0] == 5)) {
    dbt_result_t rc = dbt_smp_session_event(&p->session, view);
    if (rc != DBT_OK && rc != DBT_UNSUPPORTED)
      ++p->errors;
  }
}
static void link_change(void *ctx, const dbt_link_change_t *change) {
  peer_t *p = ctx;
  dbt_smp_session_link(&p->session, change);
}
static void completed(peer_t *p, uint16_t handle) {
  uint8_t payload[5] = {1};
  w16(payload + 1, handle);
  w16(payload + 3, 1);
  if (event(p, 0x13, payload, 5) != DBT_OK)
    ++p->errors;
}
static void status(peer_t *p, uint16_t opcode, uint8_t code) {
  uint8_t payload[4] = {code, 1, (uint8_t)opcode, (uint8_t)(opcode >> 8)};
  if (event(p, 0x0f, payload, 4) != DBT_OK)
    ++p->errors;
}
static void complete(peer_t *p, uint16_t opcode, uint8_t code, bool with_handle) {
  uint8_t payload[6] = {1, (uint8_t)opcode, (uint8_t)(opcode >> 8), code, 1, 0};
  if (event(p, 0x0e, payload, with_handle ? 6 : 4) != DBT_OK)
    ++p->errors;
}
static void encryption_event(peer_t *p) {
  uint8_t payload[4] = {0, 1, 0, 1};
  if (event(p, 0x08, payload, 4) != DBT_OK)
    ++p->errors;
}
static bool inject_identity(peer_t *p) {
  uint8_t a[26]={2,1,0x20,21,0,17,0,6,0,8};
  uint8_t b[17]={2,1,0x20,12,0,8,0,6,0,9,0};
  memcpy(a+10,identity_irk,16); memcpy(b+11,addr_a,6);
  return dbt_runtime_receive(&p->runtime,a,sizeof(a))==DBT_OK &&
         dbt_runtime_receive(&p->runtime,b,sizeof(b))==DBT_OK;
}
static void controller(peer_t *p, peer_t *other) {
  if (p->ready)
    return;
  if (memcmp(p->wire, p->borrowed, p->wire_size))
    ++p->errors;
  if (p->wire[0] == 2) {
    uint8_t frame[DBT_MAX_H4_BYTES];
    memcpy(frame, p->wire, p->wire_size);
    uint16_t handle = dbt_read_le16(frame + 1) & 0xfff;
    if ((frame[2] & 0x30) == 0)
      frame[2] |= 0x20;
    if (dbt_runtime_receive(&other->runtime, frame, p->wire_size) != DBT_OK)
      ++p->errors;
    ++p->tx_acl;
    p->ready = true;
    completed(p, handle);
    return;
  }
  if (p->wire[0] != 1) {
    ++p->errors;
    p->ready = true;
    return;
  }
  uint16_t op = dbt_read_le16(p->wire + 1);
  ++p->tx_commands;
  p->ready = true;
  if (op == 0x2019) {
    memcpy(central_key, p->wire + 16, 16);
    have_central_key = true;
    status(p, op, 0);
    uint8_t request[13] = {5, 1, 0};
    if (invalid_ltk_request)
      request[3] = 1;
    if (event(other, 0x3e, request, 13) != DBT_OK)
      ++p->errors;
    return;
  }
  if (op == 0x201a) {
    bool match = have_central_key && !memcmp(central_key, p->wire + 6, 16);
    complete(p, op, match ? 0 : 5, true);
    if (match) {
      encryption_event(p);
      if (burst_on_encrypt) { burst_injected=inject_identity(p); burst_on_encrypt=false; }
      encryption_event(other);
    }
    return;
  }
  if (op == 0x201b) {
    ++negative_replies;
    complete(p, op, 0, true);
    return;
  }
  if (op == 0x0c35)
    return;
  uint8_t result[68] = {1, (uint8_t)op, (uint8_t)(op >> 8), 0};
  uint8_t n = 4;
  switch (op) {
  case 0x1001:
    n = 12;
    result[4] = 8;
    result[7] = 8;
    break;
  case 0x1002:
    n = 68;
    break;
  case 0x1003:
    n = 12;
    result[8] = 0x40;
    break;
  case 0x1009:
    n = 10;
    result[4] = 0x42;
    break;
  case 0x1005:
    n = 11;
    result[4] = 27;
    result[7] = 2;
    break;
  case 0x2003:
    n = 12;
    break;
  case 0x2002:
    n = 7;
    result[4] = 27;
    result[6] = 2;
    break;
  default:
    break;
  }
  if (event(p, 0x0e, result, n) != DBT_OK)
    ++p->errors;
}
static int setup_peer(unsigned index) {
  peer_t *p = &peers[index];
  memset(p, 0, sizeof(*p));
  p->ready = true;
  dbt_transport_t transport = {p, can_send, send};
  dbt_runtime_hooks_t hooks = {0};
  hooks.context = p;
  hooks.allocate = allocate;
  hooks.release = release;
  hooks.packet = packet;
  hooks.link = link_change;
  hooks.channels.context = p;
  hooks.channels.fixed_data = fixed;
  if (dbt_runtime_init(&p->runtime, transport, hooks, 1024, 5000) != DBT_OK)
    return 1;
  if (dbt_runtime_start(&p->runtime, now_ms) != DBT_OK)
    return 2;
  for (unsigned i = 0; i < 40 && !p->runtime.ready; i++) {
    controller(p, p);
    dbt_runtime_poll(&p->runtime, ++now_ms, 8, 4);
  }
  return p->runtime.ready ? 0 : 3;
}
API int smp_fixture_open(int bonding, int fail_large, int distribution, int mode) {
  now_ms = 0;
  have_central_key = false;
  invalid_ltk_request = false;
  burst_on_encrypt=burst_injected=false;
  negative_replies = 0;
  memset(central_key, 0, sizeof(central_key));
  if (setup_peer(0) || setup_peer(1))
    return 1;
  for (unsigned i = 0; i < 2; i++) {
    peer_t *p = &peers[i];
    uint8_t le[19] = {1, 0, 1, 0};
    le[4] = (uint8_t)i;
    le[5] = (uint8_t)(i && mode==4);
    memcpy(le + 6, i ? (mode==4 ? addr_rpa : addr_a) : addr_b, 6);
    if (event(p, 0x3e, le, sizeof(le)) != DBT_OK)
      return 2;
    dbt_runtime_poll(&p->runtime, ++now_ms, 8, 0);
    const dbt_link_t *l = dbt_acl_find(&p->runtime.acl, 1);
    if (!l)
      return 3;
    if (dbt_smp_guard_init(&p->guard, 100, 1000, 10000, now_ms) != DBT_OK)
      return 4;
    dbt_smp_session_config_t cfg = {0};
    cfg.allow_just_works = true;
    cfg.features.io = mode==2 ? 1 : (mode==1 || mode==3) ? (i ? 0 : 2) : 3;
    cfg.features.min_key_size = 16;
    cfg.features.max_key_size = 16;
    cfg.features.support_sc = mode != 1;
    cfg.features.require_sc = mode != 1;
    cfg.features.require_mitm = mode != 0 && mode != 4;
    cfg.have_passkey = mode == 1 || mode == 3;
    cfg.passkey = 456789;
    cfg.features.bonding = bonding != 0;
    if (distribution==1) {
      cfg.features.initiator_keys = 4;
      cfg.features.responder_keys = 4;
      cfg.have_local_keys = true;
      memset(cfg.local_keys.csrk, (int)(0x30 + i), 16);
    }
    if (distribution==2) {
      cfg.features.initiator_keys=2; cfg.features.responder_keys=0;
      if (!i) {
        cfg.have_local_keys=true; cfg.local_keys.identity_type=0;
        memcpy(cfg.local_keys.identity,addr_a,6); memcpy(cfg.local_keys.irk,identity_irk,16);
      }
    }
    dbt_result_t rc = dbt_smp_session_open(&p->session, &p->runtime, 1, l->generation,
                                           mode==4 && !i ? 1 : 0, i ? addr_b : mode==4 ? addr_rpa : addr_a,
                                           mode==4 && i ? 1 : 0, i ? mode==4 ? addr_rpa : addr_a : addr_b,
                                           &p->guard, &cfg, now_ms);
    if (rc != DBT_OK)
      return 5;
    p->fail_large = fail_large != 0;
  }
  return dbt_smp_session_start(&peers[0].session, now_ms) == DBT_OK ? 0 : 6;
}
API void smp_fixture_step(void) {
  controller(&peers[0], &peers[1]);
  controller(&peers[1], &peers[0]);
  ++now_ms;
  for (unsigned i = 0; i < 2; i++)
    dbt_runtime_poll(&peers[i].runtime, now_ms, 8, 4);
  for (unsigned i = 0; i < 2; i++)
    dbt_smp_session_poll(&peers[i].session, now_ms);
}
API uint32_t smp_fixture_generation(unsigned i) { return i < 2 ? peers[i].session.generation : 0; }
API int smp_fixture_crypto(unsigned i, dbt_smp_sc_job_t *out) {
  if (i >= 2 || !out)
    return DBT_INVALID;
  dbt_smp_session_crypto_job_t j;
  dbt_result_t rc = dbt_smp_session_crypto_job(&peers[i].session, &j);
  if (rc == DBT_OK) {
    if (j.kind != DBT_SMP_SESSION_CRYPTO_SC)
      return DBT_UNSUPPORTED;
    *out = j.input.sc;
  }
  return rc;
}
API int smp_fixture_crypto_done(unsigned i, uint32_t gen, uint32_t serial, bool ok,
                                const dbt_smp_sc_result_t *r) {
  if (i >= 2)
    return DBT_INVALID;
  return dbt_smp_session_crypto_complete(&peers[i].session, gen, serial, ok, 0, r, now_ms);
}
API int smp_fixture_legacy_crypto(unsigned i, dbt_smp_job_t *out) {
  if (i >= 2 || !out) return DBT_INVALID;
  dbt_smp_session_crypto_job_t j;
  dbt_result_t rc=dbt_smp_session_crypto_job(&peers[i].session,&j);
  if (rc==DBT_OK) {
    if (j.kind!=DBT_SMP_SESSION_CRYPTO_LEGACY) return DBT_UNSUPPORTED;
    *out=j.input.legacy;
  }
  return rc;
}
API int smp_fixture_legacy_crypto_done(unsigned i,uint32_t gen,uint32_t serial,bool ok,
                                       const dbt_smp_job_result_t *r) {
  return i < 2 ? dbt_smp_session_crypto_complete(&peers[i].session,gen,serial,ok,r,0,now_ms) : DBT_INVALID;
}
API int smp_fixture_numeric_value(unsigned i,uint32_t *value) {
  return i < 2 ? dbt_smp_session_numeric_value(&peers[i].session,value) : DBT_INVALID;
}
API int smp_fixture_numeric_confirm(unsigned i,bool accepted) {
  return i < 2 ? dbt_smp_session_numeric_confirm(&peers[i].session,accepted,now_ms) : DBT_INVALID;
}
API int smp_fixture_store(unsigned i, dbt_smp_bond_job_t *out) {
  return i < 2 ? dbt_smp_session_store_job(&peers[i].session, out) : DBT_INVALID;
}
API int smp_fixture_store_done(unsigned i, uint32_t gen, uint32_t serial, bool committed) {
  return i < 2 ? dbt_smp_session_store_complete(&peers[i].session, gen, serial, committed, now_ms) : DBT_INVALID;
}
API int smp_fixture_security(unsigned i, dbt_smp_session_security_t *out) {
  return i < 2 ? dbt_smp_session_security(&peers[i].session, out) : DBT_INVALID;
}
API unsigned smp_fixture_phase(unsigned i) { return i < 2 ? (unsigned)peers[i].session.phase : 255; }
API unsigned smp_fixture_errors(unsigned i) { return i < 2 ? peers[i].errors : 1; }
API unsigned smp_fixture_acl_count(unsigned i) { return i < 2 ? peers[i].tx_acl : 0; }
API unsigned smp_fixture_bearer_size(unsigned i) { return i < 2 ? peers[i].session.bearer_size : 0; }
API unsigned smp_fixture_key_command_accepted(unsigned i) {
  return i < 2 && peers[i].session.command_accepted;
}
API unsigned smp_fixture_key_command_pending(unsigned i) {
  return i < 2 && peers[i].session.command_pending;
}
API unsigned smp_fixture_key_command_submitted(unsigned i) {
  return i < 2 && peers[i].session.command_pending && peers[i].runtime.host.pending_opcode==peers[i].session.command_opcode;
}
API unsigned smp_fixture_host_ok(unsigned i) { return i < 2 && peers[i].runtime.host.state==DBT_HOST_IDENTIFIED; }
API unsigned smp_fixture_uncertain(unsigned i) { return i < 2 && peers[i].session.uncertain_generation; }
API unsigned smp_fixture_identity_ready(unsigned i) {
  if (i>=2 || peers[i].session.phase!=DBT_SMP_SESSION_SC || !peers[i].session.bearer || peers[i].session.rx_count)
    return 0;
  dbt_smp_sc_bearer_t *b=peers[i].session.bearer;
  return b->engine.state==DBT_SC_DISTRIBUTION && b->engine.dist.phase==2 && b->engine.dist.step==0;
}
/* Two separate complete H4 ACL packets are submitted before one runtime poll;
 * this is a real ingress/L2CAP/fixed-CID burst in the software controller. */
API int smp_fixture_identity_burst(unsigned i) {
  if (!smp_fixture_identity_ready(i)) return DBT_INVALID;
  peer_t *p=&peers[i];
  if (!inject_identity(p)) return DBT_INVALID;
  dbt_runtime_poll(&p->runtime,++now_ms,8,0);
  return p->errors || p->runtime.host.state!=DBT_HOST_IDENTIFIED ? DBT_INVALID : p->session.rx_count;
}
API void smp_fixture_burst_on_encryption(void) { burst_on_encrypt=true; }
API unsigned smp_fixture_burst_injected(void) { return burst_injected; }
API int smp_fixture_identity_overflow(unsigned i) {
  if (!smp_fixture_identity_ready(i)) return DBT_INVALID;
  peer_t *p=&peers[i]; uint8_t a[26]={2,1,0x20,21,0,17,0,6,0,8};
  memcpy(a+10,identity_irk,16);
  for(unsigned n=0;n<DBT_SMP_SESSION_RX_DEPTH+1;n++)
    if(dbt_runtime_receive(&p->runtime,a,sizeof(a))!=DBT_OK) return DBT_INVALID;
  dbt_runtime_poll(&p->runtime,++now_ms,8,0);
  if(p->session.rx_count!=DBT_SMP_SESSION_RX_DEPTH || p->errors) return DBT_INVALID;
  dbt_runtime_poll(&p->runtime,++now_ms,8,0);
  return p->session.phase==DBT_SMP_SESSION_FAILED && !p->session.rx_count && p->errors==1 ? DBT_OK : DBT_INVALID;
}
API int smp_fixture_rx_oom(void) {
  now_ms=0; if(setup_peer(0)) return DBT_INVALID;
  peer_t *p=&peers[0]; uint8_t le[19]={1,0,1,0}; le[4]=1; memcpy(le+6,addr_a,6);
  if(event(p,0x3e,le,sizeof(le))!=DBT_OK) return DBT_INVALID;
  dbt_runtime_poll(&p->runtime,++now_ms,8,0);
  const dbt_link_t *l=dbt_acl_find(&p->runtime.acl,1); if(!l) return DBT_INVALID;
  if(dbt_smp_guard_init(&p->guard,100,1000,10000,now_ms)!=DBT_OK) return DBT_INVALID;
  dbt_smp_session_config_t cfg={0}; cfg.allow_just_works=true; cfg.features.io=3;
  cfg.features.min_key_size=7; cfg.features.max_key_size=16; cfg.features.support_sc=true;
  p->fail_rx=true;
  dbt_result_t rc=dbt_smp_session_open(&p->session,&p->runtime,1,l->generation,0,addr_b,0,addr_a,&p->guard,&cfg,now_ms);
  dbt_runtime_abort(&p->runtime,DBT_FAULT_BAD_RESPONSE);
  return rc==DBT_NO_SPACE && !p->session.runtime && p->allocated==p->released ? DBT_OK : DBT_INVALID;
}
API void smp_fixture_poll_only(unsigned i) {
  if(i<2) dbt_smp_session_poll(&peers[i].session,++now_ms);
}
API void smp_fixture_runtime_only(unsigned i) {
  if(i<2) dbt_runtime_poll(&peers[i].runtime,++now_ms,8,0);
}
API unsigned smp_fixture_rx_zero(unsigned i) {
  if(i>=2 || !peers[i].session.rx_queue) return 0;
  const uint8_t *p=(const uint8_t *)peers[i].session.rx_queue;
  for(unsigned n=0;n<DBT_SMP_SESSION_RX_DEPTH*sizeof(dbt_smp_session_rx_t);n++) if(p[n]) return 0;
  return 1;
}
API int smp_fixture_disconnect(unsigned i) {
  if (i>=2) return DBT_INVALID;
  uint8_t data[4]={0,1,0,0x13}; return event(&peers[i],0x05,data,sizeof(data));
}
API void smp_fixture_invalid_ltk(void) { invalid_ltk_request = true; }
API unsigned smp_fixture_negative_replies(void) { return negative_replies; }
API void smp_fixture_jump(uint32_t delta) { now_ms += delta; }
API int smp_fixture_inject_encryption(unsigned i) {
  if (i >= 2)
    return DBT_INVALID;
  encryption_event(&peers[i]);
  return DBT_OK;
}
API int smp_fixture_inject_ltk(unsigned i) {
  if (i >= 2) return DBT_INVALID;
  uint8_t request[13]={5,1,0};
  return event(&peers[i],0x3e,request,sizeof(request));
}
API int smp_fixture_final_token(unsigned i, uint32_t *serial) {
  return i < 2 ? dbt_smp_session_distribution_token(&peers[i].session, serial) : DBT_INVALID;
}
API int smp_fixture_final_ack(unsigned i,uint32_t generation,uint32_t serial) {
  return i < 2 ? dbt_smp_session_final_ack(&peers[i].session,generation,serial,true,now_ms) : DBT_INVALID;
}
API int smp_fixture_try_close(unsigned i) {
  return i < 2 ? dbt_smp_session_close(&peers[i].session) : DBT_INVALID;
}
API int smp_fixture_close(void) {
  int error = 0;
  for (unsigned i = 0; i < 2; i++) {
    peer_t *p = &peers[i];
    dbt_runtime_abort(&p->runtime, DBT_FAULT_BAD_RESPONSE);
    if (dbt_smp_session_close(&p->session) != DBT_OK)
      error = 1;
    if (p->allocated != p->released)
      error = 2;
  }
  return error;
}
