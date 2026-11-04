#include "dbt_ble_control.h"
#include <assert.h>
#include <string.h>
typedef struct {
  unsigned commits, publishes, commands;
  bool fail;
  int8_t eq[3];
} state_t;
static bool commit(void *p, int8_t b, int8_t m, int8_t t) {
  state_t *s = p;
  s->commits++;
  if (s->fail)
    return false;
  s->eq[0] = b;
  s->eq[1] = m;
  s->eq[2] = t;
  return true;
}
static bool publish(void *p, int8_t b, int8_t m, int8_t t) {
  state_t *s = p;
  assert(b == s->eq[0] && m == s->eq[1] && t == s->eq[2]);
  s->publishes++;
  return true;
}
static bool control(void *p, uint8_t x) {
  (void)p;
  (void)x;
  return true;
}
static bool provider(void *p, const dbt_ble_control_peer_t *peer, uint8_t cmd,
                     const uint8_t *data, uint16_t n, uint8_t *out,
                     uint8_t *size) {
  state_t *s = p;
  assert(peer && peer->generation == 7);
  (void)data;
  (void)n;
  s->commands++;
  assert(*size >= 2);
  out[0] = 0x10;
  out[1] = cmd;
  *size = 2;
  return true;
}
int main(void) {
  state_t x = {0};
  dbt_ble_control_state_t s = {0};
  s.valid = true;
  dbt_ble_control_hooks_t h = {0};
  h.context = &x;
  h.commit_eq = commit;
  h.publish_eq = publish;
  h.commit_control = control;
  h.publish_control = control;
  h.command = provider;
  dbt_ble_control_peer_t peer = {1, 7};
  dbt_ble_control_result_t out;
  uint8_t eq[] = {1, 0xf4, 0, 12};
  assert(dbt_ble_control_execute(&s, &h, &peer, eq, sizeof(eq), &out) ==
             DBT_OK &&
         out.durable_eq && out.data[0] == 0x10);
  assert(x.eq[0] == -12 && x.eq[1] == 0 && x.eq[2] == 12 && x.commits == 1 &&
         x.publishes == 1);
  assert(dbt_ble_control_execute(&s, &h, &peer, eq, sizeof(eq), &out) ==
             DBT_OK &&
         x.commits == 2 && x.publishes == 2);
  for (unsigned band = 0; band < 3; band++)
    for (int value = -128; value <= 127; value++) {
      if (value >= -12 && value <= 12)
        continue;
      uint8_t invalid[] = {1, 0, 0, 0};
      invalid[band + 1] = (uint8_t)value;
      assert(dbt_ble_control_execute(&s, &h, &peer, invalid, sizeof(invalid),
                                     &out) == DBT_OK &&
             out.data[0] == 0x11 && out.data[2] == 2 && !out.durable_eq &&
             x.commits == 2 && x.publishes == 2);
      assert(s.eq[0] == -12 && s.eq[1] == 0 && s.eq[2] == 12);
    }
  x.fail = true;
  assert(dbt_ble_control_execute(&s, &h, &peer, eq, sizeof(eq), &out) ==
             DBT_OK &&
         out.data[0] == 0x11 && !out.durable_eq);
  assert(x.commits == 3 && x.publishes == 2);
  x.fail = false;
  uint8_t preset[] = {2, 11};
  assert(dbt_ble_control_execute(&s, &h, &peer, preset, 2, &out) == DBT_OK &&
         x.eq[0] == 4 && x.eq[1] == 2 && x.eq[2] == 4);
  preset[1] = 12;
  assert(dbt_ble_control_execute(&s, &h, &peer, preset, 2, &out) == DBT_OK &&
         out.data[0] == 0x11 && out.data[2] == 2);
  uint8_t c[] = {3, 0xff};
  assert(dbt_ble_control_execute(&s, &h, &peer, c, 2, &out) == DBT_OK &&
         s.control == 7 && out.durable_control);
  const struct {
    uint8_t op, len;
  } cases[] = {{4, 1},    {5, 10},   {6, 1},    {7, 1},    {0x10, 1},
               {0x11, 1}, {0x12, 4}, {0x13, 2}, {0x14, 0}, {0x20, 4},
               {0x21, 2}, {0x22, 0}, {0x23, 0}};
  for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
    uint8_t p[16] = {0};
    p[0] = cases[i].op;
    if(p[0]==4)p[1]='a';
    if (cases[i].len) {
      assert(dbt_ble_control_execute(&s, &h, &peer, p, cases[i].len, &out) ==
                 DBT_OK &&
             out.data[0] == 0x11 && out.data[2] == 2);
    }
    unsigned before = x.commands;
    assert(dbt_ble_control_execute(
               &s, &h, &peer, p, (uint16_t)(cases[i].len + 1), &out) == DBT_OK);
    assert(x.commands == before + 1 && out.size == 2 && out.data[0] == 0x10 &&
           out.data[1] == p[0]);
  }
  /* Invalid names must not reach a persistence-capable provider. */
  const uint8_t names[][6]={{4,0},{4,0xc0,0x80},{4,0xed,0xa0,0x80},{4,0xf4,0x90,0x80,0x80},{4,0xe2,0x82}};
  const unsigned name_sizes[]={2,3,4,5,3};
  unsigned named=x.commands;
  for(unsigned i=0;i<sizeof(name_sizes)/sizeof(name_sizes[0]);i++){
    assert(dbt_ble_control_execute(&s,&h,&peer,names[i],(uint16_t)name_sizes[i],&out)==DBT_OK);
    assert(out.data[0]==0x11 && out.data[2]==2 && x.commands==named);
  }
  const uint8_t valid_name[]={4,'A',0xe2,0x82,0xac,0xf0,0x9f,0x98,0x80};
  assert(dbt_ble_control_execute(&s,&h,&peer,valid_name,sizeof(valid_name),&out)==DBT_OK && out.data[0]==0x10 && x.commands==named+1);
  h.command = NULL;
  uint8_t ota[] = {0x20, 1, 0, 0, 0};
  assert(dbt_ble_control_execute(&s, &h, &peer, ota, sizeof(ota), &out) ==
             DBT_OK &&
         out.data[0] == 0x11 && out.data[2] == 3);
  uint8_t ping = 0xff;
  assert(dbt_ble_control_execute(&s, &h, &peer, &ping, 1, &out) == DBT_OK &&
         out.size == 1 && out.data[0] == 0xff);
  uint8_t bad = 0x77;
  assert(dbt_ble_control_execute(&s, &h, &peer, &bad, 1, &out) == DBT_OK &&
         out.data[0] == 0x11 && out.data[2] == 1);
  s.name_size = 32;
  s.firmware_size = 32;
  memset(s.name, 'n', 32);
  memset(s.firmware, 'f', 32);
  uint8_t full[DBT_BLE_CONTROL_MAX_STATUS], len;
  assert(dbt_ble_control_status(&s, full, &len) == DBT_OK && len == 82);
  s.valid = false;
  assert(dbt_ble_control_status(&s, full, &len) == DBT_INVALID);
  return 0;
}
