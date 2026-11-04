#include "dbt_input.h"
#include <stddef.h>
#ifdef _WIN32
#define EX __declspec(dllexport)
void *memcpy(void *d, const void *s, size_t n) {
  unsigned char *a = d;
  const unsigned char *b = s;
  for (size_t i = 0; i < n; i++)
    a[i] = b[i];
  return d;
}
void *memset(void *d, int v, size_t n) {
  unsigned char *a = d;
  for (size_t i = 0; i < n; i++)
    a[i] = (unsigned char)v;
  return d;
}
#else
#define EX
#endif
#define CHECK(x)                                                               \
  do {                                                                         \
    if (!(x))                                                                  \
      return __LINE__;                                                         \
  } while (0)
EX int test_input(void) {
  dbt_input_t in;
  dbt_input_init(&in);
  dbt_input_sample_t s = {0};
  dbt_input_action_t a;
  CHECK(dbt_input_feed(&in, &s));
  s.position[1] = 12;
  s.position[2] = -4;
  s.position[0] = 4;
  s.ms = 25;
  CHECK(dbt_input_feed(&in, &s));
  CHECK(dbt_input_next(&in, &a) && a.kind == DBT_INPUT_VOLUME && a.delta == 3);
  for (int j = 0; j < 3; j++)
    CHECK(dbt_input_next(&in, &a) && a.kind == DBT_INPUT_EQ_BASS &&
          a.delta == 1);
  CHECK(dbt_input_next(&in, &a) && a.kind == DBT_INPUT_EQ_MID && a.delta == -1);
  CHECK(!dbt_input_next(&in, &a));
  s.enc_buttons = 2;
  for (int j = 0; j < 3; j++) {
    s.ms += 25;
    CHECK(dbt_input_feed(&in, &s));
  }
  s.enc_buttons = 0;
  s.position[1] += 4;
  CHECK(dbt_input_feed(&in, &s));
  CHECK(dbt_input_next(&in, &a) && a.kind == DBT_INPUT_BRIGHTNESS &&
        a.delta == 5);
  s.enc_buttons = 4;
  for (int j = 0; j < 3; j++) {
    s.ms += 25;
    CHECK(dbt_input_feed(&in, &s));
  }
  CHECK(dbt_input_next(&in, &a) && a.kind == DBT_INPUT_PAIR);
  s.enc_buttons = 1;
  for (int j = 0; j < 3; j++) {
    s.ms += 25;
    CHECK(dbt_input_feed(&in, &s));
  }
  s.enc_buttons = 0;
  s.ms += 301;
  CHECK(dbt_input_feed(&in, &s));
  CHECK(dbt_input_next(&in, &a) && a.kind == DBT_INPUT_PLAY_PAUSE);
  s.position[2] = 10000;
  CHECK(!dbt_input_feed(&in, &s));
  CHECK(in.state.last[2] == -4);
  s.position[2] = -4;
  /* Backpressure cannot consume sample or collapse individual EQ writes. */
  in.state.brightness_mode = 0;
  for (int z = 0; z < 8; z++) {
    s.position[1] += 16;
    CHECK(dbt_input_feed(&in, &s));
  }
  s.position[1] += 4;
  int old = in.state.last[1];
  CHECK(!dbt_input_feed(&in, &s));
  CHECK(in.state.last[1] == old);
  CHECK(dbt_input_next(&in, &a));
  CHECK(dbt_input_feed(&in, &s));
  unsigned count = 1;
  while (dbt_input_next(&in, &a))
    count++;
  CHECK(count == 33);
  dbt_input_init(&in);
  s = (dbt_input_sample_t){0};
  s.position[2] = INT32_MAX - 1;
  s.gpio_buttons = 1;
  s.ms = 6000;
  CHECK(dbt_input_feed(&in, &s));
  s.position[2] = INT32_MIN + 2;
  s.gpio_buttons = 0;
  s.ms += 1;
  CHECK(dbt_input_feed(&in, &s));
  CHECK(dbt_input_next(&in, &a) && a.kind == DBT_INPUT_EQ_MID && a.delta == 1);
  s.ms += 26;
  CHECK(dbt_input_feed(&in, &s));
  CHECK(!dbt_input_next(&in, &a)); /* held at startup: no phantom release */
  return 0;
}

EX int test_large_inputs(void) {
  dbt_input_t in;
  dbt_input_sample_t s = {0};
  dbt_input_action_t a;
  dbt_input_init(&in);
  CHECK(dbt_input_feed_ex(&in, &s, true) == DBT_INPUT_FEED_ACCEPTED);
  s.position[1] = 100;
  s.enc_buttons = 4;
  s.ms = 25;
  CHECK(dbt_input_feed_ex(&in, &s, true) == DBT_INPUT_FEED_PARTIAL);
  for (unsigned j = 0; j < 3; j++)
    CHECK(dbt_input_feed_ex(&in, &s, false) == DBT_INPUT_FEED_PARTIAL);
  CHECK(dbt_input_feed_ex(&in, &s, false) == DBT_INPUT_FEED_ACCEPTED);
  for (unsigned j = 0; j < 25; j++)
    CHECK(dbt_input_next(&in, &a) && a.kind == DBT_INPUT_EQ_BASS &&
          a.delta == 1);
  CHECK(!dbt_input_next(&in, &a));
  s.ms += 25;
  CHECK(dbt_input_feed_ex(&in, &s, true) == DBT_INPUT_FEED_ACCEPTED);
  s.ms += 25;
  CHECK(dbt_input_feed_ex(&in, &s, true) == DBT_INPUT_FEED_ACCEPTED);
  CHECK(dbt_input_next(&in, &a) && a.kind == DBT_INPUT_PAIR);
  CHECK(!dbt_input_next(&in, &a));
  s.enc_buttons = 0;
  s.position[1] += 5000;
  s.position[2] = 4;
  CHECK(dbt_input_feed_ex(&in, &s, true) == DBT_INPUT_FEED_INVALID);
  CHECK(dbt_input_rebase_invalid(&in, &s) == 1);
  CHECK(dbt_input_feed_ex(&in, &s, true) == DBT_INPUT_FEED_ACCEPTED);
  CHECK(dbt_input_next(&in, &a) && a.kind == DBT_INPUT_EQ_MID && a.delta == 1);
  /* Full queue retains state; draining space enables exact partial progress. */
  for (unsigned z = 0; z < 8; z++) {
    s.position[2] += 16;
    CHECK(dbt_input_feed_ex(&in, &s, true) == DBT_INPUT_FEED_ACCEPTED);
  }
  s.position[2] += 80;
  int32_t previous = in.state.last[2];
  CHECK(dbt_input_feed_ex(&in, &s, true) == DBT_INPUT_FEED_BUSY);
  CHECK(in.state.last[2] == previous);
  for (unsigned j = 0; j < 5; j++)
    CHECK(dbt_input_next(&in, &a));
  CHECK(dbt_input_feed_ex(&in, &s, true) == DBT_INPUT_FEED_PARTIAL);
  CHECK(in.state.last[2] == previous + 20);
  unsigned n = 0;
  while (dbt_input_next(&in, &a))
    n++;
  CHECK(n == 32);
  CHECK(dbt_input_feed_ex(&in, &s, false) == DBT_INPUT_FEED_PARTIAL);
  CHECK(dbt_input_feed_ex(&in, &s, false) == DBT_INPUT_FEED_PARTIAL);
  CHECK(dbt_input_feed_ex(&in, &s, false) == DBT_INPUT_FEED_ACCEPTED);
  n = 0;
  while (dbt_input_next(&in, &a)) {
    CHECK(a.kind == DBT_INPUT_EQ_MID && a.delta == 1);
    n++;
  }
  CHECK(n == 15);
  return 0;
}
