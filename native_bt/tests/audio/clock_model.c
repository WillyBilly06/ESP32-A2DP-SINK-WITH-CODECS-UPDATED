#include "dbt_audio.h"
#include <stdlib.h>
#include <string.h>
#ifdef _WIN32
#define EX __declspec(dllexport)
#else
#define EX __attribute__((visibility("default")))
#endif
#define CHECK(x)                                                               \
  do {                                                                         \
    if (!(x))                                                                  \
      return __LINE__;                                                         \
  } while (0)
static dbt_audio_t a;
static unsigned live;
static uint8_t *alloc(void *unused, uint32_t n) {
  (void)unused;
  uint8_t *p = malloc(n);
  if (p)
    live++;
  return p;
}
static void release(void *unused, uint8_t *p) {
  (void)unused;
  if (p) {
    live--;
    free(p);
  }
}
static bool fixture(uint32_t rate) {
  dbt_audio_config_t c = {rate,           8192, 16384, (rate * 64u) / 1000u,
                          DBT_PCM_S32_LE, 2,    true};
  return dbt_audio_init(&a, (dbt_audio_memory_t){0, alloc, release}) ==
             DBT_OK &&
         dbt_audio_open(&a, c) == DBT_OK;
}
static void queues(uint32_t q) {
  uint32_t out = q > a.output.frames ? a.output.frames : q;
  uint32_t in = q - out;
  atomic_store(&a.input.read, 0);
  atomic_store(&a.input.written, in);
  atomic_store(&a.output.read, 0);
  atomic_store(&a.output.written, out);
}
static int model(uint32_t rate, int32_t sink_offset) {
  CHECK(fixture(rate));
  uint32_t time = UINT32_MAX - 1499u, submitted = UINT32_MAX - 100000u,
           taken = UINT32_MAX - 100000u, rendered = UINT32_MAX - 100000u;
  atomic_store(&a.submitted_frames, submitted);
  atomic_store(&a.taken_frames, taken);
  atomic_store(&a.rendered_frames, rendered);
  dbt_audio_clock_reset(&a, time);
  int64_t q = (int64_t)((rate * 64u) / 1000u) << 32;
  uint32_t sink_half = 0, source_half = 0, prior_target = 0;
  int32_t prior_ppm = 0, min_tail = 10000, max_tail = -10000;
  for (unsigned i = 0; i < 1200; i++) {
    source_half += rate;
    uint32_t source = source_half / 2u;
    source_half %= 2u;
    sink_half += (uint32_t)((int32_t)rate + sink_offset);
    uint32_t consume = sink_half / 2u;
    sink_half %= 2u;
    q += (int64_t)source * 4294967296ll -
         (int64_t)consume * (4294967296ll + a.step_adjust);
    int32_t jitter = (int32_t)(rate / 1000u) * (int32_t)((int)(i % 7u) - 3);
    int64_t observed = (q >> 32) + jitter;
    CHECK(observed > 1 &&
          observed < (int64_t)(a.input.frames + a.output.frames));
    queues((uint32_t)observed);
    submitted += source;
    taken += consume;
    rendered += consume;
    atomic_store(&a.submitted_frames, submitted);
    atomic_store(&a.taken_frames, taken);
    atomic_store(&a.rendered_frames, rendered);
    time += 500;
    dbt_audio_clock_update(&a, time);
    dbt_audio_clock_stats_t st;
    CHECK(dbt_audio_clock_snapshot(&a, &st));
    CHECK(st.ppm <= 500 && st.ppm >= -500 && st.ppm - prior_ppm <= 25 &&
          prior_ppm - st.ppm <= 25);
    prior_ppm = st.ppm;
    if (i > 1100) {
      if (st.ppm < min_tail)
        min_tail = st.ppm;
      if (st.ppm > max_tail)
        max_tail = st.ppm;
    }
    if (st.state == DBT_AUDIO_CLOCK_TRACKING)
      prior_target = st.target_frames;
  }
  int32_t want = (int32_t)(-(int64_t)sink_offset * 1000000 /
                           (int64_t)(rate + sink_offset));
  CHECK(a.clock_state == DBT_AUDIO_CLOCK_TRACKING && prior_target > 0 &&
        a.clock_observed_valid);
  CHECK(a.clock_ppm - want < 50 && want - a.clock_ppm < 50 &&
        max_tail - min_tail < 100);
  int64_t err = (q >> 32) - (int64_t)prior_target;
  if (err < 0)
    err = -err;
  CHECK(err < (int64_t)rate / 1000);
  int32_t quantization = (int32_t)(1000000u / (rate * 8u)) + 4;
  CHECK(a.clock_observed_ppm - want < quantization &&
        want - a.clock_observed_ppm < quantization);
  int32_t integral = a.clock_integral_q16;
  /* A stalled consumer/backpressured producer is not learned as drift. */
  for (unsigned i = 0; i < 12; i++) {
    submitted += rate / 2u;
    atomic_store(&a.submitted_frames, submitted);
    atomic_fetch_add(&a.producer_backpressure, 1);
    time += 500;
    dbt_audio_clock_update(&a, time);
    CHECK(a.clock_state == DBT_AUDIO_CLOCK_RECOVERING &&
          a.clock_integral_q16 == integral && !a.clock_observed_valid);
  }
  for (unsigned i = 0; i < 4; i++) {
    submitted += rate / 2u;
    taken += rate / 2u;
    rendered += rate / 2u;
    atomic_store(&a.submitted_frames, submitted);
    atomic_store(&a.taken_frames, taken);
    atomic_store(&a.rendered_frames, rendered);
    time += 500;
    dbt_audio_clock_update(&a, time);
  }
  CHECK(a.clock_state == DBT_AUDIO_CLOCK_TRACKING &&
        a.clock_target == prior_target);
  atomic_fetch_add(&a.zero_frames, 1);
  time += 500;
  dbt_audio_clock_update(&a, time);
  CHECK(a.clock_state == DBT_AUDIO_CLOCK_RECOVERING && !a.clock_observed_valid);
  dbt_audio_clock_reset(&a, time);
  CHECK(a.clock_state == DBT_AUDIO_CLOCK_CALIBRATING && !a.clock_ppm &&
        !a.clock_integral_q16);
  dbt_audio_close(&a);
  CHECK(!live);
  return 0;
}
EX int clock_model_rates(void) {
  static const uint32_t rates[] = {8000,  11025,  12000, 16000, 22050,
                                   24000, 32000,  44100, 48000, 88200,
                                   96000, 176400, 192000};
  for (unsigned i = 0; i < sizeof(rates) / sizeof(rates[0]); i++) {
    int32_t offset = (int32_t)(rates[i] / 10000u);
    if (!offset)
      offset = 1;
    int r = model(rates[i], offset);
    if (r)
      return r;
    r = model(rates[i], -offset);
    if (r)
      return r;
  }
  return model(96000, 10);
}
EX int clock_fractional_waveform(void) {
  static int32_t data[8192 * 2], output[8000];
  fixed_dsp_t expected;
  for (unsigned sign = 0; sign < 2; sign++) {
    CHECK(fixture(96000));
    CHECK(dbt_audio_controls(&a, 0, 0, 0, 127, FDSP_BYPASS) == DBT_OK);
    (void)dbt_audio_render(&a, 0);
    fixed_dsp_controls(&a.dsp, 0, 0, 0, 127, FDSP_BYPASS, true);
    CHECK(fixed_dsp_init(&expected, 96000, 90, 500, 512));
    fixed_dsp_controls(&expected, 0, 0, 0, 127, FDSP_BYPASS, true);
    for (unsigned i = 0; i < 8192; i++) {
      data[2 * i] = (int32_t)(i * 65536u);
      data[2 * i + 1] = (int32_t)(i * 32768u);
    }
    CHECK(dbt_audio_submit(&a, data, 8192) == DBT_OK);
    a.step_adjust = sign ? -2147483 : 2147483;
    CHECK(dbt_audio_render(&a, 4000) == 4000 &&
          dbt_audio_take(&a, output, 4000) == 4000);
    uint64_t phase = 0;
    uint64_t step = (uint64_t)(4294967296ll + a.step_adjust);
    for (unsigned i = 0; i < 4000; i++) {
      uint64_t delay = (uint64_t)dbt_resampler_delay_frames(96000) << 32;
      uint64_t position = phase > delay ? phase - delay : 0;
      int32_t left = (int32_t)((position * 65536ull) >> 32),
              right = (int32_t)((position * 32768ull) >> 32);
      fixed_dsp_process(&expected, &left, &right);
      if (i >= 64) {
        int64_t dl = (int64_t)output[2 * i] - left,
                dr = (int64_t)output[2 * i + 1] - right;
        CHECK(dl >= -512 && dl <= 512 && dr >= -512 && dr <= 512);
        CHECK(output[2 * i] > output[2 * i - 2]);
      }
      phase += step;
    }
    dbt_audio_close(&a);
    CHECK(!live);
  }
  return 0;
}
static int pipeline(uint32_t rate, int32_t offset) {
  static int32_t silence[8192 * 2], out[4096 * 2];
  CHECK(fixture(rate));
  uint32_t initial = (rate * 64u) / 1000u;
  while (initial) {
    uint32_t n = initial > 4096 ? 4096 : initial;
    CHECK(dbt_audio_submit(&a, silence, n) == DBT_OK);
    CHECK(dbt_audio_render(&a, n) == n);
    initial -= n;
  }
  dbt_audio_clock_reset(&a, 0);
  uint32_t now = 0, sink_fraction = 0;
  for (unsigned tick = 0; tick < 1000; tick++) {
    uint32_t queued = dbt_audio_output_frames(&a),
             target = a.config.target_frames;
    if (queued < target)
      (void)dbt_audio_render(&a, target - queued);
    if (!(tick & 1u)) {
      uint32_t n = rate / 25u;
      CHECK(n <= 8192 && dbt_audio_submit(&a, silence, n) == DBT_OK);
    }
    queued = dbt_audio_output_frames(&a);
    if (queued < target)
      (void)dbt_audio_render(&a, target - queued);
    sink_fraction += (uint32_t)((int32_t)rate + offset);
    uint32_t wanted = sink_fraction / 50u;
    sink_fraction %= 50u;
    CHECK(wanted <= 4096 && dbt_audio_take(&a, out, wanted) == wanted);
    now += 20;
    if (now % 500u == 0)
      dbt_audio_clock_update(&a, now);
  }
  CHECK(a.clock_state == DBT_AUDIO_CLOCK_TRACKING &&
        !atomic_load(&a.zero_frames) && !atomic_load(&a.producer_backpressure));
  if (rate >= 44100)
    CHECK(offset > 0 ? a.clock_ppm < 0 : a.clock_ppm > 0);
  dbt_audio_close(&a);
  CHECK(!live);
  return 0;
}
EX int clock_real_pipeline(void) {
  static const uint32_t rates[] = {8000,  11025,  12000, 16000, 22050,
                                   24000, 32000,  44100, 48000, 88200,
                                   96000, 176400, 192000};
  for (unsigned i = 0; i < sizeof(rates) / sizeof(rates[0]); i++) {
    int32_t offset = (int32_t)(rates[i] / 10000u);
    if (!offset)
      offset = 1;
    int r = pipeline(rates[i], offset);
    if (r)
      return r;
    r = pipeline(rates[i], -offset);
    if (r)
      return r;
  }
  return pipeline(96000, 10);
}
EX int clock_anti_windup(void) {
  CHECK(fixture(48000));
  uint32_t now = 0, source = 0, taken = 0;
  queues(3000);
  dbt_audio_clock_reset(&a, now);
  for (unsigned i = 0; i < 8; i++) {
    source += 24000;
    taken += 24000;
    atomic_store(&a.submitted_frames, source);
    atomic_store(&a.rendered_frames, taken);
    atomic_store(&a.taken_frames, taken);
    now += 500;
    dbt_audio_clock_update(&a, now);
  }
  CHECK(a.clock_state == DBT_AUDIO_CLOCK_TRACKING && a.clock_target == 3000);
  queues(11000);
  for (unsigned i = 0; i < 200; i++) {
    source += 24000;
    taken += 24000;
    atomic_store(&a.submitted_frames, source);
    atomic_store(&a.rendered_frames, taken);
    atomic_store(&a.taken_frames, taken);
    now += 500;
    dbt_audio_clock_update(&a, now);
  }
  CHECK(a.clock_ppm == 500 && a.clock_integral_q16 < 30 * 65536 &&
        a.clock_integral_q16 > -30 * 65536);
  queues(2000);
  for (unsigned i = 0; i < 50; i++) {
    source += 24000;
    taken += 24000;
    atomic_store(&a.submitted_frames, source);
    atomic_store(&a.rendered_frames, taken);
    atomic_store(&a.taken_frames, taken);
    now += 500;
    dbt_audio_clock_update(&a, now);
  }
  CHECK(a.clock_ppm < 0);
  dbt_audio_close(&a);
  CHECK(!live);
  return 0;
}
EX int clock_backpressure_relief(void) {
  CHECK(fixture(48000));
  uint32_t now = 0, source = 0, taken = 0;
  queues(3000);
  dbt_audio_clock_reset(&a, now);
  for (unsigned i = 0; i < 8; i++) {
    source += 24000;
    taken += 24000;
    atomic_store(&a.submitted_frames, source);
    atomic_store(&a.rendered_frames, taken);
    atomic_store(&a.taken_frames, taken);
    now += 500;
    dbt_audio_clock_update(&a, now);
  }
  CHECK(a.clock_state == DBT_AUDIO_CLOCK_TRACKING);
  queues(6000);
  int32_t integral = a.clock_integral_q16;
  for (unsigned i = 0; i < 24; i++) {
    taken += 24000;
    atomic_store(&a.rendered_frames, taken);
    atomic_store(&a.taken_frames, taken);
    atomic_fetch_add(&a.producer_backpressure, 1);
    now += 500;
    dbt_audio_clock_update(&a, now);
    CHECK(a.clock_state == DBT_AUDIO_CLOCK_RECOVERING &&
          a.clock_integral_q16 == integral && !a.clock_observed_valid);
  }
  CHECK(a.clock_ppm == 500);
  dbt_audio_close(&a);
  CHECK(!live);
  return 0;
}
EX int clock_switch_reset(void) {
  CHECK(fixture(48000));
  CHECK(dbt_audio_controls(&a, 5, -4, 3, 42, FDSP_CHANNEL_FLIP) == DBT_OK);
  CHECK(dbt_audio_volume(&a, 57) == DBT_OK);
  uint32_t gains = atomic_load(&a.controls_gains),
           options = atomic_load(&a.controls_options);
  queues(3000);
  dbt_audio_clock_reset(&a, 100);
  uint32_t count = 0;
  for (unsigned i = 0; i < 30; i++) {
    count += 24000;
    atomic_store(&a.submitted_frames, count);
    atomic_store(&a.rendered_frames, count);
    atomic_store(&a.taken_frames, count);
    dbt_audio_clock_update(&a, 600 + i * 500);
  }
  CHECK(a.clock_state == DBT_AUDIO_CLOCK_TRACKING);
  dbt_audio_config_t next = {192000,         8192, 16384, 12288,
                             DBT_PCM_S32_LE, 2,    true};
  CHECK(dbt_audio_open(&a, next) == DBT_OK && a.config.rate == 192000 &&
        !a.clock_integral_q16 && !a.step_adjust && !a.clock_target);
  CHECK(atomic_load(&a.controls_gains) == gains &&
        atomic_load(&a.controls_options) == options);
  dbt_audio_clock_stats_t stats;
  CHECK(dbt_audio_clock_snapshot(&a, &stats) &&
        stats.state == DBT_AUDIO_CLOCK_CALIBRATING && !stats.observed_valid);
  dbt_audio_close(&a);
  CHECK(dbt_audio_clock_snapshot(&a, &stats) &&
        stats.state == DBT_AUDIO_CLOCK_OFF && !live);
  return 0;
}
EX int clock_interval_jitter(void) {
  CHECK(fixture(96000));
  uint32_t now = 0, submitted = 0, taken = 0, source_rem = 0, sink_rem = 0;
  int64_t q = (int64_t)6144 << 32;
  queues(6144);
  dbt_audio_clock_reset(&a, 0);
  static const uint32_t intervals[] = {200, 750, 350, 700};
  int32_t previous = 0;
  for (unsigned i = 0; i < 1400; i++) {
    uint32_t dt = intervals[i % 4], source_total = 96000u * dt + source_rem,
             sink_total = 96010u * dt + sink_rem;
    uint32_t source = source_total / 1000u, consume = sink_total / 1000u;
    source_rem = source_total % 1000u;
    sink_rem = sink_total % 1000u;
    q += (int64_t)source * 4294967296ll -
         (int64_t)consume * (4294967296ll + a.step_adjust);
    int32_t jitter = 96 * (int32_t)((int)(i % 7) - 3);
    int64_t observed = (q >> 32) + jitter;
    CHECK(observed > 0 &&
          observed < (int64_t)(a.input.frames + a.output.frames));
    queues((uint32_t)observed);
    submitted += source;
    taken += consume;
    atomic_store(&a.submitted_frames, submitted);
    atomic_store(&a.rendered_frames, taken);
    atomic_store(&a.taken_frames, taken);
    now += dt;
    dbt_audio_clock_update(&a, now);
    int32_t limit = (int32_t)(50u * dt / 1000u);
    CHECK(a.clock_ppm - previous <= limit && previous - a.clock_ppm <= limit);
    previous = a.clock_ppm;
  }
  CHECK(a.clock_state == DBT_AUDIO_CLOCK_TRACKING && a.clock_ppm < -60 &&
        a.clock_ppm > -150);
  int32_t integral = a.clock_integral_q16;
  now += 3000;
  atomic_fetch_add(&a.submitted_frames, 288000);
  atomic_fetch_add(&a.rendered_frames, 288030);
  atomic_fetch_add(&a.taken_frames, 288030);
  dbt_audio_clock_update(&a, now);
  CHECK(a.clock_state == DBT_AUDIO_CLOCK_RECOVERING &&
        a.clock_integral_q16 == integral && !a.clock_observed_valid);
  dbt_audio_close(&a);
  CHECK(!live);
  return 0;
}
EX int clock_disabled(void) {
  CHECK(fixture(48000));
  a.config.adaptive_clock = false;
  dbt_audio_clock_reset(&a, 0);
  uint32_t count = 0;
  for (unsigned i = 0; i < 20; i++) {
    count += 24000;
    atomic_store(&a.submitted_frames, count);
    atomic_store(&a.rendered_frames, count);
    atomic_store(&a.taken_frames, count);
    dbt_audio_clock_update(&a, (i + 1) * 500u);
  }
  dbt_audio_clock_stats_t stats;
  CHECK(dbt_audio_clock_snapshot(&a, &stats) &&
        stats.state == DBT_AUDIO_CLOCK_OFF && !stats.ppm && !a.step_adjust);
  dbt_audio_close(&a);
  CHECK(!live);
  return 0;
}

EX const int32_t *clock_resampler_row(uint32_t rate, uint8_t phase,
                                      uint8_t *taps) {
  return dbt_resampler_phase(rate, phase, taps);
}
EX void clock_resampler_tone(uint32_t rate, uint32_t phase,
                             const int32_t *input, int32_t *output,
                             uint32_t count) {
  dbt_resampler_t state;
  dbt_resampler_init(&state, rate);
  for (uint32_t i = 0; i < count; i++) {
    int32_t right;
    dbt_resampler_filter(&state, input[i], input[i], phase, &output[i], &right);
    dbt_resampler_push(&state, input[i], input[i]);
  }
}
#include <math.h>
static double spectral_low, spectral_high, spectral_ripple, spectral_phase;
EX int clock_spectral_tones(void) {
  static const uint32_t rates[] = {8000,  11025,  12000, 16000, 22050,
                                   24000, 32000,  44100, 48000, 88200,
                                   96000, 176400, 192000};
  static const uint32_t frequencies[] = {1000, 10000, 18000, 20000};
  enum { N = 640, START = 96 };
  static int32_t in[N], out[N];
  const double pi = 3.14159265358979323846, amplitude = 400000000.0;
  spectral_low = 100;
  spectral_high = -100;
  spectral_ripple = 0;
  spectral_phase = 0;
  for (unsigned r = 0; r < sizeof(rates) / sizeof(rates[0]); r++)
    for (unsigned f = 0; f < sizeof(frequencies) / sizeof(frequencies[0]);
         f++) {
      if (frequencies[f] * 2u >= rates[r])
        continue;
      double w = 2 * pi * frequencies[f] / rates[r];
      for (unsigned i = 0; i < N; i++)
        in[i] = (int32_t)(amplitude * sin(w * i));
      double low = 1e9, high = 0, worst_phase = 0;
      for (unsigned p = 0; p < 128; p++) {
        uint32_t phase = (p << 25) | 0x01000001u;
        clock_resampler_tone(rates[r], phase, in, out, N);
        double ss = 0, cc = 0, sc = 0, ys = 0, yc = 0;
        for (unsigned i = START; i < N; i++) {
          double x = sin(w * i), y = cos(w * i), sample = out[i];
          ss += x * x;
          cc += y * y;
          sc += x * y;
          ys += sample * x;
          yc += sample * y;
        }
        double det = ss * cc - sc * sc;
        CHECK(det > 1);
        double a = (ys * cc - yc * sc) / det, b = (yc * ss - ys * sc) / det;
        double gain = sqrt(a * a + b * b) / amplitude;
        double reference = w * ((double)phase / 4294967296.0 -
                                dbt_resampler_delay_frames(rates[r]));
        double delta =
            atan2(sin(atan2(b, a) - reference), cos(atan2(b, a) - reference));
        double degrees = fabs(delta) * 180 / pi;
        if (gain < low)
          low = gain;
        if (gain > high)
          high = gain;
        if (degrees > worst_phase)
          worst_phase = degrees;
      }
      double low_db = 20 * log10(low), high_db = 20 * log10(high),
             ripple = high_db - low_db;
      if (low_db < spectral_low)
        spectral_low = low_db;
      if (high_db > spectral_high)
        spectral_high = high_db;
      if (ripple > spectral_ripple)
        spectral_ripple = ripple;
      if (worst_phase > spectral_phase)
        spectral_phase = worst_phase;
      CHECK(low_db > -0.06 && high_db < 0.01 && ripple < 0.005 &&
            worst_phase < 0.03);
    }
  return 0;
}
EX void clock_spectral_stats(double *low, double *high, double *ripple,
                             double *phase) {
  *low = spectral_low;
  *high = spectral_high;
  *ripple = spectral_ripple;
  *phase = spectral_phase;
}
static double drift_worst_dbc, drift_sideband_dbc;
EX int clock_drift_tone(void) {
  static const uint32_t rates[] = {44100, 48000, 96000, 192000};
  static const uint32_t freqs[] = {1000, 10000, 18000, 20000};
  enum { N = 8192, START = 256 };
  static int32_t input[N + 16], output[N];
  const double pi = 3.14159265358979323846, amp = 400000000.0;
  drift_worst_dbc = -300;
  drift_sideband_dbc = -300;
  for (unsigned r = 0; r < 4; r++)
    for (unsigned f = 0; f < 4; f++)
      for (unsigned sign = 0; sign < 2; sign++) {
        double w = 2 * pi * freqs[f] / rates[r];
        for (unsigned i = 0; i < N + 16; i++)
          input[i] = (int32_t)(amp * sin(w * i));
        dbt_resampler_t state;
        dbt_resampler_init(&state, rates[r]);
        uint64_t phase = 0, step = sign ? 4292819813ull : 4297114779ull;
        double ss = 0, cc = 0, sc = 0, ys = 0, yc = 0;
        for (unsigned i = 0; i < N; i++) {
          unsigned index = (unsigned)(phase >> 32);
          CHECK(index < N + 8);
          int32_t right;
          dbt_resampler_filter(&state, input[index], input[index],
                               (uint32_t)phase, &output[i], &right);
          uint64_t following = phase + step;
          unsigned consumed = (unsigned)(following >> 32) - index;
          for (unsigned k = 0; k < consumed; k++)
            dbt_resampler_push(&state, input[index + k], input[index + k]);
          if (i >= START) {
            double angle = w * ((double)phase / 4294967296.0 -
                                dbt_resampler_delay_frames(rates[r]));
            double x = sin(angle), y = cos(angle);
            ss += x * x;
            cc += y * y;
            sc += x * y;
            ys += output[i] * x;
            yc += output[i] * y;
          }
          phase = following;
        }
        double det = ss * cc - sc * sc;
        CHECK(det > 1);
        double gain_s = (ys * cc - yc * sc) / det,
               gain_c = (yc * ss - ys * sc) / det, energy = 0;
        phase = step * START;
        for (unsigned i = START; i < N; i++) {
          double angle = w * ((double)phase / 4294967296.0 -
                              dbt_resampler_delay_frames(rates[r]));
          double e = output[i] - gain_s * sin(angle) - gain_c * cos(angle);
          energy += e * e;
          phase += step;
        }
        double normal = (gain_s * gain_s + gain_c * gain_c) / 2.0;
        double dbc = 10 * log10((energy / (N - START)) / normal);
        if (dbc > drift_worst_dbc)
          drift_worst_dbc = dbc;
        CHECK(dbc < -68.0);
        /* Project residual onto the first four phase-sweep sidebands. The
         * fitted fundamental was removed, so leakage from finite windows is
         * not mistaken for an actual sideband. */
        double mod = (double)(sign ? 2147483 : 2147483) / 4294967296.0;
        for (unsigned order = 1; order <= 4; order++)
          for (unsigned side = 0; side < 2; side++) {
            double re = 0, im = 0;
            phase = step * START;
            double frequency = w * (double)step / 4294967296.0 +
                               (side ? 1 : -1) * 2 * pi * mod * order;
            for (unsigned i = START; i < N; i++) {
              double angle = w * ((double)phase / 4294967296.0 -
                                  dbt_resampler_delay_frames(rates[r]));
              double e = output[i] - gain_s * sin(angle) - gain_c * cos(angle);
              re += e * cos(frequency * i);
              im += e * sin(frequency * i);
              phase += step;
            }
            double sideband =
                2 * sqrt(re * re + im * im) / (N - START) / sqrt(2 * normal);
            double level = sideband > 0 ? 20 * log10(sideband) : -300;
            if (level > drift_sideband_dbc)
              drift_sideband_dbc = level;
            CHECK(level < -72.0);
          }
      }
  return 0;
}
EX double clock_drift_tone_worst_dbc(void) { return drift_worst_dbc; }
EX double clock_drift_sideband_worst_dbc(void) { return drift_sideband_dbc; }
EX int clock_resampler_bounds_and_wrap(void) {
  static const uint32_t rates[] = {44100, 96000, 192000};
  for (unsigned r = 0; r < 3; r++) {
    for (unsigned p = 0; p <= 128; p++) {
      uint8_t taps;
      const int32_t *row = dbt_resampler_phase(rates[r], (uint8_t)p, &taps);
      CHECK(row);
      uint64_t sum = 0;
      int64_t dc = 0;
      for (unsigned k = 0; k < taps; k++) {
        dc += row[k];
        sum += row[k] < 0 ? -(int64_t)row[k] : row[k];
      }
      CHECK(dc == (1ll << 30) && sum + taps < (1ull << 32));
    }
    dbt_resampler_t state;
    dbt_resampler_init(&state, rates[r]);
    uint8_t taps;
    const int32_t *extreme = dbt_resampler_phase(rates[r], 64, &taps);
    for (unsigned k = taps - 1; k > 0; k--) {
      int32_t l = extreme[k] >= 0 ? INT32_MAX : INT32_MIN;
      int32_t rr = extreme[k] >= 0 ? INT32_MIN : INT32_MAX;
      dbt_resampler_push(&state, l, rr);
    }
    int32_t left, right;
    dbt_resampler_filter(&state, extreme[0] >= 0 ? INT32_MAX : INT32_MIN,
                         extreme[0] >= 0 ? INT32_MIN : INT32_MAX, 64u << 25,
                         &left, &right);
    CHECK(left == INT32_MAX && right == INT32_MIN);
    dbt_resampler_init(&state, rates[r]);
    for (unsigned i = 0; i < 128; i++) {
      int32_t x = (int32_t)((i * 137u % 255u) * 1000000u) - 127000000;
      dbt_resampler_push(&state, x, x);
    }
    int32_t l1, r1, l2, r2;
    dbt_resampler_filter(&state, 50000000, 50000000, UINT32_MAX, &l1, &r1);
    dbt_resampler_push(&state, 50000000, 50000000);
    dbt_resampler_filter(&state, -43000000, -43000000, 0, &l2, &r2);
    int64_t delta = (int64_t)l1 - l2;
    if (delta < 0)
      delta = -delta;
    CHECK(delta < 64 && l1 == r1 && l2 == r2);
  }
  return 0;
}
EX int clock_resampler_drain(void) {
  static int32_t data[2048], output[4096];
  CHECK(fixture(96000));
  CHECK(dbt_audio_controls(&a, 0, 0, 0, 127, FDSP_BYPASS) == DBT_OK);
  (void)dbt_audio_render(&a, 0);
  fixed_dsp_controls(&a.dsp, 0, 0, 0, 127, FDSP_BYPASS, true);
  for (unsigned i = 0; i < 1024; i++) {
    data[2 * i] = (int32_t)(i * 65536u);
    data[2 * i + 1] = (int32_t)(i * 32768u);
  }
  CHECK(dbt_audio_submit(&a, data, 1024) == DBT_OK);
  /* Stop can arrive before the first render, so queued input still primes the
   * tail. */
  dbt_audio_begin_drain(&a);
  uint32_t delay = dbt_resampler_delay_frames(96000);
  CHECK(a.drain_remaining == delay &&
        dbt_audio_render(&a, 2048) == 1024 + delay);
  CHECK(dbt_audio_take(&a, output, 1024 + delay) == 1024 + delay &&
        !a.drain_remaining);
  uint8_t taps;
  const int32_t *row = dbt_resampler_phase(96000, 0, &taps);
  int64_t left_sum = 0, right_sum = 0;
  for (unsigned k = 0; k < taps; k++) {
    unsigned index = 1023u + delay - k;
    if (index > 1023u)
      index = 1023u;
    left_sum += (int64_t)data[2 * index] * row[k];
    right_sum += (int64_t)data[2 * index + 1] * row[k];
  }
  int32_t left = (int32_t)(left_sum >> 30), right = (int32_t)(right_sum >> 30);
  fixed_dsp_t reference;
  CHECK(fixed_dsp_init(&reference, 96000, 90, 500, 512));
  fixed_dsp_controls(&reference, 0, 0, 0, 127, FDSP_BYPASS, true);
  fixed_dsp_process(&reference, &left, &right);
  CHECK(output[2 * (1024 + delay - 1)] == left &&
        output[2 * (1024 + delay - 1) + 1] == right);
  dbt_audio_begin_drain(&a);
  CHECK(dbt_audio_render(&a, 20) == 0);
  dbt_audio_close(&a);
  CHECK(!live);
  return 0;
}
