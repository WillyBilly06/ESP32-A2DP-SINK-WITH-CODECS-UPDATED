#include "lhdcv5_perf_helper.h"
#include "dbt_lhdcv5_codec.h"
#include "dbt_resampler.h"
#include <limits.h>
#include <string.h>

/* Enabled only in a dedicated DBT_LHDC_STAGE_PROFILE build. One synchronous
 * worker owns these counters; reset/read only outside decode. */
static uint64_t stage_sum[6];
static uint32_t stage_max[6], stage_calls;
void dbt_lhdc_stage_record(uint32_t pre, uint32_t entropy, uint32_t mantissa,
                           uint32_t synthesis, uint32_t imdct,
                           uint32_t overlap) {
  uint32_t values[6] = {pre, entropy, mantissa, synthesis, imdct, overlap};
  for (unsigned i = 0; i < 6; i++) {
    stage_sum[i] += values[i];
    if (values[i] > stage_max[i])
      stage_max[i] = values[i];
  }
  stage_calls++;
}
typedef struct {
  dbt_lhdc_perf_hooks_t hooks;
  dbt_lhdc_perf_result_t *result;
  uint64_t packet_callback_us;
  uint32_t expected_rate, expected_width, expected_channels;
  bool measuring;
} runner_t;
static uint8_t *allocate(void *context, uint32_t n) {
  runner_t *r = context;
  return r->hooks.allocate(r->hooks.context, n);
}
static void release(void *context, uint8_t *p) {
  runner_t *r = context;
  r->hooks.release(r->hooks.context, p);
}
static uint32_t elapsed(uint64_t start, uint64_t end) {
  return end < start                ? UINT32_MAX
         : end - start > UINT32_MAX ? UINT32_MAX
                                    : (uint32_t)(end - start);
}
static dbt_result_t pcm(void *context, const void *samples, uint16_t frames,
                        const dbt_codec_config_t *config, uint32_t timestamp) {
  runner_t *r = context;
  (void)timestamp;
  if (!samples || !frames || !config || config->rate != r->expected_rate ||
      config->channels != r->expected_channels ||
      config->sample_bytes != r->expected_width)
    return DBT_INVALID;
  uint64_t start = r->measuring ? r->hooks.now_us(r->hooks.context) : 0;
  const uint8_t *p = samples;
  uint32_t bytes = (uint32_t)frames * config->channels * config->sample_bytes;
  r->result->checksum = (r->result->checksum * 16777619u) ^ p[0] ^
                        ((uint32_t)p[bytes / 2] << 8) ^
                        ((uint32_t)p[bytes - 1] << 16);
  if (r->measuring) {
    r->result->callbacks++;
    r->result->sample_frames += frames;
    uint64_t end = r->hooks.now_us(r->hooks.context);
    uint32_t duration = elapsed(start, end);
    if (duration == UINT32_MAX)
      return DBT_INVALID;
    r->packet_callback_us += duration;
  }
  return DBT_OK;
}
dbt_result_t dbt_lhdc_perf_run(dbt_lhdc_perf_hooks_t hooks, const uint8_t *cie,
                               uint8_t cie_size,
                               const dbt_lhdc_perf_packet_t *packets,
                               uint8_t count, uint16_t warmup,
                               uint16_t measured,
                               dbt_lhdc_perf_result_t *result) {
  if (!hooks.allocate || !hooks.release || !hooks.now_us || !cie || !packets ||
      !count || count > 4 || !measured || measured > 1000 || warmup > 1000 ||
      !result)
    return DBT_INVALID;
  for (unsigned i = 0; i < count; i++)
    if (!packets[i].bytes || !packets[i].size || packets[i].size > 4096)
      return DBT_INVALID;
  memset(result, 0, sizeof(*result));
  result->packet_min_us = result->decoder_estimate_min_us = UINT32_MAX;
  memset(stage_sum, 0, sizeof(stage_sum));
  memset(stage_max, 0, sizeof(stage_max));
  stage_calls = 0;
  dbt_codec_config_t config;
  dbt_result_t r = dbt_lhdcv5_configure(cie, cie_size, &config);
  if (r != DBT_OK)
    return r;
  runner_t runner = {.hooks = hooks,
                     .result = result,
                     .expected_rate = config.rate,
                     .expected_width = config.sample_bytes,
                     .expected_channels = config.channels};
  dbt_codec_decoder_t decoder;
  if (dbt_codec_init(&decoder, (dbt_codec_hooks_t){&runner, allocate, release,
                                                   pcm}) != DBT_OK)
    return DBT_INVALID;
  config.ops = &dbt_lhdcv5_codec;
  r = dbt_codec_open(&decoder, &config);
  if (r != DBT_OK)
    return r;
  for (unsigned i = 0; i < (unsigned)warmup + measured; i++) {
    if (i == warmup) {
      runner.measuring = true;
      memset(stage_sum, 0, sizeof(stage_sum));
      memset(stage_max, 0, sizeof(stage_max));
      stage_calls = 0;
    }
    const dbt_lhdc_perf_packet_t *packet = &packets[i % count];
    dbt_avdtp_media_t media = {.payload = packet->bytes,
                               .size = packet->size,
                               .sequence = (uint16_t)i,
                               .timestamp = i * config.samples * 4u,
                               .ssrc = 1,
                               .payload_type = 96};
    runner.packet_callback_us = 0;
    uint32_t before_callbacks = result->callbacks;
    uint64_t start = runner.measuring ? hooks.now_us(hooks.context) : 0;
    r = config.ops->media(&decoder, &media, i);
    if (r != DBT_OK)
      break;
    if (runner.measuring) {
      uint64_t end = hooks.now_us(hooks.context);
      uint32_t total = elapsed(start, end);
      if (total == UINT32_MAX || result->callbacks == before_callbacks ||
          runner.packet_callback_us > total) {
        r = DBT_INVALID;
        break;
      }
      uint32_t estimate = total - (uint32_t)runner.packet_callback_us;
      result->packets++;
      result->packet_sum_us += total;
      result->callback_sum_us += runner.packet_callback_us;
      result->decoder_estimate_sum_us += estimate;
      if (total < result->packet_min_us)
        result->packet_min_us = total;
      if (total > result->packet_max_us)
        result->packet_max_us = total;
      if (estimate < result->decoder_estimate_min_us)
        result->decoder_estimate_min_us = estimate;
      if (estimate > result->decoder_estimate_max_us)
        result->decoder_estimate_max_us = estimate;
    }
  }
  memcpy(result->stage_sum_us, stage_sum, sizeof(stage_sum));
  memcpy(result->stage_max_us, stage_max, sizeof(stage_max));
  result->stage_calls = stage_calls;
  dbt_codec_close(&decoder);
  return r;
}
dbt_result_t dbt_lhdc_perf_resampler(dbt_lhdc_perf_clock_fn now, void *ctx,
                                     uint32_t rate, uint32_t samples,
                                     uint64_t *duration, uint32_t *checksum) {
  if (!now || !rate || !samples || samples > 65536 || !duration || !checksum)
    return DBT_INVALID;
  dbt_resampler_t state;
  dbt_resampler_init(&state, rate);
  uint32_t hash = 2166136261u;
  uint64_t start = now(ctx);
  for (uint32_t i = 0; i < samples; i++) {
    int32_t left = (int32_t)(i * 104729u), right = (int32_t)(i * 65537u), x, y;
    dbt_resampler_filter(&state, left, right, i * 2147483u, &x, &y);
    dbt_resampler_push(&state, left, right);
    hash = (hash ^ (uint32_t)x) * 16777619u;
    hash = (hash ^ (uint32_t)y) * 16777619u;
  }
  uint64_t end = now(ctx);
  if (end < start)
    return DBT_INVALID;
  *duration = end - start;
  *checksum = hash;
  return DBT_OK;
}
