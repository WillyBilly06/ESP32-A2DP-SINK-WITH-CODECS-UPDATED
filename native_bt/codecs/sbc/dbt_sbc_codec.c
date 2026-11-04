#include "dbt_sbc_codec.h"
#include "dbt_sbc_decoder.h"

typedef struct {
  dbt_codec_decoder_t *owner;
  dbt_sbc_decoder_t decoder;
  dbt_sbc_media_t media;
} sbc_state_t;
static uint8_t *allocate(void *context, uint16_t bytes) {
  return dbt_codec_allocate(((sbc_state_t *)context)->owner, bytes);
}
static void release(void *context, uint8_t *bytes) {
  dbt_codec_release(((sbc_state_t *)context)->owner, bytes);
}
static dbt_result_t pcm(void *context, const int16_t *samples, uint16_t frames,
                        uint8_t channels, uint32_t rate, uint32_t timestamp) {
  sbc_state_t *s = context;
  dbt_codec_decoder_t *d = s->owner;
  if (channels != d->config.channels || rate != d->config.rate ||
      frames != d->config.samples)
    return DBT_INVALID;

  dbt_result_t r =
      d->hooks.pcm(d->hooks.context, samples, frames, &d->config, timestamp);
  if (r == DBT_OK)
    ++d->frames;
  return r;
}
static dbt_result_t frame(void *context, const dbt_sbc_frame_t *f,
                          uint32_t timestamp) {
  return dbt_sbc_decoder_frame(&((sbc_state_t *)context)->decoder, f,
                               timestamp);
}
static dbt_result_t configure(const uint8_t *cie, uint16_t size,
                              dbt_codec_config_t *out) {
  dbt_sbc_config_t c;
  if (!cie || !out || size != 6 || cie[0] || cie[1])
    return DBT_UNSUPPORTED;
  dbt_result_t r = dbt_sbc_configure(cie, size, &c);
  if (r != DBT_OK)
    return r;
  dbt_codec_config_t n = {0};
  n.rate = c.rate;
  n.channels = c.channels;
  n.samples = c.samples;
  n.sample_bytes = 2;
  n.cie_size = 6;
  for (unsigned i = 0; i < 6; ++i)
    n.cie[i] = cie[i];
  *out = n;
  return DBT_OK;
}
static void close_decoder(dbt_codec_decoder_t *d) {
  sbc_state_t *s = d->state;
  if (!s)
    return;
  dbt_sbc_media_reset(&s->media);
  dbt_sbc_decoder_close(&s->decoder);
  dbt_codec_release(d, s);
  d->state = 0;
}
static dbt_result_t open_decoder(dbt_codec_decoder_t *d) {
  dbt_codec_config_t c;
  dbt_sbc_config_t selected;
  if (configure(d->config.cie, d->config.cie_size, &c) != DBT_OK ||
      c.rate != d->config.rate || c.channels != d->config.channels ||
      c.samples != d->config.samples || d->config.sample_bytes != 2 ||
      dbt_sbc_configure(d->config.cie, d->config.cie_size, &selected) != DBT_OK)
    return DBT_INVALID;

  sbc_state_t *s = (sbc_state_t *)dbt_codec_allocate(d, sizeof(*s));
  if (!s)
    return DBT_NO_SPACE;
  *s = (sbc_state_t){0};
  s->owner = d;
  d->state = s;
  dbt_sbc_decoder_hooks_t dh = {s, allocate, release, pcm};
  dbt_result_t r = dbt_sbc_decoder_init(&s->decoder, dh);
  if (r != DBT_OK)
    return r;
  r = dbt_sbc_decoder_open(&s->decoder, selected);
  if (r != DBT_OK)
    return r;
  dbt_sbc_media_hooks_t mh = {s, allocate, release, frame};
  return dbt_sbc_media_init(&s->media, mh, selected, 100);
}
static dbt_result_t reset_decoder(dbt_codec_decoder_t *d) {
  sbc_state_t *s = d->state;
  if (!s)
    return DBT_INVALID;

  dbt_sbc_media_reset(&s->media);
  return dbt_sbc_decoder_reset(&s->decoder);
}
static dbt_result_t media(dbt_codec_decoder_t *d, const dbt_avdtp_media_t *p,
                          uint32_t now) {
  sbc_state_t *s = d->state;
  return s ? dbt_sbc_media_receive(&s->media, p, now) : DBT_INVALID;
}
static void expire(dbt_codec_decoder_t *d, uint32_t now) {
  sbc_state_t *s = d->state;
  if (s)
    dbt_sbc_media_expire(&s->media, now);
}
const dbt_codec_ops_t dbt_sbc_codec = {
    "SBC",         configure, open_decoder, close_decoder,
    reset_decoder, media,     expire};
