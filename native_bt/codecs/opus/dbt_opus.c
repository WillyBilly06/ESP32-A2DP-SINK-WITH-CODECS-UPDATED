#include "dbt_opus.h"
#include "opus.h"
#include "scratch/dbt_opus_guarded_api.h"
#include "scratch/dbt_opus_scratch.h"

typedef struct {
  OpusDecoder *codec;
  uint8_t *allocation;
  dbt_opus_scratch scratch;
  bool usable;
  uint8_t *fragment;
  uint32_t fragment_at, timestamp, ssrc, last_ssrc;
  uint16_t fragment_size, fragment_capacity, next_sequence, last_sequence;
  uint8_t remaining;
  bool sequence_valid;
  uint32_t codec_bytes;
} opus_state_t;
static uint32_t align8(uint32_t n) { return (n + 7u) & ~7u; }
static void copy(void *to, const void *from, unsigned n) {
  uint8_t *d = to;
  const uint8_t *s = from;
  for (unsigned i = 0; i < n; ++i)
    d[i] = s[i];
}
static uint32_t le32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}
static unsigned le16(const uint8_t *p) {
  return (unsigned)p[0] | ((unsigned)p[1] << 8);
}
static bool one(unsigned v) { return v && !(v & (v - 1u)); }
static bool android(const uint8_t *p) {
  return le32(p + 2) == 0xe0u && le16(p + 6) == 1u;
}
static bool pipewire(const uint8_t *p) {
  return le32(p + 2) == 0x5f1u && le16(p + 6) == 0x1005u;
}
dbt_result_t dbt_opus_configure(const uint8_t *p, uint16_t size,
                                dbt_codec_config_t *out) {
  if (!p || !out || size < 8 || p[0] || p[1] != 0xff)
    return DBT_UNSUPPORTED;
  uint16_t samples = 0;
  uint8_t channels = 0;
  if (android(p)) {
    if (size != 9)
      return DBT_INVALID;
    unsigned ch = p[8] & 7u, frame = p[8] & 0x18u;
    if ((p[8] & 0x60u) || !(p[8] & 0x80u) || (ch != 1u && ch != 2u) ||
        (frame != 8u && frame != 16u))
      return DBT_INVALID;
    channels = (uint8_t)ch;
    samples = frame == 8u ? 480u : 960u;
  } else if (pipewire(p)) {
    if (size != 26)
      return DBT_INVALID;
    uint32_t location = le32(p + 10);
    unsigned duration = p[14];
    channels = p[8];
    if ((channels != 1 && channels != 2) || p[9] != (channels == 2) ||
        (channels == 1 ? (location != 1u && location != 2u) : location != 3u) ||
        !one(duration) || duration > 16u)
      return DBT_INVALID;
    /* This sink has no reverse audio stream. */
    for (unsigned i = 17; i < 26; ++i)
      if (p[i])
        return DBT_UNSUPPORTED;
    switch (duration) {
    case 1:
      samples = 120;
      break;
    case 2:
      samples = 240;
      break;
    case 4:
      samples = 480;
      break;
    case 8:
      samples = 960;
      break;
    case 16:
      samples = 1920;
      break;
    default:
      return DBT_INVALID;
    }
  } else
    return DBT_UNSUPPORTED;
  dbt_codec_config_t next = {0};
  next.rate = 48000;
  next.samples = samples;
  next.channels = channels;
  next.sample_bytes = 2;
  next.cie_size = (uint8_t)size;
  copy(next.cie, p, size);
  *out = next;
  return DBT_OK;
}
uint8_t dbt_a2dp_opus_validate(const dbt_avdtp_endpoint_t *endpoint,
                               const dbt_avdtp_cap_t *codec) {
  if (!endpoint || !codec || codec->category != 7 || !codec->data)
    return 0x18;
  dbt_codec_config_t checked;
  dbt_avdtp_caps_t local;
  dbt_avdtp_cap_error_t error;
  if (dbt_opus_configure(codec->data, codec->size, &checked) != DBT_OK ||
      dbt_avdtp_caps_validate(endpoint->capabilities, endpoint->size,
                              DBT_AVDTP_CAPABILITIES, &local,
                              &error) != DBT_OK ||
      local.codec.size != codec->size)
    return 0x29;
  const uint8_t *p = codec->data, *c = local.codec.data;
  if (c[0] || c[1] != 0xff || le32(p + 2) != le32(c + 2) ||
      le16(p + 6) != le16(c + 6))
    return 0x29;
  if (android(p))
    return (p[8] & c[8]) == p[8] ? 0 : 0x29;
  if (p[8] > c[8] || p[9] > c[9] ||
      (le32(p + 10) & le32(c + 10)) != le32(p + 10) || !(p[14] & c[14]))
    return 0x29;
  unsigned bitrate = le16(p + 15), limit = le16(c + 15);
  return limit && (!bitrate || bitrate > limit) ? 0x29 : 0;
}
static uint8_t *workspace(opus_state_t *s) {
  return (uint8_t *)s + align8(sizeof(*s));
}
static opus_int16 *pcm(opus_state_t *s) {
  return (opus_int16 *)(workspace(s) + s->codec_bytes);
}
static void clear_fragment(dbt_codec_decoder_t *d) {
  opus_state_t *s = d->state;
  if (!s)
    return;
  if (s->fragment)
    dbt_codec_release(d, s->fragment);
  s->fragment = 0;
  s->fragment_size = s->fragment_capacity = 0;
  s->remaining = 0;
}
static void close_decoder(dbt_codec_decoder_t *d) {
  if (!d->state)
    return;
  clear_fragment(d);
  dbt_codec_release(d, ((opus_state_t *)d->state)->allocation);
  d->state = 0;
}
static dbt_result_t reset_decoder(dbt_codec_decoder_t *d) {
  opus_state_t *s = d->state;
  if (!s)
    return DBT_INVALID;
  clear_fragment(d);
  s->sequence_valid = false;
  s->usable = opus_decoder_ctl(s->codec, OPUS_RESET_STATE) == OPUS_OK;
  return s->usable ? DBT_OK : DBT_INVALID;
}
static dbt_result_t open_decoder(dbt_codec_decoder_t *d) {
  dbt_codec_config_t c;
  if (dbt_opus_configure(d->config.cie, d->config.cie_size, &c) != DBT_OK ||
      c.rate != d->config.rate || c.channels != d->config.channels ||
      c.samples != d->config.samples || d->config.sample_bytes != 2)
    return DBT_INVALID;
  int size = opus_decoder_get_size(c.channels);
  if (size <= 0)
    return DBT_INVALID;
  uint32_t codec_bytes = align8((uint32_t)size);
  uint32_t pcm_bytes = (uint32_t)c.samples * c.channels * 2u;
  uint32_t scratch_bytes = (uint32_t)dbt_opus_scratch_capacity(c.channels);
  uint32_t bytes =
      align8(sizeof(opus_state_t)) + codec_bytes + pcm_bytes + scratch_bytes;
  uint32_t padding = (uint32_t)_Alignof(opus_state_t) - 1u;
  uint8_t *allocation = dbt_codec_allocate(d, bytes + padding);
  if (!allocation)
    return DBT_NO_SPACE;
  opus_state_t *s =
      (opus_state_t *)(((uintptr_t)allocation + padding) & ~(uintptr_t)padding);
  *s = (opus_state_t){0};
  s->allocation = allocation;
  d->state = s;
  s->codec_bytes = codec_bytes;
  s->codec = (OpusDecoder *)workspace(s);
  if (dbt_opus_scratch_bind(&s->scratch, (uint8_t *)pcm(s) + pcm_bytes,
                            scratch_bytes) != DBT_OPUS_SCRATCH_OK)
    return DBT_INVALID;
  s->usable = opus_decoder_init(s->codec, 48000, c.channels) == OPUS_OK;
  return s->usable ? DBT_OK : DBT_INVALID;
}
typedef struct {
  dbt_codec_decoder_t *decoder;
  const uint8_t *packet;
  uint16_t bytes;
} opus_job_t;
static int run_decode(void *opaque) {
  opus_job_t *job = opaque;
  dbt_codec_decoder_t *d = job->decoder;
  opus_state_t *s = d->state;
  return dbt_opus_arena_decode(s->codec, job->packet, job->bytes, pcm(s),
                               d->config.samples, 0);
}
static int recover_decode(void *opaque) {
  opus_job_t *job = opaque;
  dbt_codec_decoder_t *d = job->decoder;
  opus_state_t *s = d->state;
  uint8_t *out = (uint8_t *)pcm(s);
  unsigned size = (unsigned)d->config.samples * d->config.channels * 2u;
  for (unsigned i = 0; i < size; ++i)
    out[i] = 0;
  s->usable = opus_decoder_ctl(s->codec, OPUS_RESET_STATE) == OPUS_OK;
  return s->usable ? 0 : -1;
}
static dbt_result_t decode(dbt_codec_decoder_t *d, const uint8_t *p,
                           uint16_t size, uint32_t timestamp) {
  opus_state_t *s = d->state;
  if (!s || !s->usable)
    return DBT_INVALID;
  if (opus_packet_get_nb_samples(p, size, 48000) != (int)d->config.samples)
    return DBT_INVALID;
  opus_job_t job = {d, p, size};
  int samples = 0;
  int result = dbt_opus_scratch_run(&s->scratch, run_decode, &job,
                                    recover_decode, &job, &samples);
  if (result != DBT_OPUS_SCRATCH_OK)
    return result == DBT_OPUS_SCRATCH_EXHAUSTED ? DBT_NO_SPACE : DBT_INVALID;
  if (samples != (int)d->config.samples) {
    (void)recover_decode(&job);
    return DBT_INVALID;
  }
  dbt_result_t r = d->hooks.pcm(d->hooks.context, pcm(s), (uint16_t)samples,
                                &d->config, timestamp);
  if (r == DBT_OK)
    ++d->frames;
  return r;
}
static void expire(dbt_codec_decoder_t *d, uint32_t now) {
  opus_state_t *s = d->state;
  if (s && s->fragment && now - s->fragment_at >= 100u)
    (void)reset_decoder(d);
}
static dbt_result_t media(dbt_codec_decoder_t *d, const dbt_avdtp_media_t *p,
                          uint32_t now) {
  opus_state_t *s = d->state;
  if (!s || !p || !p->payload || p->size < 2 || p->payload_type < 96)
    return DBT_INVALID;
  expire(d, now);
  if (s->sequence_valid && (p->ssrc != s->last_ssrc ||
                            p->sequence != (uint16_t)(s->last_sequence + 1u))) {
    if (reset_decoder(d) != DBT_OK)
      return DBT_INVALID;
  }
  s->sequence_valid = true;
  s->last_ssrc = p->ssrc;
  s->last_sequence = p->sequence;
  unsigned h = p->payload[0], count = h & 15u;
  bool fragmented = (h & 128u) != 0, first = (h & 64u) != 0,
       last = (h & 32u) != 0;
  const uint8_t *data = p->payload + 1;
  uint16_t size = (uint16_t)(p->size - 1u);
  if (h & 16u) {
    clear_fragment(d);
    return DBT_INVALID;
  }
  if (!fragmented) {
    if (s->fragment && reset_decoder(d) != DBT_OK)
      return DBT_INVALID;
    if (first || last || count != 1)
      return DBT_INVALID;
    return decode(d, data, size, p->timestamp);
  }
  if (!count || (first && (last || count < 2u)) || (last && count != 1u) ||
      (!last && count == 1u)) {
    clear_fragment(d);
    return DBT_INVALID;
  }
  if (first) {
    if (s->fragment && reset_decoder(d) != DBT_OK)
      return DBT_INVALID;
    s->timestamp = p->timestamp;
    s->ssrc = p->ssrc;
    s->fragment_at = now;
    s->remaining = (uint8_t)count;
    s->next_sequence = p->sequence;
  }
  if (!s->remaining || count != s->remaining ||
      p->sequence != s->next_sequence || p->timestamp != s->timestamp ||
      p->ssrc != s->ssrc) {
    clear_fragment(d);
    return DBT_INVALID;
  }
  uint32_t needed = (uint32_t)s->fragment_size + size;
  if (needed > DBT_OPUS_FRAGMENT_LIMIT) {
    clear_fragment(d);
    return DBT_INVALID;
  }
  if (needed > s->fragment_capacity) {
    uint32_t capacity = 256u;
    while (capacity < needed && capacity < 4096u)
      capacity <<= 1;
    if (capacity < needed)
      capacity = DBT_OPUS_FRAGMENT_LIMIT;
    uint8_t *next = dbt_codec_allocate(d, capacity);
    if (!next) {
      clear_fragment(d);
      return DBT_NO_SPACE;
    }
    copy(next, s->fragment, s->fragment_size);
    if (s->fragment)
      dbt_codec_release(d, s->fragment);
    s->fragment = next;
    s->fragment_capacity = (uint16_t)capacity;
  }
  copy(s->fragment + s->fragment_size, data, size);
  s->fragment_size = (uint16_t)needed;
  --s->remaining;
  ++s->next_sequence;
  if (!last)
    return DBT_OK;
  dbt_result_t result = decode(d, s->fragment, s->fragment_size, s->timestamp);
  clear_fragment(d);
  return result;
}
const dbt_codec_ops_t dbt_opus_codec = {"Opus",        dbt_opus_configure,
                                        open_decoder,  close_decoder,
                                        reset_decoder, media,
                                        expire};
