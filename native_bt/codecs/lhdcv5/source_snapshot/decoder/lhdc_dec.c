#include "lhdc_dec.h"
#include "lhdc_dec_internal.h"
#include "lhdc_bit_reader.h"
#include "lhdc_imdct.h"
#include "lhdc_sns_synth.h"
#include "lhdc_entropy_dec.h"
#include "lhdc_tables.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>   /* snprintf used in g_lhdc_trace diagnostics */
#if defined(LHDC_HOST_BUILD)
  #define ESP_LOGI(tag, ...) do { printf("[I]" tag ": " __VA_ARGS__); printf("\n"); } while (0)
  #define ESP_LOGW(tag, ...) do { printf("[W]" tag ": " __VA_ARGS__); printf("\n"); } while (0)
  #define ESP_LOGE(tag, ...) do { printf("[E]" tag ": " __VA_ARGS__); printf("\n"); } while (0)
#else
  #include "esp_log.h"
  #include "esp_timer.h"
  #include "esp_attr.h"
#endif

/* Split-workspace allocation: the rate-sized work buffers are allocated
 * individually instead of being carved from one contiguous ~32.5 KB slab. The
 * largest single buffer is mdct_out = M*4 B (7,680 B at 192k), so LHDC can
 * still start when the classic-ESP32 byte-DRAM heap is fragmented into holes
 * far smaller than the old slab. Buffers stay in internal DRAM (IMDCT/entropy
 * hot buffers); the host harness uses plain malloc/free. */
#if defined(LHDC_HOST_BUILD)
  #define LHDC_DEC_MALLOC(n) malloc(n)
  #define LHDC_DEC_FREE(p)   free(p)
#else
  #include "esp_heap_caps.h"
  #define LHDC_DEC_MALLOC(n) heap_caps_malloc((n), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)
  #define LHDC_DEC_FREE(p)   heap_caps_free(p)
#endif

/* Hot per-element decode loops placed in IRAM: at 96k (N=960, 2x the work of
 * 48k in the same 5ms frame) the decode runs ~86% of budget, so flash-cache
 * misses under the BT controller's IRQs spike individual frames over budget ->
 * underrun -> pops/skips. IRAM removes that jitter. (HOST build: no-op.) */
#if defined(LHDC_HOST_BUILD)
  #define LHDC_HOT
#else
  #define LHDC_HOT IRAM_ATTR
#endif

/* Stage profiler: cheap on-device timing of the per-channel decode stages so we
 * can see where the real-time budget goes (IMDCT vs entropy vs the rest).
 * The periodic PROF ESP_LOGI stalls the decode task ~9 ms on the UART and
 * stutters audio, so it is OFF by default; set to 1 for bring-up only
 * (overridable from the build: -DLHDCV5_DEC_PROFILE=1). */
#ifndef LHDCV5_DEC_PROFILE
#define LHDCV5_DEC_PROFILE 0
#endif
#if defined(LHDC_HOST_BUILD)
  /* Real microsecond clock on host so the PROF path works there too. */
  #include <time.h>
  static int64_t lhdc_host_now_us(void)
  {
      struct timespec ts;
      timespec_get(&ts, TIME_UTC);
      return (int64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
  }
  #define LHDC_NOW_US() lhdc_host_now_us()
#else
  #define LHDC_NOW_US() esp_timer_get_time()
#endif

/* 96k output level. MUST be 1.0: the IMDCT 1/M normalization (self-test-verified
 * == the exact 2/N formula) already yields the correct level, so a full-scale
 * source decodes to full scale. A >1.0 boost (was 1.10 for a subjective LDAC
 * level-match) pushes loud content — e.g. a near-0 dBFS test tone / frequency
 * generator — PAST full scale, where LHDC_OUT_SAMPLE hard-clips it → continuous
 * buzzy static. Not worth clipping for ~0.8 dB; system/phone volume covers it. */
#define LHDC_96K_GAIN 1.0f

/* One-shot decode trace: set to 1 by the wrapper at stream start; the decoder
 * logs the first frame's leading-section values, then self-clears. Diagnostic. */
volatile int g_lhdc_trace = 0;

/*
 * Frame scramble (PROJECT_CONTEXT §6, validated against 2400 real frames):
 * the first 8 payload bytes are permuted (perm selected by payload[8]&1) and
 * XOR-masked. Byte 8 onward is plaintext. The decoder inverts this in place.
 */
static const uint8_t LHDC_XOR_MASK[8] = {
    0xFF, 0xE7, 0x7A, 0xB3, 0xDA, 0xE5, 0xCD, 0x73
};

/*
 * Shared IMDCT window cache. The Princen-Bradley window depends only on
 * mdct_size and is read-only after generation, so one copy is shared across all
 * decoder instances instead of living in each ~per-instance workspace (saves
 * MAX_MDCT floats of heap). Regenerated only when mdct_size changes.
 * Not re-entrant, but A2DP sink decodes on a single task.
 */
/* For 480 (the only negotiated config) the window is a const flash table
 * (lhdc_get_window_const) -> no RAM copy. Any other size falls back to a lazily
 * malloc'd buffer (never hit today; kept for future 96/192k). */
static float   *s_imdct_window = NULL;
static int      s_imdct_window_size = 0;

static const float *lhdc_dec_get_window(int mdct_size)
{
#if defined(LHDC_HOST_BUILD)
    /* Host-only A/B hook: load the synthesis window verbatim from a raw float32
     * file (LHDC_WINFILE) so candidate 960/1920 windows can be tested against the
     * real encoder round-trip without recompiling. */
    { const char *wf = getenv("LHDC_WINFILE");
      if (wf) {
          static float s_winfile[2048]; static int s_winfile_n = 0;
          if (s_winfile_n != mdct_size) {
              FILE *f = fopen(wf, "rb");
              if (f) { s_winfile_n = (int)fread(s_winfile, sizeof(float), (size_t)mdct_size, f); fclose(f); }
          }
          if (s_winfile_n == mdct_size) return s_winfile;
      } }
#endif
    const float *cw = lhdc_get_window_const(mdct_size);
    if (cw) {                                  /* flash window for this size -> zero RAM */
        /* Release any RAM window kept from a previous higher rate (e.g. after
         * switching 96k->48k): 48k uses the flash const window, so the malloc'd
         * buffer is dead weight. Free it back to the heap. */
        if (s_imdct_window) { free(s_imdct_window); s_imdct_window = NULL; s_imdct_window_size = 0; }
        return cw;
    }
    if (mdct_size != s_imdct_window_size && mdct_size <= LHDC_DEC_MAX_MDCT_SIZE) {
        /* Allocate EXACTLY this rate's window (not MAX_MDCT): 96k needs 960 floats
         * (3.8 KB), not 1920 (7.7 KB). Reallocate on a size change. */
        if (s_imdct_window) { free(s_imdct_window); s_imdct_window = NULL; }
        s_imdct_window = (float *)malloc(sizeof(float) * (size_t)mdct_size);
        if (!s_imdct_window) { s_imdct_window_size = 0; return NULL; }
        lhdc_gen_kbd_window(s_imdct_window, mdct_size);
        s_imdct_window_size = mdct_size;
    }
    return s_imdct_window;
}

/* ---------------------- structured synthesis window -----------------------
 *
 * PROFILED, not assumed: at 192k the post-IMDCT stage (window + overlap-add)
 * cost 326 us/channel -- ~81 cycles per sample for what is one fused
 * multiply-add and one multiply. Two things were wrong, both fixed here.
 *
 * 1. The window lived in FLASH. lhdc_get_window_const() returns a `const float
 *    []`, i.e. .rodata, and overlap_add streamed all 1920 of them every frame,
 *    per channel: 3.1 MB/s of flash reads whose cache lines are being evicted
 *    by the BT controller's IRQs. This is the exact failure the IMDCT tables
 *    were moved to DRAM for (see lhdc_imdct.c: 364us -> 3800us/ch in flash);
 *    the window was simply left behind.
 *
 * 2. The window is LOW-OVERLAP, so 63% of it is exactly 0.0 or exactly 1.0 and
 *    needs no multiply at all. Measured over all three tables:
 *
 *      w[0     .. z)      = 0        -> output is just the overlap tail
 *      w[z     .. z+T)    = ramp     -> the only region needing a multiply
 *      w[z+T   .. h+z)    = 1        -> plain add / plain copy
 *      w[h+z   .. h+z+T)  = reverse(ramp)
 *      w[h+z+T .. N)      = 0        -> stores zero
 *
 *    with h = N/2, z = (h - T)/2, and w[N-1-n] == w[n] BIT-EXACTLY (verified:
 *    max|w[n]-w[N-1-n]| == 0.0 for N = 480/960/1920). So the down-ramp is the
 *    up-ramp reversed and a single T-entry table serves both halves.
 *
 * Result: the only table that has to be resident is the ramp -- 352 floats
 * (1408 B) at 192k instead of 7680 B -- and it goes in DRAM, so overlap_add
 * makes ZERO flash accesses. Arithmetic is unchanged: every output is still
 * mdct_out[n]*window[n] (+ overlap_buf[n]) with the identical window value.
 *
 * The structure is VERIFIED at runtime in lhdc_dec_window_prepare(). If a
 * window ever fails the check, s_win.ramp stays NULL and overlap_add falls
 * back to the original general loop, so a table change can never corrupt
 * audio -- it only costs speed. */
typedef struct {
    int    n;        /* mdct_size this was built for (0 = not built) */
    int    h;        /* n / 2 */
    int    z;        /* leading zero run */
    int    t;        /* ramp length */
    float *ramp;     /* DRAM copy of w[z .. z+t), or NULL -> generic path */
} lhdc_win_desc_t;

static lhdc_win_desc_t s_win = { 0, 0, 0, 0, NULL };

static void lhdc_dec_window_free_desc(void)
{
    if (s_win.ramp) { free(s_win.ramp); s_win.ramp = NULL; }
    s_win.n = s_win.h = s_win.z = s_win.t = 0;
}

/* Build the structured descriptor for `mdct_size`. Cheap (runs once per rate
 * change) and self-checking: any deviation from the expected shape leaves
 * s_win.ramp NULL, which selects the generic loop. */
static void lhdc_dec_window_prepare(int mdct_size)
{
    if (s_win.n == mdct_size) return;          /* already built for this rate */
    lhdc_dec_window_free_desc();

    const float *w = lhdc_dec_get_window(mdct_size);
    if (!w || mdct_size < 4 || (mdct_size & 1)) return;

    const int h = mdct_size / 2;
    int z = 0, t = 0, i;

    while (z < mdct_size && w[z] == 0.0f) z++;
    if (z <= 0 || z >= h) return;
    while (z + t < mdct_size && w[z + t] != 1.0f) t++;
    if (t <= 0 || z + t > h) return;
    if (z != (h - t) / 2) return;              /* z == (h - T)/2 */

    for (i = z + t; i < h + z; i++)            /* flat unity run */
        if (w[i] != 1.0f) return;
    for (i = 0; i < t; i++)                    /* down-ramp mirrors the up-ramp */
        if (w[h + z + i] != w[z + t - 1 - i]) return;
    for (i = h + z + t; i < mdct_size; i++)    /* trailing zeros */
        if (w[i] != 0.0f) return;

    float *ramp = (float *)LHDC_DEC_MALLOC(sizeof(float) * (size_t)t);
    if (!ramp) return;                         /* out of memory -> generic path */
    memcpy(ramp, w + z, sizeof(float) * (size_t)t);

    s_win.n = mdct_size;
    s_win.h = h;
    s_win.z = z;
    s_win.t = t;
    s_win.ramp = ramp;
}

/* Release the KBD analysis window. Call at decoder teardown so it doesn't
 * linger in DRAM after LHDC stops. */
void lhdc_dec_free_window(void)
{
    lhdc_dec_window_free_desc();
    if (s_imdct_window) { free(s_imdct_window); s_imdct_window = NULL; s_imdct_window_size = 0; }
}
/* perm[k] = source byte index that lands at position k after descrambling. */
static const uint8_t LHDC_PERM[2][8] = {
    { 4, 0, 1, 5, 7, 3, 2, 6 },
    { 6, 3, 7, 0, 2, 1, 4, 5 },
};

/* Descramble one channel's leading 8 bytes IN PLACE. The encoder scrambles
 * EACH channel's header independently (lhdc_v5_enc_encode @0xd2598 scrambles
 * regions 6 and 7 separately), so every channel section — not just the first —
 * needs this before its bit reader runs. */
static void lhdc_descramble_inplace_sel(uint8_t *buf, size_t avail, int sel)
{
    if (avail < 9) return;
    uint8_t tmp[8];
    sel &= 1;
    const uint8_t *perm = LHDC_PERM[sel];
    for (int i = 0; i < 8; i++) tmp[perm[i]] = buf[i];
    for (int k = 0; k < 8; k++) buf[k] = tmp[k] ^ LHDC_XOR_MASK[k];
}

static void lhdc_descramble_inplace(uint8_t *buf, size_t avail)
{
    if (avail < 9) return;
    lhdc_descramble_inplace_sel(buf, avail, buf[8] & 1);
}

__attribute__((unused))
static void lhdc_descramble(const uint8_t *in, uint8_t *out, size_t len)
{
    if (len < 9) {
        memcpy(out, in, len);
        return;
    }
    int sel = in[8] & 1;
    const uint8_t *perm = LHDC_PERM[sel];
    for (int i = 0; i < 8; i++) {
        out[perm[i]] = in[i];
    }
    for (int k = 0; k < 8; k++) {
        out[k] ^= LHDC_XOR_MASK[k];
    }
    memcpy(out + 8, in + 8, len - 8);
}

/*
 * Parse one LHDC V5 frame (PROJECT_CONTEXT §5/§6, validated).
 *
 * A frame is [u16 LE header][payload]:
 *   payload_len = (hdr & 0x3FF) * 2
 *   flags       = hdr >> 10   (=1 for 48k/5ms stereo)
 * There is no per-frame sync word or codec-param header; the stream format
 * (sample rate, channels, bit depth, frame duration) comes from the A2DP CIE
 * captured in dec->config. The payload's first 9 bytes are descrambled into
 * dec->payload_buf, on which the per-channel bit reader then operates.
 *
 * Returns the number of input bytes consumed (header + payload) via *consumed.
 */
static lhdc_dec_ret_t lhdc_dec_parse_header(lhdc_decoder_t *dec,
                                            const uint8_t *in_data,
                                            size_t in_bytes,
                                            size_t *consumed)
{
    lhdc_dec_frame_header_t *hdr = &dec->header;

    memset(hdr, 0, sizeof(*hdr));
    *consumed = 0;

    if (in_bytes < 2) {
        return LHDC_DEC_NEED_MORE_DATA;
    }

    uint16_t fhdr = (uint16_t)(in_data[0] | (in_data[1] << 8));   /* LE */
    size_t payload_len = (size_t)(fhdr & 0x3FF) * 2;
    uint16_t flags = (uint16_t)(fhdr >> 10);

    if (payload_len == 0 || payload_len > LHDC_DEC_MAX_FRAME_BYTES) {
        return LHDC_DEC_BITSTREAM_ERROR;
    }
    if (in_bytes < 2 + payload_len) {
        return LHDC_DEC_NEED_MORE_DATA;
    }

    /* Stream params come from the configured CIE, not the frame. */
    hdr->sample_rate       = dec->config.sample_rate;
    hdr->channels          = dec->config.channels ? dec->config.channels : 2;
    hdr->bit_depth         = dec->config.bit_depth;
    hdr->frame_duration_ms = dec->config.frame_duration;
    hdr->version           = LHDC_DEC_VERSION_1;
    hdr->frame_flags       = flags;
    hdr->frame_bytes       = (uint16_t)payload_len;

    /*
     * New PCM samples emitted per frame = the MDCT hop = mdct_size/2 (the
     * overlap, see lhdc_dec_overlap_add). The time-based spf
     * (sample_rate*dur/1000) happens to equal mdct_size/2 for 48/96/192k, but
     * at 44.1k it gives 220 instead of 240 — so 20 of the 240 overlap-added
     * samples per frame were silently dropped, compressing the audio and
     * raising pitch by exactly 48000/44100 = 8.8%. Derive the emit count from
     * the band config's MDCT size so every rate plays at the right speed.
     */
    const lhdc_band_cfg_desc_t *bcfg = lhdc_get_band_cfg(
        hdr->sample_rate, hdr->frame_duration_ms);
    hdr->band_cfg_idx = (uint8_t)bcfg->cfg_idx;
    hdr->samples_per_channel = (uint16_t)(bcfg->mdct_size / 2);

    /* Copy the raw payload into the scratch buffer. Each channel's 8-byte header
     * is descrambled later in decode_channel (uniformly, so the selector can be
     * retried per channel). */
    memcpy(dec->payload_buf, in_data + 2, payload_len);

    /* The per-channel bit reader runs over the descrambled payload. (Runs on the
     * calling task before the parallel region; each channel re-inits its own
     * slot's reader over its byte slice at the top of decode_channel.) */
    lhdc_bit_reader_init(&dec->slot[LHDC_DEC_SLOT()].bit_reader, dec->payload_buf, payload_len);

#if defined(LHDC_HOST_BUILD)
    extern volatile int g_lhdc_trace;
    if (g_lhdc_trace > 0) {
        printf("[RAWPAY] first 20 raw (pre-descramble) bytes:");
        for (int b = 0; b < 20; b++) printf(" %02x", in_data[2 + b]);
        printf("\n");
        printf("[PAYLOAD] len=%zu  byte:flag(msb) for 0,118..130:\n", payload_len);
        printf("  b0 msb=%d\n", (dec->payload_buf[0] >> 7) & 1);
        for (int b = 118; b <= 130 && b < (int)payload_len; b++)
            printf("  b%d=0x%02x msb=%d gain9=%d\n", b, dec->payload_buf[b],
                   (dec->payload_buf[b] >> 7) & 1,
                   ((dec->payload_buf[b] << 1) | (dec->payload_buf[b+1] >> 7)) & 0x1FF);
    }
#endif

    *consumed = 2 + payload_len;
    return LHDC_DEC_OK;
}

/*
 * Global-gain exponent e(gain_idx), reversed from GenerateGainTable /
 * EncGainByTable (PROJECT_CONTEXT §11). The encoder quantizes the whole
 * SNS-shaped spectrum by q[k] = round(spectrum[k] * 2^-e(gidx)); the decoder
 * dequantizes linearly: spectrum[k] = q[k] * 2^e(gidx).
 *
 * e(idx) is piecewise in the 512-entry gain table:
 *   idx <= split        : e = idx
 *   split < idx <= 503  : e = split + (idx-split)*slope
 *   idx > 503           : e = e(503) + (idx-504), clamped to 30
 * (split, slope) depend on frame length; for 48k/16-bit:
 *   split = (int)(((L-50)*(-0.017144) + 26.18) / 2.5)
 *   slope = (e_full - split) / (503 - split)   with e_full ~ 30
 */
static float lhdc_dec_gain_exponent(int gain_idx, int enc_frame_len, int bit_depth,
                                    uint32_t sample_rate)
{
    int L = enc_frame_len;
#if defined(LHDC_HOST_BUILD)
    { const char *ld = getenv("LHDC_LDIV"); if (ld) { int d = atoi(ld); if (d > 0) L = enc_frame_len / d; } }
#endif
    const int OFFSET_MAX = 504;   /* (1<<9) - 8 */
    float xf = (float)(L - 50);
    float divisor;
    float offset_max = 30.0f;      /* reference default when no rate case matches */
    if (bit_depth == 24) {
        divisor = 4.0f;
        if (sample_rate == 96000)       offset_max = 25.7f - (25.7f - 20.75f) / 575.0f * xf;
        else if (sample_rate == 192000) offset_max = 25.7f - (25.7f - 24.42f) / 575.0f * xf;
        else if (sample_rate <= 48000)  offset_max = 26.2f - (26.2f  - 16.4f)  / 575.0f * xf; /* <=48k */
        /* else: 30.0 default */
    } else {
        divisor = 2.5f;
        if (sample_rate == 48000)       offset_max = 26.18f - (26.18f - 16.32f) / 575.0f * xf;
        else if (sample_rate <= 44100)  offset_max = 26.15f - (26.15f - 16.42f) / 575.0f * xf; /* <=44.1k */
        /* else (96k/192k 16-bit): 30.0 default */
    }
    int start = (int)(offset_max / divisor);          /* trunc toward zero */
#if defined(LHDC_HOST_BUILD)
    { const char *sp = getenv("LHDC_SPLIT"); if (sp) start = atoi(sp); }
#endif
    int deno = OFFSET_MAX - start - 1;                 /* = 503 - start */
    float jump = (deno != 0) ? (offset_max - (float)start) / (float)deno : 0.0f;
#if defined(LHDC_HOST_BUILD)
    { const char *sl = getenv("LHDC_SLOPE"); if (sl) jump = (float)atof(sl); }
#endif

    float e;
    if (gain_idx <= start) {
        e = (float)gain_idx;
    } else {
        int lo = (gain_idx < OFFSET_MAX) ? gain_idx : OFFSET_MAX;
        int hi = (gain_idx > OFFSET_MAX) ? gain_idx : OFFSET_MAX;
        e = (float)(lo - start) * jump + (float)start + (float)(hi - OFFSET_MAX);
        if (e > 30.0f) e = 30.0f;
    }
    return e;
}

/* Per-rate output level normalization. 44.1k/48k (mdct_size 480) decode at unity,
 * but the reconstruction gain falls ~1/N^2 with the transform size N, so 96k
 * (mdct 960) came out ~12 dB quieter and 192k (mdct 1920) ~24 dB quieter than
 * 48k -- the volume DROPPED when switching to a higher sample rate. The old
 * LHDC_96K_GAIN=1.0 hack never corrected it. Restore a consistent level with
 * (mdct_size/480)^2 (480->x1, 960->x4, 1920->x16). Verified via the real-encoder
 * round-trip: brings 96k/192k to within <0.5 dB of 48k. */
static float lhdc_dec_rate_norm(int mdct_size)
{
    float r = (float)mdct_size / 480.0f;
#if defined(LHDC_HOST_BUILD)
    { const char *e = getenv("LHDC_RATEPOW"); if (e) return powf(r, (float)atof(e)); }
#endif
    return r;
}

/* Per-config output level calibration toward VALUE-PRESERVING (0 dBFS in ->
 * 0 dBFS out) so every LHDC config — and thus every codec, since the standard
 * decoders are already value-preserving — plays at the same volume regardless
 * of sample rate / bit depth. Residual decode gain (after rate_norm) measured
 * via near-transparent multitone round-trip through the real encoder
 * (tools/lhdc_roundtrip busy=1 @900k, broadband RMS): 24-bit 48k +0.2, 96k +1.3,
 * 44.1k +1.3, 192k -0.8 dB; 16-bit ~+5.6 dB (its gain table was never calibrated
 * — the slope/split fixes were 24-bit only). These trims cancel it. NOTE:
 * approximate (signal-dependent within ~1-2 dB); host env LHDC_LEVELDB overrides
 * for re-calibration by ear. */
static float lhdc_dec_level_cal(uint32_t sample_rate, int bit_depth)
{
    float db = 0.0f;
    if (bit_depth >= 24) {
        switch (sample_rate) {
            case 44100:  db = -1.32f; break;
            case 48000:  db = -0.21f; break;
            case 96000:  db = -1.29f; break;
            case 192000: db = +0.79f; break;
            default:     db = 0.0f;   break;
        }
    } else {
        db = -5.60f;   /* 16-bit (44.1/48k) */
    }
#if defined(LHDC_HOST_BUILD)
    { const char *e = getenv("LHDC_LEVELDB"); if (e) db = (float)atof(e); }
#endif
    return exp2f(db / 6.020599913f);   /* 10^(db/20) */
}

/*
 * Inverse quantization (PROJECT_CONTEXT §11): linear dequant by the global gain
 * step. spectrum[k] = q[k] * 2^e(gain_idx). The SNS per-band shaping is undone
 * separately in lhdc_sns_synth_apply.
 */
static LHDC_HOT void lhdc_dec_inverse_quantize(int32_t *quant, float *spectrum,
                                       int num_coeffs,
                                       const lhdc_dec_sns_params_t *sns,
                                       int enc_frame_len, int bit_depth,
                                       uint32_t sample_rate)
{
    /* step = 2^e with FRACTIONAL e -> exp2f (single precision), NOT ldexpf
     * (which would truncate the fractional exponent and shift the level). */
    float step = exp2f(lhdc_dec_gain_exponent(
                           sns->global_gain, enc_frame_len, bit_depth, sample_rate));
    for (int k = 0; k < num_coeffs; k++) {
        spectrum[k] = (float)quant[k] * step;
    }
}

/*
 * Apply overlap-add for smooth frame transitions.
 * Uses 50% overlap with the previous frame.
 */
static LHDC_HOT void lhdc_dec_overlap_add(lhdc_decoder_t *dec, float *pcm_out,
                                  int channel, int mdct_size)
{
    int overlap = mdct_size / 2;
    /* Only the generic path needs the full window; fetching it here would run
     * lhdc_dec_get_window()'s lookup every frame for nothing on the fast path. */
    const float *window = (s_win.ramp && s_win.n == mdct_size)
                        ? NULL : lhdc_dec_get_window(mdct_size);
    float *overlap_buf = dec->overlap_buf[channel];   /* per-channel: no slot needed */
    float *mdct_out = dec->slot[LHDC_DEC_SLOT()].mdct_out;

    /* Windowing is fused into the two halves instead of being a separate pass
     * over all `mdct_size` samples. The old shape read+scaled+stored the whole
     * IMDCT output (1920 loads + 1920 stores at 192k) and then read it back
     * again; folding window[] into each consumer removes that entire round trip
     * and lets the first half become a single fused multiply-add. Identical
     * arithmetic: each output is still mdct_out[n]*window[n] (+ overlap_buf[n]).
     *
     * Note mdct_out[] is no longer left windowed in place -- nothing downstream
     * reads it (the caller consumes pcm_out, and the next frame overwrites
     * mdct_out from the IMDCT). */

    /* Structured fast path: skip the 63% of the window that is exactly 0 or 1
     * (see lhdc_dec_window_prepare). Same arithmetic, but multiplies happen
     * only across the ramp, and the ramp is in DRAM so this touches no flash.
     * s_win.ramp == NULL (unrecognized window / out of memory) -> generic loop
     * below, which is what always ran before. */
    if (s_win.ramp && s_win.n == mdct_size) {
        const float *const ramp = s_win.ramp;
        const int z = s_win.z, t = s_win.t;

        /* --- first half: pcm_out[n] = mdct_out[n]*w[n] + overlap_buf[n] --- */
        /* w = 0 on [0,z): the IMDCT contributes nothing, emit the tail as-is. */
        memcpy(pcm_out, overlap_buf, sizeof(float) * (size_t)z);
        /* w = ramp on [z, z+t) */
        for (int k = 0; k < t; k++) {
            const int n = z + k;
            pcm_out[n] = mdct_out[n] * ramp[k] + overlap_buf[n];
        }
        /* w = 1 on [z+t, overlap): plain add, no multiply. */
        for (int n = z + t; n < overlap; n++) {
            pcm_out[n] = mdct_out[n] + overlap_buf[n];
        }

        /* --- second half: overlap_buf[n] = mdct_out[overlap+n]*w[overlap+n] --- */
        /* w = 1 on window[overlap .. overlap+z): straight copy. */
        memcpy(overlap_buf, mdct_out + overlap, sizeof(float) * (size_t)z);
        /* Down-ramp: w[overlap+z+k] == ramp[t-1-k] (verified bit-exact). */
        for (int k = 0; k < t; k++) {
            overlap_buf[z + k] = mdct_out[overlap + z + k] * ramp[t - 1 - k];
        }
        /* w = 0 to the end: this frame contributes no tail there. */
        memset(overlap_buf + z + t, 0, sizeof(float) * (size_t)(overlap - z - t));
        return;
    }

    if (!window) {                       /* no window available -> silence (safe) */
        memset(pcm_out, 0, sizeof(float) * (size_t)overlap);
        memset(overlap_buf, 0, sizeof(float) * (size_t)overlap);
        return;
    }

    /* First half: window + overlap-add with the previous frame's tail. Only
     * `overlap` (= mdct_size/2 = samples_per_channel) samples are emitted, so
     * pcm_out is exactly that long (= ch_pcm = w_buf, half-size). */
    for (int n = 0; n < overlap; n++) {
        pcm_out[n] = mdct_out[n] * window[n] + overlap_buf[n];
    }

    /* Second half: window and save as next frame's overlap. (The old code also
     * wrote it to pcm_out[overlap..mdct_size-1], but that range is never emitted
     * -- the caller reads only pcm_out[0..overlap-1] -- so those were dead
     * writes that forced w_buf to be full mdct_size. Dropped -> w_buf halves.) */
    for (int n = 0; n < overlap; n++) {
        overlap_buf[n] = mdct_out[overlap + n] * window[overlap + n];
    }
}

/*
 * FAC moving-average window 1 (stream-2 window = win1*8). Validated 24/24
 * across configs: win1 = payload/2 for high-bitrate frames (resid field == 4,
 * i.e. enc_frame_len >= 160), else payload/4. `enc_frame_len` here is the
 * per-frame payload byte count (hdr->frame_bytes).
 */
static int lhdc_dec_ma_window(uint32_t sample_rate, int enc_frame_len)
{
    (void)sample_rate;
    int payload = enc_frame_len;
    int win1 = (enc_frame_len >= 160) ? (payload / 2) : (payload / 4);
    if (win1 < 1) win1 = 1;
    return win1;
}

/* number of bits to represent n (encoder's calc_bits). Uses unsigned and a
 * hard cap of 31 so a large/garbage magnitude can't spin forever (1<<31 wraps
 * on ARM and made the naive loop never terminate).
 * Hot path: called per-coefficient in the mantissa plane. The old while-loop
 * counted up to 31 times; Xtensa LX6 has a 1-cycle count-leading-zeros (NSAU),
 * exposed as __builtin_clz, so compute the bit width directly. Bit-identical to
 * the loop: calc_bits(u) = (u==0)?0 : floor(log2(u))+1, capped at 31. `inline`
 * so it folds into the IRAM mantissa loop instead of being a flash call. */
static inline int lhdc_dec_calc_bits(int n)
{
    unsigned u = (n < 0) ? (unsigned)(-(long)n) : (unsigned)n;
    if (u == 0) return 0;
    int b = 32 - __builtin_clz(u);
    return (b > 31) ? 31 : b;
}

/*
 * Decode the mantissa+sign bit-plane for one channel into out[0..cnt-1] from the
 * post-Rice quotients coeff[0..cnt-1].
 *   M[k]  = (coeff[k] << shift[k]) | mantissa,  shift[k] from causal IIR predictor
 *   sign  = 1 extra bit when M != 0
 * `start_bit` is the plane start (relative to the channel byte slice). Returns the
 * peak |M| so the caller can pick the correct plane offset (the range coder leaves
 * a data-dependent 2- or 3-byte lookahead gap; the wrong start desyncs the predictor
 * and explodes |M|, so min-peak reliably selects the right offset). pred_mode = flag2.
 */
static LHDC_HOT int64_t lhdc_dec_mant_plane(const int32_t *coeff, int32_t *out, int cnt,
                                   const uint8_t *pl, int total_bits,
                                   int start_bit, int pred_mode, int init_shift)
{
    int p = start_bit;
    /* The mantissa shift predictor is seeded by the transmitted initial shift
     * (the 4-bit "resid" field), NOT 0. shift[0] = (pred+0x40)>>7, and the
     * first iteration leaves pred unchanged ((a+b)*pred>>3 == pred), so seeding
     * pred = init_shift<<7 yields shift[0] = init_shift = calc_bits(|M[0]|).
     * Without this, dense frames whose first coeff has a large magnitude (and a
     * zero Rice quotient) lose all mantissa bits -> spectrum desync/garble. */
    int pred = init_shift << 7;
    int64_t peak = 0;
    for (int k = 0; k < cnt; k++) {
        int s = (pred + 0x40) >> 7;
        /* Read s mantissa bits MSB-first. The ARM-style per-bit loop did s byte
         * loads + shifts; on Xtensa we batch them into ONE shifted 4-byte load
         * when the 4 bytes are in-bounds (the common mid-buffer case): the field
         * is w bits (31-off)..(32-off-s), so (w<<off)>>(32-s) right-aligns it.
         * Guard (byte+4)*8<=total_bits ensures all 32 bits are valid (no
         * zero-fill) and s<=24 keeps off+s<=31. Bit-identical to the loop
         * (verified 2,000,000 random cases). Boundary/large-s -> per-bit fallback. */
        int mant;
        {
            int byte = p >> 3, off = p & 7;
            if (s == 0) {
                mant = 0;
            } else if (s <= 24 && ((byte + 4) << 3) <= total_bits) {
                uint32_t w = ((uint32_t)pl[byte] << 24) | ((uint32_t)pl[byte + 1] << 16) |
                             ((uint32_t)pl[byte + 2] << 8) | (uint32_t)pl[byte + 3];
                mant = (int)((w << off) >> (32 - s));
                p += s;
            } else {
                mant = 0;
                for (int b = 0; b < s; b++) {
                    int bit = (p < total_bits) ? ((pl[p >> 3] >> (7 - (p & 7))) & 1) : 0;
                    p++;
                    mant = (mant << 1) | bit;
                }
            }
        }
        int mag = ((int)coeff[k] << s) | mant;
        int val = mag;
        if (mag != 0) {
            int neg = (p < total_bits) ? ((pl[p >> 3] >> (7 - (p & 7))) & 1) : 0;
            p++;
            if (neg) val = -mag;
        }
        out[k] = val;
        { int64_t a = mag < 0 ? -(int64_t)mag : mag; if (a > peak) peak = a; }
        int x = lhdc_dec_calc_bits(mag) << 7;
#if defined(LHDC_HOST_BUILD)
        { extern volatile int g_lhdc_trace; if (g_lhdc_trace > 0 && k < 24)
            printf("[MANT] k=%d coeff=%d s=%d mant=%d mag=%d pred=%d p=%d\n", k, (int)coeff[k], s, mant, mag, pred, p); }
#endif
        pred = pred_mode ? ((7 * pred + x) >> 3) : ((5 * x + 3 * pred) >> 3);
    }
    return peak;
}

/*
 * Decode a single channel.
 */
/* force_sel: -1 = auto (descramble selector = header byte[8]&1, save original
 * header bytes for a possible retry); 0/1 = retry with this forced selector
 * (restores the saved original header bytes first). The byte[8]&1 selector source
 * is reverse-engineered and is wrong on a minority of frames, desyncing that
 * channel's FAC into huge |M| (the 900k dense-channel garble). The caller re-runs
 * the channel with the flipped selector when the decode explodes and keeps the
 * in-range result. g_lhdc_chan_maxM exposes this channel's peak |M| for that check. */
float g_lhdc_chan_specmax = 0;   /* peak of post-SNS spectrum; desync detector */
static LHDC_HOT lhdc_dec_ret_t lhdc_dec_decode_channel(lhdc_decoder_t *dec,
                                                int channel,
                                                float *pcm_out,
                                                int force_sel)
{
    /* Per-core scratch: with the two channels decoding concurrently (one per
     * core) each decode must use its own working buffers. Single-slot targets
     * resolve this to &dec->slot[0] and behave exactly as before. */
    lhdc_dec_slot_t *slt = &dec->slot[LHDC_DEC_SLOT()];
    lhdc_dec_bit_reader_t *br = &slt->bit_reader;
    lhdc_dec_frame_header_t *hdr = &dec->header;
    const lhdc_band_cfg_desc_t *band_cfg = dec->band_cfg;

    int mdct_size = band_cfg->mdct_size;
    int num_sfb = band_cfg->num_sfb;
    int num_coeffs = mdct_size / 2;
    int enc_frame_len = hdr->frame_bytes;
    int64_t t_chan0 = LHDC_NOW_US();
    int64_t t_ent0 = 0, t_ent1 = 0, t_im0 = 0, t_im1 = 0;
    /* Sub-marks that split the profiler's catch-all "other" bucket:
     *   pre   = t_ent0 - t_chan0        header + band cfg + SNS side info
     *   mant  = t_mant1 - t_ent1        mantissa-plane decode
     *   dq    = t_im0 - t_mant1         inverse quantize + SNS apply
     *   post  = end - t_im1             window + overlap-add + PCM out */
    int64_t t_mant1 = 0;

    /*
     * Each channel occupies a FIXED, equal slice of the payload:
     * ch_bytes = frame_bytes / channels (the encoder writes two independent
     * byte-aligned channel buffers of frame_bytes/channels each — verified:
     * 125+125 for a 250-byte 2ch frame). Channel `channel` starts at
     * ch_start = channel*ch_bytes. Position the bit reader there and confine the
     * channel's mantissa plane to its own slice (pl = payload+ch_start). This
     * replaces the old (wrong) "decode ch0, probe for ch1 byte boundary" logic. */
    int n_ch = (hdr->channels < 1) ? 1 : hdr->channels;
    int ch0_bytes = enc_frame_len / n_ch;
    int ch_start = channel * ch0_bytes;
    int ch_bytes = (channel + 1 < n_ch) ? ch0_bytes : (enc_frame_len - ch_start);
#if defined(LHDC_HOST_BUILD)
    /* Override ch1's start byte to probe the real (possibly unequal) split. */
    if (channel == 1) { const char *e = getenv("LHDC_CH1_START");
        if (e) { ch_start = atoi(e); ch_bytes = enc_frame_len - ch_start; } }
#endif
    /* Each channel's 8-byte header is scrambled independently. parse_header now
     * only COPIES the payload (no descramble), so every channel descrambles its
     * own header here — uniformly, which lets the caller retry the selector. */
    {
        /* Per-slot, not function-static: with both channels decoding at once the
         * two cores would otherwise race on one saved-header buffer. */
        uint8_t *cb = dec->payload_buf + ch_start;
        if (force_sel < 0) {
            if (ch_bytes >= 9) { for (int i = 0; i < 8; i++) slt->hdr_save[i] = cb[i]; slt->hdr_saved_start = ch_start; }
            lhdc_descramble_inplace(cb, (size_t)ch_bytes);              /* sel = cb[8]&1 */
        } else {
            if (slt->hdr_saved_start == ch_start) { for (int i = 0; i < 8; i++) cb[i] = slt->hdr_save[i]; }
            lhdc_descramble_inplace_sel(cb, (size_t)ch_bytes, force_sel);
        }
    }
    lhdc_bit_reader_init(br, dec->payload_buf + ch_start, (size_t)ch_bytes);

    /*
     * Per-channel leading section (PROJECT_CONTEXT §7, validated):
     *   [1] flag (==0)
     *   [9] global gain index
     *   [4] sns_mode-8
     *   [num_sfb-1] SNS dir bits     <- consumed inside lhdc_sns_decode_params
     *   [7] (nzc>>1)-1               <- ECB[0x18] = calc_bits(samples)-1 = 7
     *   [1] flag2
     *   [4] residual field           <- ECB[0x1c] = 4 if frame_len>=160 else 0
     * then the FAC spectral stream.
     */
    int flag = (int)lhdc_bit_reader_read(br, 1);
    if (flag != 0) {
        if (g_lhdc_trace > 0)
            ESP_LOGW("LHDCV5_DEC", "ch%d: flag!=0 (flag=%d) frame_bytes=%d mdct=%d sfb=%d",
                     channel, flag, enc_frame_len, mdct_size, num_sfb);
        return LHDC_DEC_BITSTREAM_ERROR;
    }
    slt->sns_params.global_gain = (int32_t)lhdc_bit_reader_read(br, 9);

    /* SNS: 4-bit sns_mode + (num_sfb-1) dir bits -> per-band scale factors. */
    lhdc_dec_ret_t ret = lhdc_sns_decode_params(br, &slt->sns_params, num_sfb);
    if (ret != LHDC_DEC_OK) {
        ESP_LOGW("LHDCV5_DEC", "ch%d: sns_decode ret=%d gain=%d sfb=%d",
                 channel, ret, (int)slt->sns_params.global_gain, num_sfb);
        return ret;
    }
    if (g_lhdc_trace > 0) {
        ESP_LOGI("LHDCV5_DEC", "ch%d leading ok: gain=%d sns_mode=%d num_sfb=%d frame_bytes=%d",
                 channel, (int)slt->sns_params.global_gain,
                 (int)slt->sns_params.sns_mode, num_sfb, enc_frame_len);
    }

    /* nzc field width = calc_bits(samples_per_frame) - 1
     * (7 for spf=240, 8 for 480, 9 for 960). */
    int nzc_bits = lhdc_dec_calc_bits(hdr->samples_per_channel) - 1;
    if (nzc_bits < 1) nzc_bits = 7;
    int nzc_raw = (int)lhdc_bit_reader_read(br, nzc_bits);
    int nzc = (nzc_raw + 1) * 2;
#if defined(LHDC_HOST_BUILD)
    { extern int g_nzc_formula; if (g_nzc_formula == 1) nzc = nzc_raw * 2; }
#endif
    if (g_lhdc_trace > 0 && channel == 0)
        printf("[I]LHDCV5_DEC: nzc_bits=%d nzc_raw=%d nzc=%d (clamp %d)\n",
               nzc_bits, nzc_raw, nzc, num_coeffs);
    if (nzc > num_coeffs) nzc = num_coeffs;
    slt->sns_params.nzc = nzc;

    /* flag2 selects the mantissa bit-allocation predictor: 0 = fast IIR
     * A[k]=(5*x+3*A[k-1])>>3, 1 = slow IIR B[k]=(7*B[k-1]+x)>>3. (Verified:
     * flag2 == the encoder's predictor-selection field for 500/1k/2k/5k/10k Hz.) */
    int pred_mode = (int)lhdc_bit_reader_read(br, 1);   /* flag2 */
    /* resid_bits = ECB[0x1c], read 4 bits between flag2 and the FAC stream.
     * GROUND TRUTH (bcfg_probe over all 24 configs, ecb+0x1c): resid=4 for
     *   44.1k br6,7,8 (enc_frame_len 326,612,680), 48k/96k br7,8 (562,624);
     *   resid=0 for everything <=312 (48k/96k br6, 44.1k br5).
     * The break is between 312 (resid=0) and 326 (resid=4), so threshold 320.
     * The old >=400 wrongly gave resid=0 for 44.1k br6 (326), desyncing it. */
    int resid_bits = (enc_frame_len >= 320) ? 4 : 0;
#if defined(LHDC_HOST_BUILD)
    { const char *e = getenv("LHDC_RESID"); if (e) resid_bits = atoi(e); }
#endif
    int resid_val = 0;
    if (resid_bits) resid_val = (int)lhdc_bit_reader_read(br, resid_bits);
#if defined(LHDC_HOST_BUILD)
    if (g_lhdc_trace > 0)
        printf("[RESID] ch=%d resid_bits=%d resid_val=%d\n", channel, resid_bits, resid_val);
#endif
#if defined(LHDC_HOST_BUILD)
    { const char *e = getenv("LHDC_FACBIT");
      if (e) { int fb = atoi(e);
               lhdc_bit_reader_init(br, dec->payload_buf + ch_start, (size_t)ch_bytes);
               lhdc_bit_reader_skip(br, fb); } }
#endif

    /*
     * FAC entropy stream: snapshot the remaining payload as a byte buffer (the
     * range coder is byte-oriented and bit-packed from here, MSB-first), decode
     * the two ternary streams, and Rice-decode into |quant coeff|. Then the
     * sign plane (1 bit per nonzero coeff) follows at fac_start + fac_bytes.
     */
    int fac_start_bit = (int)lhdc_bit_reader_tell(br);
#if defined(LHDC_HOST_BUILD)
    if (g_lhdc_trace > 0) {
        printf("[CHBYTES] ch=%d fac_start_bit=%d ch_start=%d ch_bytes=%d nzc=%d bytes:",
               channel, fac_start_bit, ch_start, ch_bytes, nzc);
        for (int b = 0; b < ch_bytes && b < 281; b++)
            printf(" %02x", dec->payload_buf[ch_start + b]);
        printf("\n");
        /* full payload + metadata for python replay of the real algorithm */
        char chpath[64]; snprintf(chpath, sizeof(chpath), "/data/local/tmp/lhdccal/chpay_ch%d.txt", channel);
        FILE *pf = fopen(chpath, "w");
        if (pf) {
            fprintf(pf, "fac_start_bit %d\nch_start %d\nch_bytes %d\nnzc %d\nflag2 %d\nresid_bits %d\n",
                    fac_start_bit, ch_start, ch_bytes, nzc, pred_mode, resid_bits);
            fprintf(pf, "payload:");
            for (int b = 0; b < ch_bytes; b++) fprintf(pf, " %02x", dec->payload_buf[ch_start + b]);
            fprintf(pf, "\n");
            fclose(pf);
        }
    }
#endif
    /*
     * Rice split = bandcfg[0x20], which the encoder passes to RiceQuotientEncode
     * (verified at the call site BitsPredictCompressEval@0xd4878: split is loaded
     * from *(ECB+0x848)+0x20, the band-config struct, NOT ECB[0x20]). Dumping it
     * across configs gives split = spf/3 = num_coeffs/3 (240->80, 480->160,
     * 960->320), constant across bitrate and bit depth. It sets pivot =
     * max(count-split,0); coeffs beyond `pivot` switch to LSB-plane coding. The
     * old hardcoded split=0 (pivot=count) only matched frames with no |coeff|>=2
     * past index num_coeffs/3*2, so it passed the low-energy 24-config test but
     * misdecodes real high-frequency content into noise.
     */
    int split = num_coeffs / 3;
    /* FAC moving-average windows = band_cfg[0x28]/[0x2c] (57/63 @48k, 96/64 @96k).
     * NOTE: the ecb[0x28] "win1" (80/125/281 per bitrate, == frame_header&0x3FF on
     * the synthetic tone probe) is a DIFFERENT encoder parameter and is NOT the
     * decoder's FAC model window — using it (enc_frame_len/2) desynced every rate
     * (billions). The band_cfg constant is what the decoder's range model needs. */
    int ma_win1 = band_cfg->ma_win1;
    int ma_win2 = band_cfg->ma_win2;
    (void)lhdc_dec_ma_window;
#if defined(LHDC_HOST_BUILD)
    { const char *e = getenv("LHDC_MAWIN1"); if (e) { ma_win1 = atoi(e); ma_win2 = ma_win1 * 8; } }
    { const char *e = getenv("LHDC_MAWIN2"); if (e) ma_win2 = atoi(e); }
#endif
#if defined(LHDC_HOST_BUILD)
    if (g_lhdc_trace > 0 && channel == 0) {
        FILE *pf = fopen("/data/local/tmp/lhdccal/chpay2.txt", "w");
        if (pf) { fprintf(pf, "split %d\nma_win1 %d\nma_win2 %d\n", split, ma_win1, ma_win2); fclose(pf); }
    }
#endif
    int fac_bytes = 0;
    t_ent0 = LHDC_NOW_US();
    ret = lhdc_entropy_decode_spectrum_ex2(slt->quant_spectrum,
                                          num_coeffs, br, nzc, split,
                                          ma_win1, ma_win2, &fac_bytes,
                                          slt->fac_buf, (int)sizeof(slt->fac_buf),
                                          slt->ent_s1,
                                          slt->ent_s2, dec->alloc_mdct_size);  /* ent_s2 cap = num_coeffs*2 */
    t_ent1 = LHDC_NOW_US();
    if (ret != LHDC_DEC_OK) {
        return ret;
    }

    if (g_lhdc_trace > 0) {
        char buf[600]; int p = 0;
        p += snprintf(buf + p, sizeof(buf) - p, "ch%d COEFF(pre-sign)[0..47]:", channel);
        for (int k = 0; k < 48 && k < num_coeffs; k++)
            p += snprintf(buf + p, sizeof(buf) - p, " %d", (int)slt->quant_spectrum[k]);
        ESP_LOGI("LHDCV5_DEC", "%s", buf);
#if defined(LHDC_HOST_BUILD)
        /* dump full coeff list "idx val" to /tmp for reorder analysis */
        char cpath[64]; snprintf(cpath, sizeof(cpath), "/data/local/tmp/lhdccal/coeff_ch%d.txt", channel);
        FILE *cf = fopen(cpath, "w");
        if (cf) {
            for (int k = 0; k < num_coeffs; k++)
                fprintf(cf, "%d %d\n", k, (int)slt->quant_spectrum[k]);
            fclose(cf);
        }
#endif
    }

    /*
     * Mantissa + sign plane (immediately after the FAC byte stream at
     * fac_start + fac_bytes*8). The FAC stream only carried q[k] = the HIGH bits
     * of each quantized coeff. The full magnitude is reconstructed sequentially:
     *
     *   shift[k] = calc_bits(|M[k-1]|)         (running context; M[-1]=0)
     *   M[k]     = (q[k] << shift[k]) | mantissa  (mantissa = shift[k] raw bits)
     *   if M[k] != 0: read 1 sign bit (1 = negative)
     *
     * This is the encoder's per-coeff bit-plane (LhdcEncode @0xd1e80): it writes
     * shift[k] mantissa bits then a sign bit (when M[k]!=0), MSB-first. Without
     * this the residual at high-energy bins (e.g. a tone band, where q is shifted
     * to 0 but the magnitude lives entirely in the mantissa) is lost.
     */
    {
        /* The range coder leaves a data-dependent lookahead gap before the mantissa
         * plane: 2 or 3 bytes (16 or 24 bits, per lhdc_dec_arith_stop, keyed on the
         * final range). Each channel may differ. Decode the plane for BOTH candidate
         * starts and keep the one with the smaller peak |M|: the wrong start desyncs
         * the causal shift predictor and explodes |M| by orders of magnitude, so this
         * selects the correct offset per channel. */
        const uint8_t *pl = dec->payload_buf + ch_start;   /* this channel's byte slice */
        int total_bits = ch_bytes * 8;
        int32_t *qs = slt->quant_spectrum;
        int cnt = nzc; if (cnt > num_coeffs) cnt = num_coeffs;
        int32_t *coeff = (int32_t *)slt->mdct_in;   /* scratch (free until inverse_quantize): post-Rice quotients */
        for (int k = 0; k < cnt; k++) coeff[k] = qs[k];

        int forced = -1, sel = 1;   /* sel: 0=min-peak, 1=range-rule (default) */
        /* All rates use the exact range-rule mantissa-gap selection (the same
         * path that decodes 48k bit-clean). The earlier huge-|M| desyncs at 96k
         * were the wrong FAC model window (114/126); with the correct 96/64 the
         * range coder stays in sync, so the exact rule applies and the 3-pass
         * min-peak heuristic (extra CPU + occasional mispick) is unneeded. */
#if defined(LHDC_HOST_BUILD)
        { const char *e = getenv("LHDC_MANT_DELTA"); if (e) forced = atoi(e); }
        { const char *e = getenv(channel == 0 ? "LHDC_D0" : "LHDC_D1"); if (e) forced = atoi(e); }
        { const char *e = getenv("LHDC_SEL"); if (e) sel = atoi(e); }
#endif
        extern uint32_t g_fac_final_range[LHDC_NSLOTS];   /* per-slot: set by this core's entropy pass */
        int best_delta = 3;
        if (forced >= 0) {
            best_delta = forced;
        } else if (sel == 1) {
            /* exact arith_stop rule: leftover = 16 bits (2 bytes) if final range
             * <= 0x2000000, else 24 bits (3 bytes). */
            best_delta = (g_fac_final_range[LHDC_DEC_SLOT()] <= 0x2000000u) ? 2 : 3;
        } else {
            int64_t best_peak = -1;
            for (int d = 2; d <= 3; d++) {
                int fr = fac_bytes - d; if (fr < 0) fr = 0;
                int64_t peak = lhdc_dec_mant_plane(coeff, qs, cnt, pl, total_bits,
                                                   fac_start_bit + fr * 8, pred_mode, resid_val);
                if (best_peak < 0 || peak < best_peak) { best_peak = peak; best_delta = d; }
            }
        }
#if defined(LHDC_HOST_BUILD)
        { if (getenv("LHDC_MANT_OFF")) { goto mant_done; } }
        { const char *e = getenv("LHDC_MANT_ABS");
          if (e) { int sb = atoi(e);
              (void)lhdc_dec_mant_plane(coeff, qs, cnt, pl, total_bits, sb, pred_mode, resid_val);
              for (int k = cnt; k < num_coeffs; k++) qs[k] = 0;
              goto mant_done; } }
#endif
        {
            int fr = fac_bytes - best_delta; if (fr < 0) fr = 0;
            int64_t pk = lhdc_dec_mant_plane(coeff, qs, cnt, pl, total_bits,
                                             fac_start_bit + fr * 8, pred_mode, resid_val);
            /* Desync backstop for the 2-vs-3-byte arith-stop gap. The range rule
             * above is exact (it mirrors the encoder's arith clear: final range
             * <= 0x2000000 flushes 2 bytes, else 1), so a retry should only fire
             * on a genuine desync. The threshold must NOT be 24-bit full scale:
             * the encoder's quantizer output |M| = round(spec * level) is not
             * clamped to the PCM range and legitimately exceeds 2^23 on dense
             * high-bitrate frames (measured 8390219 at 500 kbps/ch on a full-scale
             * tone). With the old 8388607 threshold every such frame "failed" the
             * sanity check, re-decoded with the wrong gap, and kept the garbage
             * because desynced planes often have SMALLER peaks — a systematic
             * corruption of correct frames. A real wrong-gap desync explodes |M|
             * far beyond 2^26. */
            if (forced < 0 && pk > (int64_t)(1 << 26)) {
                int other = (best_delta == 2) ? 3 : 2;
                int fr2 = fac_bytes - other; if (fr2 < 0) fr2 = 0;
                int64_t pk2 = lhdc_dec_mant_plane(coeff, qs, cnt, pl, total_bits,
                                                  fac_start_bit + fr2 * 8, pred_mode, resid_val);
                if (pk2 >= pk) {   /* other no better -> restore the range-rule result */
                    (void)lhdc_dec_mant_plane(coeff, qs, cnt, pl, total_bits,
                                              fac_start_bit + fr * 8, pred_mode, resid_val);
                } else {
                    best_delta = other;   /* qs already holds the better result */
                }
            }
        }
        for (int k = cnt; k < num_coeffs; k++) qs[k] = 0;
#if defined(LHDC_HOST_BUILD)
    mant_done:;
#endif
        /* Expose this channel's peak |M| so the caller can detect a desync (wrong
         * descramble selector) and retry with the flipped selector. */
        { int64_t mx = 0; for (int k = 0; k < cnt; k++) { int64_t a = qs[k] < 0 ? -qs[k] : qs[k]; if (a > mx) mx = a; }
          slt->chan_maxM = mx; }
        if (g_lhdc_trace) {
            ESP_LOGI("LHDCV5_DEC",
                     "ch%d: ch_start=%d ch_bytes=%d fac_start_bit=%d fac_bytes=%d best_delta=%d nzc=%d",
                     channel, ch_start, ch_bytes, fac_start_bit, fac_bytes, best_delta, nzc);
        }
        /* Next channel re-inits its own bit reader at its fixed slice (top of
         * decode_channel), so no boundary probing is needed here. */
    }

#if defined(LHDC_HOST_BUILD)
    if (g_lhdc_trace > 0) {
        char mpath[64]; snprintf(mpath, sizeof(mpath), "/data/local/tmp/lhdccal/Mdec_ch%d.txt", channel);
        FILE *mf = fopen(mpath, "w");
        if (mf) { for (int k = 0; k < num_coeffs; k++)
                      fprintf(mf, "%d %d\n", k, (int)slt->quant_spectrum[k]);
                  fclose(mf); }
    }
#endif

    /*
     * Coefficient de-reversal. The encoder quantizes the spectrum in REVERSED
     * bin order: its quant loop (LhdcEncode @0xd1b48) reads the whitened
     * spectrum backward from bin (nzc-1) down to 0 while writing the coeff
     * array forward, so transmitted coeff[j] = quantize(spectrum[nzc-1-j]).
     * The decoder must reverse the first nzc decoded coeffs back to natural bin
     * order before dequant + SNS, otherwise a low-frequency tone lands at a high
     * bin (the long-standing "1kHz -> noise" bug). Bins [nzc..num_coeffs) stay 0.
     */
    {
        int32_t *qs = slt->quant_spectrum;
        /* The decoded coeffs occupy qs[0..nzc-1] = M[nc-nzc+k] = the encoder's
         * significant M region, which is the spectrum REVERSED. So bin s comes
         * from qs[nzc-1-s]. Reverse the first nzc entries in place; bins
         * [nzc..num_coeffs) (the dropped top spectrum lines) stay 0. Verified:
         * decoder M == encoder M bit-exact, and this puts a 1kHz tone at bin 9. */
        int rn = nzc; if (rn > num_coeffs) rn = num_coeffs;
        for (int a = 0, b = rn - 1; a < b; a++, b--) {
            int32_t t = qs[a]; qs[a] = qs[b]; qs[b] = t;
        }
    }

    t_mant1 = LHDC_NOW_US();
    /* Inverse quantization: linear dequant by 2^e(global_gain) (§11). */
    int bit_depth = (int)dec->config.bit_depth;
    lhdc_dec_inverse_quantize(slt->quant_spectrum,
                               slt->mdct_in, num_coeffs,
                               &slt->sns_params, (enc_frame_len / n_ch), bit_depth,
                               dec->config.sample_rate);
    if (g_lhdc_trace > 0) {
        int qmax = 0; float qsum = 0.0f; float dmax = 0;
        for (int k = 0; k < num_coeffs; k++) {
            int q = slt->quant_spectrum[k];
            if (q < 0) q = -q;
            if (q > qmax) qmax = q;
            qsum += q;
            float d = slt->mdct_in[k] < 0 ? -slt->mdct_in[k] : slt->mdct_in[k];
            if (d > dmax) dmax = d;
        }
        /* dominant bin (1kHz tone -> bin 10 @ 100Hz/bin); show its q and spectrum */
        int pk = 0; float pkv = 0;
        for (int k = 0; k < num_coeffs; k++) {
            float a = slt->mdct_in[k] < 0 ? -slt->mdct_in[k] : slt->mdct_in[k];
            if (a > pkv) { pkv = a; pk = k; }
        }
        ESP_LOGI("LHDCV5_DEC", "ch%d dequant: gain=%d e=%.2f qmean=%.3f specmax=%.1f peakbin=%d | q[8..12]=%d %d %d %d %d",
                 channel, (int)slt->sns_params.global_gain,
                 lhdc_dec_gain_exponent(slt->sns_params.global_gain, enc_frame_len, bit_depth, dec->config.sample_rate),
                 qsum / num_coeffs, dmax, pk,
                 (int)slt->quant_spectrum[8], (int)slt->quant_spectrum[9],
                 (int)slt->quant_spectrum[10], (int)slt->quant_spectrum[11],
                 (int)slt->quant_spectrum[12]);
    }

    if (g_lhdc_trace > 0 && channel == 0) {
        printf("[I]LHDCV5_DEC: PRESNS[0..%d]:", num_coeffs);
        for (int k = 0; k < num_coeffs; k++) printf(" %.0f", slt->mdct_in[k]);
        printf("\n");
    }

#if defined(LHDC_HOST_BUILD)
    if (channel == 0) {
        const char *sp = getenv("LHDC_DUMP_PRESPEC");
        if (sp) { FILE *spf = fopen(sp, "ab");
            if (spf) { fwrite(slt->mdct_in, sizeof(float), num_coeffs, spf); fclose(spf); } }
    }
#endif
    /* Apply SNS synthesis (inverse noise shaping) */
    lhdc_sns_synth_apply(slt->mdct_in, &slt->sns_params,
                          mdct_size / 2,
                          band_cfg->band_off, band_cfg->band_scale, num_sfb);
    if (g_lhdc_trace > 0) {
        float smax = 0; int pk = 0;
        for (int k = 0; k < num_coeffs; k++) {
            float d = slt->mdct_in[k] < 0 ? -slt->mdct_in[k] : slt->mdct_in[k];
            if (d > smax) { smax = d; pk = k; }
        }
        ESP_LOGI("LHDCV5_DEC", "ch%d after SNS: specmax=%.1f peakbin=%d sf[0..7]=%d %d %d %d %d %d %d %d",
                 channel, smax, pk, (int)slt->sns_params.scale_factors[0],
                 (int)slt->sns_params.scale_factors[1],
                 (int)slt->sns_params.scale_factors[2],
                 (int)slt->sns_params.scale_factors[3],
                 (int)slt->sns_params.scale_factors[4],
                 (int)slt->sns_params.scale_factors[5],
                 (int)slt->sns_params.scale_factors[6],
                 (int)slt->sns_params.scale_factors[7]);
    }

#if defined(LHDC_HOST_BUILD)
    /* Host diagnostic: append the exact IMDCT input (post-SNS spectrum) for ch0
     * to a binary file so an offline tool can run a known-clean IMDCT+OLA and
     * localize the frame-rate AM (spectrum vs synthesis). */
    if (channel == 0) {
        const char *sp = getenv("LHDC_DUMP_SPEC");
        if (sp) {
            FILE *spf = fopen(sp, "ab");
            if (spf) { fwrite(slt->mdct_in, sizeof(float), num_coeffs, spf); fclose(spf); }
        }
    }
#endif
#if defined(LHDC_HOST_BUILD)
    /* STAGE-MAX localization: print the peak magnitude at each decode stage so a
     * garble frame's explosion can be pinned to entropy / dequant / SNS. */
    if (getenv("LHDC_STAGEMAX")) {
        int qmax = 0; for (int k = 0; k < num_coeffs; k++) { int q = slt->quant_spectrum[k]; if (q<0) q=-q; if (q>qmax) qmax=q; }
        float smax = 0.0f; for (int k = 0; k < num_coeffs; k++) { float a = slt->mdct_in[k]; if (a<0) a=-a; if (a>smax) smax=a; }
        printf("[STAGEMAX] fr=%u ch=%d nzc_qmax=%d postSNS_specmax=%.0f gain=%d sns_mode=%d\n",
               dec->frame_index, channel, qmax, smax,
               (int)slt->sns_params.global_gain, (int)slt->sns_params.sns_mode);
    }
#endif
    /* IMDCT: reads w.mdct_in, writes u.mdct_out. After this w.mdct_in is dead. */
    t_im0 = LHDC_NOW_US();
    lhdc_imdct_transform(slt->mdct_in, slt->mdct_out, mdct_size);
    t_im1 = LHDC_NOW_US();
#if defined(LHDC_HOST_BUILD)
    if (channel == 0) { const char *mo = getenv("LHDC_DUMP_MDCTOUT");
        if (mo) { FILE *f = fopen(mo, "ab"); if (f) { fwrite(slt->mdct_out, sizeof(float), mdct_size, f); fclose(f); } } }
#endif

#if defined(LHDC_HOST_BUILD)
    float dbg_preov = 0.0f;
    if (getenv("LHDC_OVLDBG")) {
        const float *w = lhdc_dec_get_window(mdct_size);
        for (int n = 0; n < mdct_size/2; n++) { float a = slt->mdct_out[n]*w[n]; if (a<0) a=-a; if (a>dbg_preov) dbg_preov=a; }
    }
#endif
    /* Overlap-add: reads u.mdct_out + overlap_buf, writes pcm_out (= w.ch_pcm,
     * which aliases the now-dead w.mdct_in). overlap_add never reads mdct_in. */
    lhdc_dec_overlap_add(dec, pcm_out, channel, mdct_size);
#if defined(LHDC_HOST_BUILD)
    if (channel == 0) { const char *po = getenv("LHDC_DUMP_POSTOV");
        if (po) { FILE *f = fopen(po, "ab"); if (f) { fwrite(pcm_out, sizeof(float), dec->header.samples_per_channel, f); fclose(f); } } }
#endif
#if defined(LHDC_HOST_BUILD)
    if (getenv("LHDC_OVLDBG")) {
        float postov = 0.0f; int sp = dec->header.samples_per_channel;
        for (int n = 0; n < sp; n++) { float a = pcm_out[n]; if (a<0) a=-a; if (a>postov) postov=a; }
        printf("[OVLDBG] fr=%u ch=%d preov_peak=%.0f postov_peak=%.0f ratio=%.2f\n",
               dec->frame_index, channel, dbg_preov, postov, dbg_preov>0?postov/dbg_preov:0.0f);
    }
#endif

    /* Steady-state stage profile. Disabled: the periodic ESP_LOGI blocks the
     * decode task for ~9 ms on the 115200-baud UART, which audibly stutters the
     * audio. Re-enable by setting LHDCV5_DEC_PROFILE to 1 for bring-up only. */
#if LHDCV5_DEC_PROFILE
    if (g_lhdc_trace == 0) {
        static int64_t a_chan = 0, a_ent = 0, a_im = 0;
        static int64_t a_pre = 0, a_mant = 0, a_dq = 0, a_post = 0;
        static uint32_t a_n = 0;
        const int64_t t_end = LHDC_NOW_US();
        a_chan += t_end - t_chan0;
        a_ent  += t_ent1 - t_ent0;
        a_im   += t_im1 - t_im0;
        a_pre  += t_ent0 - t_chan0;
        a_mant += t_mant1 - t_ent1;
        a_dq   += t_im0 - t_mant1;
        a_post += t_end - t_im1;
        if (++a_n >= 2000) {   /* 2000 channels = 1000 frames = ~5 s at 192k */
            ESP_LOGI("LHDCV5_DEC",
                     "PROF (avg/chan over %u): total=%lld  entropy=%lld  imdct=%lld  other=%lld us",
                     a_n, a_chan / a_n, a_ent / a_n, a_im / a_n,
                     (a_chan - a_ent - a_im) / a_n);
            ESP_LOGI("LHDCV5_DEC",
                     "PROF   other split: pre=%lld  mant=%lld  dq+sns=%lld  post=%lld us",
                     a_pre / a_n, a_mant / a_n, a_dq / a_n, a_post / a_n);
            a_chan = a_ent = a_im = 0;
            a_pre = a_mant = a_dq = a_post = 0;
            a_n = 0;
        }
    }
#else
    (void)t_chan0; (void)t_ent0; (void)t_ent1; (void)t_im0; (void)t_im1; (void)t_mant1;
#endif

    return LHDC_DEC_OK;
}

/* Decode a channel, auto-correcting the descramble selector. The byte[8]&1
 * selector is right for most frames but wrong for a minority, desyncing that
 * channel's FAC into a huge spectrum (the 900k dense-channel garble / stutter).
 * Detection uses g_lhdc_chan_specmax — the peak of the post-SNS spectrum, which
 * is computed BEFORE the IMDCT/overlap-add and so is independent of the overlap
 * history (a post-overlap time peak is context-dependent and makes the selector
 * decision cascade across frames). Decode with the auto selector; if the spectrum
 * is in range keep it, else re-decode with the flipped selector and keep whichever
 * is valid with the smaller spectral peak. Only bad frames pay the extra pass.
 * decode_channel mutates overlap_buf via overlap-add, so the prior-frame overlap
 * is snapshotted and restored before each re-decode. */
/* Peak |sample| of a decoded channel (post overlap-add) in the internal scale. */
static float lhdc_chan_peak(const float *pcm, int n)
{
    float pk = 0.0f;
    for (int i = 0; i < n; i++) { float a = pcm[i] < 0 ? -pcm[i] : pcm[i]; if (a > pk) pk = a; }
    return pk;
}

static lhdc_dec_ret_t lhdc_dec_decode_channel_autosel(lhdc_decoder_t *dec,
                                                       int channel, float *pcm_out)
{
#if CONFIG_IDF_TARGET_ESP32
    /* Classic ESP32 (compute-limited at 192k): SKIP the desync retry. It only fires
     * on a dropped-packet-corrupted frame (the decoder is bit-exact on intact frames,
     * verified on the S31), which re-decoding with a flipped selector cannot repair,
     * and the 2-3x re-decode spirals: drop -> desync -> retry -> slower -> more drops.
     * Decode once (auto selector); the frame-level conceal still mutes any garbage. */
    return lhdc_dec_decode_channel(dec, channel, pcm_out, -1);
#else
    /* Desync detector = the same condition the frame conceal uses, per channel:
     * |pcm|*sc > 2*clip_lim  ->  |pcm| > 2*clip_lim/sc. Measured AFTER overlap-add
     * (matches what the listener hears). The overlap is restored before each retry
     * so the decision uses the true prior-frame context (no cascade). */
    int bd = (int)dec->config.bit_depth;
    float sc = ldexpf(1.0f, bd - 23) * lhdc_dec_rate_norm(dec->band_cfg ? dec->band_cfg->mdct_size : 480)
             * lhdc_dec_level_cal(dec->header.sample_rate, bd);
    float clip_lim = (bd == 16) ? 32767.0f : (bd == 24) ? 8388607.0f : 2147483647.0f;
    const float PEAK_LIMIT = 2.0f * clip_lim / sc;
    int samples = dec->header.samples_per_channel;
    int n_ch = (dec->header.channels < 1) ? 1 : dec->header.channels;
    int ch0_bytes = dec->header.frame_bytes / n_ch;
    int ch_start = channel * ch0_bytes;
    int auto_sel = (ch_start + 8 < (int)sizeof(dec->payload_buf))
                 ? (dec->payload_buf[ch_start + 8] & 1) : 0;

    /* Snapshot the prior-frame overlap (was a 3.8 KB .bss; now the rate-sized
     * ov_save buffer carved from the workspace, so it frees with the decoder).
     * overlap_buf holds H = alloc_mdct_size/2 floats; cap to that. */
    int ov_n = samples;
    int ov_cap = dec->alloc_mdct_size / 2;
    if (ov_n > ov_cap) ov_n = ov_cap;
    float *s_ov_save = dec->ov_save;
    for (int i = 0; i < ov_n; i++) s_ov_save[i] = dec->overlap_buf[channel][i];

    lhdc_dec_ret_t r0 = lhdc_dec_decode_channel(dec, channel, pcm_out, -1);
    float p0 = (r0 == LHDC_DEC_OK) ? lhdc_chan_peak(pcm_out, samples) : 3.0e30f;
    if (r0 == LHDC_DEC_OK && p0 <= PEAK_LIMIT) return r0;   /* clean — common path */

    /* Suspect a wrong selector: try the flipped one from the saved overlap. */
    for (int i = 0; i < ov_n; i++) dec->overlap_buf[channel][i] = s_ov_save[i];
    lhdc_dec_ret_t r1 = lhdc_dec_decode_channel(dec, channel, pcm_out, auto_sel ^ 1);
    float p1 = (r1 == LHDC_DEC_OK) ? lhdc_chan_peak(pcm_out, samples) : 3.0e30f;
#if defined(LHDC_HOST_BUILD)
    if (getenv("LHDC_SELLOG"))
        printf("[SELLOG] ch%d auto_sel=%d r0=%d p0=%.0f r1=%d p1=%.0f limit=%.0f -> %s\n",
               channel, auto_sel, r0, p0, r1, p1, PEAK_LIMIT,
               (r1 == LHDC_DEC_OK && p1 < p0) ? "FLIP" : "keep-auto");
#endif

    if (r1 == LHDC_DEC_OK && p1 < p0) return r1;            /* flipped is better (pcm_out holds it) */

    /* Keep the auto result: restore overlap, re-decode with the auto selector. */
    for (int i = 0; i < ov_n; i++) dec->overlap_buf[channel][i] = s_ov_save[i];
    return lhdc_dec_decode_channel(dec, channel, pcm_out, auto_sel);
#endif  /* !CONFIG_IDF_TARGET_ESP32 */
}

#if LHDC_NSLOTS > 1
/*
 * Channel-1 worker (core 0). The A2DP decode task owns core 1 and is the
 * bottleneck at 192k/1000kbps (measured ~100% busy while core 0 sat at ~37%),
 * so channel 1 is handed to a small task pinned to core 0 and the two channels
 * decode concurrently. Two binary semaphores give a per-frame dispatch/barrier;
 * the worker only ever touches slot 0's scratch (it runs on core 0) while the
 * caller uses slot 1, so there is no shared decode state.
 */
#include "freertos/semphr.h"

/* Below the A2DP decode task (BT_TASK_MAX_PRIORITIES-6) and the BT controller,
 * but well above idle so the barrier is not held up by background work. */
#define LHDC_CH1_TASK_PRIO   (configMAX_PRIORITIES - 7)
/* The worker's call chain is shallow (decode_channel -> entropy/IMDCT), whose
 * biggest frames are decode_channel's ~600 B trace buffer and fft480's
 * float ar[32]/ai[32]; 4 KB leaves ample margin while keeping byte-DRAM down. */
#define LHDC_CH1_TASK_STACK  (4 * 1024)

static TaskHandle_t      s_ch1_task  = NULL;
static SemaphoreHandle_t s_ch1_start = NULL;
static SemaphoreHandle_t s_ch1_done  = NULL;
static struct {
    lhdc_decoder_t *dec;
    float          *out;
    lhdc_dec_ret_t  ret;
} s_ch1_job;

static lhdc_dec_ret_t lhdc_dec_decode_channel_autosel(lhdc_decoder_t *dec,
                                                       int channel, float *pcm_out);

static void lhdc_dec_ch1_worker(void *arg)
{
    (void)arg;
    for (;;) {
        xSemaphoreTake(s_ch1_start, portMAX_DELAY);
        s_ch1_job.ret = lhdc_dec_decode_channel_autosel(s_ch1_job.dec, 1, s_ch1_job.out);
        xSemaphoreGive(s_ch1_done);
    }
}

/* Post channel 1 to the core-0 worker. Returns true if dispatched (caller then
 * decodes ch0 and must call lhdc_dec_ch1_join), false to decode inline. */
static bool lhdc_dec_ch1_dispatch(lhdc_decoder_t *dec, float *out_r)
{
    if (!s_ch1_task) {
        if (!s_ch1_start) s_ch1_start = xSemaphoreCreateBinary();
        if (!s_ch1_done)  s_ch1_done  = xSemaphoreCreateBinary();
        if (!s_ch1_start || !s_ch1_done) return false;
        if (xTaskCreatePinnedToCore(lhdc_dec_ch1_worker, "LhdcCh1", LHDC_CH1_TASK_STACK,
                                    NULL, LHDC_CH1_TASK_PRIO, &s_ch1_task, 0) != pdPASS) {
            s_ch1_task = NULL;
            return false;
        }
    }
    s_ch1_job.dec = dec;
    s_ch1_job.out = out_r;
    s_ch1_job.ret = LHDC_DEC_OK;
    xSemaphoreGive(s_ch1_start);
    return true;
}

static lhdc_dec_ret_t lhdc_dec_ch1_join(void)
{
    xSemaphoreTake(s_ch1_done, portMAX_DELAY);
    return s_ch1_job.ret;
}
#endif /* LHDC_NSLOTS > 1 */

/* Public API */

/* Public API helpers.  The rate-sized buffers are allocated one-by-one
 * (see lhdc_dec_alloc_tails); nothing is carved from the workspace tail any
 * more, so the caller only supplies storage for the struct itself. */
#if LHDC_NSLOTS > 1
static size_t lhdc_dec_slot_bytes(int M);
#endif

static int lhdc_dec_mdct_size(uint32_t sr, uint8_t dur)
{
    const lhdc_band_cfg_desc_t *bc = lhdc_get_band_cfg(sr, dur ? dur : 5);
    int M = bc ? (int)bc->mdct_size : 480;
    if (M <= 0 || M > LHDC_DEC_MAX_MDCT_SIZE) M = LHDC_DEC_MAX_MDCT_SIZE;
    return M;
}

/* Bytes of per-slot scratch (the buffers a single channel decode writes). Only
 * slot 1 uses this: slot 0's buffers are allocated one-by-one. */
#if LHDC_NSLOTS > 1
static size_t lhdc_dec_slot_bytes(int M)
{
    int H = M / 2;
    size_t b = 0;
    b += (size_t)H * sizeof(float);   /* w_buf (mdct_in / int32 coeff scratch) */
    b += (size_t)M * sizeof(float);   /* u_buf (mdct_out[M] / quant_spectrum[H]) */
    b += (size_t)H;                   /* ent_s1 (num_coeffs) */
    b += (size_t)M;                   /* ent_s2 (num_coeffs*2) */
    return b;
}
#endif /* LHDC_NSLOTS > 1 */

#if LHDC_NSLOTS > 1
/* Point one slot's work pointers at `p` (slot-scratch layout: 4-byte arrays
 * first for natural alignment, then the byte arrays). Returns the end pointer.
 * Only slot 1 is carved from a shared block now; slot 0's buffers are
 * allocated individually (see lhdc_dec_alloc_tails). */
static uint8_t *lhdc_dec_carve_slot(lhdc_dec_slot_t *s, uint8_t *p, int M)
{
    int H = M / 2;
    s->mdct_in        = (float *)p;   p += (size_t)H * sizeof(float);
    s->mdct_out       = (float *)p;   p += (size_t)M * sizeof(float);
    s->quant_spectrum = (int32_t *)s->mdct_out;         /* alias (lifetimes disjoint) */
    s->ent_s1         = p;            p += (size_t)H;   /* num_coeffs */
    s->ent_s2         = p;            p += (size_t)M;   /* num_coeffs*2 */
    s->hdr_saved_start = -1;
    /* Re-align for whatever float array follows this slot in the same block. */
    return (uint8_t *)(((uintptr_t)p + 3u) & ~(uintptr_t)3u);
}
#endif /* LHDC_NSLOTS > 1 */

#if LHDC_NSLOTS > 1
/* Slot 1's scratch lives in its OWN heap block, deliberately not appended to the
 * workspace: the classic ESP32 cannot find one ~55 KB contiguous byte-DRAM hole
 * once BT is streaming, but a ~33 KB block plus a ~14 KB block fit comfortably. */
static uint8_t *s_slot1_mem = NULL;
static int      s_slot1_mdct = 0;

void lhdc_dec_free_slot_scratch(void)
{
    if (s_slot1_mem) { LHDC_DEC_FREE(s_slot1_mem); s_slot1_mem = NULL; }
    s_slot1_mdct = 0;
}
#else
void lhdc_dec_free_slot_scratch(void) { }
#endif

/* Free the individually-allocated rate-sized work buffers. Guarded by
 * alloc_mdct_size, so it no-ops on a zero-initialized workspace (the caller
 * MUST zero the workspace before the first lhdc_dec_init). Pointers that are
 * only aliases (ch_pcm -> mdct_in, quant_spectrum -> mdct_out) are cleared,
 * never freed. */
static void lhdc_dec_free_tails(lhdc_decoder_t *dec)
{
    if (!dec || dec->alloc_mdct_size == 0) {
        return;
    }
#define LHDC_DEC_FREET(f) do { if (f) { LHDC_DEC_FREE(f); (f) = NULL; } } while (0)
    LHDC_DEC_FREET(dec->slot[0].mdct_in);
    LHDC_DEC_FREET(dec->slot[0].mdct_out);
    LHDC_DEC_FREET(dec->slot[0].ent_s1);
    LHDC_DEC_FREET(dec->slot[0].ent_s2);
    dec->slot[0].quant_spectrum = NULL;                 /* alias of mdct_out */
    dec->ch_pcm                 = NULL;                 /* alias of slot0 mdct_in */
    LHDC_DEC_FREET(dec->overlap_buf[0]);
    LHDC_DEC_FREET(dec->overlap_buf[1]);
    LHDC_DEC_FREET(dec->pcm_mid);
#if LHDC_NSLOTS > 1
    LHDC_DEC_FREET(dec->pcm_r);
    /* slot[1] points into the separately-managed s_slot1_mem block (or aliased
     * slot[0] on OOM); drop the pointers, the block itself is owned by
     * lhdc_dec_free_slot_scratch(). */
    memset(&dec->slot[1], 0, sizeof(dec->slot[1]));
#else
    LHDC_DEC_FREET(dec->ov_save);
#endif
#undef LHDC_DEC_FREET
    dec->alloc_mdct_size = 0;
}

/* Allocate the rate-sized work buffers individually and point the decoder's
 * work pointers at them (preserving the mdct_in/ch_pcm and mdct_out/
 * quant_spectrum aliasing). Largest single request is mdct_out = M*4 B, i.e.
 * 7,680 B at 192k -- always findable even on a fragmented internal heap,
 * unlike the old 32.5 KB contiguous slab. Zeroes the overlap buffers.
 * Returns 0 and frees everything on failure (alloc_mdct_size stays 0 so a
 * later retry re-allocates cleanly). */
static int lhdc_dec_alloc_tails(lhdc_decoder_t *dec, int M)
{
    int H = M / 2;
    lhdc_dec_slot_t *s0 = &dec->slot[0];

    s0->mdct_in         = (float   *)LHDC_DEC_MALLOC((size_t)H * sizeof(float));
    s0->mdct_out        = (float   *)LHDC_DEC_MALLOC((size_t)M * sizeof(float));
    s0->ent_s1          = (uint8_t *)LHDC_DEC_MALLOC((size_t)H);   /* num_coeffs */
    s0->ent_s2          = (uint8_t *)LHDC_DEC_MALLOC((size_t)M);   /* num_coeffs*2 */
    dec->overlap_buf[0] = (float   *)LHDC_DEC_MALLOC((size_t)H * sizeof(float));
    dec->overlap_buf[1] = (float   *)LHDC_DEC_MALLOC((size_t)H * sizeof(float));
    dec->pcm_mid        = (float   *)LHDC_DEC_MALLOC((size_t)H * sizeof(float));
#if LHDC_NSLOTS > 1
    /* The autosel selector-retry is compiled out on this target, so ov_save is
     * dead; pcm_r takes its place as channel 1's dedicated PCM output. */
    dec->pcm_r          = (float   *)LHDC_DEC_MALLOC((size_t)H * sizeof(float));
#else
    dec->ov_save        = (float   *)LHDC_DEC_MALLOC((size_t)H * sizeof(float));
#endif

    s0->quant_spectrum  = (int32_t *)s0->mdct_out;      /* alias (lifetimes disjoint) */
    s0->hdr_saved_start = -1;
    dec->ch_pcm         = s0->mdct_in;                  /* sequential path: ch1 PCM aliases w_buf */

    if (!s0->mdct_in || !s0->mdct_out || !s0->ent_s1 || !s0->ent_s2 ||
        !dec->overlap_buf[0] || !dec->overlap_buf[1] || !dec->pcm_mid ||
#if LHDC_NSLOTS > 1
        !dec->pcm_r) {
#else
        !dec->ov_save) {
#endif
        dec->alloc_mdct_size = M;   /* let free_tails run, then reset */
        lhdc_dec_free_tails(dec);
        return 0;
    }

    /* overlap_buf[0..1] are separate blocks now: zero each one. (The old
     * contiguous slab allowed a single 2*H memset; doing that here would run
     * off buffer[0] into heap metadata.) */
    memset(dec->overlap_buf[0], 0, (size_t)H * sizeof(float));
    memset(dec->overlap_buf[1], 0, (size_t)H * sizeof(float));
    dec->alloc_mdct_size = M;

#if LHDC_NSLOTS > 1
    /* Slot 1: one contiguous block of its own (~14 KB), rate-sized, reallocated
     * only when the rate changes. */
    if (s_slot1_mem && s_slot1_mdct != M) lhdc_dec_free_slot_scratch();
    if (!s_slot1_mem) {
        s_slot1_mem = (uint8_t *)LHDC_DEC_MALLOC(lhdc_dec_slot_bytes(M));
        if (s_slot1_mem) s_slot1_mdct = M;
    }
    if (s_slot1_mem) {
        (void)lhdc_dec_carve_slot(&dec->slot[1], s_slot1_mem, M);
    } else {
        /* Out of memory: alias slot 1 onto slot 0. The parallel dispatch is
         * skipped in that case (see lhdc_dec_decode_frame), so the two channels
         * still never share scratch concurrently. */
        dec->slot[1] = dec->slot[0];
    }
#endif
    return 1;
}

size_t lhdc_dec_get_workspace_size(uint32_t sample_rate, uint8_t frame_duration)
{
    (void)sample_rate; (void)frame_duration;
    /* Split-workspace: the rate-sized work buffers are allocated by the decoder
     * itself (lhdc_dec_init), so the caller only supplies storage for the
     * rate-independent struct. Both arguments are kept for API compatibility. */
    return sizeof(lhdc_decoder_t);
}

lhdc_decoder_t *lhdc_dec_init(void *workspace, const lhdc_dec_config_t *config)
{
    lhdc_decoder_t *dec = (lhdc_decoder_t *)workspace;

    if (!dec) {
        return NULL;
    }

    /* Split-workspace: release the previous init's rate-sized buffers BEFORE the
     * memset zeroes their pointers, otherwise a rate change leaks them. On first
     * use the caller zero-initialized the workspace, so this no-ops. */
    lhdc_dec_free_tails(dec);

    memset(dec, 0, sizeof(*dec));

    if (config) {
        memcpy(&dec->config, config, sizeof(*config));
    } else {
        /* Default config - will be auto-detected from bitstream */
        dec->config.sample_rate    = LHDC_DEC_SR_48000;
        dec->config.bit_depth      = LHDC_DEC_BITDEPTH_24;
        dec->config.frame_duration = LHDC_DEC_FRAME_5MS;
        dec->config.channels       = 2;
        dec->config.max_frame_bytes = LHDC_DEC_MAX_FRAME_BYTES;
        dec->config.lossless_enable = 0;
    }

    /* Split-workspace: allocate each rate-sized work buffer on its own (largest
     * single request ~7.7 KB at 192k) instead of carving one 32.5 KB contiguous
     * slab, so a fragmented internal heap can never block LHDC from starting. */
    {
        int M = lhdc_dec_mdct_size(dec->config.sample_rate, dec->config.frame_duration);
        if (!lhdc_dec_alloc_tails(dec, M)) {
            return NULL;
        }
        /* Build this rate's IMDCT tables now and, crucially, FREE the 96k
         * (N=960) fast tables (~15 KB) when this rate is not 96k. Otherwise they
         * linger on the heap after a 96k->48k switch (transform() only inits when
         * the fast path isn't built yet, so it never frees them on downswitch). */
        lhdc_imdct_init(M);
    }

    /* Allocate the FAC entropy models NOW (config time), while the heap still has
     * a large contiguous block. Allocating them lazily on the first decode failed
     * at 96k -- by then BT streaming fragments the heap to a ~4 KB largest block
     * and the ~6 KB request fails -> every frame errored -> no audio. */
    lhdc_entropy_alloc();

    dec->initialized = 1;
    dec->header_parsed = 0;
    dec->frame_index = 0;

    return dec;
}

/*
 * Frame-latency telemetry against the real-time budget.
 *
 * An LHDC V5 frame is 5 ms of audio (2.5 ms in low-latency mode) and carries
 * BOTH channels, so the whole decode_frame call -- entropy + mantissa + IMDCT +
 * overlap-add for L and R, plus the interleave -- has to finish inside
 * frame_duration_ms. These counters make that directly observable instead of
 * inferred from CPU% or audible artefacts.
 *
 * Deliberately just counters: the decode task does two esp_timer reads and a
 * handful of integer ops per frame (well under 1 us of a 5000 us budget) and
 * NEVER logs. A separate low-priority task drains them via
 * lhdc_dec_latency_stats(), so the UART stall of printing lands there and not in
 * the audio path -- which is exactly what made the old LHDCV5_DEC_PROFILE
 * ESP_LOGI unusable (~9 ms on the wire, i.e. two whole frames).
 */
static volatile uint32_t s_lat_frames  = 0;   /* frames timed since last drain */
static volatile uint32_t s_lat_sum_us  = 0;   /* sum of per-frame decode times */
static volatile uint32_t s_lat_max_us  = 0;   /* worst frame since last drain */
static volatile uint32_t s_lat_over    = 0;   /* frames that missed the budget */
static volatile uint32_t s_lat_budget  = 0;   /* frame_duration_ms * 1000 */
static volatile uint32_t s_lat_bytes   = 0;   /* encoded bytes of the worst frame */

void lhdc_dec_latency_stats(lhdc_dec_latency_t *out, int reset)
{
    if (out) {
        out->frames    = s_lat_frames;
        out->avg_us    = s_lat_frames ? (s_lat_sum_us / s_lat_frames) : 0;
        out->max_us    = s_lat_max_us;
        out->over      = s_lat_over;
        out->budget_us = s_lat_budget;
        out->worst_bytes = s_lat_bytes;
    }
    if (reset) {
        s_lat_frames = 0; s_lat_sum_us = 0; s_lat_max_us = 0;
        s_lat_over = 0; s_lat_bytes = 0;
    }
}

/* Struct-free accessor so a consumer outside this component (e.g. the app's
 * monitor task) can drain the counters with a one-line extern declaration and no
 * dependency on this private header. */
void lhdc_dec_latency_stats_raw(uint32_t *frames, uint32_t *avg_us, uint32_t *max_us,
                                uint32_t *over, uint32_t *budget_us, uint32_t *worst_bytes,
                                int reset)
{
    lhdc_dec_latency_t st;
    lhdc_dec_latency_stats(&st, reset);
    if (frames)      *frames      = st.frames;
    if (avg_us)      *avg_us      = st.avg_us;
    if (max_us)      *max_us      = st.max_us;
    if (over)        *over        = st.over;
    if (budget_us)   *budget_us   = st.budget_us;
    if (worst_bytes) *worst_bytes = st.worst_bytes;
}

lhdc_dec_ret_t lhdc_dec_decode_frame(
    lhdc_decoder_t *dec,
    const uint8_t  *in_data,
    size_t          in_bytes,
    void           *out_pcm,
    uint32_t        out_samples,
    size_t         *consumed,
    uint32_t       *generated,
    lhdc_dec_frame_info_t *info)
{
    if (!dec || !dec->initialized) {
        return LHDC_DEC_NOT_INITIALIZED;
    }
    if (!in_data || in_bytes < 4 || !out_pcm || !consumed || !generated) {
        return LHDC_DEC_INVALID_PARAM;
    }

    *consumed = 0;
    *generated = 0;
    const int64_t t_frame0 = LHDC_NOW_US();

    /*
     * Parse the [u16 LE][payload] frame header and descramble the payload
     * (§5/§6). On success the bit reader is positioned at the start of the
     * descrambled payload and frame_consumed = header + payload bytes.
     */
    size_t frame_consumed = 0;
    lhdc_dec_ret_t ret = lhdc_dec_parse_header(dec, in_data, in_bytes,
                                               &frame_consumed);
    if (ret != LHDC_DEC_OK) {
        return ret;
    }

    dec->header_parsed = 1;

    /* Set up band configuration. Selected PER FRAME: at 44.1/48k the encoder
     * switches segment configs with the per-channel bitrate (<=59 bytes/ch at
     * 5 ms -> the 24-band LB config; 60..74 -> 480-edge; >=75 -> 400-edge), and
     * num_sfb changes the SNS side-bit count in the leading section. */
    {
        int bc_ch = (dec->header.channels < 1) ? 1 : dec->header.channels;
        dec->band_cfg = lhdc_get_band_cfg_frame(dec->header.sample_rate,
                                                dec->header.frame_duration_ms,
                                                (int)dec->header.frame_bytes / bc_ch);
        dec->header.band_cfg_idx = (uint8_t)dec->band_cfg->cfg_idx;
    }

    int mdct_size = dec->band_cfg->mdct_size;
    int samples_per_ch = dec->header.samples_per_channel;
    int channels = dec->header.channels;

    /* Check output buffer size */
    if ((uint32_t)samples_per_ch > out_samples) {
        return LHDC_DEC_BUF_NOT_ENOUGH;
    }

    /* Initialize IMDCT if needed */
    if (lhdc_imdct_init(mdct_size) != 0) {
        return LHDC_DEC_ERROR;
    }

    /* Window is produced on demand from the shared static cache (overlap-add).
     * Also build the structured descriptor (DRAM ramp + region bounds) here, so
     * the per-frame overlap-add never touches flash. Cheap and idempotent: it
     * returns immediately once built for this mdct_size. */
    (void)lhdc_dec_get_window(mdct_size);
    lhdc_dec_window_prepare(mdct_size);

    /* Decode each channel */
    /* Output container width. The decode/gain math still runs at the real source
     * bit depth (dec->header.bit_depth), but a 24-bit source is EMITTED in a
     * 32-bit int container (4-byte, interleaved L/R) instead of 3-byte packed,
     * matching what LDAC/aptX-HD emit. This is audio-neutral (the pipeline's
     * 24-bit path just <<8's to the same int32), but gives a power-of-2 sample
     * stride end-to-end, removing the only 3-byte-aligned step in the real-time
     * I2S/pipeline path. 16-bit is left as-is. */
    int out_bit_depth = ((int)dec->header.bit_depth == 24) ? 32 : (int)dec->header.bit_depth;
    int bytes_per_sample = out_bit_depth / 8;
    int do_stereo = (channels >= 2);

    /*
     * STEREO. LHDC V5 transmits the two channels as DIRECT L/R (ch0 = L,
     * ch1 = R) — verified: encoding L=1kHz/R=2kHz decodes ch0->1kHz, ch1->2kHz.
     * (NOT mid/side.) Each channel's 8-byte header is descrambled separately
     * (see lhdc_descramble_inplace in decode_channel), and the channels are fully
     * independent — no in-frame cross-channel dependency. Mono just emits ch0.
     *
     * Each channel decodes into its OWN output buffer so the two can run at the
     * same time: out_l = pcm_mid, out_r = pcm_r (dual-core) or the shared ch_pcm
     * scratch (sequential, where ch1 simply reuses w_buf as before).
     */
    float *out_l = dec->pcm_mid;
#if LHDC_NSLOTS > 1
    float *out_r = dec->pcm_r;
#else
    float *out_r = dec->ch_pcm;
#endif

#if LHDC_NSLOTS > 1
    /* Parallel: hand ch1 to the core-0 worker, decode ch0 here on core 1, join. */
    if (do_stereo && s_slot1_mem && lhdc_dec_ch1_dispatch(dec, out_r)) {
        ret = lhdc_dec_decode_channel_autosel(dec, 0, out_l);
        lhdc_dec_ret_t ret1 = lhdc_dec_ch1_join();      /* barrier */
        if (ret != LHDC_DEC_OK) return ret;
        if (ret1 != LHDC_DEC_OK) {
            for (int n = 0; n < samples_per_ch; n++) out_r[n] = 0.0f;
        }
    } else
#endif
    {
        ret = lhdc_dec_decode_channel_autosel(dec, 0, out_l);
        if (ret != LHDC_DEC_OK) {
            return ret;
        }
        if (do_stereo) {
            ret = lhdc_dec_decode_channel_autosel(dec, 1, out_r);
            if (ret != LHDC_DEC_OK) {
                /* Fall back to silence on this channel if ch1 fails. */
                for (int n = 0; n < samples_per_ch; n++) out_r[n] = 0.0f;
            }
        }
    }

    /* Output scale: the encoder works in a fixed 24-bit internal scale; combined
     * with the IMDCT/overlap reconstruction gain of 2 the total is
     * 2^(bit_depth-23). Brings output to the original level and keeps 16/24-bit
     * levels equal. */
    float sc = ldexpf(1.0f, out_bit_depth - 23) * lhdc_dec_rate_norm(mdct_size)
             * lhdc_dec_level_cal(dec->header.sample_rate, (int)dec->header.bit_depth);
    float clip_lim = (bytes_per_sample == 2) ? 32767.0f
                   : (bytes_per_sample == 3) ? 8388607.0f : 2147483647.0f;

    /*
     * Decode-garbage concealment. A correctly decoded frame can never exceed the
     * source's full-scale range, so a peak well past it (>2x) means this frame's
     * entropy/mantissa decode desynced and produced huge bogus coefficients
     * (heard as a loud pop/beep, ~3% of frames on busy passages — the mantissa
     * shift M=q<<s amplifies a desync into tens of millions). Conceal such frames
     * with silence and reset the overlap so the glitch can't smear into the next
     * frame. A 5ms mute is far less audible than the beep. (Proper fix = the
     * upstream entropy desync; tracked separately.) */
    float frame_peak = 0.0f;
    int clip_cnt = 0;
    for (int n = 0; n < samples_per_ch; n++) {
        float l = out_l[n];
        float r = do_stereo ? out_r[n] : out_l[n];
        float al = (l < 0 ? -l : l) * sc, ar = (r < 0 ? -r : r) * sc;
        if (al > frame_peak) frame_peak = al;
        if (ar > frame_peak) frame_peak = ar;
        if (al > clip_lim) clip_cnt++;
        if (ar > clip_lim) clip_cnt++;
    }
    int conceal = (frame_peak > 2.0f * clip_lim);
#if defined(LHDC_HOST_BUILD)
    { extern int g_pkframe; float p0 = 0.0f, p1 = 0.0f;
        for (int n = 0; n < samples_per_ch; n++) {
            float a0 = (out_l[n] < 0 ? -out_l[n] : out_l[n]) * sc;
            float a1 = (out_r[n] < 0 ? -out_r[n] : out_r[n]) * sc;
            if (a0 > p0) p0 = a0; if (a1 > p1) p1 = a1;
        }
        if (getenv("LHDC_PKALL") || conceal)
            printf("[PK] frame=%d ch0peak=%.0f ch1peak=%.0f %s\n",
                   g_pkframe, p0, p1, conceal ? (p1>2*clip_lim?"CONCEAL-ch1":"CONCEAL-ch0") : "");
        g_pkframe++;
    }
#endif
    if (conceal) {
        /* Zero the FULL overlap buffers (H = alloc_mdct_size/2 floats each), not
         * sizeof(float*) — the latter only cleared 8 bytes, leaving stale overlap
         * to leak into the next frame on a concealed error. */
        size_t ov_bytes = (size_t)(dec->alloc_mdct_size / 2) * sizeof(float);
        memset(dec->overlap_buf[0], 0, ov_bytes);
        if (channels >= 2) memset(dec->overlap_buf[1], 0, ov_bytes);
    }

    /* Interleave to output (l/r already include the bit-depth scale `sc`). */
    #define LHDC_OUT_SAMPLE(dst, fval, lo, hi) do { \
        float _s = (fval); if (_s > (hi)) _s = (hi); if (_s < (lo)) _s = (lo); \
        (dst) = (int32_t)_s; } while (0)
    for (int n = 0; n < samples_per_ch; n++) {
        float l, r;
        if (conceal)        { l = r = 0.0f; }
        else if (do_stereo) { l = out_l[n] * sc; r = out_r[n] * sc; }
        else                { l = r = out_l[n] * sc; }
        int32_t lv, rv;
        if (bytes_per_sample == 2) {
            LHDC_OUT_SAMPLE(lv, l, -32768.0f, 32767.0f);
            LHDC_OUT_SAMPLE(rv, r, -32768.0f, 32767.0f);
            int16_t *o = (int16_t *)out_pcm;
            o[n * channels + 0] = (int16_t)lv;
            if (channels >= 2) o[n * channels + 1] = (int16_t)rv;
        } else if (bytes_per_sample == 3) {
            /*
             * 24-bit PCM output for ESP32 I2S/DMA.
             *
             * Old behavior packed samples as 3 bytes:
             *   L0 L1 L2 R0 R1 R2 ...
             * That is valid packed PCM, but it is wrong for the common ESP32
             * I2S configuration that uses 24-bit audio in 32-bit slots. Feeding
             * 3-byte packed audio to a 32-bit-slot DMA stream shifts the channel
             * framing and can sound like low-frequency leakage / motorboating.
             *
             * New behavior writes interleaved 32-bit slots:
             *   int32 L, int32 R, int32 L, int32 R ...
             * The 24 valid bits are left-justified by default: sample << 8.
             * Use LHDC_DEC_24BIT_I2S_RIGHT_JUSTIFIED=1 if your I2S path expects
             * right-justified 24-bit values in the low 24 bits instead.
             *
             * IMPORTANT for caller:
             *   output bytes = samples_per_ch * channels * sizeof(int32_t)
             * not samples_per_ch * channels * 3.
             */
            LHDC_OUT_SAMPLE(lv, l, -8388608.0f, 8388607.0f);
            LHDC_OUT_SAMPLE(rv, r, -8388608.0f, 8388607.0f);
            int32_t *o = (int32_t *)out_pcm;
#if defined(LHDC_DEC_24BIT_I2S_RIGHT_JUSTIFIED) && LHDC_DEC_24BIT_I2S_RIGHT_JUSTIFIED
            o[n * channels + 0] = lv;
            if (channels >= 2) o[n * channels + 1] = rv;
#else
            o[n * channels + 0] = (int32_t)(lv << 8);
            if (channels >= 2) o[n * channels + 1] = (int32_t)(rv << 8);
#endif
        } else if (bytes_per_sample == 4) {
            LHDC_OUT_SAMPLE(lv, l, -2147483648.0f, 2147483647.0f);
            LHDC_OUT_SAMPLE(rv, r, -2147483648.0f, 2147483647.0f);
            int32_t *o = (int32_t *)out_pcm;
            o[n * channels + 0] = lv;
            if (channels >= 2) o[n * channels + 1] = rv;
        }
    }
    #undef LHDC_OUT_SAMPLE

    /* Stats: concealed (garbage) frames and clipping, rate-limited. */
    {
        static uint32_t s_conceal = 0, s_clip_frames = 0, s_total = 0, s_rl = 0;
        s_total++;
        if (conceal)      s_conceal++;
        if (clip_cnt > 0) s_clip_frames++;
        if (conceal && (s_rl++ % 16) == 0) {
            ESP_LOGW("LHDCV5_DEC",
                     "CONCEAL frame#%u: peak=%.0f (lim=%.0f) | concealed=%u/%u clipframes=%u",
                     (unsigned)dec->frame_index, frame_peak, clip_lim,
                     s_conceal, s_total, s_clip_frames);
        }
    }

    /* Update output ([u16 LE] header + payload bytes consumed). */
    *consumed = frame_consumed;
    *generated = (uint32_t)samples_per_ch;
    dec->frame_index++;

    if (g_lhdc_trace > 0) {
        g_lhdc_trace--;
        ESP_LOGI("LHDCV5_DEC", "FRAME ok: consumed=%u frame_bytes=%d gen=%u ch=%d",
                 (unsigned)frame_consumed, (int)dec->header.frame_bytes,
                 (unsigned)samples_per_ch, channels);
    }

    /* Fill frame info if requested */
    if (info) {
        info->frame_index        = dec->frame_index - 1;
        info->encoded_frame_bytes = dec->header.frame_bytes;
        info->samples_per_channel = dec->header.samples_per_channel;
        info->channels           = dec->header.channels;
        info->sample_rate        = dec->header.sample_rate;
        info->bit_depth          = (uint8_t)out_bit_depth;   /* emitted container width (24->32) */
        info->frame_duration_ms  = dec->header.frame_duration_ms;
        info->version            = dec->header.version;
        info->ext_func_flags     = dec->header.ext_func_flags;
        info->target_bitrate     = dec->header.target_bitrate;
    }

    /* Frame decode latency vs the frame's own real-time budget (5 ms normally,
     * 2.5 ms low-latency). Counters only -- see lhdc_dec_latency_stats(). */
    {
        uint32_t us = (uint32_t)(LHDC_NOW_US() - t_frame0);
        uint32_t dur = dec->header.frame_duration_ms ? dec->header.frame_duration_ms : 5;
        s_lat_budget = dur * 1000u;
        s_lat_frames++;
        s_lat_sum_us += us;
        if (us > s_lat_max_us) { s_lat_max_us = us; s_lat_bytes = (uint32_t)dec->header.frame_bytes; }
        if (us > s_lat_budget) s_lat_over++;
    }

    return LHDC_DEC_OK;
}

/* Destroy a decoder instance: frees the rate-sized work buffers the decoder
 * allocated itself. The caller still owns `workspace` (struct storage) and
 * frees that separately. Safe to call more than once. */
void lhdc_dec_deinit(lhdc_decoder_t *dec)
{
    lhdc_dec_free_tails(dec);
}

void lhdc_dec_flush(lhdc_decoder_t *dec)
{
    if (!dec) return;

    /* Clear the FULL overlap buffers (H = alloc_mdct_size/2 floats each), not
     * sizeof(float*) — the latter cleared only 4-8 bytes, leaving stale overlap
     * from a previous stream to leak into the first frame after a flush/reset
     * (codec switch / track start) -> a cold-start click. */
    size_t ov_bytes = (size_t)(dec->alloc_mdct_size / 2) * sizeof(float);
    if (ov_bytes == 0) ov_bytes = (size_t)(LHDC_DEC_MAX_MDCT_SIZE / 2) * sizeof(float);
    for (int ch = 0; ch < LHDC_DEC_MAX_CHANNELS; ch++) {
        if (dec->overlap_buf[ch]) memset(dec->overlap_buf[ch], 0, ov_bytes);
    }
}

void lhdc_dec_reset(lhdc_decoder_t *dec)
{
    if (!dec) return;

    lhdc_dec_flush(dec);
    dec->header_parsed = 0;
    dec->frame_index = 0;
    memset(&dec->header, 0, sizeof(dec->header));
    for (int s = 0; s < LHDC_NSLOTS; s++) {
        memset(&dec->slot[s].sns_params, 0, sizeof(dec->slot[s].sns_params));
    }
}

lhdc_dec_ret_t lhdc_dec_get_config(lhdc_decoder_t *dec, lhdc_dec_config_t *config)
{
    if (!dec || !config) {
        return LHDC_DEC_INVALID_PARAM;
    }

    if (dec->header_parsed) {
        config->sample_rate    = (lhdc_dec_sample_rate_t)dec->header.sample_rate;
        config->bit_depth      = (lhdc_dec_bitdepth_t)dec->header.bit_depth;
        config->frame_duration = (lhdc_dec_frame_duration_t)dec->header.frame_duration_ms;
        config->channels       = dec->header.channels;
        config->max_frame_bytes = dec->header.frame_bytes;
        config->lossless_enable = dec->header.lossless;
    } else {
        memcpy(config, &dec->config, sizeof(*config));
    }

    return LHDC_DEC_OK;
}

const char *lhdc_dec_strerror(lhdc_dec_ret_t ret)
{
    switch (ret) {
        case LHDC_DEC_OK:                  return "Success";
        case LHDC_DEC_ERROR:               return "Generic error";
        case LHDC_DEC_INVALID_PARAM:       return "Invalid parameter";
        case LHDC_DEC_INVALID_HANDLE:      return "Invalid handle";
        case LHDC_DEC_NOT_INITIALIZED:     return "Decoder not initialized";
        case LHDC_DEC_BUF_NOT_ENOUGH:      return "Output buffer not large enough";
        case LHDC_DEC_BITSTREAM_ERROR:     return "Bitstream error";
        case LHDC_DEC_UNSUPPORTED_VERSION: return "Unsupported LHDC version";
        case LHDC_DEC_UNSUPPORTED_SR:      return "Unsupported sample rate";
        case LHDC_DEC_UNSUPPORTED_FORMAT:  return "Unsupported format";
        case LHDC_DEC_NEED_MORE_DATA:      return "Need more input data";
        default:                           return "Unknown error";
    }
}
