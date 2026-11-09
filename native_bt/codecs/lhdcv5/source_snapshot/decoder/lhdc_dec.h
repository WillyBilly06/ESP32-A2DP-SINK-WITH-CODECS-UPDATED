#ifndef LHDC_DEC_H
#define LHDC_DEC_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LHDC_DEC_VERSION_MAJOR 1
#define LHDC_DEC_VERSION_MINOR 0
#define LHDC_DEC_VERSION_PATCH 0

/* Diagnostic one-shot trace flag (see lhdc_dec.c). Set to 1 to log the first
 * decoded frame's leading-section values; the decoder clears it after logging. */
extern volatile int g_lhdc_trace;

/* Return codes */
typedef enum {
    LHDC_DEC_OK                    =  0,
    LHDC_DEC_ERROR                 = -1,
    LHDC_DEC_INVALID_PARAM         = -2,
    LHDC_DEC_INVALID_HANDLE        = -3,
    LHDC_DEC_NOT_INITIALIZED       = -4,
    LHDC_DEC_BUF_NOT_ENOUGH        = -5,
    LHDC_DEC_BITSTREAM_ERROR       = -6,
    LHDC_DEC_UNSUPPORTED_VERSION   = -7,
    LHDC_DEC_UNSUPPORTED_SR        = -8,
    LHDC_DEC_UNSUPPORTED_FORMAT    = -9,
    LHDC_DEC_NEED_MORE_DATA        = -10,
} lhdc_dec_ret_t;

/* Sample rates */
typedef enum {
    LHDC_DEC_SR_44100  = 44100,
    LHDC_DEC_SR_48000  = 48000,
    LHDC_DEC_SR_96000  = 96000,
    LHDC_DEC_SR_192000 = 192000,
} lhdc_dec_sample_rate_t;

/* Bit depths */
typedef enum {
    LHDC_DEC_BITDEPTH_16 = 16,
    LHDC_DEC_BITDEPTH_24 = 24,
    LHDC_DEC_BITDEPTH_32 = 32,
} lhdc_dec_bitdepth_t;

/* Frame duration */
typedef enum {
    LHDC_DEC_FRAME_5MS  = 5,
    LHDC_DEC_FRAME_7P5MS = 7,
    LHDC_DEC_FRAME_10MS = 10,
} lhdc_dec_frame_duration_t;

/* Version */
typedef enum {
    LHDC_DEC_VERSION_1 = 1,
} lhdc_dec_version_t;

/* Extra functions (from bitstream) */
typedef enum {
    LHDC_DEC_EXT_FUNC_NONE  = 0,
    LHDC_DEC_EXT_FUNC_AR    = (1 << 0),
    LHDC_DEC_EXT_FUNC_JAS   = (1 << 1),
    LHDC_DEC_EXT_FUNC_LARC  = (1 << 2),
    LHDC_DEC_EXT_FUNC_META  = (1 << 3),
} lhdc_dec_ext_func_t;

/* Decoder configuration */
typedef struct {
    lhdc_dec_sample_rate_t   sample_rate;
    lhdc_dec_bitdepth_t      bit_depth;
    lhdc_dec_frame_duration_t frame_duration;
    uint8_t                  channels;
    uint32_t                 max_frame_bytes;
    uint8_t                  lossless_enable;
} lhdc_dec_config_t;

/* Frame information */
typedef struct {
    uint32_t  frame_index;
    uint32_t  encoded_frame_bytes;
    uint32_t  samples_per_channel;
    uint8_t   channels;
    uint32_t  sample_rate;
    uint8_t   bit_depth;
    uint8_t   frame_duration_ms;
    uint8_t   version;
    uint32_t  ext_func_flags;
    uint32_t  target_bitrate;
} lhdc_dec_frame_info_t;

/* Opaque decoder handle */
typedef struct lhdc_decoder_t lhdc_decoder_t;

/*
 * Get the required workspace size. With the split-workspace design the decoder
 * allocates its rate-sized work buffers itself (each <= ~7.7 KB), so this only
 * covers the rate-independent struct. sample_rate/frame_duration are accepted
 * for API compatibility and ignored.
 */
size_t lhdc_dec_get_workspace_size(uint32_t sample_rate, uint8_t frame_duration);

/*
 * Initialize a decoder instance.
 * workspace: pre-allocated struct storage of size lhdc_dec_get_workspace_size(),
 *            ZERO-INITIALIZED before the first call (lhdc_dec_init frees the
 *            previous rate's work buffers, so it must not see garbage pointers)
 * config:    decoder configuration (may be NULL for auto-detect from bitstream)
 * Returns a decoder handle, or NULL on failure.
 */
lhdc_decoder_t *lhdc_dec_init(void *workspace, const lhdc_dec_config_t *config);

/*
 * Destroy a decoder instance: frees the rate-sized work buffers that the
 * decoder allocated itself (split-workspace design). The caller still owns
 * the `workspace` block (struct storage) and frees it separately.
 */
void lhdc_dec_deinit(lhdc_decoder_t *dec);

/*
 * Frame-latency telemetry. An LHDC V5 frame is 5 ms of audio (2.5 ms in
 * low-latency mode) and holds BOTH channels, so a full decode_frame call must
 * finish inside `budget_us` or the output ring starves. The decoder only keeps
 * counters (no logging in the audio path); call this from a low-priority task to
 * read and reset them.
 *   frames      - frames timed since the last reset
 *   avg_us/max_us - mean and worst full-frame decode time
 *   over        - frames that exceeded budget_us
 *   budget_us   - frame_duration_ms * 1000 for the running config
 *   worst_bytes - encoded size of the worst frame (i.e. how hot the bitrate was)
 */
typedef struct {
    uint32_t frames;
    uint32_t avg_us;
    uint32_t max_us;
    uint32_t over;
    uint32_t budget_us;
    uint32_t worst_bytes;
} lhdc_dec_latency_t;

void lhdc_dec_latency_stats(lhdc_dec_latency_t *out, int reset);

/*
 * Decode one frame.
 * dec:       decoder handle
 * in_data:   input encoded bitstream data
 * in_bytes:  size of input data in bytes
 * out_pcm:   output PCM buffer (interleaved, native endian)
 * out_samples: max samples (per channel) that out_pcm can hold
 * consumed:  [out] number of bytes consumed from in_data
 * generated: [out] number of samples (per channel) written to out_pcm
 * info:      [out] frame information (may be NULL)
 */
lhdc_dec_ret_t lhdc_dec_decode_frame(
    lhdc_decoder_t *dec,
    const uint8_t  *in_data,
    size_t          in_bytes,
    void           *out_pcm,
    uint32_t        out_samples,
    size_t         *consumed,
    uint32_t       *generated,
    lhdc_dec_frame_info_t *info);

/*
 * Flush internal state (for seeking / discontinuities).
 */
void lhdc_dec_flush(lhdc_decoder_t *dec);

/*
 * Reset decoder to initial state.
 */
void lhdc_dec_reset(lhdc_decoder_t *dec);

/*
 * Get decoder configuration (useful after auto-detect).
 */
lhdc_dec_ret_t lhdc_dec_get_config(lhdc_decoder_t *dec, lhdc_dec_config_t *config);

/*
 * Get string description of a return code.
 */
const char *lhdc_dec_strerror(lhdc_dec_ret_t ret);

/* Free the module-static KBD analysis window (call at decoder teardown). */
void lhdc_dec_free_window(void);

/* Release the second decode slot's scratch block (dual-core channel decode on the
 * classic ESP32). No-op on single-core/host builds. Call at decoder teardown. */
void lhdc_dec_free_slot_scratch(void);

#ifdef __cplusplus
}
#endif

#endif /* LHDC_DEC_H */
