#include "lhdc_sns_synth.h"
#include "lhdc_diag_config.h"
#include "lhdc_bit_reader.h"
#include "lhdc_tables.h"
#include <math.h>
#include <string.h>
#if defined(LHDC_HOST_BUILD)
#include <stdio.h>
#include <stdlib.h>
#define LHDC_HOT
#else
#include "esp_attr.h"
#define LHDC_HOT IRAM_ATTR
#endif

/*
 * LHDC V5 SNS synthesis — reverse-engineered and validated bit-exact against
 * liblhdcv5.so (the reconstructed scalefactors re-encode to the frame's exact
 * SNS side bits via the real adsq_enc).
 *
 * Per-band scalefactors are coded by adsq_enc: an adaptive-step DPCM over the
 * transmitted 1-bit-per-band side info, seeded sf[0]=0, state=8.
 *   for band i>=1, bit = side[i-1]:
 *     if i>=2: if bit==prev: run++, state=min(state+run,63)
 *              else:         state=max((3*state+2)>>2,0), run=1
 *     sf[i] = sf[i-1] + STEP[bit][state]
 *   (state update happens BEFORE the step.)
 *
 * sns_apply gain per band:  idx = (sf>=0) ? sf>>4 : (sf+0xa0f)>>4
 *                           gain = POW2_MANT[idx] * (sf>=0 ? 2^-20 : 2^-30)
 * POW2_MANT[i] = round(2^(20 + i/16)). The decoder MULTIPLIES band lines by gain.
 */

/* step_size @ rodata 0x77b4: 2x64 int32, row0 positive, row1 = negation. */
static const int32_t LHDC_SNS_STEP_POS[64] = {
    6,13,19,26,32,38,45,51,58,64,77,90,102,115,128,141,154,166,186,205,224,243,
    262,282,301,326,352,378,403,429,461,493,525,557,595,634,672,717,762,813,870,
    934,1005,1082,1165,1254,1350,1453,1562,1677,1798,1926,2061,2202,2349,2502,
    2662,2829,3002,3187,3386,3603,3840,4096
};

/* POW2 mantissa table @ rodata 0x8030 (161 int32), table[i] = round(2^(20+i/16)). */
static const int32_t LHDC_POW2_MANT[161] = {
    1048576, 1095000, 1143480, 1194106, 1246974, 1302182, 1359834, 1420039,
    1482910, 1548564, 1617125, 1688721, 1763487, 1841563, 1923096, 2008239,
    2097152, 2190000, 2286960, 2388212, 2493948, 2604364, 2719669, 2840079,
    2965820, 3097128, 3234250, 3377443, 3526975, 3683127, 3846193, 4016479,
    4194304, 4380001, 4573920, 4776425, 4987896, 5208729, 5439339, 5680159,
    5931641, 6194257, 6468501, 6754886, 7053950, 7366255, 7692387, 8032958,
    8388608, 8760003, 9147841, 9552851, 9975792, 10417458, 10878678, 11360318,
    11863283, 12388515, 12937002, 13509772, 14107900, 14732510, 15384774, 16065917,
    16777216, 17520006, 18295683, 19105702, 19951584, 20834916, 21757357, 22720637,
    23726566, 24777031, 25874004, 27019544, 28215801, 29465021, 30769549, 32131834,
    33554432, 35040013, 36591367, 38211405, 39903169, 41669833, 43514714, 45441275,
    47453132, 49554062, 51748008, 54039088, 56431603, 58930043, 61539099, 64263668,
    67108864, 70080027, 73182735, 76422811, 79806338, 83339667, 87029429, 90882551,
    94906265, 99108124, 103496016, 108078176, 112863206, 117860087, 123078199, 128527336,
    134217728, 140160054, 146365470, 152845623, 159612677, 166679334, 174058858, 181765102,
    189812531, 198216249, 206992033, 216156353, 225726412, 235720174, 246156398, 257054673,
    268435456, 280320108, 292730940, 305691246, 319225354, 333358668, 348117717, 363530205,
    379625062, 396432499, 413984066, 432312706, 451452825, 471440349, 492312796, 514109346,
    536870912, 560640217, 585461880, 611382492, 638450708, 666717336, 696235434, 727060410,
    759250124, 792864999, 827968132, 864625413, 902905650, 942880699, 984625593, 1028218693,
    1073741824
};

static inline int32_t sns_step(int bit, int state)
{
    if (state < 0) state = 0;
    if (state > 63) state = 63;
    int32_t s = LHDC_SNS_STEP_POS[state];
    return bit ? -s : s;
}

/* Per-band sns_apply gain (the value the decoder multiplies band lines by). */
static float sns_band_gain(int32_t sf)
{
    int idx;
    float scale;
    if (sf >= 0) { idx = sf >> 4;            scale = 0x1p-20f; }   /* 2^-20, exact float, no divide */
    else         { idx = (sf + 0xa0f) >> 4;  scale = 0x1p-30f; }   /* 2^-30, exact float, no divide */
    if (idx < 0) idx = 0;
    if (idx >= (int)(sizeof(LHDC_POW2_MANT) / sizeof(int32_t)))
        idx = (int)(sizeof(LHDC_POW2_MANT) / sizeof(int32_t)) - 1;
    /* LHDC_POW2_MANT values are < 2^24 so the float cast is exact. */
    return (float)LHDC_POW2_MANT[idx] * scale;
}

/* Reciprocal of sns_band_gain WITHOUT a float divide.
 *
 * The Xtensa LX6 FPU has no divide instruction, so `1.0f / g` compiles to a
 * SOFTWARE __divsf3 call (~80 cycles) -- and it ran once per band, per channel,
 * per frame (32 x 2 x 200 fps = ~12.8k/s). Since sns_band_gain returns
 * mant * 2^-k with an EXACT power-of-two scale, 1/g = (1/mant) * 2^k, and scaling
 * by a power of two only changes the exponent (never the mantissa). So a
 * precomputed 1/mant table yields a BIT-IDENTICAL result using a single FPU
 * multiply. The table costs 644 B of .bss and is built once (cold). */
static float s_pow2_mant_recip[sizeof(LHDC_POW2_MANT) / sizeof(int32_t)];
static int   s_pow2_mant_recip_built = 0;

static float sns_band_gain_recip(int32_t sf)
{
    const int n = (int)(sizeof(LHDC_POW2_MANT) / sizeof(int32_t));
    int idx;
    float inv_scale;
    if (sf >= 0) { idx = sf >> 4;            inv_scale = 0x1p20f; }  /* 1 / 2^-20 */
    else         { idx = (sf + 0xa0f) >> 4;  inv_scale = 0x1p30f; }  /* 1 / 2^-30 */
    if (idx < 0) idx = 0;
    if (idx >= n) idx = n - 1;
    if (!s_pow2_mant_recip_built) {
        for (int i = 0; i < n; i++) s_pow2_mant_recip[i] = 1.0f / (float)LHDC_POW2_MANT[i];
        s_pow2_mant_recip_built = 1;
    }
    return s_pow2_mant_recip[idx] * inv_scale;
}

/* floor division matching Python's // (for negative numerators). 32-bit: the
 * only caller passes a sum of <=32 small scale-factors and n<=32, which fit in
 * int32 with huge margin, so this uses the ESP32's HARDWARE 32-bit divide
 * instead of the software __divdi3 64-bit routine. */
static int sns_floordiv(int32_t a, int32_t b)
{
    int32_t q = a / b, r = a % b;
    if (r != 0 && ((a < 0) != (b < 0))) q--;
    return (int)q;
}

/*
 * sns_encode post-processing (0xd5b64..0xd5d8c), validated bit-exact in the
 * Python reference (tools/sns_synth.py post_smooth): mean removal, then a 4-tap
 * sliding smoothing, then clamp to [-2560, 1024]. The decoder MUST apply this to
 * the adsq reconstruction before using sf as the SNS envelope — otherwise the sf
 * has a large DC offset and the wrong shape, leaving the spectrum whitened
 * (audible as noise with the right rhythm but no pitch).
 */
/* div_euclid for a positive divisor (floors toward -inf), matching Rust's
 * i32::div_euclid used by the reference encoder's div_round. */
static inline int32_t sns_div_euclid_pos(int32_t a, int32_t b)
{
    int32_t q = a / b, r = a % b;
    if (r < 0) q -= 1;   /* b > 0 */
    return q;
}
/* div_round(x, 4) = (x + 2).div_euclid(4)  (reference math::div_round).
 * div_euclid by a POSITIVE power of two is exactly floor(x / 2^k), and an
 * ARITHMETIC right shift on two's complement already floors toward -inf for
 * negative values -- so this is a single sra, no divide and no remainder.
 * Bit-identical to sns_div_euclid_pos(x + 2, 4) for every int32 input. */
static inline int32_t sns_div_round4(int32_t x)
{
    return (x + 2) >> 2;
}
/* div::<3>(x) = (x * floor(2^31/3)) >> 31  (reference math::div, N=3). This is
 * (x-1)/3 for positive x and (x-2)/3 for negative x, NOT ordinary /3. */
static inline int32_t sns_div3(int32_t x)
{
    return (int32_t)(((int64_t)x * 715827882) >> 31);
}

/*
 * SNS envelope reconstruction. Ported bit-exactly from the reference LHDC-V5
 * encoder (Android lhdcv5 Rust, src/enc/process.rs `lhdc_enc_freq_shift`,
 * lines 521-538 + `moving_average`). `sf` on entry is the DPCM/adsq
 * reconstruction (== the encoder's post-`jump_adust` `se`, with sf[0]=0, which
 * our lhdc_sns_adsq_inverse already reproduces bit-exactly). This routine
 * reproduces exactly what the encoder applies to the spectrum:
 *   1. mean-remove: se[0] = -(sum(se[1..n]) * num_inv >> 31);  se[i] += se[0]
 *      (num_inv = segment_num_inv = round(2^31/n): 67108864 for 32, 89478485 for 24)
 *   2. asymmetric WINDOW=4 moving average (window is one below + two above; the
 *      leftmost keeps itself, the last two use widths 3 and 1)
 *   3. clamp to [-2560, 1024], THEN negate (this order matters)
 *
 * The previous implementation used a symmetric 4-tap [1/4,1/4,1/4,1/4] average,
 * a rounded mean, and negate-before-clamp -- an approximation that mis-shaped
 * the low-frequency SNS envelope and distorted between-bin sub-bass at 96k/192k.
 */
static void sns_post_smooth(int32_t *sf, int n)
{
    (void)sns_floordiv;
    (void)sns_div_euclid_pos;   /* kept as the documented reference for div_round4 */
    if (n < 4) return;   /* real configs use 24 or 32 segments */
    int64_t num_inv = (n == 24) ? 89478485 : 67108864;   /* round(2^31 / n) */

    /* 1. mean-remove (exact). sf[0] is 0 on entry. */
    int32_t sum = 0;
    for (int i = 1; i < n; i++) sum += sf[i];
    int32_t s0 = (int32_t)(((int64_t)(-sum) * num_inv) >> 31);
    sf[0] = s0;
    for (int i = 1; i < n; i++) sf[i] += s0;

    /* 2. asymmetric WINDOW=4 moving average -> out. */
    int32_t out[LHDC_DEC_MAX_SFB];
    out[0] = sf[0];
    int32_t total = sf[0] + sf[1] + sf[2] + sf[3];
    out[1] = sns_div_round4(total);
    int i = 2;
    while (i < n - 2) {
        total += sf[i + 2] - sf[i - 2];
        out[i] = sns_div_round4(total);
        i++;
    }
    total -= sf[i - 2];
    out[i] = sns_div3(total);
    i++;
    out[i] = sf[i - 1];   /* = sf[n-2] */

    /* 3. clamp [-2560, 1024], THEN negate. */
    for (int k = 0; k < n; k++) {
        int32_t v = out[k];
        if (v < -2560) v = -2560;
        if (v > 1024) v = 1024;
        sf[k] = -v;
    }
}

/* Reconstruct per-band scalefactors from the SNS side bits (adsq inverse).
 * The initial adaptive-step state is the transmitted sns_mode (NOT a constant 8):
 * verified against liblhdcv5.so adsq_enc — for a frame with sns_mode=11 the first
 * step is STEP[11]=90, for sns_mode=8 it is STEP[8]=58. Hardcoding 8 only worked
 * for sns_mode=8 frames (e.g. the 24-bit test) and corrupted every other config. */
static void sns_adsq_inverse(const uint8_t *side_bits, int32_t *sf, int num_sfb,
                              int sns_mode)
{
    sf[0] = 0;
    int state = sns_mode;
    int run = 1;
    int prev = -1;
    for (int i = 1; i < num_sfb; i++) {
        int bit = side_bits[i - 1] & 1;
        if (i >= 2) {
            if (bit == prev) {
                run++;
                state = state + run;
                if (state > 63) state = 63;
            } else {
                state = (3 * state + 2) >> 2;
                if (state < 0) state = 0;
                run = 1;
            }
        }
        sf[i] = sf[i - 1] + sns_step(bit, state);
        prev = bit;
    }
}

LHDC_HOT void lhdc_sns_synth_apply(float *spectrum,
                           const lhdc_dec_sns_params_t *params,
                           int mdct_size,
                           const uint16_t *band_off,
                           const uint16_t *band_scale,
                           int num_sfb)
{
    (void)band_scale;
#if defined(LHDC_SNS_BYPASS)
    (void)params; (void)band_off; (void)num_sfb; (void)mdct_size; (void)spectrum;
    return;   /* calibration: leave spectrum untouched to isolate dequant/IMDCT scale */
#else
    int sns_dir = g_lhdc_diag_sns_mode;   /* 0=divide, 1=multiply, 2=bypass */
#if defined(LHDC_HOST_BUILD)
    { const char *e = getenv("LHDC_SNS"); if (e) sns_dir = atoi(e); }
#endif
    if (sns_dir == 2) return;
    for (int sfb = 0; sfb < num_sfb; sfb++) {
        int start = band_off[sfb];
        int end = (sfb + 1 < num_sfb) ? band_off[sfb + 1] : mdct_size;
        if (end > mdct_size) end = mdct_size;
        int32_t sf = params->scale_factors[sfb];
        float g;
#if defined(LHDC_GAIN_NEG)
        g = powf(2.0f, -(float)sf / 16.0f);
#elif defined(LHDC_GAIN_POS)
        g = powf(2.0f,  (float)sf / 16.0f);
#else
        if (sns_dir == 1) { g = sns_band_gain(sf); }        /* multiply */
        else              { g = sns_band_gain_recip(sf); }  /* divide-POW2, no __divsf3 */
#endif
        for (int k = start; k < end; k++) spectrum[k] *= g;
    }
#endif
}

/*
 * Decode the SNS side info for one channel. The bit reader must be positioned
 * at the 4-bit sns_mode field. Reconstructs per-band scalefactors via adsq.
 */
lhdc_dec_ret_t lhdc_sns_decode_params(lhdc_dec_bit_reader_t *br,
                                        lhdc_dec_sns_params_t *params,
                                        int num_sfb)
{
    if (num_sfb < 1 || num_sfb > LHDC_DEC_MAX_SFB) {
        return LHDC_DEC_BITSTREAM_ERROR;
    }

    int sns_mode = (int)lhdc_bit_reader_read(br, 4) + 8;

    uint8_t side_bits[LHDC_DEC_MAX_SFB];
    for (int i = 0; i < num_sfb - 1; i++) {
        side_bits[i] = (uint8_t)lhdc_bit_reader_read(br, 1);
    }

    /*
     * sns_mode == 23 (the 4-bit field = 15) is the encoder's FLAT-SNS escape
     * (sns_encode @0xd5ad8): when a channel's energy is low/flat it zeroes the
     * side bits, sets mode 23 and SKIPS adsq, so the transmitted envelope is
     * flat. The decoder must use sf = 0 (gain 1, no whitening) — running adsq
     * here would synthesize a bogus envelope (audible crackle / wrong band).
     */
    if (sns_mode == 23) {
        memset(params->scale_factors, 0, sizeof(params->scale_factors));
        memset(params->noise_level, 0, sizeof(params->noise_level));
        params->num_sfb = (uint8_t)num_sfb;
        params->sns_mode = (int32_t)sns_mode;
        return LHDC_DEC_OK;
    }

    sns_adsq_inverse(side_bits, params->scale_factors, num_sfb, sns_mode);
#if defined(LHDC_HOST_BUILD)
    extern volatile int g_lhdc_trace;
    if (g_lhdc_trace > 0) {
        printf("[SNS] side:");
        for (int i = 0; i < num_sfb - 1; i++) printf("%d", side_bits[i]);
        printf("\n[SNS] adsq_raw:");
        for (int i = 0; i < num_sfb; i++) printf(" %d", (int)params->scale_factors[i]);
        printf("\n");
    }
#endif
#ifndef LHDC_SNS_RAW
    sns_post_smooth(params->scale_factors, num_sfb);
#if defined(LHDC_HOST_BUILD)
    if (g_lhdc_trace > 0) {
        printf("[SNS] post_smooth:");
        for (int i = 0; i < num_sfb; i++) printf(" %d", (int)params->scale_factors[i]);
        printf("\n");
    }
#endif
#endif

    memset(params->noise_level, 0, sizeof(params->noise_level));
    params->num_sfb = (uint8_t)num_sfb;
    params->sns_mode = (int32_t)sns_mode;
    return LHDC_DEC_OK;
}
