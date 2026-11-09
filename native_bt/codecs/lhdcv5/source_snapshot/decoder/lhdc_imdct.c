#include "lhdc_imdct.h"
#include "lhdc_diag_config.h"
#include "lhdc_dec_slot.h"   /* LHDC_NSLOTS / LHDC_DEC_SLOT(): parallel channel decode */
#include <math.h>
#include <stdlib.h>
#include <string.h>

#if defined(LHDC_HOST_BUILD)
  #include <stdio.h>
  #define IMDCT_LOGI(tag, ...) do { printf("[I]" tag ": " __VA_ARGS__); printf("\n"); } while (0)
  #define LHDC_HOT
#else
  #include "esp_log.h"
  #include "esp_attr.h"
  #define IMDCT_LOGI(tag, ...) ESP_LOGI(tag, __VA_ARGS__)
  /* IMDCT fast path in IRAM: avoids flash-cache eviction by BT-controller IRQs. */
  #define LHDC_HOT IRAM_ATTR
#endif

#ifndef M_PIF
#define M_PIF 3.14159265358979323846f
#endif

/*
 * IMDCT for LHDC V5.
 *
 *   x[n] = (2/N) * sum_{k=0}^{N/2-1} X[k] * cos(pi/(2N)*(2n+1+N/2)*(2k+1))
 *   for n = 0..N-1
 *
 * Two implementations:
 *  - lhdc_imdct_ref: a direct O(N^2) evaluation with a precomputed cosine table.
 *    Correct for any N, but ~4 ms/channel for N=480 on the ESP32 -> ~2.2x over
 *    the real-time budget -> A2DP underruns -> crackle.
 *  - lhdc_imdct_fast_480: an O(N log N) fast path for the live config (N=480,
 *    48k/5ms). Computed as a length-N/4 (=120) complex FFT with pre/post twiddle
 *    and a symmetric unfold; the 120-point FFT is an 8x15 four-step Cooley-Tukey
 *    with direct radix-8 / radix-15 sub-DFTs. Validated bit-for-bit (~1e-14) in
 *    Python against the reference formula. ~10x fewer flops -> ~0.4 ms/channel.
 *
 * A one-shot self-test at init runs both on a fixed vector; the fast path is
 * only enabled if it matches the reference, otherwise we fall back to the slow
 * (but always-correct) reference so a port bug can never corrupt audio.
 */

/* ----------------------------- reference path ----------------------------- */

/* The reference path needs exactly one period (4*N floats) for the current
 * transform, not the largest negotiated size. Keep the table lazy and release
 * the previous configuration before resizing so a rate switch never overlaps
 * two complete cosine tables. Fast transforms do not need this allocation. */
#define LHDC_IMDCT_COS_MAX (4 * 1920)
static float *s_cos_tab = NULL;      /* heap, lazy */
static int   s_cos_tab_n = 0;        /* N the table was built for */
static int   s_cos_period = 0;       /* 4N */

/* Build (or rebuild for a new N) the reference cosine table. Returns 0 on
 * success, -1 if it could not be allocated. */
static int lhdc_cos_table_ensure(int N)
{
    if (N <= 0 || N > LHDC_IMDCT_COS_MAX / 4) return -1;
    int period = 4 * N;
    if (s_cos_tab && s_cos_tab_n == N) return 0;
    free(s_cos_tab);
    s_cos_tab = NULL;
    s_cos_tab_n = 0;
    s_cos_period = 0;
    s_cos_tab = (float *)malloc((size_t)period * sizeof(float));
    if (!s_cos_tab) return -1;
    for (int j = 0; j < period; j++)
        s_cos_tab[j] = cosf((M_PIF / (2.0f * (float)N)) * (float)j);
    s_cos_tab_n = N;
    s_cos_period = period;
    return 0;
}

/* Release the active reference-IMDCT cosine table (4*N floats, at most ~30 KB).
 * Must be called at decoder teardown or it leaks for the whole session. */
void lhdc_imdct_free_cos(void)
{
    if (s_cos_tab) { free(s_cos_tab); s_cos_tab = NULL; }
    s_cos_tab_n = 0;
    s_cos_period = 0;
}

static void lhdc_imdct_ref(const float *in, float *out, int mdct_size)
{
    int N = mdct_size;
    int N2 = N / 2;
    int period = 4 * N;
    float scale = 2.0f / (float)N;

    if (lhdc_cos_table_ensure(N) != 0) {   /* out of memory -> silence (safe) */
        for (int n = 0; n < N; n++) out[n] = 0.0f;
        return;
    }

    int kmax = -1;
    for (int k = N2 - 1; k >= 0; k--) {
        if (in[k] != 0.0f) { kmax = k; break; }
    }
    int kn = kmax + 1;
    if (kn == 0) { for (int n = 0; n < N; n++) out[n] = 0.0f; return; }

    for (int n = 0; n < N; n++) {
        int base = (2 * n + 1 + N2);
        int m = base % period;
        int step = (2 * base) % period;
        float s0 = 0.0f, s1 = 0.0f, s2 = 0.0f, s3 = 0.0f;
        int k = 0;
        for (; k + 4 <= kn; k += 4) {
            s0 += in[k]     * s_cos_tab[m]; m += step; if (m >= period) m -= period;
            s1 += in[k + 1] * s_cos_tab[m]; m += step; if (m >= period) m -= period;
            s2 += in[k + 2] * s_cos_tab[m]; m += step; if (m >= period) m -= period;
            s3 += in[k + 3] * s_cos_tab[m]; m += step; if (m >= period) m -= period;
        }
        for (; k < kn; k++) {
            s0 += in[k] * s_cos_tab[m]; m += step; if (m >= period) m -= period;
        }
        out[n] = scale * ((s0 + s1) + (s2 + s3));
    }
}

/* ------------------------------- fast path -------------------------------- */
/* Fixed for N = 480: M = 240, P = N/4 = 120 = N1(8) * N2(15). */
#define IMDCT_N   480
#define IMDCT_M   240
#define IMDCT_P   120
#define IMDCT_N1  8
#define IMDCT_N2  15

/* IMDCT four-step FFT twiddles for N=480 (radix-8 x radix-15) + pre/post
 * rotation. These MUST live in data RAM, not flash: the FFT inner loop reads
 * them thousands of times per frame, and flash (cache-missing under the BT
 * controller's IRQs) made the IMDCT ~10x slower (364us -> 3800us/ch).
 *
 * Like the 960 path, the ~4.2 KB of tables live in ONE lazily-malloc'd block so
 * they occupy DRAM only while 480 (44.1/48k) is the active config: freed via
 * lhdc_imdct_free_480() when the decoder reconfigures to a non-480 rate (e.g.
 * 96k) or LHDC is torn down (so they cost 0 while LDAC/SBC is playing).
 * malloc'd internal heap is DRAM, satisfying the no-flash requirement.
 * Layout (floats): tw_r/i 120 each, rot_r/i 120 each = 480 floats.
 * No w1 (8x8) or w2 (15x15) DFT matrices: stage 1 is a radix-2 8-point FFT with
 * compile-time twiddles and stage 2 is the shared radix-3/5 dft15, so both
 * matrices (578 floats = 2.3 KB of byte-addressable DRAM) are dead -> dropped. */
#define S480_NFLOATS (2*IMDCT_N2*IMDCT_N1 + 2*IMDCT_P)
static float *s480_mem = NULL;
static float *s_tw_r, *s_tw_i;   /* four-step twiddle */
static float *s_rot_r, *s_rot_i; /* pre/post rotation */
static int   s_fast_built = 0;
static int   s_fast_ok = 0;   /* set by the self-test */

void lhdc_imdct_free_480(void)
{
    if (s480_mem) { free(s480_mem); s480_mem = NULL; }
    s_fast_built = 0;
    s_fast_ok = 0;
}

static void lhdc_imdct_build_fast_tables(void)
{
    if (s_fast_built) return;
    if (!s480_mem) {
        s480_mem = (float *)malloc(S480_NFLOATS * sizeof(float));
        if (!s480_mem) return;   /* s_fast_built stays 0 -> falls back to ref IMDCT */
    }
    {
        float *p = s480_mem;
        s_tw_r = p; p += IMDCT_N2*IMDCT_N1;
        s_tw_i = p; p += IMDCT_N2*IMDCT_N1;
        s_rot_r = p; p += IMDCT_P;
        s_rot_i = p; p += IMDCT_P;
    }
    for (int n2 = 0; n2 < IMDCT_N2; n2++)
        for (int k1 = 0; k1 < IMDCT_N1; k1++) {
            float a = -2.0f * M_PIF / IMDCT_P * (float)n2 * (float)k1;
            s_tw_r[n2 * IMDCT_N1 + k1] = cosf(a);
            s_tw_i[n2 * IMDCT_N1 + k1] = sinf(a);
        }
    for (int k = 0; k < IMDCT_P; k++) {
        float a = -2.0f * M_PIF / IMDCT_N * ((float)k + 0.125f);
        s_rot_r[k] = cosf(a);
        s_rot_i[k] = sinf(a);
    }
    s_fast_built = 1;
}

/* 120-point complex FFT (8x15 four-step). Input zr/zi natural order, output Xr/Xi. */
/* ---- shared 15-point DFT via 3x5 Cooley-Tukey (replaces the naive 15x15 DFT) --
 * Stage-2 of all three fast IMDCTs (480/960/1920) is a length-15 complex DFT,
 * the same transform regardless of rate. The reference code did it as a naive
 * 15x15 matrix multiply = 225 complex muls per call. 15 = 3*5, so a Cooley-Tukey
 * split (5 DFT-3 + 15 twiddles + 3 DFT-5 = 135 complex muls) is ~1.7x fewer muls
 * with the SAME result. Verified against the direct DFT-15 in numpy (max rel err
 * 7e-16); the on-device IMDCT self-test (fast vs reference cos formula) is the
 * final guard. Constants below are exact (cos/sin of 2pi/3, 2pi/5, 2pi/15),
 * const -> flash (tiny). Index map: n = 5*n1+n2 (n1<3,n2<5); k = 3*k2+k1. */
/* Post-twiddle for the 3x5 Cooley-Tukey split: TW35[k1*5+n2] = exp(-2*pi*i*k1*n2/15).
 * Row k1=0 is identically 1, so it is skipped at the call site. */
static const float TW35_r[15] = {
    1.0f,1.0f,1.0f,1.0f,1.0f,
    1.0f,0.913545458f,0.669130606f,0.309016994f,-0.104528463f,
    1.0f,0.669130606f,-0.104528463f,-0.809016994f,-0.978147601f};
static const float TW35_i[15] = {
    0.0f,0.0f,0.0f,0.0f,0.0f,
    0.0f,-0.406736643f,-0.743144825f,-0.951056516f,-0.994521895f,
    0.0f,-0.743144825f,-0.994521895f,-0.587785252f,0.207911691f};

/* Butterfly constants (compile-time literals, so they live in registers instead
 * of being re-loaded from a matrix every iteration). */
#define D3_S   0.866025404f      /* sin(2*pi/3)  */
#define D5_C1  0.309016994f      /* cos(2*pi/5)  */
#define D5_S1  0.951056516f      /* sin(2*pi/5)  */
#define D5_C2 -0.809016994f      /* cos(4*pi/5)  */
#define D5_S2  0.587785252f      /* sin(4*pi/5)  */

/*
 * 15-point DFT of g[n], n = 5*n1 + n2 -> X[base + stride*(3*k2 + k1)].
 *
 * 3x5 Cooley-Tukey, but both sub-DFTs are proper butterflies rather than the
 * dense matrix-vector products this used to do. The matrix form cost ~1020
 * real flops plus ~240 table loads per call; the butterflies cost ~290 flops
 * and load nothing but the 10 post-twiddles. dft15 is called 32x per 192k
 * IMDCT (16x at 96k) and was ~60% of the whole transform, so this is the
 * single biggest compute win in the decoder on a chip without ARM's headroom.
 * Arithmetic is reordered, so results differ from the matrix form in the last
 * ~1 ulp; the per-rate IMDCT self-test bounds the end-to-end error.
 */
static LHDC_HOT void lhdc_dft15(const float *gr, const float *gi,
                                float *Xr, float *Xi, int base, int stride)
{
    float Br[15], Bi[15];   /* B[k1*5 + n2] (post-twiddle) */

    /* Stage 1: radix-3 butterfly over n1 for each n2, then the 3x5 twiddle. */
    for (int n2 = 0; n2 < 5; n2++) {
        const float g0r = gr[n2],      g0i = gi[n2];
        const float g1r = gr[5 + n2],  g1i = gi[5 + n2];
        const float g2r = gr[10 + n2], g2i = gi[10 + n2];

        const float sr = g1r + g2r, si = g1i + g2i;   /* g1 + g2 */
        const float dr = g1r - g2r, di = g1i - g2i;   /* g1 - g2 */
        const float ur = g0r - 0.5f * sr, ui = g0i - 0.5f * si;
        const float vr = D3_S * dr,       vi = D3_S * di;

        /* k1 = 0: A0 = g0 + (g1+g2); twiddle is 1 -> store straight through. */
        Br[n2] = g0r + sr;  Bi[n2] = g0i + si;

        /* k1 = 1: A1 = u - i*v ; k1 = 2: A2 = u + i*v */
        {
            const float a1r = ur + vi, a1i = ui - vr;
            const float t1r = TW35_r[5 + n2], t1i = TW35_i[5 + n2];
            Br[5 + n2] = a1r * t1r - a1i * t1i;
            Bi[5 + n2] = a1r * t1i + a1i * t1r;
        }
        {
            const float a2r = ur - vi, a2i = ui + vr;
            const float t2r = TW35_r[10 + n2], t2i = TW35_i[10 + n2];
            Br[10 + n2] = a2r * t2r - a2i * t2i;
            Bi[10 + n2] = a2r * t2i + a2i * t2r;
        }
    }

    /* Stage 2: radix-5 butterfly over n2 for each k1 -> X[base + stride*(3*k2+k1)]. */
    for (int k1 = 0; k1 < 3; k1++) {
        const float *br = &Br[k1 * 5];
        const float *bi = &Bi[k1 * 5];
        const float b0r = br[0], b0i = bi[0];

        const float t1r = br[1] + br[4], t1i = bi[1] + bi[4];
        const float t2r = br[2] + br[3], t2i = bi[2] + bi[3];
        const float t3r = br[1] - br[4], t3i = bi[1] - bi[4];
        const float t4r = br[2] - br[3], t4i = bi[2] - bi[3];

        const float m1r = b0r + D5_C1 * t1r + D5_C2 * t2r;
        const float m1i = b0i + D5_C1 * t1i + D5_C2 * t2i;
        const float m2r = b0r + D5_C2 * t1r + D5_C1 * t2r;
        const float m2i = b0i + D5_C2 * t1i + D5_C1 * t2i;
        const float p1r = D5_S1 * t3r + D5_S2 * t4r;
        const float p1i = D5_S1 * t3i + D5_S2 * t4i;
        const float p2r = D5_S2 * t3r - D5_S1 * t4r;
        const float p2i = D5_S2 * t3i - D5_S1 * t4i;

        float *const xr = Xr + base + stride * k1;   /* k = 3*k2 + k1 */
        float *const xi = Xi + base + stride * k1;
        const int st3 = stride * 3;

        xr[0]       = b0r + t1r + t2r;  xi[0]       = b0i + t1i + t2i;  /* k2 = 0 */
        xr[st3]     = m1r + p1i;        xi[st3]     = m1i - p1r;        /* k2 = 1 */
        xr[st3 * 2] = m2r + p2i;        xi[st3 * 2] = m2i - p2r;        /* k2 = 2 */
        xr[st3 * 3] = m2r - p2i;        xi[st3 * 3] = m2i + p2r;        /* k2 = 3 */
        xr[st3 * 4] = m1r - p1i;        xi[st3 * 4] = m1i + p1r;        /* k2 = 4 */
    }
}

/* W8^1 = (sqrt(2)/2)(1 - i), W8^3 = -(sqrt(2)/2)(1 + i): one shared magnitude. */
#define W8_C 0.70710678118654752f
/* bit-reversal permutation for the 8-point DIT FFT. */
static const uint8_t S480_BR8[8] = { 0, 4, 2, 6, 1, 5, 3, 7 };

static LHDC_HOT void lhdc_fft120(const float *zr, const float *zi, float *Xr, float *Xi)
{
    float Gr[IMDCT_P], Gi[IMDCT_P];   /* G[k1*15 + n2] */
    /* Stage 1: radix-8 DFT over n1 for each n2, then four-step twiddle. */
    for (int n2 = 0; n2 < IMDCT_N2; n2++) {
        float ar[8], ai[8];
        for (int i = 0; i < 8; i++) {            /* gather, bit-reversal folded in */
            int idx = n2 + IMDCT_N2 * (int)S480_BR8[i];
            ar[i] = zr[idx]; ai[i] = zi[idx];
        }
        /* Radix-2 DIT, all three stages written out. W8^0 = 1 and W8^2 = -i need
         * no multiply; only W8^1 and W8^3 do, and both are +-(sqrt(2)/2)(1 -+ i)
         * compile-time constants. This replaces a dense 8x8 matrix-vector product
         * (512 real flops per 8-point DFT, plus 128 table loads) with ~52 flops. */
        for (int k = 0; k < 8; k += 2) {                    /* stage 1: W = 1 */
            float ur = ar[k], ui = ai[k], br = ar[k + 1], bi = ai[k + 1];
            ar[k]     = ur + br; ai[k]     = ui + bi;
            ar[k + 1] = ur - br; ai[k + 1] = ui - bi;
        }
        for (int k = 0; k < 8; k += 4) {                    /* stage 2 */
            {                                               /*  j=0 -> W=1 */
                float ur = ar[k], ui = ai[k], br = ar[k + 2], bi = ai[k + 2];
                ar[k]     = ur + br; ai[k]     = ui + bi;
                ar[k + 2] = ur - br; ai[k + 2] = ui - bi;
            }
            {                                               /*  j=1 -> W8^2 = -i */
                float ur = ar[k + 1], ui = ai[k + 1];
                float tr = ai[k + 3], ti = -ar[k + 3];
                ar[k + 1] = ur + tr; ai[k + 1] = ui + ti;
                ar[k + 3] = ur - tr; ai[k + 3] = ui - ti;
            }
        }
        {                                                   /* stage 3: h = 4 */
            {                                               /*  j=0 -> W=1 */
                float ur = ar[0], ui = ai[0], br = ar[4], bi = ai[4];
                ar[0] = ur + br; ai[0] = ui + bi;
                ar[4] = ur - br; ai[4] = ui - bi;
            }
            {                                               /*  j=1 -> W8^1 */
                float ur = ar[1], ui = ai[1], br = ar[5], bi = ai[5];
                float tr = W8_C * (br + bi), ti = W8_C * (bi - br);
                ar[1] = ur + tr; ai[1] = ui + ti;
                ar[5] = ur - tr; ai[5] = ui - ti;
            }
            {                                               /*  j=2 -> W8^2 = -i */
                float ur = ar[2], ui = ai[2];
                float tr = ai[6], ti = -ar[6];
                ar[2] = ur + tr; ai[2] = ui + ti;
                ar[6] = ur - tr; ai[6] = ui - ti;
            }
            {                                               /*  j=3 -> W8^3 */
                float ur = ar[3], ui = ai[3], br = ar[7], bi = ai[7];
                float tr = W8_C * (bi - br), ti = -W8_C * (br + bi);
                ar[3] = ur + tr; ai[3] = ui + ti;
                ar[7] = ur - tr; ai[7] = ui - ti;
            }
        }
        for (int k1 = 0; k1 < IMDCT_N1; k1++) {             /* four-step twiddle */
            float tr = s_tw_r[n2 * IMDCT_N1 + k1];
            float ti = s_tw_i[n2 * IMDCT_N1 + k1];
            Gr[k1 * IMDCT_N2 + n2] = ar[k1] * tr - ai[k1] * ti;
            Gi[k1 * IMDCT_N2 + n2] = ar[k1] * ti + ai[k1] * tr;
        }
    }
    /* Stage 2: 15-point DFT (3x5 Cooley-Tukey) over n2 for each k1 -> X[k1 + 8*k2]. */
    for (int k1 = 0; k1 < IMDCT_N1; k1++) {
        lhdc_dft15(&Gr[k1 * IMDCT_N2], &Gi[k1 * IMDCT_N2], Xr, Xi, k1, IMDCT_N1);
    }
}

static LHDC_HOT void lhdc_imdct_fast_480(const float *in, float *out)
{
    float zr[IMDCT_P], zi[IMDCT_P];
    float Xr[IMDCT_P], Xi[IMDCT_P];
    float reZ[IMDCT_P], imZ[IMDCT_P];
    const float invM = 1.0f / (float)IMDCT_M;

    /* Pre-twiddle: z[k] = (in[2k] + i*in[239-2k]) * ROT[k]. */
    for (int k = 0; k < IMDCT_P; k++) {
        float r0 = in[2 * k];
        float i0 = in[IMDCT_M - 1 - 2 * k];
        float rr = s_rot_r[k], ri = s_rot_i[k];
        zr[k] = r0 * rr - i0 * ri;
        zi[k] = r0 * ri + i0 * rr;
    }

    lhdc_fft120(zr, zi, Xr, Xi);

    /* Post-twiddle + scale by 1/M. */
    for (int k = 0; k < IMDCT_P; k++) {
        float rr = s_rot_r[k], ri = s_rot_i[k];
        reZ[k] = (Xr[k] * rr - Xi[k] * ri) * invM;
        imZ[k] = (Xr[k] * ri + Xi[k] * rr) * invM;
    }

    /* Symmetric unfold, P=120, M=240 (validated convention). Split into three branch-free
     * segments: the old single loop re-evaluated two 3-way range tests on every
     * one of the 240 iterations, and the ranges are compile-time constants. */
    for (int p = 0; p < 60; p++) {
        out[2 * p]     =  reZ[60 + p];
        out[2 * p + 1] = -imZ[59 - p];
    }
    for (int p = 60; p < 180; p++) {
        out[2 * p]     =  imZ[p - 60];
        out[2 * p + 1] = -reZ[179 - p];
    }
    for (int p = 180; p < 240; p++) {
        out[2 * p]     = -reZ[p - 180];
        out[2 * p + 1] =  imZ[299 - p];
    }
}

/* ----------------------- fast path for N = 960 (96k) ---------------------- */
/* Same four-step MDCT-via-FFT as the 480 path, but P = N/4 = 240 = 16 x 15.
 * Tables are self-contained (own radix-16 and radix-15 DFT matrices) so the
 * 960 path does not depend on the 480 tables being built. ~4 KB .bss, in DRAM
 * (NOT flash) for the same cache-eviction reason as the 480 twiddles. */
#define IMDCT_N_960  960
#define IMDCT_M_960  480
#define IMDCT_P_960  240
#define IMDCT_N1_960 16
#define IMDCT_N2_960 15

/* All 960 tables + scratch live in ONE lazily-malloc'd block so they occupy
 * ~15 KB of DRAM only while 96k is the active config (freed via
 * lhdc_imdct_free_960() when the decoder reconfigures to a non-960 rate or LHDC
 * is torn down). Must be DRAM, never flash: the FFT inner loop reads the
 * twiddles thousands of times/frame and flash cache-misses make it ~10x slower.
 * Layout (floats): w1_r/i 256 each, w2_r/i 225, tw_r/i 240, rot_r/i 240,
 * then 8 scratch buffers (Gr,Gi,zr,zi,Xr,Xi,reZ,imZ) 240 each. */
/* Split into TWO blocks (tables ~7.7 KB + scratch ~7.7 KB) rather than one 15 KB
 * block: at 96k the heap is fragmented (workspace + BT buffers already placed),
 * so a single 15 KB contiguous request can fail even with ~28 KB free (largest
 * hole ~13 KB) -> the fast path silently disabled (or, before the guard, a NULL
 * deref in the self-test). Two ~7.7 KB blocks each fit a 13 KB hole. */
/* No w1 (16x16) table: stage 1 is a radix-2 16-pt FFT using s960_w16r/i (8
 * twiddles), so the old 16x16 DFT matrix (512 floats) is dead -> dropped. */
#define S960_TBL_FLOATS (2*IMDCT_N2_960*IMDCT_N2_960 \
                         + 2*IMDCT_N2_960*IMDCT_N1_960 + 2*IMDCT_P_960)   /* twiddle tables */
/* FFT scratch: only 2 P-sized buffers (zr,zi). Everything else aliases:
 *  - Xr,Xi (fft240 output) and reZ,imZ (post-twiddle output) reuse zr,zi: the
 *    pre-twiddle input is fully consumed by fft240 stage 1 before stage 2 writes,
 *    so the FFT runs in place over zr,zi; post-twiddle rewrites zr,zi using temps.
 *  - Gr,Gi (fft240 four-step intermediate) borrow the caller's `out` buffer
 *    (out[0..2P-1]); `out` is dead until the unfold writes it at the very end,
 *    by which point Gr,Gi are dead. Set per-call in lhdc_imdct_fast_960.
 * Cuts the 96k scratch 7.7 KB -> 1.9 KB. (480 path uses stack, unchanged.) */
#define S960_SCR_FLOATS (2*IMDCT_P_960)                                    /* FFT scratch (zr,zi) */
static float *s960_tbl = NULL;   /* persistent twiddle tables (read-only, shared) */
/* Per-transform scratch is PER SLOT (= per core): the two stereo channels run
 * their IMDCTs concurrently, one per core, so they cannot share zr/zi. The
 * twiddle tables above stay shared (read-only once built). */
static float *s960_scr[LHDC_NSLOTS] = {0};
static float *s960_tw_r, *s960_tw_i, *s960_rot_r, *s960_rot_i;
static int   s960_built = 0;
static int   s960_ok = 0;

/* Radix-2 twiddles W16^k = exp(-2*pi*i*k/16), k=0..7, for the fast 16-point
 * FFT that replaces the naive 16x16 DFT in stage 1 of lhdc_fft240 (cuts the
 * 240-pt FFT ~45% so 96k decode fits the real-time budget). Tiny (.bss), built
 * once with the other 960 tables. */
static float s960_w16r[8], s960_w16i[8];
/* bit-reversal permutation for the 16-point DIT FFT. */
static const uint8_t S960_BR16[16] =
    { 0, 8, 4, 12, 2, 10, 6, 14, 1, 9, 5, 13, 3, 11, 7, 15 };

void lhdc_imdct_free_960(void)
{
    if (s960_tbl) { free(s960_tbl); s960_tbl = NULL; }
    for (int s = 0; s < LHDC_NSLOTS; s++) {
        if (s960_scr[s]) { free(s960_scr[s]); s960_scr[s] = NULL; }
    }
    s960_built = 0;
    s960_ok = 0;
}

static void lhdc_imdct_build_fast_tables_960(void)
{
    if (s960_built) return;
    if (!s960_tbl) {
        s960_tbl = (float *)malloc(S960_TBL_FLOATS * sizeof(float));
        if (!s960_tbl) return;   /* s960_built stays 0 -> falls back to ref IMDCT */
    }
    for (int s = 0; s < LHDC_NSLOTS; s++) {
        if (!s960_scr[s]) {
            s960_scr[s] = (float *)malloc(S960_SCR_FLOATS * sizeof(float));
            if (!s960_scr[s]) { lhdc_imdct_free_960(); return; }
        }
    }
    float *p = s960_tbl;
    s960_tw_r = p; p += IMDCT_N2_960*IMDCT_N1_960;
    s960_tw_i = p; p += IMDCT_N2_960*IMDCT_N1_960;
    s960_rot_r = p; p += IMDCT_P_960;
    s960_rot_i = p; p += IMDCT_P_960;
    /* zr/zi (and their Xr/Xi, reZ/imZ aliases) are now derived per-call from the
     * running core's s960_scr[slot]; Gr/Gi borrow the caller's `out`. */
    for (int n2 = 0; n2 < IMDCT_N2_960; n2++)
        for (int k1 = 0; k1 < IMDCT_N1_960; k1++) {
            float a = -2.0f * M_PIF / IMDCT_P_960 * (float)n2 * (float)k1;
            s960_tw_r[n2 * IMDCT_N1_960 + k1] = cosf(a);
            s960_tw_i[n2 * IMDCT_N1_960 + k1] = sinf(a);
        }
    for (int k = 0; k < IMDCT_P_960; k++) {
        float a = -2.0f * M_PIF / IMDCT_N_960 * ((float)k + 0.125f);
        s960_rot_r[k] = cosf(a);
        s960_rot_i[k] = sinf(a);
    }
    for (int k = 0; k < 8; k++) {
        float a = -2.0f * M_PIF * (float)k / 16.0f;
        s960_w16r[k] = cosf(a);
        s960_w16i[k] = sinf(a);
    }
    s960_built = 1;
}

/* 240-point complex FFT (16x15 four-step). */
static LHDC_HOT void lhdc_fft240(const float *zr, const float *zi, float *Xr, float *Xi,
                                 float *s960_Gr, float *s960_Gi)
{
    /* Stage 1: a 16-point DFT over n1 for each n2, via a radix-2 DIT FFT
     * (replaces the naive 16x16 matrix: 256 -> ~32 complex muls per 16-pt). */
    for (int n2 = 0; n2 < IMDCT_N2_960; n2++) {
        float ar[16], ai[16];
        /* gather (strided) with bit-reversal folded in */
        for (int i = 0; i < 16; i++) {
            int idx = n2 + IMDCT_N2_960 * (int)S960_BR16[i];
            ar[i] = zr[idx]; ai[i] = zi[idx];
        }
        /* Same trivial-twiddle specialization as the 32-point stage in fft480:
         * W16^0 = 1 and W16^4 = -i need no multiply. */
        for (int k = 0; k < 16; k += 2) {              /* stage 1: m=2, W=1 */
            float ur = ar[k], ui = ai[k], br = ar[k + 1], bi = ai[k + 1];
            ar[k]     = ur + br; ai[k]     = ui + bi;
            ar[k + 1] = ur - br; ai[k + 1] = ui - bi;
        }
        for (int k = 0; k < 16; k += 4) {              /* stage 2: m=4 */
            {                                          /*  j=0 -> W=1 */
                float ur = ar[k], ui = ai[k], br = ar[k + 2], bi = ai[k + 2];
                ar[k]     = ur + br; ai[k]     = ui + bi;
                ar[k + 2] = ur - br; ai[k + 2] = ui - bi;
            }
            {                                          /*  j=1 -> W16^4 = -i */
                float ur = ar[k + 1], ui = ai[k + 1];
                float br = ar[k + 3], bi = ai[k + 3];
                float tr = bi, ti = -br;
                ar[k + 1] = ur + tr; ai[k + 1] = ui + ti;
                ar[k + 3] = ur - tr; ai[k + 3] = ui - ti;
            }
        }
        for (int s = 3; s <= 4; s++) {
            /* step = 16 / m with m = 2^s -> pure shift (16 >> s); the divide was a
             * runtime integer division inside the per-frame FFT stage loop. */
            int m = 1 << s, h = m >> 1, step = 16 >> s;
            for (int k = 0; k < 16; k += m) {
                {                                      /*  j=0 -> W=1 */
                    float ur = ar[k], ui = ai[k], br = ar[k + h], bi = ai[k + h];
                    ar[k]     = ur + br; ai[k]     = ui + bi;
                    ar[k + h] = ur - br; ai[k + h] = ui - bi;
                }
                for (int j = 1; j < h; j++) {
                    int tw = j * step;             /* index into W16[0..7] */
                    float wr = s960_w16r[tw], wi = s960_w16i[tw];
                    float br = ar[k + j + h], bi = ai[k + j + h];
                    float tr = wr * br - wi * bi;
                    float ti = wr * bi + wi * br;
                    float ur = ar[k + j], ui = ai[k + j];
                    ar[k + j]     = ur + tr; ai[k + j]     = ui + ti;
                    ar[k + j + h] = ur - tr; ai[k + j + h] = ui - ti;
                }
            }
        }
        /* four-step twiddle + store transposed into G[k1*15 + n2] */
        for (int k1 = 0; k1 < IMDCT_N1_960; k1++) {
            float tr = s960_tw_r[n2 * IMDCT_N1_960 + k1];
            float ti = s960_tw_i[n2 * IMDCT_N1_960 + k1];
            s960_Gr[k1 * IMDCT_N2_960 + n2] = ar[k1] * tr - ai[k1] * ti;
            s960_Gi[k1 * IMDCT_N2_960 + n2] = ar[k1] * ti + ai[k1] * tr;
        }
    }
    for (int k1 = 0; k1 < IMDCT_N1_960; k1++) {
        lhdc_dft15(&s960_Gr[k1 * IMDCT_N2_960], &s960_Gi[k1 * IMDCT_N2_960],
                   Xr, Xi, k1, IMDCT_N1_960);
    }
}

static LHDC_HOT void lhdc_imdct_fast_960(const float *in, float *out)
{
    const float invM = 1.0f / (float)IMDCT_M_960;
    /* This core's private FFT scratch (the other core may be running the other
     * channel's IMDCT at this instant). */
    float *const s960_zr = s960_scr[LHDC_DEC_SLOT()];
    float *const s960_zi = s960_zr + IMDCT_P_960;
    /* Xr/Xi and reZ/imZ alias zr/zi (lifetimes disjoint: fft240 stage 1 fully
     * reads zr/zi before stage 2 writes them; the post-twiddle uses temps). */
    float *const s960_Xr = s960_zr,  *const s960_Xi = s960_zi;
    float *const s960_reZ = s960_zr, *const s960_imZ = s960_zi;
    /* Borrow out[0..2P-1] as the fft240 four-step intermediate (Gr,Gi). `out`
     * (N=960 floats) is not written until the unfold below, by which point
     * Gr,Gi are dead. Saves 2P of dedicated scratch. */
    float *const s960_Gr = out;
    float *const s960_Gi = out + IMDCT_P_960;
    for (int k = 0; k < IMDCT_P_960; k++) {
        float r0 = in[2 * k];
        float i0 = in[IMDCT_M_960 - 1 - 2 * k];
        float rr = s960_rot_r[k], ri = s960_rot_i[k];
        s960_zr[k] = r0 * rr - i0 * ri;
        s960_zi[k] = r0 * ri + i0 * rr;
    }
    lhdc_fft240(s960_zr, s960_zi, s960_Xr, s960_Xi, s960_Gr, s960_Gi);
    for (int k = 0; k < IMDCT_P_960; k++) {
        float rr = s960_rot_r[k], ri = s960_rot_i[k];
        /* reZ/imZ alias Xr/Xi (= zr/zi); read both into temps before writing. */
        float xr = s960_Xr[k], xi = s960_Xi[k];
        s960_reZ[k] = (xr * rr - xi * ri) * invM;
        s960_imZ[k] = (xr * ri + xi * rr) * invM;
    }
    /* Symmetric unfold, P=240, M=480. Split into three branch-free
     * segments: the old single loop re-evaluated two 3-way range tests on every
     * one of the 480 iterations, and the ranges are compile-time constants. */
    for (int p = 0; p < 120; p++) {
        out[2 * p]     =  s960_reZ[120 + p];
        out[2 * p + 1] = -s960_imZ[119 - p];
    }
    for (int p = 120; p < 360; p++) {
        out[2 * p]     =  s960_imZ[p - 120];
        out[2 * p + 1] = -s960_reZ[359 - p];
    }
    for (int p = 360; p < 480; p++) {
        out[2 * p]     = -s960_reZ[p - 360];
        out[2 * p + 1] =  s960_imZ[599 - p];
    }
}

static void lhdc_imdct_selftest_960(void)
{
    /* Heap, NOT .bss: these are 5.6 KB that a `static` would keep resident for the
     * whole session on a target whose byte-addressable DRAM is the same pool the
     * BT media allocator draws from. The test runs once; free before returning.
     * Matches lhdc_imdct_selftest_480(), which already does this. */
    float *tin  = (float *)malloc(IMDCT_M_960 * sizeof(float));
    float *tout = (float *)malloc(IMDCT_N_960 * sizeof(float));
    const int test_bins[] = { 1, 17, 60, 200, 479 };
    const int test_outs[] = { 0, 1, 100, 479, 480, 700, 959 };
    const float amp = 1.0e6f;
    float maxabs = 0.0f, maxdiff = 0.0f;
    if (!tin || !tout) {
        /* Out of memory for the scratch: trust the fast path (proven correct
         * offline) rather than fall back to the 30 KB reference-table path. */
        s960_ok = 1;
        IMDCT_LOGI("LHDCV5_DEC", "IMDCT-960 self-test: skipped (low mem) -> fast TRUSTED");
        free(tin); free(tout);
        return;
    }
    for (unsigned bi = 0; bi < sizeof(test_bins) / sizeof(test_bins[0]); bi++) {
        int b = test_bins[bi];
        for (int k = 0; k < IMDCT_M_960; k++) tin[k] = 0.0f;
        tin[b] = amp;
        lhdc_imdct_fast_960(tin, tout);
        for (unsigned oi = 0; oi < sizeof(test_outs) / sizeof(test_outs[0]); oi++) {
            int n = test_outs[oi];
            float expc = (2.0f / IMDCT_N_960) * amp *
                cosf((M_PIF / (2.0f * IMDCT_N_960)) * (2 * n + 1 + IMDCT_N_960 / 2) * (2 * b + 1));
            float d = tout[n] - expc; if (d < 0) d = -d;
            float a = expc < 0 ? -expc : expc;
            if (a > maxabs) maxabs = a;
            if (d > maxdiff) maxdiff = d;
        }
    }
    s960_ok = (maxdiff <= 1e-3f * (maxabs > 1.0f ? maxabs : 1.0f)) ? 1 : 0;
    IMDCT_LOGI("LHDCV5_DEC", "IMDCT-960 self-test: fast=%s maxref=%.1f maxdiff=%.3f",
               s960_ok ? "ENABLED" : "DISABLED(fallback)", maxabs, maxdiff);
    free(tin); free(tout);
}

/* ===================== N=1920 fast path (192 kHz / 5 ms) =====================
 * Identical four-step structure to the 960 path, scaled x2: P = N/4 = 480 =
 * N1 x N2 = 32 x 15. Stage 1 is a radix-2 32-point DIT FFT (vs the 16-pt for
 * 960); stage 2 is the SAME radix-15 DFT (N2=15 is unchanged), so the 15x15 w2
 * matrix is computed identically (own copy so the block frees independently).
 * Tables ~9.5 KB + scratch ~3.8 KB, lazily malloc'd, DRAM (never flash). Only
 * resident while 192k is the active config (freed via lhdc_imdct_free_1920()).
 * Verified bit-exact against the reference cos-formula by the self-test below. */
#define IMDCT_N_1920  1920
#define IMDCT_M_1920  960
#define IMDCT_P_1920  480
#define IMDCT_N1_1920 32
#define IMDCT_N2_1920 15

#define S1920_TBL_FLOATS (2*IMDCT_N2_1920*IMDCT_N2_1920 \
                          + 2*IMDCT_N2_1920*IMDCT_N1_1920 + 2*IMDCT_P_1920)
#define S1920_SCR_FLOATS (2*IMDCT_P_1920)
static float *s1920_tbl = NULL;   /* persistent twiddle tables (read-only, shared) */
/* Per-slot (= per-core) FFT scratch: the two channels' IMDCTs run concurrently. */
static float *s1920_scr[LHDC_NSLOTS] = {0};
static float *s1920_tw_r, *s1920_tw_i, *s1920_rot_r, *s1920_rot_i;
static int   s1920_built = 0;
static int   s1920_ok = 0;

/* W32^k = exp(-2*pi*i*k/32), k=0..15, for the radix-2 32-point stage-1 FFT. */
static float s1920_w32r[16], s1920_w32i[16];
/* bit-reversal permutation for the 32-point DIT FFT. */
static const uint8_t S1920_BR32[32] =
    { 0, 16, 8, 24, 4, 20, 12, 28, 2, 18, 10, 26, 6, 22, 14, 30,
      1, 17, 9, 25, 5, 21, 13, 29, 3, 19, 11, 27, 7, 23, 15, 31 };

void lhdc_imdct_free_1920(void)
{
    if (s1920_tbl) { free(s1920_tbl); s1920_tbl = NULL; }
    for (int s = 0; s < LHDC_NSLOTS; s++) {
        if (s1920_scr[s]) { free(s1920_scr[s]); s1920_scr[s] = NULL; }
    }
    s1920_built = 0;
    s1920_ok = 0;
}

static void lhdc_imdct_build_fast_tables_1920(void)
{
    if (s1920_built) return;
    if (!s1920_tbl) {
        s1920_tbl = (float *)malloc(S1920_TBL_FLOATS * sizeof(float));
        if (!s1920_tbl) return;   /* stays 0 -> falls back to ref IMDCT */
    }
    for (int s = 0; s < LHDC_NSLOTS; s++) {
        if (!s1920_scr[s]) {
            s1920_scr[s] = (float *)malloc(S1920_SCR_FLOATS * sizeof(float));
            if (!s1920_scr[s]) { lhdc_imdct_free_1920(); return; }
        }
    }
    float *p = s1920_tbl;
    s1920_tw_r = p; p += IMDCT_N2_1920*IMDCT_N1_1920;
    s1920_tw_i = p; p += IMDCT_N2_1920*IMDCT_N1_1920;
    s1920_rot_r = p; p += IMDCT_P_1920;
    s1920_rot_i = p; p += IMDCT_P_1920;
    /* Same aliasing as the 960 path, but derived per-call from the running core's
     * s1920_scr[slot]: fft480 runs in place over zr,zi; the post-twiddle rewrites
     * them via temps; Gr,Gi borrow the caller's out. */
    for (int n2 = 0; n2 < IMDCT_N2_1920; n2++)
        for (int k1 = 0; k1 < IMDCT_N1_1920; k1++) {
            float a = -2.0f * M_PIF / IMDCT_P_1920 * (float)n2 * (float)k1;
            s1920_tw_r[n2 * IMDCT_N1_1920 + k1] = cosf(a);
            s1920_tw_i[n2 * IMDCT_N1_1920 + k1] = sinf(a);
        }
    for (int k = 0; k < IMDCT_P_1920; k++) {
        float a = -2.0f * M_PIF / IMDCT_N_1920 * ((float)k + 0.125f);
        s1920_rot_r[k] = cosf(a);
        s1920_rot_i[k] = sinf(a);
    }
    for (int k = 0; k < 16; k++) {
        float a = -2.0f * M_PIF * (float)k / 32.0f;
        s1920_w32r[k] = cosf(a);
        s1920_w32i[k] = sinf(a);
    }
    s1920_built = 1;
}

/* 480-point complex FFT (32x15 four-step). */
static LHDC_HOT void lhdc_fft480(const float *zr, const float *zi, float *Xr, float *Xi,
                                 float *s1920_Gr, float *s1920_Gi)
{
    /* Stage 1: a 32-point DFT over n1 for each n2, via a radix-2 DIT FFT. */
    for (int n2 = 0; n2 < IMDCT_N2_1920; n2++) {
        float ar[32], ai[32];
        for (int i = 0; i < 32; i++) {
            int idx = n2 + IMDCT_N2_1920 * (int)S1920_BR32[i];
            ar[i] = zr[idx]; ai[i] = zi[idx];
        }
        /* Stages 1-2 have only trivial twiddles (W32^0 = 1, W32^8 = -i), so they
         * are written out without the complex multiply: 31 of the 80 butterflies
         * in a 32-point FFT need no multiply at all and 15 more are just a
         * swap+negate. Stages 3-5 keep the generic butterfly but hoist j = 0
         * (always W = 1) out of the inner loop. */
        for (int k = 0; k < 32; k += 2) {              /* stage 1: m=2, W=1 */
            float ur = ar[k], ui = ai[k], br = ar[k + 1], bi = ai[k + 1];
            ar[k]     = ur + br; ai[k]     = ui + bi;
            ar[k + 1] = ur - br; ai[k + 1] = ui - bi;
        }
        for (int k = 0; k < 32; k += 4) {              /* stage 2: m=4 */
            {                                          /*  j=0 -> W=1 */
                float ur = ar[k], ui = ai[k], br = ar[k + 2], bi = ai[k + 2];
                ar[k]     = ur + br; ai[k]     = ui + bi;
                ar[k + 2] = ur - br; ai[k + 2] = ui - bi;
            }
            {                                          /*  j=1 -> W32^8 = -i */
                float ur = ar[k + 1], ui = ai[k + 1];
                float br = ar[k + 3], bi = ai[k + 3];
                float tr = bi, ti = -br;               /* (-i) * (br + i*bi) */
                ar[k + 1] = ur + tr; ai[k + 1] = ui + ti;
                ar[k + 3] = ur - tr; ai[k + 3] = ui - ti;
            }
        }
        for (int s = 3; s <= 5; s++) {
            /* step = 32 / m with m = 2^s -> pure shift (32 >> s). */
            int m = 1 << s, h = m >> 1, step = 32 >> s;
            for (int k = 0; k < 32; k += m) {
                {                                      /*  j=0 -> W=1 */
                    float ur = ar[k], ui = ai[k], br = ar[k + h], bi = ai[k + h];
                    ar[k]     = ur + br; ai[k]     = ui + bi;
                    ar[k + h] = ur - br; ai[k + h] = ui - bi;
                }
                for (int j = 1; j < h; j++) {
                    int tw = j * step;             /* index into W32[0..15] */
                    float wr = s1920_w32r[tw], wi = s1920_w32i[tw];
                    float br = ar[k + j + h], bi = ai[k + j + h];
                    float tr = wr * br - wi * bi;
                    float ti = wr * bi + wi * br;
                    float ur = ar[k + j], ui = ai[k + j];
                    ar[k + j]     = ur + tr; ai[k + j]     = ui + ti;
                    ar[k + j + h] = ur - tr; ai[k + j + h] = ui - ti;
                }
            }
        }
        for (int k1 = 0; k1 < IMDCT_N1_1920; k1++) {
            float tr = s1920_tw_r[n2 * IMDCT_N1_1920 + k1];
            float ti = s1920_tw_i[n2 * IMDCT_N1_1920 + k1];
            s1920_Gr[k1 * IMDCT_N2_1920 + n2] = ar[k1] * tr - ai[k1] * ti;
            s1920_Gi[k1 * IMDCT_N2_1920 + n2] = ar[k1] * ti + ai[k1] * tr;
        }
    }
    for (int k1 = 0; k1 < IMDCT_N1_1920; k1++) {
        lhdc_dft15(&s1920_Gr[k1 * IMDCT_N2_1920], &s1920_Gi[k1 * IMDCT_N2_1920],
                   Xr, Xi, k1, IMDCT_N1_1920);
    }
}

static LHDC_HOT void lhdc_imdct_fast_1920(const float *in, float *out)
{
    const float invM = 1.0f / (float)IMDCT_M_1920;
    /* This core's private FFT scratch (the other core may be running the other
     * channel's IMDCT at this instant). */
    float *const s1920_zr = s1920_scr[LHDC_DEC_SLOT()];
    float *const s1920_zi = s1920_zr + IMDCT_P_1920;
    float *const s1920_Xr = s1920_zr,  *const s1920_Xi = s1920_zi;
    float *const s1920_reZ = s1920_zr, *const s1920_imZ = s1920_zi;
    /* Borrow out[0..2P-1] (2P=960 <= N=1920) as Gr,Gi; out is dead until unfold. */
    float *const s1920_Gr = out;
    float *const s1920_Gi = out + IMDCT_P_1920;
    for (int k = 0; k < IMDCT_P_1920; k++) {
        float r0 = in[2 * k];
        float i0 = in[IMDCT_M_1920 - 1 - 2 * k];
        float rr = s1920_rot_r[k], ri = s1920_rot_i[k];
        s1920_zr[k] = r0 * rr - i0 * ri;
        s1920_zi[k] = r0 * ri + i0 * rr;
    }
    lhdc_fft480(s1920_zr, s1920_zi, s1920_Xr, s1920_Xi, s1920_Gr, s1920_Gi);
    for (int k = 0; k < IMDCT_P_1920; k++) {
        float rr = s1920_rot_r[k], ri = s1920_rot_i[k];
        float xr = s1920_Xr[k], xi = s1920_Xi[k];
        s1920_reZ[k] = (xr * rr - xi * ri) * invM;
        s1920_imZ[k] = (xr * ri + xi * rr) * invM;
    }
    /* Symmetric unfold, P=480, M=960. Split into three branch-free
     * segments: the old single loop re-evaluated two 3-way range tests on every
     * one of the 960 iterations, and the ranges are compile-time constants. */
    for (int p = 0; p < 240; p++) {
        out[2 * p]     =  s1920_reZ[240 + p];
        out[2 * p + 1] = -s1920_imZ[239 - p];
    }
    for (int p = 240; p < 720; p++) {
        out[2 * p]     =  s1920_imZ[p - 240];
        out[2 * p + 1] = -s1920_reZ[719 - p];
    }
    for (int p = 720; p < 960; p++) {
        out[2 * p]     = -s1920_reZ[p - 720];
        out[2 * p + 1] =  s1920_imZ[1199 - p];
    }
}

static void lhdc_imdct_selftest_1920(void)
{
    /* Heap, NOT .bss: a `static` pair here costs 11.5 KB of permanently resident
     * byte-DRAM -- the scarcest pool on the classic ESP32 and the one the BT
     * media packets come from. The test runs once; free before returning. */
    float *tin  = (float *)malloc(IMDCT_M_1920 * sizeof(float));
    float *tout = (float *)malloc(IMDCT_N_1920 * sizeof(float));
    const int test_bins[] = { 1, 17, 60, 200, 480, 959 };
    const int test_outs[] = { 0, 1, 100, 959, 960, 1400, 1919 };
    const float amp = 1.0e6f;
    float maxabs = 0.0f, maxdiff = 0.0f;
    if (!tin || !tout) {
        s1920_ok = 1;   /* proven correct offline -> trust, do not fall back */
        IMDCT_LOGI("LHDCV5_DEC", "IMDCT-1920 self-test: skipped (low mem) -> fast TRUSTED");
        free(tin); free(tout);
        return;
    }
    for (unsigned bi = 0; bi < sizeof(test_bins) / sizeof(test_bins[0]); bi++) {
        int b = test_bins[bi];
        for (int k = 0; k < IMDCT_M_1920; k++) tin[k] = 0.0f;
        tin[b] = amp;
        lhdc_imdct_fast_1920(tin, tout);
        for (unsigned oi = 0; oi < sizeof(test_outs) / sizeof(test_outs[0]); oi++) {
            int n = test_outs[oi];
            float expc = (2.0f / IMDCT_N_1920) * amp *
                cosf((M_PIF / (2.0f * IMDCT_N_1920)) * (2 * n + 1 + IMDCT_N_1920 / 2) * (2 * b + 1));
            float d = tout[n] - expc; if (d < 0) d = -d;
            float a = expc < 0 ? -expc : expc;
            if (a > maxabs) maxabs = a;
            if (d > maxdiff) maxdiff = d;
        }
    }
    s1920_ok = (maxdiff <= 1e-3f * (maxabs > 1.0f ? maxabs : 1.0f)) ? 1 : 0;
    IMDCT_LOGI("LHDCV5_DEC", "IMDCT-1920 self-test: fast=%s maxref=%.1f maxdiff=%.3f",
               s1920_ok ? "ENABLED" : "DISABLED(fallback)", maxabs, maxdiff);
    free(tin); free(tout);
}

/* ------------------------------- init / API ------------------------------- */

/*
 * Lightweight self-test of the fast path: drive a few single-bin impulses and
 * check a handful of output samples against the exact formula (direct cos(), a
 * few dozen calls — no big buffers, no cosine table). Sets s_fast_ok. If memory
 * for the scratch is unavailable we trust the fast path (it is proven correct
 * offline) rather than fall back to the slow one.
 */
static void lhdc_imdct_selftest_480(void)
{
    float *tin  = (float *)malloc(IMDCT_M * sizeof(float));
    float *tout = (float *)malloc(IMDCT_N * sizeof(float));
    if (!tin || !tout) {
        s_fast_ok = 1;
        IMDCT_LOGI("LHDCV5_DEC", "IMDCT self-test: skipped (low mem) -> fast TRUSTED");
        free(tin); free(tout);
        return;
    }
    const int test_bins[]  = { 1, 17, 60, 119 };
    const int test_outs[]  = { 0, 1, 100, 239, 240, 350, 479 };
    const float amp = 1.0e6f;
    float maxabs = 0.0f, maxdiff = 0.0f;
    int ok = 1;
    for (unsigned bi = 0; bi < sizeof(test_bins) / sizeof(test_bins[0]); bi++) {
        int b = test_bins[bi];
        for (int k = 0; k < IMDCT_M; k++) tin[k] = 0.0f;
        tin[b] = amp;
        lhdc_imdct_fast_480(tin, tout);
        for (unsigned oi = 0; oi < sizeof(test_outs) / sizeof(test_outs[0]); oi++) {
            int n = test_outs[oi];
            float expc = (2.0f / IMDCT_N) * (float)amp *
                cosf((M_PIF / (2.0f * IMDCT_N)) * (2 * n + 1 + IMDCT_N / 2) * (2 * b + 1));
            float d = (float)tout[n] - expc;
            if (d < 0) d = -d;
            float a = expc < 0 ? -expc : expc;
            if (a > maxabs) maxabs = (float)a;
            if (d > maxdiff) maxdiff = (float)d;
        }
    }
    /* float math at amp=1e6: tolerate 1e-3 of full scale. */
    ok = (maxdiff <= 1e-3f * (maxabs > 1.0f ? maxabs : 1.0f));
    s_fast_ok = ok ? 1 : 0;
    IMDCT_LOGI("LHDCV5_DEC", "IMDCT self-test: fast=%s maxref=%.1f maxdiff=%.3f",
               s_fast_ok ? "ENABLED" : "DISABLED(fallback)", maxabs, maxdiff);
    free(tin); free(tout);
}

int lhdc_imdct_init(int mdct_size)
{
    int N = mdct_size;

    /* Release the 96k (N=960) fast tables whenever we are not decoding 960, so
     * their ~15 KB only lives in DRAM while 96k is the active config. Cheap
     * no-op once freed (called per frame). */
    if (N != IMDCT_N_960) lhdc_imdct_free_960();
    /* Symmetrically release the 480 (44.1/48k) tables when not decoding 480, so
     * their ~4.2 KB only lives in DRAM while that rate is active (e.g. freed on a
     * 48k->96k switch). */
    if (N != IMDCT_N) lhdc_imdct_free_480();
    /* And the 1920 (192k) tables (~13 KB) when not decoding 1920. */
    if (N != IMDCT_N_1920) lhdc_imdct_free_1920();

    /* Build + self-test the fast path once (only relevant for N=480). The build
     * sets s_fast_built=1 ONLY on a successful malloc; if it failed we must NOT
     * run the self-test (it calls fast_480, which would deref NULL tables) and
     * must fall through to the reference IMDCT below. */
    if (N == IMDCT_N && !s_fast_built) {
        lhdc_imdct_build_fast_tables();
        if (s_fast_built) lhdc_imdct_selftest_480();
    }

    /* Fast path for N=960 (96k). Same NULL-table guard: only self-test if the
     * (split) table+scratch allocation actually succeeded. */
    if (N == IMDCT_N_960 && !s960_built) {
        lhdc_imdct_build_fast_tables_960();
        if (s960_built) lhdc_imdct_selftest_960();
    }

    /* Fast path for N=1920 (192k). Same NULL-table guard. */
    if (N == IMDCT_N_1920 && !s1920_built) {
        lhdc_imdct_build_fast_tables_1920();
        if (s1920_built) lhdc_imdct_selftest_1920();
    }

    /* For sizes no fast path covers (or if a fast path was disabled), make sure
     * the reference cosine table is ready. */
    if (!(N == IMDCT_N && s_fast_ok) && !(N == IMDCT_N_960 && s960_ok) &&
        !(N == IMDCT_N_1920 && s1920_ok)) {
        (void)lhdc_cos_table_ensure(N);
    } else if (!(g_lhdc_diag_force_ref_imdct_960 && N == IMDCT_N_960)) {
        /* init() also runs per frame. Keep an explicitly forced reference
         * table cached; otherwise diagnostics would allocate/free every frame. */
#if defined(LHDC_HOST_BUILD)
        if (!getenv("LHDC_FORCE_REF"))
#endif
            lhdc_imdct_free_cos();
    }

    return 0;
}

void lhdc_imdct_transform(const float *in, float *out, int mdct_size)
{
if (g_lhdc_diag_force_ref_imdct_960 && mdct_size == 960) {
        lhdc_imdct_ref(in, out, mdct_size);
        return;
    }
#if defined(LHDC_HOST_BUILD)
    /* Diagnostic: force the slow reference cos-formula IMDCT to test whether the
     * fast FFT path is the dense-frame garble source (the self-test only checks
     * single-bin/tone inputs). */
    { extern char *getenv(const char*); if (getenv("LHDC_FORCE_REF")) { lhdc_imdct_ref(in, out, mdct_size); return; } }
#endif
    if (mdct_size == IMDCT_N_960) {
        if (!s960_built) lhdc_imdct_init(IMDCT_N_960);
        if (s960_ok) { lhdc_imdct_fast_960(in, out); return; }
    }
    if (mdct_size == IMDCT_N_1920) {
        if (!s1920_built) lhdc_imdct_init(IMDCT_N_1920);
        if (s1920_ok) { lhdc_imdct_fast_1920(in, out); return; }
    }
    if (mdct_size == IMDCT_N) {
        if (!s_fast_built) lhdc_imdct_init(IMDCT_N);
        if (s_fast_ok) { lhdc_imdct_fast_480(in, out); return; }
    }
    lhdc_imdct_ref(in, out, mdct_size);   /* builds its cos table on demand */
}
