#pragma once

// -----------------------------------------------------------
// Fixed-point Goertzel for beat detection at 30/60/100 Hz.
//
// NOTE: the DSP processor decimates the audio to a fixed ~11 kHz effective rate
// before feeding this bank (so the per-sample cost stays low at 88.2/96 kHz).
// init() is therefore called with that EFFECTIVE rate, not the codec rate.
//
// Two things make this robust across every codec sample rate AND keep the
// update rate (and therefore the BLE meter refresh) identical to before:
//
// 1) SAMPLE-RATE-SCALED WINDOW
//    A Goertzel detects bin m = N * f / fs. A tone is only resolvable when the
//    window holds ~one cycle (m >~ 1). We scale N with the (effective) rate
//    (N = BASE_N * fs / BASE_FS) so m - and thus the detection behaviour - is
//    IDENTICAL regardless of codec rate, and the window time stays ~constant
//    (~11.6 ms) so magnitudes refresh at ~86 Hz (no meter lag).
//
// 2) FIXED POINT (Q29 feedback coeff, Q15 input, 64-bit state)
//    The per-sample recurrence is integer-only. NOTE the coefficient needs high
//    fractional precision: at 30 Hz/96k, 2*cos(w)=1.99999614, so a Q14 int16
//    coeff (as in many DTMF examples) would quantise to ~169 Hz. Q29 keeps
//    thousands of LSBs between the three bands. The magnitude/dB conversion
//    stays in float but only runs once per window (~86 Hz), so it is cheap.
// -----------------------------------------------------------

#include <math.h>
#include <stdint.h>
#include "esp_attr.h"
#include "fast_math.h"
#include "../config/app_config.h"

class Goertzel {
public:
    Goertzel() : m_coeffQ29(0), m_s1(0), m_s2(0), m_cosW(0.0f), m_sinW(0.0f) {}

    void init(float f, float fs) {
        float w = fast_div(2.0f * DSP_PI_F * f, fs);
        m_cosW = cosf(w);
        m_sinW = sinf(w);
        // 2*cos(w) in Q29. Range (-2,2) -> |value| < 2^30, fits int64 with room.
        m_coeffQ29 = (int64_t)llroundf(2.0f * m_cosW * 536870912.0f); // 2^29
        m_s1 = 0;
        m_s2 = 0;
    }

    void reset() {
        m_s1 = 0;
        m_s2 = 0;
    }

    // x: Q15 sample (-32768..32767 == -1.0..+1.0). State is Q15 in int64 so the
    // resonator (which can grow to ~N/2 * input) never overflows.
    inline void IRAM_ATTR feed(int32_t x) {
        int64_t s0 = (int64_t)x + ((m_coeffQ29 * m_s1) >> 29) - m_s2;
        m_s2 = m_s1;
        m_s1 = s0;
    }

    // Magnitude in Q15 input units (~ component_amplitude * N). Done in float
    // once per window; the bank normalises this to a 0..1 linear level.
    float magnitude() const {
        float s1 = (float)m_s1;
        float s2 = (float)m_s2;
        float real = s1 - s2 * m_cosW;
        float imag = s2 * m_sinW;
        return sqrtf(real * real + imag * imag);
    }

private:
    int64_t m_coeffQ29;
    int64_t m_s1;
    int64_t m_s2;
    float   m_cosW;
    float   m_sinW;
};

// -----------------------------------------------------------
// GoertzelBank - 3 fixed-point detectors (30/60/100 Hz)
// -----------------------------------------------------------

class GoertzelBank {
public:
    static constexpr int NUM_BANDS = 3;
    static constexpr float FREQS[NUM_BANDS] = {30.0f, 60.0f, 100.0f};

    // Reference window: BASE_N samples at BASE_FS. N is scaled from these so the
    // detection bin m = N*f/fs (and the window time / update rate) is identical
    // at every sample rate. BASE matches the original behaviour at 44.1 kHz.
    static constexpr float BASE_FS = 44100.0f;
    static constexpr int   BASE_N  = APP_GOERTZEL_N;   // 512 by default
    static constexpr int   MIN_N   = 64;
    static constexpr int   MAX_N   = 4096;

    GoertzelBank() : m_sampleRate(0), m_n(BASE_N), m_count(0) {
        for (int i = 0; i < NUM_BANDS; i++) {
            m_dB[i] = -120.0f;
            m_lin[i] = 0.0f;
        }
    }

    void init(uint32_t sampleRate) {
        m_sampleRate = sampleRate;
        if (sampleRate == 0) return;

        float fs = (float)sampleRate;

        // Scale the window so m = N*f/fs stays constant across sample rates.
        int n = (int)lroundf((float)BASE_N * fs / BASE_FS);
        if (n < MIN_N) n = MIN_N;
        else if (n > MAX_N) n = MAX_N;
        m_n = (uint16_t)n;

        for (int i = 0; i < NUM_BANDS; i++) {
            m_goertzel[i].init(FREQS[i], fs);
        }
        m_count = 0;
    }

    // Process a single full-rate mono sample.
    inline void IRAM_ATTR processSample(float x) {
        int32_t xq = (int32_t)(x * 32768.0f);
        if (xq > 32767) xq = 32767;
        else if (xq < -32768) xq = -32768;

        for (int i = 0; i < NUM_BANDS; i++) {
            m_goertzel[i].feed(xq);
        }

        if (++m_count >= m_n) {
            computeMagnitudes();
        }
    }

    float getDB(int band) const {
        if (band < 0 || band >= NUM_BANDS) return -120.0f;
        return m_dB[band];
    }

    float getLin(int band) const {
        if (band < 0 || band >= NUM_BANDS) return 0.0f;
        return m_lin[band];
    }

    void zeroLevels() {
        for (int i = 0; i < NUM_BANDS; i++) {
            m_dB[i] = -120.0f;
            m_lin[i] = 0.0f;
        }
    }

private:
    void computeMagnitudes() {
        // magnitude() scales with the window length and with the Q15 input
        // scale; divide by (N/2) and by 32768 to recover a 0..1 linear
        // amplitude (same scale/dB as the old float code, so existing
        // thresholds still apply).
        const float invNorm = 1.0f / ((float)m_n * 0.5f * 32768.0f);

        for (int i = 0; i < NUM_BANDS; i++) {
            float norm = m_goertzel[i].magnitude() * invNorm;
            if (norm < 1e-9f) norm = 1e-9f;

            float dB = 20.0f * log10f(norm);
            if (dB < -120.0f || !isfinite(dB)) dB = -120.0f;

            m_dB[i] = dB;
            m_lin[i] = norm;

            m_goertzel[i].reset();
        }
        m_count = 0;
    }

    Goertzel m_goertzel[NUM_BANDS];
    uint32_t m_sampleRate;
    uint16_t m_n;        // window length (samples) for current sample rate
    uint16_t m_count;    // samples fed toward current window
    volatile float m_dB[NUM_BANDS];
    volatile float m_lin[NUM_BANDS];
};

// Static member initialization
constexpr float GoertzelBank::FREQS[GoertzelBank::NUM_BANDS];
