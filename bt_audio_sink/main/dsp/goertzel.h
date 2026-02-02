#pragma once

// -----------------------------------------------------------
// Goertzel algorithm for efficient single-frequency DFT
// Used for beat detection at 30/60/100 Hz
// -----------------------------------------------------------

#include <math.h>
#include "fast_math.h"
#include "../config/app_config.h"

class Goertzel {
public:
    float coeff;
    float s1;
    float s2;
    float freq;

    Goertzel() : coeff(0.0f), s1(0.0f), s2(0.0f), freq(0.0f) {}

    void init(float f, float fs) {
        freq = f;
        s1 = 0.0f;
        s2 = 0.0f;
        float w = fast_div(2.0f * DSP_PI_F * f, fs);
        coeff = 2.0f * cosf(w);
    }

    void reset() {
        s1 = 0.0f;
        s2 = 0.0f;
    }

    inline void feed(float x) {
        float s0 = x + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }

    float magnitude(float fs) const {
        float w = fast_div(2.0f * DSP_PI_F * freq, fs);
        float real = s1 - s2 * cosf(w);
        float imag = s2 * sinf(w);
        return sqrtf(real * real + imag * imag);
    }
};

// -----------------------------------------------------------
// GoertzelBank - manages multiple Goertzel filters
// -----------------------------------------------------------

class GoertzelBank {
public:
    static constexpr int NUM_BANDS = 3;
    static constexpr float FREQS[NUM_BANDS] = {30.0f, 60.0f, 100.0f};

    GoertzelBank() : m_sampleRate(0), m_count(0) {
        for (int i = 0; i < NUM_BANDS; i++) {
            m_dB[i] = -120.0f;
            m_lin[i] = 0.0f;
        }
    }

    void init(uint32_t sampleRate) {
        m_sampleRate = sampleRate;
        if (sampleRate == 0) return;
        
        float fs = (float)sampleRate;
        for (int i = 0; i < NUM_BANDS; i++) {
            m_goertzel[i].init(FREQS[i], fs);
        }
        m_count = 0;
    }

    // Process a single sample (mono)
    void processSample(float x) {
        for (int i = 0; i < NUM_BANDS; i++) {
            m_goertzel[i].feed(x);
        }

        m_count++;
        if (m_count >= APP_GOERTZEL_N) {
            computeMagnitudes();
        }
    }

    // Get dB level for band (0=30Hz, 1=60Hz, 2=100Hz)
    float getDB(int band) const {
        if (band < 0 || band >= NUM_BANDS) return -120.0f;
        return m_dB[band];
    }

    // Get linear magnitude for band
    float getLin(int band) const {
        if (band < 0 || band >= NUM_BANDS) return 0.0f;
        return m_lin[band];
    }

    // Zero all levels (called when audio stops)
    void zeroLevels() {
        for (int i = 0; i < NUM_BANDS; i++) {
            m_dB[i] = -120.0f;
            m_lin[i] = 0.0f;
        }
    }

private:
    void computeMagnitudes() {
        float fs = (float)m_sampleRate;
        if (fs <= 0.0f) fs = (float)APP_I2S_DEFAULT_SR;

        static const float invNormFactor = 1.0f / (float)(APP_GOERTZEL_N / 2);

        for (int i = 0; i < NUM_BANDS; i++) {
            float mag = m_goertzel[i].magnitude(fs);
            float norm = mag * invNormFactor;
            
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
    uint16_t m_count;
    volatile float m_dB[NUM_BANDS];
    volatile float m_lin[NUM_BANDS];
};

// Static member initialization
constexpr float GoertzelBank::FREQS[GoertzelBank::NUM_BANDS];
