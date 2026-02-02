#pragma once

// -----------------------------------------------------------
// BiQuad filter implementation
// Transposed Direct Form II for good numeric stability
// -----------------------------------------------------------

#include <math.h>
#include "fast_math.h"
#include "../config/app_config.h"

class Biquad {
public:
    float b0, b1, b2, a1, a2;
    float z1, z2;

    Biquad() : b0(1.0f), b1(0.0f), b2(0.0f), a1(0.0f), a2(0.0f), z1(0.0f), z2(0.0f) {}

    // Process single sample using Transposed Direct Form II
    inline float process(float in) {
        float out = b0 * in + z1;
        z1 = b1 * in + z2 - a1 * out;
        z2 = b2 * in - a2 * out;
        return out;
    }

    void reset() {
        z1 = 0.0f;
        z2 = 0.0f;
    }

    // Low shelf filter
    void makeLowShelf(float fs, float fc, float gainDB) {
        float A = sqrtf(powf(10.0f, fast_div(gainDB, 20.0f)));
        float w0 = fast_div(2.0f * DSP_PI_F * fc, fs);
        float cs = cosf(w0);
        float sn = sinf(w0);
        float alpha = sn * 0.5f * sqrtf(2.0f);

        float twoSqrtAAlpha = 2.0f * sqrtf(A) * alpha;

        float b0_ = A * ((A + 1.0f) - (A - 1.0f) * cs + twoSqrtAAlpha);
        float b1_ = 2.0f * A * ((A - 1.0f) - (A + 1.0f) * cs);
        float b2_ = A * ((A + 1.0f) - (A - 1.0f) * cs - twoSqrtAAlpha);
        float a0_ = (A + 1.0f) + (A - 1.0f) * cs + twoSqrtAAlpha;
        float a1_ = -2.0f * ((A - 1.0f) + (A + 1.0f) * cs);
        float a2_ = (A + 1.0f) + (A - 1.0f) * cs - twoSqrtAAlpha;

        float inv_a0 = fast_recipsf2(a0_);
        b0 = b0_ * inv_a0;
        b1 = b1_ * inv_a0;
        b2 = b2_ * inv_a0;
        a1 = a1_ * inv_a0;
        a2 = a2_ * inv_a0;
        z1 = z2 = 0.0f;
    }

    // High shelf filter
    void makeHighShelf(float fs, float fc, float gainDB) {
        float A = sqrtf(powf(10.0f, fast_div(gainDB, 20.0f)));
        float w0 = fast_div(2.0f * DSP_PI_F * fc, fs);
        float cs = cosf(w0);
        float sn = sinf(w0);
        float alpha = sn * 0.5f * sqrtf(2.0f);

        float twoSqrtAAlpha = 2.0f * sqrtf(A) * alpha;

        float b0_ = A * ((A + 1.0f) + (A - 1.0f) * cs + twoSqrtAAlpha);
        float b1_ = -2.0f * A * ((A - 1.0f) + (A + 1.0f) * cs);
        float b2_ = A * ((A + 1.0f) + (A - 1.0f) * cs - twoSqrtAAlpha);
        float a0_ = (A + 1.0f) - (A - 1.0f) * cs + twoSqrtAAlpha;
        float a1_ = 2.0f * ((A - 1.0f) - (A + 1.0f) * cs);
        float a2_ = (A + 1.0f) - (A - 1.0f) * cs - twoSqrtAAlpha;

        float inv_a0 = fast_recipsf2(a0_);
        b0 = b0_ * inv_a0;
        b1 = b1_ * inv_a0;
        b2 = b2_ * inv_a0;
        a1 = a1_ * inv_a0;
        a2 = a2_ * inv_a0;
        z1 = z2 = 0.0f;
    }

    // Peaking EQ filter
    void makePeakingEQ(float fs, float fc, float Q, float gainDB) {
        float A = sqrtf(powf(10.0f, fast_div(gainDB, 20.0f)));
        float w0 = fast_div(2.0f * DSP_PI_F * fc, fs);
        float cs = cosf(w0);
        float sn = sinf(w0);
        float alpha = fast_div(sn, 2.0f * Q);

        float inv_A = fast_recipsf2(A);
        float b0_ = 1.0f + alpha * A;
        float b1_ = -2.0f * cs;
        float b2_ = 1.0f - alpha * A;
        float a0_ = 1.0f + alpha * inv_A;
        float a1_ = -2.0f * cs;
        float a2_ = 1.0f - alpha * inv_A;

        float inv_a0 = fast_recipsf2(a0_);
        b0 = b0_ * inv_a0;
        b1 = b1_ * inv_a0;
        b2 = b2_ * inv_a0;
        a1 = a1_ * inv_a0;
        a2 = a2_ * inv_a0;
        z1 = z2 = 0.0f;
    }

    // Butterworth 2nd-order Low-Pass Filter (Q = 0.7071)
    void makeLowPass(float fs, float fc) {
        float w0 = fast_div(2.0f * DSP_PI_F * fc, fs);
        float cs = cosf(w0);
        float sn = sinf(w0);
        float alpha = sn * (0.5f * 1.41421356f);

        float one_minus_cs = 1.0f - cs;
        float b0_ = one_minus_cs * 0.5f;
        float b1_ = one_minus_cs;
        float b2_ = one_minus_cs * 0.5f;
        float a0_ = 1.0f + alpha;
        float a1_ = -2.0f * cs;
        float a2_ = 1.0f - alpha;

        float inv_a0 = fast_recipsf2(a0_);
        b0 = b0_ * inv_a0;
        b1 = b1_ * inv_a0;
        b2 = b2_ * inv_a0;
        a1 = a1_ * inv_a0;
        a2 = a2_ * inv_a0;
        z1 = z2 = 0.0f;
    }

    // Butterworth 2nd-order High-Pass Filter (Q = 0.7071)
    void makeHighPass(float fs, float fc) {
        float w0 = fast_div(2.0f * DSP_PI_F * fc, fs);
        float cs = cosf(w0);
        float sn = sinf(w0);
        float alpha = sn * (0.5f * 1.41421356f);

        float one_plus_cs = 1.0f + cs;
        float b0_ = one_plus_cs * 0.5f;
        float b1_ = -one_plus_cs;
        float b2_ = one_plus_cs * 0.5f;
        float a0_ = 1.0f + alpha;
        float a1_ = -2.0f * cs;
        float a2_ = 1.0f - alpha;

        float inv_a0 = fast_recipsf2(a0_);
        b0 = b0_ * inv_a0;
        b1 = b1_ * inv_a0;
        b2 = b2_ * inv_a0;
        a1 = a1_ * inv_a0;
        a2 = a2_ * inv_a0;
        z1 = z2 = 0.0f;
    }
};
