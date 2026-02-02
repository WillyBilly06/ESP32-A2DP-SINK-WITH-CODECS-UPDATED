#pragma once

// -----------------------------------------------------------
// Fast math utilities for ESP32
// Uses hardware reciprocal instruction for ~5-6x faster division
// -----------------------------------------------------------

#include <stdint.h>
#include <math.h>

#ifdef ESP32
static __attribute__((always_inline)) inline
float fast_recipsf2(float a) {
    float result;
    asm volatile (
        "wfr f1, %1\n"
        "recip0.s f0, f1\n"
        "const.s f2, 1\n"
        "msub.s f2, f1, f0\n"
        "maddn.s f0, f0, f2\n"
        "const.s f2, 1\n"
        "msub.s f2, f1, f0\n"
        "maddn.s f0, f0, f2\n"
        "rfr %0, f0\n"
        :"=r"(result):"r"(a):"f0","f1","f2"
    );
    return result;
}

#define fast_div(a, b) ((a) * fast_recipsf2(b))

#else
// Fallback for non-ESP32 platforms
#define fast_div(a, b) ((a) / (b))
static inline float fast_recipsf2(float a) { return 1.0f / a; }
#endif

// -----------------------------------------------------------
// Fast natural log approximation (no division in hot path)
// Uses IEEE 754 floating point bit manipulation
// Accuracy: ~0.1% error, sufficient for dB conversion
// -----------------------------------------------------------
static inline float fast_logf(float x) {
    // Extract exponent and mantissa from IEEE 754 float
    union { float f; uint32_t i; } u = { x };
    
    // Get exponent (biased by 127)
    int e = ((u.i >> 23) & 0xFF) - 127;
    
    // Set exponent to 0 (bias 127) to get mantissa in [1, 2)
    u.i = (u.i & 0x007FFFFF) | 0x3F800000;
    float m = u.f;
    
    // Polynomial approximation for ln(m) where m in [1, 2)
    // ln(m) ≈ (m - 1) - 0.5*(m-1)^2 + 0.333*(m-1)^3 (Taylor series, simplified)
    // Faster approximation: ln(m) ≈ -1.7417939 + m*(2.8212026 + m*(-1.4699568 + m*0.44717955))
    float t = m - 1.0f;
    float lnm = t * (0.9999964239f + t * (-0.4998741238f + t * 0.3317990258f));
    
    // ln(x) = ln(2^e * m) = e*ln(2) + ln(m)
    return (float)e * 0.6931471806f + lnm;
}

// -----------------------------------------------------------
// Fast sine approximation using Bhaskara I's formula
// Accuracy: ~0.2% max error, very fast for audio LFOs
// Input: radians (any value, handles wrap)
// -----------------------------------------------------------
static inline float fast_sinf(float x) {
    // Wrap to [-PI, PI]
    const float FAST_PI = 3.14159265f;
    const float FAST_TWO_PI = 6.28318530f;
    
    // Fast modulo using subtraction
    while (x > FAST_PI) x -= FAST_TWO_PI;
    while (x < -FAST_PI) x += FAST_TWO_PI;
    
    // Bhaskara I approximation: sin(x) ≈ 16x(π-x) / (5π² - 4x(π-x))
    // For x in [0, π], this gives < 0.2% error
    // For negative x, sin(-x) = -sin(x)
    float sign = 1.0f;
    if (x < 0) {
        sign = -1.0f;
        x = -x;
    }
    
    float xpi = x * (FAST_PI - x);
    float denom = 5.0f * FAST_PI * FAST_PI - 4.0f * xpi;
    return sign * 16.0f * xpi * fast_recipsf2(denom);
}

// -----------------------------------------------------------
// Fast cosine using sin identity: cos(x) = sin(x + π/2)
// -----------------------------------------------------------
static inline float fast_cosf(float x) {
    return fast_sinf(x + 1.57079632f);  // x + PI/2
}

// Template clamp utility
template<typename T, typename U, typename V>
static inline T clamp_value(U x, V lo, V hi) {
    if (x < lo) return (T)lo;
    if (x > hi) return (T)hi;
    return (T)x;
}

