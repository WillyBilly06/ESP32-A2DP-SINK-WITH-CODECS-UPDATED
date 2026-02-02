#pragma once

// -----------------------------------------------------------
// LED Effects Engine
// Audio-reactive effects for 16x16 WS2812B matrix
// OPTIMIZED: No divisions - all bit shifts and multiplies
// Uses SPI DMA driver for reliable LED output
// -----------------------------------------------------------

#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include "led_config.h"
#include "led_driver_spi.h"
#include "../dsp/fast_math.h"

// Use SPI driver types
typedef LedDriverSPI LedDriver;
typedef RGB_SPI RGB;
#define Colors ColorsSPI

// ============================================================
// FAST MATH UTILITIES (No Division!)
// ============================================================

// Fast 8-bit random (LFSR)
static inline uint8_t random8() {
    static uint16_t seed = 1337;
    seed = (seed * 2053) + 13849;
    return (uint8_t)(seed >> 8);
}

// Fast modulo for power-of-2: x % n == x & (n-1)
// For non-power-of-2, use multiply-shift approximation
static inline uint8_t random8(uint8_t max) {
    // Use multiply-high for division-free modulo approximation
    // result ≈ random8() * max / 256 (but faster)
    return (uint8_t)(((uint16_t)random8() * max) >> 8);
}

static inline uint8_t random8(uint8_t min, uint8_t max) {
    return min + random8(max - min);
}

// Division-free approximations:
// x / 255 ≈ (x * 257) >> 16  (for x up to 65535)
// x / 256 = x >> 8
// x / 128 = x >> 7
// x / 64  = x >> 6
// x / 32  = x >> 5
// x / 16  = x >> 4
// x / 8   = x >> 3
// x / 4   = x >> 2
// x / 3   ≈ (x * 85) >> 8    (or x * 171 >> 9 for better precision)
// x / 5   ≈ (x * 51) >> 8
// x / 6   ≈ (x * 43) >> 8
// x / 15  ≈ (x * 17) >> 8
// x / 60  ≈ (x * 17) >> 10   (or x * 4 >> 8, roughly)

// Divide by 255 (for normalizing 0-255 to 0.0-1.0 range, returns 0-256 scaled)
static inline uint16_t div255(uint16_t x) {
    return (x * 257 + 256) >> 16;  // More accurate
}

// Divide by 3 (integer)
static inline uint16_t div3(uint16_t x) {
    return (x * 171) >> 9;  // x * 0.333 ≈ x * 171 / 512
}

// Divide by 60 (for dB scaling) - returns fixed-point 8.8
static inline uint16_t div60_fp8(uint16_t x) {
    return (x * 273) >> 8;  // 273/16384 ≈ 1/60, but we want x*256/60
}

// Fast float-to-int8 with scale: converts 0.0-1.0 to 0-255
static inline uint8_t floatToU8(float f) {
    if (f <= 0.0f) return 0;
    if (f >= 1.0f) return 255;
    return (uint8_t)(f * 255.0f);
}

// Multiply 8-bit value by 0-255 factor, result 0-255
// (a * b) / 255, but division-free
static inline uint8_t scale8(uint8_t a, uint8_t b) {
    return (uint8_t)(((uint16_t)a * (uint16_t)b + 128) >> 8);
}

// Linear interpolation: lerp8(a, b, t) where t is 0-255
static inline uint8_t lerp8(uint8_t a, uint8_t b, uint8_t t) {
    return a + scale8(b - a, t);
}

// Fast sin8 - COMPUTED (no lookup table, saves 256 bytes RAM)
// Uses parabolic approximation: sin(x) ≈ 4x(π-x)/π² 
// Input: 0-255 (0-2π), Output: 0-255 (maps to -1 to +1 centered at 128)
static inline uint8_t sin8(uint8_t theta) {
    // Use symmetry: only compute 0-127, mirror for 128-255
    uint8_t offset = theta;
    if (theta >= 128) {
        offset = theta - 128;  // Fold second half
    }
    
    // Parabolic approximation for half-wave (0-127 -> 0 to peak to 0)
    // y = 4 * x * (128 - x) / 128 for x in [0, 128]
    // Simplified: y = (x * (128 - x)) >> 5 gives 0-128 range
    // We want 0-127 peak at 64
    uint16_t x = offset;
    uint16_t y = (x * (128 - x)) >> 5;  // 0 at edges, ~128 at center
    if (y > 127) y = 127;  // Clamp
    
    // Map to output: first half goes 128->255->128, second half 128->0->128
    if (theta < 64) {
        return 128 + y;  // Rising: 128 -> 255
    } else if (theta < 128) {
        return 128 + y;  // Falling: 255 -> 128 (y decreases)
    } else if (theta < 192) {
        return 128 - y;  // Below: 128 -> 0
    } else {
        return 128 - y;  // Rising back: 0 -> 128 (y decreases)
    }
}

static inline uint8_t cos8(uint8_t theta) {
    return sin8((uint8_t)(theta + 64));
}

// Quadratic ease in/out for smoother animations
static inline uint8_t ease8(uint8_t x) {
    if (x < 128) {
        return (x * x) >> 7;
    } else {
        uint8_t n = 255 - x;
        return 255 - ((n * n) >> 7);
    }
}

// Audio data structure passed to effects
struct AudioData {
    float bass;         // Bass level (linear, 0-1)
    float mid;          // Mid level (linear, 0-1)
    float high;         // High level (linear, 0-1)
    float bassDB;       // Bass in dB (-60 to 0)
    float midDB;        // Mid in dB
    float highDB;       // High in dB
    bool beat;          // Beat detected this frame
    float beatIntensity; // Beat strength (0-1)
    bool audioActive;   // Audio is currently playing
};

// Base effect class
class LedEffect {
public:
    virtual ~LedEffect() {}
    virtual void init(LedDriver* driver) { m_driver = driver; m_frame = 0; }
    virtual void update(const AudioData& audio) = 0;
    virtual void updateDemo() { update(AudioData{}); }  // Default demo just runs normal update
    virtual const char* getName() const = 0;
    
protected:
    LedDriver* m_driver = nullptr;
    uint32_t m_frame = 0;
    
    // Utility: convert linear audio level to display height (0-15)
    // Uses fixed-point math: (db+60) * 15 / 60 = (db+60) / 4
    int levelToHeight(float level, float minDb = -60.0f) {
        float db = (level > 1e-6f) ? 20.0f * log10f(level) : minDb;
        if (db < minDb) db = minDb;
        if (db > 0) db = 0;
        // (db+60)/60*15 = (db+60)*0.25 = (db+60)>>2 approximately
        int result = (int)((db - minDb) * 0.25f);  // 15/60 = 0.25
        return (result > 15) ? 15 : result;
    }
    
    // Utility: convert dB to display height (division-free)
    int dbToHeight(float db) {
        if (db < -60.0f) db = -60.0f;
        if (db > 0) db = 0;
        // (db+60)*15/60 = (db+60)*0.25
        int result = (int)((db + 60.0f) * 0.25f);
        return (result > 15) ? 15 : result;
    }
};

// -----------------------------------------------------------
// Effect 1: Spectrum Bars
// Classic spectrum analyzer with 16 frequency bands
// -----------------------------------------------------------
class SpectrumBarsEffect : public LedEffect {
public:
    void init(LedDriver* driver) override {
        LedEffect::init(driver);
        for (int i = 0; i < 16; i++) {
            m_heights[i] = 0;
            m_peaks[i] = 0;
            m_peakDecay[i] = 0;
        }
    }
    
    void update(const AudioData& audio) override {
        m_driver->clear();
        m_frame++;
        
        // Simulate 16 bands from 3 frequency inputs (interpolate)
        // Use integer math: i/15 ≈ (i*17)>>8 for i in 0-15
        float bands[16];
        for (int i = 0; i < 16; i++) {
            // pos = i * (1/15) ≈ i * 0.0667
            // For i=0-4: bass->mid, i=5-9: mid->high, i=10-15: high
            if (i < 5) {
                float t = i * 0.2f;  // 0, 0.2, 0.4, 0.6, 0.8
                bands[i] = audio.bass * (1.0f - t) + audio.mid * t;
            } else if (i < 10) {
                float t = (i - 5) * 0.2f;
                bands[i] = audio.mid * (1.0f - t) + audio.high * t;
            } else {
                bands[i] = audio.high;
            }
            // Add some variation: * (0.8 + 0.4 * sin8/255) = * (0.8 + sin8 * 0.00157)
            // Approx: scale by (204 + sin8*0.4) >> 8
            uint16_t scale = 204 + ((sin8(i * 16 + m_frame) * 102) >> 8);  // 204-306 range
            bands[i] = bands[i] * scale * 0.00390625f;  // /256
        }
        
        for (int x = 0; x < 16; x++) {
            int targetHeight = levelToHeight(bands[x] * 1.5f);
            
            // Smooth rise, fast fall
            if (targetHeight > m_heights[x]) {
                m_heights[x] = targetHeight;
            } else {
                m_heights[x] = m_heights[x] * 0.85f;
            }
            
            int h = (int)m_heights[x];
            
            // Update peak
            if (h > m_peaks[x]) {
                m_peaks[x] = h;
                m_peakDecay[x] = 30;
            } else if (m_peakDecay[x] > 0) {
                m_peakDecay[x]--;
            } else if (m_peaks[x] > 0) {
                m_peaks[x]--;
            }
            
            // Draw bar with gradient
            for (int y = 0; y < h && y < 16; y++) {
                int yFlip = 15 - y;  // Draw from bottom
                RGB color;
                if (y < 5) {
                    color = RGB::fromHSV(96, 255, 255);  // Green
                } else if (y < 10) {
                    color = RGB::fromHSV(64 - (y - 5) * 12, 255, 255);  // Yellow to orange
                } else {
                    color = RGB::fromHSV(0, 255, 255);  // Red
                }
                m_driver->setPixelXY(x, yFlip, color);
            }
            
            // Draw peak
            if (m_peaks[x] > 0 && m_peaks[x] < 16) {
                m_driver->setPixelXY(x, 15 - m_peaks[x], Colors::White);
            }
        }
    }
    
    void updateDemo() override {
        AudioData demo;
        // sin8/255 ≈ sin8 * 0.00392 ≈ (sin8 * 257) >> 16
        demo.bass = 0.3f + 0.3f * (sin8(m_frame * 2) * 0.00392f);
        demo.mid = 0.3f + 0.2f * (sin8(m_frame * 3 + 85) * 0.00392f);
        demo.high = 0.2f + 0.2f * (sin8(m_frame * 4 + 170) * 0.00392f);
        demo.beat = false;
        demo.audioActive = false;
        update(demo);
    }
    
    const char* getName() const override { return "Spectrum"; }
    
private:
    float m_heights[16];
    int m_peaks[16];
    int m_peakDecay[16];
};

// -----------------------------------------------------------
// Effect 2: Beat Pulse
// Full matrix pulse on beat detection
// -----------------------------------------------------------
class BeatPulseEffect : public LedEffect {
public:
    void init(LedDriver* driver) override {
        LedEffect::init(driver);
        m_pulseLevel = 0;
        m_hue = 0;
    }
    
    void update(const AudioData& audio) override {
        m_frame++;
        
        if (audio.beat) {
            m_pulseLevel = 255;
            m_hue += 32;  // Cycle color on each beat
        }
        
        // Background based on bass
        uint8_t bgLevel = (uint8_t)(audio.bass * 30);
        RGB bgColor = RGB::fromHSV(m_hue + 128, 255, bgLevel);
        m_driver->fill(bgColor);
        
        // Pulse overlay
        if (m_pulseLevel > 10) {
            RGB pulseColor = RGB::fromHSV(m_hue, 255, m_pulseLevel);
            for (int i = 0; i < LED_MATRIX_COUNT; i++) {
                RGB current = m_driver->getPixel(i);
                m_driver->setPixel(i, current.add(pulseColor));
            }
            m_pulseLevel = m_pulseLevel * 0.85f;
        }
    }
    
    void updateDemo() override {
        m_frame++;
        if ((m_frame % 30) == 0) {
            m_pulseLevel = 200;
            m_hue += 32;
        }
        
        RGB bgColor = RGB::fromHSV(m_hue + 128, 200, 20);
        m_driver->fill(bgColor);
        
        if (m_pulseLevel > 10) {
            RGB pulseColor = RGB::fromHSV(m_hue, 255, m_pulseLevel);
            for (int i = 0; i < LED_MATRIX_COUNT; i++) {
                RGB current = m_driver->getPixel(i);
                m_driver->setPixel(i, current.add(pulseColor));
            }
            m_pulseLevel = m_pulseLevel * 0.9f;
        }
    }
    
    const char* getName() const override { return "Beat Pulse"; }
    
private:
    float m_pulseLevel;
    uint8_t m_hue;
};

// -----------------------------------------------------------
// Effect 3: Ripple
// Concentric ripples from center on beat
// -----------------------------------------------------------
class RippleEffect : public LedEffect {
    struct Ripple {
        float radius;
        float speed;
        uint8_t hue;
        bool active;
    };
    
public:
    void init(LedDriver* driver) override {
        LedEffect::init(driver);
        for (int i = 0; i < MAX_RIPPLES; i++) {
            m_ripples[i].active = false;
        }
    }
    
    void update(const AudioData& audio) override {
        m_frame++;
        m_driver->fadeAll(200);
        
        // Spawn ripple on beat
        if (audio.beat) {
            for (int i = 0; i < MAX_RIPPLES; i++) {
                if (!m_ripples[i].active) {
                    m_ripples[i].active = true;
                    m_ripples[i].radius = 0;
                    m_ripples[i].speed = 0.3f + audio.beatIntensity * 0.3f;
                    m_ripples[i].hue = random8();
                    break;
                }
            }
        }
        
        // Update and draw ripples
        float cx = 7.5f, cy = 7.5f;
        for (int i = 0; i < MAX_RIPPLES; i++) {
            if (!m_ripples[i].active) continue;
            
            m_ripples[i].radius += m_ripples[i].speed;
            if (m_ripples[i].radius > 15) {
                m_ripples[i].active = false;
                continue;
            }
            
            // Draw ring
            float r = m_ripples[i].radius;
            uint8_t brightness = 255 - (uint8_t)(r * 15);
            RGB color = RGB::fromHSV(m_ripples[i].hue, 255, brightness);
            
            for (int y = 0; y < 16; y++) {
                for (int x = 0; x < 16; x++) {
                    float dx = x - cx, dy = y - cy;
                    float dist = sqrtf(dx * dx + dy * dy);
                    if (fabsf(dist - r) < 1.0f) {
                        float intensity = 1.0f - fabsf(dist - r);
                        RGB c = color.scale((uint8_t)(intensity * 255));
                        RGB current = m_driver->getPixelXY(x, y);
                        m_driver->setPixelXY(x, y, current.add(c));
                    }
                }
            }
        }
    }
    
    void updateDemo() override {
        m_frame++;
        AudioData demo;
        demo.beat = (m_frame % 25) == 0;
        demo.beatIntensity = 0.7f;
        update(demo);
    }
    
    const char* getName() const override { return "Ripple"; }
    
private:
    static const int MAX_RIPPLES = 5;
    Ripple m_ripples[MAX_RIPPLES];
};

// -----------------------------------------------------------
// Effect 4: Fire
// Fire effect with bass modulation
// -----------------------------------------------------------
class FireEffect : public LedEffect {
public:
    void init(LedDriver* driver) override {
        LedEffect::init(driver);
        for (int i = 0; i < 16 * 16; i++) {
            m_heat[i] = 0;
        }
    }
    
    void update(const AudioData& audio) override {
        m_frame++;
        
        // Cooling - modulated by audio
        int cooling = 55 - (int)(audio.bass * 30);
        for (int x = 0; x < 16; x++) {
            for (int y = 0; y < 16; y++) {
                int idx = y * 16 + x;
                int cooldown = random8(0, ((cooling * 10) >> 4) + 2);  // /16 = >>4
                if (cooldown > m_heat[idx]) {
                    m_heat[idx] = 0;
                } else {
                    m_heat[idx] -= cooldown;
                }
            }
        }
        
        // Heat rises
        for (int x = 0; x < 16; x++) {
            for (int y = 0; y < 15; y++) {
                int idx = y * 16 + x;
                // /3 ≈ (*171)>>9 or (*85)>>8
                uint16_t sum = m_heat[idx + 16] + m_heat[idx + 16] + m_heat[(y + 1) * 16 + ((x + 1) & 15)];
                m_heat[idx] = (sum * 85) >> 8;  // divide by 3
            }
        }
        
        // Sparking at bottom - more sparks with more bass
        int sparking = 80 + (int)(audio.bass * 120);
        for (int x = 0; x < 16; x++) {
            if (random8() < sparking) {
                int idx = 15 * 16 + x;
                m_heat[idx] = m_heat[idx] + random8(160, 255);
                if (m_heat[idx] > 255) m_heat[idx] = 255;
            }
        }
        
        // Map heat to colors
        for (int y = 0; y < 16; y++) {
            for (int x = 0; x < 16; x++) {
                int idx = y * 16 + x;
                uint8_t heat = m_heat[idx];
                
                // Heat color palette
                RGB color;
                if (heat < 85) {
                    color = RGB(heat * 3, 0, 0);
                } else if (heat < 170) {
                    color = RGB(255, (heat - 85) * 3, 0);
                } else {
                    color = RGB(255, 255, (heat - 170) * 3);
                }
                
                m_driver->setPixelXY(x, y, color);
            }
        }
    }
    
    void updateDemo() override {
        AudioData demo;
        demo.bass = 0.3f + 0.2f * (sin8(m_frame * 2) * 0.00392f);
        update(demo);
    }
    
    const char* getName() const override { return "Fire"; }
    
private:
    uint8_t m_heat[16 * 16];
};

// -----------------------------------------------------------
// Effect 5: Plasma
// Animated plasma with audio modulation
// -----------------------------------------------------------
class PlasmaEffect : public LedEffect {
public:
    void update(const AudioData& audio) override {
        m_frame++;
        
        uint8_t time1 = m_frame;
        uint8_t time2 = m_frame * 2;
        float audioMod = 1.0f + (audio.bass + audio.mid) * 0.5f;
        
        for (int y = 0; y < 16; y++) {
            for (int x = 0; x < 16; x++) {
                uint8_t v1 = sin8(x * 16 + time1);
                uint8_t v2 = sin8(y * 16 + time2);
                uint8_t v3 = sin8((x + y) * 8 + m_frame);
                uint8_t v4 = sin8(sqrtf(x * x + y * y) * 8 - m_frame);
                
                uint8_t hue = (v1 + v2 + v3 + v4) >> 2;  // /4 = >>2
                uint8_t value = 128 + (sin8(hue + m_frame) >> 1);
                value = (uint8_t)(value * audioMod);
                if (value > 255) value = 255;
                
                m_driver->setPixelXY(x, y, RGB::fromHSV(hue, 255, value));
            }
        }
    }
    
    const char* getName() const override { return "Plasma"; }
};

// -----------------------------------------------------------
// Effect 6: Matrix Rain
// Digital rain effect, speed modulated by audio
// -----------------------------------------------------------
class MatrixRainEffect : public LedEffect {
    struct Drop {
        float y;
        float speed;
        int length;
        bool active;
    };
    
public:
    void init(LedDriver* driver) override {
        LedEffect::init(driver);
        for (int x = 0; x < 16; x++) {
            m_drops[x].active = false;
            m_drops[x].y = -5;
        }
    }
    
    void update(const AudioData& audio) override {
        m_frame++;
        m_driver->fadeAll(180);
        
        float speedMod = 0.3f + audio.bass * 0.5f + audio.mid * 0.3f;
        int spawnChance = 30 + (int)(audio.high * 40);
        
        for (int x = 0; x < 16; x++) {
            if (!m_drops[x].active) {
                if (random8() < spawnChance) {
                    m_drops[x].active = true;
                    m_drops[x].y = -random8(1, 8);
                    m_drops[x].speed = 0.2f + random8() * 0.001953f;  // /512 = *0.00195
                    m_drops[x].length = 3 + random8(8);
                }
            }
            
            if (m_drops[x].active) {
                m_drops[x].y += m_drops[x].speed * speedMod;
                
                // Draw drop with trail
                for (int i = 0; i < m_drops[x].length; i++) {
                    int y = (int)m_drops[x].y - i;
                    if (y >= 0 && y < 16) {
                        uint8_t brightness = 255 - (i * 255 / m_drops[x].length);
                        RGB color;
                        if (i == 0) {
                            color = Colors::White;
                        } else {
                            color = RGB(0, brightness, 0);
                        }
                        m_driver->setPixelXY(x, y, color);
                    }
                }
                
                if (m_drops[x].y - m_drops[x].length > 16) {
                    m_drops[x].active = false;
                }
            }
        }
    }
    
    void updateDemo() override {
        AudioData demo;
        demo.bass = 0.3f;
        demo.mid = 0.2f;
        demo.high = 0.2f;
        update(demo);
    }
    
    const char* getName() const override { return "Matrix Rain"; }
    
private:
    Drop m_drops[16];
};

// -----------------------------------------------------------
// Effect 7: VU Meter
// Dual stereo VU meter bars
// -----------------------------------------------------------
class VUMeterEffect : public LedEffect {
public:
    void init(LedDriver* driver) override {
        LedEffect::init(driver);
        m_leftLevel = 0;
        m_rightLevel = 0;
        m_leftPeak = 0;
        m_rightPeak = 0;
        m_peakHold = 0;
    }
    
    void update(const AudioData& audio) override {
        m_frame++;
        m_driver->clear();
        
        // Use bass + mid for overall level
        float level = (audio.bass * 0.6f + audio.mid * 0.3f + audio.high * 0.1f) * 2.0f;
        
        // Smooth levels
        float target = level * 15.0f;
        m_leftLevel += (target - m_leftLevel) * 0.3f;
        m_rightLevel += (target * (0.9f + 0.2f * sin8(m_frame * 3) * 0.00392f) - m_rightLevel) * 0.3f;
        
        // Peaks
        if (m_leftLevel > m_leftPeak) { m_leftPeak = m_leftLevel; m_peakHold = 30; }
        if (m_rightLevel > m_rightPeak) { m_rightPeak = m_rightLevel; m_peakHold = 30; }
        
        if (m_peakHold > 0) {
            m_peakHold--;
        } else {
            if (m_leftPeak > 0) m_leftPeak -= 0.2f;
            if (m_rightPeak > 0) m_rightPeak -= 0.2f;
        }
        
        // Draw left VU (columns 0-6)
        drawVUBar(0, 7, (int)m_leftLevel, (int)m_leftPeak);
        
        // Draw right VU (columns 9-15)
        drawVUBar(9, 7, (int)m_rightLevel, (int)m_rightPeak);
        
        // Center decoration
        uint8_t hue = m_frame;
        for (int y = 0; y < 16; y++) {
            m_driver->setPixelXY(7, y, RGB::fromHSV(hue + y * 16, 255, 100));
            m_driver->setPixelXY(8, y, RGB::fromHSV(hue + y * 16 + 128, 255, 100));
        }
    }
    
    void updateDemo() override {
        m_frame++;
        AudioData demo;
        demo.bass = 0.3f + 0.3f * sin8(m_frame * 2) * 0.00392f;
        demo.mid = 0.2f + 0.2f * sin8(m_frame * 3 + 85) * 0.00392f;
        demo.high = 0.1f + 0.1f * sin8(m_frame * 4 + 170) * 0.00392f;
        update(demo);
    }
    
    const char* getName() const override { return "VU Meter"; }
    
private:
    void drawVUBar(int xStart, int width, int level, int peak) {
        for (int x = xStart; x < xStart + width; x++) {
            for (int y = 0; y < level && y < 16; y++) {
                int yFlip = 15 - y;
                RGB color;
                if (y < 10) {
                    color = RGB::fromHSV(96 - y * 8, 255, 255);  // Green to yellow
                } else {
                    color = RGB::fromHSV(0, 255, 255);  // Red
                }
                m_driver->setPixelXY(x, yFlip, color);
            }
            
            // Peak indicator
            if (peak > 0 && peak < 16) {
                m_driver->setPixelXY(x, 15 - peak, Colors::White);
            }
        }
    }
    
    float m_leftLevel, m_rightLevel;
    float m_leftPeak, m_rightPeak;
    int m_peakHold;
};

// -----------------------------------------------------------
// Effect 8: Starfield
// Stars that twinkle and flash on beat
// -----------------------------------------------------------
class StarfieldEffect : public LedEffect {
    struct Star {
        int x, y;
        uint8_t brightness;
        int8_t delta;
    };
    
public:
    void init(LedDriver* driver) override {
        LedEffect::init(driver);
        for (int i = 0; i < NUM_STARS; i++) {
            m_stars[i].x = random8() % 16;
            m_stars[i].y = random8() % 16;
            m_stars[i].brightness = random8(50, 200);
            m_stars[i].delta = random8(2, 8);
            if (random8() & 1) m_stars[i].delta = -m_stars[i].delta;
        }
    }
    
    void update(const AudioData& audio) override {
        m_frame++;
        m_driver->clear();
        
        // Flash all stars on beat
        uint8_t beatBoost = 0;
        if (audio.beat) {
            beatBoost = 200;
        }
        m_beatLevel = m_beatLevel * 0.8f + beatBoost * 0.2f;
        
        // Background glow based on bass
        uint8_t bgLevel = (uint8_t)(audio.bass * 20);
        RGB bg = RGB::fromHSV(160, 255, bgLevel);
        m_driver->fill(bg);
        
        for (int i = 0; i < NUM_STARS; i++) {
            // Twinkle
            m_stars[i].brightness += m_stars[i].delta;
            if (m_stars[i].brightness > 250 || m_stars[i].brightness < 30) {
                m_stars[i].delta = -m_stars[i].delta;
            }
            
            // Beat flash
            uint8_t b = m_stars[i].brightness + (uint8_t)m_beatLevel;
            if (b > 255) b = 255;
            
            m_driver->setPixelXY(m_stars[i].x, m_stars[i].y, RGB(b, b, b));
        }
    }
    
    void updateDemo() override {
        m_frame++;
        AudioData demo;
        demo.bass = 0.1f;
        demo.beat = (m_frame % 40) == 0;
        update(demo);
    }
    
    const char* getName() const override { return "Starfield"; }
    
private:
    static const int NUM_STARS = 40;
    Star m_stars[NUM_STARS];
    float m_beatLevel = 0;
};

// -----------------------------------------------------------
// Effect 9: Wave
// Sine wave visualization with multiple harmonics
// -----------------------------------------------------------
class WaveEffect : public LedEffect {
public:
    void update(const AudioData& audio) override {
        m_frame++;
        m_driver->fadeAll(200);
        
        float amp1 = 3.0f + audio.bass * 4.0f;
        float amp2 = 2.0f + audio.mid * 3.0f;
        float amp3 = 1.0f + audio.high * 2.0f;
        
        for (int x = 0; x < 16; x++) {
            float t = m_frame * 0.1f + x * 0.4f;
            
            float y1 = 7.5f + amp1 * sinf(t);
            float y2 = 7.5f + amp2 * sinf(t * 1.5f + 1.0f);
            float y3 = 7.5f + amp3 * sinf(t * 2.0f + 2.0f);
            
            // Draw waves
            if ((int)y1 >= 0 && (int)y1 < 16)
                m_driver->setPixelXY(x, (int)y1, RGB::fromHSV(0, 255, 255));    // Red - bass
            if ((int)y2 >= 0 && (int)y2 < 16)
                m_driver->setPixelXY(x, (int)y2, RGB::fromHSV(85, 255, 255));   // Green - mid
            if ((int)y3 >= 0 && (int)y3 < 16)
                m_driver->setPixelXY(x, (int)y3, RGB::fromHSV(170, 255, 255));  // Blue - high
        }
    }
    
    void updateDemo() override {
        AudioData demo;
        demo.bass = 0.4f + 0.2f * sinf(m_frame * 0.05f);
        demo.mid = 0.3f + 0.2f * sinf(m_frame * 0.07f);
        demo.high = 0.2f + 0.1f * sinf(m_frame * 0.09f);
        update(demo);
    }
    
    const char* getName() const override { return "Wave"; }
};

// -----------------------------------------------------------
// Effect 10: Fireworks
// Fireworks that launch on beat
// -----------------------------------------------------------
class FireworksEffect : public LedEffect {
    struct Particle {
        float x, y;
        float vx, vy;
        uint8_t hue;
        uint8_t life;
        bool active;
    };
    
    struct Rocket {
        float x, y;
        float vy;
        uint8_t hue;
        bool active;
    };
    
public:
    void init(LedDriver* driver) override {
        LedEffect::init(driver);
        for (int i = 0; i < MAX_PARTICLES; i++) {
            m_particles[i].active = false;
        }
        for (int i = 0; i < MAX_ROCKETS; i++) {
            m_rockets[i].active = false;
        }
    }
    
    void update(const AudioData& audio) override {
        m_frame++;
        m_driver->fadeAll(180);
        
        // Launch rocket on beat
        if (audio.beat) {
            for (int i = 0; i < MAX_ROCKETS; i++) {
                if (!m_rockets[i].active) {
                    m_rockets[i].active = true;
                    m_rockets[i].x = 4 + random8(8);
                    m_rockets[i].y = 15;
                    m_rockets[i].vy = -0.5f - random8() * 0.002f;  // /500 ≈ *0.002
                    m_rockets[i].hue = random8();
                    break;
                }
            }
        }
        
        // Update rockets
        for (int i = 0; i < MAX_ROCKETS; i++) {
            if (!m_rockets[i].active) continue;
            
            m_rockets[i].y += m_rockets[i].vy;
            m_rockets[i].vy += 0.01f;  // Gravity
            
            // Draw rocket
            int y = (int)m_rockets[i].y;
            if (y >= 0 && y < 16) {
                m_driver->setPixelXY((int)m_rockets[i].x, y, Colors::White);
            }
            
            // Explode at top or when slowing
            if (m_rockets[i].vy > -0.1f || m_rockets[i].y < 4) {
                // Create explosion
                for (int p = 0; p < 20; p++) {
                    for (int j = 0; j < MAX_PARTICLES; j++) {
                        if (!m_particles[j].active) {
                            m_particles[j].active = true;
                            m_particles[j].x = m_rockets[i].x;
                            m_particles[j].y = m_rockets[i].y;
                            float angle = random8() * 0.0246f;  // 2*PI/255 = 0.0246
                            float speed = 0.3f + random8() * 0.0025f;  // /400 ≈ *0.0025
                            m_particles[j].vx = cosf(angle) * speed;
                            m_particles[j].vy = sinf(angle) * speed;
                            m_particles[j].hue = m_rockets[i].hue + random8(30);
                            m_particles[j].life = 40 + random8(30);
                            break;
                        }
                    }
                }
                m_rockets[i].active = false;
            }
        }
        
        // Update particles
        for (int i = 0; i < MAX_PARTICLES; i++) {
            if (!m_particles[i].active) continue;
            
            m_particles[i].x += m_particles[i].vx;
            m_particles[i].y += m_particles[i].vy;
            m_particles[i].vy += 0.02f;  // Gravity
            m_particles[i].life--;
            
            if (m_particles[i].life == 0 || m_particles[i].y > 16) {
                m_particles[i].active = false;
                continue;
            }
            
            int x = (int)m_particles[i].x;
            int y = (int)m_particles[i].y;
            if (x >= 0 && x < 16 && y >= 0 && y < 16) {
                uint8_t brightness = (m_particles[i].life > 30) ? 255 : m_particles[i].life * 8;
                m_driver->setPixelXY(x, y, RGB::fromHSV(m_particles[i].hue, 255, brightness));
            }
        }
    }
    
    void updateDemo() override {
        m_frame++;
        AudioData demo;
        demo.beat = (m_frame % 45) == 0;
        update(demo);
    }
    
    const char* getName() const override { return "Fireworks"; }
    
private:
    static const int MAX_PARTICLES = 100;
    static const int MAX_ROCKETS = 3;
    Particle m_particles[MAX_PARTICLES];
    Rocket m_rockets[MAX_ROCKETS];
};

// -----------------------------------------------------------
// Effect 11: Rainbow Wave
// Rainbow that pulses with bass
// -----------------------------------------------------------
class RainbowWaveEffect : public LedEffect {
public:
    void update(const AudioData& audio) override {
        m_frame++;
        
        float bassScale = 1.0f + audio.bass * 2.0f;
        uint8_t baseHue = m_frame;
        
        for (int y = 0; y < 16; y++) {
            for (int x = 0; x < 16; x++) {
                uint8_t hue = baseHue + x * 8 + y * 8 + (sin8(m_frame + x * 16) >> 2);  // /4 = >>2
                uint8_t sat = 255;
                uint8_t val = 128 + (uint8_t)(audio.bass * 127);
                
                // Wave distortion: /32 = >>5
                int waveOffset = (int)((sin8(m_frame * 2 + y * 20) >> 5) * bassScale);
                int xShift = x + waveOffset;
                if (xShift < 0) xShift += 16;
                if (xShift >= 16) xShift -= 16;
                
                m_driver->setPixelXY(xShift, y, RGB::fromHSV(hue, sat, val));
            }
        }
    }
    
    const char* getName() const override { return "Rainbow Wave"; }
};

// -----------------------------------------------------------
// Effect 12: Particle Burst
// Particles explode from center on beat
// -----------------------------------------------------------
class ParticleBurstEffect : public LedEffect {
    struct Particle {
        float x, y;
        float vx, vy;
        uint8_t hue;
        float life;
        bool active;
    };
    
public:
    void init(LedDriver* driver) override {
        LedEffect::init(driver);
        for (int i = 0; i < MAX_PARTICLES; i++) {
            m_particles[i].active = false;
        }
        m_hue = 0;
    }
    
    void update(const AudioData& audio) override {
        m_frame++;
        m_driver->fadeAll(220);
        
        // Burst on beat
        if (audio.beat) {
            m_hue += 40;
            int numParticles = 20 + (int)(audio.beatIntensity * 20);
            for (int i = 0; i < numParticles; i++) {
                for (int j = 0; j < MAX_PARTICLES; j++) {
                    if (!m_particles[j].active) {
                        m_particles[j].active = true;
                        m_particles[j].x = 7.5f;
                        m_particles[j].y = 7.5f;
                        float angle = random8() * 0.0246f;  // 2*PI/255
                        float speed = 0.2f + random8() * 0.00333f;  // /300 ≈ *0.00333
                        m_particles[j].vx = cosf(angle) * speed * (1.0f + audio.beatIntensity);
                        m_particles[j].vy = sinf(angle) * speed * (1.0f + audio.beatIntensity);
                        m_particles[j].hue = m_hue + random8(40);
                        m_particles[j].life = 1.0f;
                        break;
                    }
                }
            }
        }
        
        // Update particles
        for (int i = 0; i < MAX_PARTICLES; i++) {
            if (!m_particles[i].active) continue;
            
            m_particles[i].x += m_particles[i].vx;
            m_particles[i].y += m_particles[i].vy;
            m_particles[i].life -= 0.02f;
            
            if (m_particles[i].life <= 0 || 
                m_particles[i].x < 0 || m_particles[i].x >= 16 ||
                m_particles[i].y < 0 || m_particles[i].y >= 16) {
                m_particles[i].active = false;
                continue;
            }
            
            uint8_t brightness = (uint8_t)(m_particles[i].life * 255);
            m_driver->setPixelXY((int)m_particles[i].x, (int)m_particles[i].y,
                                RGB::fromHSV(m_particles[i].hue, 255, brightness));
        }
        
        // Center glow based on audio
        uint8_t centerBright = 50 + (uint8_t)(audio.bass * 100);
        for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
                RGB current = m_driver->getPixelXY(7 + dx, 7 + dy);
                m_driver->setPixelXY(7 + dx, 7 + dy, current.add(RGB::fromHSV(m_hue, 255, centerBright)));
            }
        }
    }
    
    void updateDemo() override {
        m_frame++;
        AudioData demo;
        demo.beat = (m_frame % 30) == 0;
        demo.beatIntensity = 0.7f;
        demo.bass = 0.3f;
        update(demo);
    }
    
    const char* getName() const override { return "Particle Burst"; }
    
private:
    static const int MAX_PARTICLES = 80;
    Particle m_particles[MAX_PARTICLES];
    uint8_t m_hue;
};

// -----------------------------------------------------------
// Effect 13: Kaleidoscope
// Kaleidoscope pattern reactive to audio
// -----------------------------------------------------------
class KaleidoscopeEffect : public LedEffect {
public:
    void update(const AudioData& audio) override {
        m_frame++;
        
        float scale = 1.0f + audio.bass * 0.5f;
        uint8_t hueOffset = m_frame + (uint8_t)(audio.mid * 50);
        
        // Only compute one quadrant, then mirror
        for (int y = 0; y < 8; y++) {
            for (int x = 0; x < 8; x++) {
                float dx = x - 3.5f;
                float dy = y - 3.5f;
                float angle = atan2f(dy, dx);
                float dist = sqrtf(dx * dx + dy * dy);
                
                uint8_t hue = hueOffset + (uint8_t)(angle * 40.5f) + (uint8_t)(dist * 20 * scale);
                uint8_t val = sin8((uint8_t)(dist * 30 - m_frame * 2));
                val = 100 + (val >> 1) + (uint8_t)(audio.bass * 80);
                
                RGB color = RGB::fromHSV(hue, 255, val);
                
                // Mirror to all 4 quadrants
                m_driver->setPixelXY(7 - x, 7 - y, color);
                m_driver->setPixelXY(8 + x, 7 - y, color);
                m_driver->setPixelXY(7 - x, 8 + y, color);
                m_driver->setPixelXY(8 + x, 8 + y, color);
            }
        }
    }
    
    const char* getName() const override { return "Kaleidoscope"; }
};

// -----------------------------------------------------------
// Effect 14: Frequency Spiral
// Spiral colored by frequency content
// -----------------------------------------------------------
class FrequencySpiralEffect : public LedEffect {
public:
    void update(const AudioData& audio) override {
        m_frame++;
        m_driver->fadeAll(230);
        
        float cx = 7.5f, cy = 7.5f;
        float angleOffset = m_frame * 0.05f;
        
        // Draw spiral arms
        for (int arm = 0; arm < 3; arm++) {
            float armAngle = angleOffset + arm * 2.0944f;  // 120 degrees apart
            float* level;
            uint8_t baseHue;
            
            if (arm == 0) { level = (float*)&audio.bass; baseHue = 0; }      // Red for bass
            else if (arm == 1) { level = (float*)&audio.mid; baseHue = 85; } // Green for mid
            else { level = (float*)&audio.high; baseHue = 170; }             // Blue for high
            
            float armLength = 2.0f + (*level) * 6.0f;
            
            for (float r = 0; r < armLength; r += 0.3f) {
                float angle = armAngle + r * 0.5f;
                int x = (int)(cx + cosf(angle) * r);
                int y = (int)(cy + sinf(angle) * r);
                
                if (x >= 0 && x < 16 && y >= 0 && y < 16) {
                    uint8_t brightness = 255 - (uint8_t)(r * 30);
                    RGB color = RGB::fromHSV(baseHue + (uint8_t)(r * 10), 255, brightness);
                    RGB current = m_driver->getPixelXY(x, y);
                    m_driver->setPixelXY(x, y, current.add(color));
                }
            }
        }
    }
    
    void updateDemo() override {
        AudioData demo;
        demo.bass = 0.4f + 0.3f * sinf(m_frame * 0.05f);
        demo.mid = 0.3f + 0.2f * sinf(m_frame * 0.07f + 1.0f);
        demo.high = 0.2f + 0.2f * sinf(m_frame * 0.09f + 2.0f);
        update(demo);
    }
    
    const char* getName() const override { return "Freq Spiral"; }
};

// -----------------------------------------------------------
// Effect 15: Bass Reactor
// Concentric rings that react to bass
// -----------------------------------------------------------
class BassReactorEffect : public LedEffect {
public:
    void init(LedDriver* driver) override {
        LedEffect::init(driver);
        for (int i = 0; i < 8; i++) {
            m_ringLevels[i] = 0;
        }
    }
    
    void update(const AudioData& audio) override {
        m_frame++;
        m_driver->clear();
        
        // Shift rings outward
        for (int i = 7; i > 0; i--) {
            m_ringLevels[i] = m_ringLevels[i - 1] * 0.9f;
        }
        m_ringLevels[0] = audio.bass;
        
        float cx = 7.5f, cy = 7.5f;
        
        for (int y = 0; y < 16; y++) {
            for (int x = 0; x < 16; x++) {
                float dx = x - cx, dy = y - cy;
                float dist = sqrtf(dx * dx + dy * dy);
                int ring = (int)dist;
                
                if (ring < 8) {
                    float level = m_ringLevels[ring];
                    uint8_t hue = m_frame + ring * 20;
                    uint8_t brightness = (uint8_t)(level * 255);
                    
                    // Add pulsing effect
                    // /5 ≈ (*51)>>8, but we want /5 of 0-255 = 0-51
                    brightness = (brightness * (200 + ((sin8(m_frame * 3 + ring * 30) * 51) >> 8))) >> 8;
                    
                    m_driver->setPixelXY(x, y, RGB::fromHSV(hue, 255, brightness));
                }
            }
        }
    }
    
    void updateDemo() override {
        m_frame++;
        AudioData demo;
        demo.bass = 0.4f + 0.4f * (sin8(m_frame * 3) > 200 ? 1.0f : 0.0f);
        update(demo);
    }
    
    const char* getName() const override { return "Bass Reactor"; }
    
private:
    float m_ringLevels[8];
};

// -----------------------------------------------------------
// Effect 16: Meteor Shower
// Meteors fall down, triggered by bass
// -----------------------------------------------------------
class MeteorShowerEffect : public LedEffect {
public:
    struct Meteor {
        float x, y;
        float speed;
        uint8_t hue;
        uint8_t tailLength;
        bool active;
    };
    
    void init(LedDriver* driver) override {
        LedEffect::init(driver);
        for (int i = 0; i < MAX_METEORS; i++) {
            m_meteors[i].active = false;
        }
    }
    
    void update(const AudioData& audio) override {
        m_frame++;
        
        // Fade existing pixels (trail effect)
        for (int i = 0; i < LED_MATRIX_COUNT; i++) {
            RGB pixel = m_driver->getPixel(i);
            pixel.r = scale8(pixel.r, 180);
            pixel.g = scale8(pixel.g, 180);
            pixel.b = scale8(pixel.b, 180);
            m_driver->setPixel(i, pixel);
        }
        
        // Spawn new meteor on bass hit
        if (audio.bass > 0.4f || (audio.beat && audio.beatIntensity > 0.3f)) {
            for (int i = 0; i < MAX_METEORS; i++) {
                if (!m_meteors[i].active) {
                    m_meteors[i].x = random8(16);
                    m_meteors[i].y = 0;
                    m_meteors[i].speed = 0.5f + audio.bass * 1.5f;
                    m_meteors[i].hue = random8();
                    m_meteors[i].tailLength = 4 + random8(6);
                    m_meteors[i].active = true;
                    break;
                }
            }
        }
        
        // Update and draw meteors
        for (int i = 0; i < MAX_METEORS; i++) {
            if (!m_meteors[i].active) continue;
            
            Meteor& m = m_meteors[i];
            
            // Draw meteor head
            int headX = (int)m.x;
            int headY = (int)m.y;
            if (headY >= 0 && headY < 16) {
                m_driver->setPixelXY(headX, headY, RGB::fromHSV(m.hue, 200, 255));
            }
            
            // Draw tail
            for (int t = 1; t < m.tailLength; t++) {
                int tailY = headY - t;
                if (tailY >= 0 && tailY < 16) {
                    uint8_t brightness = 255 - (t * 255 / m.tailLength);
                    m_driver->setPixelXY(headX, tailY, RGB::fromHSV(m.hue, 255, brightness));
                }
            }
            
            // Move meteor
            m.y += m.speed;
            if (m.y >= 16 + m.tailLength) {
                m.active = false;
            }
        }
    }
    
    void updateDemo() override {
        m_frame++;
        AudioData demo;
        demo.bass = (m_frame % 20 == 0) ? 0.8f : 0.1f;
        demo.beat = (m_frame % 30 == 0);
        demo.beatIntensity = demo.beat ? 0.7f : 0.0f;
        update(demo);
    }
    
    const char* getName() const override { return "Meteor Shower"; }
    
private:
    static const int MAX_METEORS = 8;
    Meteor m_meteors[MAX_METEORS];
};

// -----------------------------------------------------------
// Effect 17: Breathing
// Smooth breathing/pulsing that reacts to audio
// -----------------------------------------------------------
class BreathingEffect : public LedEffect {
public:
    void init(LedDriver* driver) override {
        LedEffect::init(driver);
        m_hue = 0;
        m_targetBrightness = 0;
        m_currentBrightness = 0;
    }
    
    void update(const AudioData& audio) override {
        m_frame++;
        
        // Calculate target brightness from audio
        float audioLevel = audio.bass * 0.5f + audio.mid * 0.3f + audio.high * 0.2f;
        m_targetBrightness = audioLevel;
        
        // Smooth transition (breathing effect)
        float breathSpeed = 0.08f + audioLevel * 0.15f;
        if (m_currentBrightness < m_targetBrightness) {
            m_currentBrightness += breathSpeed;
            if (m_currentBrightness > m_targetBrightness) 
                m_currentBrightness = m_targetBrightness;
        } else {
            m_currentBrightness -= breathSpeed * 0.5f;  // Slower exhale
            if (m_currentBrightness < 0.05f) 
                m_currentBrightness = 0.05f;  // Never fully off
        }
        
        // Shift hue slowly
        if (audio.beat) {
            m_hue += 15;
        } else {
            m_hue++;
        }
        
        // Apply breathing to all pixels with gradient
        uint8_t brightness = (uint8_t)(m_currentBrightness * 255);
        
        for (int y = 0; y < 16; y++) {
            for (int x = 0; x < 16; x++) {
                // Distance from center for radial gradient
                float dx = x - 7.5f, dy = y - 7.5f;
                float dist = sqrtf(dx * dx + dy * dy);
                float falloff = 1.0f - (dist * fast_recipsf2(11.0f));
                if (falloff < 0.1f) falloff = 0.1f;
                
                uint8_t pixelBright = scale8(brightness, (uint8_t)(falloff * 255));
                uint8_t hueOffset = (uint8_t)(dist * 3);
                m_driver->setPixelXY(x, y, RGB::fromHSV(m_hue + hueOffset, 220, pixelBright));
            }
        }
    }
    
    void updateDemo() override {
        m_frame++;
        AudioData demo;
        float breath = sin8(m_frame * 2) * fast_recipsf2(255.0f);
        demo.bass = breath * 0.6f + 0.2f;
        demo.mid = breath * 0.4f + 0.1f;
        demo.high = breath * 0.3f;
        demo.beat = (m_frame % 40 == 0);
        update(demo);
    }
    
    const char* getName() const override { return "Breathing"; }
    
private:
    uint8_t m_hue;
    float m_targetBrightness;
    float m_currentBrightness;
};

// -----------------------------------------------------------
// Effect 18: DNA Helix
// Double helix rotating, colored by frequency
// -----------------------------------------------------------
class DNAHelixEffect : public LedEffect {
public:
    void update(const AudioData& audio) override {
        m_frame++;
        m_driver->clear();
        
        // Rotation speed based on audio
        float rotSpeed = 0.05f + audio.mid * 0.15f;
        float phase = m_frame * rotSpeed;
        
        // Helix parameters
        float amplitude = 4.0f + audio.bass * 3.0f;
        
        for (int y = 0; y < 16; y++) {
            // First strand
            float angle1 = phase + y * 0.5f;
            int x1 = 8 + (int)(amplitude * sinf(angle1));
            
            // Second strand (180 degrees offset)
            float angle2 = phase + y * 0.5f + 3.14159f;
            int x2 = 8 + (int)(amplitude * sinf(angle2));
            
            // Clamp to matrix
            if (x1 >= 0 && x1 < 16) {
                uint8_t hue = (uint8_t)(audio.bass * 60);  // Red for bass
                uint8_t brightness = 180 + (uint8_t)(audio.bass * 75);
                m_driver->setPixelXY(x1, y, RGB::fromHSV(hue, 255, brightness));
            }
            
            if (x2 >= 0 && x2 < 16) {
                uint8_t hue = 160 + (uint8_t)(audio.high * 60);  // Blue for high
                uint8_t brightness = 180 + (uint8_t)(audio.high * 75);
                m_driver->setPixelXY(x2, y, RGB::fromHSV(hue, 255, brightness));
            }
            
            // Draw connecting rungs every 4 rows
            if ((y + m_frame / 4) % 4 == 0 && x1 != x2) {
                int minX = (x1 < x2) ? x1 : x2;
                int maxX = (x1 > x2) ? x1 : x2;
                for (int rx = minX + 1; rx < maxX; rx++) {
                    if (rx >= 0 && rx < 16) {
                        uint8_t brightness = 60 + (uint8_t)(audio.mid * 100);
                        m_driver->setPixelXY(rx, y, RGB::fromHSV(96, 200, brightness));  // Green rungs
                    }
                }
            }
        }
    }
    
    void updateDemo() override {
        m_frame++;
        AudioData demo;
        demo.bass = 0.4f + 0.2f * sinf(m_frame * 0.03f);
        demo.mid = 0.3f + 0.15f * sinf(m_frame * 0.05f);
        demo.high = 0.25f + 0.15f * sinf(m_frame * 0.07f);
        update(demo);
    }
    
    const char* getName() const override { return "DNA Helix"; }
};

// -----------------------------------------------------------
// Effect 19: Audio Scope
// Oscilloscope-style waveform visualization
// -----------------------------------------------------------
class AudioScopeEffect : public LedEffect {
public:
    void init(LedDriver* driver) override {
        LedEffect::init(driver);
        for (int i = 0; i < 16; i++) {
            m_waveform[i] = 8;
        }
    }
    
    void update(const AudioData& audio) override {
        m_frame++;
        m_driver->clear();
        
        // Shift waveform left
        for (int i = 0; i < 15; i++) {
            m_waveform[i] = m_waveform[i + 1];
        }
        
        // Add new sample based on audio mix
        float sample = audio.bass * 0.5f + audio.mid * 0.3f + audio.high * 0.2f;
        // Add some "noise" based on high frequency for texture
        sample += (random8() * fast_recipsf2(255.0f) - 0.5f) * audio.high * 0.3f;
        
        int newY = 8 + (int)(sample * 7);
        if (newY < 0) newY = 0;
        if (newY > 15) newY = 15;
        m_waveform[15] = newY;
        
        // Draw waveform with color based on intensity
        for (int x = 0; x < 16; x++) {
            int y = m_waveform[x];
            int prevY = (x > 0) ? m_waveform[x - 1] : y;
            
            // Draw line between points
            int minY = (y < prevY) ? y : prevY;
            int maxY = (y > prevY) ? y : prevY;
            
            for (int ly = minY; ly <= maxY; ly++) {
                // Color based on distance from center
                int dist = (ly > 8) ? (ly - 8) : (8 - ly);
                uint8_t hue = 160 - dist * 15;  // Blue to green to yellow
                uint8_t brightness = 150 + dist * 13;
                m_driver->setPixelXY(x, ly, RGB::fromHSV(hue, 255, brightness));
            }
        }
        
        // Draw center reference line (dim)
        for (int x = 0; x < 16; x++) {
            RGB current = m_driver->getPixelXY(x, 8);
            if (current.r == 0 && current.g == 0 && current.b == 0) {
                m_driver->setPixelXY(x, 8, RGB(15, 15, 15));
            }
        }
    }
    
    void updateDemo() override {
        m_frame++;
        AudioData demo;
        demo.bass = 0.3f + 0.25f * sinf(m_frame * 0.08f);
        demo.mid = 0.2f + 0.2f * sinf(m_frame * 0.12f);
        demo.high = 0.15f + 0.15f * sinf(m_frame * 0.18f);
        update(demo);
    }
    
    const char* getName() const override { return "Audio Scope"; }
    
private:
    int m_waveform[16];
};

// -----------------------------------------------------------
// Effect 20: Bouncing Balls
// Balls bounce to the beat
// -----------------------------------------------------------
class BouncingBallsEffect : public LedEffect {
public:
    struct Ball {
        float x, y;
        float vx, vy;
        uint8_t hue;
        uint8_t radius;
        bool active;
    };
    
    void init(LedDriver* driver) override {
        LedEffect::init(driver);
        // Initialize some balls
        for (int i = 0; i < MAX_BALLS; i++) {
            spawnBall(i);
        }
    }
    
    void spawnBall(int idx) {
        m_balls[idx].x = random8(4, 12);
        m_balls[idx].y = random8(4, 12);
        m_balls[idx].vx = (random8() * fast_recipsf2(255.0f) - 0.5f) * 0.5f;
        m_balls[idx].vy = 0;
        m_balls[idx].hue = random8();
        m_balls[idx].radius = 1 + random8(2);
        m_balls[idx].active = true;
    }
    
    void update(const AudioData& audio) override {
        m_frame++;
        m_driver->clear();
        
        // Gravity strength based on bass
        float gravity = 0.1f + audio.bass * 0.2f;
        
        // Beat creates upward force
        float bounce = 0.0f;
        if (audio.beat) {
            bounce = -2.0f * audio.beatIntensity;
        }
        
        for (int i = 0; i < MAX_BALLS; i++) {
            Ball& b = m_balls[i];
            
            // Apply gravity
            b.vy += gravity;
            
            // Apply beat bounce
            if (bounce < 0) {
                b.vy += bounce;
            }
            
            // Move
            b.x += b.vx;
            b.y += b.vy;
            
            // Bounce off walls
            if (b.x < b.radius) { b.x = b.radius; b.vx = -b.vx * 0.8f; }
            if (b.x >= 16 - b.radius) { b.x = 15 - b.radius; b.vx = -b.vx * 0.8f; }
            if (b.y < b.radius) { b.y = b.radius; b.vy = -b.vy * 0.7f; }
            if (b.y >= 16 - b.radius) { 
                b.y = 15 - b.radius; 
                b.vy = -b.vy * 0.7f;  // Dampened bounce
            }
            
            // Draw ball
            int cx = (int)b.x, cy = (int)b.y;
            uint8_t brightness = 200 + (uint8_t)(audio.bass * 55);
            
            for (int dy = -b.radius; dy <= b.radius; dy++) {
                for (int dx = -b.radius; dx <= b.radius; dx++) {
                    if (dx * dx + dy * dy <= b.radius * b.radius) {
                        int px = cx + dx, py = cy + dy;
                        if (px >= 0 && px < 16 && py >= 0 && py < 16) {
                            m_driver->setPixelXY(px, py, RGB::fromHSV(b.hue, 255, brightness));
                        }
                    }
                }
            }
            
            // Slowly shift hue
            b.hue++;
        }
    }
    
    void updateDemo() override {
        m_frame++;
        AudioData demo;
        demo.bass = 0.3f;
        demo.beat = (m_frame % 25 == 0);
        demo.beatIntensity = demo.beat ? 0.8f : 0.0f;
        update(demo);
    }
    
    const char* getName() const override { return "Bouncing Balls"; }
    
private:
    static const int MAX_BALLS = 5;
    Ball m_balls[MAX_BALLS];
};

// -----------------------------------------------------------
// Effect 21: Lava Lamp
// Blob-like lava lamp effect reactive to audio
// -----------------------------------------------------------
class LavaLampEffect : public LedEffect {
public:
    struct Blob {
        float x, y;
        float vx, vy;
        float radius;
        uint8_t hue;
    };
    
    void init(LedDriver* driver) override {
        LedEffect::init(driver);
        // Initialize blobs
        for (int i = 0; i < NUM_BLOBS; i++) {
            m_blobs[i].x = random8(4, 12);
            m_blobs[i].y = random8(4, 12);
            m_blobs[i].vx = (random8() * fast_recipsf2(255.0f) - 0.5f) * 0.3f;
            m_blobs[i].vy = (random8() * fast_recipsf2(255.0f) - 0.5f) * 0.3f;
            m_blobs[i].radius = 3.0f + random8(3);
            m_blobs[i].hue = random8();
        }
    }
    
    void update(const AudioData& audio) override {
        m_frame++;
        
        // Move blobs based on audio
        float movement = 0.05f + audio.mid * 0.15f;
        
        for (int i = 0; i < NUM_BLOBS; i++) {
            Blob& b = m_blobs[i];
            
            // Add some pseudo-random movement
            b.vx += (sinf(m_frame * 0.02f + i) * 0.02f);
            b.vy += (cosf(m_frame * 0.025f + i * 2) * 0.02f);
            
            // Audio influence
            b.vx += (audio.bass - 0.5f) * 0.05f;
            b.vy += (audio.high - 0.5f) * 0.05f;
            
            // Damping
            b.vx *= 0.95f;
            b.vy *= 0.95f;
            
            b.x += b.vx * movement * 10;
            b.y += b.vy * movement * 10;
            
            // Bounce off edges
            if (b.x < 2) { b.x = 2; b.vx = -b.vx; }
            if (b.x > 13) { b.x = 13; b.vx = -b.vx; }
            if (b.y < 2) { b.y = 2; b.vy = -b.vy; }
            if (b.y > 13) { b.y = 13; b.vy = -b.vy; }
            
            // Pulse radius with bass
            b.radius = 3.0f + audio.bass * 2.0f + sinf(m_frame * 0.05f + i) * 0.5f;
            
            // Shift hue slowly
            b.hue += (audio.beat ? 5 : 1);
        }
        
        // Render metaballs
        for (int y = 0; y < 16; y++) {
            for (int x = 0; x < 16; x++) {
                float sum = 0;
                float hueSum = 0;
                float hueWeight = 0;
                
                for (int i = 0; i < NUM_BLOBS; i++) {
                    float dx = x - m_blobs[i].x;
                    float dy = y - m_blobs[i].y;
                    float dist = sqrtf(dx * dx + dy * dy);
                    float influence = m_blobs[i].radius / (dist + 0.5f);
                    sum += influence;
                    hueSum += m_blobs[i].hue * influence;
                    hueWeight += influence;
                }
                
                if (sum > 1.0f) {
                    uint8_t avgHue = (uint8_t)(hueSum / hueWeight);
                    uint8_t brightness = (sum > 2.0f) ? 255 : (uint8_t)((sum - 1.0f) * 255);
                    m_driver->setPixelXY(x, y, RGB::fromHSV(avgHue, 200, brightness));
                } else {
                    m_driver->setPixelXY(x, y, RGB(0, 0, 0));
                }
            }
        }
    }
    
    void updateDemo() override {
        m_frame++;
        AudioData demo;
        demo.bass = 0.4f + 0.2f * sinf(m_frame * 0.04f);
        demo.mid = 0.3f + 0.15f * sinf(m_frame * 0.06f);
        demo.high = 0.25f + 0.15f * sinf(m_frame * 0.08f);
        demo.beat = (m_frame % 30 == 0);
        update(demo);
    }
    
    const char* getName() const override { return "Lava Lamp"; }
    
private:
    static const int NUM_BLOBS = 4;
    Blob m_blobs[NUM_BLOBS];
};

// ============================================================
// AMBIENT EFFECT - Configurable ambient lighting with user colors
// Supports: solid color, horizontal/vertical/radial/diagonal gradients
// User configurable: 2 colors, gradient type, speed, brightness
// ============================================================
class AmbientEffect : public LedEffect {
public:
    void init(LedDriver* driver) override {
        m_driver = driver;
        m_frame = 0;
        m_phase = 0.0f;
        // Default colors (can be updated via setSettings)
        m_color1 = RGB(255, 0, 128);   // Magenta-ish
        m_color2 = RGB(0, 128, 255);   // Cyan-ish
        m_gradientType = 0;            // None (solid)
        m_speed = 50;
        m_brightness = 128;
    }
    
    // Set LED settings from BLE: [brightness, r1, g1, b1, r2, g2, b2, gradient, speed, effectId]
    void setSettings(const uint8_t* data, size_t len) {
        if (len >= 10) {
            m_brightness = data[0];
            m_color1 = RGB(data[1], data[2], data[3]);
            m_color2 = RGB(data[4], data[5], data[6]);
            m_gradientType = data[7];
            m_speed = data[8];
            // data[9] is effectId, handled elsewhere
        }
    }
    
    void setBrightness(uint8_t brightness) { m_brightness = brightness; }
    void setColors(RGB c1, RGB c2) { m_color1 = c1; m_color2 = c2; }
    void setGradientType(uint8_t type) { m_gradientType = type; }
    void setSpeed(uint8_t speed) { m_speed = speed; }
    
    void update(const AudioData& audio) override {
        m_frame++;
        
        // Animation phase based on speed (0-255 maps to slow-fast)
        float speedFactor = (m_speed * fast_recipsf2(255.0f)) * 0.1f;
        m_phase += speedFactor;
        if (m_phase > 1.0f) m_phase -= 1.0f;
        
        // Render based on gradient type
        switch (m_gradientType) {
            case GRADIENT_NONE:
                renderSolid();
                break;
            case GRADIENT_LINEAR_H:
                renderLinearH();
                break;
            case GRADIENT_LINEAR_V:
                renderLinearV();
                break;
            case GRADIENT_RADIAL:
                renderRadial();
                break;
            case GRADIENT_DIAGONAL:
                renderDiagonal();
                break;
            default:
                renderSolid();
                break;
        }
    }
    
    void updateDemo() override {
        m_frame++;
        AudioData demo;
        demo.bass = 0.3f;
        demo.mid = 0.25f;
        demo.high = 0.2f;
        demo.beat = false;
        update(demo);
    }
    
    const char* getName() const override { return "Ambient"; }
    
private:
    RGB m_color1;
    RGB m_color2;
    uint8_t m_gradientType;
    uint8_t m_speed;
    uint8_t m_brightness;
    float m_phase;
    
    // Interpolate between two colors
    RGB lerpColor(RGB c1, RGB c2, float t) {
        // Add phase offset for animation
        t = fmodf(t + m_phase, 1.0f);
        uint8_t r = (uint8_t)(c1.r + (c2.r - c1.r) * t);
        uint8_t g = (uint8_t)(c1.g + (c2.g - c1.g) * t);
        uint8_t b = (uint8_t)(c1.b + (c2.b - c1.b) * t);
        return RGB(r, g, b);
    }
    
    RGB applyBrightness(RGB c) {
        float scale = m_brightness * fast_recipsf2(255.0f);
        return RGB((uint8_t)(c.r * scale), (uint8_t)(c.g * scale), (uint8_t)(c.b * scale));
    }
    
    void renderSolid() {
        RGB color = applyBrightness(m_color1);
        for (int y = 0; y < LED_MATRIX_HEIGHT; y++) {
            for (int x = 0; x < LED_MATRIX_WIDTH; x++) {
                m_driver->setPixelXY(x, y, color);
            }
        }
    }
    
    void renderLinearH() {
        for (int y = 0; y < LED_MATRIX_HEIGHT; y++) {
            for (int x = 0; x < LED_MATRIX_WIDTH; x++) {
                float t = (float)x / (LED_MATRIX_WIDTH - 1);
                RGB color = lerpColor(m_color1, m_color2, t);
                m_driver->setPixelXY(x, y, applyBrightness(color));
            }
        }
    }
    
    void renderLinearV() {
        for (int y = 0; y < LED_MATRIX_HEIGHT; y++) {
            float t = (float)y / (LED_MATRIX_HEIGHT - 1);
            RGB color = lerpColor(m_color1, m_color2, t);
            color = applyBrightness(color);
            for (int x = 0; x < LED_MATRIX_WIDTH; x++) {
                m_driver->setPixelXY(x, y, color);
            }
        }
    }
    
    void renderRadial() {
        float cx = (LED_MATRIX_WIDTH - 1) * 0.5f;
        float cy = (LED_MATRIX_HEIGHT - 1) * 0.5f;
        float maxDist = sqrtf(cx * cx + cy * cy);
        
        for (int y = 0; y < LED_MATRIX_HEIGHT; y++) {
            for (int x = 0; x < LED_MATRIX_WIDTH; x++) {
                float dx = x - cx;
                float dy = y - cy;
                float dist = sqrtf(dx * dx + dy * dy);
                float t = dist * fast_recipsf2(maxDist);
                RGB color = lerpColor(m_color1, m_color2, t);
                m_driver->setPixelXY(x, y, applyBrightness(color));
            }
        }
    }
    
    void renderDiagonal() {
        float maxDiag = (float)(LED_MATRIX_WIDTH + LED_MATRIX_HEIGHT - 2);
        float invMaxDiag = fast_recipsf2(maxDiag);
        for (int y = 0; y < LED_MATRIX_HEIGHT; y++) {
            for (int x = 0; x < LED_MATRIX_WIDTH; x++) {
                float t = (float)(x + y) * invMaxDiag;
                RGB color = lerpColor(m_color1, m_color2, t);
                m_driver->setPixelXY(x, y, applyBrightness(color));
            }
        }
    }
};

// ============================================================
// VOLUME VISUALIZER EFFECT
// Displays current volume level as a colorful bar/ring visualization
// ============================================================
class VolumeEffect : public LedEffect {
public:
    void init(LedDriver* driver) override {
        m_driver = driver;
        m_frame = 0;
        m_displayVolume = 0.0f;
        m_targetVolume = 0.0f;
        m_pulsePhase = 0.0f;
        m_lastChangeTime = 0;
    }
    
    void setVolume(uint8_t volume) {
        // Volume is 0-127 (A2DP range), convert to 0.0-1.0
        m_targetVolume = volume * fast_recipsf2(127.0f);
        m_lastChangeTime = m_frame;
    }
    
    void update(const AudioData& audio) override {
        m_frame++;
        
        // Smooth volume transitions
        float diff = m_targetVolume - m_displayVolume;
        if (fabsf(diff) > 0.001f) {
            m_displayVolume += diff * 0.2f;  // Smooth interpolation
        }
        
        // Pulse effect when volume recently changed
        uint32_t sinceLast = m_frame - m_lastChangeTime;
        float pulse = 0.0f;
        if (sinceLast < 30) {  // ~1 second of pulsing
            pulse = sinf(sinceLast * 0.3f) * (1.0f - sinceLast * fast_recipsf2(30.0f));
        }
        
        m_pulsePhase += 0.05f;
        if (m_pulsePhase > 6.28f) m_pulsePhase -= 6.28f;
        
        m_driver->clear();
        
        // Calculate how many rows to light up (bottom to top)
        int totalRows = LED_MATRIX_HEIGHT;
        int litRows = (int)(m_displayVolume * totalRows + 0.5f);
        
        // Render volume bar with color gradient
        for (int y = 0; y < LED_MATRIX_HEIGHT; y++) {
            int rowFromBottom = LED_MATRIX_HEIGHT - 1 - y;
            
            if (rowFromBottom < litRows) {
                // Color gradient: green (low) -> yellow (mid) -> red (high)
                float rowLevel = (float)rowFromBottom / (totalRows - 1);
                uint8_t hue;
                if (rowLevel < 0.5f) {
                    hue = 96;  // Green
                } else if (rowLevel < 0.75f) {
                    hue = 64;  // Yellow
                } else if (rowLevel < 0.9f) {
                    hue = 32;  // Orange
                } else {
                    hue = 0;   // Red
                }
                
                // Add brightness variation along x for wave effect
                for (int x = 0; x < LED_MATRIX_WIDTH; x++) {
                    float xWave = sinf(x * 0.4f + m_pulsePhase) * 0.15f + 0.85f;
                    float brightness = xWave * (1.0f + pulse * 0.3f);
                    if (brightness > 1.0f) brightness = 1.0f;
                    
                    uint8_t v = (uint8_t)(brightness * 255);
                    RGB color = RGB::fromHSV(hue, 255, v);
                    m_driver->setPixelXY(x, y, color);
                }
            } else if (rowFromBottom == litRows && litRows < totalRows) {
                // Top edge glow - dimmer
                uint8_t hue = (litRows < totalRows / 2) ? 96 : 
                              (litRows < totalRows * 3 / 4) ? 64 : 32;
                uint8_t v = 60 + (uint8_t)(pulse * 40);
                RGB color = RGB::fromHSV(hue, 255, v);
                for (int x = 0; x < LED_MATRIX_WIDTH; x++) {
                    m_driver->setPixelXY(x, y, color);
                }
            }
        }
        
        // Add volume percentage indicator in center (using dots)
        int percent = (int)(m_displayVolume * 100);
        renderVolumeNumber(percent);
    }
    
    void updateDemo() override {
        m_frame++;
        // Demo mode: cycle through volume levels
        float demoLevel = (sinf(m_frame * 0.02f) + 1.0f) * 0.5f;
        m_targetVolume = demoLevel;
        AudioData demo = {};
        update(demo);
    }
    
    const char* getName() const override { return "Volume"; }
    
private:
    float m_displayVolume;
    float m_targetVolume;
    float m_pulsePhase;
    uint32_t m_lastChangeTime;
    
    // Simple digit rendering in center of matrix
    void renderVolumeNumber(int percent) {
        // Only render if there's enough space (16x16 matrix)
        if (LED_MATRIX_WIDTH < 16 || LED_MATRIX_HEIGHT < 16) return;
        
        // Draw percentage in 3x5 pixel digits at center
        int cx = LED_MATRIX_WIDTH / 2;
        int cy = LED_MATRIX_HEIGHT / 2;
        
        RGB white(200, 200, 200);
        
        // For simplicity, just draw 3 dots to indicate volume range
        // Low (left dot), Medium (center dot), High (right dot)
        int dots = (percent < 33) ? 1 : (percent < 66) ? 2 : 3;
        
        for (int i = 0; i < dots; i++) {
            int dx = cx - 3 + i * 3;
            m_driver->setPixelXY(dx, cy, white);
            m_driver->setPixelXY(dx + 1, cy, white);
            m_driver->setPixelXY(dx, cy + 1, white);
            m_driver->setPixelXY(dx + 1, cy + 1, white);
        }
    }
};
