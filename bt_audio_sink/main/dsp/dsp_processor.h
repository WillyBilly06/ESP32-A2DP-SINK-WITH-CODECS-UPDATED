#pragma once

// -----------------------------------------------------------
// DSP Processor - handles all audio signal processing
// - 3-band EQ (bass/mid/treble shelving)
// - Crossover split-ear mode
// - Bass boost
// - Goertzel frequency analysis
// -----------------------------------------------------------

#include <stdint.h>
#include <math.h>
#include <new>
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "biquad.h"
#include "goertzel.h"
#include "fast_math.h"
#include "../config/app_config.h"

class DSPProcessor {
public:
    DSPProcessor();

    // Initialize/update for given sample rate
    void init(uint32_t sampleRate);
    void setSampleRate(uint32_t sampleRate);
    void updateSampleRate(uint32_t sampleRate) { setSampleRate(sampleRate); }
    void resetAllFilters();

    // EQ settings
    void setEQ(float bassDB, float midDB, float trebleDB);
    void setEQ(int8_t bassDB, int8_t midDB, int8_t trebleDB, uint32_t sampleRate) {
        if (sampleRate != m_sampleRate) setSampleRate(sampleRate);
        setEQ((float)bassDB, (float)midDB, (float)trebleDB);
    }
    float getBassDB() const { return m_eqBassDB; }
    float getMidDB() const { return m_eqMidDB; }
    float getTrebleDB() const { return m_eqTrebleDB; }

    // Control flags
    void setBassBoost(bool enable) { m_bassBoostEnabled = enable; }
    void setChannelFlip(bool enable) { m_channelFlipEnabled = enable; }
    void setBypass(bool enable) { m_bypassEnabled = enable; }
    void setAnalysisEnabled(bool enable) { m_analysisEnabled = enable; }
    void set3DSound(bool enable);  // 3D sound removed: no-op

    bool isBassBoostEnabled() const { return m_bassBoostEnabled; }
    bool isChannelFlipEnabled() const { return m_channelFlipEnabled; }
    bool isBypassEnabled() const { return m_bypassEnabled; }
    bool isAnalysisEnabled() const { return m_analysisEnabled; }
    bool is3DSoundEnabled() const { return false; }

    // Volume-based bass compensation (0-127 A2DP range)
    // As volume decreases, bass boost increases (max +3dB at 0% volume)
    void setVolume(uint8_t volume) {
        m_volumeGain = (volume == 0) ? 0.0f : ((float)volume * fast_recipsf2(127.0f));
        m_volume = volume;
        updateBassCompensation();
    }
    uint8_t getVolume() const { return m_volume; }
    float getBassCompensationDB() const { return m_bassCompensationDB; }
    
    // LED audio boost factor - increases as volume decreases for better reactivity
    // Returns a linear gain multiplier to compensate for low volume
    // At 1% volume we need ~100x boost to get similar LED reactivity as 100% volume
    float getLedAudioBoost() const {
        // Volume 0-127 maps to 0-100%
        // At 100% volume: boost = 1.0 (no boost)
        // At 1% volume: boost = ~100.0 (100x boost for full LED visibility)
        // Use exponential curve for smoother transition at low volumes
        float volumePct = (float)m_volume * fast_recipsf2(127.0f);
        if (volumePct < 0.01f) volumePct = 0.01f;  // Clamp minimum to 1%
        
        // Inverse relationship: boost = 1/volumePct, clamped to 100x max
        float boost = fast_recipsf2(volumePct);
        if (boost > 100.0f) boost = 100.0f;
        return boost;
    }

    // Process a single stereo sample (in-place)
    void processStereo(float &L, float &R);
    void processStereoQ31(int32_t &L, int32_t &R);

    // Output limiter diagnostics. Read+reset together once per second. peakIn is
    // the loudest pre-clip sample (1.0 == 0 dBFS; >1.0 means the chain overshot
    // full scale and the limiter had to compress it). satCount/overCount are the
    // number of samples that engaged the soft knee / would have hard-clipped.
    void getAndResetClipStats(float &peakIn, uint32_t &satCount, uint32_t &overCount) {
        peakIn = m_clipper.peakIn;
        satCount = m_clipper.satCount;
        overCount = m_clipper.overCount;
        m_clipper.peakIn = 0.0f;
        m_clipper.satCount = 0;
        m_clipper.overCount = 0;
    }

    // Get Goertzel levels (legacy - keeping for beat detection)
    float getDB(int band) const { return m_goertzel.getDB(band); }
    float getLin(int band) const { return m_goertzel.getLin(band); }
    void zeroLevels() { m_goertzel.zeroLevels(); m_peakMeter.zero(); }
    
    // Named accessors for Goertzel bands (used for beat detection)
    float getGoertzel30dB() const { return m_goertzel.getDB(0); }
    float getGoertzel60dB() const { return m_goertzel.getDB(1); }
    float getGoertzel100dB() const { return m_goertzel.getDB(2); }
    float getGoertzel30Lin() const { return m_goertzel.getLin(0); }
    float getGoertzel60Lin() const { return m_goertzel.getLin(1); }
    float getGoertzel100Lin() const { return m_goertzel.getLin(2); }

    // Fast peak meter levels (for BLE level display) - much more responsive
    float getPeakDB(int band) const { return m_peakMeter.getDB(band); }
    float getPeakLin(int band) const { return m_peakMeter.getLin(band); }

    // Get control byte for BLE
    uint8_t getControlByte() const;
    void applyControlByte(uint8_t v);

private:
    void updateFilters();
    void updateLPAlpha();

    // -----------------------------------------------------------
    // Soft Clipper (replaces compressor for better sound quality)
    // - No envelope tracking = instant response, zero latency effect
    // - Tanh-style soft clipping = gentle saturation instead of hard clip
    // - Much lower CPU usage than compressor
    // - Preserves transients and dynamics better
    // - Threshold raised to 0.98 to avoid bass distortion
    // -----------------------------------------------------------
    struct SoftClipper {
        float threshold = 0.85f;     // Linear (transparent) below this level
        float ceiling = 1.0f;        // Hard asymptote the output can never exceed
        float invRange = 1.0f / (1.0f - 0.85f);  // 1/(1-threshold), precomputed

        // Diagnostics (read+reset once per second by the stats logger)
        uint32_t satCount = 0;       // samples that entered saturation (|x|>threshold)
        uint32_t overCount = 0;      // samples that overshot full scale (|x|>=1.0)
        float peakIn = 0.0f;         // loudest |x| seen pre-clip (1.0 == 0 dBFS)

        void init(float /* sampleRate */) {
            invRange = 1.0f / (1.0f - threshold);
        }

        // True soft-knee saturator (replaces the old brick-wall clamp).
        //
        // Below `threshold` the signal passes through untouched. Above it, the
        // excess is smoothly compressed with a rational curve that is
        // C1-continuous at the knee (slope 1) and asymptotes to `ceiling`, so the
        // output NEVER exceeds full scale yet never flat-tops into a square wave.
        // This matters at max volume, where the +3 dB crossover makeup (1.41x)
        // plus bass boost drives the chain past 0 dBFS: the old hard clamp turned
        // that overshoot into static; this turns it into gentle, analog-style
        // saturation. The curve uses one divide and only runs on peaks above the
        // knee, so it stays cheap even at 96 kHz.
        inline float softClip(float x) {
            float ax = x < 0.0f ? -x : x;
            if (ax > peakIn) peakIn = ax;            // track loudest input peak
            if (ax <= threshold) return x;
            satCount++;                              // limiter engaged on this sample
            if (ax >= ceiling) overCount++;          // would have hard-clipped before
            float sign = x < 0.0f ? -1.0f : 1.0f;
            float u = (ax - threshold) * invRange;   // normalized overshoot >= 0
            float y = threshold + (ceiling - threshold) * (u / (1.0f + u));
            return sign * y;
        }

        // Process stereo pair
        void process(float &L, float &R) {
            L = softClip(L);
            R = softClip(R);
        }
    };
    
    SoftClipper m_clipper;

    // -----------------------------------------------------------
    // 3D Sound Crossfeed Processor
    // Creates a more spacious, speaker-like sound from headphones/stereo
    // by blending a delayed, filtered version of each channel into the other
    // -----------------------------------------------------------
    // ============================================================
    // STAGE PRESENCE 3D PROCESSOR
    // Creates a concert/stage soundstage experience:
    // - Bass stays TIGHT and CENTERED (punchy, not muddy)
    // - Mids/Vocals/Instruments spread WIDE like a real stage
    // - Early reflections simulate room acoustics
    // - Creates depth - sounds like performers are IN FRONT of you
    // ============================================================
    struct Immersive3DProcessor {
        static constexpr int MAX_DELAY = 256;  // Power of 2 for fast wrap
        
        // === STAGE EFFECT PARAMETERS ===
        static constexpr float BASS_CROSSOVER = 180.0f;   // Hz - keep bass below this centered
        static constexpr float MID_WIDTH = 2.2f;          // Dramatic widening for mids
        static constexpr float HIGH_WIDTH = 2.5f;         // Even wider for highs/presence
        
        // Early reflections (simulates room/stage)
        static constexpr float REFLECT1_MS = 8.0f;        // First reflection
        static constexpr float REFLECT2_MS = 15.0f;       // Second reflection
        static constexpr float REFLECT3_MS = 23.0f;       // Third reflection
        static constexpr float REFLECT_GAIN1 = 0.25f;     // Strong first reflection
        static constexpr float REFLECT_GAIN2 = 0.18f;     // Medium second
        static constexpr float REFLECT_GAIN3 = 0.12f;     // Subtle third
        
        // Depth enhancement
        static constexpr float DEPTH_DELAY_MS = 4.0f;     // Creates front-stage depth
        static constexpr float DEPTH_GAIN = 0.35f;
        
        // Delay buffers for reflections
        float delayL[MAX_DELAY];
        float delayR[MAX_DELAY];
        int writeIdx = 0;
        int reflect1Samples = 0;
        int reflect2Samples = 0;
        int reflect3Samples = 0;
        int depthSamples = 0;
        
        // Bass extraction filter (LP for bass, HP for mids/highs)
        float bassLpCoef = 0;
        float bassStateL = 0, bassStateR = 0;
        
        // Mid/high presence filter (emphasize 2-6kHz for clarity)
        float presenceHpCoef = 0;
        float presenceStateL = 0, presenceStateR = 0;
        
        // All-pass for externalization (phase decorrelation)
        float apCoef1 = 0.6f, apCoef2 = -0.4f;
        float apState1L = 0, apState1R = 0;
        float apState2L = 0, apState2R = 0;
        
        // Soft saturation for warmth
        inline float softClip(float x) {
            if (x > 1.0f) return 1.0f;
            if (x < -1.0f) return -1.0f;
            return x * (1.5f - 0.5f * x * x);
        }
        
        void init(float sampleRate) {
            // Clear delay buffers
            for (int i = 0; i < MAX_DELAY; i++) {
                delayL[i] = 0.0f;
                delayR[i] = 0.0f;
            }
            writeIdx = 0;
            bassStateL = bassStateR = 0;
            presenceStateL = presenceStateR = 0;
            apState1L = apState1R = 0;
            apState2L = apState2R = 0;
            
            // Calculate reflection delay samples
            reflect1Samples = (int)(sampleRate * REFLECT1_MS * 0.001f);
            reflect2Samples = (int)(sampleRate * REFLECT2_MS * 0.001f);
            reflect3Samples = (int)(sampleRate * REFLECT3_MS * 0.001f);
            depthSamples = (int)(sampleRate * DEPTH_DELAY_MS * 0.001f);
            
            // Clamp to buffer size
            if (reflect1Samples >= MAX_DELAY) reflect1Samples = MAX_DELAY - 1;
            if (reflect2Samples >= MAX_DELAY) reflect2Samples = MAX_DELAY - 1;
            if (reflect3Samples >= MAX_DELAY) reflect3Samples = MAX_DELAY - 1;
            if (depthSamples >= MAX_DELAY) depthSamples = MAX_DELAY - 1;
            
            // Bass LP filter coefficient (~180Hz crossover)
            float wc = 2.0f * DSP_PI_F * BASS_CROSSOVER;
            bassLpCoef = wc * fast_recipsf2(wc + sampleRate);
            
            // Presence HP filter (~300Hz to separate from bass)
            float wcHp = 2.0f * DSP_PI_F * 300.0f;
            presenceHpCoef = sampleRate * fast_recipsf2(wcHp + sampleRate);
        }
        
        inline void process(float &L, float &R) {
            // ========== 1. BASS/MID SEPARATION ==========
            // Extract bass (keep centered) and mids/highs (widen)
            bassStateL += bassLpCoef * (L - bassStateL);
            bassStateR += bassLpCoef * (R - bassStateR);
            
            float bassL = bassStateL;
            float bassR = bassStateR;
            float bassMono = (bassL + bassR) * 0.5f;  // Center the bass!
            
            // Mids/highs = original minus bass
            float midHighL = L - bassL;
            float midHighR = R - bassR;
            
            // ========== 2. DRAMATIC MID/HIGH WIDENING ==========
            // M-S processing on mids/highs only (not bass!)
            float mid = (midHighL + midHighR) * 0.5f;
            float side = (midHighL - midHighR) * 0.5f;
            
            // Progressive widening - highs wider than mids
            // Simple approach: boost side channel significantly
            side *= MID_WIDTH;
            
            // Reconstruct widened mids/highs
            float wideL = mid + side;
            float wideR = mid - side;
            
            // ========== 3. STORE IN DELAY BUFFER ==========
            delayL[writeIdx] = wideL;
            delayR[writeIdx] = wideR;
            
            // ========== 4. EARLY REFLECTIONS (Stage Acoustics) ==========
            // Multiple short delays create sense of space/room
            int idx1 = (writeIdx - reflect1Samples) & (MAX_DELAY - 1);
            int idx2 = (writeIdx - reflect2Samples) & (MAX_DELAY - 1);
            int idx3 = (writeIdx - reflect3Samples) & (MAX_DELAY - 1);
            int depthIdx = (writeIdx - depthSamples) & (MAX_DELAY - 1);
            
            // Cross-channel reflections (sound bouncing around stage)
            float ref1L = delayR[idx1] * REFLECT_GAIN1;  // From opposite side
            float ref1R = delayL[idx1] * REFLECT_GAIN1;
            float ref2L = delayL[idx2] * REFLECT_GAIN2;  // Same side bounce
            float ref2R = delayR[idx2] * REFLECT_GAIN2;
            float ref3L = delayR[idx3] * REFLECT_GAIN3;  // Cross again
            float ref3R = delayL[idx3] * REFLECT_GAIN3;
            
            // Depth delay (creates front-stage positioning)
            float depthL = delayL[depthIdx] * DEPTH_GAIN;
            float depthR = delayR[depthIdx] * DEPTH_GAIN;
            
            // ========== 5. EXTERNALIZATION (Out-of-Head) ==========
            // Cascaded all-pass filters for phase decorrelation
            // This makes sound feel like it's OUTSIDE your head
            float ap1OutL = apCoef1 * wideL + apState1L;
            apState1L = wideL - apCoef1 * ap1OutL;
            float ap1OutR = apCoef1 * wideR + apState1R;
            apState1R = wideR - apCoef1 * ap1OutR;
            
            // Second stage with different coefficient
            float ap2OutL = apCoef2 * ap1OutL + apState2L;
            apState2L = ap1OutL - apCoef2 * ap2OutL;
            float ap2OutR = apCoef2 * ap1OutR + apState2R;
            apState2R = ap1OutR - apCoef2 * ap2OutR;
            
            // Blend original and phase-shifted for subtle effect
            float extL = wideL * 0.5f + ap2OutL * 0.5f;
            float extR = wideR * 0.5f + ap2OutR * 0.5f;
            
            // ========== 6. FINAL MIX ==========
            // Bass: centered mono (punchy, tight)
            // Mids/Highs: widened + reflections + depth + externalization
            float outL = bassMono * 0.9f + extL * 0.65f + depthL + ref1L + ref2L + ref3L;
            float outR = bassMono * 0.9f + extR * 0.65f + depthR + ref1R + ref2R + ref3R;
            
            // Soft clip for warmth and protection
            L = softClip(outL);
            R = softClip(outR);
            
            // Advance write index
            writeIdx = (writeIdx + 1) & (MAX_DELAY - 1);
        }
        
        void reset() {
            for (int i = 0; i < MAX_DELAY; i++) {
                delayL[i] = 0.0f;
                delayR[i] = 0.0f;
            }
            writeIdx = 0;
            bassStateL = bassStateR = 0;
            presenceStateL = presenceStateR = 0;
            apState1L = apState1R = 0;
            apState2L = apState2R = 0;
        }
    };
    
    // 3D sound removed from this low-RAM build.

    // -----------------------------------------------------------
    // 3-Band Peak Meter for 30Hz, 60Hz, 100Hz display
    // - 3 LP filters at different cutoffs to capture each sub-bass band
    // - Instant attack, smooth release for punchy visual response
    // - Ultra-lightweight: just 3 cascaded 1-pole LP filters
    // -----------------------------------------------------------
    struct PeakMeter {
        static constexpr int NUM_BANDS = 3;  // 30Hz, 60Hz, 100Hz
        
        // Release envelope
        float releaseCoef = 0.0f;
        float releaseCoefInv = 0.0f;
        
        // LP filter for 30Hz band (~45Hz cutoff)
        float lp30State = 0.0f;
        float lp30Coef = 0.0f;
        float lp30CoefInv = 0.0f;
        float lp30Envelope = 0.0f;
        float lp30PeakLin = 0.0f;
        
        // LP filter for 60Hz band (~80Hz cutoff)
        float lp60State = 0.0f;
        float lp60Coef = 0.0f;
        float lp60CoefInv = 0.0f;
        float lp60Envelope = 0.0f;
        float lp60PeakLin = 0.0f;
        
        // LP filter for 100Hz band (~120Hz cutoff)
        float lp100State = 0.0f;
        float lp100Coef = 0.0f;
        float lp100CoefInv = 0.0f;
        float lp100Envelope = 0.0f;
        float lp100PeakLin = 0.0f;
        
        // Pre-computed constant for dB conversion
        static constexpr float DB_SCALE = 8.685889638f;  // 20/ln(10)
        
        void init(float sampleRate) {
            if (sampleRate <= 0) sampleRate = 44100.0f;
            
            // Release time ~50ms for smooth response
            float invSampleRate = fast_recipsf2(sampleRate);
            float releaseMs = 50.0f;
            float releaseSamples = releaseMs * 0.001f * sampleRate;
            releaseCoef = expf(-fast_recipsf2(releaseSamples));
            releaseCoefInv = 1.0f - releaseCoef;
            
            float dt = invSampleRate;
            
            // LP for 30Hz band (~45Hz cutoff)
            float fc30 = 45.0f;
            float rc30 = fast_recipsf2(2.0f * DSP_PI_F * fc30);
            lp30Coef = dt * fast_recipsf2(rc30 + dt);
            lp30CoefInv = 1.0f - lp30Coef;
            
            // LP for 60Hz band (~80Hz cutoff)
            float fc60 = 80.0f;
            float rc60 = fast_recipsf2(2.0f * DSP_PI_F * fc60);
            lp60Coef = dt * fast_recipsf2(rc60 + dt);
            lp60CoefInv = 1.0f - lp60Coef;
            
            // LP for 100Hz band (~120Hz cutoff)
            float fc100 = 120.0f;
            float rc100 = fast_recipsf2(2.0f * DSP_PI_F * fc100);
            lp100Coef = dt * fast_recipsf2(rc100 + dt);
            lp100CoefInv = 1.0f - lp100Coef;
            
            zero();
        }
        
        // Process with 3 LP filters for 30Hz, 60Hz, 100Hz bands
        inline void process(float x) {
            // LP for 30Hz band (~45Hz cutoff)
            lp30State = lp30CoefInv * lp30State + lp30Coef * x;
            float lp30Abs = lp30State >= 0.0f ? lp30State : -lp30State;
            if (lp30Abs > lp30Envelope) {
                lp30Envelope = lp30Abs;
            } else {
                lp30Envelope = releaseCoef * lp30Envelope + releaseCoefInv * lp30Abs;
            }
            lp30PeakLin = lp30Envelope;
            
            // LP for 60Hz band (~80Hz cutoff)
            lp60State = lp60CoefInv * lp60State + lp60Coef * x;
            float lp60Abs = lp60State >= 0.0f ? lp60State : -lp60State;
            if (lp60Abs > lp60Envelope) {
                lp60Envelope = lp60Abs;
            } else {
                lp60Envelope = releaseCoef * lp60Envelope + releaseCoefInv * lp60Abs;
            }
            lp60PeakLin = lp60Envelope;
            
            // LP for 100Hz band (~120Hz cutoff)
            lp100State = lp100CoefInv * lp100State + lp100Coef * x;
            float lp100Abs = lp100State >= 0.0f ? lp100State : -lp100State;
            if (lp100Abs > lp100Envelope) {
                lp100Envelope = lp100Abs;
            } else {
                lp100Envelope = releaseCoef * lp100Envelope + releaseCoefInv * lp100Abs;
            }
            lp100PeakLin = lp100Envelope;
        }
        
        // Get dB for band: 0=30Hz, 1=60Hz, 2=100Hz
        float getDB(int band) const {
            float lin;
            if (band == 0) lin = lp30PeakLin;       // 30Hz (LP ~45Hz)
            else if (band == 1) lin = lp60PeakLin;  // 60Hz (LP ~80Hz)
            else lin = lp100PeakLin;                 // 100Hz (LP ~120Hz)
            
            if (lin < 1e-6f) return -60.0f;
            float dB = DB_SCALE * fast_logf(lin);
            if (dB < -60.0f) return -60.0f;
            if (dB > 0.0f) return 0.0f;
            return dB;
        }
        
        float getLin(int band) const {
            if (band == 0) return lp30PeakLin;
            if (band == 1) return lp60PeakLin;
            return lp100PeakLin;
        }
        
        void zero() {
            lp30State = 0.0f;
            lp30Envelope = 0.0f;
            lp30PeakLin = 0.0f;
            lp60State = 0.0f;
            lp60Envelope = 0.0f;
            lp60PeakLin = 0.0f;
            lp100State = 0.0f;
            lp100Envelope = 0.0f;
            lp100PeakLin = 0.0f;
        }
    };
    
    PeakMeter m_peakMeter;

    // Sample rate
    uint32_t m_sampleRate;

    // EQ filters
    Biquad m_eqBassL, m_eqBassR;
    Biquad m_eqMidL, m_eqMidR;
    Biquad m_eqTrebleL, m_eqTrebleR;

    // Bass boost shelf filters (separate for L/R stereo)
    Biquad m_bassShelfL, m_bassShelfR;

    // Crossover filters
    Biquad m_crossoverLPL, m_crossoverHPR;

    // Goertzel frequency analysis
    GoertzelBank m_goertzel;

    // EQ gains
    float m_eqBassDB;
    float m_eqMidDB;
    float m_eqTrebleDB;
    bool m_eqActive;

    // 1-pole low-pass state
    float m_lpAlpha;
    float m_lpState;

    // Control flags
    bool m_bassBoostEnabled;
    bool m_channelFlipEnabled;
    bool m_bypassEnabled;
    bool m_analysisEnabled;

    // Analysis decimation: the Goertzel + peak meter only need a low effective
    // rate (~11 kHz) to resolve 30/60/100 Hz. Running them at the full codec
    // rate (esp. 88.2/96 kHz) starves the render task. We box-average the input
    // down by m_analysisDecim and run the analysis on that, while the Goertzel
    // window (N) and the peak-meter coefficients are computed for the resulting
    // effective rate so detection behaviour and update rate are unchanged.
    int   m_analysisDecim;
    int   m_analysisDecimCount;
    float m_analysisAccum;

    // Volume-based bass compensation
    uint8_t m_volume;           // Current volume (0-127)
    float m_volumeGain;          // Linear gain (0.0 - 1.0) applied to output
    float m_bassCompensationDB; // Calculated bass boost in dB
    Biquad m_bassCompL, m_bassCompR;  // Bass compensation filters

private:
    void updateBassCompensation();
    void updateAnalysisRate();
};

// -----------------------------------------------------------
// Implementation
// -----------------------------------------------------------

inline DSPProcessor::DSPProcessor() 
    : m_sampleRate(APP_I2S_DEFAULT_SR)
    , m_eqBassDB(0.0f)
    , m_eqMidDB(0.0f)
    , m_eqTrebleDB(0.0f)
    , m_eqActive(false)
    , m_lpAlpha(0.0f)
    , m_lpState(0.0f)
    , m_bassBoostEnabled(false)
    , m_channelFlipEnabled(false)
    , m_bypassEnabled(false)
    , m_analysisEnabled(true)
    , m_analysisDecim(1)
    , m_analysisDecimCount(0)
    , m_analysisAccum(0.0f)
    , m_volume(127)
    , m_bassCompensationDB(0.0f)
{
}

inline void DSPProcessor::set3DSound(bool /*enable*/) {
    // 3D sound has been removed in the low-RAM build.
    // Keep this API as a no-op so older BLE/encoder code still compiles.
}

// Decimate the analysis path to a fixed ~11 kHz effective rate and (re)init the
// Goertzel + peak meter for that effective rate. The Goertzel scales its window
// from the rate it is given, so detection bins and the ~86 Hz update rate stay
// identical across 44.1/48/88.2/96 kHz while the filters run 4-9x less often.
inline void DSPProcessor::updateAnalysisRate() {
    static constexpr float TARGET_ANALYSIS_HZ = 11025.0f;
    uint32_t fs = m_sampleRate ? m_sampleRate : APP_I2S_DEFAULT_SR;
    int d = (int)lroundf((float)fs / TARGET_ANALYSIS_HZ);
    if (d < 1) d = 1;
    m_analysisDecim = d;
    m_analysisDecimCount = 0;
    m_analysisAccum = 0.0f;
    uint32_t effRate = fs / (uint32_t)d;
    m_goertzel.init(effRate);
    m_peakMeter.init((float)effRate);
}

inline void DSPProcessor::init(uint32_t sampleRate) {
    m_sampleRate = sampleRate > 0 ? sampleRate : APP_I2S_DEFAULT_SR;
    updateFilters();
    updateLPAlpha();
    updateAnalysisRate();
    m_clipper.init((float)m_sampleRate);
    updateBassCompensation();  // Initialize bass compensation filter
}

inline void DSPProcessor::setSampleRate(uint32_t sampleRate) {
    if (sampleRate == m_sampleRate && sampleRate != 0) return;
    m_sampleRate = sampleRate > 0 ? sampleRate : APP_I2S_DEFAULT_SR;
    updateFilters();
    updateLPAlpha();
    updateBassCompensation();  // Re-initialize bass compensation filter for new sample rate
    resetAllFilters();  // Clear all filter states to prevent noise on codec switch
    updateAnalysisRate();
    m_clipper.init((float)m_sampleRate);
    m_lpState = 0.0f;  // Reset filter state
}

inline void DSPProcessor::setEQ(float bassDB, float midDB, float trebleDB) {
    // Scale input range from ±12dB (phone) to actual audio range
    // Bass scaled more conservatively to prevent clipping
    // Mid/treble can be more aggressive as they clip less
    m_eqBassDB = bassDB * 0.5f;
    m_eqMidDB = midDB * 0.7f;
    m_eqTrebleDB = trebleDB * 0.7f;
    updateFilters();
}

inline void DSPProcessor::updateFilters() {
    if (m_sampleRate == 0) return;
    float fs = (float)m_sampleRate;

    m_eqBassL.makeLowShelf(fs, 150.0f, m_eqBassDB);
    m_eqBassR.makeLowShelf(fs, 150.0f, m_eqBassDB);
    m_eqMidL.makePeakingEQ(fs, 1000.0f, 1.0f, m_eqMidDB);
    m_eqMidR.makePeakingEQ(fs, 1000.0f, 1.0f, m_eqMidDB);
    m_eqTrebleL.makeHighShelf(fs, 6000.0f, m_eqTrebleDB);
    m_eqTrebleR.makeHighShelf(fs, 6000.0f, m_eqTrebleDB);

    // Bass boost shelf (+2 dB at 150 Hz)
    m_bassShelfL.makeLowShelf(fs, 150.0f, 2.0f);
    m_bassShelfR.makeLowShelf(fs, 150.0f, 2.0f);

    // Crossover filters
    m_crossoverLPL.makeLowPass(fs, APP_CROSSOVER_LP_FREQ);
    m_crossoverHPR.makeHighPass(fs, APP_CROSSOVER_HP_FREQ);

    // Cache EQ active state
    m_eqActive = (fabsf(m_eqBassDB) >= 0.1f) ||
                 (fabsf(m_eqMidDB) >= 0.1f) ||
                 (fabsf(m_eqTrebleDB) >= 0.1f);
}

inline void DSPProcessor::resetAllFilters() {
    // Reset all biquad filter states to prevent noise when sample rate changes
    m_eqBassL.reset();
    m_eqBassR.reset();
    m_eqMidL.reset();
    m_eqMidR.reset();
    m_eqTrebleL.reset();
    m_eqTrebleR.reset();
    m_bassShelfL.reset();
    m_bassShelfR.reset();
    m_bassCompL.reset();
    m_bassCompR.reset();
    m_crossoverLPL.reset();
    m_crossoverHPR.reset();
    m_peakMeter.zero();
    m_lpState = 0.0f;
}

// Update bass compensation filter based on current volume
// At low volumes, bass is perceived as quieter (equal loudness contour)
// This compensates by boosting bass as volume decreases
inline void DSPProcessor::updateBassCompensation() {
    if (m_sampleRate == 0) return;
    
    // Calculate bass boost: 0dB at full volume (127), +3dB at zero volume
    // Use exponential curve for more natural perception
    // volumePct: 0.0 (mute) to 1.0 (max)
    float volumePct = (float)m_volume * fast_recipsf2(127.0f);
    
    // Exponential curve: more boost at lower volumes
    // boost_dB = 3.0 * (1 - volumePct)^2
    // This gives ~0dB at 100%, ~0.75dB at 50%, ~3dB at 0%
    float factor = 1.0f - volumePct;
    m_bassCompensationDB = 5.0f * factor * factor;
    
    // Update bass compensation filters (low shelf at 100Hz)
    float fs = (float)m_sampleRate;
    m_bassCompL.makeLowShelf(fs, 100.0f, m_bassCompensationDB);
    m_bassCompR.makeLowShelf(fs, 100.0f, m_bassCompensationDB);
}

inline void DSPProcessor::updateLPAlpha() {
    const float fc = 300.0f;

    if (m_sampleRate == 0) {
        m_lpAlpha = 0.0f;
        return;
    }

    float dt = fast_recipsf2((float)m_sampleRate);
    float rc = fast_recipsf2(2.0f * DSP_PI_F * fc);
    float alpha = fast_div(dt, rc + dt);

    if (alpha < 0.001f) alpha = 0.001f;
    if (alpha > 0.999f) alpha = 0.999f;

    m_lpAlpha = alpha;
}

inline void IRAM_ATTR DSPProcessor::processStereo(float &L, float &R) {
    // Create mono mix for analysis (before any processing)
    float mono = (L + R) * 0.5f;

    // EQ always applies (regardless of bypass)
    if (m_eqActive) {
        L = m_eqBassL.process(L);
        R = m_eqBassR.process(R);
        L = m_eqMidL.process(L);
        R = m_eqMidR.process(R);
        L = m_eqTrebleL.process(L);
        R = m_eqTrebleR.process(R);
    }
    
    // Apply volume-based bass compensation (always active when volume < 100%)
    // This compensates for the ear's reduced bass sensitivity at lower volumes
    if (m_bassCompensationDB > 0.1f) {
        L = m_bassCompL.process(L);
        R = m_bassCompR.process(R);
    }
    
    // Audio analysis (using original audio before DSP). Decimated to ~11 kHz so
    // it stays cheap at 88.2/96 kHz; box-averaging also anti-aliases the input.
    if (m_analysisEnabled) {
        m_analysisAccum += mono;
        if (++m_analysisDecimCount >= m_analysisDecim) {
            float a = m_analysisAccum * fast_div(1.0f, (float)m_analysisDecim);
            m_analysisAccum = 0.0f;
            m_analysisDecimCount = 0;
            m_goertzel.processSample(a);  // For beat detection
            m_peakMeter.process(a);       // For level display (fast response)
        }
    }

    // Crossover split-ear mode only when NOT bypassed
    // Stereo preserved: LP applied to L channel, HP applied to R channel
    if (!m_bypassEnabled) {
        // Apply LP filter to LEFT channel (keeps stereo L content, low frequencies only)
        float lpOut = m_crossoverLPL.process(L);
        // Apply HP filter to RIGHT channel (keeps stereo R content, high frequencies only)
        float hpOut = m_crossoverHPR.process(R);

        // Apply bass boost shelf to LP channel
        if (m_bassBoostEnabled) {
            lpOut = m_bassShelfL.process(lpOut) * DSP_BASS_GAIN_BOOST;
        }

        // Gain compensation: each ear only gets part of the spectrum
        constexpr float CROSSOVER_GAIN = 1.41f;
        lpOut *= CROSSOVER_GAIN;
        hpOut *= CROSSOVER_GAIN;

        // Channel flip controls which ear gets LP vs HP
        if (!m_channelFlipEnabled) {
            L = lpOut;  // Left ear: low frequencies from L channel
            R = hpOut;  // Right ear: high frequencies from R channel
        } else {
            L = hpOut;  // Left ear: high frequencies from R channel
            R = lpOut;  // Right ear: low frequencies from L channel
        }
    } else {
        // Bypass mode: full-range stereo with optional bass boost
        if (m_bassBoostEnabled) {
            // Apply bass shelf to each channel independently (keeps stereo)
            L = m_bassShelfL.process(L) * DSP_BASS_GAIN_BOOST;
            R = m_bassShelfR.process(R) * DSP_BASS_GAIN_BOOST;
        }
    }

    // Apply volume scaling (AVRCP/encoder volume 0-127 → 0.0-1.0)
    L *= m_volumeGain;
    R *= m_volumeGain;

    // Apply soft clipper to prevent harsh digital clipping
    m_clipper.process(L, R);
}

inline void IRAM_ATTR DSPProcessor::processStereoQ31(int32_t &L, int32_t &R) {
    float left = (float)L * (1.0f / 2147483648.0f);
    float right = (float)R * (1.0f / 2147483648.0f);

    processStereo(left, right);

    if (left >= 0.999999999f) L = 2147483647;
    else if (left <= -1.0f) L = (int32_t)0x80000000;
    else L = (int32_t)(left * 2147483647.0f);

    if (right >= 0.999999999f) R = 2147483647;
    else if (right <= -1.0f) R = (int32_t)0x80000000;
    else R = (int32_t)(right * 2147483647.0f);
}

inline uint8_t DSPProcessor::getControlByte() const {
    uint8_t v = 0;
    if (m_bassBoostEnabled) v |= 0x01;
    if (m_channelFlipEnabled) v |= 0x02;
    if (m_bypassEnabled) v |= 0x04;
    return v;
}

inline void DSPProcessor::applyControlByte(uint8_t v) {
    m_bassBoostEnabled = (v & 0x01) != 0;
    m_channelFlipEnabled = (v & 0x02) != 0;
    m_bypassEnabled = (v & 0x04) != 0;
}
