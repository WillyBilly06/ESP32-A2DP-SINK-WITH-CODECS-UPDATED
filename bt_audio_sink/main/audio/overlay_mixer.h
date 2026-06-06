#pragma once

// -----------------------------------------------------------
// Overlay Mixer - ring buffer for sound effect overlay
// Sound effects push samples here, DSP pulls and mixes them
// with Bluetooth audio before I2S output
// -----------------------------------------------------------

#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

class OverlayMixer {
public:
    static constexpr const char* TAG = "OverlayMixer";
    
    // Ring buffer: 1024 stereo frames. This gives the streaming WAV task enough
    // cushion at LDAC 96 kHz so small SPIFFS/resampler scheduling jitter does
    // not underflow the overlay or disturb the render loop.
    // 1024 frames = 10.7 ms @ 96 kHz / 23.2 ms @ 44.1 kHz.
    static constexpr size_t RING_SIZE = 1024 * 2 * sizeof(int32_t);  // 8192 bytes
    
    // Ducking settings (Q15 fixed point for efficiency)
    static constexpr int32_t DUCK_GAIN_Q15 = 6554;   // ~0.2 = -14dB (duck BT by 80%)
    static constexpr int32_t UNITY_Q15 = 32767;      // 1.0
    static constexpr int32_t DUCK_RAMP_STEP = 328;   // ~10ms ramp at 96kHz (32767 / (96*10))
    // Overlay attenuation so ducked-BT (~0.2) + overlay stays within full scale
    // and never saturates into static. ~0.7 leaves ample headroom and still keeps
    // the effect clearly dominant over the ducked music.
    static constexpr int32_t OVERLAY_GAIN_Q15 = 22938;  // ~0.7
    
    OverlayMixer() = default;
    
    bool init() {
        if (m_initialized) return true;
        
        // Allocate ring buffer in internal RAM
        m_ringBuf = (int32_t*)heap_caps_malloc(RING_SIZE, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!m_ringBuf) {
            ESP_LOGE(TAG, "Failed to allocate %u-byte ring buffer in internal RAM", (unsigned)RING_SIZE);
            return false;
        }
        
        memset(m_ringBuf, 0, RING_SIZE);
        
        m_mutex = xSemaphoreCreateMutex();
        if (!m_mutex) {
            heap_caps_free(m_ringBuf);
            m_ringBuf = nullptr;
            return false;
        }
        
        m_ringFrameCount = RING_SIZE / (2 * sizeof(int32_t));  // Stereo frames
        m_writeIdx = 0;
        m_readIdx = 0;
        m_framesAvailable = 0;
        m_duckGainQ15 = UNITY_Q15;  // Start at full volume (no ducking)
        m_targetDuckQ15 = UNITY_Q15;
        m_overlayActive = false;
        
        m_initialized = true;
        ESP_LOGI(TAG, "Initialized: %u frames capacity", (unsigned)m_ringFrameCount);
        return true;
    }
    
    // Begin an overlay: start ducking the BT audio. Call once when a sound starts.
    void startOverlay() {
        if (!m_initialized) return;
        xSemaphoreTake(m_mutex, portMAX_DELAY);
        // Drop any stale frames from a previous (possibly aborted) effect so the
        // new sound starts clean and can't blip leftover audio into the mix.
        m_writeIdx = 0;
        m_readIdx = 0;
        m_framesAvailable = 0;
        memset(m_ringBuf, 0, RING_SIZE);
        m_overlayActive = true;
        m_stopPending = false;
        // Do not snap the current duck gain here; ramp from wherever it is.
        // But always aim at the ducked level for the whole effect.
        m_targetDuckQ15 = DUCK_GAIN_Q15;  // ramp BT audio down
        xSemaphoreGive(m_mutex);
    }

    // End an overlay. The caller (sound player) signals this right after pushing
    // the LAST chunk, but the ring usually still holds un-played frames. If we
    // un-ducked now, those leftover full-scale overlay frames would be summed onto
    // full-scale (restored) BT audio -> clipping/static for the tail. So we defer
    // the un-duck: keep the BT ducked until the ring fully drains, THEN ramp back.
    void stopOverlay() {
        if (!m_initialized) return;
        xSemaphoreTake(m_mutex, portMAX_DELAY);
        m_overlayActive = false;
        m_stopPending = true;
        if (m_framesAvailable == 0) {
            // Nothing queued -> safe to restore immediately.
            m_targetDuckQ15 = UNITY_Q15;
            m_stopPending = false;
        }
        // else: leave m_targetDuckQ15 at DUCK; mixIntoOutput un-ducks once drained.
        xSemaphoreGive(m_mutex);
    }

    // Free frames currently available in the ring (for paced, drop-free pushing).
    size_t getFreeFrames() {
        if (!m_initialized) return 0;
        xSemaphoreTake(m_mutex, portMAX_DELAY);
        size_t f = m_ringFrameCount - m_framesAvailable;
        xSemaphoreGive(m_mutex);
        return f;
    }

    // Push stereo samples from SoundPlayer (called from sound playback task).
    // Samples are already resampled to match the I2S rate. Returns the number of
    // frames actually accepted (may be < frames if the ring is full); the caller
    // paces itself by retrying the remainder so nothing is silently dropped.
    size_t IRAM_ATTR pushSamples(const int32_t* stereoSamples, size_t frames) {
        if (!m_initialized || frames == 0) return 0;
        
        xSemaphoreTake(m_mutex, portMAX_DELAY);
        
        size_t freeFrames = m_ringFrameCount - m_framesAvailable;
        if (frames > freeFrames) {
            frames = freeFrames;  // accept only what fits; caller retries the rest
        }
        
        // Copy frames to ring buffer (wrap around if needed)
        for (size_t i = 0; i < frames; i++) {
            size_t idx = ((m_writeIdx + i) % m_ringFrameCount) * 2;
            m_ringBuf[idx + 0] = stereoSamples[i * 2 + 0];  // Left
            m_ringBuf[idx + 1] = stereoSamples[i * 2 + 1];  // Right
        }
        
        m_writeIdx = (m_writeIdx + frames) % m_ringFrameCount;
        m_framesAvailable += frames;
        
        xSemaphoreGive(m_mutex);
        return frames;
    }
    
    // Mix overlay samples into DSP output buffer (called from DSP processing)
    // This applies ducking to BT audio when overlay is playing
    void IRAM_ATTR mixIntoOutput(int32_t* dspOut, size_t frames) {
        if (!m_initialized) return;
        
        if (xSemaphoreTake(m_mutex, 0) != pdTRUE) {
            return;
        }
        
        // Smooth gain ramping for duck in/out
        for (size_t i = 0; i < frames; i++) {
            // Update duck gain (ramp toward target).  This must use int32_t,
            // not int16_t: restoring from DUCK_GAIN_Q15 to UNITY_Q15 can step
            // past 32767.  With int16_t that overflows negative and the gain
            // becomes a permanent sawtooth, which sounds like destroyed/static
            // Bluetooth audio right after the overlay finishes.
            int32_t duck = m_duckGainQ15;
            const int32_t target = m_targetDuckQ15;
            if (duck < target) {
                duck += DUCK_RAMP_STEP;
                if (duck > target) duck = target;
            } else if (duck > target) {
                duck -= DUCK_RAMP_STEP;
                if (duck < target) duck = target;
            }
            if (duck < 0) duck = 0;
            if (duck > UNITY_Q15) duck = UNITY_Q15;
            m_duckGainQ15 = duck;
            
            // Apply duck gain to BT audio (Q15 fixed point multiply)
            int32_t btL = dspOut[i * 2 + 0];
            int32_t btR = dspOut[i * 2 + 1];
            
            btL = (int32_t)(((int64_t)btL * m_duckGainQ15) >> 15);
            btR = (int32_t)(((int64_t)btR * m_duckGainQ15) >> 15);
            
            // Add overlay sample if available. If the overlay ring is empty, the
            // overlay contribution is explicit silence. This prevents old/stale
            // WAV data or uninitialized values from ever being mixed into BT audio
            // during the post-effect duck restore ramp.
            int32_t ovL = 0;
            int32_t ovR = 0;
            if (m_framesAvailable > 0) {
                size_t idx = (m_readIdx % m_ringFrameCount) * 2;
                ovL = m_ringBuf[idx + 0];
                ovR = m_ringBuf[idx + 1];

                // Clear consumed slots. This is not needed for normal ring-buffer
                // correctness, but it makes stale-sample bugs impossible if state
                // is interrupted or cleared while the render task is running.
                m_ringBuf[idx + 0] = 0;
                m_ringBuf[idx + 1] = 0;

                m_readIdx = (m_readIdx + 1) % m_ringFrameCount;
                m_framesAvailable--;
            }

            ovL = (int32_t)(((int64_t)ovL * OVERLAY_GAIN_Q15) >> 15);
            ovR = (int32_t)(((int64_t)ovR * OVERLAY_GAIN_Q15) >> 15);

            int64_t sumL = (int64_t)btL + ovL;
            int64_t sumR = (int64_t)btR + ovR;

            // Saturate only as a final safety net.
            if (sumL > 2147483647LL) sumL = 2147483647LL;
            if (sumL < -2147483648LL) sumL = -2147483648LL;
            if (sumR > 2147483647LL) sumR = 2147483647LL;
            if (sumR < -2147483648LL) sumR = -2147483648LL;

            btL = (int32_t)sumL;
            btR = (int32_t)sumR;
            
            dspOut[i * 2 + 0] = btL;
            dspOut[i * 2 + 1] = btR;
        }
        
        // Deferred un-duck: once a stop has been requested AND the ring has fully
        // drained (the whole effect, including its tail, has been emitted), it is
        // finally safe to ramp the BT audio back up. Doing it here (instead of in
        // stopOverlay) guarantees no leftover full-scale overlay frame is ever
        // summed onto restored-gain BT, which is what produced the post-effect
        // clipping/static.
        if (m_stopPending && m_framesAvailable == 0) {
            m_targetDuckQ15 = UNITY_Q15;
            m_stopPending = false;
        }
        // While a sound is still playing we deliberately do NOT un-duck on a
        // momentary empty ring -- mid-sound underflow would otherwise pump the
        // duck up and down. A transient empty ring just adds no overlay sample.
        
        xSemaphoreGive(m_mutex);
    }
    
    // Check if overlay is currently active (samples in buffer or ducking active)
    bool isActive() {
        if (!m_initialized) return false;

        if (xSemaphoreTake(m_mutex, 0) != pdTRUE) {
            // If the producer owns the mutex, assume active so the render loop
            // keeps pumping instead of accidentally stalling the overlay.
            return true;
        }

        const bool active = m_overlayActive ||
                            m_stopPending ||
                            m_framesAvailable > 0 ||
                            m_duckGainQ15 < UNITY_Q15 ||
                            m_targetDuckQ15 < UNITY_Q15;

        xSemaphoreGive(m_mutex);
        return active;
    }
    
    // Get frames available for mixing
    size_t getFramesAvailable() {
        if (!m_initialized) return 0;
        xSemaphoreTake(m_mutex, portMAX_DELAY);
        size_t v = m_framesAvailable;
        xSemaphoreGive(m_mutex);
        return v;
    }
    
    // Clear overlay buffer (e.g., when stopping playback)
    void clear() {
        if (!m_initialized) return;
        
        xSemaphoreTake(m_mutex, portMAX_DELAY);
        m_writeIdx = 0;
        m_readIdx = 0;
        m_framesAvailable = 0;
        m_overlayActive = false;
        m_stopPending = false;
        m_duckGainQ15 = UNITY_Q15;
        m_targetDuckQ15 = UNITY_Q15;
        memset(m_ringBuf, 0, RING_SIZE);
        xSemaphoreGive(m_mutex);
    }
    
private:
    bool m_initialized = false;
    int32_t* m_ringBuf = nullptr;
    SemaphoreHandle_t m_mutex = nullptr;
    
    size_t m_ringFrameCount = 0;
    size_t m_writeIdx = 0;
    size_t m_readIdx = 0;
    size_t m_framesAvailable = 0;
    
    bool m_overlayActive = false;
    bool m_stopPending = false;     // stop signalled; un-duck deferred until ring drains
    int32_t m_duckGainQ15 = UNITY_Q15;
    int32_t m_targetDuckQ15 = UNITY_Q15;
};
