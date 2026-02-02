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
#include "esp_heap_caps.h"
#include "esp_log.h"

class OverlayMixer {
public:
    static constexpr const char* TAG = "OverlayMixer";
    
    // Ring buffer size: ~100ms at 96kHz stereo 32-bit = 96000 * 0.1 * 2 * 4 = 76800 bytes
    // We use PSRAM so we can be generous
    static constexpr size_t RING_SIZE = 96 * 1024;  // 96KB - plenty for any sample rate
    
    // Ducking settings (Q15 fixed point for efficiency)
    static constexpr int16_t DUCK_GAIN_Q15 = 6554;   // ~0.2 = -14dB (duck BT by 80%)
    static constexpr int16_t UNITY_Q15 = 32767;      // 1.0
    static constexpr int16_t DUCK_RAMP_STEP = 328;   // ~10ms ramp at 96kHz (32767 / (96*10))
    
    OverlayMixer() = default;
    
    bool init() {
        if (m_initialized) return true;
        
        // Allocate ring buffer in PSRAM
        m_ringBuf = (int32_t*)heap_caps_malloc(RING_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!m_ringBuf) {
            ESP_LOGE(TAG, "Failed to allocate ring buffer in PSRAM");
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
    
    // Push stereo samples from SoundPlayer (called from sound playback task)
    // Samples are already resampled to match I2S rate
    void pushSamples(const int32_t* stereoSamples, size_t frames) {
        if (!m_initialized || frames == 0) return;
        
        xSemaphoreTake(m_mutex, portMAX_DELAY);
        
        size_t freeFrames = m_ringFrameCount - m_framesAvailable;
        if (frames > freeFrames) {
            frames = freeFrames;  // Clip to available space
        }
        
        // Copy frames to ring buffer (wrap around if needed)
        for (size_t i = 0; i < frames; i++) {
            size_t idx = ((m_writeIdx + i) % m_ringFrameCount) * 2;
            m_ringBuf[idx + 0] = stereoSamples[i * 2 + 0];  // Left
            m_ringBuf[idx + 1] = stereoSamples[i * 2 + 1];  // Right
        }
        
        m_writeIdx = (m_writeIdx + frames) % m_ringFrameCount;
        m_framesAvailable += frames;
        m_overlayActive = true;
        m_targetDuckQ15 = DUCK_GAIN_Q15;  // Start ducking BT audio
        
        xSemaphoreGive(m_mutex);
    }
    
    // Mix overlay samples into DSP output buffer (called from DSP processing)
    // This applies ducking to BT audio when overlay is playing
    void mixIntoOutput(int32_t* dspOut, size_t frames) {
        if (!m_initialized) return;
        
        xSemaphoreTake(m_mutex, portMAX_DELAY);
        
        // Smooth gain ramping for duck in/out
        bool hasOverlay = (m_framesAvailable > 0);
        
        for (size_t i = 0; i < frames; i++) {
            // Update duck gain (ramp toward target)
            if (m_duckGainQ15 < m_targetDuckQ15) {
                m_duckGainQ15 += DUCK_RAMP_STEP;
                if (m_duckGainQ15 > m_targetDuckQ15) m_duckGainQ15 = m_targetDuckQ15;
            } else if (m_duckGainQ15 > m_targetDuckQ15) {
                m_duckGainQ15 -= DUCK_RAMP_STEP;
                if (m_duckGainQ15 < m_targetDuckQ15) m_duckGainQ15 = m_targetDuckQ15;
            }
            
            // Apply duck gain to BT audio (Q15 fixed point multiply)
            int32_t btL = dspOut[i * 2 + 0];
            int32_t btR = dspOut[i * 2 + 1];
            
            btL = (int32_t)(((int64_t)btL * m_duckGainQ15) >> 15);
            btR = (int32_t)(((int64_t)btR * m_duckGainQ15) >> 15);
            
            // Add overlay sample if available
            if (m_framesAvailable > 0) {
                size_t idx = (m_readIdx % m_ringFrameCount) * 2;
                int32_t ovL = m_ringBuf[idx + 0];
                int32_t ovR = m_ringBuf[idx + 1];
                
                // Simple add with saturation
                int64_t sumL = (int64_t)btL + ovL;
                int64_t sumR = (int64_t)btR + ovR;
                
                // Soft clip at 32-bit max
                if (sumL > 2147483647LL) sumL = 2147483647LL;
                if (sumL < -2147483648LL) sumL = -2147483648LL;
                if (sumR > 2147483647LL) sumR = 2147483647LL;
                if (sumR < -2147483648LL) sumR = -2147483648LL;
                
                btL = (int32_t)sumL;
                btR = (int32_t)sumR;
                
                m_readIdx = (m_readIdx + 1) % m_ringFrameCount;
                m_framesAvailable--;
            }
            
            dspOut[i * 2 + 0] = btL;
            dspOut[i * 2 + 1] = btR;
        }
        
        // If overlay finished, start ramping gain back up
        if (m_framesAvailable == 0 && m_overlayActive) {
            m_overlayActive = false;
            m_targetDuckQ15 = UNITY_Q15;  // Ramp back to full volume
        }
        
        xSemaphoreGive(m_mutex);
    }
    
    // Check if overlay is currently active (samples in buffer or ducking active)
    bool isActive() const {
        return m_overlayActive || m_duckGainQ15 < UNITY_Q15;
    }
    
    // Get frames available for mixing
    size_t getFramesAvailable() const { return m_framesAvailable; }
    
    // Clear overlay buffer (e.g., when stopping playback)
    void clear() {
        if (!m_initialized) return;
        
        xSemaphoreTake(m_mutex, portMAX_DELAY);
        m_writeIdx = 0;
        m_readIdx = 0;
        m_framesAvailable = 0;
        m_overlayActive = false;
        m_targetDuckQ15 = UNITY_Q15;
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
    int16_t m_duckGainQ15 = UNITY_Q15;
    int16_t m_targetDuckQ15 = UNITY_Q15;
};
