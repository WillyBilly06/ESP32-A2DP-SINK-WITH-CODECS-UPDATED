#pragma once

// -----------------------------------------------------------
// LED Controller
// Manages effects, demo mode switching, and audio integration
// Uses SPI DMA driver for reliable LED output
// -----------------------------------------------------------

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/idf_additions.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

#include "led_config.h"
#include "led_driver_spi.h"
#include "led_effects.h"
#include "../dsp/dsp_processor.h"

static const char* LED_TAG = "LedController";

class LedController {
public:
    static LedController& getInstance() {
        static LedController instance;
        return instance;
    }
    
    bool init(gpio_num_t gpioPin, uint8_t brightness = LED_DEFAULT_BRIGHTNESS, bool playStartup = false) {
        m_brightness = brightness;
        
        if (m_driver.init(gpioPin) != ESP_OK) {
            ESP_LOGE(LED_TAG, "Failed to initialize LED driver");
            return false;
        }
        
        // Create all effects
        createEffects();
        
        // Play startup animation if requested
        if (playStartup) {
            playStartupAnimation();
        }
        
        // Load saved effect from NVS
        loadSettings();
        
        // Set initial effect
        if (m_currentEffect < LED_EFFECT_COUNT && m_effects[m_currentEffect]) {
            m_effects[m_currentEffect]->init(&m_driver);
        }
        
        m_initialized = true;
        ESP_LOGI(LED_TAG, "LED Controller initialized, effect: %d", m_currentEffect);
        return true;
    }
    
    void update(float bass, float mid, float high, 
                float bassDb, float midDb, float highDb,
                bool beat, float beatIntensity, bool audioPlaying) {
        if (!m_initialized) return;
        
        // Track audio activity
        if (audioPlaying) {
            m_lastAudioTime = xTaskGetTickCount();
            m_inDemoMode = false;
        } else {
            // Check if we should switch to demo mode
            TickType_t elapsed = xTaskGetTickCount() - m_lastAudioTime;
            if (elapsed > pdMS_TO_TICKS(m_demoTimeoutMs) && !m_inDemoMode) {
                m_inDemoMode = true;
                ESP_LOGI(LED_TAG, "Switching to demo mode");
            }
        }
        
        // Update current effect
        LedEffect* effect = getCurrentEffect();
        if (!effect) return;
        
        if (m_inDemoMode) {
            effect->updateDemo();
        } else {
            AudioData audio;
            audio.bass = bass;
            audio.mid = mid;
            audio.high = high;
            audio.bassDB = bassDb;
            audio.midDB = midDb;
            audio.highDB = highDb;
            audio.beat = beat;
            audio.beatIntensity = beatIntensity;
            audio.audioActive = audioPlaying;
            effect->update(audio);
        }
        
        // Apply global brightness and show
        m_driver.setBrightness(m_brightness);
        m_driver.show();
    }
    
    void nextEffect() {
        m_currentEffect = (m_currentEffect + 1) % LED_EFFECT_USER_COUNT;
        
        LedEffect* effect = getCurrentEffect();
        if (effect) {
            effect->init(&m_driver);
            ESP_LOGI(LED_TAG, "Effect changed to: %s (%d)", effect->getName(), m_currentEffect);
        }
        
        saveSettings();
    }
    
    void previousEffect() {
        if (m_currentEffect == 0) {
            m_currentEffect = LED_EFFECT_USER_COUNT - 1;
        } else {
            m_currentEffect--;
            // Ensure we don't go past user-selectable effects
            if (m_currentEffect >= LED_EFFECT_USER_COUNT) {
                m_currentEffect = LED_EFFECT_USER_COUNT - 1;
            }
        }
        
        LedEffect* effect = getCurrentEffect();
        if (effect) {
            effect->init(&m_driver);
            ESP_LOGI(LED_TAG, "Effect changed to: %s (%d)", effect->getName(), m_currentEffect);
        }
        
        saveSettings();
    }
    
    void setEffect(int effectId, bool save = true) {
        if (effectId >= 0 && effectId < LED_EFFECT_COUNT) {
            m_currentEffect = effectId;
            m_ledSettings[9] = (uint8_t)effectId;  // Also update LED settings array
            LedEffect* effect = getCurrentEffect();
            if (effect) {
                effect->init(&m_driver);
            }
            if (save) {
                saveSettings();
            }
        }
    }
    
    int getCurrentEffectId() const { return m_currentEffect; }
    
    const char* getCurrentEffectName() const {
        LedEffect* effect = const_cast<LedController*>(this)->getCurrentEffect();
        return effect ? effect->getName() : "Unknown";
    }
    
    void setBrightness(uint8_t brightness, bool save = false) {
        m_brightness = brightness;
        m_ledSettings[0] = brightness;  // Keep ledSettings in sync
        // Also update Ambient effect if active
        if (m_currentEffect == LED_EFFECT_AMBIENT) {
            AmbientEffect* ambient = static_cast<AmbientEffect*>(m_effects[LED_EFFECT_AMBIENT]);
            if (ambient) ambient->setBrightness(brightness);
        }
        if (save) {
            saveBrightness();
        }
    }
    
    // Set full LED settings (for Ambient effect): [brightness, r1, g1, b1, r2, g2, b2, gradient, speed, effectId]
    void setLedSettings(const uint8_t* data, size_t len) {
        if (len < 10) return;
        
        // Store for persistence
        memcpy(m_ledSettings, data, 10);
        m_brightness = data[0];
        
        // Update Ambient effect
        AmbientEffect* ambient = static_cast<AmbientEffect*>(m_effects[LED_EFFECT_AMBIENT]);
        if (ambient) {
            ambient->setSettings(data, len);
        }
        
        // Save settings to NVS
        saveLedSettings();
    }
    
    const uint8_t* getLedSettings() const { return m_ledSettings; }
    
    // Request startup animation to be played from LED task context
    void requestStartupAnimation() {
        m_pendingStartupAnimation = true;
        ESP_LOGI(LED_TAG, "Startup animation requested");
    }
    
    // Check if startup animation is pending and clear the flag
    bool consumePendingStartupAnimation() {
        if (m_pendingStartupAnimation) {
            m_pendingStartupAnimation = false;
            return true;
        }
        return false;
    }
    
    // Check if startup animation is currently running
    bool isStartupAnimationRunning() const {
        return m_startupAnimationRunning;
    }
    
    // Mark startup animation as running/complete (called from LED task)
    void setStartupAnimationRunning(bool running) {
        m_startupAnimationRunning = running;
    }
    
    void saveLedSettings() {
        nvs_handle_t handle;
        if (nvs_open("led", NVS_READWRITE, &handle) == ESP_OK) {
            nvs_set_blob(handle, "settings", m_ledSettings, 10);
            nvs_commit(handle);
            nvs_close(handle);
            ESP_LOGI(LED_TAG, "Saved LED settings");
        }
    }
    
    void saveBrightness() {
        // Ensure m_ledSettings is in sync
        m_ledSettings[0] = m_brightness;
        
        nvs_handle_t handle;
        if (nvs_open("led", NVS_READWRITE, &handle) == ESP_OK) {
            nvs_set_u8(handle, "bright", m_brightness);
            nvs_set_blob(handle, "settings", m_ledSettings, 10);  // Also save blob for consistency
            nvs_commit(handle);
            nvs_close(handle);
            ESP_LOGI(LED_TAG, "Saved brightness: %d", m_brightness);
        }
    }
    
    uint8_t getBrightness() const { return m_brightness; }
    
    // Set current volume level (0-127 A2DP range)
    void setVolume(uint8_t volume) {
        m_currentVolume = volume;
        
        // Trigger volume overlay display
        m_volumeOverlayActive = true;
        m_volumeOverlayStart = xTaskGetTickCount();
        m_volumeDisplayTarget = volume;
        
        // If brightness is 0 (or very low), temporarily boost it so volume overlay is visible
        // This doesn't notify BLE, just affects local driver output
        if (m_brightness < 10 && !m_volumeBrightnessOverride) {
            m_volumeBrightnessOverride = true;
            m_brightnessBeforeVolume = m_brightness;
            m_driver.setBrightness(10);
        }
        
        // Update volume effect if active
        if (m_currentEffect == LED_EFFECT_VOLUME) {
            VolumeEffect* volEffect = static_cast<VolumeEffect*>(m_effects[LED_EFFECT_VOLUME]);
            if (volEffect) {
                volEffect->setVolume(volume);
            }
        }
    }
    
    uint8_t getVolume() const { return m_currentVolume; }
    
    // Set current EQ levels (-12 to +12 dB) - triggers EQ overlay display
    void setEq(int8_t bass, int8_t mid, int8_t treble, uint8_t changedType = 255) {
        m_eqBass = bass;
        m_eqMid = mid;
        m_eqTreble = treble;
        m_eqType = changedType;  // 0=bass, 1=mid, 2=treble, 255=all
        
        // Trigger EQ overlay display
        m_eqOverlayActive = true;
        m_eqOverlayStart = xTaskGetTickCount();
        
        // Temporarily boost brightness if very low
        if (m_brightness < 10 && !m_volumeBrightnessOverride) {
            m_volumeBrightnessOverride = true;
            m_brightnessBeforeVolume = m_brightness;
            m_driver.setBrightness(10);
        }
    }
    
    // Check if EQ overlay should be shown
    bool isEqOverlayActive() const {
        if (!m_eqOverlayActive) return false;
        TickType_t elapsed = xTaskGetTickCount() - m_eqOverlayStart;
        return elapsed < pdMS_TO_TICKS(VOLUME_OVERLAY_DURATION_MS);
    }
    
    // Get EQ overlay fade factor
    float getEqOverlayFade() const {
        if (!m_eqOverlayActive) return 0.0f;
        TickType_t elapsed = xTaskGetTickCount() - m_eqOverlayStart;
        uint32_t elapsedMs = pdTICKS_TO_MS(elapsed);
        
        if (elapsedMs >= VOLUME_OVERLAY_DURATION_MS) return 0.0f;
        if (elapsedMs < VOLUME_OVERLAY_HOLD_MS) return 1.0f;
        
        uint32_t fadeElapsed = elapsedMs - VOLUME_OVERLAY_HOLD_MS;
        uint32_t fadeDuration = VOLUME_OVERLAY_DURATION_MS - VOLUME_OVERLAY_HOLD_MS;
        return 1.0f - (float)fadeElapsed * fast_recipsf2((float)fadeDuration);
    }
    
    // Render EQ overlay - shows 3 vertical bars for bass/mid/treble
    void renderEqOverlay() {
        if (!m_initialized) return;
        
        float fade = getEqOverlayFade();
        if (fade <= 0.0f) {
            m_eqOverlayActive = false;
            if (m_volumeBrightnessOverride && !isVolumeOverlayActive()) {
                m_volumeBrightnessOverride = false;
                m_driver.setBrightness(m_brightnessBeforeVolume);
            }
            return;
        }
        
        m_driver.clear();
        
        // EQ range: -12 to +12 dB, map to 0-16 rows (center = 8)
        auto mapEqToRows = [](int8_t val) -> int {
            // Map -12..+12 to 0..16 rows (center at 8)
            return 8 + (val * 8 / 12);
        };
        
        int bassRows = mapEqToRows(m_eqBass);
        int midRows = mapEqToRows(m_eqMid);
        int trebleRows = mapEqToRows(m_eqTreble);
        
        uint8_t effectiveBrightness = (m_brightness < 10) ? 10 : m_brightness;
        
        // Colors for each EQ band
        RGB_SPI bassColor = {(uint8_t)(255 * fade), 0, 0};                           // Red
        RGB_SPI midColor = {0, 0, (uint8_t)(255 * fade)};                            // Blue  
        RGB_SPI trebleColor = {(uint8_t)(255 * fade), (uint8_t)(255 * fade), 0};     // Yellow
        
        // Draw 3 bars side by side (each 4 pixels wide, 2 pixel gaps)
        // Bass: cols 0-3, Mid: cols 6-9, Treble: cols 12-15
        auto drawBar = [&](int startCol, int rows, RGB_SPI color, bool highlight) {
            int centerRow = 8;  // Center line
            int barWidth = 4;
            
            // Draw from center up or down based on value
            if (rows >= centerRow) {
                // Positive: draw from center upward
                for (int row = centerRow; row < rows && row < LED_MATRIX_HEIGHT; row++) {
                    int displayRow = LED_MATRIX_HEIGHT - 1 - row;
                    for (int col = startCol; col < startCol + barWidth && col < LED_MATRIX_WIDTH; col++) {
                        RGB_SPI c = color;
                        if (highlight) {
                            c.r = (uint8_t)(c.r * 1.0f);
                            c.g = (uint8_t)(c.g * 1.0f);
                            c.b = (uint8_t)(c.b * 1.0f);
                        }
                        m_driver.setPixelXY(col, displayRow, c);
                    }
                }
            } else {
                // Negative: draw from center downward
                for (int row = rows; row < centerRow; row++) {
                    int displayRow = LED_MATRIX_HEIGHT - 1 - row;
                    for (int col = startCol; col < startCol + barWidth && col < LED_MATRIX_WIDTH; col++) {
                        RGB_SPI c = {(uint8_t)(color.r * 0.5f), (uint8_t)(color.g * 0.5f), (uint8_t)(color.b * 0.5f)};  // Dimmer for negative
                        m_driver.setPixelXY(col, displayRow, c);
                    }
                }
            }
            
            // Draw center line marker (dim white)
            int displayCenterRow = LED_MATRIX_HEIGHT - 1 - centerRow;
            for (int col = startCol; col < startCol + barWidth && col < LED_MATRIX_WIDTH; col++) {
                RGB_SPI marker = {(uint8_t)(40 * fade), (uint8_t)(40 * fade), (uint8_t)(40 * fade)};
                m_driver.setPixelXY(col, displayCenterRow, marker);
            }
        };
        
        // Draw the 3 bars
        drawBar(1, bassRows, bassColor, m_eqType == 0);      // Bass on left
        drawBar(6, midRows, midColor, m_eqType == 1);        // Mid in center
        drawBar(11, trebleRows, trebleColor, m_eqType == 2); // Treble on right
        
        m_driver.setBrightness(effectiveBrightness);
        m_driver.show();
        m_driver.setBrightness(m_brightness);
    }
    
    // -----------------------------------------------------------
    // Pairing Mode Animation - slow blue pulsing
    // -----------------------------------------------------------
    void setPairingMode(bool active) {
        m_pairingModeActive = active;
        m_pairingModeStart = xTaskGetTickCount();
        if (active) {
            ESP_LOGI(LED_TAG, "Pairing mode LED animation started");
        } else {
            ESP_LOGI(LED_TAG, "Pairing mode LED animation stopped");
        }
    }
    
    bool isPairingModeActive() const { return m_pairingModeActive; }
    
    // Called when pairing succeeds - show 2 fast pulses then exit pairing mode
    void showPairingSuccess() {
        m_pairingSuccessActive = true;
        m_pairingSuccessStart = xTaskGetTickCount();
        m_pairingModeActive = false;  // Exit pairing mode after success animation
        ESP_LOGI(LED_TAG, "Pairing success animation started");
    }
    
    bool isPairingSuccessActive() const {
        if (!m_pairingSuccessActive) return false;
        TickType_t elapsed = xTaskGetTickCount() - m_pairingSuccessStart;
        return elapsed < pdMS_TO_TICKS(PAIRING_SUCCESS_DURATION_MS);
    }
    
    // Render pairing mode animation - slow blue pulsing
    void renderPairingMode() {
        if (!m_initialized) return;
        
        // Calculate pulse phase (0-1, repeating every PAIRING_PULSE_PERIOD_MS)
        TickType_t elapsed = xTaskGetTickCount() - m_pairingModeStart;
        uint32_t elapsedMs = pdTICKS_TO_MS(elapsed);
        float phase = (float)(elapsedMs % PAIRING_PULSE_PERIOD_MS) * fast_recipsf2((float)PAIRING_PULSE_PERIOD_MS);
        
        // Sine wave for smooth pulsing (0.2 to 1.0 range for visibility)
        float sineVal = sinf(phase * 2.0f * M_PI);
        float brightness = 0.2f + 0.8f * (0.5f + 0.5f * sineVal);
        
        // Blue color with pulsing brightness
        uint8_t b = (uint8_t)(255 * brightness);
        uint8_t g = (uint8_t)(30 * brightness);  // Slight teal tint
        RGB_SPI blueColor = {0, g, b};
        
        // Fill entire matrix with pulsing blue
        m_driver.clear();
        int totalLeds = LED_MATRIX_WIDTH * LED_MATRIX_HEIGHT;
        for (int i = 0; i < totalLeds; i++) {
            m_driver.setPixel(i, blueColor);
        }
        
        // Use a reasonable brightness for pairing mode
        uint8_t effectiveBrightness = (m_brightness > 0) ? m_brightness : 40;
        m_driver.setBrightness(effectiveBrightness);
        m_driver.show();
        m_driver.setBrightness(m_brightness);
    }
    
    // Render pairing success animation - 2 fast blue pulses
    void renderPairingSuccess() {
        if (!m_initialized) return;
        
        TickType_t elapsed = xTaskGetTickCount() - m_pairingSuccessStart;
        uint32_t elapsedMs = pdTICKS_TO_MS(elapsed);
        
        if (elapsedMs >= PAIRING_SUCCESS_DURATION_MS) {
            m_pairingSuccessActive = false;
            return;
        }
        
        // 2 fast pulses: on-off-on-off pattern
        // Each pulse: 150ms on, 100ms off = 250ms per pulse, 500ms total
        float brightness = 0.0f;
        if (elapsedMs < 150) {
            brightness = 1.0f;  // First pulse ON
        } else if (elapsedMs < 250) {
            brightness = 0.0f;  // First pulse OFF
        } else if (elapsedMs < 400) {
            brightness = 1.0f;  // Second pulse ON
        } else {
            brightness = 0.0f;  // Second pulse OFF
        }
        
        // Bright blue/cyan for success
        uint8_t b = (uint8_t)(255 * brightness);
        uint8_t g = (uint8_t)(200 * brightness);
        RGB_SPI successColor = {0, g, b};
        
        m_driver.clear();
        int totalLeds = LED_MATRIX_WIDTH * LED_MATRIX_HEIGHT;
        for (int i = 0; i < totalLeds; i++) {
            m_driver.setPixel(i, successColor);
        }
        
        uint8_t effectiveBrightness = (m_brightness > 0) ? m_brightness : 60;
        m_driver.setBrightness(effectiveBrightness);
        m_driver.show();
        m_driver.setBrightness(m_brightness);
    }
    
    // Check if volume overlay should be shown
    bool isVolumeOverlayActive() const {
        if (!m_volumeOverlayActive) return false;
        TickType_t elapsed = xTaskGetTickCount() - m_volumeOverlayStart;
        return elapsed < pdMS_TO_TICKS(VOLUME_OVERLAY_DURATION_MS);
    }
    
    // Get volume overlay fade factor (1.0 = full, 0.0 = faded out)
    float getVolumeOverlayFade() const {
        if (!m_volumeOverlayActive) return 0.0f;
        TickType_t elapsed = xTaskGetTickCount() - m_volumeOverlayStart;
        uint32_t elapsedMs = pdTICKS_TO_MS(elapsed);
        
        if (elapsedMs >= VOLUME_OVERLAY_DURATION_MS) {
            return 0.0f;
        }
        
        // Hold full brightness for first part, then fade
        if (elapsedMs < VOLUME_OVERLAY_HOLD_MS) {
            return 1.0f;
        }
        
        // Fade out over remaining time
        uint32_t fadeElapsed = elapsedMs - VOLUME_OVERLAY_HOLD_MS;
        uint32_t fadeDuration = VOLUME_OVERLAY_DURATION_MS - VOLUME_OVERLAY_HOLD_MS;
        return 1.0f - (float)fadeElapsed * fast_recipsf2((float)fadeDuration);
    }
    
    // Render volume overlay on the LED matrix
    void renderVolumeOverlay() {
        if (!m_initialized) return;
        
        float fade = getVolumeOverlayFade();
        if (fade <= 0.0f) {
            m_volumeOverlayActive = false;
            // Restore original brightness if we temporarily boosted it
            if (m_volumeBrightnessOverride) {
                m_volumeBrightnessOverride = false;
                m_driver.setBrightness(m_brightnessBeforeVolume);
            }
            return;
        }
        
        // Smooth the displayed volume
        float targetVol = m_volumeDisplayTarget * fast_recipsf2(127.0f);
        m_volumeDisplaySmooth += (targetVol - m_volumeDisplaySmooth) * 0.3f;
        
        m_driver.clear();
        
        // Volume percentage (0-100)
        int volumePercent = (int)(m_volumeDisplaySmooth * 100.0f + 0.5f);
        
        // Calculate how many rows to fill (16 rows total)
        int filledRows = (int)(m_volumeDisplaySmooth * LED_MATRIX_HEIGHT + 0.5f);
        
        // Pulsing effect
        static uint32_t volFrame = 0;
        volFrame++;
        float pulse = 0.85f + 0.15f * sinf(volFrame * 0.15f);
        
        // Use brightness setting but with minimum floor of 10 (so volume is always visible)
        uint8_t effectiveBrightness = (m_brightness < 10) ? 10 : m_brightness;
        // Don't scale colors by brightness - let the driver handle it via setBrightness()
        // This prevents double-dimming when brightness is very low
        float brightnessScale = 1.0f;
        
        // Draw volume bar from bottom to top
        for (int row = 0; row < LED_MATRIX_HEIGHT; row++) {
            int displayRow = LED_MATRIX_HEIGHT - 1 - row;  // Bottom to top
            
            if (row < filledRows) {
                // Color gradient: white -> red (bottom to top)
                float rowPct = (float)row * fast_recipsf2((float)(LED_MATRIX_HEIGHT - 1));
                uint8_t r, g, b;
                
                // White (255,255,255) at bottom, Red (255,0,0) at top
                r = 255;
                g = (uint8_t)(255 * (1.0f - rowPct));
                b = (uint8_t)(255 * (1.0f - rowPct));
                
                // Apply pulse, fade and brightness (with minimum floor)
                float intensity = pulse * fade * brightnessScale;
                r = (uint8_t)(r * intensity);
                g = (uint8_t)(g * intensity);
                b = (uint8_t)(b * intensity);
                
                // Fill the row with a slight gradient from center outward
                for (int col = 0; col < LED_MATRIX_WIDTH; col++) {
                    // Center columns brighter
                    float colDist = fabsf(col - (LED_MATRIX_WIDTH - 1) * 0.5f) * fast_recipsf2((LED_MATRIX_WIDTH - 1) * 0.5f);
                    float colFade = 1.0f - colDist * 0.3f;  // 70% brightness at edges
                    
                    RGB_SPI color = {
                        (uint8_t)(r * colFade),
                        (uint8_t)(g * colFade),
                        (uint8_t)(b * colFade)
                    };
                    m_driver.setPixelXY(col, displayRow, color);
                }
            }
        }
        
        // Temporarily set driver brightness to effective value (in case global brightness is 0)
        // This doesn't notify BLE, just affects the driver output
        m_driver.setBrightness(effectiveBrightness);
        m_driver.show();
        // Restore original brightness for next regular effect render
        m_driver.setBrightness(m_brightness);
    }
    
    // Draw volume number on the display
    void drawVolumeNumber(int percent, float fade) {
        // Simple 3x5 digit font patterns (each digit is 3 wide, 5 tall)
        static const uint8_t DIGITS[10][5] = {
            {0b111, 0b101, 0b101, 0b101, 0b111},  // 0
            {0b010, 0b110, 0b010, 0b010, 0b111},  // 1
            {0b111, 0b001, 0b111, 0b100, 0b111},  // 2
            {0b111, 0b001, 0b111, 0b001, 0b111},  // 3
            {0b101, 0b101, 0b111, 0b001, 0b001},  // 4
            {0b111, 0b100, 0b111, 0b001, 0b111},  // 5
            {0b111, 0b100, 0b111, 0b101, 0b111},  // 6
            {0b111, 0b001, 0b001, 0b001, 0b001},  // 7
            {0b111, 0b101, 0b111, 0b101, 0b111},  // 8
            {0b111, 0b101, 0b111, 0b001, 0b111},  // 9
        };
        
        // Clamp percent
        if (percent < 0) percent = 0;
        if (percent > 100) percent = 100;
        
        // Calculate digits
        int d1 = percent / 100;       // Hundreds (0 or 1)
        int d2 = (percent / 10) % 10; // Tens
        int d3 = percent % 10;        // Ones
        
        // Determine how many digits to show
        int numDigits = (percent >= 100) ? 3 : (percent >= 10) ? 2 : 1;
        int digitWidth = 3;
        int spacing = 1;
        int totalWidth = numDigits * digitWidth + (numDigits - 1) * spacing;
        
        // Center horizontally and vertically
        int startX = (LED_MATRIX_WIDTH - totalWidth) / 2;
        int startY = (LED_MATRIX_HEIGHT - 5) / 2;
        
        // White color with fade
        uint8_t brightness = (uint8_t)(255 * fade);
        
        // Draw digits
        int x = startX;
        int digits[3] = {d1, d2, d3};
        int startDigit = 3 - numDigits;
        
        for (int d = startDigit; d < 3; d++) {
            int digit = digits[d];
            for (int row = 0; row < 5; row++) {
                for (int col = 0; col < 3; col++) {
                    if (DIGITS[digit][row] & (0b100 >> col)) {
                        int px = x + col;
                        int py = startY + row;
                        if (px >= 0 && px < LED_MATRIX_WIDTH && py >= 0 && py < LED_MATRIX_HEIGHT) {
                            RGB_SPI color = {brightness, brightness, brightness};
                            m_driver.setPixelXY(px, py, color);
                        }
                    }
                }
            }
            x += digitWidth + spacing;
        }
    }
    
    void setDemoTimeout(uint32_t timeoutMs) {
        m_demoTimeoutMs = timeoutMs;
    }
    
    bool isInDemoMode() const { return m_inDemoMode; }
    
    void forceDemoMode(bool demo) {
        m_inDemoMode = demo;
        if (!demo) {
            m_lastAudioTime = xTaskGetTickCount();
        }
    }
    
    // OTA progress display mode
    void setOtaMode(bool enabled) {
        m_otaMode = enabled;
        m_otaProgress = 0;
        if (enabled) {
            ESP_LOGI(LED_TAG, "LED OTA mode enabled");
        }
    }
    
    void setOtaProgress(uint8_t percent) {
        if (percent > 100) percent = 100;
        m_otaProgress = percent;
    }
    
    bool isOtaMode() const { return m_otaMode; }
    
    // Render OTA progress bar on the LED matrix
    void renderOtaProgress() {
        if (!m_initialized) return;
        
        m_driver.clear();
        
        // Calculate how many LEDs to light up (256 total, map 0-100% to 0-256)
        int totalLeds = LED_MATRIX_WIDTH * LED_MATRIX_HEIGHT;
        int litLeds = (m_otaProgress * totalLeds) / 100;
        
        // Pulsing brightness effect using frame counter
        static uint32_t otaFrame = 0;
        uint8_t pulse = sin8((otaFrame * 4) & 0xFF);
        uint8_t baseBrightness = 180 + (pulse >> 2);  // 180-243 range
        otaFrame++;
        
        // Fill from bottom to top, left to right
        for (int y = 0; y < LED_MATRIX_HEIGHT; y++) {
            int yFlip = LED_MATRIX_HEIGHT - 1 - y;  // Start from bottom
            for (int x = 0; x < LED_MATRIX_WIDTH; x++) {
                int ledIndex = y * LED_MATRIX_WIDTH + x;
                
                if (ledIndex < litLeds) {
                    // Color gradient from green (bottom) to cyan (middle) to blue (top)
                    uint8_t hue;
                    if (y < LED_MATRIX_HEIGHT / 3) {
                        hue = 96;  // Green
                    } else if (y < 2 * LED_MATRIX_HEIGHT / 3) {
                        hue = 128;  // Cyan
                    } else {
                        hue = 160;  // Blue
                    }
                    
                    m_driver.setPixelXY(x, yFlip, RGB::fromHSV(hue, 255, baseBrightness));
                } else if (ledIndex == litLeds) {
                    // Leading edge - bright white
                    m_driver.setPixelXY(x, yFlip, RGB(255, 255, 255));
                } else {
                    // Background - very dim
                    m_driver.setPixelXY(x, yFlip, RGB(2, 2, 2));
                }
            }
        }
        
        m_driver.setBrightness(m_brightness);
        m_driver.show();
    }
    
    LedDriver& getDriver() { return m_driver; }
    
private:
    LedController() = default;
    ~LedController() {
        for (int i = 0; i < LED_EFFECT_COUNT; i++) {
            delete m_effects[i];
        }
    }
    
    LedController(const LedController&) = delete;
    LedController& operator=(const LedController&) = delete;
    
    void createEffects() {
        m_effects[LED_EFFECT_SPECTRUM_BARS] = new SpectrumBarsEffect();
        m_effects[LED_EFFECT_BEAT_PULSE] = new BeatPulseEffect();
        m_effects[LED_EFFECT_RIPPLE] = new RippleEffect();
        m_effects[LED_EFFECT_FIRE] = new FireEffect();
        m_effects[LED_EFFECT_PLASMA] = new PlasmaEffect();
        m_effects[LED_EFFECT_RAIN] = new MatrixRainEffect();
        m_effects[LED_EFFECT_VU_METER] = new VUMeterEffect();
        m_effects[LED_EFFECT_STARFIELD] = new StarfieldEffect();
        m_effects[LED_EFFECT_WAVE] = new WaveEffect();
        m_effects[LED_EFFECT_FIREWORKS] = new FireworksEffect();
        m_effects[LED_EFFECT_RAINBOW_WAVE] = new RainbowWaveEffect();
        m_effects[LED_EFFECT_PARTICLE_BURST] = new ParticleBurstEffect();
        m_effects[LED_EFFECT_KALEIDOSCOPE] = new KaleidoscopeEffect();
        m_effects[LED_EFFECT_FREQUENCY_SPIRAL] = new FrequencySpiralEffect();
        m_effects[LED_EFFECT_BASS_REACTOR] = new BassReactorEffect();
        // New effects
        m_effects[LED_EFFECT_METEOR_SHOWER] = new MeteorShowerEffect();
        m_effects[LED_EFFECT_BREATHING] = new BreathingEffect();
        m_effects[LED_EFFECT_DNA_HELIX] = new DNAHelixEffect();
        m_effects[LED_EFFECT_AUDIO_SCOPE] = new AudioScopeEffect();
        m_effects[LED_EFFECT_BOUNCING_BALLS] = new BouncingBallsEffect();
        m_effects[LED_EFFECT_LAVA_LAMP] = new LavaLampEffect();
        m_effects[LED_EFFECT_AMBIENT] = new AmbientEffect();
        m_effects[LED_EFFECT_VOLUME] = new VolumeEffect();
    }
    
    LedEffect* getCurrentEffect() {
        if (m_currentEffect >= 0 && m_currentEffect < LED_EFFECT_COUNT) {
            return m_effects[m_currentEffect];
        }
        return nullptr;
    }
    
    void loadSettings() {
        nvs_handle_t handle;
        if (nvs_open("led", NVS_READONLY, &handle) == ESP_OK) {
            // First try to load the blob (has all settings)
            size_t len = 10;
            bool blobLoaded = false;
            if (nvs_get_blob(handle, "settings", m_ledSettings, &len) == ESP_OK && len == 10) {
                // Blob has: [brightness, r1, g1, b1, r2, g2, b2, gradient, speed, effectId]
                m_brightness = m_ledSettings[0];
                m_currentEffect = m_ledSettings[9];
                if (m_currentEffect >= LED_EFFECT_COUNT) {
                    m_currentEffect = 0;
                }
                blobLoaded = true;
                ESP_LOGI(LED_TAG, "Loaded LED settings blob: effect=%d, brightness=%d", m_currentEffect, m_brightness);
                
                // Apply to Ambient effect
                AmbientEffect* ambient = static_cast<AmbientEffect*>(m_effects[LED_EFFECT_AMBIENT]);
                if (ambient) {
                    ambient->setSettings(m_ledSettings, 10);
                }
            }
            
            // Fallback: load individual keys if blob not available (legacy support)
            if (!blobLoaded) {
                int32_t effect = 0;
                if (nvs_get_i32(handle, "effect", &effect) == ESP_OK) {
                    if (effect >= 0 && effect < LED_EFFECT_COUNT) {
                        m_currentEffect = effect;
                    }
                }
                
                uint8_t brightness = LED_DEFAULT_BRIGHTNESS;
                if (nvs_get_u8(handle, "bright", &brightness) == ESP_OK) {
                    m_brightness = brightness;
                }
                
                // Sync m_ledSettings with loaded values
                m_ledSettings[0] = m_brightness;
                m_ledSettings[9] = (uint8_t)m_currentEffect;
                
                ESP_LOGI(LED_TAG, "Loaded settings from keys: effect=%d, brightness=%d", m_currentEffect, m_brightness);
            }
            
            nvs_close(handle);
        }
    }
    
    void saveSettings() {
        // Ensure m_ledSettings is in sync
        m_ledSettings[0] = m_brightness;
        m_ledSettings[9] = (uint8_t)m_currentEffect;
        
        nvs_handle_t handle;
        if (nvs_open("led", NVS_READWRITE, &handle) == ESP_OK) {
            nvs_set_i32(handle, "effect", m_currentEffect);
            nvs_set_u8(handle, "bright", m_brightness);
            nvs_set_blob(handle, "settings", m_ledSettings, 10);
            nvs_commit(handle);
            nvs_close(handle);
            ESP_LOGI(LED_TAG, "Saved settings: effect=%d, brightness=%d", m_currentEffect, m_brightness);
        }
    }
    
public:
    // Startup animation - colorful spiral wipe (public so main can trigger it)
    void playStartupAnimation() {
        ESP_LOGI(LED_TAG, "Playing startup animation...");
        
        // Fixed brightness for startup animation (15% = 38/255)
        const uint8_t startupBrightness = 38;
        
        const int totalFrames = 60;  // ~2 seconds at 30fps
        const int spiralSteps = LED_MATRIX_WIDTH * LED_MATRIX_HEIGHT;
        
        // Phase 1: Spiral fill inward with rainbow colors
        for (int frame = 0; frame < totalFrames / 2; frame++) {
            m_driver.clear();
            
            int ledsToShow = (frame * spiralSteps) / (totalFrames / 2);
            
            // Spiral coordinates generator
            int x = 0, y = 0;
            int dx = 1, dy = 0;
            int minX = 0, maxX = LED_MATRIX_WIDTH - 1;
            int minY = 0, maxY = LED_MATRIX_HEIGHT - 1;
            
            for (int i = 0; i < ledsToShow && i < spiralSteps; i++) {
                // Rainbow color based on position
                uint8_t hue = (i * 256 / spiralSteps + frame * 4) & 0xFF;
                uint8_t brightness = startupBrightness;
                RGB color = RGB::fromHSV(hue, 255, brightness);
                
                m_driver.setPixelXY(x, y, color);
                
                // Move to next position in spiral
                int nx = x + dx;
                int ny = y + dy;
                
                if (dx == 1 && nx > maxX) { dx = 0; dy = 1; minY++; }
                else if (dy == 1 && ny > maxY) { dx = -1; dy = 0; maxX--; }
                else if (dx == -1 && nx < minX) { dx = 0; dy = -1; maxY--; }
                else if (dy == -1 && ny < minY) { dx = 1; dy = 0; minX++; }
                
                x += dx;
                y += dy;
            }
            
            m_driver.setBrightness(startupBrightness);
            m_driver.show();
            vTaskDelay(pdMS_TO_TICKS(33));  // ~30fps
        }
        
        // Phase 2: Color pulse from center
        for (int frame = 0; frame < totalFrames / 4; frame++) {
            float cx = (LED_MATRIX_WIDTH - 1) * 0.5f;
            float cy = (LED_MATRIX_HEIGHT - 1) * 0.5f;
            float maxDist = sqrtf(cx * cx + cy * cy);
            float pulse = (float)frame / (totalFrames / 4);
            
            for (int y = 0; y < LED_MATRIX_HEIGHT; y++) {
                for (int x = 0; x < LED_MATRIX_WIDTH; x++) {
                    float dx = x - cx;
                    float dy = y - cy;
                    float dist = sqrtf(dx * dx + dy * dy) * fast_recipsf2(maxDist);
                    
                    // Wave effect from center
                    float wave = sinf((dist - pulse * 2) * 6.28f);
                    if (wave < 0) wave = 0;
                    
                    uint8_t hue = (uint8_t)((dist * 128 + frame * 8) * 255) & 0xFF;
                    uint8_t v = (uint8_t)(wave * startupBrightness);
                    
                    RGB color = RGB::fromHSV(hue, 255, v);
                    m_driver.setPixelXY(x, y, color);
                }
            }
            
            m_driver.setBrightness(255);  // Brightness already applied per-pixel
            m_driver.show();
            vTaskDelay(pdMS_TO_TICKS(33));
        }
        
        // Phase 3: Quick flash and fade
        for (int frame = 0; frame < totalFrames / 4; frame++) {
            float fade = 1.0f - (float)frame / (totalFrames / 4);
            uint8_t v = (uint8_t)(fade * startupBrightness);
            
            // Cyan/white flash
            RGB color = RGB(v / 2, v, v);
            for (int y = 0; y < LED_MATRIX_HEIGHT; y++) {
                for (int x = 0; x < LED_MATRIX_WIDTH; x++) {
                    m_driver.setPixelXY(x, y, color);
                }
            }
            
            m_driver.setBrightness(255);
            m_driver.show();
            vTaskDelay(pdMS_TO_TICKS(33));
        }
        
        m_driver.clear();
        m_driver.show();
        ESP_LOGI(LED_TAG, "Startup animation complete");
    }
    
    LedDriverSPI m_driver;  // SPI DMA driver
    LedEffect* m_effects[LED_EFFECT_COUNT] = {nullptr};
    
    int m_currentEffect = LED_EFFECT_SPECTRUM_BARS;
    uint8_t m_brightness = LED_DEFAULT_BRIGHTNESS;
    uint8_t m_currentVolume = 64;  // Current volume level (0-127)
    uint8_t m_ledSettings[10] = {128, 255, 0, 128, 0, 128, 255, 0, 50, 0};  // Default LED settings
    bool m_initialized = false;
    
    bool m_inDemoMode = true;
    TickType_t m_lastAudioTime = 0;
    uint32_t m_demoTimeoutMs = LED_DEMO_TIMEOUT_MS;
    
    // OTA progress display
    bool m_otaMode = false;
    uint8_t m_otaProgress = 0;
    
    // Volume overlay display
    static constexpr uint32_t VOLUME_OVERLAY_DURATION_MS = 2500;  // Total display time
    static constexpr uint32_t VOLUME_OVERLAY_HOLD_MS = 1500;      // Hold at full brightness
    bool m_volumeOverlayActive = false;
    TickType_t m_volumeOverlayStart = 0;
    uint8_t m_volumeDisplayTarget = 64;
    float m_volumeDisplaySmooth = 0.5f;
    
    // Temporary brightness override for volume overlay when user brightness is 0
    // This allows volume overlay to be visible even when effects are disabled
    uint8_t m_overlayBrightness = 0;
    
    // Flag to request startup animation from LED task context
    volatile bool m_pendingStartupAnimation = false;
    volatile bool m_startupAnimationRunning = false;
    bool m_volumeBrightnessOverride = false;
    uint8_t m_brightnessBeforeVolume = 0;
    
    // EQ overlay display (same timing as volume)
    bool m_eqOverlayActive = false;
    TickType_t m_eqOverlayStart = 0;
    int8_t m_eqBass = 0;
    int8_t m_eqMid = 0;
    int8_t m_eqTreble = 0;
    uint8_t m_eqType = 0;  // 0=bass, 1=mid, 2=treble
    
    // Pairing mode animation
    static constexpr uint32_t PAIRING_PULSE_PERIOD_MS = 2000;     // 2 seconds per pulse cycle
    static constexpr uint32_t PAIRING_SUCCESS_DURATION_MS = 500;  // 500ms for 2 fast pulses
    bool m_pairingModeActive = false;
    TickType_t m_pairingModeStart = 0;
    bool m_pairingSuccessActive = false;
    TickType_t m_pairingSuccessStart = 0;
};

// -----------------------------------------------------------
// LED Task for FreeRTOS
// -----------------------------------------------------------

static TaskHandle_t ledTaskHandle = nullptr;
static volatile bool ledTaskRunning = false;
static volatile bool ledBeatDetected = false;
static DSPProcessor* g_ledDsp = nullptr;

inline void setLedBeat(bool beat) { ledBeatDetected = beat; }

// Helper struct to hold audio readings - avoids optimizer issues
struct LedAudioReadings {
    float bass;
    float mid;
    float high;
    float bassDb;
    float midDb;
    float highDb;
    bool audioPlaying;
};

// -----------------------------------------------------------
// Automatic Gain Control (AGC) for LED effects
// Ultra-aggressive normalization for 1% volume support
// Uses dB-domain processing for proper low-level handling
// NEW: Fixed floor normalization - always boost to minimum target level
// -----------------------------------------------------------
struct AudioAGC {
    // Running peak in dB domain (much better for quiet audio)
    float peakDbBass = -60.0f;
    float peakDbMid = -60.0f;
    float peakDbHigh = -60.0f;
    
    // AGC parameters - MORE AGGRESSIVE for 1% volume support
    float targetDb = -3.0f;      // Target output level (-3dB = 0.7 linear = 70%)
    float minFloorLin = 0.7f;    // FIXED MINIMUM: Always boost audio to at least 70% for LEDs
    float attackRate = 0.4f;     // Faster attack to catch beats at low volume
    float decayRate = 0.999f;    // Slower decay (keeps gain high for quiet parts)
    float minInputDb = -100.0f;  // Minimum detectable input (-100dB for 1% volume support)
    float maxGainDb = 80.0f;     // Maximum gain in dB (80dB = 10000x amplification for 1% volume!)
    
    void updatePeaks(float bassLin, float midLin, float highLin) {
        // Convert linear to dB with floor
        auto linToDb = [](float lin) -> float {
            if (lin < 0.000001f) return -120.0f;
            return 20.0f * log10f(lin);
        };
        
        float bassDb = linToDb(bassLin);
        float midDb = linToDb(midLin);
        float highDb = linToDb(highLin);
        
        // Update peak tracking in dB domain (handles quiet audio much better)
        if (bassDb > peakDbBass) {
            peakDbBass = peakDbBass + (bassDb - peakDbBass) * attackRate;
        } else {
            // Decay towards minimum (allows gain to increase for quiet audio)
            peakDbBass = peakDbBass * decayRate + minInputDb * (1.0f - decayRate);
        }
        
        if (midDb > peakDbMid) {
            peakDbMid = peakDbMid + (midDb - peakDbMid) * attackRate;
        } else {
            peakDbMid = peakDbMid * decayRate + minInputDb * (1.0f - decayRate);
        }
        
        if (highDb > peakDbHigh) {
            peakDbHigh = peakDbHigh + (highDb - peakDbHigh) * attackRate;
        } else {
            peakDbHigh = peakDbHigh * decayRate + minInputDb * (1.0f - decayRate);
        }
        
        // Clamp peaks to valid range
        if (peakDbBass < minInputDb) peakDbBass = minInputDb;
        if (peakDbMid < minInputDb) peakDbMid = minInputDb;
        if (peakDbHigh < minInputDb) peakDbHigh = minInputDb;
    }
    
    void normalize(float& bass, float& mid, float& high) {
        // Convert input to dB
        auto linToDb = [](float lin) -> float {
            if (lin < 0.000001f) return -120.0f;
            return 20.0f * log10f(lin);
        };
        
        // Convert dB back to linear
        auto dbToLin = [](float db) -> float {
            if (db < -120.0f) return 0.0f;
            return powf(10.0f, db * fast_recipsf2(20.0f));
        };
        
        // Calculate required gain in dB to reach target
        float gainDbBass = targetDb - peakDbBass;
        float gainDbMid = targetDb - peakDbMid;
        float gainDbHigh = targetDb - peakDbHigh;
        
        // Clamp gain to maximum (80dB = 10000x for 1% volume support)
        if (gainDbBass > maxGainDb) gainDbBass = maxGainDb;
        if (gainDbMid > maxGainDb) gainDbMid = maxGainDb;
        if (gainDbHigh > maxGainDb) gainDbHigh = maxGainDb;
        
        // Don't apply negative gain (don't attenuate loud signals for LED effects)
        if (gainDbBass < 0.0f) gainDbBass = 0.0f;
        if (gainDbMid < 0.0f) gainDbMid = 0.0f;
        if (gainDbHigh < 0.0f) gainDbHigh = 0.0f;
        
        // Apply gain in dB domain then convert back to linear
        float bassDb = linToDb(bass) + gainDbBass;
        float midDb = linToDb(mid) + gainDbMid;
        float highDb = linToDb(high) + gainDbHigh;
        
        bass = dbToLin(bassDb);
        mid = dbToLin(midDb);
        high = dbToLin(highDb);
        
        // FIXED FLOOR NORMALIZATION: Ensure minimum level for LED reactivity
        // This guarantees LEDs react even at 1% volume by scaling to minimum floor
        float maxLevel = (bass > mid) ? bass : mid;
        if (high > maxLevel) maxLevel = high;
        
        if (maxLevel > 0.001f && maxLevel < minFloorLin) {
            // Scale all bands proportionally to reach minimum floor
            float scale = minFloorLin * fast_recipsf2(maxLevel);
            bass *= scale;
            mid *= scale;
            high *= scale;
        }
        
        // Clamp to 0-1 range
        if (bass > 1.0f) bass = 1.0f;
        if (mid > 1.0f) mid = 1.0f;
        if (high > 1.0f) high = 1.0f;
    }
};

static AudioAGC g_agc;  // Global AGC instance

// Non-inline helper to read DSP data with AGC - helps avoid GCC ICE
static void __attribute__((noinline)) readDspData(LedAudioReadings& r) {
    r.bass = 0.0f;
    r.mid = 0.0f;
    r.high = 0.0f;
    r.bassDb = -60.0f;
    r.midDb = -60.0f;
    r.highDb = -60.0f;
    r.audioPlaying = false;
    
    if (g_ledDsp) {
        // Get LED audio boost factor (increases as volume decreases)
        // At low volumes, boost audio levels so LEDs can still react
        float ledBoost = g_ledDsp->getLedAudioBoost();
        
        r.bassDb = g_ledDsp->getGoertzel30dB();
        r.midDb = g_ledDsp->getGoertzel60dB();
        r.highDb = g_ledDsp->getGoertzel100dB();
        
        // Read raw linear levels
        float rawBass = g_ledDsp->getGoertzel30Lin() + g_ledDsp->getGoertzel60Lin();
        float rawMid = g_ledDsp->getGoertzel100Lin();
        float rawHigh = g_ledDsp->getPeakLin(2);
        
        // Apply LED audio boost for low volume visibility
        // This allows LEDs to react even at 1-2% volume
        rawBass *= ledBoost;
        rawMid *= ledBoost;
        rawHigh *= ledBoost;
        
        if (rawBass > 1.0f) rawBass = 1.0f;
        if (rawMid > 1.0f) rawMid = 1.0f;
        if (rawHigh > 1.0f) rawHigh = 1.0f;
        
        // Update AGC peak tracking
        g_agc.updatePeaks(rawBass, rawMid, rawHigh);
        
        // Apply AGC normalization
        r.bass = rawBass;
        r.mid = rawMid;
        r.high = rawHigh;
        g_agc.normalize(r.bass, r.mid, r.high);
        
        // Very low threshold for audio detection (-90dB for 1% volume support)
        r.audioPlaying = (r.bassDb > -90.0f || r.midDb > -90.0f || r.highDb > -90.0f);
    }
}

static void ledTask(void* param) {
    ESP_LOGI(LED_TAG, ">>> LED TASK ENTRY <<<");
    
    // Delay briefly to let other startup logs clear
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(LED_TAG, "LED task starting on core %d...", xPortGetCoreID());
    
    LedController& controller = LedController::getInstance();
    
    // Initialize with configured GPIO and brightness
    #ifdef CONFIG_LED_MATRIX_GPIO
        int gpio = CONFIG_LED_MATRIX_GPIO;
    #else
        int gpio = 4;  // Default GPIO
    #endif
    
    ESP_LOGI(LED_TAG, "Initializing LED controller: GPIO=%d", gpio);
    
    #ifdef CONFIG_LED_BRIGHTNESS
        uint8_t brightness = CONFIG_LED_BRIGHTNESS;
    #else
        uint8_t brightness = LED_DEFAULT_BRIGHTNESS;
    #endif
    
    if (!controller.init(gpio, brightness)) {
        ESP_LOGE(LED_TAG, "Failed to initialize LED controller");
        vTaskDelete(nullptr);
        return;
    }
    
    #ifdef CONFIG_LED_DEMO_TIMEOUT
        controller.setDemoTimeout(CONFIG_LED_DEMO_TIMEOUT * 1000);
    #endif
    
    TickType_t lastWake = xTaskGetTickCount();
    
    #ifdef CONFIG_LED_FPS
        const TickType_t frameDelay = pdMS_TO_TICKS(1000 / CONFIG_LED_FPS);
    #else
        const TickType_t frameDelay = pdMS_TO_TICKS(1000 / 30);  // 30 FPS default
    #endif
    
    bool lastBeat = false;
    LedAudioReadings readings;
    
    while (ledTaskRunning) {
        // Check if startup animation was requested (runs once at startup)
        if (controller.consumePendingStartupAnimation()) {
            controller.setStartupAnimationRunning(true);
            controller.playStartupAnimation();
            controller.setStartupAnimationRunning(false);
        }
        
        // Check if in OTA mode - render progress bar instead of effects
        if (controller.isOtaMode()) {
            controller.renderOtaProgress();
            vTaskDelayUntil(&lastWake, frameDelay);
            continue;
        }
        
        // Check if pairing success animation should be shown (highest priority after OTA)
        if (controller.isPairingSuccessActive()) {
            controller.renderPairingSuccess();
            vTaskDelayUntil(&lastWake, frameDelay);
            continue;
        }
        
        // Check if pairing mode animation should be shown
        if (controller.isPairingModeActive()) {
            controller.renderPairingMode();
            vTaskDelayUntil(&lastWake, frameDelay);
            continue;
        }
        
        // Check if volume overlay should be shown (takes priority)
        if (controller.isVolumeOverlayActive()) {
            controller.renderVolumeOverlay();
            vTaskDelayUntil(&lastWake, frameDelay);
            continue;
        }
        
        // Check if EQ overlay should be shown
        if (controller.isEqOverlayActive()) {
            controller.renderEqOverlay();
            vTaskDelayUntil(&lastWake, frameDelay);
            continue;
        }
        
        // Get audio data from DSP processor using helper to avoid ICE
        readDspData(readings);
        
        // Detect beat edge
        bool beat = false;
        bool currentBeat = ledBeatDetected;
        if (currentBeat && !lastBeat) {
            beat = true;
        }
        lastBeat = currentBeat;
        
        // Calculate beat intensity from bass
        float beatIntensity = readings.bass;
        
        // Update LED controller
        controller.update(readings.bass, readings.mid, readings.high, 
                          readings.bassDb, readings.midDb, readings.highDb, 
                          beat, beatIntensity, readings.audioPlaying);
        
        vTaskDelayUntil(&lastWake, frameDelay);
    }
    
    ESP_LOGI(LED_TAG, "LED task stopped");
    vTaskDelete(nullptr);
}

inline void startLedTask(DSPProcessor* dsp, int priority = 3, int stackSize = 4096) {
    ESP_LOGI(LED_TAG, "startLedTask() called: dsp=%p, priority=%d, stack=%d", dsp, priority, stackSize);
    ESP_LOGI(LED_TAG, "Free heap: %lu, internal: %lu, PSRAM: %lu", 
             esp_get_free_heap_size(), 
             heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
             heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    if (!ledTaskRunning) {
        g_ledDsp = dsp;
        ledTaskRunning = true;
        
        // RMT driver requires task stack in internal RAM (not PSRAM)
        // Using standard xTaskCreatePinnedToCore for compatibility
        ESP_LOGI(LED_TAG, "Creating LED task with internal RAM stack...");
        BaseType_t ret = xTaskCreatePinnedToCore(
            ledTask, 
            "led_task", 
            stackSize, 
            nullptr, 
            priority, 
            &ledTaskHandle, 
            0  // Core 0 - core 1 is busy with Bluetooth
        );
        
        ESP_LOGI(LED_TAG, "xTaskCreatePinnedToCore returned: %d, handle=%p", ret, ledTaskHandle);
        if (ret != pdPASS) {
            ESP_LOGE(LED_TAG, "FAILED to create LED task! Error: %d", ret);
            ledTaskRunning = false;
        }
    } else {
        ESP_LOGW(LED_TAG, "LED task already running!");
    }
}

inline void stopLedTask() {
    ledTaskRunning = false;
    if (ledTaskHandle) {
        vTaskDelay(pdMS_TO_TICKS(100));
        ledTaskHandle = nullptr;
    }
}
