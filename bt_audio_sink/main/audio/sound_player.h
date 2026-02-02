#pragma once

// -----------------------------------------------------------
// Sound Player - plays WAV files with dynamic resampling
// Supports: startup, pairing, connected, max volume sounds
// Uses linear interpolation resampler for any I2S sample rate
// 
// Two playback modes:
// - EXCLUSIVE: Sound replaces BT audio (writes directly to I2S)
// - OVERLAY: Sound is mixed with BT audio via OverlayMixer
// -----------------------------------------------------------

#include <stdint.h>
#include <string.h>
#include <cmath>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_heap_caps.h"
#include "../config/app_config.h"
#include "../dsp/fast_math.h"

// -----------------------------------------------------------
// Dynamic Audio Resampler (linear interpolation)
// Works with any input → any output sample rate
// -----------------------------------------------------------

// Fade duration in milliseconds (to eliminate pop sounds)
static constexpr int FADE_MS = 10;  // 10ms fade-in/fade-out

typedef struct {
    float ratio;         // input_rate / output_rate
    float pos;           // fractional position in input stream
    int16_t lastLeft;    // last sample for interpolation (left/mono)
    int16_t lastRight;   // last sample for interpolation (right)
    bool hasLast;        // true after first sample processed
    bool isStereo;       // source is stereo
    
    // Fade-in control to eliminate pop at start
    size_t fadeInFrames;     // Total frames in fade-in ramp
    size_t fadeInRemaining;  // Frames left in fade-in
} AudioResampler;

// Initialize resampler - call when WAV or I2S config changes
static inline void resampler_init(AudioResampler* r,
                                   uint32_t inputRate,
                                   uint32_t outputRate,
                                   bool stereo) {
    r->ratio = (float)inputRate * fast_recipsf2((float)outputRate);
    r->pos = 0.0f;
    r->lastLeft = 0;
    r->lastRight = 0;
    r->hasLast = false;
    r->isStereo = stereo;
    
    // Calculate fade-in frames based on output rate
    r->fadeInFrames = (outputRate * FADE_MS) / 1000;
    r->fadeInRemaining = r->fadeInFrames;
}

// Resample 16-bit audio to 32-bit stereo I2S output
// Returns number of stereo frames written to output
static inline size_t resample_s16_to_s32_stereo(
    AudioResampler* r,
    const int16_t* in,       // Input samples (mono or stereo based on r->isStereo)
    size_t inSamples,        // Number of input FRAMES (not samples)
    int32_t* out,            // Output buffer (stereo interleaved: L,R,L,R...)
    size_t outCapacity)      // Max output FRAMES
{
    size_t outCount = 0;
    
    if (!r->hasLast && inSamples > 0) {
        if (r->isStereo) {
            r->lastLeft = in[0];
            r->lastRight = in[1];
        } else {
            r->lastLeft = r->lastRight = in[0];
        }
        r->hasLast = true;
    }
    
    const size_t inStep = r->isStereo ? 2 : 1;
    
    while (outCount < outCapacity) {
        uint32_t idx = (uint32_t)r->pos;
        uint32_t next = idx + 1;
        
        if (next >= inSamples) {
            break; // Need more input
        }
        
        float frac = r->pos - (float)idx;
        
        int16_t l0, r0, l1, r1;
        
        if (r->isStereo) {
            if (idx == 0 && r->hasLast) {
                l0 = r->lastLeft;
                r0 = r->lastRight;
            } else {
                l0 = in[idx * 2];
                r0 = in[idx * 2 + 1];
            }
            l1 = in[next * 2];
            r1 = in[next * 2 + 1];
        } else {
            // Mono input → stereo output
            if (idx == 0 && r->hasLast) {
                l0 = r0 = r->lastLeft;
            } else {
                l0 = r0 = in[idx];
            }
            l1 = r1 = in[next];
        }
        
        // Linear interpolation
        float sampleL = (1.0f - frac) * l0 + frac * l1;
        float sampleR = (1.0f - frac) * r0 + frac * r1;
        
        // Apply fade-in envelope to eliminate pop at start
        if (r->fadeInRemaining > 0) {
            float fadeGain = 1.0f - ((float)r->fadeInRemaining * fast_recipsf2((float)r->fadeInFrames));
            sampleL *= fadeGain;
            sampleR *= fadeGain;
            r->fadeInRemaining--;
        }
        
        // Expand 16-bit → 32-bit signed (left-align for I2S)
        out[outCount * 2] = (int32_t)(sampleL * 65536.0f);
        out[outCount * 2 + 1] = (int32_t)(sampleR * 65536.0f);
        
        outCount++;
        r->pos += r->ratio;
    }
    
    // Keep fractional remainder, save last sample for next chunk
    uint32_t consumed = (uint32_t)r->pos;
    r->pos -= (float)consumed;
    
    if (inSamples > 0) {
        if (r->isStereo) {
            r->lastLeft = in[(inSamples - 1) * 2];
            r->lastRight = in[(inSamples - 1) * 2 + 1];
        } else {
            r->lastLeft = r->lastRight = in[inSamples - 1];
        }
    }
    
    return outCount;
}

// Convert 8-bit unsigned to 16-bit signed
static inline int16_t convert_u8_to_s16(uint8_t sample) {
    return ((int16_t)sample - 128) << 8;
}

// -----------------------------------------------------------

// Sound types (matches Android app)
enum SoundType {
    SOUND_STARTUP = 0,
    SOUND_PAIRING = 1,
    SOUND_CONNECTED = 2,
    SOUND_MAX_VOLUME = 3,
    SOUND_TYPE_COUNT = 4
};

// Playback modes
enum SoundPlayMode {
    SOUND_MODE_EXCLUSIVE,    // Stop A2DP, play sound, then resume
    SOUND_MODE_OVERLAY       // Mix with current A2DP audio
};

// WAV file header structure
struct WavHeader {
    char riff[4];           // "RIFF"
    uint32_t fileSize;      // File size - 8
    char wave[4];           // "WAVE"
    char fmt[4];            // "fmt "
    uint32_t fmtSize;       // Format chunk size (16 for PCM)
    uint16_t audioFormat;   // 1 = PCM
    uint16_t numChannels;   // 1 = mono, 2 = stereo
    uint32_t sampleRate;    // e.g., 44100
    uint32_t byteRate;      // sampleRate * numChannels * bitsPerSample/8
    uint16_t blockAlign;    // numChannels * bitsPerSample/8
    uint16_t bitsPerSample; // 8 or 16
    // Data chunk follows
};

// Sound file paths in SPIFFS
static const char* SOUND_PATHS[SOUND_TYPE_COUNT] = {
    "/spiffs/startup.wav",
    "/spiffs/pairing.wav",
    "/spiffs/connected.wav",
    "/spiffs/volumemax.wav"
};

class SoundPlayer {
public:
    static SoundPlayer& getInstance() {
        static SoundPlayer instance;
        return instance;
    }

    bool init() {
        if (m_initialized) return true;
        
        // Check if SPIFFS is already mounted (by main.cpp)
        if (!esp_spiffs_mounted("spiffs")) {
            // Initialize SPIFFS
            esp_vfs_spiffs_conf_t conf = {
                .base_path = "/spiffs",
                .partition_label = "spiffs",  // Must match partition table name
                .max_files = 5,
                .format_if_mount_failed = true
            };
            
            esp_err_t ret = esp_vfs_spiffs_register(&conf);
            if (ret != ESP_OK) {
                if (ret == ESP_FAIL) {
                    ESP_LOGE(TAG, "Failed to mount SPIFFS");
                } else if (ret == ESP_ERR_NOT_FOUND) {
                    ESP_LOGE(TAG, "Failed to find SPIFFS partition");
                }
                return false;
            }
            
            ESP_LOGI(TAG, "SPIFFS mounted by SoundPlayer");
        } else {
            ESP_LOGI(TAG, "SPIFFS already mounted");
        }
        
        // Check SPIFFS info
        size_t total = 0, used = 0;
        esp_err_t ret = esp_spiffs_info("spiffs", &total, &used);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "SPIFFS: %u KB used / %u KB total", 
                     (unsigned)(used / 1024), (unsigned)(total / 1024));
        }
        
        // Create mutex for thread-safe access
        m_mutex = xSemaphoreCreateMutex();
        if (!m_mutex) {
            ESP_LOGE(TAG, "Failed to create mutex");
            return false;
        }
        
        // Pre-allocate task stack in internal RAM while we have plenty available
        // This avoids task creation failures later when BT consumes internal RAM
        if (!m_taskStack) {
            m_taskStack = (StackType_t*)heap_caps_malloc(TASK_STACK_SIZE * sizeof(StackType_t), 
                                                          MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
            if (!m_taskStack) {
                ESP_LOGE(TAG, "Failed to pre-allocate task stack from internal RAM");
                return false;
            }
            ESP_LOGI(TAG, "Pre-allocated task stack: %u bytes in internal RAM", 
                     (unsigned)(TASK_STACK_SIZE * sizeof(StackType_t)));
        }
        
        // Check which sound files exist
        updateSoundStatus();
        
        m_initialized = true;
        ESP_LOGI(TAG, "SoundPlayer initialized, status: 0x%02X", m_soundStatus);
        return true;
    }

    // Initialize with a target sample rate
    bool init(uint32_t targetRate) {
        m_targetSampleRate = targetRate;
        return init();
    }

    // Set the current I2S sample rate (called when codec changes)
    // This is now atomic so playback can detect changes
    void setTargetSampleRate(uint32_t rate) {
        m_targetSampleRate = rate;
        m_sampleRateChanged = true;  // Signal to playback loop
    }
    
    // Get current target sample rate
    uint32_t getTargetSampleRate() const {
        return m_targetSampleRate;
    }

    // Set mute state
    void setMuted(bool muted) {
        m_muted = muted;
        ESP_LOGI(TAG, "Sound effects %s", muted ? "muted" : "unmuted");
    }
    bool isMuted() const { return m_muted; }

    // Get sound status byte: bit 0-3 = sound exists, bit 7 = muted
    uint8_t getStatus() const {
        return m_soundStatus | (m_muted ? 0x80 : 0x00);
    }

    // Check if a sound file exists (alias for compatibility)
    bool soundExists(SoundType type) const { return hasSound(type); }

    // Check if a sound file exists
    bool hasSound(SoundType type) const {
        if (type >= SOUND_TYPE_COUNT) return false;
        return (m_soundStatus & (1 << type)) != 0;
    }

    // Queue next sound to play after current finishes
    void queueNext(SoundType type) {
        m_pendingSound = type;
        m_pendingMode = SOUND_MODE_EXCLUSIVE;
        ESP_LOGI(TAG, "Queued next sound: %d", type);
    }

    // Play a sound with explicit sample rate (non-blocking)
    bool play(SoundType type, uint32_t targetRate, SoundPlayMode mode) {
        m_targetSampleRate = targetRate;
        return play(type, mode);
    }

    // Play a sound (non-blocking, starts playback task)
    bool play(SoundType type, SoundPlayMode mode = SOUND_MODE_EXCLUSIVE) {
        if (!m_initialized || m_muted) return false;
        if (type >= SOUND_TYPE_COUNT) return false;
        if (!hasSound(type)) return false;
        
        // If already playing something in exclusive mode, queue it
        if (m_playing && mode == SOUND_MODE_EXCLUSIVE) {
            m_pendingSound = type;
            m_pendingMode = mode;
            ESP_LOGI(TAG, "Queued sound: %d", type);
            return true;
        }
        
        // If a previous task exists, wait for it to fully clean up
        if (m_playbackTaskHandle != nullptr) {
            ESP_LOGW(TAG, "Waiting for previous playback task to finish");
            uint32_t waitStart = millis32();
            while (m_playbackTaskHandle != nullptr && (millis32() - waitStart) < 500) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            if (m_playbackTaskHandle != nullptr) {
                ESP_LOGE(TAG, "Previous task did not finish, aborting");
                return false;
            }
        }
        
        // Start playback
        m_currentSound = type;
        m_playMode = mode;
        m_playing = true;
        m_stopRequested = false;
        
        // Log heap before task creation
        ESP_LOGI(TAG, "Creating playback task, free heap: internal=%u, PSRAM=%u",
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
        
        // Use pre-allocated internal RAM stack with static task creation
        // This avoids dynamic allocation when internal RAM is fragmented
        if (!m_taskStack) {
            ESP_LOGE(TAG, "Task stack not pre-allocated!");
            m_playing = false;
            return false;
        }
        
        m_playbackTaskHandle = xTaskCreateStaticPinnedToCore(
            playbackTask, "sound_play", TASK_STACK_SIZE, this, 5,
            m_taskStack, &m_taskTCB, 0);  // Core 0, away from BT on core 1
        
        if (!m_playbackTaskHandle) {
            ESP_LOGE(TAG, "Failed to create playback task (static), free internal=%u",
                     (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
            m_playing = false;
            return false;
        }
        
        ESP_LOGI(TAG, "Playing sound: %d (mode=%d)", type, mode);
        return true;
    }

    // Stop current playback
    void stop() {
        m_stopRequested = true;
        m_pendingSound = -1;
    }

    // Wait for current sound to finish
    void waitForCompletion(uint32_t timeoutMs = 10000) {
        uint32_t start = millis32();
        while (m_playing && (millis32() - start) < timeoutMs) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }

    // Check if currently playing
    bool isPlaying() const { return m_playing; }

    // Delete a sound file
    bool deleteSound(SoundType type) {
        if (type >= SOUND_TYPE_COUNT) return false;
        
        xSemaphoreTake(m_mutex, portMAX_DELAY);
        
        int ret = remove(SOUND_PATHS[type]);
        if (ret == 0) {
            m_soundStatus &= ~(1 << type);
            ESP_LOGI(TAG, "Deleted sound: %s", SOUND_PATHS[type]);
        }
        
        xSemaphoreGive(m_mutex);
        return ret == 0;
    }

    // Save uploaded sound file
    bool saveSound(SoundType type, const uint8_t* data, size_t len) {
        if (type >= SOUND_TYPE_COUNT) return false;
        if (len < sizeof(WavHeader)) return false;
        
        xSemaphoreTake(m_mutex, portMAX_DELAY);
        
        FILE* f = fopen(SOUND_PATHS[type], "wb");
        if (!f) {
            ESP_LOGE(TAG, "Failed to create file: %s", SOUND_PATHS[type]);
            xSemaphoreGive(m_mutex);
            return false;
        }
        
        size_t written = fwrite(data, 1, len, f);
        fclose(f);
        
        if (written == len) {
            m_soundStatus |= (1 << type);
            ESP_LOGI(TAG, "Saved sound: %s (%u bytes)", SOUND_PATHS[type], (unsigned)len);
        }
        
        xSemaphoreGive(m_mutex);
        return written == len;
    }

    // Get samples for mixing with A2DP (for overlay mode)
    // Returns number of samples written, 0 if no sound playing
    // Note: Overlay mode not fully implemented with streaming resampler
    size_t getMixSamples(int32_t* outL, int32_t* outR, size_t frames) {
        if (!m_playing || m_playMode != SOUND_MODE_OVERLAY) return 0;
        
        // Zero the buffers - overlay mode needs ring buffer implementation
        for (size_t i = 0; i < frames; i++) {
            outL[i] = 0;
            outR[i] = 0;
        }
        
        return 0;  // Not implemented for streaming resampler
    }

    // Callback for audio pipeline to check if sound is active
    // Returns true if exclusive sound is playing (should pause A2DP)
    bool isExclusivePlaying() const {
        return m_playing && m_playMode == SOUND_MODE_EXCLUSIVE;
    }

    // Refresh sound file status (public for use after upload)
    void refreshStatus() {
        m_soundStatus = 0;
        for (int i = 0; i < SOUND_TYPE_COUNT; i++) {
            FILE* f = fopen(SOUND_PATHS[i], "rb");
            if (f) {
                fclose(f);
                m_soundStatus |= (1 << i);
            }
        }
        ESP_LOGI(TAG, "Sound status refreshed: 0x%02X", m_soundStatus);
    }

private:
    static constexpr const char* TAG = "SoundPlayer";
    
    SoundPlayer() = default;
    
    void updateSoundStatus() {
        m_soundStatus = 0;
        for (int i = 0; i < SOUND_TYPE_COUNT; i++) {
            FILE* f = fopen(SOUND_PATHS[i], "rb");
            if (f) {
                fclose(f);
                m_soundStatus |= (1 << i);
            }
        }
    }
    
    static uint32_t millis32() {
        return (uint32_t)(esp_timer_get_time() / 1000);
    }
    
    // Playback task
    static void playbackTask(void* param) {
        SoundPlayer* self = (SoundPlayer*)param;
        self->doPlayback();
        vTaskDelete(NULL);
    }
    
    void doPlayback() {
        ESP_LOGI(TAG, ">>> doPlayback task started");
        
        xSemaphoreTake(m_mutex, portMAX_DELAY);
        
        ESP_LOGI(TAG, ">>> mutex acquired, opening file");
        
        const char* path = SOUND_PATHS[m_currentSound];
        FILE* f = fopen(path, "rb");
        if (!f) {
            ESP_LOGE(TAG, "Failed to open: %s", path);
            m_playing = false;
            xSemaphoreGive(m_mutex);
            return;
        }
        
        // Read WAV header
        WavHeader header;
        if (fread(&header, 1, sizeof(header), f) != sizeof(header)) {
            ESP_LOGE(TAG, "Failed to read WAV header");
            fclose(f);
            m_playing = false;
            xSemaphoreGive(m_mutex);
            return;
        }
        
        // Validate WAV
        if (memcmp(header.riff, "RIFF", 4) != 0 || 
            memcmp(header.wave, "WAVE", 4) != 0 ||
            header.audioFormat != 1) {
            ESP_LOGE(TAG, "Invalid WAV file");
            fclose(f);
            m_playing = false;
            xSemaphoreGive(m_mutex);
            return;
        }
        
        ESP_LOGI(TAG, "WAV: %uHz %ubit %uch -> I2S %uHz", 
                 (unsigned)header.sampleRate, 
                 (unsigned)header.bitsPerSample,
                 (unsigned)header.numChannels,
                 (unsigned)m_targetSampleRate);
        
        // Find data chunk
        char chunkId[4];
        uint32_t chunkSize;
        while (fread(chunkId, 1, 4, f) == 4) {
            fread(&chunkSize, 4, 1, f);
            if (memcmp(chunkId, "data", 4) == 0) {
                break;
            }
            fseek(f, chunkSize, SEEK_CUR);
        }
        
        xSemaphoreGive(m_mutex);
        
        // Initialize the streaming resampler
        AudioResampler resampler;
        bool isStereo = (header.numChannels == 2);
        uint32_t currentOutputRate = m_targetSampleRate;
        m_sampleRateChanged = false;  // Clear flag
        resampler_init(&resampler, header.sampleRate, currentOutputRate, isStereo);
        
        // Calculate buffer sizes
        // Input: enough for ~20ms of audio at source rate
        const size_t inputChunkFrames = (header.sampleRate * 20) / 1000;  // ~20ms worth
        const size_t inputChunkSamples = inputChunkFrames * header.numChannels;
        const size_t inputBytesPerFrame = header.blockAlign;
        
        // Output: enough for resampled frames (upsampling ratio can be ~4.4x for 22050->96000)
        // Use max possible rate (96kHz) for buffer sizing to handle any rate change
        const size_t maxOutputFrames = (inputChunkFrames * 96000 / header.sampleRate) + 16;
        
        // Allocate buffers - prefer PSRAM to avoid exhausting internal RAM
        // Internal RAM is needed for BLE/BT operations
        uint8_t* inputRaw = (uint8_t*)heap_caps_malloc(inputChunkFrames * inputBytesPerFrame, 
                                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!inputRaw) {
            inputRaw = (uint8_t*)heap_caps_malloc(inputChunkFrames * inputBytesPerFrame, MALLOC_CAP_8BIT);
        }
        int16_t* inputS16 = (int16_t*)heap_caps_malloc(inputChunkSamples * sizeof(int16_t), 
                                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!inputS16) {
            inputS16 = (int16_t*)heap_caps_malloc(inputChunkSamples * sizeof(int16_t), MALLOC_CAP_8BIT);
        }
        // Output buffer doesn't need DMA capability - I2S driver copies from it
        int32_t* outputS32 = (int32_t*)heap_caps_malloc(maxOutputFrames * 2 * sizeof(int32_t), 
                                                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!outputS32) {
            outputS32 = (int32_t*)heap_caps_malloc(maxOutputFrames * 2 * sizeof(int32_t), MALLOC_CAP_8BIT);
        }
        
        if (!inputRaw || !inputS16 || !outputS32) {
            ESP_LOGE(TAG, "Failed to allocate playback buffers (in=%u, out=%u bytes)", 
                     (unsigned)(inputChunkFrames * inputBytesPerFrame),
                     (unsigned)(maxOutputFrames * 2 * sizeof(int32_t)));
            if (inputRaw) heap_caps_free(inputRaw);
            if (inputS16) heap_caps_free(inputS16);
            if (outputS32) heap_caps_free(outputS32);
            fclose(f);
            m_playing = false;
            return;
        }
        
        ESP_LOGI(TAG, "Resampler: ratio=%.4f, input=%u frames, output max=%u frames",
                 resampler.ratio, (unsigned)inputChunkFrames, (unsigned)maxOutputFrames);
        
        // Streaming playback loop
        size_t totalDataBytes = chunkSize;
        size_t bytesRead = 0;
        uint32_t playbackStartTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
        uint32_t maxPlaybackTimeMs = 10000;  // Maximum 10 seconds for any sound
        uint32_t consecutiveWriteFailures = 0;
        const uint32_t maxConsecutiveFailures = 20;  // Abort after 20 consecutive I2S write failures
        
        while (bytesRead < totalDataBytes && !m_stopRequested) {
            // Check for timeout to prevent infinite loops
            uint32_t elapsed = (xTaskGetTickCount() * portTICK_PERIOD_MS) - playbackStartTime;
            if (elapsed > maxPlaybackTimeMs) {
                ESP_LOGW(TAG, "Sound playback timeout after %u ms, aborting", elapsed);
                break;
            }
            // Check if sample rate changed mid-playback
            if (m_sampleRateChanged) {
                uint32_t newRate = m_targetSampleRate;
                if (newRate != currentOutputRate) {
                    ESP_LOGI(TAG, "Sample rate changed %u -> %u Hz, reinit resampler",
                             (unsigned)currentOutputRate, (unsigned)newRate);
                    currentOutputRate = newRate;
                    // Reinitialize resampler with new output rate (preserving position)
                    resampler_init(&resampler, header.sampleRate, currentOutputRate, isStereo);
                }
                m_sampleRateChanged = false;
            }
            
            // Read a chunk of raw audio
            size_t toRead = inputChunkFrames * inputBytesPerFrame;
            size_t remaining = totalDataBytes - bytesRead;
            if (toRead > remaining) toRead = remaining;
            
            xSemaphoreTake(m_mutex, portMAX_DELAY);
            size_t actualRead = fread(inputRaw, 1, toRead, f);
            xSemaphoreGive(m_mutex);
            
            if (actualRead == 0) break;
            
            bytesRead += actualRead;
            size_t inputFrames = actualRead / inputBytesPerFrame;
            
            // Convert to 16-bit signed if needed
            if (header.bitsPerSample == 16) {
                // Already 16-bit, just copy
                memcpy(inputS16, inputRaw, inputFrames * header.numChannels * sizeof(int16_t));
            } else if (header.bitsPerSample == 8) {
                // Convert 8-bit unsigned to 16-bit signed
                for (size_t i = 0; i < inputFrames * header.numChannels; i++) {
                    inputS16[i] = convert_u8_to_s16(inputRaw[i]);
                }
            }
            
            // Resample this chunk to 32-bit stereo for I2S
            size_t outputFrames = resample_s16_to_s32_stereo(
                &resampler,
                inputS16,
                inputFrames,
                outputS32,
                maxOutputFrames
            );
            
            // Output samples
            if (outputFrames > 0) {
                if (m_playMode == SOUND_MODE_OVERLAY && m_overlayPushFunc) {
                    // Push to overlay mixer for mixing with BT audio in DSP
                    m_overlayPushFunc(outputS32, outputFrames);
                    // Rate-limit to match real-time playback
                    // Calculate how many ms of audio we just pushed and delay accordingly
                    // This prevents buffer overflow and ensures audio plays at correct speed
                    uint32_t audioMs = (outputFrames * 1000) / currentOutputRate;
                    if (audioMs < 1) audioMs = 1;
                    // Delay slightly less than the audio duration to stay ahead of consumption
                    vTaskDelay(pdMS_TO_TICKS(audioMs > 2 ? audioMs - 1 : audioMs));
                } else if (m_playMode == SOUND_MODE_EXCLUSIVE && m_i2sWriteFunc) {
                    // Write directly to I2S (exclusive mode)
                    size_t written = m_i2sWriteFunc((uint8_t*)outputS32, outputFrames * 2 * sizeof(int32_t));
                    if (written == 0) {
                        consecutiveWriteFailures++;
                        if (consecutiveWriteFailures >= maxConsecutiveFailures) {
                            ESP_LOGW(TAG, "Too many I2S write failures (%u), aborting playback", consecutiveWriteFailures);
                            break;
                        }
                        // I2S might be reconfiguring, wait a bit
                        vTaskDelay(pdMS_TO_TICKS(10));
                    } else {
                        consecutiveWriteFailures = 0;  // Reset on successful write
                    }
                }
            }
            
            // Small yield to prevent watchdog
            vTaskDelay(1);
        }
        
        // Cleanup
        heap_caps_free(inputRaw);
        heap_caps_free(inputS16);
        heap_caps_free(outputS32);
        fclose(f);
        
        ESP_LOGI(TAG, "Playback complete");
        
        // Check for pending sound BEFORE clearing state
        SoundType pending = (SoundType)m_pendingSound;
        SoundPlayMode pendingMode = m_pendingMode;
        m_pendingSound = -1;
        
        // Clear state - important: do this BEFORE calling play() to avoid
        // the "previous task still running" check blocking us
        m_playing = false;
        m_playbackTaskHandle = nullptr;
        
        // Now play the pending sound (this creates a new task)
        if (pending >= 0 && pending < SOUND_TYPE_COUNT) {
            ESP_LOGI(TAG, "Playing queued sound: %d", pending);
            play(pending, pendingMode);
        }
    }
    
    // Task stack size - allocated once during init for reuse
    static constexpr size_t TASK_STACK_SIZE = 4096;
    
    bool m_initialized = false;
    bool m_muted = false;
    uint8_t m_soundStatus = 0;
    uint32_t m_targetSampleRate = 44100;
    
    SemaphoreHandle_t m_mutex = nullptr;
    TaskHandle_t m_playbackTaskHandle = nullptr;
    
    // Pre-allocated task stack and TCB for static task creation
    StackType_t* m_taskStack = nullptr;
    StaticTask_t m_taskTCB;
    
    volatile bool m_playing = false;
    volatile bool m_stopRequested = false;
    volatile bool m_sampleRateChanged = false;  // Set when I2S rate changes
    SoundType m_currentSound = SOUND_STARTUP;
    SoundPlayMode m_playMode = SOUND_MODE_EXCLUSIVE;
    
    int m_pendingSound = -1;
    SoundPlayMode m_pendingMode = SOUND_MODE_EXCLUSIVE;
    
public:
    // I2S write function pointer (set by main for exclusive mode)
    using I2SWriteFunc = size_t(*)(const uint8_t*, size_t);
    I2SWriteFunc m_i2sWriteFunc = nullptr;
    
    void setI2SWriteFunc(I2SWriteFunc func) {
        m_i2sWriteFunc = func;
    }
    
    // Overlay push function pointer (set by main for overlay mode)
    // Pushes resampled stereo samples to OverlayMixer for mixing with BT audio
    using OverlayPushFunc = void(*)(const int32_t* stereoSamples, size_t frames);
    OverlayPushFunc m_overlayPushFunc = nullptr;
    
    void setOverlayPushFunc(OverlayPushFunc func) {
        m_overlayPushFunc = func;
    }
};
