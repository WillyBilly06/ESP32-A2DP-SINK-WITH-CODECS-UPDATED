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
#include "esp_timer.h"
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
    float pos;           // fractional position in a virtual stream
                         // where index 0 is the previous frame and
                         // index 1 is the first frame of the current chunk
    int16_t lastLeft;    // previous frame for interpolation across chunks
    int16_t lastRight;   // previous frame for interpolation across chunks
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
static inline void resampler_get_virtual_frame(AudioResampler* r,
                                                const int16_t* in,
                                                size_t frameIndex,
                                                int16_t* outL,
                                                int16_t* outR)
{
    // Virtual source layout for each streaming chunk:
    //   frame 0      = previous chunk's last sample
    //   frame 1..N   = current chunk samples 0..N-1
    // This prevents interpolation from skipping or reusing the wrong sample at
    // chunk boundaries. It also lets the WAV stream stay purely buffer-by-buffer.
    if (frameIndex == 0) {
        *outL = r->lastLeft;
        *outR = r->lastRight;
        return;
    }

    const size_t srcFrame = frameIndex - 1;
    if (r->isStereo) {
        *outL = in[srcFrame * 2 + 0];
        *outR = in[srcFrame * 2 + 1];
    } else {
        *outL = in[srcFrame];
        *outR = in[srcFrame];
    }
}

// Resample 16-bit audio to 32-bit stereo output.
// Streaming-safe: keeps one previous frame and interpolates across chunk
// boundaries without loading the whole WAV into RAM.
// Returns number of stereo frames written to output.
static inline size_t resample_s16_to_s32_stereo(
    AudioResampler* r,
    const int16_t* in,       // Input samples (mono or stereo based on r->isStereo)
    size_t inFrames,         // Number of input FRAMES (not samples)
    int32_t* out,            // Output buffer (stereo interleaved: L,R,L,R...)
    size_t outCapacity)      // Max output FRAMES
{
    if (!r || !in || !out || inFrames == 0 || outCapacity == 0) {
        return 0;
    }

    // Seed the previous-frame state with the first frame. This produces a safe
    // zero-length interpolation at playback start and then true continuous
    // interpolation for all later chunks.
    if (!r->hasLast) {
        if (r->isStereo) {
            r->lastLeft = in[0];
            r->lastRight = in[1];
        } else {
            r->lastLeft = in[0];
            r->lastRight = in[0];
        }
        r->hasLast = true;
        r->pos = 0.0f;
    }

    size_t outCount = 0;

    // Virtual frame count = previous frame + current chunk frames.
    // Valid interpolation requires idx+1 <= inFrames.
    while (outCount < outCapacity) {
        uint32_t idx = (uint32_t)r->pos;
        uint32_t next = idx + 1;

        if (next > inFrames) {
            break;  // Need the next input chunk.
        }

        float frac = r->pos - (float)idx;

        int16_t l0, r0, l1, r1;
        resampler_get_virtual_frame(r, in, idx, &l0, &r0);
        resampler_get_virtual_frame(r, in, next, &l1, &r1);

        float sampleL = (1.0f - frac) * (float)l0 + frac * (float)l1;
        float sampleR = (1.0f - frac) * (float)r0 + frac * (float)r1;

        // Apply fade-in envelope to eliminate pop at start.
        if (r->fadeInRemaining > 0 && r->fadeInFrames > 0) {
            float fadeGain = 1.0f - ((float)r->fadeInRemaining * fast_recipsf2((float)r->fadeInFrames));
            sampleL *= fadeGain;
            sampleR *= fadeGain;
            r->fadeInRemaining--;
        }

        out[outCount * 2 + 0] = (int32_t)(sampleL * 65536.0f);
        out[outCount * 2 + 1] = (int32_t)(sampleR * 65536.0f);

        outCount++;
        r->pos += r->ratio;
    }

    // Keep position relative to the last frame of this chunk. The next call will
    // provide that frame as virtual index 0 via lastLeft/lastRight.
    uint32_t consumed = (uint32_t)r->pos;
    if (consumed > inFrames) consumed = (uint32_t)inFrames;
    r->pos -= (float)consumed;

    // Save the last current input frame so the next chunk can interpolate across
    // the boundary. No old WAV samples are reused after playback ends because the
    // task exits and the overlay end hook is called immediately.
    if (r->isStereo) {
        r->lastLeft = in[(inFrames - 1) * 2 + 0];
        r->lastRight = in[(inFrames - 1) * 2 + 1];
    } else {
        r->lastLeft = in[inFrames - 1];
        r->lastRight = in[inFrames - 1];
    }

    return outCount;
}

// Convert 8-bit unsigned to 16-bit signed
static inline int16_t convert_u8_to_s16(uint8_t sample) {
    return ((int16_t)sample - 128) << 8;
}

static inline int16_t convert_s24le_to_s16(const uint8_t* sample) {
    int32_t v = (int32_t)sample[0] | ((int32_t)sample[1] << 8) | ((int32_t)sample[2] << 16);
    if (v & 0x00800000) {
        v |= 0xFF000000;
    }
    return (int16_t)(v >> 8);
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
        
        // Check which sound files exist
        updateSoundStatus();
        
        // Pre-allocate playback buffers ONCE so no play ever touches the heap.
        m_inputRaw  = (uint8_t*)heap_caps_malloc(kInputRawBytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        m_inputS16  = (int16_t*)heap_caps_malloc(kInputChunkSamples * sizeof(int16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        m_outputS32 = (int32_t*)heap_caps_malloc(kOutputFramesAlloc * 2 * sizeof(int32_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!m_inputRaw || !m_inputS16 || !m_outputS32) {
            ESP_LOGE(TAG, "Failed to pre-allocate playback buffers");
            if (m_inputRaw)  { heap_caps_free(m_inputRaw);  m_inputRaw = nullptr; }
            if (m_inputS16)  { heap_caps_free(m_inputS16);  m_inputS16 = nullptr; }
            if (m_outputS32) { heap_caps_free(m_outputS32); m_outputS32 = nullptr; }
            return false;
        }
        
        // Create the persistent playback task once. It idles on a notification.
        if (xTaskCreatePinnedToCore(playbackTask, "sound_play", TASK_STACK_SIZE,
                                    this, 5, &m_playbackTaskHandle, 0) != pdPASS) {
            ESP_LOGE(TAG, "Failed to create persistent playback task");
            return false;
        }
        
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

        // Hard safety rule for the mixer architecture:
        // the max-volume UI sound must NEVER take over I2S directly. It is a
        // PCM overlay sound and must be mixed by AudioPipeline after DSP. This
        // prevents the I2S/DMA handoff corruption that made BT audio stay
        // distorted until power-cycle.
        if (type == SOUND_MAX_VOLUME) {
            mode = SOUND_MODE_OVERLAY;
        }

        // If overlay was requested but the mixer hooks are not connected, do not
        // silently fall back to exclusive I2S. Failing safely is better than
        // poisoning the shared I2S output path.
        if (mode == SOUND_MODE_OVERLAY &&
            (!m_overlayPushFunc || !m_overlayStartFunc || !m_overlayEndFunc)) {
            ESP_LOGE(TAG, "Overlay sound requested but overlay mixer hooks are missing");
            return false;
        }
        
        // If already playing, queue it (single pending slot).
        if (m_playing) {
            m_pendingSound = type;
            m_pendingMode = mode;
            ESP_LOGI(TAG, "Queued sound: %d", type);
            return true;
        }
        
        // Hand off to the persistent task. No allocation, no task spawn -> zero
        // heap cost, so triggering a sound can't starve the A2DP decoder.
        m_currentSound = type;
        m_playMode = mode;
        m_playing = true;
        m_stopRequested = false;
        m_playRequested = true;
        if (m_playbackTaskHandle) xTaskNotifyGive(m_playbackTaskHandle);
        
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
    
    // Persistent playback task: idles on a notification, plays, loops forever.
    static void playbackTask(void* param) {
        SoundPlayer* self = (SoundPlayer*)param;
        for (;;) {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            if (self->m_playRequested) {
                self->m_playRequested = false;
                self->doPlayback();
            }
        }
    }
    
    void doPlayback() {
        ESP_LOGD(TAG, ">>> doPlayback task started");
        
        xSemaphoreTake(m_mutex, portMAX_DELAY);
        
        ESP_LOGD(TAG, ">>> mutex acquired, opening file");
        
        const char* path = SOUND_PATHS[m_currentSound];
        FILE* f = fopen(path, "rb");
        if (!f) {
            ESP_LOGE(TAG, "Failed to open: %s", path);
            m_playing = false;
            xSemaphoreGive(m_mutex);
            return;
        }
        
        uint8_t riffHeader[12];
        if (fread(riffHeader, 1, sizeof(riffHeader), f) != sizeof(riffHeader)) {
            ESP_LOGE(TAG, "Failed to read WAV RIFF header");
            fclose(f);
            m_playing = false;
            xSemaphoreGive(m_mutex);
            return;
        }

        if (memcmp(riffHeader, "RIFF", 4) != 0 || memcmp(riffHeader + 8, "WAVE", 4) != 0) {
            ESP_LOGE(TAG, "Invalid WAV file");
            fclose(f);
            m_playing = false;
            xSemaphoreGive(m_mutex);
            return;
        }

        uint16_t audioFormat = 0;
        uint16_t numChannels = 0;
        uint32_t sampleRate = 0;
        uint16_t blockAlign = 0;
        uint16_t bitsPerSample = 0;
        uint32_t dataChunkSize = 0;
        bool foundFmt = false;
        bool foundData = false;

        char chunkId[4];
        uint32_t chunkSize;
        while (fread(chunkId, 1, 4, f) == 4) {
            if (fread(&chunkSize, 4, 1, f) != 1) {
                break;
            }

            long chunkDataPos = ftell(f);
            if (memcmp(chunkId, "data", 4) == 0) {
                dataChunkSize = chunkSize;
                foundData = true;
                break;
            }

            if (memcmp(chunkId, "fmt ", 4) == 0) {
                uint8_t fmt[16];
                if (chunkSize < sizeof(fmt) || fread(fmt, 1, sizeof(fmt), f) != sizeof(fmt)) {
                    ESP_LOGE(TAG, "Invalid WAV fmt chunk");
                    fclose(f);
                    m_playing = false;
                    xSemaphoreGive(m_mutex);
                    return;
                }

                audioFormat = (uint16_t)fmt[0] | ((uint16_t)fmt[1] << 8);
                numChannels = (uint16_t)fmt[2] | ((uint16_t)fmt[3] << 8);
                sampleRate = (uint32_t)fmt[4] | ((uint32_t)fmt[5] << 8) |
                             ((uint32_t)fmt[6] << 16) | ((uint32_t)fmt[7] << 24);
                blockAlign = (uint16_t)fmt[12] | ((uint16_t)fmt[13] << 8);
                bitsPerSample = (uint16_t)fmt[14] | ((uint16_t)fmt[15] << 8);
                foundFmt = true;
            }

            long nextChunk = chunkDataPos + chunkSize + (chunkSize & 1);
            fseek(f, nextChunk, SEEK_SET);
        }

        if (!foundFmt || !foundData || audioFormat != 1 ||
            (numChannels != 1 && numChannels != 2) ||
            (bitsPerSample != 8 && bitsPerSample != 16 && bitsPerSample != 24) ||
            sampleRate == 0 || blockAlign == 0) {
            ESP_LOGE(TAG, "Unsupported WAV: fmt=%u foundFmt=%d foundData=%d sr=%u bits=%u ch=%u align=%u",
                     audioFormat, foundFmt, foundData, (unsigned)sampleRate,
                     bitsPerSample, numChannels, blockAlign);
            fclose(f);
            m_playing = false;
            xSemaphoreGive(m_mutex);
            return;
        }

        ESP_LOGD(TAG, "WAV: %uHz %ubit %uch -> I2S %uHz",
                 (unsigned)sampleRate,
                 bitsPerSample,
                 numChannels,
                 (unsigned)m_targetSampleRate);
        
        xSemaphoreGive(m_mutex);
        
        // Initialize the streaming resampler
        AudioResampler resampler;
        bool isStereo = (numChannels == 2);
        uint32_t currentOutputRate = m_targetSampleRate;
        m_sampleRateChanged = false;  // Clear flag
        resampler_init(&resampler, sampleRate, currentOutputRate, isStereo);
        
        // Use the PRE-ALLOCATED persistent buffers (no per-play heap allocation).
        // Output is bounded to kMaxOutputFrames; derive a matching input chunk and
        // clamp it to what the persistent buffers can hold.
        uint32_t outRate = m_targetSampleRate ? m_targetSampleRate : 44100;
        const size_t inputBytesPerFrame = blockAlign;
        size_t inputChunkFrames = (size_t)(((uint64_t)kMaxOutputFrames * sampleRate) /
                                           (outRate ? outRate : sampleRate));
        if (inputChunkFrames == 0) inputChunkFrames = 1;
        // Clamp so we never exceed the pre-allocated buffer capacities.
        size_t maxFramesByS16 = kInputChunkSamples / (numChannels ? numChannels : 1);
        size_t maxFramesByRaw = kInputRawBytes / (inputBytesPerFrame ? inputBytesPerFrame : 1);
        if (inputChunkFrames > maxFramesByS16) inputChunkFrames = maxFramesByS16;
        if (inputChunkFrames > maxFramesByRaw) inputChunkFrames = maxFramesByRaw;
        if (inputChunkFrames == 0) inputChunkFrames = 1;
        const size_t maxOutputFrames = kMaxOutputFrames + 32;
        
        // For 16-bit WAVs we read directly into the s16 buffer; otherwise into raw.
        uint8_t* inputRaw = m_inputRaw;
        int16_t* inputS16 = m_inputS16;
        int32_t* outputS32 = m_outputS32;
        
        ESP_LOGD(TAG, "Resampler: ratio=%.4f, input=%u frames, output max=%u frames",
                 resampler.ratio, (unsigned)inputChunkFrames, (unsigned)maxOutputFrames);
        
        // Begin ducking the BT audio for the whole overlay effect.
        bool overlayStarted = false;
        if (m_playMode == SOUND_MODE_OVERLAY && m_overlayStartFunc && m_overlayPushFunc) {
            m_overlayStartFunc();
            overlayStarted = true;
        }
        
        // Streaming playback loop
        size_t totalDataBytes = dataChunkSize;
        size_t bytesRead = 0;
        uint32_t playbackStartTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
        uint32_t maxPlaybackTimeMs = 10000;  // Maximum 10 seconds for any sound
        uint32_t consecutiveWriteFailures = 0;
        const uint32_t maxConsecutiveFailures = 20;  // Abort after 20 consecutive I2S write failures

        // This is a strict streaming loop: one small SPIFFS read, one resample,
        // one overlay push. When bytesRead reaches the data chunk size or stop()
        // is requested, the loop exits, the FILE is closed, and overlayEnd is
        // signalled. Nothing keeps reading or resampling after the effect ends.
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
                    ESP_LOGD(TAG, "Sample rate changed %u -> %u Hz, reinit resampler",
                             (unsigned)currentOutputRate, (unsigned)newRate);
                    currentOutputRate = newRate;
                    // Reinitialize resampler with new output rate (preserving position)
                    resampler_init(&resampler, sampleRate, currentOutputRate, isStereo);
                }
                m_sampleRateChanged = false;
            }
            
            // Read a chunk of raw audio
            size_t toRead = inputChunkFrames * inputBytesPerFrame;
            size_t remaining = totalDataBytes - bytesRead;
            if (toRead > remaining) toRead = remaining;
            
            xSemaphoreTake(m_mutex, portMAX_DELAY);
            size_t actualRead;
            if (bitsPerSample == 16) {
                actualRead = fread(inputS16, 1, toRead, f);
            } else {
                actualRead = fread(inputRaw, 1, toRead, f);
            }
            xSemaphoreGive(m_mutex);
            
            if (actualRead == 0) break;
            
            bytesRead += actualRead;
            size_t inputFrames = actualRead / inputBytesPerFrame;
            
            // Convert to 16-bit signed if needed
            if (bitsPerSample == 8) {
                // Convert 8-bit unsigned to 16-bit signed
                for (size_t i = 0; i < inputFrames * numChannels; i++) {
                    inputS16[i] = convert_u8_to_s16(inputRaw[i]);
                }
            } else if (bitsPerSample == 24) {
                for (size_t i = 0; i < inputFrames * numChannels; i++) {
                    inputS16[i] = convert_s24le_to_s16(inputRaw + (i * 3));
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
                    // Push to the overlay mixer, retrying the remainder when the
                    // ring is full. The render drains the ring at the real I2S
                    // rate, so waiting for space paces us to real time AND drops
                    // nothing (no truncation/choppiness at high sample rates).
                    size_t pushed = 0;
                    uint32_t guard = 0;
                    while (pushed < outputFrames && !m_stopRequested) {
                        size_t n = m_overlayPushFunc(outputS32 + pushed * 2,
                                                     outputFrames - pushed);
                        pushed += n;
                        if (pushed < outputFrames) {
                            vTaskDelay(1);  // ring full: let the render drain it
                            if (++guard > 120) {
                                ESP_LOGW(TAG, "Overlay ring did not drain; aborting sound stream");
                                m_stopRequested = true;
                                break;
                            }
                        }
                    }
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
        
        // End ducking: ramp the BT audio back to full volume.
        if (overlayStarted && m_overlayEndFunc) {
            m_overlayEndFunc();
        }

        // If this was stopped/aborted before the overlay was started, make sure
        // there is no stale overlay state left behind. This is intentionally only
        // a stop signal; OverlayMixer will drain already queued frames before
        // restoring the BT gain.
        if (m_playMode == SOUND_MODE_OVERLAY && !overlayStarted && m_overlayEndFunc) {
            m_overlayEndFunc();
        }
        
        // Close the file. Buffers are persistent -- do NOT free them, and do NOT
        // touch m_playbackTaskHandle (the task lives for the program's lifetime).
        fclose(f);
        
        ESP_LOGD(TAG, "Playback complete");
        
        // Check for a queued sound, then clear playing state.
        SoundType pending = (SoundType)m_pendingSound;
        SoundPlayMode pendingMode = m_pendingMode;
        m_pendingSound = -1;
        m_playing = false;
        
        // Re-arm the persistent task for the pending sound (no spawn/alloc).
        if (pending >= 0 && pending < SOUND_TYPE_COUNT && hasSound(pending) && !m_muted) {
            m_currentSound = pending;
            m_playMode = pendingMode;
            m_playing = true;
            m_stopRequested = false;
            m_playRequested = true;
            if (m_playbackTaskHandle) xTaskNotifyGive(m_playbackTaskHandle);
            ESP_LOGD(TAG, "Playing queued sound: %d", pending);
        }
    }
    
    // Persistent playback task: created ONCE at init and reused for every sound.
    // (Previously a task was spawned per play, which at LDAC 96k drove free heap
    // down to ~2 KB and disrupted the A2DP decoder -> burst of dropped BT frames
    // heard as lasting static. A persistent task + pre-allocated buffers makes
    // triggering a sound cost ZERO heap.)
    static constexpr size_t TASK_STACK_SIZE = 3072;
    // Pre-allocated playback buffers, sized for a low-RAM build. The sound
    // effect is paced by the overlay ring, so smaller chunks are fine and avoid
    // permanently burning internal heap while LDAC is streaming. Total ~3 KB.
    //   output: 128 frames per chunk (+32 slack)
    //   input:  128 output frames need <=160 input frames even when the WAV rate
    //           slightly exceeds the I2S rate (e.g. 48k WAV -> 44.1k I2S).
    static constexpr size_t kMaxOutputFrames = 128;        // <= overlay ring size
    static constexpr size_t kMaxInputFrames  = 160;
    static constexpr size_t kInputChunkSamples = kMaxInputFrames * 2;       // stereo max
    static constexpr size_t kInputRawBytes     = kMaxInputFrames * 2 * 3;   // 24-bit stereo
    static constexpr size_t kOutputFramesAlloc = kMaxOutputFrames + 32;
    
    bool m_initialized = false;
    bool m_muted = false;
    uint8_t m_soundStatus = 0;
    uint32_t m_targetSampleRate = 44100;
    
    SemaphoreHandle_t m_mutex = nullptr;
    TaskHandle_t m_playbackTaskHandle = nullptr;
    
    // Persistent playback buffers (allocated once in init()).
    uint8_t*  m_inputRaw = nullptr;
    int16_t*  m_inputS16 = nullptr;
    int32_t*  m_outputS32 = nullptr;
    
    volatile bool m_playing = false;
    volatile bool m_stopRequested = false;
    volatile bool m_sampleRateChanged = false;  // Set when I2S rate changes
    volatile bool m_playRequested = false;      // set by play(), cleared by task
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
    // Pushes resampled stereo samples to OverlayMixer for mixing with BT audio.
    // Returns the number of frames actually accepted (for drop-free pacing).
    using OverlayPushFunc = size_t(*)(const int32_t* stereoSamples, size_t frames);
    OverlayPushFunc m_overlayPushFunc = nullptr;
    
    void setOverlayPushFunc(OverlayPushFunc func) {
        m_overlayPushFunc = func;
    }

    // Overlay start/stop hooks (set by main) — drive BT-audio ducking around the
    // overlay sound so the duck holds for the whole effect, then ramps back.
    using OverlayCtrlFunc = void(*)();
    OverlayCtrlFunc m_overlayStartFunc = nullptr;
    OverlayCtrlFunc m_overlayEndFunc = nullptr;

    void setOverlayCtrlFuncs(OverlayCtrlFunc startFn, OverlayCtrlFunc endFn) {
        m_overlayStartFunc = startFn;
        m_overlayEndFunc = endFn;
    }
};
