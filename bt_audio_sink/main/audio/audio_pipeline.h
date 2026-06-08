#pragma once

/*
 * audio_pipeline.h
 *
 * Core split audio pipeline:
 *   Core 1 A2DP callback: decoded PCM -> Q1.31 stereo ring-buffer push only.
 *   Core 0 audio_render: ring-buffer pop -> DSP -> overlay mix -> I2S write.
 */

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "../config/app_config.h"
#include "../dsp/dsp_processor.h"
#include "i2s_output.h"
#include "overlay_mixer.h"

typedef bool (*ShouldSkipWriteCallback)();

class AudioPipeline {
public:
    AudioPipeline()
        : m_dspOut(nullptr)
        , m_ringBuf(nullptr)
        , m_ringFrameCount(0)
        , m_writeIdx(0)
        , m_readIdx(0)
        , m_writeCount(0)
        , m_dropCount(0)
        , m_catchupDrops(0)
        , m_underrunCount(0)
        , m_maxFillFrames(0)
        , m_fillAvg(0)
        , m_lastProcessMs(0)
        , m_sampleRate(APP_I2S_DEFAULT_SR)
        , m_skipWriteCallback(nullptr)
        , m_wasSkipping(false)
        , m_audioActive(false)
        , m_overlayMixer(nullptr)
        , m_secondaryOutput(nullptr)
        , m_renderTask(nullptr)
    {
    }

    ~AudioPipeline() {
        if (m_dspOut) heap_caps_free(m_dspOut);
        if (m_ringBuf) heap_caps_free(m_ringBuf);
    }

    void setOverlayMixer(OverlayMixer* mixer) {
        m_overlayMixer = mixer;
    }

    OverlayMixer* getOverlayMixer() const { return m_overlayMixer; }

    void setSecondaryOutput(I2SOutput* output) {
        m_secondaryOutput = output;
    }

    void setRenderTaskHandle(TaskHandle_t task) {
        m_renderTask = task;
    }

    bool init() {
        // +2 frames of headroom so drift compensation can append a duplicated
        // frame (ASRC insert) without overflowing the output buffer.
        const size_t dspSize = sizeof(int32_t) * (APP_DSP_OUT_FRAMES + 2) * 2;
        const size_t ringBytes = APP_AUDIO_POOL_COUNT * APP_AUDIO_POOL_BUF_SIZE;

        ESP_LOGI(TAG, "Free heap before init: internal=%u KB",
                 (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) / 1024));

        m_dspOut = (int32_t*)heap_caps_malloc(dspSize, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        if (!m_dspOut) {
            ESP_LOGW(TAG, "DMA-capable RAM dsp_out failed, trying internal 8BIT");
            m_dspOut = (int32_t*)heap_caps_malloc(dspSize, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        }
        if (!m_dspOut) {
            ESP_LOGE(TAG, "Failed to allocate DSP output buffer");
            return false;
        }

        m_ringFrameCount = ringBytes / (2 * sizeof(int32_t));
        if (m_ringFrameCount < APP_DSP_OUT_FRAMES * 2) {
            ESP_LOGE(TAG, "Audio ring too small: %u frames", (unsigned)m_ringFrameCount);
            return false;
        }

        m_ringBuf = (int32_t*)heap_caps_malloc(m_ringFrameCount * 2 * sizeof(int32_t),
                                               MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!m_ringBuf) {
            ESP_LOGE(TAG, "Failed to allocate %u-byte Q1.31 ring buffer",
                     (unsigned)(m_ringFrameCount * 2 * sizeof(int32_t)));
            return false;
        }
        memset(m_ringBuf, 0, m_ringFrameCount * 2 * sizeof(int32_t));

        ESP_LOGI(TAG, "Q1.31 ring pipeline initialized: %u frames, %u bytes",
                 (unsigned)m_ringFrameCount,
                 (unsigned)(m_ringFrameCount * 2 * sizeof(int32_t)));
        ESP_LOGI(TAG, "Free heap after init: internal=%u KB",
                 (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) / 1024));
        return true;
    }

    void setSkipWriteCallback(ShouldSkipWriteCallback cb) {
        m_skipWriteCallback = cb;
    }

    void setSampleRate(uint32_t sampleRate) {
        m_sampleRate = sampleRate ? sampleRate : APP_I2S_DEFAULT_SR;
    }

    // Core 1: producer path (runs in the A2DP decoder task context).
    void enqueueFromBt(const uint8_t *data, uint32_t len, uint8_t bits, uint8_t channels) {
        if (!m_ringBuf || len == 0) return;

        const uint32_t bytesPerSample = (bits <= 16) ? 2u : ((bits <= 24) ? 3u : 4u);
        const uint32_t ch = channels ? channels : 2u;
        const uint32_t bytesPerFrame = bytesPerSample * ch;
        if (bytesPerFrame == 0) return;

        // --- A2DP backpressure / flow control ---
        // Many phones stream LDAC FASTER than real time (they send ahead to fill
        // the sink buffer). If we just drop whatever doesn't fit, the stack thinks
        // we consumed everything and the source never throttles -> the ring stays
        // pinned full and we drop forever (constant "overruns", grainy audio).
        // Instead, once the ring reaches its setpoint we pause HERE in the decoder
        // task. That backs up Bluedroid's RxSbcQ -> L2CAP flow-controls the link ->
        // the phone stops sending ahead. This is the correct A2DP sink behaviour.
        // Bounded so a stalled/idle consumer can never wedge the BT task.
        if (m_audioActive) {
            const uint32_t watermark = m_ringFrameCount >> 1;   // == drift setpoint
            uint32_t spins = 0;
            while (getFillFrames() >= watermark && spins < kMaxBackpressureSpins) {
                if (m_renderTask) xTaskNotifyGive(m_renderTask);
                vTaskDelay(1);
                spins++;
            }
        }

        const uint8_t *ptr = data;
        uint32_t remaining = len;
        uint32_t dropped = 0;
        bool pushedAny = false;

        while (remaining >= bytesPerFrame) {
            int32_t left = loadQ31(ptr, bytesPerSample);
            int32_t right = (ch == 1) ? left : loadQ31(ptr + bytesPerSample, bytesPerSample);

            if (!pushFrame(left, right)) {
                dropped++;
            } else {
                pushedAny = true;
            }

            ptr += bytesPerFrame;
            remaining -= bytesPerFrame;
        }

        if (dropped) {
            m_dropCount += dropped;
        }
        uint32_t fill = getFillFrames();
        if (fill > m_maxFillFrames) {
            m_maxFillFrames = fill;
        }

        if (pushedAny && m_renderTask) {
            xTaskNotifyGive(m_renderTask);
        }
    }

    // Core 0 only: consumer path. Runs DSP, overlay mixing, and blocking I2S writes.
    bool renderAvailable(DSPProcessor &dsp, I2SOutput &i2s) {
        if (!m_dspOut || !m_ringBuf) return false;

        uint32_t readIdx = m_readIdx;
        const uint32_t writeIdx = m_writeIdx;
        if (readIdx == writeIdx) {
            // No BT data right now. But an overlay sound (e.g. max-volume) must
            // still play even when music is paused/not yet streaming -- otherwise
            // the overlay ring never drains, its producer stalls, and the duck
            // gets stuck. So pump a silence+overlay chunk straight to I2S, just
            // like an always-on mixer would. Returns true so the render keeps
            // servicing the overlay until it finishes.
            if (m_overlayMixer && m_overlayMixer->isActive()) {
                return renderOverlayOnly(i2s);
            }
            // Ring momentarily drained with no overlay either. Record and bail --
            // do NOT force a full re-prebuffer (that starves the small DMA bridge
            // and turns one short gap into a stutter loop).
            if (m_audioActive) m_underrunCount++;
            return false;
        }

        uint32_t available = getFillFrames();
        uint32_t cap = getCapacityFrames();

        // Standing buffer setpoint (also the prebuffer watermark and drift
        // setpoint). Centred at 1/2 of capacity so the ring has equal room to
        // absorb a delivery GAP (drain below the setpoint -> underrun reserve)
        // and a delivery BURST (fill above the setpoint -> overrun headroom).
        // BLE shares Core 1 with the A2DP decoder, so when BLE is busy the
        // decoder stalls then catches up in a burst; the headroom above the
        // setpoint is what keeps those bursts from overflowing the ring.
        uint32_t target = cap >> 1;
        if (target < APP_DSP_OUT_FRAMES) target = APP_DSP_OUT_FRAMES;

        const bool overlayActiveNow = (m_overlayMixer && m_overlayMixer->isActive());

        if (!m_audioActive) {
            if (available < target) {
                // Normal BT playback waits for prebuffer, but overlay playback
                // must be clocked immediately. Otherwise a max-volume effect can
                // start while the BT ring is partially filled and the mixer would
                // stall before draining the overlay.
                if (overlayActiveNow) {
                    return renderPartialBtPlusOverlay(dsp, i2s, available);
                }
                return false;
            }
            m_fillAvg = (int32_t)available;   // seed drift estimator on (re)start
        }

        if (available < APP_DSP_OUT_FRAMES) {
            if (overlayActiveNow) {
                // Important: when an overlay is active, the render loop must keep
                // clocking I2S even if the BT ring has only a partial chunk.
                // Otherwise the overlay ring stops draining exactly at the sound
                // finish/restore transition and the pipeline can get stuck in a
                // bad post-effect state.  Consume what BT has, pad the rest with
                // silence, and keep the overlay/duck state moving forward.
                return renderPartialBtPlusOverlay(dsp, i2s, available);
            }

            if (m_audioActive) {
                m_underrunCount++;
            }
            return false;
        }

        // --- Adaptive clock-drift / burst compensation (software ASRC) ---
        // The phone (source) and I2S/APLL (sink) run on independent clocks, and
        // because BLE shares Core 1 with the A2DP decoder the decoder delivers in
        // bursts. We steer the SMOOTHED fill toward the setpoint and DELIBERATELY
        // ignore transient bursts -- the large ring absorbs a burst and drains it
        // back down on its own, so reacting to the instantaneous level just throws
        // away audio the ring would have played fine (which starves I2S and is the
        // worst kind of stutter). Only a SUSTAINED average offset is corrected:
        //   - sustained too LOW  -> insert one duplicated frame (raise net fill)
        //   - sustained too HIGH -> drop a few frames (lower net fill)
        // The drop is proportional but tightly bounded, so a genuine rate surplus
        // is bled off gradually (spread out, inaudible) instead of dumped at once.
        m_fillAvg += ((int32_t)available - m_fillAvg) >> 4;
        const int32_t dead = (int32_t)(cap >> 3);   // tolerate +/- 1/8 cap of jitter
        const int32_t err = m_fillAvg - (int32_t)target;
        (void)err;

        // IMPORTANT STABILITY FIX:
        // Do not drop or duplicate Bluetooth frames in the render path. The old
        // software-ASRC catch-up path could be triggered by the max-volume overlay
        // transition and then keep removing/duplicating frames after the WAV ended,
        // which sounded like the LDAC stream suddenly dropped to a low bitrate.
        // A2DP backpressure on the producer side is safer here; keep output chunk
        // length stable and let the ring absorb burst timing.
        int32_t driftDrop = 0;
        int driftInsert = 0;

        if (overlayActiveNow) {
            m_fillAvg = (int32_t)available;
        }

        uint32_t frames = contiguousFrames(readIdx, writeIdx);
        if (frames > APP_DSP_OUT_FRAMES) frames = APP_DSP_OUT_FRAMES;
        if (frames == 0) return false;
        // Cap to ONE frame per chunk. Multiple adjacent catch-up drops are very
        // audible and can sound like static/tearing after an overlay transition.
        if (driftDrop > 1) driftDrop = 1;

        for (uint32_t i = 0; i < frames; i++) {
            const uint32_t idx = ((readIdx + i) % m_ringFrameCount) * 2;
            int32_t left = m_ringBuf[idx + 0];
            int32_t right = m_ringBuf[idx + 1];

            dsp.processStereoQ31(left, right);

            m_dspOut[i * 2 + 0] = left;
            m_dspOut[i * 2 + 1] = right;
        }

        // Ring is always drained by exactly `frames`; only the emitted count
        // changes so the net pipeline latency rises (insert) or falls (drop).
        m_readIdx = (readIdx + frames) % m_ringFrameCount;

        uint32_t outFrames = frames;
        if (driftInsert) {
            // Sustained deficit: duplicate the last frame so I2S emits one extra
            // frame that was not drained from the ring -> net fill rises.
            m_dspOut[frames * 2 + 0] = m_dspOut[(frames - 1) * 2 + 0];
            m_dspOut[frames * 2 + 1] = m_dspOut[(frames - 1) * 2 + 1];
            outFrames = frames + 1;
        } else if (driftDrop > 0 && (int32_t)frames - driftDrop >= 1) {
            // Sustained surplus: emit fewer frames than were drained from the ring
            // (the extra drained frames are discarded) -> net fill falls.
            outFrames = frames - (uint32_t)driftDrop;
            m_catchupDrops += (uint32_t)driftDrop;
        }

        // Only let the overlay mixer touch the BT output while it is actually
        // active (ducking, playing, or ramping back). When idle it must be a
        // complete no-op -- otherwise its per-sample duck multiply and "add ring
        // sample" branch run on every frame forever, and any stale/leftover ring
        // frame would be summed into the music as static after a sound finishes.
        if (m_overlayMixer && m_overlayMixer->isActive()) {
            m_overlayMixer->mixIntoOutput(m_dspOut, outFrames);
        }

        const bool skipWrite = m_skipWriteCallback && m_skipWriteCallback();
        if (skipWrite && !m_wasSkipping) {
            zeroOutputs(i2s);
        }
        m_wasSkipping = skipWrite;

        if (!skipWrite) {
            writeOutputs(i2s, m_dspOut, outFrames * 2u * sizeof(int32_t));
            m_writeCount++;
        }

        m_lastProcessMs = millis32();
        m_audioActive = true;
        return true;
    }

    void clear() {
        m_readIdx = 0;
        m_writeIdx = 0;
        m_audioActive = false;
        m_fillAvg = 0;
    }

    // Recovery used after an overlay sound on high-rate LDAC streams.
    // Discard the buffered BT PCM and force the I2S clock/DMA path through the
    // same kind of same-rate refresh that a manual codec switch performs.
    // This is intentionally called only after the overlay has fully drained.
    void recoverAfterOverlay(I2SOutput& i2s) {
        const uint32_t wr = m_writeIdx;
        m_readIdx = wr;
        m_audioActive = false;
        m_fillAvg = 0;
        m_underrunCount = 0;
        m_catchupDrops = 0;
        i2s.forceClockRefresh();
        if (m_secondaryOutput && m_secondaryOutput->isInitialized()) {
            m_secondaryOutput->forceClockRefresh();
        }
    }

    void clearWithDMA(I2SOutput& i2s) {
        clear();
        zeroOutputs(i2s);
    }

    uint32_t getLastProcessMs() const { return m_lastProcessMs; }
    uint32_t getWriteCount() const { return m_writeCount; }
    uint32_t getDropCount() const { return m_dropCount; }
    uint32_t getCatchupDrops() const { return m_catchupDrops; }
    uint32_t getUnderrunCount() const { return m_underrunCount; }
    uint32_t getAndResetMaxFillFrames() {
        uint32_t v = m_maxFillFrames;
        m_maxFillFrames = getFillFrames();
        return v;
    }
    uint32_t getFillFrames() const {
        uint32_t writeIdx = m_writeIdx;
        uint32_t readIdx = m_readIdx;
        return (writeIdx >= readIdx) ? (writeIdx - readIdx) : (m_ringFrameCount - readIdx + writeIdx);
    }
    uint32_t getCapacityFrames() const { return m_ringFrameCount ? (m_ringFrameCount - 1) : 0; }
    bool isAudioActive() const { return m_audioActive; }

private:
    static constexpr const char* TAG = "AudioPipe";
    // Max 1 ms ticks the producer will pause for ring space before giving up and
    // dropping (safety valve so a dead/paused consumer can't wedge the BT task).
    static constexpr uint32_t kMaxBackpressureSpins = 30;

    static uint32_t millis32() {
        return (uint32_t)(esp_timer_get_time() / 1000ULL);
    }

    uint32_t framesForMs(uint32_t ms) const {
        uint32_t rate = m_sampleRate ? m_sampleRate : APP_I2S_DEFAULT_SR;
        return (uint32_t)(((uint64_t)rate * ms + 999u) / 1000u);
    }

    static int32_t IRAM_ATTR loadQ31(const uint8_t *p, uint32_t bytesPerSample) {
        if (bytesPerSample == 2) {
            int16_t v = (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
            return ((int32_t)v) << 16;
        }
        if (bytesPerSample == 3) {
            int32_t v = ((int32_t)p[0]) | ((int32_t)p[1] << 8) | ((int32_t)p[2] << 16);
            v = (v << 8) >> 8;
            return v << 8;
        }
        uint32_t v = ((uint32_t)p[0]) | ((uint32_t)p[1] << 8) |
                     ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
        return (int32_t)v;
    }

    bool IRAM_ATTR pushFrame(int32_t left, int32_t right) {
        const uint32_t nextWrite = (m_writeIdx + 1) % m_ringFrameCount;
        if (nextWrite == m_readIdx) {
            return false;
        }

        const uint32_t idx = m_writeIdx * 2;
        m_ringBuf[idx + 0] = left;
        m_ringBuf[idx + 1] = right;
        m_writeIdx = nextWrite;
        return true;
    }

    void zeroOutputs(I2SOutput& primary) {
        primary.zeroDMA();
        if (m_secondaryOutput && m_secondaryOutput->isInitialized()) {
            m_secondaryOutput->zeroDMA();
        }
    }

    void writeOutputs(I2SOutput& primary, const void* data, size_t bytes) {
        primary.write(data, bytes);
        if (m_secondaryOutput && m_secondaryOutput->isInitialized()) {
            m_secondaryOutput->write(data, bytes);
        }
    }

    // Emit one chunk of silence with the overlay mixed on top. Used when an
    // overlay sound is active but no BT audio is flowing, so the effect still
    // plays (and its ring keeps draining) instead of stalling its producer.
    bool renderOverlayOnly(I2SOutput &i2s) {
        const uint32_t frames = APP_DSP_OUT_FRAMES;
        memset(m_dspOut, 0, frames * 2u * sizeof(int32_t));
        m_overlayMixer->mixIntoOutput(m_dspOut, frames);

        const bool skipWrite = m_skipWriteCallback && m_skipWriteCallback();
        if (skipWrite && !m_wasSkipping) {
            zeroOutputs(i2s);
        }
        m_wasSkipping = skipWrite;

        if (!skipWrite) {
            writeOutputs(i2s, m_dspOut, frames * 2u * sizeof(int32_t));
            m_writeCount++;
        }
        m_fillAvg = (int32_t)getFillFrames();
        m_lastProcessMs = millis32();
        m_audioActive = true;
        return true;
    }



    // Emit one full I2S chunk while BT has only a partial chunk available.
    // This is only used while overlay/ducking is active. It prevents the render
    // task from stopping at the exact moment the WAV producer needs the overlay
    // ring to keep draining.
    bool renderPartialBtPlusOverlay(DSPProcessor &dsp, I2SOutput &i2s, uint32_t available) {
        const uint32_t framesToRead = (available > APP_DSP_OUT_FRAMES)
                                      ? APP_DSP_OUT_FRAMES
                                      : available;
        uint32_t readIdx = m_readIdx;

        for (uint32_t i = 0; i < framesToRead; i++) {
            const uint32_t idx = ((readIdx + i) % m_ringFrameCount) * 2;
            int32_t left = m_ringBuf[idx + 0];
            int32_t right = m_ringBuf[idx + 1];

            dsp.processStereoQ31(left, right);

            m_dspOut[i * 2 + 0] = left;
            m_dspOut[i * 2 + 1] = right;
        }

        for (uint32_t i = framesToRead; i < APP_DSP_OUT_FRAMES; i++) {
            m_dspOut[i * 2 + 0] = 0;
            m_dspOut[i * 2 + 1] = 0;
        }

        if (framesToRead > 0) {
            m_readIdx = (readIdx + framesToRead) % m_ringFrameCount;
        }

        m_overlayMixer->mixIntoOutput(m_dspOut, APP_DSP_OUT_FRAMES);

        const bool skipWrite = m_skipWriteCallback && m_skipWriteCallback();
        if (skipWrite && !m_wasSkipping) {
            zeroOutputs(i2s);
        }
        m_wasSkipping = skipWrite;

        if (!skipWrite) {
            writeOutputs(i2s, m_dspOut, APP_DSP_OUT_FRAMES * 2u * sizeof(int32_t));
            m_writeCount++;
        }

        m_fillAvg = (int32_t)getFillFrames();
        m_lastProcessMs = millis32();
        m_audioActive = true;
        return true;
    }

    uint32_t contiguousFrames(uint32_t readIdx, uint32_t writeIdx) const {
        if (writeIdx > readIdx) {
            return writeIdx - readIdx;
        }
        return m_ringFrameCount - readIdx;
    }

    int32_t *m_dspOut;
    int32_t *m_ringBuf;
    uint32_t m_ringFrameCount;

    volatile uint32_t m_writeIdx;
    volatile uint32_t m_readIdx;
    volatile uint32_t m_writeCount;
    volatile uint32_t m_dropCount;
    volatile uint32_t m_catchupDrops;
    volatile uint32_t m_underrunCount;
    volatile uint32_t m_maxFillFrames;
    int32_t m_fillAvg;            // smoothed ring fill (frames) for drift control
    volatile uint32_t m_lastProcessMs;
    volatile uint32_t m_sampleRate;

    ShouldSkipWriteCallback m_skipWriteCallback;
    bool m_wasSkipping;
    volatile bool m_audioActive;

    OverlayMixer* m_overlayMixer;
    I2SOutput* m_secondaryOutput;
    TaskHandle_t m_renderTask;
};
