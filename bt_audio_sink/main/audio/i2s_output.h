#pragma once

// -----------------------------------------------------------
// I2S Output - manages I2S driver for audio output (ESP-IDF v5 new driver)
// Always operates in 32-bit stereo mode
// -----------------------------------------------------------

#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/i2s_std.h"
#include "driver/i2s_common.h"
#include "esp_log.h"
#include "../config/app_config.h"

// Callback for sample rate change notifications
using SampleRateChangeCallback = void(*)(uint32_t newRate);

class I2SOutput {
public:
    I2SOutput()
        : m_initialized(false)
        , m_sampleRate(0)
        , m_reconfig(false)
        , m_usingApll(false)
        , m_mutex(nullptr)
        , m_txHandle(nullptr)
        , m_sampleRateCallback(nullptr)
    {
    }

    ~I2SOutput() {
        if (m_txHandle) {
            i2s_channel_disable(m_txHandle);
            i2s_del_channel(m_txHandle);
        }
        if (m_mutex) {
            vSemaphoreDelete(m_mutex);
        }
    }

    // Initialize I2S driver (call once at startup)
    esp_err_t init(uint32_t sampleRate) {
        if (m_initialized) return ESP_OK;

        m_mutex = xSemaphoreCreateMutex();
        if (!m_mutex) {
            ESP_LOGE(TAG, "Failed to create I2S mutex");
            return ESP_ERR_NO_MEM;
        }

        i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG((i2s_port_t)APP_I2S_PORT, I2S_ROLE_MASTER);
        chan_cfg.auto_clear = true;
        // Keep the DMA as a SMALL bridge (3x256 = 768 frames, ~8 ms @ 96k / ~17 ms
        // @ 44.1k) -- just enough to cover render scheduling latency. The jitter
        // cushion deliberately lives in the large software ring instead of here:
        // a big DMA causes the render to dump the whole ring into hardware, hiding
        // the cushion where drift control and underrun detection can't manage it.
        chan_cfg.dma_desc_num = 3;
        chan_cfg.dma_frame_num = 128;

        esp_err_t err = i2s_new_channel(&chan_cfg, &m_txHandle, NULL);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "i2s_new_channel failed: %s", esp_err_to_name(err));
            return err;
        }

        i2s_std_clk_config_t clk_cfg = makeClockConfig(sampleRate, true);
        i2s_std_config_t std_cfg = {
            .clk_cfg = clk_cfg,
            .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
            .gpio_cfg = {
                .mclk = I2S_GPIO_UNUSED,
                .bclk = (gpio_num_t)APP_I2S_BCK_PIN,
                .ws   = (gpio_num_t)APP_I2S_LRCK_PIN,
                .dout = (gpio_num_t)APP_I2S_DATA_PIN,
                .din  = I2S_GPIO_UNUSED,
                .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
            },
        };

        err = i2s_channel_init_std_mode(m_txHandle, &std_cfg);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "I2S APLL init failed: %s; falling back to default clock", esp_err_to_name(err));
            clk_cfg = makeClockConfig(sampleRate, false);
            std_cfg.clk_cfg = clk_cfg;
            err = i2s_channel_init_std_mode(m_txHandle, &std_cfg);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "i2s_channel_init_std_mode failed: %s", esp_err_to_name(err));
                i2s_del_channel(m_txHandle);
                m_txHandle = nullptr;
                return err;
            }
            m_usingApll = false;
        } else {
            m_usingApll = true;
        }

        err = i2s_channel_enable(m_txHandle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "i2s_channel_enable failed: %s", esp_err_to_name(err));
            i2s_del_channel(m_txHandle);
            m_txHandle = nullptr;
            return err;
        }

        m_initialized = true;
        m_sampleRate = sampleRate;
        ESP_LOGI(TAG, "I2S initialized (%s): sr=%u, 32-bit stereo, dma=%ux%u",
                 m_usingApll ? "APLL" : "default", (unsigned)sampleRate,
                 (unsigned)chan_cfg.dma_desc_num,
                 (unsigned)chan_cfg.dma_frame_num);
        return ESP_OK;
    }

    // Update sample rate (clock only, no driver reinstall)
    void updateClock(uint32_t sampleRate) {
        if (!m_initialized || !m_txHandle) return;
        if (sampleRate == 0) sampleRate = APP_I2S_DEFAULT_SR;
        if (m_sampleRate == sampleRate) return;

        lock();
        m_reconfig = true;

        i2s_channel_disable(m_txHandle);

        i2s_std_clk_config_t clk_cfg = makeClockConfig(sampleRate, true);
        esp_err_t err = i2s_channel_reconfig_std_clock(m_txHandle, &clk_cfg);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "I2S APLL clock update failed: %s; falling back to default clock",
                     esp_err_to_name(err));
            clk_cfg = makeClockConfig(sampleRate, false);
            err = i2s_channel_reconfig_std_clock(m_txHandle, &clk_cfg);
            if (err == ESP_OK) {
                m_usingApll = false;
            }
        } else {
            m_usingApll = true;
        }
        if (err == ESP_OK) {
            m_sampleRate = sampleRate;
            ESP_LOGI(TAG, "I2S %s clock updated: sr=%u",
                     m_usingApll ? "APLL" : "default", (unsigned)sampleRate);
            if (m_sampleRateCallback) {
                m_sampleRateCallback(sampleRate);
            }
        } else {
            ESP_LOGE(TAG, "i2s_channel_reconfig_std_clock failed: %s", esp_err_to_name(err));
        }

        i2s_channel_enable(m_txHandle);
        m_reconfig = false;
        unlock();
    }

    void setSampleRateCallback(SampleRateChangeCallback cb) {
        m_sampleRateCallback = cb;
    }

    uint32_t getSampleRate() const {
        return m_sampleRate;
    }

    // Force a same-rate clock/DMA recovery. This intentionally does what a
    // manual codec/sample-rate switch does, but without changing Bluetooth
    // codec settings. It is used after a short UI overlay on high-rate LDAC
    // streams if the flash/resampler overlay disturbed the output path.
    void forceClockRefresh() {
        if (!m_initialized || !m_txHandle) return;

        lock();
        m_reconfig = true;

        const uint32_t sr = m_sampleRate ? m_sampleRate : APP_I2S_DEFAULT_SR;
        i2s_channel_disable(m_txHandle);

        i2s_std_clk_config_t clk_cfg = makeClockConfig(sr, true);
        esp_err_t err = i2s_channel_reconfig_std_clock(m_txHandle, &clk_cfg);
        if (err != ESP_OK) {
            clk_cfg = makeClockConfig(sr, false);
            err = i2s_channel_reconfig_std_clock(m_txHandle, &clk_cfg);
            if (err == ESP_OK) m_usingApll = false;
        } else {
            m_usingApll = true;
        }

        i2s_channel_enable(m_txHandle);
        m_reconfig = false;
        unlock();

        // Clear any queued DMA samples after the re-enable.
        zeroDMA();

        if (err == ESP_OK) {
            ESP_LOGW(TAG, "I2S same-rate recovery refresh: sr=%u", (unsigned)sr);
        } else {
            ESP_LOGE(TAG, "I2S same-rate recovery failed: %s", esp_err_to_name(err));
        }
    }

    void resetToDefault() {
        updateClock(APP_I2S_DEFAULT_SR);
        zeroDMA();
    }

    size_t write(const void *data, size_t bytes) {
        if (!m_initialized || m_reconfig || !m_txHandle) return 0;

        const uint8_t* ptr = (const uint8_t*)data;
        size_t remaining = bytes;
        size_t totalWritten = 0;
        lock();
        while (remaining > 0) {
            size_t written = 0;
            esp_err_t err = i2s_channel_write(m_txHandle, ptr, remaining, &written, pdMS_TO_TICKS(50));
            if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
                break;
            }
            totalWritten += written;
            if (written == 0) break;
            ptr += written;
            remaining -= written;
        }
        unlock();
        return totalWritten;
    }

    // Zero DMA buffer by writing silence and disabling/enabling to clear
    void zeroDMA() {
        if (m_initialized && m_txHandle) {
            uint8_t silence[512];
            memset(silence, 0, sizeof(silence));
            size_t written = 0;
            i2s_channel_write(m_txHandle, silence, sizeof(silence), &written, 0);
            i2s_channel_disable(m_txHandle);
            i2s_channel_enable(m_txHandle);
        }
    }

    void stop() {
        if (m_initialized && m_txHandle) {
            i2s_channel_disable(m_txHandle);
        }
    }

    void start() {
        if (m_initialized && m_txHandle) {
            i2s_channel_enable(m_txHandle);
        }
    }

    bool isInitialized() const { return m_initialized; }
    bool isReconfiguring() const { return m_reconfig; }

    void lock() {
        if (m_mutex) xSemaphoreTake(m_mutex, portMAX_DELAY);
    }

    void unlock() {
        if (m_mutex) xSemaphoreGive(m_mutex);
    }

private:
    static constexpr const char* TAG = "I2S";

    static i2s_std_clk_config_t makeClockConfig(uint32_t sampleRate, bool useApll) {
        i2s_std_clk_config_t cfg = I2S_STD_CLK_DEFAULT_CONFIG(sampleRate);
        if (useApll) {
            cfg.clk_src = I2S_CLK_SRC_APLL;
        }
        cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;
        return cfg;
    }

    bool m_initialized;
    uint32_t m_sampleRate;
    volatile bool m_reconfig;
    bool m_usingApll;
    SemaphoreHandle_t m_mutex;
    i2s_chan_handle_t m_txHandle;
    SampleRateChangeCallback m_sampleRateCallback;
};
