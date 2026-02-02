#pragma once

// -----------------------------------------------------------
// WS2812B LED Driver using SPI DMA
// Alternative to RMT driver - more reliable under heavy CPU load
// Based on: https://github.com/okhsunrog/esp_ws28xx
// -----------------------------------------------------------

#include <stdint.h>
#include <string.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "led_config.h"

static const char* TAG_SPI = "LED_SPI";

// RGB structure
struct RGB_SPI {
    uint8_t r, g, b;
    
    RGB_SPI() : r(0), g(0), b(0) {}
    RGB_SPI(uint8_t _r, uint8_t _g, uint8_t _b) : r(_r), g(_g), b(_b) {}
    
    // Scale brightness (no division)
    RGB_SPI scale(uint8_t brightness) const {
        return RGB_SPI(
            (r * brightness + 128) >> 8,
            (g * brightness + 128) >> 8,
            (b * brightness + 128) >> 8
        );
    }
    
    // HSV to RGB conversion (no division)
    static RGB_SPI fromHSV(uint8_t h, uint8_t s, uint8_t v) {
        if (s == 0) return RGB_SPI(v, v, v);
        
        uint8_t region = (h * 6) >> 8;  // h/43 approx
        uint8_t remainder = (h * 6) - (region << 8);
        
        uint8_t p = (v * (255 - s) + 128) >> 8;
        uint8_t q = (v * (255 - ((s * remainder + 128) >> 8)) + 128) >> 8;
        uint8_t t = (v * (255 - ((s * (255 - remainder) + 128) >> 8)) + 128) >> 8;
        
        switch (region) {
            case 0:  return RGB_SPI(v, t, p);
            case 1:  return RGB_SPI(q, v, p);
            case 2:  return RGB_SPI(p, v, t);
            case 3:  return RGB_SPI(p, q, v);
            case 4:  return RGB_SPI(t, p, v);
            default: return RGB_SPI(v, p, q);
        }
    }
    
    RGB_SPI add(const RGB_SPI& other) const {
        return RGB_SPI(
            (r + other.r > 255) ? 255 : r + other.r,
            (g + other.g > 255) ? 255 : g + other.g,
            (b + other.b > 255) ? 255 : b + other.b
        );
    }
};

// Timing lookup table: 4 bits -> 16-bit SPI pattern
// At 3.2MHz: each bit = 312.5ns
// WS2812B: T0H=400ns (1 bit high), T0L=850ns (2-3 bits low)
//          T1H=800ns (2-3 bits high), T1L=450ns (1 bit low)
// Pattern: 0b0001 = 0 bit (short high), 0b0111 = 1 bit (long high)
// 4 bits packed per 16-bit word (LSB first for SPI)
static const uint16_t WS2812_TIMING_TABLE[16] = {
    0x1111,  // 0b0000 -> all zeros
    0x7111,  // 0b0001
    0x1711,  // 0b0010
    0x7711,  // 0b0011
    0x1171,  // 0b0100
    0x7171,  // 0b0101
    0x1771,  // 0b0110
    0x7771,  // 0b0111
    0x1117,  // 0b1000
    0x7117,  // 0b1001
    0x1717,  // 0b1010
    0x7717,  // 0b1011
    0x1177,  // 0b1100
    0x7177,  // 0b1101
    0x1777,  // 0b1110
    0x7777   // 0b1111 -> all ones
};

class LedDriverSPI {
public:
    LedDriverSPI() : m_brightness(LED_DEFAULT_BRIGHTNESS), m_spi(nullptr),
                     m_dmaBuffer(nullptr), m_initialized(false) {
        m_txMutex = xSemaphoreCreateMutex();
    }
    
    ~LedDriverSPI() {
        if (m_spi) {
            spi_bus_remove_device(m_spi);
        }
        if (m_dmaBuffer) {
            heap_caps_free(m_dmaBuffer);
        }
        if (m_txMutex) {
            vSemaphoreDelete(m_txMutex);
        }
    }
    
    esp_err_t init(gpio_num_t pin = LED_GPIO_PIN) {
        // Calculate DMA buffer size:
        // Each LED = 3 bytes = 24 bits
        // Each bit = 4 SPI bits (encoded)
        // Each byte = 8 bits * 4 = 32 SPI bits = 2 uint16_t
        // Each LED = 6 uint16_t = 12 bytes
        // Plus reset pulse (50us at 3.2MHz = ~160 bits = ~10 uint16_t)
        m_dmaBufferSize = (LED_MATRIX_COUNT * 12) + 20;  // 12 bytes per LED + reset
        
        ESP_LOGI(TAG_SPI, "Initializing SPI LED driver: %d LEDs, GPIO %d", LED_MATRIX_COUNT, pin);
        ESP_LOGI(TAG_SPI, "DMA buffer size: %d bytes", m_dmaBufferSize);
        
        // Allocate DMA-capable buffer
        m_dmaBuffer = (uint16_t*)heap_caps_malloc(m_dmaBufferSize, MALLOC_CAP_DMA);
        if (!m_dmaBuffer) {
            ESP_LOGE(TAG_SPI, "Failed to allocate DMA buffer");
            return ESP_ERR_NO_MEM;
        }
        memset(m_dmaBuffer, 0, m_dmaBufferSize);
        
        // Configure SPI bus
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = pin;
        buscfg.miso_io_num = -1;
        buscfg.sclk_io_num = -1;
        buscfg.quadwp_io_num = -1;
        buscfg.quadhd_io_num = -1;
        buscfg.max_transfer_sz = (int)m_dmaBufferSize;
        buscfg.flags = 0;
        buscfg.intr_flags = 0;
        
        esp_err_t err = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
        if (err != ESP_OK) {
            ESP_LOGE(TAG_SPI, "SPI bus init failed: %s", esp_err_to_name(err));
            heap_caps_free(m_dmaBuffer);
            m_dmaBuffer = nullptr;
            return err;
        }
        
        // Configure SPI device
        spi_device_interface_config_t devcfg = {};
        devcfg.mode = 0;
        devcfg.clock_speed_hz = 3200000;  // 3.2 MHz
        devcfg.spics_io_num = -1;
        devcfg.flags = SPI_DEVICE_TXBIT_LSBFIRST;
        devcfg.queue_size = 1;
        
        err = spi_bus_add_device(SPI2_HOST, &devcfg, &m_spi);
        if (err != ESP_OK) {
            ESP_LOGE(TAG_SPI, "SPI device add failed: %s", esp_err_to_name(err));
            spi_bus_free(SPI2_HOST);
            heap_caps_free(m_dmaBuffer);
            m_dmaBuffer = nullptr;
            return err;
        }
        
        // Clear all LEDs
        clear();
        show();
        
        m_initialized = true;
        ESP_LOGI(TAG_SPI, "SPI LED driver initialized successfully");
        return ESP_OK;
    }
    
    void clear() {
        for (int i = 0; i < LED_MATRIX_COUNT; i++) {
            m_framebuffer[i] = RGB_SPI();
        }
    }
    
    void fill(RGB_SPI color) {
        for (int i = 0; i < LED_MATRIX_COUNT; i++) {
            m_framebuffer[i] = color;
        }
    }
    
    RGB_SPI getPixel(int index) {
        if (index >= 0 && index < LED_MATRIX_COUNT) {
            return m_framebuffer[index];
        }
        return RGB_SPI();
    }
    
    void setPixel(int index, RGB_SPI color) {
        if (index >= 0 && index < LED_MATRIX_COUNT) {
            m_framebuffer[index] = color;
        }
    }
    
    void setPixelXY(int x, int y, RGB_SPI color) {
        if (x < 0 || x >= LED_MATRIX_WIDTH || y < 0 || y >= LED_MATRIX_HEIGHT) return;
        
        int index;
        if (y & 1) {
            index = y * LED_MATRIX_WIDTH + (LED_MATRIX_WIDTH - 1 - x);
        } else {
            index = y * LED_MATRIX_WIDTH + x;
        }
        m_framebuffer[index] = color;
    }
    
    RGB_SPI getPixelXY(int x, int y) {
        if (x < 0 || x >= LED_MATRIX_WIDTH || y < 0 || y >= LED_MATRIX_HEIGHT) {
            return RGB_SPI();
        }
        int index;
        if (y & 1) {
            index = y * LED_MATRIX_WIDTH + (LED_MATRIX_WIDTH - 1 - x);
        } else {
            index = y * LED_MATRIX_WIDTH + x;
        }
        return m_framebuffer[index];
    }
    
    void setBrightness(uint8_t brightness) {
        m_brightness = brightness;
    }
    
    uint8_t getBrightness() const {
        return m_brightness;
    }
    
    void fadeAll(uint8_t scale) {
        for (int i = 0; i < LED_MATRIX_COUNT; i++) {
            m_framebuffer[i].r = (m_framebuffer[i].r * scale + 128) >> 8;
            m_framebuffer[i].g = (m_framebuffer[i].g * scale + 128) >> 8;
            m_framebuffer[i].b = (m_framebuffer[i].b * scale + 128) >> 8;
        }
    }
    
    esp_err_t show() {
        if (!m_initialized || !m_dmaBuffer) {
            return ESP_ERR_INVALID_STATE;
        }
        
        // Try to acquire mutex (skip frame if busy)
        if (xSemaphoreTake(m_txMutex, pdMS_TO_TICKS(5)) != pdTRUE) {
            return ESP_ERR_TIMEOUT;
        }
        
        // Encode framebuffer into DMA buffer
        int n = 0;
        
        // Initial zero byte
        m_dmaBuffer[n++] = 0;
        
        for (int i = 0; i < LED_MATRIX_COUNT; i++) {
            RGB_SPI pixel = m_framebuffer[i].scale(m_brightness);
            
            // WS2812B is GRB order
            uint8_t g = pixel.g;
            uint8_t r = pixel.r;
            uint8_t b = pixel.b;
            
            // Encode G (8 bits -> 2 uint16_t)
            m_dmaBuffer[n++] = WS2812_TIMING_TABLE[(g >> 4) & 0x0F];
            m_dmaBuffer[n++] = WS2812_TIMING_TABLE[g & 0x0F];
            
            // Encode R
            m_dmaBuffer[n++] = WS2812_TIMING_TABLE[(r >> 4) & 0x0F];
            m_dmaBuffer[n++] = WS2812_TIMING_TABLE[r & 0x0F];
            
            // Encode B
            m_dmaBuffer[n++] = WS2812_TIMING_TABLE[(b >> 4) & 0x0F];
            m_dmaBuffer[n++] = WS2812_TIMING_TABLE[b & 0x0F];
        }
        
        // Reset pulse (zeros for >50us)
        for (int i = 0; i < 5; i++) {
            m_dmaBuffer[n++] = 0;
        }
        
        // Transmit via SPI DMA
        spi_transaction_t trans = {
            .flags = 0,
            .cmd = 0,
            .addr = 0,
            .length = (size_t)(n * 16),  // bits
            .rxlength = 0,
            .user = nullptr,
            .tx_buffer = m_dmaBuffer,
            .rx_buffer = nullptr
        };
        
        esp_err_t err = spi_device_transmit(m_spi, &trans);
        
        xSemaphoreGive(m_txMutex);
        return err;
    }
    
    bool isInitialized() const { return m_initialized; }
    
private:
    RGB_SPI m_framebuffer[LED_MATRIX_COUNT];
    uint8_t m_brightness;
    spi_device_handle_t m_spi;
    uint16_t* m_dmaBuffer;
    size_t m_dmaBufferSize;
    bool m_initialized;
    SemaphoreHandle_t m_txMutex;
};

// Common colors for SPI driver
namespace ColorsSPI {
    static const RGB_SPI Black(0, 0, 0);
    static const RGB_SPI White(255, 255, 255);
    static const RGB_SPI Red(255, 0, 0);
    static const RGB_SPI Green(0, 255, 0);
    static const RGB_SPI Blue(0, 0, 255);
    static const RGB_SPI Yellow(255, 255, 0);
    static const RGB_SPI Cyan(0, 255, 255);
    static const RGB_SPI Magenta(255, 0, 255);
    static const RGB_SPI Orange(255, 128, 0);
    static const RGB_SPI Purple(128, 0, 255);
}
