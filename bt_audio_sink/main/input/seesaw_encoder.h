#pragma once

/**
 * @file seesaw_encoder.h
 * 
 * ESP-IDF port of Adafruit Seesaw library for Quad Rotary Encoder Breakout
 * Based on Adafruit_seesaw.h / Adafruit_seesaw.cpp
 * 
 * This driver uses the SAME I2C communication pattern as Arduino:
 *   1. Write [regHigh, regLow] with STOP
 *   2. Delay (default 250µs)
 *   3. Read response with STOP
 * 
 * Product: Adafruit Quad Rotary Encoder Breakout (Product 5752)
 * Default I2C Address: 0x36 (configurable via EEPROM)
 */

#include <stdint.h>
#include <string.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_rom_sys.h"  // For esp_rom_delay_us

// ============================================================================
// Seesaw Module Base Addresses (from Adafruit_seesaw.h)
// ============================================================================
enum {
    SEESAW_STATUS_BASE      = 0x00,
    SEESAW_GPIO_BASE        = 0x01,
    SEESAW_SERCOM0_BASE     = 0x02,
    SEESAW_TIMER_BASE       = 0x08,
    SEESAW_ADC_BASE         = 0x09,
    SEESAW_DAC_BASE         = 0x0A,
    SEESAW_INTERRUPT_BASE   = 0x0B,
    SEESAW_DAP_BASE         = 0x0C,
    SEESAW_EEPROM_BASE      = 0x0D,
    SEESAW_NEOPIXEL_BASE    = 0x0E,
    SEESAW_TOUCH_BASE       = 0x0F,
    SEESAW_KEYPAD_BASE      = 0x10,
    SEESAW_ENCODER_BASE     = 0x11,
    SEESAW_SPECTRUM_BASE    = 0x12,
};

// ============================================================================
// GPIO Function Registers
// ============================================================================
enum {
    SEESAW_GPIO_DIRSET_BULK = 0x02,
    SEESAW_GPIO_DIRCLR_BULK = 0x03,
    SEESAW_GPIO_BULK        = 0x04,
    SEESAW_GPIO_BULK_SET    = 0x05,
    SEESAW_GPIO_BULK_CLR    = 0x06,
    SEESAW_GPIO_BULK_TOGGLE = 0x07,
    SEESAW_GPIO_INTENSET    = 0x08,
    SEESAW_GPIO_INTENCLR    = 0x09,
    SEESAW_GPIO_INTFLAG     = 0x0A,
    SEESAW_GPIO_PULLENSET   = 0x0B,
    SEESAW_GPIO_PULLENCLR   = 0x0C,
};

// ============================================================================
// Status Function Registers
// ============================================================================
enum {
    SEESAW_STATUS_HW_ID     = 0x01,
    SEESAW_STATUS_VERSION   = 0x02,
    SEESAW_STATUS_OPTIONS   = 0x03,
    SEESAW_STATUS_TEMP      = 0x04,
    SEESAW_STATUS_SWRST     = 0x7F,
};

// ============================================================================
// Encoder Function Registers
// ============================================================================
enum {
    SEESAW_ENCODER_STATUS   = 0x00,
    SEESAW_ENCODER_INTENSET = 0x10,
    SEESAW_ENCODER_INTENCLR = 0x20,
    SEESAW_ENCODER_POSITION = 0x30,
    SEESAW_ENCODER_DELTA    = 0x40,
};

// ============================================================================
// NeoPixel Function Registers
// ============================================================================
enum {
    SEESAW_NEOPIXEL_STATUS      = 0x00,
    SEESAW_NEOPIXEL_PIN         = 0x01,
    SEESAW_NEOPIXEL_SPEED       = 0x02,
    SEESAW_NEOPIXEL_BUF_LENGTH  = 0x03,
    SEESAW_NEOPIXEL_BUF         = 0x04,
    SEESAW_NEOPIXEL_SHOW        = 0x05,
};

// ============================================================================
// Hardware ID Codes
// ============================================================================
#define SEESAW_HW_ID_CODE_SAMD09    0x55
#define SEESAW_HW_ID_CODE_TINY806   0x84
#define SEESAW_HW_ID_CODE_TINY807   0x85
#define SEESAW_HW_ID_CODE_TINY816   0x86
#define SEESAW_HW_ID_CODE_TINY817   0x87
#define SEESAW_HW_ID_CODE_TINY1616  0x88
#define SEESAW_HW_ID_CODE_TINY1617  0x89

// ============================================================================
// Quad Rotary Encoder Breakout (Product 5752) Specifics
// ============================================================================
#define SEESAW_QUAD_DEFAULT_ADDR    0x36    // Default I2C address (datasheet)
#define SEESAW_QUAD_ALT_ADDR        0x07    // Alternate address some boards use
#define SEESAW_QUAD_PRODUCT_ID      5752

// Button GPIO pins for each encoder (from Adafruit example code)
static const int SEESAW_QUAD_BTN_PINS[4] = {12, 14, 17, 9};

// NeoPixel pin on the Quad Encoder board
#define SEESAW_QUAD_NEOPIXEL_PIN    18

// Default I2C read delay (microseconds) - matches Arduino default
#define SEESAW_DEFAULT_DELAY_US     250
// Delay for encoder position reads (reduced to minimize audio interference)
#define SEESAW_ENCODER_DELAY_US     500

static const char* SEESAW_TAG = "SeesawQuad";

/**
 * @brief SeesawQuadEncoder - ESP-IDF driver for Adafruit Quad Rotary Encoder
 * 
 * This class implements the same I2C protocol as the Arduino Adafruit_seesaw library:
 * - Separate write and read transactions (write-delay-read pattern)
 * - Default 250µs delay between write and read
 * - Big-endian data format for multi-byte values
 */
class SeesawQuadEncoder {
public:
    SeesawQuadEncoder() = default;
    ~SeesawQuadEncoder() = default;
    
    /**
     * @brief Initialize the Seesaw device
     * 
     * @param i2cPort I2C port number (I2C_NUM_0 or I2C_NUM_1)
     * @param addr I2C address of the device
     * @param reset Whether to perform software reset (default: true)
     * @return true if initialization successful
     */
    bool begin(i2c_port_t i2cPort, uint8_t addr = SEESAW_QUAD_DEFAULT_ADDR, bool reset = true) {
        m_i2cPort = i2cPort;
        m_addr = addr;
        
        // Reset stored positions
        for (int i = 0; i < 4; i++) {
            m_positions[i] = 0;
            m_lastPositions[i] = 0;
            m_changed[i] = false;
        }
        
        // Try to detect the device
        bool found = false;
        for (int retries = 0; retries < 10; retries++) {
            if (i2cDetect()) {
                found = true;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        
        if (!found) {
            ESP_LOGW(SEESAW_TAG, "Device at 0x%02X not found", m_addr);
            return false;
        }
        
        ESP_LOGI(SEESAW_TAG, "Device detected at 0x%02X", m_addr);
        
        // Perform software reset if requested
        if (reset) {
            SWReset();
            
            // Wait for device to come back
            found = false;
            for (int retries = 0; retries < 10; retries++) {
                vTaskDelay(pdMS_TO_TICKS(10));
                if (i2cDetect()) {
                    found = true;
                    break;
                }
            }
            
            if (!found) {
                ESP_LOGW(SEESAW_TAG, "Device not responding after reset");
                return false;
            }
            
            ESP_LOGI(SEESAW_TAG, "Device reset complete");
        }
        
        // Read and verify hardware ID
        found = false;
        for (int retries = 0; retries < 10; retries++) {
            uint8_t hwId = 0;
            if (read(SEESAW_STATUS_BASE, SEESAW_STATUS_HW_ID, &hwId, 1) == ESP_OK) {
                if (hwId == SEESAW_HW_ID_CODE_SAMD09 ||
                    hwId == SEESAW_HW_ID_CODE_TINY806 ||
                    hwId == SEESAW_HW_ID_CODE_TINY807 ||
                    hwId == SEESAW_HW_ID_CODE_TINY816 ||
                    hwId == SEESAW_HW_ID_CODE_TINY817 ||
                    hwId == SEESAW_HW_ID_CODE_TINY1616 ||
                    hwId == SEESAW_HW_ID_CODE_TINY1617) {
                    m_hardwareType = hwId;
                    found = true;
                    ESP_LOGI(SEESAW_TAG, "Hardware ID: 0x%02X", hwId);
                    break;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        
        if (!found) {
            ESP_LOGW(SEESAW_TAG, "Invalid or unknown hardware ID");
            // Continue anyway - might still work
        }
        
        // Read version/product info
        uint32_t version = getVersion();
        uint16_t product = (version >> 16) & 0xFFFF;
        ESP_LOGI(SEESAW_TAG, "Version: 0x%08lX, Product ID: %u", (unsigned long)version, product);
        
        if (product != SEESAW_QUAD_PRODUCT_ID) {
            ESP_LOGW(SEESAW_TAG, "Unexpected product ID (expected %u)", SEESAW_QUAD_PRODUCT_ID);
        }
        
        // Initialize button pullups
        initButtonPullups();
        
        // Read initial encoder positions
        for (int e = 0; e < 4; e++) {
            m_positions[e] = getEncoderPosition(e);
            m_lastPositions[e] = m_positions[e];
            ESP_LOGI(SEESAW_TAG, "Encoder %d initial position: %ld", e, (long)m_positions[e]);
        }
        
        // Initialize NeoPixels
        initNeoPixels();
        
        m_initialized = true;
        ESP_LOGI(SEESAW_TAG, "Quad Rotary Encoder initialized successfully");
        return true;
    }
    
    /**
     * @brief Perform software reset
     * @return true on success
     */
    bool SWReset() {
        return write8(SEESAW_STATUS_BASE, SEESAW_STATUS_SWRST, 0xFF);
    }
    
    /**
     * @brief Get firmware version
     * @return Version code: bits [31:16] = product ID, [15:0] = date code
     */
    uint32_t getVersion() {
        uint8_t buf[4];
        if (read(SEESAW_STATUS_BASE, SEESAW_STATUS_VERSION, buf, 4) != ESP_OK) {
            return 0;
        }
        return beToU32(buf);
    }
    
    /**
     * @brief Get available options/modules
     * @return Bitmask of available modules
     */
    uint32_t getOptions() {
        uint8_t buf[4];
        if (read(SEESAW_STATUS_BASE, SEESAW_STATUS_OPTIONS, buf, 4) != ESP_OK) {
            return 0;
        }
        return beToU32(buf);
    }
    
    // ========================================================================
    // Encoder Functions (matches Arduino API)
    // ========================================================================
    
    /**
     * @brief Get encoder position
     * @param encoder Encoder number (0-3)
     * @return Current position as signed 32-bit integer
     */
    int32_t getEncoderPosition(uint8_t encoder = 0) {
        if (encoder >= 4) return 0;
        
        uint8_t buf[4];
        // Use longer delay for encoder reads - they need more time to prepare data
        if (read(SEESAW_ENCODER_BASE, SEESAW_ENCODER_POSITION + encoder, buf, 4, SEESAW_ENCODER_DELAY_US) != ESP_OK) {
            return 0;
        }
        return (int32_t)beToU32(buf);
    }
    
    /**
     * @brief Set encoder position
     * @param pos Position value to set
     * @param encoder Encoder number (0-3)
     */
    void setEncoderPosition(int32_t pos, uint8_t encoder = 0) {
        if (encoder >= 4) return;
        
        uint8_t buf[4];
        u32ToBe((uint32_t)pos, buf);
        write(SEESAW_ENCODER_BASE, SEESAW_ENCODER_POSITION + encoder, buf, 4);
    }
    
    /**
     * @brief Get encoder delta (change since last read)
     * @param encoder Encoder number (0-3)
     * @return Delta as signed 32-bit integer
     */
    int32_t getEncoderDelta(uint8_t encoder = 0) {
        if (encoder >= 4) return 0;
        
        uint8_t buf[4];
        // Use longer delay for encoder reads - they need more time to prepare data
        if (read(SEESAW_ENCODER_BASE, SEESAW_ENCODER_DELTA + encoder, buf, 4, SEESAW_ENCODER_DELAY_US) != ESP_OK) {
            return 0;
        }
        return (int32_t)beToU32(buf);
    }
    
    /**
     * @brief Enable encoder interrupt
     * @param encoder Encoder number (0-3)
     * @return true on success
     */
    bool enableEncoderInterrupt(uint8_t encoder = 0) {
        if (encoder >= 4) return false;
        return write8(SEESAW_ENCODER_BASE, SEESAW_ENCODER_INTENSET + encoder, 0x01);
    }
    
    /**
     * @brief Disable encoder interrupt
     * @param encoder Encoder number (0-3)
     * @return true on success
     */
    bool disableEncoderInterrupt(uint8_t encoder = 0) {
        if (encoder >= 4) return false;
        return write8(SEESAW_ENCODER_BASE, SEESAW_ENCODER_INTENCLR + encoder, 0x01);
    }
    
    // ========================================================================
    // High-level polling interface
    // ========================================================================
    
    /**
     * @brief Poll all encoders for position changes
     * Call this regularly in your main loop.
     * Uses delta-based reading which is more reliable and prevents wild jumps.
     */
    void poll() {
        if (!m_initialized) return;
        
        for (int e = 0; e < 4; e++) {
            // Use delta reading instead of absolute position for reliability
            int32_t delta = getEncoderDelta(e);
            
            // Validate delta - reject wild jumps (more than ±30 in one poll)
            // Normal encoder movement should be small
            if (delta != 0 && delta >= -20 && delta <= 20) {
                m_changed[e] = true;
                m_positions[e] += delta;
            } else if (delta != 0) {
                // Reject corrupt read - don't update position
                m_changed[e] = false;
            } else {
                m_changed[e] = false;
            }
        }
    }
    
    /**
     * @brief Check if encoder changed during last poll
     * @param encoder Encoder number (0-3)
     * @return true if position changed
     */
    bool hasChanged(uint8_t encoder) const {
        return (encoder < 4) ? m_changed[encoder] : false;
    }
    
    /**
     * @brief Get cached position (from last poll)
     * @param encoder Encoder number (0-3)
     * @return Cached position value
     */
    int32_t getPosition(uint8_t encoder) const {
        return (encoder < 4) ? m_positions[encoder] : 0;
    }
    
    // ========================================================================
    // GPIO / Button Functions
    // ========================================================================
    
    /**
     * @brief Check if encoder button is pressed
     * @param encoder Encoder number (0-3)
     * @return true if button is pressed (active low)
     */
    bool isButtonPressed(uint8_t encoder) {
        if (encoder >= 4) return false;
        
        uint32_t gpio = digitalReadBulk(0xFFFFFFFF);
        
        // Active low: bit=0 means pressed
        return ((gpio >> SEESAW_QUAD_BTN_PINS[encoder]) & 0x1) == 0;
    }
    
    /**
     * @brief Read GPIO pins in bulk
     * @param pins Bitmask of pins to read
     * @return GPIO values masked by pins (0xFFFFFFFF if read fails = no buttons pressed)
     */
    uint32_t digitalReadBulk(uint32_t pins) {
        uint8_t buf[4];
        if (read(SEESAW_GPIO_BASE, SEESAW_GPIO_BULK, buf, 4) != ESP_OK) {
            // Return all bits high = all buttons unpressed (active low)
            return 0xFFFFFFFF;
        }
        return beToU32(buf) & pins;
    }
    
    /**
     * @brief Set GPIO pin mode for multiple pins
     * @param pins Bitmask of pins
     * @param mode Pin mode (INPUT=0, OUTPUT=1, INPUT_PULLUP=2, INPUT_PULLDOWN=3)
     */
    void pinModeBulk(uint32_t pins, uint8_t mode) {
        uint8_t cmd[4];
        u32ToBe(pins, cmd);
        
        switch (mode) {
            case 1: // OUTPUT
                write(SEESAW_GPIO_BASE, SEESAW_GPIO_DIRSET_BULK, cmd, 4);
                break;
            case 0: // INPUT
                write(SEESAW_GPIO_BASE, SEESAW_GPIO_DIRCLR_BULK, cmd, 4);
                break;
            case 2: // INPUT_PULLUP
                write(SEESAW_GPIO_BASE, SEESAW_GPIO_DIRCLR_BULK, cmd, 4);
                write(SEESAW_GPIO_BASE, SEESAW_GPIO_PULLENSET, cmd, 4);
                write(SEESAW_GPIO_BASE, SEESAW_GPIO_BULK_SET, cmd, 4);
                break;
            case 3: // INPUT_PULLDOWN
                write(SEESAW_GPIO_BASE, SEESAW_GPIO_DIRCLR_BULK, cmd, 4);
                write(SEESAW_GPIO_BASE, SEESAW_GPIO_PULLENSET, cmd, 4);
                write(SEESAW_GPIO_BASE, SEESAW_GPIO_BULK_CLR, cmd, 4);
                break;
        }
    }
    
    // ========================================================================
    // NeoPixel Functions
    // ========================================================================
    
    /**
     * @brief Set NeoPixel color for an encoder
     * @param encoder Encoder number (0-3)
     * @param r Red value (0-255)
     * @param g Green value (0-255)
     * @param b Blue value (0-255)
     */
    void setPixelColor(uint8_t encoder, uint8_t r, uint8_t g, uint8_t b) {
        if (encoder >= 4) return;
        
        // GRB order, offset = encoder * 3
        uint8_t buf[5];
        uint16_t offset = encoder * 3;
        buf[0] = (offset >> 8) & 0xFF;
        buf[1] = offset & 0xFF;
        buf[2] = g;  // GRB order
        buf[3] = r;
        buf[4] = b;
        write(SEESAW_NEOPIXEL_BASE, SEESAW_NEOPIXEL_BUF, buf, 5);
    }
    
    /**
     * @brief Show NeoPixels (call after setPixelColor)
     */
    void showPixels() {
        write(SEESAW_NEOPIXEL_BASE, SEESAW_NEOPIXEL_SHOW, nullptr, 0);
    }
    
    /**
     * @brief Set pixel color and immediately show
     */
    void setPixelColorAndShow(uint8_t encoder, uint8_t r, uint8_t g, uint8_t b) {
        setPixelColor(encoder, r, g, b);
        showPixels();
    }
    
    /**
     * @brief Color wheel helper (0-255 input)
     */
    static void wheelColor(uint8_t wheelPos, uint8_t& r, uint8_t& g, uint8_t& b) {
        wheelPos = 255 - wheelPos;
        if (wheelPos < 85) {
            r = 255 - wheelPos * 3;
            g = 0;
            b = wheelPos * 3;
        } else if (wheelPos < 170) {
            wheelPos -= 85;
            r = 0;
            g = wheelPos * 3;
            b = 255 - wheelPos * 3;
        } else {
            wheelPos -= 170;
            r = wheelPos * 3;
            g = 255 - wheelPos * 3;
            b = 0;
        }
    }
    
    // ========================================================================
    // Accessors
    // ========================================================================
    
    bool isInitialized() const { return m_initialized; }
    uint8_t getHardwareType() const { return m_hardwareType; }
    uint8_t getAddress() const { return m_addr; }

protected:
    // ========================================================================
    // Low-level I2C functions (matching Arduino Adafruit_seesaw pattern)
    // ========================================================================
    
    /**
     * @brief Write a single byte to a register
     * @param regHigh Module base address
     * @param regLow Function register
     * @param value Byte value to write
     * @return true on success
     */
    bool write8(uint8_t regHigh, uint8_t regLow, uint8_t value) {
        return write(regHigh, regLow, &value, 1) == ESP_OK;
    }
    
    /**
     * @brief Read a single byte from a register
     * @param regHigh Module base address
     * @param regLow Function register
     * @param delayUs Delay in microseconds before reading (default 250)
     * @return Byte value read (0 on error)
     */
    uint8_t read8(uint8_t regHigh, uint8_t regLow, uint16_t delayUs = SEESAW_DEFAULT_DELAY_US) {
        uint8_t ret = 0;
        read(regHigh, regLow, &ret, 1, delayUs);
        return ret;
    }
    
    /**
     * @brief Read multiple bytes from a register
     * 
     * This follows the EXACT Arduino pattern:
     *   1. Write [regHigh, regLow] with STOP
     *   2. Delay for specified microseconds
     *   3. Read response with STOP
     * 
     * @param regHigh Module base address
     * @param regLow Function register
     * @param buf Buffer to store data
     * @param num Number of bytes to read
     * @param delayUs Delay in microseconds (default 250, like Arduino)
     * @return ESP_OK on success
     */
    esp_err_t read(uint8_t regHigh, uint8_t regLow, uint8_t* buf, uint8_t num, 
                   uint16_t delayUs = SEESAW_DEFAULT_DELAY_US) {
        uint8_t prefix[2] = { regHigh, regLow };
        
        // Step 1: Write the register address (with STOP)
        esp_err_t err = i2c_master_write_to_device(m_i2cPort, m_addr, prefix, 2, 
                                                    pdMS_TO_TICKS(100));
        if (err != ESP_OK) {
            ESP_LOGW(SEESAW_TAG, "I2C write failed: reg=0x%02X%02X err=%s", 
                     regHigh, regLow, esp_err_to_name(err));
            return err;
        }
        
        // Step 2: Delay (required for seesaw to prepare data)
        esp_rom_delay_us(delayUs);
        
        // Step 3: Read the data (new START, with STOP)
        err = i2c_master_read_from_device(m_i2cPort, m_addr, buf, num, 
                                          pdMS_TO_TICKS(100));
        if (err != ESP_OK) {
            ESP_LOGW(SEESAW_TAG, "I2C read failed: reg=0x%02X%02X err=%s", 
                     regHigh, regLow, esp_err_to_name(err));
        }
        
        return err;
    }
    
    /**
     * @brief Write multiple bytes to a register
     * 
     * This follows the Arduino pattern:
     *   Write [regHigh, regLow, data...] with STOP
     * 
     * @param regHigh Module base address
     * @param regLow Function register
     * @param buf Data buffer (can be nullptr if num=0)
     * @param num Number of bytes to write
     * @return ESP_OK on success
     */
    esp_err_t write(uint8_t regHigh, uint8_t regLow, const uint8_t* buf = nullptr, uint8_t num = 0) {
        uint8_t writeBuffer[34];  // 2 prefix + 32 max data
        if (num > 32) return ESP_ERR_INVALID_SIZE;
        
        writeBuffer[0] = regHigh;
        writeBuffer[1] = regLow;
        
        if (buf != nullptr && num > 0) {
            memcpy(writeBuffer + 2, buf, num);
        }
        
        esp_err_t err = i2c_master_write_to_device(m_i2cPort, m_addr, writeBuffer, 
                                                    2 + num, pdMS_TO_TICKS(100));
        if (err != ESP_OK) {
            ESP_LOGW(SEESAW_TAG, "I2C write failed: reg=0x%02X%02X err=%s", 
                     regHigh, regLow, esp_err_to_name(err));
        }
        
        return err;
    }
    
private:
    /**
     * @brief Detect if device is present on I2C bus
     * @return true if device responds to address
     */
    bool i2cDetect() {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (m_addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t err = i2c_master_cmd_begin(m_i2cPort, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);
        return (err == ESP_OK);
    }
    
    /**
     * @brief Initialize button pullups for all 4 encoders
     */
    void initButtonPullups() {
        // Build mask of all button pins
        uint32_t mask = 0;
        for (int i = 0; i < 4; i++) {
            mask |= (1u << SEESAW_QUAD_BTN_PINS[i]);
        }
        
        // Set as INPUT_PULLUP (mode 2)
        pinModeBulk(mask, 2);
        
        ESP_LOGI(SEESAW_TAG, "Button pullups configured: pins %d, %d, %d, %d",
                 SEESAW_QUAD_BTN_PINS[0], SEESAW_QUAD_BTN_PINS[1], 
                 SEESAW_QUAD_BTN_PINS[2], SEESAW_QUAD_BTN_PINS[3]);
    }
    
    /**
     * @brief Initialize NeoPixels
     */
    void initNeoPixels() {
        // Set NeoPixel pin
        uint8_t pin = SEESAW_QUAD_NEOPIXEL_PIN;
        write(SEESAW_NEOPIXEL_BASE, SEESAW_NEOPIXEL_PIN, &pin, 1);
        
        // Set speed (1 = 800kHz WS2812)
        uint8_t speed = 1;
        write(SEESAW_NEOPIXEL_BASE, SEESAW_NEOPIXEL_SPEED, &speed, 1);
        
        // Set buffer length (4 LEDs * 3 bytes = 12 bytes)
        uint8_t len[2] = {0, 4 * 3};
        write(SEESAW_NEOPIXEL_BASE, SEESAW_NEOPIXEL_BUF_LENGTH, len, 2);
        
        // Initialize all pixels to off
        for (int i = 0; i < 4; i++) {
            setPixelColor(i, 0, 0, 0);
        }
        showPixels();
        
        ESP_LOGI(SEESAW_TAG, "NeoPixels initialized on pin %d", SEESAW_QUAD_NEOPIXEL_PIN);
    }
    
    // ========================================================================
    // Byte order conversion helpers
    // ========================================================================
    
    static void u32ToBe(uint32_t v, uint8_t out[4]) {
        out[0] = (v >> 24) & 0xFF;
        out[1] = (v >> 16) & 0xFF;
        out[2] = (v >> 8) & 0xFF;
        out[3] = v & 0xFF;
    }
    
    static uint32_t beToU32(const uint8_t b[4]) {
        return ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | 
               ((uint32_t)b[2] << 8) | b[3];
    }
    
    // ========================================================================
    // Member variables
    // ========================================================================
    
    i2c_port_t m_i2cPort = I2C_NUM_0;
    uint8_t m_addr = SEESAW_QUAD_DEFAULT_ADDR;
    uint8_t m_hardwareType = 0;
    bool m_initialized = false;
    
    // Cached encoder positions
    int32_t m_positions[4] = {0, 0, 0, 0};
    int32_t m_lastPositions[4] = {0, 0, 0, 0};
    bool m_changed[4] = {false, false, false, false};
};
