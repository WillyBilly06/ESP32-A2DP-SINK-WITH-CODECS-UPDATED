/**
 * @file recovery_main.cpp
 * @brief WiFi OTA Recovery Firmware with Encrypted Streaming Updates
 * 
 * Features:
 * - WiFi AP mode with captive portal for configuration
 * - Professional web UI for WiFi setup and OTA
 * - Encrypted OTA from Google Drive (AES-256-CBC)
 * - Streaming download+flash (no full download required)
 * - Automatic boot to main partition after power cycle
 */

#include <cstring>
#include <cstdlib>
#include <cinttypes>
#include <cmath>
#include <string>
#include <vector>

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "esp_tls.h"
#include "mbedtls/aes.h"
#include "mbedtls/md.h"
#include "mbedtls/pkcs5.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
}

static const char* TAG = "RECOVERY";

// ============================================================
// LED Matrix Configuration (for recovery status display)
// ============================================================
#define LED_MATRIX_WIDTH    16
#define LED_MATRIX_HEIGHT   16
#define LED_MATRIX_COUNT    (LED_MATRIX_WIDTH * LED_MATRIX_HEIGHT)
#define LED_GPIO_PIN        GPIO_NUM_4
#define LED_DEFAULT_BRIGHTNESS  64

// RGB structure for LED
struct RGB {
    uint8_t r, g, b;
    RGB() : r(0), g(0), b(0) {}
    RGB(uint8_t _r, uint8_t _g, uint8_t _b) : r(_r), g(_g), b(_b) {}
    RGB scale(uint8_t brightness) const {
        return RGB((r * brightness + 128) >> 8, (g * brightness + 128) >> 8, (b * brightness + 128) >> 8);
    }
};

// WS2812B timing lookup table for SPI encoding
static const uint16_t WS2812_TIMING_TABLE[16] = {
    0x1111, 0x7111, 0x1711, 0x7711, 0x1171, 0x7171, 0x1771, 0x7771,
    0x1117, 0x7117, 0x1717, 0x7717, 0x1177, 0x7177, 0x1777, 0x7777
};

// LED driver state
static RGB s_led_framebuffer[LED_MATRIX_COUNT];
static uint8_t s_led_brightness = LED_DEFAULT_BRIGHTNESS;
static spi_device_handle_t s_led_spi = nullptr;
static uint16_t* s_led_dma_buffer = nullptr;
static size_t s_led_dma_buffer_size = 0;
static bool s_led_initialized = false;
static SemaphoreHandle_t s_led_mutex = nullptr;
static TaskHandle_t s_led_task_handle = nullptr;

// LED initialization
static esp_err_t led_init() {
    s_led_dma_buffer_size = (LED_MATRIX_COUNT * 12) + 20;
    ESP_LOGI(TAG, "Initializing SPI LED driver: %d LEDs, GPIO %d", LED_MATRIX_COUNT, LED_GPIO_PIN);
    
    s_led_dma_buffer = (uint16_t*)heap_caps_malloc(s_led_dma_buffer_size, MALLOC_CAP_DMA);
    if (!s_led_dma_buffer) {
        ESP_LOGE(TAG, "Failed to allocate LED DMA buffer");
        return ESP_ERR_NO_MEM;
    }
    memset(s_led_dma_buffer, 0, s_led_dma_buffer_size);
    
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = LED_GPIO_PIN;
    buscfg.miso_io_num = -1;
    buscfg.sclk_io_num = -1;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = (int)s_led_dma_buffer_size;
    
    esp_err_t err = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(err));
        heap_caps_free(s_led_dma_buffer);
        s_led_dma_buffer = nullptr;
        return err;
    }
    
    spi_device_interface_config_t devcfg = {};
    devcfg.mode = 0;
    devcfg.clock_speed_hz = 3200000;
    devcfg.spics_io_num = -1;
    devcfg.flags = SPI_DEVICE_TXBIT_LSBFIRST;
    devcfg.queue_size = 1;
    
    err = spi_bus_add_device(SPI2_HOST, &devcfg, &s_led_spi);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI device add failed: %s", esp_err_to_name(err));
        spi_bus_free(SPI2_HOST);
        heap_caps_free(s_led_dma_buffer);
        s_led_dma_buffer = nullptr;
        return err;
    }
    
    s_led_mutex = xSemaphoreCreateMutex();
    s_led_initialized = true;
    ESP_LOGI(TAG, "LED driver initialized");
    return ESP_OK;
}

static void led_clear() {
    for (int i = 0; i < LED_MATRIX_COUNT; i++) {
        s_led_framebuffer[i] = RGB();
    }
}

static void led_fill(RGB color) {
    for (int i = 0; i < LED_MATRIX_COUNT; i++) {
        s_led_framebuffer[i] = color;
    }
}

static void led_set_pixel_xy(int x, int y, RGB color) {
    if (x < 0 || x >= LED_MATRIX_WIDTH || y < 0 || y >= LED_MATRIX_HEIGHT) return;
    int index = (y & 1) ? (y * LED_MATRIX_WIDTH + (LED_MATRIX_WIDTH - 1 - x)) : (y * LED_MATRIX_WIDTH + x);
    s_led_framebuffer[index] = color;
}

static esp_err_t led_show() {
    if (!s_led_initialized || !s_led_dma_buffer) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(s_led_mutex, pdMS_TO_TICKS(5)) != pdTRUE) return ESP_ERR_TIMEOUT;
    
    int n = 0;
    s_led_dma_buffer[n++] = 0;
    
    for (int i = 0; i < LED_MATRIX_COUNT; i++) {
        RGB pixel = s_led_framebuffer[i].scale(s_led_brightness);
        uint8_t g = pixel.g, r = pixel.r, b = pixel.b;
        s_led_dma_buffer[n++] = WS2812_TIMING_TABLE[(g >> 4) & 0x0F];
        s_led_dma_buffer[n++] = WS2812_TIMING_TABLE[g & 0x0F];
        s_led_dma_buffer[n++] = WS2812_TIMING_TABLE[(r >> 4) & 0x0F];
        s_led_dma_buffer[n++] = WS2812_TIMING_TABLE[r & 0x0F];
        s_led_dma_buffer[n++] = WS2812_TIMING_TABLE[(b >> 4) & 0x0F];
        s_led_dma_buffer[n++] = WS2812_TIMING_TABLE[b & 0x0F];
    }
    for (int i = 0; i < 5; i++) s_led_dma_buffer[n++] = 0;
    
    spi_transaction_t trans = {};
    trans.length = (size_t)(n * 16);
    trans.tx_buffer = s_led_dma_buffer;
    esp_err_t err = spi_device_transmit(s_led_spi, &trans);
    
    xSemaphoreGive(s_led_mutex);
    return err;
}

// ============================================================
// LED Effect Patterns for Recovery Mode
// ============================================================
static volatile int s_led_effect_mode = 0;  // 0=idle, 1=wifi_wait, 2=downloading, 3=flashing, 4=success, 5=error
static volatile int s_led_effect_progress = 0;  // 0-100 for progress

static void led_effect_task(void* param) {
    uint32_t frame = 0;
    
    while (true) {
        frame++;
        led_clear();
        
        switch (s_led_effect_mode) {
            case 0:  // Idle - gentle purple breathing
            {
                int brightness = 20 + (int)(15.0f * sinf(frame * 0.05f));
                led_fill(RGB(brightness, 0, brightness));
                break;
            }
            
            case 1:  // WiFi waiting - blue spinning circle
            {
                // Precomputed: PI / 180 = degrees to radians conversion
                constexpr float kDegToRad = 3.14159f / 180.0f;  // Compile-time constant
                int cx = LED_MATRIX_WIDTH / 2;
                int cy = LED_MATRIX_HEIGHT / 2;
                for (int angle = 0; angle < 360; angle += 30) {
                    float rad = (angle + frame * 10) * kDegToRad;
                    int x = cx + (int)(6.0f * cosf(rad));
                    int y = cy + (int)(6.0f * sinf(rad));
                    int fade = 255 - (angle * 200 / 360);
                    led_set_pixel_xy(x, y, RGB(0, 0, fade > 50 ? fade : 50));
                }
                break;
            }
            
            case 2:  // Downloading - cyan progress bar from bottom
            {
                int rows = (s_led_effect_progress * LED_MATRIX_HEIGHT) / 100;
                for (int y = LED_MATRIX_HEIGHT - 1; y >= LED_MATRIX_HEIGHT - rows; y--) {
                    for (int x = 0; x < LED_MATRIX_WIDTH; x++) {
                        led_set_pixel_xy(x, y, RGB(0, 150, 200));
                    }
                }
                // Animated top line
                int top_y = LED_MATRIX_HEIGHT - rows - 1;
                if (top_y >= 0 && top_y < LED_MATRIX_HEIGHT) {
                    for (int x = 0; x < LED_MATRIX_WIDTH; x++) {
                        int wave = (int)(127.0f * (1.0f + sinf((x + frame) * 0.5f)));
                        led_set_pixel_xy(x, top_y, RGB(0, wave, 255));
                    }
                }
                break;
            }
            
            case 3:  // Flashing - orange/yellow progress bar from bottom
            {
                int rows = (s_led_effect_progress * LED_MATRIX_HEIGHT) / 100;
                for (int y = LED_MATRIX_HEIGHT - 1; y >= LED_MATRIX_HEIGHT - rows; y--) {
                    for (int x = 0; x < LED_MATRIX_WIDTH; x++) {
                        led_set_pixel_xy(x, y, RGB(255, 100, 0));
                    }
                }
                // Animated top line
                int top_y = LED_MATRIX_HEIGHT - rows - 1;
                if (top_y >= 0 && top_y < LED_MATRIX_HEIGHT) {
                    for (int x = 0; x < LED_MATRIX_WIDTH; x++) {
                        int wave = (int)(127.0f * (1.0f + sinf((x + frame) * 0.5f)));
                        led_set_pixel_xy(x, top_y, RGB(255, wave, 0));
                    }
                }
                break;
            }
            
            case 4:  // Success - green checkmark animation
            {
                led_fill(RGB(0, 50, 0));
                // Draw checkmark
                int offset = (frame < 30) ? frame / 3 : 10;
                // Short part of check
                for (int i = 0; i < offset && i < 4; i++) {
                    led_set_pixel_xy(4 + i, 8 + i, RGB(0, 255, 0));
                }
                // Long part of check
                for (int i = 0; i < offset - 3 && i < 8; i++) {
                    led_set_pixel_xy(7 + i, 11 - i, RGB(0, 255, 0));
                }
                break;
            }
            
            case 5:  // Error - red X pulsing
            {
                int pulse = 128 + (int)(127.0f * sinf(frame * 0.2f));
                // Draw X
                for (int i = 0; i < 10; i++) {
                    led_set_pixel_xy(3 + i, 3 + i, RGB(pulse, 0, 0));
                    led_set_pixel_xy(3 + i, 12 - i, RGB(pulse, 0, 0));
                }
                break;
            }
        }
        
        led_show();
        vTaskDelay(pdMS_TO_TICKS(33));  // ~30 FPS
    }
}

static void led_set_mode(int mode) {
    s_led_effect_mode = mode;
}

static void led_set_progress(int progress) {
    s_led_effect_progress = progress;
}

// ============================================================
// Configuration
// ============================================================
static const char* DEVICE_NAME = "ESP32-Recovery";
static const char* AP_SSID = "ESP32-Recovery-Setup";
static const char* AP_PASSWORD = "recovery123";  // Min 8 chars for WPA2
static const char* RECOVERY_VERSION = "RECOVERY v2.0 (WiFi)";

// Recovery button - must be held during power-on to stay in recovery
// If not held, we boot back to the main firmware
static const gpio_num_t RECOVERY_BUTTON_GPIO = GPIO_NUM_18;

// NVS Keys for stored WiFi credentials and OTA version
static const char* NVS_NAMESPACE = "wifi_creds";
static const char* NVS_KEY_SSID = "ssid";
static const char* NVS_KEY_PASS = "password";
static const char* NVS_KEY_OTA_VER = "ota_version";

// ============================================================
// Google Drive Configuration
// ============================================================
// Upload latest.txt to Google Drive, share it, and put its FILE_ID here.
// latest.txt contains one line: VERSION,FIRMWARE_FILE_ID
// Example: 1.0.0,ABC123XYZ
//
static const char* GDRIVE_LATEST_TXT_ID = "YOUR_GOOGLE_DRIVE_FILE_ID_HERE";

// Current installed firmware version (update this when building new firmware)
static const char* CURRENT_FW_VERSION = "1.0.0";

// OTA state
static char s_available_version[32] = {0};
static char s_firmware_file_id[64] = {0};
static bool s_update_available = false;

// ============================================================
// Encryption Configuration
// ============================================================
// AES-256 Key (32 bytes) - CHANGE THIS TO YOUR OWN SECRET KEY!
// This key must match the one used to encrypt firmware files
// Generate a new key with: openssl rand -hex 32
static const uint8_t AES_KEY[32] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F
};

// IV will be read from the first 16 bytes of encrypted firmware
static const size_t AES_BLOCK_SIZE = 16;
static const size_t AES_KEY_SIZE = 32;

// ============================================================
// State Management
// ============================================================
static EventGroupHandle_t s_wifi_event_group;
static const int WIFI_CONNECTED_BIT = BIT0;
static const int WIFI_FAIL_BIT = BIT1;

static httpd_handle_t s_server = NULL;
static bool s_wifi_connected = false;
static bool s_ota_in_progress = false;
static int s_ota_progress = 0;
static char s_ota_status[128] = "Idle";
static char s_stored_ssid[64] = {0};
static char s_stored_pass[64] = {0};
static int s_wifi_retry_count = 0;
static const int MAX_WIFI_RETRIES = 5;

// ============================================================
// DNS Server for Captive Portal
// ============================================================
static TaskHandle_t s_dns_task = NULL;
static bool s_dns_running = false;

static void dns_server_task(void* pvParameters) {
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "DNS: Failed to create socket");
        vTaskDelete(NULL);
        return;
    }
    
    struct sockaddr_in server_addr = {};
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(53);
    
    if (bind(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "DNS: Failed to bind");
        close(sock);
        vTaskDelete(NULL);
        return;
    }
    
    // Set receive timeout
    struct timeval tv = { .tv_sec = 1, .tv_usec = 0 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    
    ESP_LOGI(TAG, "DNS server started on port 53");
    s_dns_running = true;
    
    uint8_t buffer[512];
    struct sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);
    
    // AP IP address (192.168.4.1)
    uint8_t ap_ip[4] = {192, 168, 4, 1};
    
    while (s_dns_running) {
        int len = recvfrom(sock, buffer, sizeof(buffer), 0, 
                          (struct sockaddr*)&client_addr, &client_len);
        if (len < 12) continue;  // Minimum DNS header size
        
        // Build DNS response - redirect all queries to AP IP
        uint8_t response[512];
        memcpy(response, buffer, len);
        
        // Set response flags: QR=1 (response), AA=1 (authoritative)
        response[2] = 0x84;  // QR=1, OPCODE=0, AA=1, TC=0, RD=0
        response[3] = 0x00;  // RA=0, Z=0, RCODE=0 (no error)
        
        // Set answer count to 1
        response[6] = 0x00;
        response[7] = 0x01;
        
        // Find end of question section
        int pos = 12;
        while (pos < len && buffer[pos] != 0) {
            pos += buffer[pos] + 1;
        }
        pos += 5;  // Skip null byte + QTYPE + QCLASS
        
        // Add answer: pointer to question name
        response[pos++] = 0xC0;  // Pointer
        response[pos++] = 0x0C;  // Offset to name in question
        
        // Type A
        response[pos++] = 0x00;
        response[pos++] = 0x01;
        
        // Class IN
        response[pos++] = 0x00;
        response[pos++] = 0x01;
        
        // TTL (60 seconds)
        response[pos++] = 0x00;
        response[pos++] = 0x00;
        response[pos++] = 0x00;
        response[pos++] = 0x3C;
        
        // Data length (4 bytes for IPv4)
        response[pos++] = 0x00;
        response[pos++] = 0x04;
        
        // IP address (192.168.4.1)
        response[pos++] = ap_ip[0];
        response[pos++] = ap_ip[1];
        response[pos++] = ap_ip[2];
        response[pos++] = ap_ip[3];
        
        sendto(sock, response, pos, 0, (struct sockaddr*)&client_addr, client_len);
    }
    
    close(sock);
    vTaskDelete(NULL);
}

static void start_dns_server() {
    if (s_dns_task == NULL) {
        xTaskCreate(dns_server_task, "dns_server", 4096, NULL, 5, &s_dns_task);
    }
}

static void stop_dns_server() {
    s_dns_running = false;
    vTaskDelay(pdMS_TO_TICKS(100));
    s_dns_task = NULL;
}

// ============================================================
// HTML/CSS/JS for Web UI
// ============================================================
static const char* HTML_HEADER = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ESP32 Recovery Mode</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }
        
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #1a1a2e 0%, #16213e 50%, #0f3460 100%);
            min-height: 100vh;
            display: flex;
            justify-content: center;
            align-items: center;
            padding: 20px;
        }
        
        .container {
            background: rgba(255, 255, 255, 0.05);
            backdrop-filter: blur(20px);
            border-radius: 24px;
            padding: 40px;
            max-width: 480px;
            width: 100%;
            box-shadow: 0 25px 50px rgba(0, 0, 0, 0.3),
                        inset 0 1px 1px rgba(255, 255, 255, 0.1);
            border: 1px solid rgba(255, 255, 255, 0.1);
        }
        
        .logo {
            text-align: center;
            margin-bottom: 30px;
        }
        
        .logo-icon {
            width: 80px;
            height: 80px;
            background: linear-gradient(135deg, #e94560 0%, #ff6b6b 100%);
            border-radius: 20px;
            margin: 0 auto 15px;
            display: flex;
            justify-content: center;
            align-items: center;
            box-shadow: 0 10px 30px rgba(233, 69, 96, 0.3);
        }
        
        .logo-icon svg {
            width: 45px;
            height: 45px;
            fill: white;
        }
        
        h1 {
            color: #ffffff;
            font-size: 1.8em;
            font-weight: 600;
            text-align: center;
            margin-bottom: 10px;
        }
        
        .subtitle {
            color: rgba(255, 255, 255, 0.6);
            text-align: center;
            font-size: 0.9em;
            margin-bottom: 30px;
        }
        
        .version {
            color: #e94560;
            font-weight: 500;
        }
        
        .card {
            background: rgba(255, 255, 255, 0.03);
            border-radius: 16px;
            padding: 25px;
            margin-bottom: 20px;
            border: 1px solid rgba(255, 255, 255, 0.05);
        }
        
        .card-title {
            color: #ffffff;
            font-size: 1.1em;
            font-weight: 600;
            margin-bottom: 20px;
            display: flex;
            align-items: center;
            gap: 10px;
        }
        
        .card-title svg {
            width: 22px;
            height: 22px;
            fill: #e94560;
        }
        
        .form-group {
            margin-bottom: 20px;
        }
        
        label {
            display: block;
            color: rgba(255, 255, 255, 0.8);
            font-size: 0.9em;
            margin-bottom: 8px;
            font-weight: 500;
        }
        
        input[type="text"],
        input[type="password"] {
            width: 100%;
            padding: 14px 18px;
            background: rgba(255, 255, 255, 0.08);
            border: 2px solid rgba(255, 255, 255, 0.1);
            border-radius: 12px;
            color: #ffffff;
            font-size: 1em;
            transition: all 0.3s ease;
        }
        
        input[type="text"]:focus,
        input[type="password"]:focus {
            outline: none;
            border-color: #e94560;
            background: rgba(255, 255, 255, 0.12);
            box-shadow: 0 0 20px rgba(233, 69, 96, 0.2);
        }
        
        input::placeholder {
            color: rgba(255, 255, 255, 0.3);
        }
        
        .btn {
            width: 100%;
            padding: 16px;
            border: none;
            border-radius: 12px;
            font-size: 1em;
            font-weight: 600;
            cursor: pointer;
            transition: all 0.3s ease;
            display: flex;
            justify-content: center;
            align-items: center;
            gap: 10px;
        }
        
        .btn-primary {
            background: linear-gradient(135deg, #e94560 0%, #ff6b6b 100%);
            color: white;
            box-shadow: 0 8px 25px rgba(233, 69, 96, 0.3);
        }
        
        .btn-primary:hover {
            transform: translateY(-2px);
            box-shadow: 0 12px 35px rgba(233, 69, 96, 0.4);
        }
        
        .btn-primary:active {
            transform: translateY(0);
        }
        
        .btn-secondary {
            background: rgba(255, 255, 255, 0.1);
            color: white;
            border: 2px solid rgba(255, 255, 255, 0.2);
        }
        
        .btn-secondary:hover {
            background: rgba(255, 255, 255, 0.15);
            border-color: rgba(255, 255, 255, 0.3);
        }
        
        .btn:disabled {
            opacity: 0.6;
            cursor: not-allowed;
            transform: none !important;
        }
        
        .btn svg {
            width: 20px;
            height: 20px;
            fill: currentColor;
        }
        
        .status-indicator {
            display: flex;
            align-items: center;
            gap: 10px;
            padding: 15px;
            border-radius: 12px;
            margin-bottom: 20px;
        }
        
        .status-connected {
            background: rgba(46, 213, 115, 0.15);
            border: 1px solid rgba(46, 213, 115, 0.3);
        }
        
        .status-connected .dot {
            background: #2ed573;
            box-shadow: 0 0 10px #2ed573;
        }
        
        .status-disconnected {
            background: rgba(255, 165, 2, 0.15);
            border: 1px solid rgba(255, 165, 2, 0.3);
        }
        
        .status-disconnected .dot {
            background: #ffa502;
            box-shadow: 0 0 10px #ffa502;
        }
        
        .dot {
            width: 10px;
            height: 10px;
            border-radius: 50%;
            animation: pulse 2s infinite;
        }
        
        @keyframes pulse {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.5; }
        }
        
        .status-text {
            color: rgba(255, 255, 255, 0.9);
            font-size: 0.9em;
        }
        
        .progress-container {
            background: rgba(255, 255, 255, 0.1);
            border-radius: 10px;
            height: 12px;
            overflow: hidden;
            margin: 20px 0;
        }
        
        .progress-bar {
            height: 100%;
            background: linear-gradient(90deg, #e94560, #ff6b6b);
            border-radius: 10px;
            transition: width 0.3s ease;
            position: relative;
            overflow: hidden;
        }
        
        .progress-bar::after {
            content: '';
            position: absolute;
            top: 0;
            left: 0;
            right: 0;
            bottom: 0;
            background: linear-gradient(90deg, 
                transparent, 
                rgba(255, 255, 255, 0.3), 
                transparent);
            animation: shimmer 1.5s infinite;
        }
        
        @keyframes shimmer {
            0% { transform: translateX(-100%); }
            100% { transform: translateX(100%); }
        }
        
        .progress-text {
            text-align: center;
            color: rgba(255, 255, 255, 0.8);
            font-size: 0.95em;
            margin-top: 10px;
        }
        
        .message {
            padding: 15px;
            border-radius: 12px;
            margin-bottom: 20px;
            font-size: 0.9em;
        }
        
        .message-success {
            background: rgba(46, 213, 115, 0.15);
            border: 1px solid rgba(46, 213, 115, 0.3);
            color: #2ed573;
        }
        
        .message-error {
            background: rgba(255, 71, 87, 0.15);
            border: 1px solid rgba(255, 71, 87, 0.3);
            color: #ff4757;
        }
        
        .message-info {
            background: rgba(52, 152, 219, 0.15);
            border: 1px solid rgba(52, 152, 219, 0.3);
            color: #3498db;
        }
        
        .nav-tabs {
            display: flex;
            gap: 10px;
            margin-bottom: 25px;
        }
        
        .nav-tab {
            flex: 1;
            padding: 12px;
            background: rgba(255, 255, 255, 0.05);
            border: 2px solid rgba(255, 255, 255, 0.1);
            border-radius: 12px;
            color: rgba(255, 255, 255, 0.6);
            text-align: center;
            cursor: pointer;
            transition: all 0.3s ease;
            font-weight: 500;
        }
        
        .nav-tab.active {
            background: rgba(233, 69, 96, 0.2);
            border-color: #e94560;
            color: #e94560;
        }
        
        .nav-tab:hover:not(.active) {
            background: rgba(255, 255, 255, 0.08);
            color: rgba(255, 255, 255, 0.8);
        }
        
        .hidden {
            display: none !important;
        }
        
        .spinner {
            width: 20px;
            height: 20px;
            border: 2px solid rgba(255, 255, 255, 0.3);
            border-top-color: white;
            border-radius: 50%;
            animation: spin 1s linear infinite;
        }
        
        @keyframes spin {
            to { transform: rotate(360deg); }
        }
        
        .wifi-network {
            display: flex;
            align-items: center;
            gap: 12px;
            padding: 15px;
            background: rgba(255, 255, 255, 0.05);
            border-radius: 12px;
            margin-bottom: 10px;
            cursor: pointer;
            transition: all 0.3s ease;
            border: 2px solid transparent;
        }
        
        .wifi-network:hover {
            background: rgba(255, 255, 255, 0.1);
            border-color: rgba(233, 69, 96, 0.3);
        }
        
        .wifi-network.selected {
            border-color: #e94560;
            background: rgba(233, 69, 96, 0.1);
        }
        
        .wifi-signal {
            width: 24px;
            height: 24px;
            fill: rgba(255, 255, 255, 0.6);
        }
        
        .wifi-name {
            flex: 1;
            color: white;
            font-weight: 500;
        }
        
        .wifi-rssi {
            color: rgba(255, 255, 255, 0.4);
            font-size: 0.8em;
        }
        
        footer {
            text-align: center;
            color: rgba(255, 255, 255, 0.3);
            font-size: 0.8em;
            margin-top: 20px;
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="logo">
            <div class="logo-icon">
                <svg viewBox="0 0 24 24"><path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm-2 15l-5-5 1.41-1.41L10 14.17l7.59-7.59L19 8l-9 9z"/></svg>
            </div>
            <h1>ESP32 Recovery</h1>
            <p class="subtitle">Firmware Update System • <span class="version">v2.0</span></p>
        </div>
)rawliteral";

static const char* HTML_FOOTER = R"rawliteral(
        <footer>
            ESP32 Recovery Mode • Secure OTA Updates
        </footer>
    </div>
</body>
</html>
)rawliteral";

// ============================================================
// WiFi Configuration HTML Page
// ============================================================
static const char* HTML_WIFI_CONFIG = R"rawliteral(
        <div class="nav-tabs">
            <div class="nav-tab active" onclick="showTab('wifi')">WiFi Setup</div>
            <div class="nav-tab" onclick="showTab('ota')" id="otaTab">OTA Update</div>
        </div>
        
        <div id="wifiSection">
            <div class="status-indicator %WIFI_STATUS_CLASS%">
                <div class="dot"></div>
                <span class="status-text">%WIFI_STATUS_TEXT%</span>
            </div>
            
            <div class="card">
                <div class="card-title">
                    <svg viewBox="0 0 24 24"><path d="M1 9l2 2c4.97-4.97 13.03-4.97 18 0l2-2C16.93 2.93 7.08 2.93 1 9zm8 8l3 3 3-3c-1.65-1.66-4.34-1.66-6 0zm-4-4l2 2c2.76-2.76 7.24-2.76 10 0l2-2C15.14 9.14 8.87 9.14 5 13z"/></svg>
                    Configure WiFi
                </div>
                
                <form action="/wifi/connect" method="POST">
                    <div class="form-group">
                        <label for="ssid">WiFi Network Name (SSID)</label>
                        <input type="text" id="ssid" name="ssid" placeholder="Enter WiFi name" value="%STORED_SSID%" required>
                    </div>
                    
                    <div class="form-group">
                        <label for="password">Password</label>
                        <input type="password" id="password" name="password" placeholder="Enter WiFi password">
                    </div>
                    
                    <button type="submit" class="btn btn-primary">
                        <svg viewBox="0 0 24 24"><path d="M1 9l2 2c4.97-4.97 13.03-4.97 18 0l2-2C16.93 2.93 7.08 2.93 1 9zm8 8l3 3 3-3c-1.65-1.66-4.34-1.66-6 0zm-4-4l2 2c2.76-2.76 7.24-2.76 10 0l2-2C15.14 9.14 8.87 9.14 5 13z"/></svg>
                        Connect to WiFi
                    </button>
                </form>
            </div>
            
            <button class="btn btn-secondary" onclick="scanWifi()">
                <svg viewBox="0 0 24 24"><path d="M17.65 6.35C16.2 4.9 14.21 4 12 4c-4.42 0-7.99 3.58-7.99 8s3.57 8 7.99 8c3.73 0 6.84-2.55 7.73-6h-2.08c-.82 2.33-3.04 4-5.65 4-3.31 0-6-2.69-6-6s2.69-6 6-6c1.66 0 3.14.69 4.22 1.78L13 11h7V4l-2.35 2.35z"/></svg>
                Scan for Networks
            </button>
            
            <div id="scanResults" class="card hidden" style="margin-top: 20px;">
                <div class="card-title">Available Networks</div>
                <div id="networkList"></div>
            </div>
        </div>
        
        <div id="otaSection" class="hidden">
            <!-- OTA content will be shown here when connected -->
        </div>
        
        <script>
            function showTab(tab) {
                document.querySelectorAll('.nav-tab').forEach(t => t.classList.remove('active'));
                document.querySelector('.nav-tab[onclick*="' + tab + '"]').classList.add('active');
                document.getElementById('wifiSection').classList.toggle('hidden', tab !== 'wifi');
                document.getElementById('otaSection').classList.toggle('hidden', tab !== 'ota');
                if (tab === 'ota') loadOtaPage();
            }
            
            function loadOtaPage() {
                fetch('/ota/page')
                    .then(r => r.text())
                    .then(html => document.getElementById('otaSection').innerHTML = html);
            }
            
            function scanWifi() {
                document.getElementById('scanResults').classList.remove('hidden');
                document.getElementById('networkList').innerHTML = '<div class="spinner" style="margin: 20px auto;"></div>';
                
                fetch('/wifi/scan')
                    .then(r => r.json())
                    .then(data => {
                        let html = '';
                        data.networks.forEach(n => {
                            html += '<div class="wifi-network" onclick="selectNetwork(\'' + n.ssid.replace(/'/g, "\\'") + '\')">' +
                                '<svg class="wifi-signal" viewBox="0 0 24 24"><path d="M1 9l2 2c4.97-4.97 13.03-4.97 18 0l2-2C16.93 2.93 7.08 2.93 1 9zm8 8l3 3 3-3c-1.65-1.66-4.34-1.66-6 0zm-4-4l2 2c2.76-2.76 7.24-2.76 10 0l2-2C15.14 9.14 8.87 9.14 5 13z"/></svg>' +
                                '<span class="wifi-name">' + n.ssid + '</span>' +
                                '<span class="wifi-rssi">' + n.rssi + ' dBm</span></div>';
                        });
                        document.getElementById('networkList').innerHTML = html || '<p style="color: rgba(255,255,255,0.5); text-align: center;">No networks found</p>';
                    });
            }
            
            function selectNetwork(ssid) {
                document.getElementById('ssid').value = ssid;
                document.querySelectorAll('.wifi-network').forEach(n => n.classList.remove('selected'));
                event.currentTarget.classList.add('selected');
            }
            
            // OTA Functions
            let otaPolling = null;
            
            function startOta() {
                console.log('startOta called');
                document.getElementById('otaBtn').disabled = true;
                document.getElementById('otaBtn').innerHTML = '<div class="spinner"></div> Checking...';
                document.getElementById('otaProgress').classList.remove('hidden');
                document.getElementById('otaInfo').classList.add('hidden');
                document.getElementById('otaResult').classList.add('hidden');
                
                fetch('/ota/start', { method: 'POST' })
                    .then(r => r.json())
                    .then(data => {
                        console.log('ota/start response:', data);
                        if (data.success) {
                            otaPolling = setInterval(pollOtaStatus, 500);
                        } else {
                            showResult('error', data.message || 'Failed to start update');
                        }
                    })
                    .catch(e => {
                        console.error('ota/start error:', e);
                        showResult('error', 'Connection error: ' + e.message);
                    });
            }
            
            function pollOtaStatus() {
                fetch('/ota/status')
                    .then(r => r.json())
                    .then(data => {
                        document.getElementById('progressBar').style.width = data.progress + '%';
                        document.getElementById('progressText').textContent = data.status;
                        
                        if (data.complete) {
                            clearInterval(otaPolling);
                            if (data.success) {
                                showResult('success', 'Update complete! Device will restart...');
                                setTimeout(() => location.reload(), 5000);
                            } else {
                                showResult('error', data.status);
                            }
                        }
                    })
                    .catch(() => {
                        clearInterval(otaPolling);
                        showResult('success', 'Device is restarting with new firmware...');
                    });
            }
            
            function showResult(type, message) {
                document.getElementById('otaProgress').classList.add('hidden');
                document.getElementById('otaResult').classList.remove('hidden');
                document.getElementById('otaResult').className = 'message message-' + type;
                document.getElementById('otaResult').textContent = message;
                
                document.getElementById('otaBtn').disabled = false;
                document.getElementById('otaBtn').innerHTML = 
                    '<svg viewBox="0 0 24 24"><path d="M5 20h14v-2H5v2zM19 9h-4V3H9v6H5l7 7 7-7z"/></svg> Check for Updates';
            }
        </script>
)rawliteral";

// ============================================================
// OTA Update HTML Page
// ============================================================
static const char* HTML_OTA_PAGE = R"rawliteral(
<div class="status-indicator %OTA_STATUS_CLASS%">
    <div class="dot"></div>
    <span class="status-text">%OTA_STATUS_TEXT%</span>
</div>

<div class="card">
    <div class="card-title">
        <svg viewBox="0 0 24 24"><path d="M5 20h14v-2H5v2zM19 9h-4V3H9v6H5l7 7 7-7z"/></svg>
        Firmware Update
    </div>
    
    <div id="otaInfo">
        <div style="display: flex; justify-content: space-between; margin-bottom: 15px; padding: 12px; background: rgba(255,255,255,0.05); border-radius: 10px;">
            <div>
                <div style="color: rgba(255,255,255,0.5); font-size: 0.8em;">Current Version</div>
                <div style="color: white; font-weight: 600;">v%CURRENT_VERSION%</div>
            </div>
            <div style="text-align: right;">
                <div style="color: rgba(255,255,255,0.5); font-size: 0.8em;">Mode</div>
                <div style="color: #e94560; font-weight: 600;">Recovery</div>
            </div>
        </div>
        
        <p style="color: rgba(255,255,255,0.7); margin-bottom: 15px;">
            Click the button below to check for firmware updates.
            If a newer version is available, it will be downloaded and installed.
            If not, you can reinstall the current version for recovery.
        </p>
        
        <div class="message message-info">
            <strong>Note:</strong> Do not power off the device during the update process.
        </div>
    </div>
    
    <div id="otaProgress" class="hidden">
        <div class="progress-container">
            <div class="progress-bar" id="progressBar" style="width: 0%"></div>
        </div>
        <div class="progress-text" id="progressText">Preparing update...</div>
    </div>
    
    <div id="otaResult" class="hidden"></div>
    
    <button id="otaBtn" class="btn btn-primary" onclick="startOta()">
        <svg viewBox="0 0 24 24"><path d="M17.65 6.35C16.2 4.9 14.21 4 12 4c-4.42 0-7.99 3.58-7.99 8s3.57 8 7.99 8c3.73 0 6.84-2.55 7.73-6h-2.08c-.82 2.33-3.04 4-5.65 4-3.31 0-6-2.69-6-6s2.69-6 6-6c1.66 0 3.14.69 4.22 1.78L13 11h7V4l-2.35 2.35z"/></svg>
        Check for Updates
    </button>
</div>
)rawliteral";

// ============================================================
// Captive Portal Detection Handler
// ============================================================
static const char* CAPTIVE_PORTAL_REDIRECT = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta http-equiv="refresh" content="0; url=http://192.168.4.1/">
    <script>window.location.href = 'http://192.168.4.1/';</script>
</head>
<body>
    <p>Redirecting to <a href="http://192.168.4.1/">ESP32 Recovery</a>...</p>
</body>
</html>
)rawliteral";

// ============================================================
// NVS Storage Functions
// ============================================================
static void loadStoredCredentials() {
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) == ESP_OK) {
        size_t ssidLen = sizeof(s_stored_ssid);
        size_t passLen = sizeof(s_stored_pass);
        nvs_get_str(handle, NVS_KEY_SSID, s_stored_ssid, &ssidLen);
        nvs_get_str(handle, NVS_KEY_PASS, s_stored_pass, &passLen);
        nvs_close(handle);
        ESP_LOGI(TAG, "Loaded stored SSID: %s", s_stored_ssid);
    }
}

static void saveCredentials(const char* ssid, const char* password) {
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK) {
        nvs_set_str(handle, NVS_KEY_SSID, ssid);
        nvs_set_str(handle, NVS_KEY_PASS, password);
        nvs_commit(handle);
        nvs_close(handle);
        strncpy(s_stored_ssid, ssid, sizeof(s_stored_ssid) - 1);
        strncpy(s_stored_pass, password, sizeof(s_stored_pass) - 1);
        ESP_LOGI(TAG, "Saved WiFi credentials for: %s", ssid);
    }
}

// ============================================================
// WiFi Event Handler
// ============================================================
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_STA_START) {
            esp_wifi_connect();
        } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            s_wifi_connected = false;
            if (s_wifi_retry_count < MAX_WIFI_RETRIES) {
                esp_wifi_connect();
                s_wifi_retry_count++;
                ESP_LOGI(TAG, "WiFi retry %d/%d", s_wifi_retry_count, MAX_WIFI_RETRIES);
            } else {
                xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
                ESP_LOGW(TAG, "WiFi connection failed after %d retries", MAX_WIFI_RETRIES);
            }
        } else if (event_id == WIFI_EVENT_AP_STACONNECTED) {
            wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*)event_data;
            ESP_LOGI(TAG, "Client connected to AP, AID=%d", event->aid);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
        ESP_LOGI(TAG, "Connected! IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_wifi_connected = true;
        s_wifi_retry_count = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        led_set_mode(0);  // Idle mode (purple breathing) - connected and ready
    }
}

// ============================================================
// WiFi Initialization - AP+STA Mode
// ============================================================
static void init_wifi() {
    s_wifi_event_group = xEventGroupCreate();
    
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // Create both AP and STA netif
    esp_netif_create_default_wifi_ap();
    esp_netif_create_default_wifi_sta();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));
    
    // Configure AP
    wifi_config_t ap_config = {};
    strncpy((char*)ap_config.ap.ssid, AP_SSID, sizeof(ap_config.ap.ssid));
    strncpy((char*)ap_config.ap.password, AP_PASSWORD, sizeof(ap_config.ap.password));
    ap_config.ap.ssid_len = strlen(AP_SSID);
    ap_config.ap.channel = 1;
    ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ap_config.ap.max_connection = 4;
    
    // Configure STA (empty initially)
    wifi_config_t sta_config = {};
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "WiFi AP started: %s (password: %s)", AP_SSID, AP_PASSWORD);
}

static bool connect_to_wifi(const char* ssid, const char* password) {
    ESP_LOGI(TAG, "Connecting to WiFi: %s", ssid);
    
    s_wifi_retry_count = 0;
    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
    
    wifi_config_t sta_config = {};
    strncpy((char*)sta_config.sta.ssid, ssid, sizeof(sta_config.sta.ssid));
    strncpy((char*)sta_config.sta.password, password, sizeof(sta_config.sta.password));
    sta_config.sta.threshold.authmode = strlen(password) > 0 ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
    esp_wifi_connect();
    
    // Wait for connection
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE, pdMS_TO_TICKS(15000));
    
    if (bits & WIFI_CONNECTED_BIT) {
        saveCredentials(ssid, password);
        return true;
    }
    return false;
}

// ============================================================
// AES Decryption for OTA
// ============================================================
struct OtaDecryptContext {
    mbedtls_aes_context aes;
    uint8_t iv[16];
    uint8_t buffer[AES_BLOCK_SIZE];
    size_t buffer_len;
    bool iv_read;
    esp_ota_handle_t ota_handle;
    const esp_partition_t* partition;
    size_t total_written;
    size_t total_size;
};

static OtaDecryptContext* s_decrypt_ctx = nullptr;

static bool ota_decrypt_init(size_t total_size) {
    s_decrypt_ctx = new OtaDecryptContext();
    memset(s_decrypt_ctx, 0, sizeof(OtaDecryptContext));
    
    mbedtls_aes_init(&s_decrypt_ctx->aes);
    mbedtls_aes_setkey_dec(&s_decrypt_ctx->aes, AES_KEY, AES_KEY_SIZE * 8);
    
    s_decrypt_ctx->partition = esp_ota_get_next_update_partition(NULL);
    if (!s_decrypt_ctx->partition) {
        ESP_LOGE(TAG, "No OTA partition available");
        delete s_decrypt_ctx;
        s_decrypt_ctx = nullptr;
        return false;
    }
    
    esp_err_t err = esp_ota_begin(s_decrypt_ctx->partition, OTA_WITH_SEQUENTIAL_WRITES, &s_decrypt_ctx->ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        delete s_decrypt_ctx;
        s_decrypt_ctx = nullptr;
        return false;
    }
    
    s_decrypt_ctx->total_size = total_size;
    ESP_LOGI(TAG, "OTA started, target: %s, encrypted size: %u", s_decrypt_ctx->partition->label, (unsigned)total_size);
    return true;
}

static bool ota_decrypt_write(const uint8_t* data, size_t len) {
    if (!s_decrypt_ctx) return false;
    
    size_t offset = 0;
    
    // First 16 bytes are IV
    if (!s_decrypt_ctx->iv_read) {
        size_t iv_needed = 16 - s_decrypt_ctx->buffer_len;
        size_t iv_copy = (len < iv_needed) ? len : iv_needed;
        memcpy(s_decrypt_ctx->buffer + s_decrypt_ctx->buffer_len, data, iv_copy);
        s_decrypt_ctx->buffer_len += iv_copy;
        offset += iv_copy;
        
        if (s_decrypt_ctx->buffer_len >= 16) {
            memcpy(s_decrypt_ctx->iv, s_decrypt_ctx->buffer, 16);
            s_decrypt_ctx->iv_read = true;
            s_decrypt_ctx->buffer_len = 0;
            ESP_LOGI(TAG, "IV extracted from encrypted firmware");
        }
        
        if (offset >= len) return true;
    }
    
    // Decrypt remaining data in 16-byte blocks
    const uint8_t* ptr = data + offset;
    size_t remaining = len - offset;
    
    // Add to buffer
    while (remaining > 0) {
        size_t to_copy = (remaining < (AES_BLOCK_SIZE - s_decrypt_ctx->buffer_len)) 
                        ? remaining : (AES_BLOCK_SIZE - s_decrypt_ctx->buffer_len);
        memcpy(s_decrypt_ctx->buffer + s_decrypt_ctx->buffer_len, ptr, to_copy);
        s_decrypt_ctx->buffer_len += to_copy;
        ptr += to_copy;
        remaining -= to_copy;
        
        // Process complete blocks
        if (s_decrypt_ctx->buffer_len == AES_BLOCK_SIZE) {
            uint8_t decrypted[AES_BLOCK_SIZE];
            mbedtls_aes_crypt_cbc(&s_decrypt_ctx->aes, MBEDTLS_AES_DECRYPT, 
                                  AES_BLOCK_SIZE, s_decrypt_ctx->iv, 
                                  s_decrypt_ctx->buffer, decrypted);
            
            esp_err_t err = esp_ota_write(s_decrypt_ctx->ota_handle, decrypted, AES_BLOCK_SIZE);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "OTA write failed: %s", esp_err_to_name(err));
                return false;
            }
            
            s_decrypt_ctx->total_written += AES_BLOCK_SIZE;
            s_decrypt_ctx->buffer_len = 0;
        }
    }
    
    return true;
}

static bool ota_decrypt_finish() {
    if (!s_decrypt_ctx) return false;
    
    // Handle any remaining data in buffer (should be padding)
    if (s_decrypt_ctx->buffer_len > 0) {
        // Pad to 16 bytes if needed
        while (s_decrypt_ctx->buffer_len < AES_BLOCK_SIZE) {
            s_decrypt_ctx->buffer[s_decrypt_ctx->buffer_len++] = 0;
        }
        
        uint8_t decrypted[AES_BLOCK_SIZE];
        mbedtls_aes_crypt_cbc(&s_decrypt_ctx->aes, MBEDTLS_AES_DECRYPT, 
                              AES_BLOCK_SIZE, s_decrypt_ctx->iv, 
                              s_decrypt_ctx->buffer, decrypted);
        
        // Check for PKCS7 padding and remove it
        uint8_t pad_len = decrypted[AES_BLOCK_SIZE - 1];
        if (pad_len > 0 && pad_len <= AES_BLOCK_SIZE) {
            size_t write_len = AES_BLOCK_SIZE - pad_len;
            if (write_len > 0) {
                esp_ota_write(s_decrypt_ctx->ota_handle, decrypted, write_len);
                s_decrypt_ctx->total_written += write_len;
            }
        }
    }
    
    esp_err_t err = esp_ota_end(s_decrypt_ctx->ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        mbedtls_aes_free(&s_decrypt_ctx->aes);
        delete s_decrypt_ctx;
        s_decrypt_ctx = nullptr;
        return false;
    }
    
    err = esp_ota_set_boot_partition(s_decrypt_ctx->partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        mbedtls_aes_free(&s_decrypt_ctx->aes);
        delete s_decrypt_ctx;
        s_decrypt_ctx = nullptr;
        return false;
    }
    
    ESP_LOGI(TAG, "OTA complete! Written %u bytes to %s", 
             (unsigned)s_decrypt_ctx->total_written, s_decrypt_ctx->partition->label);
    
    mbedtls_aes_free(&s_decrypt_ctx->aes);
    delete s_decrypt_ctx;
    s_decrypt_ctx = nullptr;
    
    return true;
}

static void ota_decrypt_abort() {
    if (s_decrypt_ctx) {
        esp_ota_abort(s_decrypt_ctx->ota_handle);
        mbedtls_aes_free(&s_decrypt_ctx->aes);
        delete s_decrypt_ctx;
        s_decrypt_ctx = nullptr;
    }
}

// ============================================================
// Version Comparison Helper
// ============================================================
static int compare_versions(const char* v1, const char* v2) {
    // Compare semantic versions (e.g., "1.2.3" vs "1.2.4")
    int major1 = 0, minor1 = 0, patch1 = 0;
    int major2 = 0, minor2 = 0, patch2 = 0;
    
    sscanf(v1, "%d.%d.%d", &major1, &minor1, &patch1);
    sscanf(v2, "%d.%d.%d", &major2, &minor2, &patch2);
    
    if (major1 != major2) return major1 - major2;
    if (minor1 != minor2) return minor1 - minor2;
    return patch1 - patch2;
}

// ============================================================
// Simple JSON Parser Helpers
// ============================================================
static bool json_get_string(const char* json, const char* key, char* value, size_t value_len) {
    // Find "key":"value" pattern
    char search[64];
    snprintf(search, sizeof(search), "\"%s\"", key);
    
    const char* key_pos = strstr(json, search);
    if (!key_pos) return false;
    
    // Find the colon after key
    const char* colon = strchr(key_pos + strlen(search), ':');
    if (!colon) return false;
    
    // Skip whitespace and find opening quote
    const char* p = colon + 1;
    while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') p++;
    
    if (*p != '"') return false;
    p++;  // Skip opening quote
    
    // Copy until closing quote
    size_t i = 0;
    while (*p && *p != '"' && i < value_len - 1) {
        value[i++] = *p++;
    }
    value[i] = '\0';
    
    return i > 0;
}

// ============================================================
// HTTP Helper - Download text content (with redirect following)
// ============================================================
static bool http_download_text(const char* url, char* buffer, size_t buffer_size) {
    esp_http_client_config_t config = {};
    config.url = url;
    config.timeout_ms = 30000;
    config.buffer_size = 2048;
    config.crt_bundle_attach = esp_crt_bundle_attach;
    config.max_redirection_count = 10;  // Follow up to 10 redirects
    
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "Failed to create HTTP client");
        return false;
    }
    
    // Use esp_http_client_perform which handles redirects automatically
    esp_err_t err = esp_http_client_perform(client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "HTTP request failed: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return false;
    }
    
    int status = esp_http_client_get_status_code(client);
    int content_length = esp_http_client_get_content_length(client);
    
    ESP_LOGI(TAG, "HTTP Status: %d, Content-Length: %d", status, content_length);
    
    if (status != 200) {
        ESP_LOGE(TAG, "HTTP error %d", status);
        esp_http_client_cleanup(client);
        return false;
    }
    
    // esp_http_client_perform already read the data, need to re-open for reading
    esp_http_client_cleanup(client);
    
    // Re-do with open/read pattern but with proper redirect handling
    config.disable_auto_redirect = false;
    client = esp_http_client_init(&config);
    if (!client) return false;
    
    // Set header to look like a browser
    esp_http_client_set_header(client, "User-Agent", "Mozilla/5.0");
    
    err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open connection: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return false;
    }
    
    int64_t total_len = esp_http_client_fetch_headers(client);
    status = esp_http_client_get_status_code(client);
    
    // Follow redirects manually if needed
    int redirect_count = 0;
    while ((status == 301 || status == 302 || status == 303 || status == 307 || status == 308) && redirect_count < 10) {
        redirect_count++;
        ESP_LOGI(TAG, "Following redirect %d, status: %d", redirect_count, status);
        
        err = esp_http_client_set_redirection(client);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set redirection: %s", esp_err_to_name(err));
            break;
        }
        
        err = esp_http_client_open(client, 0);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to open after redirect: %s", esp_err_to_name(err));
            esp_http_client_cleanup(client);
            return false;
        }
        
        total_len = esp_http_client_fetch_headers(client);
        status = esp_http_client_get_status_code(client);
    }
    
    ESP_LOGI(TAG, "Final HTTP Status: %d", status);
    
    if (status != 200) {
        ESP_LOGE(TAG, "HTTP error after redirects: %d", status);
        esp_http_client_cleanup(client);
        return false;
    }
    
    int total_read = 0;
    while (total_read < (int)buffer_size - 1) {
        int read = esp_http_client_read(client, buffer + total_read, buffer_size - 1 - total_read);
        if (read <= 0) break;
        total_read += read;
    }
    buffer[total_read] = '\0';
    
    ESP_LOGI(TAG, "Downloaded %d bytes", total_read);
    
    esp_http_client_cleanup(client);
    return total_read > 0;
}

// ============================================================
// Check for OTA Updates from Google Drive Folder
// ============================================================
static bool check_for_updates() {
    if (strlen(GDRIVE_LATEST_TXT_ID) == 0) {
        ESP_LOGE(TAG, "Google Drive latest.txt ID not configured");
        return false;
    }
    
    snprintf(s_ota_status, sizeof(s_ota_status), "Checking for updates...");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Starting OTA update check...");
    ESP_LOGI(TAG, "Google Drive latest.txt ID: %s", GDRIVE_LATEST_TXT_ID);
    
    // Download latest.txt from Google Drive
    // Format: VERSION,FILE_ID (e.g., "1.0.0,ABC123XYZ")
    char latest_url[256];
    snprintf(latest_url, sizeof(latest_url),
             "https://drive.google.com/uc?export=download&id=%s", 
             GDRIVE_LATEST_TXT_ID);
    
    ESP_LOGI(TAG, "Downloading latest.txt from: %s", latest_url);
    
    char* latest_buffer = (char*)malloc(256);
    if (!latest_buffer) {
        ESP_LOGE(TAG, "Out of memory");
        return false;
    }
    
    if (!http_download_text(latest_url, latest_buffer, 256)) {
        ESP_LOGE(TAG, "========================================");
        ESP_LOGE(TAG, "FAILED to download latest.txt!");
        ESP_LOGE(TAG, "Possible causes:");
        ESP_LOGE(TAG, "  1. ID is a FOLDER ID, not a FILE ID");
        ESP_LOGE(TAG, "  2. File is not shared (Anyone with link)");
        ESP_LOGE(TAG, "  3. Network/WiFi connection issue");
        ESP_LOGE(TAG, "========================================");
        free(latest_buffer);
        snprintf(s_ota_status, sizeof(s_ota_status), "Error: Failed to download latest.txt");
        return false;
    }
    
    ESP_LOGI(TAG, "SUCCESS: Downloaded latest.txt");
    ESP_LOGI(TAG, "Content (raw): [%s]", latest_buffer);
    
    // Parse latest.txt: "VERSION,FILE_ID"
    char* comma = strchr(latest_buffer, ',');
    if (!comma) {
        ESP_LOGE(TAG, "========================================");
        ESP_LOGE(TAG, "FAILED to parse latest.txt!");
        ESP_LOGE(TAG, "Expected format: VERSION,FILE_ID");
        ESP_LOGE(TAG, "Example: 1.0.0,ABC123XYZ");
        ESP_LOGE(TAG, "Got: %s", latest_buffer);
        ESP_LOGE(TAG, "========================================");
        free(latest_buffer);
        snprintf(s_ota_status, sizeof(s_ota_status), "Error: Invalid latest.txt format");
        return false;
    }
    
    // Extract version
    size_t version_len = comma - latest_buffer;
    if (version_len >= sizeof(s_available_version)) {
        version_len = sizeof(s_available_version) - 1;
    }
    strncpy(s_available_version, latest_buffer, version_len);
    s_available_version[version_len] = '\0';
    
    // Extract file_id (skip comma, trim whitespace/newlines)
    char* file_id_start = comma + 1;
    while (*file_id_start == ' ') file_id_start++;
    
    strncpy(s_firmware_file_id, file_id_start, sizeof(s_firmware_file_id) - 1);
    s_firmware_file_id[sizeof(s_firmware_file_id) - 1] = '\0';
    
    // Trim trailing whitespace/newlines
    size_t len = strlen(s_firmware_file_id);
    while (len > 0 && (s_firmware_file_id[len-1] == '\n' || s_firmware_file_id[len-1] == '\r' || s_firmware_file_id[len-1] == ' ')) {
        s_firmware_file_id[--len] = '\0';
    }
    
    free(latest_buffer);
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Parsed latest.txt successfully:");
    ESP_LOGI(TAG, "  Available version: %s", s_available_version);
    ESP_LOGI(TAG, "  Current version:   %s", CURRENT_FW_VERSION);
    ESP_LOGI(TAG, "  Firmware file ID:  %s", s_firmware_file_id);
    
    // Compare versions
    int cmp = compare_versions(s_available_version, CURRENT_FW_VERSION);
    s_update_available = (cmp > 0);
    
    if (s_update_available) {
        ESP_LOGI(TAG, "  Status: UPDATE AVAILABLE!");
        snprintf(s_ota_status, sizeof(s_ota_status), "Update available: v%s", s_available_version);
    } else if (cmp == 0) {
        ESP_LOGI(TAG, "  Status: Same version (recovery reinstall available)");
        snprintf(s_ota_status, sizeof(s_ota_status), "Current version: v%s (recovery available)", CURRENT_FW_VERSION);
    } else {
        ESP_LOGI(TAG, "  Status: Current version is newer");
        snprintf(s_ota_status, sizeof(s_ota_status), "Current version: v%s", CURRENT_FW_VERSION);
    }
    ESP_LOGI(TAG, "========================================");
    
    return true;
}


// ============================================================
// OTA Download Task
// ============================================================
static void ota_task(void* param) {
    s_ota_in_progress = true;
    s_ota_progress = 0;
    snprintf(s_ota_status, sizeof(s_ota_status), "Checking for updates...");
    led_set_mode(2);  // Downloading mode (cyan)
    led_set_progress(0);
    
    // First check for updates from Google Drive
    if (strlen(GDRIVE_LATEST_TXT_ID) == 0) {
        snprintf(s_ota_status, sizeof(s_ota_status), "Error: Google Drive not configured");
        led_set_mode(5);  // Error mode
        s_ota_in_progress = false;
        vTaskDelete(NULL);
        return;
    }
    
    // Check for available updates
    if (!check_for_updates()) {
        ESP_LOGE(TAG, "check_for_updates() failed - aborting OTA");
        led_set_mode(5);  // Error mode
        s_ota_in_progress = false;
        vTaskDelete(NULL);
        return;
    }
    
    // Check if we have a firmware file to download
    if (strlen(s_firmware_file_id) == 0) {
        ESP_LOGE(TAG, "No firmware file ID - aborting OTA");
        snprintf(s_ota_status, sizeof(s_ota_status), "Error: No firmware file available");
        led_set_mode(5);  // Error mode
        s_ota_in_progress = false;
        vTaskDelete(NULL);
        return;
    }
    
    // Show what we're going to do
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Starting firmware download...");
    if (s_update_available) {
        snprintf(s_ota_status, sizeof(s_ota_status), "Downloading v%s...", s_available_version);
        ESP_LOGI(TAG, "Action: UPGRADE to v%s", s_available_version);
    } else {
        snprintf(s_ota_status, sizeof(s_ota_status), "Recovery: Reinstalling v%s...", s_available_version);
        ESP_LOGI(TAG, "Action: REINSTALL v%s (recovery mode)", s_available_version);
    }
    
    // Build firmware download URL
    char firmware_url[256];
    snprintf(firmware_url, sizeof(firmware_url),
             "https://drive.google.com/uc?export=download&id=%s",
             s_firmware_file_id);
    
    ESP_LOGI(TAG, "Firmware file ID: %s", s_firmware_file_id);
    ESP_LOGI(TAG, "Firmware URL: %s", firmware_url);
    ESP_LOGI(TAG, "========================================");
    
    // Configure HTTP client with User-Agent to avoid Google Drive issues
    esp_http_client_config_t config = {};
    config.url = firmware_url;
    config.timeout_ms = 60000;  // Longer timeout for large files
    config.buffer_size = 4096;
    config.buffer_size_tx = 1024;
    config.crt_bundle_attach = esp_crt_bundle_attach;  // For HTTPS
    config.max_redirection_count = 10;
    
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "Failed to create HTTP client");
        snprintf(s_ota_status, sizeof(s_ota_status), "Error: Failed to create HTTP client");
        s_ota_in_progress = false;
        vTaskDelete(NULL);
        return;
    }
    
    // Set User-Agent to look like a browser
    esp_http_client_set_header(client, "User-Agent", "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36");
    
    ESP_LOGI(TAG, "Connecting to Google Drive...");
    
    // Open connection
    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "========================================");
        ESP_LOGE(TAG, "FAILED to connect: %s", esp_err_to_name(err));
        ESP_LOGE(TAG, "Possible causes:");
        ESP_LOGE(TAG, "  1. Network/WiFi issue");
        ESP_LOGE(TAG, "  2. HTTPS certificate issue");
        ESP_LOGE(TAG, "  3. Google Drive unavailable");
        ESP_LOGE(TAG, "========================================");
        snprintf(s_ota_status, sizeof(s_ota_status), "Error: Connection failed - %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        s_ota_in_progress = false;
        vTaskDelete(NULL);
        return;
    }
    
    // Get content length
    int content_length = esp_http_client_fetch_headers(client);
    int status_code = esp_http_client_get_status_code(client);
    
    ESP_LOGI(TAG, "HTTP Response: Status=%d, Content-Length=%d bytes", status_code, content_length);
    
    // Handle Google Drive redirects (may need multiple)
    int redirect_count = 0;
    while ((status_code == 303 || status_code == 302 || status_code == 301 || status_code == 307 || status_code == 308) && redirect_count < 10) {
        redirect_count++;
        ESP_LOGI(TAG, "Following redirect %d (status %d)...", redirect_count, status_code);
        
        err = esp_http_client_set_redirection(client);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set redirection: %s", esp_err_to_name(err));
            break;
        }
        
        err = esp_http_client_open(client, 0);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Redirect open failed: %s", esp_err_to_name(err));
            snprintf(s_ota_status, sizeof(s_ota_status), "Error: Redirect failed");
            esp_http_client_cleanup(client);
            s_ota_in_progress = false;
            vTaskDelete(NULL);
            return;
        }
        
        content_length = esp_http_client_fetch_headers(client);
        status_code = esp_http_client_get_status_code(client);
        ESP_LOGI(TAG, "After redirect %d: Status=%d, Content-Length=%d", redirect_count, status_code, content_length);
    }
    
    if (status_code != 200) {
        ESP_LOGE(TAG, "========================================");
        ESP_LOGE(TAG, "FAILED: HTTP error %d", status_code);
        if (status_code == 404) {
            ESP_LOGE(TAG, "File not found - check firmware FILE_ID");
        } else if (status_code == 403) {
            ESP_LOGE(TAG, "Access denied - file not shared properly");
        }
        ESP_LOGE(TAG, "========================================");
        snprintf(s_ota_status, sizeof(s_ota_status), "Error: HTTP %d", status_code);
        esp_http_client_cleanup(client);
        s_ota_in_progress = false;
        vTaskDelete(NULL);
        return;
    }
    
    if (content_length <= 0) {
        // Try chunked transfer
        content_length = 0;
        ESP_LOGW(TAG, "Unknown content length, proceeding anyway");
    }
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "SUCCESS: Connected to firmware file");
    ESP_LOGI(TAG, "Firmware size: %d bytes (%.1f KB)", content_length, content_length / 1024.0);
    ESP_LOGI(TAG, "Starting download and decryption...");
    ESP_LOGI(TAG, "========================================");
    snprintf(s_ota_status, sizeof(s_ota_status), "Downloading firmware...");
    
    // Initialize OTA with decryption
    size_t estimated_size = content_length > 0 ? content_length : 2 * 1024 * 1024;  // Default 2MB
    if (!ota_decrypt_init(estimated_size)) {
        ESP_LOGE(TAG, "Failed to initialize OTA partition");
        snprintf(s_ota_status, sizeof(s_ota_status), "Error: Failed to start OTA");
        esp_http_client_cleanup(client);
        s_ota_in_progress = false;
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "OTA partition initialized, starting download...");
    
    // Read and decrypt firmware data
    uint8_t* buffer = (uint8_t*)malloc(4096);
    if (!buffer) {
        ESP_LOGE(TAG, "Out of memory for download buffer");
        snprintf(s_ota_status, sizeof(s_ota_status), "Error: Out of memory");
        ota_decrypt_abort();
        esp_http_client_cleanup(client);
        s_ota_in_progress = false;
        vTaskDelete(NULL);
        return;
    }
    
    int total_read = 0;
    bool success = true;
    int last_progress_log = 0;
    
    while (true) {
        int read_len = esp_http_client_read(client, (char*)buffer, 4096);
        if (read_len < 0) {
            ESP_LOGE(TAG, "HTTP read error at offset %d", total_read);
            success = false;
            break;
        } else if (read_len == 0) {
            // End of stream
            ESP_LOGI(TAG, "Download complete: %d bytes received", total_read);
            break;
        }
        
        if (!ota_decrypt_write(buffer, read_len)) {
            ESP_LOGE(TAG, "Decryption/write error at offset %d", total_read);
            success = false;
            break;
        }
        
        total_read += read_len;
        if (content_length > 0) {
            s_ota_progress = (int)((int64_t)total_read * 100 / content_length);
            // Log progress every 10%
            if (s_ota_progress >= last_progress_log + 10) {
                last_progress_log = (s_ota_progress / 10) * 10;
                ESP_LOGI(TAG, "Download progress: %d%% (%d / %d KB)", 
                         s_ota_progress, total_read / 1024, content_length / 1024);
            }
        } else {
            // Estimate progress based on typical firmware size
            s_ota_progress = (int)((int64_t)total_read * 100 / (2 * 1024 * 1024));
            if (s_ota_progress > 99) s_ota_progress = 99;
        }
        led_set_mode(3);  // Flashing mode (orange)
        led_set_progress(s_ota_progress);
        snprintf(s_ota_status, sizeof(s_ota_status), "Downloading & flashing: %d%% (%d KB)", s_ota_progress, total_read / 1024);
    }
    
    free(buffer);
    esp_http_client_cleanup(client);
    
    ESP_LOGI(TAG, "========================================");
    if (success && total_read > 0 && ota_decrypt_finish()) {
        ESP_LOGI(TAG, "OTA UPDATE SUCCESSFUL!");
        ESP_LOGI(TAG, "Downloaded: %d bytes", total_read);
        ESP_LOGI(TAG, "Firmware installed to OTA partition");
        ESP_LOGI(TAG, "Restarting in 2 seconds...");
        ESP_LOGI(TAG, "========================================");
        snprintf(s_ota_status, sizeof(s_ota_status), "Update complete! Restarting...");
        s_ota_progress = 100;
        led_set_mode(4);  // Success mode (green checkmark)
        led_set_progress(100);
        vTaskDelay(pdMS_TO_TICKS(2000));
        esp_restart();
    } else {
        ESP_LOGE(TAG, "OTA UPDATE FAILED!");
        ESP_LOGE(TAG, "Downloaded: %d bytes", total_read);
        ESP_LOGE(TAG, "Possible causes:");
        ESP_LOGE(TAG, "  1. Decryption key mismatch");
        ESP_LOGE(TAG, "  2. Corrupted firmware file");
        ESP_LOGE(TAG, "  3. Flash write error");
        ESP_LOGE(TAG, "========================================");
        ota_decrypt_abort();
        snprintf(s_ota_status, sizeof(s_ota_status), "Error: Update failed");
        led_set_mode(5);  // Error mode (red X)
    }
    
    s_ota_in_progress = false;
    vTaskDelete(NULL);
}

// ============================================================
// HTTP Request Handlers
// ============================================================
static std::string replaceAll(std::string str, const std::string& from, const std::string& to) {
    size_t pos = 0;
    while ((pos = str.find(from, pos)) != std::string::npos) {
        str.replace(pos, from.length(), to);
        pos += to.length();
    }
    return str;
}

static esp_err_t handle_root(httpd_req_t* req) {
    ESP_LOGI(TAG, "HTTP: / (root) requested");
    std::string html = HTML_HEADER;
    html += HTML_WIFI_CONFIG;
    html += HTML_FOOTER;
    
    // Replace placeholders
    html = replaceAll(html, "%WIFI_STATUS_CLASS%", 
                      s_wifi_connected ? "status-connected" : "status-disconnected");
    html = replaceAll(html, "%WIFI_STATUS_TEXT%", 
                      s_wifi_connected ? "Connected to WiFi" : "Not connected - configure WiFi below");
    html = replaceAll(html, "%STORED_SSID%", s_stored_ssid);
    
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html.c_str(), html.length());
    return ESP_OK;
}

static esp_err_t handle_captive_portal(httpd_req_t* req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, CAPTIVE_PORTAL_REDIRECT, strlen(CAPTIVE_PORTAL_REDIRECT));
    return ESP_OK;
}

static esp_err_t handle_wifi_connect(httpd_req_t* req) {
    char buf[256];
    int received = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (received <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No data");
        return ESP_FAIL;
    }
    buf[received] = '\0';
    
    // Parse form data (URL encoded)
    char ssid[64] = {0};
    char password[64] = {0};
    
    // Simple URL decode and parse
    char* ssid_start = strstr(buf, "ssid=");
    char* pass_start = strstr(buf, "password=");
    
    if (ssid_start) {
        ssid_start += 5;
        char* end = strchr(ssid_start, '&');
        size_t len = end ? (size_t)(end - ssid_start) : strlen(ssid_start);
        if (len >= sizeof(ssid)) len = sizeof(ssid) - 1;
        
        // URL decode
        size_t j = 0;
        for (size_t i = 0; i < len && j < sizeof(ssid) - 1; i++, j++) {
            if (ssid_start[i] == '+') {
                ssid[j] = ' ';
            } else if (ssid_start[i] == '%' && i + 2 < len) {
                char hex[3] = {ssid_start[i+1], ssid_start[i+2], 0};
                ssid[j] = (char)strtol(hex, NULL, 16);
                i += 2;
            } else {
                ssid[j] = ssid_start[i];
            }
        }
    }
    
    if (pass_start) {
        pass_start += 9;
        char* end = strchr(pass_start, '&');
        size_t len = end ? (size_t)(end - pass_start) : strlen(pass_start);
        if (len >= sizeof(password)) len = sizeof(password) - 1;
        
        // URL decode
        size_t j = 0;
        for (size_t i = 0; i < len && j < sizeof(password) - 1; i++, j++) {
            if (pass_start[i] == '+') {
                password[j] = ' ';
            } else if (pass_start[i] == '%' && i + 2 < len) {
                char hex[3] = {pass_start[i+1], pass_start[i+2], 0};
                password[j] = (char)strtol(hex, NULL, 16);
                i += 2;
            } else {
                password[j] = pass_start[i];
            }
        }
    }
    
    ESP_LOGI(TAG, "Connecting to: %s", ssid);
    
    // Try to connect in background
    xTaskCreate([](void* param) {
        char* creds = (char*)param;
        char* sep = strchr(creds, '\n');
        if (sep) {
            *sep = '\0';
            connect_to_wifi(creds, sep + 1);
        }
        free(creds);
        vTaskDelete(NULL);
    }, "wifi_connect", 4096, strdup((std::string(ssid) + "\n" + password).c_str()), 5, NULL);
    
    // Redirect back to main page
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t handle_wifi_scan(httpd_req_t* req) {
    wifi_scan_config_t scan_config = {};
    scan_config.show_hidden = false;
    
    esp_wifi_scan_start(&scan_config, true);
    
    uint16_t ap_count = 0;
    esp_wifi_scan_get_ap_num(&ap_count);
    
    wifi_ap_record_t* ap_list = (wifi_ap_record_t*)malloc(ap_count * sizeof(wifi_ap_record_t));
    esp_wifi_scan_get_ap_records(&ap_count, ap_list);
    
    std::string json = "{\"networks\":[";
    for (int i = 0; i < ap_count && i < 20; i++) {
        if (i > 0) json += ",";
        json += "{\"ssid\":\"";
        // Escape special characters in SSID
        for (size_t j = 0; j < strlen((char*)ap_list[i].ssid); j++) {
            char c = ap_list[i].ssid[j];
            if (c == '"' || c == '\\') {
                json += '\\';
            }
            json += c;
        }
        json += "\",\"rssi\":";
        json += std::to_string(ap_list[i].rssi);
        json += "}";
    }
    json += "]}";
    
    free(ap_list);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json.c_str(), json.length());
    return ESP_OK;
}

static esp_err_t handle_ota_page(httpd_req_t* req) {
    ESP_LOGI(TAG, "HTTP: /ota/page requested");
    std::string html = HTML_OTA_PAGE;
    
    html = replaceAll(html, "%OTA_STATUS_CLASS%", 
                      s_wifi_connected ? "status-connected" : "status-disconnected");
    html = replaceAll(html, "%OTA_STATUS_TEXT%", 
                      s_wifi_connected ? "Ready for OTA update" : "Connect to WiFi first");
    html = replaceAll(html, "%CURRENT_VERSION%", CURRENT_FW_VERSION);
    
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html.c_str(), html.length());
    return ESP_OK;
}

static esp_err_t handle_ota_start(httpd_req_t* req) {
    ESP_LOGI(TAG, "HTTP: /ota/start requested");
    std::string json;
    
    if (!s_wifi_connected) {
        json = "{\"success\":false,\"message\":\"Connect to WiFi first\"}";
    } else if (s_ota_in_progress) {
        json = "{\"success\":false,\"message\":\"Update already in progress\"}";
    } else if (strlen(GDRIVE_LATEST_TXT_ID) == 0) {
        json = "{\"success\":false,\"message\":\"Google Drive not configured\"}";
    } else {
        // Start OTA task
        xTaskCreate(ota_task, "ota_task", 8192, NULL, 5, NULL);
        json = "{\"success\":true}";
    }
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json.c_str(), json.length());
    return ESP_OK;
}

static esp_err_t handle_ota_status(httpd_req_t* req) {
    char json[256];
    snprintf(json, sizeof(json), 
             "{\"progress\":%d,\"status\":\"%s\",\"complete\":%s,\"success\":%s}",
             s_ota_progress, s_ota_status,
             s_ota_in_progress ? "false" : "true",
             (s_ota_progress >= 100 && !s_ota_in_progress) ? "true" : "false");
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, strlen(json));
    return ESP_OK;
}

// ============================================================
// HTTP Server Initialization
// ============================================================
static void start_webserver() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 12;
    config.uri_match_fn = httpd_uri_match_wildcard;
    config.stack_size = 8192;
    
    if (httpd_start(&s_server, &config) == ESP_OK) {
        // Main page
        httpd_uri_t root = { .uri = "/", .method = HTTP_GET, .handler = handle_root };
        httpd_register_uri_handler(s_server, &root);
        
        // WiFi handlers
        httpd_uri_t wifi_connect = { .uri = "/wifi/connect", .method = HTTP_POST, .handler = handle_wifi_connect };
        httpd_register_uri_handler(s_server, &wifi_connect);
        
        httpd_uri_t wifi_scan = { .uri = "/wifi/scan", .method = HTTP_GET, .handler = handle_wifi_scan };
        httpd_register_uri_handler(s_server, &wifi_scan);
        
        // OTA handlers
        httpd_uri_t ota_page = { .uri = "/ota/page", .method = HTTP_GET, .handler = handle_ota_page };
        httpd_register_uri_handler(s_server, &ota_page);
        
        httpd_uri_t ota_start = { .uri = "/ota/start", .method = HTTP_POST, .handler = handle_ota_start };
        httpd_register_uri_handler(s_server, &ota_start);
        
        httpd_uri_t ota_status = { .uri = "/ota/status", .method = HTTP_GET, .handler = handle_ota_status };
        httpd_register_uri_handler(s_server, &ota_status);
        
        // Captive portal detection endpoints
        httpd_uri_t generate204 = { .uri = "/generate_204", .method = HTTP_GET, .handler = handle_captive_portal };
        httpd_register_uri_handler(s_server, &generate204);
        
        httpd_uri_t hotspot = { .uri = "/hotspot-detect.html", .method = HTTP_GET, .handler = handle_captive_portal };
        httpd_register_uri_handler(s_server, &hotspot);
        
        httpd_uri_t ncsi = { .uri = "/ncsi.txt", .method = HTTP_GET, .handler = handle_captive_portal };
        httpd_register_uri_handler(s_server, &ncsi);
        
        httpd_uri_t connecttest = { .uri = "/connecttest.txt", .method = HTTP_GET, .handler = handle_captive_portal };
        httpd_register_uri_handler(s_server, &connecttest);
        
        // Catch-all for captive portal
        httpd_uri_t catchall = { .uri = "/*", .method = HTTP_GET, .handler = handle_captive_portal };
        httpd_register_uri_handler(s_server, &catchall);
        
        ESP_LOGI(TAG, "Web server started on http://192.168.4.1");
    }
}

// ============================================================
// Main
// ============================================================
extern "C" void app_main(void) {
    // ----------------------------------------------------------------
    // Boot check: Only stay in recovery if GPIO 18 is held
    // Otherwise, return to main firmware partition
    // ----------------------------------------------------------------
    gpio_config_t btn_config = {
        .pin_bit_mask = (1ULL << RECOVERY_BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&btn_config);
    
    // Small delay for GPIO to stabilize
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Check if recovery button is being held (active LOW)
    bool button_held = (gpio_get_level(RECOVERY_BUTTON_GPIO) == 0);
    
    if (!button_held) {
        ESP_LOGW(TAG, "Recovery button NOT held - returning to main firmware");
        
        // Find the main app partition (try ota_0 first, then ota_1)
        const esp_partition_t* main_app = esp_partition_find_first(
            ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);
        
        if (!main_app) {
            main_app = esp_partition_find_first(
                ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_1, NULL);
        }
        
        if (main_app) {
            ESP_LOGI(TAG, "Setting boot partition to: %s @ 0x%08" PRIx32,
                     main_app->label, (uint32_t)main_app->address);
            
            esp_err_t err = esp_ota_set_boot_partition(main_app);
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "Rebooting to main firmware...");
                vTaskDelay(pdMS_TO_TICKS(500));
                esp_restart();
            } else {
                ESP_LOGE(TAG, "Failed to set boot partition: %s", esp_err_to_name(err));
            }
        } else {
            ESP_LOGE(TAG, "No main app partition found! Staying in recovery.");
        }
    }
    
    // Button is held - stay in recovery mode
    ESP_LOGI(TAG, "===================================");
    ESP_LOGI(TAG, " ESP32 Recovery Mode (WiFi OTA)");
    ESP_LOGI(TAG, " Version: %s", RECOVERY_VERSION);
    ESP_LOGI(TAG, "===================================");
    
    // Initialize LED matrix for status display
    if (led_init() == ESP_OK) {
        // Start LED effect task
        xTaskCreate(led_effect_task, "led_effect", 4096, NULL, 3, &s_led_task_handle);
        led_set_mode(1);  // WiFi waiting mode (blue spinning)
    }
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Load stored WiFi credentials
    loadStoredCredentials();
    
    // Show partition info
    const esp_partition_t* running = esp_ota_get_running_partition();
    if (running) {
        ESP_LOGI(TAG, "Running: %s @ 0x%08" PRIx32, running->label, (uint32_t)running->address);
    }
    
    const esp_partition_t* next = esp_ota_get_next_update_partition(NULL);
    if (next) {
        ESP_LOGI(TAG, "OTA target: %s (%u KB)", next->label, (unsigned)(next->size / 1024));
    }
    
    // Initialize WiFi
    init_wifi();
    
    // Start DNS server for captive portal
    start_dns_server();
    
    // Start web server
    start_webserver();
    
    // Try to connect to stored WiFi if available
    if (strlen(s_stored_ssid) > 0) {
        ESP_LOGI(TAG, "Attempting to connect to stored WiFi: %s", s_stored_ssid);
        // Connect in background so we don't block
        xTaskCreate([](void* param) {
            vTaskDelay(pdMS_TO_TICKS(2000));  // Wait for AP to stabilize
            connect_to_wifi(s_stored_ssid, s_stored_pass);
            vTaskDelete(NULL);
        }, "auto_connect", 4096, NULL, 5, NULL);
    }
    
    ESP_LOGI(TAG, "Ready!");
    ESP_LOGI(TAG, "Connect to WiFi: %s (password: %s)", AP_SSID, AP_PASSWORD);
    ESP_LOGI(TAG, "Then open http://192.168.4.1 in your browser");
    
    // Main loop - just keep running
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
