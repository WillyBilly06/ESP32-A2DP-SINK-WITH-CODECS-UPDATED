#pragma once

// -----------------------------------------------------------
// Unified BLE Protocol - Single command/status/meter architecture
// All control through one command characteristic
// All status through one status characteristic  
// Real-time meters through dedicated meter characteristic
// -----------------------------------------------------------

#include <stdint.h>
#include <string.h>
#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include "../config/app_config.h"

// Protocol Command IDs (Phone -> ESP32)
namespace BleCmd {
    constexpr uint8_t SET_EQ           = 0x01;  // [bass, mid, treble] 3 bytes signed
    constexpr uint8_t SET_EQ_PRESET    = 0x02;  // [preset_id] 1 byte
    constexpr uint8_t SET_CONTROL      = 0x03;  // [control_byte] 1 byte
    constexpr uint8_t SET_NAME         = 0x04;  // [name...] 1-32 bytes
    constexpr uint8_t SET_LED          = 0x05;  // [effect, bright, speed, r1,g1,b1, r2,g2,b2, gradient] 10 bytes
    constexpr uint8_t SET_LED_EFFECT   = 0x06;  // [effect_id] 1 byte
    constexpr uint8_t SET_LED_BRIGHT   = 0x07;  // [brightness] 1 byte
    
    constexpr uint8_t SOUND_MUTE       = 0x10;  // [0/1] 1 byte
    constexpr uint8_t SOUND_DELETE     = 0x11;  // [type] 1 byte
    constexpr uint8_t SOUND_UP_START   = 0x12;  // [type, size_lo, size_mid, size_hi] 4 bytes
    constexpr uint8_t SOUND_UP_DATA    = 0x13;  // [seq, data...] 1+N bytes
    constexpr uint8_t SOUND_UP_END     = 0x14;  // no payload
    
    constexpr uint8_t OTA_BEGIN        = 0x20;  // [size_lo, size_mid, size_hi, size_hhi] 4 bytes
    constexpr uint8_t OTA_DATA         = 0x21;  // [seq, data...] 1+N bytes
    constexpr uint8_t OTA_END          = 0x22;  // no payload
    constexpr uint8_t OTA_ABORT        = 0x23;  // no payload
    
    constexpr uint8_t REQUEST_STATUS   = 0xF0;  // no payload - request full sync
    constexpr uint8_t PING             = 0xFF;  // no payload
}

// Protocol Response IDs (ESP32 -> Phone)
namespace BleResp {
    constexpr uint8_t STATUS_EQ        = 0x01;  // [bass, mid, treble] 3 bytes
    constexpr uint8_t STATUS_CONTROL   = 0x02;  // [control_byte] 1 byte
    constexpr uint8_t STATUS_NAME      = 0x03;  // [name...] 1-32 bytes
    constexpr uint8_t STATUS_FW        = 0x04;  // [version...] string
    constexpr uint8_t STATUS_LED       = 0x05;  // [effect, bright, speed, r1,g1,b1, r2,g2,b2, gradient] 10 bytes
    constexpr uint8_t STATUS_SOUND     = 0x06;  // [status_byte] 1 byte
    
    constexpr uint8_t ACK_OK           = 0x10;  // [cmd] 1 byte
    constexpr uint8_t ACK_ERROR        = 0x11;  // [cmd, error_code] 2 bytes
    
    constexpr uint8_t OTA_PROGRESS     = 0x20;  // [percent] 1 byte
    constexpr uint8_t OTA_READY        = 0x21;  // no payload - ready for next chunk
    constexpr uint8_t OTA_COMPLETE     = 0x22;  // no payload
    constexpr uint8_t OTA_FAILED       = 0x23;  // [error_code] 1 byte
    
    constexpr uint8_t SOUND_PROGRESS   = 0x30;  // [percent] 1 byte
    constexpr uint8_t SOUND_READY      = 0x31;  // no payload - ready for next chunk
    constexpr uint8_t SOUND_COMPLETE   = 0x32;  // no payload
    constexpr uint8_t SOUND_FAILED     = 0x33;  // [error_code] 1 byte
    
    constexpr uint8_t FULL_STATUS      = 0xF0;  // full status dump
    constexpr uint8_t PONG             = 0xFF;  // ping response
}

// Error codes
namespace BleError {
    constexpr uint8_t NONE             = 0x00;
    constexpr uint8_t INVALID_CMD      = 0x01;
    constexpr uint8_t INVALID_PARAM    = 0x02;
    constexpr uint8_t BUSY             = 0x03;
    constexpr uint8_t OTA_INIT_FAIL    = 0x10;
    constexpr uint8_t OTA_WRITE_FAIL   = 0x11;
    constexpr uint8_t OTA_VERIFY_FAIL  = 0x12;
    constexpr uint8_t SOUND_INIT_FAIL  = 0x20;
    constexpr uint8_t SOUND_WRITE_FAIL = 0x21;
}

// Forward declaration
class BleUnifiedService;
static BleUnifiedService* s_bleUnifiedInstance = nullptr;

class BleUnifiedService {
public:
    // Callbacks for command handling
    using EqCallback = void(*)(int8_t bass, int8_t mid, int8_t treble);
    using EqPresetCallback = void(*)(uint8_t presetId);
    using ControlCallback = void(*)(uint8_t controlByte);
    using NameCallback = void(*)(const char* name, size_t len);
    using LedCallback = void(*)(const uint8_t* data, size_t len);
    using LedEffectCallback = void(*)(uint8_t effectId);
    using LedBrightnessCallback = void(*)(uint8_t brightness);
    using SoundMuteCallback = void(*)(bool muted);
    using SoundDeleteCallback = void(*)(uint8_t soundType);
    using SoundDataCallback = void(*)(uint8_t cmd, const uint8_t* data, size_t len);
    using OtaCallback = void(*)(uint8_t cmd, const uint8_t* data, size_t len);

    BleUnifiedService()
        : m_gattsIf(0)
        , m_connId(0)
        , m_connected(false)
        , m_mtu(23)  // Default BLE MTU
        , m_mtuExchanged(false)
        , m_advConfigDone(0)
        , m_serviceHandle(0)
        , m_cmdCharHandle(0)
        , m_statusCharHandle(0)
        , m_meterCharHandle(0)
        // Callbacks
        , m_eqCb(nullptr)
        , m_eqPresetCb(nullptr)
        , m_controlCb(nullptr)
        , m_nameCb(nullptr)
        , m_ledCb(nullptr)
        , m_ledEffectCb(nullptr)
        , m_ledBrightCb(nullptr)
        , m_soundMuteCb(nullptr)
        , m_soundDeleteCb(nullptr)
        , m_soundDataCb(nullptr)
        , m_otaCb(nullptr)
    {
        memset(m_uuidService, 0, 16);
        memset(m_uuidCmdChar, 0, 16);
        memset(m_uuidStatusChar, 0, 16);
        memset(m_uuidMeterChar, 0, 16);
        
        // Initialize state
        memset(m_eqValue, 0, sizeof(m_eqValue));
        m_controlValue = 0;
        memset(m_nameValue, 0, sizeof(m_nameValue));
        memset(m_fwValue, 0, sizeof(m_fwValue));
        memset(m_ledValue, 0, sizeof(m_ledValue));
        m_soundStatus = 0;
        
        // Default LED values
        m_ledValue[1] = 128;  // brightness
    }

    void setCallbacks(
        EqCallback eqCb,
        EqPresetCallback eqPresetCb,
        ControlCallback controlCb,
        NameCallback nameCb,
        LedCallback ledCb,
        LedEffectCallback ledEffectCb,
        LedBrightnessCallback ledBrightCb,
        SoundMuteCallback soundMuteCb,
        SoundDeleteCallback soundDeleteCb,
        SoundDataCallback soundDataCb,
        OtaCallback otaCb
    ) {
        m_eqCb = eqCb;
        m_eqPresetCb = eqPresetCb;
        m_controlCb = controlCb;
        m_nameCb = nameCb;
        m_ledCb = ledCb;
        m_ledEffectCb = ledEffectCb;
        m_ledBrightCb = ledBrightCb;
        m_soundMuteCb = soundMuteCb;
        m_soundDeleteCb = soundDeleteCb;
        m_soundDataCb = soundDataCb;
        m_otaCb = otaCb;
    }

    bool init(const char* deviceName, const char* fwVersion,
              uint8_t controlByte, int8_t bassDb, int8_t midDb, int8_t trebleDb,
              uint8_t ledEffect = 0, uint8_t brightness = 128) {
        
        s_bleUnifiedInstance = this;
        
        // Store initial state
        strncpy(m_nameValue, deviceName, sizeof(m_nameValue) - 1);
        strncpy(m_fwValue, fwVersion, sizeof(m_fwValue) - 1);
        m_controlValue = controlByte;
        m_eqValue[0] = bassDb;
        m_eqValue[1] = midDb;
        m_eqValue[2] = trebleDb;
        
        // LED values use internal format: [brightness, r1, g1, b1, r2, g2, b2, gradient, speed, effectId]
        // Initialize with defaults - will be overwritten by setLedValue() after init
        memset(m_ledValue, 0, sizeof(m_ledValue));
        m_ledValue[0] = brightness;  // brightness at index 0
        m_ledValue[9] = ledEffect;   // effectId at index 9

        // Parse UUIDs - New unified UUIDs
        uuid128FromString(BLE_UNIFIED_SERVICE_UUID, m_uuidService);
        uuid128FromString(BLE_UNIFIED_CHAR_CMD, m_uuidCmdChar);
        uuid128FromString(BLE_UNIFIED_CHAR_STATUS, m_uuidStatusChar);
        uuid128FromString(BLE_UNIFIED_CHAR_METER, m_uuidMeterChar);

        // Init Bluetooth controller
        esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
        ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BTDM));
        ESP_ERROR_CHECK(esp_bluedroid_init());
        ESP_ERROR_CHECK(esp_bluedroid_enable());
        
        // Set maximum MTU for faster transfers
        esp_err_t mtu_err = esp_ble_gatt_set_local_mtu(517);
        if (mtu_err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to set local MTU to 517: %s", esp_err_to_name(mtu_err));
        } else {
            ESP_LOGI(TAG, "BLE local MTU set to 517 bytes");
        }

        // Register callbacks
        ESP_ERROR_CHECK(esp_ble_gap_register_callback(gapEventHandler));
        ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gattsEventHandler));
        ESP_ERROR_CHECK(esp_ble_gatts_app_register(0));

        ESP_LOGI(TAG, "BLE Unified GATT initialized");
        return true;
    }

    bool isConnected() const { return m_connected; }

    // ========== Update state and notify ==========
    
    void updateLevels(uint8_t l30, uint8_t l60, uint8_t l100) {
        if (m_connected && m_meterCharHandle && m_gattsIf) {
            uint8_t data[3] = { l30, l60, l100 };
            esp_err_t err = esp_ble_gatts_send_indicate(m_gattsIf, m_connId, m_meterCharHandle, 3, data, false);
            if (err != ESP_OK) {
                // Connection likely dead, mark as disconnected
                ESP_LOGW(TAG, "Meter notify failed: %s - marking disconnected", esp_err_to_name(err));
                m_connected = false;
            }
        }
    }

    void updateEq(int8_t bass, int8_t mid, int8_t treble) {
        m_eqValue[0] = bass;
        m_eqValue[1] = mid;
        m_eqValue[2] = treble;
        notifyStatus(BleResp::STATUS_EQ, (uint8_t*)m_eqValue, 3);
    }

    void updateControl(uint8_t controlByte) {
        m_controlValue = controlByte;
        notifyStatus(BleResp::STATUS_CONTROL, &m_controlValue, 1);
    }

    void updateName(const char* name) {
        strncpy(m_nameValue, name, sizeof(m_nameValue) - 1);
        m_nameValue[sizeof(m_nameValue) - 1] = '\0';
        notifyStatus(BleResp::STATUS_NAME, (uint8_t*)m_nameValue, strlen(m_nameValue));
    }

    void updateFw(const char* fw) {
        strncpy(m_fwValue, fw, sizeof(m_fwValue) - 1);
        m_fwValue[sizeof(m_fwValue) - 1] = '\0';
        notifyStatus(BleResp::STATUS_FW, (uint8_t*)m_fwValue, strlen(m_fwValue));
    }

    // Fast brightness conversion: 0-255 -> 0-100 using multiply+shift (no division)
    // Formula: (v * 25 + 32) >> 6 ≈ v * 0.3906 with rounding
    // Endpoints: 0→0, 255→100
    static inline uint8_t brightness255to100(uint8_t v) {
        return (v * 25 + 32) >> 6;
    }
    
    void updateLed(const uint8_t* data, size_t len) {
        // data is in old internal format: [brightness(0-255), r1, g1, b1, r2, g2, b2, gradient, speed, effectId]
        if (len > sizeof(m_ledValue)) len = sizeof(m_ledValue);
        memcpy(m_ledValue, data, len);
        
        // Convert to unified format for sending: [effect, bright(0-100), speed, r1, g1, b1, r2, g2, b2, gradient]
        // Map brightness 0-255 to 0-100 using fast multiply+shift
        uint8_t unified[10];
        unified[0] = m_ledValue[9];  // effectId
        unified[1] = brightness255to100(m_ledValue[0]);  // brightness 0-255 -> 0-100
        unified[2] = m_ledValue[8];  // speed
        unified[3] = m_ledValue[1];  // r1
        unified[4] = m_ledValue[2];  // g1
        unified[5] = m_ledValue[3];  // b1
        unified[6] = m_ledValue[4];  // r2
        unified[7] = m_ledValue[5];  // g2
        unified[8] = m_ledValue[6];  // b2
        unified[9] = m_ledValue[7];  // gradient
        
        notifyStatus(BleResp::STATUS_LED, unified, 10);
    }

    void updateLedEffect(uint8_t effectId) {
        m_ledValue[9] = effectId;  // effectId in old format is at index 9
        
        // Convert to unified format - map brightness 0-255 to 0-100 using fast multiply+shift
        uint8_t unified[10];
        unified[0] = m_ledValue[9];  // effectId
        unified[1] = brightness255to100(m_ledValue[0]);  // brightness 0-255 -> 0-100
        unified[2] = m_ledValue[8];  // speed
        unified[3] = m_ledValue[1];  // r1
        unified[4] = m_ledValue[2];  // g1
        unified[5] = m_ledValue[3];  // b1
        unified[6] = m_ledValue[4];  // r2
        unified[7] = m_ledValue[5];  // g2
        unified[8] = m_ledValue[6];  // b2
        unified[9] = m_ledValue[7];  // gradient
        
        notifyStatus(BleResp::STATUS_LED, unified, 10);
    }

    void updateLedBrightness(uint8_t brightness) {
        m_ledValue[0] = brightness;  // brightness in old format is at index 0
        
        // Convert to unified format - map brightness 0-255 to 0-100 using fast multiply+shift
        uint8_t unified[10];
        unified[0] = m_ledValue[9];  // effectId
        unified[1] = brightness255to100(brightness);  // brightness 0-255 -> 0-100
        unified[2] = m_ledValue[8];  // speed
        unified[3] = m_ledValue[1];  // r1
        unified[4] = m_ledValue[2];  // g1
        unified[5] = m_ledValue[3];  // b1
        unified[6] = m_ledValue[4];  // r2
        unified[7] = m_ledValue[5];  // g2
        unified[8] = m_ledValue[6];  // b2
        unified[9] = m_ledValue[7];  // gradient
        
        notifyStatus(BleResp::STATUS_LED, unified, 10);
    }

    void updateSoundStatus(uint8_t status) {
        ESP_LOGI(TAG, "updateSoundStatus: status=0x%02X, connected=%d", status, m_connected);
        m_soundStatus = status;
        notifyStatus(BleResp::STATUS_SOUND, &m_soundStatus, 1);
    }

    // ========== Response helpers ==========
    
    void sendAck(uint8_t cmd) {
        notifyStatus(BleResp::ACK_OK, &cmd, 1);
    }

    void sendError(uint8_t cmd, uint8_t errorCode) {
        uint8_t data[2] = { cmd, errorCode };
        notifyStatus(BleResp::ACK_ERROR, data, 2);
    }

    void sendOtaProgress(uint8_t percent) {
        notifyStatus(BleResp::OTA_PROGRESS, &percent, 1);
    }

    void sendOtaReady() {
        notifyStatus(BleResp::OTA_READY, nullptr, 0);
    }

    void sendOtaComplete() {
        notifyStatus(BleResp::OTA_COMPLETE, nullptr, 0);
    }

    void sendOtaFailed(uint8_t errorCode) {
        notifyStatus(BleResp::OTA_FAILED, &errorCode, 1);
    }

    void sendSoundProgress(uint8_t percent) {
        notifyStatus(BleResp::SOUND_PROGRESS, &percent, 1);
    }

    void sendSoundReady() {
        notifyStatus(BleResp::SOUND_READY, nullptr, 0);
    }

    void sendSoundComplete() {
        notifyStatus(BleResp::SOUND_COMPLETE, nullptr, 0);
    }

    void sendSoundFailed(uint8_t errorCode) {
        notifyStatus(BleResp::SOUND_FAILED, &errorCode, 1);
    }

    void sendPong() {
        notifyStatus(BleResp::PONG, nullptr, 0);
    }

    void sendFullStatus() {
        // Build full status packet:
        // [resp_id, bass, mid, treble, control, led[10], sound, name_len, name..., fw_len, fw...]
        // LED is sent in unified format: [effect, bright, speed, r1, g1, b1, r2, g2, b2, gradient]
        // Brightness is mapped from 0-255 (ESP32) to 0-100 (phone)
        uint8_t buffer[64];
        size_t idx = 0;
        
        buffer[idx++] = BleResp::FULL_STATUS;
        
        // EQ (3 bytes)
        buffer[idx++] = m_eqValue[0];
        buffer[idx++] = m_eqValue[1];
        buffer[idx++] = m_eqValue[2];
        
        // Control (1 byte)
        buffer[idx++] = m_controlValue;
        
        // LED (10 bytes) - convert from internal to unified format
        // Internal: [brightness, r1, g1, b1, r2, g2, b2, gradient, speed, effectId]
        // Unified:  [effect, bright, speed, r1, g1, b1, r2, g2, b2, gradient]
        // Map brightness/speed 0-255 to 0-100: (v * 100 + 127) / 255 (exact endpoints: 0→0, 255→100)
        buffer[idx++] = m_ledValue[9];  // effectId
        buffer[idx++] = (m_ledValue[0] * 100 + 127) / 255;  // brightness 0-255 -> 0-100
        buffer[idx++] = m_ledValue[8];  // speed
        buffer[idx++] = m_ledValue[1];  // r1
        buffer[idx++] = m_ledValue[2];  // g1
        buffer[idx++] = m_ledValue[3];  // b1
        buffer[idx++] = m_ledValue[4];  // r2
        buffer[idx++] = m_ledValue[5];  // g2
        buffer[idx++] = m_ledValue[6];  // b2
        buffer[idx++] = m_ledValue[7];  // gradient
        
        // Sound status (1 byte)
        buffer[idx++] = m_soundStatus;
        
        // Name (length + string)
        size_t nameLen = strlen(m_nameValue);
        buffer[idx++] = (uint8_t)nameLen;
        memcpy(&buffer[idx], m_nameValue, nameLen);
        idx += nameLen;
        
        // FW (length + string)
        size_t fwLen = strlen(m_fwValue);
        buffer[idx++] = (uint8_t)fwLen;
        memcpy(&buffer[idx], m_fwValue, fwLen);
        idx += fwLen;
        
        if (m_connected && m_statusCharHandle && m_gattsIf) {
            esp_ble_gatts_send_indicate(m_gattsIf, m_connId, m_statusCharHandle, idx, buffer, false);
        }
    }

    // Getters
    uint8_t getControlValue() const { return m_controlValue; }
    const int8_t* getEqValue() const { return m_eqValue; }
    const uint8_t* getLedValue() const { return m_ledValue; }
    uint8_t getSoundStatus() const { return m_soundStatus; }

    // Setters for initialization
    void setSoundStatus(uint8_t status) { m_soundStatus = status; }
    void setLedValue(const uint8_t* data, size_t len) {
        if (len > sizeof(m_ledValue)) len = sizeof(m_ledValue);
        memcpy(m_ledValue, data, len);
    }

    // Compatibility method for legacy OTA notifications (maps ASCII to unified responses)
    void notifyOtaCtrl(const char* msg) {
        if (strcmp(msg, "BEGIN_OK") == 0) {
            sendOtaReady();
        } else if (strcmp(msg, "BEGIN_ERR") == 0) {
            sendOtaFailed(BleError::OTA_INIT_FAIL);
        } else if (strcmp(msg, "END_OK") == 0) {
            sendOtaComplete();
        } else if (strcmp(msg, "END_ERR") == 0) {
            sendOtaFailed(BleError::OTA_VERIFY_FAIL);
        } else if (strcmp(msg, "CHECK_OK") == 0) {
            // CHECK_OK - ready for END
            sendOtaReady();
        } else if (strncmp(msg, "CHECK_FAIL:", 11) == 0) {
            // CHECK_FAIL:xxx - partial failure
            sendOtaFailed(BleError::OTA_WRITE_FAIL);
        } else if (strcmp(msg, "ABORT_OK") == 0) {
            // Abort acknowledged
            sendAck(BleCmd::OTA_ABORT);
        } else {
            ESP_LOGW(TAG, "Unknown OTA ctrl message: %s", msg);
        }
    }

private:
    static constexpr const char* TAG = "BLE_U";
    static constexpr uint8_t ADV_CONFIG_FLAG = 0x01;
    static constexpr uint8_t SCAN_RSP_CONFIG_FLAG = 0x02;

    void notifyStatus(uint8_t respId, const uint8_t* data, size_t len) {
        if (!m_connected || !m_statusCharHandle || !m_gattsIf) {
            ESP_LOGW(TAG, "notifyStatus(0x%02X) failed: connected=%d, handle=%d, gattsIf=%d",
                     respId, m_connected, m_statusCharHandle, m_gattsIf);
            return;
        }
        
        uint8_t buffer[256];
        buffer[0] = respId;
        if (data && len > 0) {
            memcpy(&buffer[1], data, len);
        }
        esp_err_t err = esp_ble_gatts_send_indicate(m_gattsIf, m_connId, m_statusCharHandle, 1 + len, buffer, false);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "notifyStatus(0x%02X) send failed: %s - marking disconnected", respId, esp_err_to_name(err));
            // Connection likely dead, mark as disconnected to stop further attempts
            m_connected = false;
        }
    }

    // UUID parsing helper
    static void uuid128FromString(const char* str, uint8_t* out) {
        uint8_t temp[16];
        memset(temp, 0, 16);
        int idx = 0;
        for (int i = 0; str[i] && idx < 32; i++) {
            char c = str[i];
            if (c == '-') continue;
            int val = 0;
            if (c >= '0' && c <= '9') val = c - '0';
            else if (c >= 'a' && c <= 'f') val = c - 'a' + 10;
            else if (c >= 'A' && c <= 'F') val = c - 'A' + 10;
            else continue;
            
            if ((idx & 1) == 0) {
                temp[idx >> 1] = (uint8_t)(val << 4);
            } else {
                temp[idx >> 1] |= (uint8_t)val;
            }
            idx++;
        }
        // Reverse for little-endian
        for (int i = 0; i < 16; i++) {
            out[i] = temp[15 - i];
        }
    }

    static bool uuidEqual128(const esp_bt_uuid_t& uuid, const uint8_t* raw) {
        if (uuid.len != ESP_UUID_LEN_128) return false;
        return memcmp(uuid.uuid.uuid128, raw, 16) == 0;
    }

    static bool uuidEqual128Raw(const uint8_t* a, const uint8_t* b) {
        return memcmp(a, b, 16) == 0;
    }

    // Static callback wrappers
    static void gapEventHandler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param) {
        if (s_bleUnifiedInstance) s_bleUnifiedInstance->handleGapEvent(event, param);
    }

    static void gattsEventHandler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                  esp_ble_gatts_cb_param_t* param) {
        if (s_bleUnifiedInstance) s_bleUnifiedInstance->handleGattsEvent(event, gatts_if, param);
    }

    void handleGapEvent(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param) {
        switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "GAP: ADV_DATA_SET_COMPLETE");
            m_advConfigDone &= ~ADV_CONFIG_FLAG;
            if (m_advConfigDone == 0) startAdvertising();
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "GAP: SCAN_RSP_DATA_SET_COMPLETE");
            m_advConfigDone &= ~SCAN_RSP_CONFIG_FLAG;
            if (m_advConfigDone == 0) startAdvertising();
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "Advertising start failed: 0x%x", param->adv_start_cmpl.status);
            } else {
                ESP_LOGI(TAG, "Advertising started");
            }
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(TAG, "GAP: CONN_PARAMS update, status=%d, bda=%02x:%02x:%02x:%02x:%02x:%02x",
                     param->update_conn_params.status,
                     param->update_conn_params.bda[0], param->update_conn_params.bda[1],
                     param->update_conn_params.bda[2], param->update_conn_params.bda[3],
                     param->update_conn_params.bda[4], param->update_conn_params.bda[5]);
            break;
        case ESP_GAP_BLE_SEC_REQ_EVT:
            ESP_LOGI(TAG, "GAP: SEC_REQ from %02x:%02x:%02x:%02x:%02x:%02x",
                     param->ble_security.ble_req.bd_addr[0], param->ble_security.ble_req.bd_addr[1],
                     param->ble_security.ble_req.bd_addr[2], param->ble_security.ble_req.bd_addr[3],
                     param->ble_security.ble_req.bd_addr[4], param->ble_security.ble_req.bd_addr[5]);
            // Accept security request
            esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
            break;
        case ESP_GAP_BLE_AUTH_CMPL_EVT:
            ESP_LOGI(TAG, "GAP: AUTH_CMPL, success=%d, fail_reason=%d",
                     param->ble_security.auth_cmpl.success,
                     param->ble_security.auth_cmpl.fail_reason);
            break;
        default:
            ESP_LOGD(TAG, "GAP event: %d", event);
            break;
        }
    }

    void handleGattsEvent(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                          esp_ble_gatts_cb_param_t* param) {
        switch (event) {
        case ESP_GATTS_REG_EVT:
            handleRegEvent(gatts_if, param);
            break;
        case ESP_GATTS_CREATE_EVT:
            handleCreateEvent(gatts_if, param);
            break;
        case ESP_GATTS_ADD_CHAR_EVT:
            handleAddCharEvent(gatts_if, param);
            break;
        case ESP_GATTS_CONNECT_EVT:
            handleConnectEvent(gatts_if, param);
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            handleDisconnectEvent(gatts_if, param);
            break;
        case ESP_GATTS_READ_EVT:
            handleReadEvent(gatts_if, param);
            break;
        case ESP_GATTS_WRITE_EVT:
            handleWriteEvent(gatts_if, param);
            break;
        case ESP_GATTS_MTU_EVT:
            handleMtuEvent(gatts_if, param);
            break;
        case ESP_GATTS_CONF_EVT:
            ESP_LOGD(TAG, "GATTS: CONF_EVT, status=%d", param->conf.status);
            break;
        case ESP_GATTS_OPEN_EVT:
            ESP_LOGI(TAG, "GATTS: OPEN_EVT, status=%d", param->open.status);
            break;
        case ESP_GATTS_CANCEL_OPEN_EVT:
            ESP_LOGI(TAG, "GATTS: CANCEL_OPEN_EVT");
            break;
        case ESP_GATTS_CLOSE_EVT:
            ESP_LOGI(TAG, "GATTS: CLOSE_EVT, status=%d", param->close.status);
            break;
        default:
            ESP_LOGD(TAG, "GATTS event: %d", event);
            break;
        }
    }

    void handleRegEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param) {
        m_gattsIf = gatts_if;
        
        // Configure advertising
        esp_ble_adv_data_t adv_data = {};
        adv_data.set_scan_rsp = false;
        adv_data.include_name = false;
        adv_data.include_txpower = true;
        adv_data.min_interval = 0x0006;
        adv_data.max_interval = 0x0010;
        adv_data.flag = ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT;
        adv_data.service_uuid_len = 16;
        adv_data.p_service_uuid = m_uuidService;

        esp_ble_adv_data_t scan_rsp = {};
        scan_rsp.set_scan_rsp = true;
        scan_rsp.include_name = true;
        scan_rsp.include_txpower = true;

        esp_ble_gap_set_device_name(m_nameValue);
        m_advConfigDone = ADV_CONFIG_FLAG | SCAN_RSP_CONFIG_FLAG;
        esp_ble_gap_config_adv_data(&adv_data);
        esp_ble_gap_config_adv_data(&scan_rsp);

        // Create service - 3 characteristics with CCCD = 3*3 + 1 = 10 handles
        esp_gatt_srvc_id_t svc = {};
        svc.is_primary = true;
        svc.id.inst_id = 0x00;
        svc.id.uuid.len = ESP_UUID_LEN_128;
        memcpy(svc.id.uuid.uuid.uuid128, m_uuidService, 16);
        esp_ble_gatts_create_service(gatts_if, &svc, 10);
    }

    void handleCreateEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param) {
        m_serviceHandle = param->create.service_handle;
        esp_ble_gatts_start_service(m_serviceHandle);
        
        // Add CMD characteristic (Write + Write No Response)
        addCharacteristic(m_serviceHandle, m_uuidCmdChar,
                         ESP_GATT_PERM_WRITE,
                         ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR);
        
        // Add STATUS characteristic (Notify)
        addCharacteristic(m_serviceHandle, m_uuidStatusChar,
                         ESP_GATT_PERM_READ,
                         ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY);
        
        // Add METER characteristic (Notify)
        addCharacteristic(m_serviceHandle, m_uuidMeterChar,
                         ESP_GATT_PERM_READ,
                         ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY);
    }

    void addCharacteristic(uint16_t svcHandle, const uint8_t* uuid128, esp_gatt_perm_t perm, uint8_t prop) {
        static esp_attr_control_t rspByApp = { .auto_rsp = ESP_GATT_RSP_BY_APP };
        esp_bt_uuid_t uuid = {};
        uuid.len = ESP_UUID_LEN_128;
        memcpy(uuid.uuid.uuid128, uuid128, 16);
        esp_attr_value_t val = {};
        val.attr_max_len = 512;
        val.attr_len = 0;
        esp_ble_gatts_add_char(svcHandle, &uuid, perm, (esp_gatt_char_prop_t)prop, &val, &rspByApp);
    }

    void handleAddCharEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param) {
        uint16_t handle = param->add_char.attr_handle;
        
        if (uuidEqual128(param->add_char.char_uuid, m_uuidCmdChar)) {
            m_cmdCharHandle = handle;
            ESP_LOGI(TAG, "CMD char handle: %d", handle);
        } else if (uuidEqual128(param->add_char.char_uuid, m_uuidStatusChar)) {
            m_statusCharHandle = handle;
            ESP_LOGI(TAG, "STATUS char handle: %d", handle);
            // Add CCCD for notifications
            addCccd(param->add_char.service_handle);
        } else if (uuidEqual128(param->add_char.char_uuid, m_uuidMeterChar)) {
            m_meterCharHandle = handle;
            ESP_LOGI(TAG, "METER char handle: %d", handle);
            // Add CCCD for notifications
            addCccd(param->add_char.service_handle);
        }
    }

    void addCccd(uint16_t svcHandle) {
        esp_bt_uuid_t cccdUuid = {};
        cccdUuid.len = ESP_UUID_LEN_16;
        cccdUuid.uuid.uuid16 = 0x2902;
        // Use AUTO_RSP so ESP-IDF handles CCCD writes automatically
        static esp_attr_control_t cccdRsp = { .auto_rsp = ESP_GATT_AUTO_RSP };
        static uint8_t cccdValue[2] = {0, 0};  // Initial value
        esp_attr_value_t cccdAttr = {
            .attr_max_len = 2,
            .attr_len = 2,
            .attr_value = cccdValue
        };
        esp_ble_gatts_add_char_descr(svcHandle, &cccdUuid,
                                     ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, &cccdAttr, &cccdRsp);
    }

    void handleConnectEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param) {
        m_connId = param->connect.conn_id;
        m_connected = true;
        m_mtu = 23;  // Default, will be updated if MTU exchange happens
        m_mtuExchanged = false;
        ESP_LOGI(TAG, "Client connected, conn_id=%d", m_connId);

        // Send initial status after connection (same as old working code)
        // The app may or may not request MTU exchange
        xTaskCreate([](void* arg) {
            BleUnifiedService* self = (BleUnifiedService*)arg;
            vTaskDelay(pdMS_TO_TICKS(500));  // Wait for CCCD setup
            
            if (!self->m_connected) {
                vTaskDelete(NULL);
                return;
            }
            
            // Send full status
            self->sendFullStatus();
            
            ESP_LOGI("BLE_U", "Initial status sent");
            vTaskDelete(NULL);
        }, "ble_init", 2048, this, 5, NULL);
    }
    
    void handleMtuEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param) {
        m_mtu = param->mtu.mtu;
        m_mtuExchanged = true;
        ESP_LOGI(TAG, "MTU exchanged: %d bytes", m_mtu);
    }

    void handleDisconnectEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param) {
        m_connected = false;
        m_connId = 0;
        m_mtu = 23;
        m_mtuExchanged = false;
        ESP_LOGI(TAG, "Client disconnected");
        startAdvertising();
    }

    void handleReadEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param) {
        esp_gatt_rsp_t rsp = {};
        rsp.attr_value.handle = param->read.handle;
        
        // For reads, just return empty - all data sent via notifications
        rsp.attr_value.len = 0;
        
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id,
                                    param->read.trans_id, ESP_GATT_OK, &rsp);
    }

    void handleWriteEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param) {
        uint16_t handle = param->write.handle;
        uint8_t* data = param->write.value;
        uint16_t len = param->write.len;

        if (handle == m_cmdCharHandle && len >= 1) {
            processCommand(data[0], &data[1], len - 1);
        }

        if (param->write.need_rsp) {
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                        param->write.trans_id, ESP_GATT_OK, NULL);
        }
    }

    void processCommand(uint8_t cmd, const uint8_t* payload, size_t len) {
        ESP_LOGI(TAG, "CMD: 0x%02X, len=%d", cmd, len);
        
        switch (cmd) {
        case BleCmd::SET_EQ:
            if (len >= 3 && m_eqCb) {
                m_eqCb((int8_t)payload[0], (int8_t)payload[1], (int8_t)payload[2]);
                sendAck(cmd);
            } else {
                sendError(cmd, BleError::INVALID_PARAM);
            }
            break;
            
        case BleCmd::SET_EQ_PRESET:
            if (len >= 1 && m_eqPresetCb) {
                m_eqPresetCb(payload[0]);
                sendAck(cmd);
            } else {
                sendError(cmd, BleError::INVALID_PARAM);
            }
            break;
            
        case BleCmd::SET_CONTROL:
            if (len >= 1 && m_controlCb) {
                m_controlCb(payload[0]);
                sendAck(cmd);
            } else {
                sendError(cmd, BleError::INVALID_PARAM);
            }
            break;
            
        case BleCmd::SET_NAME:
            if (len >= 1 && m_nameCb) {
                m_nameCb((const char*)payload, len);
                sendAck(cmd);
            } else {
                sendError(cmd, BleError::INVALID_PARAM);
            }
            break;
            
        case BleCmd::SET_LED:
            if (len >= 10 && m_ledCb) {
                m_ledCb(payload, len);
                sendAck(cmd);
            } else {
                sendError(cmd, BleError::INVALID_PARAM);
            }
            break;
            
        case BleCmd::SET_LED_EFFECT:
            if (len >= 1 && m_ledEffectCb) {
                m_ledEffectCb(payload[0]);
                sendAck(cmd);
            } else {
                sendError(cmd, BleError::INVALID_PARAM);
            }
            break;
            
        case BleCmd::SET_LED_BRIGHT:
            if (len >= 1 && m_ledBrightCb) {
                m_ledBrightCb(payload[0]);
                sendAck(cmd);
            } else {
                sendError(cmd, BleError::INVALID_PARAM);
            }
            break;
            
        case BleCmd::SOUND_MUTE:
            if (len >= 1 && m_soundMuteCb) {
                m_soundMuteCb(payload[0] != 0);
                sendAck(cmd);
            } else {
                sendError(cmd, BleError::INVALID_PARAM);
            }
            break;
            
        case BleCmd::SOUND_DELETE:
            if (len >= 1 && m_soundDeleteCb) {
                m_soundDeleteCb(payload[0]);
                sendAck(cmd);
            } else {
                sendError(cmd, BleError::INVALID_PARAM);
            }
            break;
            
        case BleCmd::SOUND_UP_START:
        case BleCmd::SOUND_UP_DATA:
        case BleCmd::SOUND_UP_END:
            if (m_soundDataCb) {
                m_soundDataCb(cmd, payload, len);
                // ACK handled by callback
            }
            break;
            
        case BleCmd::OTA_BEGIN:
        case BleCmd::OTA_DATA:
        case BleCmd::OTA_END:
        case BleCmd::OTA_ABORT:
            if (m_otaCb) {
                m_otaCb(cmd, payload, len);
                // ACK handled by callback
            }
            break;
            
        case BleCmd::REQUEST_STATUS:
            sendFullStatus();
            break;
            
        case BleCmd::PING:
            sendPong();
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown command: 0x%02X", cmd);
            sendError(cmd, BleError::INVALID_CMD);
            break;
        }
    }

    void startAdvertising() {
        esp_ble_adv_params_t adv_params = {};
        adv_params.adv_int_min = 0x20;
        adv_params.adv_int_max = 0x40;
        adv_params.adv_type = ADV_TYPE_IND;
        adv_params.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
        adv_params.channel_map = ADV_CHNL_ALL;
        adv_params.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;
        esp_ble_gap_start_advertising(&adv_params);
    }

    // GATT interface
    esp_gatt_if_t m_gattsIf;
    uint16_t m_connId;
    bool m_connected;
    uint16_t m_mtu;  // Negotiated MTU size
    bool m_mtuExchanged;  // True after MTU exchange completes
    uint8_t m_advConfigDone;

    // Service handle
    uint16_t m_serviceHandle;

    // Characteristic handles
    uint16_t m_cmdCharHandle;
    uint16_t m_statusCharHandle;
    uint16_t m_meterCharHandle;

    // UUIDs (little-endian)
    uint8_t m_uuidService[16];
    uint8_t m_uuidCmdChar[16];
    uint8_t m_uuidStatusChar[16];
    uint8_t m_uuidMeterChar[16];

    // State values
    int8_t m_eqValue[3];
    uint8_t m_controlValue;
    uint8_t m_ledValue[10];  // [effect, bright, speed, r1,g1,b1, r2,g2,b2, gradient]
    uint8_t m_soundStatus;
    char m_nameValue[64];
    char m_fwValue[32];

    // Callbacks
    EqCallback m_eqCb;
    EqPresetCallback m_eqPresetCb;
    ControlCallback m_controlCb;
    NameCallback m_nameCb;
    LedCallback m_ledCb;
    LedEffectCallback m_ledEffectCb;
    LedBrightnessCallback m_ledBrightCb;
    SoundMuteCallback m_soundMuteCb;
    SoundDeleteCallback m_soundDeleteCb;
    SoundDataCallback m_soundDataCb;
    OtaCallback m_otaCb;
};
