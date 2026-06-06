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
#include "freertos/timers.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
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

// Build-step state machine for sequential GATT construction. The service and
// its characteristics/descriptors are added one event at a time; this enum
// tracks which attribute we are waiting on so the chain advances reliably
// regardless of UUID echo quirks.
enum BuildStep : uint8_t {
    BUILD_IDLE = 0,
    BUILD_CMD,
    BUILD_STATUS,
    BUILD_STATUS_CCCD,
    BUILD_METER,
    BUILD_METER_CCCD,
    BUILD_DONE
};

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
    using DisconnectCallback = void(*)();

    BleUnifiedService()
        : m_gattsIf(0)
        , m_connId(0)
        , m_connected(false)
        , m_mtu(23)  // Default BLE MTU
        , m_mtuExchanged(false)
        , m_notifyEnabled(false)
        , m_meterNotifyEnabled(false)
        , m_advConfigDone(0)
        , m_serviceHandle(0)
        , m_cmdCharHandle(0)
        , m_statusCharHandle(0)
        , m_meterCharHandle(0)
        , m_statusCccdHandle(0)
        , m_meterCccdHandle(0)
        , m_buildStep(BUILD_IDLE)
        , m_initTimer(nullptr)
        , m_advRetryTimer(nullptr)
        , m_advWanted(false)
        , m_advertising(false)
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
        , m_disconnectCb(nullptr)
    {
        memset(m_uuidService, 0, 16);
        memset(m_uuidCmdChar, 0, 16);
        memset(m_uuidStatusChar, 0, 16);
        memset(m_uuidMeterChar, 0, 16);
        m_statusCccdStore[0] = m_statusCccdStore[1] = 0;
        m_meterCccdStore[0] = m_meterCccdStore[1] = 0;
        
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

        // Bluetooth controller and bluedroid are initialized centrally in main.cpp
        
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

    // Set a callback that fires when BLE client disconnects
    // Used to clean up sound upload buffers, abort OTA, etc.
    void setDisconnectCallback(DisconnectCallback cb) { m_disconnectCb = cb; }

    bool isConnected() const { return m_connected; }

    // Restart BLE advertising when Classic BT A2DP is active (controller may stop it)
    void restartAdvertising() {
        if (!m_connected) {
            m_advWanted = true;
            startAdvertisingRetry();
            startAdvertising();
        }
    }

    // ========== Update state and notify ==========
    
    // Meter notifications are the highest-frequency BLE traffic. When internal
    // RAM is critically low (e.g. LDAC 96k streaming), skip them so the BT stack
    // keeps its remaining heap for ACL/GATT bookkeeping instead of dropping audio.
    static constexpr size_t kMinHeapForMeterBytes = 8 * 1024;

    void updateLevels(uint8_t l30, uint8_t l60, uint8_t l100) {
        if (m_connected && m_meterNotifyEnabled && m_meterCharHandle && m_gattsIf) {
            if (heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) < kMinHeapForMeterBytes) {
                return;
            }
            uint8_t data[3] = { l30, l60, l100 };
            if (getNotifyPayloadMax() < sizeof(data)) {
                ESP_LOGD(TAG, "Meter notify skipped: mtu=%u payload_max=%u",
                         (unsigned)m_mtu, (unsigned)getNotifyPayloadMax());
                return;
            }
            esp_err_t err = esp_ble_gatts_send_indicate(m_gattsIf, m_connId, m_meterCharHandle, 3, data, false);
            if (err != ESP_OK) {
                // Meter notifications are best-effort. Do not mark the BLE
                // session dead here; command/status traffic can still be valid
                // if the app has not subscribed to the meter characteristic yet.
                ESP_LOGD(TAG, "Meter notify skipped: %s", esp_err_to_name(err));
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
            notifyBuffer(buffer, idx, "sendFullStatus");
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
        if (!m_notifyEnabled) {
            ESP_LOGD(TAG, "notifyStatus(0x%02X) skipped: notifications not enabled", respId);
            return;
        }
        
        uint8_t buffer[80];
        buffer[0] = respId;
        if (len > sizeof(buffer) - 1) {
            len = sizeof(buffer) - 1;
        }
        if (data && len > 0) {
            memcpy(&buffer[1], data, len);
        }
        notifyBuffer(buffer, 1 + len, "notifyStatus");
    }

    uint16_t getNotifyPayloadMax() const {
        return (m_mtu > 3) ? (m_mtu - 3) : 20;
    }

    void notifyBuffer(const uint8_t* data, size_t len, const char* label) {
        uint16_t maxPayload = getNotifyPayloadMax();
        if (maxPayload == 0) maxPayload = 20;

        if (len <= maxPayload) {
            if (!m_notifyEnabled) {
                ESP_LOGD(TAG, "%s skipped: notifications not enabled", label);
                return;
            }
            esp_err_t err = esp_ble_gatts_send_indicate(m_gattsIf, m_connId, m_statusCharHandle,
                                                        len, (uint8_t*)data, false);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "%s skipped: %s mtu=%u payload_max=%u len=%u",
                         label, esp_err_to_name(err), (unsigned)m_mtu,
                         (unsigned)maxPayload, (unsigned)len);
            }
            return;
        }

        ESP_LOGI(TAG, "%s chunked: len=%u mtu=%u payload_max=%u mtu_exchanged=%d",
                 label, (unsigned)len, (unsigned)m_mtu, (unsigned)maxPayload, m_mtuExchanged);

        size_t offset = 0;
        while (offset < len) {
            if (!m_notifyEnabled) {
                ESP_LOGD(TAG, "%s chunk skipped: notifications not enabled", label);
                return;
            }
            size_t chunk = len - offset;
            if (chunk > maxPayload) chunk = maxPayload;
            esp_err_t err = esp_ble_gatts_send_indicate(m_gattsIf, m_connId, m_statusCharHandle,
                                                        chunk, (uint8_t*)(data + offset), false);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "%s chunk skipped: %s offset=%u chunk=%u mtu=%u",
                         label, esp_err_to_name(err), (unsigned)offset,
                         (unsigned)chunk, (unsigned)m_mtu);
                return;
            }
            offset += chunk;
            taskYIELD();
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
                m_advertising = false;
                if (m_advWanted && !m_connected) {
                    startAdvertisingRetry();
                }
            } else {
                ESP_LOGI(TAG, "Advertising started");
                m_advertising = true;
                stopAdvertisingRetry();
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            m_advertising = false;
            if (m_advWanted && !m_connected) {
                startAdvertisingRetry();
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
        case ESP_GATTS_ADD_CHAR_DESCR_EVT:
            handleAddCharDescrEvent(gatts_if, param);
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
        // Note: Using ESP_BLE_ADV_FLAG_DMT_CONTROLLER_SPT instead of ESP_BLE_ADV_FLAG_BREDR_NOT_SPT
        // because we DO support Classic Bluetooth (for A2DP). This tells Windows that both
        // BLE and Classic BT are available from this device.
        esp_ble_adv_data_t adv_data = {};
        adv_data.set_scan_rsp = false;
        adv_data.include_name = false;
        adv_data.include_txpower = true;
        adv_data.min_interval = 0x0006;
        adv_data.max_interval = 0x0010;
        adv_data.flag = ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_DMT_CONTROLLER_SPT;
        adv_data.service_uuid_len = 16;
        adv_data.p_service_uuid = m_uuidService;

        esp_ble_adv_data_t scan_rsp = {};
        scan_rsp.set_scan_rsp = true;
        scan_rsp.include_name = true;
        scan_rsp.include_txpower = true;

        // Use a different name for BLE to avoid confusion with Classic BT A2DP
        // Windows sometimes connects to BLE instead of Classic BT for audio
        char bleName[64];
        snprintf(bleName, sizeof(bleName), "%s BLE", m_nameValue);
        esp_ble_gap_set_device_name(bleName);
        
        m_advConfigDone = ADV_CONFIG_FLAG | SCAN_RSP_CONFIG_FLAG;
        esp_ble_gap_config_adv_data(&adv_data);
        esp_ble_gap_config_adv_data(&scan_rsp);

        // Create the primary service. Characteristics are then added one at a
        // time, driven by a build-step state machine (see handleAddCharEvent),
        // which is robust and works on all IDF builds. (The atomic attr-table
        // API wedged the BT host task on this build, killing BLE *and* A2DP.)
        esp_gatt_srvc_id_t svc = {};
        svc.is_primary = true;
        svc.id.inst_id = 0x00;
        svc.id.uuid.len = ESP_UUID_LEN_128;
        memcpy(svc.id.uuid.uuid.uuid128, m_uuidService, 16);
        esp_err_t err = esp_ble_gatts_create_service(gatts_if, &svc, 12);
        ESP_LOGI(TAG, "create_service: %s", esp_err_to_name(err));
    }

    // Add a 128-bit characteristic. Pass NULL for the value (char_val) because
    // these characteristics are entirely app-managed (ESP_GATT_RSP_BY_APP);
    // handing bluedroid an esp_attr_value_t with a NULL buffer was what made the
    // previous add_char calls fail silently, so the chain never advanced.
    esp_err_t addChar(const uint8_t* uuid128, esp_gatt_perm_t perm, uint8_t prop) {
        static esp_attr_control_t rspByApp = { .auto_rsp = ESP_GATT_RSP_BY_APP };
        esp_bt_uuid_t uuid = {};
        uuid.len = ESP_UUID_LEN_128;
        memcpy(uuid.uuid.uuid128, uuid128, 16);
        return esp_ble_gatts_add_char(m_serviceHandle, &uuid, perm,
                                      (esp_gatt_char_prop_t)prop, NULL, &rspByApp);
    }

    esp_err_t addCccd(uint8_t* storage) {
        esp_bt_uuid_t cccdUuid = {};
        cccdUuid.len = ESP_UUID_LEN_16;
        cccdUuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;  // 0x2902
        static esp_attr_control_t autoRsp = { .auto_rsp = ESP_GATT_AUTO_RSP };
        esp_attr_value_t attr = {};
        attr.attr_max_len = 2;
        attr.attr_len = 2;
        attr.attr_value = storage;
        return esp_ble_gatts_add_char_descr(m_serviceHandle, &cccdUuid,
                                            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                            &attr, &autoRsp);
    }

    void handleCreateEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param) {
        m_serviceHandle = param->create.service_handle;
        ESP_LOGI(TAG, "Service created: handle=%u status=%d",
                 (unsigned)m_serviceHandle, param->create.status);
        m_buildStep = BUILD_CMD;
        esp_err_t err = addChar(m_uuidCmdChar, ESP_GATT_PERM_WRITE,
                                ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR);
        ESP_LOGI(TAG, "add CMD char: %s", esp_err_to_name(err));
    }

    void handleAddCharEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param) {
        ESP_LOGI(TAG, "ADD_CHAR_EVT: step=%u handle=%u status=%d",
                 (unsigned)m_buildStep, (unsigned)param->add_char.attr_handle,
                 param->add_char.status);
        switch (m_buildStep) {
        case BUILD_CMD:
            m_cmdCharHandle = param->add_char.attr_handle;
            m_buildStep = BUILD_STATUS;
            ESP_LOGI(TAG, "add STATUS char: %s", esp_err_to_name(
                addChar(m_uuidStatusChar, ESP_GATT_PERM_READ,
                        ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY)));
            break;
        case BUILD_STATUS:
            m_statusCharHandle = param->add_char.attr_handle;
            m_buildStep = BUILD_STATUS_CCCD;
            ESP_LOGI(TAG, "add STATUS cccd: %s", esp_err_to_name(addCccd(m_statusCccdStore)));
            break;
        case BUILD_METER:
            m_meterCharHandle = param->add_char.attr_handle;
            m_buildStep = BUILD_METER_CCCD;
            ESP_LOGI(TAG, "add METER cccd: %s", esp_err_to_name(addCccd(m_meterCccdStore)));
            break;
        default:
            ESP_LOGW(TAG, "Unexpected ADD_CHAR_EVT in step %u", (unsigned)m_buildStep);
            break;
        }
    }

    void handleAddCharDescrEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param) {
        ESP_LOGI(TAG, "ADD_DESCR_EVT: step=%u handle=%u status=%d",
                 (unsigned)m_buildStep, (unsigned)param->add_char_descr.attr_handle,
                 param->add_char_descr.status);
        switch (m_buildStep) {
        case BUILD_STATUS_CCCD:
            m_statusCccdHandle = param->add_char_descr.attr_handle;
            m_buildStep = BUILD_METER;
            ESP_LOGI(TAG, "add METER char: %s", esp_err_to_name(
                addChar(m_uuidMeterChar, ESP_GATT_PERM_READ,
                        ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY)));
            break;
        case BUILD_METER_CCCD:
            m_meterCccdHandle = param->add_char_descr.attr_handle;
            m_buildStep = BUILD_DONE;
            ESP_LOGI(TAG, "GATT build complete: cmd=%u status=%u statusCccd=%u meter=%u meterCccd=%u; start=%s",
                     (unsigned)m_cmdCharHandle, (unsigned)m_statusCharHandle,
                     (unsigned)m_statusCccdHandle, (unsigned)m_meterCharHandle,
                     (unsigned)m_meterCccdHandle,
                     esp_err_to_name(esp_ble_gatts_start_service(m_serviceHandle)));
            break;
        default:
            ESP_LOGW(TAG, "Unexpected ADD_DESCR_EVT in step %u", (unsigned)m_buildStep);
            break;
        }
    }

    void handleConnectEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param) {
        m_connId = param->connect.conn_id;
        m_connected = true;
        m_advWanted = false;
        m_advertising = false;
        stopAdvertisingRetry();
        m_mtu = 23;  // Default, will be updated if MTU exchange happens
        m_mtuExchanged = false;
        m_notifyEnabled = false;
        m_meterNotifyEnabled = false;
        ESP_LOGI(TAG, "Client connected, conn_id=%d mtu=%u", m_connId, (unsigned)m_mtu);

        esp_ble_conn_update_params_t connParams = {};
        memcpy(connParams.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        connParams.latency = 0;
        connParams.min_int = 18;   // 22.5 ms
        connParams.max_int = 24;   // 30 ms
        connParams.timeout = 400;  // 4 s
        esp_err_t connErr = esp_ble_gap_update_conn_params(&connParams);
        ESP_LOGI(TAG, "Requested BLE conn params: min=22.5ms max=30ms latency=0 timeout=4s err=%s",
                 esp_err_to_name(connErr));

        // Send initial status after a short delay (wait for CCCD setup)
        // Uses a one-shot FreeRTOS timer instead of xTaskCreate to avoid
        // allocating 2KB stack from heap on every connect (heap leak fix)
        if (m_initTimer == nullptr) {
            m_initTimer = xTimerCreate("ble_init", pdMS_TO_TICKS(500),
                                       pdFALSE, this, initTimerCallback);
        }
        if (m_initTimer) {
            // Reset and start the timer (if already running, this restarts it)
            xTimerReset(m_initTimer, 0);
        }
    }
    
    void handleMtuEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param) {
        m_mtu = param->mtu.mtu;
        m_mtuExchanged = true;
        ESP_LOGI(TAG, "MTU exchanged: %d bytes, notify_payload_max=%u",
                 m_mtu, (unsigned)getNotifyPayloadMax());
    }

    void handleDisconnectEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param) {
        m_connected = false;
        m_connId = 0;
        m_advWanted = true;
        m_advertising = false;
        m_mtu = 23;
        m_mtuExchanged = false;
        m_notifyEnabled = false;
        m_meterNotifyEnabled = false;
        
        // Stop the init timer if it's still pending (client disconnected before status sent)
        if (m_initTimer) {
            xTimerStop(m_initTimer, 0);
        }
        
        ESP_LOGI(TAG, "Client disconnected");
        
        // Notify app so it can clean up (e.g., abort sound upload, free buffers)
        if (m_disconnectCb) {
            m_disconnectCb();
        }
        
        startAdvertisingRetry();
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

        if (param->write.need_rsp) {
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                        param->write.trans_id, ESP_GATT_OK, NULL);
        }

        if (handle == m_cmdCharHandle && len >= 1) {
            ESP_LOGI(TAG, "CMD write: len=%u mtu=%u need_rsp=%d",
                     (unsigned)len, (unsigned)m_mtu, param->write.need_rsp);
            // Receiving a command proves the client discovered our service and is
            // talking to us. Make sure server-side notifications are enabled so the
            // ACK/status reply can actually be sent back (covers the case where the
            // status CCCD write event was never delivered to us).
            if (!m_notifyEnabled) {
                m_notifyEnabled = true;
                ESP_LOGI(TAG, "Status notifications enabled on first command");
            }
            processCommand(data[0], &data[1], len - 1);
        } else if (len == 2) {
            uint16_t value = data[0] | (data[1] << 8);
            ESP_LOGI(TAG, "CCCD/descr write: handle=%u value=0x%04X mtu=%u",
                     (unsigned)handle, (unsigned)value, (unsigned)m_mtu);
            bool enabled = (value & 0x0001) != 0;
            if (handle == m_statusCccdHandle || m_statusCccdHandle == 0) {
                m_notifyEnabled = enabled;
            }
            if (handle == m_meterCccdHandle) {
                m_meterNotifyEnabled = enabled;
            }
            if (m_notifyEnabled && (handle == m_statusCccdHandle || m_statusCccdHandle == 0)) {
                sendFullStatus();
            }
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
        if (m_connected || m_advConfigDone != 0) {
            return;
        }
        m_advWanted = true;
        esp_ble_adv_params_t adv_params = {};
        adv_params.adv_int_min = 0x80;
        adv_params.adv_int_max = 0x100;
        adv_params.adv_type = ADV_TYPE_IND;
        adv_params.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
        adv_params.channel_map = ADV_CHNL_ALL;
        adv_params.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;
        esp_err_t err = esp_ble_gap_start_advertising(&adv_params);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Advertising start request failed: %s", esp_err_to_name(err));
            m_advertising = false;
            startAdvertisingRetry();
        }
    }

    void startAdvertisingRetry() {
        if (m_connected) {
            return;
        }
        if (m_advRetryTimer == nullptr) {
            m_advRetryTimer = xTimerCreate("ble_adv_retry", pdMS_TO_TICKS(1500),
                                           pdTRUE, this, advRetryTimerCallback);
        }
        if (m_advRetryTimer) {
            xTimerReset(m_advRetryTimer, 0);
        }
    }

    void stopAdvertisingRetry() {
        if (m_advRetryTimer) {
            xTimerStop(m_advRetryTimer, 0);
        }
    }

    // Timer callback for sending initial status after BLE connect
    // Runs on the FreeRTOS timer daemon task (no new stack allocation per connect)
    static void initTimerCallback(TimerHandle_t timer) {
        BleUnifiedService* self = (BleUnifiedService*)pvTimerGetTimerID(timer);
        if (!self || !self->m_connected) return;
        // Fallback: some controllers/IDF builds handle the status CCCD write with
        // ESP_GATT_AUTO_RSP and never surface a GATTS_WRITE_EVT for it, so the
        // explicit CCCD-enable path may never run. By this point the client has
        // connected (and usually negotiated MTU), and the Android app always calls
        // setCharacteristicNotification() locally, so it is safe to enable server
        // notifications here. Without this, the ESP32 could never send anything back.
        if (!self->m_notifyEnabled) {
            ESP_LOGI("BLE_U", "Enabling status notifications (CCCD-write fallback)");
            self->m_notifyEnabled = true;
        }
        self->sendFullStatus();
        ESP_LOGI("BLE_U", "Initial status sent");
    }

    static void advRetryTimerCallback(TimerHandle_t timer) {
        BleUnifiedService* self = (BleUnifiedService*)pvTimerGetTimerID(timer);
        if (!self || self->m_connected || !self->m_advWanted) {
            return;
        }
        if (!self->m_advertising) {
            self->startAdvertising();
        }
    }

    // GATT interface
    esp_gatt_if_t m_gattsIf;
    uint16_t m_connId;
    bool m_connected;
    uint16_t m_mtu;  // Negotiated MTU size
    bool m_mtuExchanged;  // True after MTU exchange completes
    bool m_notifyEnabled;
    bool m_meterNotifyEnabled;
    uint8_t m_advConfigDone;

    // Service handle
    uint16_t m_serviceHandle;

    // Characteristic handles
    uint16_t m_cmdCharHandle;
    uint16_t m_statusCharHandle;
    uint16_t m_meterCharHandle;
    uint16_t m_statusCccdHandle;
    uint16_t m_meterCccdHandle;

    // GATT build-step state + per-CCCD storage (AUTO_RSP needs distinct buffers)
    uint8_t m_buildStep;
    uint8_t m_statusCccdStore[2];
    uint8_t m_meterCccdStore[2];

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

    // Timer for deferred init status send (replaces xTaskCreate per connect)
    TimerHandle_t m_initTimer;
    TimerHandle_t m_advRetryTimer;
    bool m_advWanted;
    bool m_advertising;

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
    DisconnectCallback m_disconnectCb;
};
