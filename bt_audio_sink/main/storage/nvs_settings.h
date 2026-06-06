#pragma once

// -----------------------------------------------------------
// NVS Settings - persistent storage for device configuration
// -----------------------------------------------------------

#include <string>
#include <stdint.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "../config/app_config.h"

class NVSSettings {
public:
    // Settings structure
    struct Settings {
        std::string deviceName;
        uint8_t controlByte;      // bit0=bassBoost, bit1=channelFlip, bit2=bypass
        int8_t eqBassDB;
        int8_t eqMidDB;
        int8_t eqTrebleDB;

        Settings() 
            : deviceName(APP_DEFAULT_DEVICE_NAME)
            , controlByte(0)
            , eqBassDB(0)
            , eqMidDB(0)
            , eqTrebleDB(0) 
        {}
    };

    NVSSettings() {}

    // Load all settings from NVS (stores internally)
    bool load() {
        return load(m_settings);
    }

    // Get current settings
    void getControl(bool &bassBoost, bool &channelFlip, bool &bypass) const {
        bassBoost = (m_settings.controlByte & 0x01) != 0;
        channelFlip = (m_settings.controlByte & 0x02) != 0;
        bypass = (m_settings.controlByte & 0x04) != 0;
    }

    void getEQ(int8_t &bass, int8_t &mid, int8_t &treble) const {
        bass = m_settings.eqBassDB;
        mid = m_settings.eqMidDB;
        treble = m_settings.eqTrebleDB;
    }

    void getDeviceName(std::string &name) const {
        name = m_settings.deviceName;
    }

    // Save control flags
    bool saveControl(bool bassBoost, bool channelFlip, bool bypass) {
        uint8_t ctrl = 0;
        if (bassBoost) ctrl |= 0x01;
        if (channelFlip) ctrl |= 0x02;
        if (bypass) ctrl |= 0x04;
        m_settings.controlByte = ctrl;
        return saveControl(ctrl);
    }

    // Load all settings from NVS (explicit struct)
    bool load(Settings &settings) {
        nvs_handle_t h;
        esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "NVS open failed, using defaults");
            settings = Settings();
            return false;
        }

        // Device name
        char buf[32];
        size_t len = sizeof(buf);
        err = nvs_get_str(h, NVS_KEY_DEVNAME, buf, &len);
        if (err == ESP_OK && len > 0) {
            settings.deviceName.assign(buf, len - 1);
        } else {
            settings.deviceName = APP_DEFAULT_DEVICE_NAME;
        }

        // Control byte
        uint8_t ctrl = 0;
        if (nvs_get_u8(h, NVS_KEY_CTRL, &ctrl) == ESP_OK) {
            settings.controlByte = ctrl;
        }

        // EQ
        int8_t b = 0, m = 0, t = 0;
        if (nvs_get_i8(h, NVS_KEY_EQ_BASS, &b) == ESP_OK) settings.eqBassDB = b;
        if (nvs_get_i8(h, NVS_KEY_EQ_MID, &m) == ESP_OK) settings.eqMidDB = m;
        if (nvs_get_i8(h, NVS_KEY_EQ_TREB, &t) == ESP_OK) settings.eqTrebleDB = t;

        nvs_close(h);

        ESP_LOGI(TAG, "NVS loaded: name='%s', ctrl=0x%02x, EQ(b,m,t)=(%d,%d,%d)",
                 settings.deviceName.c_str(), settings.controlByte,
                 settings.eqBassDB, settings.eqMidDB, settings.eqTrebleDB);
        return true;
    }

    // Save control byte
    bool saveControl(uint8_t ctrl) {
        nvs_handle_t h;
        if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return false;
        nvs_set_u8(h, NVS_KEY_CTRL, ctrl);
        nvs_commit(h);
        nvs_close(h);
        return true;
    }

    // Save EQ settings
    bool saveEQ(int8_t bass, int8_t mid, int8_t treble) {
        nvs_handle_t h;
        if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
            ESP_LOGE(TAG, "saveEQ: NVS open failed!");
            return false;
        }
        nvs_set_i8(h, NVS_KEY_EQ_BASS, bass);
        nvs_set_i8(h, NVS_KEY_EQ_MID, mid);
        nvs_set_i8(h, NVS_KEY_EQ_TREB, treble);
        esp_err_t err = nvs_commit(h);
        nvs_close(h);
        
        // Update in-memory cache
        m_settings.eqBassDB = bass;
        m_settings.eqMidDB = mid;
        m_settings.eqTrebleDB = treble;
        
        ESP_LOGI(TAG, "saveEQ: saved EQ(%d,%d,%d) to NVS, commit=%s", 
                 bass, mid, treble, esp_err_to_name(err));
        return err == ESP_OK;
    }

    // Save device name
    bool saveDeviceName(const std::string &name) {
        m_settings.deviceName = name;
        nvs_handle_t h;
        if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return false;
        nvs_set_str(h, NVS_KEY_DEVNAME, name.c_str());
        nvs_commit(h);
        nvs_close(h);
        return true;
    }
    
    bool saveDeviceName(const char* name) {
        return saveDeviceName(std::string(name));
    }

    // Load LED effect from NVS
    uint8_t loadLedEffect() {
        nvs_handle_t h;
        if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return 0;
        uint8_t effect = 0;
        nvs_get_u8(h, "led_effect", &effect);
        nvs_close(h);
        return effect;
    }

    // Save LED effect to NVS
    bool saveLedEffect(uint8_t effect) {
        nvs_handle_t h;
        if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return false;
        nvs_set_u8(h, "led_effect", effect);
        nvs_commit(h);
        nvs_close(h);
        return true;
    }

    // Load sound muted state from NVS
    bool loadSoundMuted() {
        nvs_handle_t h;
        if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return false;
        uint8_t muted = 0;
        nvs_get_u8(h, "sound_muted", &muted);
        nvs_close(h);
        return muted != 0;
    }

    // Save sound muted state to NVS
    bool saveSoundMuted(bool muted) {
        nvs_handle_t h;
        if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return false;
        nvs_set_u8(h, "sound_muted", muted ? 1 : 0);
        nvs_commit(h);
        nvs_close(h);
        return true;
    }

    // Load 3D sound enabled state from NVS
    bool load3DSound() {
        nvs_handle_t h;
        if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return false;
        uint8_t enabled = 0;
        nvs_get_u8(h, "3d_sound", &enabled);
        nvs_close(h);
        return enabled != 0;
    }

    // Save 3D sound enabled state to NVS
    bool save3DSound(bool enabled) {
        nvs_handle_t h;
        if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return false;
        nvs_set_u8(h, "3d_sound", enabled ? 1 : 0);
        nvs_commit(h);
        nvs_close(h);
        ESP_LOGI(TAG, "save3DSound: saved %s to NVS", enabled ? "ON" : "OFF");
        return true;
    }

private:
    static constexpr const char* TAG = "NVS";
    Settings m_settings;
};
