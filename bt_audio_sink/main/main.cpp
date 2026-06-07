/*
 * main.cpp
 *
 * Main entry point for the ESP32 A2DP Sink with high-res codec support.
 * This coordinates all the modules: Bluetooth audio, DSP, LED matrix,
 * BLE control, rotary encoders, and the whole shebang.
 *
 * Supports LDAC, aptX, aptX-HD, AAC, and SBC codecs.
 */

 #include <string>
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "freertos/timers.h"
 #include "freertos/queue.h"
 #include "driver/gpio.h"
 #include "esp_heap_caps.h"
 #include "esp_rom_sys.h"
 #include "driver/i2s.h"
 #include "nvs_flash.h"
 #include "esp_timer.h"
 #include "esp_log.h"
 #include "esp_ota_ops.h"
 #include "esp_partition.h"
 #include "esp_bt_device.h"
 #include "bt/a2dp_sink_native.h"
 
 // Modular components
 #include "config/app_config.h"
 #include "dsp/dsp_processor.h"
 #include "storage/nvs_settings.h"
 #include "audio/i2s_output.h"
 #include "audio/audio_pipeline.h"
 #include "audio/sound_player.h"
 #include "audio/overlay_mixer.h"
 #include "ble/ble_unified.h"
 #include "ota/idf_update.h"
 
 // SPIFFS for sound storage
 #include "esp_spiffs.h"
 #include <sys/stat.h>
 #include <string.h>  // for strerror
 
 // LED Matrix support
 #ifdef CONFIG_LED_MATRIX_ENABLE
 #include "led/led_controller.h"
 #endif
 
 // Encoder support
 #ifdef CONFIG_ENCODER_ENABLE
 #include "input/encoder_controller.h"
 #endif
 
 static const char* TAG = "Main";
 
 // -----------------------------------------------------------
 // Global instances (no raw global state - encapsulated in classes)
 // -----------------------------------------------------------
 static NVSSettings     g_settings;
 static DSPProcessor    g_dsp;
 static I2SOutput       g_i2s;
 static AudioPipeline   g_pipeline;
 static OverlayMixer    g_overlayMixer;
 static BleUnifiedService g_ble;
 static NativeA2DPSink g_a2dp;
 static IdfUpdate       g_update;
 
 // Sound player reference (singleton)
 #define g_sound SoundPlayer::getInstance()
 
 // Sound upload state
 static volatile bool     g_soundUploadActive = false;
 static volatile SoundType g_soundUploadType = SOUND_STARTUP;
 static volatile uint32_t g_soundUploadSize = 0;
 static volatile uint32_t g_soundUploadReceived = 0;
 static volatile uint32_t g_soundUploadQueued = 0;
 
 // Audio state
 static volatile uint8_t  g_bitsPerSample = 16;
 static volatile uint8_t  g_channels = 2;
 static volatile uint32_t g_sampleRate = APP_I2S_DEFAULT_SAMPLE_RATE;
 static volatile bool     g_otaActive = false;
 static volatile int64_t  g_otaCheckPassedTime = 0;  // Timestamp when CHECK passed (0 = not passed)
 static volatile bool     g_codecReconfiguring = false;
 static TaskHandle_t      audioTaskHandle = nullptr;
 
 struct CodecConfigRequest {
     uint32_t rate;
     uint8_t bitsPerSample;
     uint8_t channels;
     esp_a2d_mct_t codecType;
     int64_t timestampUs;
 };
 
 // Pairing mode coordination flag - prevents disconnect callback from clearing pairing mode
 static volatile bool     g_pairingModeActive = false;  // True while in pairing mode (discoverable)
 
 // Codec switch detection - to suppress connected sound during rapid codec changes
 static volatile int64_t  g_lastDisconnectTime = 0;     // Timestamp of last disconnect (esp_timer_get_time)
 static const int64_t     CODEC_SWITCH_TIMEOUT_US = 5000000;  // 5 seconds - if reconnect within this, skip connected sound
 
 // Connected sound delay - wait for codec to stabilize before playing
 static volatile int64_t  g_lastCodecConfigTime = 0;    // Timestamp of last codec config
 static volatile bool     g_connectedSoundPending = false;  // True if waiting to play connected sound
 static const int64_t     CODEC_STABLE_DELAY_US = 400000;   // 400ms - wait this long after last codec config
 
 // Connection timestamp - to ignore initial volume report from phone
 static volatile int64_t  g_lastConnectTime = 0;        // Timestamp of last A2DP connection
 static const int64_t     VOLUME_GRACE_PERIOD_US = 2000000;  // 2 seconds - ignore volume changes after connection
 static volatile bool     g_a2dpConnected = false;      // True when A2DP is in CONNECTED state
 
 // Max volume sound rate limiting - prevent crash from spamming
 static volatile int64_t  g_lastMaxVolumeSound = 0;     // Timestamp of last max volume sound
 static const int64_t     MAX_VOLUME_SOUND_COOLDOWN_US = 2000000;  // 2 seconds cooldown between plays
 
 // Max-volume overlay now returns seamlessly through OverlayMixer's duck ramp.
 // No same-rate I2S recovery/reset is used here; the previous post-effect
 // distortion was caused by int16_t duck-gain overflow in OverlayMixer.
 
 // BLE notification pause flag - used during SPIFFS writes
 static volatile bool g_pauseBleNotifications = false;
 
 // Sound delete state
 static volatile bool g_soundDeletePending = false;
 static volatile SoundType g_soundDeleteType = SOUND_STARTUP;
 
 // Beat detection state
 static float smooth30_dB = -60.0f;
 static float smooth60_dB = -60.0f;
 static float smooth100_dB = -60.0f;
 
 // -----------------------------------------------------------
 // Control helpers
 // -----------------------------------------------------------
 static uint8_t getControlByte() {
     uint8_t b = 0;
     if (g_dsp.isBassBoostEnabled()) b |= 0x01;
     if (g_dsp.isChannelFlipEnabled()) b |= 0x02;
     if (g_dsp.isBypassEnabled()) b |= 0x04;
     return b;
 }
 
 static void applyControlByte(uint8_t b, bool notifyBle = true) {
     g_dsp.setBassBoost(b & 0x01);
     g_dsp.setChannelFlip(b & 0x02);
     g_dsp.setBypass(b & 0x04);
     g_settings.saveControl(b & 0x01, b & 0x02, b & 0x04);
     if (notifyBle) {
         g_ble.updateControl(getControlByte());
     }
 }
 
 // Heap logging helper for init stages
 static void logHeap(const char* stage) {
     size_t free_int = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
     size_t largest  = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
     size_t min_ever = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
     ESP_LOGI(TAG, "HEAP %-24s free=%5u KB  largest=%5u KB  min_ever=%5u KB",
              stage,
              (unsigned)(free_int / 1024),
              (unsigned)(largest / 1024),
              (unsigned)(min_ever / 1024));
 }
 
 static void applyEq(int8_t bass, int8_t mid, int8_t treble, bool notifyBle = true) {
     g_dsp.setEQ(bass, mid, treble, g_sampleRate);
     g_settings.saveEQ(bass, mid, treble);
     if (notifyBle) {
         g_ble.updateEq(bass, mid, treble);
     }
     
     // Sync encoder controller if encoders are enabled
     #ifdef CONFIG_ENCODER_ENABLE
     EncoderController::getInstance().setCurrentEq(bass, mid, treble);
     #endif
 }
 
 // -----------------------------------------------------------
 // Encoder callbacks (hardware rotary encoders)
 // -----------------------------------------------------------
 #ifdef CONFIG_ENCODER_ENABLE
 static void onEncoderVolume(uint8_t volume) {
     // Volume encoder: set absolute volume (0-127)
     g_a2dp.set_volume(volume);
     g_dsp.setVolume(volume);
     
     // Update LED effect with volume level
     #ifdef CONFIG_LED_MATRIX_ENABLE
     LedController::getInstance().setVolume(volume);
     #endif
     
     ESP_LOGI(TAG, "Encoder volume: %d", volume);
 }
 
 static void onEncoderPlayPause() {
     // Volume encoder button single click: toggle play/pause
     static bool isPlaying = true;
     if (isPlaying) {
         g_a2dp.pause();
     } else {
         g_a2dp.play();
     }
     isPlaying = !isPlaying;
     ESP_LOGI(TAG, "Encoder: %s", isPlaying ? "play" : "pause");
 }
 
 static void onEncoderNextTrack() {
     // Volume encoder button double click: next track
     g_a2dp.next();
     ESP_LOGI(TAG, "Encoder: next track");
 }
 
 static void onEncoderPrevTrack() {
     // Volume encoder button triple click: previous track
     g_a2dp.previous();
     ESP_LOGI(TAG, "Encoder: previous track");
 }
 
 static void onEncoderEq(int8_t bass, int8_t mid, int8_t treble) {
     // EQ encoders: apply new values and sync to app
     applyEq(bass, mid, treble);
     
     // Show EQ overlay on LED matrix
     #ifdef CONFIG_LED_MATRIX_ENABLE
     // Determine which EQ band changed (simple heuristic: compare to cached values)
     static int8_t lastBass = 0, lastMid = 0, lastTreble = 0;
     uint8_t changedType = 255;  // 255 = unknown/all
     if (bass != lastBass) changedType = 0;
     else if (mid != lastMid) changedType = 1;
     else if (treble != lastTreble) changedType = 2;
     lastBass = bass; lastMid = mid; lastTreble = treble;
     
     LedController::getInstance().setEq(bass, mid, treble, changedType);
     #endif
     
     ESP_LOGI(TAG, "Encoder EQ: %d/%d/%d", bass, mid, treble);
 }
 
 static void onEncoderBrightness(uint8_t brightness) {
     // Bass encoder button: adjust brightness
     #ifdef CONFIG_LED_MATRIX_ENABLE
     LedController::getInstance().setBrightness(brightness, true);  // Save to NVS
     
     // Notify app - copy current LED settings and update brightness
     const uint8_t* currentSettings = LedController::getInstance().getLedSettings();
     uint8_t settings[10];
     memcpy(settings, currentSettings, 10);
     settings[0] = brightness;  // Update brightness byte
     g_ble.updateLed(settings, sizeof(settings));
     
     ESP_LOGI(TAG, "Encoder brightness: %d", brightness);
     #endif
 }
 
 static void onEncoderPairingMode() {
     // Mid encoder button: enter pairing mode
     // If connected, disconnect first
     esp_a2d_connection_state_t state = g_a2dp.get_connection_state();
     
     // Set pairing mode flag - prevents disconnect callback from interfering
     g_pairingModeActive = true;
     
     if (state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
         ESP_LOGI(TAG, "Disconnecting current device to enter pairing mode...");
         g_a2dp.disconnect();
         vTaskDelay(pdMS_TO_TICKS(500));  // Wait for disconnect
     }
     
     // Reset I2S to default sample rate for sound playback
     g_sampleRate = APP_I2S_DEFAULT_SAMPLE_RATE;
     g_i2s.updateClock(APP_I2S_DEFAULT_SAMPLE_RATE);
     g_dsp.setSampleRate(APP_I2S_DEFAULT_SAMPLE_RATE);
     
     // Enable discoverable mode for new device pairing
     // ESP_BT_GENERAL_DISCOVERABLE = visible to all devices for pairing
     ESP_LOGI(TAG, "Entering pairing mode - device is now discoverable");
     g_a2dp.set_discoverability(ESP_BT_GENERAL_DISCOVERABLE);
     
     // Play pairing sound (exclusive mode - no A2DP during pairing anyway)
     // Use actual I2S sample rate to ensure proper resampling
     g_sound.play(SOUND_PAIRING, g_i2s.getSampleRate(), SOUND_MODE_EXCLUSIVE);
     
     // Start pairing mode LED animation (slow blue pulsing)
     #ifdef CONFIG_LED_MATRIX_ENABLE
     LedController::getInstance().setPairingMode(true);
     #endif
     
     // NOTE: g_pairingModeActive stays true until a device connects
     // This prevents the disconnect callback from resetting discoverability or LED animation
 }
 
 static void onEncoderEffectChange(int effectId, bool confirmed) {
     // Treble encoder: change LED effect
     #ifdef CONFIG_LED_MATRIX_ENABLE
     // Preview mode: don't save to NVS (save=false)
     // Confirmed mode: save to NVS (save=true)
     LedController::getInstance().setEffect(effectId, confirmed);
     
     // Always notify app of effect change (both preview and confirmed)
     g_ble.updateLedEffect((uint8_t)effectId);
     
     if (confirmed) {
         // Effect confirmed - also update full LED settings
         const uint8_t* currentSettings = LedController::getInstance().getLedSettings();
         g_ble.updateLed(currentSettings, 10);
         
         ESP_LOGI(TAG, "Encoder effect confirmed: %d (%s)", effectId, 
                  LedController::getInstance().getCurrentEffectName());
     } else {
         // Just previewing
         ESP_LOGI(TAG, "Encoder effect preview: %d (%s)", effectId, 
                  LedController::getInstance().getCurrentEffectName());
     }
     #endif
 }
 
 static void onEncoder3DSound(bool enabled) {
     (void)enabled;
     ESP_LOGI(TAG, "3D Sound removed in low-RAM build");
 }
 #endif
 
 // -----------------------------------------------------------
 // BLE callbacks
 // -----------------------------------------------------------
 static void onBleControl(uint8_t ctrl) {
     applyControlByte(ctrl, false);  // Don't notify back - command came from phone
     ESP_LOGI(TAG, "BLE control: 0x%02x", ctrl);
 }
 
 static void onBleEq(int8_t bass, int8_t mid, int8_t treble) {
     applyEq(bass, mid, treble, false);  // Don't notify back - command came from phone
     
     // Note: encoder sync already done in applyEq
     
     // Show EQ overlay on LED matrix (same as encoder)
     #ifdef CONFIG_LED_MATRIX_ENABLE
     // Determine which EQ band changed (compare to cached values)
     static int8_t lastBleBass = 0, lastBleMid = 0, lastBleTreble = 0;
     uint8_t changedType = 255;  // 255 = unknown/all
     if (bass != lastBleBass) changedType = 0;
     else if (mid != lastBleMid) changedType = 1;
     else if (treble != lastBleTreble) changedType = 2;
     lastBleBass = bass; lastBleMid = mid; lastBleTreble = treble;
     
     LedController::getInstance().setEq(bass, mid, treble, changedType);
     #endif
     
     ESP_LOGI(TAG, "BLE EQ: %d/%d/%d", bass, mid, treble);
 }
 
 // EQ preset values (matches BleUnifiedProtocol.kt)
 static const int8_t EQ_PRESETS[][3] = {
     {0, 0, 0},      // Flat
     {6, 2, 0},      // Bass Boost
     {0, 2, 6},      // Treble Boost
     {-2, 4, 2},     // Vocal
     {4, 0, 4},      // Rock
     {2, 3, 4},      // Pop
     {3, 0, 2},      // Jazz
     {0, 2, 3},      // Classical
     {5, 2, 4},      // Electronic
     {6, 0, 2},      // Hip Hop
     {2, 3, 3},      // Acoustic
     {4, 2, 4}       // Loudness
 };
 static const size_t EQ_PRESET_COUNT = sizeof(EQ_PRESETS) / sizeof(EQ_PRESETS[0]);
 
 static void onBleEqPreset(uint8_t presetId) {
     if (presetId >= EQ_PRESET_COUNT) {
         ESP_LOGW(TAG, "Invalid EQ preset: %d", presetId);
         return;
     }
     int8_t bass = EQ_PRESETS[presetId][0];
     int8_t mid = EQ_PRESETS[presetId][1];
     int8_t treble = EQ_PRESETS[presetId][2];
     applyEq(bass, mid, treble, false);  // Don't notify back - command came from phone
     ESP_LOGI(TAG, "BLE EQ preset %d: %d/%d/%d", presetId, bass, mid, treble);
 }
 
 static void onBleName(const char* name, size_t len) {
     g_settings.saveDeviceName(name);
     // Update Classic Bluetooth (A2DP) device name
     esp_bt_dev_set_device_name(name);
     // BLE advertising name is updated by ble_gatt
     ESP_LOGI(TAG, "BLE name changed: %s", name);
 }
 
 #ifdef CONFIG_LED_MATRIX_ENABLE
 static void onBleLedEffect(uint8_t effectId) {
     LedController::getInstance().setEffect(effectId);
     g_settings.saveLedEffect(effectId);
     // Don't notify back - command came from phone
     
     // Sync encoder controller's effect value
     #ifdef CONFIG_ENCODER_ENABLE
     EncoderController::getInstance().setCurrentEffect(effectId);
     #endif
     
     ESP_LOGI(TAG, "LED effect: %s", LedController::getInstance().getCurrentEffectName());
 }
 
 // Fast brightness conversion: 0-100 -> 0-255 using multiply+shift (no division)
 // Formula: (v * 41) >> 4 ≈ v * 2.5625 (true factor: 2.55)
 static inline uint8_t brightness100to255(uint8_t v) {
     uint16_t out = (v * 41) >> 4;
     return (out > 255) ? 255 : (uint8_t)out;
 }
 
 // LED settings callback - handles full 10-byte packet from phone:
 // Phone sends: [effectId, brightness(0-100), speed, r1, g1, b1, r2, g2, b2, gradient]
 // Internal format: [brightness(0-255), r1, g1, b1, r2, g2, b2, gradient, speed, effectId]
 static void onBleLedSettings(const uint8_t* data, size_t len) {
     if (len < 10) return;
     
     // Extract values from phone format
     uint8_t effectId       = data[0];
     uint8_t brightness100  = data[1];  // Phone sends 0-100
     uint8_t speed          = data[2];
     uint8_t r1             = data[3];
     uint8_t g1             = data[4];
     uint8_t b1             = data[5];
     uint8_t r2             = data[6];
     uint8_t g2             = data[7];
     uint8_t b2             = data[8];
     uint8_t gradient       = data[9];
     
     // Convert brightness from phone 0-100 to internal 0-255
     uint8_t brightness255 = brightness100to255(brightness100);
     
     // Reorder to internal format for LED controller (using 0-255 brightness)
     uint8_t reordered[10] = {
         brightness255, r1, g1, b1, r2, g2, b2, gradient, speed, effectId
     };
     
     // Pass to LED controller (handles brightness + ambient effect settings)
     LedController::getInstance().setLedSettings(reordered, 10);
     
     // Also set the effect
     LedController::getInstance().setEffect(effectId);
     
     // Sync encoder controller's brightness value (using internal 0-255)
     #ifdef CONFIG_ENCODER_ENABLE
     EncoderController::getInstance().setCurrentBrightness(brightness255);
     EncoderController::getInstance().setCurrentEffect(effectId);
     #endif
     
     // Don't notify back - command came from phone
 
     ESP_LOGI(TAG, "LED settings: effect=%d, brightness=%d->%d, speed=%d, gradient=%d",
              effectId, brightness100, brightness255, speed, gradient);
 }
 
 // LED brightness callback - handles brightness-only updates
 // Phone sends brightness in 0-100 range
 static void onBleLedBrightness(uint8_t brightness100) {
     // Convert from phone 0-100 to internal 0-255
     uint8_t brightness255 = brightness100to255(brightness100);
     
     LedController::getInstance().setBrightness(brightness255, true);  // Save to NVS
     
     // Sync encoder controller's brightness value (using internal 0-255)
     #ifdef CONFIG_ENCODER_ENABLE
     EncoderController::getInstance().setCurrentBrightness(brightness255);
     #endif
     
     // Don't notify back - command came from phone
     ESP_LOGI(TAG, "LED brightness: %d -> %d", brightness100, brightness255);
 }
 #endif
 
 static volatile uint32_t g_otaReceived = 0;
 static volatile uint32_t g_otaTotalSize = 0;
 
 // Parse OTA control command - supports both binary and ASCII protocols
 // Binary: 0x01 + 4-byte size (BEGIN), 0x03 (END), 0x04 (ABORT)
 // ASCII: "BEGIN:<size>", "END", "ABORT"
 static void onBleOtaCtrl(const uint8_t* data, size_t len) {
     ESP_LOGI(TAG, "OTA CTRL received: len=%u, first=0x%02X", (unsigned)len, len > 0 ? data[0] : 0);
     if (len < 1) return;
     
     // Check for ASCII protocol (starts with 'B', 'E', or 'A')
     if (data[0] == 'B' && len >= 6 && memcmp(data, "BEGIN:", 6) == 0) {
         // Parse ASCII: "BEGIN:<size>"
         char sizeBuf[16] = {0};
         size_t copyLen = len - 6;
         if (copyLen >= sizeof(sizeBuf)) copyLen = sizeof(sizeBuf) - 1;
         memcpy(sizeBuf, data + 6, copyLen);
         uint32_t size = (uint32_t)atoi(sizeBuf);
         
         ESP_LOGI(TAG, "OTA BEGIN (ASCII): %u bytes", (unsigned)size);
         
         // Pause phone playback via AVRCP, disable audio, stop I2S
         g_a2dp.pause();  // Send AVRCP pause to phone
         vTaskDelay(pdMS_TO_TICKS(50));  // Brief delay for AVRCP command
         g_a2dp.set_output_active(false);
         g_i2s.stop();  // Stop I2S to free resources
         
         // Enable LED OTA progress display
         #ifdef CONFIG_LED_MATRIX_ENABLE
         LedController::getInstance().setOtaMode(true);
         LedController::getInstance().setOtaProgress(0);
         #endif
         
         g_otaActive = true;
         g_otaReceived = 0;
         g_otaTotalSize = size;
         if (!g_update.begin(size)) {
             ESP_LOGE(TAG, "OTA begin failed: %s", g_update.errorString());
             g_ble.notifyOtaCtrl("BEGIN_ERR");
             g_otaActive = false;
             #ifdef CONFIG_LED_MATRIX_ENABLE
             LedController::getInstance().setOtaMode(false);
             #endif
             g_i2s.start();
         } else {
             ESP_LOGI(TAG, "OTA begin OK, waiting for data...");
             g_ble.notifyOtaCtrl("BEGIN_OK");
         }
         return;
     }
     
     if (data[0] == 'E' && len >= 3 && memcmp(data, "END", 3) == 0) {
         // Clear auto-finalize timer since we got END
         g_otaCheckPassedTime = 0;
         ESP_LOGI(TAG, "OTA END received, flushing remaining data...");
         
         // Wait for any remaining BLE packets to be processed
         // The BLE stack may have queued packets that haven't been delivered yet
         uint32_t lastReceived = g_otaReceived;
         for (int i = 0; i < 10; i++) {  // Wait up to 1 second
             vTaskDelay(pdMS_TO_TICKS(100));
             if (g_otaReceived == lastReceived) {
                 // No new data for 100ms, assume all data received
                 break;
             }
             lastReceived = g_otaReceived;
             ESP_LOGI(TAG, "OTA: still receiving data, now at %u bytes", (unsigned)g_otaReceived);
         }
         
         ESP_LOGI(TAG, "OTA END (ASCII): received %u / %u bytes total", (unsigned)g_otaReceived, (unsigned)g_otaTotalSize);
         
         // Check if we received all expected bytes
         if (g_otaTotalSize > 0 && g_otaReceived < g_otaTotalSize) {
             ESP_LOGW(TAG, "OTA incomplete: missing %u bytes", (unsigned)(g_otaTotalSize - g_otaReceived));
         }
         
         if (g_update.end(true)) {
             g_ble.notifyOtaCtrl("END_OK");
             ESP_LOGI(TAG, "OTA complete, restarting...");
             vTaskDelay(pdMS_TO_TICKS(500));
             esp_restart();
         } else {
             ESP_LOGE(TAG, "OTA end failed: %s", g_update.errorString());
             g_ble.notifyOtaCtrl("END_ERR");
         }
         g_otaActive = false;
         return;
     }
     
     // CHECK command - verify byte count before END
     if (data[0] == 'C' && len >= 6 && memcmp(data, "CHECK:", 6) == 0) {
         // Parse expected size from "CHECK:<size>"
         uint32_t expectedSize = 0;
         for (size_t i = 6; i < len && data[i] >= '0' && data[i] <= '9'; i++) {
             expectedSize = expectedSize * 10 + (data[i] - '0');
         }
         
         ESP_LOGI(TAG, "OTA CHECK: received %u / %u expected bytes", (unsigned)g_otaReceived, (unsigned)expectedSize);
         
         // Wait a moment for any remaining BLE packets
         vTaskDelay(pdMS_TO_TICKS(200));
         
         if (g_otaReceived >= expectedSize) {
             g_ble.notifyOtaCtrl("CHECK_OK");
             ESP_LOGI(TAG, "OTA CHECK passed");
             
             // Record timestamp for auto-finalize fallback
             g_otaCheckPassedTime = esp_timer_get_time();
             ESP_LOGI(TAG, "Starting auto-finalize timer (3s timeout)");
             
             // Start a task to auto-finalize if no END received within 3 seconds
             xTaskCreatePinnedToCore([](void* param) {
                 vTaskDelay(pdMS_TO_TICKS(3000));
                 
                 // Check if CHECK passed but no END was received (g_otaActive still true)
                 if (g_otaActive && g_otaCheckPassedTime > 0) {
                     ESP_LOGW(TAG, "OTA: No END received after CHECK passed, auto-finalizing...");
                     
                     if (g_update.end(true)) {
                         ESP_LOGI(TAG, "OTA auto-finalize complete, rebooting in 1s");
                         g_ble.notifyOtaCtrl("END_OK");
                         vTaskDelay(pdMS_TO_TICKS(1000));
                         esp_restart();
                     } else {
                         ESP_LOGE(TAG, "OTA auto-finalize failed: %s", g_update.errorString());
                         g_ble.notifyOtaCtrl("END_ERR");
                     }
                     g_otaActive = false;
                     g_otaCheckPassedTime = 0;
                 }
                 vTaskDelete(NULL);
             }, "ota_auto_end", 4096, NULL, 1, NULL, 0);
         } else {
             // Report how many bytes we're missing
             char resp[32];
             snprintf(resp, sizeof(resp), "CHECK_FAIL:%u", (unsigned)g_otaReceived);
             g_ble.notifyOtaCtrl(resp);
             ESP_LOGW(TAG, "OTA CHECK failed: missing %u bytes", (unsigned)(expectedSize - g_otaReceived));
         }
         return;
     }
     
     if (data[0] == 'A' && len >= 5 && memcmp(data, "ABORT", 5) == 0) {
         ESP_LOGW(TAG, "OTA ABORT (ASCII)");
         g_otaActive = false;
         g_otaReceived = 0;
         g_ble.notifyOtaCtrl("ABORT_OK");
         return;
     }
     
     // Binary protocol fallback
     uint8_t cmd = data[0];
     
     if (cmd == 0x01 && len >= 5) { // BEGIN
         uint32_t size = data[1] | (data[2] << 8) | (data[3] << 16) | (data[4] << 24);
         ESP_LOGI(TAG, "OTA BEGIN (binary): %u bytes", (unsigned)size);
         
         // Pause phone playback via AVRCP, disable audio, stop I2S
         g_a2dp.pause();  // Send AVRCP pause to phone
         vTaskDelay(pdMS_TO_TICKS(50));  // Brief delay for AVRCP command
         g_a2dp.set_output_active(false);
         g_i2s.stop();  // Stop I2S to free resources
         
         // Enable LED OTA progress display
         #ifdef CONFIG_LED_MATRIX_ENABLE
         LedController::getInstance().setOtaMode(true);
         LedController::getInstance().setOtaProgress(0);
         #endif
         
         g_otaActive = true;
         g_otaReceived = 0;
         g_otaTotalSize = size;
         if (!g_update.begin(size)) {
             ESP_LOGE(TAG, "OTA begin failed: %s", g_update.errorString());
             g_ble.notifyOtaCtrl("BEGIN_ERR");
             g_otaActive = false;
             #ifdef CONFIG_LED_MATRIX_ENABLE
             LedController::getInstance().setOtaMode(false);
             #endif
             g_i2s.start();
         } else {
             ESP_LOGI(TAG, "OTA begin OK, waiting for data...");
             g_ble.notifyOtaCtrl("BEGIN_OK");
         }
     } else if (cmd == 0x03) { // END
         ESP_LOGI(TAG, "OTA END (binary) received, flushing remaining data...");
         
         // Wait for any remaining BLE packets to be processed
         uint32_t lastReceived = g_otaReceived;
         for (int i = 0; i < 10; i++) {  // Wait up to 1 second
             vTaskDelay(pdMS_TO_TICKS(100));
             if (g_otaReceived == lastReceived) {
                 break;
             }
             lastReceived = g_otaReceived;
             ESP_LOGI(TAG, "OTA: still receiving data, now at %u bytes", (unsigned)g_otaReceived);
         }
         
         ESP_LOGI(TAG, "OTA END (binary): received %u / %u bytes total", (unsigned)g_otaReceived, (unsigned)g_otaTotalSize);
         
         if (g_otaTotalSize > 0 && g_otaReceived < g_otaTotalSize) {
             ESP_LOGW(TAG, "OTA incomplete: missing %u bytes", (unsigned)(g_otaTotalSize - g_otaReceived));
         }
         
         if (g_update.end(true)) {
             g_ble.notifyOtaCtrl("END_OK");
             ESP_LOGI(TAG, "OTA complete, restarting...");
             vTaskDelay(pdMS_TO_TICKS(500));
             esp_restart();
         } else {
             ESP_LOGE(TAG, "OTA end failed: %s", g_update.errorString());
             g_ble.notifyOtaCtrl("END_ERR");
         }
         g_otaActive = false;
     } else if (cmd == 0x04) { // ABORT
         ESP_LOGW(TAG, "OTA ABORT (binary)");
         g_otaActive = false;
         g_otaReceived = 0;
     } else {
         ESP_LOGW(TAG, "OTA CTRL unknown cmd: 0x%02X (char='%c')", cmd, (cmd >= 32 && cmd < 127) ? cmd : '?');
     }
 }
 
 static void onBleOtaData(const uint8_t* data, size_t len) {
     if (!g_otaActive || len == 0) return;
     
     // Ignore 1-byte flush packets (used by Android to sync at end)
     if (len == 1) return;
     
     // Direct write - rely on BLE ACK for ordering
     g_update.write(data, len);
     g_otaReceived += len;
     
     // Calculate and update progress
     uint8_t pct = (g_otaTotalSize > 0) ? (uint8_t)((uint64_t)g_otaReceived * 100 / g_otaTotalSize) : 0;
     
     // Update LED progress in real-time
     #ifdef CONFIG_LED_MATRIX_ENABLE
     LedController::getInstance().setOtaProgress(pct);
     #endif
     
     // Log progress at each 5% milestone
     static uint8_t lastPctLogged = 255;
     if (pct != lastPctLogged && (pct % 5 == 0 || pct == 100)) {
         ESP_LOGI(TAG, "OTA: %3u%% (%u / %u bytes)", pct, (unsigned)g_otaReceived, (unsigned)g_otaTotalSize);
         lastPctLogged = pct;
     }
 }
 
 // Unified OTA callback - handles all OTA commands from ble_unified protocol
 static void onBleOtaUnified(uint8_t cmd, const uint8_t* data, size_t len) {
     // Translate unified protocol commands to existing handlers
     // BleCmd::OTA_BEGIN (0x20) -> binary 0x01 + 4-byte size
     // BleCmd::OTA_DATA  (0x21) -> raw data (no seq prefix in unified)
     // BleCmd::OTA_END   (0x22) -> ASCII "END"
     // BleCmd::OTA_ABORT (0x23) -> ASCII "ABORT"
     
     switch (cmd) {
         case 0x20: {  // OTA_BEGIN - [size_lo, size_mid, size_hi, size_hhi]
             if (len >= 4) {
                 uint8_t pkt[5] = { 0x01, data[0], data[1], data[2], data[3] };
                 onBleOtaCtrl(pkt, 5);
             } else {
                 g_ble.sendOtaFailed(BleError::INVALID_PARAM);
             }
             break;
         }
         case 0x21: {  // OTA_DATA - [seq, data...]
             // Skip sequence byte if present, write raw data
             if (len > 1) {
                 onBleOtaData(data + 1, len - 1);
             }
             break;
         }
         case 0x22: {  // OTA_END
             onBleOtaCtrl((const uint8_t*)"END", 3);
             break;
         }
         case 0x23: {  // OTA_ABORT
             onBleOtaCtrl((const uint8_t*)"ABORT", 5);
             break;
         }
         default:
             ESP_LOGW(TAG, "Unknown OTA cmd: 0x%02X", cmd);
             break;
     }
 }
 
 // -----------------------------------------------------------
 // Sound upload callbacks
 // -----------------------------------------------------------
 // Control protocol (matching Android app):
 //   CMD [0x00, value] = Set mute (value: 0=unmute, 1=mute)
 //   CMD [0x01, type]  = Delete sound (type: 0=startup, 1=pairing, 2=connected, 3=maxvol)
 //   CMD [0x02]        = Request status (reply: status byte)
 // 
 // Upload protocol (on SoundData characteristic):
 //   START: [0x01][soundType][size(4)][reserved(4)] -> ACK: 0xA1
 //   DATA:  [0x02][seq(2)][len(2)][payload...]      -> ACK: 0xA2
 //   END:   [0x03]                                  -> ACK: 0xA3
 //   ERROR responses: 0xE0+code
 // NOTE: ACKs use 0xAx to avoid conflict with muted status (0x80-0x8F)
 
 // Sound upload state
 static volatile uint16_t g_soundUploadExpectedSeq = 0;
 static volatile bool g_soundUploadSeq8Mode = false;
 static TaskHandle_t g_soundSaveTaskHandle = nullptr;
 static volatile uint8_t g_soundSaveResult = 0;  // 0=pending, 0xA3=success, 0xE4+=error
 static volatile bool g_soundUploadEndRequested = false;
 static volatile bool g_soundUploadAbortRequested = false;
 static volatile uint8_t g_soundUploadAbortCode = 0x0B;
 static volatile bool g_soundUploadFailureNotified = false;
 
 static constexpr size_t SOUND_UPLOAD_CHUNK_MAX = 512;
 static constexpr size_t SOUND_UPLOAD_QUEUE_DEPTH = 4;
 static constexpr size_t SOUND_FILE_IO_BUFFER_SIZE = 2048;
 
 struct SoundUploadChunk {
     uint16_t seq;
     uint16_t len;
     uint8_t data[SOUND_UPLOAD_CHUNK_MAX];
 };
 
 static constexpr uint32_t SOUND_SAVE_STACK_SIZE = 6144;
 static QueueHandle_t g_soundUploadQueue = nullptr;
 static SoundUploadChunk* g_soundEnqueueChunk = nullptr;
 static SoundUploadChunk* g_soundWriteChunk = nullptr;
 static uint8_t* g_soundFileIoBuffer = nullptr;
 static uint8_t* g_soundLegacyUploadPacket = nullptr;
 
 // Forward declarations
 static void notifySoundAck(uint8_t code);
 static void freeSoundUploadResources();
 
 static bool ensureSoundUploadResources() {
     if (!g_soundUploadQueue) {
         g_soundUploadQueue = xQueueCreate(SOUND_UPLOAD_QUEUE_DEPTH, sizeof(SoundUploadChunk));
         if (!g_soundUploadQueue) {
             ESP_LOGE(TAG, "Failed to create sound upload queue");
             return false;
         }
     }
 
     if (!g_soundEnqueueChunk) {
         g_soundEnqueueChunk = (SoundUploadChunk*)heap_caps_malloc(sizeof(SoundUploadChunk),
                                                                   MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
     }
     if (!g_soundWriteChunk) {
         g_soundWriteChunk = (SoundUploadChunk*)heap_caps_malloc(sizeof(SoundUploadChunk),
                                                                 MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
     }
     if (!g_soundFileIoBuffer) {
         g_soundFileIoBuffer = (uint8_t*)heap_caps_malloc(SOUND_FILE_IO_BUFFER_SIZE,
                                                          MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
     }
     if (!g_soundLegacyUploadPacket) {
         g_soundLegacyUploadPacket = (uint8_t*)heap_caps_malloc(5 + SOUND_UPLOAD_CHUNK_MAX,
                                                                MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
     }
 
     if (!g_soundEnqueueChunk || !g_soundWriteChunk || !g_soundFileIoBuffer || !g_soundLegacyUploadPacket) {
         ESP_LOGE(TAG, "Failed to allocate sound upload resources");
         freeSoundUploadResources();
         return false;
     }
     return true;
 }
 
 static void freeSoundUploadResources() {
     if (g_soundUploadQueue) {
         vQueueDelete(g_soundUploadQueue);
         g_soundUploadQueue = nullptr;
     }
     if (g_soundEnqueueChunk) {
         heap_caps_free(g_soundEnqueueChunk);
         g_soundEnqueueChunk = nullptr;
     }
     if (g_soundWriteChunk) {
         heap_caps_free(g_soundWriteChunk);
         g_soundWriteChunk = nullptr;
     }
     if (g_soundFileIoBuffer) {
         heap_caps_free(g_soundFileIoBuffer);
         g_soundFileIoBuffer = nullptr;
     }
     if (g_soundLegacyUploadPacket) {
         heap_caps_free(g_soundLegacyUploadPacket);
         g_soundLegacyUploadPacket = nullptr;
     }
 }
 
 static void abortSoundUpload(uint8_t code, bool notifyNow = true) {
     g_soundUploadAbortCode = code;
     g_soundUploadAbortRequested = true;
     g_soundUploadEndRequested = true;
     if (notifyNow && !g_soundUploadFailureNotified) {
         g_soundUploadFailureNotified = true;
         g_ble.sendSoundFailed(code);
     }
 }
 
 // Sound upload resources are allocated only while an upload is active.
 // This returns roughly 5-6 KB of permanent internal heap during LDAC playback.
 static void preallocSoundSaveStack() {
     ESP_LOGI(TAG, "Sound upload resources are lazy: depth=%u chunk=%u bytes, stack=%u bytes",
              (unsigned)SOUND_UPLOAD_QUEUE_DEPTH,
              (unsigned)SOUND_UPLOAD_CHUNK_MAX,
              (unsigned)SOUND_SAVE_STACK_SIZE);
 }
 
 // Task to save sound file (deferred from BLE callback to avoid crash)
 static void soundSaveTask(void* param) {
     uint8_t result = 0xE5;  // Default: no data error
     
     // Copy volatile values to local
     uint32_t expectedSize = g_soundUploadSize;
     SoundType type = g_soundUploadType;
     const char* path = (type < SOUND_TYPE_COUNT) ? SOUND_PATHS[type] : nullptr;
     FILE* f = nullptr;
     bool fileOpened = false;
     bool headerChecked = false;
     uint8_t wavHeader[44] = {0};
     size_t wavHeaderLen = 0;
     uint32_t written = 0;
     
     ESP_LOGI(TAG, "Sound streaming task started: type=%d, expected=%u", 
              type, (unsigned)expectedSize);
     
     if (expectedSize == 0) {
         ESP_LOGE(TAG, "Sound streaming task: empty upload");
         goto notify_and_cleanup;
     }
 
     if (type >= SOUND_TYPE_COUNT) {
         ESP_LOGE(TAG, "Invalid sound type: %d", type);
         result = 0x09;  // Error: invalid type
         goto notify_and_cleanup;
     }
 
     if (!g_soundUploadQueue) {
         ESP_LOGE(TAG, "Sound upload queue is not initialized");
         result = 0x06;
         goto notify_and_cleanup;
     }
 
     if (expectedSize < 44) {
         ESP_LOGE(TAG, "Sound upload too small for WAV");
         result = 0x08;  // Error: data too small
         goto notify_and_cleanup;
     }
     
     // Pause BLE meter notifications during SPIFFS write.
     // Sound upload ACKs still flow, but meter spam stays out of the BLE stack.
     g_pauseBleNotifications = true;
 
     ESP_LOGI(TAG, "Sound streaming task: writing to SPIFFS...");
     
     {
         size_t total = 0, used = 0;
         esp_err_t spiffs_err = esp_spiffs_info("spiffs", &total, &used);
         if (spiffs_err != ESP_OK) {
             ESP_LOGE(TAG, "SPIFFS not mounted! Error: %s", esp_err_to_name(spiffs_err));
             result = 0x05;  // Error: SPIFFS not mounted
             goto notify_and_cleanup;
         }
 
         size_t replacing = 0;
         struct stat st;
         if (stat(path, &st) == 0 && st.st_size > 0) {
             replacing = (size_t)st.st_size;
         }
 
         size_t available = (total > used) ? (total - used) : 0;
         if (available + replacing < expectedSize) {
             ESP_LOGE(TAG, "SPIFFS full! Need %u bytes, have %u free (+%u replacing)",
                      (unsigned)expectedSize, (unsigned)available, (unsigned)replacing);
             result = 0x06;  // Error: no space
             goto notify_and_cleanup;
         }
 
         if (replacing > 0) {
             ESP_LOGI(TAG, "Deleting existing file: %s (%u bytes)", path, (unsigned)replacing);
             remove(path);
         }
 
         ESP_LOGI(TAG, "SPIFFS check: %u KB total, %u KB used, uploading %u KB",
                  (unsigned)(total / 1024), (unsigned)(used / 1024), (unsigned)(expectedSize / 1024));
     }
 
     f = fopen(path, "wb");
     if (!f) {
         ESP_LOGE(TAG, "Failed to open file: %s (errno=%d: %s)", path, errno, strerror(errno));
         result = 0x04;  // Error: file open failed
         goto notify_and_cleanup;
     }
     fileOpened = true;
     setvbuf(f, (char*)g_soundFileIoBuffer, _IOFBF, SOUND_FILE_IO_BUFFER_SIZE);
 
     while (!g_soundUploadAbortRequested) {
         if (xQueueReceive(g_soundUploadQueue, g_soundWriteChunk, pdMS_TO_TICKS(250)) != pdTRUE) {
             if (g_soundUploadEndRequested) {
                 break;
             }
             continue;
         }
 
         if (g_soundWriteChunk->len == 0) {
             continue;
         }
 
         size_t accepted = g_soundWriteChunk->len;
         if (written + accepted > expectedSize) {
             accepted = expectedSize - written;
         }
 
         if (!headerChecked && wavHeaderLen < sizeof(wavHeader)) {
             size_t headerCopy = sizeof(wavHeader) - wavHeaderLen;
             if (headerCopy > accepted) headerCopy = accepted;
             memcpy(wavHeader + wavHeaderLen, g_soundWriteChunk->data, headerCopy);
             wavHeaderLen += headerCopy;
             if (wavHeaderLen == sizeof(wavHeader)) {
                 if (memcmp(wavHeader, "RIFF", 4) != 0 || memcmp(wavHeader + 8, "WAVE", 4) != 0) {
                     ESP_LOGE(TAG, "Sound upload is not a WAV file");
                     result = 0x08;
                     goto notify_and_cleanup;
                 }
                 headerChecked = true;
             }
         }
 
         if (accepted > 0) {
             size_t w = fwrite(g_soundWriteChunk->data, 1, accepted, f);
             if (w != accepted) {
                 ESP_LOGE(TAG, "Write error at offset %u", (unsigned)written);
                 result = 0x04;
                 goto notify_and_cleanup;
             }
             written += (uint32_t)w;
             g_soundUploadReceived = written;
             g_ble.sendSoundReady();
 
             taskYIELD();
         }
 
         if (written >= expectedSize) {
             break;
         }
     }
 
     if (g_soundUploadAbortRequested) {
         ESP_LOGW(TAG, "Sound upload aborted");
         result = g_soundUploadAbortCode ? g_soundUploadAbortCode : 0x0B;
         goto notify_and_cleanup;
     }
 
     if (written != expectedSize) {
         ESP_LOGE(TAG, "Sound upload size mismatch: wrote %u / %u", (unsigned)written, (unsigned)expectedSize);
         result = 0x05;
         goto notify_and_cleanup;
     }
 
     if (!headerChecked) {
         ESP_LOGE(TAG, "Sound upload ended before full WAV header");
         result = 0x08;
         goto notify_and_cleanup;
     }
 
     fflush(f);
     fclose(f);
     fileOpened = false;
     ESP_LOGI(TAG, "Sound saved successfully: %s (%u bytes)", path, (unsigned)written);
     g_sound.refreshStatus();
     result = 0;  // Success (0 means no error)
     
 notify_and_cleanup:
     if (fileOpened && f) {
         fclose(f);
         remove(path);
     }
 
     // Resume BLE notifications after SPIFFS operations complete
     g_pauseBleNotifications = false;
 
     if (g_soundUploadQueue) {
         xQueueReset(g_soundUploadQueue);
     }
 
     g_soundUploadActive = false;
     g_soundUploadSize = 0;
     g_soundUploadReceived = 0;
     g_soundUploadQueued = 0;
     g_soundUploadExpectedSeq = 0;
     g_soundUploadSeq8Mode = false;
     g_soundUploadEndRequested = false;
     g_soundUploadAbortRequested = false;
     g_soundUploadAbortCode = 0x0B;
     
     // Longer delay before sending notification to let BLE stack recover
     vTaskDelay(pdMS_TO_TICKS(200));
     
     // Send result notification using proper unified protocol response
     if (result == 0) {
         ESP_LOGI(TAG, "Sound save complete, sending SOUND_COMPLETE (0x32)");
         g_ble.sendSoundComplete();
     } else {
         ESP_LOGI(TAG, "Sound save failed, sending SOUND_FAILED with error: 0x%02X", result);
         if (!g_soundUploadFailureNotified) {
             g_ble.sendSoundFailed(result);
         }
     }
     g_soundUploadFailureNotified = false;
     
     // Send updated sound status (which sounds are present, muted, etc.)
     vTaskDelay(pdMS_TO_TICKS(150));
     g_ble.updateSoundStatus(g_sound.getStatus());
     
     g_soundSaveTaskHandle = nullptr;
     freeSoundUploadResources();
     vTaskDelete(NULL);
 }
 
 static void notifySoundAck(uint8_t code) {
     uint8_t ack[1] = { code };
     g_ble.updateSoundStatus(ack[0]);
 }
 
 static void notifySoundAckSeq(uint8_t code, uint16_t seq) {
     // For DATA ACK, we need to send [0x82][seq_lo][seq_hi]
     // But notifySoundStatus only sends 1 byte, so we need a different approach
     // For now, just send 0x82 and rely on sequential processing
     g_ble.updateSoundStatus(code);
 }
 
 // ========== Unified BLE Sound callbacks ==========
 
 static void onBleSoundMute(bool muted) {
     g_sound.setMuted(muted);
     g_settings.saveSoundMuted(g_sound.isMuted());
     g_ble.updateSoundStatus(g_sound.getStatus());
     ESP_LOGI(TAG, "Sound mute set: %s", muted ? "MUTED" : "UNMUTED");
 }
 
 // Task to delete sound file (deferred from BLE callback to avoid crash)
 static void soundDeleteTask(void* param) {
     SoundType type = g_soundDeleteType;
     
     // Pause BLE meter notifications during critical SPIFFS operation
     // This prevents BLE stack congestion that causes disconnects
     g_pauseBleNotifications = true;
     
     // Longer delay to let BLE stack fully finish any pending operations
     vTaskDelay(pdMS_TO_TICKS(200));
     
     ESP_LOGI(TAG, "Deleting sound: type=%d", type);
     
     // Yield multiple times to let BT/BLE tasks complete any pending work
     for (int i = 0; i < 5; i++) {
         taskYIELD();
     }
     vTaskDelay(pdMS_TO_TICKS(50));
     
     g_sound.deleteSound(type);
     
     // Longer delay after SPIFFS delete to let flash subsystem fully settle
     // and allow BLE stack to recover from any blocking
     vTaskDelay(pdMS_TO_TICKS(300));
     
     // Resume BLE notifications before sending status update
     g_pauseBleNotifications = false;
     
     // Additional delay to let any queued meter updates flush
     vTaskDelay(pdMS_TO_TICKS(100));
     
     // Yield before BLE operation
     taskYIELD();
     
     g_ble.updateSoundStatus(g_sound.getStatus());
     
     g_soundDeletePending = false;
     ESP_LOGI(TAG, "Sound deleted: type=%d", type);
     vTaskDelete(NULL);
 }
 
 static void onBleSoundDelete(uint8_t soundType) {
     SoundType type = (SoundType)(soundType & 0x03);
     if (type <= SOUND_MAX_VOLUME && !g_soundDeletePending) {
         // Defer delete to separate task to avoid crashing from BLE callback
         g_soundDeletePending = true;
         g_soundDeleteType = type;
         xTaskCreatePinnedToCore(soundDeleteTask, "snd_del", 4096, nullptr, 1, nullptr, 0);
         ESP_LOGI(TAG, "Sound delete queued: type=%d", type);
     }
 }
 
 // Unified sound upload callback - handles START/DATA/END commands from ble_unified
 static void onBleSoundUpload(uint8_t cmd, const uint8_t* data, size_t len);
 
 // Legacy callbacks for backward compatibility (kept for reference during transition)
 static void onBleSoundCtrl(const uint8_t* data, size_t len) {
     if (len < 1) return;
     uint8_t cmd = data[0];
     
     ESP_LOGI(TAG, "SOUND CTRL: cmd=0x%02X, len=%u", cmd, (unsigned)len);
     
     // Set mute [0x00, value]
     if (cmd == 0x00 && len >= 2) {
         bool mute = (data[1] != 0);
         g_sound.setMuted(mute);
         g_settings.saveSoundMuted(g_sound.isMuted());
         g_ble.updateSoundStatus(g_sound.getStatus());
         ESP_LOGI(TAG, "Sound mute set: %s", mute ? "MUTED" : "UNMUTED");
         return;
     }
     
     // Delete sound [0x01, type]
     if (cmd == 0x01 && len >= 2) {
         SoundType type = (SoundType)(data[1] & 0x03);
         if (type <= SOUND_MAX_VOLUME && !g_soundDeletePending) {
             // Defer delete to separate task to avoid crashing from BLE callback
             g_soundDeletePending = true;
             g_soundDeleteType = type;
             xTaskCreatePinnedToCore(soundDeleteTask, "snd_del", 4096, nullptr, 1, nullptr, 0);
             ESP_LOGI(TAG, "Sound delete queued (legacy): type=%d", type);
         }
         return;
     }
     
     // Request status [0x02]
     if (cmd == 0x02) {
         g_ble.updateSoundStatus(g_sound.getStatus());
         return;
     }
 }
 
 static void onBleSoundData(const uint8_t* data, size_t len) {
     if (len < 1) return;
     
     uint8_t pktType = data[0];
     // START packet: [0x01][soundType][size(4)][reserved(4)]
     if (pktType == 0x01 && len >= 6) {
         uint8_t rawType = data[1];
         SoundType type = (SoundType)(rawType & 0x03);
         uint32_t size = data[2] | (data[3] << 8) | (data[4] << 16) | (data[5] << 24);
         
         ESP_LOGI(TAG, "Sound START parsed: raw_byte1=0x%02X, type=%d, size=%u", rawType, type, (unsigned)size);
         
         // Validate type - reject if raw value > 3 (indicates invalid value like -1/0xFF)
         if (rawType > 3) {
             ESP_LOGE(TAG, "Invalid sound type: 0x%02X (must be 0-3)", rawType);
             g_ble.sendSoundFailed(0x09);  // Error: invalid type
             return;
         }
         
         if (g_soundUploadActive || g_soundSaveTaskHandle != nullptr) {
             ESP_LOGW(TAG, "Sound upload already active");
             g_ble.sendSoundFailed(0x07);  // Error: busy
             return;
         }
 
 
         if (size < 44) {
             ESP_LOGE(TAG, "Sound too small: %u bytes", (unsigned)size);
             g_ble.sendSoundFailed(0x08);  // Error: invalid WAV
             return;
         }
 
         // Validate size against SPIFFS at START instead of internal RAM.
         size_t total = 0, used = 0;
         if (esp_spiffs_info("spiffs", &total, &used) != ESP_OK) {
             ESP_LOGE(TAG, "SPIFFS not mounted for sound upload");
             g_ble.sendSoundFailed(0x05);
             return;
         }
         if (size > total) {
             ESP_LOGE(TAG, "Sound too large: %u bytes (SPIFFS total %u bytes)", (unsigned)size, (unsigned)total);
             g_ble.sendSoundFailed(0x01);  // Error: too large
             return;
         }
 
         if (!ensureSoundUploadResources()) {
             g_soundUploadActive = false;
             g_ble.sendSoundFailed(0x06);
             return;
         }
         xQueueReset(g_soundUploadQueue);
 
         g_soundUploadType = type;
         g_soundUploadSize = size;
         g_soundUploadReceived = 0;
         g_soundUploadQueued = 0;
         g_soundUploadExpectedSeq = 0;
         g_soundUploadEndRequested = false;
         g_soundUploadAbortRequested = false;
         g_soundUploadAbortCode = 0x0B;
         g_soundUploadFailureNotified = false;
         g_soundUploadActive = true;
 
         BaseType_t taskOk = xTaskCreatePinnedToCore(
             soundSaveTask,
             "snd_save",
             SOUND_SAVE_STACK_SIZE,
             nullptr,
             1,
             &g_soundSaveTaskHandle,
             0);
 
         if (taskOk != pdPASS) {
             ESP_LOGE(TAG, "Failed to create sound streaming task");
             g_soundSaveTaskHandle = nullptr;
             g_soundUploadActive = false;
             freeSoundUploadResources();
             g_ble.sendSoundFailed(0x06);  // Error: task creation failed
             return;
         }
 
         g_ble.sendSoundReady();  // Ready for data chunks
         ESP_LOGI(TAG, "Sound streaming upload started, sent SOUND_READY (0x31)");
         return;
     }
     
     // DATA packet: [0x02][seq(2)][len(2)][payload...]
     if (pktType == 0x02 && len >= 5 && g_soundUploadActive && g_soundSaveTaskHandle) {
         uint16_t seq = data[1] | (data[2] << 8);
         uint16_t payloadLen = data[3] | (data[4] << 8);
         
         if (len < 5 + payloadLen) {
             ESP_LOGE(TAG, "DATA packet truncated: got %u, expected %u", (unsigned)len, (unsigned)(5 + payloadLen));
             abortSoundUpload(0x03);
             return;
         }
 
         if (payloadLen > SOUND_UPLOAD_CHUNK_MAX) {
             ESP_LOGE(TAG, "DATA packet too large: %u > %u", (unsigned)payloadLen, (unsigned)SOUND_UPLOAD_CHUNK_MAX);
             abortSoundUpload(0x03);
             return;
         }
         
         // Check sequence
         if (seq != g_soundUploadExpectedSeq) {
             ESP_LOGW(TAG, "Seq mismatch: got %u, expected %u", seq, g_soundUploadExpectedSeq);
             abortSoundUpload(0x03);
             return;
         }
         
         if (g_soundUploadQueued + payloadLen > g_soundUploadSize) {
             ESP_LOGE(TAG, "Sound upload overflow: queued %u + %u > %u",
                      (unsigned)g_soundUploadQueued, (unsigned)payloadLen, (unsigned)g_soundUploadSize);
             abortSoundUpload(0x03);
             return;
         }
 
         *g_soundEnqueueChunk = {};
         g_soundEnqueueChunk->seq = seq;
         g_soundEnqueueChunk->len = payloadLen;
         if (payloadLen > 0) {
             memcpy(g_soundEnqueueChunk->data, data + 5, payloadLen);
         }
 
         if (xQueueSend(g_soundUploadQueue, g_soundEnqueueChunk, pdMS_TO_TICKS(100)) != pdTRUE) {
             ESP_LOGW(TAG, "Sound upload queue full");
             abortSoundUpload(0x0A);
             return;
         }
 
         g_soundUploadQueued += payloadLen;
         uint32_t queuedBytes = g_soundUploadQueued;
         g_soundUploadExpectedSeq = g_soundUploadSeq8Mode ? (uint8_t)(seq + 1) : (uint16_t)(seq + 1);
         
         // Log progress every 10%
         static uint8_t lastPct = 255;
         uint8_t pct = (g_soundUploadSize > 0) ? (uint8_t)((uint64_t)queuedBytes * 100 / g_soundUploadSize) : 0;
         if (pct / 10 != lastPct / 10) {
             ESP_LOGI(TAG, "Sound upload queued: %u%% (%u/%u)", pct, (unsigned)queuedBytes, (unsigned)g_soundUploadSize);
             lastPct = pct;
         }
         return;
     }
     
     // END packet: [0x03]
     if (pktType == 0x03) {
         ESP_LOGI(TAG, "Sound END: received %u / %u bytes", 
                  (unsigned)g_soundUploadReceived, (unsigned)g_soundUploadSize);
         
         if (g_soundUploadActive && g_soundSaveTaskHandle) {
             g_soundUploadEndRequested = true;
             ESP_LOGI(TAG, "Sound upload END accepted; writer will finalize");
         } else {
             g_ble.sendSoundFailed(0x05);  // Error: no data
             g_soundUploadActive = false;
         }
         return;
     }
 
     if (pktType == 0x04) {
         ESP_LOGW(TAG, "Sound upload abort requested");
         g_soundUploadAbortRequested = true;
         return;
     }
 }
 
 // Unified sound upload callback - adapts from BleCmd:: format to legacy format
 static void onBleSoundUpload(uint8_t cmd, const uint8_t* data, size_t len) {
     // The unified protocol uses a simplified format:
     // BleCmd::SOUND_UP_START (0x12): [type, size_lo, size_mid, size_hi] (4 bytes, 3-byte size)
     // BleCmd::SOUND_UP_DATA  (0x13): [seq(1), data...] (1 + N bytes)
     // BleCmd::SOUND_UP_END   (0x14): (no payload)
     //
     // Legacy format expected by onBleSoundData:
     // START: [0x01][type][size(4 bytes LE)] (6 bytes minimum)
     // DATA:  [0x02][seq_lo][seq_hi][len_lo][len_hi][payload...] (5 + N bytes)
     // END:   [0x03] (1 byte)
     
     switch (cmd) {
         case 0x12: {  // SOUND_UP_START
             // Phone sends: [type][size_lo][size_mid][size_hi] (3-byte size)
             // Legacy expects: [0x01][type][size_lo][size_mid][size_hi][size_hh] (4-byte size)
             // We need to pad the size to 4 bytes
             if (len < 4) {
                 ESP_LOGE(TAG, "SOUND_UP_START too short: %u bytes", (unsigned)len);
                 return;
             }
             uint8_t pkt[6];
             pkt[0] = 0x01;      // pktType = START
             pkt[1] = data[0];   // soundType
             pkt[2] = data[1];   // size_lo
             pkt[3] = data[2];   // size_mid
             pkt[4] = data[3];   // size_hi
             pkt[5] = 0;         // size_hh (pad to 4-byte size, assuming <16MB)
             g_soundUploadSeq8Mode = true;
             onBleSoundData(pkt, 6);
             break;
         }
         case 0x13: {  // SOUND_UP_DATA
             // Convert from [seq(1)][data...] to [0x02][seq_lo][seq_hi][len_lo][len_hi][data...]
             if (len < 1) return;
             
             uint8_t seq = data[0];
             size_t payloadLen = len - 1;
             if (payloadLen > SOUND_UPLOAD_CHUNK_MAX) {
                 ESP_LOGE(TAG, "SOUND_UP_DATA too large: %u", (unsigned)payloadLen);
                 abortSoundUpload(0x03);
                 return;
             }
             const uint8_t* payload = data + 1;
             
             if (!g_soundLegacyUploadPacket) {
                 ESP_LOGE(TAG, "Sound upload resources missing");
                 abortSoundUpload(0x06);
                 return;
             }
 
             // Build legacy format packet
             size_t newLen = 5 + payloadLen;  // [0x02][seq16][len16][payload]
             uint8_t* pkt = g_soundLegacyUploadPacket;
             pkt[0] = 0x02;
             pkt[1] = seq;         // seq_lo
             pkt[2] = 0;           // seq_hi (always 0 since unified uses 1-byte seq)
             pkt[3] = payloadLen & 0xFF;         // len_lo
             pkt[4] = (payloadLen >> 8) & 0xFF;  // len_hi
             if (payloadLen > 0) memcpy(pkt + 5, payload, payloadLen);
             
             onBleSoundData(pkt, newLen);
             break;
         }
         case 0x14: {  // SOUND_UP_END
             uint8_t pkt[1] = { 0x03 };
             onBleSoundData(pkt, 1);
             break;
         }
         default:
             ESP_LOGW(TAG, "Unknown sound upload cmd: 0x%02X", cmd);
             return;
     }
 }
 
 // -----------------------------------------------------------
 // A2DP callbacks
 // -----------------------------------------------------------
 
 // Helper to get codec name from type
 static const char* getCodecName(esp_a2d_mct_t type) {
     switch (type) {
         case ESP_A2D_MCT_SBC:      return "SBC";
         case ESP_A2D_MCT_M12:      return "MPEG-1/2";
         case ESP_A2D_MCT_M24:      return "AAC";
         case ESP_A2D_MCT_ATRAC:    return "ATRAC";
         case ESP_A2D_MCT_NON_A2DP: return "Vendor Codec (see CODEC_CONFIG log above for details)";
         default:                   return "Unknown";
     }
 }
 
 static void applyCodecConfig(const CodecConfigRequest& cfg) {
     uint32_t rate = cfg.rate ? cfg.rate : 44100;
     uint8_t bps = cfg.bitsPerSample ? cfg.bitsPerSample : 16;
     uint8_t channels = cfg.channels ? cfg.channels : 2;
 
     ESP_LOGI(TAG, "Applying codec: %s (0x%02X), %u Hz, %u-bit, %u ch",
              getCodecName(cfg.codecType), cfg.codecType,
              (unsigned)rate, (unsigned)bps, (unsigned)channels);
 
     g_pipeline.clear();
     g_overlayMixer.clear();
 
     g_sampleRate = rate;
     g_bitsPerSample = bps;
     g_channels = channels;
 
     g_i2s.updateClock(rate);
     g_pipeline.setSampleRate(rate);
     g_dsp.setSampleRate(rate);
     // Analysis runs at every sample rate now: the Goertzel is fixed-point and
     // its window scales with fs, so 88.2/96 kHz cost the same per-window work as
     // 44.1/48 kHz and stay well within the render headroom (no underruns).
     g_dsp.setAnalysisEnabled(true);
 
     g_lastCodecConfigTime = cfg.timestampUs;
     g_connectedSoundPending = true;
     g_codecReconfiguring = false;
     logHeap("after_codec_config");
 }
 
 static void onCodecConfig(uint32_t rate, uint8_t bps, uint8_t channels) {
     if (rate == 0) rate = 44100;
     
     // Get codec type (the library's codec_config.c already logs vendor ID details)
     esp_a2d_mct_t codecType = g_a2dp.get_audio_type();
     const char* codecName = getCodecName(codecType);
     
     ESP_LOGI(TAG, "========================================");
     ESP_LOGI(TAG, "CODEC CONFIGURATION RECEIVED");
     ESP_LOGI(TAG, "========================================");
     ESP_LOGI(TAG, "  Codec type: %s (0x%02X)", codecName, codecType);
     ESP_LOGI(TAG, "  Sample rate: %u Hz", (unsigned)rate);
     ESP_LOGI(TAG, "  Bits/sample: %u", (unsigned)bps);
     ESP_LOGI(TAG, "  Channels: %u", (unsigned)channels);
     
     // Note: For vendor codecs, check CODEC_CONFIG log above which shows:
     // - "configure LDAC codec" for LDAC (Vendor ID: 0x012D, Codec ID: 0x00AA)
     // - "configure aptX codec" for aptX (Vendor ID: 0x004F, Codec ID: 0x0001)
     // - "configure aptX-HD codec" for aptX-HD (Vendor ID: 0x00D7, Codec ID: 0x0024)
     
     ESP_LOGI(TAG, "========================================");
     
     // Pause pipeline during codec reconfiguration to prevent race conditions.
     g_codecReconfiguring = true;
     g_pipeline.clear();
 
     CodecConfigRequest req = {
         .rate = rate,
         .bitsPerSample = bps,
         .channels = channels,
         .codecType = codecType,
         .timestampUs = esp_timer_get_time()
     };
 
     applyCodecConfig(req);
 }
 
 static void onStreamData(const uint8_t* data, uint32_t len) {
     if (g_codecReconfiguring || g_otaActive) {
         return;
     }
 
     g_pipeline.enqueueFromBt(data, len, g_bitsPerSample, g_channels);
 }
 
 // Real-time audio logging must be off by default. ESP_LOGI writes to UART/USB
 // and can block long enough to create a once-per-second tick/stutter at
 // 96 kHz LDAC. Enable only when debugging buffer counters.
 #ifndef APP_AUDIO_RENDER_STATS_LOG_MS
 #define APP_AUDIO_RENDER_STATS_LOG_MS 0u
 #endif
 
 static void audioRenderTask(void* arg) {
 #if APP_AUDIO_RENDER_STATS_LOG_MS > 0
     uint32_t lastStatsMs = 0;
     uint32_t lastWrites = 0;
     uint32_t lastDrops = 0;
     uint32_t lastCatchup = 0;
     uint32_t lastUnderruns = 0;
 #endif
     const TickType_t waitTicks = pdMS_TO_TICKS(10) ? pdMS_TO_TICKS(10) : 1;
 
 #if APP_AUDIO_RENDER_STATS_LOG_MS > 0
     auto maybeLogStats = [&]() {
         uint32_t nowMs = (uint32_t)(esp_timer_get_time() / 1000ULL);
         if ((uint32_t)(nowMs - lastStatsMs) < APP_AUDIO_RENDER_STATS_LOG_MS) return;
 
         uint32_t fill = g_pipeline.getFillFrames();
         uint32_t cap = g_pipeline.getCapacityFrames();
         uint32_t maxFill = g_pipeline.getAndResetMaxFillFrames();
         uint32_t writes = g_pipeline.getWriteCount();
         uint32_t drops = g_pipeline.getDropCount();
         uint32_t catchup = g_pipeline.getCatchupDrops();
         uint32_t underruns = g_pipeline.getUnderrunCount();
         float clipPeak = 0.0f;
         uint32_t clipSat = 0, clipOver = 0;
         g_dsp.getAndResetClipStats(clipPeak, clipSat, clipOver);
 
         ESP_LOGI(TAG, "audio: fill=%u/%u max=%u writes/s=%u over=%u catch=%u under=%u i2s=%u/%ub vol=%u peak=%u%% lim=%u clip=%u",
                  (unsigned)fill, (unsigned)cap, (unsigned)maxFill,
                  (unsigned)(writes - lastWrites),
                  (unsigned)(drops - lastDrops),
                  (unsigned)(catchup - lastCatchup),
                  (unsigned)(underruns - lastUnderruns),
                  (unsigned)g_i2s.getSampleRate(),
                  (unsigned)g_bitsPerSample,
                  (unsigned)g_dsp.getVolume(),
                  (unsigned)(clipPeak * 100.0f + 0.5f),
                  (unsigned)clipSat,
                  (unsigned)clipOver);
 
         lastWrites = writes;
         lastDrops = drops;
         lastCatchup = catchup;
         lastUnderruns = underruns;
         lastStatsMs = nowMs;
     };
 #endif
 
     while (true) {
         ulTaskNotifyTake(pdTRUE, waitTicks);
 
         uint32_t chunks = 0;
         while (g_pipeline.renderAvailable(g_dsp, g_i2s)) {
             // i2s.write() already blocks/yields on DMA room. Do not sleep for a
             // full RTOS tick here; that can create audio modulation/stutter.
             if (++chunks >= 16) {
                 taskYIELD();
                 chunks = 0;
 #if APP_AUDIO_RENDER_STATS_LOG_MS > 0
                 maybeLogStats();
 #endif
             }
         }
 
 #if APP_AUDIO_RENDER_STATS_LOG_MS > 0
         maybeLogStats();
 #endif
     }
 }
 
 static void onConnectionState(esp_a2d_connection_state_t state, void* user) {
     const char* stateStr = "Unknown";
     switch (state) {
         case ESP_A2D_CONNECTION_STATE_DISCONNECTED: stateStr = "DISCONNECTED"; break;
         case ESP_A2D_CONNECTION_STATE_CONNECTING:   stateStr = "CONNECTING"; break;
         case ESP_A2D_CONNECTION_STATE_CONNECTED:    stateStr = "CONNECTED"; break;
         case ESP_A2D_CONNECTION_STATE_DISCONNECTING: stateStr = "DISCONNECTING"; break;
     }
     
     ESP_LOGI(TAG, ">>> A2DP Connection: %s", stateStr);
     
     // Set connection timestamp early (on CONNECTING) to ensure grace period works
     // Volume events can arrive before CONNECTED state
     if (state == ESP_A2D_CONNECTION_STATE_CONNECTING) {
         g_lastConnectTime = esp_timer_get_time();
     }
     
     if (state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
         // Mark as disconnected
         g_a2dpConnected = false;
         
         // Record disconnect time for codec switch detection
         g_lastDisconnectTime = esp_timer_get_time();
         
         // Cancel any pending connected sound
         g_connectedSoundPending = false;
         
         ESP_LOGW(TAG, "A2DP disconnected - waiting for phone to reconnect with new codec...");
         g_pipeline.clear();
         g_i2s.zeroDMA();  // Clear any stale audio data
         
         // Only reset sample rate and discoverability if NOT in pairing mode
         // (pairing mode handler sets these intentionally and they should persist)
         if (!g_pairingModeActive) {
             g_sampleRate = APP_I2S_DEFAULT_SAMPLE_RATE;  // Reset sample rate for sound effects
             
             g_a2dp.set_discoverability(ESP_BT_GENERAL_DISCOVERABLE);
             ESP_LOGI(TAG, "Discoverability enabled after disconnect");
             
             // Stop pairing animation if running (only if not intentionally in pairing mode)
             #ifdef CONFIG_LED_MATRIX_ENABLE
             LedController::getInstance().setPairingMode(false);
             #endif
         } else {
             ESP_LOGI(TAG, "In pairing mode - keeping discoverable and LED animation active");
         }
     } else if (state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
         ESP_LOGI(TAG, "A2DP connected - codec should already be configured");
         
         // Mark as connected
         g_a2dpConnected = true;
         
         // Restart BLE advertising so the app can still connect via BLE while A2DP is active
         g_ble.restartAdvertising();
         
         // Record connection time - used to ignore initial volume report from phone
         g_lastConnectTime = esp_timer_get_time();
         
         // Check if this is a rapid reconnect (codec switch) - don't play connected sound
         int64_t timeSinceDisconnect = esp_timer_get_time() - g_lastDisconnectTime;
         bool isCodecSwitch = (g_lastDisconnectTime > 0) && (timeSinceDisconnect < CODEC_SWITCH_TIMEOUT_US);
         
         if (isCodecSwitch) {
             ESP_LOGI(TAG, "Codec switch detected (reconnect in %lld ms) - skipping connected sound", 
                      timeSinceDisconnect / 1000);
         }
         
         // Clear pairing mode flag - we're now connected
         g_pairingModeActive = false;
         
         ESP_LOGI(TAG, "Pairing mode cleared after A2DP connection");
         
         // Stop any playing sound (e.g., pairing sound)
         if (g_sound.isPlaying()) {
             g_sound.stop();
         }
         
         // Connected sound is now played by buttonsTask after codec stabilizes
         // (see g_connectedSoundPending flag set in onCodecConfig)
         
         // Show pairing success animation if we were in pairing mode
         #ifdef CONFIG_LED_MATRIX_ENABLE
         if (LedController::getInstance().isPairingModeActive()) {
             LedController::getInstance().showPairingSuccess();
         } else {
             // Just in case, stop pairing mode
             LedController::getInstance().setPairingMode(false);
         }
         #endif
         // Restart BLE advertising so the app can connect while A2DP is active
         g_ble.restartAdvertising();
     }
 }
 
 static void onAudioState(esp_a2d_audio_state_t state, void* user) {
     const char* stateStr = "Unknown";
     switch (state) {
         case ESP_A2D_AUDIO_STATE_SUSPEND: stateStr = "SUSPEND"; break;
         case ESP_A2D_AUDIO_STATE_STARTED: stateStr = "STARTED"; break;
     }
     
     ESP_LOGI(TAG, ">>> A2DP Audio State: %s", stateStr);
     
     if (state == ESP_A2D_AUDIO_STATE_SUSPEND) {
         smooth30_dB = smooth60_dB = smooth100_dB = -60.0f;
         g_pipeline.clear();
         g_pipeline.clear();
     } else if (state == ESP_A2D_AUDIO_STATE_STARTED) {
         // Restart BLE advertising so app can connect while audio is playing
         g_ble.restartAdvertising();
     }
 }
 
 // -----------------------------------------------------------
 // Button handling task
 // -----------------------------------------------------------
 static void buttonsTask(void* arg) {
     bool lastBtn1 = true, btn1Pressed = false;
     uint32_t debounce1 = 0, pressStart1 = 0;
     bool lastBtn2 = true, btn2State = true;
     uint32_t debounce2 = 0;
     
     // LED effect button (can be separate or shared with button 2)
     #ifdef CONFIG_LED_MATRIX_ENABLE
     bool lastBtnLed = true, btnLedState = true;
     uint32_t debounceLed = 0;
     #ifdef CONFIG_LED_EFFECT_BUTTON_GPIO
         const gpio_num_t ledBtnGpio = (gpio_num_t)CONFIG_LED_EFFECT_BUTTON_GPIO;
         // Only init if different from existing buttons
         if (ledBtnGpio != APP_BUTTON1_GPIO && ledBtnGpio != APP_BUTTON2_GPIO) {
             gpio_config_t ledBtn = {};
             ledBtn.intr_type = GPIO_INTR_DISABLE;
             ledBtn.mode = GPIO_MODE_INPUT;
             ledBtn.pin_bit_mask = (1ULL << ledBtnGpio);
             ledBtn.pull_up_en = GPIO_PULLUP_ENABLE;
             gpio_config(&ledBtn);
         }
     #else
         const gpio_num_t ledBtnGpio = (gpio_num_t)19;  // Default
     #endif
     #endif
 
     while (true) {
         uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
         
         // Check if connected sound is pending and codec has stabilized
         if (g_connectedSoundPending) {
             int64_t timeSinceCodecConfig = esp_timer_get_time() - g_lastCodecConfigTime;
             if (timeSinceCodecConfig >= CODEC_STABLE_DELAY_US) {
                 g_connectedSoundPending = false;
                 
                 // Check if this is a codec switch (rapid reconnect) - don't play sound
                 int64_t timeSinceDisconnect = g_lastCodecConfigTime - g_lastDisconnectTime;
                 bool isCodecSwitch = (g_lastDisconnectTime > 0) && (timeSinceDisconnect < CODEC_SWITCH_TIMEOUT_US);
                 
                 if (isCodecSwitch) {
                     ESP_LOGI(TAG, "Codec switch detected - skipping connected sound");
                 } else if (g_a2dpConnected && g_pipeline.isAudioActive()) {
                     ESP_LOGI(TAG, "A2DP stream already active - skipping connected sound to protect audio startup");
                 } else if (g_sampleRate > 48000 ||
                            heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) < 24 * 1024) {
                     ESP_LOGI(TAG, "Skipping connected sound: sr=%u free_internal=%u",
                              (unsigned)g_sampleRate,
                              (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
                 } else {
                     ESP_LOGI(TAG, "Codec stable for %lld ms - playing connected sound at %u Hz",
                              timeSinceCodecConfig / 1000, (unsigned)g_i2s.getSampleRate());
                     g_sound.play(SOUND_CONNECTED, g_i2s.getSampleRate(),
                                  g_a2dpConnected ? SOUND_MODE_OVERLAY : SOUND_MODE_EXCLUSIVE);
                 }
             }
         }
         
         bool r1 = gpio_get_level((gpio_num_t)APP_BUTTON1_GPIO);
         if (r1 != lastBtn1) debounce1 = now;
         if ((now - debounce1) > 25) {
             if (!btn1Pressed && r1 == 0) {
                 btn1Pressed = true;
                 pressStart1 = now;
             } else if (btn1Pressed && r1 == 1) {
                 btn1Pressed = false;
                 if ((now - pressStart1) < 1000) {
                     g_dsp.setBassBoost(!g_dsp.isBassBoostEnabled());
                 } else {
                     g_dsp.setChannelFlip(!g_dsp.isChannelFlipEnabled());
                 }
                 g_settings.saveControl(g_dsp.isBassBoostEnabled(), 
                                        g_dsp.isChannelFlipEnabled(), 
                                        g_dsp.isBypassEnabled());
                 g_ble.updateControl(getControlByte());
             }
         }
         lastBtn1 = r1;
 
         bool r2 = gpio_get_level((gpio_num_t)APP_BUTTON2_GPIO);
         if (r2 != lastBtn2) debounce2 = now;
         if ((now - debounce2) > 25 && r2 != btn2State) {
             btn2State = r2;
             if (r2 == 1) {
                 g_dsp.setBypass(!g_dsp.isBypassEnabled());
                 g_settings.saveControl(g_dsp.isBassBoostEnabled(),
                                        g_dsp.isChannelFlipEnabled(),
                                        g_dsp.isBypassEnabled());
                 g_ble.updateControl(getControlByte());
             }
         }
         lastBtn2 = r2;
 
         // LED effect cycle button
         #ifdef CONFIG_LED_MATRIX_ENABLE
         bool rLed = gpio_get_level(ledBtnGpio);
         if (rLed != lastBtnLed) debounceLed = now;
         if ((now - debounceLed) > 25 && rLed != btnLedState) {
             btnLedState = rLed;
             if (rLed == 1) {  // Button released
                 LedController::getInstance().nextEffect();
                 uint8_t newEffect = LedController::getInstance().getCurrentEffectId();
                 g_settings.saveLedEffect(newEffect);
                 g_ble.updateLedEffect(newEffect);
                 ESP_LOGI(TAG, "LED effect: %s", LedController::getInstance().getCurrentEffectName());
             }
         }
         lastBtnLed = rLed;
         #endif
 
         vTaskDelay(pdMS_TO_TICKS(10));
     }
 }
 
 // -----------------------------------------------------------
 // Beat detection + levels task
 // -----------------------------------------------------------
 static void beatTask(void* arg) {
     uint32_t lastLevelMs = 0;
     uint32_t lastBeatMs = 0;
     bool flashActive = false;
     uint32_t flashOffMs = 0;
     float bassSmooth = 0, bassAvg = 0;
 
     while (true) {
         uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
 
         if (!g_otaActive) {
             float bass = g_dsp.getGoertzel60Lin() + g_dsp.getGoertzel100Lin();
 
             if (bass >= APP_BASS_MIN_LEVEL) {
                 bassSmooth += APP_BASS_SMOOTH_ALPHA * (bass - bassSmooth);
                 bassAvg += APP_BASS_AVG_ALPHA * (bassSmooth - bassAvg);
                 
                 float ratio = (bassAvg > 1e-6f) ? (bassSmooth / (bassAvg + 1e-6f)) : 0;
                 if (ratio > APP_BASS_RATIO_THRESH && (now - lastBeatMs) > APP_BEAT_MIN_INTERVAL_MS) {
                     gpio_set_level((gpio_num_t)APP_BEAT_LED_GPIO, 1);
                     flashActive = true;
                     flashOffMs = now + APP_BEAT_FLASH_DURATION_MS;
                     lastBeatMs = now;
                     
                     // Signal beat to LED matrix
                     #ifdef CONFIG_LED_MATRIX_ENABLE
                     setLedBeat(true);
                     #endif
                 }
             } else {
                 bassSmooth *= 0.9f;
                 bassAvg *= 0.999f;
             }
 
             if (flashActive && now >= flashOffMs) {
                 gpio_set_level((gpio_num_t)APP_BEAT_LED_GPIO, 0);
                 flashActive = false;
                 
                 // Clear beat signal
                 #ifdef CONFIG_LED_MATRIX_ENABLE
                 setLedBeat(false);
                 #endif
             }
 
             // Update BLE levels every 50ms
             if ((now - lastLevelMs) >= APP_LEVELS_UPDATE_MS) {
                 lastLevelMs = now;
                 
                 bool audioStopped = (now - g_pipeline.getLastProcessMs()) > 100;
                 if (audioStopped) {
                     smooth30_dB = smooth30_dB * 0.85f + (-60.0f) * 0.15f;
                     smooth60_dB = smooth60_dB * 0.85f + (-60.0f) * 0.15f;
                     smooth100_dB = smooth100_dB * 0.85f + (-60.0f) * 0.15f;
                 } else {
                     // Drive the app meter from the same Goertzel bands used
                     // by bass-reactive LED effects, with peak fallback so
                     // tracks with weak sub-bass still show movement.
                     float g30 = g_dsp.getGoertzel30dB();
                     float g60 = g_dsp.getGoertzel60dB();
                     float g100 = g_dsp.getGoertzel100dB();
                     float p30 = g_dsp.getPeakDB(0);
                     float p60 = g_dsp.getPeakDB(1);
                     float p100 = g_dsp.getPeakDB(2);
                     smooth30_dB += 0.45f * (((g30 > p30) ? g30 : p30) - smooth30_dB);
                     smooth60_dB += 0.45f * (((g60 > p60) ? g60 : p60) - smooth60_dB);
                     smooth100_dB += 0.45f * (((g100 > p100) ? g100 : p100) - smooth100_dB);
                 }
 
                 // Convert dB to display position (0-100)
                 // Range: -60dB to 0dB maps to 0-100 (phone max is 120)
                 auto dbToPos = [](float dB) -> int {
                     // Clamp to range
                     if (dB < -60.0f) dB = -60.0f;
                     if (dB > 0.0f) dB = 0.0f;
                     // Map -60..0 to 0..100
                     int v = (int)roundf((dB + 60.0f) * (100.0f / 60.0f));
                     return (v < 0) ? 0 : ((v > 100) ? 100 : v);
                 };
 
                 if (g_ble.isConnected() && !g_pauseBleNotifications) {
                     g_ble.updateLevels(dbToPos(smooth30_dB), dbToPos(smooth60_dB), dbToPos(smooth100_dB));
                 }
             }
         } else {
             gpio_set_level((gpio_num_t)APP_BEAT_LED_GPIO, 0);
         }
 
         vTaskDelay(pdMS_TO_TICKS(10));
     }
 }
 
 // -----------------------------------------------------------
 // app_main
 // -----------------------------------------------------------
 extern "C" void app_main(void) {
     // ========================================================================
     // RECOVERY BOOT CHECK - Must be done FIRST before anything else
     // Hold Button 1 (GPIO 18) during boot to enter recovery mode
     // ========================================================================
     {
         // Configure button GPIO as input with pullup (before NVS init for fastest check)
         gpio_config_t btn_cfg = {};
         btn_cfg.mode = GPIO_MODE_INPUT;
         btn_cfg.pin_bit_mask = (1ULL << APP_BUTTON1_GPIO);
         btn_cfg.pull_up_en = GPIO_PULLUP_ENABLE;
         gpio_config(&btn_cfg);
         
         // Read button state (active low = pressed when 0)
         bool buttonPressed = (gpio_get_level((gpio_num_t)APP_BUTTON1_GPIO) == 0);
         
         if (buttonPressed) {
             ESP_LOGW(TAG, "Recovery button held - checking for recovery partition...");
             
             // Wait a moment to debounce and confirm intentional press
             vTaskDelay(pdMS_TO_TICKS(500));
             buttonPressed = (gpio_get_level((gpio_num_t)APP_BUTTON1_GPIO) == 0);
             
             if (buttonPressed) {
                 // Find recovery partition by label (uses "ota_2" subtype for bootloader compatibility)
                 const esp_partition_t* recovery = esp_partition_find_first(
                     ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_2, "recovery");
                 
                 if (recovery) {
                     ESP_LOGW(TAG, "Booting into RECOVERY MODE...");
                     ESP_LOGW(TAG, "Recovery partition: %s @ 0x%lx, size=%lu",
                              recovery->label, (unsigned long)recovery->address,
                              (unsigned long)recovery->size);
                     
                     // Set boot partition to recovery and restart
                     esp_err_t err = esp_ota_set_boot_partition(recovery);
                     if (err == ESP_OK) {
                         vTaskDelay(pdMS_TO_TICKS(100));
                         esp_restart();
                         // Should never reach here
                     } else {
                         ESP_LOGE(TAG, "Failed to set boot partition: %s", esp_err_to_name(err));
                     }
                 } else {
                     ESP_LOGW(TAG, "No recovery partition found - continuing normal boot");
                 }
             }
         }
     }
     // ========================================================================
     // END RECOVERY BOOT CHECK - Continue with normal boot
     // ========================================================================
 
     // NVS init
     esp_err_t ret = nvs_flash_init();
     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
         ESP_ERROR_CHECK(nvs_flash_erase());
         ESP_ERROR_CHECK(nvs_flash_init());
     }
     logHeap("after_nvs");
     ESP_LOGI(TAG, "Booting ESP32 A2DP Sink + BLE + DSP...");
 
     // OTA validation
     const esp_partition_t* running = esp_ota_get_running_partition();
     esp_ota_img_states_t state;
     if (esp_ota_get_state_partition(running, &state) == ESP_OK) {
         if (state == ESP_OTA_IMG_PENDING_VERIFY) {
             esp_ota_mark_app_valid_cancel_rollback();
             ESP_LOGI(TAG, "OTA validated");
         }
     }
 
     // Initialize SPIFFS for sound storage
     esp_vfs_spiffs_conf_t spiffsConf = {
         .base_path = "/spiffs",
         .partition_label = "spiffs",  // Must match partition table name
         .max_files = 5,
         .format_if_mount_failed = true
     };
     ret = esp_vfs_spiffs_register(&spiffsConf);
     if (ret == ESP_OK) {
         ESP_LOGI(TAG, "SPIFFS mounted");
         size_t total = 0, used = 0;
         esp_spiffs_info("spiffs", &total, &used);
         ESP_LOGI(TAG, "SPIFFS: %u KB total, %u KB used", (unsigned)(total / 1024), (unsigned)(used / 1024));
     } else {
         ESP_LOGW(TAG, "SPIFFS mount failed: %s", esp_err_to_name(ret));
     }
     logHeap("after_spiffs");
 
     // Sound upload buffers/queue are lazy-allocated only during upload to preserve
     // internal RAM for LDAC/A2DP playback. This just logs the configured sizes.
     preallocSoundSaveStack();
 
     // Load settings
     g_settings.load();
     bool bassBoost, channelFlip, bypass;
     int8_t eqBass, eqMid, eqTreble;
     std::string deviceName;
     bool soundMuted;
     g_settings.getControl(bassBoost, channelFlip, bypass);
     g_settings.getEQ(eqBass, eqMid, eqTreble);
     g_settings.getDeviceName(deviceName);
     soundMuted = g_settings.loadSoundMuted();
 
     // Initialize sound player (sets muted state and scans SPIFFS for existing sounds)
     g_sound.init(APP_I2S_DEFAULT_SAMPLE_RATE);
     g_sound.setMuted(soundMuted);
     ESP_LOGI(TAG, "Sound player initialized: muted=%d, status=0x%02X", soundMuted, g_sound.getStatus());
     logHeap("after_sound");
 
     // Initialize DSP
     g_dsp.setSampleRate(APP_I2S_DEFAULT_SAMPLE_RATE);
     g_dsp.setEQ(eqBass, eqMid, eqTreble, APP_I2S_DEFAULT_SAMPLE_RATE);
     g_dsp.setBassBoost(bassBoost);
     g_dsp.setChannelFlip(channelFlip);
     g_dsp.setBypass(bypass);
 
     logHeap("after_dsp");
 
     // Initialize I2S
     if (g_i2s.init(APP_I2S_DEFAULT_SAMPLE_RATE) != ESP_OK) {
         ESP_LOGE(TAG, "I2S init failed");
         return;
     }
     
     // Set up I2S sample rate change callback to notify SoundPlayer
     g_i2s.setSampleRateCallback([](uint32_t newRate) {
         g_sound.setTargetSampleRate(newRate);
     });
 
     // Set up sound player I2S write function (after I2S init)
     g_sound.setI2SWriteFunc([](const uint8_t* data, size_t len) -> size_t {
         return g_i2s.write(data, len);
     });
 
     logHeap("after_i2s");
 
     // Initialize audio pipeline
     if (!g_pipeline.init()) {
         ESP_LOGE(TAG, "Audio pipeline init failed");
         return;
     }
     
     // Initialize overlay mixer for sound effects during playback
     if (!g_overlayMixer.init()) {
         ESP_LOGE(TAG, "Overlay mixer init failed");
         return;
     }
     g_pipeline.setOverlayMixer(&g_overlayMixer);
     logHeap("after_pipeline");
 
     // Set up sound player overlay push function for mixing with BT audio
     g_sound.setOverlayPushFunc([](const int32_t* samples, size_t frames) -> size_t {
         size_t n = g_overlayMixer.pushSamples(samples, frames);
         if (n > 0 && audioTaskHandle) {
             xTaskNotifyGive(audioTaskHandle);
         }
         return n;
     });
     // Duck the BT audio while an overlay sound (e.g. max-volume) plays, then restore.
     g_sound.setOverlayCtrlFuncs(
         []() { g_overlayMixer.startOverlay(); },
         []() { g_overlayMixer.stopOverlay(); });
     
     // Set callback to skip I2S writes when exclusive sound is playing
     g_pipeline.setSkipWriteCallback([]() -> bool {
         return g_sound.isExclusivePlaying();
     });
 
     const UBaseType_t high_priority = configMAX_PRIORITIES - 2;
     if (xTaskCreatePinnedToCore(audioRenderTask, "audio_render", 5120, nullptr,
                                 high_priority, &audioTaskHandle, 0) != pdPASS) {
         ESP_LOGE(TAG, "Failed to create audio_render task");
         return;
     }
     g_pipeline.setRenderTaskHandle(audioTaskHandle);
     ESP_LOGI(TAG, "audio_render task started on Core 0 priority %u", (unsigned)high_priority);
 
     // GPIO init (buttons + LED)
     gpio_config_t io = {};
     io.intr_type = GPIO_INTR_DISABLE;
     io.mode = GPIO_MODE_INPUT;
     io.pin_bit_mask = (1ULL << APP_BUTTON1_GPIO) | (1ULL << APP_BUTTON2_GPIO);
     io.pull_up_en = GPIO_PULLUP_ENABLE;
     gpio_config(&io);
 
     gpio_config_t led = {};
     led.mode = GPIO_MODE_OUTPUT;
     led.pin_bit_mask = (1ULL << APP_BEAT_LED_GPIO);
     gpio_config(&led);
     gpio_set_level((gpio_num_t)APP_BEAT_LED_GPIO, 0);
 
     // ========================================================================
     // STARTUP SEQUENCE: Play startup sound + LED animation BEFORE BLE/A2DP
     // Both must complete before initializing anything else
     // ========================================================================
     
     // Start LED matrix task first (needed for startup animation)
     #ifdef CONFIG_LED_MATRIX_ENABLE
     startLedTask(&g_dsp, 3, 4096);  // 4KB stack - internal RAM only
     logHeap("after_led");
     ESP_LOGI(TAG, "LED matrix started on GPIO %d", CONFIG_LED_MATRIX_GPIO);
     
     // Give LED task time to initialize before requesting animation
     vTaskDelay(pdMS_TO_TICKS(100));
     
     // Start startup sound and LED animation together
     bool hasStartupSound = g_sound.hasSound(SOUND_STARTUP);
     if (hasStartupSound) {
         ESP_LOGI(TAG, "Playing startup sound + animation...");
         g_sound.play(SOUND_STARTUP, g_i2s.getSampleRate(), SOUND_MODE_EXCLUSIVE);
     } else {
         ESP_LOGI(TAG, "Playing startup animation (no sound)...");
     }
     LedController::getInstance().requestStartupAnimation();
     
     // Small delay to ensure animation starts (flag is set before playStartupAnimation runs)
     vTaskDelay(pdMS_TO_TICKS(50));
     
     // Wait for BOTH sound AND LED animation to complete
     while (g_sound.isPlaying() || LedController::getInstance().isStartupAnimationRunning()) {
         vTaskDelay(pdMS_TO_TICKS(50));
     }
     ESP_LOGI(TAG, "Startup sequence complete - sound and LED animation finished");
     
     #else
     // No LED matrix - just play startup sound and wait for completion
     if (g_sound.hasSound(SOUND_STARTUP)) {
         ESP_LOGI(TAG, "Playing startup sound...");
         g_sound.play(SOUND_STARTUP, g_i2s.getSampleRate(), SOUND_MODE_EXCLUSIVE);
         while (g_sound.isPlaying()) {
             vTaskDelay(pdMS_TO_TICKS(50));
         }
         ESP_LOGI(TAG, "Startup sound complete");
     }
     #endif
     // ========================================================================
     // END STARTUP SEQUENCE - Now initialize BLE, A2DP, and other services
     // ========================================================================
 
     // Centralized Bluetooth stack initialization (shared for BLE + Classic BT)
     // This must be done once before any BLE or A2DP profile registration.
     {
         esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
         esp_err_t err = esp_bt_controller_init(&bt_cfg);
         if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
             ESP_LOGE(TAG, "BT controller init failed: %s", esp_err_to_name(err));
         }
         err = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
         if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
             ESP_LOGE(TAG, "BT controller enable failed: %s", esp_err_to_name(err));
         }
         esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
         bluedroid_cfg.ssp_en = true;
         err = esp_bluedroid_init_with_cfg(&bluedroid_cfg);
         if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
             ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(err));
         }
         err = esp_bluedroid_enable();
         if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
             ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(err));
         }
         logHeap("after_bt_stack");
     }
 
     // Initialize BLE
     #ifdef CONFIG_LED_MATRIX_ENABLE
     // Unified BLE callbacks: EQ, EqPreset, Control, Name, LED, LedEffect, LedBright, SoundMute, SoundDelete, SoundData, OTA
     g_ble.setCallbacks(
         onBleEq,            // EqCallback
         onBleEqPreset,      // EqPresetCallback
         onBleControl,       // ControlCallback
         onBleName,          // NameCallback
         onBleLedSettings,   // LedCallback (full 10-byte settings)
         onBleLedEffect,     // LedEffectCallback
         onBleLedBrightness, // LedBrightnessCallback
         onBleSoundMute,     // SoundMuteCallback
         onBleSoundDelete,   // SoundDeleteCallback
         onBleSoundUpload,   // SoundDataCallback
         onBleOtaUnified     // OtaCallback
     );
     uint8_t savedLedEffect = g_settings.loadLedEffect();
     uint8_t savedBrightness = LedController::getInstance().getBrightness();
     g_ble.init(deviceName.c_str(), APP_FW_VERSION, getControlByte(), eqBass, eqMid, eqTreble, savedLedEffect, savedBrightness);
     #else
     // Unified BLE callbacks without LED matrix
     g_ble.setCallbacks(
         onBleEq,            // EqCallback
         onBleEqPreset,      // EqPresetCallback
         onBleControl,       // ControlCallback
         onBleName,          // NameCallback
         nullptr,            // LedCallback (no LED)
         nullptr,            // LedEffectCallback
         nullptr,            // LedBrightnessCallback
         onBleSoundMute,     // SoundMuteCallback
         onBleSoundDelete,   // SoundDeleteCallback
         onBleSoundUpload,   // SoundDataCallback
         onBleOtaUnified     // OtaCallback
     );
     g_ble.init(deviceName.c_str(), APP_FW_VERSION, getControlByte(), eqBass, eqMid, eqTreble);
     #endif
     
     // Initialize BLE sound status with current sound player status
     // This ensures the correct status is sent when a client connects
     g_ble.setSoundStatus(g_sound.getStatus());
     logHeap("after_ble");
 
     g_ble.setDisconnectCallback([]() {
         if (g_soundUploadActive) {
             abortSoundUpload(0x0B, false);
         }
         ESP_LOGI(TAG, "BLE disconnect - internal heap free=%u min_ever=%u",
                  (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                  (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
     });
 
     // ========================================================================
     // A2DP Initialization
     // ========================================================================
     
     // Delay before A2DP initialization to avoid AVRCP race condition
     // ESP-IDF Bluetooth stack has a bug where simultaneous AVRCP connection attempts
     // can crash in bta_av_rc_create - waiting allows the phone's connection attempt
     // to start first and avoids the race condition
     vTaskDelay(pdMS_TO_TICKS(1000));
     
     // Start A2DP
     g_a2dp.set_output_active(false);
     g_a2dp.set_stream_reader(onStreamData, false);
     g_a2dp.set_codec_config_callback(onCodecConfig);
     g_a2dp.set_auto_reconnect(false);
     g_a2dp.set_task_core(1);
     g_a2dp.set_on_connection_state_changed(onConnectionState);
     g_a2dp.set_on_audio_state_changed(onAudioState);
     
     // Volume change callback - sync with LED and encoder controller
     g_a2dp.set_avrc_rn_volumechange([](int volume) {
         g_dsp.setVolume((uint8_t)volume);
         #ifdef CONFIG_LED_MATRIX_ENABLE
         LedController::getInstance().setVolume((uint8_t)volume);
         #endif
         #ifdef CONFIG_ENCODER_ENABLE
         // Sync encoder controller with phone's volume so local adjustments are relative
         EncoderController::getInstance().setCurrentVolume((uint8_t)volume);
         ESP_LOGI(TAG, "Phone volume changed to %d - encoder synced", volume);
         #endif
         
         // Skip max volume sound during connection grace period (ignore initial volume report)
         // Also skip if g_lastConnectTime is 0 (no connection yet - system still initializing)
         int64_t timeSinceConnect = esp_timer_get_time() - g_lastConnectTime;
         if (g_lastConnectTime == 0 || timeSinceConnect < VOLUME_GRACE_PERIOD_US) {
             ESP_LOGI(TAG, "Ignoring volume report during connection grace period (%lld ms)", 
                      timeSinceConnect / 1000);
             return;
         }
         
         // Play max volume sound when hitting maximum.
         // IMPORTANT: never use exclusive mode while A2DP is connected. The audio
         // render task must remain the only I2S writer; the sound effect is mixed
         // through OverlayMixer so I2S/DMA state cannot be corrupted.
         // Only play if we're actually connected (not during connection setup)
         // Rate limit: 2 second cooldown to prevent crash from rapid volume spam
         if (volume >= 122 && g_a2dpConnected && g_sound.hasSound(SOUND_MAX_VOLUME)) {
             int64_t now = esp_timer_get_time();
             int64_t timeSinceLastMaxSound = now - g_lastMaxVolumeSound;
             if (timeSinceLastMaxSound >= MAX_VOLUME_SOUND_COOLDOWN_US) {
                 ESP_LOGI(TAG, "Max volume reached - playing sound");
                 g_lastMaxVolumeSound = now;
                 // Do not clear the overlay from the AVRCP/BT callback. startOverlay()
                 // clears stale frames inside the sound task path, and calling clear()
                 // here can block the BT control task if the render task owns the
                 // overlay mutex. The max-volume sound is overlay-only; it never
                 // touches I2S directly.
                 g_sound.play(SOUND_MAX_VOLUME, g_i2s.getSampleRate(), SOUND_MODE_OVERLAY);
             } else {
                 ESP_LOGD(TAG, "Max volume sound rate limited (%lld ms since last)", 
                          timeSinceLastMaxSound / 1000);
             }
         }
     });
     
     g_a2dp.start(deviceName.c_str());
     logHeap("after_a2dp");
 
     // Wait for A2DP stack to fully initialize before changing discoverability
     // The library sets connectable=true during init, we need to override after
     // Longer delay to allow AVRCP connection to stabilize (ESP-IDF race condition workaround)
     vTaskDelay(pdMS_TO_TICKS(500));
     
     // Keep discoverable at startup so the phone can always recover after a bad bond
     // or codec negotiation failure. Pairing mode can still be used for feedback UI.
     g_a2dp.set_discoverability(ESP_BT_GENERAL_DISCOVERABLE);
     ESP_LOGI(TAG, "A2DP started as '%s' - discoverable/connectable", deviceName.c_str());
     logHeap("after_bt_ready");
 
     // Log memory status before starting tasks
     ESP_LOGI(TAG, "Free heap: internal=%u KB",
              (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) / 1024));
 
     xTaskCreatePinnedToCore(buttonsTask, "buttons", 3072, nullptr, 3, nullptr, 0);
     xTaskCreatePinnedToCore(beatTask, "beat", 2048, nullptr, 2, nullptr, 0);
 
     // Initialize and start encoder task
     #ifdef CONFIG_ENCODER_ENABLE
     {
         auto& enc = EncoderController::getInstance();
         enc.setVolumeCallback(onEncoderVolume);
         enc.setPlayPauseCallback(onEncoderPlayPause);
         enc.setNextTrackCallback(onEncoderNextTrack);
         enc.setPrevTrackCallback(onEncoderPrevTrack);
         enc.setEqCallback(onEncoderEq);
         enc.setBrightnessCallback(onEncoderBrightness);
         enc.setPairingModeCallback(onEncoderPairingMode);
         enc.setEffectCallback(onEncoderEffectChange);
         // 3D sound removed in low-RAM build.
         
         // Set initial values from NVS
         enc.setCurrentVolume((uint8_t)g_a2dp.get_volume());  // Get current volume (0-127)
         enc.setCurrentEq(eqBass, eqMid, eqTreble);
         // 3D sound removed in low-RAM build; do not load/enable it.
         
         #ifdef CONFIG_LED_MATRIX_ENABLE
         enc.setCurrentBrightness(LedController::getInstance().getBrightness());
         enc.setCurrentEffect(LedController::getInstance().getCurrentEffectId());
         enc.setMaxEffect(LED_EFFECT_COUNT);
         #endif
         
         startEncoderTask();  // Uses default priority 2, pinned to core 0
         ESP_LOGI(TAG, "Encoder controller started on I2C SDA=%d, SCL=%d",
                  ENCODER_I2C_SDA_GPIO, ENCODER_I2C_SCL_GPIO);
     }
     #endif
 
     // Periodic heap logging is disabled by default because UART logging can
     // create audible ticks during 96 kHz LDAC playback. Enable only for debug.
 #ifndef APP_PERIODIC_HEAP_LOG_MS
 #define APP_PERIODIC_HEAP_LOG_MS 0u
 #endif
 #if APP_PERIODIC_HEAP_LOG_MS > 0
     static TimerHandle_t heapTimer = xTimerCreate("heap_log", pdMS_TO_TICKS(APP_PERIODIC_HEAP_LOG_MS), pdTRUE, nullptr,
         [](TimerHandle_t) { logHeap("periodic"); });
     xTimerStart(heapTimer, 0);
 #endif
 
     ESP_LOGI(TAG, "System ready");
 }
 