#pragma once

/*
 * Native ESP-IDF A2DP Sink Wrapper
 * Replaces ESP32-A2DP external library with direct ESP-IDF API usage.
 * Provides the same C++ interface as BluetoothA2DPSink for minimal main.cpp changes.
 */

#include <string.h>
#include <stdint.h>
#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#include "sys/lock.h"

// Same callback signatures as BluetoothA2DPSink
using a2dp_stream_reader_cb = void (*)(const uint8_t *data, uint32_t len);
using a2dp_connection_state_cb = void (*)(esp_a2d_connection_state_t state, void *user);
using a2dp_audio_state_cb = void (*)(esp_a2d_audio_state_t state, void *user);
using a2dp_codec_config_cb = void (*)(uint32_t rate, uint8_t bps, uint8_t channels);
using a2dp_volumechange_cb = void (*)(int volume);
using a2dp_data_received_cb = void (*)();

enum ReconnectStatus { NoReconnect, AutoReconnect, IsReconnecting };

class NativeA2DPSink {
public:
    NativeA2DPSink();
    ~NativeA2DPSink();

    // Start/stop
    void start(const char *name);
    void start(const char *name, bool auto_reconnect);
    void end(bool release_memory = false);

    // Audio output callback (same as set_stream_reader)
    void set_stream_reader(a2dp_stream_reader_cb callback, bool is_i2s_output = true);
    void set_raw_stream_reader(a2dp_stream_reader_cb callback);
    void set_on_data_received(a2dp_data_received_cb callback);

    // Codec config
    void set_codec_config_callback(a2dp_codec_config_cb callback);
    esp_a2d_mct_t get_audio_type();

    // Connection callbacks
    void set_on_connection_state_changed(a2dp_connection_state_cb callback);
    void set_on_audio_state_changed(a2dp_audio_state_cb callback);

    // AVRCP
    void set_avrc_rn_volumechange(a2dp_volumechange_cb callback);
    void set_avrc_rn_volumechange_completed(a2dp_volumechange_cb callback);

    // Controls
    void play();
    void pause();
    void stop();
    void next();
    void previous();
    void fast_forward();
    void rewind();
    void volume_up();
    void volume_down();
    void set_volume(uint8_t volume);
    int get_volume();

    // Discoverability / scan mode
    void set_discoverability(esp_bt_discovery_mode_t mode);

    // Connection management
    esp_a2d_connection_state_t get_connection_state();
    void disconnect();
    void set_auto_reconnect(bool reconnect, int count = 3);
    void set_task_core(int core);

    // Output active
    void set_output_active(bool active);

    esp_bd_addr_t* get_current_peer_address();

private:
    static constexpr const char* TAG = "NativeA2DP";
    static constexpr int AUTOCONNECT_TRY_NUM = 3;

    // Internal state
    std::string bt_name;
    bool is_i2s_active = false;
    bool is_start_disabled = false;
    volatile bool is_autoreconnect_allowed = false;
    ReconnectStatus reconnect_status = NoReconnect;
    int try_reconnect_max_count = AUTOCONNECT_TRY_NUM;
    int connection_retry_count = 0;
    int reconnect_delay = 1000;
    int task_core = 0;

    // Volume
    _lock_t s_volume_lock;
    uint8_t s_volume = 0;
    bool s_volume_notify = false;

    // Connection
    esp_a2d_connection_state_t connection_state = ESP_A2D_CONNECTION_STATE_DISCONNECTED;
    esp_bd_addr_t peer_bd_addr = {0};
    esp_bd_addr_t last_connection = {0};
    uint16_t m_sample_rate = 44100;
    esp_a2d_mct_t audio_type = ESP_A2D_MCT_SBC;
    bool avrc_connection_state = false;

    // AVRCP peer caps
    esp_avrc_rn_evt_cap_mask_t s_avrc_peer_rn_cap = {0};

    // Callbacks
    a2dp_stream_reader_cb stream_reader = nullptr;
    a2dp_stream_reader_cb raw_stream_reader = nullptr;
    a2dp_data_received_cb data_received = nullptr;
    a2dp_connection_state_cb connection_state_cb = nullptr;
    a2dp_audio_state_cb audio_state_cb = nullptr;
    a2dp_codec_config_cb codec_config_cb = nullptr;
    a2dp_volumechange_cb volumechange_cb = nullptr;
    a2dp_volumechange_cb volumechange_completed_cb = nullptr;

    // App task dispatch (work queue)
    QueueHandle_t app_task_queue = nullptr;
    TaskHandle_t app_task_handle = nullptr;
    static constexpr uint16_t APP_SIG_WORK_DISPATCH = 0x01;
    enum { BT_APP_EVT_STACK_UP = 0 };

    // C trampoline callbacks (needed for ESP-IDF C API)
    static NativeA2DPSink* instance;
    static void gap_cb_trampoline(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
    static void a2d_cb_trampoline(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);
    static void rc_ct_cb_trampoline(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param);
    static void rc_tg_cb_trampoline(esp_avrc_tg_cb_event_t event, esp_avrc_tg_cb_param_t *param);
    static void data_cb_trampoline(const uint8_t *data, uint32_t len);
    static void app_task_handler_trampoline(void *arg);

    // Actual handlers
    void gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
    void a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);
    void rc_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param);
    void rc_tg_cb(esp_avrc_tg_cb_event_t event, esp_avrc_tg_cb_param_t *param);
    void data_cb(const uint8_t *data, uint32_t len);
    void app_task_handler(void *arg);

    // Event handlers
    void av_hdl_stack_evt(uint16_t event, void *p_param);
    void av_hdl_a2d_evt(uint16_t event, void *p_param);
    void av_hdl_avrc_ct_evt(uint16_t event, void *p_param);
    void av_hdl_avrc_tg_evt(uint16_t event, void *p_param);

    // Helpers
    void init_bluetooth();
    bool app_work_dispatch(void (*p_cback)(uint16_t, void*), uint16_t event, void *p_params, int param_len);
    void execute_avrc_command(int cmd);
    void av_notify_evt_handler(uint8_t event_id, esp_avrc_rn_param_t *event_parameter);
    void volume_set_by_controller(uint8_t volume);
    void volume_set_by_local_host(uint8_t volume);
    void av_new_track();
    void av_playback_changed();
    void av_play_pos_changed();
    void set_scan_mode_connectable(bool connectable);

    bool has_last_connection();
    void get_last_connection();
    void set_last_connection(esp_bd_addr_t bda);
};
