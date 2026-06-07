/* Native ESP-IDF A2DP Sink Implementation */

#include "a2dp_sink_native.h"
#include "nvs_flash.h"
#include "nvs.h"

// LDAC vendor codec constants (inlined to avoid internal stack header dependency)
#define A2DP_LDAC_VENDOR_ID           0x0000012D
#define A2DP_LDAC_CODEC_ID            0x00AA
#define A2DP_LDAC_SAMPLING_FREQ_MASK  0x3F
#define A2DP_LDAC_SAMPLING_FREQ_44100 0x20
#define A2DP_LDAC_SAMPLING_FREQ_48000 0x10
#define A2DP_LDAC_SAMPLING_FREQ_88200 0x08
#define A2DP_LDAC_SAMPLING_FREQ_96000   0x04
#define A2DP_LDAC_SAMPLING_FREQ_176400  0x02
#define A2DP_LDAC_SAMPLING_FREQ_192000  0x01
#define A2DP_LDAC_CHANNEL_MODE_MASK   0x07
#define A2DP_LDAC_CHANNEL_MODE_MONO   0x04

// Opus over A2DP vendor codec constants. This stack decodes Opus to 48 kHz
// signed 16-bit PCM, so report 48k/16-bit to the render pipeline.
#define A2DP_OPUS_VENDOR_ID          0x000005F1
#define A2DP_OPUS_CODEC_ID           0x1005

#define BT_AV_TAG    "BT_AV"
#define BT_RC_TG_TAG "BT_RC_TG"
#define BT_RC_CT_TAG "BT_RC_CT"

NativeA2DPSink* NativeA2DPSink::instance = nullptr;

struct bt_app_msg_t {
    uint16_t sig;
    uint16_t event;
    void *param;
    void (*cb)(uint16_t event, void *param);
};

NativeA2DPSink::NativeA2DPSink() { instance = this; _lock_init(&s_volume_lock); }
NativeA2DPSink::~NativeA2DPSink() { if (app_task_handle) { end(true); } }

void NativeA2DPSink::gap_cb_trampoline(esp_bt_gap_cb_event_t e, esp_bt_gap_cb_param_t *p) { if (instance) instance->gap_cb(e,p); }
void NativeA2DPSink::a2d_cb_trampoline(esp_a2d_cb_event_t e, esp_a2d_cb_param_t *p) { if (instance) instance->a2d_cb(e,p); }
void NativeA2DPSink::rc_ct_cb_trampoline(esp_avrc_ct_cb_event_t e, esp_avrc_ct_cb_param_t *p) { if (instance) instance->rc_ct_cb(e,p); }
void NativeA2DPSink::rc_tg_cb_trampoline(esp_avrc_tg_cb_event_t e, esp_avrc_tg_cb_param_t *p) { if (instance) instance->rc_tg_cb(e,p); }
void NativeA2DPSink::data_cb_trampoline(const uint8_t *data, uint32_t len) { if (instance) instance->data_cb(data,len); }
void NativeA2DPSink::app_task_handler_trampoline(void *arg) { if (instance) instance->app_task_handler(arg); }

void NativeA2DPSink::init_bluetooth() {
    // Bluetooth controller and bluedroid are initialized centrally in main.cpp
    // to avoid double-init when both BLE and Classic BT (A2DP) are used.
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_FIXED;
    esp_bt_pin_code_t pin_code; pin_code[0]='1'; pin_code[1]='2'; pin_code[2]='3'; pin_code[3]='4';
    esp_bt_gap_set_pin(pin_type, 4, pin_code);
}

void NativeA2DPSink::start(const char *name) {
    if (is_start_disabled) { ESP_LOGE(TAG, "re-start not supported after end(true)"); return; }
    if (name) bt_name = name;
    init_bluetooth();
    app_task_queue = xQueueCreate(20, sizeof(bt_app_msg_t));
    if (!app_task_queue) { ESP_LOGE(TAG, "queue create failed"); return; }
    if (xTaskCreatePinnedToCore(app_task_handler_trampoline, "btAppTask", 3072, this,
            configMAX_PRIORITIES - 3, &app_task_handle, task_core) != pdPASS) {
        ESP_LOGE(TAG, "task create failed"); vQueueDelete(app_task_queue); app_task_queue=nullptr; return;
    }
    app_work_dispatch([](uint16_t e, void *p){ if(instance) instance->av_hdl_stack_evt(e,p); }, 0, nullptr, 0);
}

void NativeA2DPSink::start(const char *name, bool auto_reconnect) {
    set_auto_reconnect(auto_reconnect);
    start(name);
}

void NativeA2DPSink::end(bool release_memory) {
    is_autoreconnect_allowed = false;
    if (app_task_handle) { vTaskDelete(app_task_handle); app_task_handle = nullptr; }
    if (app_task_queue) { vQueueDelete(app_task_queue); app_task_queue = nullptr; }
    esp_a2d_sink_deinit();
    esp_avrc_ct_deinit();
    esp_avrc_tg_deinit();
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
    if (release_memory) is_start_disabled = true;
}

bool NativeA2DPSink::app_work_dispatch(void (*p_cback)(uint16_t,void*), uint16_t event, void *p_params, int param_len) {
    if (!app_task_queue) return false;
    bt_app_msg_t msg = {}; msg.sig = APP_SIG_WORK_DISPATCH; msg.event = event; msg.cb = p_cback;
    if (param_len > 0) { msg.param = malloc(param_len); if (!msg.param) return false; memcpy(msg.param, p_params, param_len); }
    if (xQueueSend(app_task_queue, &msg, portMAX_DELAY) != pdPASS) { free(msg.param); return false; }
    return true;
}

void NativeA2DPSink::app_task_handler(void *arg) {
    bt_app_msg_t msg;
    while (true) {
        if (xQueueReceive(app_task_queue, &msg, portMAX_DELAY) == pdPASS) {
            if (msg.sig == APP_SIG_WORK_DISPATCH && msg.cb) { msg.cb(msg.event, msg.param); free(msg.param); }
        }
    }
}

void NativeA2DPSink::av_hdl_stack_evt(uint16_t event, void *p_param) {
    ESP_LOGI(TAG, "av_hdl_stack_evt: starting A2DP sink init");
    esp_err_t name_err = esp_bt_dev_set_device_name(bt_name.c_str()); ESP_LOGI(TAG, "set_device_name: %s", esp_err_to_name(name_err));
    esp_bt_gap_register_callback(gap_cb_trampoline);
    esp_avrc_ct_register_callback(rc_ct_cb_trampoline);
    esp_avrc_ct_init();
    esp_avrc_tg_register_callback(rc_tg_cb_trampoline);
    esp_avrc_tg_init();
    esp_avrc_rn_evt_cap_mask_t evt_set = {}; esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &evt_set, ESP_AVRC_RN_VOLUME_CHANGE);
    esp_avrc_tg_set_rn_evt_cap(&evt_set);
    esp_a2d_register_callback(a2d_cb_trampoline);
    esp_a2d_sink_init();
    esp_a2d_sink_register_data_callback(data_cb_trampoline);
    esp_err_t scan_err = esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE); ESP_LOGI(TAG, "set_scan_mode: %s", esp_err_to_name(scan_err));
    if (scan_err != ESP_OK) { ESP_LOGE(TAG, "set_scan_mode failed: %s", esp_err_to_name(scan_err)); }
}

void NativeA2DPSink::gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    uint8_t *bda = nullptr;
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) ESP_LOGI(BT_AV_TAG, "auth success: %s", param->auth_cmpl.device_name);
        else ESP_LOGE(BT_AV_TAG, "auth failed, status:%d", param->auth_cmpl.stat);
        break;
    case ESP_BT_GAP_CFM_REQ_EVT:
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_MODE_CHG_EVT:
        ESP_LOGI(BT_AV_TAG, "mode_chg: %d", param->mode_chg.mode);
        break;
    default: break;
    }
}

void NativeA2DPSink::a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param) {
    app_work_dispatch([](uint16_t e, void *p){ if(instance) instance->av_hdl_a2d_evt(e,p); }, event, param, sizeof(esp_a2d_cb_param_t));
}

void NativeA2DPSink::av_hdl_a2d_evt(uint16_t event, void *p_param) {
    esp_a2d_cb_param_t *a2d = (esp_a2d_cb_param_t*)p_param;
    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT: {
        uint8_t *bda = a2d->conn_stat.remote_bda;
        connection_state = a2d->conn_stat.state;
        ESP_LOGI(BT_AV_TAG, "conn state %d [%02x:%02x:%02x:%02x:%02x:%02x]",
            connection_state, bda[0],bda[1],bda[2],bda[3],bda[4],bda[5]);
        if (connection_state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
            esp_err_t scan_err = esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE); ESP_LOGI(TAG, "set_scan_mode: %s", esp_err_to_name(scan_err));
    if (scan_err != ESP_OK) { ESP_LOGE(TAG, "set_scan_mode failed: %s", esp_err_to_name(scan_err)); }
            memcpy(peer_bd_addr, bda, ESP_BD_ADDR_LEN);
        } else if (connection_state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
            memcpy(peer_bd_addr, bda, ESP_BD_ADDR_LEN);
            set_last_connection(peer_bd_addr);
        }
        if (connection_state_cb) connection_state_cb(connection_state, this);
        break;
    }
    case ESP_A2D_AUDIO_STATE_EVT:
        ESP_LOGI(BT_AV_TAG, "audio state %d", a2d->audio_stat.state);
        if (audio_state_cb) audio_state_cb(a2d->audio_stat.state, this);
        break;
    case ESP_A2D_AUDIO_CFG_EVT: {
        esp_a2d_mcc_t *p_mcc = &a2d->audio_cfg.mcc;
        audio_type = p_mcc->type;
        uint32_t sr = 44100; uint8_t bits = 16; uint8_t ch = 2;
        if (audio_type == ESP_A2D_MCT_SBC) {
            if (p_mcc->cie.sbc_info.samp_freq & ESP_A2D_SBC_CIE_SF_32K) sr = 32000;
            else if (p_mcc->cie.sbc_info.samp_freq & ESP_A2D_SBC_CIE_SF_44K) sr = 44100;
            else if (p_mcc->cie.sbc_info.samp_freq & ESP_A2D_SBC_CIE_SF_48K) sr = 48000;
            if (p_mcc->cie.sbc_info.ch_mode & ESP_A2D_SBC_CIE_CH_MODE_MONO) ch = 1;
        } else if (audio_type == ESP_A2D_MCT_M24) {
            const esp_a2d_cie_m24_t &aac = p_mcc->cie.m24_info;
            const uint8_t *raw = reinterpret_cast<const uint8_t*>(&aac);
            uint16_t sampleBits = ((uint16_t)aac.samp_freq1 << 4) | (aac.samp_freq2 & 0x0F);
            if (sampleBits & 0x800) sr = 8000;
            else if (sampleBits & 0x400) sr = 11025;
            else if (sampleBits & 0x200) sr = 12000;
            else if (sampleBits & 0x100) sr = 16000;
            else if (sampleBits & 0x080) sr = 22050;
            else if (sampleBits & 0x040) sr = 24000;
            else if (sampleBits & 0x020) sr = 32000;
            else if (sampleBits & 0x010) sr = 44100;
            else if (sampleBits & 0x008) sr = 48000;
            else if (sampleBits & 0x004) sr = 64000;
            else if (sampleBits & 0x002) sr = 88200;
            else if (sampleBits & 0x001) sr = 96000;
            else sr = 44100;
            ch = (aac.ch == 1) ? 1 : 2;
            bits = 16;
            ESP_LOGI(BT_AV_TAG, "Detected AAC codec sr=%u bits=0x%03X ch_field=0x%X raw=%02X %02X %02X %02X %02X %02X",
                     sr, sampleBits, aac.ch,
                     raw[0], raw[1], raw[2], raw[3], raw[4], raw[5]);
        } else if (audio_type == ESP_A2D_MCT_NON_A2DP) {
            // Vendor codec (aptX, LDAC, etc.): parse raw CIE bytes
            // LDAC layout in 8-byte cie.ldac: vendorId[0-3], codecId[4-5], sampleRate[6], channelMode[7]
            uint8_t *raw = p_mcc->cie.ldac;
            uint32_t vendorId = raw[0] | (raw[1] << 8) | (raw[2] << 16) | (raw[3] << 24);
            uint16_t codecId  = raw[4] | (raw[5] << 8);
            if (vendorId == A2DP_LDAC_VENDOR_ID && codecId == A2DP_LDAC_CODEC_ID) {
                uint8_t sr_bits = raw[6] & A2DP_LDAC_SAMPLING_FREQ_MASK;
                if (sr_bits & A2DP_LDAC_SAMPLING_FREQ_44100) sr = 44100;
                else if (sr_bits & A2DP_LDAC_SAMPLING_FREQ_48000) sr = 48000;
                else if (sr_bits & A2DP_LDAC_SAMPLING_FREQ_88200) sr = 88200;
                else if (sr_bits & A2DP_LDAC_SAMPLING_FREQ_96000) sr = 96000;
                else if (sr_bits & A2DP_LDAC_SAMPLING_FREQ_176400) sr = 176400;
                else if (sr_bits & A2DP_LDAC_SAMPLING_FREQ_192000) sr = 192000;
                uint8_t ch_bits = raw[7] & A2DP_LDAC_CHANNEL_MODE_MASK;
                if (ch_bits == A2DP_LDAC_CHANNEL_MODE_MONO) ch = 1;
                // LDACBT decoder outputs LDACBT_SMPL_FMT_S24 (24-bit)
                bits = 24;
                ESP_LOGI(BT_AV_TAG, "Detected LDAC codec sr=%u ch=%u", sr, ch);
            } else if (vendorId == 0x0000004F && codecId == 0x0001) {
                // aptX Classic: aptx_decode32 outputs 24-bit in 32-bit LE containers
                uint8_t sr_ch = raw[6];
                if (sr_ch & 0x20) sr = 44100;
                else if (sr_ch & 0x10) sr = 48000;
                if ((sr_ch & 0x0F) == 0x01) ch = 1;
                bits = 32;
                ESP_LOGI(BT_AV_TAG, "Detected aptX codec sr=%u ch=%u", sr, ch);
            } else if (vendorId == 0x000000D7 && codecId == 0x0024) {
                // aptX-HD: aptx_decode32 outputs 24-bit in 32-bit LE containers
                uint8_t sr_ch = raw[6];
                if (sr_ch & 0x20) sr = 44100;
                else if (sr_ch & 0x10) sr = 48000;
                if ((sr_ch & 0x0F) == 0x01) ch = 1;
                bits = 32;
                ESP_LOGI(BT_AV_TAG, "Detected aptX-HD codec sr=%u ch=%u", sr, ch);
            } else if (vendorId == 0x0000000A && codecId == 0x0002) {
                // aptX-LL
                bits = 32;
                sr = 48000;
                ESP_LOGI(BT_AV_TAG, "Detected aptX-LL codec sr=%u ch=%u", sr, ch);
            } else if (vendorId == A2DP_OPUS_VENDOR_ID && codecId == A2DP_OPUS_CODEC_ID) {
                // A2DP Opus vendor codec. The ESP-IDF Opus decoder path outputs
                // interleaved opus_int16 PCM at 48 kHz. In the CIE, raw[6] is
                // channel count and raw[7] is coupled stream count.
                sr = 48000;
                bits = 16;
                ch = raw[6];
                if (ch == 0 || ch > 2) ch = 2;
                ESP_LOGI(BT_AV_TAG, "Detected Opus codec sr=%u bits=%u ch=%u coupled=%u",
                         sr, bits, ch, raw[7]);
            } else {
                sr = 48000; bits = 32;
                ESP_LOGI(BT_AV_TAG, "Unknown vendor codec vendorId=0x%08X codecId=0x%04X", vendorId, codecId);
            }
        }
        m_sample_rate = sr;
        ESP_LOGI(BT_AV_TAG, "codec cfg sr=%u bits=%u ch=%u type=%d", sr, bits, ch, audio_type);
        if (codec_config_cb) codec_config_cb(sr, bits, ch);
        break;
    }
    case ESP_A2D_SNK_GET_DELAY_VALUE_EVT:
        // Delay value is in 1/10 ms. A deeper advertised sink delay gives
        // high-bitrate codecs, especially LDAC 96 kHz, enough jitter margin
        // while BLE/GATT and UI tasks are active.
        esp_a2d_sink_set_delay_value(a2d->a2d_get_delay_value_stat.delay_value + 1500);
        break;
    default: break;
    }
}

void NativeA2DPSink::data_cb(const uint8_t *data, uint32_t len) {
    if (raw_stream_reader) raw_stream_reader(data, len);
    if (stream_reader) stream_reader(data, len);
    if (data_received) data_received();
}

void NativeA2DPSink::rc_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param) {
    app_work_dispatch([](uint16_t e, void *p){ if(instance) instance->av_hdl_avrc_ct_evt(e,p); }, event, param, sizeof(esp_avrc_ct_cb_param_t));
}

void NativeA2DPSink::av_hdl_avrc_ct_evt(uint16_t event, void *p_param) {
    esp_avrc_ct_cb_param_t *rc = (esp_avrc_ct_cb_param_t*)p_param;
    switch (event) {
    case ESP_AVRC_CT_CONNECTION_STATE_EVT:
        if (rc->conn_stat.connected) esp_avrc_ct_send_get_rn_capabilities_cmd(0);
        else s_avrc_peer_rn_cap.bits = 0;
        avrc_connection_state = rc->conn_stat.connected;
        break;
    case ESP_AVRC_CT_CHANGE_NOTIFY_EVT:
        av_notify_evt_handler(rc->change_ntf.event_id, &rc->change_ntf.event_parameter);
        break;
    case ESP_AVRC_CT_GET_RN_CAPABILITIES_RSP_EVT:
        s_avrc_peer_rn_cap.bits = rc->get_rn_caps_rsp.evt_set.bits;
        av_new_track(); av_playback_changed(); av_play_pos_changed();
        break;
    default: break;
    }
}

void NativeA2DPSink::rc_tg_cb(esp_avrc_tg_cb_event_t event, esp_avrc_tg_cb_param_t *param) {
    app_work_dispatch([](uint16_t e, void *p){ if(instance) instance->av_hdl_avrc_tg_evt(e,p); }, event, param, sizeof(esp_avrc_tg_cb_param_t));
}

void NativeA2DPSink::av_hdl_avrc_tg_evt(uint16_t event, void *p_param) {
    esp_avrc_tg_cb_param_t *rc = (esp_avrc_tg_cb_param_t*)p_param;
    switch (event) {
    case ESP_AVRC_TG_SET_ABSOLUTE_VOLUME_CMD_EVT:
        volume_set_by_controller(rc->set_abs_vol.volume);
        break;
    case ESP_AVRC_TG_REGISTER_NOTIFICATION_EVT:
        if (rc->reg_ntf.event_id == ESP_AVRC_RN_VOLUME_CHANGE) {
            s_volume_notify = true;
            esp_avrc_rn_param_t rn_param; rn_param.volume = s_volume;
            esp_avrc_tg_send_rn_rsp(ESP_AVRC_RN_VOLUME_CHANGE, ESP_AVRC_RN_RSP_INTERIM, &rn_param);
        }
        break;
    default: break;
    }
}

void NativeA2DPSink::execute_avrc_command(int cmd) {
    esp_avrc_ct_send_passthrough_cmd(0, (uint8_t)cmd, ESP_AVRC_PT_CMD_STATE_PRESSED);
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_avrc_ct_send_passthrough_cmd(0, (uint8_t)cmd, ESP_AVRC_PT_CMD_STATE_RELEASED);
}

void NativeA2DPSink::play()        { execute_avrc_command(ESP_AVRC_PT_CMD_PLAY); }
void NativeA2DPSink::pause()       { execute_avrc_command(ESP_AVRC_PT_CMD_PAUSE); }
void NativeA2DPSink::stop()        { execute_avrc_command(ESP_AVRC_PT_CMD_STOP); }
void NativeA2DPSink::next()        { execute_avrc_command(ESP_AVRC_PT_CMD_FORWARD); }
void NativeA2DPSink::previous()    { execute_avrc_command(ESP_AVRC_PT_CMD_BACKWARD); }
void NativeA2DPSink::fast_forward(){ execute_avrc_command(ESP_AVRC_PT_CMD_FAST_FORWARD); }
void NativeA2DPSink::rewind()      { execute_avrc_command(ESP_AVRC_PT_CMD_REWIND); }

void NativeA2DPSink::volume_up()   { volume_set_by_local_host((s_volume + 5) & 0x7f); }
void NativeA2DPSink::volume_down() { volume_set_by_local_host(s_volume > 5 ? s_volume - 5 : 0); }
void NativeA2DPSink::set_volume(uint8_t v) { volume_set_by_local_host(v & 0x7f); }
int  NativeA2DPSink::get_volume()  { _lock_acquire(&s_volume_lock); int v=s_volume; _lock_release(&s_volume_lock); return v; }

void NativeA2DPSink::volume_set_by_controller(uint8_t volume) {
    _lock_acquire(&s_volume_lock); s_volume = volume; _lock_release(&s_volume_lock);
    if (volumechange_cb) volumechange_cb(volume);
    if (s_volume_notify) {
        esp_avrc_rn_param_t rn_param; rn_param.volume = s_volume;
        esp_avrc_tg_send_rn_rsp(ESP_AVRC_RN_VOLUME_CHANGE, ESP_AVRC_RN_RSP_CHANGED, &rn_param);
        s_volume_notify = false;
    }
    if (volumechange_completed_cb) volumechange_completed_cb(volume);
}

void NativeA2DPSink::volume_set_by_local_host(uint8_t volume) {
    _lock_acquire(&s_volume_lock); s_volume = volume; _lock_release(&s_volume_lock);
    if (volumechange_cb) volumechange_cb(volume);
    if (s_volume_notify) {
        esp_avrc_rn_param_t rn_param; rn_param.volume = s_volume;
        esp_avrc_tg_send_rn_rsp(ESP_AVRC_RN_VOLUME_CHANGE, ESP_AVRC_RN_RSP_CHANGED, &rn_param);
        s_volume_notify = false;
    }
}

void NativeA2DPSink::av_notify_evt_handler(uint8_t event_id, esp_avrc_rn_param_t *event_parameter) {
    switch (event_id) {
    case ESP_AVRC_RN_TRACK_CHANGE: av_new_track(); break;
    case ESP_AVRC_RN_PLAY_STATUS_CHANGE: av_playback_changed(); break;
    case ESP_AVRC_RN_PLAY_POS_CHANGED: av_play_pos_changed(); break;
    case ESP_AVRC_RN_VOLUME_CHANGE:
        volume_set_by_controller(event_parameter->volume);
        break;
    default: break;
    }
}

void NativeA2DPSink::av_new_track() {
    if (esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_TEST, &s_avrc_peer_rn_cap, ESP_AVRC_RN_TRACK_CHANGE))
        esp_avrc_ct_send_register_notification_cmd(2, ESP_AVRC_RN_TRACK_CHANGE, 0);
}
void NativeA2DPSink::av_playback_changed() {
    if (esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_TEST, &s_avrc_peer_rn_cap, ESP_AVRC_RN_PLAY_STATUS_CHANGE))
        esp_avrc_ct_send_register_notification_cmd(3, ESP_AVRC_RN_PLAY_STATUS_CHANGE, 0);
}
void NativeA2DPSink::av_play_pos_changed() {
    if (esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_TEST, &s_avrc_peer_rn_cap, ESP_AVRC_RN_PLAY_POS_CHANGED))
        esp_avrc_ct_send_register_notification_cmd(4, ESP_AVRC_RN_PLAY_POS_CHANGED, 10);
}

void NativeA2DPSink::set_discoverability(esp_bt_discovery_mode_t mode) {
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, (mode==ESP_BT_NON_DISCOVERABLE)?ESP_BT_NON_DISCOVERABLE:mode);
}

void NativeA2DPSink::set_scan_mode_connectable(bool connectable) {
    esp_bt_gap_set_scan_mode(connectable?ESP_BT_CONNECTABLE:ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
}

esp_a2d_connection_state_t NativeA2DPSink::get_connection_state() { return connection_state; }
void NativeA2DPSink::disconnect() { esp_a2d_sink_disconnect(peer_bd_addr); }
void NativeA2DPSink::set_auto_reconnect(bool reconnect, int count) {
    reconnect_status = reconnect?AutoReconnect:NoReconnect;
    try_reconnect_max_count = count;
    is_autoreconnect_allowed = reconnect;
}
void NativeA2DPSink::set_task_core(int core) { task_core = core; }
void NativeA2DPSink::set_output_active(bool active) { is_i2s_active = active; }
esp_bd_addr_t* NativeA2DPSink::get_current_peer_address() { return &peer_bd_addr; }

bool NativeA2DPSink::has_last_connection() {
    nvs_handle_t handle;
    if (nvs_open("a2dp", NVS_READONLY, &handle) != ESP_OK) return false;
    uint8_t bda[6]; size_t len=6;
    esp_err_t err = nvs_get_blob(handle, "last_bda", bda, &len);
    nvs_close(handle);
    return (err==ESP_OK && len==6);
}
void NativeA2DPSink::get_last_connection() {
    nvs_handle_t handle;
    if (nvs_open("a2dp", NVS_READONLY, &handle)==ESP_OK) { size_t len=6; nvs_get_blob(handle, "last_bda", last_connection, &len); nvs_close(handle); }
}
void NativeA2DPSink::set_last_connection(esp_bd_addr_t bda) {
    nvs_handle_t handle;
    if (nvs_open("a2dp", NVS_READWRITE, &handle)==ESP_OK) { nvs_set_blob(handle, "last_bda", bda, 6); nvs_commit(handle); nvs_close(handle); }
}

// Callback setters
void NativeA2DPSink::set_stream_reader(a2dp_stream_reader_cb cb, bool is_i2s) { stream_reader = cb; is_i2s_active = is_i2s; }
void NativeA2DPSink::set_raw_stream_reader(a2dp_stream_reader_cb cb) { raw_stream_reader = cb; }
void NativeA2DPSink::set_on_data_received(a2dp_data_received_cb cb) { data_received = cb; }
void NativeA2DPSink::set_codec_config_callback(a2dp_codec_config_cb cb) { codec_config_cb = cb; }
void NativeA2DPSink::set_on_connection_state_changed(a2dp_connection_state_cb cb) { connection_state_cb = cb; }
void NativeA2DPSink::set_on_audio_state_changed(a2dp_audio_state_cb cb) { audio_state_cb = cb; }
void NativeA2DPSink::set_avrc_rn_volumechange(a2dp_volumechange_cb cb) { volumechange_cb = cb; }
void NativeA2DPSink::set_avrc_rn_volumechange_completed(a2dp_volumechange_cb cb) { volumechange_completed_cb = cb; }
esp_a2d_mct_t NativeA2DPSink::get_audio_type() { return audio_type; }
