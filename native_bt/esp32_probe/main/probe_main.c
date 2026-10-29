#include <inttypes.h>
#include <stdio.h>
#include "dbt_gap.h"
#ifdef DBT_PROBE_AUDIO_TEST
#include "probe_audio.h"
#endif
#ifdef DBT_PROBE_GATT_TEST
#include "probe_gatt.h"
#endif
#ifdef DBT_PROBE_CLASSIC_SECURITY_TEST
#include "probe_security.h"
#include "probe_echo.h"
#include "probe_sdp.h"
#endif
#ifdef DBT_PROBE_INGRESS_TRACE
#include "probe_ingress.h"
#endif
#include "esp_bt.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#if !CONFIG_IDF_TARGET_ESP32 || !CONFIG_BT_CONTROLLER_ONLY || !CONFIG_BTDM_CTRL_MODE_BTDM
#error "Probe requires original ESP32, controller-only host selection and dual mode"
#endif

static dbt_runtime_t runtime;
static dbt_gap_t gap;
static TaskHandle_t owner;
static const char *tag = "native_bt_probe";

static uint32_t now_ms(void) {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static void wake_owner(void) {
    if (xPortInIsrContext()) {
        BaseType_t higher_priority = pdFALSE;
        vTaskNotifyGiveFromISR(owner, &higher_priority);
        if (higher_priority) portYIELD_FROM_ISR();
    } else {
        xTaskNotifyGive(owner);
    }
}

static void send_available(void) { wake_owner(); }

static int receive_packet(uint8_t *data, uint16_t size) {
    /* No allocation, logging, decoding, profile state mutation or waiting here. */
#ifdef DBT_PROBE_INGRESS_TRACE
    uint32_t token=probe_ingress_begin(data,size);
#endif
    (void)dbt_runtime_receive(&runtime, data, size);
#ifdef DBT_PROBE_INGRESS_TRACE
    probe_ingress_end(token,data,size);
#endif
    wake_owner();
    return 0; /* VHCI has no supported retry contract; core latches any loss. */
}

static bool transport_ready(void *context) {
    (void)context;
    return esp_vhci_host_check_send_available();
}

static bool transport_send(void *context, uint8_t *data, uint16_t size) {
    (void)context;
    if (!esp_vhci_host_check_send_available()) return false;
#ifdef DBT_PROBE_CLASSIC_SECURITY_TEST
    probe_echo_transmit(data,size);
#endif
    /* Host task only, outside critical sections and scheduler suspension. */
    esp_vhci_host_send_packet(data, size);
    return true;
}

static uint8_t *allocate_rx(void *context, uint16_t size) {
    (void)context;
    return heap_caps_malloc(size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
}
static void release_rx(void *context, uint8_t *data) {
    (void)context;
    heap_caps_free(data);
}
#if defined(DBT_PROBE_GATT_TEST) || defined(DBT_PROBE_CLASSIC_SECURITY_TEST)
static void profile_link(void *context,const dbt_link_change_t *change) {
    (void)context;
#ifdef DBT_PROBE_CLASSIC_SECURITY_TEST
    probe_security_link(change);
#endif
#ifdef DBT_PROBE_GATT_TEST
    probe_gatt_link(context,change);
#endif
}
#endif
static void observe(void *context, const dbt_hci_packet_t *packet) {
    (void)context;
#ifdef DBT_PROBE_INGRESS_TRACE
    if(packet->type==DBT_H4_ACL) {
        probe_ingress_sample_t sample;
        bool valid=probe_ingress_observe(packet,&sample);
        static bool trace_failure_reported;
        if(!valid && !trace_failure_reported) {
            trace_failure_reported=true;
            probe_ingress_fault_t failure;
            probe_ingress_fault(&failure);
            probe_security_trace(8,failure.snapshot_size,failure.packet_size,0,failure.reason);
            probe_security_trace(9,failure.snapshot_flags&0xfffu,failure.packet_flags&0xfffu,
                                  failure.snapshot_flags>>12,failure.packet_flags>>12);
            probe_security_trace(10,(uint16_t)failure.published,(uint16_t)failure.consumed,0,(uint8_t)failure.lost);
        }
        probe_echo_receive_packet(packet,valid?&sample:0);
        return;
    }
#endif
    if (packet->type != DBT_H4_EVENT || gap.state == DBT_GAP_IDLE) return;
#ifdef DBT_PROBE_CLASSIC_SECURITY_TEST
    dbt_result_t owned=probe_security_receive(packet,now_ms());
    if(owned!=DBT_UNSUPPORTED) {
        if(owned!=DBT_OK)dbt_host_abort(&runtime.host,owned==DBT_NO_SPACE?DBT_FAULT_RX_OVERRUN:DBT_FAULT_BAD_PACKET);
        return;
    }
#endif
    dbt_result_t result = dbt_gap_event(&gap, packet);
    if (result == DBT_INVALID || result == DBT_NO_SPACE)
        dbt_host_abort(&runtime.host, result == DBT_NO_SPACE ? DBT_FAULT_RX_OVERRUN : DBT_FAULT_BAD_PACKET);
}

#ifdef DBT_PROBE_CLASSIC_SECURITY_TEST
static void profile_channel(void *context, dbt_channel_event_t event, const dbt_channel_t *channel,
                            dbt_channel_reason_t reason, uint16_t result) {
    probe_echo_event(context, event, channel, reason, result);
    probe_sdp_event(event, channel);
#ifdef DBT_PROBE_AUDIO_TEST
    probe_audio_event(event,channel);
#endif
}
static void profile_data(void *context, const dbt_channel_t *channel, const uint8_t *data, uint16_t size) {
#ifdef DBT_PROBE_AUDIO_TEST
    if(channel->psm==0x19){probe_audio_receive(channel,data,size);return;}
#endif
    if (channel->psm != 1) { probe_echo_data(context, channel, data, size); return; }
    if (probe_sdp_receive(channel, data, size) != DBT_OK) {
        /* Invalid/pipelined SDP is terminated explicitly, never silently lost. */
        if (dbt_channels_close(&runtime.channels, channel->handle, channel->local_cid, runtime.now_ms) != DBT_OK) {
            const dbt_link_t *link = dbt_acl_find(&runtime.acl, channel->handle);
            if (link) (void)dbt_gap_disconnect(&gap, link, 0x13);
        }
    }
}
#endif

void app_main(void) {
    owner = xTaskGetCurrentTaskHandle();
    vTaskPrioritySet(owner, 6);
    dbt_transport_t transport = { 0, transport_ready, transport_send };
    dbt_runtime_hooks_t hooks = {0};
    hooks.allocate = allocate_rx; hooks.release = release_rx; hooks.packet = observe;
#ifdef DBT_PROBE_GATT_TEST
    hooks.channels.fixed_data = probe_gatt_receive;
#endif
#if defined(DBT_PROBE_GATT_TEST) || defined(DBT_PROBE_CLASSIC_SECURITY_TEST)
    hooks.link = profile_link;
#endif
#ifdef DBT_PROBE_CLASSIC_SECURITY_TEST
    hooks.channels.authorize=probe_security_authorize;
    hooks.channels.security_request=probe_security_request;
    hooks.channels.event=profile_channel;
    hooks.channels.data=profile_data;
#ifdef DBT_PROBE_AUDIO_TEST
    hooks.channels.accept=probe_audio_accept;
#endif
#endif
    if (dbt_runtime_init(&runtime, transport, hooks, 4096, 2000) != DBT_OK) {
        ESP_LOGE(tag, "core initialization failed");
        return;
    }
    dbt_gap_config_t gap_config = { .name = "NATIVE_BT_PROBE",
        .advertising = {2, 1, 2}, .advertising_size = 3,
        .advertising_minimum = 0x80, .advertising_maximum = 0x100 };
    dbt_gap_hooks_t gap_hooks = {0};
#ifdef DBT_PROBE_CLASSIC_SECURITY_TEST
    gap_hooks.accept_classic=probe_security_accept;
#endif
#ifdef DBT_PROBE_GATT_TEST
    probe_gatt_advertisement(&gap_config);
#endif
    if (dbt_gap_init(&gap, &runtime.host, gap_config, gap_hooks) != DBT_OK) {
        ESP_LOGE(tag, "GAP initialization failed");
        return;
    }
    /* Existing NVS is preserved. Initialization failure does not erase bonds/settings. */
    ESP_ERROR_CHECK(nvs_flash_init());
#ifdef DBT_PROBE_CLASSIC_SECURITY_TEST
    if(probe_security_init(&runtime,&gap,owner)!=DBT_OK) {
        ESP_LOGE(tag,"Classic security/storage initialization failed without erasing NVS");return;
    }
#endif
    esp_bt_controller_config_t configuration = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&configuration));
    static const esp_vhci_host_callback_t callbacks = { send_available, receive_packet };
    ESP_ERROR_CHECK(esp_vhci_host_register_callback(&callbacks));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BTDM));
    if (dbt_runtime_start(&runtime, now_ms()) != DBT_OK) {
        ESP_LOGE(tag, "core start failed");
        return;
    }
    dbt_host_t *host = &runtime.host;
    dbt_host_state_t previous = DBT_HOST_IDLE;
    dbt_gap_state_t previous_gap = DBT_GAP_IDLE;
    bool receive_flow_started=false,receive_flow_reported=false;
#ifdef DBT_PROBE_CLASSIC_SECURITY_TEST
    bool security_reported=false,echo_initialized=false;
#endif
#ifdef DBT_PROBE_GATT_TEST
    bool gatt_initialized = false, gatt_reported = false;
#endif
    for (;;) {
        uint32_t now = now_ms();
        dbt_runtime_poll(&runtime, now, 8, 8);
        if(runtime.ready && !receive_flow_started) {
            receive_flow_started=true;
#ifndef DBT_PROBE_DISABLE_RX_FLOW
            uint16_t bytes=DBT_MAX_H4_BYTES-5u;
            uint16_t packets=(DBT_QUEUE_BYTES-DBT_CONTROL_RESERVE_BYTES)/(DBT_MAX_H4_BYTES+2u);
            dbt_result_t result=dbt_runtime_enable_rx_flow(&runtime,bytes,packets);
            if(result!=DBT_OK) {
                printf("{\"probe\":\"rx_flow_failed\",\"result\":%u,\"supported_octet10\":%u}\n",
                       result,runtime.host.controller.supported_commands[10]);
                dbt_runtime_abort(&runtime,DBT_FAULT_COMMAND_REJECTED);
            }
#endif
        }
        bool receive_flow_ready=runtime.rx_flow.state==DBT_RX_FLOW_ACTIVE;
#ifdef DBT_PROBE_DISABLE_RX_FLOW
        receive_flow_ready=true;
#endif
        if(runtime.ready && receive_flow_ready && !receive_flow_reported) {
            receive_flow_reported=true;
            printf("{\"probe\":\"rx_flow_ready\",\"enabled\":%s,\"packet_bytes\":%u,\"packets\":%u,\"metadata_bytes\":%u,\"sco_buffer_bytes\":255,\"sco_buffers\":1,\"voice_ready\":false}\n",
                   runtime.rx_flow.state==DBT_RX_FLOW_ACTIVE?"true":"false",runtime.rx_flow.packet_bytes,
                   runtime.rx_flow.packet_count,(unsigned)sizeof(runtime.rx_flow));
        }
#ifdef DBT_PROBE_CLASSIC_SECURITY_TEST
        if(runtime.ready && !echo_initialized) {
            if(probe_echo_init(&runtime,&gap)!=DBT_OK || probe_sdp_init(&runtime)!=DBT_OK
#ifdef DBT_PROBE_AUDIO_TEST
               || probe_audio_init(&runtime,owner)!=DBT_OK
#endif
              )
                dbt_runtime_abort(&runtime,DBT_FAULT_BAD_RESPONSE);
            else {
                echo_initialized=true;
                printf("{\"probe\":\"sdp_ready\",\"records\":2,\"metadata_bytes\":%u,\"audio_ready\":false}\n",probe_sdp_bytes());
#ifdef DBT_PROBE_INGRESS_TRACE
                printf("{\"probe\":\"ingress_trace\",\"diagnostic_bytes\":%u,\"production_measurement\":false}\n",probe_ingress_bytes());
#endif
            }
        }
        probe_security_poll(now);
        if(echo_initialized) {
            probe_echo_poll();
#ifdef DBT_PROBE_AUDIO_TEST
            probe_audio_poll(now);
#endif
            if(probe_sdp_poll()!=DBT_OK)dbt_runtime_abort(&runtime,DBT_FAULT_BAD_RESPONSE);
        }
#endif
#ifdef DBT_PROBE_GATT_TEST
        if (runtime.ready && !gatt_initialized) {
            if (probe_gatt_init(&runtime, &gap) != DBT_OK) {
                dbt_runtime_abort(&runtime, DBT_FAULT_BAD_RESPONSE);
            } else gatt_initialized = true;
        }
        if (gatt_initialized) probe_gatt_poll(now);
#endif
        if (runtime.ready && receive_flow_ready && gap.state == DBT_GAP_IDLE) (void)dbt_gap_start(&gap);
        dbt_gap_poll(&gap, now);
#ifdef DBT_PROBE_CLASSIC_SECURITY_TEST
        if(gap.state==DBT_GAP_READY && !gap.classic_pending && gap.classic_scan!=3)
            (void)dbt_gap_set_classic_scan(&gap,true,true);
        if(gap.classic_scan==3 && !security_reported) {
            security_reported=true;
            printf("{\"probe\":\"classic_security_probe_ready\",\"configured\":true,"
                   "\"metadata_bytes\":%u,\"loaded_bonds\":%u,\"worker_stack_bytes\":3072,"
                   "\"logger_stack_bytes\":2048,\"audio_ready\":false}\n",
                   probe_security_bytes(),probe_security_loaded_bonds());
        }
#endif
#ifdef DBT_PROBE_GATT_TEST
        if (gatt_initialized && gap.state == DBT_GAP_READY && !gap.advertising &&
            !gap.advertising_pending && !probe_gatt_connected())
            (void)dbt_gap_set_advertising(&gap, true);
        if (gap.advertising && !gatt_reported) {
            gatt_reported = true;
            printf("{\"probe\":\"gatt_probe_ready\",\"att_mtu\":517,\"gatt_bytes\":%u,"
                   "\"diagnostic_only\":true,\"bonding_ready\":false,\"audio_ready\":false}\n",
                   probe_gatt_bytes());
        }
#endif
        if (host->state != previous) {
            previous = host->state;
            if (host->state == DBT_HOST_IDENTIFIED) {
                printf("{\"probe\":\"controller_identified\",\"profiles_ready\":false,"
                       "\"hci_version\":%u,\"manufacturer\":%u,\"host_bytes\":%u,\"runtime_bytes\":%u,"
                       "\"classic_acl_bytes\":%u,\"classic_acl_packets\":%u,"
                       "\"le_acl_bytes\":%u,\"le_acl_packets\":%u,\"shared_acl_pool\":%s,"
                       "\"internal_free\":%u,\"internal_largest\":%u}\n",
                       host->controller.hci_version, host->controller.manufacturer,
                       (unsigned)sizeof(*host), (unsigned)sizeof(runtime), host->controller.classic_acl_bytes,
                       host->controller.classic_acl_packets, host->controller.le_acl_bytes,
                       host->controller.le_acl_packets, host->controller.shared_acl_pool ? "true" : "false",
                       (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                       (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
            } else if (host->state == DBT_HOST_FAILED) {
                printf("{\"probe\":\"failed\",\"fault\":%u,\"controller_status\":%u,"
                       "\"pending_opcode\":%u,\"rx_rejected\":%" PRIu32 ",\"rx_flow_failed_opcode\":%u}\n",
                       (unsigned)host->fault, host->controller_status, host->pending_opcode,
                       atomic_load_explicit(&host->ingress.rejected, memory_order_relaxed),runtime.rx_flow.failed_opcode);
            }
        }
        if (gap.state != previous_gap) {
            previous_gap = gap.state;
            printf("{\"probe\":\"gap_state\",\"state\":%u,\"controller_status\":%u,"
                   "\"profiles_ready\":false,\"advertising\":%s,\"classic_scan\":%u}\n",
                   (unsigned)gap.state, gap.last_status, gap.advertising ? "true" : "false", gap.classic_scan);
        }
        uint32_t wait = dbt_runtime_wait_ms(&runtime, now_ms());
        uint32_t gap_wait = dbt_gap_wait_ms(&gap);
        if (gap_wait < wait) wait = gap_wait;
#ifdef DBT_PROBE_GATT_TEST
        uint32_t att_wait = probe_gatt_wait_ms(now_ms());
        if (att_wait < wait) wait = att_wait;
#endif
#ifdef DBT_PROBE_CLASSIC_SECURITY_TEST
        uint32_t security_wait=probe_security_wait_ms(now_ms());
        if(security_wait<wait)wait=security_wait;
        uint32_t echo_wait=probe_echo_wait_ms();
        if(echo_wait<wait)wait=echo_wait;
        uint32_t sdp_wait=probe_sdp_wait_ms();
        if(sdp_wait<wait)wait=sdp_wait;
#ifdef DBT_PROBE_AUDIO_TEST
        uint32_t audio_wait=probe_audio_wait_ms(now_ms());
        if(audio_wait<wait)wait=audio_wait;
#endif
#endif
        if (wait == 0) continue;
        TickType_t ticks = portMAX_DELAY;
        if (wait != UINT32_MAX) {
            ticks = pdMS_TO_TICKS(wait);
            if (ticks == 0) ticks = 1;
        }
        /* A notification before this call remains pending, avoiding a lost wakeup. */
        ulTaskNotifyTake(pdTRUE, ticks);
    }
}
