#include <inttypes.h>
#include <stdio.h>
#include "dbt_gap.h"
#ifdef DBT_PROBE_AUDIO_TEST
#include "probe_audio.h"
#endif
#ifdef DBT_PROBE_GATT_TEST
#include "probe_gatt.h"
#include "dbt_ble_nvs.h"
#ifdef DBT_PROBE_AUDIO_OUTPUT
#include "probe_output.h"
#endif
#endif
#ifdef DBT_PROBE_CLASSIC_SECURITY_TEST
#include "probe_security.h"
#include "probe_echo.h"
#include "probe_sdp.h"
#endif
#ifdef DBT_PROBE_INGRESS_TRACE
#include "probe_ingress.h"
#endif
#ifdef DBT_PROBE_SMP
#include "probe_smp.h"
#endif
#if defined(DBT_PROBE_SMP) && defined(DBT_PROBE_GATT_TEST) && defined(DBT_PROBE_AUDIO_OUTPUT)
#define DBT_PROBE_PRODUCT_RUNTIME 1
#include "probe_product_runtime.h"
#endif
#ifdef DBT_PROBE_PRODUCT_IO
#if !defined(DBT_PROBE_PRODUCT_RUNTIME) || !defined(DBT_PROBE_AUDIO_TEST) || !defined(DBT_PROBE_CLASSIC_SECURITY_TEST) || !defined(DBT_PROBE_AVRCP) || !defined(DBT_PROBE_SOUND_BUILD)
#error "Product IO requires product runtime, audio, AVRCP and sound support"
#endif
#include "probe_product_io_owner.h"
#include "esp_spiffs.h"
#ifdef DBT_PROBE_PRODUCT_SECURITY
#include "probe_admission.h"
#include "probe_restart.h"
#endif
#endif
#ifdef DBT_PROBE_AVRCP
#include "probe_avrcp.h"
#include "probe_output.h"
#endif
#ifdef DBT_PROBE_RFCOMM
#include "probe_rfcomm.h"
#endif
#include "esp_system.h"
#include "esp_bt.h"
#ifdef DBT_PROBE_KEEP_OWNED_ACTIVE
#include "esp_ota_ops.h"
#include "esp_app_desc.h"
#include "dbt_boot_policy.h"
#if CONFIG_BOOTLOADER_APP_ANTI_ROLLBACK
#error "Owned boot confirmation must not update anti-rollback eFuses"
#endif
#endif
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#if !CONFIG_IDF_TARGET_ESP32 || !CONFIG_BT_CONTROLLER_ONLY || !CONFIG_BTDM_CTRL_MODE_BTDM
#error "Probe requires original ESP32, controller-only host selection and dual mode"
#endif

#ifdef DBT_PROBE_CODEC_CHECK
extern bool probe_codec_check(TaskHandle_t owner);
#endif
static dbt_runtime_t runtime;
static dbt_gap_t gap;
static TaskHandle_t owner;
#ifdef DBT_PROBE_SMP
static bool smp_initialized,disconnect_overflow;
static struct {bool used;uint16_t handle;uint32_t generation;} disconnects[DBT_MAX_LINKS];
static void defer_disconnect(const dbt_link_t *link){
    if(!link)return;
    for(unsigned i=0;i<DBT_MAX_LINKS;++i)if(disconnects[i].used){
        const dbt_link_t *current=dbt_acl_find(&runtime.acl,disconnects[i].handle);
        if(!current || current->generation!=disconnects[i].generation)disconnects[i].used=false;
    }
    for(unsigned i=0;i<DBT_MAX_LINKS;++i)if(disconnects[i].used && disconnects[i].handle==link->handle && disconnects[i].generation==link->generation)return;
    for(unsigned i=0;i<DBT_MAX_LINKS;++i)if(!disconnects[i].used){disconnects[i].used=true;disconnects[i].handle=link->handle;disconnects[i].generation=link->generation;return;}
    disconnect_overflow=true;
}
#endif
#ifdef DBT_PROBE_AVRCP
#ifdef DBT_PROBE_PRODUCT_IO
static void product_volume_notify(uint8_t volume);
#endif
static dbt_result_t avrcp_volume(void *context,uint8_t volume){(void)context;dbt_result_t r=probe_output_volume(volume);
#ifdef DBT_PROBE_PRODUCT_IO
    if(r==DBT_OK)product_volume_notify(volume);
#endif
    return r;}
#endif
static const char *tag = "native_bt_probe";
/* app_main runs in the IDF main task, which IDF deletes when app_main returns.
 * Existing providers, render workers and controller callbacks retain owner.
 * Park that task for the lifetime of a fault, even if a worker is still
 * retiring a loan; an unexpected resume simply parks it again. */
static _Noreturn void owner_fault_park(const char *stage) {
    ESP_LOGE(tag,"Owner parked after startup failure: %s",stage);
    for(;;)vTaskSuspend(NULL);
}
#if defined(DBT_PROBE_GATT_TEST) && defined(DBT_PROBE_AUDIO_OUTPUT)
static dbt_ble_control_state_t product_state;
static char product_name[33];
static _Atomic bool product_loaded;
static bool product_load_ok;
#ifdef DBT_PROBE_PRODUCT_RUNTIME
static probe_product_t product;
static probe_product_runtime_t product_runtime;
#ifdef DBT_PROBE_PRODUCT_SECURITY
static probe_restart_t product_restart;
#endif
static bool product_runtime_ready,product_provider_ready,product_starting;
static probe_application_other_command_t product_other_callback;
static probe_application_audio_busy_t product_audio_busy_callback;
static void *product_other_context;
#ifdef DBT_PROBE_PRODUCT_IO
static _Atomic bool product_io_done;
static bool product_io_started,product_io_ready,product_io_sound;
static probe_product_io_owner_t product_io_owner;
static _Atomic bool startup_sound_release,startup_sound_done;
#endif
static uint32_t now_ms(void);
static void wake_owner(void);
#ifdef DBT_PROBE_PRODUCT_IO
static void load_product_io(void *unused);
static void product_volume_notify(uint8_t volume){
    probe_avrcp_snapshot_t view;
    if(probe_avrcp_snapshot(&view)==DBT_OK && view.connected && product_io_owner.audio_connected &&
       view.generation==product_io_owner.audio_generation)
        probe_product_io_owner_volume(&product_io_owner,volume,now_ms());
    else if(product_io_owner.io_ready)probe_product_io_volume_feedback(volume);
}
static void product_audio_event(void *unused,probe_audio_product_event_t event,uint16_t handle,uint32_t generation,uint32_t rate,uint32_t now){
    (void)unused;probe_product_io_owner_audio(&product_io_owner,(unsigned)event,handle,generation,rate,now);
}
static void startup_sound_worker(void *unused){
    (void)unused;
    do {
        (void)probe_product_io_worker_poll(NULL,false);
        vTaskDelay(pdMS_TO_TICKS(10));
    }while(!atomic_load_explicit(&startup_sound_release,memory_order_acquire) || probe_product_io_sound_busy());
    atomic_store_explicit(&startup_sound_done,true,memory_order_release);wake_owner();vTaskDelete(NULL);
}
static bool product_boot_io(void){
    if(!product_runtime_ready || probe_audio_output_bootstrap()!=DBT_OK)return false;
    product_io_started=true;
    if(xTaskCreatePinnedToCore(load_product_io,"native_product_io",4096,NULL,1,NULL,0)!=pdPASS)return false;
    int64_t deadline=esp_timer_get_time()+5000000;
    while(!atomic_load_explicit(&product_io_done,memory_order_acquire)){
        if(esp_timer_get_time()>=deadline)return false;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if(!product_io_ready)return false;
    product_io_owner.io_ready=true;product_io_owner.sound_enabled=product_io_sound;product_io_owner.gap=&gap;
    uint8_t sound=product_io_sound?probe_product_io_sound_status():0;
    if(!probe_product_io_owner_startup_needed(product_io_sound,sound))return true;
    if(probe_product_io_play(0,48000,true)!=DBT_OK ||
       xTaskCreatePinnedToCore(startup_sound_worker,"native_boot_sound",4096,NULL,1,NULL,0)!=pdPASS)return false;
    bool received=false,good=false,cancelled=false;
    deadline=esp_timer_get_time()+30000000;
    for(;;){
        probe_product_io_play_result_t result;
        if(probe_product_io_take_play_result(&result)){received=true;good=result.type==0 && result.result==DBT_OK;}
        if(received && !probe_product_io_sound_busy())break;
        if(esp_timer_get_time()>=deadline){
            if(cancelled){atomic_store_explicit(&startup_sound_release,true,memory_order_release);return false;}
            probe_product_io_cancel_sound();cancelled=true;deadline=esp_timer_get_time()+5000000;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    atomic_store_explicit(&startup_sound_release,true,memory_order_release);
    deadline=esp_timer_get_time()+5000000;
    while(!atomic_load_explicit(&startup_sound_done,memory_order_acquire)){
        if(esp_timer_get_time()>=deadline)return false;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    printf("{\"probe\":\"startup_sound\",\"attached\":%s,\"retired\":true,\"cancelled\":%s}\n",good?"true":"false",cancelled?"true":"false");
    probe_output_stats_t output;probe_output_stats(&output);
    return !output.failed && !output.request_fault;
}
#endif
static uint32_t product_clock(void *unused){(void)unused;return now_ms();}
static void product_wake_owner(void *unused){(void)unused;wake_owner();}
static void product_wake_worker(void *unused){(void)unused;probe_gatt_control_wake();}
static bool product_other(void *unused,const dbt_ble_control_peer_t *peer,uint8_t command,const uint8_t *data,uint16_t size,uint8_t *out,uint8_t *out_size){
    (void)unused;return product_other_callback && product_other_callback(product_other_context,peer,command,data,size,out,out_size);
}
bool probe_application_set_other_provider(probe_application_other_command_t callback,probe_application_audio_busy_t busy,void *context){
    if(xTaskGetCurrentTaskHandle()!=owner || product_starting)return false;
    product_other_callback=callback;product_audio_busy_callback=busy;product_other_context=context;return true;
}
static bool product_audio_busy(void){return product_audio_busy_callback && product_audio_busy_callback(product_other_context);}
static bool product_mux_command(void *ctx,const dbt_ble_control_peer_t *peer,uint8_t command,const uint8_t *data,uint16_t size,uint8_t *out,uint8_t *out_size){
    if((command==4 || (command>=0x20 && command<=0x23)) && product_audio_busy())return false;
    return probe_product_command(ctx,peer,command,data,size,out,out_size);
}
#ifdef DBT_PROBE_PRODUCT_SECURITY
static bool product_ota_worker(void *ctx,const dbt_ble_control_peer_t *peer,uint32_t epoch,uint8_t cmd,const uint8_t *data,uint16_t size,uint8_t *out,uint8_t *out_size){
    if(product_audio_busy())return false;
    return probe_product_runtime_ota_worker(ctx,peer,epoch,cmd,data,size,out,out_size);
}
static dbt_result_t product_ota_prepare(void *ctx,const dbt_ble_control_peer_t *peer,uint32_t epoch){
    if(ctx!=&product_runtime || !probe_product_runtime_ota_valid(ctx,peer,epoch))return DBT_INVALID;
    return probe_restart_prepare(&product_restart,epoch,now_ms());
}
static _Noreturn void product_restart_now(void){esp_restart();}
static void product_restart_step(uint32_t now){
    if(!product_runtime_ready)return;
    if(!probe_gatt_restart_fenced()){
        if(probe_restart_active(&product_restart))probe_restart_cancel(&product_restart,now);
        return;
    }
    bool quiescent=probe_restart_active(&product_restart) && probe_restart_quiescent(&product_restart,now);
    if(product_restart.failed){probe_gatt_restart_cancel();probe_restart_cancel(&product_restart,now);return;}
    bool approved=probe_product_runtime_restart_ready(&product_runtime,now,quiescent);
    if(quiescent && approved)product_restart_now();
}
#endif
static bool product_mux_worker_poll(void *ctx,bool retiring){
    bool product_ok=probe_product_worker_poll_guarded(ctx,retiring,!product_audio_busy());
#ifdef DBT_PROBE_PRODUCT_IO
    bool io_ok=product_io_ready?probe_product_io_worker_poll(NULL,retiring):true;
    return product_ok && io_ok;
#else
    return product_ok;
#endif
}
static void product_mux_peer_lost(void *ctx,const dbt_ble_control_peer_t *peer){
    probe_product_peer_lost(ctx,peer);
#ifdef DBT_PROBE_PRODUCT_IO
    if(product_io_ready)probe_product_io_peer_lost(NULL,peer);
#endif
}
#ifdef DBT_PROBE_PRODUCT_IO
static bool product_io_other(void *unused,const dbt_ble_control_peer_t *peer,uint8_t cmd,const uint8_t *data,uint16_t n,uint8_t *out,uint8_t *size){
    (void)unused;return product_io_ready && probe_product_io_command(NULL,peer,cmd,data,n,out,size);
}
static bool product_io_busy(void *unused){(void)unused;return product_io_ready && probe_product_io_sound_busy();}
static bool product_io_peer(void *unused,const dbt_ble_control_peer_t *peer){
    (void)unused;dbt_product_peer_value_t v;
    return peer && dbt_product_peer_read(&product.peer,&v) && v.peer.handle==peer->handle &&
           v.peer.generation==peer->generation && v.encrypted && v.bonded && v.allow_name;
}
static void load_product_io(void *unused){
    (void)unused;
#ifdef DBT_PROBE_PRODUCT_SECURITY
    if(probe_admission_startup_worker(&runtime,owner)!=DBT_OK){
        product_io_ready=false;product_io_sound=false;
        atomic_store_explicit(&product_io_done,true,memory_order_release);wake_owner();vTaskDelete(NULL);return;
    }
#endif
    bool mounted=esp_spiffs_mounted("spiffs");
    if(!mounted){
        esp_vfs_spiffs_conf_t conf={.base_path="/spiffs",.partition_label="spiffs",.max_files=5,.format_if_mount_failed=false};
        mounted=esp_vfs_spiffs_register(&conf)==ESP_OK && esp_spiffs_mounted("spiffs");
    }
    probe_product_io_hooks_t hooks={0};hooks.peer_valid=product_io_peer;hooks.wake_owner=product_wake_owner;
    hooks.led=true;hooks.input=true;hooks.input_optional=true;hooks.sound=mounted;
    product_io_ready=probe_product_io_startup_worker(&product_state,&hooks);
    product_io_sound=product_io_ready && mounted;
    probe_product_io_capabilities_t caps;
    if(probe_product_io_capabilities(&caps))
        printf("{\"probe\":\"product_io_capabilities\",\"ready\":%s,\"led\":%s,\"sound\":%s,\"input_available\":%s,\"input_no_ack\":%s,\"input_probe_status\":%u}\n",
            caps.ready?"true":"false",caps.led?"true":"false",caps.sound?"true":"false",
            caps.input==PROBE_PRODUCT_INPUT_AVAILABLE?"true":"false",
            caps.input==PROBE_PRODUCT_INPUT_NO_ACK?"true":"false",(unsigned)caps.input);
    atomic_store_explicit(&product_io_done,true,memory_order_release);wake_owner();vTaskDelete(NULL);
}
#endif
dbt_result_t probe_application_grant_ota(uint16_t handle,uint32_t generation,const dbt_ota_manifest_t *manifest){
    dbt_ble_control_peer_t peer={handle,generation};
    return product_runtime_ready && xTaskGetCurrentTaskHandle()==owner?probe_product_runtime_grant_ota(&product_runtime,&peer,manifest):DBT_INVALID;
}
dbt_result_t probe_application_activate_ota(uint16_t handle,uint32_t generation){
    dbt_ble_control_peer_t peer={handle,generation};
    return product_runtime_ready && xTaskGetCurrentTaskHandle()==owner?probe_product_runtime_request_activation(&product_runtime,&peer):DBT_INVALID;
}
probe_product_action_result_t probe_application_take_ota_result(void){
    return product_runtime_ready && xTaskGetCurrentTaskHandle()==owner?probe_product_runtime_take_activation(&product_runtime):PROBE_PRODUCT_NO_RESULT;
}
#endif
static bool product_authorize(void *context,const dbt_link_t *link,uint8_t command){
    (void)context;(void)command;
#ifdef DBT_PROBE_PRODUCT_SECURITY
    uint8_t type,id[6],tag[32];
    return link && probe_admission_trusted_le(link->handle,link->generation,&type,id,tag)==DBT_OK;
#else
    return link && link->kind==DBT_LINK_LE;
#endif
}
static bool publish_eq(void *context,int8_t b,int8_t m,int8_t t){(void)context;return probe_output_eq(b,m,t)==DBT_OK;}
static bool publish_control(void *context,uint8_t value){(void)context;return probe_output_control(value)==DBT_OK;}
static void load_product(void *unused){
    (void)unused;
#ifdef DBT_PROBE_PRODUCT_RUNTIME
    probe_product_hooks_t hooks={0};hooks.now_ms=product_clock;hooks.wake_owner=product_wake_owner;
    hooks.wake_worker=product_wake_worker;hooks.other_command=product_other;hooks.ota_timeout_ms=30000;hooks.name_timeout_ms=2000;
    product_provider_ready=probe_product_startup_worker(&product,&product_state,&hooks);
    product_load_ok=product_state.valid;
#else
    product_load_ok=dbt_ble_nvs_load_state(&product_state)==DBT_OK;
#endif
    atomic_store_explicit(&product_loaded,true,memory_order_release);vTaskDelete(NULL);
}
static bool product_startup(void){
    static const uint8_t name[]="BDK SPEAKER",firmware[]="owned-dev";
    for(unsigned i=0;i<sizeof(name)-1;++i)product_state.name[i]=name[i];
    for(unsigned i=0;i<sizeof(firmware)-1;++i)product_state.firmware[i]=firmware[i];
    product_state.name_size=sizeof(name)-1;product_state.firmware_size=sizeof(firmware)-1;
    product_state.led[0]=14;product_state.led[1]=10;
#ifdef DBT_PROBE_PRODUCT_RUNTIME
    product_starting=true;
#endif
    if(xTaskCreatePinnedToCore(load_product,"native_cfg_load",4096,NULL,1,NULL,0)!=pdPASS)return false;
    int64_t deadline=esp_timer_get_time()+2000000;
    while(!atomic_load_explicit(&product_loaded,memory_order_acquire)){
        if(esp_timer_get_time()>=deadline)return false;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if(product_load_ok){for(unsigned i=0;i<product_state.name_size;++i)product_name[i]=(char)product_state.name[i];product_name[product_state.name_size]=0;}
    return product_load_ok;
}
#endif


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
    uint8_t *raw=heap_caps_aligned_alloc(8,(size_t)size+8u,MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
    if(!raw)return NULL;
    uint32_t *header=(uint32_t *)raw;header[0]=size;header[1]=~(uint32_t)size;
    return raw+8;
}
static void release_rx(void *context, uint8_t *data) {
    (void)context;
    if(!data)return;
    uint32_t *header=(uint32_t *)(data-8);uint32_t size=header[0];
    configASSERT(size<=UINT16_MAX && header[1]==~size);
    volatile uint8_t *wipe=data-8;
    for(uint32_t i=0;i<size+8u;++i)wipe[i]=0;
    heap_caps_free(data-8);
}
#if defined(DBT_PROBE_GATT_TEST) || defined(DBT_PROBE_CLASSIC_SECURITY_TEST) || defined(DBT_PROBE_SMP)
static void profile_link(void *context,const dbt_link_change_t *change) {
    (void)context;
#ifdef DBT_PROBE_PRODUCT_SECURITY
    probe_admission_link(change);
#endif
#ifdef DBT_PROBE_PRODUCT_IO
    probe_product_io_owner_link(&product_io_owner,change);
#endif
#ifdef DBT_PROBE_SMP
    if(smp_initialized)probe_smp_link(change);
#endif
#ifdef DBT_PROBE_CLASSIC_SECURITY_TEST
    probe_security_link(change);
#endif
#ifdef DBT_PROBE_GATT_TEST
    probe_gatt_link(context,change);
#endif
}
#endif
#if defined(DBT_PROBE_GATT_TEST) || defined(DBT_PROBE_SMP)
static void profile_fixed(void *context,const dbt_link_t *link,uint16_t cid,const uint8_t *data,uint16_t size){
#ifdef DBT_PROBE_SMP
    if(cid==6){if(!smp_initialized || probe_smp_receive(link,cid,data,size)!=DBT_OK)defer_disconnect(link);return;}
#endif
#ifdef DBT_PROBE_GATT_TEST
    if(cid==4){
#ifdef DBT_PROBE_PRODUCT_RUNTIME
        if(product_runtime_ready)probe_product_runtime_refresh_security(&product_runtime);
#endif
        probe_gatt_receive(context,link,cid,data,size);
    }
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
#ifdef DBT_PROBE_SMP
    if(smp_initialized && probe_smp_event(packet))return;
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
#ifdef DBT_PROBE_RFCOMM
    probe_rfcomm_event(event,channel);
#endif
#ifdef DBT_PROBE_AVRCP
    probe_avrcp_event(event,channel);
#endif
#ifdef DBT_PROBE_AUDIO_TEST
    probe_audio_event(event,channel);
#endif
}
static bool profile_accept(void *context,const dbt_link_t *link,uint16_t psm){
    bool allowed=true;
#ifdef DBT_PROBE_PRODUCT_SECURITY
    if(probe_restart_active(&product_restart))return false;
#endif
#ifdef DBT_PROBE_RFCOMM
    allowed=allowed && probe_rfcomm_accept(link,psm);
#endif
#ifdef DBT_PROBE_AUDIO_TEST
    allowed=allowed && probe_audio_accept(context,link,psm);
#endif
#ifdef DBT_PROBE_AVRCP
    allowed=allowed && probe_avrcp_accept(context,link,psm);
#endif
    return allowed;
}
static void profile_data(void *context, const dbt_channel_t *channel, const uint8_t *data, uint16_t size) {
#ifdef DBT_PROBE_RFCOMM
    if(channel->psm==3){(void)probe_rfcomm_data(channel,data,size);return;}
#endif
#ifdef DBT_PROBE_AUDIO_TEST
    if(channel->psm==0x19){probe_audio_receive(channel,data,size);return;}
#endif
#ifdef DBT_PROBE_AVRCP
    if(channel->psm==0x17){probe_avrcp_receive(channel,data,size);return;}
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

#ifdef DBT_PROBE_KEEP_OWNED_ACTIVE
/* Owner starts this only after controller/GAP/service startup qualification.
 * Flash work stays outside the protocol owner. OTA VALID acknowledges boot
 * selection only; it is not production, profile or RF qualification. */
static void confirm_owned_boot(void *unused){
    (void)unused;
    const esp_partition_t *running=esp_ota_get_running_partition();
    esp_ota_img_states_t state=ESP_OTA_IMG_UNDEFINED;
    esp_err_t result=ESP_ERR_INVALID_STATE;
    const esp_app_desc_t *description=esp_app_get_description();
    if(running && description && dbt_boot_owned_slot(running->type,running->subtype,running->label,description->project_name)){
        result=esp_ota_get_state_partition(running,&state);
        if(result==ESP_OK && state!=ESP_OTA_IMG_VALID)result=esp_ota_mark_app_valid_cancel_rollback();
        if(result==ESP_OK)result=esp_ota_get_state_partition(running,&state);
    }
    bool kept=result==ESP_OK && state==ESP_OTA_IMG_VALID;
    printf("{\"probe\":\"boot_selection\",\"host\":\"owned\",\"kept_active\":%s,\"partition\":%u,\"ota_state\":%u,\"result\":%u,\"production_ready\":false}\n",
           kept?"true":"false",running?(unsigned)running->address:0,(unsigned)state,(unsigned)result);
    vTaskDelete(0);
}
#endif

void app_main(void) {
    owner = xTaskGetCurrentTaskHandle();
    vTaskPrioritySet(owner, 6);
    dbt_transport_t transport = { 0, transport_ready, transport_send };
    dbt_runtime_hooks_t hooks = {0};
    hooks.allocate = allocate_rx; hooks.release = release_rx; hooks.packet = observe;
#if defined(DBT_PROBE_GATT_TEST) || defined(DBT_PROBE_SMP)
    hooks.channels.fixed_data = profile_fixed;
#endif
#if defined(DBT_PROBE_GATT_TEST) || defined(DBT_PROBE_CLASSIC_SECURITY_TEST) || defined(DBT_PROBE_SMP)
    hooks.link = profile_link;
#endif
#ifdef DBT_PROBE_CLASSIC_SECURITY_TEST
    hooks.channels.authorize=probe_security_authorize;
    hooks.channels.security_request=probe_security_request;
    hooks.channels.event=profile_channel;
    hooks.channels.data=profile_data;
    hooks.channels.accept=profile_accept;
#endif
    if (dbt_runtime_init(&runtime, transport, hooks, 4096, 2000) != DBT_OK) {
        ESP_LOGE(tag, "core initialization failed");
        owner_fault_park("runtime initialization");
    }
    /* Startup storage work precedes controller enable; never erase user NVS. */
    esp_err_t startup_error=nvs_flash_init();
    if(startup_error!=ESP_OK){ESP_LOGE(tag,"NVS initialization failed: %d",(int)startup_error);owner_fault_park("NVS initialization");}
#if defined(DBT_PROBE_GATT_TEST) && defined(DBT_PROBE_AUDIO_OUTPUT)
#ifdef DBT_PROBE_PRODUCT_IO
    if(!probe_application_set_other_provider(product_io_other,product_io_busy,NULL))owner_fault_park("product IO registration");
#endif
    if(!product_startup())owner_fault_park("product settings load");
#endif
    dbt_gap_config_t gap_config = { .name = "NATIVE_BT_PROBE",
        .advertising = {2, 1, 2}, .advertising_size = 3,
        .advertising_minimum = 0x80, .advertising_maximum = 0x100 };
#if defined(DBT_PROBE_GATT_TEST) && defined(DBT_PROBE_AUDIO_OUTPUT)
    if(product_name[0]){unsigned i=0;for(;i<sizeof(gap_config.name)-1u && product_name[i];++i)gap_config.name[i]=product_name[i];gap_config.name[i]=0;}
#endif
    dbt_gap_hooks_t gap_hooks = {0};
#ifdef DBT_PROBE_CLASSIC_SECURITY_TEST
    gap_hooks.accept_classic=probe_security_accept;
#endif
#ifdef DBT_PROBE_GATT_TEST
    probe_gatt_advertisement(&gap_config);
#endif
    if (dbt_gap_init(&gap, &runtime.host, gap_config, gap_hooks) != DBT_OK) {
        ESP_LOGE(tag, "GAP initialization failed");
        owner_fault_park("GAP initialization");
    }
#ifdef DBT_PROBE_PRODUCT_RUNTIME
    if(product_provider_ready)product_runtime_ready=probe_product_runtime_init(&product_runtime,&product,&runtime,&gap,probe_smp_security);
#ifdef DBT_PROBE_PRODUCT_SECURITY
    if(product_runtime_ready && !probe_product_runtime_set_trust(&product_runtime,probe_admission_trusted_le))
        owner_fault_park("product trust registration");
#endif
#endif
#ifdef DBT_PROBE_PRODUCT_IO
    probe_audio_product_observer(product_audio_event,NULL);
    if(!product_boot_io())owner_fault_park("product IO bootstrap; controller not enabled");
#endif
    /* Existing NVS is preserved. Initialization failure does not erase bonds/settings. */
    /* NVS was initialized before GAP configuration. */
#ifdef DBT_PROBE_CODEC_CHECK
    if(!probe_codec_check(owner))owner_fault_park("target codec check; controller not enabled");
#endif
#ifdef DBT_PROBE_CLASSIC_SECURITY_TEST
    if(probe_security_init(&runtime,&gap,owner)!=DBT_OK) {
        owner_fault_park("Classic security/storage initialization");
    }
#endif
#ifdef DBT_PROBE_PRODUCT_SECURITY
    probe_admission_first_boot(now_ms(),probe_security_loaded_bonds());
#endif
    esp_bt_controller_config_t configuration = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    startup_error=esp_bt_controller_init(&configuration);
    if(startup_error!=ESP_OK){ESP_LOGE(tag,"Controller init failed: %d",(int)startup_error);owner_fault_park("controller initialization");}
    static const esp_vhci_host_callback_t callbacks = { send_available, receive_packet };
    startup_error=esp_vhci_host_register_callback(&callbacks);
    if(startup_error!=ESP_OK){ESP_LOGE(tag,"Controller callback registration failed: %d",(int)startup_error);owner_fault_park("controller callback registration");}
    startup_error=esp_bt_controller_enable(ESP_BT_MODE_BTDM);
    if(startup_error!=ESP_OK){ESP_LOGE(tag,"Controller enable failed: %d",(int)startup_error);owner_fault_park("controller enable");}
    if (dbt_runtime_start(&runtime, now_ms()) != DBT_OK) {
        ESP_LOGE(tag, "core start failed");
        owner_fault_park("runtime start after controller enable");
    }
    dbt_host_t *host = &runtime.host;
    dbt_host_state_t previous = DBT_HOST_IDLE;
    dbt_gap_state_t previous_gap = DBT_GAP_IDLE;
    bool receive_flow_started=false,receive_flow_reported=false;
#ifdef DBT_PROBE_KEEP_OWNED_ACTIVE
    bool owned_boot_started=false;
#endif
#ifdef DBT_PROBE_CLASSIC_SECURITY_TEST
    bool security_reported=false,echo_initialized=false;
#endif
#ifdef DBT_PROBE_GATT_TEST
    bool gatt_initialized = false, gatt_reported = false;
#endif
    for (;;) {
        uint32_t now = now_ms();
#ifdef DBT_PROBE_SMP
        if(smp_initialized)probe_smp_security_request_preflight(now);
#endif
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
            dbt_result_t services=DBT_OK;
#ifdef DBT_PROBE_AVRCP
            services=probe_avrcp_init(&runtime,avrcp_volume,0,6);
            if(services==DBT_OK)probe_sdp_enable_avrcp(true,true);
#endif
#ifdef DBT_PROBE_RFCOMM
            if(services==DBT_OK)services=probe_rfcomm_init(&runtime,1,0x10004);
            if(services==DBT_OK)services=probe_sdp_enable_extra_record(probe_rfcomm_record());
#endif
            if(services!=DBT_OK || probe_echo_init(&runtime,&gap)!=DBT_OK || probe_sdp_init(&runtime)!=DBT_OK
#ifdef DBT_PROBE_AUDIO_TEST
               || probe_audio_init(&runtime,owner)!=DBT_OK
#endif
              )
                dbt_runtime_abort(&runtime,DBT_FAULT_BAD_RESPONSE);
            else {
                echo_initialized=true;
                printf("{\"probe\":\"sdp_ready\",\"records\":%u,\"metadata_bytes\":%u,\"audio_sink_advertised\":%s,\"audio_ready\":false}\n",
                       probe_sdp_records(),probe_sdp_bytes(),probe_sdp_audio_sink()?"true":"false");
#ifdef DBT_PROBE_INGRESS_TRACE
                printf("{\"probe\":\"ingress_trace\",\"diagnostic_bytes\":%u,\"production_measurement\":false}\n",probe_ingress_bytes());
#endif
            }
        }
        probe_security_poll(now);
        if(echo_initialized) {
            probe_echo_poll();
#ifdef DBT_PROBE_AVRCP
            probe_avrcp_poll(now);
#endif
#ifdef DBT_PROBE_AUDIO_TEST
            probe_audio_poll(now);
#endif
            if(probe_sdp_poll()!=DBT_OK)dbt_runtime_abort(&runtime,DBT_FAULT_BAD_RESPONSE);
        }
#endif
#ifdef DBT_PROBE_SMP
        if(runtime.ready && !smp_initialized){
            probe_smp_config_t config={0};
            config.peer_identity_only=true;
            config.local_address_type=0;
            for(unsigned i=0;i<6;++i)config.local_address[i]=runtime.host.controller.address_le[i];
            config.pairing.features=(dbt_smp_feature_policy_t){.io=3,.min_key_size=16,.max_key_size=16,
                .support_sc=true,.require_sc=true,.bonding=true};
            config.pairing.allow_just_works=true;
#ifdef DBT_PROBE_PRODUCT_SECURITY
            config.peripheral_security_requests=true;
#endif
            if(probe_smp_init(&runtime,owner,&config)==DBT_OK)smp_initialized=true;
            else dbt_runtime_abort(&runtime,DBT_FAULT_BAD_RESPONSE);
        }
        if(smp_initialized){
            if(disconnect_overflow)dbt_runtime_abort(&runtime,DBT_FAULT_BAD_RESPONSE);
            probe_smp_poll(now);
            uint16_t failed_handle;uint32_t failed_generation;
            while(probe_smp_take_failure(&failed_handle,&failed_generation)){
                const dbt_link_t *link=dbt_acl_find(&runtime.acl,failed_handle);
                if(link && link->generation==failed_generation)defer_disconnect(link);
            }
            for(unsigned i=0;i<DBT_MAX_LINKS;++i)if(disconnects[i].used){
                const dbt_link_t *link=dbt_acl_find(&runtime.acl,disconnects[i].handle);
                if(!link || link->generation!=disconnects[i].generation || dbt_gap_disconnect(&gap,link,0x13)==DBT_OK)
                    disconnects[i].used=false;
            }
        }
#endif
#ifdef DBT_PROBE_PRODUCT_SECURITY
        probe_admission_poll(now);
#endif
#ifdef DBT_PROBE_GATT_TEST
        bool product_io_settled=true;
#ifdef DBT_PROBE_PRODUCT_IO
        if(runtime.ready && echo_initialized && !product_io_started){
            product_io_started=true;
            if(!product_runtime_ready || xTaskCreatePinnedToCore(load_product_io,"native_product_io",4096,NULL,1,NULL,0)!=pdPASS)
                atomic_store_explicit(&product_io_done,true,memory_order_release);
        }
        product_io_settled=atomic_load_explicit(&product_io_done,memory_order_acquire);
#endif
        if (runtime.ready && !gatt_initialized && product_io_settled) {
#ifdef DBT_PROBE_PRODUCT_SECURITY
            if(!probe_restart_init(&product_restart,&runtime,&product_io_owner) ||
               probe_gatt_control_trust_configure(probe_admission_control_cookie,probe_admission_control_cookie_valid)!=DBT_OK){
                dbt_runtime_abort(&runtime,DBT_FAULT_BAD_RESPONSE);continue;
            }
            probe_gatt_ota_hooks_t ota={.context=&product_runtime,.admit=probe_product_runtime_ota_admit,
                .abandon=probe_product_runtime_ota_abandon,.valid=probe_product_runtime_ota_valid,.command=product_ota_worker,
                .finish=probe_product_runtime_ota_finish,.delivered=probe_product_runtime_ota_delivered,.prepare=product_ota_prepare};
            if(!product_runtime_ready || probe_gatt_ota_configure(&ota)!=DBT_OK){dbt_runtime_abort(&runtime,DBT_FAULT_BAD_RESPONSE);continue;}
#endif
#ifdef DBT_PROBE_SMP
            if(probe_gatt_security_query_configure(probe_smp_security)!=DBT_OK ||
               probe_gatt_bond_query_configure(probe_smp_committed_bond)!=DBT_OK){
                dbt_runtime_abort(&runtime,DBT_FAULT_BAD_RESPONSE);continue;
            }
#endif
#ifdef DBT_PROBE_AUDIO_OUTPUT
            dbt_ble_control_hooks_t control_hooks={0};
            control_hooks.authorize=product_authorize;control_hooks.commit_eq=dbt_ble_nvs_commit_eq;
            control_hooks.commit_control=dbt_ble_nvs_commit_control;control_hooks.publish_eq=publish_eq;control_hooks.publish_control=publish_control;
#ifdef DBT_PROBE_PRODUCT_RUNTIME
            if(product_runtime_ready){control_hooks.context=&product;control_hooks.command=product_mux_command;
                control_hooks.worker_poll=product_mux_worker_poll;control_hooks.peer_lost=product_mux_peer_lost;}
#endif
            if(probe_output_eq(product_state.eq[0],product_state.eq[1],product_state.eq[2])!=DBT_OK ||
               probe_output_control(product_state.control)!=DBT_OK || probe_gatt_control_configure(&product_state,&control_hooks)!=DBT_OK){
                dbt_runtime_abort(&runtime,DBT_FAULT_BAD_RESPONSE);continue;
            }
#endif
            if (probe_gatt_init(&runtime, &gap) != DBT_OK) {
                dbt_runtime_abort(&runtime, DBT_FAULT_BAD_RESPONSE);
            } else gatt_initialized = true;
        }
        if (gatt_initialized){
            probe_gatt_poll(now);probe_gatt_retry_eq_publication();probe_gatt_retry_control_publication();
#ifdef DBT_PROBE_PRODUCT_IO
            if(product_io_ready){product_io_owner.gap=&gap;product_io_owner.sound_enabled=product_io_sound;probe_product_io_owner_poll(&product_io_owner,now);}
#endif
#ifdef DBT_PROBE_AUDIO_OUTPUT
            static bool analysis;static uint32_t meter_at;
            bool ble_meter=probe_gatt_connected(),desired=ble_meter;
#ifdef DBT_PROBE_PRODUCT_IO
            desired=desired || product_io_ready;
#endif
#ifdef DBT_PROBE_PRODUCT_IO
            desired=desired || product_io_ready;
#endif
            if(desired!=analysis && probe_output_analysis(desired)==DBT_OK)analysis=desired;
            if(desired && now-meter_at>=40){uint8_t levels[3];probe_output_meter(levels);if(ble_meter)probe_gatt_meter(levels[0],levels[1],levels[2]);
#ifdef DBT_PROBE_PRODUCT_IO
                if(product_io_ready){probe_output_stats_t stats;probe_output_stats(&stats);probe_product_io_meter(levels[0],levels[1],levels[2],stats.running!=0);}
#endif
                meter_at=now;}
#endif
        }
#endif
#ifdef DBT_PROBE_RFCOMM
        if(echo_initialized)probe_rfcomm_poll(now);
#endif
        if (runtime.ready && receive_flow_ready && gap.state == DBT_GAP_IDLE) (void)dbt_gap_start(&gap);
        dbt_gap_poll(&gap, now);
#ifdef DBT_PROBE_PRODUCT_RUNTIME
        if(product_runtime_ready && gatt_initialized && smp_initialized)probe_product_runtime_poll(&product_runtime);
#ifdef DBT_PROBE_PRODUCT_SECURITY
        if(gatt_initialized)product_restart_step(now);
#endif
#endif
#ifdef DBT_PROBE_CLASSIC_SECURITY_TEST
#ifdef DBT_PROBE_PRODUCT_SECURITY
        bool discoverable=probe_admission_window(now),connectable=!probe_admission_frozen();
        uint8_t wanted_scan=(uint8_t)((discoverable?1u:0u)|(connectable?2u:0u));
        if(gap.state==DBT_GAP_READY && !gap.classic_pending && gap.classic_scan!=wanted_scan)
            (void)dbt_gap_set_classic_scan(&gap,discoverable,connectable);
#else
        if(gap.state==DBT_GAP_READY && !gap.classic_pending && gap.classic_scan!=3)
            (void)dbt_gap_set_classic_scan(&gap,true,true);
#endif
#ifdef DBT_PROBE_PRODUCT_SECURITY
        bool classic_ready=gap.state==DBT_GAP_READY && !gap.classic_pending && gap.classic_scan==wanted_scan && connectable;
#else
        bool classic_ready=gap.classic_scan==3;
#endif
        if(classic_ready && !security_reported) {
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
        if ((gap.advertising || probe_gatt_connected()) && !gatt_reported) {
            gatt_reported = true;
            printf("{\"probe\":\"gatt_probe_ready\",\"att_mtu\":517,\"gatt_bytes\":%u,"
                   "\"diagnostic_only\":true,\"bonding_ready\":false,\"audio_ready\":false}\n",
                   probe_gatt_bytes());
        }
#endif
#ifdef DBT_PROBE_KEEP_OWNED_ACTIVE
        bool qualified=runtime.ready && receive_flow_ready && gap.state==DBT_GAP_READY && host->state==DBT_HOST_IDENTIFIED &&
            host->controller.hci_version==8 && host->controller.manufacturer==96;
#ifdef DBT_PROBE_CLASSIC_SECURITY_TEST
        qualified=qualified && security_reported;
#endif
#ifdef DBT_PROBE_GATT_TEST
        qualified=qualified && gatt_reported;
#endif
        if(qualified && !owned_boot_started){
            owned_boot_started=true;
            if(xTaskCreatePinnedToCore(confirm_owned_boot,"owned_boot",4096,0,1,0,0)!=pdPASS)
                printf("{\"probe\":\"boot_selection\",\"kept_active\":false,\"result\":\"task_oom\"}\n");
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
                const dbt_runtime_failure_t *f=&runtime.failure;
                printf("{\"probe\":\"parser_failure\",\"at_ms\":%" PRIu32 ",\"origin\":%u,\"result\":%u,"
                       "\"handle\":%u,\"generation\":%" PRIu32 ",\"link_present\":%u,\"cid\":%u,\"bytes\":%u,\"declared\":%u,"
                       "\"type\":%u,\"event\":%u,\"pb\":%u,\"bc\":%u,\"code\":%u,\"identifier\":%u,"
                       "\"host_stage\":%u,\"host_packet_bytes\":%u,\"host_type\":%u,\"host_event\":%u}\n",
                       f->at_ms,f->origin,f->result,f->handle,f->generation,(unsigned)f->link_present,f->cid,f->bytes,f->declared,
                       f->packet_type,f->event,f->pb,f->bc,f->code,f->identifier,
                       host->fault_stage,host->fault_packet_size,host->fault_packet_type,host->fault_event);
            }
        }
        if (gap.state != previous_gap) {
            previous_gap = gap.state;
            printf("{\"probe\":\"gap_state\",\"state\":%u,\"controller_status\":%u,"
                   "\"profiles_ready\":false,\"advertising\":%s,\"classic_scan\":%u}\n",
                   (unsigned)gap.state, gap.last_status, gap.advertising ? "true" : "false", gap.classic_scan);
        }
        uint32_t wait = dbt_runtime_wait_ms(&runtime, now_ms());
#ifdef DBT_PROBE_PRODUCT_SECURITY
        {uint32_t admission_wait=probe_admission_wait_ms(now_ms());if(admission_wait<wait)wait=admission_wait;}
#endif
#ifdef DBT_PROBE_SMP
        if(smp_initialized){uint32_t smp_wait=probe_smp_wait_ms(now_ms());if(smp_wait<wait)wait=smp_wait;}
#endif
#ifdef DBT_PROBE_AVRCP
        if(echo_initialized){uint32_t avrcp_wait=probe_avrcp_wait_ms(now_ms());if(avrcp_wait<wait)wait=avrcp_wait;}
#endif
#ifdef DBT_PROBE_PRODUCT_SECURITY
        if(probe_restart_active(&product_restart) && wait>10)wait=10;
#endif
#ifdef DBT_PROBE_RFCOMM
        {uint32_t rfcomm_wait=probe_rfcomm_wait_ms(now_ms());if(rfcomm_wait<wait)wait=rfcomm_wait;}
#endif
        uint32_t gap_wait = dbt_gap_wait_ms(&gap);
        if (gap_wait < wait) wait = gap_wait;
#ifdef DBT_PROBE_PRODUCT_RUNTIME
        if(product_runtime_ready){uint32_t product_wait=probe_product_runtime_wait_ms(&product_runtime,now_ms());if(product_wait<wait)wait=product_wait;}
#endif
#ifdef DBT_PROBE_PRODUCT_IO
        if(product_io_ready){uint32_t io_wait=probe_product_io_owner_wait_ms(&product_io_owner,now_ms());if(io_wait<wait)wait=io_wait;}
#endif
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
