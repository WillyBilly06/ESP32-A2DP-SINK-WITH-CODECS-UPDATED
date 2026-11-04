/* Finite, non-radio target check. This entry point never initializes the
 * controller, NVS, filesystem or product providers. I2S is enabled only by the
 * explicit pipeline diagnostic build. It never marks itself
 * valid. The existing bootloader rollback remains a second restoration path. */
#include "probe_codec_check.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include <stdio.h>
#include <string.h>
#ifdef DBT_PROBE_PIPELINE_DIAGNOSTIC
#include "probe_pipeline_check.h"
#endif

static const uint8_t baseline_elf_sha256[32]={
    0x9b,0xb5,0xc0,0x5b,0x24,0xb7,0xd7,0x65,0x30,0x75,0x5f,0xfd,0x71,0xc8,0x15,0xa3,
    0xd5,0x9f,0x3d,0x44,0x85,0x2a,0xb9,0xb2,0xf7,0x83,0xf2,0xc0,0x88,0x1c,0x93,0x8d};

static void diagnostic(void *unused){
    (void)unused;
    const esp_partition_t *running=esp_ota_get_running_partition();
    const esp_partition_t *baseline=esp_partition_find_first(ESP_PARTITION_TYPE_APP,
        ESP_PARTITION_SUBTYPE_APP_OTA_0,"ota_0");
    esp_app_desc_t description;
    esp_ota_img_states_t state;
    bool admitted=running && running->address==0x390000 && running->size==0x280000 &&
        running->subtype==ESP_PARTITION_SUBTYPE_APP_OTA_1 &&
        esp_ota_get_state_partition(running,&state)==ESP_OK && state==ESP_OTA_IMG_PENDING_VERIFY &&
        baseline && baseline->address==0x110000 && baseline->size==0x280000 &&
        esp_ota_get_partition_description(baseline,&description)==ESP_OK &&
        !strcmp(description.project_name,"bt_audio_sink") &&
        !strcmp(description.version,"1.0.4-idf6.1") &&
        !memcmp(description.app_elf_sha256,baseline_elf_sha256,sizeof(baseline_elf_sha256));
    printf("{\"probe\":\"codec_diagnostic_start\",\"admitted\":%s,\"core\":%d,\"radio_enabled\":false,\"storage_initialized\":false}\n",
        admitted?"true":"false",xPortGetCoreID());
    bool passed=admitted &&
#ifdef DBT_PROBE_PIPELINE_DIAGNOSTIC
        probe_pipeline_check();
#else
        probe_codec_check(xTaskGetCurrentTaskHandle());
#endif
    /* This task is pinned to core 0: esp_restart must not be invoked from the
     * old core-1 path that failed during the released-firmware OTA work. */
    esp_err_t restore=admitted?esp_ota_set_boot_partition(baseline):ESP_ERR_INVALID_STATE;
    printf("{\"probe\":\"codec_diagnostic_return\",\"passed\":%s,\"baseline_selected\":%s,\"restore_result\":%d,\"radio_test\":false}\n",
        passed?"true":"false",restore==ESP_OK?"true":"false",(int)restore);
    fflush(stdout);
    /* One bounded log-drain delay; never a polling loop. A pending image that
     * cannot explicitly select the baseline rolls back on this reset. */
    vTaskDelay(pdMS_TO_TICKS(250));
    esp_restart();
}
void app_main(void){
    if(xTaskCreatePinnedToCore(diagnostic,"codec_return",4096,NULL,6,NULL,0)!=pdPASS){
        /* Remain pending, so an external reset also returns to the baseline.
         * Do not initialize a radio/product fallback after admission failure. */
        printf("{\"probe\":\"codec_diagnostic_start\",\"admitted\":false,\"task_creation_failed\":true}\n");
    }
}
