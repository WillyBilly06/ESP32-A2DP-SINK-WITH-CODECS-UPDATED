#include <setjmp.h>
#include <stdbool.h>
#include <string.h>
#include "esp_ota_ops.h"
#include "probe_codec_check.h"
static jmp_buf restart_target;
static unsigned scenario,checks,selects,restarts,delay_ms;
static esp_partition_t running={17,0x390000,0x280000},baseline={16,0x110000,0x280000};
const esp_partition_t *esp_ota_get_running_partition(void){return scenario==2?NULL:&running;}
const esp_partition_t *esp_partition_find_first(int type,int subtype,const char *name){
    if(type || subtype!=16 || strcmp(name,"ota_0"))return NULL;
    return scenario==3?NULL:&baseline;
}
esp_err_t esp_ota_get_state_partition(const esp_partition_t *p,esp_ota_img_states_t *state){
    (void)p;*state=scenario==4?ESP_OTA_IMG_VALID:ESP_OTA_IMG_PENDING_VERIFY;return scenario==5?1:ESP_OK;
}
#include "../../../esp32_probe/main/probe_codec_diagnostic.c"
esp_err_t esp_ota_get_partition_description(const esp_partition_t *p,esp_app_desc_t *desc){
    (void)p;memset(desc,0,sizeof(*desc));strcpy(desc->project_name,"bt_audio_sink");strcpy(desc->version,"1.0.4-idf6.1");
    memcpy(desc->app_elf_sha256,baseline_elf_sha256,32);
    if(scenario==6)desc->project_name[0]='x';if(scenario==7)desc->version[0]='x';if(scenario==8)desc->app_elf_sha256[0]^=1;
    return scenario==9?1:ESP_OK;
}
esp_err_t esp_ota_set_boot_partition(const esp_partition_t *p){
    if(p!=&baseline)return 2;++selects;return scenario==10?1:ESP_OK;
}
bool probe_codec_check(TaskHandle_t owner){if(owner!=(TaskHandle_t)1)return false;++checks;return scenario!=1;}
TaskHandle_t xTaskGetCurrentTaskHandle(void){return (TaskHandle_t)1;}
int xPortGetCoreID(void){return 0;}
void vTaskDelay(unsigned ticks){delay_ms+=ticks;}
_Noreturn void esp_restart(void){++restarts;longjmp(restart_target,1);}
static unsigned created_core,created_stack,creates;
int xTaskCreatePinnedToCore(void (*work)(void *),const char *name,unsigned stack,void *arg,unsigned priority,TaskHandle_t *task,int core){
    (void)name;(void)arg;(void)priority;(void)task;if(work!=diagnostic)return 0;
    ++creates;created_core=(unsigned)core;created_stack=stack;return scenario==11?0:pdPASS;
}
#define REQUIRE(x) do {if(!(x))return __LINE__;}while(0)
#ifdef _WIN32
__declspec(dllexport)
#endif
int codec_diagnostic_return_tests(void){
    for(scenario=0;scenario<=10;++scenario){
        checks=selects=restarts=delay_ms=0;
        if(!setjmp(restart_target))diagnostic(NULL);
        bool admitted=scenario<=1 || scenario==10;
        REQUIRE(checks==(unsigned)admitted && selects==(unsigned)admitted);
        REQUIRE(restarts==1 && delay_ms==250);
    }
    unsigned address=running.address;running.address=0x110000;scenario=0;checks=selects=0;
    if(!setjmp(restart_target))diagnostic(NULL);
    REQUIRE(!checks && !selects);running.address=address;
    creates=0;scenario=0;app_main();REQUIRE(creates==1 && created_core==0 && created_stack==4096);
    scenario=11;app_main();REQUIRE(creates==2);
    return 0;
}
