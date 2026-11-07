#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "dbt_codec.h"
#include "dbt_esp32_affinity.h"
#include "probe_codec_check.h"
static void (*pending)(void *);
static unsigned creates,callbacks,notifications,waits,fail_create,timeout,spurious;
static bool callback_passed;
static int64_t clock_us;
int64_t esp_timer_get_time(void){clock_us+=timeout?60000001:1;return clock_us;}
void *heap_caps_aligned_alloc(size_t align,size_t bytes,unsigned caps){(void)align;(void)caps;return malloc(bytes);}
void heap_caps_free(void *p){free(p);}
size_t heap_caps_get_free_size(unsigned caps){(void)caps;return 100000;}
unsigned uxTaskGetStackHighWaterMark(TaskHandle_t task){(void)task;return 4096;}
void vTaskDelete(TaskHandle_t task){(void)task;}
TaskHandle_t xTaskGetCurrentTaskHandle(void){return (TaskHandle_t)1;}
void xTaskNotifyGive(TaskHandle_t task){if(task==(TaskHandle_t)1)++notifications;}
unsigned ulTaskNotifyTake(int clear,unsigned ticks){
    (void)clear;if(!ticks)abort();++waits;
    if(spurious){--spurious;return 1;}
    if(pending){void (*work)(void *)=pending;pending=NULL;work(NULL);}
    return 1;
}
int xTaskCreatePinnedToCore(void (*work)(void *),const char *name,unsigned stack,void *arg,unsigned priority,TaskHandle_t *task,int core){
    (void)name;(void)arg;(void)priority;(void)task;if(stack!=8192 || core!=DBT_ESP32_DECODER_CORE)return 0;
    ++creates;if(fail_create)return 0;pending=work;return pdPASS;
}
dbt_result_t dbt_codec_init(dbt_codec_decoder_t *decoder,dbt_codec_hooks_t hooks){memset(decoder,0,sizeof(*decoder));decoder->hooks=hooks;return DBT_OK;}
dbt_result_t dbt_codec_open(dbt_codec_decoder_t *decoder,const dbt_codec_config_t *config){(void)decoder;(void)config;return DBT_INVALID;}
void dbt_codec_close(dbt_codec_decoder_t *decoder){decoder->allocated_bytes=0;}
#include "../../../esp32_probe/main/probe_codec_check.c"
static void completed(void *context,bool passed){if(context!=(void *)1)abort();++callbacks;callback_passed=passed;}
#define REQUIRE(x) do {if(!(x))return __LINE__;}while(0)
#ifdef _WIN32
__declspec(dllexport)
#endif
int codec_diagnostic_worker_tests(void){
    REQUIRE(!probe_codec_check(NULL) && !creates);
    REQUIRE(!probe_codec_check((TaskHandle_t)2) && !creates);
    fail_create=1;REQUIRE(!probe_codec_check_start(completed,(void *)1) && !callbacks);
    fail_create=0;REQUIRE(probe_codec_check_start(completed,(void *)1));
    REQUIRE(!probe_codec_check_start(completed,(void *)1));
    void (*work)(void *)=pending;pending=NULL;work(NULL);
    /* No selected codec is an explicit failure, still completed exactly once. */
    REQUIRE(callbacks==1 && !callback_passed && !atomic_load(&check.running));
    spurious=2;waits=notifications=0;
    REQUIRE(!probe_codec_check((TaskHandle_t)1));
    REQUIRE(waits==3 && notifications==1 && !atomic_load(&check.running));
    timeout=1;REQUIRE(!probe_codec_check((TaskHandle_t)1));
    REQUIRE(atomic_load(&check.running) && pending!=NULL);
    REQUIRE(!probe_codec_check_start(completed,(void *)1));
    work=pending;pending=NULL;work(NULL);
    REQUIRE(!atomic_load(&check.running) && notifications==2);
    return 0;
}
