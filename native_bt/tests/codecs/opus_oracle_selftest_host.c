/* Actual probe check with only Opus selected; FreeRTOS/timing are host stubs. */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#ifdef _WIN32
#include <malloc.h>
#endif
int64_t esp_timer_get_time(void){static int64_t tick;return ++tick;}
void *heap_caps_aligned_alloc(size_t alignment,size_t size,unsigned caps){
    (void)caps;
#ifdef _WIN32
    return _aligned_malloc(size,alignment);
#else
    void *p=0;return posix_memalign(&p,alignment,size)?0:p;
#endif
}
void heap_caps_free(void *p){
#ifdef _WIN32
    _aligned_free(p);
#else
    free(p);
#endif
}
size_t heap_caps_get_free_size(unsigned caps){(void)caps;return 1000000;}
unsigned uxTaskGetStackHighWaterMark(TaskHandle_t task){(void)task;return 4096;}
void vTaskDelay(unsigned ticks){(void)ticks;}
void vTaskDelete(TaskHandle_t task){(void)task;}
int xTaskCreatePinnedToCore(void (*fn)(void *),const char *name,unsigned stack,void *arg,unsigned priority,TaskHandle_t *task,int core){
    (void)name;(void)priority;(void)task;(void)core;if(stack!=8192)return 0;fn(arg);return pdPASS;
}
#include "../../esp32_probe/main/probe_codec_check.c"
int main(void){return probe_codec_check(0)?0:1;}
