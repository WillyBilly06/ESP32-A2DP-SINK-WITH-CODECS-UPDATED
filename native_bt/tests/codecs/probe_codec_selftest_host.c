/* Synchronous host harness for self-test logic; stack/time values are stubs. */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdlib.h>
#ifdef _WIN32
#include <malloc.h>
#endif
int g_nzc_formula=0;
int g_pkframe=0;
static int64_t ticks;
static bool fail_next;
static unsigned low_water=4096;
int64_t esp_timer_get_time(void){return ++ticks;}
void *heap_caps_aligned_alloc(size_t alignment,size_t bytes,unsigned caps){
    (void)caps;
    if(fail_next){fail_next=false;return 0;}
#ifdef _WIN32
    return _aligned_malloc(bytes,alignment);
#else
    void *p=0;return posix_memalign(&p,alignment,bytes)?0:p;
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
unsigned uxTaskGetStackHighWaterMark(TaskHandle_t task){(void)task;return low_water;}
void vTaskDelay(unsigned ticks_to_wait){(void)ticks_to_wait;}
void vTaskDelete(TaskHandle_t task){(void)task;}
int xTaskCreatePinnedToCore(void (*work)(void *),const char *name,unsigned stack,void *arg,unsigned priority,TaskHandle_t *task,int core){
    (void)name;(void)priority;(void)task;(void)core;if(stack!=8192)return 0;work(arg);return pdPASS;
}
#include "../../esp32_probe/main/probe_codec_check.c"
int main(int argc,char **argv){
    (void)argv;
    if(argc>1){fail_next=true;return probe_codec_check(0)?1:0;}
    if(!probe_codec_check(0))return 1;
    const probe_codec_vector_t *exact=0,*bounded=0;
    for(unsigned i=0;i<sizeof(probe_codec_vectors)/sizeof(probe_codec_vectors[0]);++i){
        const probe_codec_vector_t *v=&probe_codec_vectors[i];if(!v->name)continue;
        if(v->exact && !exact)exact=v;
        if(!v->exact && !bounded)bounded=v;
    }
    if(!exact || !bounded || bounded->point_count<4)return 2;
    probe_codec_vector_t changed=*bounded;probe_pcm_point_t points[4];
    for(unsigned i=0;i<4;++i)points[i]=bounded->points[i];
    points[0].value=1000000;changed.points=points;changed.point_count=4;
    if(run_vector(&changed) || check.live)return 3;
    changed=*exact;changed.expected_hash^=1;
    if(run_vector(&changed) || check.live)return 4;
    low_water=512;
    if(run_vector(exact) || check.live)return 5;
    low_water=4096;fail_next=true;
    if(run_vector(exact) || check.live)return 6;
    return 0;
}
