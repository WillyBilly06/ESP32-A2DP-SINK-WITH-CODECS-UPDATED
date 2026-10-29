#ifndef FAKE_FREERTOS_H
#define FAKE_FREERTOS_H
#include <stdint.h>
typedef void *TaskHandle_t;typedef unsigned UBaseType_t;typedef int BaseType_t;
#define pdTRUE 1
#define pdFALSE 0
#define pdPASS 1
#define pdMS_TO_TICKS(x) (x)
#endif
