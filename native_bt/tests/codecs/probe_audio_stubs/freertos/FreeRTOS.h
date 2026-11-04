#ifndef FAKE_FREERTOS_H
#define FAKE_FREERTOS_H
#include <stdint.h>
typedef void *TaskHandle_t;typedef unsigned UBaseType_t;typedef int BaseType_t;
typedef uint32_t TickType_t;
#define pdTRUE 1
#define pdFALSE 0
#define pdPASS 1
#ifndef DBT_TEST_TICK_HZ
#define DBT_TEST_TICK_HZ 1000
#endif
#define pdMS_TO_TICKS(x) ((TickType_t)(((uint64_t)(x)*DBT_TEST_TICK_HZ)/1000u))
#endif
