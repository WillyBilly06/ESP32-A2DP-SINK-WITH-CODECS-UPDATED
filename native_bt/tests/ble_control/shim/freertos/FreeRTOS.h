#ifndef BLE_TEST_FREERTOS_H
#define BLE_TEST_FREERTOS_H
#include <stdint.h>
typedef void *TaskHandle_t;
typedef void *QueueHandle_t;
typedef unsigned UBaseType_t;
typedef int BaseType_t;
typedef uint32_t TickType_t;
#define pdTRUE 1
#define pdPASS 1
#define portMAX_DELAY UINT32_MAX
#endif

#ifndef DBT_BLE_TEST_TICK_HZ
#define DBT_BLE_TEST_TICK_HZ 1000u
#endif
#define pdMS_TO_TICKS(n) ((TickType_t)((uint64_t)(n)*DBT_BLE_TEST_TICK_HZ/1000u))
