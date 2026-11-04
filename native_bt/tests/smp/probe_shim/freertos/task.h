#ifndef PROBE_TEST_TASK_H
#define PROBE_TEST_TASK_H
#include "FreeRTOS.h"
typedef void *TaskHandle_t;
BaseType_t xTaskCreate(void (*fn)(void *),const char *,uint32_t,void *,UBaseType_t,TaskHandle_t *);
void xTaskNotifyGive(TaskHandle_t);
void vTaskDelete(TaskHandle_t);
#endif
