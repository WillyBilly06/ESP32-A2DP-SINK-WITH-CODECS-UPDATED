#include "FreeRTOS.h"
TaskHandle_t xTaskGetCurrentTaskHandle(void);
BaseType_t xTaskCreate(void (*)(void *),const char *,uint32_t,void *,UBaseType_t,TaskHandle_t *);
void xTaskNotifyGive(TaskHandle_t);
void vTaskDelete(TaskHandle_t);
void taskYIELD(void);
