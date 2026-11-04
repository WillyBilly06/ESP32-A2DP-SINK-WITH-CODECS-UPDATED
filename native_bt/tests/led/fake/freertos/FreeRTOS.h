#ifndef FAKE_FREERTOS
#define FAKE_FREERTOS
#include <stddef.h>
typedef void *TaskHandle_t;typedef void *SemaphoreHandle_t;
#define pdTRUE 1
#define pdPASS 1
#define portMAX_DELAY 0xffffffffu
#define pdMS_TO_TICKS(n) (n)
int xTaskCreate(void (*f)(void*),const char*,unsigned,void*,unsigned,TaskHandle_t*);
unsigned ulTaskNotifyTake(int,unsigned);void xTaskNotifyGive(TaskHandle_t);void vTaskDelete(TaskHandle_t);void vTaskSuspend(TaskHandle_t);void vTaskDelay(unsigned);
SemaphoreHandle_t xSemaphoreCreateBinary(void);int xSemaphoreGive(SemaphoreHandle_t);int xSemaphoreTake(SemaphoreHandle_t,unsigned);void vSemaphoreDelete(SemaphoreHandle_t);
#endif
