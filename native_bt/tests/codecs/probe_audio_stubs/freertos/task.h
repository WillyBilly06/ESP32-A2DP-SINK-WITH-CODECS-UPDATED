#include "FreeRTOS.h"
TaskHandle_t xTaskGetCurrentTaskHandle(void);
BaseType_t xTaskGetSchedulerState(void);
#define taskSCHEDULER_SUSPENDED 0
#define taskSCHEDULER_NOT_STARTED 1
#define taskSCHEDULER_RUNNING 2
void xTaskNotifyGive(TaskHandle_t);void taskYIELD(void);unsigned ulTaskNotifyTake(int,unsigned);void vTaskDelay(unsigned);void vTaskDelete(TaskHandle_t);unsigned uxTaskGetStackHighWaterMark(TaskHandle_t);int xTaskCreatePinnedToCore(void (*)(void *),const char *,unsigned,void *,unsigned,TaskHandle_t *,int);

void vTaskNotifyGiveFromISR(TaskHandle_t,BaseType_t *);
