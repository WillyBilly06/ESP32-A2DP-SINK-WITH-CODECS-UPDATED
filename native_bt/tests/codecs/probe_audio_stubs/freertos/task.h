#include "FreeRTOS.h"
void xTaskNotifyGive(TaskHandle_t);void taskYIELD(void);unsigned ulTaskNotifyTake(int,unsigned);void vTaskDelay(unsigned);void vTaskDelete(TaskHandle_t);unsigned uxTaskGetStackHighWaterMark(TaskHandle_t);int xTaskCreatePinnedToCore(void (*)(void *),const char *,unsigned,void *,unsigned,TaskHandle_t *,int);

void vTaskNotifyGiveFromISR(TaskHandle_t,BaseType_t *);
