#include "FreeRTOS.h"
TaskHandle_t xTaskGetCurrentTaskHandle(void);
BaseType_t xPortGetCoreID(void);
BaseType_t xPortInIsrContext(void);
unsigned uxTaskGetStackHighWaterMark(TaskHandle_t);
