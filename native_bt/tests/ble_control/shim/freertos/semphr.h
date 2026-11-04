#ifndef BLE_SHIM_SEMPHR_H
#define BLE_SHIM_SEMPHR_H
#include "queue.h"
typedef QueueHandle_t SemaphoreHandle_t;
static inline SemaphoreHandle_t xSemaphoreCreateBinary(void){return xQueueCreate(1,1);}
static inline BaseType_t xSemaphoreGive(SemaphoreHandle_t h){uint8_t value=1;return xQueueSend(h,&value,0);}
static inline BaseType_t xSemaphoreTake(SemaphoreHandle_t h,TickType_t t){uint8_t value;return xQueueReceive(h,&value,t);}
static inline void vSemaphoreDelete(SemaphoreHandle_t h){vQueueDelete(h);}
#endif
