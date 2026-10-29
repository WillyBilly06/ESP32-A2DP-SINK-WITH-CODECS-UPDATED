#include "FreeRTOS.h"
typedef void *QueueHandle_t;QueueHandle_t xQueueCreate(unsigned,unsigned);int xQueueSend(QueueHandle_t,const void *,unsigned);int xQueueReceive(QueueHandle_t,void *,unsigned);
