#ifndef PROBE_TEST_QUEUE_H
#define PROBE_TEST_QUEUE_H
#include "FreeRTOS.h"
typedef void *QueueHandle_t;
QueueHandle_t xQueueCreate(UBaseType_t,UBaseType_t);
BaseType_t xQueueReceive(QueueHandle_t,void *,TickType_t);
BaseType_t xQueueSend(QueueHandle_t,const void *,TickType_t);
UBaseType_t uxQueueMessagesWaiting(QueueHandle_t);
void vQueueDelete(QueueHandle_t);
#endif
