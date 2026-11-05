#pragma once
#include "FreeRTOS.h"
TaskHandle_t xTaskGetCurrentTaskHandle(void);
TaskHandle_t xTaskGetHandle(const char *);
TaskHandle_t xTaskGetIdleTaskHandleForCore(unsigned);
