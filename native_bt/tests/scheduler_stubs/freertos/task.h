#pragma once
#include "FreeRTOS.h"
#include <stdint.h>
#ifndef pdFALSE
#define pdFALSE 0
#endif
TaskHandle_t xTaskGetCurrentTaskHandle(void);
int xPortInIsrContext(void);
TaskHandle_t xTaskGetHandle(const char *);
TaskHandle_t xTaskGetIdleTaskHandleForCore(unsigned);

#ifdef DBT_PROBE_TASK_RUNTIME_STATS
#define configGENERATE_RUN_TIME_STATS 1
#define configUSE_TRACE_FACILITY 1
#define configTASKLIST_INCLUDE_COREID 1
#define CONFIG_FREERTOS_RUN_TIME_STATS_USING_ESP_TIMER 1
#define configRUN_TIME_COUNTER_TYPE uint32_t
typedef enum {eRunning,eReady,eBlocked,eSuspended,eDeleted,eInvalid} eTaskState;
typedef struct {TaskHandle_t xHandle;const char *pcTaskName;uint32_t xTaskNumber,uxCurrentPriority,ulRunTimeCounter;int32_t xCoreID;} TaskStatus_t;
void vTaskGetInfo(TaskHandle_t,TaskStatus_t *,int,eTaskState);
#endif
