#ifndef PROBE_TASK_ACCOUNTING_TRACE_H
#define PROBE_TASK_ACCOUNTING_TRACE_H
/* Forced into the kernel's C units only by the opt-in project configuration.
 * The standard trace macros remain absent from ordinary builds. */
#include "sdkconfig.h"
#if CONFIG_FREERTOS_SMP || CONFIG_FREERTOS_NUMBER_OF_CORES != 2 || CONFIG_ESP_TRACE_ENABLE
#error Task-cycle trace targets the pinned dual-core IDF kernel without another trace backend
#endif
#if CONFIG_PM_ENABLE || CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ != 240
#error Task-cycle trace requires fixed 240 MHz
#endif
void probe_task_cycles_in(unsigned,void *);
void probe_task_cycles_out(unsigned,void *);
#if defined(traceTASK_SWITCHED_IN) || defined(traceTASK_SWITCHED_OUT)
#error Another task-switch trace is already installed
#endif
/* pxCurrentTCBs and portGET_CORE_ID are provided by this pinned IDF kernel at
 * the documented trace expansion points. No private TCB field is inspected. */
#define traceTASK_SWITCHED_IN() probe_task_cycles_in((unsigned)portGET_CORE_ID(),(void *)pxCurrentTCBs[portGET_CORE_ID()])
#define traceTASK_SWITCHED_OUT() probe_task_cycles_out((unsigned)portGET_CORE_ID(),(void *)pxCurrentTCBs[portGET_CORE_ID()])
#endif
