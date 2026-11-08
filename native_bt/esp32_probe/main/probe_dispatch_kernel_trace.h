#ifndef PROBE_DISPATCH_KERNEL_TRACE_H
#define PROBE_DISPATCH_KERNEL_TRACE_H
#include "sdkconfig.h"
#if CONFIG_FREERTOS_SMP || CONFIG_FREERTOS_NUMBER_OF_CORES != 2 || CONFIG_ESP_TRACE_ENABLE || CONFIG_PM_ENABLE || CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ != 240
#error Dispatch trace requires the pinned fixed240MHz dual-core kernel without another trace backend
#endif
#if defined(traceTASK_SWITCHED_IN) || defined(traceTASK_SWITCHED_OUT)
#error Another task-switch trace is installed
#endif
void probe_dispatch_switch(unsigned,void *);
/* Standard trace expansion supplies the selected TCB identity; no TCB field
 * is read and no kernel API is called by this hook. */
#define traceTASK_SWITCHED_IN() probe_dispatch_switch((unsigned)portGET_CORE_ID(),(void *)pxCurrentTCBs[portGET_CORE_ID()])
#endif
