#pragma once
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
typedef void (*esp_freertos_tick_cb_t)(void);
esp_err_t esp_register_freertos_tick_hook_for_cpu(esp_freertos_tick_cb_t,unsigned);
void esp_deregister_freertos_tick_hook_for_cpu(esp_freertos_tick_cb_t,unsigned);
