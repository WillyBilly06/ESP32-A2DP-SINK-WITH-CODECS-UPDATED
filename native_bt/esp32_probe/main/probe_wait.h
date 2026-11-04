#ifndef PROBE_WAIT_H
#define PROBE_WAIT_H
#include <stdatomic.h>
#include <stdint.h>
#include <stdbool.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Startup owner only. The producer publishes the flag then notifies the owner.
 * A notification arriving between the check and wait stays pending. Unrelated
 * notifications recheck the condition without extending the absolute deadline.
 * Runtime protocol handlers must return to their owner loop instead of waiting. */
static inline bool probe_wait_flag(_Atomic bool *ready,int64_t deadline_us){
    while(!atomic_load_explicit(ready,memory_order_acquire)){
        int64_t remaining=deadline_us-esp_timer_get_time();
        if(remaining<=0)return false;
        uint32_t ms=(uint32_t)((remaining+999)/1000);
        TickType_t ticks=pdMS_TO_TICKS(ms);
        (void)ulTaskNotifyTake(pdTRUE,ticks?ticks:1);
    }
    return true;
}
#endif
