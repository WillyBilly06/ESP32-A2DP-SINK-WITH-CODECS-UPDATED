#include "probe_reset_dispatch.h"
#include "esp_timer.h"
#include "esp_system.h"
#include <stdatomic.h>
#if !CONFIG_ESP_TIMER_TASK_AFFINITY_CPU0
#error "The restart timer must dispatch on core 0"
#endif
enum {UNINITIALIZED,READY,PENDING,FAILED,FIRED};
static _Atomic unsigned state;
static esp_timer_handle_t timer;
static TaskHandle_t owner;
static void restart_timer(void *unused){
    (void)unused;
    if(atomic_load_explicit(&state,memory_order_acquire)!=PENDING)return;
    if(xPortGetCoreID()!=0){
        atomic_store_explicit(&state,FAILED,memory_order_release);
        if(owner)xTaskNotifyGive(owner);
        return;
    }
    atomic_store_explicit(&state,FIRED,memory_order_release);
    esp_restart();
}
bool probe_reset_dispatch_init(TaskHandle_t task){
    if(!task || task!=xTaskGetCurrentTaskHandle())return false;
    unsigned current=atomic_load_explicit(&state,memory_order_acquire);
    if(current!=UNINITIALIZED)return current==READY && task==owner;
    owner=task;
    esp_timer_create_args_t args={.callback=restart_timer,.dispatch_method=ESP_TIMER_TASK,.name="owned_restart"};
    if(esp_timer_create(&args,&timer)!=ESP_OK){atomic_store(&state,FAILED);return false;}
    atomic_store_explicit(&state,READY,memory_order_release);return true;
}
bool probe_reset_dispatch_request(void){
    if(!owner || xTaskGetCurrentTaskHandle()!=owner)return false;
    unsigned expected=READY;
    if(!atomic_compare_exchange_strong_explicit(&state,&expected,PENDING,memory_order_acq_rel,memory_order_acquire))return false;
    /* A finite completion/log-drain interval, after the existing restart
     * barrier has approved. The callback uses the existing core-0 timer task. */
    if(esp_timer_start_once(timer,250000)!=ESP_OK){atomic_store_explicit(&state,FAILED,memory_order_release);return false;}
    return true;
}
bool probe_reset_dispatch_pending(void){unsigned s=atomic_load_explicit(&state,memory_order_acquire);return s==PENDING || s==FIRED;}
bool probe_reset_dispatch_failed(void){return atomic_load_explicit(&state,memory_order_acquire)==FAILED;}
