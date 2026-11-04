#include "probe_pair_button.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include <stdatomic.h>
#if ATOMIC_BOOL_LOCK_FREE != 2
#error "The GPIO ISR requires lock-free boolean publication"
#endif

enum { WAIT_RELEASE, ARMED, HOLDING, LATCHED };
static TaskHandle_t owner;
static _Atomic bool changed;
static bool initialized,settling;
static unsigned state;
static uint32_t stable_at,held_until;

static void IRAM_ATTR edge(void *unused) {
    (void)unused;
    atomic_store_explicit(&changed,true,memory_order_release);
    BaseType_t wake=pdFALSE;
    vTaskNotifyGiveFromISR(owner,&wake);
    if(wake)portYIELD_FROM_ISR();
}
bool probe_pair_button_init(TaskHandle_t task) {
    if(!task || task!=xTaskGetCurrentTaskHandle())return false;
    if(initialized)return owner==task;
    gpio_config_t config={.pin_bit_mask=UINT64_C(1),.mode=GPIO_MODE_INPUT,
        .pull_up_en=GPIO_PULLUP_ENABLE,.pull_down_en=GPIO_PULLDOWN_DISABLE,.intr_type=GPIO_INTR_ANYEDGE};
    if(gpio_config(&config)!=ESP_OK)return false;
    esp_err_t result=gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if(result!=ESP_OK && result!=ESP_ERR_INVALID_STATE)return false;
    owner=task;
    if(gpio_isr_handler_add(GPIO_NUM_0,edge,NULL)!=ESP_OK){owner=NULL;return false;}
    state=WAIT_RELEASE;settling=false;initialized=true;
    atomic_store_explicit(&changed,true,memory_order_release);
    return true;
}
static uint32_t remaining(uint32_t now,uint32_t deadline) {
    return (int32_t)(now-deadline)>=0?0:deadline-now;
}
bool probe_pair_button_poll(uint32_t now) {
    if(!initialized || xTaskGetCurrentTaskHandle()!=owner)return false;
    if(atomic_exchange_explicit(&changed,false,memory_order_acq_rel)){
        /* Any edge cancels an unfinished hold, including edges coalesced while
         * flash/cache work prevented the owner from running. */
        if(state==HOLDING)state=ARMED;
        settling=true;stable_at=now+30u;
    }
    if(settling && !remaining(now,stable_at)){
        settling=false;
        if(gpio_get_level(GPIO_NUM_0))state=ARMED;
        else if(state==ARMED){state=HOLDING;held_until=now+3000u;}
    }
    if(state==HOLDING && !settling && !remaining(now,held_until)){
        if(!gpio_get_level(GPIO_NUM_0)){state=LATCHED;return true;}
        state=WAIT_RELEASE;settling=true;stable_at=now+30u;
    }
    return false;
}
uint32_t probe_pair_button_wait_ms(uint32_t now) {
    if(!initialized || xTaskGetCurrentTaskHandle()!=owner)return UINT32_MAX;
    if(atomic_load_explicit(&changed,memory_order_acquire))return 0;
    if(settling)return remaining(now,stable_at);
    return state==HOLDING?remaining(now,held_until):UINT32_MAX;
}
