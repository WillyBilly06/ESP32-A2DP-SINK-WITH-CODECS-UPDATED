#include "probe_owner_pacer.h"
#include "esp_timer.h"
#include <stdatomic.h>
_Static_assert(ATOMIC_INT_LOCK_FREE==2,"Owner release requires lock-free words");
static struct {
    esp_timer_handle_t timer;TaskHandle_t owner;
    uint32_t slice_at,pause_at;bool attempted,started;
    _Atomic uint32_t released,enabled,waiting,failed,pauses,releases,wait_checks,max_slice_us,max_pause_us,arm_failures;
} pacer;
static void maximum(_Atomic uint32_t *cell,uint32_t n){if(n>atomic_load(cell))atomic_store(cell,n);}
static void increment(_Atomic uint32_t *cell){atomic_store(cell,atomic_load(cell)+1u);}
static void release_owner(void *unused){
    (void)unused;
    atomic_store_explicit(&pacer.released,1,memory_order_release);
    if(pacer.owner)xTaskNotifyGive(pacer.owner);
}
esp_err_t probe_owner_pacer_init(TaskHandle_t owner){
    if(pacer.attempted)return ESP_ERR_INVALID_STATE;
    pacer.attempted=true;if(!owner)return ESP_ERR_INVALID_ARG;
    pacer.owner=owner;
    esp_timer_create_args_t args={.callback=release_owner,.name="host_budget"};
    esp_err_t result=esp_timer_create(&args,&pacer.timer);
    if(result==ESP_OK)atomic_store_explicit(&pacer.enabled,1,memory_order_release);
    return result;
}
bool probe_owner_pacer_ready(void){
    if(!atomic_load_explicit(&pacer.enabled,memory_order_acquire))return false;
    uint32_t now=(uint32_t)esp_timer_get_time();
    if(atomic_load(&pacer.waiting)){
        if(!atomic_load_explicit(&pacer.released,memory_order_acquire)){increment(&pacer.wait_checks);return false;}
        atomic_store(&pacer.waiting,0);increment(&pacer.releases);
        maximum(&pacer.max_pause_us,now-pacer.pause_at);pacer.started=false;
    }
    if(!pacer.started){pacer.slice_at=now;pacer.started=true;}
    return true;
}
bool probe_owner_pacer_charge(bool immediate_work,bool pressure){
    if(!atomic_load(&pacer.enabled) || atomic_load(&pacer.failed))return false;
    if(atomic_load(&pacer.waiting))return true;
    if(!pressure){pacer.started=false;return false;}
    if(!immediate_work)return false; /* natural sleep may still be very short */
    uint32_t now=(uint32_t)esp_timer_get_time();
    if(!pacer.started){pacer.slice_at=now;pacer.started=true;return false;}
    uint32_t spent=now-pacer.slice_at;maximum(&pacer.max_slice_us,spent);
    if(spent<PROBE_OWNER_SLICE_US)return false;
    pacer.pause_at=now;atomic_store_explicit(&pacer.released,0,memory_order_relaxed);
    atomic_store(&pacer.waiting,1);
    esp_err_t result=esp_timer_start_once(pacer.timer,PROBE_OWNER_RELEASE_US);
    if(result!=ESP_OK){
        /* Preserve protocol progress on a scheduler-resource error. Explicitly
         * invalidate this pacing trial and continue the existing owned host;
         * no retry loop, dropped reply or alternate Bluetooth host. */
        atomic_store(&pacer.waiting,0);atomic_store(&pacer.failed,1);increment(&pacer.arm_failures);pacer.started=false;return false;
    }
    increment(&pacer.pauses);return true;
}
void probe_owner_pacer_waited(uint32_t elapsed){
    if(elapsed>=PROBE_OWNER_RELEASE_US && !atomic_load(&pacer.waiting))pacer.started=false;
}
void probe_owner_pacer_snapshot(probe_owner_pacer_stats_t *out){
    if(!out)return;
#define PACER_READ(field) out->field=atomic_load_explicit(&pacer.field,memory_order_relaxed)
    PACER_READ(enabled);PACER_READ(waiting);PACER_READ(failed);PACER_READ(pauses);PACER_READ(releases);
    PACER_READ(wait_checks);PACER_READ(max_slice_us);PACER_READ(max_pause_us);PACER_READ(arm_failures);
#undef PACER_READ
}
