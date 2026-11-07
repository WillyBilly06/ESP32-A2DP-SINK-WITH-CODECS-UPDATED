#include <stdio.h>
#include <string.h>
#include "../../esp32_probe/main/probe_owner_pacer.c"
#define CHECK(x) do{if(!(x)){printf("FAIL owner budget line %u\n",(unsigned)__LINE__);return 1;}}while(0)
static uint32_t clock_us,created,armed,notifications;static esp_err_t create_result,arm_result;
static struct fake_timer {esp_timer_create_args_t args;bool active;} timer;
int64_t esp_timer_get_time(void){return clock_us;}
void xTaskNotifyGive(TaskHandle_t owner){if(owner==(void *)1)++notifications;}
esp_err_t esp_timer_create(const esp_timer_create_args_t *args,esp_timer_handle_t *handle){
    ++created;if(create_result)return create_result;timer.args=*args;*handle=&timer;return ESP_OK;
}
esp_err_t esp_timer_start_once(esp_timer_handle_t handle,uint64_t us){
    ++armed;if(handle!=&timer || us!=500u || timer.active)return ESP_ERR_INVALID_STATE;
    if(arm_result)return arm_result;timer.active=true;return ESP_OK;
}
static void fixture(void){memset(&pacer,0,sizeof(pacer));memset(&timer,0,sizeof(timer));clock_us=created=armed=notifications=0;create_result=arm_result=ESP_OK;}
static void fire(void){timer.active=false;timer.args.callback(timer.args.arg);}
int main(void){
    probe_owner_pacer_stats_t s;
    fixture();CHECK(!probe_owner_pacer_ready() && !probe_owner_pacer_charge(true,true));
    CHECK(probe_owner_pacer_init(NULL)==ESP_ERR_INVALID_ARG && !created);
    CHECK(probe_owner_pacer_init((void *)1)==ESP_ERR_INVALID_STATE);
    fixture();create_result=ESP_ERR_NO_MEM;CHECK(probe_owner_pacer_init((void *)1)==ESP_ERR_NO_MEM && created==1);
    CHECK(!probe_owner_pacer_ready() && probe_owner_pacer_init((void *)1)==ESP_ERR_INVALID_STATE);
    fixture();CHECK(probe_owner_pacer_init((void *)1)==ESP_OK && created==1 && !armed);
    CHECK(probe_owner_pacer_ready());clock_us=10000;CHECK(!probe_owner_pacer_charge(false,true) && !armed);
    CHECK(probe_owner_pacer_ready());clock_us=100000;CHECK(!probe_owner_pacer_charge(true,false) && !armed);
    CHECK(probe_owner_pacer_ready());clock_us+=999;CHECK(!probe_owner_pacer_charge(true,true));
    ++clock_us;CHECK(probe_owner_pacer_charge(true,true) && timer.active && armed==1);
    for(unsigned i=0;i<8;++i)CHECK(!probe_owner_pacer_ready());
    CHECK(probe_owner_pacer_charge(true,false) && armed==1); /* one outstanding timer */
    clock_us+=500;fire();CHECK(notifications==1);
    /* Callback before notification wait is not lost. */
    CHECK(probe_owner_pacer_ready());probe_owner_pacer_snapshot(&s);
    CHECK(s.enabled && !s.waiting && !s.failed && s.pauses==1 && s.releases==1 && s.wait_checks==8 && s.max_slice_us==1000 && s.max_pause_us==500);
    clock_us+=999;CHECK(!probe_owner_pacer_charge(true,true));
    probe_owner_pacer_waited(800);CHECK(probe_owner_pacer_ready());clock_us+=1;CHECK(!probe_owner_pacer_charge(true,true));
    /* Natural sub-500us notifications do not reset the burst budget. */
    probe_owner_pacer_waited(100);clock_us+=999;CHECK(probe_owner_pacer_charge(true,true) && armed==2);
    clock_us+=800;fire();CHECK(probe_owner_pacer_ready());probe_owner_pacer_snapshot(&s);
    CHECK(s.max_pause_us==800 && s.releases==2);
    /* Short real-event waits retain a burst's elapsed budget; idle never
     * schedules its own wake. A genuine >=500us idle resets it above. */
    fixture();CHECK(probe_owner_pacer_init((void *)1)==ESP_OK && probe_owner_pacer_ready());
    clock_us=400;CHECK(!probe_owner_pacer_charge(false,true));probe_owner_pacer_waited(100);
    clock_us=700;CHECK(probe_owner_pacer_ready() && !probe_owner_pacer_charge(false,true));probe_owner_pacer_waited(100);
    clock_us=1000;CHECK(probe_owner_pacer_ready() && probe_owner_pacer_charge(true,true) && armed==1);
    fixture();CHECK(probe_owner_pacer_init((void *)1)==ESP_OK);clock_us=UINT32_MAX-700u;CHECK(probe_owner_pacer_ready());
    clock_us=399;CHECK(probe_owner_pacer_charge(true,true));clock_us=899;fire();CHECK(probe_owner_pacer_ready());
    probe_owner_pacer_snapshot(&s);CHECK(s.max_slice_us==1100 && s.max_pause_us==500);
    /* Arm failure is explicit and preserves host protocol progress, with no
     * allocation/arm retries, alternate stack, or permanently blocked owner. */
    fixture();CHECK(probe_owner_pacer_init((void *)1)==ESP_OK && probe_owner_pacer_ready());
    clock_us=1000;arm_result=ESP_ERR_INVALID_STATE;CHECK(!probe_owner_pacer_charge(true,true));
    CHECK(probe_owner_pacer_ready());clock_us=5000;CHECK(!probe_owner_pacer_charge(true,true) && armed==1);
    probe_owner_pacer_snapshot(&s);CHECK(s.failed && s.arm_failures==1 && !s.waiting && !s.pauses);
    probe_owner_pacer_snapshot(NULL);
    puts("PASS owner budget: real-work/pressure-only one-shot, idle never arms, no polling/waits in API, early notifications, pre-wait release, natural idle, time wrap, explicit create/arm failure and no retry");return 0;
}
