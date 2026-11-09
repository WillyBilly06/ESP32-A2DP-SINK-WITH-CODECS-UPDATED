#include "probe_coex.h"
#include "esp_coexist.h"
#include "esp_timer.h"
#include <limits.h>
/* Public API instead of host-only VSC0xfc82. esp_bt_vs.h explicitly excludes
 * that command in controller-only mode; do not enable its private init hook. */
_Static_assert(ESP_COEX_ST_TYPE_BT==2,"Check SDK coexistence type");
_Static_assert(ESP_COEX_BT_ST_A2DP_STREAMING==PROBE_COEX_STREAMING &&
    ESP_COEX_BT_ST_A2DP_PAUSED==PROBE_COEX_PAUSED,"Check SDK A2DP bits");
static uint8_t next_status(const probe_coex_t *s,bool streaming){
    if(!(s->known&PROBE_COEX_STREAMING) ||
       ((s->applied&PROBE_COEX_STREAMING)!=0)!=streaming)return PROBE_COEX_STREAMING;
    if(!(s->known&PROBE_COEX_PAUSED) || (s->applied&PROBE_COEX_PAUSED))return PROBE_COEX_PAUSED;
    return 0;
}
static void note(probe_coex_t *s,probe_coex_stage_t stage,int32_t result){
    atomic_store_explicit(&s->state_snapshot,(uint32_t)s->known|((uint32_t)s->applied<<8)|
        ((uint32_t)s->pending<<16)|((uint32_t)s->failed<<17),memory_order_release);
    if(s->note)s->note(s,stage,result);
}
void probe_coex_poll(probe_coex_t *s,bool ready,bool streaming){
    if(!s)return;
    if(s->pending){
        if((atomic_load_explicit(&s->work_state,memory_order_acquire)&255u)!=PROBE_COEX_WORK_DONE)return;
        s->pending=false;
        if(s->sdk_result!=ESP_OK){s->known&=(uint8_t)~s->submitted_status;s->failed=true;
            note(s,PROBE_COEX_FAILED,s->sdk_result);
        }else {
            if(s->submitted_op)s->applied|=s->submitted_status;
            else s->applied&=(uint8_t)~s->submitted_status;
            s->known|=s->submitted_status;
            if(s->submitted_status==PROBE_COEX_STREAMING)s->known&=(uint8_t)~PROBE_COEX_PAUSED;
            note(s,PROBE_COEX_COMPLETED,s->sdk_result);
        }
        atomic_store_explicit(&s->work_state,PROBE_COEX_WORK_IDLE,memory_order_release);
    }
    if(s->failed || !ready)return;
    uint8_t status=next_status(s,streaming);if(!status)return;
    if(s->token==UINT32_MAX){s->failed=true;note(s,PROBE_COEX_FAILED,ESP_ERR_INVALID_STATE);return;}
    ++s->token;s->submitted_status=status;s->submitted_op=(uint8_t)(status==PROBE_COEX_STREAMING && streaming);
    s->started_us=s->finished_us=0;s->pending=true;
    /* Publish the queued diagnostic BEFORE worker release: it cannot race the
     * worker's timestamp writes. PROBE_COEX_WORK_DONE diagnostics read after acquire above. */
    note(s,PROBE_COEX_QUEUED,INT32_MIN);
    atomic_store_explicit(&s->work_state,PROBE_COEX_WORK_READY,memory_order_release);
}
uint32_t probe_coex_wait_ms(const probe_coex_t *s,bool ready,bool streaming){
    if(!s)return UINT32_MAX;
    if(s->pending)return (atomic_load_explicit(&s->work_state,memory_order_acquire)&255u)==PROBE_COEX_WORK_DONE?0:UINT32_MAX;
    return !s->failed && ready && next_status(s,streaming)?0:UINT32_MAX;
}
bool probe_coex_work_pending(const probe_coex_t *s){
    return s && atomic_load_explicit(&s->work_state,memory_order_acquire)!=PROBE_COEX_WORK_IDLE;
}
bool probe_coex_worker_step(probe_coex_t *s,unsigned core){
    if(!s)return false;
    uint32_t expected=PROBE_COEX_WORK_READY;
    if(!atomic_compare_exchange_strong_explicit(&s->work_state,&expected,PROBE_COEX_WORK_RUNNING,memory_order_acq_rel,memory_order_acquire))return false;
    s->started_us=(uint32_t)esp_timer_get_time();
    s->sdk_result=s->submitted_op?esp_coex_status_bit_set(ESP_COEX_ST_TYPE_BT,s->submitted_status):
        esp_coex_status_bit_clear(ESP_COEX_ST_TYPE_BT,s->submitted_status);
    s->finished_us=(uint32_t)esp_timer_get_time();
    atomic_store_explicit(&s->work_state,PROBE_COEX_WORK_DONE|((core&1u)<<8),memory_order_release);
    return true;
}
