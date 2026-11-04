#include "probe_sound.h"
#include "probe_output.h"
#include "esp_heap_caps.h"
#include <string.h>
static void wake(void *p){
    probe_sound_t *s=p;
    if(s->wake_external)s->wake_external(s->wake_context);
    else if(s->worker)xTaskNotifyGive(s->worker);
}
static void *allocate(void *p,size_t n){(void)p;return heap_caps_malloc(n,MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);}
static void release(void *p,void *data){(void)p;heap_caps_free(data);}
static bool output_start(void *p,bool exclusive){
    probe_sound_t *s=p;return probe_output_sound_reserve(s->rate,exclusive,&s->token)==DBT_OK;
}
static void output_stop(void *p){(void)p;/* Explicit token retirement precedes close. */}
dbt_result_t probe_sound_init(probe_sound_t *s,dbt_sound_t *provider,TaskHandle_t worker){
    if(!s || !provider || !worker)return DBT_INVALID;
    memset(s,0,sizeof(*s));s->provider=provider;s->worker=worker;probe_output_sound_wake(wake,s);return DBT_OK;
}
dbt_result_t probe_sound_init_with_wake(probe_sound_t *s,dbt_sound_t *provider,void (*callback)(void *),void *context){
    if(!s || !provider || !callback)return DBT_INVALID;
    if(s->provider)return DBT_BUSY;
    memset(s,0,sizeof(*s));s->provider=provider;s->wake_external=callback;s->wake_context=context;
    probe_output_sound_wake(wake,s);return DBT_OK;
}
dbt_result_t probe_sound_play(probe_sound_t *s,uint8_t type,uint32_t preferred_rate,bool exclusive){
    if(!s || !s->provider || preferred_rate<8000 || preferred_rate>192000)return DBT_INVALID;
    if(s->play)return dbt_sound_play_queue(s->provider,type,preferred_rate,exclusive);
    if(s->token)return DBT_BUSY;
    uint32_t active_rate=probe_output_active_rate();s->rate=active_rate?active_rate:preferred_rate;
    /* Preserve an active music producer: play its UI sound as an overlay. */
    if(active_rate)exclusive=false;
    dbt_sound_output_t hooks={s,output_start,output_stop};
    dbt_result_t r=dbt_sound_play_open(s->provider,type,s->rate,exclusive,hooks,s,allocate,release,&s->play);
    if(r!=DBT_OK)return r;
    s->type=type;s->last_result=DBT_OK;s->stopping=false;
    for(unsigned i=0;i<3 && !dbt_sound_play_ready(s->play);i++){
        r=dbt_sound_play_pump(s->play,64);if(r!=DBT_OK && r!=DBT_BUSY && r!=DBT_EMPTY)break;
    }
    if(!dbt_sound_play_ready(s->play) || r==DBT_INVALID || !probe_output_sound_alive(s->token)){
        dbt_sound_play_close(s->play);s->play=0;s->last_result=DBT_INVALID;return DBT_INVALID;
    }
    r=probe_output_sound_attach(s->token,s->play);
    if(r!=DBT_OK){
        s->last_result=r;
        if(r==DBT_BUSY){s->stopping=true;dbt_sound_play_request_stop(s->play);return r;}
        dbt_sound_play_close(s->play);s->play=0;return r;
    }
    s->attached=true;return DBT_OK;
}
dbt_result_t probe_sound_poll(probe_sound_t *s,uint32_t now,dbt_sound_peer_t *expired){
    if(!s || !s->provider)return DBT_INVALID;
    bool timeout=dbt_sound_poll(s->provider,now,expired);
    if(s->token && !s->play){
        dbt_result_t r=probe_output_sound_release(s->token);
        if(r==DBT_OK || !probe_output_sound_alive(s->token))s->token=0;
    }
    if(s->play){
        if(!probe_output_sound_alive(s->token)){
            dbt_sound_play_close(s->play);s->play=0;s->token=0;s->attached=false;
        }else {
            dbt_result_t r=dbt_sound_play_pump(s->play,128);
            if(r==DBT_OK)probe_output_sound_data_ready();
            if(r!=DBT_OK && r!=DBT_BUSY && r!=DBT_EMPTY){s->last_result=r;dbt_sound_play_request_stop(s->play);s->stopping=true;}
            if(s->stopping || dbt_sound_play_drained(s->play)){
                r=probe_output_sound_release(s->token);
                if(r==DBT_OK || !probe_output_sound_alive(s->token)){
                    dbt_sound_play_close(s->play);s->play=0;s->token=0;s->attached=false;
                }
            }
        }
    }
    if(!s->play && !s->token){
        uint8_t type;uint32_t rate;bool exclusive;
        if(dbt_sound_play_take_queued(s->provider,&type,&rate,&exclusive)){
            dbt_result_t r=probe_sound_play(s,type,rate,exclusive);if(r!=DBT_OK)s->last_result=r;
        }
    }
    return timeout?DBT_EMPTY:DBT_OK;
}
void probe_sound_stop(probe_sound_t *s){
    if(!s || !s->provider)return;
    dbt_sound_play_clear_queue(s->provider);
    if(s->play){s->stopping=true;dbt_sound_play_request_stop(s->play);probe_output_sound_data_ready();}
}
dbt_result_t probe_sound_disconnect(probe_sound_t *s,dbt_sound_peer_t peer){return s && s->provider?dbt_sound_disconnect(s->provider,peer):DBT_INVALID;}
