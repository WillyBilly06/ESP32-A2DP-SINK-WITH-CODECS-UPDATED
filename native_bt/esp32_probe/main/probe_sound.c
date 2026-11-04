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
#ifndef DBT_PROBE_ASYNC_AUDIO
static bool output_start(void *p,bool exclusive){
    probe_sound_t *s=p;return probe_output_sound_reserve(s->rate,exclusive,&s->token)==DBT_OK;
}
#else
static bool output_start(void *p,bool exclusive){(void)p;(void)exclusive;return true;}
#endif
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
#ifndef DBT_PROBE_ASYNC_AUDIO
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
bool probe_sound_deadline(const probe_sound_t *s,uint32_t *when){(void)s;(void)when;return false;}
#else
enum {S_IDLE=0,S_OPEN,S_PREFILL,S_RESERVE,S_RESERVE_WAIT,S_ATTACH,S_ATTACH_WAIT,S_PLAY,S_RELEASE,S_RELEASE_WAIT,S_FAULT};
static void phase(probe_sound_t *s,uint8_t next,uint32_t now){
    s->phase=next;s->phase_since=now;s->phase_timedout=false;s->cancel_sent=false;
}
static void clean_retired(probe_sound_t *s){
    if(s->ticket || (s->token && probe_output_sound_alive(s->token)))return;
    if(s->play)dbt_sound_play_close(s->play);
    s->play=0;s->token=0;s->attached=false;s->stopping=false;s->phase=S_IDLE;s->start_pending=false;
    wake(s);
}
static void release_after_error(probe_sound_t *s,dbt_result_t result,uint32_t now){
    s->last_result=result;s->stopping=true;
    if(s->play)dbt_sound_play_request_stop(s->play);
    if(s->token && probe_output_sound_alive(s->token))phase(s,S_RELEASE,now);
    else clean_retired(s);
}
dbt_result_t probe_sound_play(probe_sound_t *s,uint8_t type,uint32_t preferred_rate,bool exclusive){
    if(!s || !s->provider || type>3u || preferred_rate<8000 || preferred_rate>192000)return DBT_INVALID;
    if(probe_sound_busy(s))return dbt_sound_play_queue(s->provider,type,preferred_rate,exclusive);
    s->type=type;s->preferred_rate=preferred_rate;s->exclusive=exclusive && type!=3u;s->stopping=false;
    s->last_result=DBT_PENDING;s->start_pending=true;s->phase=S_OPEN;s->ticket=0;wake(s);return DBT_PENDING;
}
static void sound_step(probe_sound_t *s,uint32_t now){
    if(s->phase==S_IDLE)return;
    if(s->phase==S_FAULT){clean_retired(s);return;}
    if(s->phase==S_OPEN){
        if(s->stopping){s->last_result=DBT_CANCELLED;clean_retired(s);return;}
        uint32_t active=probe_output_active_rate();s->rate=active?active:s->preferred_rate;
        if(active)s->exclusive=false;
        dbt_sound_output_t hooks={s,output_start,output_stop};
        dbt_result_t r=dbt_sound_play_open(s->provider,s->type,s->rate,s->exclusive,hooks,s,allocate,release,&s->play);
        if(r!=DBT_OK){s->last_result=r;clean_retired(s);return;}
        phase(s,S_PREFILL,now);wake(s);return;
    }
    if(s->phase==S_PREFILL){
        if(s->stopping){release_after_error(s,DBT_CANCELLED,now);return;}
        dbt_result_t r=dbt_sound_play_pump(s->play,128);
        if(dbt_sound_play_ready(s->play)){phase(s,S_RESERVE,now);wake(s);return;}
        if(r!=DBT_OK && r!=DBT_BUSY){release_after_error(s,r==DBT_EMPTY?DBT_INVALID:r,now);return;}
        wake(s);return;
    }
    if(s->phase==S_RESERVE || s->phase==S_ATTACH || s->phase==S_RELEASE){
        if(s->stopping && s->phase!=S_RELEASE){release_after_error(s,DBT_CANCELLED,now);return;}
        if(s->phase!=S_RESERVE && !probe_output_sound_alive(s->token)){if(s->start_pending)s->last_result=DBT_CANCELLED;clean_retired(s);return;}
        if(now-s->phase_since>=1500u){
            s->last_result=DBT_BUSY;s->stopping=true;
            if(s->phase!=S_RELEASE){release_after_error(s,DBT_BUSY,now);return;}
            s->phase=S_FAULT;s->start_pending=false;wake(s);return;
        }
        uint32_t ticket=0;dbt_result_t r;
        if(s->phase==S_RESERVE)r=probe_output_sound_reserve_begin(s->rate,s->exclusive,&ticket);
        else if(s->phase==S_ATTACH)r=probe_output_sound_attach_begin(s->token,s->play,&ticket);
        else r=probe_output_sound_release_begin(s->token,&ticket);
        if(r==DBT_BUSY)return;
        if(r!=DBT_PENDING || !ticket){
            if(s->phase==S_RELEASE){s->last_result=r;s->phase=S_FAULT;s->start_pending=false;wake(s);}
            else release_after_error(s,r,now);
            return;
        }
        uint8_t waiting=s->phase==S_RESERVE?S_RESERVE_WAIT:s->phase==S_ATTACH?S_ATTACH_WAIT:S_RELEASE_WAIT;
        s->ticket=ticket;phase(s,waiting,now);return;
    }
    if(s->phase==S_RESERVE_WAIT || s->phase==S_ATTACH_WAIT || s->phase==S_RELEASE_WAIT){
        bool cancel=s->stopping && s->phase!=S_RELEASE_WAIT;
        if(now-s->phase_since>=1500u){s->phase_timedout=true;cancel=true;s->last_result=DBT_BUSY;}
        if(cancel && !s->cancel_sent){
            dbt_result_t r=probe_output_request_cancel(s->ticket);
            if(r==DBT_BUSY)return;
            if(r!=DBT_PENDING){s->last_result=DBT_INVALID;s->phase=S_FAULT;s->start_pending=false;wake(s);return;}
            s->cancel_sent=true;
        }
        dbt_result_t r=probe_output_request_poll(s->ticket);
        if(r==DBT_PENDING)return;
        uint32_t ticket=s->ticket;s->ticket=0;uint8_t completed=s->phase;
        if(completed==S_RESERVE_WAIT && r==DBT_OK)s->token=ticket;
        if(completed==S_RESERVE_WAIT && r==DBT_CANCELLED && probe_output_sound_alive(ticket))s->token=ticket;
        if(completed==S_ATTACH_WAIT && r==DBT_OK)s->attached=true;
        if(r==DBT_OK && !s->stopping){
            if(completed==S_RESERVE_WAIT){phase(s,S_ATTACH,now);wake(s);return;}
            if(completed==S_ATTACH_WAIT){s->attached=true;s->last_result=DBT_OK;s->start_pending=false;phase(s,S_PLAY,now);wake(s);return;}
        }
        if(completed==S_RELEASE_WAIT){
            if(!probe_output_sound_alive(s->token)){clean_retired(s);return;}
            if(r==DBT_BUSY || r==DBT_CANCELLED){phase(s,S_PLAY,now);s->stopping=true;wake(s);return;}
            s->last_result=r;s->phase=S_FAULT;s->start_pending=false;wake(s);return;
        }
        release_after_error(s,s->phase_timedout?DBT_BUSY:s->stopping?DBT_CANCELLED:r,now);return;
    }
    if(s->phase==S_PLAY){
        if(!probe_output_sound_alive(s->token)){clean_retired(s);return;}
        probe_output_stats_t stats;probe_output_stats(&stats);
        if(stats.failed){s->last_result=DBT_INVALID;s->stopping=true;dbt_sound_play_request_stop(s->play);phase(s,S_RELEASE,now);wake(s);return;}
        dbt_result_t r=dbt_sound_play_pump(s->play,128);
        if(r==DBT_OK)probe_output_sound_data_ready();
        if(r!=DBT_OK && r!=DBT_BUSY && r!=DBT_EMPTY){s->last_result=r;s->stopping=true;dbt_sound_play_request_stop(s->play);}
        if(!s->attached || dbt_sound_play_drained(s->play)){phase(s,S_RELEASE,now);wake(s);}
    }
}
dbt_result_t probe_sound_poll(probe_sound_t *s,uint32_t now,dbt_sound_peer_t *expired){
    if(!s || !s->provider)return DBT_INVALID;
    bool timeout=dbt_sound_poll(s->provider,now,expired);sound_step(s,now);
    if(!probe_sound_busy(s)){
        uint8_t type;uint32_t rate;bool exclusive;
        if(dbt_sound_play_take_queued(s->provider,&type,&rate,&exclusive)){
            dbt_result_t r=probe_sound_play(s,type,rate,exclusive);if(r!=DBT_PENDING && r!=DBT_OK)s->last_result=r;
        }
    }
    return timeout?DBT_EMPTY:DBT_OK;
}
void probe_sound_stop(probe_sound_t *s){
    if(!s || !s->provider)return;
    dbt_sound_play_clear_queue(s->provider);s->stopping=true;
    if(s->phase==S_FAULT && !s->ticket)
        phase(s,S_RELEASE,s->provider->store.now_ms(s->provider->store.context));
    if(s->play)dbt_sound_play_request_stop(s->play);
    probe_output_sound_data_ready();wake(s);
}
bool probe_sound_deadline(const probe_sound_t *s,uint32_t *when){
    if(!s || !when || s->phase_timedout)return false;
    if(s->phase==S_RESERVE || s->phase==S_ATTACH || s->phase==S_RELEASE ||
       s->phase==S_RESERVE_WAIT || s->phase==S_ATTACH_WAIT || s->phase==S_RELEASE_WAIT){*when=s->phase_since+1500u;return true;}
    return false;
}
#endif
dbt_result_t probe_sound_disconnect(probe_sound_t *s,dbt_sound_peer_t peer){return s && s->provider?dbt_sound_disconnect(s->provider,peer):DBT_INVALID;}
