/* Real output/render and sound code with a thread-safe mock I2S/FreeRTOS. */
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "probe_sound.h"
#include "probe_output.h"
#include "driver/i2s_std.h"
#include "esp_heap_caps.h"
#include "freertos/queue.h"
#define CHECK(x) do {if(!(x)){fprintf(stderr,"integration failure %d: %s\n",__LINE__,#x);return 1;}}while(0)
typedef struct {CRITICAL_SECTION lock;unsigned cap,size,read,write;uint8_t *data;} queue_t;
static _Atomic unsigned live,nonzero,failed_alloc,render_wakes;
static _Atomic uintptr_t render_handle;
static _Atomic unsigned hold_hook,hook_reached,renderer_pause,pause_reached;
static DWORD main_id;static _Atomic unsigned main_waits,blocking_sends;
static void external_wake(void *p){atomic_fetch_add((_Atomic unsigned *)p,1);}
static _Atomic DWORD render_id;static i2s_event_callbacks_t callbacks;static void *callback_context;
static _Atomic bool i2s_enabled;static int32_t dma[3][256];static unsigned dma_index,preload;
static _Atomic bool bad_dma;static _Atomic unsigned isr_wakes;
void dbt_output_test_bad_receipt(uint32_t id);
void dbt_output_test_step(unsigned stage){
    if(atomic_load(&hold_hook)==stage){atomic_store(&hook_reached,stage);while(atomic_load(&hold_hook)==stage)Sleep(1);}
}
static void count_pcm(const int32_t *pcm,unsigned n){for(unsigned i=0;i<n;i++)if(pcm[i]){atomic_fetch_add(&nonzero,1);break;}}
void *heap_caps_malloc(size_t n,unsigned flags){(void)flags;if(atomic_exchange(&failed_alloc,0))return 0;void *p=malloc(n);if(p)atomic_fetch_add(&live,1);return p;}
void heap_caps_free(void *p){if(p){atomic_fetch_sub(&live,1);free(p);}}
int64_t esp_timer_get_time(void){return (int64_t)GetTickCount64()*1000;}
void xTaskNotifyGive(TaskHandle_t t){if((uintptr_t)t==atomic_load(&render_handle))atomic_fetch_add(&render_wakes,1);}
void vTaskNotifyGiveFromISR(TaskHandle_t t,BaseType_t *w){if((uintptr_t)t==atomic_load(&render_handle)){atomic_fetch_add(&isr_wakes,1);*w=pdTRUE;}}
unsigned ulTaskNotifyTake(int b,unsigned ticks){
    (void)b;
    if(GetCurrentThreadId()==main_id)atomic_fetch_add(&main_waits,1);
    if(GetCurrentThreadId()==atomic_load(&render_id) && atomic_load(&renderer_pause)){
        atomic_store(&pause_reached,1);while(atomic_load(&renderer_pause))Sleep(1);atomic_store(&pause_reached,0);
    }
    if(GetCurrentThreadId()==atomic_load(&render_id) && atomic_load(&i2s_enabled) && callbacks.on_sent){
        i2s_event_data_t event={.dma_buf=dma[dma_index],.size=sizeof(dma[0])};
        if(atomic_exchange(&bad_dma,false))event.size=0;
        callbacks.on_sent((void*)1,&event,callback_context);count_pcm(dma[dma_index],256);dma_index=(dma_index+1)%3;
    }
    if(ticks)Sleep(1);return 0;
}
typedef struct {void (*fn)(void *);void *arg;} task_args;
static DWORD WINAPI task_entry(void *p){task_args a=*(task_args*)p;free(p);atomic_store(&render_id,GetCurrentThreadId());a.fn(a.arg);return 0;}
int xTaskCreatePinnedToCore(void (*fn)(void *),const char *name,unsigned stack,void *arg,unsigned priority,TaskHandle_t *task,int core){
    (void)name;(void)stack;(void)priority;(void)core;task_args *a=malloc(sizeof(*a));if(!a)return 0;*a=(task_args){fn,arg};
    HANDLE h=CreateThread(NULL,0,task_entry,a,0,NULL);if(!h){free(a);return 0;}*task=h;atomic_store(&render_handle,(uintptr_t)h);return pdPASS;
}
QueueHandle_t xQueueCreate(unsigned cap,unsigned size){queue_t *q=calloc(1,sizeof(*q));if(!q)return 0;InitializeCriticalSection(&q->lock);q->cap=cap;q->size=size;q->data=calloc(cap,size);return q;}
int xQueueSend(QueueHandle_t h,const void *data,unsigned ticks){
    if(ticks)atomic_fetch_add(&blocking_sends,1);
    queue_t *q=h;ULONGLONG until=GetTickCount64()+ticks;
    do {EnterCriticalSection(&q->lock);if(q->write-q->read<q->cap){memcpy(q->data+(q->write%q->cap)*q->size,data,q->size);q->write++;LeaveCriticalSection(&q->lock);return pdTRUE;}LeaveCriticalSection(&q->lock);if(ticks)Sleep(1);}while(ticks && GetTickCount64()<until);return pdFALSE;
}
int xQueueReceive(QueueHandle_t h,void *data,unsigned ticks){
    queue_t *q=h;ULONGLONG until=GetTickCount64()+ticks;
    do {EnterCriticalSection(&q->lock);if(q->read!=q->write){memcpy(data,q->data+(q->read%q->cap)*q->size,q->size);q->read++;LeaveCriticalSection(&q->lock);return pdTRUE;}LeaveCriticalSection(&q->lock);if(ticks)Sleep(1);}while(ticks && GetTickCount64()<until);return pdFALSE;
}
esp_err_t i2s_new_channel(const i2s_chan_config_t *c,i2s_chan_handle_t *ch,void *rx){(void)c;(void)rx;*ch=(void*)1;preload=dma_index=0;return ESP_OK;}
esp_err_t i2s_del_channel(i2s_chan_handle_t ch){(void)ch;return ESP_OK;}
esp_err_t i2s_channel_init_std_mode(i2s_chan_handle_t ch,const i2s_std_config_t *c){(void)ch;(void)c;return ESP_OK;}
esp_err_t i2s_channel_register_event_callback(i2s_chan_handle_t ch,const i2s_event_callbacks_t *cb,void *ctx){(void)ch;callbacks=*cb;callback_context=ctx;return ESP_OK;}
esp_err_t i2s_channel_disable(i2s_chan_handle_t ch){(void)ch;atomic_store(&i2s_enabled,false);return ESP_OK;}
esp_err_t i2s_channel_enable(i2s_chan_handle_t ch){(void)ch;atomic_store(&i2s_enabled,true);return ESP_OK;}
esp_err_t i2s_channel_preload_data(i2s_chan_handle_t ch,const void *p,size_t n,size_t *loaded){(void)ch;if(preload>=3 || n!=sizeof(dma[0]))return 1;memcpy(dma[preload],p,n);count_pcm(dma[preload++],256);*loaded=n;return ESP_OK;}

typedef struct {uint8_t wav[8192];uint32_t len;} store_t;
static bool no_begin(void *v,uint8_t t,uint32_t n){(void)v;(void)t;(void)n;return false;}
static bool no_write(void *v,uint32_t a,const uint8_t *p,uint16_t n){(void)v;(void)a;(void)p;(void)n;return false;}
static bool no_seal(void *v){(void)v;return false;}
static bool no_temp(void *v,uint32_t a,uint8_t *p,uint16_t n){(void)v;(void)a;(void)p;(void)n;return false;}
static bool no_commit(void *v,uint8_t t){(void)v;(void)t;return false;}
static void no_abort(void *v){(void)v;}
static dbt_result_t size_of(void *v,uint8_t t,uint32_t *n){store_t *s=v;if(t!=0)return DBT_EMPTY;*n=s->len;return DBT_OK;}
static bool read_wav(void *v,uint8_t t,uint32_t at,uint8_t *p,uint16_t n){store_t *s=v;if(t || at+n>s->len)return false;memcpy(p,s->wav+at,n);return true;}
static bool no_remove(void *v,uint8_t t){(void)v;(void)t;return false;}
static bool mute(void *v,bool b){(void)v;(void)b;return true;}
static uint32_t now(void *v){(void)v;return (uint32_t)GetTickCount64();}
static bool valid(void *v,dbt_sound_peer_t p){(void)v;return p.generation!=0;}
static void put16(uint8_t *p,uint16_t x){p[0]=(uint8_t)x;p[1]=(uint8_t)(x>>8);}
static void put32(uint8_t *p,uint32_t x){put16(p,(uint16_t)x);put16(p+2,(uint16_t)(x>>16));}
static void prepare(store_t *s){
    memset(s,0,sizeof(*s));s->len=8044;memcpy(s->wav,"RIFF",4);put32(s->wav+4,s->len-8);memcpy(s->wav+8,"WAVEfmt ",8);
    put32(s->wav+16,16);put16(s->wav+20,1);put16(s->wav+22,1);put32(s->wav+24,48000);put32(s->wav+28,96000);
    put16(s->wav+32,2);put16(s->wav+34,16);memcpy(s->wav+36,"data",4);put32(s->wav+40,8000);
    for(unsigned i=0;i<4000;i++)put16(s->wav+44+i*2,(uint16_t)((i%64<32)?12000:-12000));
}
typedef struct {dbt_result_t result;uint32_t rate;} codec_request_t;
static DWORD WINAPI codec_start(void *p){codec_request_t *r=p;r->result=probe_output_stream(true,r->rate,2);return 0;}
static DWORD WINAPI codec_stop(void *p){codec_request_t *r=p;r->result=probe_output_stream(false,0,0);return 0;}
typedef struct {probe_sound_t *sound;dbt_result_t result;} sound_request_t;
static DWORD WINAPI sound_start(void *p){sound_request_t *r=p;r->result=probe_sound_play(r->sound,0,48000,true);return 0;}
static bool pause_render(void){atomic_store(&renderer_pause,1);for(unsigned i=0;i<3000;i++){if(atomic_load(&pause_reached))return true;Sleep(1);}return false;}
static bool await_recovery(void){
    for(unsigned i=0;i<5000;i++){probe_output_stats_t st;probe_output_stats(&st);if(!st.request_fault)return true;Sleep(1);}return false;
}
static dbt_result_t await_ticket(uint32_t ticket){
    for(unsigned i=0;i<5000;++i){dbt_result_t r=probe_output_request_poll(ticket);if(r!=DBT_PENDING)return r;Sleep(1);}
    return DBT_PENDING;
}
static bool yes_start(void *p,bool exclusive){(void)p;(void)exclusive;return true;}
static void no_stop(void *p){(void)p;}
static void *play_allocate(void *p,size_t n){(void)p;return heap_caps_malloc(n,0);}
static void play_release(void *p,void *data){(void)p;heap_caps_free(data);}

int main(void){
    main_id=GetCurrentThreadId();
    CHECK(probe_output_init(0,0)==DBT_OK);store_t file;prepare(&file);dbt_sound_t provider;
    dbt_sound_store_t st={&file,no_begin,no_write,no_seal,no_temp,no_commit,no_abort,size_of,read_wav,no_remove,mute,now,valid};
    CHECK(dbt_sound_init(&provider,st,false)==DBT_OK);probe_sound_t sound;
    CHECK(probe_sound_init(&sound,&provider,(void*)1)==DBT_OK);
    CHECK(probe_sound_play(&sound,0,48000,true)==DBT_OK);
    CHECK(probe_output_sound_release(sound.token+1)==DBT_INVALID && probe_output_sound_alive(sound.token));
    unsigned before_wake=atomic_load(&render_wakes);CHECK(probe_sound_poll(&sound,now(0),0)==DBT_OK);
    CHECK(atomic_load(&render_wakes)>before_wake);
    for(unsigned i=0;i<5000 && (sound.play || sound.token);i++){(void)probe_sound_poll(&sound,now(0),0);Sleep(1);}
    CHECK(!sound.play && !sound.token && atomic_load(&nonzero)>0 && atomic_load(&live)==0);
    CHECK(probe_sound_play(&sound,0,48000,true)==DBT_OK);probe_sound_stop(&sound);
    for(unsigned i=0;i<5000 && (sound.play || sound.token);i++){(void)probe_sound_poll(&sound,now(0),0);Sleep(1);}
    CHECK(!sound.play && !sound.token && atomic_load(&live)==0);
    CHECK(probe_output_stream(true,48000,2)==DBT_OK);
    int16_t music[256];for(unsigned i=0;i<256;i++)music[i]=4000;
    for(unsigned i=0;i<32;i++)CHECK(probe_output_pcm(music,128,2,48000)==DBT_OK);
    CHECK(probe_sound_play(&sound,0,48000,true)==DBT_OK);
    for(unsigned i=0;i<5000 && (sound.play || sound.token);i++){
        (void)probe_sound_poll(&sound,now(0),0);(void)probe_output_pcm(music,128,2,48000);Sleep(1);
    }
    CHECK(!sound.play && !sound.token && probe_output_active_rate()==48000);
    CHECK(probe_output_pcm(music,128,2,48000)==DBT_OK);
    CHECK(probe_output_stream(false,0,0)==DBT_OK && atomic_load(&live)==0);
    CHECK(probe_output_sound_release(0)==DBT_INVALID);
    uint8_t wide[128*2*4];memset(wide,0,sizeof(wide));
    for(unsigned width=3;width<=4;width++){
        for(unsigned i=0;i<256;i++){unsigned pos=i*width;wide[pos+width-1]=0x20;}
        CHECK(probe_output_stream_format(true,48000,2,(dbt_pcm_format_t)width)==DBT_OK);
        for(unsigned i=0;i<32;i++)CHECK(probe_output_pcm_format(wide,128,2,48000,(dbt_pcm_format_t)width)==DBT_OK);
        CHECK(probe_sound_play(&sound,0,48000,false)==DBT_OK);
        for(unsigned i=0;i<5000 && (sound.play || sound.token);i++){
            (void)probe_sound_poll(&sound,now(0),0);(void)probe_output_pcm_format(wide,128,2,48000,(dbt_pcm_format_t)width);Sleep(1);
        }
        CHECK(!sound.play && !sound.token && probe_output_active_rate()==48000);
        CHECK(probe_output_pcm_format(wide,128,2,48000,(dbt_pcm_format_t)width)==DBT_OK);
        CHECK(probe_output_stream(false,0,0)==DBT_OK && atomic_load(&live)==0);
    }
    /* An open music stream with no PCM must still render a sound, then accept music. */
    CHECK(probe_output_stream(true,48000,2)==DBT_OK);
    CHECK(probe_sound_play(&sound,0,48000,false)==DBT_OK);
    for(unsigned i=0;i<5000 && (sound.play || sound.token);i++){(void)probe_sound_poll(&sound,now(0),0);Sleep(1);}
    CHECK(!sound.play && !sound.token && probe_output_active_rate()==48000);
    CHECK(probe_output_pcm(music,128,2,48000)==DBT_OK);
    CHECK(probe_output_stream(false,0,0)==DBT_OK && atomic_load(&live)==0);
    CHECK(probe_sound_play(&sound,0,48000,true)==DBT_OK);
    codec_request_t request={DBT_INVALID,44100};HANDLE t=CreateThread(NULL,0,codec_start,&request,0,NULL);CHECK(t);
    for(unsigned i=0;i<5000 && WaitForSingleObject(t,0)!=WAIT_OBJECT_0;i++){(void)probe_sound_poll(&sound,now(0),0);Sleep(1);}
    CHECK(WaitForSingleObject(t,0)==WAIT_OBJECT_0 && request.result==DBT_OK);CloseHandle(t);
    for(unsigned i=0;i<100 && (sound.play || sound.token);i++){(void)probe_sound_poll(&sound,now(0),0);Sleep(1);}
    CHECK(!sound.play && !sound.token && probe_output_active_rate()==44100);
    CHECK(probe_output_stream(false,0,0)==DBT_OK && atomic_load(&live)==0);
    CHECK(probe_output_stream(true,48000,2)==DBT_OK);
    for(unsigned i=0;i<32;i++)CHECK(probe_output_pcm(music,128,2,48000)==DBT_OK);
    CHECK(probe_sound_play(&sound,0,48000,false)==DBT_OK);
    codec_request_t stopped={DBT_INVALID,0};HANDLE stopper=CreateThread(NULL,0,codec_stop,&stopped,0,NULL);CHECK(stopper);
    for(unsigned i=0;i<5000 && WaitForSingleObject(stopper,0)!=WAIT_OBJECT_0;i++){(void)probe_sound_poll(&sound,now(0),0);Sleep(1);}
    CHECK(WaitForSingleObject(stopper,0)==WAIT_OBJECT_0 && stopped.result==DBT_OK);CloseHandle(stopper);
    for(unsigned i=0;i<100 && (sound.play || sound.token);i++){(void)probe_sound_poll(&sound,now(0),0);Sleep(1);}
    CHECK(!sound.play && !sound.token && atomic_load(&live)==0);
    for(unsigned i=0;i<32;i++){
        codec_request_t a={DBT_INVALID,44100},b={DBT_INVALID,96000};
        HANDLE t1=CreateThread(NULL,0,codec_start,&a,0,NULL),t2=CreateThread(NULL,0,codec_start,&b,0,NULL);CHECK(t1 && t2);
        CHECK(WaitForSingleObject(t1,5000)==WAIT_OBJECT_0 && WaitForSingleObject(t2,5000)==WAIT_OBJECT_0);
        CloseHandle(t1);CloseHandle(t2);
        CHECK((a.result==DBT_OK && b.result==DBT_BUSY && probe_output_active_rate()==44100) ||
              (b.result==DBT_OK && a.result==DBT_BUSY && probe_output_active_rate()==96000));
        CHECK(probe_output_stream(false,0,0)==DBT_OK && atomic_load(&live)==0);
    }
    atomic_store(&failed_alloc,1);CHECK(probe_sound_play(&sound,0,48000,true)==DBT_NO_SPACE && !sound.play);
    /* A permanently paused render returns a bounded fault and retains its one
     * ticket. Wrong receipts cannot satisfy it; late processing cancels it. */
    CHECK(pause_render());codec_request_t stalled={DBT_INVALID,48000};
    HANDLE blocked=CreateThread(NULL,0,codec_start,&stalled,0,NULL);CHECK(blocked);Sleep(20);
    dbt_output_test_bad_receipt(0);
    CHECK(WaitForSingleObject(blocked,4000)==WAIT_OBJECT_0 && stalled.result==DBT_BUSY);CloseHandle(blocked);
    probe_output_stats_t stats;probe_output_stats(&stats);CHECK(stats.request_fault && stats.stale_receipts>=1);
    probe_output_restart_hold(true);CHECK(!probe_output_restart_quiescent());probe_output_restart_hold(false);
    CHECK(probe_output_stream(true,44100,2)==DBT_BUSY && probe_output_active_rate()==0);
    atomic_store(&renderer_pause,0);CHECK(await_recovery());CHECK(probe_output_active_rate()==0);
    CHECK(probe_output_stream(true,48000,2)==DBT_OK);
    CHECK(pause_render());codec_request_t late_stop={DBT_INVALID,0};
    blocked=CreateThread(NULL,0,codec_stop,&late_stop,0,NULL);CHECK(blocked);
    CHECK(WaitForSingleObject(blocked,4000)==WAIT_OBJECT_0 && late_stop.result==DBT_BUSY);CloseHandle(blocked);
    atomic_store(&renderer_pause,0);CHECK(await_recovery());CHECK(probe_output_active_rate()==48000);
    CHECK(probe_output_stream(false,0,0)==DBT_OK && atomic_load(&live)==0);
    /* Timeout immediately before and immediately after the render borrow. */
    const unsigned stages[]={1,2,4};
    for(unsigned index=0;index<3;index++){
        unsigned stage=stages[index];
        atomic_store(&hook_reached,0);atomic_store(&hold_hook,stage);
        sound_request_t request_sound={&sound,DBT_INVALID};HANDLE starter=CreateThread(NULL,0,sound_start,&request_sound,0,NULL);CHECK(starter);
        for(unsigned i=0;i<3000 && atomic_load(&hook_reached)!=stage;i++)Sleep(1);
        CHECK(atomic_load(&hook_reached)==stage);
        CHECK(WaitForSingleObject(starter,4000)==WAIT_OBJECT_0 && request_sound.result==DBT_BUSY);CloseHandle(starter);
        CHECK(sound.play && sound.token && atomic_load(&live)>=3);
        probe_sound_stop(&sound);
        for(unsigned i=0;i<5;i++){(void)probe_sound_poll(&sound,now(0),0);Sleep(1);}
        CHECK(sound.play && atomic_load(&live)>=3);
        atomic_store(&hold_hook,0);
        for(unsigned i=0;i<5000 && (sound.play || sound.token);i++){(void)probe_sound_poll(&sound,now(0),0);Sleep(1);}
        CHECK(!sound.play && !sound.token && !atomic_load(&live) && await_recovery());
    }
    CHECK(probe_sound_play(&sound,0,48000,true)==DBT_OK);
    atomic_store(&hook_reached,0);atomic_store(&hold_hook,3);
    codec_request_t late_restart={DBT_INVALID,44100};HANDLE switcher=CreateThread(NULL,0,codec_start,&late_restart,0,NULL);CHECK(switcher);
    for(unsigned i=0;i<5000 && atomic_load(&hook_reached)!=3;i++){(void)probe_sound_poll(&sound,now(0),0);Sleep(1);}
    CHECK(atomic_load(&hook_reached)==3 && WaitForSingleObject(switcher,4000)==WAIT_OBJECT_0 && late_restart.result==DBT_BUSY);CloseHandle(switcher);
    atomic_store(&hold_hook,0);CHECK(await_recovery());
    for(unsigned i=0;i<100 && (sound.play || sound.token);i++){(void)probe_sound_poll(&sound,now(0),0);Sleep(1);}
    CHECK(!sound.play && !sound.token && probe_output_active_rate()==0 && !atomic_load(&live));
    /* A persistent dispatcher wake does not retain any recreatable task handle. */
    _Atomic unsigned wakes=0;probe_sound_t external={0};
    CHECK(probe_sound_init_with_wake(&external,&provider,external_wake,&wakes)==DBT_OK && !external.worker);
    CHECK(probe_sound_play(&external,0,48000,true)==DBT_OK);
    CHECK(probe_sound_init_with_wake(&external,&provider,external_wake,&wakes)==DBT_BUSY && external.play && external.token);
    for(unsigned i=0;i<5000 && (external.play || external.token);i++){(void)probe_sound_poll(&external,now(0),0);Sleep(1);}
    CHECK(!external.play && !external.token && atomic_load(&wakes)>0 && !atomic_load(&live));
    /* Async admission/completion/cancel never wait for the paused renderer. */
    CHECK(pause_render());uint32_t ticket=0,other=123;
    unsigned waits=atomic_load(&main_waits),sends=atomic_load(&blocking_sends);
    CHECK(probe_output_stream_begin(true,48000,2,DBT_PCM_S16_LE,&ticket)==DBT_PENDING && ticket);
    CHECK(probe_output_stream_begin(true,44100,2,DBT_PCM_S16_LE,&other)==DBT_BUSY && !other);
    CHECK(probe_output_request_poll(ticket)==DBT_PENDING && probe_output_request_cancel(ticket)==DBT_PENDING);
    CHECK(probe_output_request_poll(ticket)==DBT_PENDING && atomic_load(&main_waits)==waits && atomic_load(&blocking_sends)==sends);
    atomic_store(&renderer_pause,0);CHECK(await_ticket(ticket)==DBT_CANCELLED && !probe_output_active_rate() && !atomic_load(&live));
    uint32_t stale=ticket;
    CHECK(probe_output_stream_begin(true,48000,2,DBT_PCM_S16_LE,&ticket)==DBT_PENDING);
    CHECK(await_ticket(ticket)==DBT_OK && pause_render());
    CHECK(probe_output_request_cancel(stale)==DBT_INVALID);
    /* A full producer ring returns BUSY without a sleep or partial publish. */
    waits=atomic_load(&main_waits);unsigned submitted=0;
    for(unsigned i=0;i<100;++i){dbt_result_t r=probe_output_try_pcm_format(music,128,2,48000,DBT_PCM_S16_LE);
        if(r==DBT_BUSY)break;CHECK(r==DBT_OK);++submitted;}
    CHECK(submitted && submitted<100 && atomic_load(&main_waits)==waits);
    probe_output_stats_t before,after;probe_output_stats(&before);
    CHECK(probe_output_try_pcm_format(music,128,2,48000,DBT_PCM_S16_LE)==DBT_BUSY);
    probe_output_stats(&after);CHECK(before.accepted_frames==after.accepted_frames);
    CHECK(probe_output_stream_begin(false,0,0,DBT_PCM_S16_LE,&ticket)==DBT_PENDING);
    atomic_store(&renderer_pause,0);CHECK(await_ticket(ticket)==DBT_OK && !atomic_load(&live));
    /* Cancellation after DONE retires an applied start on the owner thread. */
    CHECK(probe_output_stream_begin(true,48000,2,DBT_PCM_S16_LE,&ticket)==DBT_PENDING);
    for(unsigned i=0;i<5000 && probe_output_active_rate()!=48000;++i)Sleep(1);
    CHECK(probe_output_active_rate()==48000 && pause_render());
    CHECK(probe_output_request_cancel(ticket)==DBT_PENDING && probe_output_request_poll(ticket)==DBT_PENDING && atomic_load(&live));
    atomic_store(&renderer_pause,0);CHECK(await_ticket(ticket)==DBT_CANCELLED && !atomic_load(&live));
    /* Keep an attached play object live until cancellation really retires it. */
    for(unsigned stage=2;stage<=4;stage+=2){
        CHECK(probe_output_sound_reserve_begin(48000,true,&ticket)==DBT_PENDING && await_ticket(ticket)==DBT_OK);
        uint32_t token=ticket;dbt_sound_play_t *play=NULL;
        dbt_sound_output_t hooks={NULL,yes_start,no_stop};
        CHECK(dbt_sound_play_open(&provider,0,48000,true,hooks,NULL,play_allocate,play_release,&play)==DBT_OK);
        for(unsigned i=0;i<3 && !dbt_sound_play_ready(play);++i)(void)dbt_sound_play_pump(play,64);
        CHECK(dbt_sound_play_ready(play));atomic_store(&hook_reached,0);atomic_store(&hold_hook,stage);
        CHECK(probe_output_sound_attach_begin(token,play,&ticket)==DBT_PENDING);
        for(unsigned i=0;i<5000 && atomic_load(&hook_reached)!=stage;++i)Sleep(1);
        CHECK(atomic_load(&hook_reached)==stage && probe_output_request_cancel(ticket)==DBT_PENDING);
        CHECK(probe_output_request_poll(ticket)==DBT_PENDING && probe_output_sound_alive(token) && atomic_load(&live)>=3);
        atomic_store(&hold_hook,0);CHECK(await_ticket(ticket)==DBT_CANCELLED && !probe_output_sound_alive(token));
        dbt_sound_play_close(play);CHECK(!atomic_load(&live));
    }
    /* A terminal restart cannot acquire a fresh output loan or restart codec. */
    probe_output_restart_hold(true);CHECK(probe_output_restart_quiescent());
    CHECK(probe_output_stream(true,48000,2)==DBT_BUSY);
    uint32_t blocked_token=123;CHECK(probe_output_sound_reserve(48000,true,&blocked_token)==DBT_BUSY && !blocked_token);
    CHECK(probe_output_restart_quiescent());
    probe_output_restart_hold(false);CHECK(!probe_output_restart_quiescent());
    CHECK(probe_output_stream(true,48000,2)==DBT_OK);
    probe_output_restart_hold(true);CHECK(!probe_output_restart_quiescent());
    CHECK(probe_output_stream(false,0,0)==DBT_OK && probe_output_restart_quiescent());
    probe_output_restart_hold(false);
    puts("actual sound/output integration passed with mocked I2S and concurrent codec request");return 0;
}
