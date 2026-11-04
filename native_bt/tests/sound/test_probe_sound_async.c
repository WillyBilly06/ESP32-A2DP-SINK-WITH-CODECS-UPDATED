/* Reuse the concurrent FreeRTOS/I2S simulator; exercise the async client. */
#define main legacy_integration_main
#include "test_probe_sound_output.c"
#undef main
static dbt_result_t all_size(void *v,uint8_t t,uint32_t *n){return t==1?DBT_EMPTY:size_of(v,0,n);}
static bool all_read(void *v,uint8_t t,uint32_t at,uint8_t *p,uint16_t n){return t!=1 && read_wav(v,0,at,p,n);}
static bool start_finished(probe_sound_t *s){
    for(unsigned i=0;i<5000 && probe_sound_start_pending(s);++i){(void)probe_sound_poll(s,now(NULL),NULL);Sleep(1);}
    return !probe_sound_start_pending(s);
}
static bool idle(probe_sound_t *s){
    for(unsigned i=0;i<5000 && probe_sound_busy(s);++i){(void)probe_sound_poll(s,now(NULL),NULL);Sleep(1);}
    return !probe_sound_busy(s);
}
int main(void){
    main_id=GetCurrentThreadId();CHECK(probe_output_init(NULL,NULL)==DBT_OK);
    store_t file;prepare(&file);dbt_sound_t provider;
    dbt_sound_store_t st={&file,no_begin,no_write,no_seal,no_temp,no_commit,no_abort,all_size,all_read,no_remove,mute,now,valid};
    CHECK(dbt_sound_init(&provider,st,false)==DBT_OK);
    probe_sound_t sound={0};CHECK(probe_sound_init(&sound,&provider,(void *)1)==DBT_OK);
    unsigned waits=atomic_load(&main_waits),sends=atomic_load(&blocking_sends);
    CHECK(probe_sound_play(&sound,0,48000,true)==DBT_PENDING && probe_sound_busy(&sound) && probe_sound_start_pending(&sound));
    CHECK(atomic_load(&main_waits)==waits && atomic_load(&blocking_sends)==sends);
    CHECK(start_finished(&sound) && sound.last_result==DBT_OK && sound.play && sound.attached);
    CHECK(idle(&sound) && !atomic_load(&live) && atomic_load(&nonzero));
    /* Missing assets fail without reporting a successful start or leaking. */
    CHECK(probe_sound_play(&sound,1,48000,true)==DBT_PENDING && start_finished(&sound));
    CHECK(sound.last_result!=DBT_OK && !probe_sound_busy(&sound) && !atomic_load(&live));
    atomic_store(&failed_alloc,1);
    CHECK(probe_sound_play(&sound,0,48000,true)==DBT_PENDING && start_finished(&sound));
    CHECK(sound.last_result==DBT_NO_SPACE && !probe_sound_busy(&sound) && !atomic_load(&live));
    /* Stop before allocation and while the renderer owns a pending reserve. */
    CHECK(probe_sound_play(&sound,0,48000,true)==DBT_PENDING);probe_sound_stop(&sound);
    CHECK(idle(&sound) && sound.last_result==DBT_CANCELLED && !atomic_load(&live));
    CHECK(pause_render());CHECK(probe_sound_play(&sound,0,48000,true)==DBT_PENDING);
    for(unsigned i=0;i<30 && !sound.ticket;++i)(void)probe_sound_poll(&sound,now(NULL),NULL);
    CHECK(sound.ticket && probe_sound_start_pending(&sound));uint32_t deadline;
    CHECK(probe_sound_deadline(&sound,&deadline));
    waits=atomic_load(&main_waits);probe_sound_stop(&sound);(void)probe_sound_poll(&sound,now(NULL),NULL);
    CHECK(probe_sound_busy(&sound) && sound.play && probe_sound_start_pending(&sound) && atomic_load(&main_waits)==waits);
    atomic_store(&renderer_pause,0);CHECK(idle(&sound) && sound.last_result==DBT_CANCELLED && !atomic_load(&live));
    /* A timed-out accepted reserve retains ownership until renderer recovery. */
    CHECK(pause_render());CHECK(probe_sound_play(&sound,0,48000,true)==DBT_PENDING);
    for(unsigned i=0;i<30 && !sound.ticket;++i)(void)probe_sound_poll(&sound,now(NULL),NULL);
    CHECK(sound.ticket && probe_sound_deadline(&sound,&deadline));
    (void)probe_sound_poll(&sound,deadline,NULL);
    CHECK(sound.ticket && sound.play && probe_sound_start_pending(&sound) && !probe_sound_deadline(&sound,&deadline));
    atomic_store(&renderer_pause,0);CHECK(idle(&sound) && sound.last_result==DBT_BUSY && !atomic_load(&live));
    for(unsigned stage=1;stage<=4;++stage){
        if(stage==3)continue;
        atomic_store(&hook_reached,0);atomic_store(&hold_hook,stage);
        CHECK(probe_sound_play(&sound,0,48000,true)==DBT_PENDING);
        for(unsigned i=0;i<5000 && atomic_load(&hook_reached)!=stage;++i){(void)probe_sound_poll(&sound,now(NULL),NULL);Sleep(1);}
        CHECK(atomic_load(&hook_reached)==stage && sound.play && sound.token && probe_sound_start_pending(&sound));
        waits=atomic_load(&main_waits);probe_sound_stop(&sound);(void)probe_sound_poll(&sound,now(NULL),NULL);
        CHECK(sound.play && probe_sound_busy(&sound) && atomic_load(&main_waits)==waits);
        atomic_store(&hold_hook,0);CHECK(idle(&sound) && sound.last_result==DBT_CANCELLED && !atomic_load(&live));
    }
    /* Preserve overlays and the latest-one queued notification. */
    uint32_t ticket;CHECK(probe_output_stream_begin(true,48000,2,DBT_PCM_S16_LE,&ticket)==DBT_PENDING && await_ticket(ticket)==DBT_OK);
    int16_t music[256];for(unsigned i=0;i<256;++i)music[i]=4000;
    for(unsigned i=0;i<32;++i){(void)probe_output_try_pcm_format(music,128,2,48000,DBT_PCM_S16_LE);Sleep(1);}
    CHECK(probe_sound_play(&sound,0,44100,true)==DBT_PENDING && start_finished(&sound) && sound.last_result==DBT_OK);
    CHECK(sound.rate==48000 && !sound.exclusive);
    CHECK(probe_sound_play(&sound,2,48000,true)==DBT_OK && probe_sound_play(&sound,3,48000,true)==DBT_OK);
    bool saw_last=false;
    for(unsigned i=0;i<5000 && probe_sound_busy(&sound);++i){
        (void)probe_sound_poll(&sound,now(NULL),NULL);(void)probe_output_try_pcm_format(music,128,2,48000,DBT_PCM_S16_LE);
        if(sound.type==3)saw_last=true;Sleep(1);
    }
    CHECK(saw_last && !probe_sound_busy(&sound) && probe_output_active_rate()==48000);
    CHECK(probe_output_stream_begin(false,0,0,DBT_PCM_S16_LE,&ticket)==DBT_PENDING && await_ticket(ticket)==DBT_OK && !atomic_load(&live));
    /* Failed DMA must retire a borrowed sound even when it cannot drain. */
    CHECK(probe_sound_play(&sound,0,48000,true)==DBT_PENDING && start_finished(&sound));
    for(unsigned i=0;i<5000 && !atomic_load(&i2s_enabled);++i){(void)probe_sound_poll(&sound,now(NULL),NULL);Sleep(1);}
    CHECK(atomic_load(&i2s_enabled));atomic_store(&bad_dma,true);
    CHECK(idle(&sound) && sound.last_result==DBT_INVALID && !atomic_load(&live));
    /* A codec start retires standalone sound before publishing its new rate. */
    CHECK(probe_sound_play(&sound,0,48000,true)==DBT_PENDING && start_finished(&sound));
    CHECK(probe_output_stream_begin(true,44100,2,DBT_PCM_S16_LE,&ticket)==DBT_PENDING);
    dbt_result_t result=DBT_PENDING;
    for(unsigned i=0;i<5000 && result==DBT_PENDING;++i){(void)probe_sound_poll(&sound,now(NULL),NULL);result=probe_output_request_poll(ticket);Sleep(1);}
    CHECK(result==DBT_OK && idle(&sound) && probe_output_active_rate()==44100);
    CHECK(probe_output_stream_begin(false,0,0,DBT_PCM_S16_LE,&ticket)==DBT_PENDING && await_ticket(ticket)==DBT_OK && !atomic_load(&live));
    CHECK(atomic_load(&blocking_sends)==sends);
    /* A malformed DMA event wakes the render owner without a periodic poll. */
    CHECK(probe_output_stream_begin(true,48000,2,DBT_PCM_S16_LE,&ticket)==DBT_PENDING && await_ticket(ticket)==DBT_OK);
    for(unsigned i=0;i<100 && !atomic_load(&i2s_enabled);++i){(void)probe_output_try_pcm_format(music,128,2,48000,DBT_PCM_S16_LE);Sleep(1);}
    CHECK(atomic_load(&i2s_enabled));unsigned irq_wakes=atomic_load(&isr_wakes);atomic_store(&bad_dma,true);
    probe_output_stats_t stats={0};
    for(unsigned i=0;i<5000 && !stats.failed;++i){probe_output_stats(&stats);Sleep(1);}
    CHECK(stats.failed && atomic_load(&isr_wakes)>irq_wakes);
    CHECK(probe_output_stream_begin(false,0,0,DBT_PCM_S16_LE,&ticket)==DBT_PENDING && await_ticket(ticket)==DBT_OK && !atomic_load(&live));
    puts("async sound client passed: pending ownership, cancellation/deadline, overlays/queue, codec takeover; mocked I2S");return 0;
}
