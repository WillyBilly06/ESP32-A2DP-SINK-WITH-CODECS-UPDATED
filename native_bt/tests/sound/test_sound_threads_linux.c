/* Independent POSIX producer and render threads for local sanitizer runs. */
#define main sound_fixture_main
#include "test_sound.c"
#undef main
#include <pthread.h>
#include <sched.h>
typedef struct {dbt_sound_play_t *play;_Atomic int failure;} thread_state;
static void *worker(void *arg){
    thread_state *s=arg;
    for(unsigned i=0;i<2000000;i++){
        dbt_result_t r=dbt_sound_play_pump(s->play,31);
        if(r==DBT_EMPTY)return NULL;
        if(r!=DBT_OK && r!=DBT_BUSY){atomic_store(&s->failure,1);return NULL;}
        sched_yield();
    }
    atomic_store(&s->failure,1);return NULL;
}
static void *render(void *arg){
    thread_state *s=arg;int32_t pcm[74];
    for(unsigned i=0;i<2000000;i++){
        if(dbt_sound_play_drained(s->play))return NULL;
        for(unsigned j=0;j<74;j++)pcm[j]=100000000;
        dbt_sound_mix(s->play,pcm,37);sched_yield();
    }
    atomic_store(&s->failure,1);return NULL;
}
int main(void){
    fixture_t f={0};dbt_sound_t sound;f.sound=&sound;f.peer=(dbt_sound_peer_t){8,12};f.valid=true;
    dbt_sound_store_t store={&f,begin,write_file,seal,read_temp,commit,abort_file,size_of,read_file,remove_file,mute,clock_ms,peer_valid};
    ASSERT(dbt_sound_init(&sound,store,false)==DBT_OK);
    uint8_t input[8192];uint32_t size=wav(input,16,2,44100,1500);ASSERT(upload(&f,0,input,size,400));
    dbt_sound_output_t output={&f,output_start,output_stop};
    for(unsigned i=0;i<64;i++){
        dbt_sound_play_t *play=0;ASSERT(dbt_sound_play_open(&sound,0,96000,false,output,&f,allocate,release,&play)==DBT_OK);
        thread_state state={.play=play};atomic_init(&state.failure,0);pthread_t a,b;
        ASSERT(!pthread_create(&a,NULL,worker,&state) && !pthread_create(&b,NULL,render,&state));
        ASSERT(!pthread_join(a,NULL) && !pthread_join(b,NULL));
        ASSERT(!atomic_load(&state.failure) && dbt_sound_play_drained(play));
        dbt_sound_play_close(play);ASSERT(!f.allocations);
    }
    puts("sound POSIX producer/render thread stress passed");return 0;
}
