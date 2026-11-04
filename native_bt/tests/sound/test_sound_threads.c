/* The real SPSC worker/render path, with independent Windows threads. */
#define main sound_fixture_main
#include "test_sound.c"
#undef main
#include <windows.h>
typedef struct {dbt_sound_play_t *play;_Atomic int failure;} thread_state;
static DWORD WINAPI worker(LPVOID arg){
    thread_state *s=arg;
    for(unsigned i=0;i<2000000;i++){
        dbt_result_t r=dbt_sound_play_pump(s->play,31);
        if(r==DBT_EMPTY)return 0;
        if(r!=DBT_OK && r!=DBT_BUSY){atomic_store(&s->failure,1);return 1;}
        SwitchToThread();
    }
    atomic_store(&s->failure,1);return 1;
}
static DWORD WINAPI render(LPVOID arg){
    thread_state *s=arg;int32_t pcm[74];
    for(unsigned i=0;i<2000000;i++){
        if(dbt_sound_play_drained(s->play))return 0;
        for(unsigned j=0;j<74;j++)pcm[j]=100000000;
        dbt_sound_mix(s->play,pcm,37);SwitchToThread();
    }
    atomic_store(&s->failure,1);return 1;
}
int main(void){
    fixture_t f={0};dbt_sound_t sound;f.sound=&sound;f.peer=(dbt_sound_peer_t){8,12};f.valid=true;
    dbt_sound_store_t store={&f,begin,write_file,seal,read_temp,commit,abort_file,size_of,read_file,remove_file,mute,clock_ms,peer_valid};
    ASSERT(dbt_sound_init(&sound,store,false)==DBT_OK);
    uint8_t input[8192];uint32_t size=wav(input,16,2,44100,1500);ASSERT(upload(&f,0,input,size,400));
    dbt_sound_output_t output={&f,output_start,output_stop};
    for(unsigned i=0;i<64;i++){
        dbt_sound_play_t *play=0;ASSERT(dbt_sound_play_open(&sound,0,96000,false,output,&f,allocate,release,&play)==DBT_OK);
        thread_state state={.play=play};atomic_init(&state.failure,0);
        HANDLE a=CreateThread(NULL,0,worker,&state,0,NULL),b=CreateThread(NULL,0,render,&state,0,NULL);
        ASSERT(a && b);ASSERT(WaitForSingleObject(a,10000)==WAIT_OBJECT_0 && WaitForSingleObject(b,10000)==WAIT_OBJECT_0);
        CloseHandle(a);CloseHandle(b);ASSERT(!atomic_load(&state.failure) && dbt_sound_play_drained(play));
        dbt_sound_play_close(play);ASSERT(!f.allocations);
    }
    puts("sound producer/render thread stress passed");return 0;
}
