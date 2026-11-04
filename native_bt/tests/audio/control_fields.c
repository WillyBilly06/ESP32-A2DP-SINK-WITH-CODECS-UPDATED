#include "dbt_audio.h"
#include <limits.h>
#include <string.h>
#ifdef _WIN32
#define EX __declspec(dllexport)
#else
#define EX __attribute__((visibility("default")))
#endif
#define CHECK(x) do{if(!(x))return __LINE__;}while(0)
static dbt_audio_t audio;
static fixed_dsp_t reference;
static uint32_t memory[2][2048];
static bool in_use[2];
static unsigned failures;
static uint8_t *allocate(void *ctx,uint32_t n){(void)ctx;if(n>sizeof(memory[0]))return 0;for(unsigned i=0;i<2;i++)if(!in_use[i]){in_use[i]=true;return (uint8_t *)memory[i];}return 0;}
static void release(void *ctx,uint8_t *p){(void)ctx;for(unsigned i=0;i<2;i++)if(p==(uint8_t *)memory[i]){if(!in_use[i])failures++;in_use[i]=false;return;}failures++;}
static int fixture(void){
    memset(in_use,0,sizeof(in_use));failures=0;
    CHECK(dbt_audio_init(&audio,(dbt_audio_memory_t){0,allocate,release})==DBT_OK);
    CHECK(dbt_audio_open(&audio,(dbt_audio_config_t){48000,256,512,128,DBT_PCM_S32_LE,2,false})==DBT_OK);
    CHECK(fixed_dsp_init(&reference,48000,90,500,512));fixed_dsp_controls(&reference,0,0,0,0,0,true);return 0;
}
static int rendered(int8_t bass,int8_t mid,int8_t treble,uint8_t volume,uint8_t flags){
    int32_t input[128],output[128];
    for(unsigned i=0;i<64;i++){input[2*i]=(int32_t)((int)i-32)*1000000;input[2*i+1]=(int32_t)(31-(int)i)*700000;}
    fixed_dsp_controls(&reference,bass,mid,treble,volume,flags,false);
    CHECK(dbt_audio_submit(&audio,input,64)==DBT_OK && dbt_audio_render(&audio,64)==64 && dbt_audio_take(&audio,output,64)==64);
    for(unsigned i=0;i<64;i++){int32_t l=input[2*i],r=input[2*i+1];fixed_dsp_process(&reference,&l,&r);CHECK(output[2*i]==l && output[2*i+1]==r);}
    CHECK(audio.dsp.gains[0]==bass && audio.dsp.gains[1]==mid && audio.dsp.gains[2]==treble && audio.dsp.requested_volume==volume && audio.dsp.flags==flags);
    return 0;
}
EX int control_fields_rendered(void){
    int r=fixture();if(r)return r;
    CHECK(dbt_audio_controls(&audio,3,-2,1,45,FDSP_ANALYSIS|FDSP_BASS_BOOST)==DBT_OK);
    r=rendered(3,-2,1,45,FDSP_ANALYSIS|FDSP_BASS_BOOST);if(r)return r;
    CHECK(dbt_audio_eq(&audio,-4,5,-6)==DBT_OK);
    r=rendered(-4,5,-6,45,FDSP_ANALYSIS|FDSP_BASS_BOOST);if(r)return r;
    CHECK(dbt_audio_volume(&audio,64)==DBT_OK);
    r=rendered(-4,5,-6,64,FDSP_ANALYSIS|FDSP_BASS_BOOST);if(r)return r;
    CHECK(dbt_audio_control_flags(&audio,FDSP_CHANNEL_FLIP|FDSP_BYPASS)==DBT_OK);
    r=rendered(-4,5,-6,64,FDSP_ANALYSIS|FDSP_CHANNEL_FLIP|FDSP_BYPASS);if(r)return r;
    CHECK(dbt_audio_analysis(&audio,false)==DBT_OK);
    r=rendered(-4,5,-6,64,FDSP_CHANNEL_FLIP|FDSP_BYPASS);if(r)return r;
    CHECK(dbt_audio_analysis(&audio,true)==DBT_OK);
    r=rendered(-4,5,-6,64,FDSP_ANALYSIS|FDSP_CHANNEL_FLIP|FDSP_BYPASS);if(r)return r;
    dbt_audio_close(&audio);CHECK(!in_use[0] && !in_use[1] && !failures);return 0;
}
EX int control_invalid_busy_generation(void){
    int r=fixture();if(r)return r;
    CHECK(dbt_audio_controls(&audio,1,2,3,44,9)==DBT_OK);
    uint32_t generation=atomic_load(&audio.controls_generation),gains=atomic_load(&audio.controls_gains),options=atomic_load(&audio.controls_options);
    CHECK(dbt_audio_eq(&audio,13,0,0)==DBT_INVALID && dbt_audio_eq(0,0,0,0)==DBT_INVALID);
    CHECK(dbt_audio_volume(&audio,128)==DBT_INVALID && dbt_audio_volume(0,1)==DBT_INVALID);
    CHECK(dbt_audio_control_flags(&audio,8)==DBT_INVALID && dbt_audio_control_flags(0,0)==DBT_INVALID && dbt_audio_analysis(0,true)==DBT_INVALID);
    CHECK(dbt_audio_controls(&audio,0,0,-13,0,0)==DBT_INVALID && atomic_load(&audio.controls_generation)==generation);
    atomic_store(&audio.controls_lock,1);
    CHECK(dbt_audio_controls(&audio,0,0,0,0,0)==DBT_BUSY && dbt_audio_volume(&audio,1)==DBT_BUSY && dbt_audio_eq(&audio,0,0,0)==DBT_BUSY);
    CHECK(dbt_audio_control_flags(&audio,0)==DBT_BUSY && dbt_audio_analysis(&audio,false)==DBT_BUSY);
    CHECK(atomic_load(&audio.controls_generation)==generation && atomic_load(&audio.controls_gains)==gains && atomic_load(&audio.controls_options)==options);
    atomic_store(&audio.controls_lock,0);
    atomic_store(&audio.controls_generation,UINT32_MAX-1u);audio.applied_generation=UINT32_MAX-1u;
    CHECK(dbt_audio_volume(&audio,77)==DBT_OK && atomic_load(&audio.controls_generation)==0);
    (void)dbt_audio_render(&audio,0);CHECK(audio.dsp.requested_volume==77 && audio.applied_generation==0);
    dbt_audio_close(&audio);CHECK(!in_use[0] && !in_use[1] && !failures);return 0;
}
#ifdef DBT_AUDIO_CONTROL_TEST_IRQ
static uint32_t mask_level,masked_calls,restore_calls;static bool arm_interrupt,pending_interrupt,preempted_with_lock;
static int interrupt_result;static dbt_audio_t *interrupt_audio;
uint32_t dbt_audio_test_mask_enter(void){uint32_t before=mask_level;if(mask_level<3)mask_level=3;masked_calls++;return before;}
void dbt_audio_test_publication_step(dbt_audio_t *a){
    /* Other-core reader can run while this writer is between generation stores. */
    int8_t old=a->dsp.gains[0];(void)dbt_audio_render(a,0);if(a->dsp.gains[0]!=old)failures++;
    if(arm_interrupt){arm_interrupt=false;interrupt_audio=a;
        if(mask_level<3){preempted_with_lock=true;interrupt_result=dbt_audio_volume(a,76);}
        else pending_interrupt=true;}
}
void dbt_audio_test_mask_exit(uint32_t before){
    mask_level=before;restore_calls++;
    if(!mask_level && pending_interrupt){pending_interrupt=false;
        if(atomic_load(&interrupt_audio->controls_lock))preempted_with_lock=true;
        interrupt_result=dbt_audio_volume(interrupt_audio,76);}
}
EX int control_preemption_model(void){
    int r=fixture();if(r)return r;mask_level=masked_calls=restore_calls=0;arm_interrupt=true;pending_interrupt=preempted_with_lock=false;interrupt_result=-1;
    CHECK(dbt_audio_eq(&audio,7,-3,5)==DBT_OK && interrupt_result==DBT_OK && !pending_interrupt && !preempted_with_lock);
    CHECK(mask_level==0 && masked_calls==2 && restore_calls==2 && atomic_load(&audio.controls_generation)==4);
    (void)dbt_audio_render(&audio,0);CHECK(audio.dsp.gains[0]==7 && audio.dsp.gains[1]==-3 && audio.dsp.gains[2]==5 && audio.dsp.requested_volume==76);
    mask_level=5;CHECK(dbt_audio_analysis(&audio,true)==DBT_OK && mask_level==5);mask_level=0;
    atomic_store(&audio.controls_lock,1);unsigned enters=masked_calls,exits=restore_calls;
    CHECK(dbt_audio_volume(&audio,3)==DBT_BUSY && mask_level==0 && masked_calls==enters+1 && restore_calls==exits+1);
    atomic_store(&audio.controls_lock,0);dbt_audio_close(&audio);CHECK(!failures);return 0;
}
#endif
EX int control_race_begin(void){return fixture();}
EX int control_race_eq(void){return dbt_audio_eq(&audio,-7,8,-9);}
EX int control_race_volume(void){return dbt_audio_volume(&audio,89);}
EX int control_race_flags(void){return dbt_audio_control_flags(&audio,5);}
EX int control_race_analysis(void){return dbt_audio_analysis(&audio,true);}
EX int control_race_end(void){
    CHECK(atomic_load(&audio.controls_generation)==8000 && !atomic_load(&audio.controls_lock));
    (void)dbt_audio_render(&audio,0);CHECK(audio.dsp.gains[0]==-7 && audio.dsp.gains[1]==8 && audio.dsp.gains[2]==-9 && audio.dsp.requested_volume==89 && audio.dsp.flags==(FDSP_ANALYSIS|5));
    dbt_audio_close(&audio);CHECK(!in_use[0] && !in_use[1] && !failures);return 0;
}
