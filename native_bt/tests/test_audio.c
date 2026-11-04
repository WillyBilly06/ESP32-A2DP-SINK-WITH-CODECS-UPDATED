#include "dbt_audio.h"
#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif
#define CHECK(x) do{if(!(x))return __LINE__;}while(0)
static dbt_audio_t audio;
static fixed_dsp_t reference;
static uint32_t storage[2][2048];
static bool used[2];
static unsigned allocations,releases,errors,fail_call;
static uint8_t *allocate(void *context,uint32_t bytes){
    (void)context;++allocations;if(fail_call==allocations || bytes>sizeof(storage[0]))return 0;
    for(unsigned i=0;i<2;++i)if(!used[i]){used[i]=true;return (uint8_t *)storage[i];}
    return 0;
}
static void release(void *context,uint8_t *bytes){
    (void)context;
    for(unsigned i=0;i<2;++i)if(bytes==(uint8_t *)storage[i]){if(!used[i])++errors;used[i]=false;++releases;return;}
    ++errors;
}
static bool fixture(dbt_pcm_format_t format,uint8_t channels){
    used[0]=used[1]=false;allocations=releases=errors=fail_call=0;
    dbt_audio_memory_t memory={0,allocate,release};
    if(dbt_audio_init(&audio,memory)!=DBT_OK)return false;
    dbt_audio_config_t config={48000,256,512,128,format,channels,false};
    return dbt_audio_open(&audio,config)==DBT_OK;
}
static void put(uint8_t *p,int32_t value,unsigned width){for(unsigned i=0;i<width;++i)p[i]=(uint8_t)((uint32_t)value>>(i*8));}
EXPORT int test_audio_formats_match_integer_dsp_reference(void){
    static uint8_t input[256*8];static int32_t output[512];
    for(unsigned width=2;width<=4;++width)for(unsigned channels=1;channels<=2;++channels){
        CHECK(fixture((dbt_pcm_format_t)width,(uint8_t)channels));
        CHECK(dbt_audio_controls(&audio,2,-1,3,64,FDSP_BYPASS)==DBT_OK);
        CHECK(fixed_dsp_init(&reference,48000,90,500,512));fixed_dsp_controls(&reference,0,0,0,0,0,true);
        fixed_dsp_controls(&reference,2,-1,3,64,FDSP_BYPASS,false);
        for(unsigned n=0;n<256;++n)for(unsigned ch=0;ch<channels;++ch){
            int32_t value=((int32_t)(n%64)-32)*(ch? -31:57);if(width==3)value*=256;if(width==4)value*=65536;
            put(input+(n*channels+ch)*width,value,width);
        }
        CHECK(dbt_audio_submit(&audio,input,256)==DBT_OK);
        CHECK(dbt_audio_render(&audio,256)==256 && dbt_audio_take(&audio,output,256)==256);
        for(unsigned n=0;n<256;++n){
            int32_t left=((int32_t)(n%64)-32)*57*65536;
            int32_t right=channels==1?left:((int32_t)(n%64)-32)*-31*65536;
            fixed_dsp_process(&reference,&left,&right);CHECK(output[2*n]==left && output[2*n+1]==right);
        }
        dbt_audio_close(&audio);CHECK(!used[0] && !used[1] && !errors && releases==2);
    }
    return 0;
}
EXPORT int test_audio_whole_blocks_wrap_and_silence_accounting(void){
    CHECK(fixture(DBT_PCM_S16_LE,2));static uint8_t data[256*4];int32_t output[600];
    atomic_store(&audio.input.written,UINT32_MAX-127u);atomic_store(&audio.input.read,UINT32_MAX-127u);
    atomic_store(&audio.output.written,UINT32_MAX-127u);atomic_store(&audio.output.read,UINT32_MAX-127u);
    CHECK(dbt_audio_submit(&audio,data,256)==DBT_OK);
    CHECK(dbt_audio_submit(&audio,data,1)==DBT_BUSY && atomic_load(&audio.submitted_frames)==256);
    CHECK(dbt_audio_render(&audio,256)==256 && dbt_audio_input_frames(&audio)==0 && dbt_audio_output_frames(&audio)==256);
    CHECK(dbt_audio_take(&audio,output,300)==256 && atomic_load(&audio.zero_frames)==44 && atomic_load(&audio.taken_frames)==256);
    for(unsigned i=0;i<600;++i)CHECK(output[i]==0);
    dbt_audio_close(&audio);CHECK(!errors);return 0;
}
EXPORT int test_audio_oom_cleanup_and_control_snapshot(void){
    CHECK(fixture(DBT_PCM_S16_LE,1));dbt_audio_config_t config=audio.config;
    fail_call=allocations+2;CHECK(dbt_audio_open(&audio,config)==DBT_NO_SPACE && !audio.opened && !used[0] && !used[1]);
    fail_call=0;CHECK(dbt_audio_open(&audio,config)==DBT_OK);
    CHECK(dbt_audio_controls(&audio,13,0,0,6,0)==DBT_INVALID);
    atomic_store(&audio.controls_lock,1);CHECK(dbt_audio_controls(&audio,1,2,3,64,0)==DBT_BUSY);atomic_store(&audio.controls_lock,0);
    CHECK(dbt_audio_controls(&audio,-12,12,-3,40,FDSP_CHANNEL_FLIP)==DBT_OK);
    int16_t input[1]={0};CHECK(dbt_audio_submit(&audio,input,1)==DBT_OK && dbt_audio_render(&audio,1)==1);
    CHECK(audio.dsp.gains[0]==-12 && audio.dsp.gains[1]==12 && audio.dsp.gains[2]==-3 && audio.dsp.requested_volume==40);
    dbt_audio_close(&audio);CHECK(!errors);return 0;
}
EXPORT int test_audio_adaptive_fraction_bounds_and_clock_limit(void){
    CHECK(fixture(DBT_PCM_S32_LE,2));audio.config.adaptive_clock=true;
    static int32_t input[512];for(unsigned i=0;i<512;++i)input[i]=(i&1u)?INT32_MIN:INT32_MAX;
    CHECK(dbt_audio_submit(&audio,input,256)==DBT_OK);
    for(unsigned i=0;i<1000;++i)dbt_audio_clock(&audio);
    CHECK(audio.step_adjust<=2147483 && audio.step_adjust>=-2147483);
    audio.fraction=UINT32_MAX;audio.step_adjust=2147483;
    CHECK(dbt_audio_render(&audio,128)==128);
    int32_t output[256];CHECK(dbt_audio_take(&audio,output,128)==128);
    dbt_audio_close(&audio);CHECK(!errors);return 0;
}
EXPORT int test_audio_consumer_fade_exact_duration_and_full_scale(void){
    dbt_audio_fade_t fade;dbt_audio_fade_init(&fade,960);int32_t previous_l=INT32_MAX,previous_r=INT32_MIN;
    for(unsigned i=0;i<1000;++i){
        int32_t pair[2]={INT32_MAX,INT32_MIN};dbt_audio_fade(&fade,pair,1);
        CHECK(pair[0]<=previous_l && pair[1]>=previous_r && pair[0]>=0 && pair[1]<=0);
        if(i==0)CHECK(pair[0]==INT32_MAX && pair[1]==INT32_MIN);
        if(i>=960)CHECK(pair[0]==0 && pair[1]==0);
        previous_l=pair[0];previous_r=pair[1];
    }
    CHECK(fade.remaining==0 && fade.gain==0);return 0;
}
static uint32_t consumed_sequence;
static void source_frame(uint32_t sequence,int32_t *left,int32_t *right){
    *left=((int32_t)(sequence%128u)-64)*1048576;
    *right=(63-(int32_t)(sequence%128u))*524288;
}
EXPORT int audio_stress_begin(void){
    if(!fixture(DBT_PCM_S32_LE,2))return 1;
    if(dbt_audio_controls(&audio,0,0,0,127,FDSP_BYPASS)!=DBT_OK)return 2;
    (void)fixed_dsp_init(&reference,48000,90,500,512);fixed_dsp_controls(&reference,0,0,0,0,0,true);
    fixed_dsp_controls(&reference,0,0,0,127,FDSP_BYPASS,false);consumed_sequence=0;
    atomic_store(&audio.input.written,UINT32_MAX-1023u);atomic_store(&audio.input.read,UINT32_MAX-1023u);
    atomic_store(&audio.output.written,UINT32_MAX-1023u);atomic_store(&audio.output.read,UINT32_MAX-1023u);
    return 0;
}
EXPORT int audio_stress_produce(uint32_t sequence){
    int32_t samples[128];for(unsigned i=0;i<64;++i)source_frame(sequence+i,&samples[2*i],&samples[2*i+1u]);
    return (int)dbt_audio_submit(&audio,samples,64);
}
EXPORT uint32_t audio_stress_render(void){return dbt_audio_render(&audio,128);}
EXPORT uint32_t audio_stress_rendered(void){return atomic_load(&audio.rendered_frames);}
EXPORT int audio_stress_consume(void){
    if(dbt_audio_output_frames(&audio)<64)return DBT_EMPTY;
    int32_t actual[128];if(dbt_audio_take(&audio,actual,64)!=64)return DBT_INVALID;
    for(unsigned i=0;i<64;++i){
        int32_t left,right;source_frame(consumed_sequence++,&left,&right);fixed_dsp_process(&reference,&left,&right);
        if(actual[2*i]!=left || actual[2*i+1u]!=right)return DBT_INVALID;
    }
    return DBT_OK;
}
EXPORT int audio_stress_end(void){
    int result=atomic_load(&audio.zero_frames)==0 && atomic_load(&audio.taken_frames)==16384 &&
        atomic_load(&audio.submitted_frames)==16384?0:1;
    dbt_audio_close(&audio);return result || errors || used[0] || used[1];
}
EXPORT int audio_snapshot_begin(void){
    if(!fixture(DBT_PCM_S16_LE,2))return 1;
    if(dbt_audio_controls(&audio,0,0,0,64,FDSP_BYPASS)!=DBT_OK)return 2;
    (void)dbt_audio_render(&audio,0);return 0;
}
EXPORT int audio_snapshot_write(int gain){return (int)dbt_audio_controls(&audio,(int8_t)gain,(int8_t)-gain,(int8_t)gain,(uint8_t)(64+gain),FDSP_BYPASS);}
EXPORT int audio_snapshot_read(void){
    (void)dbt_audio_render(&audio,0);int gain=audio.dsp.gains[0];
    return audio.dsp.gains[1]!=-gain || audio.dsp.gains[2]!=gain || audio.dsp.requested_volume!=64+gain;
}
EXPORT int audio_snapshot_end(void){dbt_audio_close(&audio);return errors || used[0] || used[1];}
