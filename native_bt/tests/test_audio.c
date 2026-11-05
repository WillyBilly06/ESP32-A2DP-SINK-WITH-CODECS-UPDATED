#include "dbt_audio.h"
#if defined(__has_feature)
#if __has_feature(address_sanitizer)
#include <sanitizer/asan_interface.h>
#define PAGE_POISON(p,n) __asan_poison_memory_region(p,n)
#define PAGE_UNPOISON(p,n) __asan_unpoison_memory_region(p,n)
#endif
#endif
#ifndef PAGE_POISON
#define PAGE_POISON(p,n) ((void)(p),(void)(n))
#define PAGE_UNPOISON(p,n) ((void)(p),(void)(n))
#endif
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
static bool unaligned_input;
static uint8_t *allocate(void *context,uint32_t bytes){
    (void)context;++allocations;if(fail_call==allocations || bytes>sizeof(storage[0]))return 0;
    for(unsigned i=0;i<2;++i)if(!used[i]){
        unsigned offset=unaligned_input && !i?1u:0u;if(bytes>sizeof(storage[i])-offset)return 0;
        used[i]=true;return (uint8_t *)storage[i]+offset;
    }
    return 0;
}
static void release(void *context,uint8_t *bytes){
    (void)context;
    for(unsigned i=0;i<2;++i)if(bytes==(uint8_t *)storage[i]+(unaligned_input && !i?1u:0u)){if(!used[i])++errors;used[i]=false;++releases;return;}
    ++errors;
}
static bool fixture_aligned(dbt_pcm_format_t format,uint8_t channels,bool unaligned){
    used[0]=used[1]=false;allocations=releases=errors=fail_call=0;
    unaligned_input=unaligned;
    dbt_audio_memory_t memory={0,allocate,release};
    if(dbt_audio_init(&audio,memory)!=DBT_OK)return false;
    dbt_audio_config_t config={48000,256,512,128,format,channels,false};
    return dbt_audio_open(&audio,config)==DBT_OK;
}
static bool fixture(dbt_pcm_format_t format,uint8_t channels){return fixture_aligned(format,channels,false);}
static void put(uint8_t *p,int32_t value,unsigned width){for(unsigned i=0;i<width;++i)p[i]=(uint8_t)((uint32_t)value>>(i*8));}
/* Model the measured ESP32 constraint: plenty of total SRAM, but no allocation
 * above 32 KiB. No allocator fallback/retry is permitted. */
static uint64_t page_storage[DBT_AUDIO_OUTPUT_PAGES+1u][4096];
static bool page_used[DBT_AUDIO_OUTPUT_PAGES+1u];
static unsigned page_calls,page_releases,page_fail,page_errors;
static uint8_t *page_allocate(void *context,uint32_t bytes){
    (void)context;++page_calls;
    if(page_calls==page_fail || bytes>sizeof(page_storage[0]))return 0;
    for(unsigned i=0;i<DBT_AUDIO_OUTPUT_PAGES+1u;++i)if(!page_used[i]){
        page_used[i]=true;
        /* Give ASan the actual requested extent, even though the fixed host
         * arena reserves a larger slot. Detect intra-slot page overruns too. */
        PAGE_POISON((uint8_t *)page_storage[i]+bytes,sizeof(page_storage[i])-bytes);
        return (uint8_t *)page_storage[i];
    }
    return 0;
}
static void page_release(void *context,uint8_t *bytes){
    (void)context;
    for(unsigned i=0;i<DBT_AUDIO_OUTPUT_PAGES+1u;++i)if(bytes==(uint8_t *)page_storage[i]){
        if(!page_used[i])++page_errors;
        PAGE_UNPOISON(page_storage[i],sizeof(page_storage[i]));
        page_used[i]=false;++page_releases;return;
    }
    ++page_errors;
}
static void page_reset(unsigned fail){
    for(unsigned i=0;i<DBT_AUDIO_OUTPUT_PAGES+1u;++i)page_used[i]=false;
    page_calls=page_releases=page_errors=0;page_fail=fail;
}
static void page_mix(void *context,int32_t *pcm,uint32_t frames){
    uint32_t *position=context;
    for(uint32_t i=0;i<frames;++i){int32_t n=(int32_t)(++*position);pcm[2*i]=n*17;pcm[2*i+1u]=-n*31;}
}
EXPORT int test_audio_paged_ring_capacity_wrap_mix_and_oom(void){
    static dbt_audio_t a;dbt_audio_memory_t memory={0,page_allocate,page_release};
    dbt_audio_config_t config={192000,4096,32768,12288,DBT_PCM_S32_LE,2,true};
    for(unsigned fail=1;fail<=DBT_AUDIO_OUTPUT_PAGES+1u;++fail){
        page_reset(fail);CHECK(dbt_audio_init(&a,memory)==DBT_OK);
        CHECK(dbt_audio_open(&a,config)==DBT_NO_SPACE && !a.opened && !a.allocated_bytes && !a.input.bytes && !a.output.bytes);
        CHECK(page_calls==fail && page_releases==fail-1u && !page_errors);
        for(unsigned i=0;i<DBT_AUDIO_OUTPUT_PAGES+1u;++i)CHECK(!page_used[i]);
        for(unsigned i=0;i<DBT_AUDIO_OUTPUT_PAGES;++i)CHECK(!a.output_pages[i]);
    }
    const uint32_t sizes[]={2,7,511,512,513,3584,4096,6656,8192,12800,16384,32767,32768};
    for(unsigned shape=0;shape<sizeof(sizes)/sizeof(sizes[0]);++shape){
        uint32_t size=sizes[shape];
        page_reset(0);config.output_frames=size;config.target_frames=size/2;
        CHECK(dbt_audio_init(&a,memory)==DBT_OK && dbt_audio_open(&a,config)==DBT_OK);
        CHECK(a.allocated_bytes==32768u+size*8u);
        /* Near a page boundary and uint32 counter wrap, then a whole ring. */
        uint32_t position=0,start=UINT32_MAX-5u;
        atomic_store(&a.output.written,start);atomic_store(&a.output.read,start);
        /* Physical wrap and counter wrap deliberately occur at different
         * samples. A non-power-of-two ring cannot derive one from the other. */
        a.output_write_slot=a.output_read_slot=size>129u?size-129u:0;
        dbt_audio_set_mix(&a,page_mix,&position);
        CHECK(dbt_audio_render_silence(&a,size)==size && position==size);
        CHECK(dbt_audio_render_silence(&a,1)==0);
        for(uint32_t consumed=0;consumed<size;){
            uint32_t count=size-consumed;if(count>128u)count=128u;
            int32_t pcm[256];CHECK(dbt_audio_take(&a,pcm,count)==count);
            for(unsigned i=0;i<count;++i){int32_t n=(int32_t)(consumed+i+1u);CHECK(pcm[2*i]==n*17 && pcm[2*i+1u]==-n*31);}
            consumed+=count;
        }
        int32_t silence[4]={1,1,1,1};CHECK(dbt_audio_take(&a,silence,2)==0);
        CHECK(!silence[0] && !silence[1] && !silence[2] && !silence[3] && atomic_load(&a.zero_frames)==2);
        dbt_audio_close(&a);CHECK(page_releases==page_calls && !page_errors);
        for(unsigned i=0;i<DBT_AUDIO_OUTPUT_PAGES+1u;++i)CHECK(!page_used[i]);
    }
    return 0;
}
EXPORT int test_audio_paged_render_matches_small_ring(void){
    static dbt_audio_t large,small;static uint8_t pcm[256*8];static int32_t actual[512],expected[512];
    dbt_audio_memory_t memory={0,page_allocate,page_release};
    const uint32_t capacities[]={513,3584,6656,12800,16384};
    for(unsigned shape=0;shape<sizeof(capacities)/sizeof(capacities[0]);++shape)
    for(unsigned width=2;width<=4;++width)for(unsigned channels=1;channels<=2;++channels){
        uint32_t capacity=capacities[shape];
        page_reset(0);dbt_audio_config_t config={192000,256,capacity,capacity/2u,(dbt_pcm_format_t)width,(uint8_t)channels,true};
        CHECK(dbt_audio_init(&large,memory)==DBT_OK && dbt_audio_init(&small,memory)==DBT_OK);
        CHECK(dbt_audio_open(&large,config)==DBT_OK);config.output_frames=512;config.target_frames=256;
        CHECK(dbt_audio_open(&small,config)==DBT_OK);
        CHECK(dbt_audio_controls(&large,3,-2,2,80,9)==DBT_OK && dbt_audio_controls(&small,3,-2,2,80,9)==DBT_OK);
        atomic_store(&large.output.written,4090u);atomic_store(&large.output.read,4090u);
        large.output_write_slot=large.output_read_slot=capacity-6u;
        for(unsigned block=0;block<160;++block){
            for(unsigned i=0;i<256*channels;++i){int32_t value=((int32_t)((block*256u+i)%65536u)-32768)*3;put(pcm+i*width,value,width);}
            CHECK(dbt_audio_submit(&large,pcm,256)==DBT_OK && dbt_audio_submit(&small,pcm,256)==DBT_OK);
            CHECK(dbt_audio_render(&large,256)==256 && dbt_audio_render(&small,256)==256);
            CHECK(dbt_audio_take(&large,actual,256)==256 && dbt_audio_take(&small,expected,256)==256);
            for(unsigned i=0;i<512;++i)CHECK(actual[i]==expected[i]);
        }
        dbt_audio_close(&large);dbt_audio_close(&small);CHECK(page_releases==page_calls && !page_errors);
    }
    return 0;
}
EXPORT int test_audio_formats_match_integer_dsp_reference(void){
    static uint8_t input[256*8];static int32_t output[512];
    for(unsigned width=2;width<=4;++width)for(unsigned channels=1;channels<=2;++channels)for(unsigned unaligned=0;unaligned<2;++unaligned){
        CHECK(fixture_aligned((dbt_pcm_format_t)width,(uint8_t)channels,unaligned!=0));
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
    page_reset(0);dbt_audio_memory_t memory={0,page_allocate,page_release};
    dbt_audio_config_t config={48000,256,8192,4096,DBT_PCM_S32_LE,2,false};
    if(dbt_audio_init(&audio,memory)!=DBT_OK || dbt_audio_open(&audio,config)!=DBT_OK)return 1;
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
    dbt_audio_close(&audio);
    if(page_errors || page_calls!=page_releases)result=1;
    for(unsigned i=0;i<DBT_AUDIO_OUTPUT_PAGES+1u;++i)if(page_used[i])result=1;
    return result;
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
