/* Actual paged-ring resize and PCM ordering; owners are quiesced explicitly.
 * SDK/IRQ timing and target concurrency are outside this host fixture. */
#include <stdio.h>
#include <string.h>
#include "../test_audio.c"

static int check_sequence(dbt_audio_t *a,uint32_t count,uint32_t *next){
    int32_t pcm[192*2];
    while(count){
        uint32_t n=count>192u?192u:count;CHECK(dbt_audio_take(a,pcm,n)==n);
        for(uint32_t i=0;i<n;++i){int32_t v=(int32_t)(++*next);CHECK(pcm[2*i]==v*17 && pcm[2*i+1u]==-v*31);}
        count-=n;
    }
    return 0;
}
static int resize_order(void){
    static dbt_audio_t a;dbt_audio_memory_t memory={0,page_allocate,page_release};
    const uint32_t starts[]={0,37,2048,2559};
    for(unsigned shape=0;shape<sizeof(starts)/sizeof(starts[0]);++shape){
        page_reset(0);CHECK(dbt_audio_init(&a,memory)==DBT_OK);
        dbt_audio_config_t config={192000,1024,2560,2048,DBT_PCM_S32_LE,2,false};
        CHECK(dbt_audio_open(&a,config)==DBT_OK);uint32_t base_bytes=a.allocated_bytes;
        uint32_t position=0,next=0;dbt_audio_set_mix(&a,page_mix,&position);
        a.output_read_slot=a.output_write_slot=starts[shape];
        atomic_store(&a.output.read,UINT32_MAX-40u);atomic_store(&a.output.written,UINT32_MAX-40u);
        CHECK(dbt_audio_render_silence(&a,900)==900);
        uint8_t *extra[3];for(unsigned i=0;i<3;++i){extra[i]=page_allocate(0,4096);CHECK(extra[i]);memset(extra[i],0xa5,4096);}
        dbt_result_t rc;unsigned attempts=0;
        do{
            dbt_audio_t before=a;rc=dbt_audio_output_extend(&a,extra,3);++attempts;
            if(rc==DBT_BUSY){CHECK(!memcmp(&a,&before,sizeof a));CHECK(check_sequence(&a,1,&next)==0);}
            CHECK(attempts<=2561);
        }while(rc==DBT_BUSY);
        CHECK(rc==DBT_OK && a.output.frames==4096 && a.allocated_bytes==base_bytes+12288);
        CHECK(dbt_audio_render_silence(&a,2048)==2048);
        uint8_t *detached[3]={0};uint32_t n=99;attempts=0;
        do{
            dbt_audio_t before=a;rc=dbt_audio_output_trim(&a,2560,detached,3,&n);++attempts;
            if(rc==DBT_BUSY){CHECK(!n && !memcmp(&a,&before,sizeof a));CHECK(check_sequence(&a,1,&next)==0);}
            CHECK(attempts<=4097);
        }while(rc==DBT_BUSY);
        CHECK(rc==DBT_OK && n==3 && a.allocated_bytes==base_bytes && a.output.frames==2560);
        for(unsigned i=0;i<3;++i){CHECK(detached[i]==extra[i] && !a.output_pages[5+i]);page_release(0,detached[i]);}
        CHECK(check_sequence(&a,dbt_audio_output_frames(&a),&next)==0 && next==position);
        /* Continue normal production after both physical and counter wraps. */
        for(unsigned i=0;i<30;++i){CHECK(dbt_audio_render_silence(&a,128)==128);CHECK(check_sequence(&a,128,&next)==0);}
        CHECK(next==position && !page_errors);dbt_audio_close(&a);CHECK(page_calls==page_releases && !page_errors);
    }
    return 0;
}
static int resize_guards(void){
    static dbt_audio_t a;dbt_audio_memory_t memory={0,page_allocate,page_release};
    page_reset(0);CHECK(dbt_audio_init(&a,memory)==DBT_OK);
    dbt_audio_config_t config={192000,1024,2560,2048,DBT_PCM_S32_LE,2,false};CHECK(dbt_audio_open(&a,config)==DBT_OK);
    uint8_t *extra[3];for(unsigned i=0;i<3;++i){extra[i]=page_allocate(0,4096);CHECK(extra[i]);}
    dbt_audio_t before=a;uint8_t *bad[3]={extra[0],extra[0],extra[2]};
    CHECK(dbt_audio_output_extend(&a,bad,3)==DBT_INVALID && !memcmp(&a,&before,sizeof a));
    bad[1]=0;CHECK(dbt_audio_output_extend(&a,bad,3)==DBT_INVALID && !memcmp(&a,&before,sizeof a));
    bad[1]=extra[1]+1;CHECK(dbt_audio_output_extend(&a,bad,3)==DBT_INVALID && !memcmp(&a,&before,sizeof a));
    bad[1]=a.output_pages[0];CHECK(dbt_audio_output_extend(&a,bad,3)==DBT_INVALID && !memcmp(&a,&before,sizeof a));
    a.render_pending=1;before=a;CHECK(dbt_audio_output_extend(&a,extra,3)==DBT_BUSY && !memcmp(&a,&before,sizeof a));a.render_pending=0;
    uint32_t saved=a.output.frames;a.output.frames=a.config.output_frames=2559;before=a;
    CHECK(dbt_audio_output_extend(&a,extra,3)==DBT_UNSUPPORTED && !memcmp(&a,&before,sizeof a));a.output.frames=a.config.output_frames=saved;
    atomic_store(&a.output.written,2561);before=a;CHECK(dbt_audio_output_extend(&a,extra,3)==DBT_INVALID && !memcmp(&a,&before,sizeof a));atomic_store(&a.output.written,0);
    CHECK(dbt_audio_output_extend(&a,extra,DBT_AUDIO_OUTPUT_PAGES)==DBT_NO_SPACE);
    CHECK(dbt_audio_output_extend(&a,extra,3)==DBT_OK);
    uint8_t *removed[3]={0};uint32_t n=44;before=a;
    CHECK(dbt_audio_output_trim(&a,2560,removed,2,&n)==DBT_NO_SPACE && !n && !memcmp(&a,&before,sizeof a));
    CHECK(dbt_audio_output_trim(&a,2048,removed,3,&n)==DBT_INVALID && !n && !memcmp(&a,&before,sizeof a));
    a.output_read_slot=a.output_write_slot=3000;uint32_t position=0,next=0;dbt_audio_set_mix(&a,page_mix,&position);
    CHECK(dbt_audio_render_silence(&a,1000)==1000);before=a;
    CHECK(dbt_audio_output_trim(&a,2560,removed,3,&n)==DBT_BUSY && !n && !memcmp(&a,&before,sizeof a));
    CHECK(check_sequence(&a,1000,&next)==0);CHECK(a.output_read_slot==4000 && a.output_write_slot==4000);
    CHECK(dbt_audio_output_trim(&a,2560,removed,3,&n)==DBT_OK && n==3 && !a.output_read_slot && !a.output_write_slot);
    for(unsigned i=0;i<3;++i)page_release(0,removed[i]);dbt_audio_close(&a);CHECK(page_calls==page_releases && !page_errors);
    /* Close, rather than trim, owns and frees all still-attached pages. */
    page_reset(0);CHECK(dbt_audio_init(&a,memory)==DBT_OK && dbt_audio_open(&a,config)==DBT_OK);
    for(unsigned i=0;i<3;++i){extra[i]=page_allocate(0,4096);CHECK(extra[i]);}
    CHECK(dbt_audio_output_extend(&a,extra,3)==DBT_OK);dbt_audio_close(&a);CHECK(page_calls==page_releases && !page_errors);
    return 0;
}
int main(void){
    int a=resize_order(),b=resize_guards(),c=test_audio_paged_ring_capacity_wrap_mix_and_oom(),d=test_audio_paged_render_matches_small_ring(),e=test_audio_formats_match_integer_dsp_reference();
    printf("resize PCM/counter/physical wrap=%d ownership/refusal/close=%d existing page/OOM=%d render=%d formats=%d\n",a,b,c,d,e);
    return a||b||c||d||e;
}
