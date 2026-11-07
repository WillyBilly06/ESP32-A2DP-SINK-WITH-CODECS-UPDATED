#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <inttypes.h>
#define LHDC_HOT
#define LHDC_NOINLINE __attribute__((noinline))
#define LHDC_ALWAYS_INLINE inline __attribute__((always_inline))
#define LHDC_NSLOTS 1
#define LHDC_DEC_SLOT() 0
#define dbt_lhdc_mem_alloc malloc
#define dbt_lhdc_mem_free free
#include "lhdc_entropy_dec.h"
#include "lhdc_bit_reader.h"
#include "dbt_lhdc_bits.h"
#include "lhdcv5_int.h"
volatile int g_lhdc_trace;
static uint64_t peek_calls;
#include "rice_under_test.h"
#define CHECK(x) do{if(!(x)){printf("FAIL line %u\n",(unsigned)__LINE__);return 1;}}while(0)

/* A symbol tape isolates lazy lookahead rules. The other test below uses the
 * actual FAC arithmetic decoder/model and compares every state/history byte. */
static int tape_at(s2_lazy_t *z,int i){
    ++peek_calls;
    while(i>z->cur_idx){
        if(z->eos)return -1;
        if(z->produced>=z->hard_cap || z->produced>=z->d->len){z->eos=1;return -1;}
        z->save_p=z->d->p;z->save_range=z->d->range;
        z->cur_val=z->d->data[z->produced];++z->cur_idx;++z->produced;++z->d->p;
        z->d->range=z->d->range*1664525u+(uint32_t)z->cur_val+1013904223u;
    }
    return i==z->cur_idx?z->cur_val:-1;
}
#undef DBT_LHDC_RICE_H
#define s2_at tape_at
#define rice_read_quotient tape_rice
#include "dbt_lhdc_rice.h"
#undef rice_read_quotient
#undef s2_at
#include "rice_tape_reference.h"

typedef struct {
    fac_model_t m;fac_dec_t d;s2_lazy_t z;
    uint8_t history[3072+32];
} context_t;
static uint32_t rng=0x7326ab9du;
static uint32_t random_word(void){rng^=rng<<13;rng^=rng>>17;rng^=rng<<5;return rng;}
static void initialize(context_t *c,const uint8_t *data,int len,int window,int cap,bool tape){
    memset(c,0,sizeof(*c));memset(c->history,0x6b,sizeof c->history);
    c->m.hist=c->history+16;c->m.hist_cap=3072;
    fac_model_init(&c->m,FAC_FQ2,window);
    if(tape)c->d=(fac_dec_t){data,len,0,0x12345678u,0};
    else fac_dec_init(&c->d,data,len);
    c->z=(s2_lazy_t){.d=&c->d,.m=&c->m,.cur_idx=-1,.hard_cap=cap};
    c->z.save_p=c->d.p;c->z.save_range=c->d.range;
}
static bool same(const context_t *a,const context_t *b){
    fac_model_t ma,mb;memcpy(&ma,&a->m,sizeof ma);memcpy(&mb,&b->m,sizeof mb);
    ma.hist=mb.hist=NULL;
    return !memcmp(&ma,&mb,sizeof ma) && !memcmp(a->history,b->history,sizeof a->history) &&
        a->d.p==b->d.p && a->d.range==b->d.range && a->d.code==b->d.code &&
        a->z.cur_idx==b->z.cur_idx && a->z.cur_val==b->z.cur_val && a->z.produced==b->z.produced &&
        a->z.hard_cap==b->z.hard_cap && a->z.eos==b->z.eos && a->z.save_p==b->z.save_p &&
        a->z.save_range==b->z.save_range && !a->z.error && !b->z.error;
}
static uint64_t valid,invalid,old_peeks,new_peeks;
static int compare_stream(const uint8_t *data,int len,int window,int cap,bool tape){
    context_t candidate,reference;initialize(&candidate,data,len,window,cap,tape);initialize(&reference,data,len,window,cap,tape);
    int a=0,b=0;
    for(unsigned step=0;step<64;++step){
        peek_calls=0;int qa=tape?tape_rice(&candidate.z,&a):rice_read_quotient(&candidate.z,&a);new_peeks+=peek_calls;
        if(candidate.z.error){++invalid;CHECK(candidate.z.produced<=cap);break;}
        peek_calls=0;int qb=tape?reference_tape_rice(&reference.z,&b):reference_rice_read_quotient(&reference.z,&b);old_peeks+=peek_calls;
        CHECK(qa==qb && a==b && same(&candidate,&reference));++valid;
        if(candidate.z.eos)break;
    }
    return 0;
}
static unsigned extended(uint8_t *out,uint32_t value,unsigned groups){
    unsigned n=0;
    for(unsigned k=0;k<groups;++k){out[n++]=2;out[n++]=(uint8_t)((value>>(2*k+1))&1u);out[n++]=(uint8_t)((value>>(2*k))&1u);}
    uint32_t top=value>>(2*groups);
    for(uint32_t k=1;k<top;++k)out[n++]=1;
    out[n++]=0;return n;
}
static int overflow_and_boundary_cases(void){
    uint8_t tape[128];context_t c;int j;
    const uint8_t three[]={1,1,1,2,0,0,0};initialize(&c,three,sizeof three,8,128,true);j=0;
    CHECK(tape_rice(&c.z,&j)==3 && j==3 && c.z.produced==3 && !c.z.error);
    unsigned n=0;for(unsigned k=0;k<16;++k){tape[n++]=2;tape[n++]=0;tape[n++]=0;}tape[n++]=0;
    initialize(&c,tape,(int)n,8,128,true);j=0;(void)tape_rice(&c.z,&j);CHECK(c.z.error);
    n=extended(tape,(uint32_t)INT32_MAX,15);initialize(&c,tape,(int)n,8,128,true);j=0;
    CHECK(tape_rice(&c.z,&j)==INT32_MAX && !c.z.error);
    int32_t coefficients[2]={-7,-7};int used=0;uint8_t s1[]={2,0};
    initialize(&c,tape,(int)n,8,128,true);
    tape_coefficients(s1,&c.z,1,0,coefficients,&used);CHECK(c.z.error && coefficients[0]==-7);
    n=extended(tape,(uint32_t)INT32_MAX-2u,15);initialize(&c,tape,(int)n,8,128,true);
    tape_coefficients(s1,&c.z,1,0,coefficients,&used);CHECK(!c.z.error && coefficients[0]==INT32_MAX);
    n=extended(tape,1u<<30,15);initialize(&c,tape,(int)n,8,128,true);s1[0]=s1[1]=0;coefficients[1]=-7;
    tape_coefficients(s1,&c.z,2,2,coefficients,&used);CHECK(c.z.error && coefficients[0]==0 && coefficients[1]==-7);
    n=extended(tape,(uint32_t)INT32_MAX/2u,14);initialize(&c,tape,(int)n,8,128,true);s1[1]=1;
    tape_coefficients(s1,&c.z,2,2,coefficients,&used);CHECK(!c.z.error && coefficients[1]==INT32_MAX);
    return 0;
}
static int public_entropy_case(uint32_t q,unsigned groups,bool tail,bool rejected){
    uint8_t tape[128],range_bytes[512],scratch[512],s1_scratch[4],first[]={0,0,tail?2:0,tail?1:2};
    uint32_t words[128]={0};unsigned count=0;
    if(groups==16){for(unsigned i=0;i<16;++i){tape[count++]=2;tape[count++]=0;tape[count++]=0;}tape[count++]=0;}
    else count=extended(tape,q,groups);
    lv5e_arith_enc_t encoder;lv5e_arith_base_t base;lv5e_bitout_t bits;
    lv5e_arith_enc_new(&encoder,&base,range_bytes,sizeof range_bytes);
    lv5e_arith_program(&encoder,FAC_FQ1,3,8);
    for(unsigned i=0;i<4;++i)CHECK(lv5e_arith_symbol_3(&encoder,first[i])==0);
    lv5e_arith_program(&encoder,FAC_FQ2,3,16);
    if(tail)CHECK(lv5e_arith_symbol_3(&encoder,0)==0); /* s1[2] has magnitude2 */
    for(unsigned i=0;i<count;++i)CHECK(lv5e_arith_symbol_3(&encoder,tape[i])==0);
    lv5e_bitout_new(&bits,words,sizeof words/sizeof words[0]);
    CHECK(lv5e_arith_clear(&encoder,&bits)>=0);
    int bytes=lv5e_bitout_clear(&bits);CHECK(bytes>0 && bytes+4<=(int)sizeof words);
    /* Real frames have mantissa/sign bytes after FAC. Supply zero lookahead
     * here so the lazy stream's end-of-input guard does not truncate the test. */
    bytes+=4;
    lhdc_dec_bit_reader_t reader;lhdc_bit_reader_init(&reader,(const uint8_t *)words,(size_t)bytes);
    struct {uint32_t before;int32_t values[4];uint32_t after;} output={0x12345678u,{-7,-7,-7,-7},0x87654321u};
    int consumed=-777;g_fac_final_range[0]=0xabcdef01u;
    lhdc_dec_ret_t result=lhdc_entropy_decode_spectrum_ex2(output.values,4,&reader,4,tail?2:0,8,16,&consumed,scratch,sizeof scratch,s1_scratch,NULL,0);
    CHECK(output.before==0x12345678u && output.after==0x87654321u);
    if(rejected){
        if(result!=LHDC_DEC_BITSTREAM_ERROR)printf("public q=%u groups=%u tail=%u result=%d consumed=%d s1=%u/%u/%u/%u values=%d/%d/%d/%d\n",q,groups,tail,(int)result,consumed,s1_scratch[0],s1_scratch[1],s1_scratch[2],s1_scratch[3],output.values[0],output.values[1],output.values[2],output.values[3]);
        CHECK(result==LHDC_DEC_BITSTREAM_ERROR && consumed==-777 && g_fac_final_range[0]==0xabcdef01u);
    }
    else CHECK(result==LHDC_DEC_OK && output.values[0]==0 && output.values[1]==0 && output.values[3]==INT32_MAX && consumed>=4);
    return 0;
}
int main(void){
    CHECK(!overflow_and_boundary_cases());
    lhdc_entropy_alloc();CHECK(dbt_lhdc_entropy_ready());
    CHECK(!public_entropy_case((uint32_t)INT32_MAX,15,false,true));
    CHECK(!public_entropy_case((uint32_t)INT32_MAX-2u,15,false,false));
    CHECK(!public_entropy_case(1u<<30,15,true,true));
    CHECK(!public_entropy_case((uint32_t)INT32_MAX/2u,14,true,false));
    CHECK(!public_entropy_case(0,16,false,true));
    lhdc_entropy_free();
    uint8_t data[512];const int windows[]={1,2,3,8,32,256,281,512,1024,2248,3072,3073};
    for(unsigned w=0;w<sizeof windows/sizeof windows[0];++w)for(unsigned trial=0;trial<3000;++trial){
        int len=(int)(random_word()%513u),cap=(int)(random_word()%15361u);
        for(int i=0;i<len;++i)data[i]=trial%5==0?0:trial%5==1?255:trial%5==2?(uint8_t)i:(uint8_t)random_word();
        CHECK(!compare_stream(data,len,windows[w],cap,false));
    }
    uint32_t cases=1;
    for(unsigned len=0;len<=10;++len){
        if(len)cases*=3;
        for(uint32_t bits=0;bits<cases;++bits){uint32_t v=bits;
            for(unsigned i=0;i<len;++i){data[i]=(uint8_t)(v%3u);v/=3u;}
            CHECK(!compare_stream(data,(int)len,8,128,true));
        }
    }
    CHECK(old_peeks>new_peeks);
    printf("PASS %" PRIu64 " valid Rice/FAC/model/history transitions; overflow_rejects=%" PRIu64
        "; old_peeks=%" PRIu64 "; new_peeks=%" PRIu64 "; explicit_boundary_cases=7; public_entropy_cases=5; full ternary tapes through10 symbols; no extra lookahead and checked coefficient boundaries\n",
        valid,invalid,old_peeks,new_peeks);
    return 0;
}
