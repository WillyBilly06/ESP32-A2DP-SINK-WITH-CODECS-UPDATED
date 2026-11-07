#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
static unsigned rescales;
volatile int g_lhdc_trace;
int g_fdc,g_block;
#include "fac_run_under_test.h"
static uint32_t rnd(uint32_t *s){uint32_t x=*s;x^=x<<13;x^=x>>17;x^=x<<5;return *s=x;}
static void equal(fac_dec_t *a,fac_dec_t *b,fac_model_t *ma,fac_model_t *mb,uint8_t *ha,uint8_t *hb){
 assert(!memcmp(a,b,sizeof(*a)));uint8_t *save=mb->hist;mb->hist=ma->hist;assert(!memcmp(ma,mb,sizeof(*ma)));mb->hist=save;assert(!memcmp(ha,hb,3074));
}
int main(void){
 const int windows[]={1,2,3,31,32,64,65,96,114,126,128,512,3072};
 const int lengths[]={0,1,2,3,4,5,15,31,512};
 const int chunks[]={0,1,2,3,15,31,64,127,512,960,1024};
 uint32_t cases=0,symbols=0,seed=0x72817364u;
 uint8_t data[512],ha[3074],hb[3074],oa[1026],ob[1026];
 for(unsigned w=0;w<13;++w)for(unsigned len=0;len<9;++len)for(unsigned pattern=0;pattern<5;++pattern){
  for(unsigned i=0;i<512;++i)data[i]=pattern==0?0:pattern==1?255:pattern==2?(uint8_t)(i*13u):pattern==3?(i<8?0:(uint8_t)rnd(&seed)):(uint8_t)rnd(&seed);
  memset(ha,0xa5,sizeof(ha));memset(hb,0xa5,sizeof(hb));
  fac_model_t ma={0},mb={0};ma.hist=ha+1;mb.hist=hb+1;ma.hist_cap=mb.hist_cap=3072;
  const uint32_t f[]={54,12,1};fac_model_init(&ma,f,windows[w]);fac_model_init(&mb,f,windows[w]);
  fac_dec_t a={0},b={0};fac_dec_init(&a,data,lengths[len]);fac_dec_init(&b,data,lengths[len]);
  for(unsigned stage=0;stage<12;++stage){
   int n=stage<11?chunks[stage]:1024;
   for(int part=0;part<(stage==11?1024:1);++part){
    int count=stage==11?1:n;if(part%97==0){ma.thresh[0]=ma.freq[0];mb.thresh[0]=mb.freq[0];}
    memset(oa,0xa5,sizeof(oa));memset(ob,0xa5,sizeof(ob));
    unsigned start=rescales;for(int i=0;i<count;++i)oa[i+1]=(uint8_t)fac_decode(&a,&ma);unsigned expected_calls=rescales-start;
    start=rescales;dbt_lhdc_fac_run(&b,&mb,ob+1,count);assert(rescales-start==expected_calls);
    assert(!memcmp(oa,ob,sizeof(oa)));equal(&a,&b,&ma,&mb,ha,hb);assert(ha[0]==0xa5 && ha[3073]==0xa5);symbols+=(uint32_t)count;cases++;
   }
  }
 }
 /* A trace-active host must keep the exact original printing/counter path. */
 fac_model_t m={0};m.hist=ha+1;m.hist_cap=3072;const uint32_t f[]={54,12,1};fac_dec_t d={0};memset(data,0,sizeof(data));
 g_lhdc_trace=1;g_block=321;g_fdc=0;fac_model_init(&m,f,32);fac_dec_init(&d,data,512);
 puts("BEGIN_TRACE_REF");for(int i=0;i<16;++i)(void)fac_decode(&d,&m);puts("END_TRACE_REF");assert(g_fdc==16);
 g_fdc=0;fac_model_init(&m,f,32);fac_dec_init(&d,data,512);unsigned fast_before=dbt_run_symbols;
 puts("BEGIN_TRACE_FAST");dbt_lhdc_fac_run(&d,&m,oa,16);puts("END_TRACE_FAST");assert(g_fdc==16 && dbt_run_symbols==fast_before);
 assert(dbt_run_symbols>0 && dbt_run_padding>0);
 printf("PASS %u FAC call-boundary states / %u symbols; fast=%u padding=%u\n",cases,symbols,dbt_run_symbols,dbt_run_padding);return 0;
}
