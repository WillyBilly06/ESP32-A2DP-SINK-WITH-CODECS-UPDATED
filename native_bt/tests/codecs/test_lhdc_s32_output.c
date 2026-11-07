#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "lhdc_output_test_common.h"
static uint32_t rnd(uint32_t *s){uint32_t x=*s;x^=x<<13;x^=x>>17;x^=x<<5;return *s=x;}
int main(int argc,char **argv){
 bool exhaustive=argc==1;if(!exhaustive)assert(argc==2&&!strcmp(argv[1],"--quick"));
 uint64_t checked=0;uint32_t seed=0x73913467u,bits=0;
 do{uint32_t v=exhaustive?bits:rnd(&seed);int32_t a=reference_sample(v),b=candidate_sample(v);if(a!=b){fprintf(stderr,"FAIL bits=%08x ref=%d candidate=%d\n",v,a,b);abort();}++checked;if(!exhaustive&&checked==1000000)break;++bits;}while(bits);
 float l[960],r[960],oa[960],ob[960],pa[960],pb[960];_Alignas(8) uint8_t a[7696],b[7696];
 const int counts[]={0,1,17,127,128,240,480,960};const float scales[]={0.00732421875f,0.0146484375f,0.029296875f,480.0f,960.0f,1920.0f};
 unsigned frames=0;
 for(unsigned k=0;k<12000;++k){int count=counts[k%8],channels=1+(k/8)%2,width=(k/16)%2?2:4;float scale=scales[(k/32)%6];
  for(int i=0;i<count;++i){uint32_t x=rnd(&seed),y=rnd(&seed);if(k%3==0){x=(x&0x807fffffu)|0x46000000u;y=(y&0x807fffffu)|0x46000000u;}memcpy(l+i,&x,4);memcpy(r+i,&y,4);}
  if(count&&k%19==0){uint32_t edge[]={0x7fc12345u,0x7f800001u,0xff800000u,0x7f800000u,0x80000000u,0x4f000000u};memcpy(l,&edge[(k/19)%6],4);}
  memset(a,0xa5,sizeof(a));memset(b,0xa5,sizeof(b));memset(oa,0x3c,sizeof(oa));memcpy(ob,oa,sizeof(oa));memset(pa,0x5a,sizeof(pa));memcpy(pb,pa,sizeof(pa));
  output_event_t ea={0},eb={0};reference_frame(l,r,a+8,count,channels,width,scale,&ea,oa,pa);candidate_frame(l,r,b+8,count,channels,width,scale,&eb,ob,pb);
  assert(!memcmp(a,b,sizeof(a))&&!memcmp(&ea,&eb,sizeof(ea))&&!memcmp(oa,ob,sizeof(oa))&&!memcmp(pa,pb,sizeof(pa)));++frames;
  int32_t xa[1920],xb[1920];reference_emit(l,r,xa,count,scale);candidate_emit(l,r,xb,count,scale);assert(!memcmp(xa,xb,(size_t)count*8));
 }
 printf("PASS %" PRIu64 " float patterns; %u frame output/quality/overlap cases\n",checked,frames);return 0;
}
