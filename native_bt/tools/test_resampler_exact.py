"""Wide-integer oracle for the actual optimized FIR; host ASan/UBSan only."""
import hashlib,json,os,pathlib,shutil,subprocess
ROOT=pathlib.Path(__file__).resolve().parents[1]
OUT=ROOT/'_build/resampler_exact';OUT.mkdir(exist_ok=True)
main=OUT/'oracle.c'
main.write_text(r'''#include "dbt_resampler.h"
#include <limits.h>
#include <stdio.h>
#include <string.h>
static uint32_t seed=0x718392abu;
static uint32_t random32(void){seed^=seed<<13;seed^=seed>>17;seed^=seed<<5;return seed;}
static int32_t clip(__int128 n){n>>=30;return n>INT32_MAX?INT32_MAX:n<INT32_MIN?INT32_MIN:(int32_t)n;}
static void oracle(dbt_resampler_t *s,int32_t left,int32_t right,uint32_t phase,int32_t *l,int32_t *r){
 if(!s->primed){s->primed=true;s->first[0]=left;s->first[1]=right;}
 uint8_t taps;uint32_t rate=s->taps==24?192000:s->taps==48?96000:48000;
 const int32_t *a=dbt_resampler_phase(rate,(uint8_t)(phase>>25),&taps);
 const int32_t *b=dbt_resampler_phase(rate,(uint8_t)((phase>>25)+1u),0);
 __int128 sum_l=0,sum_r=0;
 for(unsigned k=0;k<taps;++k){
  __int128 coefficient=(__int128)a[k]+(((__int128)b[k]-a[k])*(phase&0x1ffffffu))/33554432;
  unsigned index=(s->next-k)&127u;
  int32_t x=k==0?left:k<=s->count?s->history[0][index]:s->first[0];
  int32_t y=k==0?right:k<=s->count?s->history[1][index]:s->first[1];
  sum_l+=(__int128)x*coefficient;sum_r+=(__int128)y*coefficient;
 }
 *l=clip(sum_l);*r=clip(sum_r);
}
int main(void){
 for(unsigned i=0;i<150000;++i){
  dbt_resampler_t a,b;memset(&a,0,sizeof a);
  for(unsigned k=0;k<128;++k){a.history[0][k]=(int32_t)random32();a.history[1][k]=(int32_t)random32();}
  a.first[0]=(int32_t)random32();a.first[1]=(int32_t)random32();a.next=(uint8_t)(random32()&127u);
  a.count=(uint8_t)(random32()%129u);a.taps=i%3==0?24:i%3==1?48:96;a.primed=(random32()&1u)!=0;b=a;
  unsigned mode=(i/3u)%4u;
  uint32_t phase=mode==0?0:mode==1?random32()&0xfe000000u:mode==2?UINT32_MAX:random32();
  int32_t left=(int32_t)random32(),right=(int32_t)random32(),al,ar,bl,br;
  oracle(&a,left,right,phase,&al,&ar);dbt_resampler_filter(&b,left,right,phase,&bl,&br);
  if(al!=bl || ar!=br || memcmp(&a,&b,sizeof a)){printf("FAIL case %u\n",i);return 1;}
 }
 puts("150000 wide-oracle resampler output/state cases passed at O3");return 0;
}
''')
env=os.environ.copy()
if os.name=='nt':
    vc=pathlib.Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
    data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
    env.update(dict(x.split('=',1) for x in data.splitlines() if '=' in x and not x.startswith('=')))
cc=shutil.which('clang',path=env.get('PATH')) or (r'C:\Program Files\LLVM\bin\clang.exe' if os.name=='nt' else 'clang')
tracked=[ROOT/'src/dbt_resampler.c',ROOT/'src/dbt_resampler_coeffs.inc',ROOT/'include/dbt_resampler.h',main,pathlib.Path(__file__).resolve()]
before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked}
report_path=OUT/'results.json';report_path.write_text(json.dumps({'passed':False,'state':'building'})+'\n')
exe=OUT/('oracle.exe' if os.name=='nt' else 'oracle')
args=[cc,'-std=c11','-O3','-g','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer','-I'+str(ROOT/'include'),str(ROOT/'src/dbt_resampler.c'),str(main),'-o',str(exe)]
if os.name=='nt':args+=['-fms-runtime-lib=dll','-D_DLL','-Wl,/NODEFAULTLIB:libcmt']
subprocess.run(args,check=True,env=env)
if os.name=='nt':
    resource=pathlib.Path(subprocess.check_output([cc,'-print-resource-dir'],text=True).strip())
    runtime=resource/'lib/windows/clang_rt.asan_dynamic-x86_64.dll';shutil.copy2(runtime,OUT/runtime.name)
subprocess.run([str(exe)],check=True,env=env,timeout=60)
assert all(hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()==digest for p,digest in before.items())
report_path.write_text(json.dumps({'passed':True,'cases':150000,'oracle':'128-bit integer convolution/interpolation','optimization':'O3','sanitizers':['address','undefined'],'scope':'full output/state, all tap lengths across zero/bank-aligned/max/random phases and priming/history states; host only','sources':before,'command':args,'hardware_accessed':False},indent=2)+'\n')
