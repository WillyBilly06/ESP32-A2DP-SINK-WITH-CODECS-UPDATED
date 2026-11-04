"""Prove generated coefficient bounds and compare exact biquad accumulation.
Host-only wide-integer oracle plus ASan/UBSan; never opens a device.
"""
import ast,hashlib,json,os,pathlib,re,shutil,subprocess
ROOT=pathlib.Path(__file__).resolve().parents[1];DSP=ROOT.parent/'bt_audio_sink/main/dsp';OUT=ROOT/'_build/dsp_accumulation';OUT.mkdir(exist_ok=True)
table=(DSP/'fixed_dsp_tables.inc').read_text(encoding='utf-8');vectors=[[1<<28,0,0,0,0]]
for name in ('fdsp_eq','fdsp_comp','fdsp_cross','fdsp_boost'):
    match=re.search(r'\b'+name+r'\b[^=]*=\s*(\{.*?\});',table,re.S)
    body=re.sub(r'/\*.*?\*/|//[^\n]*','',match.group(1),flags=re.S)
    data=ast.literal_eval(body.replace('{','[').replace('}',']'))
    def walk(x):
        if len(x)==5 and all(isinstance(v,int) for v in x):vectors.append(x)
        else:
            for child in x:walk(child)
    walk(data)
bounds=[max(abs(row[i]) for row in vectors) for i in range(5)]
declared=int(re.search(r'^#define FDSP_COEFF_BOX_BOUND (\d+)ULL$',table,re.M)[1])
assert sum(bounds)==declared and declared*(1<<31)+(1<<28)-1 < (1<<63)-1
source=(DSP/'fixed_dsp.c').as_posix()
public=('init','controls','process','zero_analysis','level_db_q8')
wrappers=[]
for kind in ('fast','reference','fallback'):
    text=''
    if kind=='reference':text+='#define FIXED_DSP_FORCE_SPLIT_SUM 1\n'
    if kind=='fallback':text+='#define FIXED_DSP_TABLES_HEADER "unproven.inc"\n'
    if kind!='fast':text+=''.join('#define fixed_dsp_'+n+' '+kind+'_'+n+'\n' for n in public)
    text+='#include "'+source+'"\n'
    text+='int32_t '+kind+'_filter(const int32_t *c,fdsp_history_t *h,int32_t x){return filter(c,h,x);}\n'
    text+='int '+kind+'_enabled(void){return FDSP_PROVEN_SUM_ENABLED;}\n'
    p=OUT/(kind+'.c');p.write_text(text);wrappers.append(p)
(OUT/'unproven.inc').write_text('#include "'+(DSP/'fixed_dsp_tables.inc').as_posix()+'"\n#undef FDSP_COEFF_BOX_BOUND\n')
main=OUT/'oracle.c'
main.write_text(r'''#include "fixed_dsp.h"
#include <limits.h>
#include <stdio.h>
#include <string.h>
int32_t fast_filter(const int32_t *,fdsp_history_t *,int32_t);
int32_t reference_filter(const int32_t *,fdsp_history_t *,int32_t);
int32_t fallback_filter(const int32_t *,fdsp_history_t *,int32_t);
int fast_enabled(void);int reference_enabled(void);int fallback_enabled(void);
bool reference_init(fixed_dsp_t *,uint32_t,uint32_t,uint32_t,uint32_t);
void reference_controls(fixed_dsp_t *,int8_t,int8_t,int8_t,uint8_t,uint8_t,bool);
void reference_process(fixed_dsp_t *,int32_t *,int32_t *);
static const int64_t bounds[5]={BOUND_VALUES};
static uint32_t seed=0x751391abu;
static uint32_t random32(void){seed^=seed<<13;seed^=seed>>17;seed^=seed<<5;return seed;}
static int32_t oracle(const int32_t *c,fdsp_history_t *h,int32_t x){
 int32_t output;
 if(c[0]==268435456 && !c[1] && !c[2] && !c[3] && !c[4]){output=x;h->fraction=0;}
 else{
  __int128 sum=h->fraction;
  sum+=(__int128)c[0]*x;sum+=(__int128)c[1]*h->x1;sum+=(__int128)c[2]*h->x2;
  sum-=(__int128)c[3]*h->y1;sum-=(__int128)c[4]*h->y2;
  __int128 integer=sum>>28;
  output=integer>INT32_MAX?INT32_MAX:integer<INT32_MIN?INT32_MIN:(int32_t)integer;
  h->fraction=integer==output?(uint32_t)((unsigned __int128)sum&0x0fffffffu):0;
 }
 h->x2=h->x1;h->x1=x;h->y2=h->y1;h->y1=output;return output;
}
int main(void){
 if(!fast_enabled() || reference_enabled() || fallback_enabled())return 1;
 for(unsigned i=0;i<300000;++i){
  int32_t c[5];bool bounded=i<200000;
  for(unsigned j=0;j<5;++j)c[j]=bounded?(int32_t)((int64_t)(random32()%(uint64_t)(2*bounds[j]+1))-bounds[j]):(int32_t)random32();
  if(i==0){c[0]=268435456;for(unsigned j=1;j<5;++j)c[j]=0;}
  fdsp_history_t a={(int32_t)random32(),(int32_t)random32(),(int32_t)random32(),(int32_t)random32(),random32()&0x0fffffffu},b=a,d=a;
  int32_t input=i%3u==0?INT32_MIN:i%3u==1?INT32_MAX:(int32_t)random32();
  int32_t expected=oracle(c,&a,input),ref=reference_filter(c,&b,input);
  int32_t actual=bounded?fast_filter(c,&d,input):fallback_filter(c,&d,input);
  if(actual!=expected || ref!=expected || memcmp(&a,&b,sizeof a) || memcmp(&a,&d,sizeof a)){printf("filter mismatch %u\n",i);return 2;}
 }
 const uint32_t rates[]={8000,11025,12000,16000,22050,24000,32000,44100,48000,88200,96000,176400,192000};
 for(unsigned rate=0;rate<13;++rate){
  fixed_dsp_t a,b;if(!fixed_dsp_init(&a,rates[rate],90,500,512) || !reference_init(&b,rates[rate],90,500,512))return 3;
  for(unsigned i=0;i<8192;++i){
   if(i%37u==0){
    int8_t bass=(int8_t)(random32()%25u)-12,mid=(int8_t)(random32()%25u)-12,treble=(int8_t)(random32()%25u)-12;
    uint8_t volume=(uint8_t)(random32()%128u),flags=(uint8_t)(random32()&15u);bool immediate=i%1024u==0;
    fixed_dsp_controls(&a,bass,mid,treble,volume,flags,immediate);reference_controls(&b,bass,mid,treble,volume,flags,immediate);
   }
   for(unsigned f=0;f<5;++f)for(unsigned j=0;j<5;++j){int64_t v=a.filters[f].value[j];if(v< -bounds[j] || v>bounds[j])return 4;}
   int32_t al=(int32_t)random32(),ar=(int32_t)random32(),bl=al,br=ar;
   fixed_dsp_process(&a,&al,&ar);reference_process(&b,&bl,&br);
   if(al!=bl || ar!=br || memcmp(&a,&b,sizeof a)){printf("stream mismatch %u %u\n",rates[rate],i);return 5;}
  }
 }
 puts("PASS 300000 wide-oracle filters and 106496 rapid-control stereo frames, full state/output equality");return 0;
}
'''.replace('BOUND_VALUES',','.join(map(str,bounds))))
env=os.environ.copy()
if os.name=='nt':
    vc=pathlib.Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat');data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
    env.update(dict(x.split('=',1) for x in data.splitlines() if '=' in x and not x.startswith('=')))
cc=shutil.which('clang',path=env.get('PATH')) or (r'C:\Program Files\LLVM\bin\clang.exe' if os.name=='nt' else 'clang')
exe=OUT/('oracle.exe' if os.name=='nt' else 'oracle')
tracked=wrappers+[main,OUT/'unproven.inc']+list(DSP.glob('fixed_dsp*'))+[pathlib.Path(__file__).resolve(),ROOT/'tools/generate_fixed_dsp_tables.py']
before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked}
args=[cc,'-std=c11','-O3','-g','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer','-I'+str(DSP),'-I'+str(OUT),*map(str,wrappers),str(main),'-o',str(exe)]
if os.name=='nt':args+=['-fms-runtime-lib=dll','-D_DLL','-Wl,/NODEFAULTLIB:libcmt']
subprocess.run(args,check=True,env=env)
if os.name=='nt':
    resource=pathlib.Path(subprocess.check_output([cc,'-print-resource-dir'],text=True).strip());runtime=resource/'lib/windows/clang_rt.asan_dynamic-x86_64.dll';shutil.copy2(runtime,OUT/runtime.name)
subprocess.run([str(exe)],check=True,env=env,timeout=60)
assert all(hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()==digest for p,digest in before.items())
(OUT/'results.json').write_text(json.dumps({'passed':True,'bounded_filter_cases':200000,'unproven_table_fallback_cases':100000,'rapid_control_stereo_frames':106496,'rates':13,'component_box':bounds,'box_sum':declared,'worst_sum_with_fraction':declared*(1<<31)+(1<<28)-1,'int64_max':(1<<63)-1,'oracle':'independent 128-bit sum with full state/output comparison','sanitizers':['address','undefined'],'signed_overflow_wrapping':False,'sources':before,'command':args,'hardware_accessed':False},indent=2)+'\n')
