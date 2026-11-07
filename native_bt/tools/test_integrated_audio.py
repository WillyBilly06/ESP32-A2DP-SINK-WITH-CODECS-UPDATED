"""Exercise actual PCM-level accumulator, deadline adjustment and owner checkpoint.
Controlled data/clock only; this does not model ESP32 physical scheduling.
"""
import hashlib,json,os,pathlib,re,shutil,subprocess
ROOT=pathlib.Path(__file__).resolve().parents[1];OUT=ROOT/'_build/integrated_audio';OUT.mkdir(exist_ok=True)
(OUT/'results.json').write_text(json.dumps({'passed':False,'state':'building','hardware_accessed':False})+'\n')
paths=[ROOT/'esp32_probe/main'/n for n in ['probe_wait.h','probe_pcm_level.h','probe_main.c']]
text={p.name:p.read_text() for p in paths};hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
def function(txt,signature):
 start=txt.index(signature);opening=txt.index('{',start);depth=1;end=opening+1
 while depth:
  if txt[end]=='{':depth+=1
  elif txt[end]=='}':depth-=1
  end+=1
 return txt[start:end]
helpers=function(text['probe_wait.h'],'static inline uint32_t probe_wait_remaining(')
checkpoint=function(text['probe_main.c'],'static void owner_render_checkpoint(')
stream_source=(ROOT/'esp32_probe/main/probe_audio.c').read_text()
stream_select=function(stream_source,'static dbt_result_t stream_begin(')
queue_select=function(stream_source,'static dbt_codec_queue_t codec_queue_policy(')
wait_wrapper=next(l for l in text['probe_main.c'].splitlines() if l.startswith('#define HOST_WAIT_VALUE('))
wait_region=text['probe_main.c'].split('uint32_t deadline_now=now_ms();',1)[1].split('probe_audio_owner_wait(wait_zeros)',1)[0]
assert wait_region.count('now_ms()')==1 and wait_region.count('probe_wait_remaining(wait,deadline_now,now_ms())')==1
assert text['probe_main.c'].count('HOST_POLL(PROBE_HOST_')==14
assert text['probe_main.c'].count('HOST_WAIT_VALUE(PROBE_COST_')==15
for sig in ['static bool transport_can_send(','static bool transport_send(']:
 if sig in text['probe_main.c']:assert 'owner_render_checkpoint' not in function(text['probe_main.c'],sig)
prefix='''#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#define DBT_PROBE_RENDER_FAIR_SHARE 1
#define CHECK(x) do{if(!(x)){printf("FAIL line %u\\n",(unsigned)__LINE__);return 1;}}while(0)
static bool pressure;
static unsigned yields,recorded;
static bool probe_audio_owner_pressure(void){return pressure;}
static void probe_audio_owner_service_yield(void){++recorded;}
static void taskYIELD(void){++yields;}
'''
main=r'''
static uint32_t rng=0x792631a5u;
static uint32_t random_word(void){rng^=rng<<13;rng^=rng>>17;rng^=rng<<5;return rng;}
static int sample(uint32_t frame,unsigned channel){return (int)((frame*97u+channel*171u)&65535u)-32768;}
int main(void){
 CHECK(probe_wait_remaining(UINT32_MAX,99,999)==UINT32_MAX);
 CHECK(probe_wait_remaining(10,UINT32_MAX-3u,2u)==4);
 CHECK(probe_wait_remaining(10,100,110)==0 && probe_wait_remaining(10,100,111)==0);
 for(unsigned n=0;n<1000000u;++n){
  uint32_t began=random_word(),span=random_word()%1000000u,wait=random_word()&0x7fffffffu;
  uint64_t absolute_due=(uint64_t)began+wait,absolute_now=(uint64_t)began+span;
  uint32_t expected=absolute_due<=absolute_now?0:(uint32_t)(absolute_due-absolute_now);
  CHECK(probe_wait_remaining(wait,began,began+span)==expected);
 }
 uint32_t ticket=0;dbt_codec_config_t c={&dbt_lhdcv5_codec,192000,960,2,4};
 CHECK(stream_begin(NULL,true,&c,&ticket)==DBT_PENDING && selected_guard==40 && !selected_share && ticket==19 && async_stream_start);
 dbt_codec_queue_t q=codec_queue_policy(NULL,&c);CHECK(q.depth==8 && q.low_water==5);
 for(unsigned queued=0;queued<=q.low_water;++queued)for(unsigned already_credited=0;already_credited<=3;++already_credited)CHECK(queued+already_credited<=q.depth);
 c.rate=48000;q=codec_queue_policy(NULL,&c);CHECK(q.depth==8 && q.low_water==4);
 c.ops=&dbt_aac_codec;c.rate=44100;c.samples=1024;CHECK(stream_begin(NULL,true,&c,&ticket)==DBT_PENDING && selected_guard==47 && selected_share);
 c.rate=48000;CHECK(stream_begin(NULL,true,&c,&ticket)==DBT_PENDING && selected_guard==43);
 c.ops=&dbt_ldac_codec;CHECK(stream_begin(NULL,true,&c,&ticket)==DBT_PENDING && selected_guard==40 && selected_share);
 c.ops=&dbt_aptx_codec;CHECK(stream_begin(NULL,true,&c,&ticket)==DBT_PENDING && selected_guard==0 && selected_share);
 c.ops=&dbt_sbc_codec;CHECK(stream_begin(NULL,true,&c,&ticket)==DBT_PENDING && selected_guard==0 && selected_share);
 CHECK(stream_begin(NULL,true,NULL,&ticket)==DBT_INVALID);
 owner_render_checkpoint();CHECK(!yields && !recorded);pressure=true;
 owner_render_checkpoint();CHECK(yields==1 && recorded==1);
 uint32_t evaluated=0,selected=0;HOST_WAIT_VALUE(UNUSED_SLOT,selected,++evaluated);
 CHECK(evaluated==1 && selected==1 && yields==3 && recorded==3);
 const uint32_t rates[]={44100,48000,96000,192000},chunks[]={1,32,128,256,960};
 uint8_t pcm[960*8];unsigned cases=0;
 for(unsigned ri=0;ri<4;++ri)for(unsigned width=2;width<=4;++width)for(unsigned ch=1;ch<=2;++ch)for(unsigned ci=0;ci<5;++ci){
  probe_pcm_level_t s;probe_pcm_level_start(&s,rates[ri],ch,width);
  uint32_t total=5u*rates[ri],emitted=0;unsigned completions=0;
  while(emitted<total){
   unsigned count=chunks[ci];
   for(unsigned i=0;i<count;++i)for(unsigned c=0;c<ch;++c){
    uint32_t v=(uint32_t)(uint16_t)sample(emitted+i,c)<<(8u*(width-2u));
    if(width>2)v|=(1u<<(8u*(width-2u)))-1u;
    for(unsigned b=0;b<width;++b)pcm[(i*ch+c)*width+b]=(uint8_t)(v>>(8u*b));
   }
   completions+=probe_pcm_level_observe(&s,pcm,count);emitted+=count;
  }
  uint64_t squares=0;uint32_t peak=0,count=0;
  for(uint32_t frame=rates[ri];frame<total;++frame)for(unsigned c=0;c<ch;++c){int v=sample(frame,c);uint32_t mag=v<0?(uint32_t)-v:(uint32_t)v;if(mag>peak)peak=mag;squares+=(uint32_t)(v*v);++count;}
  CHECK(s.complete && completions==1 && s.frames==4u*rates[ri] && s.count==count && s.peak==peak && s.squares==squares);
  probe_pcm_level_t before=s;CHECK(!probe_pcm_level_observe(&s,pcm,1) && !memcmp(&s,&before,sizeof s));++cases;
 }
 probe_pcm_level_t invalid;probe_pcm_level_start(&invalid,192001,2,4);CHECK(!probe_pcm_level_observe(&invalid,pcm,1));
 probe_pcm_level_start(&invalid,48000,3,4);CHECK(!probe_pcm_level_observe(&invalid,pcm,1));
 probe_pcm_level_start(&invalid,48000,2,1);CHECK(!probe_pcm_level_observe(&invalid,pcm,1));
 printf("PASS1000000 actual deadline cases and%u PCM-level cases: all formats/rates/chunk boundaries, negative rails, full64-bit sums, exact completion, immutable finished state; bounded owner checkpoint\n",cases);return 0;
}
'''
selector_prefix="\n#define DBT_PROBE_CODEC_AWARE_RENDER 1\n#define DBT_PROBE_AAC_BUILD 1\n#define DBT_PROBE_LDAC_BUILD 1\n#define DBT_PROBE_LHDC_BUILD 1\n#define DBT_PROBE_APTX_BUILD 1\ntypedef struct {uint8_t depth,low_water;} dbt_codec_queue_t;\ntypedef int dbt_result_t;\ntypedef int dbt_pcm_format_t;\nenum {DBT_OK=0,DBT_PENDING=1,DBT_INVALID=2};\ntypedef struct {const void *ops;uint32_t rate,samples;uint8_t channels,sample_bytes;} dbt_codec_config_t;\nstatic const int dbt_sbc_codec=1,dbt_aac_codec=2,dbt_ldac_codec=3,dbt_lhdcv5_codec=4,dbt_aptx_codec=5;\nstatic bool async_stream_start,selected_share;\nstatic uint16_t selected_guard;\nstatic dbt_result_t probe_output_stream_begin_primed(bool start,uint32_t rate,uint8_t channels,dbt_pcm_format_t format,bool share,uint16_t guard,uint32_t *ticket){\n (void)start;(void)rate;(void)channels;(void)format;selected_guard=guard;selected_share=share;*ticket=19;return DBT_PENDING;\n}\n"
source=OUT/'integrated.c';source.write_text(prefix+text['probe_pcm_level.h']+'\n'+helpers+'\n'+checkpoint+'\n'+selector_prefix+'\n'+stream_select+'\n'+queue_select+'\n#define HOST_COST_VALUE(slot,destination,expression) do{(destination)=(expression);}while(0)\n'+wait_wrapper+'\n'+main)
env=os.environ.copy()
if os.name=='nt':
 vc=pathlib.Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
 data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
 env.update(dict(x.split('=',1) for x in data.splitlines() if '=' in x and not x.startswith('=')))
cc=shutil.which('clang') or r'C:\Program Files\LLVM\bin\clang.exe';cache=shutil.which('ccache')
if not cache:
 found=sorted((pathlib.Path.home()/'.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'));cache=str(found[-1]) if found else None
assert cache,'ccache required'
exe=OUT/('integrated.exe' if os.name=='nt' else 'integrated')
command=[cache,cc,'-std=c11','-O2','-Wall','-Wextra','-Werror','-fsanitize=undefined','-fno-sanitize-recover=all',str(source),'-o',str(exe)]
subprocess.run(command,check=True,env=env);r=subprocess.run([str(exe)],capture_output=True,text=True,env=env,check=True);print(r.stdout.strip())
assert hashes=={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
(OUT/'results.json').write_text(json.dumps({'passed':True,'hardware_accessed':False,'source_sha256':hashes,'stdout':r.stdout,'command':command},indent=2)+'\n')
