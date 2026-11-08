"""Real-worker encoded duration, actual LHDC framing and actual delay math. No device."""
import gzip,hashlib,json,os,pathlib,re,shutil,subprocess,sys
ROOT=pathlib.Path(__file__).resolve().parents[1];OUT=ROOT/'_build/codec_duration';OUT.mkdir(exist_ok=True)
receipt=OUT/'results.json';receipt.write_text(json.dumps({'passed':False,'state':'building','hardware_accessed':False})+'\n')
common=ROOT/'codecs/common/dbt_codec.c';lhdc=ROOT/'codecs/lhdcv5/dbt_lhdcv5_codec.c';output=ROOT/'esp32_probe/main/probe_output.c'
app=ROOT/'esp32_probe/main/probe_audio.c'
tracked=[app,common,common.with_suffix('.h'),lhdc,lhdc.with_suffix('.h'),output,output.with_suffix('.h'),ROOT/'tests/codecs/worker_duration.c',ROOT/'tests/test_codec_deferred.c',ROOT/'esp32_probe/main/probe_audio_trace.c',ROOT/'esp32_probe/main/probe_audio_trace.h',ROOT/'esp32_probe/main/probe_host_record.h',pathlib.Path(__file__).resolve()]
hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked}
def function(text,signature):
 start=text.index(signature);opening=text.index('{',start);depth=1;end=opening+1
 while depth:
  if text[end]=='{':depth+=1
  elif text[end]=='}':depth-=1
  end+=1
 return text[start:end]
app_actual=function(app.read_text(),'static uint32_t media_frame_count(')
ltext=lhdc.read_text();lactual=function(ltext,'static unsigned payload_frame_count(')+'\n'+function(ltext,'uint32_t dbt_lhdcv5_packet_frames(')
lprefix='#include "dbt_codec.h"\n#define LHDC_FRAME_MAX (1023u*2u)\n'
otext=output.read_text();oactual=function(otext,'uint16_t probe_output_buffered_delay_100us(')
mask=re.search(r'#define PRIMING_BUDGET [^\n]+',otext).group()
oprefix=r'''
#include <stdint.h>
#include <stdbool.h>
#include <stdatomic.h>
#include <stdio.h>
#define DMA_DESCRIPTORS 2u
#define DMA_FRAMES 192u
#define OUTPUT_ADAPTIVE_CLOCK 0
static struct {_Atomic uint32_t ready_frames,input_ready_frames,running,actual_target_frames,rate,priming_state;} output;
static uint32_t pcm_target_frames(uint32_t rate){(void)rate;return 2048;}
static uint32_t dbt_resampler_delay_frames(uint32_t rate){(void)rate;return 9;}
'''+mask+'\n'
omodel=r'''
int test_delay_math(void){
 const uint32_t rates[]={44100,48000,96000,192000},ready[]={0,64,960,2048},inputs[]={0,256,960},encoded[]={0,960,1920,13440,UINT32_MAX};unsigned cases=0;
 for(unsigned r=0;r<4;++r)for(unsigned a=0;a<4;++a)for(unsigned b=0;b<3;++b)for(unsigned e=0;e<5;++e)for(unsigned running=0;running<2;++running)for(unsigned guard=0;guard<2;++guard){
  atomic_store(&output.rate,rates[r]);atomic_store(&output.actual_target_frames,2048);atomic_store(&output.ready_frames,ready[a]);atomic_store(&output.input_ready_frames,inputs[b]);atomic_store(&output.running,running);atomic_store(&output.priming_state,guard?50:0);
  uint64_t frames=(running?(uint64_t)ready[a]+inputs[b]:2048u)+464u+encoded[e];uint64_t expected=(frames*10000u+rates[r]-1u)/rates[r];if(guard&&expected<500u)expected=500u;if(expected>65535u)expected=65535u;
  unsigned actual=probe_output_buffered_delay_100us(rates[r],encoded[e]);++cases;
  if(actual!=expected){fprintf(stderr,"FAIL delay math case %u: got%u expected%u\n",cases,actual,(unsigned)expected);return 1;}
 }
 if(probe_output_buffered_delay_100us(0,UINT32_MAX))return 1;
 atomic_store(&output.running,1);atomic_store(&output.rate,192000);atomic_store(&output.priming_state,50);atomic_store(&output.ready_frames,2048);atomic_store(&output.input_ready_frames,960);
 if(probe_output_buffered_delay_100us(192000,13440)!=881){fprintf(stderr,"FAIL queued samples must precede the50ms floor\n");return 1;}
 printf("PASS actual buffered delay: %u state/rate/guard/overflow cases; queued70ms+PCM18.08ms reports88.1ms, not50/120ms\n",cases);return 0;
}
'''
env=os.environ.copy();vc=pathlib.Path(r'C:/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools/VC/Auxiliary/Build/vcvars64.bat');values=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True);env.update(dict(x.split('=',1) for x in values.splitlines() if '=' in x and not x.startswith('=')))
cc=shutil.which('clang',path=env.get('PATH')) or r'C:/Program Files/LLVM/bin/clang.exe';choices=sorted((pathlib.Path.home()/'.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'));assert choices;cache=str(choices[-1]);env.pop('CCACHE_DISABLE',None)
resource=pathlib.Path(subprocess.check_output([cc,'-print-resource-dir'],text=True).strip());env['PATH']=str(resource/'lib/windows')+os.pathsep+env.get('PATH','')
hook=OUT/'query_hook.h';hook.write_text('void codec_duration_test_step(const void *);\n#define DBT_CODEC_DURATION_TEST_STEP(worker) codec_duration_test_step(worker)\n')
base=[cache,cc,'-include',str(hook),'-std=c11','-O1','-Wall','-Wextra','-Werror','-Wno-unused-function','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fms-runtime-lib=dll','-D_DLL','-I'+str(ROOT/'include'),'-I'+str(ROOT/'codecs/common'),'-I'+str(ROOT/'esp32_probe/main'),'-I'+str(ROOT/'codecs/lhdcv5')]
commands=[];objects=[];runs=[]
def compile_one(source,obj):
 command=base+['-c',str(source),'-o',str(obj)];subprocess.run(command,env=env,check=True);commands.append(command);objects.append(obj)
shared=[]
for i,p in enumerate([ROOT/'tests/codecs/worker_duration.c',common,ROOT/'src/dbt_avdtp_caps.c',ROOT/'esp32_probe/main/probe_audio_trace.c']):
 obj=OUT/f'shared-{i}.obj';compile_one(p,obj);shared.append(obj)
for detailed in (False,True):
 tag='detailed' if detailed else 'normal';fixture=OUT/('app-'+tag+'.c')
 prefix='#include "dbt_lhdcv5_codec.h"\n#define DBT_PROBE_LHDC_BUILD 1\n'
 if detailed:prefix+='#define DBT_PROBE_SCHEDULER_TRACE 1\n'
 prefix+='static struct {struct {bool delay_enabled;} sink;} profile;\n'
 fixture.write_text(prefix+app_actual+'\nuint32_t test_app_duration_'+tag+'(bool supported,const dbt_codec_config_t *c,const dbt_avdtp_media_t *p){profile.sink.delay_enabled=supported;return media_frame_count(c,p);}\n')
 obj=OUT/('app-'+tag+'.obj');compile_one(fixture,obj);shared.append(obj)
for tag in ('candidate','one-packet-negative','missing-encoded-negative'):
 lbody=lactual
 if tag=='one-packet-negative':
  old='return payload_frame_count(p->payload+2,(uint16_t)(p->size-2u),count)*(uint32_t)config->samples;';assert old in lbody;lbody=lbody.replace(old,'return payload_frame_count(p->payload+2,(uint16_t)(p->size-2u),count)?config->samples:0;')
 obody=oactual.replace('frames+=encoded_frames;','(void)encoded_frames;') if tag=='missing-encoded-negative' else oactual
 ls=OUT/(tag+'-lhdc.c');ls.write_text(lprefix+lbody+'\n');osrc=OUT/(tag+'-delay.c');osrc.write_text(oprefix+obody+'\n'+omodel)
 lo=OUT/(tag+'-lhdc.obj');oo=OUT/(tag+'-delay.obj');compile_one(ls,lo);compile_one(osrc,oo)
 exe=OUT/(tag+'.exe');command=base+list(map(str,shared+[lo,oo]))+['-Wl,/NODEFAULTLIB:libcmt','-o',str(exe)];subprocess.run(command,env=env,check=True);commands.append(command)
 run=subprocess.run([str(exe)],env=env,capture_output=True,text=True,timeout=15);(OUT/(tag+'.stdout.txt')).write_text(run.stdout);(OUT/(tag+'.stderr.txt')).write_text(run.stderr);print(tag,run.stdout.strip(),run.stderr.strip())
 if tag=='candidate':run.check_returncode();assert 'PASS actual duration accounting' in run.stdout
 else:assert run.returncode and 'FAIL' in run.stderr,'Negative control did not detect the defect'
 runs.append({'mode':tag,'exit':run.returncode,'stdout':run.stdout,'stderr':run.stderr})
 pdb=exe.with_suffix('.pdb')
 if pdb.exists():
  data=pdb.read_bytes();digest=hashlib.sha256(data).hexdigest();dest=pdb.with_name(pdb.name+'.'+digest[:12]+'.gz');dest.write_bytes(gzip.compress(data));assert gzip.decompress(dest.read_bytes())==data;(OUT/(tag+'-symbols.json')).write_text(json.dumps({'original':str(pdb),'archive':str(dest),'sha256':digest,'roundtrip_verified':True},indent=2)+'\n');pdb.unlink()
assert all(hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()==h for p,h in hashes.items())
retired=[]
for obj in objects:
 data=obj.read_bytes();assert len(data)>=20 and int.from_bytes(data[:2],'little')==0x8664;retired.append({'path':str(obj),'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest()})
manifest=json.dumps(retired,indent=2)+'\n';(OUT/(hashlib.sha256(manifest.encode()).hexdigest()[:16]+'-retired.json')).write_text(manifest)
for obj in objects:obj.unlink()
subprocess.run([sys.executable,str(ROOT/'tools/check_workspace.py')],check=True)
receipt.write_text(json.dumps({'passed':True,'state':'complete','hardware_accessed':False,'scope':'Actual worker/deferred PCM plus extracted production LHDC framing/delay; synthetic metadata/PCM provider, no physical scheduling or acoustics','sanitizers':['address','undefined'],'sources':hashes,'commands':commands,'runs':runs},indent=2)+'\n')
