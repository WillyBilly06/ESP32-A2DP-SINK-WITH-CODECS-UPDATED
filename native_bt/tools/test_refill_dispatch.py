"""Actual bounded IRQ refill-dispatch policy and callback epilogue. No hardware."""
import gzip,hashlib,json,os,pathlib,shutil,subprocess,sys
ROOT=pathlib.Path(__file__).resolve().parents[1];OUT=ROOT/'_build/refill_dispatch';OUT.mkdir(exist_ok=True)
receipt=OUT/'results.json';receipt.write_text(json.dumps({'passed':False,'state':'building','hardware_accessed':False})+'\n')
source=ROOT/'esp32_probe/main/probe_output.c';text=source.read_text()
tracked=[source,ROOT/'esp32_probe/main/probe_output.h',ROOT/'esp32_probe/main/CMakeLists.txt',ROOT/'esp32_probe/main/probe_scheduler.lf',pathlib.Path(__file__).resolve()]
hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked}
def extract(signature):
 start=text.index(signature);opening=text.index('{',start);depth=1;end=opening+1
 while depth:
  if text[end]=='{':depth+=1
  elif text[end]=='}':depth-=1
  end+=1
 return text[start:end]
actual=extract('static bool IRAM_ATTR refill_dispatch_needed(')+'\n'+extract('static bool IRAM_ATTR finish_sent(')
prefix=r'''
#include <stdint.h>
#include <stdbool.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>
#define IRAM_ATTR
#define ESP_PLATFORM 1
#define pdTRUE 1
#define DMA_DESCRIPTORS 2u
#define REFILL_COPY_BUDGET_US 32u
typedef int BaseType_t;
typedef void *TaskHandle_t;
typedef struct {
 _Atomic uint32_t refill_pending_index,failed,stop_requested,render_core,refill_deadline_us,refill_dispatch_requests;
 _Atomic uint32_t callback_calls,callback_core,callback_elapsed_us,callback_max_us;
 _Atomic(TaskHandle_t) task;
 bool stopping;
 struct {struct {_Atomic uint32_t read,written;} input,output;} audio;
} output_t;
static unsigned core,getter_calls,clock_calls;
static uint32_t clock_us;
static TaskHandle_t current;
static unsigned xPortGetCoreID(void){return core;}
static TaskHandle_t xTaskGetCurrentTaskHandle(void){++getter_calls;return current;}
static int64_t esp_timer_get_time(void){++clock_calls;return clock_us;}
#define CHECK(x) do{if(!(x)){fprintf(stderr,"FAIL line %u case %u: %s\n",(unsigned)__LINE__,cases,#x);return 1;}}while(0)
'''
main=r'''
int main(void){
 const uint32_t times[]={0,100000,UINT32_MAX-16u,UINT32_MAX};const int remaining[]={-1,0,32,33,936};unsigned cases=0;
 for(unsigned t=0;t<4;++t)for(unsigned c=0;c<2;++c)for(unsigned r=0;r<2;++r)for(unsigned pending=0;pending<4;++pending)
 for(unsigned flags=0;flags<8;++flags)for(unsigned data=0;data<4;++data)for(unsigned task=0;task<3;++task)
 for(unsigned wake=0;wake<2;++wake)for(unsigned time=0;time<5;++time)for(unsigned wrap=0;wrap<2;++wrap){
  output_t o={0};core=c;clock_us=times[t];getter_calls=clock_calls=0;
  atomic_store(&o.refill_pending_index,pending==3?UINT32_MAX:pending);atomic_store(&o.render_core,r);
  o.stopping=(flags&1u)!=0;atomic_store(&o.failed,flags&2u);atomic_store(&o.stop_requested,flags&4u);
  atomic_store(&o.audio.input.read,UINT32_MAX);atomic_store(&o.audio.input.written,UINT32_MAX+(data&1u));
  atomic_store(&o.audio.output.read,UINT32_MAX);atomic_store(&o.audio.output.written,UINT32_MAX+((data>>1u)&1u));
  TaskHandle_t renderer=task?(void *)(uintptr_t)3:NULL;atomic_store(&o.task,renderer);current=task==1?renderer:(void *)(uintptr_t)7;
  atomic_store(&o.refill_deadline_us,clock_us+(uint32_t)remaining[time]);atomic_store(&o.refill_dispatch_requests,wrap?UINT32_MAX:0);++cases;
  bool early=!wake && pending<2 && !flags && c==1 && r==1 && data && task;
#ifndef DBT_PROBE_STAY_NATIVE
  early=false;
#endif
  bool dispatch=early && task==2 && remaining[time]>32;
  bool result=finish_sent(&o,clock_us-5u,(BaseType_t)wake);
  CHECK(result==(wake || dispatch));CHECK(atomic_load(&o.refill_dispatch_requests)==(wrap?UINT32_MAX:0u)+(uint32_t)dispatch);
  CHECK(getter_calls==(unsigned)early);CHECK(clock_calls==1u+(unsigned)(early && task==2));
  CHECK(atomic_load(&o.callback_calls)==1 && atomic_load(&o.callback_elapsed_us)==5 && atomic_load(&o.callback_max_us)==5 && atomic_load(&o.callback_core)==c);
 }
 printf("PASS actual IRQ dispatch and callback accounting: %u cases; native/disabled, existing wake, self/other task, useful data, core/stop/failure/deadline fences, timestamp/counter wrap\n",cases);return 0;
}
'''
env=os.environ.copy();vc=pathlib.Path(r'C:/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools/VC/Auxiliary/Build/vcvars64.bat')
values=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True);env.update(dict(x.split('=',1) for x in values.splitlines() if '=' in x and not x.startswith('=')))
cc=shutil.which('clang',path=env.get('PATH')) or r'C:/Program Files/LLVM/bin/clang.exe';choices=sorted((pathlib.Path.home()/'.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'));assert choices;cache=str(choices[-1]);env.pop('CCACHE_DISABLE',None)
resource=pathlib.Path(subprocess.check_output([cc,'-print-resource-dir'],text=True).strip());env['PATH']=str(resource/'lib/windows')+os.pathsep+env.get('PATH','')
commands=[];runs=[];objects=[]
for enabled,negative in [(False,False),(True,False),(True,True)]:
 tag='old-no-dispatch-negative' if negative else 'native' if enabled else 'disabled'
 body=actual.replace('return wake==pdTRUE || dispatch;','return wake==pdTRUE || (dispatch && false);') if negative else actual
 fixture=OUT/(tag+'.c');fixture.write_text(prefix+body+'\n'+main);obj=OUT/(tag+'.obj');exe=OUT/(tag+'.exe');objects.append((obj,exe))
 base=[cache,cc,'-std=c11','-O1','-Wall','-Wextra','-Werror','-Wno-unused-function','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fms-runtime-lib=dll','-D_DLL']
 if enabled:base+=['-DDBT_PROBE_STAY_NATIVE=1']
 command=base+['-c',str(fixture),'-o',str(obj)];subprocess.run(command,env=env,check=True);commands.append(command)
 command=base+[str(obj),'-Wl,/NODEFAULTLIB:libcmt','-o',str(exe)];subprocess.run(command,env=env,check=True);commands.append(command)
 run=subprocess.run([str(exe)],env=env,capture_output=True,text=True,timeout=15);(OUT/(tag+'.stdout.txt')).write_text(run.stdout);(OUT/(tag+'.stderr.txt')).write_text(run.stderr);print(tag,run.stdout.strip(),run.stderr.strip())
 if negative:assert run.returncode and 'result==' in run.stderr,'Old missing-dispatch behavior was not detected'
 else:run.check_returncode();assert 'PASS actual IRQ dispatch' in run.stdout
 runs.append({'native':enabled,'negative':negative,'exit':run.returncode,'stdout':run.stdout,'stderr':run.stderr})
assert all(hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()==h for p,h in hashes.items())
retired=[]
for obj,exe in objects:
 data=obj.read_bytes();assert len(data)>=20 and int.from_bytes(data[:2],'little')==0x8664;retired.append({'object':str(obj),'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest(),'executable':str(exe)})
manifest=json.dumps(retired,indent=2)+'\n';(OUT/(hashlib.sha256(manifest.encode()).hexdigest()[:16]+'-retired.json')).write_text(manifest)
for obj,exe in objects:
 obj.unlink();pdb=exe.with_suffix('.pdb')
 if pdb.exists():
  data=pdb.read_bytes();digest=hashlib.sha256(data).hexdigest();dest=pdb.with_name(pdb.name+'.'+digest[:12]+'.gz');dest.write_bytes(gzip.compress(data));assert gzip.decompress(dest.read_bytes())==data;(OUT/(pdb.stem+'-symbols.json')).write_text(json.dumps({'original':str(pdb),'archive':str(dest),'sha256':digest,'roundtrip_verified':True},indent=2)+'\n');pdb.unlink()
subprocess.run([sys.executable,str(ROOT/'tools/check_workspace.py')],check=True)
receipt.write_text(json.dumps({'passed':True,'state':'complete','hardware_accessed':False,'scope':'Actual extracted IRQ dispatch policy and callback epilogue; no real scheduler/ESP32 timing proof','sanitizers':['address','undefined'],'sources':hashes,'commands':commands,'runs':runs},indent=2)+'\n')
