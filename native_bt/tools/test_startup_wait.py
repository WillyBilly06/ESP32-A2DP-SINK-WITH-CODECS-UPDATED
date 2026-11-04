"""Actual notification/deadline helper with deterministic completion races; no hardware."""
import os,pathlib,shutil,subprocess
ROOT=pathlib.Path(__file__).resolve().parents[1]
OUT=ROOT/'_build/startup_wait';OUT.mkdir(parents=True,exist_ok=True)
source=OUT/'test.c'
source.write_text(r'''
#include <assert.h>
#include <stdatomic.h>
#include "probe_wait.h"
static _Atomic bool ready;
static int64_t now;
static unsigned calls,mode,last_ticks;
int64_t esp_timer_get_time(void){return now;}
unsigned ulTaskNotifyTake(int clear,unsigned ticks){
    assert(clear==pdTRUE && ticks && ticks!=portMAX_DELAY);
    calls++;last_ticks=ticks;assert(calls<10);
    if(mode==1){atomic_store(&ready,true);return 1;}
    if(mode==2 && calls==1)return 1; /* unrelated pending notification */
    now+=(int64_t)ticks*1000;
    if(mode==2){atomic_store(&ready,true);return 1;}
    return 0;
}
static void reset(void){atomic_store(&ready,false);now=100000;calls=mode=last_ticks=0;}
int main(void){
    reset();atomic_store(&ready,true);assert(probe_wait_flag(&ready,0) && !calls);
    reset();assert(!probe_wait_flag(&ready,100000) && !calls);
    reset();mode=1;assert(probe_wait_flag(&ready,2100000) && calls==1 && last_ticks==2000 && now==100000);
    reset();mode=2;assert(probe_wait_flag(&ready,110000) && calls==2 && now==110000);
    reset();assert(!probe_wait_flag(&ready,110000) && calls==1 && last_ticks==10);
    reset();assert(!probe_wait_flag(&ready,100001) && calls==1 && last_ticks==1);
    reset();now=INT64_C(5000000000000);mode=1;assert(probe_wait_flag(&ready,now+5000000) && calls==1 && last_ticks==5000);
    return 0;
}
''',encoding='utf-8')
env=os.environ.copy()
if os.name=='nt':
    vc=pathlib.Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
    data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
    env.update(dict(line.split('=',1) for line in data.splitlines() if '=' in line and not line.startswith('=')))
cc=shutil.which('clang',path=env.get('PATH')) or r'C:\Program Files\LLVM\bin\clang.exe'
cache=shutil.which('ccache',path=env.get('PATH'))
if not cache:
    candidates=sorted((pathlib.Path.home()/'.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'))
    if candidates:cache=str(candidates[-1])
assert cache,'ccache required'
runtime=[]
if os.name=='nt':
    resource=pathlib.Path(subprocess.check_output([cc,'-print-resource-dir'],text=True).strip())
    shutil.copy2(resource/'lib/windows/clang_rt.asan_dynamic-x86_64.dll',OUT)
    runtime=['-fms-runtime-lib=dll','-D_DLL','-Wl,/NODEFAULTLIB:libcmt']
exe=OUT/('test.exe' if os.name=='nt' else 'test')
subprocess.run([cache,cc,'-std=c11','-O2','-g','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fno-sanitize-recover=all',*runtime,
                '-I',str(ROOT/'esp32_probe/main'),'-I',str(ROOT/'tests/codecs/probe_audio_stubs'),str(source),'-o',str(exe)],env=env,check=True)
subprocess.run([str(exe)],env=env,check=True)
print('PASS seven actual startup-wait cases: completion, pending notification, spurious wake, timeout, sub-tick deadline and long uptime; ASan/UBSan; host only')
