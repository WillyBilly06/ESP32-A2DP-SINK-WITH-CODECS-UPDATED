"""Compile the production terminal handoff with an adversarial approval stub."""
from pathlib import Path
import os,re,shutil,subprocess
root=Path(__file__).resolve().parents[2];out=root/'_build/restart';out.mkdir(exist_ok=True)
s=(root/'esp32_probe/main/probe_main.c').read_text(encoding='utf-8')
m=re.search(r'static void product_restart_step\(uint32_t now\)\{.*?\n\}',s,re.S);assert m
source=r'''#include <stdbool.h>
#include <stdint.h>
#include <assert.h>
static bool product_runtime_ready,fenced,active,quiescent,approve;
static unsigned resets,cancels,revokes,checks;static bool passed_q;
static struct {bool failed;} product_restart;static int product_runtime;
static bool probe_gatt_restart_fenced(void){return fenced;}
static bool probe_restart_active(void *s){(void)s;return active;}
static bool probe_restart_quiescent(void *s,uint32_t n){(void)s;(void)n;return quiescent;}
static void probe_restart_cancel(void *s,uint32_t n){(void)s;(void)n;cancels++;active=false;}
static void probe_gatt_restart_cancel(void){revokes++;fenced=false;}
static bool probe_product_runtime_restart_ready(void *s,uint32_t n,bool q){(void)s;(void)n;checks++;passed_q=q;return approve;}
static void product_restart_now(void){resets++;}
'''+m.group(0)+r'''
int main(void){
 product_runtime_ready=true;fenced=active=approve=true;
 product_restart_step(1);assert(!resets && checks==1 && !passed_q);
 quiescent=true;approve=false;product_restart_step(2);assert(!resets && checks==2 && passed_q);
 approve=true;product_restart.failed=true;product_restart_step(3);assert(!resets && revokes==1 && cancels==1 && checks==2);
 product_restart.failed=false;fenced=false;active=true;product_restart_step(4);assert(cancels==2 && !resets);
 fenced=active=quiescent=approve=true;product_restart_step(5);assert(resets==1 && checks==3);
 product_runtime_ready=false;product_restart_step(6);assert(resets==1 && checks==3);
 return 0;
}
'''
p=out/'main_handoff.c';p.write_text(source,encoding='utf-8');exe=out/('main_handoff.exe' if os.name=='nt' else 'main_handoff')
env=os.environ.copy()
if os.name=='nt':
 vc=Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat');data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True);env.update(dict(x.split('=',1) for x in data.splitlines() if '=' in x and not x.startswith('=')))
cc=shutil.which('clang') or r'C:\Program Files\LLVM\bin\clang.exe'
subprocess.run([cc,'-std=c11','-Wall','-Wextra','-Werror',str(p),'-o',str(exe)],env=env,check=True);subprocess.run([str(exe)],check=True)
print('PASS production restart handoff requires both quiescence and selected/trusted runtime approval')
