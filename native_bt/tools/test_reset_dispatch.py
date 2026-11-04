"""Host adversarial tests of the actual core-0 restart dispatcher; no device."""
import hashlib,json,os,pathlib,shutil,subprocess
ROOT=pathlib.Path(__file__).resolve().parents[1];OUT=ROOT/'_build/reset_dispatch';OUT.mkdir(exist_ok=True)
STUB=OUT/'stubs';(STUB/'freertos').mkdir(parents=True,exist_ok=True)
(STUB/'freertos/FreeRTOS.h').write_text('typedef void *TaskHandle_t;\n#ifndef CONFIG_ESP_TIMER_TASK_AFFINITY_CPU0\n#define CONFIG_ESP_TIMER_TASK_AFFINITY_CPU0 1\n#endif\n')
(STUB/'freertos/task.h').write_text('TaskHandle_t xTaskGetCurrentTaskHandle(void);\nvoid xTaskNotifyGive(TaskHandle_t);\nint xPortGetCoreID(void);\n')
(STUB/'esp_system.h').write_text('void esp_restart(void);\n')
(STUB/'esp_timer.h').write_text('''#include <stdint.h>
#define ESP_OK 0
#define ESP_TIMER_TASK 0
typedef void *esp_timer_handle_t;
typedef struct {void (*callback)(void *);void *arg;int dispatch_method;const char *name;} esp_timer_create_args_t;
int esp_timer_create(const esp_timer_create_args_t *,esp_timer_handle_t *);
int esp_timer_start_once(esp_timer_handle_t,uint64_t);
''')
main=OUT/'main.c';main.write_text('''#include <assert.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include "probe_reset_dispatch.h"
#include "esp_timer.h"
static unsigned creates,starts,resets,wakes;static int scenario,core=1;
static TaskHandle_t task=(void *)0x1234;static void (*callback)(void *);static void *callback_arg;
TaskHandle_t xTaskGetCurrentTaskHandle(void){return task;}
int xPortGetCoreID(void){return core;}
void xTaskNotifyGive(TaskHandle_t t){assert(t==(void *)0x1234);++wakes;}
void esp_restart(void){assert(core==0);++resets;}
int esp_timer_create(const esp_timer_create_args_t *a,esp_timer_handle_t *h){
 assert(a->dispatch_method==ESP_TIMER_TASK);++creates;
 if(scenario==1)return -1;
 callback=a->callback;callback_arg=a->arg;*h=(void *)1;return 0;
}
int esp_timer_start_once(esp_timer_handle_t h,uint64_t delay){assert(h==(void *)1 && delay==250000);++starts;return scenario==2?-1:0;}
int main(int argc,char **argv){
 scenario=argc>1?atoi(argv[1]):0;
 assert(!probe_reset_dispatch_request() && !probe_reset_dispatch_pending() && !probe_reset_dispatch_failed());
 assert(!probe_reset_dispatch_init(0) && !probe_reset_dispatch_init((void *)2));
 if(scenario==1){assert(!probe_reset_dispatch_init(task) && probe_reset_dispatch_failed() && creates==1 && !starts && !resets);return 0;}
 assert(probe_reset_dispatch_init(task) && probe_reset_dispatch_init(task) && creates==1);
 task=(void *)2;assert(!probe_reset_dispatch_request() && !starts);task=(void *)0x1234;
 if(scenario==2){assert(!probe_reset_dispatch_request() && probe_reset_dispatch_failed() && !probe_reset_dispatch_pending());callback(callback_arg);assert(!resets);return 0;}
 assert(probe_reset_dispatch_request() && probe_reset_dispatch_pending() && !resets && starts==1);
 assert(!probe_reset_dispatch_request() && starts==1);
 if(scenario==3){callback(callback_arg);assert(probe_reset_dispatch_failed() && !probe_reset_dispatch_pending() && wakes==1 && !resets);callback(callback_arg);assert(wakes==1 && !resets);return 0;}
 core=0;callback(callback_arg);assert(resets==1 && probe_reset_dispatch_pending());
 callback(callback_arg);assert(resets==1 && !probe_reset_dispatch_request());
 puts("PASS deferred owner-approved reset executes once on core 0");return 0;
}
''')
env=os.environ.copy()
if os.name=='nt':
    vc=pathlib.Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat');data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
    env.update(dict(x.split('=',1) for x in data.splitlines() if '=' in x and not x.startswith('=')))
cc=shutil.which('clang',path=env.get('PATH')) or (r'C:\Program Files\LLVM\bin\clang.exe' if os.name=='nt' else 'clang')
exe=OUT/('dispatcher.exe' if os.name=='nt' else 'dispatcher')
source=ROOT/'esp32_probe/main/probe_reset_dispatch.c'
args=[cc,'-std=c11','-O2','-g','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fno-sanitize-recover=all','-I'+str(STUB),'-I'+str(ROOT/'esp32_probe/main'),str(source),str(main),'-o',str(exe)]
if os.name=='nt':args+=['-fms-runtime-lib=dll','-D_DLL','-Wl,/NODEFAULTLIB:libcmt']
tracked=[source,ROOT/'esp32_probe/main/probe_reset_dispatch.h',main,pathlib.Path(__file__).resolve()]+list(STUB.rglob('*.h'))
before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked};report_path=OUT/'results.json';report_path.write_text(json.dumps({'passed':False,'state':'building'})+'\n')
subprocess.run(args,check=True,env=env)
if os.name=='nt':
    resource=pathlib.Path(subprocess.check_output([cc,'-print-resource-dir'],text=True).strip());runtime=resource/'lib/windows/clang_rt.asan_dynamic-x86_64.dll';shutil.copy2(runtime,OUT/runtime.name)
for scenario in range(4):subprocess.run([str(exe),str(scenario)],check=True,env=env,timeout=10)
negative=subprocess.run(args+['-DCONFIG_ESP_TIMER_TASK_AFFINITY_CPU0=0'],env=env,capture_output=True,text=True)
assert negative.returncode!=0 and 'restart timer must dispatch on core 0' in negative.stderr
assert all(hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()==digest for p,digest in before.items())
report_path.write_text(json.dumps({'passed':True,'scenarios':4,'wrong_affinity_compile_rejected':True,'sanitizers':['address','undefined'],'scope':'actual dispatcher with host timer/core/reset stubs; no physical reset or native OTA test','sources':before,'command':args,'hardware_accessed':False},indent=2)+'\n')
print('PASS reset dispatcher normal/OOM/arm-failure/wrong-core and compile-time affinity rejection')
