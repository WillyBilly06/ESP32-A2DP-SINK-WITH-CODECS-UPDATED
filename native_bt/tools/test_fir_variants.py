"""Host actual FIR compiler-comparison runner; stub timings are not evidence."""
import hashlib,json,os,pathlib,shutil,subprocess
ROOT=pathlib.Path(__file__).resolve().parents[1];OUT=ROOT/'_build/fir_variants';OUT.mkdir(exist_ok=True)
STUB=OUT/'stubs';(STUB/'freertos').mkdir(parents=True,exist_ok=True)
(STUB/'esp_cpu.h').write_text('#include <stdint.h>\nuint32_t esp_cpu_get_cycle_count(void);\n')
(STUB/'esp_timer.h').write_text('#include <stdint.h>\nint64_t esp_timer_get_time(void);\n')
(STUB/'esp_heap_caps.h').write_text('#include <stddef.h>\n#define MALLOC_CAP_INTERNAL 1\n#define MALLOC_CAP_8BIT 2\nvoid *heap_caps_aligned_alloc(size_t,size_t,unsigned);\nvoid heap_caps_free(void *);\n')
(STUB/'freertos/FreeRTOS.h').write_text('typedef unsigned TickType_t;\n#define pdMS_TO_TICKS(n) (n)\n')
(STUB/'freertos/task.h').write_text('void vTaskDelay(unsigned);\nunsigned uxTaskGetStackHighWaterMark(void *);\n')
main=OUT/'main.c';main.write_text('''#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include "probe_fir_compare.h"
static uint32_t cycles;
uint32_t esp_cpu_get_cycle_count(void){return ++cycles;}
int64_t esp_timer_get_time(void){return (int64_t)clock()*1000000/CLOCKS_PER_SEC;}
void *heap_caps_aligned_alloc(size_t a,size_t n,unsigned f){(void)a;(void)f;return malloc(n);}
void heap_caps_free(void *p){free(p);}
void vTaskDelay(unsigned n){(void)n;}
unsigned uxTaskGetStackHighWaterMark(void *t){(void)t;return 8192;}
int main(void){return probe_fir_compare()?0:1;}
''')
env=os.environ.copy()
if os.name=='nt':
    vc=pathlib.Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat');data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
    env.update(dict(x.split('=',1) for x in data.splitlines() if '=' in x and not x.startswith('=')))
cc=shutil.which('clang',path=env.get('PATH')) or (r'C:\Program Files\LLVM\bin\clang.exe' if os.name=='nt' else 'clang')
base=[cc,'-std=c11','-g','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer','-I'+str(STUB),'-I'+str(ROOT/'esp32_probe/main'),'-I'+str(ROOT/'include')]
if os.name=='nt':base+=['-fms-runtime-lib=dll','-D_DLL']
sources=[(ROOT/'esp32_probe/main/probe_fir_os.c','-Os'),(ROOT/'esp32_probe/main/probe_fir_o2.c','-O2'),(ROOT/'esp32_probe/main/probe_fir_o3.c','-O3'),(ROOT/'esp32_probe/main/probe_fir_compare.c','-O2'),(ROOT/'esp32_probe/main/probe_diagnostic_stats.c','-O2'),(main,'-O2')]
tracked=[p for p,_ in sources]+list(STUB.rglob('*.h'))+[ROOT/'src/dbt_resampler.c',ROOT/'src/dbt_resampler_coeffs.inc',ROOT/'include/dbt_resampler.h',ROOT/'esp32_probe/main/probe_fir_compare.h',ROOT/'esp32_probe/main/probe_diagnostic_stats.h',pathlib.Path(__file__).resolve()]
before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked};objects=[];commands=[]
report_path=OUT/'results.json';report_path.write_text(json.dumps({'passed':False,'state':'building','hardware_accessed':False})+'\n')
for source,optimization in sources:
    obj=OUT/(source.stem+('.obj' if os.name=='nt' else '.o'));cmd=base+[optimization,'-c',str(source),'-o',str(obj)]
    subprocess.run(cmd,check=True,env=env);objects.append(obj);commands.append(cmd)
exe=OUT/('variants.exe' if os.name=='nt' else 'variants');cmd=base+list(map(str,objects))+['-o',str(exe)]
if os.name=='nt':cmd+=['-Wl,/NODEFAULTLIB:libcmt']
subprocess.run(cmd,check=True,env=env);commands.append(cmd)
if os.name=='nt':
    resource=pathlib.Path(subprocess.check_output([cc,'-print-resource-dir'],text=True).strip());runtime=resource/'lib/windows/clang_rt.asan_dynamic-x86_64.dll';shutil.copy2(runtime,OUT/runtime.name)
result=subprocess.run([str(exe)],check=True,env=env,timeout=60,capture_output=True,text=True)
events=[json.loads(x) for x in result.stdout.splitlines() if x.startswith('{')]
rows=[e for e in events if e['probe']=='fir_variant_case'];summary=[e for e in events if e['probe']=='fir_variant_summary']
assert len(rows)==81 and all(e['equivalent'] for e in rows) and summary[-1]['passed'] and summary[-1]['cases']==27
for rate in (48000,96000,192000):
    for ppm in (0,500,-500):
        group=[e for e in rows if e['rate']==rate and e['ppm']==ppm]
        assert len(group)==9 and len({e['pcm_hash'] for e in group})==1
assert all(hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()==digest for p,digest in before.items())
report_path.write_text(json.dumps({'passed':True,'case_groups':27,'variant_rows':81,'variants':['Os','O2','O3'],'sanitizers':['address','undefined'],'scope':'host actual runner and output/full-state equivalence; clocks/RTOS/heap are stubs, no target performance evidence','commands':commands,'sources':before,'hardware_accessed':False},indent=2)+'\n')
print('PASS 27 three-way FIR case groups, 0/+500/-500 ppm, all samples/state match; host ASan/UBSan only')
