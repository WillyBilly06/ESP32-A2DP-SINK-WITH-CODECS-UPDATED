"""Execute actual controlled DSP comparison with host stubs; no target timing."""
import argparse,hashlib,json,os,pathlib,shutil,subprocess
parser=argparse.ArgumentParser();group=parser.add_mutually_exclusive_group();group.add_argument("--profile-stages",action="store_true");group.add_argument("--inline-filter",action="store_true");options=parser.parse_args()
ROOT=pathlib.Path(__file__).resolve().parents[1];OUT=ROOT/'_build/dsp_variants';OUT.mkdir(exist_ok=True)
STUB=OUT/'stubs';(STUB/'freertos').mkdir(parents=True,exist_ok=True)
(STUB/'esp_cpu.h').write_text('#include <stdint.h>\nuint32_t esp_cpu_get_cycle_count(void);\n')
(STUB/'esp_timer.h').write_text('#include <stdint.h>\nint64_t esp_timer_get_time(void);\n')
(STUB/'esp_heap_caps.h').write_text('#include <stddef.h>\n#define MALLOC_CAP_INTERNAL 1\n#define MALLOC_CAP_8BIT 2\nvoid *heap_caps_aligned_alloc(size_t,size_t,unsigned);\nvoid heap_caps_free(void *);\n')
(STUB/'freertos/FreeRTOS.h').write_text('typedef unsigned TickType_t;\n#define pdMS_TO_TICKS(n) (n)\n')
(STUB/'freertos/task.h').write_text('void vTaskDelay(unsigned);\nunsigned uxTaskGetStackHighWaterMark(void *);\n')
main=OUT/'main.c';main.write_text('''#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include "probe_dsp_compare.h"
static uint32_t cycles;
uint32_t esp_cpu_get_cycle_count(void){return ++cycles;}
int64_t esp_timer_get_time(void){return (int64_t)clock()*1000000/CLOCKS_PER_SEC;}
void *heap_caps_aligned_alloc(size_t alignment,size_t bytes,unsigned caps){(void)alignment;(void)caps;return malloc(bytes);}
void heap_caps_free(void *p){free(p);}
void vTaskDelay(unsigned ticks){(void)ticks;}
unsigned uxTaskGetStackHighWaterMark(void *task){(void)task;return 8192;}
int main(void){return probe_dsp_compare()?0:1;}
''')
env=os.environ.copy()
if os.name=='nt':
    vc=pathlib.Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
    data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
    env.update(dict(x.split('=',1) for x in data.splitlines() if '=' in x and not x.startswith('=')))
cc=shutil.which('clang',path=env.get('PATH')) or (r'C:\Program Files\LLVM\bin\clang.exe' if os.name=='nt' else 'clang')
cache=shutil.which('ccache',path=env.get('PATH'))
if not cache:
    candidates=sorted((pathlib.Path.home()/'.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'))
    if candidates:cache=str(candidates[-1])
assert cache,'ccache required';env.pop('CCACHE_DISABLE',None)
base=[cache,cc,'-std=c11','-g','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer','-I'+str(STUB),'-I'+str(ROOT/'esp32_probe/main'),'-I'+str(ROOT.parent/'bt_audio_sink/main/dsp')]
if os.name=='nt':base+=['-fms-runtime-lib=dll','-D_DLL']
candidate='probe_dsp_profile.c' if options.profile_stages else 'probe_dsp_inline.c' if options.inline_filter else 'probe_dsp_o3.c'
sources=[(ROOT/'esp32_probe/main/probe_dsp_os.c','-Os'),(ROOT/'esp32_probe/main'/candidate,'-Os' if options.profile_stages or options.inline_filter else '-O3'),(ROOT/'esp32_probe/main/probe_dsp_compare.c','-O2'),(ROOT/'esp32_probe/main/probe_diagnostic_stats.c','-O2'),(main,'-O2')]
if not options.profile_stages and not options.inline_filter:
    sources.extend((ROOT/'esp32_probe/main'/name,'-Os') for name in ('probe_dsp_block.c','probe_dsp_transposed.c'))
if options.profile_stages:base+=['-DDBT_DSP_STAGE_PROFILE=1']
if options.inline_filter:base+=['-DDBT_PROBE_DSP_INLINE_CHECK=1']
tracked=[p for p,_ in sources]+list((ROOT/'esp32_probe/main').glob('*.h'))+list(STUB.rglob('*.h'))+list((ROOT.parent/'bt_audio_sink/main/dsp').glob('fixed_dsp*'))+[pathlib.Path(__file__).resolve()]
before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked}
objects=[];commands=[]
for source,optimization in sources:
    obj=OUT/(source.stem+('.obj' if os.name=='nt' else '.o'));cmd=base+[optimization,'-c',str(source),'-o',str(obj)]
    subprocess.run(cmd,check=True,env=env);objects.append(obj);commands.append(cmd)
exe=OUT/('variants.exe' if os.name=='nt' else 'variants')
cmd=base+list(map(str,objects))+['-o',str(exe)]
if os.name=='nt':cmd+=['-Wl,/NODEFAULTLIB:libcmt']
subprocess.run(cmd,check=True,env=env);commands.append(cmd)
if os.name=='nt':
    resource=pathlib.Path(subprocess.check_output([cc,'-print-resource-dir'],text=True).strip());runtime=resource/'lib/windows/clang_rt.asan_dynamic-x86_64.dll';shutil.copy2(runtime,OUT/runtime.name)
result=subprocess.run([str(exe)],check=True,env=env,timeout=60,capture_output=True,text=True)
events=[json.loads(line) for line in result.stdout.splitlines() if line.startswith('{')]
cases=[e for e in events if e['probe']=='dsp_variant_case'];summary=[e for e in events if e['probe']=='dsp_variant_summary']
groups=18 if options.profile_stages or options.inline_filter else 27
variants=2 if options.profile_stages or options.inline_filter else 4
assert len(cases)==groups*variants and all(e['equivalent'] for e in cases) and summary[-1]['passed'] and summary[-1]['cases']==groups
if options.profile_stages:
    stages=[e for e in events if e['probe']=='dsp_internal_stages']
    assert len(stages)==18 and all(e['valid'] and e['pipeline_muted_start'] and e['proven_sum_enabled']==1 for e in stages)
assert all(hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()==digest for p,digest in before.items())
(OUT/('inline-results.json' if options.inline_filter else 'profile-results.json' if options.profile_stages else 'results.json')).write_text(json.dumps({'passed':True,'case_groups':groups,'variants':['Os_current','Os_force_inline'] if options.inline_filter else ['Os_split_reference','Os_profiled_proven'] if options.profile_stages else ['Os_configured','O3_configured','Os_block','Os_transposed_block'],'sanitizers':['address','undefined'],'signed_overflow_wrapping':False,'scope':'host actual comparison runner, full PCM/state equality; timer/cycles/RTOS/heap are stubs, no hardware performance evidence','sources':before,'commands':commands,'hardware_accessed':False},indent=2)+'\n')
print(f'PASS {groups} controlled DSP groups with {variants} variants, profile='+str(options.profile_stages)+', inline='+str(options.inline_filter)+', full output/state comparison, host ASan/UBSan; no target timings')
