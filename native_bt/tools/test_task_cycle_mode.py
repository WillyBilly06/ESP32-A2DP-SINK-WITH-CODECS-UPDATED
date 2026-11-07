"""Check actual task-cycle CMake/image/SDK admission, without hardware."""
import ast,json,pathlib,subprocess,sys,re
ROOT=pathlib.Path(__file__).resolve().parents[1]
OUT=ROOT/'_build/task_cycle_guard';OUT.mkdir(exist_ok=True)
guard=OUT/'guard.cmake'
guard.write_text((ROOT/'esp32_probe/CMakeLists.txt').read_text().split('set(COMPONENTS main)')[0])
base={k:'ON' for k in ['TASK_CYCLE_ACCOUNTING','TASK_RUNTIME_STATS','SCHEDULER_TRACE','STAY_NATIVE','ASYNC_AUDIO','KEEP_OWNED_ACTIVE','PRODUCT_IO','PRODUCT_SECURITY','AUDIO_TEST','AUDIO_OUTPUT','SBC_BUILD','CLASSIC_SECURITY_TEST','SMP','GATT_TEST','AVRCP']}
base['DECODER_PRIORITY']='21'
cases=[({},True),({'TASK_CYCLE_ACCOUNTING':'OFF'},True),({'TASK_RUNTIME_STATS':'OFF'},False),({'SCHEDULER_TRACE':'OFF'},False),({'ASYNC_AUDIO':'OFF'},False),({'STAY_NATIVE':'OFF'},False)]
for change,valid in cases:
    r=subprocess.run(['cmake',*[f'-DDBT_PROBE_{k}={v}' for k,v in (base|change).items()],'-P',str(guard)],capture_output=True,text=True)
    assert (r.returncode==0)==valid,r.stdout+r.stderr
tree=ast.parse((ROOT/'tools/validate_local_image.py').read_text())
def at(name):
    return next(i for i,n in enumerate(tree.body) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id==name for t in n.targets))
actual=compile(ast.Module(body=tree.body[at('task_cycle_accounting'):at('volume_ramp')],type_ignores=[]),'<actual cycle admission>','exec')
macro='cc -DDBT_PROBE_TASK_CYCLE_ACCOUNTING=1'
units=['probe_main.c','probe_scheduler.c','probe_audio.c','probe_output.c','probe_task_cycles.c']
kernel={'file':'D:/sdk/components/freertos/FreeRTOS-Kernel/tasks.c','command':'cc -include D:/candidate/main/probe_task_accounting_trace.h'}
commands=[{'file':n,'command':macro} for n in units]+[kernel]
count=0
def check(changes,valid):
    global count
    state={'build_status':{'task_cycle_accounting':True},'cache':'DBT_PROBE_TASK_CYCLE_ACCOUNTING:BOOL=ON','task_runtime_stats':True,'commands':[dict(e) for e in commands],'entry_name':'probe_main.c','Path':pathlib.Path,'re':re}
    state.update(changes)
    try:exec(actual,state)
    except AssertionError:assert not valid
    else:assert valid
    count+=1
check({},True)
off={'build_status':{},'cache':'','commands':[{'file':n,'command':'cc'} for n in units[:-1]]+[{'file':kernel['file'],'command':'cc'}]}
check(off,True)
for change in [
    {'build_status':{'task_cycle_accounting':1}},
    {'task_runtime_stats':False},
    {'cache':''},
    {'commands':commands[:-1]},
    {'commands':commands[:-2]+[kernel]},
    {'commands':commands+[commands[-2]]},
    {'commands':commands[:-1]+[kernel|{'command':'cc -include D:/wrong.h'}]},
    {'commands':commands[:-1]+[kernel|{'command':kernel['command']+' -include D:/candidate/main/probe_task_accounting_trace.h'}]},
    {'commands':commands+[{'file':'app.c','command':kernel['command']}]},
    {'commands':commands[:-1]+[kernel|{'command':'cc -I D:/candidate/main/probe_task_accounting_trace.h'}]},
]:check(change,False)
for bad in ['cc',macro.replace('=1','=0'),macro+' -UDBT_PROBE_TASK_CYCLE_ACCOUNTING',macro+' -DDBT_PROBE_TASK_CYCLE_ACCOUNTING=1']:
    check({'commands':[{'file':n,'command':bad} for n in units]+[kernel]},False)
check(off|{'commands':off['commands']+[{'file':'probe_task_cycles.c','command':'cc'}]},False)
check(off|{'commands':off['commands'][:-1]+[kernel]},False)
sdk=OUT/'sdk';sdk.mkdir(exist_ok=True)
config=sdk/'sdkconfig.h'
fixed={'CONFIG_FREERTOS_SMP':0,'CONFIG_FREERTOS_NUMBER_OF_CORES':2,'CONFIG_ESP_TRACE_ENABLE':0,'CONFIG_PM_ENABLE':0,'CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ':240}
fake=OUT/'kernel.c'
fake.write_text('#define portGET_CORE_ID() 1u\nstatic void *pxCurrentTCBs[2];\nvoid expansion(void){traceTASK_SWITCHED_OUT();traceTASK_SWITCHED_IN();}\n')
compiler=r'C:/Program Files/LLVM/bin/clang.exe'
header=ROOT/'esp32_probe/main/probe_task_accounting_trace.h'
sdk_cases=[({},[],True),({'CONFIG_FREERTOS_SMP':1},[],False),({'CONFIG_FREERTOS_NUMBER_OF_CORES':1},[],False),({'CONFIG_ESP_TRACE_ENABLE':1},[],False),({'CONFIG_PM_ENABLE':1},[],False),({'CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ':160},[],False),({},['-DtraceTASK_SWITCHED_IN()=0'],False)]
for changes,extra,valid in sdk_cases:
    config.write_text('\n'.join(f'#define {k} {v}' for k,v in (fixed|changes).items())+'\n')
    r=subprocess.run([compiler,'-std=c11','-Werror','-fsyntax-only','-I'+str(sdk),'-include',str(header),*extra,str(fake)],capture_output=True,text=True)
    assert (r.returncode==0)==valid,r.stdout+r.stderr
cli=0
if sys.platform=='win32':
    r=subprocess.run(['powershell','-NoProfile','-ExecutionPolicy','Bypass','-File',str(ROOT/'tools/build_probe.ps1'),'-TaskCycleAccounting'],capture_output=True,text=True)
    assert r.returncode and 'Task-cycle accounting requires' in r.stderr;cli=1
(OUT/'results.json').write_text(json.dumps({'passed':True,'actual_cmake_cases':len(cases),'actual_image_cases':count,'actual_sdk_header_cases':len(sdk_cases),'cli_rejections':cli,'hardware_accessed':False},indent=2)+'\n')
print(f'PASS task-cycle admission: {len(cases)} CMake,{count} image,{len(sdk_cases)} SDK header,{cli} CLI')
