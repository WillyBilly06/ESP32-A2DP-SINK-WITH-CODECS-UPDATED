"""Actual runtime-only CMake/image/CLI admission; no hardware."""
import ast,hashlib,json,pathlib,re,subprocess,sys
ROOT=pathlib.Path(__file__).resolve().parents[1];OUT=ROOT/'_build/task_runtime_only_guard';OUT.mkdir(exist_ok=True)
guard=OUT/'guard.cmake';guard.write_text((ROOT/'esp32_probe/CMakeLists.txt').read_text().split('set(COMPONENTS main)')[0])
base={k:'ON' for k in ['TASK_RUNTIME_ONLY','TASK_RUNTIME_STATS','SCHEDULER_TRACE','STAY_NATIVE','ASYNC_AUDIO','KEEP_OWNED_ACTIVE','PRODUCT_IO','PRODUCT_SECURITY','AUDIO_TEST','AUDIO_OUTPUT','SBC_BUILD','CLASSIC_SECURITY_TEST','SMP','GATT_TEST','AVRCP']};base['DECODER_PRIORITY']='21';base['SCHEDULER_TRACE']='OFF'
cases=[({},True),({'TASK_RUNTIME_ONLY':'OFF','SCHEDULER_TRACE':'ON'},True),({'TASK_RUNTIME_STATS':'OFF'},False),({'SCHEDULER_TRACE':'ON'},False),({'TASK_CYCLE_ACCOUNTING':'ON'},False)]
for change,valid in cases:
    r=subprocess.run(['cmake',*[f'-DDBT_PROBE_{k}={v}' for k,v in (base|change).items()],'-P',str(guard)],capture_output=True,text=True)
    assert (r.returncode==0)==valid,r.stdout+r.stderr
tree=ast.parse((ROOT/'tools/validate_local_image.py').read_text())
def pos(name):
    return next(i for i,n in enumerate(tree.body) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id==name for t in n.targets))
actual=compile(ast.Module(body=tree.body[pos('task_runtime_only'):pos('volume_ramp')],type_ignores=[]),'<actual runtime-only admission>','exec')
units=['probe_main.c','probe_scheduler.c','probe_audio.c'];macro='cc -DDBT_PROBE_TASK_RUNTIME_ONLY=1';count=0
def check(changes,valid):
    global count
    state={'build_status':{'task_runtime_only':True},'cache':'DBT_PROBE_TASK_RUNTIME_ONLY:BOOL=ON','task_runtime_stats':True,'task_cycle_accounting':False,'scheduler_trace':False,'commands':[{'file':n,'command':macro} for n in units],'entry_name':'probe_main.c','Path':pathlib.Path,'re':re}
    state.update(changes)
    try:exec(actual,state)
    except AssertionError:assert not valid
    else:assert valid
    count+=1
check({},True);check({'build_status':{},'cache':'','commands':[{'file':n,'command':'cc'} for n in units]},True)
for change in [{'build_status':{'task_runtime_only':1}},{'cache':''},{'task_runtime_stats':False},{'task_cycle_accounting':True},{'scheduler_trace':True},{'commands':[]}]:check(change,False)
for bad in ['cc',macro.replace('=1','=0'),macro+' -UDBT_PROBE_TASK_RUNTIME_ONLY',macro+' -DDBT_PROBE_TASK_RUNTIME_ONLY=1',macro+' -DDBT_PROBE_SCHEDULER_TRACE=1']:
    check({'commands':[{'file':n,'command':bad} for n in units]},False)
cli=0
if sys.platform=='win32':
    real=ROOT/'esp32_probe/sdkconfig';before=hashlib.sha256(real.read_bytes()).hexdigest()
    pwsh=pathlib.Path.home()/'.cache/codex-runtimes/codex-primary-runtime/dependencies/native/powershell/pwsh.exe'
    r=subprocess.run([str(pwsh),'-NoProfile','-File',str(ROOT/'tools/build_probe.ps1'),'-TaskRuntimeOnly','-TaskCycleAccounting'],capture_output=True,text=True)
    assert r.returncode and 'Runtime-only observation excludes' in r.stderr;assert hashlib.sha256(real.read_bytes()).hexdigest()==before;cli=1
    r=subprocess.run([str(pwsh),'-NoProfile','-File',str(ROOT/'tools/build_probe.ps1'),'-TaskRuntimeOnly','-SchedulerTrace'],capture_output=True,text=True)
    assert r.returncode and 'excludes detailed scheduler tracing' in r.stderr;assert hashlib.sha256(real.read_bytes()).hexdigest()==before;cli+=1
(OUT/'results.json').write_text(json.dumps({'passed':True,'actual_cmake_cases':len(cases),'actual_image_cases':count,'cli_rejections':cli,'hardware_accessed':False},indent=2)+'\n')
print(f'PASS runtime-only admission: {len(cases)} CMake,{count} image,{cli} CLI')
