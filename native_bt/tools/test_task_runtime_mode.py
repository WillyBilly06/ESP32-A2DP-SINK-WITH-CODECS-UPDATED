"""Actual runtime-observer admission and private SDK option transitions. No hardware."""
import ast,hashlib,json,pathlib,re,subprocess,sys
ROOT=pathlib.Path(__file__).resolve().parents[1];OUT=ROOT/'_build/task_runtime_guard';OUT.mkdir(exist_ok=True)
guard=OUT/'guard.cmake';guard.write_text((ROOT/'esp32_probe/CMakeLists.txt').read_text().split('set(COMPONENTS main)')[0])
base={k:'ON' for k in ['TASK_RUNTIME_STATS','SCHEDULER_TRACE','STAY_NATIVE','ASYNC_AUDIO','KEEP_OWNED_ACTIVE','PRODUCT_IO','PRODUCT_SECURITY','AUDIO_TEST','AUDIO_OUTPUT','SBC_BUILD','CLASSIC_SECURITY_TEST','SMP','GATT_TEST','AVRCP']};base['DECODER_PRIORITY']='21'
cases=[({},True),({'TASK_RUNTIME_STATS':'OFF'},True),({'SCHEDULER_TRACE':'OFF'},False),({'STAY_NATIVE':'OFF'},False),({'ASYNC_AUDIO':'OFF'},False)]
for changes,valid in cases:
    r=subprocess.run(['cmake',*[f'-DDBT_PROBE_{k}={v}' for k,v in (base|changes).items()],'-P',str(guard)],capture_output=True,text=True);assert (r.returncode==0)==valid,r.stdout+r.stderr
tree=ast.parse((ROOT/'tools/validate_local_image.py').read_text())
def assignment(name):return next(i for i,n in enumerate(tree.body) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id==name for t in n.targets))
actual=compile(ast.Module(body=tree.body[assignment('task_runtime_stats'):assignment('volume_ramp')],type_ignores=[]),'<actual runtime admission>','exec')
cmd='cc -DDBT_PROBE_TASK_RUNTIME_STATS=1';units=['probe_main.c','probe_scheduler.c','probe_audio.c'];count=0

def check(changes,valid):
    global count
    state={'build_status':{'task_runtime_stats':True,'async_audio':True},'cache':'DBT_PROBE_TASK_RUNTIME_STATS:BOOL=ON','commands':[{'file':n,'command':cmd} for n in units],'entry_name':'probe_main.c','stay_native':True,'scheduler_trace':True,'diagnostic':False,'pipeline':False,'Path':pathlib.Path,'re':re};state.update(changes)
    try:exec(actual,state)
    except (AssertionError,StopIteration):assert not valid
    else:assert valid
    count+=1
check({},True);check({'build_status':{},'cache':'','commands':[{'file':n,'command':'cc'} for n in units]},True)
check({'build_status':{'task_runtime_stats':True,'task_runtime_only':True,'async_audio':True},'cache':'DBT_PROBE_TASK_RUNTIME_STATS:BOOL=ON\nDBT_PROBE_TASK_RUNTIME_ONLY:BOOL=ON','scheduler_trace':False,'commands':[{'file':n,'command':cmd+' -DDBT_PROBE_TASK_RUNTIME_ONLY=1'} for n in units]},True)
for change in [{'build_status':{'task_runtime_stats':1,'async_audio':True}},{'build_status':{'task_runtime_stats':True,'async_audio':False}},{'cache':''},{'commands':[]},{'stay_native':False},{'scheduler_trace':False},{'diagnostic':True},{'pipeline':True}]:check(change,False)
for bad in ['cc','cc -DDBT_PROBE_TASK_RUNTIME_STATS=0',cmd+' -UDBT_PROBE_TASK_RUNTIME_STATS',cmd+' -DDBT_PROBE_TASK_RUNTIME_STATS=1']:check({'commands':[{'file':n,'command':bad} for n in units]},False)
start=next(i for i,n in enumerate(tree.body) if isinstance(n,ast.For) and isinstance(n.iter,ast.Tuple) and any(isinstance(v,ast.Constant) and v.value=='CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS' for v in n.iter.elts))
config_code=compile(ast.Module(body=tree.body[start:start+2],type_ignores=[]),'<actual SDK accounting admission>','exec')
keys=['CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS','CONFIG_FREERTOS_USE_TRACE_FACILITY','CONFIG_FREERTOS_USE_STATS_FORMATTING_FUNCTIONS','CONFIG_FREERTOS_VTASKLIST_INCLUDE_COREID','CONFIG_FREERTOS_RUN_TIME_STATS_USING_ESP_TIMER','CONFIG_FREERTOS_RUN_TIME_COUNTER_TYPE_U32']
on='\n'.join('#define '+k+' 1' for k in keys);config_cases=0

def cfg(config,enabled,valid):
    global config_cases
    try:exec(config_code,{'config':config,'task_runtime_stats':enabled,'re':re})
    except AssertionError:assert not valid
    else:assert valid
    config_cases+=1
cfg(on,True,True);cfg(on.replace('\n','\r\n'),True,True);cfg('',False,True);cfg(on,False,False)
for k in keys:cfg(on.replace('#define '+k+' 1',''),True,False)
for k in ['CONFIG_FREERTOS_RUN_TIME_STATS_USING_CPU_CLK','CONFIG_FREERTOS_RUN_TIME_COUNTER_TYPE_U64']:cfg(on+'\n#define '+k+' 1',True,False)
cfg(on+'\n#define '+keys[0]+' 0',True,False)
cli=sdk_cases=0
if sys.platform=='win32':
    real=ROOT/'esp32_probe/sdkconfig';before=hashlib.sha256(real.read_bytes()).hexdigest()
    r=subprocess.run(['powershell','-NoProfile','-ExecutionPolicy','Bypass','-File',str(ROOT/'tools/build_probe.ps1'),'-TaskRuntimeStats'],capture_output=True,text=True);assert r.returncode and 'Task runtime requires' in r.stderr;cli=1
    script=(ROOT/'tools/build_probe.ps1').read_text();fragment=script.split('# The private probe SDK config follows')[1].split("$env:GIT_NO_LAZY_FETCH")[0]
    sandbox=OUT/'fake_probe';sandbox.mkdir(exist_ok=True);sdk=sandbox/'sdkconfig';sdk.write_text('CONFIG_CUSTOM_OBSERVER_TEST=42\nCONFIG_FREERTOS_USE_TRACE_FACILITY=y\n')
    runner=OUT/'sdk-options.ps1';runner.write_text('param([string]$probeRoot,[switch]$TaskRuntimeStats)\n# The private probe SDK config follows'+fragment)
    for enabled in [True,False,True,False]:
        args=['powershell','-NoProfile','-ExecutionPolicy','Bypass','-File',str(runner),'-probeRoot',str(sandbox)]+(['-TaskRuntimeStats'] if enabled else [])
        subprocess.run(args,check=True,capture_output=True,text=True);text=sdk.read_text();assert text.count('CONFIG_CUSTOM_OBSERVER_TEST=42')==1
        for key in keys[:4]:assert re.findall(r'^'+key+r'=(\S+)$',text,re.M)==(['y'] if enabled else [])
        if enabled:
            for key in keys[4:]:assert re.findall(r'^'+key+r'=(\S+)$',text,re.M)==['y']
        sdk_cases+=1
    assert hashlib.sha256(real.read_bytes()).hexdigest()==before
(OUT/'results.json').write_text(json.dumps({'passed':True,'actual_cmake_cases':len(cases),'actual_image_cases':count,'actual_sdk_admission_cases':config_cases,'actual_private_sdk_transitions':sdk_cases,'cli_rejections':cli,'hardware_accessed':False},indent=2)+'\n');print(f'PASS runtime guard: {len(cases)} CMake, {count} image, {config_cases} SDK, {sdk_cases} private SDK transitions, {cli} CLI')
