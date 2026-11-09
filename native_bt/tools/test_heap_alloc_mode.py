"""Actual allocation-sampler CMake/image/SDK/CLI guards; no hardware."""
import ast,hashlib,json,pathlib,re,subprocess,sys
ROOT=pathlib.Path(__file__).resolve().parents[1];OUT=ROOT/'_build/heap_alloc_guard';OUT.mkdir(exist_ok=True)
guard=OUT/'guard.cmake';guard.write_text((ROOT/'esp32_probe/CMakeLists.txt').read_text().split('set(COMPONENTS main)')[0])
base={k:'ON' for k in ['HEAP_ALLOC_TRACE','CONSOLE_IRQ','STAY_NATIVE','ASYNC_AUDIO','KEEP_OWNED_ACTIVE','PRODUCT_IO','PRODUCT_SECURITY','AUDIO_TEST','AUDIO_OUTPUT','SBC_BUILD','CLASSIC_SECURITY_TEST','SMP','GATT_TEST','AVRCP']};base['DECODER_PRIORITY']='21'
cases=[({},True),({'HEAP_ALLOC_TRACE':'OFF','CONSOLE_IRQ':'OFF'},True)]
for k in ['STAY_NATIVE','ASYNC_AUDIO','AUDIO_OUTPUT','GATT_TEST','SMP','CONSOLE_IRQ']:cases.append(({k:'OFF'},False))
for k in ['SCHEDULER_TRACE','TASK_RUNTIME_STATS','TASK_CYCLE_ACCOUNTING','DISPATCH_TRACE']:cases.append(({k:'ON'},False))
for change,valid in cases:
    r=subprocess.run(['cmake',*[f'-DDBT_PROBE_{k}={v}' for k,v in (base|change).items()],'-P',str(guard)],capture_output=True,text=True)
    assert (r.returncode==0)==valid,r.stdout+r.stderr
source=(ROOT/'tools/validate_local_image.py').read_text();tree=ast.parse(source)
def pos(name):return next(i for i,n in enumerate(tree.body) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id==name for t in n.targets))
actual=compile(ast.Module(body=tree.body[pos('heap_alloc_trace'):pos('console_irq')],type_ignores=[]),'<actual allocation admission>','exec')
units=['probe_main.c','probe_audio.c','probe_gatt.c','probe_smp.c','probe_heap_alloc_trace.c'];macro='cc -Os -DDBT_PROBE_HEAP_ALLOC_TRACE=1';count=0
base_status={'heap_alloc_trace':True,'async_audio':True,'console_irq':True}
def check(change,valid):
    global count
    state={'build_status':base_status.copy(),'cache':'DBT_PROBE_HEAP_ALLOC_TRACE:BOOL=ON','stay_native':True,'scheduler_trace':False,'diagnostic':False,'pipeline':False,'entry_name':'probe_main.c','commands':[{'file':n,'command':macro} for n in units],'Path':pathlib.Path,'re':re};state.update(change)
    try:exec(actual,state)
    except AssertionError:assert not valid
    else:assert valid
    count+=1
check({},True);check({'build_status':{},'cache':'','commands':[{'file':'probe_main.c','command':'cc'}]},True)
for change in [{'build_status':base_status|{'heap_alloc_trace':1}},{'cache':''},{'commands':[]},{'commands':[{'file':n,'command':macro} for n in units[:-1]]},{'stay_native':False},{'diagnostic':True},{'scheduler_trace':True}]:check(change,False)
for flag in ['async_audio','console_irq']:check({'build_status':base_status|{flag:False}},False)
for flag in ['task_runtime_stats','task_cycle_accounting','dispatch_trace']:check({'build_status':base_status|{flag:True}},False)
check({'commands':[{'file':n,'command':macro.replace('-Os ','') if n=='probe_heap_alloc_trace.c' else macro} for n in units]},False)
for bad in ['cc',macro.replace('=1','=0'),macro+' -UDBT_PROBE_HEAP_ALLOC_TRACE',macro+' -DDBT_PROBE_HEAP_ALLOC_TRACE=1']:check({'commands':[{'file':n,'command':bad} for n in units]},False)
ci=pos('config');assert isinstance(tree.body[ci+1],ast.Assert) and isinstance(tree.body[ci+2],ast.If) and tree.body[ci+2].test.id=='heap_alloc_trace'
sdk=compile(ast.Module(body=tree.body[ci+1:ci+3],type_ignores=[]),'<actual heap SDK admission>','exec');sdk_cases=0
for config,on,valid in [('#define CONFIG_HEAP_USE_HOOKS 1\n',True,True),('',False,True),('',True,False),('#define CONFIG_HEAP_USE_HOOKS 1\n',False,False),('#define CONFIG_HEAP_USE_HOOKS 0\n',True,False),('#define CONFIG_HEAP_USE_HOOKS 1\n'*2,True,False),('#define CONFIG_HEAP_USE_HOOKS 1\n#define CONFIG_HEAP_TRACING_STANDALONE 1\n',True,False),('#define CONFIG_HEAP_USE_HOOKS 1\n#define CONFIG_HEAP_PLACE_FUNCTION_INTO_FLASH 1\n',True,False)]:
    try:exec(sdk,{'config':config,'heap_alloc_trace':on,'re':re})
    except AssertionError:assert not valid
    else:assert valid
    sdk_cases+=1
cli=0;transitions=0
if sys.platform=='win32':
    pwsh=pathlib.Path.home()/'.cache/codex-runtimes/codex-primary-runtime/dependencies/native/powershell/pwsh.exe'
    script=(ROOT/'tools/build_probe.ps1').read_text();begin=script.index("$heapSdkPath=Join-Path");end=script.index("$env:GIT_NO_LAZY_FETCH",begin);block=script[begin:end]
    fixture=OUT/'sdkconfig';fixture.write_text('# CONFIG_HEAP_USE_HOOKS is not set\nCONFIG_KEEP_SAMPLE=y\n')
    for enabled in (True,False,False):
        case=OUT/'sdk-transition.ps1';case.write_text("$probeRoot='"+OUT.as_posix()+"'\n$HeapAllocTrace="+('$true' if enabled else '$false')+'\n'+block)
        subprocess.run([str(pwsh),'-NoProfile','-File',str(case)],check=True,capture_output=True,text=True)
        text=fixture.read_text();assert text.count('CONFIG_HEAP_USE_HOOKS')==1 and 'CONFIG_KEEP_SAMPLE=y' in text
        assert ('CONFIG_HEAP_USE_HOOKS=y' in text)==enabled;transitions+=1
    real=ROOT/'esp32_probe/sdkconfig';before=hashlib.sha256(real.read_bytes()).hexdigest()
    for args in [['-HeapAllocTrace'],['-HeapAllocTrace','-StayNative','-AsyncAudio','-ConsoleIrq','-TaskRuntimeOnly']]:
        r=subprocess.run([str(pwsh),'-NoProfile','-File',str(ROOT/'tools/build_probe.ps1'),*args],capture_output=True,text=True)
        assert r.returncode and 'Allocation sampling requires' in r.stderr
        assert hashlib.sha256(real.read_bytes()).hexdigest()==before;cli+=1
(OUT/'results.json').write_text(json.dumps({'passed':True,'cmake_cases':len(cases),'image_cases':count,'sdk_cases':sdk_cases,'sdk_transitions':transitions,'cli_rejections':cli,'hardware_accessed':False},indent=2)+'\n')
print(f'PASS allocation mode: {len(cases)} CMake, {count} image, {sdk_cases} SDK, {transitions} transitions, {cli} CLI')
