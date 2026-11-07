"""Actual IRQ-console CMake/image/SDK admission. No hardware."""
import ast,json,pathlib,re,subprocess,sys
ROOT=pathlib.Path(__file__).resolve().parents[1];OUT=ROOT/'_build/console_irq_guard';OUT.mkdir(exist_ok=True)
guard=OUT/'guard.cmake';guard.write_text((ROOT/'esp32_probe/CMakeLists.txt').read_text().split('set(COMPONENTS main)')[0])
base={k:'ON' for k in ['CONSOLE_IRQ','STAY_NATIVE','ASYNC_AUDIO','KEEP_OWNED_ACTIVE','PRODUCT_IO','PRODUCT_SECURITY','AUDIO_TEST','AUDIO_OUTPUT','SBC_BUILD','CLASSIC_SECURITY_TEST','SMP','GATT_TEST','AVRCP']};base['DECODER_PRIORITY']='21'
cases=[({},True),({'CONSOLE_IRQ':'OFF'},True),({'STAY_NATIVE':'OFF'},False),({'ASYNC_AUDIO':'OFF'},False)]
for changes,valid in cases:
    r=subprocess.run(['cmake',*[f'-DDBT_PROBE_{k}={v}' for k,v in (base|changes).items()],'-P',str(guard)],capture_output=True,text=True);assert (r.returncode==0)==valid,r.stdout+r.stderr
tree=ast.parse((ROOT/'tools/validate_local_image.py').read_text())
def assignment(name):return next(i for i,n in enumerate(tree.body) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id==name for t in n.targets))
actual=compile(ast.Module(body=tree.body[assignment('console_irq'):assignment('task_runtime_stats')],type_ignores=[]),'<actual IRQ console admission>','exec')
cmd='cc -DDBT_PROBE_CONSOLE_IRQ=1';units=['probe_main.c','probe_console.c'];count=0

def check(changes,valid):
    global count
    state={'build_status':{'console_irq':True,'async_audio':True},'cache':'DBT_PROBE_CONSOLE_IRQ:BOOL=ON','commands':[{'file':n,'command':cmd} for n in units],'entry_name':'probe_main.c','stay_native':True,'diagnostic':False,'pipeline':False,'Path':pathlib.Path,'re':re};state.update(changes)
    try:exec(actual,state)
    except (AssertionError,StopIteration):assert not valid
    else:assert valid
    count+=1
check({},True);check({'build_status':{},'cache':'','commands':[{'file':'probe_main.c','command':'cc'}]},True)
for c in [{'build_status':{'console_irq':1,'async_audio':True}},{'build_status':{'console_irq':True,'async_audio':False}},{'cache':''},{'commands':[]},{'stay_native':False},{'diagnostic':True},{'pipeline':True},{'commands':[{'file':'probe_main.c','command':cmd}]}]:check(c,False)
for bad in ['cc','cc -DDBT_PROBE_CONSOLE_IRQ=0',cmd+' -UDBT_PROBE_CONSOLE_IRQ',cmd+' -DDBT_PROBE_CONSOLE_IRQ=1']:check({'commands':[{'file':n,'command':bad} for n in units]},False)
node=next(n for i,n in enumerate(tree.body) if i>assignment('config') and isinstance(n,ast.If) and isinstance(n.test,ast.Name) and n.test.id=='console_irq')
code=compile(ast.Module(body=[node],type_ignores=[]),'<actual console SDK admission>','exec');cfg='#define CONFIG_ESP_CONSOLE_UART 1\r\n#define CONFIG_ESP_CONSOLE_UART_NUM 0\r\n#define CONFIG_ESP_CONSOLE_UART_BAUDRATE 115200\r\n';sdk_count=0
for config,enabled,valid in [(cfg,True,True),('',False,True),(cfg.replace('NUM 0','NUM 1'),True,False),(cfg.replace('115200','921600'),True,False),('',True,False)]:
    try:exec(code,{'console_irq':enabled,'config':config,'re':re})
    except AssertionError:assert not valid
    else:assert valid
    sdk_count+=1
cli=0
if sys.platform=='win32':
    r=subprocess.run(['powershell','-NoProfile','-ExecutionPolicy','Bypass','-File',str(ROOT/'tools/build_probe.ps1'),'-ConsoleIrq'],capture_output=True,text=True);assert r.returncode and 'Console IRQ requires' in r.stderr;cli=1
(OUT/'results.json').write_text(json.dumps({'passed':True,'actual_cmake_cases':len(cases),'actual_image_cases':count,'actual_sdk_cases':sdk_count,'cli_rejections':cli,'hardware_accessed':False},indent=2)+'\n');print(f'PASS console admission: {len(cases)} CMake,{count} image,{sdk_count} SDK,{cli} CLI')
