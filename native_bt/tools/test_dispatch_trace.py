"""Actual frozen-dispatch module/formatter, ASan/UBSan and admission. No hardware."""
import ast,hashlib,json,os,pathlib,re,shutil,subprocess,time
ROOT=pathlib.Path(__file__).resolve().parents[1];out=ROOT/'_build/dispatch_trace'/str(time.time_ns());out.mkdir(parents=True,exist_ok=False)
source=ROOT/'esp32_probe/main/probe_dispatch_trace.c';header=source.with_suffix('.h');audio=ROOT/'esp32_probe/main/probe_audio.c';fixture=ROOT/'tests/dispatch_trace.c'
tracked=[source,header,audio,fixture,pathlib.Path(__file__).resolve(),ROOT/'esp32_probe/main/probe_dispatch_kernel_trace.h',ROOT/'tools/validate_local_image.py',ROOT/'esp32_probe/CMakeLists.txt']
hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked}
text=audio.read_text();a=text.index('static void log_dispatch_trace(void){');b=text.index('#endif\nstatic void log_loop',a);(out/'formatter.inc').write_text(text[a:b])
env=os.environ.copy();vc=pathlib.Path(r'C:/Program Files (x86)/Microsoft Visual Studio/2022/BuildTools/VC/Auxiliary/Build/vcvars64.bat')
if os.name=='nt':
    vals=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True);env.update(dict(x.split('=',1) for x in vals.splitlines() if '=' in x and not x.startswith('=')))
cc=shutil.which('clang',path=env.get('PATH')) or r'C:/Program Files/LLVM/bin/clang.exe';cache=shutil.which('ccache',path=env.get('PATH'))
if not cache:cache=str(sorted((pathlib.Path.home()/'.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'))[-1])
env.pop('CCACHE_DISABLE',None);resource=pathlib.Path(subprocess.check_output([cc,'-print-resource-dir'],text=True).strip())
if os.name=='nt':env['PATH']=str(resource/'lib/windows')+os.pathsep+env.get('PATH','')
report={'passed':False,'scope':'actual trace/formatter with simulated core/clock/mask; not ESP32 timing','hardware_accessed':False,'sources':hashes,'commands':[],'runs':[]};receipt=out/'results.json';receipt.write_text(json.dumps(report))
original=source.read_text()
for mode in ('candidate','no-freeze','unchecked-read'):
    variant=source
    if mode!='candidate':
        changed=original
        if mode=='no-freeze':
            needle='    atomic_store_explicit(&trace.enabled,0,memory_order_release);\n    atomic_store_explicit(&trace.gap_us,at,memory_order_relaxed);';assert changed.count(needle)==1
            changed=changed.replace(needle,'    atomic_store_explicit(&trace.gap_us,at,memory_order_relaxed);')
        else:
            needle='out->coherent=v==atomic_load_explicit(&r->version,memory_order_seq_cst) &&';assert changed.count(needle)==1
            changed=changed.replace(needle,'out->coherent=true &&')
        variant=out/(mode+'.c');variant.write_text(changed)
    obj=out/(mode+'.obj');exe=out/(mode+('.exe' if os.name=='nt' else ''))
    base=[cc,'-std=c11','-O1','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fno-sanitize-recover=all','-DDBT_PROBE_DISPATCH_TRACE=1','-DDBT_DISPATCH_TRACE_TEST=1','-DDISPATCH_TRACE_IMPL="'+str(variant).replace('\\','/')+'"','-I'+str(source.parent),'-I'+str(out)]
    if os.name=='nt':base+=['-fms-runtime-lib=dll','-D_DLL']
    cmd=[cache]+base+['-c',str(fixture),'-o',str(obj)];subprocess.run(cmd,env=env,check=True);report['commands'].append(cmd)
    cmd=base+[str(obj)]+(['-Wl,/NODEFAULTLIB:libcmt'] if os.name=='nt' else [])+['-o',str(exe)];subprocess.run(cmd,env=env,check=True);report['commands'].append(cmd)
    run=subprocess.run([str(exe)],env=env,capture_output=True,text=True,timeout=20);report['runs'].append({'mode':mode,'exit':run.returncode,'stdout':run.stdout,'stderr':run.stderr});receipt.write_text(json.dumps(report,indent=2));print(mode,run.returncode,run.stderr or run.stdout.splitlines()[-1],flush=True)
    assert (run.returncode==0)==(mode=='candidate')
    if mode=='candidate':
        rows=[json.loads(x) for x in run.stdout.splitlines() if x.startswith('{')];windows=[r for r in rows if r['probe']=='dispatch_window'];events=[r for r in rows if r['probe']=='dispatch_event'];assert len(windows)==2 and len(events)==32
        assert all(r['count']==16 and r['prefix_overwritten'] and r['task_switch_is_selection'] for r in windows)
        assert all(type(r[k]) is int for r in events for k in ('core','ordinal','event_us','kind','task_role','value'))
# Actual CMake admission, leaving the full product entry enabled.
guard=out/'guard.cmake';guard.write_text((ROOT/'esp32_probe/CMakeLists.txt').read_text().split('set(COMPONENTS main)')[0])
base={k:'ON' for k in ['DISPATCH_TRACE','STAY_NATIVE','KEEP_OWNED_ACTIVE','PRODUCT_IO','PRODUCT_SECURITY','AUDIO_TEST','AUDIO_OUTPUT','SBC_BUILD','CLASSIC_SECURITY_TEST','SMP','GATT_TEST','AVRCP','ASYNC_AUDIO','PCM_COPIED','DSP_ASSIST','LHDC_BUILD','CODEC_AWARE_RENDER','BLOCK_DSP','WORD_DSP','UNITY_PCM_COPY','PRIORITY_ORDER','RENDER_FAIR_SHARE']};base['DECODER_PRIORITY']='21'
cases=[({},True)]+[({k:'OFF'},False) for k in ['STAY_NATIVE','ASYNC_AUDIO','PCM_COPIED','DSP_ASSIST','LHDC_BUILD']]+[({k:'ON'},False) for k in ['SCHEDULER_TRACE','TASK_RUNTIME_STATS','TASK_CYCLE_ACCOUNTING']]
for changes,valid in cases:
    result=subprocess.run(['cmake',*[f'-DDBT_PROBE_{k}={v}' for k,v in (base|changes).items()],'-P',str(guard)],capture_output=True,text=True);assert (result.returncode==0)==valid,result.stdout+result.stderr
report['cmake_cases']=len(cases)
# Execute the exact new image-admission section.
tree=ast.parse((ROOT/'tools/validate_local_image.py').read_text());assign=lambda name:next(i for i,n in enumerate(tree.body) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id==name for t in n.targets))
code=compile(ast.Module(body=tree.body[assign('dispatch_trace'):assign('task_runtime_stats')],type_ignores=[]),'<actual dispatch admission>','exec')
units=['probe_main.c','probe_audio.c','probe_output.c','probe_dispatch_trace.c'];commands=[{'file':x,'command':'cc -DDBT_PROBE_DISPATCH_TRACE=1'} for x in units]+[{'file':'X:/esp-idf/components/freertos/FreeRTOS-Kernel/tasks.c','command':'cc -DDBT_PROBE_DISPATCH_TRACE=1 -include X:/native_bt/esp32_probe/main/probe_dispatch_kernel_trace.h'}]
image_cases=0
def check(changes,valid):
    global image_cases
    state={'build_status':{'dispatch_trace':True,'async_audio':True,'pcm_copied':True,'dsp_assist':True},'cache':'DBT_PROBE_DISPATCH_TRACE:BOOL=ON','commands':commands,'Path':pathlib.Path,'re':re,'entry_name':'probe_main.c','stay_native':True,'scheduler_trace':False,'diagnostic':False,'pipeline':False};state.update(changes)
    try:exec(code,state)
    except (AssertionError,StopIteration):assert not valid
    else:assert valid
    image_cases+=1
check({},True)
for changes in [{'stay_native':False},{'scheduler_trace':True},{'cache':''},{'commands':commands[:-1]},{'commands':commands[1:]},{'commands':commands+[commands[-1]]},{'build_status':{'dispatch_trace':1}},{'pipeline':True}]:check(changes,False)
for flag in ['cc','cc -DDBT_PROBE_DISPATCH_TRACE=0','cc -DDBT_PROBE_DISPATCH_TRACE=1 -UDBT_PROBE_DISPATCH_TRACE','cc -DDBT_PROBE_DISPATCH_TRACE=1 -DDBT_PROBE_DISPATCH_TRACE=1']:
    check({'commands':[{'file':x,'command':flag} for x in units]+commands[-1:]},False)
check({'commands':commands[:-1]+[dict(commands[-1],command='cc -DDBT_PROBE_DISPATCH_TRACE=1 -DFAKE=probe_dispatch_kernel_trace.h')]},False)
check({'build_status':{},'cache':'','commands':[{'file':'probe_main.c','command':'cc'}]},True)
report['image_cases']=image_cases
assert all(hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()==h for p,h in hashes.items())
report['passed']=True;receipt.write_text(json.dumps(report,indent=2));print('PASS actual trace/formatter, negative controls,',len(cases),'CMake and',image_cases,'image cases; artifacts',out)
