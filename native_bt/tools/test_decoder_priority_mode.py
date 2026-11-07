"""Actual decoder-priority CMake/image/finite-deployment guard checks."""
import ast,json,pathlib,re,subprocess,sys
ROOT=pathlib.Path(__file__).resolve().parents[1];OUT=ROOT/'_build/decoder_priority_mode';OUT.mkdir(exist_ok=True)
guard=OUT/'guard.cmake';guard.write_text((ROOT/'esp32_probe/CMakeLists.txt').read_text().split('set(COMPONENTS main)')[0])
base={'DECODER_PRIORITY':'6','ASYNC_AUDIO':'ON','AUDIO_OUTPUT':'ON','AUDIO_TEST':'ON','SBC_BUILD':'ON','CLASSIC_SECURITY_TEST':'ON','KEEP_OWNED_ACTIVE':'OFF'}
cases=[({},True),({'DECODER_PRIORITY':'7'},False),({'DECODER_PRIORITY':'06'},False),({'ASYNC_AUDIO':'OFF'},False),({'KEEP_OWNED_ACTIVE':'ON'},False),({'AUDIO_TEST':'OFF'},False),({'RF_PROTOCOL_DIAGNOSTIC':'ON'},False),({'CODEC_DIAGNOSTIC_ONLY':'ON'},False),({'DECODER_PRIORITY':'5','KEEP_OWNED_ACTIVE':'ON'},True),({'DECODER_PRIORITY':'5','ASYNC_AUDIO':'OFF'},True)]
for change,valid in cases:
    run=subprocess.run(['cmake',*[f'-DDBT_PROBE_{k}={v}' for k,v in (base|change).items()],'-P',str(guard)],capture_output=True,text=True);assert (run.returncode==0)==valid,run.stdout+run.stderr
tree=ast.parse((ROOT/'tools/validate_local_image.py').read_text());start=next(i for i,n in enumerate(tree.body) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='decoder_priority' for t in n.targets));nodes=tree.body[start:start+5];assert isinstance(nodes[-1],ast.If);actual=compile(ast.Module(body=nodes,type_ignores=[]),'<actual priority admission>','exec');count=0
def check(change,valid):
    global count
    state={'build_status':{'decoder_priority':6,'async_audio':True},'cache':'DBT_PROBE_DECODER_PRIORITY:STRING=6\nDBT_PROBE_KEEP_OWNED_ACTIVE:BOOL=OFF','commands':[{'file':'probe_audio.c','command':'cc -DDBT_PROBE_DECODER_PRIORITY=6'}],'diagnostic':False,'pipeline':False,'stay_native':False,'Path':pathlib.Path,'re':re};state.update(change)
    try:exec(actual,state)
    except (AssertionError,StopIteration):assert not valid
    else:assert valid
    count+=1
check({},True);check({'build_status':{'decoder_priority':7}},False);check({'build_status':{'decoder_priority':True}},False);check({'cache':'DBT_PROBE_DECODER_PRIORITY:STRING=5'},False);check({'cache':''},False);check({'commands':[]},False)
for command in ['cc','cc -DDBT_PROBE_DECODER_PRIORITY=5','cc -DDBT_PROBE_DECODER_PRIORITY=6 -UDBT_PROBE_DECODER_PRIORITY','cc -DDBT_PROBE_DECODER_PRIORITY=6 -DDBT_PROBE_DECODER_PRIORITY=5']:check({'commands':[{'file':'probe_audio.c','command':command}]},False)
check({'diagnostic':True},False);check({'pipeline':True},False);check({'build_status':{'decoder_priority':6,'async_audio':False}},False);check({'build_status':{'decoder_priority':6,'async_audio':True,'rf_protocol_diagnostic':True}},False);check({'cache':'DBT_PROBE_DECODER_PRIORITY:STRING=6\nDBT_PROBE_KEEP_OWNED_ACTIVE:BOOL=ON'},False)
check({'build_status':{},'cache':'','commands':[{'file':'probe_audio.c','command':'cc'}]},True);check({'build_status':{},'cache':''},False)
check({'build_status':{'decoder_priority':5},'cache':'DBT_PROBE_DECODER_PRIORITY:STRING=5','commands':[{'file':'probe_audio.c','command':'cc -DDBT_PROBE_DECODER_PRIORITY=5'}]},True)
yield_start=next(i for i,n in enumerate(tree.body) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='owner_ready_yield' for t in n.targets));yield_nodes=tree.body[yield_start:yield_start+3];assert isinstance(yield_nodes[-1],ast.If);yield_code=compile(ast.Module(body=yield_nodes,type_ignores=[]),'<actual owner yield admission>','exec');yield_cases=0
def check_yield(change,valid):
    global yield_cases
    state={'decoder_priority':6,'host_priority':6,'build_status':{'owner_ready_yield':True},'main_command':'cc -DDBT_PROBE_DECODER_PRIORITY=6','render_fair_share':False,'re':re};state.update(change)
    try:exec(yield_code,state)
    except AssertionError:assert not valid
    else:assert valid
    yield_cases+=1
check_yield({},True);check_yield({'build_status':{}},False);check_yield({'build_status':{'owner_ready_yield':1}},False);check_yield({'main_command':'cc -DDBT_PROBE_DECODER_PRIORITY=5'},False);check_yield({'main_command':'cc -DDBT_PROBE_DECODER_PRIORITY=6 -UDBT_PROBE_DECODER_PRIORITY'},False);check_yield({'decoder_priority':5},False);check_yield({'decoder_priority':5,'build_status':{}},True)
check_yield({'host_priority':7,'build_status':{'owner_ready_yield':False}},True)
check_yield({'host_priority':7},False)
tree=ast.parse((ROOT/'tools/deploy_audio_trial.py').read_text());fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='admit_pcm_target_deployment');space={'re':re};exec(compile(ast.Module(body=[fn],type_ignores=[]),'<actual finite guard>','exec'),space)
for finite,keep,valid in [(False,False,False),(True,False,True),(True,True,False)]:
    try:space['admit_pcm_target_deployment']('DBT_PROBE_PCM_TARGET_MS:STRING=64\nDBT_PROBE_DECODER_PRIORITY:STRING=6','probe',finite,False,keep)
    except RuntimeError:assert not valid
    else:assert valid
cli=0
if sys.platform=='win32':
    run=subprocess.run(['powershell','-NoProfile','-ExecutionPolicy','Bypass','-File',str(ROOT/'tools/build_probe.ps1'),'-DecoderPriority','6'],capture_output=True,text=True);assert run.returncode and 'Decoder priority experiment requires' in run.stderr;cli=1
(OUT/'results.json').write_text(json.dumps({'passed':True,'cmake_cases':len(cases),'actual_image_cases':count,'actual_owner_yield_cases':yield_cases,'actual_finite_cases':3,'cli_rejections':cli,'hardware_accessed':False},indent=2)+'\n');print(f'PASS decoder priority: {len(cases)} CMake, {count} image, {yield_cases} owner yield, 3 finite and {cli} CLI cases')
