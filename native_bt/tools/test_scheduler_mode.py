"""Exercise actual CMake/image/finite-deployment sampler admission guards."""
import ast,json,pathlib,re,subprocess,sys
ROOT=pathlib.Path(__file__).resolve().parents[1];OUT=ROOT/'_build/scheduler_mode';OUT.mkdir(exist_ok=True)
guard=OUT/'guard.cmake';guard.write_text((ROOT/'esp32_probe/CMakeLists.txt').read_text().split('set(COMPONENTS main)')[0])
base={'SCHEDULER_TRACE':'ON','AUDIO_TEST':'ON','SBC_BUILD':'ON','CLASSIC_SECURITY_TEST':'ON','KEEP_OWNED_ACTIVE':'OFF'}
cases=[({},True),({'AUDIO_TEST':'OFF'},False),({'KEEP_OWNED_ACTIVE':'ON'},False),({'CODEC_DIAGNOSTIC_ONLY':'ON'},False),({'PIPELINE_DIAGNOSTIC':'ON'},False),({'SCHEDULER_TRACE':'OFF','KEEP_OWNED_ACTIVE':'ON'},True)]
for change,valid in cases:
    run=subprocess.run(['cmake',*[f'-DDBT_PROBE_{k}={v}' for k,v in (base|change).items()],'-P',str(guard)],capture_output=True,text=True);assert (run.returncode==0)==valid,run.stdout+run.stderr
tree=ast.parse((ROOT/'tools/validate_local_image.py').read_text());start=next(i for i,n in enumerate(tree.body) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='scheduler_trace' for t in n.targets));nodes=tree.body[start:start+6];assert isinstance(nodes[-1],ast.If)
code=compile(ast.Module(body=nodes,type_ignores=[]),'<actual sampler admission>','exec');count=0
commands=[{'file':p,'command':'cc -DDBT_PROBE_SCHEDULER_TRACE=1'} for p in ['probe_audio.c','probe_scheduler.c']]
def check(change,valid):
    global count
    state={'build_status':{'scheduler_trace':True},'cache':'DBT_PROBE_SCHEDULER_TRACE:BOOL=ON\nDBT_PROBE_KEEP_OWNED_ACTIVE:BOOL=OFF','main_command':'cc -DDBT_PROBE_SCHEDULER_TRACE=1','commands':commands,'diagnostic':False,'pipeline':False,'Path':pathlib.Path,'re':re};state.update(change)
    try:exec(code,state)
    except (AssertionError,StopIteration):assert not valid
    else:assert valid
    count+=1
check({},True);check({'main_command':'cc'},False);check({'cache':''},False);check({'commands':[]},False);check({'diagnostic':True},False);check({'pipeline':True},False)
check({'cache':'DBT_PROBE_SCHEDULER_TRACE:BOOL=ON\nDBT_PROBE_KEEP_OWNED_ACTIVE:BOOL=ON'},False)
check({'commands':[{'file':e['file'],'command':e['command']+' -UDBT_PROBE_SCHEDULER_TRACE'} for e in commands]},False)
check({'build_status':{},'cache':'','main_command':'cc','commands':[]},True)
check({'build_status':{},'cache':'','main_command':'cc'},False)
tree=ast.parse((ROOT/'tools/deploy_audio_trial.py').read_text());fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='admit_pcm_target_deployment');space={'re':re};exec(compile(ast.Module(body=[fn],type_ignores=[]),'<actual finite guard>','exec'),space)
for finite,keep,valid in [(False,False,False),(True,False,True),(True,True,False)]:
    try:space['admit_pcm_target_deployment']('DBT_PROBE_PCM_TARGET_MS:STRING=64\nDBT_PROBE_SCHEDULER_TRACE:BOOL=ON','probe',finite,False,keep)
    except RuntimeError:assert not valid
    else:assert valid
cli=0
if sys.platform=='win32':
    run=subprocess.run(['powershell','-NoProfile','-ExecutionPolicy','Bypass','-File',str(ROOT/'tools/build_probe.ps1'),'-SchedulerTrace'],capture_output=True,text=True);assert run.returncode and 'Scheduler trace requires' in run.stderr;cli=1
(OUT/'results.json').write_text(json.dumps({'passed':True,'cmake_cases':len(cases),'actual_image_cases':count,'actual_finite_cases':3,'cli_rejections':cli,'hardware_accessed':False},indent=2)+'\n');print(f'PASS sampler mode: {len(cases)} CMake, {count} image, 3 finite and {cli} CLI cases')
