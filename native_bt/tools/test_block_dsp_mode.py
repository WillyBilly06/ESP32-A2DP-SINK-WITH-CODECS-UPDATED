"""Actual block-renderer CMake, image-mode and finite-deployment guards."""
import ast,json,pathlib,re,subprocess,sys
ROOT=pathlib.Path(__file__).resolve().parents[1];OUT=ROOT/'_build/block_dsp_mode';OUT.mkdir(exist_ok=True)
guard=OUT/'guard.cmake';guard.write_text((ROOT/'esp32_probe/CMakeLists.txt').read_text().split('set(COMPONENTS main)')[0])
base={'BLOCK_DSP':'ON','PCM_LOANS':'ON','ASYNC_AUDIO':'ON','AUDIO_OUTPUT':'ON','AUDIO_TEST':'ON','SBC_BUILD':'ON','CLASSIC_SECURITY_TEST':'ON','KEEP_OWNED_ACTIVE':'OFF'}
cases=[({},True),({'PCM_LOANS':'OFF'},False),({'KEEP_OWNED_ACTIVE':'ON'},False),({'RF_PROTOCOL_DIAGNOSTIC':'ON'},False),({'ASYNC_AUDIO':'OFF'},False)]
for change,valid in cases:
    run=subprocess.run(['cmake',*[f'-DDBT_PROBE_{k}={v}' for k,v in (base|change).items()],'-P',str(guard)],capture_output=True,text=True)
    assert (run.returncode==0)==valid,run.stdout+run.stderr
tree=ast.parse((ROOT/'tools/validate_local_image.py').read_text());start=next(i for i,n in enumerate(tree.body) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='block_dsp' for t in n.targets));nodes=tree.body[start:start+6];assert isinstance(nodes[-1],ast.If)
actual=compile(ast.Module(body=nodes,type_ignores=[]),'<actual block mode>','exec');count=0
flags=' -DDBT_AUDIO_BLOCK_DSP=1 -DFIXED_DSP_BLOCK_API=1'
def check(change,valid):
    global count
    state={'build_status':{'block_dsp':True,'async_audio':True},'cache':'DBT_PROBE_BLOCK_DSP:BOOL=ON\nDBT_PROBE_KEEP_OWNED_ACTIVE:BOOL=OFF','main_command':'cc -DDBT_PROBE_BLOCK_DSP=1','pcm_loans':True,'pcm_copied':False,'rf_diagnostic':False,'commands':[{'file':p,'command':'cc'+flags} for p in ('dbt_audio.c','fixed_dsp.c')],'Path':pathlib.Path,'re':re}
    state.update(change)
    try:exec(actual,state)
    except (AssertionError,StopIteration):assert not valid
    else:assert valid
    count+=1
check({},True);check({'pcm_loans':False},False);check({'rf_diagnostic':True},False);check({'main_command':'cc'},False);check({'cache':'DBT_PROBE_BLOCK_DSP:BOOL=OFF'},False)
check({'commands':[{'file':'dbt_audio.c','command':'cc'+flags}]},False)
check({'commands':[{'file':p,'command':'cc'+flags+' -UDBT_AUDIO_BLOCK_DSP'} for p in ('dbt_audio.c','fixed_dsp.c')]},False)
check({'commands':[{'file':p,'command':'cc'+flags+' -DDBT_AUDIO_BLOCK_DSP=0'} for p in ('dbt_audio.c','fixed_dsp.c')]},False)
check({'build_status':{'block_dsp':False},'cache':'','main_command':'cc','commands':[]},True)
check({'build_status':{'block_dsp':False},'cache':'','main_command':'cc'},False)
deployment_tree=ast.parse((ROOT/'tools/deploy_audio_trial.py').read_text());function=next(n for n in deployment_tree.body if isinstance(n,ast.FunctionDef) and n.name=='admit_pcm_target_deployment');space={'re':re};exec(compile(ast.Module(body=[function],type_ignores=[]),'<actual finite guard>','exec'),space)
for finite,valid in [(False,False),(True,True)]:
    try:space['admit_pcm_target_deployment']('DBT_PROBE_PCM_TARGET_MS:STRING=64\nDBT_PROBE_BLOCK_DSP:BOOL=ON','probe',finite,False,False)
    except RuntimeError:assert not valid
    else:assert valid
cli=0
if sys.platform=='win32':
    run=subprocess.run(['powershell','-NoProfile','-ExecutionPolicy','Bypass','-File',str(ROOT/'tools/build_probe.ps1'),'-BlockDsp'],capture_output=True,text=True);assert run.returncode and 'Block DSP requires' in run.stderr;cli=1
(OUT/'results.json').write_text(json.dumps({'passed':True,'cmake_cases':len(cases),'actual_image_cases':count,'actual_deployment_cases':2,'cli_rejections':cli,'hardware_accessed':False},indent=2)+'\n');print(f'PASS block DSP mode: {len(cases)} CMake, {count} actual image, 2 actual deployment cases, {cli} early CLI rejection')
