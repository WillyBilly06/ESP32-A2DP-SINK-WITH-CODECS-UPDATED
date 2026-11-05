"""Actual compact-copy and independent logger-affinity admission checks."""
import ast,json,pathlib,re,subprocess,sys
ROOT=pathlib.Path(__file__).resolve().parents[1];OUT=ROOT/'_build/compact_mode';OUT.mkdir(exist_ok=True)
guard=OUT/'guard.cmake';guard.write_text((ROOT/'esp32_probe/CMakeLists.txt').read_text().split('set(COMPONENTS main)')[0])
base={'PCM_COPIED':'ON','PCM_LOANS':'OFF','PCM_TARGET_MS':'16','BLOCK_DSP':'ON','ASYNC_AUDIO':'ON','AUDIO_OUTPUT':'ON','AUDIO_TEST':'ON','SBC_BUILD':'ON','CLASSIC_SECURITY_TEST':'ON','KEEP_OWNED_ACTIVE':'OFF','DECODER_CORE':'1','LOGGER_CORE':'1'}
cases=[({},True),({'PCM_LOANS':'ON'},False),({'PCM_COPIED':'OFF'},False),({'ASYNC_AUDIO':'OFF'},False),({'KEEP_OWNED_ACTIVE':'ON'},False),({'RF_PROTOCOL_DIAGNOSTIC':'ON'},False),({'LOGGER_CORE':'2'},False),({'PCM_COPIED':'OFF','PCM_LOANS':'ON'},True),({'PCM_COPIED':'OFF','BLOCK_DSP':'OFF','PCM_TARGET_MS':'64'},True)]
for change,valid in cases:
    run=subprocess.run(['cmake',*[f'-DDBT_PROBE_{k}={v}' for k,v in (base|change).items()],'-P',str(guard)],capture_output=True,text=True);assert (run.returncode==0)==valid,run.stdout+run.stderr
tree=ast.parse((ROOT/'tools/validate_local_image.py').read_text())
def section(name,count):
    start=next(i for i,n in enumerate(tree.body) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id==name for t in n.targets));nodes=tree.body[start:start+count];assert isinstance(nodes[-1],ast.If);return compile(ast.Module(body=nodes,type_ignores=[]),'<actual '+name+'>','exec')
copied=section('pcm_copied',5);logger=section('logger_core',4);copy_cases=0;logger_cases=0
def check_copy(change,valid):
    global copy_cases
    s={'build_status':{'pcm_copied':True,'async_audio':True},'cache':'DBT_PROBE_PCM_COPIED:BOOL=ON\nDBT_PROBE_KEEP_OWNED_ACTIVE:BOOL=OFF','main_command':'cc -DDBT_PROBE_PCM_COPIED=1','commands':[{'file':p,'command':'cc -DDBT_PROBE_PCM_COPIED=1'} for p in ['probe_audio.c','probe_output.c']],'pcm_loans':False,'rf_diagnostic':False,'pipeline':False,'Path':pathlib.Path,'re':re};s.update(change)
    try:exec(copied,s)
    except (AssertionError,StopIteration):assert not valid
    else:assert valid
    copy_cases+=1
check_copy({},True);check_copy({'pcm_loans':True},False);check_copy({'main_command':'cc'},False);check_copy({'commands':[]},False);check_copy({'rf_diagnostic':True},False);check_copy({'build_status':{'pcm_copied':True,'async_audio':False}},False)
check_copy({'cache':'DBT_PROBE_PCM_COPIED:BOOL=ON\nDBT_PROBE_KEEP_OWNED_ACTIVE:BOOL=ON'},False)
check_copy({'commands':[{'file':p,'command':'cc -DDBT_PROBE_PCM_COPIED=1 -UDBT_PROBE_PCM_COPIED'} for p in ['probe_audio.c','probe_output.c']]},False)
check_copy({'build_status':{'pcm_copied':False},'cache':'','main_command':'cc','commands':[]},True)
check_copy({'build_status':{'pcm_copied':False},'cache':'','main_command':'cc'},False)
def check_logger(change,valid):
    global logger_cases
    s={'decoder_core':1,'build_status':{'logger_core':1},'cache':'DBT_PROBE_LOGGER_CORE:STRING=1','commands':[{'file':'probe_audio.c','command':'cc -DDBT_PROBE_LOGGER_CORE=1'}],'Path':pathlib.Path,'re':re};s.update(change)
    try:exec(logger,s)
    except AssertionError:assert not valid
    else:assert valid
    logger_cases+=1
check_logger({},True);check_logger({'cache':'DBT_PROBE_LOGGER_CORE:STRING=0'},False);check_logger({'cache':''},False);check_logger({'build_status':{},'cache':'','commands':[]},True);check_logger({'commands':[{'file':'probe_audio.c','command':'cc -DDBT_PROBE_LOGGER_CORE=0'}]},False);check_logger({'commands':[{'file':'probe_audio.c','command':'cc -DDBT_PROBE_LOGGER_CORE=1 -UDBT_PROBE_LOGGER_CORE'}]},False)
deployment=ast.parse((ROOT/'tools/deploy_audio_trial.py').read_text());fn=next(n for n in deployment.body if isinstance(n,ast.FunctionDef) and n.name=='admit_pcm_target_deployment');space={'re':re};exec(compile(ast.Module(body=[fn],type_ignores=[]),'<actual finite deployment>','exec'),space)
for finite,valid in [(False,False),(True,True)]:
    try:space['admit_pcm_target_deployment']('DBT_PROBE_PCM_TARGET_MS:STRING=64\nDBT_PROBE_PCM_COPIED:BOOL=ON','probe',finite,False,False)
    except RuntimeError:assert not valid
    else:assert valid
cli=0
if sys.platform=='win32':
    for args,needle in [(['-PcmCopied'],'Compact copied PCM requires'),(['-PcmCopied','-PcmLoans'],'mutually exclusive')]:
        run=subprocess.run(['powershell','-NoProfile','-ExecutionPolicy','Bypass','-File',str(ROOT/'tools/build_probe.ps1'),*args],capture_output=True,text=True);assert run.returncode and needle in run.stderr;cli+=1
(OUT/'results.json').write_text(json.dumps({'passed':True,'cmake_cases':len(cases),'actual_compact_mode_cases':copy_cases,'actual_logger_cases':logger_cases,'actual_deployment_cases':2,'cli_rejections':cli,'hardware_accessed':False},indent=2)+'\n');print(f'PASS compact/logger guards: {len(cases)} CMake, {copy_cases} compact, {logger_cases} logger, 2 deployment and {cli} CLI cases')
