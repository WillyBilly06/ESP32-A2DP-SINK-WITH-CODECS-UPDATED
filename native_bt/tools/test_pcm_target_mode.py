"""Actual finite PCM-target build/admission guards; no device access."""
import ast,json,pathlib,re,subprocess,sys
ROOT=pathlib.Path(__file__).resolve().parents[1];OUT=ROOT/'_build/pcm_target_mode';OUT.mkdir(exist_ok=True)
guard=OUT/'guard.cmake';guard.write_text((ROOT/'esp32_probe/CMakeLists.txt').read_text().split('set(COMPONENTS main)')[0])
base={'PCM_TARGET_MS':'16','PCM_LOANS':'ON','ASYNC_AUDIO':'ON','AUDIO_OUTPUT':'ON','AUDIO_TEST':'ON','SBC_BUILD':'ON','CLASSIC_SECURITY_TEST':'ON','KEEP_OWNED_ACTIVE':'OFF'}
cases=[({},True),({'PCM_TARGET_MS':'32'},True),({'PCM_TARGET_MS':'64'},True),({'PCM_TARGET_MS':'8'},False),
       ({'PCM_LOANS':'OFF'},False),({'ASYNC_AUDIO':'OFF'},False),({'KEEP_OWNED_ACTIVE':'ON'},False),({'RF_PROTOCOL_DIAGNOSTIC':'ON'},False)]
for change,valid in cases:
    run=subprocess.run(['cmake',*[f'-DDBT_PROBE_{k}={v}' for k,v in (base|change).items()],'-P',str(guard)],capture_output=True,text=True)
    assert (run.returncode==0)==valid,run.stdout+run.stderr
tree=ast.parse((ROOT/'tools/validate_local_image.py').read_text())
start=next(i for i,n in enumerate(tree.body) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='pcm_target_ms' for t in n.targets))
nodes=tree.body[start:start+5];assert isinstance(nodes[-1],ast.If)
actual=compile(ast.Module(body=nodes,type_ignores=[]),'<actual PCM target checks>','exec');count=0
def check(change,valid):
    global count
    files=('probe_main.c','probe_output.c','probe_audio.c')
    state={'build_status':{'pcm_target_ms':16,'async_audio':True},'pcm_loans':True,'rf_diagnostic':False,
           'cache':'DBT_PROBE_PCM_TARGET_MS:STRING=16\nDBT_PROBE_KEEP_OWNED_ACTIVE:BOOL=OFF\n',
           'commands':[{'file':p,'command':'cc -DDBT_PROBE_PCM_TARGET_MS=16 -c '+p} for p in files],
           'entry_name':'probe_main.c','Path':pathlib.Path,'re':re}
    state.update(change)
    try:exec(actual,state)
    except (AssertionError,StopIteration):assert not valid
    else:assert valid
    count+=1
check({},True)
check({'build_status':{'pcm_target_ms':64},'cache':'','commands':[]},True)
check({'build_status':{'pcm_target_ms':8,'async_audio':True}},False)
check({'build_status':{'pcm_target_ms':'16','async_audio':True}},False)
check({'build_status':{'pcm_target_ms':32,'async_audio':True}},False)
check({'cache':''},False)
check({'pcm_loans':False},False)
check({'rf_diagnostic':True},False)
check({'build_status':{'pcm_target_ms':16,'async_audio':False}},False)
check({'cache':'DBT_PROBE_PCM_TARGET_MS:STRING=16\nDBT_PROBE_KEEP_OWNED_ACTIVE:BOOL=ON'},False)
for missing in ('probe_main.c','probe_output.c','probe_audio.c'):
    check({'commands':[{'file':p,'command':'cc' if p==missing else 'cc -DDBT_PROBE_PCM_TARGET_MS=16'} for p in ('probe_main.c','probe_output.c','probe_audio.c')]},False)
check({'commands':[{'file':'probe_output.c','command':'cc -DDBT_PROBE_PCM_TARGET_MS=16 -DDBT_PROBE_PCM_TARGET_MS=64'}]},False)
check({'commands':[{'file':'probe_output.c','command':'cc -DDBT_PROBE_PCM_TARGET_MS=16 -UDBT_PROBE_PCM_TARGET_MS'}]},False)
check({'build_status':{'pcm_target_ms':64},'cache':''},False)
cli=0
if sys.platform=='win32':
    for target in (16,32):
        run=subprocess.run(['powershell','-NoProfile','-ExecutionPolicy','Bypass','-File',str(ROOT/'tools/build_probe.ps1'),'-PcmTargetMs',str(target)],capture_output=True,text=True)
        assert run.returncode and 'Short PCM target requires' in run.stderr;cli+=1
(OUT/'results.json').write_text(json.dumps({'passed':True,'cmake_cases':len(cases),'actual_validator_cases':count,'cli_rejections':cli,'hardware_accessed':False},indent=2)+'\n')
print(f'PASS PCM target: {len(cases)} CMake configurations, {count} actual admission cases, {cli} early CLI rejections')
