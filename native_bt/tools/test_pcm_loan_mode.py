"""Exercise actual PCM-loan CMake/CLI admission guards and compiled-mode checks."""
import ast,json,pathlib,re,subprocess,sys
ROOT=pathlib.Path(__file__).resolve().parents[1];OUT=ROOT/'_build/pcm_loan_mode';OUT.mkdir(exist_ok=True)
guard=OUT/'guard.cmake';guard.write_text((ROOT/'esp32_probe/CMakeLists.txt').read_text().split('set(COMPONENTS main)')[0])
base={'PCM_LOANS':'ON','ASYNC_AUDIO':'ON','AUDIO_OUTPUT':'ON','AUDIO_TEST':'ON','SBC_BUILD':'ON','CLASSIC_SECURITY_TEST':'ON','KEEP_OWNED_ACTIVE':'OFF'}
cases=[({},True),({'ASYNC_AUDIO':'OFF'},False),({'AUDIO_OUTPUT':'OFF'},False),({'KEEP_OWNED_ACTIVE':'ON'},False),
       ({'RF_PROTOCOL_DIAGNOSTIC':'ON'},False),({'PCM_LOANS':'OFF','ASYNC_AUDIO':'OFF'},True),
       ({'PIPELINE_DIAGNOSTIC':'ON','CODEC_DIAGNOSTIC_ONLY':'ON','LHDC_BUILD':'ON','AUDIO_TEST':'OFF','CLASSIC_SECURITY_TEST':'OFF'},True)]
for change,valid in cases:
    options=base|change
    result=subprocess.run(['cmake',*[f'-DDBT_PROBE_{k}={v}' for k,v in options.items()],'-P',str(guard)],capture_output=True,text=True)
    assert (result.returncode==0)==valid,result.stdout+result.stderr

tree=ast.parse((ROOT/'tools/validate_local_image.py').read_text())
start=next(i for i,node in enumerate(tree.body) if isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='pcm_loans' for t in node.targets))
nodes=tree.body[start:start+4];assert isinstance(nodes[-1],ast.If)
actual=compile(ast.Module(body=nodes,type_ignores=[]),'<actual PCM-loan admission>','exec')
count=0
def check(change,valid):
    global count
    mode=True;pipeline=False
    state={'build_status':{'pcm_loans':True,'async_audio':True},'rf_diagnostic':False,'pipeline':pipeline,
           'main_command':'cc -DDBT_PROBE_PCM_LOANS=1 -c main.c',
           'cache':'DBT_PROBE_PCM_LOANS:BOOL=ON\nDBT_PROBE_KEEP_OWNED_ACTIVE:BOOL=OFF\n',
           'commands':[{'file':p,'command':'cc -DDBT_PROBE_PCM_LOANS=1 -c '+p} for p in ('probe_output.c','probe_audio.c')],
           'Path':pathlib.Path,'re':re}
    state.update(change)
    try:exec(actual,state)
    except (AssertionError,StopIteration):assert not valid
    else:assert valid
    count+=1
check({},True)
check({'pipeline':True,'commands':[{'file':p,'command':'cc -DDBT_PROBE_PCM_LOANS=1'} for p in ('probe_output.c','probe_pipeline_check.c')]},True)
check({'build_status':{'pcm_loans':False},'cache':'DBT_PROBE_PCM_LOANS:BOOL=OFF','main_command':'cc','commands':[]},True)
check({'cache':'DBT_PROBE_PCM_LOANS:BOOL=OFF'},False)
check({'build_status':{'pcm_loans':False,'async_audio':True}},False)
check({'main_command':'cc -c main.c'},False)
check({'build_status':{'pcm_loans':True,'async_audio':False}},False)
check({'rf_diagnostic':True},False)
check({'cache':'DBT_PROBE_PCM_LOANS:BOOL=ON\nDBT_PROBE_KEEP_OWNED_ACTIVE:BOOL=ON'},False)
for missing in ('probe_output.c','probe_audio.c'):
    check({'commands':[{'file':p,'command':'cc' if p==missing else 'cc -DDBT_PROBE_PCM_LOANS=1'} for p in ('probe_output.c','probe_audio.c')]},False)
cli=0
if sys.platform=='win32':
    for args in (['-PcmLoans'],['-PcmLoans','-WithProductIo']):
        result=subprocess.run(['powershell','-NoProfile','-ExecutionPolicy','Bypass','-File',str(ROOT/'tools/build_probe.ps1'),*args],capture_output=True,text=True)
        assert result.returncode and 'PCM loans require' in result.stderr
        cli+=1
(OUT/'results.json').write_text(json.dumps({'passed':True,'cmake_cases':len(cases),'actual_validator_cases':count,'cli_rejections':cli,'scope':'actual configuration/receipt/compile-mode rules only; no hardware'},indent=2)+'\n')
print(f'PASS PCM-loan mode: {len(cases)} CMake cases, {count} actual admission cases, {cli} early CLI rejections')
