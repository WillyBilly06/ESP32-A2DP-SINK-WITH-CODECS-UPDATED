"""Actual word-DSP CMake and image-mode admission; no device access."""
import ast,json,pathlib,re,subprocess,sys
ROOT=pathlib.Path(__file__).resolve().parents[1];OUT=ROOT/'_build/word_dsp_mode';OUT.mkdir(exist_ok=True)
guard=OUT/'guard.cmake';guard.write_text((ROOT/'esp32_probe/CMakeLists.txt').read_text().split('set(COMPONENTS main)')[0])
base={'WORD_DSP':'ON','BLOCK_DSP':'ON','PCM_COPIED':'ON','ASYNC_AUDIO':'ON','AUDIO_OUTPUT':'ON',
      'AUDIO_TEST':'ON','SBC_BUILD':'ON','CLASSIC_SECURITY_TEST':'ON','KEEP_OWNED_ACTIVE':'OFF'}
cases=[({},True),({'BLOCK_DSP':'OFF'},False),({'PCM_COPIED':'OFF'},False),({'KEEP_OWNED_ACTIVE':'ON'},False),
       ({'RF_PROTOCOL_DIAGNOSTIC':'ON'},False),({'ASYNC_AUDIO':'OFF'},False),
       ({'PCM_COPIED':'OFF','PCM_LOANS':'ON'},True),({'WORD_DSP':'OFF'},True)]
for change,valid in cases:
    run=subprocess.run(['cmake',*[f'-DDBT_PROBE_{k}={v}' for k,v in (base|change).items()],'-P',str(guard)],capture_output=True,text=True)
    assert (run.returncode==0)==valid,run.stdout+run.stderr
tree=ast.parse((ROOT/'tools/validate_local_image.py').read_text())
start=next(i for i,n in enumerate(tree.body) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='word_dsp' for t in n.targets))
nodes=tree.body[start:start+6];assert isinstance(nodes[-1],ast.If)
actual=compile(ast.Module(body=nodes,type_ignores=[]),'<actual word DSP checks>','exec');count=0
def check(change,valid):
    global count
    state={'build_status':{'word_dsp':True,'async_audio':True},'cache':'DBT_PROBE_WORD_DSP:BOOL=ON',
           'main_command':'cc -DDBT_PROBE_WORD_DSP=1','block_dsp':True,'rf_diagnostic':False,
           'block_core_commands':[{'file':name,'command':'cc -DFIXED_DSP_WORD_QUANT=1'} for name in ('dbt_audio.c','fixed_dsp.c')],
           'Path':pathlib.Path,'re':re}
    state.update(change)
    try:exec(actual,state)
    except (AssertionError,StopIteration):assert not valid
    else:assert valid
    count+=1
check({},True);check({'block_dsp':False},False);check({'rf_diagnostic':True},False);check({'cache':''},False)
check({'build_status':{'word_dsp':1,'async_audio':True}},False)
check({'build_status':{'word_dsp':True,'async_audio':False}},False)
for command in ('cc','cc -DDBT_PROBE_WORD_DSP=0','cc -DDBT_PROBE_WORD_DSP=1 -UDBT_PROBE_WORD_DSP'):
    check({'main_command':command},False)
for command in ('cc','cc -DFIXED_DSP_WORD_QUANT=0','cc -DFIXED_DSP_WORD_QUANT=1 -UFIXED_DSP_WORD_QUANT',
                'cc -DFIXED_DSP_WORD_QUANT=1 -DFIXED_DSP_WORD_QUANT=0'):
    check({'block_core_commands':[{'file':name,'command':command} for name in ('dbt_audio.c','fixed_dsp.c')]},False)
check({'block_core_commands':[]},False)
check({'cache':'DBT_PROBE_WORD_DSP:BOOL=ON\nDBT_PROBE_KEEP_OWNED_ACTIVE:BOOL=ON'},False)
check({'build_status':{},'cache':'','main_command':'cc','block_dsp':False,'block_core_commands':[]},True)
cli=0
if sys.platform=='win32':
    run=subprocess.run(['powershell','-NoProfile','-ExecutionPolicy','Bypass','-File',str(ROOT/'tools/build_probe.ps1'),'-WordDsp'],capture_output=True,text=True)
    assert run.returncode and 'Word DSP requires' in run.stderr;cli=1
(OUT/'results.json').write_text(json.dumps({'passed':True,'cmake_cases':len(cases),'actual_image_cases':count,'cli_rejections':cli,'hardware_accessed':False},indent=2)+'\n')
print(f'PASS word DSP: {len(cases)} CMake, {count} actual image and {cli} early CLI cases')
