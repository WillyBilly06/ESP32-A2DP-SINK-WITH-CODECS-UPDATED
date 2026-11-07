"""Actual CMake/image admission for copied-input early release. No hardware."""
import ast,json,pathlib,re,subprocess,sys
ROOT=pathlib.Path(__file__).resolve().parents[1];OUT=ROOT/'_build/early_input_guard';OUT.mkdir(exist_ok=True)
guard=OUT/'guard.cmake';guard.write_text((ROOT/'esp32_probe/CMakeLists.txt').read_text().split('set(COMPONENTS main)')[0])
base={k:'ON' for k in ['EARLY_INPUT_RELEASE','STAY_NATIVE','BLOCK_DSP','WORD_DSP','UNITY_PCM_COPY','PRIORITY_ORDER','CODEC_AWARE_RENDER','ASYNC_AUDIO','PCM_COPIED','KEEP_OWNED_ACTIVE','PRODUCT_IO','PRODUCT_SECURITY','AUDIO_TEST','AUDIO_OUTPUT','SBC_BUILD','CLASSIC_SECURITY_TEST','SMP','GATT_TEST','AVRCP']};base['DECODER_PRIORITY']='21'
cases=[({},True),({'EARLY_INPUT_RELEASE':'OFF'},True),({'UNITY_PCM_COPY':'OFF'},False),({'CODEC_AWARE_RENDER':'OFF'},False),({'BLOCK_DSP':'OFF','WORD_DSP':'OFF','UNITY_PCM_COPY':'OFF'},False),({'MATCHED_FEEDBACK':'ON','MATCHED_FEEDBACK_COMPARE':'ON'},False),({'VOLUME_RAMP_BLOCK':'ON','VOLUME_RAMP_COMPARE':'ON'},False),({'OWNER_PACER':'ON'},False)]
for changes,valid in cases:
    r=subprocess.run(['cmake',*[f'-DDBT_PROBE_{k}={v}' for k,v in (base|changes).items()],'-P',str(guard)],capture_output=True,text=True);assert (r.returncode==0)==valid,r.stdout+r.stderr
tree=ast.parse((ROOT/'tools/validate_local_image.py').read_text())
def assignment(name):return next(i for i,n in enumerate(tree.body) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id==name for t in n.targets))
actual=compile(ast.Module(body=tree.body[assignment('early_input_release'):assignment('matched_feedback')],type_ignores=[]),'<actual early input admission>','exec')
receipt={'early_input_release':True,'codec_aware_render':True,'async_audio':True,'pcm_copied':True,'unity_pcm_copy':True,'block_dsp':True}
command='cc -DDBT_PROBE_EARLY_INPUT_RELEASE=1';units=['probe_main.c','probe_output.c'];count=0

def check(changes,valid,rc=None):
    global count
    state={'build_status':receipt|(rc or {}),'cache':'DBT_PROBE_EARLY_INPUT_RELEASE:BOOL=ON','commands':[{'file':n,'command':command} for n in units],'entry_name':'probe_main.c','stay_native':True,'volume_compare':False,'Path':pathlib.Path,'re':re};state.update(changes)
    try:exec(actual,state)
    except (AssertionError,StopIteration):assert not valid
    else:assert valid
    count+=1
check({},True)
check({'cache':'','commands':[{'file':n,'command':'cc'} for n in units]},True,{'early_input_release':False})
for field in ['codec_aware_render','async_audio','pcm_copied','unity_pcm_copy','block_dsp']:check({},False,{field:False})
for rc in [{'early_input_release':1},{'owner_pacer':True},{'matched_feedback_compare':True}]:check({},False,rc)
for changes in [{'stay_native':False},{'volume_compare':True},{'cache':''},{'commands':[]}]:check(changes,False)
for bad in ['cc','cc -DDBT_PROBE_EARLY_INPUT_RELEASE=0',command+' -UDBT_PROBE_EARLY_INPUT_RELEASE',command+' -DDBT_PROBE_EARLY_INPUT_RELEASE=1']:check({'commands':[{'file':n,'command':bad} for n in units]},False)
cli=0
if sys.platform=='win32':
    r=subprocess.run(['powershell','-NoProfile','-ExecutionPolicy','Bypass','-File',str(ROOT/'tools/build_probe.ps1'),'-EarlyInputRelease'],capture_output=True,text=True);assert r.returncode and 'Early input release requires' in r.stderr;cli=1
(OUT/'results.json').write_text(json.dumps({'passed':True,'actual_cmake_cases':len(cases),'actual_image_cases':count,'cli_rejections':cli,'hardware_accessed':False},indent=2)+'\n');print(f'PASS early input admission: {len(cases)} CMake, {count} image and {cli} CLI cases')
