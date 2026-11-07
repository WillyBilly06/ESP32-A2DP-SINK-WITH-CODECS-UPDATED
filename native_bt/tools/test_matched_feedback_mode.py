"""Exercise actual matched-feedback build/image admission. Never access hardware."""
import ast,json,pathlib,re,subprocess,sys
ROOT=pathlib.Path(__file__).resolve().parents[1];OUT=ROOT/'_build/matched_feedback_guard';OUT.mkdir(exist_ok=True)
guard=OUT/'guard.cmake';guard.write_text((ROOT/'esp32_probe/CMakeLists.txt').read_text().split('set(COMPONENTS main)')[0])
base={k:'ON' for k in ['MATCHED_FEEDBACK','MATCHED_FEEDBACK_COMPARE','STAY_NATIVE','BLOCK_DSP','WORD_DSP','PRIORITY_ORDER','CODEC_AWARE_RENDER','ASYNC_AUDIO','PCM_COPIED','KEEP_OWNED_ACTIVE','PRODUCT_IO','PRODUCT_SECURITY','AUDIO_TEST','AUDIO_OUTPUT','SBC_BUILD','CLASSIC_SECURITY_TEST','SMP','GATT_TEST','AVRCP']};base['DECODER_PRIORITY']='21'
cases=[({},True),({'MATCHED_FEEDBACK_COMPARE':'OFF'},True),({'MATCHED_FEEDBACK':'OFF','MATCHED_FEEDBACK_COMPARE':'OFF'},True),({'MATCHED_FEEDBACK':'OFF'},False),({'STAY_NATIVE':'OFF'},False),({'BLOCK_DSP':'OFF'},False),({'WORD_DSP':'OFF'},False),({'VOLUME_RAMP_COMPARE':'ON'},False),({'OWNER_PACER':'ON'},False)]
for changes,valid in cases:
    r=subprocess.run(['cmake',*[f'-DDBT_PROBE_{k}={v}' for k,v in (base|changes).items()],'-P',str(guard)],capture_output=True,text=True)
    assert (r.returncode==0)==valid,r.stdout+r.stderr
tree=ast.parse((ROOT/'tools/validate_local_image.py').read_text())
def assignment(name):return next(i for i,n in enumerate(tree.body) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id==name for t in n.targets))
actual=compile(ast.Module(body=tree.body[assignment('matched_feedback'):assignment('owner_pacer')],type_ignores=[]),'<actual matched admission>','exec')
receipt={'matched_feedback':True,'matched_feedback_compare':True,'block_dsp':True,'word_dsp':True}
cache='DBT_PROBE_MATCHED_FEEDBACK:BOOL=ON\nDBT_PROBE_MATCHED_FEEDBACK_COMPARE:BOOL=ON'
main='cc -DDBT_PROBE_MATCHED_FEEDBACK=1 -DDBT_PROBE_MATCHED_FEEDBACK_COMPARE=1'
fixed='cc -DFIXED_DSP_MATCHED_FEEDBACK=1 -DFIXED_DSP_MATCHED_FEEDBACK_COMPARE=1 -std=gnu11'
count=0
def check(changes,valid,rc=None):
    global count
    state={'build_status':receipt|(rc or {}),'cache':cache,'main_command':main,'fixed_volume_unit':{'command':fixed},'commands':[{'file':'probe_audio.c','command':main}],'stay_native':True,'diagnostic':False,'pipeline':False,'volume_compare':False,'Path':pathlib.Path,'re':re};state.update(changes)
    try:exec(actual,state)
    except (AssertionError,StopIteration):assert not valid
    else:assert valid
    count+=1
check({},True)
check({'fixed_volume_unit':{'command':fixed+' -std=c11'}},False)
check({'cache':'','main_command':'cc','fixed_volume_unit':{'command':'cc'},'commands':[]},True,{'matched_feedback':False,'matched_feedback_compare':False})
for rc in [{'matched_feedback':1},{'matched_feedback_compare':1},{'matched_feedback':False},{'word_dsp':False},{'block_dsp':False},{'owner_pacer':True}]:check({},False,rc)
for changes in [{'cache':''},{'stay_native':False},{'diagnostic':True},{'pipeline':True},{'volume_compare':True},{'commands':[]},{'main_command':main.replace('FEEDBACK=1','FEEDBACK=0')},{'fixed_volume_unit':{'command':fixed.replace('FEEDBACK=1','FEEDBACK=0')}},{'fixed_volume_unit':{'command':fixed+' -UFIXED_DSP_MATCHED_FEEDBACK'}},{'commands':[{'file':'probe_audio.c','command':main+' -UDBT_PROBE_MATCHED_FEEDBACK_COMPARE'}]}]:check(changes,False)
cli=0
if sys.platform=='win32':
    r=subprocess.run(['powershell','-NoProfile','-ExecutionPolicy','Bypass','-File',str(ROOT/'tools/build_probe.ps1'),'-MatchedFeedback'],capture_output=True,text=True);assert r.returncode and 'Matched feedback requires' in r.stderr;cli=1
(OUT/'results.json').write_text(json.dumps({'passed':True,'actual_cmake_cases':len(cases),'actual_image_cases':count,'cli_rejections':cli,'hardware_accessed':False},indent=2)+'\n')
print(f'PASS matched feedback admission: {len(cases)} CMake, {count} image and {cli} CLI checks')
