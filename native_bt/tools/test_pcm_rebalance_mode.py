"""Actual high-rate PCM redistribution admission. No hardware."""
import ast,json,pathlib,re,subprocess,sys
ROOT=pathlib.Path(__file__).resolve().parents[1];OUT=ROOT/'_build/pcm_rebalance_guard';OUT.mkdir(exist_ok=True)
guard=OUT/'guard.cmake';guard.write_text((ROOT/'esp32_probe/CMakeLists.txt').read_text().split('set(COMPONENTS main)')[0])
base={k:'ON' for k in ['HIGH_RATE_PCM_REBALANCE','EARLY_INPUT_RELEASE','LHDC_BUILD','STAY_NATIVE','BLOCK_DSP','WORD_DSP','UNITY_PCM_COPY','PRIORITY_ORDER','CODEC_AWARE_RENDER','ASYNC_AUDIO','PCM_COPIED','KEEP_OWNED_ACTIVE','PRODUCT_IO','PRODUCT_SECURITY','AUDIO_TEST','AUDIO_OUTPUT','SBC_BUILD','CLASSIC_SECURITY_TEST','SMP','GATT_TEST','AVRCP']};base|={'DECODER_PRIORITY':'21','PCM_TARGET_MS':'16'}
cases=[({},True),({'HIGH_RATE_PCM_REBALANCE':'OFF'},True),({'EARLY_INPUT_RELEASE':'OFF'},False),({'LHDC_BUILD':'OFF'},False),({'PCM_TARGET_MS':'32'},False),({'LHDC_CORE0_RENDER':'ON'},False)]
for changes,valid in cases:
    r=subprocess.run(['cmake',*[f'-DDBT_PROBE_{k}={v}' for k,v in (base|changes).items()],'-P',str(guard)],capture_output=True,text=True);assert (r.returncode==0)==valid,r.stdout+r.stderr
tree=ast.parse((ROOT/'tools/validate_local_image.py').read_text())
def assignment(name):return next(i for i,n in enumerate(tree.body) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id==name for t in n.targets))
actual=compile(ast.Module(body=tree.body[assignment('high_rate_pcm_rebalance'):assignment('matched_feedback')],type_ignores=[]),'<actual PCM rebalance admission>','exec')
receipt={'high_rate_pcm_rebalance':True,'pcm_target_ms':16};cache='DBT_PROBE_HIGH_RATE_PCM_REBALANCE:BOOL=ON\nDBT_PROBE_PCM_TARGET_MS:STRING=16\nDBT_PROBE_LHDC_BUILD:BOOL=ON';command='cc -DDBT_PROBE_HIGH_RATE_PCM_REBALANCE=1';units=['probe_main.c','probe_output.c'];count=0
def check(changes,valid,rc=None):
    global count
    state={'build_status':receipt|(rc or {}),'cache':cache,'commands':[{'file':n,'command':command} for n in units],'entry_name':'probe_main.c','early_input_release':True,'Path':pathlib.Path,'re':re};state.update(changes)
    try:exec(actual,state)
    except (AssertionError,StopIteration):assert not valid
    else:assert valid
    count+=1
check({},True);check({'cache':'','commands':[]},True,{'high_rate_pcm_rebalance':False})
for rc in [{'high_rate_pcm_rebalance':1},{'pcm_target_ms':32},{'pcm_target_ms':16.0},{'lhdc_core0_render':True}]:check({},False,rc)
for changes in [{'cache':''},{'early_input_release':False},{'commands':[]},{'cache':cache.replace('DBT_PROBE_LHDC_BUILD:BOOL=ON','')},{'cache':cache+'\nDBT_PROBE_LHDC_CORE0_RENDER:BOOL=ON'}]:check(changes,False)
for bad in ['cc',command+' -UDBT_PROBE_HIGH_RATE_PCM_REBALANCE',command+' -DDBT_PROBE_HIGH_RATE_PCM_REBALANCE=0']:check({'commands':[{'file':n,'command':bad} for n in units]},False)
(OUT/'results.json').write_text(json.dumps({'passed':True,'actual_cmake_cases':len(cases),'actual_image_cases':count,'hardware_accessed':False},indent=2)+'\n');print(f'PASS PCM rebalance admission: {len(cases)} CMake and {count} image cases')
