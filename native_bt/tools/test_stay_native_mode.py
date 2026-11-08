"""Explicit persistent development admission; no serial, reset or flashing."""
import ast,json,pathlib,re,subprocess,sys
ROOT=pathlib.Path(__file__).resolve().parents[1];OUT=ROOT/'_build/stay_native_mode';OUT.mkdir(exist_ok=True)
(OUT/'results.json').write_text(json.dumps({'passed':False,'phase':'checking','hardware_accessed':False})+'\n')
guard=OUT/'guard.cmake';guard.write_text((ROOT/'esp32_probe/CMakeLists.txt').read_text().split('set(COMPONENTS main)')[0])
base={'STAY_NATIVE':'ON','KEEP_OWNED_ACTIVE':'ON','PRODUCT_IO':'ON','PRODUCT_SECURITY':'ON','AUDIO_TEST':'ON',
      'AUDIO_OUTPUT':'ON','ASYNC_AUDIO':'ON','SBC_BUILD':'ON','CLASSIC_SECURITY_TEST':'ON','PCM_COPIED':'ON',
      'BLOCK_DSP':'ON','WORD_DSP':'ON','PCM_TARGET_MS':'16','DECODER_PRIORITY':'21','PRIORITY_ORDER':'ON',
      'SMP':'ON','GATT_TEST':'ON','AVRCP':'ON'}
cases=[({},True),({'STAY_NATIVE':'OFF'},False),({'KEEP_OWNED_ACTIVE':'OFF'},False),({'PRODUCT_IO':'OFF'},False),
       ({'PRODUCT_SECURITY':'OFF'},False),({'AUDIO_TEST':'OFF'},False),({'AUDIO_OUTPUT':'OFF'},False),
       ({'ASYNC_AUDIO':'OFF'},False),({'CODEC_DIAGNOSTIC_ONLY':'ON'},False),({'PIPELINE_DIAGNOSTIC':'ON'},False),
       ({'RF_PROTOCOL_DIAGNOSTIC':'ON'},False),({'SCHEDULER_TRACE':'ON'},True),
       ({'DECODER_PRIORITY':'5'},False),({'DECODER_PRIORITY':'6'},False)]
for change,valid in cases:
    run=subprocess.run(['cmake',*[f'-DDBT_PROBE_{k}={v}' for k,v in (base|change).items()],'-P',str(guard)],capture_output=True,text=True)
    assert (run.returncode==0)==valid,run.stdout+run.stderr
tree=ast.parse((ROOT/'tools/validate_local_image.py').read_text());start=next(i for i,n in enumerate(tree.body) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='stay_native' for t in n.targets));nodes=tree.body[start:start+5];assert isinstance(nodes[-1],ast.If);actual=compile(ast.Module(body=nodes,type_ignores=[]),'<actual stay-native admission>','exec');image_cases=0
def check(change,valid):
    global image_cases
    state={'build_status':{'stay_native':True,'async_audio':True,'product_io':True,'product_security':True},
           'cache':'DBT_PROBE_STAY_NATIVE:BOOL=ON\nDBT_PROBE_KEEP_OWNED_ACTIVE:BOOL=ON','main_command':'cc -DDBT_PROBE_STAY_NATIVE=1',
           'diagnostic':False,'pipeline':False,'entry_name':'probe_main.c','re':re}
    state.update(change)
    try:exec(actual,state)
    except AssertionError:assert not valid
    else:assert valid
    image_cases+=1
check({},True);check({'cache':''},False);check({'cache':'DBT_PROBE_STAY_NATIVE:BOOL=ON'},False)
check({'diagnostic':True},False);check({'pipeline':True},False);check({'entry_name':'probe_codec_diagnostic.c'},False)
for command in ('cc','cc -DDBT_PROBE_STAY_NATIVE=0','cc -DDBT_PROBE_STAY_NATIVE=1 -UDBT_PROBE_STAY_NATIVE'):
    check({'main_command':command},False)
for key in ('async_audio','product_io','product_security'):
    check({'build_status':{'stay_native':True,'async_audio':True,'product_io':True,'product_security':True}|{key:False}},False)
check({'build_status':{},'cache':'','main_command':'cc'},True)
deployment=ast.parse((ROOT/'tools/deploy_audio_trial.py').read_text());fn=next(n for n in deployment.body if isinstance(n,ast.FunctionDef) and n.name=='admit_pcm_target_deployment');space={'re':re};exec(compile(ast.Module(body=[fn],type_ignores=[]),'<actual deployment gate>','exec'),space)
cache='DBT_PROBE_STAY_NATIVE:BOOL=ON\nDBT_PROBE_KEEP_OWNED_ACTIVE:BOOL=ON\nDBT_PROBE_PCM_TARGET_MS:STRING=16\nDBT_PROBE_BLOCK_DSP:BOOL=ON'
deploy_cases=[('probe',False,False,True,True,True),('probe',False,False,True,False,False),('probe',False,False,False,True,False),('probe',True,False,True,True,False),('probe',False,True,True,True,False),('audio',False,False,True,True,False)]
for kind,finite,pipeline,keep,stay,valid in deploy_cases:
    try:space['admit_pcm_target_deployment'](cache,kind,finite,pipeline,keep,stay)
    except RuntimeError:assert not valid
    else:assert valid
(OUT/'results.json').write_text(json.dumps({'passed':True,'cmake_cases':len(cases),'actual_image_cases':image_cases,'actual_deployment_cases':len(deploy_cases),'hardware_accessed':False},indent=2)+'\n')
print(f'PASS stay-native: {len(cases)} CMake, {image_cases} actual image, {len(deploy_cases)} actual deployment cases')
