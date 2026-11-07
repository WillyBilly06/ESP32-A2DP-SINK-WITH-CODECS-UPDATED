"""Exercise actual CMake and image/compiler guards, no hardware."""
import ast,json,pathlib,re,subprocess
ROOT=pathlib.Path(__file__).resolve().parents[1];OUT=ROOT/'_build/owner_pacer_admission';OUT.mkdir(exist_ok=True)
guard=OUT/'guard.cmake';guard.write_text((ROOT/'esp32_probe/CMakeLists.txt').read_text().split('set(COMPONENTS main)')[0])
cache=(ROOT/'_build/esp32/CMakeCache.txt').read_text()
options=dict(re.findall(r'^(DBT_PROBE_[A-Z0-9_]+):(?:BOOL|STRING)=(.*)$',cache,re.M));options['DBT_PROBE_OWNER_PACER']='ON'
cases=[({},True),({'DBT_PROBE_OWNER_PACER':'OFF'},True)]+[({k:'OFF'},False) for k in ['DBT_PROBE_STAY_NATIVE','DBT_PROBE_PRIORITY_ORDER','DBT_PROBE_AUDIO_OUTPUT','DBT_PROBE_ASYNC_AUDIO','DBT_PROBE_PCM_COPIED','DBT_PROBE_CODEC_AWARE_RENDER']]
for change,valid in cases:
    result=subprocess.run(['cmake',*[f'-D{k}={v}' for k,v in (options|change).items()],'-P',str(guard)],capture_output=True,text=True)
    assert (result.returncode==0)==valid,result.stdout+result.stderr
tree=ast.parse((ROOT/'tools/validate_local_image.py').read_text());begin=next(i for i,n in enumerate(tree.body) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='owner_pacer' for t in n.targets));end=next(i for i in range(begin+1,len(tree.body)) if isinstance(tree.body[i],ast.Assign) and any(isinstance(t,ast.Name) and t.id=='dsp_inline' for t in tree.body[i].targets))
code=compile(ast.Module(body=tree.body[begin:end],type_ignores=[]),'<actual owner budget admission>','exec')
commands=[{'file':p,'command':'cc -DDBT_PROBE_OWNER_PACER=1'} for p in ['probe_main.c','probe_audio.c','probe_owner_pacer.c']]
base={'build_status':dict.fromkeys(['owner_pacer','stay_native','priority_order','async_audio','pcm_copied','codec_aware_render'],True),'cache':'DBT_PROBE_OWNER_PACER:BOOL=ON','main_command':commands[0]['command'],'commands':commands,'diagnostic':False,'pipeline':False,'Path':pathlib.Path,'re':re}
checks=[({},True),({'cache':''},False),({'main_command':'cc'},False),({'commands':commands[:2]},False),({'diagnostic':True},False),({'pipeline':True},False),({'commands':[e|{'command':e['command']+' -UDBT_PROBE_OWNER_PACER'} for e in commands]},False)]
checks += [({'build_status':base['build_status']|{k:False}},False) for k in ['owner_pacer','stay_native','priority_order','async_audio','pcm_copied','codec_aware_render']]
checks += [({'build_status':{},'cache':'','main_command':'cc','commands':[]},True)]
for change,valid in checks:
    try:exec(code,base|change)
    except (AssertionError,StopIteration):assert not valid
    else:assert valid
(OUT/'results.json').write_text(json.dumps({'passed':True,'hardware_accessed':False,'actual_cmake_cases':len(cases),'actual_image_guard_cases':len(checks),'scope':'actual extracted CMake/compiler/receipt admission, not timing or deployment qualification'},indent=2)+'\n')
print(f'PASS actual owner budget admission: {len(cases)} CMake and {len(checks)} compiler/receipt cases')
