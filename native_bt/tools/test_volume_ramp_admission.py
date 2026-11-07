"""Actual CMake and compile/receipt checks for the volume-ramp experiment."""
import ast,json,pathlib,re,subprocess
ROOT=pathlib.Path(__file__).resolve().parents[1];OUT=ROOT/'_build/volume_ramp_admission';OUT.mkdir(exist_ok=True)
guard=OUT/'guard.cmake';guard.write_text((ROOT/'esp32_probe/CMakeLists.txt').read_text().split('set(COMPONENTS main)')[0])
cache=(ROOT/'_build/esp32/CMakeCache.txt').read_text();options=dict(re.findall(r'^(DBT_PROBE_[A-Z0-9_]+):(?:BOOL|STRING)=(.*)$',cache,re.M))
options.update(DBT_PROBE_VOLUME_RAMP_BLOCK='ON',DBT_PROBE_VOLUME_RAMP_COMPARE='ON')
cases=[({},True),({'DBT_PROBE_VOLUME_RAMP_COMPARE':'OFF'},True),({'DBT_PROBE_VOLUME_RAMP_COMPARE':'OFF','DBT_PROBE_VOLUME_RAMP_BLOCK':'OFF'},True),({'DBT_PROBE_VOLUME_RAMP_BLOCK':'OFF'},False),({'DBT_PROBE_STAY_NATIVE':'OFF'},False),({'DBT_PROBE_BLOCK_DSP':'OFF'},False)]
for change,valid in cases:
    run=subprocess.run(['cmake',*[f'-D{k}={v}' for k,v in (options|change).items()],'-P',str(guard)],capture_output=True,text=True)
    assert (run.returncode==0)==valid,run.stdout+run.stderr
tree=ast.parse((ROOT/'tools/validate_local_image.py').read_text());start=next(i for i,n in enumerate(tree.body) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='volume_ramp' for t in n.targets));end=next(i for i in range(start+1,len(tree.body)) if isinstance(tree.body[i],ast.Assign) and any(isinstance(t,ast.Name) and t.id=='owner_pacer' for t in tree.body[i].targets))
code=compile(ast.Module(body=tree.body[start:end],type_ignores=[]),'<actual volume ramp admission>','exec')
main='cc -DDBT_PROBE_VOLUME_RAMP_BLOCK=1 -DDBT_PROBE_VOLUME_RAMP_COMPARE=1';fixed='cc -DFIXED_DSP_VOLUME_RAMP_BLOCK=1 -DFIXED_DSP_VOLUME_RAMP_COMPARE=1'
base={'build_status':dict(volume_ramp_block=True,volume_ramp_compare=True,stay_native=True,block_dsp=True),'cache':'DBT_PROBE_VOLUME_RAMP_BLOCK:BOOL=ON\nDBT_PROBE_VOLUME_RAMP_COMPARE:BOOL=ON','main_command':main,'commands':[dict(file='fixed_dsp.c',command=fixed)],'diagnostic':False,'pipeline':False,'Path':pathlib.Path,'re':re}
checks=[({},True),({'cache':''},False),({'main_command':'cc'},False),({'commands':[]},False),({'commands':[dict(file='fixed_dsp.c',command=fixed+' -UFIXED_DSP_VOLUME_RAMP_BLOCK')]},False),({'diagnostic':True},False),({'pipeline':True},False)]
checks += [({'build_status':base['build_status']|{k:False}},False) for k in ['volume_ramp_block','volume_ramp_compare','stay_native','block_dsp']]
checks += [({'build_status':{},'cache':'','main_command':'cc','commands':[dict(file='fixed_dsp.c',command='cc')]},True)]
for change,valid in checks:
    try:exec(code,base|change)
    except (AssertionError,StopIteration):assert not valid
    else:assert valid
(OUT/'results.json').write_text(json.dumps({'passed':True,'hardware_accessed':False,'cmake_cases':len(cases),'image_compiler_cases':len(checks)},indent=2)+'\n')
print(f'PASS volume ramp admission: {len(cases)} actual CMake and {len(checks)} actual compiler/receipt cases')
