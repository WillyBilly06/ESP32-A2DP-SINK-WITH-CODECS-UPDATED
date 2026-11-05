"""Actual scheduling header, CMake and image admission checks; no hardware."""
import ast
import json
import pathlib
import re
import subprocess

ROOT=pathlib.Path(__file__).resolve().parents[1]
OUT=ROOT/'_build/priority_order'
OUT.mkdir(exist_ok=True)
guard=OUT/'guard.cmake'
guard.write_text((ROOT/'esp32_probe/CMakeLists.txt').read_text().split('set(COMPONENTS main)')[0])
base={'PRIORITY_ORDER':'ON','DECODER_PRIORITY':'6','ASYNC_AUDIO':'ON','AUDIO_OUTPUT':'ON',
      'AUDIO_TEST':'ON','SBC_BUILD':'ON','CLASSIC_SECURITY_TEST':'ON','KEEP_OWNED_ACTIVE':'OFF'}
cmake_cases=[({},True),({'DECODER_PRIORITY':'5'},False),({'ASYNC_AUDIO':'OFF'},False),
             ({'AUDIO_OUTPUT':'OFF'},False),({'AUDIO_TEST':'OFF'},False),
             ({'KEEP_OWNED_ACTIVE':'ON'},False),({'RF_PROTOCOL_DIAGNOSTIC':'ON'},False),
             ({'CODEC_DIAGNOSTIC_ONLY':'ON'},False),({'PIPELINE_DIAGNOSTIC':'ON'},False),
             ({'PRIORITY_ORDER':'OFF','DECODER_PRIORITY':'5'},True)]
for change,valid in cmake_cases:
    run=subprocess.run(['cmake',*[f'-DDBT_PROBE_{k}={v}' for k,v in (base|change).items()],'-P',str(guard)],capture_output=True,text=True)
    assert (run.returncode==0)==valid,run.stdout+run.stderr

source=OUT/'limits.c'
header_cases=[([],6,5,6,0,True),(['-DDBT_PROBE_DECODER_PRIORITY=6'],6,6,6,1,True),
              (['-DDBT_PROBE_DECODER_PRIORITY=6','-DDBT_PROBE_PRIORITY_ORDER=1'],7,6,5,0,True),
              (['-DDBT_PROBE_PRIORITY_ORDER=1'],7,5,5,0,False),
              (['-DDBT_PROBE_DECODER_PRIORITY=7'],6,7,6,0,False),
              (['-DDBT_PROBE_PRIORITY_ORDER=0'],6,5,6,0,False),
              (['-DDBT_PROBE_HOST_PRIORITY=7'],7,5,6,0,False),
              (['-DDBT_PROBE_RENDER_PRIORITY=5'],6,5,5,0,False)]
for flags,host,decoder,render,yield_ready,valid in header_cases:
    source.write_text('#include "probe_schedule_limits.h"\n'+
                      ''.join(f'_Static_assert({macro}=={value},"{macro}");\n' for macro,value in
                              [('DBT_PROBE_HOST_PRIORITY',host),('DBT_PROBE_DECODER_PRIORITY',decoder),
                               ('DBT_PROBE_RENDER_PRIORITY',render),('DBT_PROBE_OWNER_READY_YIELD',yield_ready)]))
    run=subprocess.run([r'C:\Program Files\LLVM\bin\clang.exe','-std=c11','-Werror','-fsyntax-only',
                        '-I'+str(ROOT/'esp32_probe/main'),*flags,str(source)],capture_output=True,text=True)
    assert (run.returncode==0)==valid,run.stdout+run.stderr

tree=ast.parse((ROOT/'tools/validate_local_image.py').read_text())
start=next(i for i,n in enumerate(tree.body) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='priority_order' for t in n.targets))
end=next(i for i,n in enumerate(tree.body) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='owner_ready_yield' for t in n.targets))
actual=compile(ast.Module(body=tree.body[start:end],type_ignores=[]),'<actual ordered priority admission>','exec')
units=['probe_main.c','probe_audio.c','probe_output.c']
image_cases=0
def check(change,valid):
    global image_cases
    state={'build_status':{'priority_order':True,'host_priority':7,'renderer_priority':5,'async_audio':True},
           'cache':'DBT_PROBE_PRIORITY_ORDER:BOOL=ON','decoder_priority':6,'diagnostic':False,'pipeline':False,
           'entry_name':'probe_main.c','commands':[{'file':n,'command':'cc -DDBT_PROBE_PRIORITY_ORDER=1'} for n in units],
           'Path':pathlib.Path,'re':re}
    state.update(change)
    try:exec(actual,state)
    except (AssertionError,StopIteration):assert not valid
    else:assert valid
    image_cases+=1
check({},True)
for field in ('host_priority','renderer_priority'):
    values={'priority_order':True,'host_priority':7,'renderer_priority':5,'async_audio':True}
    check({'build_status':values|{field:True}},False)
    check({'build_status':values|{field:6}},False)
check({'decoder_priority':5},False)
check({'diagnostic':True},False)
check({'pipeline':True},False)
check({'cache':'DBT_PROBE_PRIORITY_ORDER:BOOL=ON\nDBT_PROBE_KEEP_OWNED_ACTIVE:BOOL=ON'},False)
check({'cache':''},False)
check({'commands':[]},False)
for command in ('cc','cc -DDBT_PROBE_PRIORITY_ORDER=0','cc -DDBT_PROBE_PRIORITY_ORDER=1 -UDBT_PROBE_PRIORITY_ORDER',
                'cc -DDBT_PROBE_PRIORITY_ORDER=1 -DDBT_PROBE_PRIORITY_ORDER=0'):
    check({'commands':[{'file':n,'command':command} for n in units]},False)
check({'build_status':{},'cache':'','commands':[{'file':n,'command':'cc'} for n in units]},True)
(OUT/'results.json').write_text(json.dumps({'passed':True,'cmake_cases':len(cmake_cases),'compiled_header_cases':len(header_cases),
                                         'actual_image_cases':image_cases,'hardware_accessed':False},indent=2)+'\n')
print(f'PASS priority order: {len(cmake_cases)} CMake, {len(header_cases)} compiled header and {image_cases} actual image cases')
