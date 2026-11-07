"""Exercise actual helper batch admission; no device or SDK build."""
import ast, json, pathlib, re, subprocess, sys
ROOT=pathlib.Path(__file__).resolve().parents[1]
OUT=ROOT/'_build/dsp_assist_guard';OUT.mkdir(exist_ok=True)
(OUT/'results.json').write_text(json.dumps({'passed':False,'hardware_accessed':False})+'\n')
guard=OUT/'guard.cmake'
guard.write_text((ROOT/'esp32_probe/CMakeLists.txt').read_text().split('set(COMPONENTS main)')[0])
deps=['STAY_NATIVE','CODEC_AWARE_RENDER','ASYNC_AUDIO','PCM_COPIED','BLOCK_DSP','UNITY_PCM_COPY']
base={k:'ON' for k in deps+['DSP_ASSIST','LHDC_BUILD','KEEP_OWNED_ACTIVE','PRODUCT_IO','PRODUCT_SECURITY','AUDIO_TEST','AUDIO_OUTPUT','SBC_BUILD','CLASSIC_SECURITY_TEST','SMP','GATT_TEST','AVRCP']}
base|={'DECODER_PRIORITY':'21','DSP_ASSIST_BLOCKS':'5'}
cmake_cases=[({},True),({'DSP_ASSIST_BLOCKS':'4'},True),({'DSP_ASSIST':'OFF','DSP_ASSIST_BLOCKS':'4'},True),({'DSP_ASSIST':'OFF'},False)]
cmake_cases += [({'DSP_ASSIST_BLOCKS':v},False) for v in ['0','2','3','6','50','4;5','5.0']]
cmake_cases += [({k:'OFF'},False) for k in deps+['LHDC_BUILD']]
cmake_cases += [({'LHDC_CORE0_RENDER':'ON'},False),({'APTX_SNAPSHOT':'ON'},False)]
for changes,valid in cmake_cases:
    result=subprocess.run(['cmake',*[f'-DDBT_PROBE_{k}={v}' for k,v in (base|changes).items()],'-P',str(guard)],capture_output=True,text=True)
    assert (result.returncode==0)==valid,result.stdout+result.stderr
tree=ast.parse((ROOT/'tools/validate_local_image.py').read_text())
def at(name):
    return next(i for i,n in enumerate(tree.body) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id==name for t in n.targets))
actual=compile(ast.Module(body=tree.body[at('dsp_assist'):at('report')],type_ignores=[]),'<actual helper admission>','exec')
status={'dsp_assist':True,'dsp_assist_blocks':5}|{k.lower():True for k in deps}
cache='\n'.join(['DBT_PROBE_DSP_ASSIST:BOOL=ON','DBT_PROBE_DSP_ASSIST_BLOCKS:STRING=5','DBT_PROBE_LHDC_BUILD:BOOL=ON']+['DBT_PROBE_'+k+':BOOL=ON' for k in deps])
units=['probe_main.c','probe_output.c','probe_audio.c']
macro='cc -DDBT_PROBE_DSP_ASSIST=1 -DDBT_PROBE_DSP_ASSIST_BLOCKS=5'
commands=[{'file':n,'command':macro} for n in units]
image_count=0
def check(changes,valid):
    global image_count
    state={'build_status':dict(status),'cache':cache,'commands':[dict(e) for e in commands],'entry_name':'probe_main.c','Path':pathlib.Path,'re':re,'diagnostic':False,'pipeline':False}
    state.update(changes)
    try:exec(actual,state)
    except AssertionError:assert not valid
    else:assert valid
    image_count+=1
check({},True)
check({'build_status':status|{'dsp_assist_blocks':4},'cache':cache.replace('STRING=5','STRING=4'),'commands':[e|{'command':macro.replace('BLOCKS=5','BLOCKS=4')} for e in commands]},True)
off={'build_status':{'dsp_assist':False,'dsp_assist_blocks':4},'cache':'DBT_PROBE_DSP_ASSIST_BLOCKS:STRING=4','commands':[{'file':n,'command':'cc'} for n in units]}
check(off,True)
for bad in [True,False,'5',5.0,0,2,3,6]:check({'build_status':status|{'dsp_assist_blocks':bad}},False)
check({'build_status':status|{'dsp_assist':1}},False)
check({'cache':cache.replace('STRING=5','STRING=4')},False)
check({'cache':cache.replace('DBT_PROBE_DSP_ASSIST:BOOL=ON','')},False)
check(off|{'build_status':off['build_status']|{'dsp_assist_blocks':5},'cache':'DBT_PROBE_DSP_ASSIST_BLOCKS:STRING=5'},False)
for k in deps:
    check({'build_status':status|{k.lower():False}},False)
    check({'cache':cache.replace('DBT_PROBE_'+k+':BOOL=ON','')},False)
for key in ['lhdc_core0_render','aptx_snapshot']:check({'build_status':status|{key:True}},False)
check({'cache':cache.replace('DBT_PROBE_LHDC_BUILD:BOOL=ON','')},False)
check({'diagnostic':True},False);check({'pipeline':True},False)
for i in range(len(commands)):
    for bad in ['cc',macro.replace('BLOCKS=5','BLOCKS=4'),macro+' -UDBT_PROBE_DSP_ASSIST_BLOCKS',macro+' -DDBT_PROBE_DSP_ASSIST_BLOCKS=5',macro+' -UDBT_PROBE_DSP_ASSIST']:
        changed=[dict(e) for e in commands];changed[i]['command']=bad;check({'commands':changed},False)
    check({'commands':commands[:i]+commands[i+1:]},False)
check(off|{'commands':commands},False)
fake=OUT/'header.c';fake.write_text('#include "probe_output.h"\nint main(void){return 0;}\n')
cc=r'C:/Program Files/LLVM/bin/clang.exe'
includes=[ROOT/'tests/codecs/probe_audio_stubs',ROOT/'include',ROOT/'esp32_probe/main',ROOT.parent/'bt_audio_sink/main/dsp']
header_cases=[([],True),(['-DDBT_PROBE_DSP_ASSIST=1'],True),(['-DDBT_PROBE_DSP_ASSIST=1','-DDBT_PROBE_DSP_ASSIST_BLOCKS=4'],True),(['-DDBT_PROBE_DSP_ASSIST=1','-DDBT_PROBE_DSP_ASSIST_BLOCKS=5'],True),(['-DDBT_PROBE_DSP_ASSIST_BLOCKS=5'],False)]
header_cases += [(['-DDBT_PROBE_DSP_ASSIST=1','-DDBT_PROBE_DSP_ASSIST_BLOCKS='+str(v)],False) for v in [0,2,3,6]]
for defines,valid in header_cases:
    result=subprocess.run([cc,'-std=c11','-Werror','-fsyntax-only',*['-I'+str(p) for p in includes],*defines,str(fake)],capture_output=True,text=True)
    assert (result.returncode==0)==valid,result.stdout+result.stderr
cli_count=0
if sys.platform=='win32':
    for value,needle in [('5','Nondefault DSP batch requires'),('6','DspAssistBlocks')]:
        result=subprocess.run(['powershell','-NoProfile','-ExecutionPolicy','Bypass','-File',str(ROOT/'tools/build_probe.ps1'),'-DspAssistBlocks',value],capture_output=True,text=True)
        assert result.returncode and needle in result.stderr,result.stdout+result.stderr
        cli_count+=1
(OUT/'results.json').write_text(json.dumps({'passed':True,'cmake_cases':len(cmake_cases),'actual_image_cases':image_count,'header_cases':len(header_cases),'cli_cases':cli_count,'hardware_accessed':False,'scope':'actual CMake, image gate, compiled header and early CLI rejection; no runtime qualification'},indent=2)+'\n')
print(f'PASS helper batch admission: {len(cmake_cases)} CMake,{image_count} actual image,{len(header_cases)} header,{cli_count} CLI cases; no hardware')
