"""Exercise actual fair-sharing header, CMake and image admission. No hardware."""
import ast,json,pathlib,re,subprocess
ROOT=pathlib.Path(__file__).resolve().parents[1];OUT=ROOT/'_build/render_fair_share';OUT.mkdir(exist_ok=True)
header=OUT/'limits.c'
header.write_text('#include "probe_schedule_limits.h"\n_Static_assert(DBT_PROBE_HOST_PRIORITY==22,"host");\n_Static_assert(DBT_PROBE_DECODER_PRIORITY==21,"decoder");\n_Static_assert(DBT_PROBE_RENDER_PRIORITY==20,"helper");\n_Static_assert(DBT_PROBE_RENDER_CORE1_PRIORITY==22,"main");\n_Static_assert(DBT_PROBE_OWNER_READY_YIELD==1,"yield");\n')
flags=['DBT_PROBE_STAY_NATIVE=1','DBT_PROBE_DECODER_PRIORITY=21','DBT_PROBE_CODEC_AWARE_RENDER=1','DBT_PROBE_RENDER_FAIR_SHARE=1']
hcases=[(flags,True),([x for x in flags if not x.startswith('DBT_PROBE_STAY_NATIVE')],False),([x for x in flags if not x.startswith('DBT_PROBE_CODEC_AWARE_RENDER')],False),(flags+['DBT_PROBE_RENDER_CORE1_PRIORITY=22'],False)]
for defines,valid in hcases:
    r=subprocess.run([r'C:/Program Files/LLVM/bin/clang.exe','-std=c11','-Werror','-fsyntax-only','-I'+str(ROOT/'esp32_probe/main'),'-I'+str(ROOT/'include'),*['-D'+d for d in defines],str(header)],capture_output=True,text=True)
    assert (r.returncode==0)==valid,r.stdout+r.stderr
guard=OUT/'guard.cmake';guard.write_text((ROOT/'esp32_probe/CMakeLists.txt').read_text().split('set(COMPONENTS main)')[0])
base={k:'ON' for k in ['RENDER_FAIR_SHARE','STAY_NATIVE','PRIORITY_ORDER','CODEC_AWARE_RENDER','ASYNC_AUDIO','PCM_COPIED','KEEP_OWNED_ACTIVE','PRODUCT_IO','PRODUCT_SECURITY','AUDIO_TEST','AUDIO_OUTPUT','SBC_BUILD','CLASSIC_SECURITY_TEST','SMP','GATT_TEST','AVRCP']};base['DECODER_PRIORITY']='21'
ccases=[({},True)]+[({k:'OFF'},False) for k in ['STAY_NATIVE','PRIORITY_ORDER','CODEC_AWARE_RENDER','ASYNC_AUDIO','PCM_COPIED']]+[({'OWNER_PACER':'ON'},False)]
for changes,valid in ccases:
    r=subprocess.run(['cmake',*[f'-DDBT_PROBE_{k}={v}' for k,v in (base|changes).items()],'-P',str(guard)],capture_output=True,text=True)
    assert (r.returncode==0)==valid,r.stdout+r.stderr
tree=ast.parse((ROOT/'tools/validate_local_image.py').read_text())
def assignment(name):return next(i for i,n in enumerate(tree.body) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id==name for t in n.targets))
actual=compile(ast.Module(body=tree.body[assignment('render_fair_share'):assignment('logger_core')],type_ignores=[]),'<actual sharing admission>','exec')
receipt={'stay_native':True,'render_fair_share':True,'priority_order':True,'codec_aware_render':True,'async_audio':True,'pcm_copied':True,'auxiliary_renderer_priority':20,'host_priority':22,'renderer_priority':22,'owner_ready_yield':True}
cache='DBT_PROBE_RENDER_FAIR_SHARE:BOOL=ON\nDBT_PROBE_PRIORITY_ORDER:BOOL=ON\nDBT_PROBE_STAY_NATIVE:BOOL=ON\nDBT_PROBE_KEEP_OWNED_ACTIVE:BOOL=ON'
units=['probe_main.c','probe_audio.c','probe_output.c'];commands=[{'file':n,'command':'cc -DDBT_PROBE_PRIORITY_ORDER=1 -DDBT_PROBE_RENDER_FAIR_SHARE=1'} for n in units]
count=0
def check(changes,valid,rc=None):
    global count
    state={'build_status':receipt|(rc or {}),'cache':cache,'commands':commands,'entry_name':'probe_main.c','main_command':commands[0]['command'],'stay_native':True,'decoder_priority':21,'diagnostic':False,'pipeline':False,'Path':pathlib.Path,'re':re};state.update(changes)
    try:exec(actual,state)
    except (AssertionError,StopIteration):assert not valid
    else:assert valid
    count+=1
check({},True)
check({'cache':cache.replace('DBT_PROBE_RENDER_FAIR_SHARE:BOOL=ON','DBT_PROBE_RENDER_FAIR_SHARE:BOOL=OFF'),'commands':[{'file':n,'command':'cc -DDBT_PROBE_PRIORITY_ORDER=1'} for n in units]},True,{'render_fair_share':False,'renderer_priority':20,'owner_ready_yield':False})
for rc in [{'renderer_priority':20},{'host_priority':21},{'auxiliary_renderer_priority':22},{'owner_ready_yield':False},{'owner_pacer':True},{'render_fair_share':1},{'pcm_copied':False},{'priority_order':False},{'auxiliary_renderer_priority':True}]:check({},False,rc)
check({'cache':cache.replace('DBT_PROBE_RENDER_FAIR_SHARE:BOOL=ON','')},False)
check({'commands':commands[:-1]},False)
for c in ['cc -DDBT_PROBE_PRIORITY_ORDER=1','cc -DDBT_PROBE_PRIORITY_ORDER=1 -DDBT_PROBE_RENDER_FAIR_SHARE=0','cc -DDBT_PROBE_PRIORITY_ORDER=1 -DDBT_PROBE_RENDER_FAIR_SHARE=1 -UDBT_PROBE_RENDER_FAIR_SHARE']:
    check({'commands':[{'file':n,'command':c} for n in units]},False)
(OUT/'results.json').write_text(json.dumps({'passed':True,'compiled_header_cases':len(hcases),'actual_cmake_cases':len(ccases),'actual_image_cases':count,'hardware_accessed':False},indent=2)+'\n')
print(f'PASS fair sharing: {len(hcases)} headers, {len(ccases)} CMake, {count} image admissions')
