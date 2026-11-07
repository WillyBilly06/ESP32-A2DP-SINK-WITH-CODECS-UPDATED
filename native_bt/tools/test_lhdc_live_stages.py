"""Actual live-stage producer/snapshot and logger routing under ASan/UBSan."""
import hashlib,json,os,pathlib,re,shutil,subprocess
R=pathlib.Path(__file__).resolve().parents[1];O=R/'_build/lhdc_live_stages';O.mkdir(exist_ok=True)
src=(R/'esp32_probe/main/probe_audio.c').read_text();m=re.search(r'static void log_lhdc_live\(uint32_t cycle\)\{.*?\n\}',src,re.S);assert m;(O/'live_stage_logger.h').write_text(m[0]+'\n')
env=os.environ.copy();vc=pathlib.Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat');values=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True);env.update(dict(s.split('=',1) for s in values.splitlines() if '=' in s and not s.startswith('=')))
cc=shutil.which('clang',path=env.get('PATH')) or r'C:\Program Files\LLVM\bin\clang.exe';cache=shutil.which('ccache',path=env.get('PATH')) or str(sorted((pathlib.Path.home()/'.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'))[-1]);env.pop('CCACHE_DISABLE',None)
source=R/'tests/codecs/test_lhdc_live_stages.c';inputs=[source,pathlib.Path(__file__).resolve(),R/'esp32_probe/main/probe_audio.c',R/'esp32_probe/main/probe_lhdc_profile.c',R/'esp32_probe/main/probe_lhdc_profile.h',O/'live_stage_logger.h'];before={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in inputs}
flags=[cc,'-std=c11','-O2','-g','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer','-fms-runtime-lib=dll','-D_DLL','-I'+str(O),'-I'+str(R/'esp32_probe/main')];obj=O/'stages.obj';exe=O/'stages.exe'
subprocess.run([cache,*flags,'-c',str(source),'-o',str(obj)],check=True,env=env);subprocess.run([*flags,str(obj),'-o',str(exe),'-Wl,/NODEFAULTLIB:libcmt'],check=True,env=env)
res=pathlib.Path(subprocess.check_output([cc,'-print-resource-dir'],text=True).strip());rt=res/'lib/windows/clang_rt.asan_dynamic-x86_64.dll'
if not rt.is_file():rt=pathlib.Path(cc).parent/rt.name
assert rt.is_file();shutil.copy2(rt,O/rt.name)
r=subprocess.run([str(exe)],capture_output=True,text=True,env=env,timeout=20);(O/'stdout.txt').write_text(r.stdout);(O/'stderr.txt').write_text(r.stderr);print(r.stdout);print(r.stderr);r.check_returncode()
event=json.loads(next(l for l in r.stdout.splitlines() if l.startswith('{')));assert event['probe']=='lhdc_live_stages' and event['rate']==192000 and event['channel_calls']==32 and event['sum_cycles'][0]==4294967264 and event['sum_cycles'][1]==64
assert event['frame_calls']==16 and event['frame_sum_cycles']==[112,128,144,160] and event['frame_stages']==['prepare','channel_envelope','scale_scan','output_tail']
assert all(hashlib.sha256(pathlib.Path(f).read_bytes()).hexdigest()==h for f,h in before.items());(O/'results.json').write_text(json.dumps(dict(passed=True,scope='actual collector and extracted logger; bounded snapshot/epochs/wrap/finite compatibility; no target timing',event=event,source_sha256=before),indent=2)+'\n')

cm=(R/'esp32_probe/CMakeLists.txt').read_text();gate=re.search(r'^if\(DBT_PROBE_LHDC_STAGE_PROFILE AND NOT DBT_PROBE_LHDC_MAX_CHECK[^\n]*\)\n.*?endif\(\)',cm,re.M|re.S)[0];(O/'gate.cmake').write_text(gate+'\n')
base={'LHDC_STAGE_PROFILE':'ON','LHDC_MAX_CHECK':'OFF','STAY_NATIVE':'ON','LHDC_BUILD':'ON','ASYNC_AUDIO':'ON','AUDIO_OUTPUT':'ON','AUDIO_STAGE_PROFILE':'ON'}
for change,ok in [({},True),({'STAY_NATIVE':'OFF'},False),({'LHDC_BUILD':'OFF'},False),({'AUDIO_STAGE_PROFILE':'OFF'},False),({'LHDC_MAX_CHECK':'ON','STAY_NATIVE':'OFF'},True),({'LHDC_STAGE_PROFILE':'OFF','STAY_NATIVE':'OFF'},True)]:
 c=subprocess.run(['cmake',*[f'-DDBT_PROBE_{k}={v}' for k,v in (base|change).items()],'-P',str(O/'gate.cmake')],capture_output=True,text=True);assert (c.returncode==0)==ok,c.stdout+c.stderr
print('PASS6 actual stage-mode admission cases')
