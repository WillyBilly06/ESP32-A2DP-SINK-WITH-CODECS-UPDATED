"""Verify exact limiter curve over every clipped magnitude, plus sanitized state edges.
Host timing is not ESP32 performance evidence. Compilation uses ccache.
"""
import hashlib,json,os,pathlib,re,shutil,subprocess
ROOT=pathlib.Path(__file__).resolve().parents[1];DSP=ROOT.parent/'bt_audio_sink/main/dsp';OUT=ROOT/'_build/dsp_limiter_word';OUT.mkdir(exist_ok=True)
env=os.environ.copy()
vc=pathlib.Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
values=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
env.update(dict(s.split('=',1) for s in values.splitlines() if '=' in s and not s.startswith('=')))
cc=shutil.which('clang',path=env.get('PATH')) or r'C:\Program Files\LLVM\bin\clang.exe'
cache=shutil.which('ccache',path=env.get('PATH'))
if not cache:
 matches=sorted((pathlib.Path.home()/'.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'));cache=str(matches[-1]) if matches else None
assert cache,'ccache required';env.pop('CCACHE_DISABLE',None)
source=ROOT/'tests/test_dsp_limiter_word.c';inputs=[source,DSP/'fixed_dsp.c',DSP/'fixed_dsp.h',DSP/'fixed_dsp_tables.inc',pathlib.Path(__file__).resolve()]
before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs};commands=[];runs=[]
for sanitize in (False,True):
 name='sanitized' if sanitize else 'exhaustive';obj=OUT/(name+'.obj');exe=OUT/(name+'.exe')
 flags=[cc,'-std=c11','-O3','-Wall','-Wextra','-Werror','-I'+str(DSP)]
 if sanitize:flags+=['-g','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer','-fms-runtime-lib=dll','-D_DLL']
 command=[cache,*flags,'-c',str(source),'-o',str(obj)];subprocess.run(command,check=True,env=env);commands.append(command)
 command=[*flags,str(obj),'-o',str(exe)]
 if sanitize:command+=['-Wl,/NODEFAULTLIB:libcmt']
 subprocess.run(command,check=True,env=env);commands.append(command)
 if sanitize:
  resource=pathlib.Path(subprocess.check_output([cc,'-print-resource-dir'],text=True).strip());runtime=resource/'lib/windows/clang_rt.asan_dynamic-x86_64.dll'
  if not runtime.is_file():runtime=pathlib.Path(cc).parent/runtime.name
  assert runtime.is_file();shutil.copy2(runtime,OUT/runtime.name)
 result=subprocess.run([str(exe)]+(['--quick'] if sanitize else []),capture_output=True,text=True,env=env,timeout=120)
 (OUT/(name+'.stdout.txt')).write_text(result.stdout);(OUT/(name+'.stderr.txt')).write_text(result.stderr)
 print(result.stdout,flush=True);print(result.stderr,flush=True);result.check_returncode()
 m=re.search(r'PASS (\d+) exhaustive limiter magnitudes; (\d+) signed full-state cases',result.stdout);assert m
 runs.append(dict(sanitized=sanitize,exhaustive_magnitudes=int(m[1]),signed_state_cases=int(m[2]),binary_sha256=hashlib.sha256(exe.read_bytes()).hexdigest()))
assert runs[0]['exhaustive_magnitudes']==2033398579
assert all(hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()==h for p,h in before.items())
(OUT/'results.json').write_text(json.dumps(dict(passed=True,scope='host exact integer curve and full limiter state; no target timing',runs=runs,commands=commands,source_sha256=before),indent=2)+'\n')
