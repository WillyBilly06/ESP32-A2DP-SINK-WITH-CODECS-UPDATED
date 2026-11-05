"""Actual optional PCM-loan core versus copied-ring reference; host ASan/UBSan."""
import hashlib,json,os,pathlib,shutil,subprocess
ROOT=pathlib.Path(__file__).resolve().parents[1];OUT=ROOT/'_build/audio_borrowed';OUT.mkdir(exist_ok=True)
(OUT/'results.json').write_text(json.dumps({'passed':False,'state':'building','hardware_accessed':False})+'\n')
env=os.environ.copy()
if os.name=='nt':
    vc=pathlib.Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
    data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
    env.update(dict(x.split('=',1) for x in data.splitlines() if '=' in x and not x.startswith('=')))
cc=shutil.which('clang',path=env.get('PATH')) or r'C:\Program Files\LLVM\bin\clang.exe'
cache=shutil.which('ccache',path=env.get('PATH'))
if not cache:
    candidates=sorted((pathlib.Path.home()/'.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'))
    if candidates:cache=str(candidates[-1])
assert cache,'ccache required';env.pop('CCACHE_DISABLE',None)
sources=[ROOT/'tests/audio/borrowed_input.c',ROOT/'src/dbt_audio.c',ROOT/'src/dbt_resampler.c',ROOT.parent/'bt_audio_sink/main/dsp/fixed_dsp.c']
tracked=sources+list((ROOT/'include').glob('*.h'))+list((ROOT.parent/'bt_audio_sink/main/dsp').glob('fixed_dsp*'))+[ROOT/'src/dbt_resampler_coeffs.inc',pathlib.Path(__file__).resolve()]
before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked}
exe=OUT/('borrowed.exe' if os.name=='nt' else 'borrowed')
cmd=[cache,cc,'-std=c11','-O3','-g','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer','-I'+str(ROOT/'include'),'-I'+str(ROOT.parent/'bt_audio_sink/main/dsp'),*map(str,sources),'-o',str(exe)]
if os.name=='nt':cmd+=['-fms-runtime-lib=dll','-D_DLL','-Wl,/NODEFAULTLIB:libcmt']
subprocess.run(cmd,check=True,env=env)
if os.name=='nt':
    resource=pathlib.Path(subprocess.check_output([cc,'-print-resource-dir'],text=True).strip());shutil.copy2(resource/'lib/windows/clang_rt.asan_dynamic-x86_64.dll',OUT/'clang_rt.asan_dynamic-x86_64.dll')
subprocess.run([str(exe)],check=True,env=env,timeout=60)
assert all(hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()==digest for p,digest in before.items())
(OUT/'results.json').write_text(json.dumps({'passed':True,'case_pairs':108,'input_frames_per_case':4096,'adaptive_and_linear_modes':True,'linear_fractional_final_lookahead_matches_legacy':True,'sanitizers':['address','undefined'],'signed_overflow_wrapping':False,'sources':before,'command':cmd,'scope':'host optional audio-core loan lifetime/math; no worker/provider integration or device qualification'},indent=2)+'\n')
