"""Sanitize the actual PCM-loan provider/render loop with deterministic SDK stubs."""
import hashlib,json,os,pathlib,shutil,subprocess
ROOT=pathlib.Path(__file__).resolve().parents[1];OUT=ROOT/'_build/output_loan';OUT.mkdir(exist_ok=True)
report=OUT/'results.json';report.write_text(json.dumps({'passed':False,'state':'building','hardware_accessed':False})+'\n')
env=os.environ.copy()
if os.name=='nt':
    vc=pathlib.Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
    data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
    env.update(dict(x.split('=',1) for x in data.splitlines() if '=' in x and not x.startswith('=')))
cc=shutil.which('clang',path=env.get('PATH')) or r'C:\Program Files\LLVM\bin\clang.exe'
cache=shutil.which('ccache',path=env.get('PATH'))
if not cache:
    matches=sorted((pathlib.Path.home()/'.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'))
    if matches:cache=str(matches[-1])
assert cache,'ccache required';env.pop('CCACHE_DISABLE',None)
sources=[ROOT/'tests/codecs/probe_output_loan.c',ROOT/'src/dbt_audio.c',ROOT/'src/dbt_resampler.c',ROOT.parent/'bt_audio_sink/main/dsp/fixed_dsp.c']
tracked=sources+[ROOT/'esp32_probe/main/probe_output.c',ROOT/'esp32_probe/main/probe_output.h',ROOT/'src/dbt_resampler_coeffs.inc',pathlib.Path(__file__).resolve()]+list((ROOT/'include').glob('*.h'))+list((ROOT/'tests/codecs/probe_audio_stubs').rglob('*.h'))+list((ROOT.parent/'bt_audio_sink/main/dsp').glob('fixed_dsp*'))
before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked}
inc=[ROOT/'tests/codecs/probe_audio_stubs',ROOT/'include',ROOT/'esp32_probe/main',ROOT.parent/'bt_audio_sink/main/dsp']
exe=OUT/('output-loan.exe' if os.name=='nt' else 'output-loan')
cmd=[cache,cc,'-std=c11','-O3','-g','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer','-DDBT_PROBE_PCM_LOANS=1','-DDBT_PROBE_ASYNC_AUDIO=1','-DDBT_OUTPUT_TEST_HOOKS=1','-DCONFIG_I2S_ISR_IRAM_SAFE=1','-DCONFIG_NATIVE_AUDIO_BCLK=26','-DCONFIG_NATIVE_AUDIO_LRCK=25','-DCONFIG_NATIVE_AUDIO_DATA=27']
cmd+=['-I'+str(p) for p in inc]+list(map(str,sources))+['-o',str(exe)]
if os.name=='nt':cmd+=['-fms-runtime-lib=dll','-D_DLL','-Wl,/NODEFAULTLIB:libcmt']
if os.name=='nt':
    resource=pathlib.Path(subprocess.check_output([cc,'-print-resource-dir'],text=True).strip());shutil.copy2(resource/'lib/windows/clang_rt.asan_dynamic-x86_64.dll',OUT/'clang_rt.asan_dynamic-x86_64.dll')
commands=[]
for target in (64,32,16):
    selected=OUT/(f'output-loan-{target}.exe' if os.name=='nt' else f'output-loan-{target}')
    selected_command=cmd.copy();selected_command[selected_command.index('-o')+1]=str(selected)
    selected_command.insert(2,'-DDBT_PROBE_PCM_TARGET_MS='+str(target))
    subprocess.run(selected_command,check=True,env=env)
    subprocess.run([str(selected)],check=True,env=env,timeout=60);commands.append(selected_command)
assert all(hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()==digest for p,digest in before.items())
report.write_text(json.dumps({'passed':True,'pcm_targets_ms':[64,32,16],'sanitizers':['address','undefined'],'signed_overflow_wrapping':False,'sources':before,'commands':commands,'scope':'actual provider/render ownership, target geometry and delay reporting with deterministic queues and mocked I2S; no hardware timing/audio qualification'},indent=2)+'\n')
