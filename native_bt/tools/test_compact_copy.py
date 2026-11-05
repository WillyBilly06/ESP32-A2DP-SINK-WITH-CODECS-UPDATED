"""Sanitize actual compact copied provider with scalar and block DSP."""
import hashlib,json,os,pathlib,shutil,subprocess
ROOT=pathlib.Path(__file__).resolve().parents[1];OUT=ROOT/'_build/compact_copy';OUT.mkdir(exist_ok=True);DSP=ROOT.parent/'bt_audio_sink/main/dsp'
env=os.environ.copy()
if os.name=='nt':
    vc=pathlib.Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat');data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True);env.update(dict(x.split('=',1) for x in data.splitlines() if '=' in x and not x.startswith('=')))
cc=shutil.which('clang',path=env.get('PATH')) or r'C:\Program Files\LLVM\bin\clang.exe';cache=shutil.which('ccache',path=env.get('PATH'))
if not cache:
    matches=sorted((pathlib.Path.home()/'.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'))
    if matches:cache=str(matches[-1])
assert cache,'ccache required';env.pop('CCACHE_DISABLE',None)
main=OUT/'main.c';main.write_text('#include <stdio.h>\nint probe_output_compact_copy(void);int probe_output_descriptor_diagnostics(void);\nint main(void){int a=probe_output_compact_copy(),b=probe_output_descriptor_diagnostics();printf("compact copied provider=%d descriptor diagnostics=%d\\n",a,b);return a||b;}\n')
sources=[ROOT/'tests/codecs/probe_output_format.c',ROOT/'src/dbt_audio.c',ROOT/'src/dbt_resampler.c',DSP/'fixed_dsp.c',main]
tracked=sources+[ROOT/'esp32_probe/main/probe_output.c',ROOT/'esp32_probe/main/probe_output.h',pathlib.Path(__file__).resolve(),ROOT/'src/dbt_resampler_coeffs.inc']+list((ROOT/'include').glob('*.h'))+list(DSP.glob('fixed_dsp*'))+list((ROOT/'tests/codecs/probe_audio_stubs').rglob('*.h'));before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked};commands=[]
if os.name=='nt':
    resource=pathlib.Path(subprocess.check_output([cc,'-print-resource-dir'],text=True).strip());shutil.copy2(resource/'lib/windows/clang_rt.asan_dynamic-x86_64.dll',OUT/'clang_rt.asan_dynamic-x86_64.dll')
for block in [False,True]:
    exe=OUT/('compact-'+str(int(block))+('.exe' if os.name=='nt' else ''));cmd=[cache,cc,'-std=c11','-O3','-g','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer','-DDBT_PROBE_PCM_COPIED=1','-DDBT_PROBE_ASYNC_AUDIO=1','-DDBT_PROBE_PCM_TARGET_MS=16','-DCONFIG_I2S_ISR_IRAM_SAFE=1','-DCONFIG_NATIVE_AUDIO_BCLK=26','-DCONFIG_NATIVE_AUDIO_LRCK=25','-DCONFIG_NATIVE_AUDIO_DATA=27']
    if block:cmd+=['-DDBT_AUDIO_BLOCK_DSP=1','-DFIXED_DSP_BLOCK_API=1','-DDBT_PROBE_BLOCK_DSP=1']
    cmd+=['-I'+str(p) for p in [ROOT/'tests/codecs/probe_audio_stubs',ROOT/'include',ROOT/'esp32_probe/main',DSP]]+list(map(str,sources))+['-o',str(exe)]
    if os.name=='nt':cmd+=['-fms-runtime-lib=dll','-D_DLL','-Wl,/NODEFAULTLIB:libcmt']
    subprocess.run(cmd,check=True,env=env);subprocess.run([str(exe)],check=True,env=env,timeout=60);commands.append(cmd)
assert all(hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()==h for p,h in before.items())
(OUT/'results.json').write_text(json.dumps({'passed':True,'scalar_and_block':True,'geometry_cases_per_mode':24,'source_reuse_after_full_frame_copy':True,'busy_is_whole_block':True,'decoder_wakeup_after_input_consumption':True,'sanitizers':['address','undefined'],'signed_overflow_wrapping':False,'sources':before,'commands':commands,'hardware_accessed':False},indent=2)+'\n')
