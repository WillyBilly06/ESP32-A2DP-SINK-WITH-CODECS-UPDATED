"""Run actual DSP-assisted output lifecycle against two mocked FreeRTOS cores. No hardware."""
import argparse,hashlib,json,os,pathlib,shutil,subprocess
parser=argparse.ArgumentParser();parser.add_argument("--fair-share",action="store_true");parser.add_argument("--early-input",action="store_true");parser.add_argument("--rebalance",action="store_true");options=parser.parse_args()
if options.rebalance and not options.early_input:parser.error("rebalance requires early input")
ROOT=pathlib.Path(__file__).resolve().parents[1];WS=ROOT.parent;OUT=ROOT/'_build/dsp_assist_output';OUT.mkdir(exist_ok=True)
(OUT/'results.json').write_text(json.dumps({'passed':False,'state':'building','hardware_accessed':False})+'\n')
env=os.environ.copy();vc=pathlib.Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
values=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
env.update(dict(x.split('=',1) for x in values.splitlines() if '=' in x and not x.startswith('=')))
cc=shutil.which('clang',path=env.get('PATH')) or r'C:\Program Files\LLVM\bin\clang.exe'
cache=shutil.which('ccache',path=env.get('PATH'))
if not cache:
    matches=sorted((pathlib.Path.home()/'.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'))
    if matches:cache=str(matches[-1])
assert cache,'ccache required';env.pop('CCACHE_DISABLE',None)
sources=[ROOT/'tests/sound/test_dsp_assist_output.c',ROOT/'esp32_probe/main/probe_output.c',ROOT/'esp32_probe/main/probe_sound.c',ROOT/'src/dbt_audio.c',ROOT/'src/dbt_dsp_job.c',ROOT/'src/dbt_resampler.c',ROOT/'src/dbt_sound.c',ROOT/'src/dbt_sound_mix.c',WS/'bt_audio_sink/main/dsp/fixed_dsp.c']
tracked=sources+[ROOT/'tests/sound/test_probe_sound_output.c',ROOT/'tests/sound/dsp_assist_mock.h',pathlib.Path(__file__).resolve()]+list((ROOT/'include').glob('*.h'))+list((ROOT/'esp32_probe/main').glob('*.h'))+list((ROOT/'tests/codecs/probe_audio_stubs').rglob('*.h'))+list((WS/'bt_audio_sink/main/dsp').glob('fixed_dsp*'))+[ROOT/'src/dbt_resampler_coeffs.inc']
before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked}
includes=[ROOT/'tests/codecs/probe_audio_stubs',ROOT/'include',ROOT/'esp32_probe/main',WS/'bt_audio_sink/main/dsp']
defines=['DBT_PROBE_DSP_ASSIST','DBT_PROBE_CODEC_AWARE_RENDER','DBT_PROBE_ASYNC_AUDIO','DBT_PROBE_PCM_COPIED','DBT_PROBE_STAY_NATIVE','DBT_PROBE_WORD_DSP','DBT_PROBE_BLOCK_DSP','DBT_AUDIO_UNITY_COPY','DBT_AUDIO_BLOCK_DSP','FIXED_DSP_BLOCK_API','FIXED_DSP_WORD_QUANT','DBT_PROBE_SOUND_BUILD','DBT_OUTPUT_TEST_HOOKS','CONFIG_I2S_ISR_IRAM_SAFE']
if options.fair_share:defines+=['DBT_PROBE_RENDER_FAIR_SHARE']
if options.early_input:defines+=['DBT_PROBE_EARLY_INPUT_RELEASE']
if options.rebalance:defines+=['DBT_PROBE_HIGH_RATE_PCM_REBALANCE']
base=[cache,cc,'-std=c11','-O1','-g','-fwrapv','-Wall','-Wextra','-Werror','-Wno-unused-function','-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer','-fms-runtime-lib=dll','-D_DLL','-DDBT_PROBE_PCM_TARGET_MS=16','-DDBT_PROBE_DECODER_PRIORITY=21','-DCONFIG_NATIVE_AUDIO_BCLK=27','-DCONFIG_NATIVE_AUDIO_LRCK=25','-DCONFIG_NATIVE_AUDIO_DATA=26','-include',str(ROOT/'tests/sound/dsp_assist_mock.h')]+['-D'+d+'=1' for d in defines]+['-I'+str(p) for p in includes]
objects=[];commands=[]
for i,source in enumerate(sources):
    obj=OUT/(str(i)+'.obj');command=base+['-c',str(source),'-o',str(obj)]
    subprocess.run(command,env=env,check=True);commands.append(command);objects.append(obj)
exe=OUT/'lifecycle.exe';command=base+list(map(str,objects))+['-Wl,/NODEFAULTLIB:libcmt','-o',str(exe)]
subprocess.run(command,env=env,check=True);commands.append(command)
resource=pathlib.Path(subprocess.check_output([cc,'-print-resource-dir'],text=True).strip());env['PATH']=str(resource/'lib/windows')+os.pathsep+env.get('PATH','')
result=subprocess.run([str(exe)],env=env,timeout=15,capture_output=True,text=True)
(OUT/'run.stdout.txt').write_text(result.stdout);(OUT/'run.stderr.txt').write_text(result.stderr)
print(result.stdout.strip());print(result.stderr.strip());result.check_returncode()
assert 'PASS actual DSP-assisted output' in result.stdout
assert all(hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()==h for p,h in before.items())
(OUT/'results.json').write_text(json.dumps({'passed':True,'fair_share':options.fair_share,'early_input':options.early_input,'rebalance':options.rebalance,'hardware_accessed':False,'sanitizers':['address','undefined'],'scope':'actual output/render/sound and DSP lane, two mocked FreeRTOS cores, held BUSY worker with queued stop/cancel, owner completion before modeled producer preemption, no early free or timed pending polling; no ESP32 timing','stdout':result.stdout,'sources':before,'commands':commands},indent=2)+'\n')
