"""Offline production output-format test with real dbt_audio and mocked I2S."""
import argparse,ctypes,os,pathlib,shutil,subprocess,sys
parser=argparse.ArgumentParser();parser.add_argument('--compact-copy',action='store_true');parser.add_argument('--block-dsp',action='store_true');parser.add_argument('--priority-order',action='store_true');parser.add_argument('--word-dsp',action='store_true');options=parser.parse_args()
if options.word_dsp and not options.block_dsp:raise SystemExit('--word-dsp requires --block-dsp')
root=pathlib.Path(__file__).resolve().parents[1];ws=root.parent;out=root/'_build/avrcp';out.mkdir(parents=True,exist_ok=True)
check=[sys.executable,str(root/'tools/check_workspace.py')];subprocess.run(check,check=True)
env=os.environ.copy()
try:
    if os.name=='nt':
        vc=pathlib.Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
        data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
        env.update(dict(x.split('=',1) for x in data.splitlines() if '=' in x and not x.startswith('=')))
    cc=shutil.which('clang',path=env.get('PATH')) or (r'C:\Program Files\LLVM\bin\clang.exe' if os.name=='nt' else 'cc')
    cache=shutil.which('ccache',path=env.get('PATH'))
    if not cache:
        matches=sorted((pathlib.Path.home()/'.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'))
        if matches:cache=str(matches[-1])
    if not cache:raise RuntimeError('ccache required')
    env.pop('CCACHE_DISABLE',None)
    incs=[root/'tests/codecs/probe_audio_stubs',root/'include',root/'esp32_probe/main',ws/'bt_audio_sink/main/dsp']
    src=[root/'tests/codecs/probe_output_format.c',root/'src/dbt_audio.c',root/'src/dbt_resampler.c',ws/'bt_audio_sink/main/dsp/fixed_dsp.c']
    cmd=[cache,cc,'-std=c11','-O1','-g','-fwrapv','-shared','-DCONFIG_I2S_ISR_IRAM_SAFE=1','-DCONFIG_NATIVE_AUDIO_BCLK=26','-DCONFIG_NATIVE_AUDIO_LRCK=25','-DCONFIG_NATIVE_AUDIO_DATA=27']
    if options.compact_copy:cmd+=['-DDBT_PROBE_PCM_COPIED=1','-DDBT_PROBE_ASYNC_AUDIO=1','-DDBT_PROBE_PCM_TARGET_MS=16']
    if options.block_dsp:cmd+=['-DDBT_AUDIO_BLOCK_DSP=1','-DFIXED_DSP_BLOCK_API=1','-DDBT_PROBE_BLOCK_DSP=1']
    if options.priority_order:cmd+=['-DDBT_PROBE_PRIORITY_ORDER=1','-DDBT_PROBE_DECODER_PRIORITY=6']
    if options.word_dsp:cmd+=['-DDBT_PROBE_WORD_DSP=1','-DFIXED_DSP_WORD_QUANT=1']
    if os.name!='nt':cmd+=['-fPIC']
    cmd+=sum((['-I',str(i)] for i in incs),[])+[str(s) for s in src]
    for hz in (100,1000):
        tag='compact-block' if options.compact_copy and options.block_dsp else 'compact' if options.compact_copy else 'block' if options.block_dsp else 'format'
        artifact=out/(f'probe-output-{tag}-{hz}.dll' if os.name=='nt' else f'probe-output-{tag}-{hz}.so')
        subprocess.run(cmd+['-DDBT_TEST_TICK_HZ='+str(hz),'-o',str(artifact)],env=env,check=True)
        lib=ctypes.CDLL(str(artifact));line=(lib.probe_output_compact_copy if options.compact_copy else lib.probe_output_formats)()
        if line:raise SystemExit('FAIL production output PCM formats line '+str(line))
        line=lib.probe_output_descriptor_diagnostics()
        if line:raise SystemExit('FAIL distinct descriptor diagnostics line '+str(line))
        print(f'PASS '+('compact copied geometry, source reuse, backpressure and notifications' if options.compact_copy else 'production output 16/24/32-bit formats, bounds, teardown and nonzero event waits')+f' at {hz} Hz, mocked I2S')
finally:subprocess.run(check,check=True)
