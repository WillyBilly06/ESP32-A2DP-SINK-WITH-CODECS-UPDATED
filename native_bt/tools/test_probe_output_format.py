"""Offline production output-format test with real dbt_audio and mocked I2S."""
import ctypes,os,pathlib,shutil,subprocess,sys
root=pathlib.Path(__file__).resolve().parents[1];ws=root.parent;out=root/'_build/avrcp';out.mkdir(parents=True,exist_ok=True)
check=[sys.executable,str(root/'tools/check_workspace.py')];subprocess.run(check,check=True)
env=os.environ.copy()
try:
    if os.name=='nt':
        vc=pathlib.Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
        data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
        env.update(dict(x.split('=',1) for x in data.splitlines() if '=' in x and not x.startswith('=')))
    cc=shutil.which('clang',path=env.get('PATH')) or (r'C:\Program Files\LLVM\bin\clang.exe' if os.name=='nt' else 'cc')
    incs=[root/'tests/codecs/probe_audio_stubs',root/'include',root/'esp32_probe/main',ws/'bt_audio_sink/main/dsp']
    src=[root/'tests/codecs/probe_output_format.c',root/'src/dbt_audio.c',root/'src/dbt_resampler.c',ws/'bt_audio_sink/main/dsp/fixed_dsp.c']
    artifact=out/('probe-output-format.dll' if os.name=='nt' else 'probe-output-format.so')
    cmd=[cc,'-std=c11','-O1','-g','-fwrapv','-shared','-DCONFIG_I2S_ISR_IRAM_SAFE=1','-DCONFIG_NATIVE_AUDIO_BCLK=26','-DCONFIG_NATIVE_AUDIO_LRCK=25','-DCONFIG_NATIVE_AUDIO_DATA=27']
    if os.name!='nt':cmd+=['-fPIC']
    cmd+=sum((['-I',str(i)] for i in incs),[])+[str(s) for s in src]
    subprocess.run(cmd+['-o',str(artifact)],env=env,check=True)
    lib=ctypes.CDLL(str(artifact));line=lib.probe_output_formats()
    if line:raise SystemExit('FAIL production output PCM formats line '+str(line))
    print('PASS production output 16/24/32-bit format, stride, bounds, and teardown with mocked I2S')
finally:subprocess.run(check,check=True)
