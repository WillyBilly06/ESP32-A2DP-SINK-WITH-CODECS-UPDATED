"""Offline digital clock-servo model and real fixed-point render tests.
--sanitize is a local Linux ASan/UBSan build; no radio, network or hardware."""
import argparse,ctypes,os,pathlib,shutil,subprocess,sys
parser=argparse.ArgumentParser();parser.add_argument('--sanitize',action='store_true');parser.add_argument('--list-inputs',action='store_true');args=parser.parse_args()
root=pathlib.Path(__file__).resolve().parents[1];ws=root.parent
paths=[root/'tests/audio/clock_model.c',root/'tests/audio/clock_main.c',root/'src/dbt_audio.c',root/'include/dbt_audio.h',root/'include/dbt_queue.h',root/'include/dbt_resampler.h',root/'src/dbt_resampler.c',root/'src/dbt_resampler_coeffs.inc',ws/'bt_audio_sink/main/dsp/fixed_dsp.c',ws/'bt_audio_sink/main/dsp/fixed_dsp.h',ws/'bt_audio_sink/main/dsp/fixed_dsp_tables.inc',root/'tools/check_workspace.py',pathlib.Path(__file__).resolve()]
if args.list_inputs:
    for p in paths:print(p.relative_to(ws).as_posix())
    sys.exit(0)
if args.sanitize and os.name=='nt':raise SystemExit('--sanitize requires Linux')
out=root/'_build/avrcp';out.mkdir(parents=True,exist_ok=True)
check=[sys.executable,str(root/'tools/check_workspace.py')];subprocess.run(check,check=True)
env=os.environ.copy()
try:
    if os.name=='nt':
        vc=pathlib.Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
        data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True);env.update(dict(x.split('=',1) for x in data.splitlines() if '=' in x and not x.startswith('=')))
    cc=shutil.which('clang',path=env.get('PATH')) or (r'C:\Program Files\LLVM\bin\clang.exe' if os.name=='nt' else 'cc')
    target=out/('audio-clock-sanitized' if args.sanitize else 'audio-clock.dll' if os.name=='nt' else 'audio-clock.so')
    cmd=[cc,'-std=c11','-O2','-g','-Wall','-Wextra','-Werror','-fwrapv']
    if args.sanitize:cmd+=['-fsanitize=address,undefined','-fno-omit-frame-pointer','-fno-sanitize-recover=all']
    else:
        cmd+=['-shared']
        if os.name!='nt':cmd+=['-fPIC']
    cmd+=['-I',str(root/'include'),'-I',str(ws/'bt_audio_sink/main/dsp'),str(root/'tests/audio/clock_model.c'),str(root/'src/dbt_audio.c'),str(root/'src/dbt_resampler.c'),str(ws/'bt_audio_sink/main/dsp/fixed_dsp.c')]
    if args.sanitize:cmd+=[str(root/'tests/audio/clock_main.c')]
    if os.name!='nt':cmd+=['-lm']
    subprocess.run(cmd+['-o',str(target)],env=env,check=True)
    if args.sanitize:subprocess.run([str(target)],env=env,check=True,timeout=60)
    else:
        lib=ctypes.CDLL(str(target))
        for name in ('clock_model_rates','clock_fractional_waveform','clock_real_pipeline','clock_anti_windup','clock_backpressure_relief','clock_switch_reset','clock_interval_jitter','clock_disabled','clock_spectral_tones','clock_drift_tone','clock_resampler_bounds_and_wrap','clock_resampler_drain'):
            line=getattr(lib,name)()
            if line:raise SystemExit('FAIL '+name+' line '+str(line))
            print('PASS',name)
        getter=lib.clock_spectral_stats;getter.argtypes=[ctypes.POINTER(ctypes.c_double)]*4
        values=[ctypes.c_double() for _ in range(4)];getter(*(ctypes.byref(x) for x in values))
        print('spectral: min/max_gain_dB=%.6f/%.6f max_phase_ripple_dB=%.6f max_phase_error_deg=%.6f' % tuple(x.value for x in values))
        lib.clock_drift_tone_worst_dbc.restype=ctypes.c_double
        print('worst_drift_sine_residual_dBc=%.3f' % lib.clock_drift_tone_worst_dbc())
        lib.clock_drift_sideband_worst_dbc.restype=ctypes.c_double
        print('worst_first_four_phase_sidebands_dBc=%.3f' % lib.clock_drift_sideband_worst_dbc())
finally:subprocess.run(check,check=True)
