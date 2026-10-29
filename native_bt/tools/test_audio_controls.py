"""Offline control publication, priority-preemption model and thread tests."""
import ctypes,os,pathlib,shutil,subprocess,sys,threading
root=pathlib.Path(__file__).resolve().parents[1];ws=root.parent;out=root/'_build/avrcp';out.mkdir(parents=True,exist_ok=True)
check=[sys.executable,str(root/'tools/check_workspace.py')];subprocess.run(check,check=True);env=os.environ.copy()
try:
    if os.name=='nt':
        vc=pathlib.Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
        data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True);env.update(dict(x.split('=',1) for x in data.splitlines() if '=' in x and not x.startswith('=')))
    cc=shutil.which('clang',path=env.get('PATH')) or (r'C:\Program Files\LLVM\bin\clang.exe' if os.name=='nt' else 'cc')
    src=[root/'tests/audio/control_fields.c',root/'src/dbt_audio.c',root/'src/dbt_resampler.c',ws/'bt_audio_sink/main/dsp/fixed_dsp.c']
    for mocked in (False,True):
        target=out/(('control-preemption' if mocked else 'control-portable')+('.dll' if os.name=='nt' else '.so'))
        cmd=[cc,'-std=c11','-O2','-g','-Wall','-Wextra','-Werror','-fwrapv','-shared','-I',str(root/'include'),'-I',str(ws/'bt_audio_sink/main/dsp')]
        if os.name!='nt':cmd+=['-fPIC']
        if mocked:cmd+=['-DDBT_AUDIO_CONTROL_TEST_IRQ=1']
        subprocess.run(cmd+[str(x) for x in src]+['-o',str(target)],env=env,check=True)
        lib=ctypes.CDLL(str(target))
        for name in ('control_fields_rendered','control_invalid_busy_generation')+(('control_preemption_model',) if mocked else ()):
            line=getattr(lib,name)()
            if line:raise SystemExit('FAIL '+name+' line '+str(line))
            print('PASS',name,'mocked_mask='+str(mocked))
        if not mocked:
            line=lib.control_race_begin()
            if line:raise SystemExit('FAIL race begin line '+str(line))
            failures=[]
            def work(name):
                fn=getattr(lib,name)
                for _ in range(1000):
                    for attempt in range(100000):
                        result=fn()
                        if result==0:break
                        if result!=5:failures.append((name,result));return
                    else:failures.append((name,'busy_timeout'));return
            threads=[threading.Thread(target=work,args=(name,)) for name in ('control_race_eq','control_race_volume','control_race_flags','control_race_analysis')]
            for t in threads:t.start()
            for t in threads:t.join(timeout=30)
            if any(t.is_alive() for t in threads) or failures:raise SystemExit('FAIL concurrent writers '+str(failures))
            line=lib.control_race_end()
            if line:raise SystemExit('FAIL concurrent fields line '+str(line))
            print('PASS four concurrent field writers, 4000 successful publications')
finally:subprocess.run(check,check=True)
