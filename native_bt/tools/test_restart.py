"""Offline restart-barrier composition tests; no reset, flash or device access."""
import json,os,shutil,subprocess,sys
from pathlib import Path
root=Path(__file__).resolve().parents[1];out=root/'_build/restart';out.mkdir(exist_ok=True)
check=[sys.executable,str(root/'tools/check_workspace.py')];subprocess.run(check,check=True)
env=os.environ.copy()
if os.name=='nt':
    vc=Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
    data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
    env.update(dict(line.split('=',1) for line in data.splitlines() if '=' in line and not line.startswith('=')))
cc=shutil.which('clang') or r'C:\Program Files\LLVM\bin\clang.exe';exe=out/('coordinator.exe' if os.name=='nt' else 'coordinator')
try:
    subprocess.run([cc,'-std=c11','-O1','-Wall','-Wextra','-Werror',
        '-I'+str(root/'tests/ble_control/shim'),'-I'+str(root/'include'),'-I'+str(root/'esp32_probe/main'),'-I'+str(root.parent/'bt_audio_sink/main/dsp'),
        str(root/'tests/restart/coordinator.c'),str(root/'esp32_probe/main/probe_restart.c'),'-o',str(exe)],check=True,env=env)
    subprocess.run([str(exe)],check=True,env=env)
    (out/'results.json').write_text(json.dumps({'passed':True,'scope':'real coordinator with subsystem/transport shims; no SDK scheduling/controller/hardware proof','gate_cases':7,'pending_work_cases':12,'epoch_timeout_fault_and_cancel':True,'hardware_accessed':False},indent=2)+'\n',encoding='utf-8')
    print('PASS restart epoch, producer fences, pending work, loans, fault, wrap-timeout and cancel')
finally:subprocess.run(check,check=True)
