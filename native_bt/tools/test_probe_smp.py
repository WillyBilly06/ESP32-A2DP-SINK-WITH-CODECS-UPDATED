"""Offline deterministic queue/provider shim for the SMP probe adapter."""
import os
from pathlib import Path
import shutil
import subprocess
import sys

root=Path(__file__).resolve().parents[1]
sdk=Path(os.environ.get('DBT_SMP_MBEDTLS_SOURCE',root.parent/'esp-idf/components/mbedtls/mbedtls'))
out=root/'_build/smp'
env=os.environ.copy()
if os.name=='nt':
    vc=Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
    data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
    env.update(dict(line.split('=',1) for line in data.splitlines() if '=' in line and not line.startswith('=')))
cc=os.environ.get('CC') or (r'C:\Program Files\LLVM\bin\clang.exe' if os.name=='nt' else shutil.which('cc'))
if not cc: raise SystemExit('C compiler required')
names='queue hci host acl l2cap credit ertm ertm_engine channels rx_flow runtime smp smp_crypto smp_bond smp_guard smp_dist smp_features smp_sc smp_bearer smp_session'.split()
check=[sys.executable,str(root/'tools/check_workspace.py')]
subprocess.run(check,check=True)
try:
    out.mkdir(parents=True,exist_ok=True)
    for fixture in ('probe_adapter','probe_reconnect'):
        exe=out/(fixture+'_test'+('.exe' if os.name=='nt' else ''))
        command=[cc,'-std=c11','-Wall','-Wextra','-Werror','-I'+str(root/'tests/smp/probe_shim'),
                 '-I'+str(root/'include'),str(root/f'tests/smp/{fixture}.c'),
                 *[str(root/f'src/dbt_{name}.c') for name in names]]
        if fixture=='probe_reconnect': command += ['-I'+str(sdk/'include'),'-I'+str(sdk/'library'),
            str(root/'src/esp32/dbt_smp_nvs.c'),str(root/'tests/smp/nvs_mock.c'),
            str(sdk/'library/aes.c'),str(sdk/'library/aesni.c'),str(sdk/'library/sha256.c'),str(sdk/'library/platform_util.c')]
        command += ['-o',str(exe)]
        subprocess.run(command,env=env,check=True)
        subprocess.run([str(exe)],env=env,check=True)
    print('SMP probe worker lifecycle and NVS-shim reconnect checks passed.')
finally:
    subprocess.run(check,check=True)
