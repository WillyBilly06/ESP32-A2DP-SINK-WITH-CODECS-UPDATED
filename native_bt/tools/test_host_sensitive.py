"""Offline focused ACL ingress wiping, failure and borrowed-lifetime tests."""
import ctypes
import hashlib
import os
from pathlib import Path
import shutil
import subprocess
import sys

root=Path(__file__).resolve().parents[1]
out=root/'_build/host_sensitive'
check=[sys.executable,str(root/'tools/check_workspace.py')]
subprocess.run(check,check=True)
try:
    out.mkdir(parents=True,exist_ok=True)
    compiler=shutil.which('clang') or shutil.which('gcc') or r'C:\Program Files\LLVM\bin\clang.exe'
    sources=[root/'src'/(n+'.c') for n in ('dbt_queue','dbt_hci','dbt_host')]+[root/'tests/host_sensitive/ingress.c']
    options=['-std=c11','-O1','-Wall','-Wextra','-Werror','-I',str(root/'include')]
    if os.name=='nt':
        options+=['-ffreestanding','-fno-builtin','-fno-stack-protector','-fuse-ld=lld','-nostdlib','-shared','-Wl,/noentry']
        sources.append(root/'tests/avrcp/runtime_freestanding.c')
    else: options+=['-shared','-fPIC']
    tracked=sources+[root/'include'/(n+'.h') for n in ('dbt_queue','dbt_hci','dbt_host')]+[Path(__file__).resolve()]
    before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked}
    binary=out/('ingress.dll' if os.name=='nt' else 'ingress.so')
    subprocess.run([compiler]+options+[str(s) for s in sources]+['-o',str(binary)],check=True)
    lib=ctypes.CDLL(str(binary))
    for name in ('acl_and_events','abort_and_borrow','abort_with_queued_and_late','error_paths','control_reserve'):
        line=getattr(lib,'host_sensitive_'+name)()
        if line: raise SystemExit(f'FAIL {name}: line {line}')
        print('PASS',name)
    env=os.environ.copy()
    if os.name=='nt':
        vc=Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
        data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
        env.update(dict(line.split('=',1) for line in data.splitlines() if '=' in line and not line.startswith('=')))
    threaded=root/'tests/host_sensitive/threaded.c'; tracked.append(threaded)
    before[str(threaded)]=hashlib.sha256(threaded.read_bytes()).hexdigest()
    exe=out/('threaded.exe' if os.name=='nt' else 'threaded')
    cmd=[compiler,'-std=c11','-O2','-Wall','-Wextra','-Werror','-I',str(root/'include'),
         *[str(root/'src'/(n+'.c')) for n in ('dbt_queue','dbt_hci','dbt_host')],str(threaded),'-o',str(exe)]
    if os.name!='nt': cmd+=['-pthread']
    subprocess.run(cmd,env=env,check=True); subprocess.run([str(exe)],env=env,check=True)
    after={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked}
    if before!=after: raise RuntimeError('Sources changed during build/test')
finally:
    subprocess.run(check,check=True)
