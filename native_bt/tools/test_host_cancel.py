"""Owner-only HCI command cancellation, queue wipe, and admission tests."""
import ctypes
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

root=Path(__file__).resolve().parents[1]
out=root/'_build/host_cancel'
out.mkdir(parents=True,exist_ok=True)
check=[sys.executable,str(root/'tools/check_workspace.py')]
subprocess.run(check,check=True)
try:
    compiler=shutil.which('clang') or shutil.which('gcc') or r'C:\Program Files\LLVM\bin\clang.exe'
    sources=[root/'src'/(n+'.c') for n in ('dbt_queue','dbt_hci','dbt_host')]+[root/'tests/host_cancel/cancel.c']
    options=['-std=c11','-O1','-Wall','-Wextra','-Werror','-I',str(root/'include')]
    if os.name=='nt':
        options+=['-ffreestanding','-fno-builtin','-fno-stack-protector','-fuse-ld=lld','-nostdlib','-shared','-Wl,/noentry']
        sources.append(root/'tests/avrcp/runtime_freestanding.c')
    else:options+=['-shared','-fPIC']
    tracked=sources+[root/'include'/(n+'.h') for n in ('dbt_queue','dbt_hci','dbt_host')]+[Path(__file__).resolve()]
    before={p.relative_to(root).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked}
    binary=out/('cancel.dll' if os.name=='nt' else 'cancel.so')
    subprocess.run([compiler]+options+[str(s) for s in sources]+['-o',str(binary)],check=True)
    lib=ctypes.CDLL(str(binary))
    names=('queued_order_and_wipe','observer_prevents_submit','admitted_and_following','reinit_and_host_identity','wrap_and_rejected_transport','abort_preserves_transport_borrow')
    for name in names:
        line=getattr(lib,'cancel_'+name)()
        if line:raise SystemExit(f'FAIL {name}: line {line}')
        print('PASS',name)
    after={p.relative_to(root).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked}
    if before!=after:raise RuntimeError('Sources changed during build/test')
    (out/'results.json').write_text(json.dumps({'passed':True,'tests':list(names),'source_sha256':before},indent=2)+'\n')
finally:
    subprocess.run(check,check=True)
