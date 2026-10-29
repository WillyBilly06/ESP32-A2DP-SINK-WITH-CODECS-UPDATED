"""Exercise peer-feature discovery in the actual channel manager offline."""
import ctypes
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys

root=Path(__file__).resolve().parents[1]
out=root/'_build/ertm/features'
out.mkdir(parents=True,exist_ok=True)
check=[sys.executable,str(root/'tools/check_workspace.py')]
subprocess.run(check,check=True)
try:
    compiler=shutil.which('clang') or shutil.which('gcc') or r'C:\Program Files\LLVM\bin\clang.exe'
    names=['dbt_queue','dbt_hci','dbt_host','dbt_acl','dbt_l2cap','dbt_channels','dbt_credit','dbt_ertm','dbt_ertm_engine','dbt_runtime','dbt_rx_flow']
    channel_tests=root/'tests/test_channels.c'
    sources=[root/'src'/(n+'.c') for n in names]+[root/'tests/ertm/feature_discovery.c',root/'tests/ertm/runtime_pair.c',channel_tests]
    options=['-std=c11','-O1','-Wall','-Wextra','-Werror','-I',str(root/'include')]
    if os.name=='nt':
        options+=['-ffreestanding','-fno-builtin','-fno-stack-protector','-fuse-ld=lld','-nostdlib','-shared','-Wl,/noentry']
        sources.append(root/'tests/avrcp/runtime_freestanding.c')
    else:options+=['-shared','-fPIC']
    headers=['dbt_acl','dbt_channels','dbt_credit','dbt_ertm','dbt_ertm_engine','dbt_hci','dbt_host','dbt_l2cap','dbt_queue','dbt_runtime','dbt_rx_flow']
    tracked=sources+[root/'include'/(n+'.h') for n in headers]+[Path(__file__).resolve()]
    before={p.relative_to(root).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked}
    binary=out/('features.dll' if os.name=='nt' else 'features.so')
    subprocess.run([compiler]+options+[str(s) for s in sources]+['-o',str(binary)],check=True)
    lib=ctypes.CDLL(str(binary))
    for name in ('success_and_generation','timeout_and_reject','short_and_cancel','ids_and_clear'):
        result=getattr(lib,'feature_discovery_'+name)()
        if result:raise SystemExit(f'FAIL {name}: line {result}')
        print('PASS',name)
    regressions=re.findall(r'EXPORT\s+int\s+(test_(?:channels|le_coc)_\w+)\s*\(void\)',channel_tests.read_text())
    for name in regressions:
        result=getattr(lib,name)()
        if result:raise SystemExit(f'FAIL {name}: line {result}')
    print('PASS',len(regressions),'existing Classic and LE channel regressions')
    runtime_tests=('pair','ack_loan_and_recovery','link_loss_pinned','bidirectional_oom','required_rejects_basic','security_revocation','missing_peer_feature','fcs_default_if_one_omits','le_coexistence','basic_same_link','wait_transport_gate')
    for name in runtime_tests:
        result=getattr(lib,'ertm_runtime_'+name)()
        if result:raise SystemExit(f'FAIL ERTM runtime {name}: line {result}')
        print('PASS ERTM runtime',name)
    after={p.relative_to(root).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked}
    if before!=after:raise RuntimeError('Sources changed during ERTM build/test; no valid receipt')
    report={'passed':True,'scope':'Classic feature discovery and required-mode ERTM with paired simulated runtimes; no RF/hardware',
            'feature_groups':4,'channel_regressions':len(regressions),'runtime_tests':list(runtime_tests),'source_sha256':before}
    (out/'results.json').write_text(json.dumps(report,indent=2)+'\n')
finally:
    subprocess.run(check,check=True)
