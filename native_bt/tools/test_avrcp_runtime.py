"""Paired real-runtime AVRCP simulation. --sanitize runs Linux ASan/UBSan.
No SSH, credentials, radio, device or network state is used."""
import argparse, ctypes, hashlib, json, os, pathlib, shutil, subprocess, sys
root=pathlib.Path(__file__).resolve().parents[1]
out=root/'_build'/'avrcp'
out.mkdir(parents=True,exist_ok=True)
parser=argparse.ArgumentParser()
parser.add_argument('--sanitize',action='store_true')
parser.add_argument('--cc')
parser.add_argument('--list-inputs',action='store_true',help='print project-relative files needed for an offline Linux copy')
args=parser.parse_args()
compiler=args.cc or os.environ.get('CC') or shutil.which('clang') or shutil.which('cc')
if not compiler and os.name=='nt': compiler=r'C:\Program Files\LLVM\bin\clang.exe'
if not compiler: raise SystemExit('C11 compiler not found')
if args.sanitize and os.name=='nt': raise SystemExit('--sanitize requires Linux')
src=['dbt_queue','dbt_hci','dbt_host','dbt_acl','dbt_l2cap','dbt_channels','dbt_runtime',
     'dbt_rx_flow','dbt_credit','dbt_att','dbt_att_bearer',
     'dbt_avctp','dbt_avctp_bearer','dbt_avrcp','dbt_avrcp_bearer']
paths=[root/'src'/(name+'.c') for name in src]+[root/'tests/avrcp/runtime_pair.c']
headers=[root/'include'/(name+'.h') for name in src]
tracked=paths+headers+[root/'tests/avrcp/runtime_main.c',root/'tests/avrcp/runtime_freestanding.c',root/'tools/check_workspace.py',pathlib.Path(__file__).resolve()]
if args.list_inputs:
    print(json.dumps([p.relative_to(root).as_posix() for p in tracked],indent=2)); sys.exit(0)
common=[compiler,'-std=c11','-O1','-g','-Wall','-Wextra','-Werror','-I',str(root/'include')]
check=[sys.executable,str(root/'tools/check_workspace.py')]
subprocess.run(check,check=True)
try:
    if args.sanitize:
        artifact=out/'runtime-sanitized'
        flags=['-fsanitize=address,undefined','-fno-omit-frame-pointer']
        command=common+flags+[str(p) for p in paths]+[str(root/'tests/avrcp/runtime_main.c'),'-o',str(artifact)]
    else:
        artifact=out/('runtime.dll' if os.name=='nt' else 'runtime.so')
        flags=['-ffreestanding','-fno-builtin','-fno-stack-protector','-fuse-ld=lld','-nostdlib','-shared','-Wl,/noentry'] if os.name=='nt' else ['-shared','-fPIC']
        extra=[root/'tests/avrcp/runtime_freestanding.c'] if os.name=='nt' else []
        command=common+flags+[str(p) for p in paths+extra]+['-o',str(artifact)]
    subprocess.run(command,check=True)
    names=('interleave_credit','disconnect_reuse','competing_loan')
    if args.sanitize:
        subprocess.run([str(artifact)],check=True,timeout=60)
    else:
        dll=ctypes.CDLL(str(artifact))
        for name in names:
            line=getattr(dll,'avrcp_runtime_'+name)()
            if line: raise SystemExit(f'FAIL {name} line {line}')
            print('PASS',name)
    hashes={p.relative_to(root).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked}
    report={'passed':True,'scope':'paired simulated controllers using owned HCI ACL L2CAP runtime and real ATT/AVRCP bearers; no RF or hardware',
            'sanitizers':'address,undefined' if args.sanitize else None,'tests':list(names),'sha256':hashes}
    suffix='linux-asan' if args.sanitize else ('windows' if os.name=='nt' else 'linux')
    (out/('runtime-'+suffix+'.json')).write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
finally:
    subprocess.run(check,check=True)
