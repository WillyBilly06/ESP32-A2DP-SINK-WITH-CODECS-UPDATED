"""Offline probe integration over two owned simulated Classic runtimes.
Actual owned ERTM engine linked; this control test uses Basic mode. No browsing or RF.
--sanitize builds a Linux ASan/UBSan executable."""
import argparse, ctypes, os, pathlib, re, shutil, subprocess, sys
root=pathlib.Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser();parser.add_argument('--sanitize',action='store_true');parser.add_argument('--list-inputs',action='store_true');parser.add_argument('--cc')
args=parser.parse_args()
if args.sanitize and os.name=='nt':raise SystemExit('--sanitize requires Linux')
cc=args.cc or os.environ.get('CC') or shutil.which('clang') or (r'C:\Program Files\LLVM\bin\clang.exe' if os.name=='nt' else 'cc')
names=('dbt_queue','dbt_hci','dbt_host','dbt_acl','dbt_l2cap','dbt_channels','dbt_runtime','dbt_rx_flow','dbt_credit','dbt_ertm','dbt_ertm_engine','dbt_att','dbt_att_bearer','dbt_avctp','dbt_avctp_bearer','dbt_avrcp','dbt_avrcp_bearer')
src=[root/'tests/avrcp/probe_runtime_pair.c',root/'esp32_probe/main/probe_avrcp.c']+[root/'src'/(n+'.c') for n in names]
extra=[root/'tests/avrcp/runtime_freestanding.c'] if os.name=='nt' else []
if args.list_inputs:
    pending=src+[root/'tests/avrcp/probe_runtime_main.c',root/'tools/check_workspace.py',pathlib.Path(__file__).resolve()]
    seen=set()
    while pending:
        p=pending.pop().resolve()
        if p in seen:continue
        seen.add(p)
        for name in re.findall(r'^\s*#include\s+"([^"]+)"',p.read_text(encoding='utf-8-sig'),re.MULTILINE):
            for q in (p.parent/name,root/'include'/name,root/'esp32_probe/main'/name,root/'tests/avrcp/probe_shim'/name):
                if q.is_file():pending.append(q);break
    print('\n'.join(sorted(p.relative_to(root).as_posix() for p in seen)));sys.exit(0)
cache=shutil.which('ccache')
if not cache:
    candidates=sorted((pathlib.Path.home()/'.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'))
    if candidates:cache=str(candidates[-1])
assert cache,'ccache required'
os.environ.pop('CCACHE_DISABLE',None)
out=root/'_build'/'avrcp';out.mkdir(parents=True,exist_ok=True)
check=[sys.executable,str(root/'tools/check_workspace.py')];subprocess.run(check,check=True)
try:
    if args.sanitize:
        flags=['-fsanitize=address,undefined','-fno-omit-frame-pointer'];artifact=out/'probe-runtime-sanitized';extra=[root/'tests/avrcp/probe_runtime_main.c']
    else:
        flags=['-ffreestanding','-fno-builtin','-fno-stack-protector','-fuse-ld=lld','-nostdlib','-shared','-Wl,/noentry'] if os.name=='nt' else ['-shared','-fPIC']
        artifact=out/('probe-runtime.dll' if os.name=='nt' else 'probe-runtime.so')
    compile_flags=[x for x in flags if x not in ('-fuse-ld=lld','-nostdlib','-shared','-Wl,/noentry')]
    objects=[]
    for index,source in enumerate(src+extra):
        obj=out/('paired-'+str(index)+('.obj' if os.name=='nt' else '.o'))
        subprocess.run([cache,cc,'-std=c11','-O1','-g','-Wall','-Wextra','-Werror','-I',str(root/'include'),'-I',str(root/'esp32_probe/main'),'-I',str(root/'tests/avrcp/probe_shim')]+compile_flags+['-c',str(source),'-o',str(obj)],check=True)
        objects.append(obj)
    subprocess.run([cc]+flags+[str(x) for x in objects]+['-o',str(artifact)],check=True)
    if args.sanitize:subprocess.run([str(artifact)],check=True,timeout=60)
    else:
        dll=ctypes.CDLL(str(artifact));line=dll.probe_runtime_exchange()
        if line:raise SystemExit('FAIL paired runtime line '+str(line))
        print('PASS paired runtime exchange and pinned disconnect')
finally:subprocess.run(check,check=True)
