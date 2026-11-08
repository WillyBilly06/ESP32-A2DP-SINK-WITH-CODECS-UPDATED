"""Paired real-runtime AVRCP simulation. --sanitize runs Linux ASan/UBSan.
No SSH, credentials, radio, device or network state is used."""
import argparse, ctypes, hashlib, json, os, pathlib, shutil, subprocess, sys
root=pathlib.Path(__file__).resolve().parents[1]
out=root/'_build'/'avrcp'
out.mkdir(parents=True,exist_ok=True)
parser=argparse.ArgumentParser()
parser.add_argument('--sanitize',action='store_true')
parser.add_argument('--cc')
parser.add_argument('--out',type=pathlib.Path,default=out)
parser.add_argument('--runtime-source',type=pathlib.Path)
parser.add_argument('--list-inputs',action='store_true',help='print project-relative files needed for an offline Linux copy')
args=parser.parse_args()
out=args.out.resolve();out.relative_to((root/'_build').resolve());out.mkdir(parents=True,exist_ok=True)
compiler=args.cc or os.environ.get('CC') or shutil.which('clang') or shutil.which('cc')
if not compiler and os.name=='nt': compiler=r'C:\Program Files\LLVM\bin\clang.exe'
if not compiler: raise SystemExit('C11 compiler not found')

src=['dbt_queue','dbt_hci','dbt_host','dbt_acl','dbt_l2cap','dbt_channels','dbt_runtime','dbt_ertm','dbt_ertm_engine',
     'dbt_rx_flow','dbt_credit','dbt_att','dbt_att_bearer',
     'dbt_avctp','dbt_avctp_bearer','dbt_avrcp','dbt_avrcp_bearer']
paths=[root/'src'/(name+'.c') for name in src]+[root/'tests/avrcp/runtime_pair.c']
if args.runtime_source:
    runtime_source=args.runtime_source.resolve();runtime_source.relative_to((root/'_build').resolve())
    paths=[runtime_source if q.name=='dbt_runtime.c' else q for q in paths]
headers=[root/'include'/(name+'.h') for name in src]
tracked=paths+headers+[root/'tests/avrcp/runtime_main.c',root/'tests/avrcp/runtime_freestanding.c',root/'tools/check_workspace.py',pathlib.Path(__file__).resolve()]
if args.list_inputs:
    print(json.dumps([p.relative_to(root).as_posix() for p in tracked],indent=2)); sys.exit(0)
initial_hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked}
cache=shutil.which('ccache')
if not cache:
    candidates=sorted((pathlib.Path.home()/'.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'))
    if candidates:cache=str(candidates[-1])
assert cache,'ccache required'
env=os.environ.copy();env.pop('CCACHE_DISABLE',None)
if args.sanitize and os.name=='nt':
    vc=pathlib.Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
    values=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
    env.update(dict(x.split('=',1) for x in values.splitlines() if '=' in x and not x.startswith('=')))
    resource=pathlib.Path(subprocess.check_output([compiler,'-print-resource-dir'],text=True).strip())
    env['PATH']=str(resource/'lib/windows')+os.pathsep+env.get('PATH','')
common=[cache,compiler,'-std=c11','-O1','-g','-Wall','-Wextra','-Werror','-I',str(root/'include')]
check=[sys.executable,str(root/'tools/check_workspace.py')]
subprocess.run(check,check=True)
try:
    if args.sanitize:
        artifact=out/('runtime-sanitized.exe' if os.name=='nt' else 'runtime-sanitized')
        flags=['-fsanitize=address,undefined','-fno-omit-frame-pointer','-fno-sanitize-recover=all']
        if os.name=='nt':flags+=['-fms-runtime-lib=dll','-D_DLL']
        command=common+flags+[str(p) for p in paths]+[str(root/'tests/avrcp/runtime_main.c'),'-o',str(artifact)]
    else:
        artifact=out/('runtime.dll' if os.name=='nt' else 'runtime.so')
        flags=['-ffreestanding','-fno-builtin','-fno-stack-protector','-fuse-ld=lld','-nostdlib','-shared','-Wl,/noentry'] if os.name=='nt' else ['-shared','-fPIC']
        extra=[root/'tests/avrcp/runtime_freestanding.c'] if os.name=='nt' else []
        command=common+flags+[str(p) for p in paths+extra]+['-o',str(artifact)]
    # Compile each object through ccache; a combined compile/link is not cached.
    object_flags=[f for f in flags if f not in ('-shared','-nostdlib','-fuse-ld=lld','-Wl,/noentry')]
    compile_paths=paths+([root/'tests/avrcp/runtime_main.c'] if args.sanitize else extra)
    objects=[]
    for i,source in enumerate(compile_paths):
        obj=out/('avrcp-runtime-'+str(i)+'.obj');objects.append(obj)
        subprocess.run(common+object_flags+['-c',str(source),'-o',str(obj)],check=True,env=env)
    command=[compiler,*flags,*map(str,objects),'-o',str(artifact)]+(['-Wl,/NODEFAULTLIB:libcmt'] if args.sanitize and os.name=='nt' else [])
    subprocess.run(command,check=True,env=env)
    names=('interleave_credit','disconnect_reuse','competing_loan','rx_checkpoint')
    if args.sanitize:
        subprocess.run([str(artifact)],check=True,timeout=60,env=env)
    else:
        dll=ctypes.CDLL(str(artifact))
        for name in names:
            line=getattr(dll,'avrcp_runtime_'+name)()
            if line: raise SystemExit(f'FAIL {name} line {line}')
            print('PASS',name)
    assert initial_hashes=={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked}
    hashes={p.relative_to(root).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked}
    report={'passed':True,'scope':'paired simulated controllers using owned HCI ACL L2CAP runtime and real ATT/AVRCP bearers; no RF or hardware',
            'sanitizers':'address,undefined' if args.sanitize else None,'tests':list(names),'sha256':hashes}
    suffix=('windows-asan' if os.name=='nt' else 'linux-asan') if args.sanitize else ('windows' if os.name=='nt' else 'linux')
    (out/('runtime-'+suffix+'.json')).write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    # Retire only this passing run's typed host objects. Tested artifacts and
    # every failed receipt remain available; target/ELF objects are untouched.
    retired=[]
    if os.name=='nt':
        for obj in objects:
            if obj.resolve().parent!=out:raise RuntimeError('Object outside test output')
            data=obj.read_bytes()
            if int.from_bytes(data[:2],'little')!=0x8664:raise RuntimeError('Not an AMD64 COFF intermediate')
            retired.append({'path':str(obj),'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest()})
        manifest=out/'passing-host-intermediates.json'
        manifest.write_text(json.dumps({'program_tests_passed':True,'artifact':str(artifact),'objects':retired},indent=2)+'\n')
        for obj in objects:obj.unlink()
finally:
    subprocess.run(check,check=True)
