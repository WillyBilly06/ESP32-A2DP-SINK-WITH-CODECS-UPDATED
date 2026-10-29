"""Independent ERTM audit. Offline only, fixed outputs and source-hash receipt."""
import argparse, ctypes, hashlib, json, os, pathlib, shutil, subprocess, sys
root=pathlib.Path(__file__).resolve().parents[1]
build=root/'_build'/'ertm'
build.mkdir(parents=True,exist_ok=True)
parser=argparse.ArgumentParser()
parser.add_argument('--sanitize',action='store_true')
parser.add_argument('--cc')
parser.add_argument('--list-inputs',action='store_true')
args=parser.parse_args()
if args.sanitize and os.name=='nt': raise SystemExit('--sanitize requires Linux')
compiler=args.cc or os.environ.get('CC') or shutil.which('clang') or shutil.which('gcc')
if not compiler and os.name=='nt': compiler=str(pathlib.Path(os.environ['ProgramFiles'])/'LLVM/bin/clang.exe')
if not compiler: raise SystemExit('C11 compiler required')
files=['src/dbt_ertm.c','src/dbt_ertm_engine.c','include/dbt_ertm.h','include/dbt_ertm_engine.h','include/dbt_queue.h','tests/ertm/audit_engine.c','tests/ertm/audit_main.c','tools/test_ertm_audit.py']
if args.list_inputs:
    print(json.dumps(files+['tools/check_workspace.py'],indent=2)); sys.exit(0)
def hashes(): return {f:hashlib.sha256((root/f).read_bytes()).hexdigest() for f in files}
names=('startup_window_matrix','duplicate_multiround','bidirectional_loss_all_windows','busy_poll_final','rej_poll_busy_window_matrix','repeated_loss_finite_recovery','repeated_poll_pins','gap_across_sequence_wrap','coalesced_final_causes_timeout','pinned_ack_receive_progress','bad_fcs_pinned_loan','infinite_retransmit_counter','seeded_loss_and_busy','pinned_teardown_matrix','retry_limit_during_other_pin','timer_wrap_and_exhaustion')
check=[sys.executable,str(root/'tools/check_workspace.py')]
subprocess.run(check,check=True)
before=hashes()
try:
    src=[root/'src/dbt_ertm.c',root/'src/dbt_ertm_engine.c',root/'tests/ertm/audit_engine.c']
    common=[compiler,'-std=c11','-O1','-g','-Wall','-Wextra','-Werror','-I',str(root/'include')]
    rows=[]
    if args.sanitize:
        artifact=build/'audit-sanitized'
        command=common+['-fsanitize=address,undefined','-fno-omit-frame-pointer']+[str(p) for p in src]+[str(root/'tests/ertm/audit_main.c'),'-o',str(artifact)]
        subprocess.run(command,check=True)
        run=subprocess.run([str(artifact)],capture_output=True,text=True,timeout=180)
        print(run.stdout,end=''); print(run.stderr,end='',file=sys.stderr)
        rows=[{'name':line.split()[1],'passed':line.startswith('PASS '),'detail':line} for line in run.stdout.splitlines() if line.startswith(('PASS ','FAIL '))]
        passed=run.returncode==0 and len(rows)==len(names) and all(row['passed'] for row in rows)
    else:
        artifact=build/('audit.dll' if os.name=='nt' else 'audit.so')
        flags=['-ffreestanding','-fno-builtin','-fno-stack-protector','-fuse-ld=lld','-nostdlib','-shared','-Wl,/noentry'] if os.name=='nt' else ['-shared','-fPIC']
        subprocess.run(common+flags+[str(p) for p in src]+['-o',str(artifact)],check=True)
        lib=ctypes.CDLL(str(artifact));
        for name in names:
            line=getattr(lib,'audit_'+name)()
            detail={'window':lib.audit_window(),'case':lib.audit_case(),'step':lib.audit_step(),'a':lib.audit_a(),'b':lib.audit_b()} if line else None
            rows.append({'name':name,'passed':not line,'line':line or None,'detail':detail})
            print(('FAIL ' if line else 'PASS ')+name+(f' line {line} {detail}' if line else ''),flush=True)
        passed=all(row['passed'] for row in rows)
    after=hashes()
    report={'scope':'independent owned ERTM engine and wire tests, no runtime integration or RF','passed':passed and before==after,
            'sanitizers':'address,undefined' if args.sanitize else None,'source_stable':before==after,'source_sha256':before,'tests':rows}
    suffix='linux-asan' if args.sanitize else ('windows' if os.name=='nt' else 'linux')
    (build/('audit-'+suffix+'.json')).write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    if not report['passed']:raise SystemExit(1)
finally:
    subprocess.run(check,check=True)
