"""Standalone owned ERTM tests; does not compile concurrently edited modules."""
import argparse
import ctypes
import getpass
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
TESTS=['ertm_wire_golden','ertm_wire_all_enhanced_fields','ertm_wire_transactional_bounds','ertm_engine_window_wrap_and_lazy_storage','ertm_engine_reject_recovers_lost_frame','ertm_engine_selective_ack_and_pinned_loan','ertm_engine_poll_monitor_and_retry_limits','ertm_engine_failure_retains_prepared_views','ertm_engine_reject_poll_and_invalid_final_ack','ertm_engine_receive_window_epoch_and_rej_pin','ertm_engine_large_window_ambiguous_gap_recovers','ertm_engine_busy_recovery_and_timer_progress','ertm_engine_timer_wrap_and_infinite_retry','ertm_engine_crc_sar_oom_and_ack_failure']
FILES=['include/dbt_ertm.h','include/dbt_ertm_engine.h','include/dbt_queue.h','src/dbt_ertm.c','src/dbt_ertm_engine.c','tests/ertm/wire.c','tests/ertm/engine.c','tools/test_ertm.py','tools/check_workspace.py','tools/test_ertm_audit.py','tests/ertm/audit_engine.c','tests/ertm/audit_main.c']

def hashes():
    return {name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in FILES}

def run():
    parser=argparse.ArgumentParser();parser.add_argument('--host');parser.add_argument('--user');args=parser.parse_args()
    before=hashes();folder=ROOT/'_build/ertm';folder.mkdir(parents=True,exist_ok=True)
    if args.host:
        if not args.user:raise RuntimeError('--user is required with --host')
        sys.path.insert(0,str(ROOT/'_build/python_deps'));import paramiko
        client=paramiko.SSHClient();client.load_host_keys(str(ROOT/'_build/linux_known_hosts'));client.set_missing_host_key_policy(paramiko.RejectPolicy())
        password=getpass.getpass('Linux SSH password (session only): ')
        try:
            client.connect(args.host,username=args.user,password=password,look_for_keys=False,allow_agent=False,timeout=8);password=None
            _,out,err=client.exec_command('mkdir -p ~/.cache/native_bt/ertm_tests/include ~/.cache/native_bt/ertm_tests/src ~/.cache/native_bt/ertm_tests/tests/ertm ~/.cache/native_bt/ertm_tests/tools',timeout=8)
            if out.channel.recv_exit_status():raise RuntimeError(err.read().decode())
            _,out,_=client.exec_command("python3 -c 'import os;print(os.path.expanduser(\"~/.cache/native_bt/ertm_tests\"))'")
            remote=out.read().decode().strip();assert remote.startswith('/home/') and remote.endswith('/.cache/native_bt/ertm_tests')
            sftp=client.open_sftp()
            for name in FILES:
                with sftp.file(remote+'/'+name,'wb') as handle:handle.write((ROOT/name).read_bytes())
            _,out,err=client.exec_command('cd ~/.cache/native_bt/ertm_tests && python3 tools/test_ertm.py',timeout=100)
            text=out.read(16000).decode();error=err.read(16000).decode()
            if out.channel.recv_exit_status():raise RuntimeError(error or text)
            with sftp.file(remote+'/_build/ertm/results.json','r') as handle:report=json.loads(handle.read())
            assert report['source_sha256']==before and hashes()==before
            (folder/'linux_results.json').write_text(json.dumps(report,indent=2)+'\n')
            with sftp.file(remote+'/_build/ertm/audit-linux-asan.json','r') as handle:audit=json.loads(handle.read())
            assert audit['passed'] and audit['source_stable']
            (folder/'audit-linux-asan.json').write_text(json.dumps(audit,indent=2)+'\n');print(text)
        finally:
            password=None;client.close()
        return
    compiler=shutil.which('clang') or shutil.which('gcc')
    if not compiler and os.name=='nt':compiler=str(Path(os.environ['ProgramFiles'])/'LLVM/bin/clang.exe')
    if not compiler:raise RuntimeError('C11 compiler required')
    output=folder/('wire.dll' if os.name=='nt' else 'wire.so')
    command=[compiler,'-std=c11','-O2','-Wall','-Wextra','-Werror','-I',str(ROOT/'include'),str(ROOT/'src/dbt_ertm.c'),str(ROOT/'src/dbt_ertm_engine.c'),str(ROOT/'tests/ertm/wire.c'),str(ROOT/'tests/ertm/engine.c')]
    if os.name=='nt':command+=['-ffreestanding','-fno-builtin','-fno-stack-protector','-fuse-ld=lld','-nostdlib','-shared','-Wl,/noentry']
    else:command+=['-shared','-fPIC']
    subprocess.run(command+['-o',str(output)],check=True)
    lib=ctypes.CDLL(str(output));rows=[]
    for name in TESTS:
        result=getattr(lib,name)();rows.append({'name':name,'passed':not result,'failed_line':result or None})
        print(('PASS ' if not result else 'FAIL ')+name+((' line '+str(result)) if result else ''),flush=True)
    sanitized=False
    if os.name!='nt':
        source=folder/'sanitizer_main.c';source.write_text(''.join('int '+name+'(void);\n' for name in TESTS)+'int main(void){\n'+''.join('if('+name+'())return 1;\n' for name in TESTS)+'return 0;}\n')
        binary=folder/'sanitized'
        subprocess.run([compiler,'-std=c11','-O1','-g','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fno-omit-frame-pointer','-I',str(ROOT/'include'),str(ROOT/'src/dbt_ertm.c'),str(ROOT/'src/dbt_ertm_engine.c'),str(ROOT/'tests/ertm/wire.c'),str(ROOT/'tests/ertm/engine.c'),str(source),'-o',str(binary)],check=True)
        subprocess.run([str(binary)],check=True,timeout=30);sanitized=True
        subprocess.run([sys.executable,str(ROOT/'tools/test_ertm_audit.py'),'--sanitize','--cc',compiler],check=True,timeout=120)
    assert hashes()==before,'Sources changed during build/test; receipt would be ambiguous'
    report={'scope':'owned ERTM wire and state engine tests; no channel/runtime negotiation or RF coverage','passed':all(x['passed'] for x in rows),'tests':rows,'enhanced_control_cases':67072,'address_undefined_sanitizers':sanitized,'source_sha256':before}
    (folder/'results.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k!='source_sha256'},indent=2))
    if not report['passed']:raise SystemExit(1)

def main():
    check=[sys.executable,str(ROOT/'tools/check_workspace.py')]
    subprocess.run(check,check=True)
    try:run()
    finally:subprocess.run(check,check=True)

if __name__=='__main__':main()
