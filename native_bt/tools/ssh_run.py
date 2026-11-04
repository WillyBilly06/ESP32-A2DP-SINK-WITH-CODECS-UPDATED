"""Run a project-owned Linux test script; password is entered only into the session."""
import argparse
import getpass
import hashlib
import json
from pathlib import Path
import shlex
import sys

ROOT=Path(__file__).resolve().parents[1]

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--host',required=True);parser.add_argument('--user',required=True)
    parser.add_argument('--script',type=Path,required=True);parser.add_argument('--timeout',type=int,default=90)
    parser.add_argument('--record',type=Path)
    parser.add_argument('--firmware',type=Path)
    parser.add_argument('arguments',nargs=argparse.REMAINDER)
    args=parser.parse_args();script=args.script.resolve();script.relative_to(ROOT/'tools/linux')
    if not 1<=args.timeout<=900:raise RuntimeError('Use a bounded test timeout')
    firmware=None
    if args.firmware:
        firmware_path=args.firmware.resolve();firmware_path.relative_to(ROOT/'_build/hardware')
        current=json.loads(firmware_path.read_text(encoding='utf-8'))
        firmware={key:current[key] for key in ['scope','candidate_sha256','candidate_elf_sha256','boot_ready_marker']}
        if not firmware['boot_ready_marker']:raise RuntimeError('Candidate readiness has not been observed')
    sys.path.insert(0,str(ROOT/'_build/python_deps'));import paramiko
    password=getpass.getpass('Linux SSH password (session only): ')
    client=paramiko.SSHClient();client.load_host_keys(str(ROOT/'_build/linux_known_hosts'));client.set_missing_host_key_policy(paramiko.RejectPolicy())
    try:
        client.connect(args.host,username=args.user,password=password,look_for_keys=False,allow_agent=False,timeout=8,auth_timeout=10)
        password=None
        command='python3 - '+shlex.join(args.arguments[1:] if args.arguments[:1]==['--'] else args.arguments)
        stdin,stdout,stderr=client.exec_command(command,timeout=args.timeout)
        stdin.write(script.read_text(encoding='utf-8'));stdin.flush();stdin.channel.shutdown_write()
        output=stdout.read(2097153);errors=stderr.read(65537)
        if len(output)>2097152 or len(errors)>65536:raise RuntimeError('Remote test output exceeded its bound')
        return_code=stdout.channel.recv_exit_status()
        if args.record:
            record=args.record.resolve();record.relative_to(ROOT/'_build')
            record.parent.mkdir(parents=True,exist_ok=True)
            report={'script':script.relative_to(ROOT).as_posix(),'script_sha256':hashlib.sha256(script.read_bytes()).hexdigest(),'return_code':return_code,'stderr':errors.decode('utf-8',errors='backslashreplace')}
            if firmware:report['firmware']=firmware
            try:report['result']=json.loads(output)
            except ValueError:report['stdout']=output.decode('utf-8',errors='backslashreplace')
            record.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
        sys.stdout.reconfigure(encoding='utf-8',errors='backslashreplace')
        sys.stdout.write(output.decode('utf-8',errors='backslashreplace'));sys.stderr.write(errors.decode('utf-8',errors='backslashreplace'))
        return return_code
    finally:password=None;client.close()

if __name__=='__main__':raise SystemExit(main())
