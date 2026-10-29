"""Run explicit offline test inputs on the authorized Linux VM."""
import argparse,getpass,hashlib,json,shlex,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
SUITES={'avrcp_runtime':('tools/test_avrcp_runtime.py',['--sanitize','--cc','gcc'],'_build/avrcp/runtime-linux-asan.json'),
        'ertm_audit':('tools/test_ertm_audit.py',['--sanitize','--cc','gcc'],'_build/ertm/audit-linux-asan.json')}
def main():
    a=argparse.ArgumentParser();a.add_argument('--host',required=True);a.add_argument('--user',required=True);a.add_argument('--suite',choices=SUITES,required=True);args=a.parse_args()
    runner,flags,receipt=SUITES[args.suite]
    names=json.loads(subprocess.check_output([sys.executable,str(ROOT/runner),'--list-inputs'],text=True))
    assert names and len(names)==len(set(names));before={}
    for name in names:
        p=(ROOT/name).resolve();p.relative_to(ROOT);assert p.is_file() and '_build' not in p.parts and not p.is_symlink()
        before[name]=(hashlib.sha256(p.read_bytes()).hexdigest(),p.stat().st_mtime_ns)
    sys.path.insert(0,str(ROOT/'_build/python_deps'));import paramiko
    client=paramiko.SSHClient();client.load_host_keys(str(ROOT/'_build/linux_known_hosts'));client.set_missing_host_key_policy(paramiko.RejectPolicy())
    password=getpass.getpass('Linux SSH password (session only): ')
    try:
        client.connect(args.host,username=args.user,password=password,look_for_keys=False,allow_agent=False,timeout=8);password=None
        _,out,_=client.exec_command("python3 -c 'import os;print(os.path.expanduser(\"~/.cache/native_bt/module_checks\"))'")
        remote=out.read().decode().strip();assert remote.startswith('/home/') and remote.endswith('/.cache/native_bt/module_checks')
        directories=sorted({str(Path(name).parent).replace('\\','/') for name in names})
        _,out,err=client.exec_command('mkdir -p '+' '.join(shlex.quote(remote+'/'+d) for d in directories),timeout=10)
        if out.channel.recv_exit_status():raise RuntimeError(err.read().decode())
        sftp=client.open_sftp()
        for name in names:
            with sftp.file(remote+'/'+name,'wb') as h:h.write((ROOT/name).read_bytes())
        command='cd '+shlex.quote(remote)+' && python3 '+shlex.quote(runner)+' '+' '.join(shlex.quote(f) for f in flags)
        _,out,err=client.exec_command(command,timeout=240)
        output=out.read(80000).decode();error=err.read(20000).decode()
        if out.channel.recv_exit_status():raise RuntimeError(error or output)
        with sftp.file(remote+'/'+receipt,'r') as h:report=json.loads(h.read())
        assert report['passed']
        for name,(digest,mtime) in before.items():
            p=ROOT/name;assert hashlib.sha256(p.read_bytes()).hexdigest()==digest and p.stat().st_mtime_ns==mtime,name
        report['uploaded_sha256']={name:v[0] for name,v in before.items()};report['local_inputs_stable_during_remote_run']=True
        destination=ROOT/receipt;destination.parent.mkdir(parents=True,exist_ok=True);destination.write_text(json.dumps(report,indent=2)+'\n')
        print(output);print(json.dumps({'suite':args.suite,'passed':True,'receipt':receipt,'uploaded_files':len(names)},indent=2))
    finally:
        password=None;client.close()
if __name__=='__main__':main()
