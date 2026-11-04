"""Upload explicit clock-test inputs and run Linux ASan/UBSan, without radio access."""
import argparse,getpass,hashlib,json,shlex,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];WS=ROOT.parent
p=argparse.ArgumentParser();p.add_argument('--host',required=True);p.add_argument('--user',required=True);p.add_argument('--suite',choices=['clock','ble_control'],default='clock');args=p.parse_args()
runner='test_audio_clock.py' if args.suite=='clock' else 'test_ble_control.py'
raw=subprocess.check_output([sys.executable,str(ROOT/'tools'/runner),'--list-inputs'],text=True)
names=raw.splitlines() if args.suite=='clock' else json.loads(raw)['workspace_relative_inputs']
cache='clock_tests' if args.suite=='clock' else 'ble_control_tests'
assert 0<len(names)<64 and len(names)==len(set(names));snapshot={}
for name in names:
    f=(WS/name).resolve();f.relative_to(WS);assert f.is_file() and not f.is_symlink() and '_build' not in f.parts
    snapshot[name]=(hashlib.sha256(f.read_bytes()).hexdigest(),f.stat().st_mtime_ns)
sys.path.insert(0,str(ROOT/'_build/python_deps'));import paramiko
client=paramiko.SSHClient();client.load_host_keys(str(ROOT/'_build/linux_known_hosts'));client.set_missing_host_key_policy(paramiko.RejectPolicy())
secret=getpass.getpass('Linux SSH password (session only): ')
try:
    client.connect(args.host,username=args.user,password=secret,look_for_keys=False,allow_agent=False,timeout=8);secret=None
    _,out,_=client.exec_command("python3 -c 'import os;print(os.path.expanduser(\"~/.cache/native_bt/"+cache+"\"))'")
    remote=out.read().decode().strip();assert remote.startswith('/home/') and remote.endswith('/.cache/native_bt/'+cache)
    dirs=sorted({str(Path(n).parent).replace('\\','/') for n in names})
    _,out,err=client.exec_command('mkdir -p '+' '.join(shlex.quote(remote+'/'+d) for d in dirs),timeout=8)
    if out.channel.recv_exit_status():raise RuntimeError(err.read().decode())
    sftp=client.open_sftp()
    for name in names:
        with sftp.file(remote+'/'+name,'wb') as h:h.write((WS/name).read_bytes())
    _,out,err=client.exec_command('cd '+shlex.quote(remote)+' && python3 native_bt/tools/'+runner+' --sanitize',timeout=240)
    output=out.read(50000).decode();error=err.read(20000).decode();code=out.channel.recv_exit_status()
    for name,(digest,mtime) in snapshot.items():
        f=WS/name;assert hashlib.sha256(f.read_bytes()).hexdigest()==digest and f.stat().st_mtime_ns==mtime,name
    report={'passed':code==0,'scope':'Linux '+args.suite+' ASan+UBSan only, no hardware/RF/acoustics','exit_code':code,'output':output[-16000:],'error':error[-16000:],'uploaded_sha256':{n:v[0] for n,v in snapshot.items()},'inputs_stable':True}
    destination=ROOT/('_build/avrcp/clock-linux-sanitized.json' if args.suite=='clock' else '_build/ble_control/linux-sanitized.json');destination.write_text(json.dumps(report,indent=2)+'\n')
    print(output[-14000:]);print(error[-4000:]);print('receipt:',destination)
    if code:raise SystemExit(code)
finally:
    secret=None;client.close()
