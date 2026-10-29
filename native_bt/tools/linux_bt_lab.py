"""Temporarily expose BlueZ's separate bearer API; restore exact runtime settings."""
import argparse
import getpass
import json
from pathlib import Path
import shlex
import sys

ROOT=Path(__file__).resolve().parents[1]
REMOTE=r'''
import dbus,hashlib,json,os,pathlib,pwd,re,select,subprocess,sys,time
action=sys.argv[1]
override=pathlib.Path('/run/systemd/system/bluetooth.service.d/90-native-bt-lab.conf')
state=pathlib.Path('/run/native-bt-lab-state.json')
def run(args):
 p=subprocess.run(args,capture_output=True,text=True,timeout=20)
 if p.returncode: raise RuntimeError('Command failed: '+args[0]+' '+p.stderr[:300])
 return p.stdout.strip()
def argv():
 pid=int(run(['systemctl','show','bluetooth.service','-p','MainPID','--value']))
 return [x.decode() for x in pathlib.Path('/proc/'+str(pid)+'/cmdline').read_bytes().split(b'\0') if x]
def connections():
 bus=dbus.SystemBus()
 objects=dbus.Interface(bus.get_object('org.bluez','/'),'org.freedesktop.DBus.ObjectManager').GetManagedObjects()
 return [str(path) for path,interfaces in objects.items() if interfaces.get('org.bluez.Device1',{}).get('Connected')]
def restart():
 run(['systemctl','daemon-reload']);run(['systemctl','restart','bluetooth.service'])
 for unused in range(20):
  if run(['systemctl','is-active','bluetooth.service'])=='active':time.sleep(.2);return
  time.sleep(.1)
 raise RuntimeError('Bluetooth daemon did not return active')
if os.geteuid()!=0: raise RuntimeError('Runtime service override requires authorized sudo authentication')
active=connections()
if action=='inspect':
 info=pathlib.Path('/var/lib/bluetooth/E8:47:3A:EC:51:9A/6C:C8:40:42:BF:66/info')
 key_section=False
 if info.is_file() and not info.is_symlink():key_section='[LinkKey]' in info.read_text()
 print(json.dumps({'action':action,'argv':argv(),'active_connections':len(active),'owned_override_exists':override.exists(),
  'diagnostic_peer_info_exists':info.exists(),'diagnostic_peer_link_key_section_exists':key_section},indent=2));raise SystemExit(0)
if action in ['trace','trace_acl']:
 if active:raise RuntimeError('Start the bounded trace before initiating the owned connection')
 account=pwd.getpwnam(os.environ['SUDO_USER'])
 folder=pathlib.Path(account.pw_dir)/'.cache/native_bt/stack_tests'
 if folder.resolve()!=folder or not folder.is_dir():raise RuntimeError('Unexpected capture directory')
 destination=folder/('classic-root-trace-filtered.txt' if action=='trace_acl' else 'le-root-trace-filtered.txt')
 previous=folder/('classic-root-trace-before-readmission.txt' if action=='trace_acl' else 'le-root-trace-no-create.txt')
 if destination.exists() and not previous.exists():
  previous.write_bytes(destination.read_bytes());os.chown(previous,account.pw_uid,account.pw_gid)
 descriptor=os.open(destination,os.O_WRONLY|os.O_CREAT|os.O_TRUNC|os.O_NOFOLLOW,0o600)
 os.fchown(descriptor,account.pw_uid,account.pw_gid)
 monitor=subprocess.Popen(['btmon','-i','0'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
 capture=bytearray();pending=bytearray();block=bytearray();key_events=[];classic_handles=set();deadline=time.monotonic()+55
 def flush(handle):
  if not block:return
  if b'Connect Complete' in block and b'Link type: ACL (0x01)' in block:
   handle_match=re.search(rb'Handle: (\d+)',block)
   if handle_match:
    if b'6C:C8:40:42:BF:66' in block.upper():classic_handles.add(int(handle_match.group(1)))
    else:classic_handles.discard(int(handle_match.group(1)))
  if (b'Link Key' in block or b'Link key' in block) and len(key_events)<32:
   # Store-hint/type metadata only; never preserve the link key itself.
   metadata=[line.strip() for line in block.decode('utf-8',errors='replace').splitlines()
    if line.strip().startswith(('Store hint:','Key type:','Key Type:'))]
   if metadata:key_events.append(metadata)
  ignored=any(value in block for value in [b'Advertising Report',b'Device Found',b'Number of Completed Packets',
   b'Link Key',b'Link key',b'Long Term Key',b'Long term key',b'LTK',b'IRK',b'CSRK',b'PIN Code',b'Passkey',b'OOB Data',b'User Confirmation',b'SMP:',b'Security Manager'])
  if b'ACL Data' in block and not any(value in block for value in [b'Connection Request',b'Connection Response',
   b'Configuration Request',b'Configuration Response',b'Disconnection Request',b'Disconnection Response',b'Command Reject',b'Information Request',b'Information Response']):
   handle_match=re.search(rb'ACL Data (?:TX|RX): Handle (\d+)',block)
   if action!='trace_acl' or not handle_match or int(handle_match.group(1)) not in classic_handles:ignored=True
  if not ignored:
   data=bytes(block[:524288-len(capture)]);capture.extend(data);handle.write(data)
  if b'Disconnect Complete' in block:
   handle_match=re.search(rb'Handle: (\d+)',block)
   if handle_match:classic_handles.discard(int(handle_match.group(1)))
  block.clear()
 try:
  with os.fdopen(descriptor,'wb',buffering=0) as handle:
   while time.monotonic()<deadline and len(capture)<524288:
    ready,unused,unused=select.select([monitor.stdout],[],[],.2)
    if not ready:continue
    data=os.read(monitor.stdout.fileno(),min(4096,524288-len(capture)))
    if not data:break
    pending.extend(data)
    while b'\n' in pending:
     index=pending.index(b'\n')+1;line=bytes(pending[:index]);del pending[:index]
     if re.match(rb'^(?:[^\s:][^\n]*?: )?[<>@=*+] ',line):flush(handle)
     block.extend(line)
     if len(block)>16384:flush(handle)
   block.extend(pending);flush(handle)
 finally:
  monitor.terminate()
  try:monitor.wait(timeout=3)
  except subprocess.TimeoutExpired:monitor.kill();monitor.wait(timeout=3)
 lines=capture.decode('utf-8',errors='replace').splitlines();selected=set()
 for index,line in enumerate(lines):
  if '6C:C8:40:42:BF:66' in line.upper() or any(text in line for text in ['Create Connection','Connection Complete','Connect Failed','Disconnect Complete']):
   selected.update(range(max(0,index-1),min(len(lines),index+9)))
 print(json.dumps({'action':action,'bytes':len(capture),'capture_sha256':hashlib.sha256(capture).hexdigest(),'key_event_metadata':key_events,'connection_trace':[lines[i] for i in sorted(selected)][:180]},indent=2));raise SystemExit(0)
if active:raise RuntimeError('Bluetooth is in use; keep current connections and do not restart')
if override.is_symlink() or state.is_symlink():raise RuntimeError('Unexpected runtime symlink')
if action=='refresh':
 if not override.exists() or not state.exists():raise RuntimeError('No owned runtime test configuration to refresh')
 saved=json.loads(state.read_text())
 if hashlib.sha256(override.read_bytes()).hexdigest()!=saved['override_sha256']:raise RuntimeError('Owned override changed')
 if argv()!=saved['original_argv']+['--experimental']:raise RuntimeError('Test service command changed')
 restart()
 print(json.dumps({'action':action,'active_connections_before':0,'current_argv':argv()},indent=2))
elif action=='enable':
 original=argv()
 if original[0] not in ['/usr/libexec/bluetooth/bluetoothd','/usr/lib/bluetooth/bluetoothd'] or any(arg not in ['-n','--nodetach'] for arg in original[1:]):
  raise RuntimeError('Unexpected Bluetooth service command; preserve it for review')
 if override.exists() or state.exists():raise RuntimeError('Prior runtime override requires inspection/restoration')
 content='[Service]\nExecStart=\nExecStart='+' '.join(original+['--experimental'])+'\n'
 override.parent.mkdir(parents=True,exist_ok=True)
 if override.parent.resolve()!=pathlib.Path('/run/systemd/system/bluetooth.service.d'):raise RuntimeError('Runtime path escaped its admitted directory')
 saved={'original_argv':original,'override_sha256':hashlib.sha256(content.encode()).hexdigest()}
 descriptor=os.open(state,os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW,0o600)
 with os.fdopen(descriptor,'w') as handle:json.dump(saved,handle)
 descriptor=os.open(override,os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW,0o644)
 with os.fdopen(descriptor,'w') as handle:handle.write(content)
 restart()
 if '--experimental' not in argv():raise RuntimeError('Experimental runtime option not observed')
 print(json.dumps({'action':action,'runtime_only':True,'original_argv':original,'current_argv':argv(),'active_connections_before':0},indent=2))
elif action=='restore':
 if not override.exists() and not state.exists():
  print(json.dumps({'action':action,'already_restored':True,'argv':argv()},indent=2));raise SystemExit(0)
 saved=json.loads(state.read_text())
 if hashlib.sha256(override.read_bytes()).hexdigest()!=saved['override_sha256']:raise RuntimeError('Owned override changed; preserve it')
 override.unlink();restart()
 current=argv()
 if current!=saved['original_argv']:raise RuntimeError('Service command differs from original after restoration')
 state.unlink()
 print(json.dumps({'action':action,'restored':True,'argv':current},indent=2))
else:raise RuntimeError('Unknown lab action')
'''


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--host',required=True)
    parser.add_argument('--user',required=True)
    parser.add_argument('action',choices=['inspect','enable','refresh','restore','trace','trace_acl'])
    args=parser.parse_args()
    sys.path.insert(0,str(ROOT/'_build/python_deps'))
    import paramiko
    password=getpass.getpass('Linux SSH/sudo password (session only): ')
    client=paramiko.SSHClient()
    client.load_host_keys(str(ROOT/'_build/linux_known_hosts'))
    client.set_missing_host_key_policy(paramiko.RejectPolicy())
    try:
        client.connect(args.host,username=args.user,password=password,look_for_keys=False,allow_agent=False,timeout=8,auth_timeout=10)
        command="sudo -S -p '' -- python3 -c "+shlex.quote(REMOTE)+' '+args.action
        stdin,stdout,stderr=client.exec_command(command,timeout=75)
        stdin.write(password+'\n');stdin.flush();stdin.channel.shutdown_write()
        password=None
        output=stdout.read(65536).decode();errors=stderr.read(8192).decode();status=stdout.channel.recv_exit_status()
        report={'action':args.action,'return_code':status,'stderr':errors}
        try:report['result']=json.loads(output)
        except ValueError:report['stdout']=output
        destination=ROOT/'_build/hardware'/('linux-bt-lab-'+args.action+'.json')
        destination.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
        print(json.dumps(report,indent=2));return status
    finally:
        password=None;client.close()


if __name__=='__main__':raise SystemExit(main())
