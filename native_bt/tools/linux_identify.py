"""Correlate the known speaker with UART using an idle-only codec change."""
import argparse
import getpass
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
REMOTE=r'''
import json,os,subprocess,time,dbus
os.environ['XDG_RUNTIME_DIR']='/run/user/'+str(os.getuid())
os.environ['DBUS_SESSION_BUS_ADDRESS']='unix:path='+os.environ['XDG_RUNTIME_DIR']+'/bus'
def run(a):
 try:
  p=subprocess.run(a,capture_output=True,text=True,timeout=8)
  return {'rc':p.returncode,'out':p.stdout[:8000],'err':p.stderr[:1000]}
 except Exception as e:return {'error':str(e)}
def listing(kind):
 p=run(['pactl','--format=json','list',kind]);return json.loads(p['out'])
address='6E:C8:40:42:BF:66'
cards=[v for v in listing('cards') if v.get('properties',{}).get('api.bluez5.address')==address]
assert len(cards)==1 and cards[0]['properties']['device.description']=='BDK SPEAKER'
card=cards[0];sinks=[v for v in listing('sinks') if v.get('properties',{}).get('api.bluez5.address')==address]
assert len(sinks)==1 and sinks[0]['state']=='SUSPENDED','Identification profile change requires idle audio'
original=card['active_profile']
report={'address':address,'original_profile':original,'scope':'existing-firmware identity correlation, not a new-host test'}
report['getent']=run(['getent','passwd','billydakidz','BillyDaKidz'])
report['sessions']=run(['loginctl','list-sessions','--no-legend'])
report['processes']=run(['ps','-eo','pid,uid,comm'])
report['processes']['out']='\n'.join(line for line in report['processes'].get('out','').splitlines() if any(s in line.lower() for s in ['gecko','firefox','gnome-shell','xorg','xwayland']))
report['programs']=run(['sh','-c','command -v firefox xdotool wmctrl xwininfo gdbus gio'])
bus=dbus.SystemBus();objects=dbus.Interface(bus.get_object('org.bluez','/'),'org.freedesktop.DBus.ObjectManager').GetManagedObjects()
report['gatt']=[{'path':str(p),'uuid':str(v['org.bluez.GattCharacteristic1'].get('UUID','')),'flags':[str(x) for x in v['org.bluez.GattCharacteristic1'].get('Flags',[])]} for p,v in objects.items() if '/dev_6E_C8_40_42_BF_66/' in str(p) and 'org.bluez.GattCharacteristic1' in v]
report['volume_limit']=run(['pactl','set-sink-volume',sinks[0]['name'],'10%'])
assert report['volume_limit']['rc']==0
assert 'a2dp-sink-ldac' in card['profiles']
try:
 report['set_ldac']=run(['pactl','set-card-profile',card['name'],'a2dp-sink-ldac'])
 assert report['set_ldac']['rc']==0
 time.sleep(2)
 current=[v for v in listing('sinks') if v.get('properties',{}).get('api.bluez5.address')==address]
 report['ldac_sinks']=[{'name':v['name'],'state':v['state'],'sample':v['sample_specification'],'codec':v['properties'].get('api.bluez5.codec')} for v in current]
 for v in current:run(['pactl','set-sink-volume',v['name'],'10%'])
 time.sleep(5)
finally:
 report['restore_profile']=run(['pactl','set-card-profile',card['name'],original])
 time.sleep(2)
 for v in listing('sinks'):
  if v.get('properties',{}).get('api.bluez5.address')==address:run(['pactl','set-sink-volume',v['name'],'10%'])
print(json.dumps(report,indent=2))
'''

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--host',required=True);parser.add_argument('--user',required=True)
    args=parser.parse_args();sys.path.insert(0,str(ROOT/'_build/python_deps'))
    import paramiko
    password=getpass.getpass('Linux SSH password (session only): ')
    client=paramiko.SSHClient();client.load_host_keys(str(ROOT/'_build/linux_known_hosts'));client.set_missing_host_key_policy(paramiko.RejectPolicy())
    try:
        client.connect(args.host,username=args.user,password=password,look_for_keys=False,allow_agent=False,timeout=8,auth_timeout=10)
        password=None
        stdin,stdout,stderr=client.exec_command('python3 -',timeout=40)
        stdin.write(REMOTE);stdin.flush();stdin.channel.shutdown_write()
        result=stdout.read(100000).decode();errors=stderr.read(10000).decode()
        if stdout.channel.recv_exit_status():raise RuntimeError(errors or result)
        report=json.loads(result);destination=ROOT/'_build/hardware/linux-identification.json';destination.parent.mkdir(exist_ok=True,parents=True)
        destination.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
        print(json.dumps(report,indent=2))
    finally:password=None;client.close()

if __name__=='__main__':main()
