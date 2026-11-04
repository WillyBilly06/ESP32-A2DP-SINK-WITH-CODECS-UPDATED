import dbus, json, shutil, socket, subprocess
from xml.etree import ElementTree
bus=dbus.SystemBus()
device='/org/bluez/hci0/dev_6C_C8_40_42_BF_66'
objects=dbus.Interface(bus.get_object('org.bluez','/'),'org.freedesktop.DBus.ObjectManager').GetManagedObjects()
report={'tools':{name:shutil.which(name) for name in ['btmon','btmgmt','bluetoothctl']}}
report['device']={key:str(value) for key,value in objects.get(device,{}).get('org.bluez.Device1',{}).items() if key in ['Name','Address','AddressType','Connected','ServicesResolved','Paired','Bonded','PreferredBearer','UUIDs']}
report['interfaces']=list(map(str,objects.get(device,{}).keys()))
for label,path in [('adapter','/org/bluez/hci0'),('device',device)]:
    try:
        text=dbus.Interface(bus.get_object('org.bluez',path),'org.freedesktop.DBus.Introspectable').Introspect(timeout=5)
        root=ElementTree.fromstring(str(text))
        report[label+'_methods']={x.attrib['name']:[v.attrib['name'] for v in x.findall('method')] for x in root.findall('interface') if x.attrib.get('name','').startswith('org.bluez')}
    except Exception as error:report[label+'_introspection_error']=str(error)
try:
    handle=socket.socket(socket.AF_BLUETOOTH,socket.SOCK_RAW,socket.BTPROTO_HCI)
    handle.close();report['hci_monitor_access']=True
except Exception as error:report['hci_monitor_access']=str(error)
p=subprocess.run(['journalctl','-u','bluetooth','--since','-8min','--no-pager','-o','cat'],capture_output=True,text=True,timeout=8)
report['bluetooth_journal']=[line[:300] for line in p.stdout.splitlines() if '6C:C8:40:42:BF:66' in line.upper() or any(v in line for v in ['hci0: Failed','ATT','GATT'])][-30:]
report['journal_error']=p.stderr[:300]
print(json.dumps(report,indent=2))
