import json,os,subprocess,shutil,dbus
os.environ['XDG_RUNTIME_DIR']='/run/user/'+str(os.getuid())
os.environ['DBUS_SESSION_BUS_ADDRESS']='unix:path='+os.environ['XDG_RUNTIME_DIR']+'/bus'
report={}
p=subprocess.run(['ps','-u',str(os.getuid()),'-o','pid=,comm='],capture_output=True,text=True,timeout=8)
report['desktop_processes']=[line for line in p.stdout.splitlines() if any(v in line.lower() for v in ['gecko','firefox','gnome','xorg','xwayland'])]
report['programs']={name:shutil.which(name) for name in ['firefox','xdotool','wmctrl','xwininfo','gdbus','gio']}
session=dbus.SessionBus()
report['media_buses']=[str(name) for name in session.list_names() if any(v in str(name).lower() for v in ['mozilla','firefox','mpris','a11y'])]
try:
    import gi
    gi.require_version('Atspi','2.0')
    from gi.repository import Atspi
    desktop=Atspi.get_desktop(0)
    applications=[]
    for index in range(min(desktop.get_child_count(),32)):
        child=desktop.get_child_at_index(index)
        applications.append({'index':index,'name':child.get_name(),'role':child.get_role_name()})
    report['accessible_applications']=applications
except Exception as error:report['accessibility_error']=str(error)
system=dbus.SystemBus();objects=dbus.Interface(system.get_object('org.bluez','/'),'org.freedesktop.DBus.ObjectManager').GetManagedObjects()
device='/org/bluez/hci0/dev_6E_C8_40_42_BF_66'
report['device']={key:str(value) for key,value in objects.get(device,{}).get('org.bluez.Device1',{}).items() if key in ['Name','Connected','ServicesResolved','Address']}
for path,interfaces in objects.items():
    characteristic=interfaces.get('org.bluez.GattCharacteristic1',{})
    if str(path).startswith(device+'/') and str(characteristic.get('UUID',''))=='12345678-1234-1234-1234-123456789002':
        try:
            value=dbus.Interface(system.get_object('org.bluez',path),'org.bluez.GattCharacteristic1').ReadValue({},timeout=8)
            report['status_read_hex']=bytes(value).hex()
        except Exception as error:report['status_read_error']=str(error)
print(json.dumps(report,indent=2))
