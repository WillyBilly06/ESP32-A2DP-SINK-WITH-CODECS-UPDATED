"""Two ordinary BlueZ Audio Sink sessions, with a full disconnect between them."""
import dbus
import json
import os
import subprocess
import time

SOURCE='E8:47:3A:EC:51:9A'
DEVICE='/org/bluez/hci0/dev_6C_C8_40_42_BF_66'
os.environ['XDG_RUNTIME_DIR']='/run/user/'+str(os.getuid())
os.environ['DBUS_SESSION_BUS_ADDRESS']='unix:path='+os.environ['XDG_RUNTIME_DIR']+'/bus'
bus=dbus.SystemBus()
manager=dbus.Interface(bus.get_object('org.bluez','/'),'org.freedesktop.DBus.ObjectManager')
report={'scope':'two ordinary BlueZ/PipeWire profile sessions; no media playback','passed':False,'attempts':[], 'start_unix':time.time()}
device=None
try:
    before=manager.GetManagedObjects()
    assert str(before['/org/bluez/hci0']['org.bluez.Adapter1']['Address'])==SOURCE
    assert not any(value.get('org.bluez.Device1',{}).get('Connected') for value in before.values())
    assert before[DEVICE]['org.bluez.Device1']['Name']=='NATIVE_BT_PROBE'
    device=dbus.Interface(bus.get_object('org.bluez',DEVICE),'org.bluez.Device1')
    for index in range(2):
        attempt={'index':index,'start_unix':time.time()}
        report['attempts'].append(attempt)
        try:
            device.ConnectProfile('0000110b-0000-1000-8000-00805f9b34fb',timeout=15)
            current=manager.GetManagedObjects()
            attempt['transports']=[{'codec':int(value['org.bluez.MediaTransport1']['Codec']),
                'configuration':bytes(value['org.bluez.MediaTransport1']['Configuration']).hex()}
                for path,value in current.items() if 'org.bluez.MediaTransport1' in value and str(path).startswith(DEVICE+'/')]
            assert attempt['transports']
            attempt['passed']=True
        except Exception as error:
            attempt.update(passed=False,error=str(error))
            raise
        finally:
            device.Disconnect(timeout=8)
            attempt['end_unix']=time.time()
        deadline=time.monotonic()+5
        while manager.GetManagedObjects().get(DEVICE,{}).get('org.bluez.Device1',{}).get('Connected'):
            assert time.monotonic()<deadline,'Disconnect did not complete'
            time.sleep(.1)
        time.sleep(.5)
    report['passed']=True
except Exception as error:
    report['error']=str(error)
finally:
    if device:
        try:device.Disconnect(timeout=8)
        except Exception:pass
    report['end_unix']=time.time()
    print(json.dumps(report,indent=2))
raise SystemExit(0 if report['passed'] else 1)
