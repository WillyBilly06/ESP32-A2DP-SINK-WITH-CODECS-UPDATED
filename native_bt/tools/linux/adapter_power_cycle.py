"""Cycle only the admitted idle test radio, leaving saved devices/keys intact."""
import dbus,json,time
bus=dbus.SystemBus()
manager=dbus.Interface(bus.get_object('org.bluez','/'),'org.freedesktop.DBus.ObjectManager')
objects=manager.GetManagedObjects()
path='/org/bluez/hci0';adapter=objects[path]['org.bluez.Adapter1']
assert str(adapter['Address'])=='E8:47:3A:EC:51:9A'
assert bool(adapter['Powered']),'Preserve a radio that was already off'
assert not any(v.get('org.bluez.Device1',{}).get('Connected') for v in objects.values()),'Preserve active sessions'
properties=dbus.Interface(bus.get_object('org.bluez',path),'org.freedesktop.DBus.Properties')
report={'scope':'idle test-adapter power-cycle; no saved device/key removal','powered_before':True}
try:
    properties.Set('org.bluez.Adapter1','Powered',dbus.Boolean(False),timeout=10)
    assert not properties.Get('org.bluez.Adapter1','Powered',timeout=5)
    report['off_observed']=True
    time.sleep(.5)
finally:
    properties.Set('org.bluez.Adapter1','Powered',dbus.Boolean(True),timeout=10)
    deadline=time.monotonic()+10
    while time.monotonic()<deadline:
        if properties.Get('org.bluez.Adapter1','Powered',timeout=5):break
        time.sleep(.1)
    report['powered_after']=bool(properties.Get('org.bluez.Adapter1','Powered',timeout=5))
    assert report['powered_after']
    assert str(properties.Get('org.bluez.Adapter1','Address',timeout=5))=='E8:47:3A:EC:51:9A'
    print(json.dumps(report,indent=2))
