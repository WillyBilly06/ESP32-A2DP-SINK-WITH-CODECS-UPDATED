"""Remove only the newly created diagnostic peer after its native bond delete."""
import dbus
import json

bus = dbus.SystemBus()
objects = dbus.Interface(bus.get_object('org.bluez', '/'),
                         'org.freedesktop.DBus.ObjectManager').GetManagedObjects()
adapter_path = '/org/bluez/hci0'
path = adapter_path + '/dev_6C_C8_40_42_BF_66'
assert str(objects[adapter_path]['org.bluez.Adapter1']['Address']) == 'E8:47:3A:EC:51:9A'
assert not any(v.get('org.bluez.Device1', {}).get('Connected') for v in objects.values()), 'Preserve active sessions'
properties = objects.get(path, {}).get('org.bluez.Device1', {})
report = {'scope': 'cleanup of the diagnostic peer created by these tests',
          'target_present': bool(properties), 'removed': False}
if properties:
    assert str(properties.get('Address')) == '6C:C8:40:42:BF:66'
    assert str(properties.get('Name')) == 'NATIVE_BT_PROBE', 'Preserve an unexpected device identity'
    dbus.Interface(bus.get_object('org.bluez', adapter_path),
                   'org.bluez.Adapter1').RemoveDevice(path, timeout=8)
    report['removed'] = True
print(json.dumps(report, indent=2))
