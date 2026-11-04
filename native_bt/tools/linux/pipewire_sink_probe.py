"""Bounded ordinary BlueZ Audio Sink connection and read-only PipeWire inspection."""
import dbus
import json
import os
import subprocess
import time

SOURCE = 'E8:47:3A:EC:51:9A'
TARGET = '6C:C8:40:42:BF:66'
DEVICE = '/org/bluez/hci0/dev_' + TARGET.replace(':', '_')
os.environ['XDG_RUNTIME_DIR'] = '/run/user/' + str(os.getuid())
os.environ['DBUS_SESSION_BUS_ADDRESS'] = 'unix:path=' + os.environ['XDG_RUNTIME_DIR'] + '/bus'
bus = dbus.SystemBus()
manager = dbus.Interface(bus.get_object('org.bluez', '/'), 'org.freedesktop.DBus.ObjectManager')
report = {'scope': 'ordinary BlueZ Audio Sink profile connection; no media playback', 'passed': False}
device = None
try:
    before = manager.GetManagedObjects()
    assert str(before['/org/bluez/hci0']['org.bluez.Adapter1']['Address']) == SOURCE
    assert not any(value.get('org.bluez.Device1', {}).get('Connected') for value in before.values())
    properties = before[DEVICE]['org.bluez.Device1']
    assert properties['Name'] == 'NATIVE_BT_PROBE'
    assert '0000110b-0000-1000-8000-00805f9b34fb' in list(properties['UUIDs'])
    assert properties['Paired'] and properties['Bonded']
    report['start_unix'] = time.time()
    device = dbus.Interface(bus.get_object('org.bluez', DEVICE), 'org.bluez.Device1')
    device.ConnectProfile('0000110b-0000-1000-8000-00805f9b34fb', timeout=15)
    deadline = time.monotonic() + 12
    while time.monotonic() < deadline:
        sinks = json.loads(subprocess.check_output(['pactl', '-f', 'json', 'list', 'sinks'], text=True, timeout=5))
        owned = [sink for sink in sinks if TARGET.replace(':', '_') in sink.get('name', '')]
        if owned:
            report['sinks'] = [{'index': sink['index'], 'name': sink['name'], 'state': sink['state'],
                               'sample_specification': sink['sample_specification'],
                               'codec': sink.get('properties', {}).get('api.bluez5.codec')}
                              for sink in owned]
            break
        time.sleep(.2)
    else:
        raise RuntimeError('No PipeWire Bluetooth output appeared')
    current = manager.GetManagedObjects()
    report['transports'] = [{'path': str(path), 'state': str(value['org.bluez.MediaTransport1']['State']),
                             'codec': int(value['org.bluez.MediaTransport1']['Codec']),
                             'configuration': bytes(value['org.bluez.MediaTransport1']['Configuration']).hex()}
                            for path, value in current.items()
                            if 'org.bluez.MediaTransport1' in value and str(path).startswith(DEVICE + '/')]
    report['passed'] = bool(report['transports'])
except Exception as error:
    report['error'] = str(error)
finally:
    if device:
        try:
            device.Disconnect(timeout=8)
            report['disconnected'] = True
        except Exception as error:
            report['disconnect_error'] = str(error)
    report['end_unix'] = time.time()
    print(json.dumps(report, indent=2))
raise SystemExit(0 if report['passed'] else 1)
