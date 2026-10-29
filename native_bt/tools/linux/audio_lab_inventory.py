"""Read-only checks for the user's desktop audio test environment."""
import json
import os
from pathlib import Path
import shutil
import subprocess

uid = os.getuid()
os.environ['XDG_RUNTIME_DIR'] = '/run/user/' + str(uid)
os.environ['DBUS_SESSION_BUS_ADDRESS'] = 'unix:path=/run/user/' + str(uid) + '/bus'
report = {'scope': 'read-only desktop/audio prerequisites', 'uid': uid}
for name in ['firefox', 'pw-dump', 'pactl', 'wpctl', 'ffmpeg']:
    report[name] = shutil.which(name)
sessions = []
for path in Path('/proc').iterdir():
    if not path.name.isdecimal():
        continue
    try:
        if path.stat().st_uid != uid:
            continue
        name = (path/'comm').read_text().strip()
        if name not in ['gnome-shell', 'firefox', 'firefox-bin', 'kwin_wayland', 'Xorg']:
            continue
        raw = (path/'environ').read_bytes()
        assert len(raw) < 65536
        environment = dict(value.split(b'=', 1) for value in raw.split(b'\0') if b'=' in value)
        sessions.append({'process': name, 'pid': int(path.name), 'environment': {
            key: environment[key.encode()].decode(errors='replace')
            for key in ['DISPLAY', 'WAYLAND_DISPLAY', 'XAUTHORITY', 'XDG_SESSION_TYPE']
            if key.encode() in environment}})
    except (PermissionError, FileNotFoundError, ProcessLookupError):
        pass
report['desktop_processes'] = sessions
try:
    report['default_sink'] = subprocess.check_output(['pactl', 'get-default-sink'], text=True, timeout=5).strip()
    for kind in ['sinks', 'sink-inputs']:
        values = json.loads(subprocess.check_output(['pactl', '-f', 'json', 'list', kind], text=True, timeout=5))
        report[kind] = [{'index': value['index'], 'name': value.get('name'),
                         'state': value.get('state'), 'sink': value.get('sink'),
                         'application': value.get('properties', {}).get('application.name')}
                        for value in values]
except Exception as error:
    report['audio_error'] = str(error)
try:
    import dbus
    bus = dbus.SystemBus()
    objects = dbus.Interface(bus.get_object('org.bluez', '/'), 'org.freedesktop.DBus.ObjectManager').GetManagedObjects()
    report['active_bluetooth_connections'] = sum(bool(value.get('org.bluez.Device1', {}).get('Connected'))
                                               for value in objects.values())
    target = '/org/bluez/hci0/dev_6C_C8_40_42_BF_66'
    report['target_interfaces'] = {str(interface): {str(key): str(value) for key, value in properties.items()
        if key in ['Connected', 'Paired', 'Bonded', 'ServicesResolved', 'PreferredBearer', 'AddressType']}
        for interface, properties in objects.get(target, {}).items()}
    report['audio_endpoints'] = [{'path': str(path), 'codec': int(value['org.bluez.MediaEndpoint1']['Codec'])}
                                 for path, value in objects.items() if 'org.bluez.MediaEndpoint1' in value]
except Exception as error:
    report['bluez_error'] = str(error)
for label, arguments in [('controller_connections', ['hcitool', 'con']),
                         ('bluetooth_journal', ['journalctl', '-u', 'bluetooth.service', '--since', '-4 min', '-n', '35', '--no-pager', '-o', 'cat'])]:
    try:
        report[label] = subprocess.check_output(arguments, text=True, stderr=subprocess.STDOUT, timeout=6)[-8000:]
    except Exception as error:
        report[label] = str(error)
print(json.dumps(report, indent=2))
