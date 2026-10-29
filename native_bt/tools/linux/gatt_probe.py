"""Bounded BlueZ test of the independently authored diagnostic ATT server."""
import json
import os
import statistics
import subprocess
import threading
from pathlib import Path
import time
import dbus
from dbus.mainloop.glib import DBusGMainLoop
from gi.repository import GLib

ADDRESS = '6C:C8:40:42:BF:66'
SERVICE = '12345678-1234-1234-1234-12345678f000'
ECHO = '12345678-1234-1234-1234-12345678f001'
COUNTER = '12345678-1234-1234-1234-12345678f002'
DEVICE = '/org/bluez/hci0/dev_' + ADDRESS.replace(':', '_')
DBusGMainLoop(set_as_default=True)
bus = dbus.SystemBus()
manager = dbus.Interface(bus.get_object('org.bluez', '/'), 'org.freedesktop.DBus.ObjectManager')
adapter = dbus.Interface(bus.get_object('org.bluez', '/org/bluez/hci0'), 'org.bluez.Adapter1')
context = GLib.MainContext.default()


def dispatch(milliseconds):
    until = time.monotonic() + milliseconds / 1000
    while True:
        while context.pending():
            context.iteration(False)
        if time.monotonic() >= until:
            return
        time.sleep(max(0, min(0.005, until - time.monotonic())))


def objects():
    return manager.GetManagedObjects(timeout=5)


def options(**values):
    return dbus.Dictionary(values, signature='sv')


def main():
    report = {'scope': 'independent diagnostic LE ATT/GATT only; no audio, bonding or simultaneous Classic proof',
              'passed': False, 'address': ADDRESS}
    before = objects()
    discovered = connected_here = connection_attempted = notifying = False
    preferred = None
    counter_characteristic = None
    notifications = []
    trace = bytearray()
    monitor = subprocess.Popen(['btmon', '-i', '0'], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    def capture():
        while True:
            data = monitor.stdout.read1(4096)
            if not data:
                return
            if len(trace) < 262144:
                trace.extend(data[:262144-len(trace)])
    reader = threading.Thread(target=capture, daemon=True)
    reader.start()
    try:
        adapter.SetDiscoveryFilter(options(Transport='le', UUIDs=dbus.Array([SERVICE], signature='s'), Pattern=ADDRESS), timeout=5)
        adapter.StartDiscovery(timeout=5)
        discovered = True
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            dispatch(100)
            properties = objects().get(DEVICE, {}).get('org.bluez.Device1', {})
            if SERVICE in [str(x).lower() for x in properties.get('UUIDs', [])] and properties.get('Name') == 'NATIVE_BT_PROBE':
                break
        else:
            raise RuntimeError('The identified diagnostic peripheral was not observed advertising')
        report['discovered_name'] = str(properties['Name'])
        report['already_known'] = DEVICE in before
        report['device_interfaces'] = list(map(str, objects().get(DEVICE, {}).keys()))
        report['le_before_connect'] = {key:str(value) for key,value in objects().get(DEVICE,{}).get('org.bluez.Bearer.LE1',{}).items()}
        report['preferred_bearer'] = str(properties.get('PreferredBearer', 'unavailable'))
        if properties.get('Connected'):
            raise RuntimeError('Target already connected; preserve its current session')
        if properties.get('Paired') or properties.get('Bonded'):
            raise RuntimeError('Unexpected saved bond on diagnostic target')
        device_object = bus.get_object('org.bluez', DEVICE)
        device = dbus.Interface(device_object, 'org.bluez.Device1')
        connection_attempted = True
        def connect_async(interface):
            nonlocal discovered
            outcome={}
            def succeeded(*args):outcome['reply']=args
            def rejected(error):outcome['error']=error
            interface.Connect(reply_handler=succeeded,error_handler=rejected,timeout=35)
            began=time.monotonic();states=[]
            while not outcome and time.monotonic()-began<38:
                dispatch(100)
                values=objects().get(DEVICE,{})
                state=[bool(values.get('org.bluez.Device1',{}).get('Connected')),
                       bool(values.get('org.bluez.Device1',{}).get('ServicesResolved')),
                       bool(values.get('org.bluez.Bearer.LE1',{}).get('Connected')),
                       bool(values.get('org.bluez.Bearer.BREDR1',{}).get('Connected'))]
                if not states or states[-1]['state']!=state:states.append({'seconds':round(time.monotonic()-began,3),'state':state})
            report['connection_states']=states
            report['connect_elapsed_seconds']=round(time.monotonic()-began,3)
            if 'error' in outcome:raise outcome['error']
            if 'reply' not in outcome:raise RuntimeError('Connection observation deadline expired; cancel owned attempt')
        try:
            if 'PreferredBearer' in properties:
                preferred=str(properties['PreferredBearer'])
                dbus.Interface(device_object, 'org.freedesktop.DBus.Properties').Set('org.bluez.Device1', 'PreferredBearer', 'le', timeout=5)
                report['connection_stage']='Device1.Connect with PreferredBearer=le'
                connect_async(device)
                report['connect_method']='Device1.Connect with PreferredBearer=le'
            elif 'org.bluez.Bearer.LE1' in report['device_interfaces']:
                report['connection_stage'] = 'Bearer.LE1.Connect'
                connect_async(dbus.Interface(device_object, 'org.bluez.Bearer.LE1'))
                report['connect_method'] = 'Bearer.LE1.Connect'
            else:
                report['connection_stage'] = 'Adapter1.ConnectDevice'
                path = adapter.ConnectDevice(options(Address=ADDRESS, AddressType='public'), timeout=15)
                assert str(path) == DEVICE
                report['connect_method'] = 'Adapter1.ConnectDevice public LE'
        except dbus.DBusException as error:
            if error.get_dbus_name() not in ['org.freedesktop.DBus.Error.UnknownMethod', 'org.bluez.Error.NotSupported']:
                raise
            if 'PreferredBearer' in properties:
                preferred = str(properties['PreferredBearer'])
                dbus.Interface(device_object, 'org.freedesktop.DBus.Properties').Set('org.bluez.Device1', 'PreferredBearer', 'le', timeout=5)
            report['connection_stage'] = 'Device1.Connect'
            connect_async(device)
            report['connect_method'] = 'Device1.Connect with LE advertisement'
        connected_here = True
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            dispatch(50)
            current = objects()
            if current.get(DEVICE, {}).get('org.bluez.Device1', {}).get('ServicesResolved'):
                break
        else:
            raise RuntimeError('GATT services did not resolve')
        characteristics = {}
        for path, interfaces in current.items():
            characteristic = interfaces.get('org.bluez.GattCharacteristic1', {})
            if str(path).startswith(DEVICE + '/'):
                uuid = str(characteristic.get('UUID', '')).lower()
                if uuid in [ECHO, COUNTER]:
                    characteristics[uuid] = str(path)
        assert set(characteristics) == {ECHO, COUNTER}, characteristics
        report['services_resolved'] = True
        report['le_bearer'] = {key:str(value) for key,value in current.get(DEVICE, {}).get('org.bluez.Bearer.LE1', {}).items() if key in ['Connected','Role','Paired','Bonded']}
        echo = dbus.Interface(bus.get_object('org.bluez', characteristics[ECHO]), 'org.bluez.GattCharacteristic1')
        counter_characteristic = dbus.Interface(bus.get_object('org.bluez', characteristics[COUNTER]), 'org.bluez.GattCharacteristic1')

        collect = True
        def changed(interface, values, invalidated):
            if collect and str(interface) == 'org.bluez.GattCharacteristic1' and 'Value' in values:
                value = bytes(values['Value'])
                if len(value) == 8:
                    notifications.append((time.monotonic(), int.from_bytes(value[:4], 'little')))

        bus.add_signal_receiver(changed, signal_name='PropertiesChanged',
            dbus_interface='org.freedesktop.DBus.Properties', path=characteristics[COUNTER])
        counter_characteristic.StartNotify(timeout=10)
        notifying = True
        dispatch(100)
        start = time.monotonic()
        rtts = []
        for sequence in range(200):
            payload = sequence.to_bytes(4, 'little') + bytes([(sequence * 17 + i) & 255 for i in range(12)])
            began = time.monotonic()
            echo.WriteValue(dbus.Array(payload, signature='y'), options(type='request'), timeout=5)
            rtts.append((time.monotonic() - began) * 1000)
            if sequence % 10 == 0:
                assert bytes(echo.ReadValue(options(), timeout=5)) == payload, ('echo mismatch', sequence)
            dispatch(max(0, 50 - int((time.monotonic() - began) * 1000)))
        for length in [0, 1, 20, 64, 244, 512]:
            payload = bytes([(length + i * 31) & 255 for i in range(length)])
            echo.WriteValue(dbus.Array(payload, signature='y'), options(type='request'), timeout=8)
            assert bytes(echo.ReadValue(options(), timeout=8)) == payload, ('long echo mismatch', length)
            dispatch(25)
        reliable = bytes([(i * 13 + 3) & 255 for i in range(512)])
        echo.WriteValue(dbus.Array(reliable, signature='y'), options(type='reliable'), timeout=10)
        assert bytes(echo.ReadValue(options(), timeout=8)) == reliable, 'reliable write mismatch'
        dispatch(100)
        collect = False
        final = bytes(counter_characteristic.ReadValue(options(), timeout=5))
        assert len(final) == 8 and int.from_bytes(final[6:8], 'little') == 1
        final_counter = int.from_bytes(final[:4], 'little')
        assert int.from_bytes(final[4:6], 'little') == 512
        assert len(notifications) >= 190, ('too few notifications', len(notifications))
        assert all(b[1] > a[1] for a, b in zip(notifications, notifications[1:])), 'counter did not increase'
        gaps = [(b[0] - a[0]) * 1000 for a, b in zip(notifications, notifications[1:])]
        report.update(passed=True, request_writes=200, periodic_readbacks=20,
            additional_lengths=[0, 1, 20, 64, 244, 512], reliable_write_bytes=512,
            notifications=len(notifications), final_counter=final_counter,
            elapsed_seconds=round(time.monotonic() - start, 3),
            write_rtt_median_ms=round(statistics.median(rtts), 3),
            write_rtt_max_ms=round(max(rtts), 3),
            notification_gap_max_ms=round(max(gaps, default=0), 3))
    except Exception as error:
        report['error'] = str(error)
    finally:
        if notifying:
            try:
                counter_characteristic.StopNotify(timeout=5)
            except Exception as error:
                report['stop_notify_error'] = str(error)
        if connected_here or connection_attempted:
            try:
                dbus.Interface(bus.get_object('org.bluez', DEVICE), 'org.bluez.Device1').Disconnect(timeout=8)
                report['disconnected'] = True
            except Exception as error:
                report['disconnect_error'] = str(error)
        if preferred is not None:
            try:
                dbus.Interface(bus.get_object('org.bluez', DEVICE), 'org.freedesktop.DBus.Properties').Set('org.bluez.Device1', 'PreferredBearer', preferred, timeout=5)
            except Exception as error:
                report['preferred_restore_error'] = str(error)
        if discovered:
            try:
                adapter.StopDiscovery(timeout=5)
            except Exception as error:
                report['stop_discovery_error'] = str(error)
        monitor.terminate()
        try:
            monitor.wait(timeout=3)
        except subprocess.TimeoutExpired:
            monitor.kill()
            monitor.wait(timeout=3)
        reader.join(timeout=2)
        trace_path = Path.home()/'.cache/native_bt/stack_tests/le-connection-trace.txt'
        trace_path.parent.mkdir(parents=True, exist_ok=True)
        trace_path.write_bytes(trace)
        lines=trace.decode('utf-8', errors='replace').splitlines()
        selected=set()
        for index,line in enumerate(lines):
            if ADDRESS in line.upper() or any(text in line for text in ['Create Connection','Connection Complete','Connect Failed','Disconnect Complete','Connection Failed']):
                selected.update(range(max(0,index-1), min(len(lines),index+8)))
        report['connection_trace']=[lines[index] for index in sorted(selected)][:100]
        report['trace_bytes']=len(trace)
        if len(trace)<200:
            report['monitor_status']=trace.decode('utf-8',errors='replace')[:200]
        print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


raise SystemExit(main())
