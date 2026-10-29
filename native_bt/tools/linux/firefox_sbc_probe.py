"""Finite Firefox music -> ordinary BlueZ/PipeWire SBC, with continuous LE ATT."""
import dbus
import http.server
import json
import math
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import threading
import time
import wave

SOURCE = 'E8:47:3A:EC:51:9A'
TARGET = '6C:C8:40:42:BF:66'
DEVICE = '/org/bluez/hci0/dev_' + TARGET.replace(':', '_')
RATE = 48000
root = Path.home()/'snap/firefox/common/native_bt_audio'
root.mkdir(parents=True, exist_ok=True)
assert root.resolve() == root and not root.is_symlink()
profile = root/'profile'
profile.mkdir(exist_ok=True)
assert profile.resolve() == profile
states = []
states_lock = threading.Lock()
report = {'scope': 'finite isolated Firefox music over ordinary BlueZ/PipeWire with continuous kernel ATT',
          'passed': False, 'start_unix': time.time()}
uid = os.getuid()
os.environ['XDG_RUNTIME_DIR'] = '/run/user/' + str(uid)
os.environ['DBUS_SESSION_BUS_ADDRESS'] = 'unix:path=' + os.environ['XDG_RUNTIME_DIR'] + '/bus'
bus = dbus.SystemBus()
manager = dbus.Interface(bus.get_object('org.bluez', '/'), 'org.freedesktop.DBus.ObjectManager')
device = browser = server = le = None
writes = readbacks = notifications = 0


def command(arguments):
    return subprocess.check_output(arguments, text=True, timeout=6)


def pulse(kind):
    return json.loads(command(['pactl', '-f', 'json', 'list', kind]))


class Server(http.server.BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        if self.path == '/':
            body = b'''<!doctype html><title>Native Bluetooth audio test</title>
<p>Finite Bluetooth music test</p><audio id="audio" src="/tone.wav" autoplay controls></audio>
<script>
const a=document.getElementById('audio');a.volume=0.15;
function state(event){fetch('/state',{method:'POST',headers:{'Content-Type':'application/json'},
body:JSON.stringify({event,time:a.currentTime,paused:a.paused,ready:a.readyState,
error:a.error?a.error.code:0})}).catch(()=>{});}
for(const e of ['playing','waiting','pause','ended','stalled','error'])a.addEventListener(e,()=>state(e));
setInterval(()=>state('tick'),500);a.play().catch(()=>state('play_rejected'));
</script>'''
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == '/tone.wav':
            path = root/'tone.wav'
            size = path.stat().st_size
            start, end = 0, size-1
            requested = self.headers.get('Range')
            if requested:
                assert requested.startswith('bytes=') and ',' not in requested
                first, last = requested[6:].split('-')
                start = int(first) if first else 0
                end = min(int(last), end) if last else end
                assert 0 <= start <= end < size
            self.send_response(206 if requested else 200)
            self.send_header('Content-Type', 'audio/wav')
            self.send_header('Accept-Ranges', 'bytes')
            self.send_header('Content-Length', str(end-start+1))
            if requested:
                self.send_header('Content-Range', f'bytes {start}-{end}/{size}')
            self.end_headers()
            try:
                with path.open('rb') as handle:
                    handle.seek(start)
                    left = end-start+1
                    while left:
                        block = handle.read(min(left, 65536))
                        self.wfile.write(block)
                        left -= len(block)
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self.send_error(404)

    def do_POST(self):
        if self.path != '/state':
            self.send_error(404)
            return
        size = int(self.headers.get('Content-Length', '0'))
        assert 0 < size <= 1024
        state = json.loads(self.rfile.read(size))
        state['received_unix'] = time.time()
        with states_lock:
            if len(states) < 160:
                states.append(state)
        self.send_response(204)
        self.end_headers()


try:
    before = manager.GetManagedObjects()
    assert str(before['/org/bluez/hci0']['org.bluez.Adapter1']['Address']) == SOURCE
    assert not any(value.get('org.bluez.Device1', {}).get('Connected') for value in before.values())
    assert not pulse('sink-inputs'), 'Preserve existing media streams'
    report['original_default_sink'] = command(['pactl', 'get-default-sink']).strip()
    properties = before[DEVICE]['org.bluez.Device1']
    assert properties['Name'] == 'NATIVE_BT_PROBE' and properties['Paired'] and properties['Bonded']
    assert '0000110b-0000-1000-8000-00805f9b34fb' in list(properties['UUIDs'])
    desktop = {}
    for line in command(['systemctl', '--user', 'show-environment']).splitlines():
        key, _, value = line.partition('=')
        if key in ['DISPLAY', 'WAYLAND_DISPLAY', 'XAUTHORITY', 'XDG_SESSION_TYPE']:
            desktop[key] = value
    if 'WAYLAND_DISPLAY' not in desktop:
        sockets = [path for path in Path(os.environ['XDG_RUNTIME_DIR']).glob('wayland-*') if path.is_socket()]
        assert len(sockets) == 1, 'Need the active user display socket'
        desktop['WAYLAND_DISPLAY'] = sockets[0].name
    device = dbus.Interface(bus.get_object('org.bluez', DEVICE), 'org.bluez.Device1')
    device.ConnectProfile('0000110b-0000-1000-8000-00805f9b34fb', timeout=15)
    deadline = time.monotonic()+12
    while time.monotonic() < deadline:
        owned = [value for value in pulse('sinks') if TARGET.replace(':', '_') in value.get('name', '')]
        if owned:
            break
        time.sleep(.2)
    assert len(owned) == 1, 'No unique PipeWire Bluetooth output'
    sink = owned[0]
    report['sink'] = {'name': sink['name'], 'index': sink['index'],
                      'sample_specification': sink['sample_specification']}
    current = manager.GetManagedObjects()
    report['transports'] = [{'path': str(path), 'codec': int(value['org.bluez.MediaTransport1']['Codec']),
                             'configuration': bytes(value['org.bluez.MediaTransport1']['Configuration']).hex()}
                            for path, value in current.items()
                            if 'org.bluez.MediaTransport1' in value and str(path).startswith(DEVICE+'/')]
    assert report['transports'] and all(item['codec'] == 0 for item in report['transports'])
    # Original generated melody, 45 seconds: playback does not loop during the workload.
    notes = [261.6256, 329.6276, 391.9954, 440.0, 391.9954, 329.6276, 293.6648, 261.6256]
    with wave.open(str(root/'tone.wav'), 'wb') as handle:
        handle.setnchannels(2)
        handle.setsampwidth(2)
        handle.setframerate(RATE)
        for second in range(45):
            block = bytearray()
            frequency = notes[second % len(notes)]
            for sample in range(RATE):
                elapsed = sample/RATE
                envelope = min(1.0, elapsed*100, (1-elapsed)*100)
                value = int(4500*envelope*(math.sin(2*math.pi*frequency*elapsed)+.2*math.sin(4*math.pi*frequency*elapsed)))
                block.extend(struct.pack('<hh', value, value))
            handle.writeframesraw(block)
    prefs = {'media.autoplay.default': 0, 'media.autoplay.blocking_policy': 0,
             'media.autoplay.block-webaudio': False, 'browser.shell.checkDefaultBrowser': False,
             'browser.aboutwelcome.enabled': False, 'browser.startup.homepage_override.mstone': 'ignore',
             'browser.cache.disk.enable': False, 'toolkit.telemetry.enabled': False,
             'datareporting.healthreport.uploadEnabled': False}
    (profile/'user.js').write_text(''.join('user_pref('+json.dumps(key)+','+json.dumps(value)+');\n' for key, value in prefs.items()))
    server = http.server.ThreadingHTTPServer(('127.0.0.1', 0), Server)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    environment = dict(os.environ, **desktop, MOZ_ENABLE_WAYLAND='1', PULSE_SINK=sink['name'])
    log = (root/'firefox.log').open('w')
    browser = subprocess.Popen(['firefox', '--no-remote', '--new-instance', '--profile', str(profile),
                                '--new-window', f'http://127.0.0.1:{server.server_port}/'],
                               env=environment, stdout=log, stderr=log, start_new_session=True)
    deadline = time.monotonic()+25
    while time.monotonic() < deadline:
        with states_lock:
            playing = any(value.get('event') == 'playing' for value in states)
        inputs = [value for value in pulse('sink-inputs')
                  if 'firefox' in value.get('properties', {}).get('application.name', '').lower()]
        if playing and inputs:
            break
        if browser.poll() is not None:
            raise RuntimeError('Isolated Firefox exited before playback')
        time.sleep(.2)
    assert playing and inputs, 'Firefox did not start the generated music'
    assert all(value['sink'] == sink['index'] for value in inputs), 'Firefox audio was not routed to Bluetooth'
    report['firefox_inputs'] = [{'index': value['index'], 'sink': value['sink'],
                               'application': value['properties'].get('application.name')}
                              for value in inputs]
    le = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_SEQPACKET, socket.BTPROTO_L2CAP)
    le.settimeout(3)
    options = bytearray(le.getsockopt(6, 1, 12))
    struct.pack_into('<HH', options, 0, 517, 517)
    options[6] = 0
    le.setsockopt(6, 1, bytes(options))
    le.setsockopt(274, 4, bytes([1, 0]))
    le.bind((SOURCE, 0, 4, 1))
    le.connect((TARGET, 0, 4, 1))

    def att(payload, opcode):
        global notifications
        assert le.send(payload) == len(payload)
        for unused in range(100):
            reply = le.recv(1024)
            assert reply
            if reply[0] == 0x1b:
                assert len(reply) == 11 and reply[1:3] == b'\x15\0'
                notifications += 1
                continue
            assert reply[0] == opcode, (opcode, reply[:16].hex())
            return reply
        raise RuntimeError('Notification limit')

    assert att(b'\x02\x05\x02', 3) == b'\x03\x05\x02'
    att(b'\x12\x16\0\x01\0', 0x13)
    began = time.monotonic()
    report['overlap_start_unix'] = time.time()
    while time.monotonic()-began < 22:
        payload = writes.to_bytes(4, 'little') + bytes((writes+i*13) & 255 for i in range(12))
        att(b'\x12\x12\0'+payload, 0x13)
        writes += 1
        if writes % 10 == 0:
            assert att(b'\x0a\x12\0', 0x0b)[1:] == payload
            readbacks += 1
        time.sleep(.02)
    report['overlap_seconds'] = round(time.monotonic()-began, 3)
    report['overlap_end_unix'] = time.time()
    with states_lock:
        overlap = [value for value in states if value['received_unix'] >= report['overlap_start_unix']]
    assert len(overlap) >= 30 and overlap[-1]['time']-overlap[0]['time'] >= 19
    assert not any(value['paused'] or value['error'] or value['event'] in ['waiting', 'stalled', 'ended', 'error'] for value in overlap)
    assert writes >= 100 and notifications >= 100
    report['passed'] = True
except Exception as error:
    report['error'] = str(error)
finally:
    if le:
        le.close()
    if browser:
        if browser.poll() is None:
            os.killpg(browser.pid, signal.SIGTERM)
            try:
                browser.wait(timeout=6)
            except subprocess.TimeoutExpired:
                os.killpg(browser.pid, signal.SIGKILL)
                browser.wait(timeout=3)
        report['browser_exit'] = browser.returncode
    if server:
        server.shutdown()
        server.server_close()
    if device:
        try:
            device.Disconnect(timeout=8)
            report['disconnected'] = True
        except Exception as error:
            report['disconnect_error'] = str(error)
    report.update(att_writes=writes, att_readbacks=readbacks, notifications=notifications,
                  browser_states=states, end_unix=time.time())
    report['final_default_sink'] = command(['pactl', 'get-default-sink']).strip()
    report['profile_bytes'] = sum(path.stat().st_size for path in profile.rglob('*') if path.is_file())
    report['firefox_log_tail'] = (root/'firefox.log').read_text(errors='replace')[-1800:] if (root/'firefox.log').exists() else ''
    print(json.dumps(report, indent=2))
raise SystemExit(0 if report['passed'] else 1)
