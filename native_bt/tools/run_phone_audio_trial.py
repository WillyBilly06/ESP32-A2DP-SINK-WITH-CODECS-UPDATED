"""Bounded Redmagic playback companion for a separately admitted finite COM6 trial.

No ESP32 port access. Start before deploy_audio_trial.py. Fresh readiness and the
exact paired Classic address are mandatory. Receipts are private local evidence.
"""
import argparse
import json
import re
import subprocess
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

from audit_audio_capture import SNAPSHOTS, audit_capture, unique_object

ROOT = Path(__file__).resolve().parents[1]
SERIAL = '9125299103B7'
MAC = '6C:C8:40:42:BF:66'
APP = 'com.bdkaudio.controller'
ACTIVITY = APP + '/com.bdkaudio.bdk_audio.MainActivity'
LIVE = ROOT / '_build/hardware/COM6-probe-trial-live.txt'


def label(node):
    return node.get('content-desc') or node.get('text', '')


def center(node, screen):
    box = re.fullmatch(r'\[(\d+),(\d+)\]\[(\d+),(\d+)\]', node.get('bounds', ''))
    if box is None or node.get('enabled') != 'true':
        raise ValueError('Disabled or invalid UI target')
    x0, y0, x1, y1 = map(int, box.groups())
    if not (0 <= x0 < x1 <= screen[0] and 0 <= y0 < y1 <= screen[1]
            and x1 - x0 >= 30 and y1 - y0 >= 30):
        raise ValueError('Clipped or off-screen UI target')
    return (x0 + x1) // 2, (y0 + y1) // 2


def valid_records(raw):
    for line in raw.splitlines():
        try:
            row = json.loads(line, object_pairs_hook=unique_object)
        except (ValueError, UnicodeDecodeError):
            continue
        if not isinstance(row, dict):
            continue
        if (row.get('probe') in SNAPSHOTS or 'record_crc32' in row or 'snapshot_begin_us' in row) and not audit_capture(line)['passed']:
            continue
        yield row


def max_config(text):
    return (text.startswith('Current connection\n') and
            all(x in text.splitlines() for x in ('LHDC V5', '192 kHz', '24-bit', 'Stereo',
                                                'Source: Android query',
                                                'Read within the last 5 seconds')) and
            'LHDC V5 quality: 1000 kbps target' in text)


def authorized_details(tree):
    return any(label(n) == "Device's Bluetooth address: " + MAC for n in tree.iter('node'))


class Trial:
    def __init__(self, args):
        self.args = args
        self.start = time.time()
        self.end = time.monotonic() + args.ready_timeout
        self.out = ROOT / '_build/hardware/phone-trials' / str(time.time_ns())
        self.out.mkdir(parents=True)
        self.events = []
        self.latest = {}
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.fault = threading.Event()
        self.playing = False
        self.bytes_read = 0
        self.offset = 0
        self.pending = b''
        self.capture_started = False
        self.readiness = set()
        self.screen = (1216, 2688)

    def event(self, phase, **values):
        row = {'phase': phase, 'unix': time.time(), **values}
        with self.lock:
            self.events.append(row)
            with (self.out / 'events.ndjson').open('a', encoding='utf-8') as output:
                output.write(json.dumps(row) + '\n')
        print(json.dumps(row), flush=True)

    def adb(self, *args, timeout=12):
        result = subprocess.run([self.args.adb, '-s', SERIAL, *args], timeout=timeout,
                                capture_output=True, encoding='utf-8', errors='replace')
        if result.returncode:
            raise RuntimeError('ADB failed: ' + ' '.join(args[:3]) + ': ' + result.stderr[:300])
        return result.stdout

    def pause(self):
        self.adb('shell', 'input', 'keyevent', '127', timeout=5)
        self.playing = False
        self.event('pause_sent')

    def read_capture(self):
        if not LIVE.exists() or LIVE.stat().st_mtime < self.start:
            return
        if not self.capture_started:
            self.capture_started = True
            self.event('fresh_capture', path=str(LIVE))
        with LIVE.open('rb') as source:
            source.seek(self.offset)
            raw = source.read(1048576)
        self.offset += len(raw)
        self.bytes_read += len(raw)
        pieces = (self.pending + raw).split(b'\n')
        self.pending = pieces.pop()[-16384:]
        for row in valid_records(b'\n'.join(pieces)):
            name = row.get('probe')
            with self.lock:
                self.latest[name] = row
                if name in ('classic_security_probe_ready', 'gatt_probe_ready'):
                    self.readiness.add(name)
            if ((name == 'native_i2s' and row.get('failed')) or
                    (name == 'i2s_descriptor_faults' and any(row.get(k, 0) for k in ('shape', 'pool', 'order')))):
                if not self.fault.is_set():
                    self.fault.set()
                    self.event('firmware_fault', record=row)
                    self.pause()
                    self.adb('shell', 'am', 'force-stop', APP, timeout=5)
        if self.playing and any(x in raw for x in (b'Guru Meditation', b'watchdog got triggered')):
            self.fault.set()
            self.event('fatal_uart_marker')
            self.pause()

    def monitor(self):
        try:
            while not self.stop.wait(.1):
                self.read_capture()
        except Exception as error:
            self.fault.set()
            self.event('monitor_error', error=str(error))
            try:
                self.pause()
            except Exception as pause_error:
                self.event('pause_error', error=str(pause_error))

    def ui(self, name='ui'):
        if time.monotonic() >= self.end or self.fault.is_set():
            raise RuntimeError('UI deadline or firmware fault')
        self.adb('shell', 'uiautomator', 'dump', '/sdcard/bdk-window.xml')
        raw = self.adb('shell', 'cat', '/sdcard/bdk-window.xml')
        (self.out / (name + '.xml')).write_text(raw, encoding='utf-8', newline='\n')
        return ET.fromstring(raw)

    def find(self, tree, pattern):
        return next((n for n in tree.iter('node') if re.search(pattern, label(n))), None)

    def wait(self, pattern, seconds=12):
        deadline = min(self.end, time.monotonic() + seconds)
        while time.monotonic() < deadline:
            tree = self.ui()
            node = self.find(tree, pattern)
            if node is not None:
                return tree, node
            time.sleep(.2)
        raise RuntimeError('UI node absent: ' + pattern)

    def tap_node(self, node):
        x, y = center(node, self.screen)
        self.adb('shell', 'input', 'tap', str(x), str(y))
        self.event('tap', label=label(node))

    def tap(self, pattern):
        _, node = self.wait(pattern)
        self.tap_node(node)

    def swipe(self, down=False):
        width, height = self.screen
        x = int(width * .82)
        a, b = (int(height * .24), int(height * .89)) if down else (int(height * .87), int(height * .39))
        self.adb('shell', 'input', 'swipe', str(x), str(a), str(x), str(b), '400')

    def choice(self, name, value):
        pattern = '^' + re.escape(name) + r'\n'
        for _ in range(3):
            tree = self.ui()
            node = self.find(tree, pattern)
            if node is not None:
                try:
                    center(node, self.screen)
                    break
                except ValueError:
                    pass
            self.swipe()
        else:
            raise RuntimeError('Visible configuration field absent: ' + name)
        if label(node).splitlines()[-1] != value:
            self.tap_node(node)
            self.tap('^' + re.escape(value) + '$')

    def run(self):
        thread = None
        result = {'audio_workload_exercised': False, 'continuous_audio_passed': False}
        try:
            if self.adb('get-state').strip() != 'device':
                raise RuntimeError('Reserved Redmagic unavailable')
            self.pause()
            self.adb('shell', 'am', 'force-stop', APP)
            self.event('waiting_readiness', timeout_seconds=self.args.ready_timeout)
            thread = threading.Thread(target=self.monitor, daemon=True)
            thread.start()
            while time.monotonic() < self.end and not self.fault.is_set():
                with self.lock:
                    ready = len(self.readiness) == 2
                    layout = self.latest.get('scheduler_layout', {}).copy()
                if ready:
                    if layout.get('decoder_priority') != 6 or layout.get('owner_ready_yield') != 1:
                        raise RuntimeError('Unexpected owner-yield scheduler layout')
                    break
                time.sleep(.1)
            else:
                raise RuntimeError('Fresh Classic/GATT readiness not observed')
            self.event('ready', scheduler_layout=layout)
            self.end = time.monotonic() + 125
            self.adb('shell', 'am', 'start', '-a', 'android.settings.BLUETOOTH_SETTINGS')
            tree, _ = self.wait('^Device details$')
            if not authorized_details(tree):
                raise RuntimeError('Settings is not the exact authorized Classic fixture')
            self.event('classic_identity_verified', address=MAC)
            for attempt in range(2):
                tree = self.ui('classic-identity')
                if self.find(tree, '^Disconnect$') is not None:
                    break
                node = self.find(tree, '^Connect$')
                if node is None:
                    raise RuntimeError('No authorized Connect action')
                self.tap_node(node)
                try:
                    self.wait('^Disconnect$', 10)
                    break
                except RuntimeError:
                    if attempt == 1:
                        raise
            self.adb('shell', 'am', 'start', '-n', ACTIVITY)
            self.tap(r'^Audio quality(?:\n|$)')
            self.wait('^Current connection\n')
            self.choice('Codec', 'LHDC V5')
            self.tap('^Advanced preferences, Collapsed$')
            self.wait('^Advanced preferences, Expanded$')
            self.swipe()
            for name, value in (('Sample rate', '192 kHz'), ('Bit depth', '24-bit'),
                                ('Channels', 'Stereo'), ('Bitrate / quality', '1000 kbps')):
                self.choice(name, value)
            self.swipe()
            self.tap('^Apply preference$')
            self.swipe(down=True)
            for _ in range(2):
                tree, node = self.wait('^Current connection\n')
                if max_config(label(node)):
                    break
                self.tap('^Refresh$')
                time.sleep(.4)
            else:
                raise RuntimeError('Android did not confirm requested maximum LHDC configuration')
            self.ui('max-config-confirmed')
            self.event('android_max_confirmed', text=label(node), actual_bitrate_reported=False)
            if self.fault.is_set():
                raise RuntimeError('Fault before playback')
            self.playing = True
            self.adb('shell', 'input', 'keyevent', '126')
            self.event('play_sent')
            end = time.monotonic() + self.args.seconds
            while time.monotonic() < end and not self.fault.wait(.1):
                pass
            self.pause()
            # Watchdog text can arrive before the next complete I2S snapshot.
            # Keep the independent reader alive for a bounded diagnostic drain;
            # otherwise an immediate stop can incorrectly report no workload.
            if self.fault.is_set():
                settle_end = time.monotonic() + 4
                while time.monotonic() < settle_end:
                    with self.lock:
                        settled = self.latest.get('native_i2s', {}).get('failed', 0)
                    if settled:
                        break
                    time.sleep(.1)
            with self.lock:
                snapshots = {k: v for k, v in self.latest.items() if k in (
                    'native_i2s', 'media_input', 'codec_configuration', 'media_drop_reasons',
                    'vhci_timing', 'decoder_execution', 'worker_timing', 'i2s_descriptor_faults')}
            i2s = snapshots.get('native_i2s', {})
            codec = snapshots.get('codec_configuration', {})
            result.update(audio_workload_exercised=bool(i2s.get('accepted_frames', 0) and
                          i2s.get('dma_source_frames', 0)), fault_observed=self.fault.is_set(),
                          native_max_config_observed=codec.get('cie_hex') == '00ff3a050000354c0102110000',
                          snapshots=snapshots)
            self.event('observation_complete', audio_workload_exercised=result['audio_workload_exercised'],
                       fault_observed=result['fault_observed'])
        except Exception as error:
            result['error'] = str(error)
            self.event('trial_error', error=str(error))
        finally:
            try:
                self.pause()
                self.adb('shell', 'am', 'force-stop', APP)
            except Exception as error:
                result['final_pause_error'] = str(error)
            self.stop.set()
            if thread:
                thread.join(15)
            result.update(events=self.events, capture_bytes_read=self.bytes_read,
                          scope='Finite playback orchestration; full audio/parity qualification remains separate')
            (self.out / 'result.json').write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8', newline='\n')
            print(json.dumps({'result_path': str(self.out / 'result.json'),
                              'audio_workload_exercised': result['audio_workload_exercised'],
                              'error': result.get('error')}), flush=True)
        return 0 if result['audio_workload_exercised'] and not result.get('error') else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--adb', default=str(Path.home() / 'AppData/Local/Android/Sdk/platform-tools/adb.exe'))
    parser.add_argument('--ready-timeout', type=int, default=600)
    parser.add_argument('--seconds', type=int, default=60)
    args = parser.parse_args()
    if not 30 <= args.ready_timeout <= 900 or not 10 <= args.seconds <= 90:
        parser.error('Bounded readiness 30..900 seconds and playback 10..90 seconds required')
    raise SystemExit(Trial(args).run())


if __name__ == '__main__':
    main()
