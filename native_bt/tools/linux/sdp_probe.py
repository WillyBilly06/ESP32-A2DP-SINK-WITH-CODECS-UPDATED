"""Real SDP requests against the restricted native diagnostic server, no audio."""
import dbus
import json
import shutil
import socket
import struct
import subprocess
import time
import uuid

ADDRESS = '6C:C8:40:42:BF:66'
SOURCE = 'E8:47:3A:EC:51:9A'
DIAGNOSTIC = uuid.UUID('e8319409-8ac4-4f14-9e87-e04799f11001')
channel = None
transaction = 0
requests = 0


def sequence(payload):
    assert len(payload) < 256
    return bytes([0x35, len(payload)]) + payload


def uuid16(value):
    return b'\x19' + struct.pack('>H', value)


def decode(data, offset=0, depth=0):
    assert depth <= 8 and offset < len(data)
    descriptor = data[offset]
    kind, index = descriptor >> 3, descriptor & 7
    at = offset + 1
    if index < 5:
        size = 0 if kind == 0 else 1 << index
    else:
        width = 1 << (index - 5)
        assert at + width <= len(data)
        size = int.from_bytes(data[at:at + width], 'big')
        at += width
    end = at + size
    assert end <= len(data)
    if kind in (6, 7):
        value = []
        while at < end:
            item, at = decode(data, at, depth + 1)
            value.append(item)
        assert at == end
    elif kind in (1, 2):
        value = int.from_bytes(data[at:end], 'big', signed=(kind == 2))
    elif kind == 3:
        assert size in (2, 4, 16)
        if size == 16:
            value = str(uuid.UUID(bytes=bytes(data[at:end])))
        else:
            value = f'{int.from_bytes(data[at:end], "big"):08x}-0000-1000-8000-00805f9b34fb'
    elif kind in (4, 8):
        value = data[at:end].decode('utf-8')
    elif kind == 0:
        value = None
    else:
        value = data[at:end].hex()
    return value, end


def exchange(kind, parameters, expected):
    global transaction, requests
    transaction = (transaction + 1) & 65535
    wire = struct.pack('>BHH', kind, transaction, len(parameters)) + parameters
    assert channel.send(wire) == len(wire)
    response = channel.recv(2048)
    requests += 1
    assert len(response) >= 5, ('short response', response.hex())
    code, received, length = struct.unpack('>BHH', response[:5])
    assert received == transaction and length == len(response) - 5
    assert code == expected, (code, response[:16].hex())
    return response[5:]


def search(pattern):
    base = sequence(pattern) + struct.pack('>H', 16)
    token = b'\0'
    handles = []
    total = None
    for unused in range(32):
        result = exchange(2, base + token, 3)
        reported, count = struct.unpack('>HH', result[:4])
        if total is None:
            total = reported
        assert total == reported
        stop = 4 + 4 * count
        assert stop < len(result)
        handles.extend(struct.unpack('>' + 'I' * count, result[4:stop]))
        token = result[stop:]
        assert token[0] <= 16 and len(token) == 1 + token[0]
        if token == b'\0':
            assert len(handles) == total
            return handles
    raise RuntimeError('Too many search continuations')


def attributes(kind, prefix, maximum=7):
    base = prefix + struct.pack('>H', maximum) + sequence(b'\x0a\0\0\xff\xff')
    token = b'\0'
    output = bytearray()
    turns = 0
    for unused in range(256):
        result = exchange(kind, base + token, kind + 1)
        count = int.from_bytes(result[:2], 'big')
        assert 2 <= count <= maximum and count + 2 < len(result)
        output.extend(result[2:2 + count])
        token = result[2 + count:]
        assert token[0] <= 16 and len(token) == 1 + token[0]
        turns += 1
        if token == b'\0':
            value, used = decode(output)
            assert used == len(output)
            return value, turns, len(output)
    raise RuntimeError('Too many attribute continuations')


def record(values):
    assert len(values) % 2 == 0
    result = dict(zip(values[0::2], values[1::2]))
    assert len(result) * 2 == len(values)
    assert list(result) == sorted(result)
    return result


def main():
    global channel
    report = {'scope': 'native diagnostic SDP interoperability; no product profiles/audio proof',
              'passed': False}
    try:
        bus = dbus.SystemBus()
        objects = dbus.Interface(bus.get_object('org.bluez', '/'),
                                 'org.freedesktop.DBus.ObjectManager').GetManagedObjects()
        assert str(objects['/org/bluez/hci0']['org.bluez.Adapter1']['Address']) == SOURCE
        assert not any(v.get('org.bluez.Device1', {}).get('Connected') for v in objects.values()), 'Preserve active sessions'
        channel = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_SEQPACKET, socket.BTPROTO_L2CAP)
        channel.settimeout(10)
        params = bytearray(channel.getsockopt(getattr(socket, 'SOL_L2CAP', 6), 1, 12))
        struct.pack_into('<HH', params, 0, 672, 672)
        params[6] = 0
        channel.setsockopt(getattr(socket, 'SOL_L2CAP', 6), 1, bytes(params))
        channel.setsockopt(274, 4, bytes([1, 0]))
        channel.bind((SOURCE, 0, 0, 0))
        began = time.monotonic()
        channel.connect((ADDRESS, 1, 0, 0))
        report['connect_seconds'] = round(time.monotonic() - began, 3)
        assert search(uuid16(0x1000)) == [0]
        assert search(b'\x1c' + DIAGNOSTIC.bytes) == [0x10000]
        assert search(b'\x1c' + DIAGNOSTIC.bytes + uuid16(0x100)) == [0x10000]
        assert search(uuid16(0x1002)) == [0x10000]
        assert search(uuid16(0x110b)) == [], 'Unimplemented AudioSink must not be advertised'
        values, turns, size = attributes(4, struct.pack('>I', 0))
        server = record(values)
        assert server[0] == 0 and server[0x200] == [0x100] and server[0x201] == 1
        report['server_attribute_transactions'] = turns
        values, turns, size = attributes(6, sequence(uuid16(0x1002)))
        assert len(values) == 1
        echo = record(values[0])
        assert echo[0] == 0x10000 and echo[1] == [str(DIAGNOSTIC)]
        assert echo[4] == [['00000100-0000-1000-8000-00805f9b34fb', 0x1001]]
        assert echo[0x100] == 'Encrypted Basic echo'
        report.update(passed=True, public_browse_records=1, diagnostic_psm=0x1001,
                      combined_attribute_transactions=turns, combined_bytes=size,
                      audio_sink_advertised=False, request_response_transactions=requests)
    except Exception as error:
        report['error'] = str(error)
        report['request_response_transactions'] = requests
    finally:
        if channel:
            channel.close()
        report['elapsed_seconds'] = round(time.monotonic() - began, 3) if 'began' in locals() else None
    if report['passed'] and shutil.which('sdptool'):
        try:
            external = subprocess.run(['sdptool', 'browse', ADDRESS], capture_output=True, text=True, timeout=20)
            report['bluez_sdptool'] = {'return_code': external.returncode,
                'diagnostic_name_found': 'Encrypted Basic echo' in external.stdout,
                'diagnostic_psm_found': '4097' in external.stdout}
            assert external.returncode == 0 and all(report['bluez_sdptool'][key] for key in ['diagnostic_name_found', 'diagnostic_psm_found'])
        except Exception as error:
            report['external_client_error'] = str(error)
            report['passed'] = False
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


raise SystemExit(main())
