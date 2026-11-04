"""Remove only the new diagnostic peer bond through its encrypted control PSM."""
import dbus,json,socket
bus=dbus.SystemBus()
objects=dbus.Interface(bus.get_object('org.bluez','/'),'org.freedesktop.DBus.ObjectManager').GetManagedObjects()
source=objects['/org/bluez/hci0']['org.bluez.Adapter1']
report={'scope':'owned diagnostic bond-delete request; commit requires separate ESP32 receipt','request_submitted':False}
channel=None
try:
    assert str(source['Address'])=='E8:47:3A:EC:51:9A'
    assert not any(v.get('org.bluez.Device1',{}).get('Connected') for v in objects.values()),'Preserve active Bluetooth sessions'
    channel=socket.socket(socket.AF_BLUETOOTH,socket.SOCK_SEQPACKET,socket.BTPROTO_L2CAP)
    channel.settimeout(12);channel.setsockopt(274,4,bytes([2,0]))
    channel.bind((str(source['Address']),0,0,0));channel.connect(('6C:C8:40:42:BF:66',0x1003,0,0))
    policy=channel.getsockopt(274,4,2);report['security_level']=policy[0];report['key_size']=policy[1]
    assert policy[0]>=2 and policy[1]==16
    assert channel.send(bytes([1]))==1;report['request_submitted']=True
    try:report['peer_disconnected']=channel.recv(1024)==b''
    except ConnectionResetError:report['peer_disconnected']=True
except Exception as error:report['error']=str(error)
finally:
    if channel:channel.close()
print(json.dumps(report,indent=2))
raise SystemExit(0 if report['request_submitted'] and report.get('peer_disconnected') else 1)
