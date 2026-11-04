import dbus,json,socket,time
bus=dbus.SystemBus()
adapter=dbus.Interface(bus.get_object('org.bluez','/org/bluez/hci0'),'org.freedesktop.DBus.Properties').GetAll('org.bluez.Adapter1')
report={'scope':'direct Linux kernel LE ATT diagnostic; not BlueZ GATT profile qualification'}
channel=None
try:
    report['stage']='socket'
    channel=socket.socket(socket.AF_BLUETOOTH,socket.SOCK_SEQPACKET,socket.BTPROTO_L2CAP)
    channel.settimeout(10)
    report['stage']='bind LE fixed CID 4'
    channel.bind((str(adapter['Address']),0,4,1))
    report['stage']='connect LE public fixed CID 4'
    began=time.monotonic();channel.connect(('6C:C8:40:42:BF:66',0,4,1))
    report['connect_seconds']=round(time.monotonic()-began,3)
    report['stage']='ATT MTU exchange'
    channel.send(bytes([2,5,2]));report['mtu_reply']=channel.recv(1024).hex()
    report['stage']='ATT name read'
    channel.send(bytes([0x0a,3,0]));report['name_reply']=channel.recv(1024).hex()
    report['passed']=report['mtu_reply']=='030502' and bytes.fromhex(report['name_reply'])==b'\x0bNATIVE_BT_PROBE'
except Exception as error:report['error']=str(error)
finally:
    if channel:channel.close()
print(json.dumps(report,indent=2))
