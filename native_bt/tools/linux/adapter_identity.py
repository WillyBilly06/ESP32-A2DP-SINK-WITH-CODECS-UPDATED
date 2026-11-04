"""Read the test adapter's transport/USB identity without changing connections."""
import dbus,json,pathlib,subprocess
bus=dbus.SystemBus()
objects=dbus.Interface(bus.get_object('org.bluez','/'),'org.freedesktop.DBus.ObjectManager').GetManagedObjects()
adapter=objects['/org/bluez/hci0']['org.bluez.Adapter1']
assert str(adapter['Address'])=='E8:47:3A:EC:51:9A'
report={'adapter_address':str(adapter['Address']),'sysfs_device':str(pathlib.Path('/sys/class/bluetooth/hci0/device').resolve())}
device=pathlib.Path('/sys/class/bluetooth/hci0/device').resolve()
usb=[]
for folder in [device,*device.parents]:
    vendor=folder/'idVendor';product=folder/'idProduct'
    if vendor.is_file() and product.is_file():
        usb.append({'vendor':vendor.read_text().strip(),'product':product.read_text().strip(),
                    'manufacturer':(folder/'manufacturer').read_text().strip() if (folder/'manufacturer').is_file() else None,
                    'name':(folder/'product').read_text().strip() if (folder/'product').is_file() else None})
report['usb']=usb
for command in [['hciconfig','hci0','-a'],['uname','-r']]:
    try:
        value=subprocess.run(command,capture_output=True,text=True,timeout=5)
        report[command[0]]={'return_code':value.returncode,'output':value.stdout[:3000]}
    except FileNotFoundError:pass
print(json.dumps(report,indent=2))
