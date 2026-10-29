import dbus,json
bus=dbus.SystemBus()
path='/org/bluez/hci0/dev_6C_C8_40_42_BF_66'
objects=dbus.Interface(bus.get_object('org.bluez','/'),'org.freedesktop.DBus.ObjectManager').GetManagedObjects()
props=objects.get(path,{}).get('org.bluez.Device1',{})
report={'target_present':bool(props)}
if props:
    if props.get('Name')!='NATIVE_BT_PROBE' or props.get('Connected') or props.get('Paired') or props.get('Bonded'):
        raise RuntimeError('Preserve target: it is not the disconnected unbonded diagnostic peer')
    dbus.Interface(bus.get_object('org.bluez','/org/bluez/hci0'),'org.bluez.Adapter1').RemoveDevice(path,timeout=8)
    report['temporary_diagnostic_peer_removed']=True
print(json.dumps(report,indent=2))
