import dbus,json
bus=dbus.SystemBus()
objects=dbus.Interface(bus.get_object('org.bluez','/'),'org.freedesktop.DBus.ObjectManager').GetManagedObjects()
adapter=objects['/org/bluez/hci0']['org.bluez.Adapter1']
target=objects.get('/org/bluez/hci0/dev_6C_C8_40_42_BF_66',{})
print(json.dumps({'adapter_address':str(adapter['Address']),'powered':bool(adapter['Powered']),
 'target':{str(interface):{str(key):str(value) for key,value in properties.items() if key in ['Name','Address','Connected','Paired','Bonded','ServicesResolved','PreferredBearer']} for interface,properties in target.items()},
 'active_connections':[str(path) for path,interfaces in objects.items() if interfaces.get('org.bluez.Device1',{}).get('Connected')]},indent=2))
