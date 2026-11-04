"""Concurrent encrypted Classic Basic echo and unbonded LE GATT diagnostics."""
import dbus,json,socket,struct,threading,time,statistics
import hashlib
from pathlib import Path
from dbus.mainloop.glib import DBusGMainLoop
from gi.repository import GLib
ADDRESS='6C:C8:40:42:BF:66';DEVICE='/org/bluez/hci0/dev_'+ADDRESS.replace(':','_')
ECHO='12345678-1234-1234-1234-12345678f001';COUNTER='12345678-1234-1234-1234-12345678f002'
DBusGMainLoop(set_as_default=True);bus=dbus.SystemBus();context=GLib.MainContext.default()
manager=dbus.Interface(bus.get_object('org.bluez','/'),'org.freedesktop.DBus.ObjectManager')
adapter=dbus.Interface(bus.get_object('org.bluez','/org/bluez/hci0'),'org.bluez.Adapter1')
def objects():return manager.GetManagedObjects(timeout=5)
def options(**values):return dbus.Dictionary(values,signature='sv')
def dispatch(ms):
    until=time.monotonic()+ms/1000
    while True:
        while context.pending():context.iteration(False)
        if time.monotonic()>=until:return
        time.sleep(max(0,min(.005,until-time.monotonic())))
def state():
    value=objects().get(DEVICE,{})
    return {'bredr':bool(value.get('org.bluez.Bearer.BREDR1',{}).get('Connected')),
            'le':bool(value.get('org.bluez.Bearer.LE1',{}).get('Connected'))}
def main():
    report={'scope':'concurrent diagnostic Classic L2CAP and LE GATT; no A2DP/codec/audio proof','passed':False}
    scanned=notify=False;preferred=None;channel=None;counter=None;thread=None;stop=threading.Event()
    classic_results=[];classic_errors=[];notifications=[];overlap=[]
    gatt_writes=readbacks=0;reliable_complete=False
    try:
        initial=objects();local=initial['/org/bluez/hci0']['org.bluez.Adapter1'];properties=initial.get(DEVICE,{}).get('org.bluez.Device1',{})
        assert str(local['Address'])=='E8:47:3A:EC:51:9A','Source adapter changed'
        br_initial=initial.get(DEVICE,{}).get('org.bluez.Bearer.BREDR1',{})
        report['initial_generic_bond']={'paired':bool(properties.get('Paired')),'bonded':bool(properties.get('Bonded'))}
        report['initial_classic_bond']={'paired':bool(br_initial.get('Paired')),'bonded':bool(br_initial.get('Bonded'))}
        report['initial_target_present']=DEVICE in initial
        active=[str(path) for path,value in initial.items() if value.get('org.bluez.Device1',{}).get('Connected')]
        assert all(path==DEVICE for path in active),'Preserve unrelated active sessions'
        initial_bearers=state();report['adopted_existing_le']=initial_bearers['le']
        assert not initial_bearers['bredr'],'Preserve an existing Classic session'
        report['stage']='discover diagnostic LE service'
        adapter.SetDiscoveryFilter(options(Transport='le',Pattern=ADDRESS),timeout=5);adapter.StartDiscovery(timeout=5);scanned=True
        deadline=time.monotonic()+15
        while time.monotonic()<deadline:
            dispatch(100);properties=objects().get(DEVICE,{}).get('org.bluez.Device1',{})
            if properties.get('Name')=='NATIVE_BT_PROBE':break
        else:raise RuntimeError('Probe advertisement missing')
        br_discovered=objects().get(DEVICE,{}).get('org.bluez.Bearer.BREDR1',{})
        report['discovered_classic_bond']={'paired':bool(br_discovered.get('Paired')),'bonded':bool(br_discovered.get('Bonded'))}
        assert all(report['discovered_classic_bond'].values()),'Classic bond unavailable after fresh discovery'
        target=bus.get_object('org.bluez',DEVICE);device=dbus.Interface(target,'org.bluez.Device1')
        props=dbus.Interface(target,'org.freedesktop.DBus.Properties')
        report['stage']='connect LE using explicit Adapter1 bearer';outcome={}
        def accepted(*arguments):outcome['ok']=True
        def rejected(error):outcome['error']=str(error)
        if initial_bearers['le']:outcome['ok']=True
        else:adapter.ConnectDevice(options(Address=ADDRESS,AddressType='public'),reply_handler=accepted,error_handler=rejected,timeout=25)
        report['le_connect_method']='Adapter1.ConnectDevice AddressType=public'
        deadline=time.monotonic()+28
        while not outcome and time.monotonic()<deadline:dispatch(50)
        if not outcome.get('ok') and not state()['le']:raise RuntimeError(str(outcome))
        deadline=time.monotonic()+15;paths={}
        while time.monotonic()<deadline:
            dispatch(50);current=objects()
            paths={str(v['org.bluez.GattCharacteristic1'].get('UUID','')).lower():str(p) for p,v in current.items() if str(p).startswith(DEVICE+'/') and 'org.bluez.GattCharacteristic1' in v}
            if ECHO in paths and COUNTER in paths:break
        else:raise RuntimeError('GATT characteristics missing')
        echo=dbus.Interface(bus.get_object('org.bluez',paths[ECHO]),'org.bluez.GattCharacteristic1')
        counter=dbus.Interface(bus.get_object('org.bluez',paths[COUNTER]),'org.bluez.GattCharacteristic1')
        def changed(interface,values,invalid):
            if str(interface)=='org.bluez.GattCharacteristic1' and 'Value' in values:
                data=bytes(values['Value'])
                if len(data)==8:notifications.append((time.monotonic(),int.from_bytes(data[:4],'little')))
        bus.add_signal_receiver(changed,signal_name='PropertiesChanged',dbus_interface='org.freedesktop.DBus.Properties',path=paths[COUNTER])
        counter.StartNotify(timeout=8);notify=True;dispatch(50)
        report['stage']='reconnect encrypted Classic diagnostic PSM'
        channel=socket.socket(socket.AF_BLUETOOTH,socket.SOCK_SEQPACKET,socket.BTPROTO_L2CAP);channel.settimeout(12)
        l2cap_level=getattr(socket,'SOL_L2CAP',6)
        parameters=bytearray(channel.getsockopt(l2cap_level,1,12));struct.pack_into('<HH',parameters,0,2048,2048);parameters[6]=0
        channel.setsockopt(l2cap_level,1,bytes(parameters));channel.setsockopt(274,4,bytes([2,0]))
        channel.bind((str(local['Address']),0,0,0));began=time.monotonic();channel.connect((ADDRESS,0x1001,0,0))
        report['classic_connect_seconds']=round(time.monotonic()-began,3)
        policy=channel.getsockopt(274,4,2);report['classic_socket_security']={'level':policy[0],'key_size':policy[1]}
        assert policy[0]>=2 and policy[1]==16,'Classic encryption/key-size qualification missing'
        dispatch(100);report['both_connected_initial']=state();assert all(report['both_connected_initial'].values())
        def classic_worker():
            sizes=[1,48,193,672,1021,2048];sequence=0
            try:
                while not stop.is_set() and sequence<1500:
                    size=sizes[sequence%len(sizes)];payload=bytes((sequence+i*29)&255 for i in range(size));started=time.monotonic()
                    assert channel.send(payload)==size
                    actual=channel.recv(4096)
                    if actual!=payload:
                        directory=Path.home()/'.cache/native_bt/stack_tests'
                        (directory/'classic-expected.bin').write_bytes(payload)
                        (directory/'classic-actual.bin').write_bytes(actual)
                        report['classic_failure']={'sequence':sequence,'expected_length':len(payload),'actual_length':len(actual),
                            'expected_sha256':hashlib.sha256(payload).hexdigest(),'actual_sha256':hashlib.sha256(actual).hexdigest(),
                            'different_byte_count':sum(a!=b for a,b in zip(actual,payload))+abs(len(actual)-len(payload)),
                            'first_difference':next((i for i,(a,b) in enumerate(zip(actual,payload)) if a!=b),None)}
                        raise RuntimeError(('Classic channel EOF' if not actual else 'Classic echo mismatch')+' at '+str(sequence))
                    classic_results.append((started,time.monotonic(),size));sequence+=1;time.sleep(.02)
            except Exception as error:
                if not stop.is_set():classic_errors.append(str(error))
        thread=threading.Thread(target=classic_worker,daemon=True);thread.start()
        report['stage']='simultaneous Classic and GATT traffic';rtts=[];began=time.monotonic()
        for sequence in range(100):
            if classic_errors:raise RuntimeError(str(classic_errors))
            payload=sequence.to_bytes(4,'little')+bytes((sequence*11+i)&255 for i in range(12))
            started=time.monotonic();echo.WriteValue(dbus.Array(payload,signature='y'),options(type='request'),timeout=6);rtts.append((time.monotonic()-started)*1000)
            gatt_writes+=1
            if sequence%10==0:
                assert bytes(echo.ReadValue(options(),timeout=6))==payload
                readbacks+=1
                link_state=state();overlap.append(link_state);assert all(link_state.values()),'A bearer disconnected during traffic'
            dispatch(20)
        for length in [0,1,20,64,244,512]:
            if classic_errors:raise RuntimeError(str(classic_errors))
            payload=bytes((length+i*31)&255 for i in range(length))
            echo.WriteValue(dbus.Array(payload,signature='y'),options(type='request'),timeout=8)
            gatt_writes+=1
            assert bytes(echo.ReadValue(options(),timeout=8))==payload
            readbacks+=1
            dispatch(20)
        payload=bytes((3+i*13)&255 for i in range(512))
        echo.WriteValue(dbus.Array(payload,signature='y'),options(type='reliable'),timeout=10)
        gatt_writes+=1;reliable_complete=True
        assert bytes(echo.ReadValue(options(),timeout=8))==payload
        readbacks+=1
        dispatch(50)
        ended=time.monotonic();stop.set();thread.join(timeout=3)
        assert not thread.is_alive(),'Classic worker did not finish';assert not classic_errors,classic_errors
        assert len(classic_results)>=50 and len(notifications)>=90,(len(classic_results),len(notifications))
        assert max(row[2] for row in classic_results)==2048
        common=max(0,min(ended,classic_results[-1][1])-max(began,classic_results[0][0]));assert common>=3
        report.update(passed=True,gatt_writes=107,gatt_periodic_readbacks=10,gatt_additional_lengths=[0,1,20,64,244,512],
            gatt_reliable_write_bytes=512,notifications=len(notifications),classic_packets=len(classic_results),
            classic_bytes=sum(row[2] for row in classic_results),classic_sizes=sorted(set(row[2] for row in classic_results)),
            overlap_observations=len(overlap),overlap_seconds=round(common,3),
            gatt_write_rtt_median_ms=round(statistics.median(rtts),3),gatt_write_rtt_max_ms=round(max(rtts),3),
            classic_rtt_max_ms=round(max((row[1]-row[0])*1000 for row in classic_results),3))
    except Exception as error:report['error']=str(error)
    finally:
        report['classic_completed_packets']=len(classic_results)
        report['classic_completed_bytes']=sum(row[2] for row in classic_results)
        report['received_notifications']=len(notifications)
        report['completed_gatt_writes']=gatt_writes
        report['completed_gatt_readbacks']=readbacks
        report['reliable_write_completed']=reliable_complete
        stop.set()
        if channel:
            try:channel.shutdown(socket.SHUT_RDWR)
            except OSError:pass
            channel.close()
        if thread:thread.join(timeout=3)
        if notify:
            try:counter.StopNotify(timeout=5)
            except Exception as error:report['notify_cleanup_error']=str(error)
        try:
            if objects().get(DEVICE,{}).get('org.bluez.Device1',{}).get('Connected'):
                dbus.Interface(bus.get_object('org.bluez',DEVICE),'org.bluez.Device1').Disconnect(timeout=8)
            report['final_connected']=state()
        except Exception as error:report['disconnect_error']=str(error)
        if preferred is not None:
            try:dbus.Interface(bus.get_object('org.bluez',DEVICE),'org.freedesktop.DBus.Properties').Set('org.bluez.Device1','PreferredBearer',preferred,timeout=5)
            except Exception as error:report['preference_restore_error']=str(error)
        if scanned:
            try:adapter.StopDiscovery(timeout=5)
            except Exception as error:report['scan_cleanup_error']=str(error)
        print(json.dumps(report,indent=2))
    return 0 if report['passed'] else 1
raise SystemExit(main())
