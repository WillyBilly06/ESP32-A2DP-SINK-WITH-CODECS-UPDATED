"""Concurrent encrypted Classic Basic and kernel ATT sockets, no BlueZ GATT API."""
import argparse,dbus,hashlib,json,socket,statistics,struct,threading,time
from pathlib import Path

SOURCE='E8:47:3A:EC:51:9A';TARGET='6C:C8:40:42:BF:66'
PATH='/org/bluez/hci0/dev_'+TARGET.replace(':','_')
bus=dbus.SystemBus()
manager=dbus.Interface(bus.get_object('org.bluez','/'),'org.freedesktop.DBus.ObjectManager')
parser=argparse.ArgumentParser()
parser.add_argument('--classic-only',action='store_true')
parser.add_argument('--writes',type=int,default=100)
args=parser.parse_args()
if not 100<=args.writes<=5000:raise SystemExit('Use a bounded 100..5000-write workload')

def objects():return manager.GetManagedObjects(timeout=5)
def configure(channel,mtu,security):
    level=getattr(socket,'SOL_L2CAP',6)
    parameters=bytearray(channel.getsockopt(level,1,12))
    struct.pack_into('<HH',parameters,0,mtu,mtu);parameters[6]=0
    channel.setsockopt(level,1,bytes(parameters));channel.setsockopt(274,4,bytes([security,0]))
    channel.settimeout(8)

def main():
    report={'scope':'concurrent encrypted Classic Basic and kernel LE ATT; no A2DP/audio proof','passed':False}
    classic=le=worker=None;stop=threading.Event();classic_errors=[];classic_results=[];notifications=[]
    writes=readbacks=0;reliable=False;started=None
    overall=time.monotonic();start_unix=time.time();rtts=[]
    try:
        initial=objects();local=initial['/org/bluez/hci0']['org.bluez.Adapter1']
        assert str(local['Address'])==SOURCE
        assert not any(v.get('org.bluez.Device1',{}).get('Connected') for v in initial.values()),'Preserve active sessions'
        bond=initial.get(PATH,{}).get('org.bluez.Bearer.BREDR1',{})
        assert bond.get('Paired') and bond.get('Bonded'),'A prior Classic bond is required'
        report['stage']='encrypted Classic connection'
        classic=socket.socket(socket.AF_BLUETOOTH,socket.SOCK_SEQPACKET,socket.BTPROTO_L2CAP)
        configure(classic,2048,2);classic.bind((SOURCE,0,0,0));began=time.monotonic()
        classic.connect((TARGET,0x1001,0,0));policy=classic.getsockopt(274,4,2)
        report['classic_connect_seconds']=round(time.monotonic()-began,3)
        report['classic_socket_security']={'level':policy[0],'key_size':policy[1]}
        assert policy[0]>=2 and policy[1]==16
        if args.classic_only:
            report['scope']='encrypted Classic Basic control with no LE connection; no audio proof'
            report['stage']='Classic-only control traffic'
            started=time.monotonic()
            le_checks=0
            sizes=[1,48,193,672,1021,2048]
            for sequence in range(240):
                if sequence%10==0:
                    bearer=objects().get(PATH,{}).get('org.bluez.Bearer.LE1')
                    assert bearer is not None,'LE bearer state unavailable for the control'
                    assert not bearer.get('Connected'),'LE unexpectedly connected during the control'
                    le_checks+=1
                    report['le_disconnected_observations']=le_checks
                size=sizes[sequence%6];payload=bytes((sequence+i*29)&255 for i in range(size))
                began=time.monotonic();assert classic.send(payload)==size;actual=classic.recv(4096)
                if actual!=payload:
                    folder=Path.home()/'.cache/native_bt/stack_tests'
                    (folder/'classic-only-expected.bin').write_bytes(payload)
                    (folder/'classic-only-actual.bin').write_bytes(actual)
                    report['classic_failure']={'sequence':sequence,'expected_length':size,'actual_length':len(actual),
                        'expected_sha256':hashlib.sha256(payload).hexdigest(),'actual_sha256':hashlib.sha256(actual).hexdigest(),
                        'first_difference':next((i for i,(a,b) in enumerate(zip(actual,payload)) if a!=b),None),
                        'different_byte_count':sum(a!=b for a,b in zip(actual,payload))+abs(len(actual)-size)}
                    raise RuntimeError('Classic-only echo mismatch at '+str(sequence))
                classic_results.append((began,time.monotonic(),size));time.sleep(.02)
            report.update(passed=True,classic_sizes=sizes,le_connection_used=False)
            return 0
        report['stage']='kernel LE ATT connection'
        le=socket.socket(socket.AF_BLUETOOTH,socket.SOCK_SEQPACKET,socket.BTPROTO_L2CAP)
        configure(le,517,1);le.bind((SOURCE,0,4,1));began=time.monotonic()
        le.connect((TARGET,0,4,1));report['le_connect_seconds']=round(time.monotonic()-began,3)
        def exchange(payload,opcode):
            assert le.send(payload)==len(payload)
            for unused in range(100):
                data=le.recv(1024)
                assert data,'LE ATT EOF'
                if data[0]==0x1b:
                    assert len(data)==11 and data[1:3]==b'\x15\0'
                    notifications.append((time.monotonic(),int.from_bytes(data[3:7],'little')));continue
                if data[0]==1:raise RuntimeError('ATT error '+data.hex())
                assert data[0]==opcode,(opcode,data[:16].hex())
                return data
            raise RuntimeError('ATT notification limit')
        mtu=exchange(b'\x02\x05\x02',3);assert mtu==b'\x03\x05\x02'
        assert exchange(b'\x0a\x03\0',0x0b)[1:]==b'NATIVE_BT_PROBE'
        exchange(b'\x12\x16\0\x01\0',0x13)
        report['both_connected_initial']={'bredr':True,'le':True}
        def traffic():
            sequence=0;sizes=[1,48,193,672,1021,2048]
            try:
                while not stop.is_set() and sequence<max(1500,args.writes*12):
                    size=sizes[sequence%6];payload=bytes((sequence+i*29)&255 for i in range(size))
                    began=time.monotonic();assert classic.send(payload)==size;actual=classic.recv(4096)
                    if actual!=payload:
                        folder=Path.home()/'.cache/native_bt/stack_tests'
                        (folder/'kernel-classic-expected.bin').write_bytes(payload)
                        (folder/'kernel-classic-actual.bin').write_bytes(actual)
                        report['classic_failure']={'sequence':sequence,'expected_length':size,'actual_length':len(actual),
                            'expected_sha256':hashlib.sha256(payload).hexdigest(),'actual_sha256':hashlib.sha256(actual).hexdigest(),
                            'first_difference':next((i for i,(a,b) in enumerate(zip(actual,payload)) if a!=b),None),
                            'different_byte_count':sum(a!=b for a,b in zip(actual,payload))+abs(len(actual)-size)}
                        raise RuntimeError('Classic echo mismatch at '+str(sequence))
                    classic_results.append((began,time.monotonic(),size));sequence+=1;time.sleep(.02)
            except Exception as error:
                if not stop.is_set():classic_errors.append(str(error))
        worker=threading.Thread(target=traffic,daemon=True);worker.start()
        report['stage']='simultaneous Classic and ATT traffic';started=time.monotonic();rtts=[]
        for sequence in range(args.writes):
            if classic_errors:raise RuntimeError(str(classic_errors))
            payload=sequence.to_bytes(4,'little')+bytes((sequence*11+i)&255 for i in range(12))
            began=time.monotonic();exchange(b'\x12\x12\0'+payload,0x13)
            writes+=1;rtts.append((time.monotonic()-began)*1000)
            if sequence%10==0:
                assert exchange(b'\x0a\x12\0',0x0b)[1:]==payload;readbacks+=1
            time.sleep(.02)
        for length in [0,1,20,64,244,512]:
            if classic_errors:raise RuntimeError(str(classic_errors))
            payload=bytes((length+i*31)&255 for i in range(length))
            exchange(b'\x12\x12\0'+payload,0x13);writes+=1
            assert exchange(b'\x0a\x12\0',0x0b)[1:]==payload;readbacks+=1
        payload=bytes((3+i*13)&255 for i in range(512))
        prepared=b'\x16\x12\0\0\0'+payload
        assert exchange(prepared,0x17)==b'\x17'+prepared[1:]
        exchange(b'\x18\x01',0x19);writes+=1;reliable=True
        assert exchange(b'\x0a\x12\0',0x0b)[1:]==payload;readbacks+=1
        ended=time.monotonic();stop.set();worker.join(timeout=3)
        assert not worker.is_alive() and not classic_errors,classic_errors
        assert len(classic_results)>=50 and len(notifications)>=args.writes*9//10,(len(classic_results),len(notifications))
        overlap=max(0,min(ended,classic_results[-1][1])-max(started,classic_results[0][0]))
        assert overlap>=3
        report.update(passed=True,overlap_seconds=round(overlap,3),classic_sizes=sorted(set(row[2] for row in classic_results)),
            classic_rtt_max_ms=round(max((row[1]-row[0])*1000 for row in classic_results),3),
            gatt_write_rtt_median_ms=round(statistics.median(rtts),3),gatt_write_rtt_max_ms=round(max(rtts),3))
    except Exception as error:report['error']=str(error)
    finally:
        stop.set()
        for channel in [classic,le]:
            if channel:
                try:channel.shutdown(socket.SHUT_RDWR)
                except OSError:pass
                channel.close()
        if worker:worker.join(timeout=3)
        report.update(classic_completed_packets=len(classic_results),classic_completed_bytes=sum(row[2] for row in classic_results),
                      completed_gatt_writes=writes,completed_gatt_readbacks=readbacks,reliable_write_completed=reliable,
                      received_notifications=len(notifications))
        report['elapsed_seconds']=round(time.monotonic()-overall,3)
        report['traffic_elapsed_seconds']=round(time.monotonic()-started,3) if started else None
        report['start_unix']=start_unix;report['end_unix']=time.time()
        if classic_results:report['classic_rtt_max_ms']=round(max((row[1]-row[0])*1000 for row in classic_results),3)
        if rtts:
            report['gatt_write_rtt_median_ms']=round(statistics.median(rtts),3)
            report['gatt_write_rtt_max_ms']=round(max(rtts),3)
        try:
            # Only this test-created target connection may be disconnected.
            current=objects().get(PATH,{}).get('org.bluez.Device1',{})
            if current.get('Connected'):dbus.Interface(bus.get_object('org.bluez',PATH),'org.bluez.Device1').Disconnect(timeout=8)
            report['target_connected_after_cleanup']=bool(objects().get(PATH,{}).get('org.bluez.Device1',{}).get('Connected'))
        except Exception as error:report['disconnect_error']=str(error)
        print(json.dumps(report,indent=2))
    return 0 if report['passed'] else 1
raise SystemExit(main())
