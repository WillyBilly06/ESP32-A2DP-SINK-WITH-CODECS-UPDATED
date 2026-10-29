"""Bounded direct AVDTP/SBC-to-PCM diagnostic with uninterrupted kernel ATT traffic."""
import argparse,ctypes,dbus,hashlib,json,queue,socket,struct,threading,time
from pathlib import Path

SOURCE='E8:47:3A:EC:51:9A';TARGET='6C:C8:40:42:BF:66'
DEVICE='/org/bluez/hci0/dev_'+TARGET.replace(':','_')
parser=argparse.ArgumentParser();parser.add_argument('--packet-frames',type=int,default=4);args=parser.parse_args()
assert 1<=args.packet_frames<=5
bus=dbus.SystemBus();manager=dbus.Interface(bus.get_object('org.bluez','/'),'org.freedesktop.DBus.ObjectManager')
def objects():return manager.GetManagedObjects(timeout=5)
def configure(channel,mtu,security):
    options=bytearray(channel.getsockopt(6,1,12));struct.pack_into('<HH',options,0,mtu,mtu);options[6]=0
    channel.setsockopt(6,1,bytes(options));channel.setsockopt(274,4,bytes([security,0]));channel.settimeout(8)
def channel(psm,cid=0,kind=0,security=2,mtu=672):
    s=socket.socket(socket.AF_BLUETOOTH,socket.SOCK_SEQPACKET,socket.BTPROTO_L2CAP)
    configure(s,mtu,security);s.bind((SOURCE,0,cid,kind));s.connect((TARGET,psm,cid,kind));return s
def frames(raw):
    cursor=0;result=[]
    while cursor<len(raw):
        assert raw[cursor]==0x9c and cursor+4<=len(raw)
        f,pool=raw[cursor+1:cursor+3];bands=8 if f&1 else 4;blocks=4*(1+((f>>4)&3));mode=(f>>2)&3;c=2 if mode else 1
        bits=blocks*pool*(c if mode<2 else 1)+(bands if mode==3 else 0)
        size=4+bands*c//2+(bits+7)//8;assert cursor+size<=len(raw)
        result.append(raw[cursor:cursor+size]);cursor+=size
    return result

def main():
    report={'scope':'direct encrypted AVDTP/SBC decode counters plus kernel ATT; no Firefox/I2S/acoustic proof','passed':False}
    signaling=media=le=None;sender=receiver=None;stop=threading.Event();done=threading.Event();errors=[]
    responses=queue.Queue(32);signal_lock=threading.Lock();delay_reports=[]
    sent=[];notifications=[];writes=readbacks=0;started=None;label=0
    began=time.monotonic();report['start_unix']=time.time()
    try:
        current=objects();assert str(current['/org/bluez/hci0']['org.bluez.Adapter1']['Address'])==SOURCE
        assert not any(v.get('org.bluez.Device1',{}).get('Connected') for v in current.values()),'Preserve active sessions'
        bond=current.get(DEVICE,{}).get('org.bluez.Bearer.BREDR1',{});assert bond.get('Paired') and bond.get('Bonded')
        root=Path.home()/'.cache/native_bt/sbc_tests';raw=(root/'tone_48000_2.sbc').read_bytes();encoded=frames(raw)
        f=encoded[0][1];mode=(f>>2)&3;assert f>>6==3 and mode==2
        cie=bytes([0,0,(0x80>>(f>>6))|(8>>mode),(0x80>>((f>>4)&3))|(4 if f&1 else 8)|(2 if f&2 else 1),2,max(p[2] for p in encoded)])
        assert cie[-1]<=53
        count=8000;reference=ctypes.CDLL(str(root/'original.so'))
        reference.probe_reference.argtypes=[ctypes.POINTER(ctypes.c_uint8),ctypes.c_uint,ctypes.POINTER(ctypes.c_int16),ctypes.POINTER(ctypes.c_uint)]
        assert reference.probe_reference_open(2)==0
        pcm_hash=2166136261;pcm_bytes=0
        for n in range(count):
            frame=encoded[n%len(encoded)];data=(ctypes.c_uint8*len(frame)).from_buffer_copy(frame)
            pcm=(ctypes.c_int16*4096)();written=ctypes.c_uint()
            assert reference.probe_reference(data,len(frame),pcm,ctypes.byref(written))==0
            for value in bytes(pcm)[:written.value]:pcm_hash=((pcm_hash^value)*16777619)&0xffffffff
            pcm_bytes+=written.value
        report.update(fixture_sha256=hashlib.sha256(raw).hexdigest(),requested_frames=count,expected_pcm_bytes=pcm_bytes,
                      expected_pcm_fnv1a32=pcm_hash,codec_cie=cie.hex(),rate=48000,channels=2)
        report['stage']='encrypted AVDTP signaling'
        signaling=channel(0x19);policy=signaling.getsockopt(274,4,2)
        assert policy[0]>=2 and policy[1]==16;report['classic_security']={'level':policy[0],'key_size':policy[1]}
        signaling.settimeout(.5)
        def receive_signals():
            try:
                while not stop.is_set():
                    try:response=signaling.recv(1024)
                    except socket.timeout:continue
                    assert len(response)>=2
                    if response[0]&3==0:
                        assert response[1]==13 and len(response)==5
                        delay_reports.append(int.from_bytes(response[3:5],'big'))
                        with signal_lock:signaling.send(bytes([(response[0]&0xf0)|2,13]))
                    else:responses.put(response,timeout=1)
            except Exception as error:
                if not stop.is_set():errors.append(str(error))
        receiver=threading.Thread(target=receive_signals,daemon=True);receiver.start()
        def avdtp(signal,body=b''):
            nonlocal label
            own=label;label=(label+1)&15
            with signal_lock:assert signaling.send(bytes([own<<4,signal])+body)==len(body)+2
            for unused in range(20):
                response=responses.get(timeout=8);assert len(response)>=2
                kind=response[0]&3;assert ((response[0]>>2)&3)==0,'Unexpected fragmented diagnostic response'
                assert response[0]>>4==own and response[1]==signal,(signal,response.hex())
                assert kind==2,('AVDTP rejection',signal,response.hex())
                return response[2:]
            raise RuntimeError('AVDTP response limit')
        discovered=avdtp(1);assert discovered==b'\x04\x08'
        capabilities=avdtp(12,b'\x04');assert capabilities[:4]==b'\x01\x00\x07\x06'
        assert avdtp(3,b'\x04\x08\x01\x00\x07\x06'+cie+b'\x08\x00')==b''
        assert avdtp(6,b'\x04')==b'';report['stage']='media channel'
        media=channel(0x19);assert avdtp(7,b'\x04')==b''
        report['stage']='simultaneous SBC media and LE ATT'
        started=time.monotonic()
        def send_media():
            try:
                for base in range(0,count,args.packet_frames):
                    if stop.is_set():break
                    deadline=started+base*128/48000
                    if deadline>time.monotonic():time.sleep(deadline-time.monotonic())
                    selected=[encoded[(base+i)%len(encoded)] for i in range(min(args.packet_frames,count-base))]
                    packet=struct.pack('!BBHII',0x80,96,(base//args.packet_frames)&65535,base*128,0x12345678)+bytes([len(selected)])+b''.join(selected)
                    assert len(packet)<=672 and media.send(packet)==len(packet)
                    sent.append((time.monotonic(),len(selected)))
            except Exception as error:errors.append(str(error))
            finally:done.set()
        sender=threading.Thread(target=send_media,daemon=True);sender.start()
        le=channel(0,4,1,1,517)
        def att(payload,opcode):
            assert le.send(payload)==len(payload)
            for unused in range(100):
                reply=le.recv(1024);assert reply
                if reply[0]==0x1b:
                    assert len(reply)==11 and reply[1:3]==b'\x15\0';notifications.append(time.monotonic());continue
                assert reply[0]==opcode,(opcode,reply[:16].hex());return reply
            raise RuntimeError('ATT notification limit')
        assert att(b'\x02\x05\x02',3)==b'\x03\x05\x02';att(b'\x12\x16\0\x01\0',0x13)
        report['both_connected_initial']={'bredr':True,'le':True}
        while not done.is_set():
            if errors:raise RuntimeError(str(errors))
            payload=writes.to_bytes(4,'little')+bytes((writes+i*13)&255 for i in range(12))
            att(b'\x12\x12\0'+payload,0x13);writes+=1
            if writes%10==0:assert att(b'\x0a\x12\0',0x0b)[1:]==payload;readbacks+=1
            time.sleep(.02)
        sender.join(2);assert not sender.is_alive() and not errors,errors
        assert sum(row[1] for row in sent)==count and writes>=100
        report['traffic_seconds']=round(sent[-1][0]-started,3)
        report['stage']='suspend and close';time.sleep(.25)
        assert avdtp(9,b'\x04')==b'' and avdtp(8,b'\x04')==b''
        report['passed']=True
    except Exception as error:report['error']=str(error)
    finally:
        stop.set()
        if sender:sender.join(timeout=3)
        for s in [media,le,signaling]:
            if s:
                try:s.shutdown(socket.SHUT_RDWR)
                except OSError:pass
                s.close()
        if receiver:receiver.join(timeout=2)
        report.update(sent_media_packets=len(sent),sent_frames=sum(row[1] for row in sent),att_writes=writes,
                      att_readbacks=readbacks,notifications=len(notifications),delay_reports=len(delay_reports),
                      last_reported_delay_100us=delay_reports[-1] if delay_reports else None,
                      frames_per_packet=args.packet_frames,elapsed_seconds=round(time.monotonic()-began,3),end_unix=time.time())
        try:
            if objects().get(DEVICE,{}).get('org.bluez.Device1',{}).get('Connected'):
                dbus.Interface(bus.get_object('org.bluez',DEVICE),'org.bluez.Device1').Disconnect(timeout=8)
        except Exception as error:report['disconnect_error']=str(error)
        print(json.dumps(report,indent=2))
    return 0 if report['passed'] else 1
raise SystemExit(main())
