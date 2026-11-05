"""Host dashboard tests: full records, CRC isolation, bounded history and reader release."""
import argparse,json,pathlib,threading,time,unittest,uuid,zlib
from unittest.mock import patch
from serial_monitor_server import Monitor,ROOT

def monitor():
    return Monitor(argparse.Namespace(port='COM6',usb_serial='CA0ACAD67933EF1180C36117A04C7D1C',baud=115200,released=True,state=str(ROOT/'_build/serial-tests/state.json'),http_port=0))

class DashboardTests(unittest.TestCase):
    def test_legacy_released_firmware_format(self):
        m=monitor();raw='LIVE1 rate=48000 bits=24 ch=2 heap=65536 largest=32768 active=1 started=1 under=0 drops=0 pcm_cb=24 i2s_err=0'
        m.line(raw);self.assertEqual(m.latest['LIVE1']['rate'],48000);self.assertEqual(m.logs[-1]['text'],raw)
        self.assertEqual(m.logs[-1]['integrity'],'Unverified');self.assertEqual(m.logs[-1]['stage'],'Decoder / queue')
    def test_periodic_records_are_retained(self):
        m=monitor()
        for name in ['native_i2s','sbc_pcm','audio_clock','media_drop_reasons']:
            raw=json.dumps({'probe':name,'value':123});m.line(raw)
            self.assertEqual(m.logs[-1]['text'],raw);self.assertEqual(m.logs[-1]['fields']['probe'],name)
        self.assertEqual(len(m.logs),4)

    def test_crc_invalid_record_cannot_replace_metric(self):
        m=monitor();prefix=b'{"probe":"native_i2s","running":1,"snapshot_begin_us":10,"at_us":20,"snapshot_span_us":10'
        raw=prefix+b',"record_crc32":"'+f'{zlib.crc32(prefix):08x}'.encode()+b'"}'
        m.line(raw.decode());self.assertEqual(m.logs[-1]['integrity'],'Verified')
        m.line(raw.replace(b'"running":1',b'"running":0').decode())
        self.assertEqual(m.logs[-1]['integrity'],'Invalid');self.assertEqual(m.latest['native_i2s']['running'],1)

    def test_history_bound_and_unique_ids(self):
        m=monitor()
        def produce(start):
            for n in range(start,start+2600):m.line(json.dumps({'probe':'worker_timing','n':n}))
        workers=[threading.Thread(target=produce,args=(i*2600,)) for i in range(4)]
        for worker in workers:worker.start()
        for worker in workers:worker.join()
        self.assertEqual(len(m.logs),10000);self.assertEqual(m.data['history_dropped'],400)
        ids=[r['id'] for r in m.logs];self.assertEqual(ids,sorted(set(ids)))

    def test_replay_rejects_arbitrary_paths(self):
        m=monitor()
        with self.assertRaises(ValueError):m.request('x','replay','../../private.bin')

    def test_release_ack_follows_close(self):
        m=monitor();opened=threading.Event();closed=threading.Event()
        class Port:
            in_waiting=0
            def __init__(self,**kwargs):self.dtr=None;self.rts=None
            def open(self):
                if self.dtr is not False or self.rts is not False:raise AssertionError('Modem lines not disabled before open')
                opened.set()
            def read(self,n):time.sleep(.01);return b''
            def close(self):closed.set()
        device=argparse.Namespace(device='COM6',vid=0x10c4,pid=0xea60,serial_number=m.args.usb_serial)
        with patch('serial_monitor_server.list_ports.comports',return_value=[device]),patch('serial_monitor_server.serial.Serial',Port):
            m.request('listen','resume');thread=threading.Thread(target=m.reader);thread.start()
            try:
                self.assertTrue(opened.wait(2));cid=uuid.uuid4().hex;m.request(cid,'release')
                deadline=time.monotonic()+2
                while time.monotonic()<deadline and m.data['applied_command_id']!=cid:time.sleep(.01)
                self.assertTrue(closed.is_set());self.assertEqual(m.data['applied_command_id'],cid)
                self.assertEqual(m.data['reader_mode'],'released');self.assertFalse(m.data['connected'])
            finally:m.stop.set();m.wake.set();thread.join(2)
            self.assertFalse(thread.is_alive())

if __name__=='__main__':unittest.main()
