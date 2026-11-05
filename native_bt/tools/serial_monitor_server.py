"""Loopback serial dashboard: bounded RAM history, no UART writes, explicit port lease."""
import argparse,collections,hmac,http.server,json,os,re,secrets,threading,time,uuid
from pathlib import Path
from urllib.parse import urlsplit,parse_qs
import serial
from serial.tools import list_ports
from audit_audio_capture import audit_capture
from serial_phone_gateway import remote_asset
ROOT=Path(__file__).resolve().parents[1]
ASSETS=Path(__file__).with_name('serial_dashboard')
HARDWARE=ROOT/'_build/hardware'
ANSI=re.compile(r'\x1b\[[0-?]*[ -/]*[@-~]')
class Monitor:
    def __init__(self,args):
        self.args=args;self.stop=threading.Event();self.wake=threading.Event();self.lock=threading.RLock();self.control=('', 'release' if args.released else 'resume');self.seen='';self.clock_text='Clock waiting';self.i2s_text='I2S waiting'
        self.state=Path(args.state).resolve();self.state.relative_to((ROOT/'_build').resolve());self.state.parent.mkdir(parents=True,exist_ok=True);self.command=self.state.with_suffix('.command.json')
        self.logs=collections.deque(maxlen=10000);self.latest={};self.token=secrets.token_urlsafe(32);self.source_epoch=0;self.capture_override='';self.clear_epoch=0;self.capture_generation=0
        self.phone_hostname=getattr(args,'phone_hostname','');self.phone_token=''
        if self.phone_hostname:
            if not re.fullmatch(r'[a-z0-9-]+\.[a-z0-9-]+\.ts\.net',self.phone_hostname):raise ValueError('Exact private Tailscale hostname required')
            phone_path=self.state.with_name('phone-access.json');previous=json.loads(phone_path.read_text()) if phone_path.exists() else {}
            self.phone_token=previous.get('token') or secrets.token_urlsafe(32)
            if not re.fullmatch(r'[A-Za-z0-9_-]{43}',self.phone_token):raise ValueError('Invalid private URL token')
            phone_path.write_text(json.dumps({'pid':os.getpid(),'token':self.phone_token,'hostname':self.phone_hostname,'url':'http://'+self.phone_hostname+'/'+self.phone_token+'/','read_only':True},indent=2)+'\n',encoding='utf-8')
        self.metrics={'audio':'Waiting for firmware counters','heap':'Waiting for a heap snapshot','counters':'Waiting'}
        self.data={'running':True,'pid':os.getpid(),'port':args.port,'baud':args.baud,'connected':False,'identity_verified':False,'received_lines':0,'received_bytes':0,'reader_mode':'starting','requested_mode':'listening','command_id':'','applied_command_id':'','note_command_id':'','firmware':'unknown','uart_writes':0,'deliberate_resets':0,'last_line_unix':0,'updated_unix':0,'revision':0,'status':'Checking USB identity'}
    def log(self,text,level='info',fields=None,integrity='Unverified',stage='System'):
        with self.lock:
            self.data['revision']+=1
            if len(self.logs)==self.logs.maxlen:self.data['history_dropped']=self.data.get('history_dropped',0)+1
            self.logs.append({'id':self.data['revision'],'time':time.time(),'text':text[:4096],'level':level,'fields':fields,
                'integrity':integrity,'stage':stage,'probe':str((fields or {}).get('probe','uart'))[:120],
                'source':self.data.get('source','Waiting'),'source_id':self.source_epoch})
    def request(self,cid,action,text=''):
        with self.lock:
            if action=='note':self.log('MONITOR: '+text[:1000]);self.data['note_command_id']=cid;return
            if action=='clear':self.logs.clear();self.clear_epoch+=1;self.data['revision']+=1;return
            if action=='replay':
                if text not in {x['path'] for x in self.captures()}:raise ValueError('Capture not admitted')
                self.capture_override=text;self.capture_generation+=1;action='release'
            elif action=='release':self.capture_override=''
            if action not in ('release','resume'):return
            self.control=(cid,action);self.data['command_id']=cid;self.data['requested_mode']='released' if action=='release' else 'listening';self.wake.set()
    def new_source(self,label,kind):
        with self.lock:
            self.source_epoch+=1;self.latest.clear();self.data.update(source=label,source_kind=kind,last_line_unix=0)
            self.data['revision']+=1
    def captures(self):
        paths=list((HARDWARE/'runs').glob('*/COM6-probe-trial.txt'))
        paths+=list((HARDWARE/'runs').glob('*/COM6-pipeline-trial.txt'))
        return [{'path':p.relative_to(HARDWARE).as_posix(),'label':p.parent.name+' · '+p.name,'modified':p.stat().st_mtime}
            for p in sorted(paths,key=lambda p:p.stat().st_mtime,reverse=True)[:60]]
    def line(self,text):
        text=text[:4096]
        fields=None;integrity='Unverified';stage='System'
        try:
            candidate=json.loads(text)
            if isinstance(candidate,dict):fields=candidate
        except (ValueError,TypeError):pass
        if text.startswith('LIVE1 '):
            fields={k:int(v) for k,v in re.findall(r'(\w+)=(-?\d+)',text)};fields['probe']='LIVE1';fields['format']='legacy_text'
        if 'LIVE_DEVICE host=' in text:
            host=re.search(r'host=(\w+)',text)
            if host:
                with self.lock:self.data['firmware']=host[1]
        if fields and ('record_crc32' in fields or 'snapshot_begin_us' in fields):
            integrity='Verified' if audit_capture(text.encode('utf-8'))['passed'] else 'Invalid'
        elif text.startswith('{') and fields is None:integrity='Malformed'
        name=str((fields or {}).get('probe','uart'))
        groups=[('OTA / storage',('ota','upload','recovery')),('BLE / controls',('gatt','smp','cccd','le_security','control','eq_','bond')),
            ('I2S / DMA',('i2s_','native_i2s')),('DSP / render',('render_','audio_clock','dsp_','fir_')),
            ('Decoder / queue',('codec_','sbc_pcm','worker_','decoder_','pcm_','media_')),
            ('Memory / scheduler',('scheduler_','host_phase','owner_wait','owned_heap','heap')),
            ('HCI / transport',('vhci','hci_','rx_flow','controller_','parser_failure')),
            ('Profiles',('avrcp','sdp','rfcomm','native_profile','gap_')),
            ('Product I/O',('product_','startup_sound','sound_','input_','led_'))]
        for label,prefixes in groups:
            if name.startswith(prefixes):stage=label;break
        if name=='LIVE1':stage='Decoder / queue'
        if 'task_wdt:' in text or 'Backtrace:' in text:stage='Memory / scheduler'
        lower=text.lower();level='info'
        if text.startswith('E (') or any(k in lower for k in ('watchdog got triggered','guru meditation','backtrace:','stack overflow')):level='error'
        if fields and (fields.get('failed') or fields.get('faults') or fields.get('descriptor_errors') or name=='parser_failure'):level='error'
        if level=='info' and (integrity in ('Invalid','Malformed') or text.startswith('W (') or (fields and any(fields.get(k,0) for k in ('queue_full','underrun_frames','decode_reject')))):level='warn'
        with self.lock:
            self.data['received_lines']+=1;self.data['last_line_unix']=time.time()
            if fields and integrity not in ('Invalid','Malformed') and (name in self.latest or len(self.latest)<128):self.latest[name]=fields
        self.log(text,level,fields,integrity,stage)
    def file_reader(self):
        key=None;offset=0;pending=b''
        while not self.stop.wait(.2):
            with self.lock:released=self.control[1]=='release' and self.data['reader_mode']=='released' and not self.data['connected'];override=self.capture_override;generation=self.capture_generation
            if not released:key=None;continue
            path=HARDWARE/override if override else HARDWARE/'COM6-probe-trial-live.txt'
            try:
                stat=path.stat();current=(str(path),generation)
                if key!=current or stat.st_size<offset:
                    key=current;offset=max(0,stat.st_size-2*1024*1024);pending=b''
                    self.new_source(('Recorded capture · '+path.parent.name if override else 'Trial capture · COM6 latest run'),'archive' if override else 'capture')
                with path.open('rb') as handle:handle.seek(offset);raw=handle.read(2*1024*1024);offset+=len(raw)
                if raw:
                    with self.lock:self.data['received_bytes']+=len(raw)
                    pending+=raw;lines=pending.split(b'\n');pending=lines.pop()
                    for line in lines:
                        with self.lock:
                            if self.control[1]!='release' or self.capture_override!=override or self.capture_generation!=generation:break
                            self.line(ANSI.sub('',line[:4096].decode('utf-8',errors='replace')).rstrip('\r'))
                    if len(pending)>4096:pending=b''
                with self.lock:self.data['source_updated_unix']=stat.st_mtime
            except OSError:pass
    def reader(self):
        while not self.stop.is_set():
            with self.lock:cid,action=self.control
            if action=='release':
                with self.lock:self.data.update(connected=False,reader_mode='released',applied_command_id=cid,status='COM6 released for flashing; log display remains open')
                self.wake.wait(.2);self.wake.clear();continue
            stream=None
            try:
                found=next((p for p in list_ports.comports() if p.device.upper()==self.args.port.upper()),None)
                if not found or found.vid!=0x10c4 or found.pid!=0xea60 or found.serial_number!=self.args.usb_serial:raise RuntimeError('Admitted speaker USB identity is absent; other ports remain untouched')
                stream=serial.Serial(port=None,baudrate=self.args.baud,timeout=.15,write_timeout=0);stream.dtr=False;stream.rts=False;stream.port=self.args.port;stream.open()
                self.new_source('Live UART · COM6','uart')
                with self.lock:self.data.update(connected=True,identity_verified=True,reader_mode='listening',applied_command_id=cid,status='Connected to the verified speaker; listening without UART commands')
                self.log('Serial reader connected');pending=bytearray()
                while not self.stop.is_set():
                    with self.lock:released=self.control[1]=='release'
                    if released:break
                    with self.lock:self.data['applied_command_id']=self.control[0]
                    raw=stream.read(min(4096,max(1,stream.in_waiting)))
                    if not raw:continue
                    with self.lock:self.data['received_bytes']+=len(raw)
                    pending.extend(raw)
                    while b'\n' in pending:line,_,pending=pending.partition(b'\n');self.line(ANSI.sub('',line[:4096].decode('utf8',errors='replace')).rstrip('\r'))
                    if len(pending)>4096:pending=pending[-4096:];self.log('Long/partial UART line clipped','warn')
            except Exception as e:
                with self.lock:self.data.update(connected=False,status=str(e))
                self.log(str(e),'warn')
            finally:
                if stream:stream.close()
                with self.lock:self.data['connected']=False
            with self.lock:released=self.control[1]=='release'
            if not released:self.wake.wait(2);self.wake.clear()
    def snapshot(self,logs=True):
        with self.lock:return {**self.data,'metrics':dict(self.metrics),'latest':dict(self.latest),'source_id':self.source_epoch,'clear_epoch':self.clear_epoch,
            'retained':len(self.logs),'history_limit':self.logs.maxlen,'oldest_id':self.logs[0]['id'] if self.logs else self.data['revision']+1,
            **({'logs':list(self.logs)} if logs else {})}
    def run(self):
        monitor=self
        class Handler(http.server.BaseHTTPRequestHandler):
            def log_message(self,*args):pass
            def reply(self,status,body,kind):
                self.send_response(status);self.send_header('Content-Type',kind);self.send_header('Cache-Control','no-store');self.send_header('X-Content-Type-Options','nosniff');self.send_header('Content-Length',str(len(body)));self.end_headers()
                try:self.wfile.write(body)
                except (BrokenPipeError,ConnectionResetError):pass
            def do_GET(self):
                host=self.headers.get('Host');remote=False
                parts=urlsplit(self.path);path=parts.path
                if host not in (f'127.0.0.1:{self.server.server_port}',f'localhost:{self.server.server_port}'):
                    if not monitor.phone_hostname or host not in (monitor.phone_hostname,monitor.phone_hostname+':80'):
                        self.reply(403,b'Rejected host','text/plain');return
                    segment,separator,suffix=path.lstrip('/').partition('/')
                    if not separator or not hmac.compare_digest(segment,monitor.phone_token):
                        self.reply(404,b'Private log link required','text/plain');return
                    remote=True;path='/'+suffix
                if path in ('/','/dashboard.js','/dashboard.css'):
                    name={'/':'index.html','/dashboard.js':'dashboard.js','/dashboard.css':'dashboard.css'}[path]
                    kind={'index.html':'text/html; charset=utf-8','dashboard.js':'text/javascript','dashboard.css':'text/css'}[name]
                    body=(ASSETS/name).read_bytes().replace(b'__CONTROL_TOKEN__',monitor.token.encode())
                    if remote:body=remote_asset(path,body,'/'+monitor.phone_token)
                    self.reply(200,body,kind)
                elif path=='/state':self.reply(200,json.dumps(monitor.snapshot(False)).encode(),'application/json')
                elif path=='/captures':self.reply(200,json.dumps(monitor.captures()).encode(),'application/json')
                elif path=='/events':
                    try:after=max(0,int(parse_qs(parts.query).get('after',['0'])[0]))
                    except ValueError:self.reply(400,b'Invalid cursor','text/plain');return
                    with monitor.lock:
                        events=[x for x in monitor.logs if x['id']>after][:400]
                        packet={'state':monitor.snapshot(False),'events':events,'cursor':events[-1]['id'] if events else monitor.data['revision']}
                    self.reply(200,json.dumps(packet).encode(),'application/json')
                elif path=='/export':
                    with monitor.lock:retained=list(monitor.logs)
                    body=b'\n'.join(json.dumps(x).encode() for x in retained)
                    self.reply(200,body,'application/x-ndjson')
                else:self.reply(404,b'Not found','text/plain')
            def do_POST(self):
                origin=self.headers.get('Origin');expected=f'http://127.0.0.1:{self.server.server_port}'
                if self.headers.get('Host') not in (f'127.0.0.1:{self.server.server_port}',f'localhost:{self.server.server_port}') or urlsplit(self.path).path!='/command' or (origin and origin!=expected) or self.headers.get('Content-Type')!='application/json' or self.headers.get('X-Monitor-Token')!=monitor.token:self.reply(403,b'Rejected','text/plain');return
                try:
                    n=int(self.headers.get('Content-Length','0'));assert 0<n<=4096;value=json.loads(self.rfile.read(n));action=value['action'];assert action in ('release','resume','replay','clear')
                    cid=uuid.uuid4().hex;monitor.request(cid,action,str(value.get('path','')));self.reply(200,json.dumps({'id':cid}).encode(),'application/json')
                except (ValueError,KeyError,AssertionError):self.reply(400,b'Invalid command','text/plain')
        server=http.server.ThreadingHTTPServer(('127.0.0.1',self.args.http_port),Handler);server.daemon_threads=True
        with self.lock:self.data['url']=f'http://127.0.0.1:{server.server_port}/'
        worker=threading.Thread(target=self.reader,daemon=False);tail=threading.Thread(target=self.file_reader,daemon=False)
        http_thread=threading.Thread(target=server.serve_forever,daemon=True);worker.start();tail.start();http_thread.start();last=0
        try:
            while not self.stop.wait(.1):
                try:
                    if self.command.is_file() and self.command.stat().st_size<=4096:
                        c=json.loads(self.command.read_text());cid=c.get('id','')
                        if c.get('pid')==os.getpid() and isinstance(cid,str) and 0<len(cid)<=64 and cid!=self.seen and c.get('action') in ('release','resume','note'):
                            self.seen=cid;self.request(cid,c['action'],str(c.get('text',''))[:1000])
                except (OSError,ValueError):pass
                now=time.time()
                if now-last>=.5:
                    with self.lock:self.data['updated_unix']=now
                    try:
                        temp=self.state.with_suffix('.tmp');temp.write_text(json.dumps(self.snapshot(False),indent=2)+'\n',encoding='utf-8');temp.replace(self.state)
                    except OSError:pass
                    last=now
        finally:
            self.stop.set();self.wake.set();worker.join(2);tail.join(2);server.shutdown();server.server_close()
            with self.lock:self.data.update(running=False,connected=False)
            self.state.write_text(json.dumps(self.snapshot(False),indent=2)+'\n')
def main():
    p=argparse.ArgumentParser();p.add_argument('--port',required=True);p.add_argument('--usb-serial',required=True);p.add_argument('--baud',type=int,default=115200);p.add_argument('--state',required=True);p.add_argument('--http-port',type=int,default=8765);p.add_argument('--released',action='store_true');p.add_argument('--restore-buffer',type=Path);p.add_argument('--phone-hostname',default='');args=p.parse_args()
    if args.port.upper()!='COM6' or args.usb_serial!='CA0ACAD67933EF1180C36117A04C7D1C':p.error('Only the released COM6 speaker is admitted')
    args.port='COM6';monitor=Monitor(args)
    if args.restore_buffer:
        path=args.restore_buffer.resolve();path.relative_to((ROOT/'_build').resolve())
        if path.stat().st_size>64*1024*1024:raise ValueError('History checkpoint exceeds 64 MiB')
        checkpoint=json.loads(path.read_text(encoding='utf-8'));records=checkpoint['logs'][-10000:]
        for item in records:
            if not isinstance(item.get('id'),int) or not isinstance(item.get('source_id'),int) or not isinstance(item.get('text'),str) or len(item['text'])>4096:raise ValueError('Invalid history record')
        monitor.logs.extend(records);monitor.source_epoch=max((x['source_id'] for x in records),default=0)
        monitor.data['revision']=max((x['id'] for x in records),default=0)
        for key in ('received_lines','received_bytes','history_dropped'):monitor.data[key]=int(checkpoint['state'].get(key,0))
    monitor.run()
if __name__=='__main__':main()
