"""Loopback serial dashboard: bounded RAM history, no UART writes, explicit port lease."""
import argparse,collections,http.server,json,os,re,threading,time,uuid
from pathlib import Path
from urllib.parse import urlsplit
import serial
from serial.tools import list_ports
ROOT=Path(__file__).resolve().parents[1]
ANSI=re.compile(r'\x1b\[[0-?]*[ -/]*[@-~]')
HTML=r'''<!doctype html><meta charset="utf-8"><title>ESP32 serial monitor</title><style>
body{margin:0;background:#0e1726;color:#e4edf9;font:14px system-ui}main{padding:18px;max-width:1400px;margin:auto}h1{font-size:22px;margin:0 0 5px}.muted{color:#aab8cc}.bar,.cards{display:flex;gap:10px;flex-wrap:wrap;margin:15px 0}.card{background:#1c2a40;padding:12px;border-radius:8px;flex:1;min-width:190px}.card b{display:block;color:#aebfd7;font-size:12px;margin-bottom:6px}button{padding:9px 13px;background:#23446d;color:white;border:1px solid #5478a5;border-radius:6px;cursor:pointer}button:disabled{opacity:.5}#status{padding:10px;background:#203653;border-radius:6px}#log{background:#080f1b;border:1px solid #30435e;height:53vh;overflow:auto;padding:10px;font:12px Consolas,monospace;white-space:pre-wrap;overflow-wrap:anywhere}.line{padding:3px 0}.error{color:#ffb3b3}.warn{color:#ffd580}footer{font-size:12px;color:#aab8cc;margin-top:10px}label{align-self:center}</style>
<main><h1>ESP32 serial monitor</h1><div class="muted">COM6 · 115,200 baud · read only</div><div id="status">Connecting to monitor service…</div>
<div class="cards"><div class="card"><b>AUDIO</b><span id="audio">Waiting</span></div><div class="card"><b>HEAP</b><span id="heap">Waiting</span></div><div class="card"><b>COUNTERS</b><span id="counters">Waiting</span></div></div>
<div class="bar"><button onclick="command('release')">Release COM6 for flash</button><button onclick="command('resume')">Resume logging</button><button onclick="paused=!paused;this.textContent=paused?'Resume display':'Pause display'">Pause display</button><label><input id="follow" type="checkbox" checked> Auto-scroll</label></div>
<div id="log"></div><footer id="footer">200 events retained in RAM. No raw log files. Digital counters do not establish audible quality.</footer></main>
<script>let paused=false,last=-1;async function command(action){await fetch('/command',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action})})}async function refresh(){try{const s=await(await fetch('/state',{cache:'no-store'})).json();document.querySelector('#status').textContent=s.status+' | Host: '+s.firmware;for(const k of ['audio','heap','counters'])document.getElementById(k).textContent=s.metrics[k];document.querySelector('#footer').textContent=`${s.received_lines.toLocaleString()} lines / ${s.received_bytes.toLocaleString()} bytes · UART ${s.reader_mode} · 200 events in RAM, no raw log files`;if(!paused&&last!==s.revision){last=s.revision;const box=document.querySelector('#log');box.replaceChildren(...s.logs.map(x=>{const d=document.createElement('div');d.className='line '+x.level;d.textContent=new Date(x.time*1000).toLocaleTimeString()+'  '+x.text;return d}));if(document.querySelector('#follow').checked)box.scrollTop=box.scrollHeight}}catch(e){document.querySelector('#status').textContent='Monitor service unavailable: '+e.message}setTimeout(refresh,500)}refresh()</script>'''
class Monitor:
    def __init__(self,args):
        self.args=args;self.stop=threading.Event();self.wake=threading.Event();self.lock=threading.RLock();self.control=('', 'release' if args.released else 'resume');self.seen='';self.clock_text='Clock waiting';self.i2s_text='I2S waiting'
        self.state=Path(args.state).resolve();self.state.relative_to((ROOT/'_build').resolve());self.state.parent.mkdir(parents=True,exist_ok=True);self.command=self.state.with_suffix('.command.json')
        self.logs=collections.deque(maxlen=200);self.metrics={'audio':'Waiting for firmware counters','heap':'Waiting for a heap snapshot','counters':'Waiting'}
        self.data={'running':True,'pid':os.getpid(),'port':args.port,'baud':args.baud,'connected':False,'identity_verified':False,'received_lines':0,'received_bytes':0,'reader_mode':'starting','requested_mode':'listening','command_id':'','applied_command_id':'','note_command_id':'','firmware':'unknown','uart_writes':0,'deliberate_resets':0,'last_line_unix':0,'updated_unix':0,'revision':0,'status':'Checking USB identity'}
    def log(self,text,level='info'):
        with self.lock:self.logs.append({'time':time.time(),'text':text[:1200],'level':level});self.data['revision']+=1
    def request(self,cid,action,text=''):
        with self.lock:
            if action=='note':self.log('MONITOR: '+text[:1000]);self.data['note_command_id']=cid;return
            if action not in ('release','resume'):return
            self.control=(cid,action);self.data['command_id']=cid;self.data['requested_mode']='released' if action=='release' else 'listening';self.wake.set()
    def line(self,text):
        with self.lock:
            self.data['received_lines']+=1;self.data['last_line_unix']=time.time()
            if text.startswith('LIVE1 '):
                f={k:int(v) for k,v in re.findall(r'(\w+)=(-?\d+)',text)};mode='PLAYING' if f.get('started') and f.get('active') else 'IDLE'
                self.metrics['audio']=f"{mode} · {f.get('rate',0)/1000:g} kHz · {f.get('bits',0)} bit · {f.get('ch',0)} channels"
                self.metrics['heap']=f"Live: {f.get('heap',0)/1024:.1f} KiB free · largest {f.get('largest',0)/1024:.1f} KiB"
                self.metrics['counters']=f"PCM callbacks {f.get('pcm_cb',0)} · under {f.get('under',0)} · drops {f.get('drops',0)} · I2S errors {f.get('i2s_err',0)}";return
            if 'LIVE_DEVICE host=' in text:
                value=re.search(r'host=(\w+)',text)
                if value:self.data['firmware']=value[1]
            if text.startswith('{'):
                try:
                    f=json.loads(text);name=f.get('probe','event')
                    if name=='controller_identified':
                        self.data['firmware']='owned development host';self.metrics['heap']=f"At controller init: {f.get('internal_free',0)/1024:.1f} KiB · largest {f.get('internal_largest',0)/1024:.1f} KiB"
                    if name=='owned_heap':self.metrics['heap']=f"Live: {f.get('free_kib',0)} KiB free · largest {f.get('largest_kib',0)} KiB";return
                    if name=='sbc_pcm':
                        self.data['firmware']='owned development host'
                        codec={0:'Waiting for codec',1:'SBC',2:'Opus Android',3:'Opus05',4:'AAC-LC',5:'aptX',6:'aptX HD',7:'aptX LL',8:'LDAC',9:'LHDC V5'}.get(f.get('codec',0),'Unknown codec')
                        self.metrics['audio']=f"{codec} · {f.get('rate',0)/1000:g} kHz · {f.get('sample_bytes',0)*8} bit · {f.get('channels',0)} channels · frames {f.get('frames',0)}"
                        if 'internal_heap_free' in f:self.metrics['heap']=f"Live: {f['internal_heap_free']/1024:.1f} KiB free · largest {f['internal_heap_largest']/1024:.1f} KiB · decoder stack low {f.get('decoder_stack_low_water_bytes','?')} bytes"
                        self.metrics['counters']=f"Media drops {f.get('drops','?')} · decoder faults {f.get('faults','?')}"
                        return
                    if name=='native_i2s':
                        self.i2s_text=f"I2S {f.get('running','?')} · underrun frames {f.get('underrun_frames','?')} · descriptor errors {f.get('descriptor_errors','?')}"
                        self.metrics['counters']=self.i2s_text+' | '+self.clock_text;return
                    if name=='audio_clock':
                        phase={0:'off',1:'calibrating',2:'tracking',3:'recovering'}.get(f.get('state'),'unknown')
                        self.clock_text=f"Clock {phase}: correction {f.get('ppm',0):+d} ppm"
                        if f.get('observed_valid') and f.get('snapshot_valid'):self.clock_text+=f" · estimate {f['observed_ppm']:+d} ppm"
                        self.metrics['counters']=self.i2s_text+' | '+self.clock_text;return
                    text=name+': '+', '.join(f'{k}={v}' for k,v in f.items() if k!='probe')
                except (ValueError,TypeError):pass
        level='error' if text.startswith('E (') or any(x in text.lower() for x in ('panic','guru meditation','parser_failure','stack overflow','***error***','backtrace')) else 'warn' if text.startswith('W (') else 'info';self.log(text,level)
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
        with self.lock:return {**self.data,'metrics':dict(self.metrics),**({'logs':list(self.logs)} if logs else {})}
    def run(self):
        monitor=self
        class Handler(http.server.BaseHTTPRequestHandler):
            def log_message(self,*args):pass
            def reply(self,status,body,kind):
                self.send_response(status);self.send_header('Content-Type',kind);self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
            def do_GET(self):
                path=urlsplit(self.path).path
                if path=='/':self.reply(200,HTML.encode(),'text/html; charset=utf-8')
                elif path=='/state':self.reply(200,json.dumps(monitor.snapshot()).encode(),'application/json')
                else:self.reply(404,b'Not found','text/plain')
            def do_POST(self):
                origin=self.headers.get('Origin');expected=f'http://127.0.0.1:{self.server.server_port}'
                if urlsplit(self.path).path!='/command' or (origin and origin!=expected) or self.headers.get('Content-Type')!='application/json':self.reply(403,b'Rejected','text/plain');return
                try:
                    n=int(self.headers.get('Content-Length','0'));assert 0<n<=4096;value=json.loads(self.rfile.read(n));action=value['action'];assert action in ('release','resume')
                    cid=uuid.uuid4().hex;monitor.request(cid,action);self.reply(200,json.dumps({'id':cid}).encode(),'application/json')
                except (ValueError,KeyError,AssertionError):self.reply(400,b'Invalid command','text/plain')
        server=http.server.ThreadingHTTPServer(('127.0.0.1',self.args.http_port),Handler);server.daemon_threads=True
        with self.lock:self.data['url']=f'http://127.0.0.1:{server.server_port}/'
        worker=threading.Thread(target=self.reader,daemon=False);http_thread=threading.Thread(target=server.serve_forever,daemon=True);worker.start();http_thread.start();last=0
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
                    temp=self.state.with_suffix('.tmp');temp.write_text(json.dumps(self.snapshot(False),indent=2)+'\n');temp.replace(self.state);last=now
        finally:
            self.stop.set();self.wake.set();worker.join(1);server.shutdown()
            with self.lock:self.data.update(running=False,connected=False)
            self.state.write_text(json.dumps(self.snapshot(False),indent=2)+'\n')
def main():
    p=argparse.ArgumentParser();p.add_argument('--port',required=True);p.add_argument('--usb-serial',required=True);p.add_argument('--baud',type=int,default=115200);p.add_argument('--state',required=True);p.add_argument('--http-port',type=int,default=0);p.add_argument('--released',action='store_true');Monitor(p.parse_args()).run()
if __name__=='__main__':main()
