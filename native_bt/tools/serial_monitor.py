"""Read-only, bounded, nonblocking ESP32 serial dashboard. No UART writes."""
import argparse,json,os,queue,re,threading,time
from pathlib import Path
import tkinter as tk
from tkinter import ttk
import serial
from serial.tools import list_ports
ROOT=Path(__file__).resolve().parents[1]
ANSI=re.compile(r'\x1b\[[0-?]*[ -/]*[@-~]')
class Monitor:
    def __init__(self,args):
        self.args=args;self.stop=threading.Event();self.changed=threading.Event();self.control_lock=threading.Lock();self.control=('', 'resume');self.events=queue.Queue(maxsize=128)
        self.stats={'running':True,'pid':os.getpid(),'port':args.port,'baud':args.baud,'connected':False,'identity_verified':False,'received_bytes':0,'received_lines':0,'ui_skipped':0,'firmware':'unknown','uart_writes':0,'deliberate_resets':0,'last_line_unix':0,'updated_unix':0,'reader_mode':'starting','command_id':'','applied_command_id':'','requested_mode':'listening','note_command_id':''}
        self.state=Path(args.state).resolve();self.state.relative_to((ROOT/'_build').resolve());self.state.parent.mkdir(parents=True,exist_ok=True);self.command=self.state.with_suffix('.command.json');self.seen_command=''
        self.root=tk.Tk();self.root.title('ESP32 Serial Monitor - read only');self.root.geometry('1040x660');self.root.minsize(800,480);self.root.configure(bg='#111827')
        self.paused=False;self.rows=0;self.last_health=0;self.last_audio=None
        tk.Label(self.root,text=f'ESP32  |  {args.port}  |  {args.baud:,} baud  |  READ ONLY',font=('Segoe UI',16,'bold'),bg='#111827',fg='#eff6ff',anchor='w').pack(fill='x',padx=18,pady=(16,4))
        self.status=tk.StringVar(value='Checking the admitted USB device...');tk.Label(self.root,textvariable=self.status,font=('Segoe UI',11),bg='#111827',fg='#93c5fd',anchor='w').pack(fill='x',padx=18)
        self.audio=tk.StringVar(value='Audio: waiting for firmware counters');self.heap=tk.StringVar(value='Heap: waiting');self.counters=tk.StringVar(value='Counters: waiting')
        for var in (self.audio,self.heap,self.counters):tk.Label(self.root,textvariable=var,font=('Consolas',11),bg='#1f2937',fg='#e5e7eb',anchor='w',padx=10,pady=5).pack(fill='x',padx=18,pady=3)
        bar=tk.Frame(self.root,bg='#111827');bar.pack(fill='x',padx=18,pady=8)
        self.pause=ttk.Button(bar,text='Pause log',command=self.toggle);self.pause.pack(side='left');self.port_button=ttk.Button(bar,text='Release port',command=self.toggle_port);self.port_button.pack(side='left',padx=8)
        ttk.Button(bar,text='Clear log',command=self.clear).pack(side='left',padx=8);ttk.Button(bar,text='Copy visible log',command=self.copy_log).pack(side='left')
        self.follow=tk.BooleanVar(value=True);tk.Checkbutton(bar,text='Auto-scroll',variable=self.follow,bg='#111827',fg='#e5e7eb',selectcolor='#1f2937',activebackground='#111827').pack(side='right')
        box=tk.Frame(self.root,bg='#111827');box.pack(fill='both',expand=True,padx=18,pady=(0,8))
        self.log=tk.Text(box,bg='#0b1220',fg='#dbeafe',font=('Consolas',10),wrap='word',state='disabled',borderwidth=0,padx=10,pady=8)
        scroll=ttk.Scrollbar(box,command=self.log.yview);self.log.configure(yscrollcommand=scroll.set);scroll.pack(side='right',fill='y');self.log.pack(fill='both',expand=True)
        self.log.tag_configure('error',foreground='#fca5a5');self.log.tag_configure('warn',foreground='#fcd34d');self.log.tag_configure('info',foreground='#bfdbfe')
        self.footer=tk.StringVar(value='Last 200 events in memory; no raw log files. Digital counters do not establish audible quality.')
        tk.Label(self.root,textvariable=self.footer,bg='#111827',fg='#9ca3af',font=('Segoe UI',9),anchor='w').pack(fill='x',padx=18,pady=(0,12))
        self.root.protocol('WM_DELETE_WINDOW',self.close);self.thread=threading.Thread(target=self.read,name='serial-reader',daemon=False);self.thread.start();self.root.after(100,self.tick);self.root.after(300,self.root.lift)
    def put(self,kind,text):
        try:self.events.put_nowait((kind,text))
        except queue.Full:self.stats['ui_skipped']+=1
    def admitted(self):
        match=next((p for p in list_ports.comports() if p.device.upper()==self.args.port.upper()),None)
        return bool(match and match.vid==0x10c4 and match.pid==0xea60 and match.serial_number==self.args.usb_serial)
    def command_snapshot(self):
        with self.control_lock:return self.control
    def request(self,command_id,action):
        with self.control_lock:self.control=(command_id,action)
        self.stats['command_id']=command_id;self.stats['requested_mode']='released' if action=='release' else 'listening';self.changed.set()
    def commands(self):
        try:
            if not self.command.is_file() or self.command.stat().st_size>4096:return
            value=json.loads(self.command.read_text());cid=value.get('id','');action=value.get('action')
            if value.get('pid')!=os.getpid() or not isinstance(cid,str) or not 0<len(cid)<=64 or cid==self.seen_command:return
            if action not in ('release','resume','note'):return
            self.seen_command=cid
            if action=='note':self.append('MONITOR: '+str(value.get('text',''))[:1000]);self.stats['note_command_id']=cid
            else:self.request(cid,action)
        except (OSError,ValueError):pass
    def toggle_port(self):
        import uuid
        self.request(uuid.uuid4().hex,'resume' if self.command_snapshot()[1]=='release' else 'release')
    def read(self):
        while not self.stop.is_set():
            cid,action=self.command_snapshot()
            if action=='release':
                self.stats['connected']=False;self.stats['reader_mode']='released';self.stats['applied_command_id']=cid
                self.changed.wait(0.2);self.changed.clear();continue
            stream=None
            try:
                if not self.admitted():raise RuntimeError('Admitted speaker USB identity is absent; other ports are untouched')
                self.stats['identity_verified']=True
                stream=serial.Serial(port=None,baudrate=self.args.baud,timeout=0.15,write_timeout=0)
                stream.dtr=False;stream.rts=False;stream.port=self.args.port;stream.open()
                self.stats['connected']=True;self.stats['reader_mode']='listening';self.stats['applied_command_id']=cid;self.put('status','Connected to verified CP210x speaker. Listening without commands or reset.')
                pending=bytearray()
                while not self.stop.is_set() and self.command_snapshot()[1]!='release':
                    data=stream.read(min(4096,max(1,stream.in_waiting)))
                    if not data:continue
                    self.stats['received_bytes']+=len(data);pending.extend(data)
                    while b'\n' in pending:
                        line,_,pending=pending.partition(b'\n');self.stats['received_lines']+=1;self.stats['last_line_unix']=time.time()
                        self.put('line',ANSI.sub('',line[:4096].decode('utf-8',errors='replace')).strip('\r'))
                    if len(pending)>4096:self.put('warn','Long/partial UART line clipped for display');pending=pending[-4096:]
            except Exception as e:
                self.put('status',str(e));self.stats['connected']=False
            finally:
                if stream:stream.close()
                self.stats['connected']=False
            if self.command_snapshot()[1]!='release':self.changed.wait(2);self.changed.clear()
    def append(self,text,tag='info'):
        if self.paused:return
        self.log.configure(state='normal');self.log.insert('end',time.strftime('%H:%M:%S')+'  '+text[:1200]+'\n',tag);self.rows+=1
        if self.rows>200:self.log.delete('1.0',f'{self.rows-199}.0');self.rows=200
        if self.follow.get():self.log.see('end')
        self.log.configure(state='disabled')
    def line(self,text):
        if text.startswith('LIVE1 '):
            fields={k:int(v) for k,v in re.findall(r'(\w+)=(-?\d+)',text)}
            active=bool(fields.get('started') and fields.get('active'));mode='PLAYING' if active else 'IDLE'
            self.audio.set(f"Audio: {mode}  |  {fields.get('rate',0)/1000:g} kHz  |  {fields.get('bits',0)} bit  |  {fields.get('ch',0)} channels")
            self.heap.set(f"Heap free: {fields.get('heap',0)/1024:.1f} KiB  |  largest block: {fields.get('largest',0)/1024:.1f} KiB")
            self.counters.set(f"PCM callbacks: {fields.get('pcm_cb',0)}  |  under counter: {fields.get('under',0)}  |  drops: {fields.get('drops',0)}  |  I2S errors: {fields.get('i2s_err',0)}")
            if active!=self.last_audio:self.append('Audio state: '+mode);self.last_audio=active
            return
        if 'LIVE_DEVICE host=' in text:
            m=re.search(r'host=(\w+)',text)
            if m:self.stats['firmware']=m[1]
        if text.startswith('{'):
            try:
                item=json.loads(text);name=item.get('probe','event')
                if name=='controller_identified':self.stats['firmware']='independent probe'
                if name=='sbc_pcm':self.audio.set(f"Decoder: SBC  |  {item.get('rate',0)/1000:g} kHz  |  frames {item.get('frames',0)}  |  drops {item.get('drops',0)}");return
                if name=='native_i2s':self.counters.set(f"I2S running: {item.get('running',0)}  |  underrun frames: {item.get('underrun_frames',0)}  |  descriptor errors: {item.get('descriptor_errors',0)}");return
                text=name+': '+', '.join(f'{k}={v}' for k,v in item.items() if k!='probe')
            except (ValueError,TypeError):pass
        tag='error' if text.startswith('E (') or any(x in text.lower() for x in ('panic','guru meditation','parser_failure')) else 'warn' if text.startswith('W (') else 'info'
        self.append(text,tag)
    def tick(self):
        self.commands()
        released=self.stats['reader_mode']=='released'
        self.port_button.configure(text='Resume port' if released else 'Release port')
        if released:self.status.set('COM6 released for flashing; window and log remain open.')
        for _ in range(100):
            try:kind,text=self.events.get_nowait()
            except queue.Empty:break
            if kind=='status':self.status.set(text);self.append(text,'warn' if not self.stats['connected'] else 'info')
            elif kind=='line':self.line(text)
            else:self.append(text,kind)
        now=time.time()
        if now-self.last_health>=1:
            self.last_health=now;self.stats['updated_unix']=now;self.stats['running']=not self.stop.is_set()
            temp=self.state.with_suffix('.tmp');temp.write_text(json.dumps(self.stats,indent=2)+'\n');temp.replace(self.state)
            self.footer.set(f"Received {self.stats['received_lines']:,} lines / {self.stats['received_bytes']:,} bytes  |  UI skipped {self.stats['ui_skipped']}  |  200 events in memory, no raw log files")
        if not self.stop.is_set():self.root.after(100,self.tick)
    def toggle(self):self.paused=not self.paused;self.pause.configure(text='Resume log' if self.paused else 'Pause log')
    def clear(self):self.log.configure(state='normal');self.log.delete('1.0','end');self.log.configure(state='disabled');self.rows=0
    def copy_log(self):self.root.clipboard_clear();self.root.clipboard_append(self.log.get('1.0','end-1c'))
    def close(self):
        self.stop.set();self.changed.set();self.stats['running']=False;self.root.destroy()
    def run(self):
        self.root.mainloop();self.stop.set();self.changed.set();self.thread.join(1);self.stats['running']=False;self.stats['connected']=False
        self.state.write_text(json.dumps(self.stats,indent=2)+'\n')
def main():
    p=argparse.ArgumentParser();p.add_argument('--port',required=True);p.add_argument('--usb-serial',required=True);p.add_argument('--baud',type=int,default=115200);p.add_argument('--state',required=True);args=p.parse_args()
    Monitor(args).run()
if __name__=='__main__':main()
