"""Known-input level measurements through actual aptX/LDAC codec workers.
Uses pinned independent AOSP LDAC encoder; no hardware or audio playback.
"""
import ctypes,hashlib,importlib, json,math,os,pathlib,shutil,struct,subprocess,sys
ROOT=pathlib.Path(__file__).resolve().parents[1]
p=__import__('argparse').ArgumentParser();p.add_argument('--out',type=pathlib.Path,required=True);a=p.parse_args()
OUT=a.out.resolve();OUT.relative_to((ROOT/'_build').resolve());OUT.mkdir(parents=True,exist_ok=True)
(OUT/'levels.json').write_text(json.dumps({'passed':False,'state':'building'})+'\n')
cc=shutil.which('clang') or r'C:\Program Files\LLVM\bin\clang.exe'
cache=shutil.which('ccache')
if not cache:
 found=sorted((pathlib.Path.home()/'.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'));cache=str(found[-1]) if found else None
assert cache,'ccache required'
env=os.environ.copy()
if os.name=='nt':
 vc=pathlib.Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
 values=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
 env.update(dict(x.split('=',1) for x in values.splitlines() if '=' in x and not x.startswith('=')))
def run(args,**kw):
 r=subprocess.run(list(map(str,args)),capture_output=True,text=True,**kw)
 if r.returncode:raise RuntimeError(r.stdout[-2000:]+'\n'+r.stderr[-4000:])
 return r.stdout
def cached_build(args,**kw):
 args=list(map(str,args));sources=[x for x in args[1:] if x.endswith('.c')]
 if pathlib.Path(args[0]).name.lower() not in ('clang','clang.exe','gcc','gcc.exe','cc') or not sources:
  return run(args,**kw)
 dest=args[args.index('-o')+1];skip={'-shared','-fPIC','-lm',*sources};flags=[];i=1
 while i<len(args):
  if args[i]=='-o':i+=2;continue
  if args[i] not in skip:flags.append(args[i])
  i+=1
 objects=[]
 for src in sources:
  obj=OUT/(hashlib.sha256((src+' '.join(flags)).encode()).hexdigest()[:16]+'.obj');objects.append(str(obj))
  run([cache,args[0],*flags,'-c',src,'-o',obj],**kw)
 link=[args[0],*objects,'-shared','-o',dest]+(['-lm'] if '-lm' in args else [])
 return run(link,**kw)
sys.path.insert(0,str(ROOT/'tools'))
apt=importlib.import_module('test_native_codec_aptx');ld=importlib.import_module('test_native_codec_ldac_real')
apt.run=cached_build;ld.run=cached_build
apt_dll,apt_sources=apt.build();ld_dll,_,ld_sources=ld.build()
tracked=set(map(pathlib.Path,apt_sources+ld_sources))
ENC=ROOT/'_build/native_codecs/ldac_fixture_encoder'
encoder_files=sorted([*(ENC/'src').glob('*.c'),*(ENC/'src').glob('*.h'),*(ENC/'inc').glob('*.h'),ENC/'LICENSE',ENC/'NOTICE'],key=lambda f:f.relative_to(ENC).as_posix())
h=hashlib.sha256()
for f in encoder_files:h.update(f.relative_to(ENC).as_posix().encode()+b'\0');h.update(f.read_bytes())
assert len(encoder_files)==32 and h.hexdigest()=='9e38fc695ea63c20d60692722006fd65ebf592d6cc04cdf0dc634c86862f2c78'
compat=OUT/'compat';compat.mkdir(exist_ok=True);(compat/'unistd.h').write_text('/* Fixture uses no unistd symbols. */\n')
encoder_sources=[ENC/'src/ldaclib.c',ENC/'src/ldacBT.c',ROOT/'tests/codecs/ldac_level_fixture.c']
objects=[]
for f in encoder_sources:
 obj=OUT/('encoder_'+f.stem+'.obj');objects.append(obj)
 run([cache,cc,'-std=c11','-O2','-fwrapv','-D_CRT_SECURE_NO_WARNINGS','-I'+str(compat),'-I'+str(ENC/'inc'),'-I'+str(ENC/'src'),'-c',f,'-o',obj],env=env)
generator=OUT/('ldac_levels.exe' if os.name=='nt' else 'ldac_levels')
run([cc,*objects,'-o',generator]+([] if os.name=='nt' else ['-lm']),env=env)
tracked.update(encoder_files+encoder_sources+[pathlib.Path(__file__).resolve()])
source_hashes={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in tracked}
u8=ctypes.c_uint8;u=ctypes.c_uint;ptr=ctypes.POINTER(u8)
def buf(data):return (u8*len(data)).from_buffer_copy(data)
def bind(path):
 lib=ctypes.CDLL(str(path));lib.worker_control.argtypes=[u,ptr,u];lib.worker_packet.argtypes=[ptr,u,u,ptr,ctypes.POINTER(u)]
 return lib
def begin(lib,cie):
 assert lib.worker_init()==0
 assert lib.worker_control(0,buf(cie),len(cie))==0 and lib.worker_control(3,None,0)==0
def decode(lib,data,seq):
 pcm=(u8*8192)();n=u();before=lib.worker_drops()
 assert lib.worker_packet(buf(data),len(data),seq,pcm,ctypes.byref(n))==0
 assert lib.worker_drops()==before
 return bytes(pcm[:n.value])
def finish(lib):assert lib.worker_control(5,None,0)==0 and lib.probe_live()==0
def fit(pcm,rate,start,amplitude):
 values=struct.unpack('<'+'i'*(len(pcm)//4),pcm);n=len(values)//2;stop=n-512
 assert stop>start+1000
 channels=[]
 for channel,freq in enumerate((997,1300)):
  ss=ccv=sc=ys=yc=power=peak=0.0
  for i in range(start,stop):
   v=values[2*i+channel]/2147483648.0;angle=2*math.pi*freq*i/rate;s=math.sin(angle);c=math.cos(angle)
   ss+=s*s;ccv+=c*c;sc+=s*c;ys+=v*s;yc+=v*c;power+=v*v;peak=max(peak,abs(v))
  det=ss*ccv-sc*sc;cs=(ys*ccv-yc*sc)/det;co=(yc*ss-ys*sc)/det;fund=math.hypot(cs,co)
  channels.append({'gain_db':20*math.log10(fund/amplitude),'peak':peak,'rms':math.sqrt(power/(stop-start))})
 return channels
rows=[];lib=bind(apt_dll);lib.reference_encode.argtypes=[ptr,u,ptr,u,ctypes.POINTER(u)]
for kind in ('standard','hd','ll'):
 for rate in (44100,48000):
  for amplitude in (0.1,0.8):
   assert lib.reference_open(kind=='hd')==0
   raw=bytearray()
   for i in range(4800):
    for freq in (997,1300):
     sample=round(amplitude*8388608*math.sin(2*math.pi*freq*i/rate));raw.extend((sample&0xffffff).to_bytes(3,'little'))
   encoded=(u8*16384)();written=u();assert lib.reference_encode(buf(raw),len(raw),encoded,len(encoded),ctypes.byref(written))==len(raw)
   stream=bytes(encoded[:written.value]);begin(lib,apt.cie(kind,rate));pcm=bytearray()
   for seq,offset in enumerate(range(0,len(stream),192)):pcm.extend(decode(lib,stream[offset:offset+192],seq))
   finish(lib);lib.reference_close()
   row={'codec':'aptx_'+kind,'rate':rate,'input_depth':24,'input_peak':amplitude,'channels':fit(pcm,rate,512,amplitude),'pcm_sha256':hashlib.sha256(pcm).hexdigest(),'encoded_sha256':hashlib.sha256(stream).hexdigest()};rows.append(row);print(json.dumps(row),flush=True)
lib=bind(ld_dll)
for rate in (44100,48000,88200,96000):
 for depth in (16,24):
  for amplitude in (0.1,0.8):
   path=OUT/f'ldac-{rate}-{depth}-{amplitude}.frames'
   run([generator,rate,1,0,path,depth,amplitude],env=env)
   stream=path.read_bytes();frames=[];pos=0
   while pos<len(stream):
    assert stream[pos]==0xaa
    n=3+((((stream[pos+1]&7)<<6)|(stream[pos+2]>>2))+1);assert n<=len(stream)-pos
    frames.append(stream[pos:pos+n]);pos+=n
   begin(lib,ld.cie(rate,1));pcm=bytearray()
   for seq,index in enumerate(range(0,len(frames),2)):
    batch=frames[index:index+2];pcm.extend(decode(lib,bytes([len(batch)])+b''.join(batch),seq))
   finish(lib)
   row={'codec':'ldac','rate':rate,'input_depth':depth,'input_peak':amplitude,'channels':fit(pcm,rate,1536,amplitude),'pcm_sha256':hashlib.sha256(pcm).hexdigest(),'encoded_sha256':hashlib.sha256(stream).hexdigest()};rows.append(row);print(json.dumps(row),flush=True)
assert source_hashes=={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in tracked}
failures=[r for r in rows if any(abs(c['gain_db'])>0.1 for c in r['channels'])]
(OUT/'levels.json').write_text(json.dumps({'passed':not failures,'rows':rows,'failures':failures,'source_sha256':source_hashes,'hardware_accessed':False,'scope':'known tone amplitude through actual worker/backend before DSP; aptX C99 encoder and pinned independent AOSP LDAC encoder; not acoustic or RF qualification','ccache_object_builds':True},indent=2)+'\n')
if failures:raise SystemExit('Codec output gain mismatch; see levels.json')
print('PASS',len(rows),'known-input codec level cases')
