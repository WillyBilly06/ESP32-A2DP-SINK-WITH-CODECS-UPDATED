"""Measure LHDC PCM level against known input, not another copy of the decoder.
Host-only: target selector/math, quality counters, two channels, all four rates.
No RF, target timing, DAC or acoustic qualification is implied.
"""
import argparse,ctypes,hashlib,json,math,os,pathlib,re,shutil,struct,subprocess,sys
ROOT=pathlib.Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--encoder',type=pathlib.Path,required=True);p.add_argument('--decoder',type=pathlib.Path,required=True);p.add_argument('--out',type=pathlib.Path,required=True);p.add_argument('--refresh-points',action='store_true');a=p.parse_args()
out=a.out.resolve();out.relative_to((ROOT/'_build').resolve());out.mkdir(parents=True,exist_ok=True)
enc=a.encoder.resolve();dec=a.decoder.resolve();port=out/'port'
def run(args,**kw):
 r=subprocess.run(list(map(str,args)),capture_output=True,text=True,**kw)
 if r.returncode:raise RuntimeError(str(args[:3])+'\n'+r.stdout[-2000:]+'\n'+r.stderr[-5000:])
 return r.stdout
env=os.environ.copy()
if os.name=='nt':
 vc=pathlib.Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
 data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
 env.update(dict(x.split('=',1) for x in data.splitlines() if '=' in x and not x.startswith('=')))
for k in list(env):
 if k.startswith('LHDC_'):env.pop(k)
cc=shutil.which('clang',path=env.get('PATH')) or r'C:\Program Files\LLVM\bin\clang.exe'
cache=shutil.which('ccache',path=env.get('PATH'))
if not cache:
 matches=list((pathlib.Path.home()/'.espressif/tools/ccache').glob('*/ccache-*/ccache.exe'));cache=str(sorted(matches)[-1]) if matches else None
assert cache,'ccache required';env.pop('CCACHE_DISABLE',None)
files=sorted([*(enc/'src').glob('*.c'),*(enc/'src').glob('*.h'),*(enc/'include').glob('*.h'),*(enc/'examples').glob('*.c'),enc/'LICENSE',enc/'README.md'],key=lambda x:x.relative_to(enc).as_posix())
h=hashlib.sha256()
for f in files:h.update(f.relative_to(enc).as_posix().encode()+b'\0');h.update(f.read_bytes())
assert len(files)==17 and h.hexdigest()=='6917725b0598e56af0f7ba20490cfadeb9b0910aefa822c4b71a0af726c0651a'
run([sys.executable,ROOT/'codecs/lhdcv5/prepare_port.py','--source',dec,'--output',port],env=env)
def build(sources,includes,prefix,flags=()):
 objects=[]
 for f in sources:
  obj=out/(prefix+f.stem+'.obj');objects.append(obj)
  run([cache,cc,'-std=c11','-O3','-fwrapv','-D_CRT_SECURE_NO_WARNINGS',*flags,*['-I'+str(x) for x in includes],'-c',f,'-o',obj],env=env)
 return objects
enc_sources=[*(enc/'src').glob('*.c'),ROOT/'tests/codecs/lhdcv5_encoder_fixture.c']
enc_objects=build(enc_sources,[enc/'include',enc/'src'],'encoder_')
generator=out/'generate.exe';run([cc,*enc_objects,'-o',generator],env=env)
names=['lhdc_dec.c','lhdc_imdct.c','lhdc_entropy_dec.c','lhdc_sns_synth.c']
dec_sources=[ROOT/'tests/codecs/lhdcv5_level_probe.c',ROOT/'codecs/lhdcv5/dbt_lhdcv5_codec.c',ROOT/'codecs/common/dbt_codec.c',ROOT/'src/dbt_avdtp_caps.c',*[port/n for n in names],*[dec/n for n in ['lhdc_bit_reader.c','lhdc_diag_config.c','lhdc_tables.c']]]
reference_sources=[ROOT/'tests/codecs/lhdcv5_reference_probe.c',*[dec/n for n in ['lhdc_dec.c','lhdc_imdct.c','lhdc_entropy_dec.c','lhdc_sns_synth.c','lhdc_bit_reader.c','lhdc_diag_config.c','lhdc_tables.c']]]
inputs=[*reference_sources,*files,*enc_sources,*dec_sources,ROOT/'tests/codecs/lhdcv5_target_math_check.c',ROOT/'codecs/lhdcv5/prepare_port.py',pathlib.Path(__file__).resolve(),*dec.glob('*.h'),*port.glob('*.h')]
before={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in inputs}
flags=['-DLHDC_HOST_BUILD','-DCONFIG_IDF_TARGET_ESP32=1','-DDBT_LHDC_TARGET_MATH=1','-DDBT_LHDC_NO_HOST_WINFILE','-DDBT_LHDC_QUALITY=1','-DDBT_LHDC_MATH_SELFTEST=1']
objects=build(dec_sources,[port,dec,ROOT/'include',ROOT/'codecs/common',ROOT/'codecs/lhdcv5'],'native_',flags)
dll=out/'level_native.dll';run([cc,'-shared',*objects,'-o',dll],env=env)
ref_objects=build(reference_sources,[dec],'reference_',['-DLHDC_HOST_BUILD','-DCONFIG_IDF_TARGET_ESP32=1'])
ref_dll=out/'level_reference.dll';run([cc,'-shared',*ref_objects,'-o',ref_dll],env=env)
# Independent unmodified sibling: override only its empirical trim through its
# public host calibration hook. exp2f rounds the exact 15/16 ratio to binary32.
os.environ['LHDC_LEVELDB']=str(20*math.log10(15/16))
ref=ctypes.CDLL(str(ref_dll))
lib=ctypes.CDLL(str(dll));u8=ctypes.c_uint8;u=ctypes.c_uint32;ptr=ctypes.POINTER(u8)
fields='revision attempts rejected channel0_errors channel1_silenced peak_concealed nonfinite_frames clipped_frames clipped_samples event_attempt generation frame_index rtp_timestamp ssrc sequence frame_bytes flags peak_f32_bits result channel0_result channel1_result'.split()
class Quality(ctypes.Structure):_fields_=[(f,u) for f in fields]
lib.level_quality.argtypes=[ctypes.POINTER(Quality)];lib.math_open.argtypes=[ptr,ctypes.c_uint8];lib.math_packet.argtypes=[ptr,ctypes.c_uint16,ctypes.c_uint16,ptr,u,ctypes.POINTER(u)]
ref.reference_frame.argtypes=[ptr,u,ptr,u,ctypes.POINTER(u),ctypes.POINTER(u)]
lib.dbt_lhdc_test_level.argtypes=[u,ctypes.c_int];lib.dbt_lhdc_test_level.restype=ctypes.c_float
for rate in (44100,48000,96000,192000):
 for depth in (16,24,32):assert lib.dbt_lhdc_test_level(rate,depth)==0.9375
def buf(b):return (u8*len(b)).from_buffer_copy(b)
def quality():
 q=Quality();assert lib.level_quality(ctypes.byref(q));return {f:getattr(q,f) for f in fields}
lib.level_quality_start();rows=[]
for rate in (44100,48000,96000,192000):
 for depth in (16,24):
  for index in (5,8,9):
   for amplitude in (0.1,0.95):
    path=out/f'{rate}_{depth}_{index}_{amplitude}.packets';run([generator,rate,depth,index,path,amplitude],env=env)
    config=bytes([0,255,58,5,0,0,53,76,{44100:32,48000:16,96000:4,192000:1}[rate],{16:4,24:2}[depth],17,64,0]);assert lib.math_open(buf(config),len(config))==0
    assert ref.reference_open(rate,depth)==0
    q0=quality();reference_pcm=bytearray();raw=path.read_bytes();pos=seq=frames=0;pcm=bytearray()
    while pos<len(raw):
     n,count=struct.unpack_from('<HH',raw,pos);pos+=4;payload=raw[pos:pos+n];pos+=n;assert n and count and len(payload)==n
     packet=bytes([count<<2,seq&255])+payload;dst=(u8*65536)();written=u()
     assert lib.math_packet(buf(packet),len(packet),seq,dst,len(dst),ctypes.byref(written))==0,(rate,depth,index,amplitude,seq)
     samples=240 if rate==44100 else rate//200;assert written.value==count*samples*2*(2 if depth==16 else 4)
     off=0
     for _ in range(count):
      size=2+2*(int.from_bytes(payload[off:off+2],'little')&1023);frame=payload[off:off+size];assert len(frame)==size
      target=(u8*(samples*2*(2 if depth==16 else 4)))();used=u();generated=u()
      assert ref.reference_frame(buf(frame),size,target,samples,ctypes.byref(used),ctypes.byref(generated))==0 and used.value==size and generated.value==samples
      reference_pcm.extend(bytes(target));off+=size
     assert off==len(payload)
     pcm.extend(bytes(dst[:written.value]));seq+=1;frames+=count
    q1=quality();assert lib.math_close()==0;ref.reference_close()
    assert pcm==reference_pcm,('independent PCM mismatch',rate,depth,index,amplitude)
    assert frames==96,('missing generated frames',rate,depth,index,amplitude,frames)
    values=[x[0] for x in struct.iter_unpack('<h' if depth==16 else '<i',pcm)];scale=float(2**(15 if depth==16 else 31));channels=[]
    for ch,freq in enumerate((997,1300)):
     pts=[(i,values[2*i+ch]/scale) for i in range(samples*6,min(samples*90,len(values)//2))]
     ss=ccv=sc=ys=yc=power=peak=0.0
     for i,v in pts:
      angle=2*math.pi*freq*i/rate;s=math.sin(angle);c=math.cos(angle);ss+=s*s;ccv+=c*c;sc+=s*c;ys+=v*s;yc+=v*c;power+=v*v;peak=max(peak,abs(v))
     det=ss*ccv-sc*sc;coef_s=(ys*ccv-yc*sc)/det;coef_c=(yc*ss-ys*sc)/det;fund=math.hypot(coef_s,coef_c);fit_power=coef_s*ys+coef_c*yc
     channels.append(dict(fundamental_gain_db=20*math.log10(fund/amplitude) if fund else None,peak=peak,rms=math.sqrt(power/len(pts)),residual_to_signal_db=10*math.log10(max(1e-30,power-fit_power)/max(1e-30,fit_power))))
    row=dict(rate=rate,depth=depth,quality_index=index,input_peak=amplitude,frames=frames,packets=seq,encoded_bps=(len(raw)-4*seq)*8*rate//(frames*samples),quality_delta={f:(q1[f]-q0[f])&0xffffffff for f in fields[1:9]},channels=channels,encoded_sha256=hashlib.sha256(raw).hexdigest(),pcm_sha256=hashlib.sha256(pcm).hexdigest())
    rows.append(row);print(json.dumps(row),flush=True)
assert all(hashlib.sha256(pathlib.Path(f).read_bytes()).hexdigest()==h for f,h in before.items()),'source changed during measurement'
failures=[dict(rate=r['rate'],depth=r['depth'],quality_index=r['quality_index'],input_peak=r['input_peak']) for r in rows if any(abs(c['fundamental_gain_db'])>0.02 or c['residual_to_signal_db']>-45 for c in r['channels']) or any(r['quality_delta'][f] for f in ['rejected','channel0_errors','channel1_silenced','peak_concealed','nonfinite_frames','clipped_frames','clipped_samples'])]
report=dict(passed=not failures,failures=failures,independent_reference_pcm_equal=True,measurement_complete=True,scope='host generated encoder tone through actual native worker with target selector/math; no RF, target timing or acoustics',ccache=True,rows=rows,source_sha256=before)
(out/'levels.json').write_text(json.dumps(report,indent=2)+'\n');print('COMPLETE',len(rows),'level cases')
if failures:raise SystemExit('PCM unity/clipping gate failed; see levels.json')

if a.refresh_points:
 def array(text,name):
  m=re.search(r'static const uint8_t '+re.escape(name)+r'\[[^]]*\]=\{([\d,]+)\};',text);assert m,name
  return bytes(int(v) for v in m[1].split(','))
 def decode_packets(config,packets,rate,depth):
  samples=240 if rate==44100 else rate//200;width=2 if depth==16 else 4
  assert lib.math_open(buf(config),len(config))==0 and ref.reference_open(rate,depth)==0
  actual=bytearray();expected=bytearray()
  for seq,packet in enumerate(packets):
   dst=(u8*65536)();written=u();assert lib.math_packet(buf(packet),len(packet),seq,dst,len(dst),ctypes.byref(written))==0
   actual.extend(bytes(dst[:written.value]));payload=packet[2:];off=0
   for _ in range(packet[0]>>2):
    size=2+2*(int.from_bytes(payload[off:off+2],'little')&1023);frame=payload[off:off+size];assert len(frame)==size
    dst=(u8*(samples*2*width))();used=u();generated=u()
    assert ref.reference_frame(buf(frame),size,dst,samples,ctypes.byref(used),ctypes.byref(generated))==0 and used.value==size and generated.value==samples
    expected.extend(bytes(dst));off+=size
   assert off==len(payload)
  assert lib.math_close()==0;ref.reference_close();assert actual==expected
  return expected,[v[0] for v in struct.iter_unpack('<h' if depth==16 else '<i',expected)]
 def fnv(data):
  h=2166136261
  for b in data:h=((h^b)*16777619)&0xffffffff
  return h
 refreshed=[]
 header=ROOT/'esp32_probe/main/probe_codec_vectors.h';original=header.read_text();updated=original
 for id,rate,depth in ((17,44100,16),(18,48000,16),(19,96000,24),(20,192000,24)):
  config=array(original,f'codec_cie_{id}');packets=[array(original,f'codec_packet_{id}_{i}') for i in range(3)]
  pcm,values=decode_packets(config,packets,rate,depth)
  pattern=r'(static const probe_pcm_point_t codec_points_'+str(id)+r'\[\]=\{)(.*?)(\};)'
  m=re.search(pattern,updated);assert m
  indices=[int(x) for x in re.findall(r'\{(\d+)u,-?\d+\}',m[2])];assert len(indices)==24
  newpoints=','.join('{%du,%d}'%(i,values[i]) for i in indices)
  updated=updated[:m.start(2)]+newpoints+updated[m.end(2):]
  line=next(x for x in updated.splitlines() if x.startswith('{"lhdcv5_'+str(rate)+'_'+str(depth)+'",'))
  frames=len(values)//2;oldfields=re.search(r','+str(rate)+'u,'+str(frames)+r'u,(\d+)u,(\d+)u,',line);assert oldfields
  new_line=line[:oldfields.start(1)]+str(fnv(pcm))+'u,'+str(max(map(abs,values))//4)+line[oldfields.end(2):]
  updated=updated.replace(line,new_line)
  refreshed.append(dict(name=f'lhdcv5_{rate}_{depth}',pcm_sha256=hashlib.sha256(pcm).hexdigest(),points=24,packet_sha256=[hashlib.sha256(b).hexdigest() for b in packets]))
 assert updated!=original
 # Only the four LHDC checkpoint arrays and four metadata lines may change.
 old_other=[l for l in original.splitlines() if not re.search(r'codec_points_(17|18|19|20)\[|^\{"lhdcv5_',l)]
 new_other=[l for l in updated.splitlines() if not re.search(r'codec_points_(17|18|19|20)\[|^\{"lhdcv5_',l)]
 assert old_other==new_other
 (out/'probe_codec_vectors.before.h').write_text(original);header.write_text(updated,newline='\n')
 maxheader=ROOT/'esp32_probe/main/probe_lhdc_max_fixture.h';maxold=maxheader.read_text()
 packets=[array(maxold,f'lhdc_max_packet_{i}') for i in range(96)]
 pcm,values=decode_packets(array(maxold,'lhdc_max_cie'),packets,192000,24)
 m=re.search(r'(static const probe_pcm_point_t lhdc_max_points\[\]=\{\s*)(.*?)(\};)',maxold,re.S);assert m
 indices=[int(x) for x in re.findall(r'\{(\d+)u,-?\d+\}',m[2])];assert len(indices)==56
 replacement='\n'.join('{%du,%d},'%(i,values[i]) for i in indices)+'\n'
 maxnew=maxold[:m.start(2)]+replacement+maxold[m.end(2):]
 (out/'probe_lhdc_max_fixture.before.h').write_text(maxold);maxheader.write_text(maxnew,newline='\n')
 (out/'refreshed-points.json').write_text(json.dumps(dict(scope='existing four cold checks and maximum fixture points only; encoded packets unchanged, no new boot cases',cases=refreshed,maximum_pcm_sha256=hashlib.sha256(pcm).hexdigest(),header_sha256=hashlib.sha256(header.read_bytes()).hexdigest(),maximum_header_sha256=hashlib.sha256(maxheader.read_bytes()).hexdigest()),indent=2)+'\n')
 print('Refreshed existing 4 cold cases and 56 maximum-fixture points; no new cases')
