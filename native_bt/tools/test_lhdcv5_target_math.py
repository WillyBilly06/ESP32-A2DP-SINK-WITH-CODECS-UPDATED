"""Bit-exact host comparison of generated target-math LHDC and original code.
Uses previously produced offline encoder fixtures; no radio, network or target."""
import ctypes,hashlib,json,os,pathlib,shutil,struct,subprocess,sys
root=pathlib.Path(__file__).resolve().parents[1];ws=root.parent;dec=ws.parent/'LHDC-V5-Decoder-main/decoder';out=root/'_build/avrcp/lhdc-perf';out.mkdir(parents=True,exist_ok=True);port=out/'math_port'
check=[sys.executable,str(root/'tools/check_workspace.py')];subprocess.run(check,check=True);env=os.environ.copy()
try:
    if os.name=='nt':
        vc=pathlib.Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat');data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True);env.update(dict(x.split('=',1) for x in data.splitlines() if '=' in x and not x.startswith('=')))
    for name in ('LHDC_WINFILE','LHDC_PKALL','LHDC_FORCE_REF','LHDC_CH1_START','LHDC_SNS','LHDC_LDIV','LHDC_SPLIT','LHDC_SLOPE','LHDC_RATEPOW','LHDC_LEVELDB'):env.pop(name,None)
    subprocess.run([sys.executable,str(root/'codecs/lhdcv5/prepare_port.py'),'--source',str(dec),'--output',str(port)],env=env,check=True,stdout=subprocess.DEVNULL)
    cc=shutil.which('clang',path=env.get('PATH')) or (r'C:\Program Files\LLVM\bin\clang.exe' if os.name=='nt' else 'cc')
    names=('lhdc_dec.c','lhdc_imdct.c','lhdc_entropy_dec.c','lhdc_sns_synth.c','lhdc_bit_reader.c','lhdc_diag_config.c','lhdc_tables.c')
    inputs=[root/'codecs/lhdcv5/prepare_port.py',root/'codecs/lhdcv5/dbt_lhdcv5_codec.c',root/'tests/codecs/lhdcv5_target_math_check.c',root/'tests/codecs/lhdcv5_reference_probe.c']+[dec/n for n in names]+list(dec.glob('*.h'))
    inputs += [root/'_build/native_codecs/lhdcv5_fixture_encoder'/f'tone_{rate}_{depth}.packets' for rate in (44100,48000,96000,192000) for depth in (16,24)]
    before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
    base=[cc,'-std=c11','-O3','-fwrapv','-shared','-DLHDC_HOST_BUILD','-DCONFIG_IDF_TARGET_ESP32=1','-D_CRT_SECURE_NO_WARNINGS']
    if os.name!='nt':base+=['-fPIC']
    incs=[root/'include',root/'codecs/common',root/'codecs/lhdcv5',dec];base+=sum((['-I',str(x)] for x in incs),[])
    generated=[port/n for n in names[:4]]+[dec/n for n in names[4:]]
    suffix='.dll' if os.name=='nt' else '.so';native=out/('math-native'+suffix);uncached=out/('math-uncached'+suffix);reference=out/('math-reference'+suffix)
    extra=[] if os.name=='nt' else ['-lm']
    subprocess.run(base+['-DDBT_LHDC_NO_HOST_WINFILE','-DDBT_LHDC_TARGET_MATH=1','-DDBT_LHDC_MATH_SELFTEST=1']+[str(p) for p in [root/'tests/codecs/lhdcv5_target_math_check.c',root/'codecs/lhdcv5/dbt_lhdcv5_codec.c',root/'codecs/common/dbt_codec.c',root/'src/dbt_avdtp_caps.c']+generated]+extra+['-o',str(native)],env=env,check=True)
    subprocess.run(base+['-DDBT_LHDC_NO_HOST_WINFILE','-DDBT_LHDC_MATH_SELFTEST=1']+[str(p) for p in [root/'tests/codecs/lhdcv5_target_math_check.c',root/'codecs/lhdcv5/dbt_lhdcv5_codec.c',root/'codecs/common/dbt_codec.c',root/'src/dbt_avdtp_caps.c']+generated]+extra+['-o',str(uncached)],env=env,check=True)
    subprocess.run(base+[str(p) for p in [root/'tests/codecs/lhdcv5_reference_probe.c']+[dec/n for n in names]]+extra+['-o',str(reference)],env=env,check=True)
    left,right=ctypes.CDLL(str(native)),ctypes.CDLL(str(reference));original_math=ctypes.CDLL(str(uncached))
    for lib in (left,original_math):
        lib.dbt_lhdc_test_gain.argtypes=[ctypes.c_int,ctypes.c_int,ctypes.c_int,ctypes.c_uint32];lib.dbt_lhdc_test_gain.restype=ctypes.c_float
        lib.dbt_lhdc_test_level.argtypes=[ctypes.c_uint32,ctypes.c_int];lib.dbt_lhdc_test_level.restype=ctypes.c_float
        lib.dbt_lhdc_test_norm.argtypes=[ctypes.c_int];lib.dbt_lhdc_test_norm.restype=ctypes.c_float
    same=lambda x,y:struct.pack('<f',x)==struct.pack('<f',y)
    comparisons=0
    for rate in (44100,48000,96000,192000):
      for depth in (16,24,32):
        assert same(left.dbt_lhdc_test_level(rate,depth),original_math.dbt_lhdc_test_level(rate,depth))
        for length in list(range(2,769,17))+[49,50,51,159,160,161,249,250,251,503,504,505,767,768]:
          for gain in range(512):
            assert same(left.dbt_lhdc_test_gain(gain,length,depth,rate),original_math.dbt_lhdc_test_gain(gain,length,depth,rate)),(rate,depth,length,gain)
            comparisons+=1
    for i in range(1024):
        rate=(44100,48000,96000,192000)[i%4];depth=(16,24,32)[i%3];length=2+(i*37)%767;gain=(i*53)%512
        assert same(left.dbt_lhdc_test_gain(gain,length,depth,rate),original_math.dbt_lhdc_test_gain(gain,length,depth,rate))
        assert same(left.dbt_lhdc_test_level(rate,depth),original_math.dbt_lhdc_test_level(rate,depth));comparisons+=1
    for n in (240,360,480,960,1920):assert same(left.dbt_lhdc_test_norm(n),original_math.dbt_lhdc_test_norm(n))
    print('PASS cached/uncached gain, level and norm binary32 comparisons',comparisons)
    u8=ctypes.c_uint8;ptr=ctypes.POINTER(u8);u=ctypes.c_uint
    left.math_open.argtypes=[ptr,ctypes.c_uint8];left.math_packet.argtypes=[ptr,ctypes.c_uint16,ctypes.c_uint16,ptr,ctypes.c_uint32,ctypes.POINTER(ctypes.c_uint32)]
    right.reference_frame.argtypes=[ptr,u,ptr,u,ctypes.POINTER(u),ctypes.POINTER(u)]
    def buf(b):return (u8*len(b)).from_buffer_copy(b)
    rows=[]
    for rate in (44100,48000,96000,192000):
      for depth in (16,24):
        cie=bytes([0,255,0x3a,5,0,0,0x35,0x4c,{44100:32,48000:16,96000:4,192000:1}[rate],{16:4,24:2}[depth],0x11,0x40,0]);samples=240 if rate==44100 else rate//200;width=2 if depth==16 else 4
        assert left.math_open(buf(cie),len(cie))==0 and right.reference_open(rate,depth)==0
        data=(root/'_build/native_codecs/lhdcv5_fixture_encoder'/f'tone_{rate}_{depth}.packets').read_bytes();pos=sequence=frames_total=0;digest=hashlib.sha256()
        while pos<len(data):
            n=int.from_bytes(data[pos:pos+2],'little');count=int.from_bytes(data[pos+2:pos+4],'little');pos+=4;raw=data[pos:pos+n];pos+=n
            assert n and count and len(raw)==n and n+2<=4096
            packet=bytes([0,0])+raw;decoded=(u8*65536)();written=ctypes.c_uint32();assert left.math_packet(buf(packet),len(packet),sequence,decoded,len(decoded),ctypes.byref(written))==0
            expected=bytearray();off=0
            for _ in range(count):
                size=2+2*(int.from_bytes(raw[off:off+2],'little')&1023);chunk=raw[off:off+size];assert len(chunk)==size
                pcm=(u8*(samples*2*width))();consumed=u();generated=u();assert right.reference_frame(buf(chunk),len(chunk),pcm,samples,ctypes.byref(consumed),ctypes.byref(generated))==0 and consumed.value==len(chunk) and generated.value==samples
                expected+=bytes(pcm);off+=size
            assert off==len(raw) and bytes(decoded[:written.value])==expected,(rate,depth,sequence)
            digest.update(expected);frames_total+=count;sequence+=1
        assert left.math_close()==0;right.reference_close();row={'rate':rate,'depth':depth,'packets':sequence,'frames':frames_total,'pcm_sha256':digest.hexdigest()};rows.append(row);print('PASS',row)
    assert all(hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()==h for p,h in before.items())
    (out/'math-results.json').write_text(json.dumps({'passed':True,'scope':'host target-math and target selector branch vs unmodified sibling on encoded fixtures; no target execution','rows':rows,'binary32_math_comparisons':comparisons,'sha256':before},indent=2)+'\n')
finally:subprocess.run(check,check=True)
