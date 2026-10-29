"""Offline AVDTP-to-worker tests with real SBC, Opus, AAC-LC and aptX codecs.
No ESP32, Bluetooth radio, connections or network are used."""
import argparse,ctypes,os,pathlib,shutil,subprocess,sys
parser=argparse.ArgumentParser();parser.add_argument('--eight-mod-16',action='store_true',help='exercise an alternate 8-byte-aligned allocator base');args=parser.parse_args()
root=pathlib.Path(__file__).resolve().parents[1];ws=root.parent;out=root/'_build/avrcp';out.mkdir(parents=True,exist_ok=True)
opus=ws/'esp-idf/components/bt/host/bluedroid/external/opus';sbc=ws/'esp-idf/components/bt/host/bluedroid/external/sbc/decoder'
check=[sys.executable,str(root/'tools/check_workspace.py')];subprocess.run(check,check=True)
env=os.environ.copy()
try:
    if os.name=='nt':
        vc=pathlib.Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
        data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
        env.update(dict(x.split('=',1) for x in data.splitlines() if '=' in x and not x.startswith('=')))
    cc=shutil.which('clang',path=env.get('PATH')) or (r'C:\Program Files\LLVM\bin\clang.exe' if os.name=='nt' else 'cc')
    ninja=shutil.which('ninja',path=env.get('PATH')) or str(pathlib.Path(env.get('USERPROFILE',''))/'.espressif/tools/ninja/1.12.1/ninja.exe')
    ffmpeg=shutil.which('ffmpeg',path=env.get('PATH'))
    if not ffmpeg:raise SystemExit('FFmpeg is required for an offline SBC vector')
    build=out/'probe_opus_guarded';override=(root/'codecs/opus/scratch/dbt_opus_scratch_override.h').as_posix()
    subprocess.run(['cmake','-S',str(opus),'-B',str(build),'-G','Ninja',f'-DCMAKE_MAKE_PROGRAM={ninja}',f'-DCMAKE_C_COMPILER={cc}',
        '-DCMAKE_BUILD_TYPE=Release',f'-DCMAKE_C_FLAGS=-include "{override}" -DDBT_OPUS_GUARDED_API=1 -std=c11 -fwrapv',
        '-DOPUS_BUILD_SHARED_LIBRARY=OFF','-DOPUS_BUILD_TESTING=OFF','-DOPUS_BUILD_PROGRAMS=OFF',
        '-DOPUS_FIXED_POINT=ON','-DOPUS_ENABLE_FLOAT_API=OFF','-DOPUS_DISABLE_INTRINSICS=ON'],env=env,check=True,stdout=subprocess.DEVNULL)
    subprocess.run(['cmake','--build',str(build),'--parallel','4'],env=env,check=True,stdout=subprocess.DEVNULL)
    libs=list(build.rglob('opus.lib' if os.name=='nt' else 'libopus.a'))
    if len(libs)!=1:raise SystemExit('expected one guarded Opus library')
    port=out/'probe_sbc_port';subprocess.run([sys.executable,str(root/'codecs/sbc/prepare_port.py'),'--source',str(sbc/'srce'),'--output',str(port)],env=env,check=True,stdout=subprocess.DEVNULL)
    helix=ws/'esp-idf/components/bt/host/bluedroid/external/libaac-lc/libhelix_aac'
    aptx=ws/'esp-idf/components/bt/host/bluedroid/external/libfreeaptx/main'
    aac_port=out/'probe_aac_port';subprocess.run([sys.executable,str(root/'codecs/aac/prepare_port.py'),'--source',str(helix/'aacdec.c'),'--output',str(aac_port)],env=env,check=True,stdout=subprocess.DEVNULL)
    names='alloc bitalloc-sbc bitalloc bitstream-decode decoder-oina decoder-private decoder-sbc dequant framing-sbc framing oi_codec_version synthesis-8-generated synthesis-dct8 synthesis-sbc'.split()
    src=[root/'tests/codecs/probe_audio_profile.c',root/'src/dbt_avdtp.c',root/'src/dbt_avdtp_caps.c',root/'src/dbt_avdtp_sink.c',root/'src/dbt_a2dp_sbc.c',root/'src/dbt_sbc_media.c',
         root/'codecs/common/dbt_codec.c',root/'codecs/sbc/dbt_sbc_codec.c',root/'codecs/sbc/dbt_sbc_decoder.c',root/'codecs/opus/dbt_opus.c',root/'codecs/opus/scratch/dbt_opus_scratch.c']
    src += [(port if x.startswith('synthesis-') else sbc/'srce')/(x+'.c') for x in names]
    aac_names='aactabs bitstream buffers dct4 decelmnt dequant fft filefmt huffman hufftabs imdct noiseless pns stproc tns trigtabs'.split()
    src += [aac_port/'aacdec_guarded.c']+[helix/(x+'.c') for x in aac_names]
    src += [root/'codecs/aac/dbt_aac_decoder.c',root/'codecs/aac/dbt_aac_transport.c',root/'codecs/aptx/dbt_aptx.c',root/'codecs/aptx/dbt_aptx_backend.c',root/'src/dbt_audio.c',root/'src/dbt_resampler.c',ws/'bt_audio_sink/main/dsp/fixed_dsp.c']
    incs=[root/'tests/codecs/probe_audio_stubs',root/'include',root/'codecs/common',root/'codecs/sbc',root/'codecs/sbc/port',root/'codecs/opus',root/'codecs/aac',root/'codecs/aptx',root/'esp32_probe/main',ws/'bt_audio_sink/main/dsp',sbc/'include',opus/'include',helix,aptx]
    cmd=[cc,'-std=c11','-O1','-g','-fwrapv','-shared','-DUSE_DEFAULT_STDLIB','-DAAC_MAX_NCHANS=2']
    if os.name!='nt':cmd+=['-fPIC']
    cmd+=sum((['-I',str(i)] for i in incs),[])+[str(s) for s in src]+[str(libs[0])]
    if os.name!='nt':cmd+=['-lm']
    raw=subprocess.check_output([ffmpeg,'-v','error','-f','lavfi','-i','sine=frequency=997:sample_rate=48000:duration=0.03','-ac','2','-c:a','sbc','-b:a','328k','-f','sbc','pipe:1'],env=env)
    assert raw[0]==0x9c
    f,pool=raw[1],raw[2];bands=8 if f&1 else 4;blocks=4*(1+((f>>4)&3));mode=(f>>2)&3;channels=2 if mode else 1
    bits=blocks*pool*(channels if mode<2 else 1)+(bands if mode==3 else 0)
    size=4+bands*channels//2+(bits+7)//8;frame=raw[:size]
    assert len(frame)==size and channels==2
    cie=bytes([0,0,(0x80>>(f>>6))|(8>>mode),(0x80>>((f>>4)&3))|(4 if f&1 else 8)|(2 if f&2 else 1),2,pool])
    def aac_packet(rate):
        adts=subprocess.check_output([ffmpeg,'-v','error','-f','lavfi','-i',f'sine=frequency=997:sample_rate={rate}:duration=0.03','-ac','2','-c:a','aac','-profile:a','aac_low','-b:a','64k','-f','adts','pipe:1'],env=env)
        assert adts[0]==255 and adts[1]&1
        n=((adts[3]&3)<<11)|(adts[4]<<3)|(adts[5]>>5);aac_raw=adts[7:n];bits=[]
        def put(v,n):bits.extend((v>>i)&1 for i in reversed(range(n)))
        for value,width in [(0,1),(0,1),(1,1),(0,6),(0,4),(0,3),(2,5),(3 if rate==48000 else 4,4),(2,4),(0,3),(0,3),(255,8),(0,1),(0,1)]:put(value,width)
        length=len(aac_raw)
        while length>=255:put(255,8);length-=255
        put(length,8)
        for b in aac_raw:put(b,8)
        while len(bits)%8:bits.append(0)
        result=bytes(sum(bits[i+j]<<(7-j) for j in range(8)) for i in range(0,len(bits),8));assert len(result)<660
        return result
    aac=aac_packet(48000);aac44=aac_packet(44100)
    for output,opus_enabled,extra_codecs in ((False,False,False),(False,True,False),(False,True,True),(True,True,True)):
        variant=('output' if output else 'all' if extra_codecs else 'multi' if opus_enabled else 'sbc')
        artifact=out/('probe-audio-'+variant+('.dll' if os.name=='nt' else '.so'))
        define=['-DDBT_PROBE_AVRCP=1']+(['-DDBT_PROBE_AUDIO_OUTPUT=1'] if output else [])+(['-DDBT_PROBE_OPUS_BUILD=1'] if opus_enabled else [])+(['-DDBT_PROBE_AAC_BUILD=1','-DDBT_PROBE_APTX_BUILD=1'] if extra_codecs else [])
        if args.eight_mod_16:define+=['-DTEST_EIGHT_MOD16=1']
        subprocess.run(cmd+define+['-o',str(artifact)],env=env,check=True)
        dll=ctypes.CDLL(str(artifact));fn=dll.probe_audio_profiles;u8=ctypes.c_ubyte;fn.argtypes=[ctypes.POINTER(u8),ctypes.c_uint16,ctypes.POINTER(u8),ctypes.POINTER(u8),ctypes.c_uint16,ctypes.POINTER(u8),ctypes.c_uint16]
        line=fn((u8*len(frame)).from_buffer_copy(frame),len(frame),(u8*6).from_buffer_copy(cie),(u8*len(aac)).from_buffer_copy(aac),len(aac),(u8*len(aac44)).from_buffer_copy(aac44),len(aac44))
        if line:raise SystemExit('FAIL real-codec probe profile/worker variant='+variant+' line '+str(line))
        if output:
            line=dll.probe_audio_formats()
            if line:raise SystemExit('FAIL PCM format/stride line '+str(line))
        line=dll.probe_audio_metadata()
        if line:raise SystemExit('FAIL AVRCP metadata escaping/revision variant='+variant+' line '+str(line))
        print('PASS '+variant+' profile/worker, real codec decoding, gating, teardown and AVRCP metadata escaping')
finally:subprocess.run(check,check=True)
