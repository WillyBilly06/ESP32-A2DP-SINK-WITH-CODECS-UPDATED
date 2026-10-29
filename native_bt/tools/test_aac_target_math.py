"""Compare host AAC math branches and guarded/original SDK decoders offline."""
import ctypes as c
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
CORE=ROOT.parent/'esp-idf/components/bt/host/bluedroid/external/libaac-lc/libhelix_aac'
OUT=ROOT/'_build/aac_math'
check=[sys.executable,str(ROOT/'tools/check_workspace.py')]
subprocess.run(check,check=True)
try:
    OUT.mkdir(parents=True,exist_ok=True)
    env=os.environ.copy()
    if os.name=='nt':
        vc=Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
        data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
        env.update(dict(line.split('=',1) for line in data.splitlines() if '=' in line and not line.startswith('=')))
    compiler=shutil.which('clang') or r'C:\Program Files\LLVM\bin\clang.exe'
    if not Path(compiler).is_file(): raise RuntimeError('clang is required')
    units='aacdec aactabs bitstream buffers dct4 decelmnt dequant fft filefmt huffman hufftabs imdct noiseless pns stproc tns trigtabs'.split()
    subprocess.run([sys.executable,str(ROOT/'codecs/aac/prepare_port.py'),'--source',str(CORE/'aacdec.c'),'--output',str(OUT)],check=True)
    source=[CORE/(name+'.c') for name in units]+[ROOT/'tests/codecs/aac_math_probe.c']
    inputs=source+[ROOT/'codecs/aac/host_target_math.h',ROOT/'codecs/aac/prepare_port.py',ROOT/'esp32_probe/main/probe_codec_vectors.h']
    vectors=ROOT/'_build/native_codecs/vectors'
    inputs += [vectors/f'tone_{rate}_{channels}.aac' for rate in (44100,48000) for channels in (1,2)]
    before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
    libraries={}
    for name,portable,guarded,opt in [('old_windows',False,False,'-O2'),('portable',True,False,'-Os'),('guarded',True,True,'-Os')]:
        flags=[compiler,'-std=c11',opt,'-fwrapv','-funsigned-char','-shared','-DUSE_DEFAULT_STDLIB','-I'+str(CORE)]
        if os.name!='nt': flags+=['-fPIC']
        if portable: flags+=['-include',str(ROOT/'codecs/aac/host_target_math.h')]
        sources=[(OUT/'aacdec_guarded.c' if guarded and p.name=='aacdec.c' else p) for p in source]
        binary=OUT/(name+('.dll' if os.name=='nt' else '.so'))
        subprocess.run(flags+[str(p) for p in sources]+['-o',str(binary)],check=True,env=env)
        lib=c.CDLL(str(binary)); lib.math_open.argtypes=[c.c_int,c.c_int]
        lib.math_decode.argtypes=[c.POINTER(c.c_uint8),c.c_int,c.POINTER(c.c_int16)]
        lib.math_mul.argtypes=[c.c_int32,c.c_int32]; lib.math_mul.restype=c.c_int32
        libraries[name]=lib
    primitives=[]
    for a,b in ((-2147483648,1073741824),(123456789,-987654321),(1073741824,1073741824),(-1,-1)):
        wanted=(a*b)>>32
        actual={name:lib.math_mul(a,b) for name,lib in libraries.items()}
        assert actual['portable']==wanted and actual['guarded']==wanted
        primitives.append({'a':a,'b':b,'expected':wanted,'old_windows':actual['old_windows']})
    print(json.dumps({'multiply_high_probes':primitives},indent=2))
    sys.path.insert(0,str(ROOT/'tools')); import test_native_codec_aac as aac
    ffmpeg=shutil.which('ffmpeg')
    if not ffmpeg: raise RuntimeError('ffmpeg is required for an independent PCM comparison')
    text=(ROOT/'esp32_probe/main/probe_codec_vectors.h').read_text()
    report=[]
    for index,(rate,channels) in enumerate(((44100,1),(44100,2),(48000,1),(48000,2)),1):
        frames=list(aac.adts(vectors/f'tone_{rate}_{channels}.aac'))[:4]
        assert len(frames)==4
        for number,frame in enumerate(frames):
            embedded=re.search(r'codec_packet_'+str(index)+'_'+str(number)+r'\[\]=\{([^}]*)\}',text)
            assert embedded, (rate,channels,number,'missing embedded packet')
            packet=bytes(int(value) for value in embedded.group(1).split(',') if value.strip())
            assert packet==aac.latm(frame,rate,channels,number>0), (rate,channels,number,'encoded fixture mismatch')
        samples={}
        for name,lib in libraries.items():
            assert lib.math_open(rate,channels)==0
            result=[]
            for frame in frames:
                padded=(c.c_uint8*(len(frame)+4))();c.memmove(padded,frame,len(frame)); pcm=(c.c_int16*2048)()
                assert lib.math_decode(padded,len(frame),pcm)==0
                result.extend(pcm[:1024*channels])
            lib.math_close();samples[name]=result
        assert samples['portable']==samples['guarded']
        point_text=re.search(r'codec_points_'+str(index)+r'\[\]=\{([^;]+)\};',text).group(1)
        points=[(int(a),int(b)) for a,b in re.findall(r'\{(\d+)u,(-?\d+)\}',point_text)]
        assert points
        expected_source='portable' if all(samples['portable'][i]==v for i,v in points) else \
                        'old_windows' if all(samples['old_windows'][i]==v for i,v in points) else 'other'
        assert expected_source!='other', (rate,channels,'fixture checkpoints do not match either decoder path')
        mismatches=[(i,old,samples['portable'][i],v) for i,v in points if (old:=samples['old_windows'][i])!=samples['portable'][i]]
        ref=subprocess.check_output([ffmpeg,'-v','error','-i',str(vectors/f'tone_{rate}_{channels}.aac'),
                                     '-f','s16le','-acodec','pcm_s16le','-'],env=env)
        decoded=[v[0] for v in struct.iter_unpack('<h',ref)][:len(samples['portable'])]
        assert len(decoded)==len(samples['portable'])
        mae={name:sum(abs(x-y) for x,y in zip(values,decoded))/len(decoded) for name,values in samples.items()}
        portable_max=max(abs(a-b) for a,b in zip(samples['portable'],decoded))
        assert mae['portable']<3 and portable_max<=16, (rate,channels,mae,portable_max)
        report.append({'rate':rate,'channels':channels,'points':len(points),'mismatched_points':len(mismatches),
                       'checkpoint_source':expected_source,'mean_abs_error_vs_ffmpeg':mae,'portable_max_error_vs_ffmpeg':portable_max,
                       'portable_peak':max(abs(x) for x in samples['portable']),
                       'first_differences':mismatches[:4],
                       'peak_delta':max(abs(a-b) for a,b in zip(samples['old_windows'],samples['portable'])),
                       'guarded_matches_original_portable':True})
    assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in before.items())
    print(json.dumps(report,indent=2))
finally:
    subprocess.run(check,check=True)
