"""Compare real AOSP-encoded LDAC audio against the standalone SDK decoder.

Requires the test-only pinned AOSP libldac checkout already in _build. It never
links the encoder into firmware or touches a Bluetooth controller.
"""
import ctypes
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import struct
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'_build/native_codecs'
ENC=OUT/'ldac_fixture_encoder'
COMMIT='eeee1a3f5f8df1282e3a6d297085885fd886737b'
ARCHIVE_SHA='25de6991570468f6e29b269085451e5af8dbb60d15eb5aa9e3da34d94d963d11'
TREE_SHA='9e38fc695ea63c20d60692722006fd65ebf592d6cc04cdf0dc634c86862f2c78'
CHECK=[sys.executable,str(ROOT/'tools/check_workspace.py')]
sys.path.insert(0,str(ROOT/'tools'))
import test_native_codec_ldac_real as decoder_test


def waveform_correlation(pcm,channels,channel):
    samples=[v[0] for v in struct.iter_unpack('<i',pcm)]
    multiplier=97 if channel==0 else 137
    points=list(range(512,12000,32));source=[((i*multiplier)%48000)-24000 for i in points]
    energy=sum(v*v for v in source);best=(-1.0,0)
    for delay in range(0,1025,4):
        output=[samples[(i+delay)*channels+channel] for i in points]
        output_energy=sum(v*v for v in output)
        if output_energy:
            correlation=sum(a*b for a,b in zip(source,output))/math.sqrt(energy*output_energy)
            if correlation>best[0]:best=(correlation,delay)
    return best


def main():
    if not (ENC/'src/ldaclib.c').is_file():raise RuntimeError('Pinned AOSP test encoder is missing from _build')
    tree=sorted([*(ENC/'src').glob('*.c'),*(ENC/'src').glob('*.h'),*(ENC/'inc').glob('*.h'),ENC/'LICENSE',ENC/'NOTICE'],key=lambda p:p.relative_to(ENC).as_posix())
    digest=hashlib.sha256()
    for path in tree:digest.update(path.relative_to(ENC).as_posix().encode()+b'\0');digest.update(path.read_bytes())
    if len(tree)!=32 or digest.hexdigest()!=TREE_SHA:raise RuntimeError('Unreviewed AOSP test encoder tree')
    compiler=shutil.which('clang') or r'C:\Program Files\LLVM\bin\clang.exe'
    env=os.environ.copy()
    if os.name=='nt':
        vc=Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
        data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
        env.update(dict(line.split('=',1) for line in data.splitlines() if '=' in line and not line.startswith('=')))
    compat=ENC/'compat';compat.mkdir(exist_ok=True)
    if os.name=='nt':
        # The encoder includes unistd.h but uses no symbols from it.
        shim=compat/'unistd.h'
        shim_text='/* No unistd symbols are used by this fixture encoder. */\n'
        if shim.exists() and shim.read_text()!=shim_text:raise RuntimeError('Unexpected existing encoder compatibility header')
        if not shim.exists():shim.write_text(shim_text)
    source=[ENC/'src/ldaclib.c',ENC/'src/ldacBT.c',ROOT/'tests/codecs/ldac_encoder_fixture.c']
    tracked=list((ENC/'src').glob('*.c'))+list((ENC/'src').glob('*.h'))+list((ENC/'inc').glob('*.h'))+list(compat.glob('*.h'))+source[2:]+[Path(__file__).resolve()]
    tracked += list((ROOT/'codecs/ldac').glob('*.[ch]'))+[ROOT/'codecs/ldac/prepare_port.py',ROOT/'codecs/common/dbt_codec.c',ROOT/'src/dbt_avdtp_caps.c',ROOT/'tests/codecs/ldac_real_probe.c',ROOT/'tests/codecs/ldac_reference_probe.c']
    tracked += [Path(decoder_test.__file__).resolve(),ROOT/'codecs/sbc/prepare_port.py',ROOT/'src/dbt_sbc_media.c']
    tracked += list((decoder_test.SDK/'src').glob('*.c'))+list((decoder_test.SDK/'src').glob('*.h'))+list((decoder_test.SDK/'inc').glob('*.h'))
    tracked += list((ROOT/'codecs/sbc').glob('*.[ch]'))+list((decoder_test.SBC/'srce').glob('*.c'))
    before={str(p.relative_to(ROOT.parent)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked}
    binary=ENC/('fixture_encoder.exe' if os.name=='nt' else 'fixture_encoder')
    cmd=[compiler,'-std=c11','-O2','-fwrapv','-D_CRT_SECURE_NO_WARNINGS','-I',str(compat),'-I',str(ENC/'inc'),'-I',str(ENC/'src')]+[str(p) for p in source]
    if os.name!='nt':cmd+=['-lm']
    subprocess.run(cmd+['-o',str(binary)],env=env,check=True,cwd=OUT)
    native,reference,sources=decoder_test.build()
    lib=ctypes.CDLL(str(native));ref=ctypes.CDLL(str(reference));u8=ctypes.c_uint8;ptr=ctypes.POINTER(u8);u=ctypes.c_uint
    assert lib.probe_shift_table()==1
    lib.worker_control.argtypes=[u,ptr,u];lib.worker_packet.argtypes=[ptr,u,u,ptr,ctypes.POINTER(u)]
    lib.worker_packet_at.argtypes=[ptr,u,u,ctypes.c_uint32,ptr,ctypes.POINTER(u)]
    lib.worker_pcm_timestamp.restype=ctypes.c_uint32
    ref.reference_decode.argtypes=[ptr,u,ptr,ctypes.POINTER(u),ctypes.POINTER(u)]
    def buf(data):return (u8*len(data)).from_buffer_copy(data)
    def control(op,cie=None):return lib.worker_control(op,buf(cie) if cie else None,len(cie) if cie else 0)
    rows=[]
    cases=[(rate,mode,1) for rate in (44100,48000,88200,96000) for mode in (4,2,1)]
    cases += [(rate,1,quality) for rate in (48000,96000) for quality in (0,2)]
    for rate,mode,quality in cases:
        fixture=ENC/f'fixture_{rate}_{mode}_q{quality}.ldac'
        subprocess.run([str(binary),str(rate),str(mode),str(quality),str(fixture)],env=env,check=True,capture_output=True,text=True,cwd=OUT)
        encoded=fixture.read_bytes()
        if not encoded or len(encoded)>1048576:raise RuntimeError('Fixture exceeds bounds')
        frames=[];pos=0
        while pos<len(encoded):
            if len(encoded)-pos<3 or encoded[pos]!=0xaa:raise RuntimeError(f'Invalid encoder frame at {pos}')
            size=3+((((encoded[pos+1]&7)<<6)|(encoded[pos+2]>>2))+1)
            if size>len(encoded)-pos or len(frames)>=2000:raise RuntimeError('Truncated or excessive fixture')
            frames.append(encoded[pos:pos+size]);pos+=size
        assert lib.worker_init()==0 and control(0,decoder_test.cie(rate,mode))==0 and control(3)==0
        assert ref.reference_open(rate,mode)==0
        actual=bytearray();expected=bytearray()
        for index in range(0,len(frames),2):
            chunk=frames[index:index+2];packet=bytes([len(chunk)])+b''.join(chunk)
            output=(u8*8192)();written=u();drops=lib.worker_drops()
            samples=128 if rate<88200 else 256;timestamp=(0xfffff000+index*samples)&0xffffffff
            assert lib.worker_packet_at(buf(packet),len(packet),index//2,timestamp,output,ctypes.byref(written))==0
            assert lib.worker_drops()==drops,(rate,mode,index)
            assert lib.worker_pcm_calls()==len(chunk)
            for j in range(len(chunk)):
                assert lib.worker_pcm_timestamp(j)==(timestamp+j*samples)&0xffffffff
                assert lib.worker_pcm_samples(j)==samples and lib.worker_pcm_channels(j)==(1 if mode==4 else 2)
                assert lib.worker_pcm_width(j)==4
            actual.extend(bytes(output[:written.value]))
            for frame in chunk:
                output=(u8*8192)();used=u();written=u()
                result=ref.reference_decode(buf(frame),len(frame),output,ctypes.byref(used),ctypes.byref(written))
                assert result==0 and used.value==len(frame),(rate,mode,index,result,used.value)
                expected.extend(bytes(output[:written.value]))
        assert actual==expected and actual,(rate,mode)
        peak=max(abs(x[0]) for x in struct.iter_unpack('<i',actual));assert peak>0
        channels=1 if mode==4 else 2
        correlations=[waveform_correlation(actual,channels,ch) for ch in range(channels)]
        assert all(correlation>0.95 for correlation,_ in correlations),(rate,mode,correlations)
        ref.reference_close();assert control(5)==0 and lib.probe_live()==0
        rows.append({'rate':rate,'mode':mode,'quality':quality,'frames':len(frames),'encoded_bytes':len(encoded),
                     'pcm_bytes':len(actual),'pcm_peak':peak,'waveform_correlation':[round(c,6) for c,_ in correlations],
                     'alignment_samples':[delay for _,delay in correlations],
                     'encoded_sha256':hashlib.sha256(encoded).hexdigest(),
                     'pcm_sha256':hashlib.sha256(actual).hexdigest()})
    after={str(p.relative_to(ROOT.parent)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked}
    assert before==after,'Encoder sources changed during test'
    report={'passed':True,'scope':'offline generated audio; AOSP encoder versus owned worker and original SDK decoder; no RF or hardware',
            'encoder_source':'https://android.googlesource.com/platform/external/libldac','encoder_commit':COMMIT,
            'decoder_forced_unsigned_char':os.environ.get('DBT_LDAC_FORCE_UNSIGNED_CHAR')=='1',
            'frame_pcm_format_and_timestamp_wrap_checked':True,
            'initial_archive_sha256':ARCHIVE_SHA,'encoder_tree_sha256':TREE_SHA,'configurations':rows,'encoder_source_sha256':before,
            'decoder_source_sha256':{str(p.relative_to(ROOT.parent)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}}
    report_name='ldac_encoded_unsigned_char_results.json' if report['decoder_forced_unsigned_char'] else 'ldac_encoded_results.json'
    (OUT/report_name).write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if not k.endswith('sha256')},indent=2))


if __name__=='__main__':
    subprocess.run(CHECK,check=True)
    try:main()
    finally:subprocess.run(CHECK,check=True)
