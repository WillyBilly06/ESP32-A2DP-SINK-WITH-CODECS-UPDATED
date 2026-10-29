"""Compare an offline generated LHDC V5 tone with the native worker and sibling decoder."""
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
ENC=OUT/'lhdcv5_fixture_encoder'
DEC=ROOT.parent.parent/'LHDC-V5-Decoder-main/decoder'
COMMIT='3f9d1980fa9c57cdfb78792f9455a7dbac50edda'
TREE_SHA='6917725b0598e56af0f7ba20490cfadeb9b0910aefa822c4b71a0af726c0651a'
CHECK=[sys.executable,str(ROOT/'tools/check_workspace.py')]
sys.path.insert(0,str(ROOT/'tools'))
import test_native_codec_lhdcv5 as native_test


def run(args,**kwargs):
    result=subprocess.run(args,capture_output=True,text=True,**kwargs)
    if result.returncode:raise RuntimeError(f'command failed {args[:3]}\n{result.stdout[-2500:]}\n{result.stderr[-3500:]}')
    return result.stdout


def tone_correlation(pcm,rate,depth,samples_per_frame,channel):
    decoded=[v[0] for v in struct.iter_unpack('<h' if depth==16 else '<i',pcm)]
    points=list(range(samples_per_frame*3,min(samples_per_frame*20,18000),32))
    tone=[math.sin(2.0*math.pi*(997 if channel==0 else 1300)*i/rate) for i in points]
    energy=sum(v*v for v in tone);best=(-1.0,0)
    for delay in range(0,samples_per_frame*4+1,8):
        actual=[decoded[(i+delay)*2+channel] for i in points]
        power=sum(v*v for v in actual)
        if power:
            correlation=sum(a*b for a,b in zip(tone,actual))/math.sqrt(energy*power)
            if correlation>best[0]:best=(correlation,delay)
    return best


def main():
    encoder_tree=sorted([*(ENC/'src').glob('*.c'),*(ENC/'src').glob('*.h'),*(ENC/'include').glob('*.h'),
                         *(ENC/'examples').glob('*.c'),ENC/'LICENSE',ENC/'README.md'],key=lambda p:p.relative_to(ENC).as_posix())
    digest=hashlib.sha256()
    for p in encoder_tree:digest.update(p.relative_to(ENC).as_posix().encode()+b'\0');digest.update(p.read_bytes())
    if len(encoder_tree)!=17 or digest.hexdigest()!=TREE_SHA:raise RuntimeError('Missing or unreviewed test encoder tree')
    compiler=shutil.which('clang') or r'C:\Program Files\LLVM\bin\clang.exe';env=os.environ.copy()
    if os.name=='nt':
        vc=Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
        data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
        env.update(dict(x.split('=',1) for x in data.splitlines() if '=' in x and not x.startswith('=')))
    for key in ('LHDC_WINFILE','LHDC_PKALL','LHDC_FORCE_REF'):env.pop(key,None)
    sources=list((ENC/'src').glob('*.c'))+[ROOT/'tests/codecs/lhdcv5_encoder_fixture.c']
    originals=[DEC/n for n in ('lhdc_bit_reader.c','lhdc_dec.c','lhdc_diag_config.c','lhdc_entropy_dec.c','lhdc_imdct.c','lhdc_sns_synth.c','lhdc_tables.c')]
    tracked=encoder_tree+sources+[Path(__file__).resolve(),Path(native_test.__file__).resolve(),ROOT/'tests/codecs/lhdcv5_probe.c',
             ROOT/'tests/codecs/lhdcv5_reference_probe.c',ROOT/'codecs/common/dbt_codec.c',ROOT/'src/dbt_avdtp_caps.c',
             ROOT/'codecs/lhdcv5/dbt_lhdcv5_codec.c',ROOT/'codecs/lhdcv5/prepare_port.py']+originals+list(DEC.glob('*.h'))
    before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in tracked}
    binary=ENC/('fixture_encoder.exe' if os.name=='nt' else 'fixture_encoder')
    flags=[compiler,'-std=c11','-O2','-fwrapv','-D_CRT_SECURE_NO_WARNINGS','-I',str(ENC/'include'),'-I',str(ENC/'src')]
    run(flags+[str(p) for p in sources]+([] if os.name=='nt' else ['-lm'])+['-o',str(binary)],env=env,cwd=OUT)
    dll,_=native_test.build()
    suffix='.dll' if os.name=='nt' else '.so';ref_path=OUT/('lhdcv5_original_reference'+suffix)
    ref_sources=[ROOT/'tests/codecs/lhdcv5_reference_probe.c']+originals
    flags=[compiler,'-std=c11','-O2','-fwrapv','-shared','-DLHDC_HOST_BUILD','-I',str(DEC)]
    if os.name!='nt':flags+=['-fPIC']
    run(flags+[str(p) for p in ref_sources]+([] if os.name=='nt' else ['-lm'])+['-o',str(ref_path)],env=env,cwd=OUT)
    native=ctypes.CDLL(str(dll));ref=ctypes.CDLL(str(ref_path));u8=ctypes.c_uint8;ptr=ctypes.POINTER(u8);u=ctypes.c_uint
    native.worker_control.argtypes=[u,ptr,u]
    native.worker_packet_pcm.argtypes=[ptr,u,u,u,ctypes.c_uint32,ptr,u,ctypes.POINTER(u)]
    native.worker_pcm_timestamp.restype=ctypes.c_uint32
    ref.reference_frame.argtypes=[ptr,u,ptr,u,ctypes.POINTER(u),ctypes.POINTER(u)]
    def buf(data):return (u8*len(data)).from_buffer_copy(data)
    def control(op,cie=None):return native.worker_control(op,buf(cie) if cie else None,len(cie) if cie else 0)
    rows=[]
    for rate in (44100,48000,96000,192000):
        for depth in (16,24):
            path=ENC/f'tone_{rate}_{depth}.packets'
            run([str(binary),str(rate),str(depth),'5',str(path)],env=env,cwd=OUT)
            data=path.read_bytes()
            if not data or len(data)>1048576:raise RuntimeError('Encoder fixture exceeds bound')
            config=native_test.cie(rate,depth);assert native.worker_init()==0 and control(0,config)==0 and control(3)==0
            assert ref.reference_open(rate,depth)==0
            actual=bytearray();expected=bytearray();pos=0;seq=0;frames_total=0;samples=240 if rate==44100 else rate//200
            while pos<len(data):
                if len(data)-pos<4:raise RuntimeError('Truncated packet record')
                n=int.from_bytes(data[pos:pos+2],'little');frames=int.from_bytes(data[pos+2:pos+4],'little');pos+=4
                if not n or n>2046 or not frames or frames>15 or n>len(data)-pos:raise RuntimeError('Invalid encoder packet record')
                payload=data[pos:pos+n];pos+=n;packet=bytes([0,0])+payload
                out=(u8*65536)();written=u();drops=native.worker_drops();timestamp=(0xfffff000+frames_total*samples)&0xffffffff
                assert native.worker_packet_pcm(buf(packet),len(packet),seq,1+seq,timestamp,out,len(out),ctypes.byref(written))==0
                if native.worker_drops()!=drops:raise AssertionError(('worker dropped actual encoded packet',rate,depth,seq,frames,n))
                assert native.worker_pcm_calls()==frames,(rate,depth,seq,frames,native.worker_pcm_calls())
                for i in range(frames):assert native.worker_pcm_timestamp(i)==((timestamp+i*samples)&0xffffffff)
                actual.extend(bytes(out[:written.value]));off=0
                for _ in range(frames):
                    if len(payload)-off<2:raise RuntimeError('Truncated encoded frame')
                    size=2+2*(int.from_bytes(payload[off:off+2],'little')&1023)
                    if size>len(payload)-off:raise RuntimeError('Invalid encoded frame')
                    raw=payload[off:off+size];out=(u8*8192)();used=u();generated=u()
                    result=ref.reference_frame(buf(raw),len(raw),out,samples,ctypes.byref(used),ctypes.byref(generated))
                    assert result==0 and used.value==len(raw) and generated.value==samples,(rate,depth,seq,result,used.value,generated.value)
                    expected.extend(bytes(out[:samples*2*(2 if depth==16 else 4)]));off+=size
                assert off==len(payload);frames_total+=frames;seq+=1
            assert actual and actual==expected,(rate,depth,len(actual),len(expected))
            peak=max(abs(x[0]) for x in struct.iter_unpack('<h' if depth==16 else '<i',actual));assert peak>0
            correlations=[tone_correlation(actual,rate,depth,samples,ch) for ch in range(2)]
            assert all(correlation>0.98 for correlation,_ in correlations),(rate,depth,correlations)
            # Replay one valid encoded packet split across two RTP fragments.
            # The marker byte belongs to the reassembled payload and is copied
            # only in the first fragment; allocation growth is measured.
            cut=len(payload)//2;first=bytes([0xc2,0])+payload[:cut];last=bytes([0xa1])+payload[cut:]
            baseline=native.probe_live();native.probe_peak_reset();out=(u8*65536)();written=u();drops=native.worker_drops()
            fragment_ts=(0xfffff000+frames_total*samples)&0xffffffff
            assert native.worker_packet_pcm(buf(first),len(first),seq,1+seq,fragment_ts,out,len(out),ctypes.byref(written))==0
            assert written.value==0 and native.worker_pcm_calls()==0
            assert native.worker_packet_pcm(buf(last),len(last),seq+1,2+seq,fragment_ts,out,len(out),ctypes.byref(written))==0
            assert native.worker_drops()==drops and native.worker_pcm_calls()==frames
            fragment_pcm=bytes(out[:written.value]);fragment_reference=bytearray();off=0
            for i in range(frames):
                assert native.worker_pcm_timestamp(i)==((fragment_ts+i*samples)&0xffffffff)
                size=2+2*(int.from_bytes(payload[off:off+2],'little')&1023);raw=payload[off:off+size]
                pcm=(u8*8192)();used=u();generated=u()
                result=ref.reference_frame(buf(raw),len(raw),pcm,samples,ctypes.byref(used),ctypes.byref(generated))
                assert result==0 and used.value==len(raw) and generated.value==samples
                fragment_reference.extend(bytes(pcm[:samples*2*(2 if depth==16 else 4)]));off+=size
            assert off==len(payload) and fragment_pcm==fragment_reference and native.probe_live()==baseline
            fragment_peak=native.probe_peak()
            ref.reference_close();assert control(5)==0 and native.probe_live()==0
            rows.append({'rate':rate,'depth':depth,'frames':frames_total,'packets':seq,'pcm_bytes':len(actual),
                         'pcm_peak':peak,'tone_correlation':[round(c,6) for c,_ in correlations],
                         'tone_alignment_samples':[delay for _,delay in correlations],
                         'fragment_reassembly_peak_bytes':fragment_peak,
                         'encoded_sha256':hashlib.sha256(data).hexdigest(),'pcm_sha256':hashlib.sha256(actual).hexdigest()})
    assert all(hashlib.sha256(Path(path).read_bytes()).hexdigest()==sha for path,sha in before.items()),'Source changed during test'
    report={'passed':True,'scope':'test encoder frames compared with native worker and original sibling decoder; no RF or target execution',
            'encoder_source':'https://github.com/WillyBilly06/LHDC-V5-Encoder','encoder_commit':COMMIT,'encoder_tree_sha256':TREE_SHA,
            'per_frame_timestamps_checked':True,'encoded_fragment_reassembly_checked':True,
            'tone_correlation_minimum':0.98,'configurations':rows,'source_sha256':before}
    (OUT/'lhdcv5_encoded_results.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='source_sha256'},indent=2))


if __name__=='__main__':
    subprocess.run(CHECK,check=True)
    try:main()
    finally:subprocess.run(CHECK,check=True)
