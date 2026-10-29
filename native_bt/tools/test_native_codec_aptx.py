"""Offline SDK freeaptx encoder/decoder and native worker comparison."""
import ctypes
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
BT=ROOT.parent/"esp-idf/components/bt/host/bluedroid/external"
APTX=BT/"libfreeaptx/main"
SBC=BT/"sbc/decoder"
OUT=ROOT/"_build/native_codecs"
ASAN=os.environ.get("DBT_NATIVE_CODEC_ASAN")=="1"
SBC_NAMES="alloc bitalloc-sbc bitalloc bitstream-decode decoder-oina decoder-private decoder-sbc dequant framing-sbc framing oi_codec_version synthesis-8-generated synthesis-dct8 synthesis-sbc".split()


def run(args,**kwargs):
    result=subprocess.run(args,capture_output=True,text=True,**kwargs)
    if result.returncode:raise RuntimeError(f"command failed {args}\n{result.stdout[-3000:]}\n{result.stderr[-3000:]}")


def build():
    OUT.mkdir(parents=True,exist_ok=True)
    run([sys.executable,str(ROOT/"codecs/sbc/prepare_port.py"),"--source",str(SBC/"srce"),"--output",str(OUT/"sbc_port")])
    env=os.environ.copy()
    if ASAN:env.pop("LD_PRELOAD",None)
    compiler=(shutil.which("gcc") if ASAN else shutil.which("clang")) or r"C:\Program Files\LLVM\bin\clang.exe"
    if os.name=="nt":
        vc=Path(r"C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat")
        data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
        env.update(dict(line.split('=',1) for line in data.splitlines() if '=' in line and not line.startswith('=')))
    includes=[ROOT/"codecs/sbc/port",SBC/"include",ROOT/"include",ROOT/"codecs/common",ROOT/"codecs/aptx",ROOT/"codecs/sbc",APTX]
    sources=[ROOT/"tests/codecs/aptx_real_probe.c",ROOT/"codecs/aptx/dbt_aptx.c",ROOT/"codecs/aptx/dbt_aptx_backend.c",
             ROOT/"codecs/common/dbt_codec.c",ROOT/"codecs/sbc/dbt_sbc_codec.c",ROOT/"codecs/sbc/dbt_sbc_decoder.c",
             ROOT/"src/dbt_sbc_media.c",ROOT/"src/dbt_avdtp_caps.c"]
    sources += [(OUT/"sbc_port" if n in {"synthesis-8-generated","synthesis-dct8","synthesis-sbc"} else SBC/"srce")/(n+".c") for n in SBC_NAMES]
    dll=OUT/(("aptx_native_asan" if ASAN else "aptx_native")+(".dll" if os.name=="nt" else ".so"))
    args=[compiler,"-std=c11","-O2","-fwrapv","-shared"]
    if ASAN:args += ["-fsanitize=address","-fno-omit-frame-pointer"]
    if os.name!="nt":args += ["-fPIC"]
    for inc in includes:args += ["-I",str(inc)]
    run(args+[str(p) for p in sources]+["-o",str(dll)],env=env,cwd=OUT)
    return dll,sources+[APTX/"freeaptx.c",APTX/"freeaptx.h",ROOT/"codecs/aptx/dbt_aptx.h",ROOT/"codecs/common/dbt_codec.h"]


def cie(kind,rate):
    r=0x10 if rate==48000 else 0x20
    if kind=="standard":return bytes([0,255,0x4f,0,0,0,1,0,r|2])
    if kind=="hd":return bytes([0,255,0xd7,0,0,0,0x24,0,r|2,0,0,0,0])
    return bytes([0,255,0x0a,0,0,0,2,0,r|2,0])


def main():
    dll,sources=build();lib=ctypes.CDLL(str(dll));u8=ctypes.c_uint8;ptr=ctypes.POINTER(u8);u=ctypes.c_uint
    lib.worker_control.argtypes=[u,ptr,u];lib.worker_packet.argtypes=[ptr,u,u,ptr,ctypes.POINTER(u)]
    lib.reference_encode.argtypes=[ptr,u,ptr,u,ctypes.POINTER(u)];lib.reference_decode.argtypes=lib.reference_encode.argtypes
    lib.probe_configure.argtypes=[ptr,u,ctypes.POINTER(u),ctypes.POINTER(u),ctypes.POINTER(u)]
    lib.probe_validate.argtypes=[ptr,u,ptr,u]
    def buf(data):return (u8*len(data)).from_buffer_copy(data)
    def control(op,c=None):return lib.worker_control(op,buf(c) if c else None,len(c) if c else 0)
    def packet(data,seq):
        out=(u8*8192)();n=u();r=lib.worker_packet(buf(data),len(data),seq,out,ctypes.byref(n));return r,bytes(out[:n.value])
    vectors=[]
    for kind in ("standard","hd","ll"):
        for rate in (44100,48000):
            config=cie(kind,rate);r=u();channels=u();sample_bytes=u()
            assert lib.probe_configure(buf(config),len(config),ctypes.byref(r),ctypes.byref(channels),ctypes.byref(sample_bytes))==0
            assert (r.value,channels.value,sample_bytes.value)==(rate,2,4)
            assert lib.reference_open(kind=="hd")==0
            samples=4800;pcm=bytearray()
            for i in range(samples):
                for channel in range(2):
                    value=round(2000000*math.sin(2*math.pi*(997+channel*300)*i/rate))
                    pcm.extend((value&0xffffff).to_bytes(3,'little'))
            encoded=(u8*8192)();n=u()
            assert lib.reference_encode(buf(pcm),len(pcm),encoded,len(encoded),ctypes.byref(n))==len(pcm)
            encoded=bytes(encoded[:n.value]);assert len(encoded)>0
            quantum=6 if kind=="hd" else 4;chunk=(192//quantum)*quantum
            assert len(encoded)%quantum==0
            assert lib.worker_init()==0 and control(0,config)==0 and control(3)==0
            live=lib.probe_live();actual=bytearray();reference=bytearray()
            for seq,start in enumerate(range(0,len(encoded),chunk)):
                part=encoded[start:start+chunk];r,decoded=packet(part,seq);assert r==0
                out=(u8*8192)();n=u();used=lib.reference_decode(buf(part),len(part),out,len(out),ctypes.byref(n))
                assert used==len(part);actual.extend(decoded);reference.extend(bytes(out[:n.value]))
            assert actual==reference and len(actual)>0
            before=lib.worker_drops();r,decoded=packet(bytes([0,1,2]),1000)
            assert r==0 and not decoded and lib.worker_drops()==before+1
            assert control(5)==0 and lib.probe_live()==0;lib.reference_close()
            vectors.append(dict(codec=kind,rate=rate,encoded_bytes=len(encoded),pcm_bytes=len(actual),
                                encoded_sha256=hashlib.sha256(encoded).hexdigest(),pcm_sha256=hashlib.sha256(actual).hexdigest(),worker_live_bytes=live))
    # Failure of the decoder or packet queue during reconfiguration restores SBC.
    sbc=bytes([0,0,0x11,0x15,2,53]);config=cie("hd",48000)
    for invalid in (config[:-1],config[:8]+bytes([0x11])+config[9:],config[:9]+bytes([1])+config[10:]):
        rate=u(77);channels=u(78);sample_bytes=u(79)
        assert lib.probe_configure(buf(invalid),len(invalid),ctypes.byref(rate),ctypes.byref(channels),ctypes.byref(sample_bytes))!=0
        assert (rate.value,channels.value,sample_bytes.value)==(77,78,79)
    caps=bytes([1,0,7,len(config)])+config
    assert lib.probe_validate(buf(caps),len(caps),buf(config),len(config))==0
    badcaps=bytearray(caps);badcaps[12]=0x22
    assert lib.probe_validate(buf(badcaps),len(badcaps),buf(config),len(config))!=0
    assert lib.worker_init()==0 and control(0,sbc)==0
    sbc_live=lib.probe_live();lib.probe_fail_in(1);assert control(1,config)==1 and lib.probe_live()==sbc_live
    lib.probe_fail_in(2);assert control(1,config)==1 and lib.probe_live()==sbc_live
    lib.probe_peak_reset();assert control(1,config)==0
    aptx_live=lib.probe_live();assert lib.probe_peak()==max(sbc_live,aptx_live)
    lib.probe_peak_reset();assert control(1,sbc)==0 and lib.probe_peak()==max(sbc_live,aptx_live)
    assert control(5)==0 and lib.probe_live()==0
    report=dict(passed=True,address_sanitizer=ASAN,scope="offline SDK freeaptx encode and native worker PCM versus standalone reference; no target or RF evidence",
                vectors=vectors,byte_exact_standalone_reference=True,sbc_live_bytes=sbc_live,aptx_worker_live_bytes=aptx_live,
                switch_peak_bytes=max(sbc_live,aptx_live),decoder_and_queue_oom_rollback=True,malformed_cie_and_packet=True,
                source_sha256={str(p.relative_to(ROOT.parent)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources})
    (OUT/("aptx_asan_results.json" if ASAN else "aptx_real_results.json")).write_text(json.dumps(report,indent=2)+"\n")
    print(json.dumps({k:v for k,v in report.items() if k!='source_sha256'},indent=2))


if __name__=="__main__":main()
