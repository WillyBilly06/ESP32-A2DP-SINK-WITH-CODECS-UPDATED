"""Offline LHDC V5 CIE, transport bounds and allocation lifecycle tests."""
import ctypes
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
LHDC=ROOT.parent.parent/"LHDC-V5-Decoder-main/decoder"
SBC=ROOT.parent/"esp-idf/components/bt/host/bluedroid/external/sbc/decoder"
OUT=ROOT/"_build/native_codecs"
ASAN=os.environ.get("DBT_NATIVE_CODEC_ASAN")=="1"
SBC_NAMES="alloc bitalloc-sbc bitalloc bitstream-decode decoder-oina decoder-private decoder-sbc dequant framing-sbc framing oi_codec_version synthesis-8-generated synthesis-dct8 synthesis-sbc".split()
PORT_NAMES=["lhdc_dec.c","lhdc_imdct.c","lhdc_entropy_dec.c","lhdc_sns_synth.c"]


def run(args,**kwargs):
    result=subprocess.run(args,capture_output=True,text=True,**kwargs)
    if result.returncode:raise RuntimeError(f"command failed {args}\n{result.stdout[-4000:]}\n{result.stderr[-4000:]}")


def build():
    OUT.mkdir(parents=True,exist_ok=True)
    run([sys.executable,str(ROOT/"codecs/lhdcv5/prepare_port.py"),"--source",str(LHDC),"--output",str(OUT/"lhdc_port")])
    run([sys.executable,str(ROOT/"codecs/sbc/prepare_port.py"),"--source",str(SBC/"srce"),"--output",str(OUT/"sbc_port")])
    env=os.environ.copy()
    if ASAN:env.pop("LD_PRELOAD",None)
    compiler=(shutil.which("gcc") if ASAN else shutil.which("clang")) or r"C:\Program Files\LLVM\bin\clang.exe"
    if os.name=="nt":
        vc=Path(r"C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat")
        data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
        env.update(dict(line.split('=',1) for line in data.splitlines() if '=' in line and not line.startswith('=')))
    includes=[ROOT/"codecs/sbc/port",SBC/"include",ROOT/"include",ROOT/"codecs/common",ROOT/"codecs/lhdcv5",ROOT/"codecs/sbc",LHDC]
    sources=[ROOT/"tests/codecs/lhdcv5_probe.c",ROOT/"codecs/lhdcv5/dbt_lhdcv5_codec.c",ROOT/"codecs/common/dbt_codec.c",
             ROOT/"codecs/sbc/dbt_sbc_codec.c",ROOT/"codecs/sbc/dbt_sbc_decoder.c",ROOT/"src/dbt_sbc_media.c",ROOT/"src/dbt_avdtp_caps.c"]
    sources += [OUT/"lhdc_port"/name for name in PORT_NAMES]
    sources += [LHDC/name for name in ("lhdc_bit_reader.c","lhdc_diag_config.c","lhdc_tables.c")]
    sources += [(OUT/"sbc_port" if n in {"synthesis-8-generated","synthesis-dct8","synthesis-sbc"} else SBC/"srce")/(n+".c") for n in SBC_NAMES]
    dll=OUT/(("lhdcv5_native_asan" if ASAN else "lhdcv5_native")+(".dll" if os.name=="nt" else ".so"))
    args=[compiler,"-std=c11","-O2","-fwrapv","-shared","-DLHDC_HOST_BUILD","-DDBT_LHDC_NO_HOST_WINFILE","-DDBT_LHDC_PORT_SELFTEST"]
    if ASAN:args += ["-fsanitize=address","-fno-omit-frame-pointer"]
    if os.name!="nt":args += ["-fPIC"]
    for inc in includes:args += ["-I",str(inc)]
    run(args+[str(p) for p in sources]+(["-lm"] if os.name!="nt" else [])+["-o",str(dll)],env=env,cwd=OUT)
    return dll,sources


def cie(rate,depth):
    return bytes([0,255,0x3a,5,0,0,0x35,0x4c,{44100:32,48000:16,96000:4,192000:1}[rate],{16:4,24:2,32:1}[depth],0x11,0x40,0])


def main():
    dll,sources=build();lib=ctypes.CDLL(str(dll));u8=ctypes.c_uint8;ptr=ctypes.POINTER(u8);u=ctypes.c_uint
    assert lib.probe_reciprocal_table()==1
    lib.worker_control.argtypes=[u,ptr,u];lib.worker_packet.argtypes=[ptr,u,u,u]
    lib.probe_configure.argtypes=[ptr,u,ctypes.POINTER(u),ctypes.POINTER(u),ctypes.POINTER(u)]
    lib.probe_validate.argtypes=[ptr,u,ptr,u]
    def buf(data):return (u8*len(data)).from_buffer_copy(data)
    def control(op,c=None):return lib.worker_control(op,buf(c) if c else None,len(c) if c else 0)
    rates=[]
    capability=bytes([0,255,0x3a,5,0,0,0x35,0x4c,0x35,7,0x11,0x40,0])
    caps=bytes([1,0,7,len(capability)])+capability
    for rate in (44100,48000,96000,192000):
        for depth in (16,24,32):
            config=cie(rate,depth);r=u();frames=u();sample_bytes=u()
            assert lib.probe_configure(buf(config),len(config),ctypes.byref(r),ctypes.byref(frames),ctypes.byref(sample_bytes))==0
            assert (r.value,frames.value,sample_bytes.value)==(rate,240 if rate==44100 else rate//200,2 if depth==16 else 4)
            assert lib.probe_validate(buf(caps),len(caps),buf(config),len(config))==0
            lib.probe_fail_in(0);assert lib.worker_init()==0
            before=lib.probe_calls();lib.probe_peak_reset();assert control(0,config)==0;calls=lib.probe_calls()-before
            live=lib.probe_live();open_peak=lib.probe_peak();assert live>0
            assert control(3)==0
            drops=lib.worker_drops();assert lib.worker_packet(buf(bytes([0,0,1,0])),4,1,1)==0
            assert lib.worker_drops()==drops+1 and lib.probe_pcm_frames()==0
            assert control(5)==0 and lib.probe_live()==0
            rates.append(dict(rate=rate,depth=depth,open_allocations=calls,worker_live_bytes=live,open_peak_bytes=open_peak))
    config=cie(192000,24)
    narrower=bytearray(caps);narrower[12]&=~1
    assert lib.probe_validate(buf(narrower),len(narrower),buf(config),len(config))!=0
    narrower=bytearray(caps);narrower[13]&=~2
    assert lib.probe_validate(buf(narrower),len(narrower),buf(config),len(config))!=0
    for bad in (config[:-1],config[:8]+bytes([3])+config[9:],config[:10]+bytes([0x10])+config[11:],
                config[:11]+bytes([0x80])+config[12:]):
        r=u(77);frames=u(78);sample_bytes=u(79)
        assert lib.probe_configure(buf(bad),len(bad),ctypes.byref(r),ctypes.byref(frames),ctypes.byref(sample_bytes))!=0
        assert (r.value,frames.value,sample_bytes.value)==(77,78,79)
    lib.probe_fail_in(0);assert lib.worker_init()==0 and control(0,config)==0 and control(3)==0
    active=lib.probe_live();fragment=bytes([0xc2,0,1,0]);tail=bytes([0xa1,0,0])
    assert lib.worker_packet(buf(fragment),len(fragment),1,1)==0 and lib.probe_live()>active
    lib.worker_expire(101);assert lib.probe_live()==active
    before=lib.worker_drops();assert lib.worker_packet(buf(tail),len(tail),2,102)==0
    assert lib.worker_drops()==before+1 and lib.probe_pcm_frames()==0
    lib.probe_fail_in(1);before=lib.worker_drops()
    assert lib.worker_packet(buf(fragment),len(fragment),3,103)==0
    assert lib.worker_drops()==before+1 and lib.probe_live()==active
    lib.probe_fail_in(0);assert control(5)==0 and lib.probe_live()==0
    failure_sites=0
    for rate in (44100,48000,96000,192000):
        case=next(v for v in rates if v['rate']==rate and v['depth']==24)
        for fail_index in range(1,case['open_allocations']+1):
            lib.probe_fail_in(0);assert lib.worker_init()==0;lib.probe_fail_in(fail_index)
            result=control(0,cie(rate,24))
            assert result==1,(rate,fail_index,case['open_allocations'],result)
            assert lib.probe_live()==0,(rate,fail_index,lib.probe_live())
            failure_sites+=1
    sbc=bytes([0,0,0x11,0x15,2,53]);lib.probe_fail_in(0)
    assert lib.worker_init()==0 and control(0,sbc)==0
    sbc_live=lib.probe_live();lib.probe_fail_in(1)
    assert control(1,config)==1 and lib.probe_live()==sbc_live
    lib.probe_fail_in(0);lib.probe_peak_reset();assert control(1,config)==0
    lhdc_live=lib.probe_live();switch_peak=lib.probe_peak()
    fresh_peak=next(v['open_peak_bytes'] for v in rates if v['rate']==192000 and v['depth']==24)
    assert switch_peak==max(sbc_live,fresh_peak)
    lib.probe_peak_reset();assert control(1,sbc)==0 and lib.probe_peak()==max(sbc_live,lhdc_live)
    assert control(5)==0 and lib.probe_live()==0
    report=dict(passed=True,address_sanitizer=ASAN,scope="LHDC V5 CIE, inactive-memory and initialization failure lifecycle only; no valid encoded frames decoded",
                configurations=rates,open_allocation_failure_sites_four_rates=failure_sites,
                sbc_live_bytes=sbc_live,lhdc_worker_live_bytes=lhdc_live,switch_peak_bytes=switch_peak,
                reciprocal_table_exact=True,all_endpoint_selections=True,capability_subset_enforced=True,
                malformed_cie=True,fragment_expiry_and_oom=True,actual_lhdc_pcm=False,
                source_sha256={str(p.relative_to(ROOT.parent.parent)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources})
    (OUT/("lhdcv5_asan_results.json" if ASAN else "lhdcv5_lifecycle_results.json")).write_text(json.dumps(report,indent=2)+"\n")
    print(json.dumps({k:v for k,v in report.items() if k!='source_sha256'},indent=2))


if __name__=="__main__":main()
