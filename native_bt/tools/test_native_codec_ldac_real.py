"""Offline LDAC worker/backend lifecycle and synthetic-frame comparison."""
import ctypes
import argparse
import hashlib
import json
import os
from pathlib import Path
import random
import shutil
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
SDK=ROOT.parent/"esp-idf/components/bt/host/bluedroid/external/libldac-dec"
SBC=ROOT.parent/"esp-idf/components/bt/host/bluedroid/external/sbc/decoder"
OUT=ROOT/"_build/native_codecs"
ASAN=os.environ.get("DBT_NATIVE_CODEC_ASAN")=="1"
SBC_NAMES="alloc bitalloc-sbc bitalloc bitstream-decode decoder-oina decoder-private decoder-sbc dequant framing-sbc framing oi_codec_version synthesis-8-generated synthesis-dct8 synthesis-sbc".split()


def run(args,**kwargs):
    result=subprocess.run(args,capture_output=True,text=True,**kwargs)
    if result.returncode:raise RuntimeError(f"command failed {args}\n{result.stdout[-4000:]}\n{result.stderr[-4000:]}")


def build():
    OUT.mkdir(parents=True,exist_ok=True)
    run([sys.executable,str(ROOT/"codecs/ldac/prepare_port.py"),"--source",str(SDK/"src"),"--output",str(OUT/"ldac_port")])
    run([sys.executable,str(ROOT/"codecs/sbc/prepare_port.py"),"--source",str(SBC/"srce"),"--output",str(OUT/"sbc_port")])
    env=os.environ.copy()
    if ASAN:env.pop("LD_PRELOAD",None)
    compiler=(shutil.which("gcc") if ASAN else shutil.which("clang")) or r"C:\Program Files\LLVM\bin\clang.exe"
    if os.name=="nt":
        vc=Path(r"C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat")
        data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
        env.update(dict(line.split('=',1) for line in data.splitlines() if '=' in line and not line.startswith('=')))
    includes=[ROOT/"codecs/sbc/port",SBC/"include",ROOT/"include",ROOT/"codecs/common",ROOT/"codecs/ldac",ROOT/"codecs/sbc",OUT/"ldac_port",SDK/"inc",SDK/"src"]
    sources=[ROOT/"tests/codecs/ldac_real_probe.c",ROOT/"codecs/ldac/dbt_ldac.c",ROOT/"codecs/ldac/dbt_ldac_decoder.c",
             ROOT/"codecs/ldac/dbt_ldac_bt.c",ROOT/"codecs/ldac/dbt_ldac_lib.c",ROOT/"codecs/common/dbt_codec.c",
             ROOT/"codecs/sbc/dbt_sbc_codec.c",ROOT/"codecs/sbc/dbt_sbc_decoder.c",ROOT/"src/dbt_sbc_media.c",ROOT/"src/dbt_avdtp_caps.c"]
    sources += [(OUT/"sbc_port" if n in {"synthesis-8-generated","synthesis-dct8","synthesis-sbc"} else SBC/"srce")/(n+".c") for n in SBC_NAMES]
    suffix=".dll" if os.name=="nt" else ".so"
    dll=OUT/(("ldac_native_asan" if ASAN else "ldac_native")+suffix)
    flags=[compiler,"-std=c11","-O2","-fwrapv","-shared","-DDBT_LDAC_PORT_SELFTEST"]
    if os.environ.get('DBT_LDAC_FORCE_UNSIGNED_CHAR')=='1':flags += ['-funsigned-char']
    if ASAN:flags += ["-fsanitize=address","-fno-omit-frame-pointer"]
    if os.name!="nt":flags += ["-fPIC"]
    args=flags.copy()
    for inc in includes:args += ["-I",str(inc)]
    run(args+[str(p) for p in sources]+(["-lm"] if os.name!="nt" else [])+["-o",str(dll)],env=env,cwd=OUT)
    ref=OUT/(("ldac_reference_asan" if ASAN else "ldac_reference")+suffix)
    ref_sources=[ROOT/"tests/codecs/ldac_reference_probe.c",SDK/"src/ldacBT.c",SDK/"src/ldaclib.c"]
    ref_args=flags+["-I",str(SDK/"inc"),"-I",str(SDK/"src")]+[str(p) for p in ref_sources]
    if os.name!="nt":ref_args += ["-Wl,-Bsymbolic","-lm"]
    run(ref_args+["-o",str(ref)],env=env,cwd=OUT)
    sources += list((SDK/"src").glob("*.h"))+list((SDK/"src").glob("*.o.c"))+list((OUT/"ldac_port").glob("*.c"))+list((OUT/"ldac_port").glob("*.h"))+ref_sources+[SDK/"inc/ldacBT.h",ROOT/"codecs/ldac/dbt_ldac_port.h"]
    return dll,ref,sources


def cie(rate,mode):return bytes([0,255,0x2d,1,0,0,0xaa,0,{44100:32,48000:16,88200:8,96000:4}[rate],mode])
def synthetic(rate_index,mode,body=120,data=None):
    index={4:0,2:1,1:2}[mode];value=body-1
    return bytes([1,0xaa,(rate_index<<5)|(index<<3)|(value>>6),(value&63)<<2])+(bytes(body) if data is None else data)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--fixture',type=Path,help='existing raw concatenated LDAC frames (no RTP header)')
    parser.add_argument('--rate',type=int,choices=(44100,48000,88200,96000),default=48000)
    parser.add_argument('--mode',type=int,choices=(1,2,4),default=1)
    args=parser.parse_args()
    dll,ref_dll,sources=build();lib=ctypes.CDLL(str(dll));ref=ctypes.CDLL(str(ref_dll));u8=ctypes.c_uint8;ptr=ctypes.POINTER(u8);u=ctypes.c_uint
    assert lib.probe_shift_table()==1
    lib.worker_control.argtypes=[u,ptr,u];lib.worker_packet.argtypes=[ptr,u,u,ptr,ctypes.POINTER(u)]
    ref.reference_decode.argtypes=[ptr,u,ptr,ctypes.POINTER(u),ctypes.POINTER(u)]
    def buf(data):return (u8*len(data)).from_buffer_copy(data)
    def control(op,c=None):return lib.worker_control(op,buf(c) if c else None,len(c) if c else 0)
    def packet(data,seq):
        out=(u8*8192)();n=u();r=lib.worker_packet(buf(data),len(data),seq,out,ctypes.byref(n));return r,bytes(out[:n.value])
    configurations=[];comparisons=0
    for index,rate in enumerate((44100,48000,88200,96000)):
        for mode in (4,2,1):
            config=cie(rate,mode);lib.probe_fail_in(0);assert lib.worker_init()==0
            before=lib.probe_calls();lib.probe_peak_reset();assert control(0,config)==0;allocs=lib.probe_calls()-before
            live=lib.probe_live();assert live>0 and control(3)==0
            frame=synthetic(index,mode);before=lib.worker_drops();status,actual=packet(frame,1)
            assert status==0
            assert ref.reference_open(rate,mode)==0
            out=(u8*8192)();used=u();written=u();rr=ref.reference_decode(buf(frame[1:]),len(frame)-1,out,ctypes.byref(used),ctypes.byref(written))
            ref.reference_close()
            if rr==0 and used.value==len(frame)-1:
                assert lib.worker_drops()==before and actual==bytes(out[:written.value]);comparisons+=1
            else:
                assert lib.worker_drops()==before+1 and not actual
            assert control(5)==0 and lib.probe_live()==0
            configurations.append(dict(rate=rate,mode=mode,open_allocations=allocs,worker_live_bytes=live,
                                       synthetic_reference_accepted=rr==0 and used.value==len(frame)-1))
    failure_sites=0
    for case in configurations:
        config=cie(case['rate'],case['mode'])
        for fail_index in range(1,case['open_allocations']+1):
            lib.probe_fail_in(0);assert lib.worker_init()==0;lib.probe_fail_in(fail_index)
            assert control(0,config)==1,(case,fail_index)
            assert lib.probe_live()==0,(case,fail_index,lib.probe_live());failure_sites+=1
    # Reset can allocate again after releasing the previous working set. Each
    # failure must discard its partial codec state before the worker faults.
    reset_failures=0;reset_peaks=[]
    for case in configurations:
        config=cie(case['rate'],case['mode']);lib.probe_fail_in(0)
        assert lib.worker_init()==0 and control(0,config)==0
        reset_live=lib.probe_live();lib.probe_peak_reset();before=lib.probe_calls()
        assert control(3)==0;reset_allocations=lib.probe_calls()-before;reset_peak=lib.probe_peak()
        assert reset_peak==reset_live,(case,reset_peak,reset_live)
        assert reset_allocations>0 and control(5)==0 and lib.probe_live()==0
        reset_peaks.append(dict(rate=case['rate'],mode=case['mode'],allocations=reset_allocations,peak_bytes=reset_peak))
        for fail_index in range(1,reset_allocations+1):
            lib.probe_fail_in(0);assert lib.worker_init()==0 and control(0,config)==0
            original_live=lib.probe_live();lib.probe_fail_in(fail_index)
            assert control(3)==2,(case,fail_index,reset_allocations)
            assert 0<lib.probe_live()<original_live,(case,fail_index,lib.probe_live(),original_live)
            lib.probe_fail_in(0);assert control(5)==0 and lib.probe_live()==0
            reset_failures+=1
    config=cie(96000,1)
    sbc=bytes([0,0,0x11,0x15,2,53]);lib.probe_fail_in(0)
    assert lib.worker_init()==0 and control(0,sbc)==0
    sbc_live=lib.probe_live();lib.probe_fail_in(1);assert control(1,config)==1 and lib.probe_live()==sbc_live
    lib.probe_fail_in(0);lib.probe_peak_reset();assert control(1,config)==0
    ldac_live=lib.probe_live();assert lib.probe_peak()==max(sbc_live,ldac_live)
    lib.probe_peak_reset();assert control(1,sbc)==0 and lib.probe_peak()==max(sbc_live,ldac_live)
    assert control(5)==0 and lib.probe_live()==0
    fixture_result=None
    if args.fixture:
        data=args.fixture.read_bytes()
        if not data or len(data)>1048576:raise ValueError('fixture must contain at most 1 MiB of encoded frames')
        frames=[];offset=0
        while offset<len(data):
            if len(data)-offset<3 or data[offset]!=0xaa:raise ValueError('invalid LDAC frame header')
            body=(((data[offset+1]&7)<<6)|(data[offset+2]>>2))+1;length=body+3
            if length>len(data)-offset or len(frames)>=2000:raise ValueError('truncated or too many LDAC frames')
            frames.append(data[offset:offset+length]);offset+=length
        config=cie(args.rate,args.mode);assert lib.worker_init()==0 and control(0,config)==0 and control(3)==0
        assert ref.reference_open(args.rate,args.mode)==0
        actual=bytearray();expected=bytearray()
        for i,frame in enumerate(frames):
            drops=lib.worker_drops();status,pcm=packet(bytes([1])+frame,i)
            output=(u8*8192)();used=u();written=u()
            rr=ref.reference_decode(buf(frame),len(frame),output,ctypes.byref(used),ctypes.byref(written))
            assert status==0 and rr==0 and used.value==len(frame) and lib.worker_drops()==drops,(i,rr,used.value)
            actual.extend(pcm);expected.extend(bytes(output[:written.value]))
        assert actual==expected and actual
        ref.reference_close();assert control(5)==0 and lib.probe_live()==0
        fixture_result=dict(frames=len(frames),encoded_sha256=hashlib.sha256(data).hexdigest(),
                            pcm_sha256=hashlib.sha256(actual).hexdigest(),pcm_bytes=len(actual),rate=args.rate,mode=args.mode)
    fuzz_count=0
    if ASAN or os.environ.get('DBT_LDAC_FUZZ')=='1':
        rng=random.Random(0x1dac);config=cie(48000,1)
        assert lib.worker_init()==0 and control(0,config)==0 and control(3)==0
        for i in range(2000):
            body=rng.randint(22,200);data=bytes(rng.randrange(256) for _ in range(body))
            status,_=packet(synthetic(1,1,body,data),i)
            assert status==0 and not lib.worker_faults();fuzz_count+=1
        assert control(5)==0 and lib.probe_live()==0
    report=dict(passed=True,address_sanitizer=ASAN,scope="LDAC lifecycle and synthetic frame comparison; no genuine encoded fixture or RF evidence",
                configurations=configurations,synthetic_reference_pcm_matches=comparisons,actual_encoded_fixture=fixture_result,
                open_allocation_failures_checked=failure_sites,reset_allocation_failures_checked=reset_failures,
                successful_reset_peaks=reset_peaks,
                sbc_live_bytes=sbc_live,ldac_worker_live_bytes=ldac_live,
                switch_peak_bytes=max(sbc_live,ldac_live),random_packets=fuzz_count,
                source_sha256={str(p.relative_to(ROOT.parent)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources})
    (OUT/("ldac_asan_results.json" if ASAN else "ldac_real_results.json")).write_text(json.dumps(report,indent=2)+"\n")
    print(json.dumps({k:v for k,v in report.items() if k!='source_sha256'},indent=2))


if __name__=="__main__":main()
