"""Offline native-worker Opus tests using the existing SDK's standalone libopus."""
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
BT=ROOT.parent/"esp-idf/components/bt/host/bluedroid"
OPUS=BT/"external/opus"
SBC=BT/"external/sbc/decoder"
OUT=ROOT/"_build/native_codecs"
ASAN=os.environ.get("DBT_NATIVE_CODEC_ASAN")=="1"
BUILD=OUT/("opus_asan_build" if ASAN else "opus_build")
SBC_NAMES="alloc bitalloc-sbc bitalloc bitstream-decode decoder-oina decoder-private decoder-sbc dequant framing-sbc framing oi_codec_version synthesis-8-generated synthesis-dct8 synthesis-sbc".split()


def run(args, **kwargs):
    result=subprocess.run(args,capture_output=True,text=True,**kwargs)
    if result.returncode: raise RuntimeError(f"command failed ({result.returncode}): {args}\n{result.stdout[-5000:]}\n{result.stderr[-5000:]}")


def build():
    OUT.mkdir(parents=True,exist_ok=True)
    run([sys.executable,str(ROOT/"codecs/sbc/prepare_port.py"),"--source",str(SBC/"srce"),"--output",str(OUT/"sbc_port")])
    env=os.environ.copy()
    if ASAN: env.pop("LD_PRELOAD",None)
    compiler=(shutil.which("gcc") if ASAN else shutil.which("clang")) or r"C:\Program Files\LLVM\bin\clang.exe"
    if os.name=="nt":
        vc=Path(r"C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat")
        data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
        env.update(dict(line.split('=',1) for line in data.splitlines() if '=' in line and not line.startswith('=')))
    ninja=shutil.which("ninja") or str(Path(os.environ.get("USERPROFILE",""))/".espressif/tools/ninja/1.12.1/ninja.exe")
    cmake=shutil.which("cmake")
    if not cmake: raise RuntimeError("CMake is required")
    cmake_args=[cmake,"-S",str(OPUS),"-B",str(BUILD),"-G","Ninja",f"-DCMAKE_MAKE_PROGRAM={ninja}",
         f"-DCMAKE_C_COMPILER={compiler}","-DCMAKE_BUILD_TYPE=Release","-DCMAKE_EXPORT_COMPILE_COMMANDS=ON",
         "-DOPUS_BUILD_SHARED_LIBRARY=ON","-DOPUS_BUILD_TESTING=OFF","-DOPUS_BUILD_PROGRAMS=OFF",
         "-DOPUS_FIXED_POINT=ON","-DOPUS_ENABLE_FLOAT_API=OFF","-DOPUS_DISABLE_INTRINSICS=ON"]
    if ASAN:cmake_args += ["-DCMAKE_C_FLAGS=-fsanitize=address -fno-omit-frame-pointer -fwrapv", "-DCMAKE_SHARED_LINKER_FLAGS=-fsanitize=address"]
    run(cmake_args,env=env)
    run([cmake,"--build",str(BUILD),"--parallel","4"],env=env)
    guarded=OUT/"opus_guarded_build"
    guarded_args=list(cmake_args);guarded_args[guarded_args.index("-B")+1]=str(guarded)
    guarded_args=[x.replace("OPUS_BUILD_SHARED_LIBRARY=ON","OPUS_BUILD_SHARED_LIBRARY=OFF") for x in guarded_args if not x.startswith("-DCMAKE_C_FLAGS=")]
    override=(ROOT/"codecs/opus/scratch/dbt_opus_scratch_override.h").as_posix()
    guard_flags=f'-include "{override}" -DDBT_OPUS_GUARDED_API=1 -std=c11 -fwrapv'
    if ASAN:guard_flags+=' -fsanitize=address -fno-omit-frame-pointer'
    run(guarded_args+["-DCMAKE_C_FLAGS="+guard_flags],env=env)
    run([cmake,"--build",str(guarded),"--parallel","4"],env=env)
    libraries=list(BUILD.glob("opus.dll" if os.name=="nt" else "libopus.so*"))
    if not libraries: raise RuntimeError("CMake did not produce an Opus shared library")
    opus_dll=libraries[0]
    includes=[ROOT/"codecs/sbc/port",SBC/"include",ROOT/"include",ROOT/"codecs/common",ROOT/"codecs/opus",ROOT/"codecs/sbc",OPUS/"include"]
    sources=[ROOT/"tests/codecs/opus_real_probe.c",ROOT/"codecs/opus/dbt_opus.c",ROOT/"codecs/common/dbt_codec.c",
             ROOT/"codecs/sbc/dbt_sbc_codec.c",ROOT/"codecs/sbc/dbt_sbc_decoder.c",ROOT/"src/dbt_sbc_media.c",ROOT/"src/dbt_avdtp_caps.c"]
    sources += [(OUT/"sbc_port" if n in {"synthesis-8-generated","synthesis-dct8","synthesis-sbc"} else SBC/"srce")/(n+".c") for n in SBC_NAMES]
    dll=OUT/(("opus_native_asan" if ASAN else "opus_native")+(".dll" if os.name=="nt" else ".so"))
    args=[compiler,"-std=c11","-O2","-fwrapv","-shared"]
    if ASAN:args += ["-fsanitize=address","-fno-omit-frame-pointer"]
    if os.name!="nt": args += ["-fPIC"]
    for inc in includes: args += ["-I",str(inc)]
    sources.append(ROOT/"codecs/opus/scratch/dbt_opus_scratch.c")
    args += [str(p) for p in sources]
    args += ([str(guarded/"opus.lib")] if os.name=="nt" else [str(guarded/"libopus.a"),"-lm"])
    run(args+["-o",str(dll)],env=env,cwd=OUT)
    stubs=OUT/"opus_reference_stubs"
    for name in ("common/bt_trace.h","osi/allocator.h","stack/a2dp_vendor_opus.h","stack/a2dp_vendor_opus_decoder.h"):
        path=stubs/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_text('#include "opus_legacy_stub.h"\n')
    if os.name=="nt":
        path=stubs/"sys/types.h";path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text('#include <stddef.h>\ntypedef ptrdiff_t ssize_t;\n')
    ref=OUT/(("opus_reference_asan" if ASAN else "opus_reference")+(".dll" if os.name=="nt" else ".so"))
    ref_args=[compiler,"-std=c11","-O2","-fwrapv","-shared"]
    if ASAN:ref_args += ["-fsanitize=address","-fno-omit-frame-pointer"]
    if os.name!="nt":ref_args += ["-fPIC"]
    for inc in (ROOT/"codecs/opus/scratch",stubs,ROOT/"tests/codecs",BT/"stack/include",BT/"stack/a2dp",OPUS/"include"):
        ref_args += ["-I",str(inc)]
    ref_sources=[ROOT/"tests/codecs/opus_legacy_reference.c",ROOT/"tests/codecs/opus_legacy_stub.h",
                 BT/"stack/a2dp/a2dp_vendor_opus_decoder.c",BT/"stack/include/stack/a2dp_vendor_opus_constants.h"]
    ref_args += [str(ref_sources[0]),str(ROOT/"codecs/opus/scratch/dbt_opus_scratch.c")]
    ref_args += ([str(guarded/"opus.lib")] if os.name=="nt" else [str(guarded/"libopus.a"),"-lm"])
    run(ref_args+["-o",str(ref)],env=env,cwd=OUT)
    compiled=json.loads((BUILD/"compile_commands.json").read_text())
    for entry in compiled:
        path=Path(entry["file"])
        if path.suffix==".c" and path.is_file(): sources.append(path)
    sources += list((ROOT/"codecs/opus/scratch").glob("*.[ch]"))
    sources += [ROOT/"codecs/common/dbt_codec.h",ROOT/"codecs/opus/dbt_opus.h",ROOT/"codecs/sbc/dbt_sbc_codec.h",
                OPUS/"include/opus.h",OPUS/"include/opus_types.h",OPUS/"include/opus_defines.h",OPUS/"CMakeLists.txt"]
    return opus_dll,dll,ref,sources+ref_sources


def cie(variant,channels,samples):
    if variant==0:
        return bytes([0,255,0xe0,0,0,0,1,0,0x80|channels|({480:8,960:16}[samples])])
    return bytes([0,255,0xf1,5,0,0,5,0x10,channels,int(channels==2),3 if channels==2 else 1,0,0,0,
                  {120:1,240:2,480:4,960:8,1920:16}[samples],0,0,0,0,0,0,0,0,0,0,0])


def main():
    opus_path,dll_path,ref_path,sources=build()
    if os.name=="nt": os.add_dll_directory(str(BUILD))
    opus=ctypes.CDLL(str(opus_path)); native=ctypes.CDLL(str(dll_path));legacy=ctypes.CDLL(str(ref_path))
    u8=ctypes.c_uint8; u=ctypes.c_uint; ptr=ctypes.POINTER(u8); i16=ctypes.c_int16
    opus.opus_encoder_create.argtypes=[ctypes.c_int,ctypes.c_int,ctypes.c_int,ctypes.POINTER(ctypes.c_int)]; opus.opus_encoder_create.restype=ctypes.c_void_p
    opus.opus_encoder_destroy.argtypes=[ctypes.c_void_p]
    opus.opus_encode.argtypes=[ctypes.c_void_p,ctypes.POINTER(i16),ctypes.c_int,ptr,ctypes.c_int]
    opus.opus_decoder_create.argtypes=[ctypes.c_int,ctypes.c_int,ctypes.POINTER(ctypes.c_int)]; opus.opus_decoder_create.restype=ctypes.c_void_p
    opus.opus_decoder_destroy.argtypes=[ctypes.c_void_p]
    opus.opus_decode.argtypes=[ctypes.c_void_p,ptr,ctypes.c_int,ctypes.POINTER(i16),ctypes.c_int,ctypes.c_int]
    native.probe_open.argtypes=[ptr,u]; native.probe_configure.argtypes=[ptr,u,ctypes.POINTER(u),ctypes.POINTER(u),ctypes.POINTER(u)]
    native.probe_validate.argtypes=[ptr,u,ptr,u]
    native.probe_media.argtypes=[ptr,u,u,u,u,ptr,ctypes.POINTER(u)]
    native.worker_control.argtypes=[u,ptr,u];native.worker_packet.argtypes=[ptr,u,u,u,ptr,ctypes.POINTER(u)]
    legacy.reference_open.argtypes=[ptr,u];legacy.reference_packet.argtypes=[ptr,u,ptr,ctypes.POINTER(u)]
    def buf(data): return (u8*len(data)).from_buffer_copy(data)
    def open_native(config): return native.probe_open(buf(config),len(config))
    def media(data,seq,timestamp=1,now=0):
        output=(u8*8192)();n=u();r=native.probe_media(buf(data),len(data),seq,timestamp,now,output,ctypes.byref(n));return r,bytes(output[:n.value])
    def control(op,config=None):
        return native.worker_control(op,buf(config) if config else None,len(config) if config else 0)
    def worker_packet(data,seq):
        output=(u8*8192)();n=u();r=native.worker_packet(buf(data),len(data),seq,1,output,ctypes.byref(n));return r,bytes(output[:n.value])
    vectors=[]; packets_480=None
    for variant in (0,1):
        for channels in (1,2):
            for samples in ((480,960) if variant==0 else (120,240,480,960,1920)):
                config=cie(variant,channels,samples); rate=u();ch=u();s=u()
                assert native.probe_configure(buf(config),len(config),ctypes.byref(rate),ctypes.byref(ch),ctypes.byref(s))==0
                assert (rate.value,ch.value,s.value)==(48000,channels,samples)
                error=ctypes.c_int();enc=opus.opus_encoder_create(48000,channels,2049,ctypes.byref(error)); assert enc and error.value==0
                packets=[]
                for frame in range(3):
                    pcm=(i16*(samples*channels))(*[round(8000*math.sin(2*math.pi*(997+channel*700)*(frame*samples+i)/48000)) for i in range(samples) for channel in range(channels)])
                    output=(u8*4000)();n=opus.opus_encode(enc,pcm,samples,output,4000);assert n>2;packets.append(bytes(output[:n]))
                opus.opus_encoder_destroy(enc)
                if channels==2 and samples==480: packets_480=packets
                dec=opus.opus_decoder_create(48000,channels,ctypes.byref(error));assert dec and error.value==0
                expected=bytearray()
                for packet in packets:
                    output=(i16*(samples*channels))();assert opus.opus_decode(dec,buf(packet),len(packet),output,samples,0)==samples;expected.extend(bytes(output))
                opus.opus_decoder_destroy(dec)
                assert legacy.reference_open(buf(config),len(config))==0
                legacy_pcm=bytearray()
                for i,packet in enumerate(packets):
                    chunks=[bytes([1])+packet] if i!=1 else [bytes([0xc2])+packet[:len(packet)//2],bytes([0xa1])+packet[len(packet)//2:]]
                    for chunk in chunks:
                        output=(u8*8192)();n=u()
                        assert legacy.reference_packet(buf(chunk),len(chunk),output,ctypes.byref(n))==0
                        legacy_pcm.extend(bytes(output[:n.value]))
                legacy.reference_close();assert legacy_pcm==expected
                assert open_native(config)==0;active=native.probe_live();actual=bytearray();seq=65534
                for i,packet in enumerate(packets):
                    if i==1:
                        cut=len(packet)//2
                        assert media(bytes([0xc2])+packet[:cut],seq)==(0,b'');seq=(seq+1)&65535
                        r,pcm=media(bytes([0xa1])+packet[cut:],seq)
                    else:r,pcm=media(bytes([1])+packet,seq)
                    seq=(seq+1)&65535;assert r==0;actual.extend(pcm)
                assert actual==expected
                native.probe_close();assert native.probe_live()==0
                assert native.worker_init()==0 and control(0,config)==0 and control(3)==0
                actual=bytearray()
                for i,packet in enumerate(packets):
                    r,pcm=worker_packet(bytes([1])+packet,i);assert r==0;actual.extend(pcm)
                assert actual==expected and control(5)==0 and native.probe_live()==0
                vectors.append(dict(variant=variant,channels=channels,samples=samples,pcm_sha256=hashlib.sha256(actual).hexdigest(),
                                    encoded_sha256=hashlib.sha256(b''.join(packets)).hexdigest(),active_bytes=active))
    config=cie(1,2,960); config480=cie(1,2,480)
    for bad in [config[:-1],config[:8]+bytes([3])+config[9:],config[:9]+bytes([0])+config[10:],
                config[:14]+bytes([3])+config[15:],config[:17]+bytes([1])+config[18:]]:
        rate=u(77);channels=u(78);samples=u(79)
        assert native.probe_configure(buf(bad),len(bad),ctypes.byref(rate),ctypes.byref(channels),ctypes.byref(samples))!=0
        assert (rate.value,channels.value,samples.value)==(77,78,79)
    caps=bytes([1,0,7,len(config)])+config
    assert native.probe_validate(buf(caps),len(caps),buf(config),len(config))==0
    capped=bytearray(config);capped[14]=4
    caps2=bytes([1,0,7,len(capped)])+capped
    assert native.probe_validate(buf(caps2),len(caps2),buf(config),len(config))!=0
    native.probe_fail_in(1);assert open_native(config)!=0 and native.probe_live()==0
    assert open_native(config)==0;active=native.probe_live()
    wrong_result,wrong_pcm=media(bytes([1])+packets_480[0],1)
    assert wrong_result!=0 and not wrong_pcm # wrong negotiated duration, no PLC publication
    native.probe_close();assert native.probe_live()==0
    assert open_native(config480)==0;active=native.probe_live();packet=packets_480[0];cut=len(packet)//2
    assert media(bytes([0xc2])+packet[:cut],1,101,1)==(0,b'')
    native.probe_expire(101);assert native.probe_live()==active
    assert media(bytes([0xa1])+packet[cut:],2,101,102)[0]!=0
    assert media(bytes([0xc2])+packet[:cut],3,102,103)==(0,b'')
    assert media(bytes([0xa1])+packet[cut:],5,102,104)[0]!=0
    native.probe_fail_in(1);assert media(bytes([0xc2])+packet[:cut],6,103,105)[0]!=0
    assert media(bytes([0xa1])+packet[cut:],7,103,106)[0]!=0
    assert media(bytes([0x11])+packet,8,104,107)[0]!=0
    assert media(bytes([0xc2])+packet[:cut],9,105,108)==(0,b'')
    native.probe_fail_in(1)
    assert media(bytes([0xa1])+bytes(500),10,105,109)[0]!=0 and native.probe_live()==active
    assert media(bytes([0xc4])+bytes(2048),11,106,110)==(0,b'')
    assert media(bytes([0x83])+bytes(2048),12,106,111)==(0,b'')
    native.probe_peak_reset()
    assert media(bytes([0x82])+bytes(2048),13,106,112)==(0,b'')
    fragment_peak=native.probe_peak();assert fragment_peak==active+4096+6144+16
    assert media(bytes([0xa1])+bytes(1),14,106,113)[0]!=0 and native.probe_live()==active
    native.probe_close();assert native.probe_live()==0
    sbc=bytes([0,0,0x11,0x15,2,53])
    assert native.worker_init()==0 and control(0,sbc)==0 and control(3)==0 and control(4)==0
    sbc_live=native.probe_live();native.probe_peak_reset();native.probe_fail_in(1)
    assert control(1,config)==1 and native.probe_live()==sbc_live
    native.probe_fail_in(2);assert control(1,config)==1 and native.probe_live()==sbc_live
    native.probe_peak_reset();assert control(1,config)==0
    opus_live=native.probe_live();assert native.probe_peak()==max(opus_live,sbc_live)
    native.probe_peak_reset();assert control(1,sbc)==0
    assert native.probe_peak()==max(opus_live,sbc_live)
    assert control(5)==0 and native.probe_live()==0
    report=dict(passed=True,address_sanitizer=ASAN,scope="offline SDK fixed-point libopus and native worker; no target or RF evidence",vectors=vectors,
                byte_exact_existing_sdk_wrapper=True,byte_exact_standalone_decoder=True,
                malformed_cie_and_capability=True,fragment_expiry_gap_and_oom=True,fragment_limit=6144,
                fragment_growth_peak_bytes=fragment_peak,sbc_live_bytes=sbc_live,opus_worker_live_bytes=opus_live,
                switch_peak_bytes=max(opus_live,sbc_live),reconfigure_decoder_and_queue_oom_rollback=True,
                source_sha256={str(p.relative_to(ROOT.parent)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources})
    (OUT/("opus_asan_results.json" if ASAN else "opus_real_results.json")).write_text(json.dumps(report,indent=2)+"\n")
    print(json.dumps({k:v for k,v in report.items() if k!='source_sha256'},indent=2))


if __name__=="__main__":main()
