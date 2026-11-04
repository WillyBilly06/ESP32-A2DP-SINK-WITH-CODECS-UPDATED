"""Offline real AAC-LC transport/PCM test against the existing SDK wrapper."""
import ctypes
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
SDK = ROOT.parent / "esp-idf/components/bt/host/bluedroid/external/libaac-lc"
CORE = SDK / "libhelix_aac"
SBC = ROOT.parent / "esp-idf/components/bt/host/bluedroid/external/sbc/decoder"
OUT = ROOT / "_build/native_codecs"
SOURCES = "aacdec aactabs bitstream buffers dct4 decelmnt dequant fft filefmt huffman hufftabs imdct noiseless pns stproc tns trigtabs".split()
SBC_SOURCES = "alloc bitalloc-sbc bitalloc bitstream-decode decoder-oina decoder-private decoder-sbc dequant framing-sbc framing oi_codec_version synthesis-8-generated synthesis-dct8 synthesis-sbc".split()


def run(args, **kw):
    subprocess.run(args, check=True, **kw)


def build():
    OUT.mkdir(parents=True, exist_ok=True)
    run([sys.executable, str(ROOT / "codecs/aac/prepare_port.py"), "--source", str(CORE / "aacdec.c"), "--output", str(OUT)])
    run([sys.executable, str(ROOT / "codecs/sbc/prepare_port.py"), "--source", str(SBC / "srce"), "--output", str(OUT / "sbc_port")])
    (OUT / "esp_log.h").write_text("#define ESP_LOGI(...) ((void)0)\n#define ESP_LOGW(...) ((void)0)\n#define ESP_LOGE(...) ((void)0)\n#define ESP_LOGD(...) ((void)0)\n")
    (OUT / "esp_heap_caps.h").write_text("#include <stddef.h>\n#include <stdint.h>\n#define MALLOC_CAP_INTERNAL 1\n#define MALLOC_CAP_8BIT 2\nvoid *heap_caps_aligned_alloc(size_t,size_t,uint32_t);\nvoid heap_caps_free(void *);\n")
    compiler = shutil.which("clang") or r"C:\Program Files\LLVM\bin\clang.exe"
    includes = [ROOT / "codecs/sbc/port", SBC / "include", ROOT / "include", ROOT / "codecs/common", ROOT / "codecs/aac", ROOT / "codecs/sbc", CORE, SDK, OUT]
    sources = [OUT / "aacdec_guarded.c"] + [CORE / (name + ".c") for name in SOURCES if name != "aacdec"]
    sources += [ROOT / "codecs/aac/dbt_aac_decoder.c", ROOT / "codecs/aac/dbt_aac_transport.c",
                ROOT / "codecs/common/dbt_codec.c", ROOT / "src/dbt_avdtp_caps.c",
                ROOT / "tests/codecs/aac_real_probe.c", ROOT / "codecs/sbc/dbt_sbc_codec.c",
                ROOT / "codecs/sbc/dbt_sbc_decoder.c", ROOT / "src/dbt_sbc_media.c",
                ROOT / "tests/codecs/sbc_reference_probe.c"]
    sources += [(OUT / "sbc_port" if name in {"synthesis-8-generated", "synthesis-dct8", "synthesis-sbc"} else SBC / "srce") / (name + ".c") for name in SBC_SOURCES]
    dll = OUT / ("aac_real.dll" if os.name == "nt" else "aac_real.so")
    args = [compiler, "-std=c11", "-O2", "-fwrapv", "-funsigned-char", "-shared", "-DUSE_DEFAULT_STDLIB",
            "-include", str(ROOT / "codecs/aac/host_target_math.h")]
    if os.name != "nt": args += ["-fPIC", "-D_POSIX_C_SOURCE=200112L"]
    for inc in includes: args += ["-I", str(inc)]
    args += [str(p) for p in sources] + ["-o", str(dll)]
    env = os.environ.copy()
    if os.name == "nt":
        vc = Path(r"C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat")
        data = subprocess.check_output(f'call "{vc}" >nul && set', shell=True, text=True)
        env.update(dict(line.split('=', 1) for line in data.splitlines() if '=' in line and not line.startswith('=')))
    run(args, env=env, cwd=OUT)
    reference = OUT / ("aac_reference.dll" if os.name == "nt" else "aac_reference.so")
    ref_sources = [CORE / (name + ".c") for name in SOURCES] + [ROOT / "tests/codecs/aac_reference_probe.c"]
    ref_args = [compiler, "-std=c11", "-O2", "-fwrapv", "-funsigned-char", "-shared", "-DUSE_DEFAULT_STDLIB",
                "-include", str(ROOT / "codecs/aac/host_target_math.h")]
    if os.name != "nt": ref_args += ["-fPIC", "-D_POSIX_C_SOURCE=200112L", "-Wl,-Bsymbolic"]
    for inc in includes: ref_args += ["-I", str(inc)]
    run(ref_args + [str(p) for p in ref_sources] + ["-o", str(reference)], env=env, cwd=OUT)
    return dll, reference, sources + ref_sources + [SDK / "aac_decoder_helix.c",ROOT / "codecs/aac/host_target_math.h"]


def adts(path):
    data = path.read_bytes(); pos = 0
    while pos < len(data):
        h = data[pos:pos+7]
        assert len(h) == 7 and h[0] == 255 and h[1] & 1
        n = ((h[3]&3)<<11) | (h[4]<<3) | (h[5]>>5)
        assert 7 < n <= len(data)-pos
        yield data[pos+7:pos+n]; pos += n
    assert pos == len(data)


def latm(raw, rate, channels, same=False):
    bits = []
    def put(v, n): bits.extend((v >> i) & 1 for i in reversed(range(n)))
    put(same, 1)
    if not same:
        for value, size in [(0,1),(1,1),(0,6),(0,4),(0,3),(2,5),(3 if rate==48000 else 4,4),
                            (channels,4),(0,3),(0,3),(255,8),(0,1),(0,1)]: put(value,size)
    n = len(raw)
    while n >= 255: put(255,8); n -= 255
    put(n,8)
    for b in raw: put(b,8)
    while len(bits)%8: bits.append(0)
    return bytes(sum(bits[i+j] << (7-j) for j in range(8)) for i in range(0,len(bits),8))


def main():
    dll, reference_dll, sources = build(); lib = ctypes.CDLL(str(dll)); reference_lib = ctypes.CDLL(str(reference_dll))
    ffmpeg=shutil.which("ffmpeg")
    if not ffmpeg: raise RuntimeError("FFmpeg is required for local encoded fixtures")
    vectors_dir=OUT / "vectors"; vectors_dir.mkdir(exist_ok=True)
    for rate in (44100,48000):
        for channels in (1,2):
            path=vectors_dir / f"tone_{rate}_{channels}.aac"
            run([ffmpeg,"-hide_banner","-loglevel","error","-y","-f","lavfi","-i",
                 f"sine=frequency=997:sample_rate={rate}:duration=0.2","-ac",str(channels),
                 "-c:a","aac","-profile:a","aac_low","-b:a","128k","-f","adts",str(path)])
    sbc_path=vectors_dir / "tone_48000_2.sbc"
    run([ffmpeg,"-v","error","-y","-f","lavfi","-i","sine=frequency=997:sample_rate=48000:duration=0.15",
         "-ac","2","-c:a","sbc","-b:a","328k","-f","sbc",str(sbc_path)])
    u8 = ctypes.c_uint8; ptr = ctypes.POINTER(u8); u = ctypes.c_uint
    lib.probe_media.argtypes = [ptr,u,u,u,u,u,ptr,ctypes.POINTER(u)]
    lib.probe_sbc_open.argtypes = [ptr]
    lib.worker_control.argtypes = [u,ptr,u]
    lib.worker_packet.argtypes = [ptr,u,u,u,ptr,ctypes.POINTER(u)]
    lib.sbc_reference.argtypes = [ptr,u,ctypes.POINTER(ctypes.c_int16),ctypes.POINTER(u)]
    reference_lib.baseline_decode.argtypes = [ptr,ctypes.c_int,ctypes.POINTER(ctypes.c_int16),ctypes.POINTER(ctypes.c_int)]
    def media(raw, seq, marker=1, now=0, timestamp=123):
        buf = (u8*len(raw)).from_buffer_copy(raw); output = (u8*8192)(); n = u()
        result = lib.probe_media(buf,len(raw),timestamp,seq,marker,now,output,ctypes.byref(n))
        return result, bytes(output[:n.value])
    vectors = []
    for rate in (44100,48000):
        for channels in (1,2):
            frames = list(adts(vectors_dir / f"tone_{rate}_{channels}.aac"))
            assert lib.probe_open(rate,channels)==0 and reference_lib.baseline_open(rate,channels)==0
            native = bytearray(); reference = bytearray()
            for i, frame in enumerate(frames):
                wrapped = latm(frame,rate,channels,i>0)
                result, output = media(wrapped,i)
                assert result==0 and len(output)==2048*channels, (rate,channels,i,result,len(output))
                raw = (u8*len(frame)).from_buffer_copy(frame); pcm = (ctypes.c_int16*2048)(); n = ctypes.c_int()
                assert reference_lib.baseline_decode(raw,len(frame),pcm,ctypes.byref(n))==0 and n.value==1024
                native.extend(output); reference.extend(bytes(pcm)[:2048*channels])
            assert native==reference
            live=lib.probe_live(); lib.probe_close(); reference_lib.baseline_close(); assert lib.probe_live()==0
            assert lib.probe_open(rate,channels)==0
            fragmented=bytearray()
            for i, frame in enumerate(frames):
                wrapped=latm(frame,rate,channels,i>0); split=len(wrapped)//2
                assert media(wrapped[:split],2*i,0,1,100+i*1024)==(0,b'')
                result, output=media(wrapped[split:],2*i+1,1,2,100+i*1024)
                assert result==0; fragmented.extend(output)
            assert fragmented==reference
            lib.probe_close(); assert lib.probe_live()==0
            vectors.append(dict(rate=rate,channels=channels,frames=len(frames),bytes=len(native),
                                sha256=hashlib.sha256(native).hexdigest(),native_live_bytes=live))
    lib.probe_fail_next(); assert lib.probe_open(48000,2)!=0 and lib.probe_live()==0
    assert lib.probe_open(48000,1)==0
    wrong=next(adts(vectors_dir / "tone_48000_2.aac"))
    assert media(latm(wrong,48000,1),1)[0]!=0
    lib.probe_close(); assert lib.probe_live()==0
    assert lib.probe_open(48000,2)==0
    wrong=next(adts(vectors_dir / "tone_48000_1.aac"))
    assert media(latm(wrong,48000,2),1)[0]!=0
    lib.probe_close(); assert lib.probe_live()==0
    assert lib.probe_open(48000,2)==0
    first=next(adts(vectors_dir / "tone_48000_2.aac")); wrapped=latm(first,48000,2)
    assert media(wrapped[:50],1,0,1)==(0,b'') and lib.probe_live()>0
    before=lib.probe_live(); lib.probe_expire(101); assert lib.probe_live()<before
    assert media(wrapped,2,1,102)[0]!=0 # A valid-looking tail must not become a new mux.
    assert media(wrapped,3,1,103,124)[0]==0
    assert media(wrapped[:40],4,0,104,125)==(0,b'')
    assert media(wrapped,6,1,105,125)[0]!=0 # Sequence gap abandons this timestamp.
    lib.probe_fail_next()
    assert media(wrapped[:40],7,0,106,126)[0]!=0
    assert media(wrapped,8,1,107,126)[0]!=0
    lib.probe_close(); assert lib.probe_live()==0
    assert lib.probe_open(48000,2)==0
    assert media(wrapped[:100],65535,0,1,200)==(0,b'')
    assert media(wrapped[100:],0,1,2,200)[0]==0
    lib.probe_close(); assert lib.probe_live()==0
    assert lib.probe_open(48000,2)==0
    base=lib.probe_live(); lib.probe_peak_reset()
    for i in range(4): assert media(bytes(2048),i,0,i,300)==(0,b'')
    fragment_peak=lib.probe_peak()
    assert fragment_peak==base+4096+8192+16
    assert media(bytes(1),4,0,5,300)[0]!=0
    assert lib.probe_live()==base
    assert media(wrapped,5,1,6,300)[0]!=0
    lib.probe_close(); assert lib.probe_live()==0
    sbc_data=sbc_path.read_bytes()
    sbc_frames=[]; offset=0
    while offset<len(sbc_data):
        f,pool=sbc_data[offset+1],sbc_data[offset+2]; bands=8 if f&1 else 4
        blocks=4*(1+((f>>4)&3)); mode=(f>>2)&3; channels=2 if mode else 1
        bits=blocks*pool*(channels if mode<2 else 1)+(bands if mode==3 else 0)
        size=4+bands*channels//2+(bits+7)//8
        assert offset+size<=len(sbc_data); sbc_frames.append(sbc_data[offset:offset+size]); offset+=size
    cie=bytes([0,0,(0x80>>(f>>6))|(8>>mode),(0x80>>((f>>4)&3))|(4 if f&1 else 8)|(2 if f&2 else 1),2,max(frame[2] for frame in sbc_frames)])
    cie_buf=(u8*6).from_buffer_copy(cie)
    assert lib.probe_sbc_open(cie_buf)==0 and lib.sbc_reference_open(channels)==0
    sbc_live=lib.probe_live(); sbc_out=bytearray(); sbc_ref=bytearray()
    for i, frame in enumerate(sbc_frames):
        result, output=media(bytes([1])+frame,i,0,1,100+i*128)
        assert result==0
        data=(u8*len(frame)).from_buffer_copy(frame); pcm=(ctypes.c_int16*256)(); n=u()
        assert lib.sbc_reference(data,len(frame),pcm,ctypes.byref(n))==0
        sbc_out.extend(output); sbc_ref.extend(bytes(pcm)[:n.value])
    assert sbc_out==sbc_ref and len(sbc_out)>0
    lib.probe_peak_reset(); assert lib.probe_open(48000,2)==0
    aac_live=lib.probe_live(); assert lib.probe_peak()==max(sbc_live,aac_live)
    lib.probe_peak_reset(); assert lib.probe_sbc_open(cie_buf)==0
    assert lib.probe_peak()==max(sbc_live,aac_live)
    lib.probe_close(); assert lib.probe_live()==0
    aac_cie=(u8*8).from_buffer_copy(bytes([0,2,0x40,0,0x84,0,0,0]))
    def control(op, cie=None): return lib.worker_control(op,cie,len(cie) if cie is not None else 0)
    def worker_packet(data,seq,marker):
        raw=(u8*len(data)).from_buffer_copy(data); out=(u8*8192)(); n=u()
        r=lib.worker_packet(raw,len(data),seq,marker,out,ctypes.byref(n)); return r,bytes(out[:n.value])
    assert lib.worker_init()==0 and control(0,cie_buf)==0 and control(3)==0
    sbc_worker_live=lib.probe_live()
    result, pcm=worker_packet(bytes([1])+sbc_frames[0],1,0)
    assert result==0 and pcm==sbc_ref[:len(pcm)] and len(pcm)>0, (result,len(pcm),pcm[:12].hex(),sbc_ref[:12].hex())
    assert control(4)==0
    lib.probe_peak_reset();lib.probe_fail_next()
    assert control(1,aac_cie)==1 and lib.probe_live()==sbc_worker_live
    assert lib.probe_peak()==sbc_worker_live # failed candidate rolled back without workspace overlap
    lib.probe_fail_in(2)
    assert control(1,aac_cie)==1 and lib.probe_live()==sbc_worker_live
    assert control(1,aac_cie)==0 and control(3)==0
    aac_worker_live=lib.probe_live()
    first=next(adts(vectors_dir / "tone_48000_2.aac"))
    result, pcm=worker_packet(latm(first,48000,2),1,1)
    assert result==0 and len(pcm)==4096
    assert reference_lib.baseline_open(48000,2)==0
    buf=(u8*len(first)).from_buffer_copy(first); ref=(ctypes.c_int16*2048)(); n=ctypes.c_int()
    assert reference_lib.baseline_decode(buf,len(first),ref,ctypes.byref(n))==0 and pcm==bytes(ref)
    reference_lib.baseline_close()
    assert control(4)==0
    lib.probe_peak_reset(); assert control(1,cie_buf)==0
    assert lib.probe_peak()==max(sbc_worker_live,aac_worker_live)
    assert control(5)==0 and lib.probe_live()==0
    report=dict(passed=True, scope="offline Helix AAC-LC transport and existing wrapper comparison; no target or RF evidence",
                vectors=vectors, fragmented_bit_exact=True, channel_mismatch_rejected=True,
                allocation_failure=True, fragment_expiry=True, fragment_gap_and_oom_discard=True,
                fragment_growth_peak_bytes=fragment_peak, sequence_wrap=True,
                sbc=dict(frames=len(sbc_frames),pcm_bytes=len(sbc_out),sha256=hashlib.sha256(sbc_out).hexdigest(),
                         byte_exact_reference=True,native_live_bytes=sbc_live,codec_switch_peak=max(sbc_live,aac_live)),
                worker=dict(real_sbc_and_aac_pcm=True,reconfigure_oom_rollback=True,
                            sbc_live_bytes=sbc_worker_live,aac_live_bytes=aac_worker_live,
                            switch_peak_bytes=max(sbc_worker_live,aac_worker_live)),
                fixture_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(vectors_dir.iterdir()) if p.is_file()},
                source_sha256={str(p.relative_to(ROOT.parent)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources})
    (OUT / "aac_real_results.json").write_text(json.dumps(report,indent=2)+"\n")
    print(json.dumps(report,indent=2))


if __name__ == "__main__": main()
