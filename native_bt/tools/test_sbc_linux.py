"""Actual SBC decode/PCM/allocation tests; transfers codec files, never an SDK."""
import argparse
import getpass
import hashlib
import json
from pathlib import Path
import shlex
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
CODEC = ROOT.parent / "esp-idf/components/bt/host/bluedroid/external/sbc/decoder"
SOURCES = ["alloc.c", "bitalloc-sbc.c", "bitalloc.c", "bitstream-decode.c", "decoder-oina.c",
           "decoder-private.c", "decoder-sbc.c", "dequant.c", "framing-sbc.c", "framing.c",
           "oi_codec_version.c", "synthesis-8-generated.c", "synthesis-dct8.c", "synthesis-sbc.c"]
DRIVER = r'''
import ctypes,hashlib,json,pathlib,subprocess
root=pathlib.Path.cwd()
def run(args):
 p=subprocess.run(args,capture_output=True,text=True,timeout=60)
 if p.returncode:raise RuntimeError(' '.join(args)+' exit='+str(p.returncode)+'\n'+p.stdout[-1000:]+p.stderr[-5000:])
 return p.stdout
sources=['decoder/'+name for name in json.loads((root/'sources.json').read_text())]
sources+=['owned/dbt_sbc_decoder.c','owned/dbt_sbc_worker.c','core/dbt_sbc_media.c','core/dbt_avdtp_caps.c','sbc_decoder_probe.c']
flags=['gcc','-std=c11','-O2','-fwrapv','-Iport','-Iinclude','-Iowned','-Icore',*sources]
run(flags+['-shared','-fPIC','-Wl,-Bsymbolic','-o','original.so'])
run(['python3','prepare_port.py','--source','decoder','--output','ported'])
ported=['synthesis-8-generated.c','synthesis-dct8.c','synthesis-sbc.c']
sources=[('ported/'+pathlib.Path(p).name) if pathlib.Path(p).name in ported else p for p in sources]
flags=['gcc','-std=c11','-O2','-fwrapv','-Iport','-Iinclude','-Iowned','-Icore',*sources]
run(flags+['-shared','-fPIC','-o','probe.so'])
thread_flags=['-pthread','-D_POSIX_C_SOURCE=200809L','-DSBC_THREAD_TEST','-DSBC_SANITIZER_MAIN']
run(flags+['-g','-O1','-fsanitize=address,undefined','-fno-sanitize-recover=all',
 '-fno-omit-frame-pointer',*thread_flags,'-o','sanitized'])
run(flags+['-g','-O1','-fsanitize=thread','-fno-sanitize-recover=all',
 '-fno-omit-frame-pointer',*thread_flags,'-o','threaded'])
lib=ctypes.CDLL(str(root/'probe.so'))
original=ctypes.CDLL(str(root/'original.so'))
bytep=ctypes.POINTER(ctypes.c_uint8);pcmp=ctypes.POINTER(ctypes.c_int16);uintp=ctypes.POINTER(ctypes.c_uint)
lib.probe_open.argtypes=[bytep];lib.probe_decode.argtypes=[bytep,ctypes.c_uint,pcmp,uintp]
lib.probe_reference.argtypes=lib.probe_decode.argtypes
original.probe_reference.argtypes=lib.probe_decode.argtypes
lib.probe_packet.argtypes=[bytep,ctypes.c_uint,ctypes.c_uint,ctypes.c_uint,pcmp,uintp]
lib.probe_worker.argtypes=[bytep,bytep,ctypes.c_uint,pcmp,uintp]
for name in ['probe_live','probe_peak','probe_workspace']:getattr(lib,name).restype=ctypes.c_size_t
def buffer(raw):return (ctypes.c_uint8*len(raw)).from_buffer_copy(raw)
def split(raw):
 offset=0
 while offset<len(raw):
  f,pool=raw[offset+1],raw[offset+2];bands=8 if f&1 else 4;blocks=4*(1+((f>>4)&3));mode=(f>>2)&3;channels=2 if mode else 1
  bits=blocks*pool*(channels if mode<2 else 1)+(bands if mode==3 else 0)
  size=4+bands*channels//2+(bits+7)//8
  assert offset+size<=len(raw)
  yield raw[offset:offset+size]
  offset+=size
 assert offset==len(raw)
vectors=[]
for path in sorted(root.glob('tone_*.sbc')):
 raw=path.read_bytes();frames=list(split(raw));f=frames[0][1];mode=(f>>2)&3;channels=2 if mode else 1
 cie=bytes([0,0,(0x80>>(f>>6))|(8>>mode),(0x80>>((f>>4)&3))|(4 if f&1 else 8)|(2 if f&2 else 1),2,max(p[2] for p in frames)])
 run(['./sanitized',str(path)])
 run(['./threaded',str(path)])
 assert lib.probe_open(buffer(cie))==0 and original.probe_reference_open(channels)==0
 workspace=lib.probe_workspace();actual=bytearray();reference=bytearray()
 for frame in frames:
  a=(ctypes.c_int16*4096)();b=(ctypes.c_int16*4096)();na=ctypes.c_uint();nb=ctypes.c_uint()
  assert lib.probe_decode(buffer(frame),len(frame),a,ctypes.byref(na))==0
  assert original.probe_reference(buffer(frame),len(frame),b,ctypes.byref(nb))==0
  assert na.value==nb.value
  actual.extend(bytes(a)[:na.value]);reference.extend(bytes(b)[:nb.value])
 assert actual==reference
 assert lib.probe_open(buffer(cie))==0
 fragmented=bytearray();seq=65530
 for frame in frames:
  chunk=(len(frame)+2)//3
  for part in range(3):
   payload=bytes([0xc3 if part==0 else 0x82 if part==1 else 0xa1])+frame[part*chunk:(part+1)*chunk]
   a=(ctypes.c_int16*4096)();n=ctypes.c_uint()
   assert lib.probe_packet(buffer(payload),len(payload),seq,1000,a,ctypes.byref(n))==0
   seq=(seq+1)&65535;fragmented.extend(bytes(a)[:n.value])
 assert fragmented==reference
 lib.probe_close();assert lib.probe_live()==0
 a=(ctypes.c_int16*4096)();n=ctypes.c_uint()
 assert lib.probe_worker(buffer(cie),buffer(frames[0]),len(frames[0]),a,ctypes.byref(n))==0
 assert lib.probe_live()==0 and n.value>0
 lib.probe_fail_next();assert lib.probe_open(buffer(cie))==3 and lib.probe_live()==0
 assert lib.probe_open(buffer(cie))==0
 lib.probe_peak_reset();assert lib.probe_open(buffer(cie))==0 and lib.probe_peak()==workspace
 lib.probe_close();assert lib.probe_live()==0
 vectors.append({'fixture':path.name,'frames':len(frames),'channels':channels,'pcm_bytes':len(actual),
  'pcm_sha256':hashlib.sha256(actual).hexdigest(),'bit_exact_reference':True,'fragment_bit_exact':True,
  'workspace_native_bytes':workspace})
report={'passed':True,'scope':'standalone SDK SBC codec + owned wrapper/media; no ESP32 timing/RF/I2S proof',
 'vectors':vectors,'address_undefined_sanitizers':True,'allocation_failure_retry':True,'reopen_without_two_workspaces':True,
 'worker_actual_pcm':True,'worker_start_stop_epoch':True,'worker_drop_releases_all':True,
 'worker_new_binding_after_close_abort_and_oom':True,
 'worker_two_threads_packets_per_vector':500,'thread_sanitizer':True}
(root/'results.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
'''


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", required=True)
    parser.add_argument("--user", required=True)
    args = parser.parse_args()
    destination = ROOT / "_build/codecs"
    ffmpeg = shutil.which("ffmpeg")
    if not ffmpeg:
        raise RuntimeError("FFmpeg is required to generate the actual encoded SBC fixtures")
    fixtures = []
    for rate in [44100, 48000]:
        for channels in [1, 2]:
            path = destination / f"tone_{rate}_{channels}.sbc"
            subprocess.run([ffmpeg, "-v", "error", "-f", "lavfi", "-i", f"sine=frequency=997:sample_rate={rate}:duration=0.15",
                "-ac", str(channels), "-c:a", "sbc", "-b:a", "192k" if channels == 1 else "328k",
                "-f", "sbc", "-y", str(path)], check=True, timeout=20)
            assert path.stat().st_size < 65536
            fixtures.append(path)
    files = {"decoder/"+name: CODEC/"srce"/name for name in SOURCES}
    for path in (CODEC/"include").glob("*.h"):
        files["include/"+path.name] = path
    files["decoder/readsamplesjoint.inc"] = CODEC/"srce/readsamplesjoint.inc"
    for path in (ROOT/"codecs/sbc").glob("dbt_sbc_*.*"):
        files["owned/"+path.name] = path
    for path in (ROOT/"codecs/sbc/port").rglob("*.h"):
        files["port/"+path.relative_to(ROOT/"codecs/sbc/port").as_posix()] = path
    for name in ["dbt_sbc_media.h", "dbt_avdtp.h", "dbt_avdtp_caps.h", "dbt_avdtp_sink.h", "dbt_hci.h", "dbt_queue.h"]:
        files["core/"+name] = ROOT/"include"/name
    files["core/dbt_sbc_media.c"] = ROOT/"src/dbt_sbc_media.c"
    files["core/dbt_avdtp_caps.c"] = ROOT/"src/dbt_avdtp_caps.c"
    files["sbc_decoder_probe.c"] = ROOT/"tests/sbc_decoder_probe.c"
    files["prepare_port.py"] = ROOT/"codecs/sbc/prepare_port.py"
    files.update({path.name: path for path in fixtures})
    sys.path.insert(0, str(ROOT/"_build/python_deps"))
    import paramiko
    password = getpass.getpass("Linux SSH password (session only): ")
    client = paramiko.SSHClient()
    client.load_host_keys(str(ROOT/"_build/linux_known_hosts"))
    client.set_missing_host_key_policy(paramiko.RejectPolicy())
    try:
        client.connect(args.host, username=args.user, password=password, look_for_keys=False, allow_agent=False, timeout=8)
        password = None
        remote = "/home/"+args.user+"/.cache/native_bt/sbc_tests"
        folders = [remote]+[remote+"/"+name for name in ["decoder", "include", "owned", "port", "port/common", "core"]]
        _, stdout, stderr = client.exec_command("mkdir -p "+" ".join(shlex.quote(path) for path in folders))
        if stdout.channel.recv_exit_status():
            raise RuntimeError(stderr.read().decode())
        sftp = client.open_sftp()
        hashes = {}
        for name, path in files.items():
            data = path.read_bytes()
            hashes[str(path.relative_to(ROOT.parent)).replace("\\", "/")] = hashlib.sha256(data).hexdigest()
            with sftp.file(remote+"/"+name, "wb") as handle:
                handle.write(data)
        with sftp.file(remote+"/sources.json", "w") as handle:
            handle.write(json.dumps(SOURCES))
        stdin, stdout, stderr = client.exec_command("cd "+shlex.quote(remote)+" && python3 -", timeout=180)
        stdin.write(DRIVER)
        stdin.flush()
        stdin.channel.shutdown_write()
        output, errors = stdout.read(100000).decode(), stderr.read(16000).decode()
        if stdout.channel.recv_exit_status():
            raise RuntimeError(errors or output)
        with sftp.file(remote+"/results.json") as handle:
            report = json.loads(handle.read())
        report["source_sha256"] = hashes
        (destination/"sbc_results.json").write_text(json.dumps(report, indent=2)+"\n")
        print(json.dumps({key: value for key, value in report.items() if key!="source_sha256"}, indent=2))
        sftp.close()
    finally:
        password = None
        client.close()


if __name__ == "__main__":
    main()
