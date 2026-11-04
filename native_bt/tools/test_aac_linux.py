"""AAC allocation/PCM equivalence tests; upload codec files only, not an SDK."""
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
LIBRARY = ROOT.parent / "esp-idf/components/bt/host/bluedroid/external/libaac-lc"
SOURCES = ["aacdec.c", "aactabs.c", "bitstream.c", "buffers.c", "dct4.c", "decelmnt.c",
           "dequant.c", "fft.c", "filefmt.c", "huffman.c", "hufftabs.c", "imdct.c",
           "noiseless.c", "pns.c", "sbr.c", "stproc.c", "tns.c", "trigtabs.c"]
DRIVER = r'''
import ctypes, hashlib, json, pathlib, subprocess
root=pathlib.Path.cwd()
sources=json.loads((root/'sources.json').read_text())
def run(command,folder=root):
 p=subprocess.run(command,cwd=folder,capture_output=True,text=True,timeout=50)
 if p.returncode: raise RuntimeError(' '.join(command)+'\n'+p.stderr[-8000:])
 return p
fixtures=[]
for rate in [44100,48000]:
 for channels in [1,2]:
  path=root/('tone_%d_%d.aac'%(rate,channels))
  assert path.is_file() and path.stat().st_size<1048576
  fixtures.append((rate,channels,path))
libraries={}
for label in ['baseline','candidate']:
 folder=root/label
 c=[str(root/'core'/name) for name in sources]
 flags=['gcc','-std=c11','-O2','-fwrapv','-I.', '-I'+str(root/'core'),'-I'+str(root/'headers'),
        '-DAAC_CANDIDATE='+str(int(label=='candidate')),'aac_workspace_probe.c',*c,'-lm']
 run(flags+['-shared','-fPIC','-o','probe.so'],folder)
 run(flags+['-g','-O1','-fsanitize=address','-fno-omit-frame-pointer','-DAAC_SANITIZER_MAIN','-o','sanitized'],folder)
 for rate,channels,path in fixtures: run(['./sanitized',str(path),str(rate),str(channels)],folder)
 lib=ctypes.CDLL(str(folder/'probe.so'))
 lib.probe_decode.argtypes=[ctypes.POINTER(ctypes.c_uint8),ctypes.c_int,ctypes.POINTER(ctypes.c_int16),ctypes.POINTER(ctypes.c_int)]
 for name in ['probe_live','probe_peak','probe_resident_workspace']: getattr(lib,name).restype=ctypes.c_size_t
 libraries[label]=lib
def frames(path):
 data=path.read_bytes(); offset=0
 while offset<len(data):
  h=data[offset:offset+7]
  assert len(h)==7 and h[0]==255 and h[1]&1
  size=((h[3]&3)<<11)|(h[4]<<3)|(h[5]>>5)
  yield data[offset+7:offset+size]
  offset+=size
 assert offset==len(data)
results=[]
for rate,channels,path in fixtures:
 outputs={}
 for label,lib in libraries.items():
  assert lib.probe_open(rate,channels)==0
  out=bytearray()
  for frame in frames(path):
   raw=(ctypes.c_uint8*len(frame)).from_buffer_copy(frame)
   pcm=(ctypes.c_int16*2048)(); samples=ctypes.c_int()
   assert lib.probe_decode(raw,len(frame),pcm,ctypes.byref(samples))==0
   assert samples.value==1024
   out.extend(bytes(pcm)[:samples.value*channels*2])
  outputs[label]=bytes(out)
  lib.probe_close(); assert lib.probe_live()==0
 assert outputs['baseline']==outputs['candidate']
 results.append({'rate':rate,'channels':channels,'pcm_bytes':len(outputs['candidate']),
                 'pcm_sha256':hashlib.sha256(outputs['candidate']).hexdigest(),'bit_exact':True})
candidate=libraries['candidate']; baseline=libraries['baseline']
assert candidate.probe_resident_workspace()==0
candidate.probe_fail_next()
assert candidate.probe_open(48000,2)==-4 and candidate.probe_live()==0
assert candidate.probe_open(48000,2)==0
workspace=candidate.probe_live()
assert candidate.probe_open(44100,1)==0 and candidate.probe_peak()==workspace
candidate.probe_close(); assert candidate.probe_live()==0
report={'passed':True,'scope':'native AAC-LC raw-frame decode and allocation; not ESP32 heap/latency or A2DP interoperability',
        'vectors':results,'address_sanitizer':True,'workspace_native_bytes':workspace,
        'inactive_workspace_before':baseline.probe_resident_workspace(),'inactive_workspace_after':0,
        'allocation_failure_retry':True,'reinit_without_two_workspaces':True}
(root/'results.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
'''


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", required=True)
    parser.add_argument("--user", required=True)
    args = parser.parse_args()
    encoder = shutil.which("ffmpeg")
    if not encoder: raise RuntimeError("A local FFmpeg AAC encoder is required")
    vector_dir = ROOT / "_build/codecs/aac_vectors"
    vector_dir.mkdir(parents=True, exist_ok=True)
    for rate in [44100, 48000]:
        for channels in [1, 2]:
            subprocess.run([encoder, "-hide_banner", "-loglevel", "error", "-y", "-f", "lavfi", "-i",
                            f"sine=frequency=997:sample_rate={rate}:duration=0.2", "-ac", str(channels),
                            "-c:a", "aac", "-profile:a", "aac_low", "-b:a", "128k", "-f", "adts",
                            str(vector_dir / f"tone_{rate}_{channels}.aac")], check=True, timeout=20)
    sys.path.insert(0, str(ROOT / "_build/python_deps"))
    import paramiko
    password = getpass.getpass("Linux SSH password (session only): ")
    client = paramiko.SSHClient()
    client.load_host_keys(str(ROOT / "_build/linux_known_hosts"))
    client.set_missing_host_key_policy(paramiko.RejectPolicy())
    try:
        client.connect(args.host, username=args.user, password=password, look_for_keys=False,
                       allow_agent=False, timeout=8, auth_timeout=10)
        password = None
        _, stdout, _ = client.exec_command("python3 -c 'import os; print(os.path.expanduser(\"~/.cache/native_bt/aac_tests\"))'", timeout=8)
        remote = stdout.read().decode().strip()
        if not remote.startswith("/home/") or not remote.endswith("/.cache/native_bt/aac_tests"):
            raise RuntimeError("Unexpected remote workspace")
        dirs = [remote + suffix for suffix in ["/core/utils", "/baseline", "/candidate", "/headers"]]
        _, stdout, stderr = client.exec_command("mkdir -p " + " ".join(shlex.quote(value) for value in dirs), timeout=8)
        if stdout.channel.recv_exit_status(): raise RuntimeError(stderr.read().decode())
        sftp = client.open_sftp()
        hashes = {}
        def upload(name, data):
            hashes[name] = hashlib.sha256(data).hexdigest()
            with sftp.file(remote + "/" + name, "wb") as handle: handle.write(data)
        core = LIBRARY / "libhelix_aac"
        # Use exactly the AAC-LC sources present in the firmware compile database.
        commands = json.loads((ROOT / "_build/audio_app/compile_commands.json").read_text())
        actual_sources = sorted({Path(item["file"]).name for item in commands
                                 if "/libhelix_aac/" in item["file"].replace("\\", "/") and item["file"].endswith(".c")})
        if not actual_sources: raise RuntimeError("Missing target compile database")
        upload("sources.json", json.dumps(actual_sources).encode())
        for path in list(core.glob("*.h")) + list((core / "utils").glob("*.h")) + [core / name for name in actual_sources]:
            upload("core/" + path.relative_to(core).as_posix(), path.read_bytes())
        upload("headers/aac_decoder.h", (LIBRARY / "aac_decoder.h").read_bytes())
        upload("headers/esp_log.h", b'#define ESP_LOGI(...) ((void)0)\n#define ESP_LOGW(...) ((void)0)\n#define ESP_LOGE(...) ((void)0)\n#define ESP_LOGD(...) ((void)0)\n')
        upload("headers/esp_heap_caps.h", b'#include <stddef.h>\n#include <stdint.h>\n#define MALLOC_CAP_INTERNAL 1\n#define MALLOC_CAP_8BIT 2\nvoid *heap_caps_aligned_alloc(size_t,size_t,uint32_t);\nvoid heap_caps_free(void *);\n')
        for label in ["baseline", "candidate"]:
            path = ROOT / "_build/codecs/baseline/aac_decoder_helix.c" if label == "baseline" else LIBRARY / "aac_decoder_helix.c"
            upload(label + "/aac_decoder_helix.c", path.read_bytes())
            upload(label + "/aac_workspace_probe.c", (ROOT / "tests/aac_workspace_probe.c").read_bytes())
        for path in vector_dir.glob("*.aac"):
            upload(path.name, path.read_bytes())
        stdin, stdout, stderr = client.exec_command("cd " + shlex.quote(remote) + " && python3 -", timeout=180)
        stdin.write(DRIVER); stdin.flush(); stdin.channel.shutdown_write()
        output = stdout.read(100000).decode(); errors = stderr.read(100000).decode()
        if stdout.channel.recv_exit_status(): raise RuntimeError(errors or output)
        with sftp.file(remote + "/results.json", "r") as handle: report = json.loads(handle.read())
        report["source_sha256"] = hashes
        (ROOT / "_build/codecs/aac_results.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(output)
        sftp.close()
    finally:
        password = None
        client.close()


if __name__ == "__main__": main()
