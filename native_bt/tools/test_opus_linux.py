"""Real Opus packets through original/candidate wrappers and their actual CIE parser."""
import argparse
import getpass
import hashlib
import json
from pathlib import Path
import shlex
import sys

ROOT = Path(__file__).resolve().parents[1]
BT = ROOT.parent / "esp-idf/components/bt/host/bluedroid"
STUBS = {
    "common/bt_target.h": '#pragma once\n#define TRUE 1\n#define OPUS_DEC_INCLUDED 1\n#define UNUSED(value) ((void)(value))\n',
    "common/bt_defs.h": '#pragma once\n#include <stdint.h>\n#include <stdbool.h>\ntypedef uint8_t UINT8;\n',
    "common/bt_trace.h": '#define APPL_TRACE_ERROR(...) ((void)0)\n#define LOG_INFO(...) ((void)0)\n#define LOG_ERROR(...) ((void)0)\n',
    "osi/allocator.h": '#include <stddef.h>\nvoid *osi_malloc(size_t);\nvoid osi_free(void *);\n',
    "stack/bt_types.h": '#pragma once\n#include <stdint.h>\n#include <stdbool.h>\n#include <stddef.h>\n#include <sys/types.h>\ntypedef struct {uint16_t event,len,offset,layer_specific;} BT_HDR;\n',
    "stack/a2d_api.h": '''#pragma once
#include "common/bt_defs.h"
typedef uint8_t tA2D_STATUS;
typedef uint8_t tA2D_CODEC_TYPE;
#define A2D_MEDIA_TYPE_AUDIO 0
#define A2D_MEDIA_CT_NON_A2DP 255
#define A2D_SUCCESS 0
#define A2D_FAIL 10
#define A2D_INVALID_PARAMS 12
#define A2D_WRONG_CODEC 13
#define A2D_BAD_SAMP_FREQ 195
#define A2D_BAD_CH_MODE 197
#define A2D_SET_ONE_BIT 1
#define A2D_SET_ZERO_BIT 0
''',
    "stack/a2d_sbc.h": '#include <stdint.h>\nstatic inline unsigned A2D_BitsSet(uint32_t value) { return value==0?0:(value&(value-1))?2:1; }\n',
    "a2d_int.h": '#include "common/bt_trace.h"\n', "avdt_api.h": '',
    "stack/a2dp_codec_api.h": '''#pragma once
#include "stack/bt_types.h"
typedef void (*decoded_data_callback_t)(uint8_t *,uint32_t);
typedef struct {
 bool (*decoder_init)(decoded_data_callback_t);
 void (*decoder_cleanup)(void);
 bool (*decoder_reset)(void);
 ssize_t (*decode_packet_header)(BT_HDR *);
 bool (*decode_packet)(BT_HDR *,unsigned char *,size_t);
 void (*decoder_start)(void);
 void (*decoder_suspend)(void);
 void (*decoder_configure)(const uint8_t *);
} tA2DP_DECODER_INTERFACE;
''',
}
DRIVER = r'''
import ctypes, ctypes.util, hashlib, json, math, pathlib, subprocess
root=pathlib.Path.cwd()
backend=ctypes.util.find_library('opus')
assert backend, 'Linux libopus runtime is required'
opus=ctypes.CDLL(backend)
opus.opus_encoder_create.argtypes=[ctypes.c_int,ctypes.c_int,ctypes.c_int,ctypes.POINTER(ctypes.c_int)]
opus.opus_encoder_create.restype=ctypes.c_void_p
opus.opus_encoder_destroy.argtypes=[ctypes.c_void_p]
opus.opus_encode.argtypes=[ctypes.c_void_p,ctypes.POINTER(ctypes.c_int16),ctypes.c_int,ctypes.POINTER(ctypes.c_uint8),ctypes.c_int]
def run(command,folder):
 p=subprocess.run(command,cwd=folder,capture_output=True,text=True,timeout=50)
 if p.returncode: raise RuntimeError(' '.join(command)+'\n'+p.stderr[-8000:])
libraries={}
for label in ['baseline','candidate']:
 folder=root/label
 flags=['gcc','-std=gnu11','-O2','-I'+str(root/'headers'),'-I'+str(root/'headers/stack'),
        '-DOPUS_CANDIDATE='+str(int(label=='candidate')),'opus_workspace_probe.c','a2dp_vendor_opus.c','-l:'+backend]
 run(flags+['-shared','-fPIC','-o','probe.so'],folder)
 run(flags+['-O1','-g','-fsanitize=address,undefined','-fno-omit-frame-pointer','-DOPUS_SANITIZER_MAIN','-o','sanitized'],folder)
 run(['./sanitized'],folder)
 lib=ctypes.CDLL(str(folder/'probe.so'))
 lib.probe_packet.argtypes=[ctypes.POINTER(ctypes.c_uint8),ctypes.c_uint16,ctypes.c_uint8,ctypes.c_int]
 lib.probe_output.argtypes=[ctypes.POINTER(ctypes.c_uint8)]; lib.probe_output.restype=ctypes.c_uint32
 for n in ['probe_scratch_bytes','probe_fragment_bytes','probe_fragment_live']:getattr(lib,n).restype=ctypes.c_size_t
 libraries[label]=lib
results=[]
for variant in [0,1]:
 for channels in [1,2]:
  for samples in ([480,960] if variant==0 else [120,240,480,960,1920]):
   error=ctypes.c_int()
   encoder=opus.opus_encoder_create(48000,channels,2049,ctypes.byref(error)); assert encoder and error.value==0
   packets=[]
   for frame in range(3):
    pcm=(ctypes.c_int16*(samples*channels))(*[round(8000*math.sin(2*math.pi*(997+channel*700)*(frame*samples+i)/48000)) for i in range(samples) for channel in range(channels)])
    encoded=(ctypes.c_uint8*4000)(); size=opus.opus_encode(encoder,pcm,samples,encoded,4000); assert size>2
    packets.append(bytes(encoded[:size]))
   opus.opus_encoder_destroy(encoder)
   outputs={}; scratch={}
   for label,lib in libraries.items():
    assert lib.probe_open(variant,channels,samples)==0
    scratch[label]=lib.probe_scratch_bytes()
    if label=='candidate': assert scratch[label]==samples*channels*2 and lib.probe_fragment_bytes()==0
    out=bytearray()
    for index,packet in enumerate(packets):
     chunks=[(packet,1)] if index!=1 else [(packet[:len(packet)//2],0xc2),(packet[len(packet)//2:],0xa1)]
     for chunk,flags in chunks:
      raw=(ctypes.c_uint8*len(chunk)).from_buffer_copy(chunk)
      assert lib.probe_packet(raw,len(chunk),flags,0)==0
      pcm=(ctypes.c_uint8*8000)(); size=lib.probe_output(pcm)
      out.extend(bytes(pcm[:size]))
    outputs[label]=bytes(out)
    if label=='candidate': assert lib.probe_fragment_bytes()==0 # unfragmented packet drops old assembly storage
    lib.probe_close(); assert lib.probe_fragment_live()==0
   assert outputs['baseline']==outputs['candidate']
   results.append({'variant':variant,'channels':channels,'samples':samples,'scratch_before':scratch['baseline'],
                   'scratch_after':scratch['candidate'],'pcm_bytes':len(outputs['candidate']),
                   'pcm_sha256':hashlib.sha256(outputs['candidate']).hexdigest(),'bit_exact':True})
report={'passed':True,'scope':'actual CIE builder/parser and wrappers using system libopus; not firmware RF or target heap/timing proof',
        'vectors':results,'address_undefined_sanitizers':True,'inactive_fragment_after':0,
        'fragment_allocation_failure':True,'fragment_storage_released':True}
(root/'results.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
'''


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", required=True)
    parser.add_argument("--user", required=True)
    args = parser.parse_args()
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
        _, stdout, _ = client.exec_command("python3 -c 'import os; print(os.path.expanduser(\"~/.cache/native_bt/opus_tests\"))'", timeout=8)
        remote = stdout.read().decode().strip()
        if not remote.startswith("/home/") or not remote.endswith("/.cache/native_bt/opus_tests"):
            raise RuntimeError("Unexpected remote workspace")
        dirs = [remote + suffix for suffix in ["/headers/common", "/headers/osi", "/headers/stack", "/baseline", "/candidate"]]
        _, stdout, stderr = client.exec_command("mkdir -p " + " ".join(shlex.quote(value) for value in dirs), timeout=8)
        if stdout.channel.recv_exit_status(): raise RuntimeError(stderr.read().decode())
        sftp = client.open_sftp(); hashes = {}
        def upload(name,data):
            hashes[name]=hashlib.sha256(data).hexdigest()
            with sftp.file(remote+"/"+name,"wb") as handle:handle.write(data)
        for name,content in STUBS.items():upload("headers/"+name,content.encode())
        for name in ["a2dp_vendor_opus.h","a2dp_vendor_opus_constants.h","a2dp_vendor_opus_decoder.h","a2dp_decoder.h"]:
            upload("headers/stack/"+name,(BT/"stack/include/stack"/name).read_bytes())
        upload("headers/bt_av.h",(BT/"stack/a2dp/include/bt_av.h").read_bytes())
        for path in (BT/"external/opus/include").glob("*.h"):upload("headers/"+path.name,path.read_bytes())
        for label in ["baseline","candidate"]:
            path=ROOT/"_build/codecs/baseline/a2dp_vendor_opus_decoder.c" if label=="baseline" else BT/"stack/a2dp/a2dp_vendor_opus_decoder.c"
            upload(label+"/a2dp_vendor_opus_decoder.c",path.read_bytes())
            upload(label+"/a2dp_vendor_opus.c",(BT/"stack/a2dp/a2dp_vendor_opus.c").read_bytes())
            upload(label+"/opus_workspace_probe.c",(ROOT/"tests/opus_workspace_probe.c").read_bytes())
        stdin,stdout,stderr=client.exec_command("cd "+shlex.quote(remote)+" && python3 -",timeout=180)
        stdin.write(DRIVER);stdin.flush();stdin.channel.shutdown_write()
        output=stdout.read(100000).decode();errors=stderr.read(100000).decode()
        if stdout.channel.recv_exit_status():raise RuntimeError(errors or output)
        with sftp.file(remote+"/results.json","r") as handle:report=json.loads(handle.read())
        report["source_sha256"]=hashes
        (ROOT/"_build/codecs/opus_results.json").write_text(json.dumps(report,indent=2)+"\n",encoding="utf-8")
        print(json.dumps({"passed": report["passed"], "vectors": len(report["vectors"]),
                          "scratch_bytes_range": [min(item["scratch_after"] for item in report["vectors"]),
                                                  max(item["scratch_after"] for item in report["vectors"])],
                          "sanitizers": report["address_undefined_sanitizers"], "scope": report["scope"]}, indent=2))
        sftp.close()
    finally:
        password=None;client.close()


if __name__=="__main__":main()
