"""Bounded baseline/candidate IMDCT tests on the authorized Ubuntu machine."""
import argparse
import ctypes
import getpass
import hashlib
import json
from pathlib import Path
import shlex
import sys

ROOT = Path(__file__).resolve().parents[1]
REMOTE_DRIVER = r'''
import ctypes, json, pathlib, random, subprocess
root=pathlib.Path.cwd()
libraries={}
builds=[]
for label in ['baseline','candidate']:
    folder=root/label
    common=['gcc','-std=c11','-O3','-DLHDC_HOST_BUILD','-I.', 'lhdc_imdct_probe.c','lhdc_diag_config.c','-lm']
    commands=[common+['-fPIC','-shared','-o','probe.so'],
              common+['-O1','-g','-fsanitize=address,undefined','-fno-omit-frame-pointer','-DDBT_SANITIZER_MAIN','-o','probe_sanitized']]
    for command in commands:
        p=subprocess.run(command,cwd=folder,capture_output=True,text=True,timeout=40)
        if p.returncode: raise RuntimeError(label+': '+p.stderr[-6000:])
    p=subprocess.run(['./probe_sanitized'],cwd=folder,capture_output=True,text=True,timeout=25)
    if p.returncode: raise RuntimeError(label+' sanitizer: '+p.stderr[-6000:])
    builds.append({'variant':label,'sanitizers_passed':True})
    lib=ctypes.CDLL(str(folder/'probe.so'))
    for name in ['probe_live','probe_peak','probe_calls','probe_cos_allocation']:
        getattr(lib,name).restype=ctypes.c_size_t
    for name in ['probe_reference','probe_fast']:
        getattr(lib,name).argtypes=[ctypes.POINTER(ctypes.c_float),ctypes.POINTER(ctypes.c_float),ctypes.c_int]
    libraries[label]=lib
results=[]
for n in [240,480,960,1920]:
    row={'transform_size':n}
    for label,lib in libraries.items():
        assert lib.probe_reset()==0
        assert lib.probe_ensure(n)==0
        row[label+'_cos_bytes']=lib.probe_cos_allocation()
        calls=lib.probe_calls()
        assert lib.probe_ensure(n)==0 and lib.probe_calls()==calls
    assert row['candidate_cos_bytes']==4*n*4
    row['saved_bytes']=row['baseline_cos_bytes']-row['candidate_cos_bytes']
    for trial in range(5):
        rng=random.Random(1000+n+trial)
        values=[rng.uniform(-1000,1000) for _ in range(n//2)]
        if trial==0: values=[0.0]*(n//2); values[n//4]=1000.0
        source=(ctypes.c_float*(n//2))(*values)
        outputs=[]
        for lib in libraries.values():
            output=(ctypes.c_float*n)()
            lib.probe_reference(source,output,n)
            outputs.append(bytes(output))
        assert outputs[0]==outputs[1], ('reference output mismatch',n,trial)
    row['reference_bit_exact_vectors']=5
    for lib in libraries.values():
        lib.probe_release(); assert lib.probe_live()==0
    results.append(row)
fast=[]
for n in [480,960,1920]:
    outputs=[]
    for label,lib in libraries.items():
        assert lib.probe_reset()==0
        assert lib.probe_init(n)==0
        rng=random.Random(n)
        source=(ctypes.c_float*(n//2))(*[rng.uniform(-100,100) for _ in range(n//2)])
        out=(ctypes.c_float*n)()
        lib.probe_fast(source,out,n)
        outputs.append(bytes(out))
        lib.probe_release(); assert lib.probe_live()==0
    assert outputs[0]==outputs[1], ('fast output mismatch',n)
    fast.append({'transform_size':n,'bit_exact':True})
lib=libraries['candidate']
assert lib.probe_reset()==0
for n in [1920,480,960,240,480]:
    assert lib.probe_ensure(n)==0 and lib.probe_cos_allocation()==4*n*4
assert lib.probe_peak()==30720 # no overlap of old and new full tables
lib.probe_fail_next()
assert lib.probe_ensure(960)==-1 and lib.probe_live()==0 and lib.probe_cos_allocation()==0
assert lib.probe_ensure(960)==0
assert lib.probe_init(480)==0 and lib.probe_cos_allocation()==0 # discard fallback table
lib.probe_release(); assert lib.probe_live()==0
assert lib.probe_ensure(0)==-1 and lib.probe_ensure(1921)==-1 and lib.probe_live()==0
lib.probe_force_reference(1)
assert lib.probe_ensure(960)==0 and lib.probe_init(960)==0
calls=lib.probe_calls()
assert lib.probe_init(960)==0 and lib.probe_cos_allocation()==15360 and lib.probe_calls()==calls
lib.probe_force_reference(0)
assert lib.probe_init(960)==0 and lib.probe_cos_allocation()==0
lib.probe_release(); assert lib.probe_live()==0
report={'passed':True,'scope':'Linux IMDCT allocation and output equivalence; not full codec, ESP32 CPU, RF or acoustic validation',
        'tables':results,'fast_transforms':fast,'sanitizers':builds,'resize_no_overlap':True,
        'allocation_failure_retry':True,'invalid_sizes_rejected':True,'inactive_cos_released':True,
        'forced_reference_cache_reused':True}
(root/'results.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
'''


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", required=True)
    parser.add_argument("--user", required=True)
    parser.add_argument("--source", type=Path, default=ROOT.parent.parent / "LHDC-V5-Decoder-main")
    args = parser.parse_args()
    sys.path.insert(0, str(ROOT / "_build/python_deps"))
    import paramiko
    source = args.source.resolve() / "decoder"
    baseline = ROOT / "_build/lhdc/baseline"
    if not (baseline / "lhdc_imdct.c").is_file():
        raise RuntimeError("Preserve the unmodified baseline before testing")
    password = getpass.getpass("Linux SSH password (session only): ")
    client = paramiko.SSHClient()
    client.load_host_keys(str(ROOT / "_build/linux_known_hosts"))
    client.set_missing_host_key_policy(paramiko.RejectPolicy())
    try:
        client.connect(args.host, username=args.user, password=password, look_for_keys=False,
                       allow_agent=False, timeout=8, auth_timeout=10)
        password = None
        # Fixed, isolated paths. No SDK, media files, credentials or history uploads.
        stdin, stdout, stderr = client.exec_command("python3 -c 'import os; print(os.path.expanduser(\"~/.cache/native_bt/lhdc_imdct\"))'", timeout=8)
        remote = stdout.read().decode().strip()
        if not remote.startswith("/home/") or not remote.endswith("/.cache/native_bt/lhdc_imdct"):
            raise RuntimeError("Unexpected remote workspace")
        stdin, stdout, stderr = client.exec_command("mkdir -p " + shlex.quote(remote + "/baseline") + " " + shlex.quote(remote + "/candidate"), timeout=8)
        if stdout.channel.recv_exit_status():
            raise RuntimeError(stderr.read().decode())
        sftp = client.open_sftp()
        inputs = {}
        for label in ["baseline", "candidate"]:
            files = list(source.glob("*.h")) + [source / "lhdc_imdct.c", source / "lhdc_diag_config.c", ROOT / "tests/lhdc_imdct_probe.c"]
            for path in files:
                origin = baseline / path.name if label == "baseline" and (baseline / path.name).is_file() else path
                data = origin.read_bytes()
                if len(data) > 200000:
                    raise RuntimeError("Unexpectedly large test input")
                with sftp.file(remote + "/" + label + "/" + path.name, "wb") as target:
                    target.write(data)
                inputs[label + "/" + path.name] = hashlib.sha256(data).hexdigest()
        stdin, stdout, stderr = client.exec_command("cd " + shlex.quote(remote) + " && python3 -", timeout=120)
        stdin.write(REMOTE_DRIVER); stdin.flush(); stdin.channel.shutdown_write()
        result = stdout.read(100000).decode()
        errors = stderr.read(100000).decode()
        if stdout.channel.recv_exit_status():
            raise RuntimeError(errors or result)
        with sftp.file(remote + "/results.json", "r") as handle:
            report = json.loads(handle.read())
        report["source_sha256"] = inputs
        destination = ROOT / "_build/lhdc/results.json"
        destination.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(result)
        print("Receipt: " + str(destination))
        sftp.close()
    finally:
        password = None
        client.close()


if __name__ == "__main__":
    main()
