"""Offline LDAC CIE and frame-header groundwork; no audio decoder linked."""
import ctypes
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/"_build/native_codecs"
OUT.mkdir(parents=True,exist_ok=True)
compiler=shutil.which("clang") or r"C:\Program Files\LLVM\bin\clang.exe"
env=os.environ.copy()
if os.name=="nt":
    vc=Path(r"C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat")
    data=subprocess.check_output(f'call "{vc}" >nul && set',shell=True,text=True)
    env.update(dict(line.split('=',1) for line in data.splitlines() if '=' in line and not line.startswith('=')))
sources=[ROOT/"codecs/ldac/dbt_ldac.c",ROOT/"tests/codecs/ldac_metadata_probe.c",ROOT/"src/dbt_avdtp_caps.c"]
dll=OUT/("ldac_metadata.dll" if os.name=="nt" else "ldac_metadata.so")
args=[compiler,"-std=c11","-O2","-Wall","-Wextra","-Werror","-shared"]
if os.name!="nt":args += ["-fPIC"]
for path in (ROOT/"include",ROOT/"codecs/common",ROOT/"codecs/ldac"):args += ["-I",str(path)]
subprocess.run(args+[str(p) for p in sources]+["-o",str(dll)],env=env,check=True,cwd=OUT)
result=ctypes.CDLL(str(dll)).test_ldac_metadata()
if result:raise RuntimeError("LDAC metadata test failed at C source line "+str(result))
report={"passed":True,"scope":"synthetic LDAC CIE and unfragmented frame bounds only; no codec or PCM",
        "configurations":12,"unsupported_rates":[176400,192000],
        "source_sha256":{str(p.relative_to(ROOT.parent)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}}
(OUT/"ldac_metadata_results.json").write_text(json.dumps(report,indent=2)+"\n")
print(json.dumps(report,indent=2))
