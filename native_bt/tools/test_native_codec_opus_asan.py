"""Linux-only address-sanitized Opus worker test; no network or credentials."""
import os
from pathlib import Path
import shutil
import subprocess
import sys

if sys.platform != "linux":
    raise SystemExit("This driver requires local Linux, GCC, CMake, and Ninja")
gcc=shutil.which("gcc")
if not gcc: raise SystemExit("GCC is required")
runtime=subprocess.check_output([gcc,"-print-file-name=libasan.so"],text=True).strip()
if not Path(runtime).is_file(): raise SystemExit("GCC AddressSanitizer runtime not found")
env=os.environ.copy()
env["DBT_NATIVE_CODEC_ASAN"]="1"
env["LD_PRELOAD"]=runtime+(":"+env["LD_PRELOAD"] if env.get("LD_PRELOAD") else "")
env["ASAN_OPTIONS"]="detect_leaks=0:halt_on_error=1:abort_on_error=1"
raise SystemExit(subprocess.call([sys.executable,str(Path(__file__).with_name("test_native_codec_opus.py"))],env=env))
