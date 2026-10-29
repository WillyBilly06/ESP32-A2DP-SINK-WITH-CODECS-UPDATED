"""Run an existing codec harness locally under Linux AddressSanitizer."""
import os
from pathlib import Path
import shutil
import subprocess
import sys

if sys.platform!="linux" or len(sys.argv)!=2 or sys.argv[1] not in ("aptx","lhdcv5","ldac"):
    raise SystemExit("Usage on Linux: python test_native_codec_asan.py aptx|lhdcv5|ldac")
gcc=shutil.which("gcc")
if not gcc:raise SystemExit("GCC is required")
runtime=subprocess.check_output([gcc,"-print-file-name=libasan.so"],text=True).strip()
if not Path(runtime).is_file():raise SystemExit("GCC AddressSanitizer runtime not found")
env=os.environ.copy();env["DBT_NATIVE_CODEC_ASAN"]="1"
env["LD_PRELOAD"]=runtime+(":"+env["LD_PRELOAD"] if env.get("LD_PRELOAD") else "")
env["ASAN_OPTIONS"]="detect_leaks=0:halt_on_error=1:abort_on_error=1"
script=Path(__file__).with_name("test_native_codec_"+("ldac_real" if sys.argv[1]=="ldac" else sys.argv[1])+".py")
raise SystemExit(subprocess.call([sys.executable,str(script)],env=env))
