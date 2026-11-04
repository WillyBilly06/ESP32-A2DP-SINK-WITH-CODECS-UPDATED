"""Run the offline SMP state, SDK-worker and NVS-shim tests on Linux.

This driver is credential-free and accesses only the supplied workspace. It
uses GCC AddressSanitizer and UndefinedBehaviorSanitizer. LeakSanitizer is
disabled because the tests load instrumented shared objects into CPython.
"""
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
if not sys.platform.startswith('linux'):
    raise SystemExit('This driver requires Linux; use the separate Windows scripts on Windows.')
gcc=shutil.which('gcc')
if not gcc or not shutil.which('cmake') or not shutil.which('ninja'):
    raise SystemExit('Linux GCC, CMake and Ninja are required.')
asan=subprocess.run([gcc,'-print-file-name=libasan.so'],check=True,text=True,capture_output=True).stdout.strip()
if not Path(asan).is_file():
    raise SystemExit('GCC AddressSanitizer runtime libasan.so was not found.')
sdk=Path(os.environ.get('DBT_SMP_MBEDTLS_SOURCE',ROOT.parent/'esp-idf/components/mbedtls/mbedtls'))
if not (sdk/'CMakeLists.txt').is_file():
    raise SystemExit('SDK mbedTLS source is missing; supply DBT_SMP_MBEDTLS_SOURCE as described in tests/smp/linux_inputs.txt.')
env=os.environ.copy()
env['CC']=gcc
env['DBT_SMP_SANITIZE']='1'
env['DBT_SMP_MBEDTLS_SOURCE']=str(sdk)
env['LD_PRELOAD']=asan+((':'+env['LD_PRELOAD']) if env.get('LD_PRELOAD') else '')
env['ASAN_OPTIONS']='detect_leaks=0:halt_on_error=1:abort_on_error=1'
env['UBSAN_OPTIONS']='halt_on_error=1:print_stacktrace=1'
check=[sys.executable,str(ROOT/'tools/check_workspace.py')]
subprocess.run(check,check=True)
try:
    for name in ('test_smp_session.py','test_smp_nvs.py'):
        subprocess.run([sys.executable,str(ROOT/'tools'/name)],env=env,check=True)
finally:
    subprocess.run(check,check=True)
print('Linux SMP AddressSanitizer and UndefinedBehaviorSanitizer runs passed; LeakSanitizer was disabled.')
