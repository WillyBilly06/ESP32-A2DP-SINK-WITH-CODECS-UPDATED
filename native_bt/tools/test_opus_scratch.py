"""Offline real SDK-Opus scratch test, using a fixed in-workspace build path."""
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
SDK = ROOT.parent / 'esp-idf/components/bt/host/bluedroid/external/opus'
SCRATCH = ROOT / 'codecs/opus/scratch'
BUILD = ROOT / '_build/opus_scratch'
env = os.environ.copy()
sanitize = env.get('DBT_OPUS_SCRATCH_ASAN') == '1'
if sanitize and os.name == 'nt':
    raise SystemExit('ASan/UBSan mode requires local Linux; Windows ASan runtime linkage was unsuccessful')
if sanitize:
    env['ASAN_OPTIONS'] = 'detect_leaks=1:halt_on_error=1'
    env['UBSAN_OPTIONS'] = 'halt_on_error=1'
if os.name == 'nt':
    vc = Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
    data = subprocess.check_output(f'call "{vc}" >nul && set', shell=True, text=True)
    env.update(dict(line.split('=', 1) for line in data.splitlines() if '=' in line and not line.startswith('=')))
    compiler = shutil.which('clang') or r'C:\Program Files\LLVM\bin\clang.exe'
    ninja = shutil.which('ninja') or str(Path(env['USERPROFILE']) / '.espressif/tools/ninja/1.12.1/ninja.exe')
else:
    compiler = shutil.which('clang') or shutil.which('gcc')
    ninja = shutil.which('ninja')
if not compiler or not ninja: raise SystemExit('clang/gcc, cmake, and ninja required')
def run(args):
    subprocess.run(args, env=env, check=True)
run([sys.executable, str(ROOT / 'tools/check_workspace.py')])
try:
    override = str(SCRATCH / 'dbt_opus_scratch_override.h').replace('\\','/')
    outputs = []
    for reference in (False, True):
        build = BUILD / ('reference' if reference else 'arena')
        flags = '-std=c11' if reference else f'-include "{override}" -std=c11'
        if sanitize: flags += ' -fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer -fwrapv'
        run(['cmake','-S',str(SDK),'-B',str(build),'-G','Ninja',f'-DCMAKE_MAKE_PROGRAM={ninja}',
             f'-DCMAKE_C_COMPILER={compiler}',f'-DCMAKE_C_FLAGS={flags}','-DCMAKE_BUILD_TYPE=Release',
             '-DOPUS_BUILD_SHARED_LIBRARY=OFF','-DOPUS_BUILD_TESTING=OFF','-DOPUS_BUILD_PROGRAMS=OFF',
             '-DOPUS_FIXED_POINT=ON','-DOPUS_ENABLE_FLOAT_API=OFF','-DOPUS_DISABLE_INTRINSICS=ON','-DOPUS_HARDENING=OFF'])
        run(['cmake','--build',str(build),'--parallel','4'])
        candidates = list(build.rglob('opus.lib' if os.name == 'nt' else 'libopus.a'))
        if len(candidates) != 1: raise RuntimeError(f'expected one static Opus library: {candidates}')
        exe = build / ('scratch_test.exe' if os.name == 'nt' else 'scratch_test')
        cmd = [compiler,'-std=c11','-O1','-g','-I'+str(SCRATCH),'-I'+str(SDK/'include'),
               str(SCRATCH/'dbt_opus_scratch.c'),str(ROOT/'tests/codecs/opus_scratch_test.c'),str(candidates[0]),'-o',str(exe)]
        if reference: cmd += ['-DOPUS_SCRATCH_REFERENCE']
        if sanitize: cmd += ['-fsanitize=address,undefined','-fno-sanitize-recover=all','-fno-omit-frame-pointer','-fwrapv']
        if os.name != 'nt': cmd += ['-lm']
        run(cmd)
        outputs.append(subprocess.check_output([str(exe)],env=env,text=True))
    if outputs[0] != outputs[1]: raise RuntimeError('arena and original VLA Opus PCM/results differ')
    print('Arena and VLA reference: all PCM samples and result lines match exactly; '
          f'{sum(line.startswith("pcm:") for line in outputs[0].splitlines())} decoded/PLC comparisons.')
finally:
    run([sys.executable, str(ROOT / 'tools/check_workspace.py')])
